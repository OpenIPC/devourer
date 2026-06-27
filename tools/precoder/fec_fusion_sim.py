#!/usr/bin/env python3
"""Offline simulation of the fused-FEC link — quantifies the sub-block-integrity
(SBI) recovery gain and picks the sub-block size, with NO hardware.

Runs the *real* pipeline end to end — `stream_fec.make_encoder` →
`fec_subblock.pack` → a byte-level channel → `fec_subblock.unpack` →
`stream_fec.make_decoder` — so the numbers reflect the exact code that runs on
air, not a model of it. For each radio body it compares two receivers:

  * baseline  — the chip drops any FCS-failed frame, so a body with even one
                bit error contributes zero symbols (today's behaviour);
  * sbi       — the body is kept (DEVOURER_RX_KEEP_CORRUPTED) and each sub-block
                is CRC-checked independently; survivors feed the outer decoder.

Channel models (per radio body):
  * uniform  — every bit flips with probability --ber.
  * slope    — "All Bits Are Not Equal" (INFOCOM 2009): per-bit error
               probability rises linearly with byte position, 0 at the head to
               2×--ber at the tail (sync/phase drift accumulates).
  * frameloss — independent whole-frame loss with probability --frame-loss
               (stacks with the bit model; both receivers lose these equally).

Usage:
  uv run python fec_fusion_sim.py --scheme rs --k 8 --overhead 0.5 \
      --symbol-size 128 --model slope --ber 2e-3 --trials 200 --sweep
"""

from __future__ import annotations

import argparse
import os
import random
import sys
from dataclasses import dataclass

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import fec_subblock  # noqa: E402
import stream_fec  # noqa: E402
from stream_fec import FecConfig  # noqa: E402


@dataclass
class Channel:
    model: str          # uniform | slope
    ber: float          # base bit-error rate
    frame_loss: float   # independent whole-frame loss prob
    rng: random.Random

    def corrupt(self, body: bytes) -> "tuple[bytes, bool, bool]":
        """Return (body_out, frame_lost, fcs_failed).

        frame_lost: the whole frame vanished (neither RX sees it).
        fcs_failed: at least one bit flipped (the chip FCS would fail).
        """
        if self.rng.random() < self.frame_loss:
            return body, True, True
        n = len(body)
        out = bytearray(body)
        flipped = False
        for i in range(n):
            if self.model == "slope":
                # 0 at head → 2*ber at tail.
                p = self.ber * 2.0 * (i / max(1, n - 1))
            else:
                p = self.ber
            if p <= 0:
                continue
            byte = out[i]
            for bit in range(8):
                if self.rng.random() < p:
                    byte ^= (1 << bit)
                    flipped = True
            out[i] = byte
        return bytes(out), False, flipped


@dataclass
class TrialResult:
    baseline_ok: bool
    sbi_ok: bool
    frames: int
    frames_lost: int
    frames_fcs_failed: int
    subblocks: int
    subblocks_failed: int


def _make_message(n_packets: int, pkt_len: int, rng: random.Random) -> list[bytes]:
    return [bytes(rng.randrange(256) for _ in range(pkt_len))
            for _ in range(n_packets)]


def run_trial(cfg: FecConfig, blocks_per_body: int, n_packets: int,
              pkt_len: int, chan: Channel, crc_bytes: int,
              rng: random.Random) -> TrialResult:
    msg = _make_message(n_packets, pkt_len, rng)

    enc = stream_fec.make_encoder(cfg)
    envs: list[bytes] = []
    for p in msg:
        envs += enc.add_packet(p)
    envs += enc.flush()
    if not envs:
        raise RuntimeError("encoder produced no symbols")
    env_size = len(envs[0])
    if any(len(e) != env_size for e in envs):
        raise RuntimeError("non-uniform envelope size — SBI needs fixed blocks")

    bodies = fec_subblock.pack(envs, block_payload=env_size,
                               blocks_per_body=blocks_per_body,
                               crc_bytes=crc_bytes)

    base_dec = stream_fec.make_decoder(cfg)
    sbi_dec = stream_fec.make_decoder(cfg)
    base_out: list[bytes] = []
    sbi_out: list[bytes] = []
    n_lost = n_fcs = sub_total = sub_failed = 0

    for body in bodies:
        corrupt, lost, fcs_failed = chan.corrupt(body)
        if lost:
            n_lost += 1
        if fcs_failed:
            n_fcs += 1

        # Baseline: a clean (no-bit-error, not-lost) frame passes all its
        # sub-blocks; an FCS-failed or lost frame contributes nothing.
        if not lost and not fcs_failed:
            for env in fec_subblock.unpack(body, env_size, crc_bytes).survivors:
                base_out += base_dec.add_symbol(env)

        # SBI: a lost frame is gone; otherwise salvage surviving sub-blocks.
        if not lost:
            res = fec_subblock.unpack(corrupt, env_size, crc_bytes)
            sub_total += res.n_blocks
            sub_failed += res.n_failed
            for env in res.survivors:
                sbi_out += sbi_dec.add_symbol(env)

    want = set(msg)
    return TrialResult(
        baseline_ok=set(base_out) >= want,
        sbi_ok=set(sbi_out) >= want,
        frames=len(bodies),
        frames_lost=n_lost,
        frames_fcs_failed=n_fcs,
        subblocks=sub_total,
        subblocks_failed=sub_failed,
    )


@dataclass
class Summary:
    blocks_per_body: int
    trials: int
    baseline_success: float
    sbi_success: float
    mean_subblock_loss: float
    overhead_pct: float
    env_size: int


def run(cfg: FecConfig, blocks_per_body: int, n_packets: int, pkt_len: int,
        chan_factory, crc_bytes: int, trials: int,
        seed: int) -> Summary:
    base_ok = sbi_ok = 0
    sub_total = sub_failed = 0
    env_size = 0
    for t in range(trials):
        rng = random.Random(seed + t)
        chan = chan_factory(rng)
        r = run_trial(cfg, blocks_per_body, n_packets, pkt_len, chan,
                      crc_bytes, rng)
        base_ok += int(r.baseline_ok)
        sbi_ok += int(r.sbi_ok)
        sub_total += r.subblocks
        sub_failed += r.subblocks_failed
    # Derive env_size from a throwaway encode for overhead accounting.
    enc = stream_fec.make_encoder(cfg)
    tmp: list[bytes] = []
    for p in _make_message(n_packets, pkt_len, random.Random(0)):
        tmp += enc.add_packet(p)
    tmp += enc.flush()
    env_size = len(tmp[0])
    body_payload = blocks_per_body * env_size
    overhead = fec_subblock.overhead_bytes(blocks_per_body, crc_bytes)
    return Summary(
        blocks_per_body=blocks_per_body,
        trials=trials,
        baseline_success=base_ok / trials,
        sbi_success=sbi_ok / trials,
        mean_subblock_loss=(sub_failed / sub_total) if sub_total else 0.0,
        overhead_pct=100.0 * overhead / (overhead + body_payload),
        env_size=env_size,
    )


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scheme", default="rs", choices=("raptorq", "rlc", "rs"))
    ap.add_argument("--k", type=int, default=8)
    ap.add_argument("--symbol-size", type=int, default=128)
    ap.add_argument("--overhead", type=float, default=0.5)
    ap.add_argument("--blocks-per-body", type=int, default=4)
    ap.add_argument("--message-packets", type=int, default=8)
    ap.add_argument("--packet-bytes", type=int, default=100)
    ap.add_argument("--model", default="slope", choices=("uniform", "slope"))
    ap.add_argument("--ber", type=float, default=2e-3)
    ap.add_argument("--frame-loss", type=float, default=0.0)
    ap.add_argument("--crc-bytes", type=int, default=2, choices=(2, 4))
    ap.add_argument("--trials", type=int, default=200)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--sweep", action="store_true",
                    help="sweep blocks-per-body 1..N and tabulate the knee")
    args = ap.parse_args(argv)

    cfg = FecConfig(k=args.k, symbol_size=args.symbol_size,
                    overhead=args.overhead, scheme=args.scheme)

    def chan_factory(rng):
        return Channel(model=args.model, ber=args.ber,
                       frame_loss=args.frame_loss, rng=rng)

    bpb_list = ([1, 2, 4, 8, 16] if args.sweep else [args.blocks_per_body])
    print(f"# fused-FEC sim: scheme={args.scheme} k={args.k} "
          f"symbol_size={args.symbol_size} overhead={args.overhead} "
          f"model={args.model} ber={args.ber} frame_loss={args.frame_loss} "
          f"crc={args.crc_bytes}B trials={args.trials}")
    print(f"# {'blk/body':>8} {'env_B':>6} {'ovh%':>6} "
          f"{'subloss%':>8} {'baseline':>9} {'sbi':>6}")
    for bpb in bpb_list:
        s = run(cfg, bpb, args.message_packets, args.packet_bytes,
                chan_factory, args.crc_bytes, args.trials, args.seed)
        print(f"  {s.blocks_per_body:>8} {s.env_size:>6} "
              f"{s.overhead_pct:>5.1f}% {100 * s.mean_subblock_loss:>7.2f}% "
              f"{100 * s.baseline_success:>8.1f}% {100 * s.sbi_success:>5.1f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
