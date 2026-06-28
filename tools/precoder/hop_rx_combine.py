"""Diversity-RX combiner for the channel-hopping stream link.

A single Wi-Fi adapter has one synthesizer, so it can only hear one channel at a
time — it cannot, by itself, receive a per-packet-hopped stream. Reception needs
one of:

  * N adapters, one camped on each hop channel (the wfb-ng diversity model);
  * a wideband SDR (e.g. a B210) demodulating all hop channels at once;
  * a single adapter hopping in lockstep with the TX (FHSS) — simplest, but it
    only hears one channel per dwell and needs hop-sequence sync.

Whichever physical front end is used, the *combining* logic is the same and
lives here: each per-channel receiver delivers the FEC symbols it heard, and
this combiner feeds them all into ONE outer-FEC decoder. Because the decoder is
erasure-based and dedups by (block_id, ESI) — any K of N symbols rebuild a
block — symbols recovered on different channels simply add up. A receiver that
hears nothing (its channel faded or was jammed) contributes nothing, and the
block still decodes as long as the surviving channels carried >= K symbols. This
is exactly what makes the per-packet hop a frequency-diversity code rather than
plain redundancy.

`combine_streams` is the deployable core: hand it one iterable of symbol
envelopes per receiver. The CLI / montecarlo below exercise it end-to-end
through the real encoder+decoder so the diversity property is a test, not a
claim.
"""
from __future__ import annotations

import argparse
import random
from typing import Iterable, Sequence

from stream_fec import FecConfig, make_decoder
from stream_fec_rs import RsEncoder, _unpack_header


def combine_streams(channel_streams: Sequence[Iterable[bytes]],
                    cfg: FecConfig) -> tuple[list[bytes], list[int]]:
    """Merge per-receiver symbol feeds into one decode.

    channel_streams[c] is the ordered sequence of symbol envelopes that
    receiver c delivered. Returns (recovered_packets, symbols_consumed_per_ch).
    Receivers are drained round-robin to approximate concurrent arrival; order
    doesn't affect the erasure decoder's result, only when each block completes.
    """
    dec = make_decoder(cfg)
    recovered: list[bytes] = []
    iters = [iter(s) for s in channel_streams]
    used = [0] * len(iters)
    active = list(range(len(iters)))
    while active:
        still: list[int] = []
        for ci in active:
            try:
                sym = next(iters[ci])
            except StopIteration:
                continue
            used[ci] += 1
            recovered.extend(dec.add_symbol(sym))
            still.append(ci)
        active = still
    return recovered, used


def _encode_symbols(cfg: FecConfig, packets: list[bytes]) -> list[bytes]:
    enc = RsEncoder(cfg)
    syms: list[bytes] = []
    for p in packets:
        syms.extend(enc.add_packet(p))
    syms.extend(enc.flush())
    return syms


def split_by_channel(symbols: list[bytes], n_ch: int,
                     dead: set[int], per: float,
                     rng: random.Random) -> list[list[bytes]]:
    """Route TX symbols to per-receiver feeds the way a per-packet hop does
    (symbol i on channel i % n_ch), dropping symbols on dead channels and a
    fraction `per` on surviving ones."""
    feeds: list[list[bytes]] = [[] for _ in range(n_ch)]
    for i, sym in enumerate(symbols):
        ch = i % n_ch
        if ch in dead:
            continue
        if per > 0 and rng.random() < per:
            continue
        feeds[ch].append(sym)
    return feeds


def montecarlo(cfg: FecConfig, n_ch: int, trials: int, per: float,
               n_packets: int, seed: int, dead_count: int = 1) -> float:
    rng = random.Random(seed)
    ok = 0
    for _ in range(trials):
        packets = [bytes([i & 0xFF]) +
                   bytes(rng.getrandbits(8) for _ in range(cfg.max_packet_size - 1))
                   for i in range(n_packets)]
        syms = _encode_symbols(cfg, packets)
        dead = set(rng.sample(range(n_ch), dead_count)) if dead_count else set()
        feeds = split_by_channel(syms, n_ch, dead, per, rng)
        recovered, _used = combine_streams(feeds, cfg)
        if set(recovered) == set(packets) and len(recovered) == len(packets):
            ok += 1
    return ok / trials


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--k", type=int, default=8)
    ap.add_argument("--overhead", type=float, default=0.5)
    ap.add_argument("--symbol-size", type=int, default=64)
    ap.add_argument("--channels", type=int, default=3)
    ap.add_argument("--dead", type=int, default=1, help="receivers fully down")
    ap.add_argument("--per", type=float, default=0.0,
                    help="residual per-symbol loss on surviving receivers")
    ap.add_argument("--packets", type=int, default=24)
    ap.add_argument("--trials", type=int, default=200)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    cfg = FecConfig(k=args.k, symbol_size=args.symbol_size,
                    overhead=args.overhead, scheme="rs")
    n = args.k + cfg.repair_count
    need = -(-n // args.channels)
    rate = montecarlo(cfg, args.channels, args.trials, args.per, args.packets,
                      args.seed, args.dead)
    print(f"K={args.k} R={cfg.repair_count} N={n} receivers={args.channels} "
          f"dead={args.dead} per={args.per}")
    print(f"  erased-per-dead-channel~={need} (need R >= dead*that to survive)")
    print(f"  diversity-combine recovery: {rate:.2%}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
