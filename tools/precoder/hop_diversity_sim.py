"""Frequency-diversity simulation for the channel-hopping stream link.

When the TX hops per packet across N_ch channels (StreamTxDemo with
DEVOURER_HOP_CHANNELS, dwell=1), the symbols of one Reed-Solomon block — emitted
in ESI order — land on channels round-robin: ESI i -> channel (g_i mod N_ch),
where g_i is the global packet index. So a Reed-Solomon block of N symbols is
spread evenly across the hop set.

The payoff: a narrowband fade or interferer that wipes ONE channel erases only
~ceil(N / N_ch) of each block's symbols instead of a whole block. Because RS is
MDS (any K of N reconstruct), the block survives iff

    repair_count  >=  symbols_erased_on_the_dead_channel  ~=  ceil(N / N_ch).

This module proves that property end to end against the real codec
(stream_fec_rs.RsEncoder/RsDecoder, not a toy model), and contrasts it with the
no-hop baseline where every symbol of a block shares one channel — there, losing
that channel loses the entire block no matter how much repair you add.

CLI:
    uv run python hop_diversity_sim.py --k 8 --overhead 0.5 --channels 3 \
        --trials 200
    uv run python hop_diversity_sim.py --sweep        # repair-overhead sweep
"""
from __future__ import annotations

import argparse
import random

from stream_fec import FecConfig
from stream_fec_rs import RsEncoder, RsDecoder, _unpack_header


def _make_symbols(cfg: FecConfig, packets: list[bytes]) -> list[bytes]:
    enc = RsEncoder(cfg)
    syms: list[bytes] = []
    for p in packets:
        syms.extend(enc.add_packet(p))
    syms.extend(enc.flush())
    return syms


def _channel_for(symbol_index: int, block_id: int, esi: int, n: int,
                 n_ch: int, mode: str) -> int:
    """Which channel a symbol is transmitted on.

    'hop'   — per-packet round-robin (what StreamTxDemo does at dwell=1):
              keyed off the global symbol index so a block spreads across all
              channels.
    'nohop' — the whole block rides one channel (channel chosen per block).
    """
    if mode == "hop":
        return symbol_index % n_ch
    return block_id % n_ch


def run_trial(cfg: FecConfig, packets: list[bytes], n_ch: int, mode: str,
              dead_channels: set[int], per: float, rng: random.Random) -> bool:
    """Encode, drop symbols on dead channels (+ random per-channel loss),
    decode, and return True iff every input packet was recovered."""
    syms = _make_symbols(cfg, packets)
    dec = RsDecoder(cfg)
    recovered: list[bytes] = []
    for i, env in enumerate(syms):
        hdr = _unpack_header(env)
        if hdr is None:
            continue
        _k, _kreal, _ss, bid, esi, n = hdr
        ch = _channel_for(i, bid, esi, n, n_ch, mode)
        if ch in dead_channels:
            continue                       # channel wiped
        if per > 0 and rng.random() < per:
            continue                       # residual random loss
        recovered.extend(dec.add_symbol(env))
    return set(recovered) == set(packets) and len(recovered) == len(packets)


def _gen_packets(rng: random.Random, count: int, size: int) -> list[bytes]:
    # Distinct payloads so set-equality is a real recovery check (tag with index).
    return [bytes([i & 0xFF]) + bytes(rng.getrandbits(8) for _ in range(size - 1))
            for i in range(count)]


def montecarlo(cfg: FecConfig, n_ch: int, trials: int, per: float,
               n_packets: int, seed: int) -> dict[str, float]:
    rng = random.Random(seed)
    res = {"hop": 0, "nohop": 0}
    for _ in range(trials):
        packets = _gen_packets(rng, n_packets, cfg.max_packet_size)
        dead = {rng.randrange(n_ch)}        # exactly one channel down
        for mode in ("hop", "nohop"):
            if run_trial(cfg, packets, n_ch, mode, dead, per, rng):
                res[mode] += 1
    return {m: res[m] / trials for m in res}


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--k", type=int, default=8, help="RS source symbols/block")
    ap.add_argument("--overhead", type=float, default=0.5,
                    help="repair overhead (repair_count = ceil(k*overhead))")
    ap.add_argument("--symbol-size", type=int, default=64)
    ap.add_argument("--channels", type=int, default=3, help="hop channels N_ch")
    ap.add_argument("--per", type=float, default=0.0,
                    help="residual per-symbol loss on surviving channels")
    ap.add_argument("--packets", type=int, default=24)
    ap.add_argument("--trials", type=int, default=200)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--sweep", action="store_true",
                    help="sweep overhead and print the recovery table")
    args = ap.parse_args()

    if args.sweep:
        print(f"single-channel-loss recovery, K={args.k}, N_ch={args.channels} "
              f"({args.trials} trials each)")
        print(f"{'overhead':>8} {'R':>3} {'N':>3} {'ceilN/Nch':>9} "
              f"{'hop':>7} {'nohop':>7}")
        for ov in (0.25, 0.34, 0.5, 0.67, 1.0):
            cfg = FecConfig(k=args.k, symbol_size=args.symbol_size,
                            overhead=ov, scheme="rs")
            n = args.k + cfg.repair_count
            need = -(-n // args.channels)   # ceil(N / N_ch)
            r = montecarlo(cfg, args.channels, args.trials, args.per,
                           args.packets, args.seed)
            print(f"{ov:>8.2f} {cfg.repair_count:>3} {n:>3} {need:>9} "
                  f"{r['hop']:>7.2%} {r['nohop']:>7.2%}")
        print("\nhop recovers once R >= ceil(N/N_ch); nohop loses the whole "
              "block whenever its channel is the dead one (~1 - 1/N_ch).")
        return 0

    cfg = FecConfig(k=args.k, symbol_size=args.symbol_size,
                    overhead=args.overhead, scheme="rs")
    n = args.k + cfg.repair_count
    need = -(-n // args.channels)
    r = montecarlo(cfg, args.channels, args.trials, args.per, args.packets,
                   args.seed)
    print(f"K={args.k} R={cfg.repair_count} N={n} N_ch={args.channels} "
          f"per={args.per} | erased-on-dead-channel~={need}")
    print(f"single-channel-loss recovery:  hop={r['hop']:.2%}  "
          f"nohop={r['nohop']:.2%}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
