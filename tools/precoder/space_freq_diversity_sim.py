"""Space-frequency diversity simulation for the outer FEC.

devourer already spreads a Reed-Solomon block across hop *channels* (see
hop_diversity_sim.py): a narrowband fade wipes one channel and erases only
~ceil(N/N_ch) of a block's symbols, which an MDS code repairs. That is the
*frequency* axis. This module adds the orthogonal *spatial* axis — a multi-chain
receiver combining several antennas — and shows how the two multiply.

The axes attack a channel outage differently:

  * Frequency (hopping) SPREADS erasures — a dead channel takes out only a slice
    of each block instead of the whole block.
  * Spatial (antenna combining) makes a channel LESS LIKELY to die at all — a
    channel is only lost if the combined signal across all antennas is in outage,
    which is rarer than any single antenna's outage.

Crucially, the spatial benefit depends on how *decorrelated* the antennas are —
exactly the quantity `docs/measuring-spatial-diversity.md` measures. Independent
antennas turn a per-branch outage p into p^N_ant; fully correlated antennas give
no help (p stays p). This sim is parameterised by that correlation ρ, so the
hardware finding drops straight in: at a static position the antennas are
correlated (high ρ, spatial adds little); under motion they decorrelate (low ρ,
spatial multiplies).

It reuses the real codec (stream_fec_rs), not a toy model, and reports the
block-recovery rate for four configurations at the same code rate:

    none       1 channel , 1 antenna   — whole block on one link
    freq       N_ch      , 1 antenna   — hopping only (today's devourer)
    space      1 channel , N_ant       — combining only
    spacefreq  N_ch      , N_ant       — both

CLI:
    uv run python space_freq_diversity_sim.py --channels 3 --antennas 4 --rho 0.2
    uv run python space_freq_diversity_sim.py --sweep-rho      # static→mobile
    uv run python space_freq_diversity_sim.py --self-test
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


def n_eff(n_ant: int, rho: float) -> float:
    """Effective spatial diversity order for `n_ant` equicorrelated branches —
    the participation ratio of the correlation matrix, N/(1+(N-1)ρ²), matching
    the N_eff metric in docs/measuring-spatial-diversity.md. ρ=0 → n_ant
    (fully independent); ρ=1 → 1 (fully redundant)."""
    if n_ant <= 1:
        return 1.0
    return n_ant / (1.0 + (n_ant - 1) * rho * rho)


def p_channel_dead(p_branch: float, n_ant: int, rho: float) -> float:
    """Probability a channel's antenna-combined signal is in outage. A single
    branch is in outage with prob `p_branch`; combining L effective independent
    branches drops that to p_branch^L (L = n_eff). At ρ=1, L=1 and there is no
    spatial help."""
    return p_branch ** n_eff(n_ant, rho)


def run_trial(cfg: FecConfig, packets: list[bytes], n_ch: int, n_ant: int,
              rho: float, p_branch: float, per: float,
              rng: random.Random) -> bool:
    """One block: each channel is independently in outage with the
    spatially-reduced probability; symbols are hopped round-robin across
    channels; erase symbols on dead channels (+ residual per-symbol loss);
    decode with the real RS codec and check every packet came back."""
    syms = _make_symbols(cfg, packets)
    dec = RsDecoder(cfg)
    pcd = p_channel_dead(p_branch, n_ant, rho)
    dead = {c for c in range(n_ch) if rng.random() < pcd}
    recovered: list[bytes] = []
    for i, env in enumerate(syms):
        hdr = _unpack_header(env)
        if hdr is None:
            continue
        ch = i % n_ch                      # per-packet hop (dwell=1)
        if ch in dead:
            continue
        if per > 0 and rng.random() < per:
            continue
        recovered.extend(dec.add_symbol(env))
    return set(recovered) == set(packets) and len(recovered) == len(packets)


def _gen_packets(rng: random.Random, count: int, size: int) -> list[bytes]:
    return [bytes([i & 0xFF]) + bytes(rng.getrandbits(8) for _ in range(size - 1))
            for i in range(count)]


# The four configurations, as (n_ch_factor_uses_channels, n_ant_uses_antennas).
CONFIGS = ("none", "freq", "space", "spacefreq")


def _dims(mode: str, n_ch: int, n_ant: int) -> tuple[int, int]:
    return {
        "none":      (1,    1),
        "freq":      (n_ch, 1),
        "space":     (1,    n_ant),
        "spacefreq": (n_ch, n_ant),
    }[mode]


def montecarlo(cfg: FecConfig, n_ch: int, n_ant: int, rho: float,
               p_branch: float, per: float, n_packets: int, trials: int,
               seed: int) -> dict[str, float]:
    rng = random.Random(seed)
    res = {m: 0 for m in CONFIGS}
    for _ in range(trials):
        packets = _gen_packets(rng, n_packets, cfg.max_packet_size)
        for mode in CONFIGS:
            nc, na = _dims(mode, n_ch, n_ant)
            if run_trial(cfg, packets, nc, na, rho, p_branch, per, rng):
                res[mode] += 1
    return {m: res[m] / trials for m in CONFIGS}


def _row(cfg, n_ch, n_ant, rho, p_branch, per, packets, trials, seed):
    r = montecarlo(cfg, n_ch, n_ant, rho, p_branch, per, packets, trials, seed)
    return r


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--k", type=int, default=8, help="RS source symbols/block")
    ap.add_argument("--overhead", type=float, default=0.5)
    ap.add_argument("--symbol-size", type=int, default=64)
    ap.add_argument("--channels", type=int, default=3, help="hop channels N_ch")
    ap.add_argument("--antennas", type=int, default=4, help="RX chains N_ant")
    ap.add_argument("--rho", type=float, default=0.2,
                    help="spatial envelope correlation (0=indep, 1=redundant)")
    ap.add_argument("--p-branch", type=float, default=0.4,
                    help="single-antenna per-channel outage probability")
    ap.add_argument("--per", type=float, default=0.0,
                    help="residual per-symbol loss on surviving channels")
    ap.add_argument("--packets", type=int, default=24)
    ap.add_argument("--trials", type=int, default=400)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--sweep-rho", action="store_true",
                    help="sweep spatial correlation (static high ρ → mobile low ρ)")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    cfg = FecConfig(k=args.k, symbol_size=args.symbol_size,
                    overhead=args.overhead, scheme="rs")
    n = args.k + cfg.repair_count

    if args.sweep_rho:
        print(f"block recovery vs spatial correlation ρ  (K={args.k} R={cfg.repair_count} "
              f"N={n}, N_ch={args.channels}, N_ant={args.antennas}, "
              f"p_branch={args.p_branch}, {args.trials} trials)")
        print(f"{'ρ':>5} {'N_eff':>6} {'p_ch_dead':>10} "
              f"{'none':>7} {'freq':>7} {'space':>7} {'spacefreq':>10}")
        for rho in (0.9, 0.7, 0.5, 0.3, 0.1, 0.0):
            r = _row(cfg, args.channels, args.antennas, rho, args.p_branch,
                     args.per, args.packets, args.trials, args.seed)
            print(f"{rho:>5.1f} {n_eff(args.antennas, rho):>6.2f} "
                  f"{p_channel_dead(args.p_branch, args.antennas, rho):>10.3f} "
                  f"{r['none']:>7.1%} {r['freq']:>7.1%} {r['space']:>7.1%} "
                  f"{r['spacefreq']:>10.1%}")
        print("\nHigh ρ (static, correlated antennas): space adds little, "
              "spacefreq ≈ freq.\nLow ρ (moving, decorrelated): space multiplies, "
              "spacefreq ≫ either axis alone.")
        return 0

    r = montecarlo(cfg, args.channels, args.antennas, args.rho, args.p_branch,
                   args.per, args.packets, args.trials, args.seed)
    print(f"K={args.k} R={cfg.repair_count} N={n} N_ch={args.channels} "
          f"N_ant={args.antennas} ρ={args.rho} (N_eff={n_eff(args.antennas, args.rho):.2f}) "
          f"p_branch={args.p_branch} → p_ch_dead={p_channel_dead(args.p_branch, args.antennas, args.rho):.3f}")
    print(f"block recovery:  none={r['none']:.1%}  freq={r['freq']:.1%}  "
          f"space={r['space']:.1%}  spacefreq={r['spacefreq']:.1%}")
    return 0


def self_test() -> int:
    print("=== space_freq_diversity_sim self-test ===")
    ok = True

    # n_eff monotone: n_ant at ρ=0, → 1 at ρ=1
    e0, e1 = n_eff(4, 0.0), n_eff(4, 1.0)
    c = abs(e0 - 4.0) < 1e-9 and abs(e1 - 1.0) < 1e-9
    ok &= c
    print(f"[{'ok' if c else 'FAIL'}] N_eff(4): ρ0={e0:.2f} (=4), ρ1={e1:.2f} (=1)")

    cfg = FecConfig(k=8, symbol_size=64, overhead=0.5, scheme="rs")
    # p_branch 0.25: low enough that hopping spreads a recoverable number of
    # channel deaths, so the four configs separate cleanly.
    r = montecarlo(cfg, n_ch=3, n_ant=4, rho=0.1, p_branch=0.25, per=0.0,
                   n_packets=24, trials=400, seed=1)
    checks = [
        ("freq > none",            r["freq"] > r["none"]),
        ("space ≥ none",           r["space"] >= r["none"]),
        ("spacefreq ≥ freq",       r["spacefreq"] >= r["freq"] - 0.02),
        ("spacefreq ≥ space",      r["spacefreq"] >= r["space"] - 0.02),
        ("spacefreq best overall", r["spacefreq"] == max(r.values())),
    ]
    for name, cond in checks:
        ok &= cond
        print(f"[{'ok' if cond else 'FAIL'}] {name}  ({r})")

    # spatial helps MORE at low ρ than high ρ (the hardware finding)
    lo = montecarlo(cfg, 1, 4, 0.1, 0.5, 0.0, 24, 400, 2)["space"]
    hi = montecarlo(cfg, 1, 4, 0.9, 0.5, 0.0, 24, 400, 2)["space"]
    c = lo > hi
    ok &= c
    print(f"[{'ok' if c else 'FAIL'}] space recovery: low-ρ {lo:.1%} > high-ρ {hi:.1%}")

    print("=== PASS ===" if ok else "=== FAIL ===")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
