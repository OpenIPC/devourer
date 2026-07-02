"""Active-chain count as an energy lever (spatial-diversity #135).

A multi-chain receiver can light up 1..N RF chains. Each active chain adds
always-on baseline draw (LNA + mixer + ADC), but more chains combine to improve
delivery — which, for a fixed video bitrate, means less airtime/FEC spent
covering losses and the baseline amortised over more delivered bits. So there is
an optimal chain count that minimises energy per *delivered* bit, and — this is
the point — it depends on the fade state:

  * static / correlated antennas (high ρ): combining barely helps delivery, so
    extra chains are pure baseline cost → run FEWER chains;
  * moving / decorrelated antennas (low ρ): combining fills the fades, so extra
    chains cheapen delivery more than their baseline costs → run MORE chains.

This is the energy corollary of the measured result (RX-MRC gives ~nothing at a
static position and eliminates dropouts under motion; see
docs/measuring-spatial-diversity.md): a ground station should adapt its chain
count to the fade/motion state, not just to range.

The model reuses the calibrated power/airtime model (energy_model.py) and adds
two terms: a per-chain baseline, and a diversity-improved delivery parameterised
by the antenna correlation ρ (via the same N_eff used elsewhere).

CLI:
    uv run python chain_count_energy_sim.py                 # ρ sweep table
    uv run python chain_count_energy_sim.py --self-test
"""
from __future__ import annotations

import argparse

from energy_model import (Calib, TxPoint, airtime_fraction, load_calibration)

# Per-active-RX-chain baseline draw (W) on top of the core baseline (LO + USB +
# baseband + one chain). LNA + mixer + ADC of an extra chain — a fraction of the
# core floor.
P_CHAIN_W = 0.15


def n_eff(n_ant: int, rho: float) -> float:
    """Effective diversity order of n_ant equicorrelated branches (participation
    ratio N/(1+(N-1)ρ²)); ρ=0 → n_ant, ρ=1 → 1. Same metric as the measurement
    doc and space_freq_diversity_sim."""
    if n_ant <= 1:
        return 1.0
    return n_ant / (1.0 + (n_ant - 1) * rho * rho)


def deliver_multichain(d1: float, n_ant: int, rho: float) -> float:
    """N-chain delivery from single-chain delivery d1. Combining drops the
    per-chain outage (1-d1) with the effective diversity order: outage^N_eff."""
    return 1.0 - (1.0 - d1) ** n_eff(n_ant, rho)


def energy_per_bit_nchain(p: TxPoint, src_bps: float, overhead: float,
                          payload: int, d1: float, n_ant: int, rho: float,
                          calib: Calib, p_chain_w: float = P_CHAIN_W) -> float:
    """J per delivered source bit with `n_ant` active RX chains at correlation ρ:

        (P_core + n_ant*P_chain + airtime*P_pa) / (src_bitrate * deliver_N)

    n_ant chains add baseline but raise deliver_N; +inf if infeasible / dead."""
    dN = deliver_multichain(d1, n_ant, rho)
    if dN <= 0.0:
        return float("inf")
    af = airtime_fraction(p, src_bps, overhead, payload, calib)
    if af > 1.0:
        return float("inf")
    p_avg = calib.p_baseline_w + n_ant * p_chain_w + min(1.0, af) * calib.pa_w(p.txagc)
    return p_avg / (src_bps * dN)


def optimal_chains(p: TxPoint, src_bps: float, overhead: float, payload: int,
                   d1: float, rho: float, calib: Calib, max_chains: int = 4,
                   p_chain_w: float = P_CHAIN_W) -> tuple[int, list[float]]:
    """Return (best_n, [E/bit for n=1..max_chains]) at fade correlation ρ."""
    eb = [energy_per_bit_nchain(p, src_bps, overhead, payload, d1, n, rho,
                                calib, p_chain_w) for n in range(1, max_chains + 1)]
    best = min(range(len(eb)), key=lambda i: eb[i]) + 1
    return best, eb


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--d1", type=float, default=0.6,
                    help="single-chain delivery at the operating point (marginal link)")
    ap.add_argument("--src-mbps", type=float, default=4.0)
    ap.add_argument("--overhead", type=float, default=0.25)
    ap.add_argument("--payload", type=int, default=1024)
    ap.add_argument("--mcs", type=int, default=3)
    ap.add_argument("--txagc", type=int, default=32)
    ap.add_argument("--max-chains", type=int, default=4)
    ap.add_argument("--p-chain-w", type=float, default=P_CHAIN_W)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        return self_test()

    cal = load_calibration()
    p = TxPoint(mcs=args.mcs, txagc=args.txagc)
    src = args.src_mbps * 1e6
    print(f"energy/delivered-bit vs active chains, by fade correlation ρ\n"
          f"(single-chain delivery d1={args.d1}, {args.src_mbps:g} Mbps video, "
          f"MCS{args.mcs}, P_chain={args.p_chain_w} W, P_core={cal.p_baseline_w} W)")
    hdr = "  " + " ".join(f"N={n}" for n in range(1, args.max_chains + 1))
    print(f"{'ρ':>5} {'N_eff':>6} {'deliverN(best)':>15}   E/bit nJ [{hdr.strip()}]   best")
    for rho in (0.9, 0.7, 0.5, 0.3, 0.1, 0.0):
        best, eb = optimal_chains(p, src, args.overhead, args.payload, args.d1,
                                  rho, cal, args.max_chains, args.p_chain_w)
        dN = deliver_multichain(args.d1, best, rho)
        cells = " ".join(f"{e*1e9:5.1f}" for e in eb)
        print(f"{rho:>5.1f} {n_eff(args.max_chains, rho):>6.2f} {dN:>15.3f}   "
              f"[{cells}]   N={best}")
    print("\nHigh ρ (static): diversity ~nil, so extra chains are pure baseline "
          "cost → best N=1.\nLow ρ (mobile): diversity fills the fades, so extra "
          "chains cheapen delivery > their baseline → best N grows.")
    return 0


def self_test() -> int:
    print("=== chain_count_energy_sim self-test ===")
    ok = True
    cal = load_calibration()
    p = TxPoint(mcs=3, txagc=32)
    src = 4e6

    # N_eff limits
    c = abs(n_eff(4, 0.0) - 4) < 1e-9 and abs(n_eff(4, 1.0) - 1) < 1e-9
    ok &= c
    print(f"[{'ok' if c else 'FAIL'}] N_eff(4): ρ0={n_eff(4,0):.2f} ρ1={n_eff(4,1):.2f}")

    # decorrelated (low ρ) improves delivery; correlated (high ρ) barely does
    d_lo = deliver_multichain(0.6, 4, 0.1)
    d_hi = deliver_multichain(0.6, 4, 0.9)
    c = d_lo > 0.95 and d_hi < d_lo
    ok &= c
    print(f"[{'ok' if c else 'FAIL'}] deliver(4ch): low-ρ={d_lo:.3f} > high-ρ={d_hi:.3f}")

    # the headline: static prefers fewer chains, mobile prefers more
    best_static, _ = optimal_chains(p, src, 0.25, 1024, 0.6, 0.9, cal)
    best_mobile, _ = optimal_chains(p, src, 0.25, 1024, 0.6, 0.1, cal)
    c = best_static < best_mobile
    ok &= c
    print(f"[{'ok' if c else 'FAIL'}] optimal chains: static N={best_static} < "
          f"mobile N={best_mobile}")

    # static optimum is 1 chain (extra chains pure cost when diversity is nil)
    c = best_static == 1
    ok &= c
    print(f"[{'ok' if c else 'FAIL'}] static optimum is a single chain (N={best_static})")

    # a more marginal link (lower d1) makes MORE chains worth their baseline
    best_marg, _ = optimal_chains(p, src, 0.25, 1024, 0.35, 0.1, cal)
    c = best_marg >= best_mobile
    ok &= c
    print(f"[{'ok' if c else 'FAIL'}] more-marginal link opens up more chains: "
          f"d1=0.35 mobile N={best_marg} >= d1=0.6 mobile N={best_mobile}")

    print("=== PASS ===" if ok else "=== FAIL ===")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
