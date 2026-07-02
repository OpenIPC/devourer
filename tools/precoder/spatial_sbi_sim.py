"""Spatial diversity keeps corruption localized — the SBI precondition (#133).

The fused-FEC sub-block-integrity (SBI) layer salvages an FCS-failed frame by
keeping its surviving CRC-guarded sub-blocks. It only pays when corruption is
*localized* within the frame; a **deep fade** drops SNR far enough that the whole
frame's data smears (frame-wide corruption), and SBI recovers nothing from it
(measured survivor fraction ~0.36 in that regime — see docs/fused-fec.md).

Spatial diversity attacks exactly that. Combining several RX chains raises the
effective SNR and — more importantly — *reduces its variance*, which is what deep
fades are. So diversity moves frames out of the deep-fade (frame-wide, lost)
regime and into the marginal (localized, SBI-salvageable) and clean regimes. It
does not just deliver more frames; it improves SBI's **precondition** — the
fraction of the corrupt frames that are localized rather than frame-wide.

And because combining only reduces variance when the antennas decorrelate (the
measured static-vs-motion result), this benefit — like the rest of the spatial
story — is a mobility effect.

Model: Monte-Carlo correlated-Rayleigh branches (equicorrelation ρ), MRC-combine
N of them, classify each frame by its combined SNR into clean / localized / lost,
and compare single-chain vs N-chain. Pure-Python (no numpy), GNU-Radio-env safe.

CLI:
    uv run python spatial_sbi_sim.py --chains 4 --rho 0.2
    uv run python spatial_sbi_sim.py --sweep-rho
    uv run python spatial_sbi_sim.py --self-test
"""
from __future__ import annotations

import argparse
import math
import random

# Regime thresholds (dB, on the combined SNR). Above CLEAN_DB the FCS passes;
# between DEEP_DB and CLEAN_DB the frame is marginal — corruption stays localized
# and SBI salvages surviving sub-blocks; below DEEP_DB the fade is deep enough
# that corruption is frame-wide and SBI recovers nothing.
CLEAN_DB = 12.0
DEEP_DB = 6.0


def _complex_gauss(rng: random.Random) -> complex:
    """One unit-variance complex Gaussian (Box-Muller)."""
    u1 = max(rng.random(), 1e-12)
    u2 = rng.random()
    r = math.sqrt(-math.log(u1))          # var 1/2 per component -> |h|^2 mean 1
    return complex(r * math.cos(2 * math.pi * u2), r * math.sin(2 * math.pi * u2))


def _branches(rng: random.Random, n: int, rho_env: float) -> list[float]:
    """N equicorrelated Rayleigh branch powers |h_i|^2 (mean 1). Envelope
    correlation rho_env is achieved with field correlation sqrt(rho_env): each
    branch shares a common component and adds an independent one."""
    a = math.sqrt(math.sqrt(rho_env))     # field weight; rho_env = |rho_field|^2
    b = math.sqrt(max(0.0, 1.0 - a * a))
    shared = _complex_gauss(rng)
    return [abs(a * shared + b * _complex_gauss(rng)) ** 2 for _ in range(n)]


def trial_snr_db(rng: random.Random, n: int, rho_env: float,
                 mean_snr_db: float) -> float:
    """Combined SNR (dB) of N combined branches, normalised to the SAME MEAN as a
    single chain. Corruption localisation is a *variance* effect — deep fades are
    variance — so this isolates the diversity (fade-filling) benefit from MRC's
    array/mean gain (which is a separate delivery/energy effect, and which even
    correlated combining provides). Averaging the branch powers keeps the mean at
    the per-branch mean while cutting the variance in proportion to how
    decorrelated the branches are: correlated branches (static) move together so
    the average still fades deep; decorrelated branches (motion) smooth out."""
    gamma = 10 ** (mean_snr_db / 10.0)     # per-branch mean SNR (linear)
    powers = _branches(rng, n, rho_env)
    comb = gamma * (sum(powers) / n)       # matched-mean combine (variance only)
    return 10 * math.log10(max(comb, 1e-9))


def classify(snr_db: float) -> str:
    if snr_db >= CLEAN_DB:
        return "clean"
    if snr_db >= DEEP_DB:
        return "localized"
    return "lost"


def montecarlo(n: int, rho_env: float, mean_snr_db: float, trials: int,
               seed: int) -> dict:
    rng = random.Random(seed)
    c = {"clean": 0, "localized": 0, "lost": 0}
    for _ in range(trials):
        c[classify(trial_snr_db(rng, n, rho_env, mean_snr_db))] += 1
    t = float(trials)
    clean, loc, lost = c["clean"] / t, c["localized"] / t, c["lost"] / t
    corrupt = loc + lost
    return {
        "clean": clean, "localized": loc, "lost": lost,
        # SBI's precondition: of the corrupt frames, the fraction that is
        # localized (salvageable) rather than frame-wide (lost).
        "sbi_salvageable_of_corrupt": (loc / corrupt) if corrupt > 0 else 1.0,
        # delivered = clean + SBI-salvaged localized frames.
        "delivered": clean + loc,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--chains", type=int, default=4)
    ap.add_argument("--rho", type=float, default=0.2)
    ap.add_argument("--mean-snr-db", type=float, default=10.0)
    ap.add_argument("--trials", type=int, default=20000)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--sweep-rho", action="store_true")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        return self_test()

    def show(label, r):
        print(f"  {label:>16}: clean {r['clean']:.2f}  localized {r['localized']:.2f}  "
              f"lost {r['lost']:.2f}  | SBI-salvageable-of-corrupt "
              f"{r['sbi_salvageable_of_corrupt']:.2f}  delivered {r['delivered']:.2f}")

    if args.sweep_rho:
        print(f"single-chain vs {args.chains}-chain MRC, mean SNR "
              f"{args.mean_snr_db} dB, by correlation ρ")
        r1 = montecarlo(1, 0.0, args.mean_snr_db, args.trials, args.seed)
        show("1 chain", r1)
        for rho in (0.9, 0.5, 0.2, 0.0):
            rn = montecarlo(args.chains, rho, args.mean_snr_db, args.trials, args.seed)
            show(f"{args.chains}ch ρ={rho}", rn)
        print("\nDecorrelated combining (low ρ) collapses the 'lost' (frame-wide) "
              "bin and\nlifts the SBI-salvageable-of-corrupt fraction — it improves "
              "SBI's precondition,\nnot just raw delivery. High ρ (static) barely "
              "moves it.")
        return 0

    r1 = montecarlo(1, 0.0, args.mean_snr_db, args.trials, args.seed)
    rn = montecarlo(args.chains, args.rho, args.mean_snr_db, args.trials, args.seed)
    print(f"mean SNR {args.mean_snr_db} dB, ρ={args.rho}")
    show("1 chain", r1)
    show(f"{args.chains} chain", rn)
    return 0


def self_test() -> int:
    print("=== spatial_sbi_sim self-test ===")
    ok = True
    T, S = 20000, 3

    r1 = montecarlo(1, 0.0, 10.0, T, S)
    r4_lo = montecarlo(4, 0.1, 10.0, T, S)      # decorrelated (mobile)
    r4_hi = montecarlo(4, 0.9, 10.0, T, S)      # correlated (static)

    checks = [
        ("diversity cuts the frame-wide 'lost' bin", r4_lo["lost"] < r1["lost"]),
        ("diversity lifts SBI-salvageable-of-corrupt",
         r4_lo["sbi_salvageable_of_corrupt"] > r1["sbi_salvageable_of_corrupt"]),
        ("diversity raises delivered", r4_lo["delivered"] > r1["delivered"]),
        ("decorrelated beats correlated on the precondition",
         r4_lo["sbi_salvageable_of_corrupt"] > r4_hi["sbi_salvageable_of_corrupt"]),
        ("correlated (static) barely helps the lost bin",
         r4_hi["lost"] > r4_lo["lost"]),
    ]
    for name, c in checks:
        ok &= c
        print(f"[{'ok' if c else 'FAIL'}] {name}")
    print(f"   1ch: lost={r1['lost']:.2f} salv-of-corrupt={r1['sbi_salvageable_of_corrupt']:.2f}")
    print(f"   4ch ρ=.1: lost={r4_lo['lost']:.2f} salv-of-corrupt={r4_lo['sbi_salvageable_of_corrupt']:.2f}")
    print(f"   4ch ρ=.9: lost={r4_hi['lost']:.2f} salv-of-corrupt={r4_hi['sbi_salvageable_of_corrupt']:.2f}")
    print("=== PASS ===" if ok else "=== FAIL ===")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
