"""Per-SVC-layer spatial UEP — the third protection axis (#136).

Unequal error protection already runs on two axes in devourer: the SVC PHY ladder
(svc_tx.h) flies important layers at a robust MCS, and per-layer FEC
(svc_uep_fec.py) gives them heavier Reed-Solomon redundancy. This module models
the *third* axis — **spatial diversity order** — which the ladder already carries
(its default puts LDPC+**STBC** on the critical/base layers and leaves the
enhancement layers single-stream), now that #132 measured what STBC actually buys
(~2-3 dB coding gain plus diversity, widening under motion).

The point is that the three axes compound. A critical layer flown at a robust
MCS, with heavy FEC, *and* STBC survives far deeper into a fade than any one axis
alone — while the enhancement layers, cheap on all three, shed first. That is the
graceful staircase, now on three knobs instead of two.

The model is analytical (no codec dependency): each layer's per-frame PHY
delivery is a soft function of the link SNR around its MCS threshold; STBC shifts
that threshold down by the measured coding gain; then the layer's Reed-Solomon
block survives if the per-symbol losses stay within its repair budget (binomial).
Sweeping the link SNR down produces the per-layer delivery staircase.

CLI:
    uv run python svc_spatial_uep_sim.py                 # staircase sweep
    uv run python svc_spatial_uep_sim.py --no-stbc       # spatial axis off (A/B)
    uv run python svc_spatial_uep_sim.py --self-test
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

# Approx HT20 per-MCS decode threshold (dB SNR for ~clean delivery). Rough but
# monotone — the staircase only needs the ordering and spacing.
MCS_THRESH_DB = {0: 5.0, 1: 7.0, 2: 9.0, 3: 11.0, 4: 15.0, 5: 18.0, 6: 22.0, 7: 25.0}
# STBC 2x1 coding gain (dB) — from the #132 measurement (static 1.94x at the MCS5
# cliff ≈ a couple dB; the diversity part adds more under motion).
STBC_GAIN_DB = 2.5
SOFT_WIDTH_DB = 2.0   # FER-vs-SNR transition sharpness


@dataclass(frozen=True)
class Layer:
    name: str
    mcs: int
    stbc: bool
    fec_overhead: float   # RS repair/source ratio
    k: int = 8            # RS source symbols/block


# Default ladder: mirrors svc_tx.h (MCS + STBC) and svc_uep_fec.py (FEC overhead).
DEFAULT_LADDER = [
    Layer("critical", mcs=0, stbc=True,  fec_overhead=1.00),
    Layer("T0-base",  mcs=1, stbc=True,  fec_overhead=0.75),
    Layer("T1",       mcs=4, stbc=False, fec_overhead=0.50),
    Layer("T2",       mcs=7, stbc=False, fec_overhead=0.25),
]


def phy_delivery(snr_db: float, mcs: int, stbc: bool) -> float:
    """Per-frame delivery probability: soft transition around the MCS threshold,
    shifted down by the STBC coding gain when enabled."""
    thresh = MCS_THRESH_DB[mcs] - (STBC_GAIN_DB if stbc else 0.0)
    return 0.5 * (1.0 + math.tanh((snr_db - thresh) / SOFT_WIDTH_DB))


def _binom_cdf(k: int, n: int, p: float) -> float:
    """P(X <= k) for X ~ Binomial(n, p)."""
    if p <= 0.0:
        return 1.0
    if p >= 1.0:
        return 1.0 if k >= n else 0.0
    return sum(math.comb(n, i) * p**i * (1 - p)**(n - i) for i in range(0, k + 1))


def layer_delivery(snr_db: float, layer: Layer, stbc_enabled: bool = True) -> float:
    """Probability the layer's RS block is recovered: PHY per-symbol loss folded
    through the layer's repair budget."""
    d = phy_delivery(snr_db, layer.mcs, layer.stbc and stbc_enabled)
    loss = 1.0 - d
    n = layer.k + max(0, round(layer.k * layer.fec_overhead))
    repair = n - layer.k
    return _binom_cdf(repair, n, loss)


def sweep(ladder=DEFAULT_LADDER, snr_hi=30.0, snr_lo=0.0, step=2.0,
          stbc_enabled=True):
    rows = []
    snr = snr_hi
    while snr >= snr_lo - 1e-9:
        rows.append((snr, [layer_delivery(snr, L, stbc_enabled) for L in ladder]))
        snr -= step
    return rows


def survival_snr(layer: Layer, thresh=0.9, stbc_enabled=True) -> float:
    """Lowest SNR (dB) at which this layer still delivers >= `thresh`."""
    snr = 30.0
    while snr >= -10.0:
        if layer_delivery(snr, layer, stbc_enabled) < thresh:
            return snr + 0.1
        snr -= 0.1
    return -10.0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-stbc", action="store_true",
                    help="disable the spatial axis (A/B: ladder without STBC)")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        return self_test()

    stbc = not args.no_stbc
    ladder = DEFAULT_LADDER
    print(f"per-SVC-layer delivery vs link SNR  (spatial axis {'ON' if stbc else 'OFF'})")
    print(f"ladder: " + "  ".join(
        f"{L.name}[MCS{L.mcs}{'/STBC' if L.stbc else ''} ov{L.fec_overhead:.2f}]"
        for L in ladder))
    print(f"{'SNR':>5} | " + " ".join(f"{L.name:>9}" for L in ladder))
    for snr, ds in sweep(ladder, stbc_enabled=stbc):
        print(f"{snr:>5.0f} | " + " ".join(f"{d:>9.2f}" for d in ds))
    print("\nsurvival SNR (>=90% delivery):")
    for L in ladder:
        s_on = survival_snr(L, stbc_enabled=True)
        s_off = survival_snr(L, stbc_enabled=False)
        tag = f"  (STBC extends survival by {s_off - s_on:.1f} dB)" if L.stbc else ""
        print(f"  {L.name:>9}: {survival_snr(L, stbc_enabled=stbc):>5.1f} dB{tag}")
    print("\nThe staircase: enhancement (T2, T1) sheds first; base/critical hold "
          "deepest.\nThe spatial axis (STBC) extends the critical/base layers' "
          "survival by the coding gain — a third UEP knob stacked on MCS and FEC.")
    return 0


def self_test() -> int:
    print("=== svc_spatial_uep_sim self-test ===")
    ok = True

    # STBC lowers the effective threshold -> higher delivery at a given SNR
    c = phy_delivery(10, 5, True) > phy_delivery(10, 5, False)
    ok &= c
    print(f"[{'ok' if c else 'FAIL'}] STBC lifts PHY delivery at fixed SNR")

    # the staircase: at a mid SNR, importance order holds (critical >= T0 >= T1 >= T2)
    snr = 14.0
    ds = [layer_delivery(snr, L) for L in DEFAULT_LADDER]
    c = ds[0] >= ds[1] >= ds[2] >= ds[3]
    ok &= c
    print(f"[{'ok' if c else 'FAIL'}] staircase at {snr}dB: "
          f"{[round(d,2) for d in ds]} (critical>=T0>=T1>=T2)")

    # critical survives deeper than enhancement
    c = survival_snr(DEFAULT_LADDER[0]) < survival_snr(DEFAULT_LADDER[3])
    ok &= c
    print(f"[{'ok' if c else 'FAIL'}] critical survival {survival_snr(DEFAULT_LADDER[0]):.1f}dB "
          f"< T2 {survival_snr(DEFAULT_LADDER[3]):.1f}dB")

    # the spatial axis genuinely extends critical survival (3rd knob adds margin)
    s_on = survival_snr(DEFAULT_LADDER[0], stbc_enabled=True)
    s_off = survival_snr(DEFAULT_LADDER[0], stbc_enabled=False)
    c = s_on < s_off - 1.0    # STBC pushes survival >=1 dB lower
    ok &= c
    print(f"[{'ok' if c else 'FAIL'}] STBC extends critical survival: "
          f"{s_off:.1f}dB -> {s_on:.1f}dB (Δ{s_off-s_on:.1f}dB)")

    print("=== PASS ===" if ok else "=== FAIL ===")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
