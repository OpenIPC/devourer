"""TX beam-steering feasibility for a moving broadcast source (#137, speculative).

Coherent multi-antenna beamforming focuses transmit energy toward the receiver:
for an N-element array it buys up to 10·log10(N) dB of directive gain over a
single antenna, and its EIRP-per-PA-watt scales with N (the energy lever of
#135 at its strongest). The catch is that it must *point*: the gain falls off the
main lobe, and a **broadcast** FPV downlink has no per-packet CSI to point with —
only a slow, coarse direction inferred from the return channel (RC/telemetry),
and the platform is moving. So the question is not "does beamforming have gain"
but "does the realised gain, after the pointing error a moving source with stale
direction suffers, beat the feedback-free diversity (STBC/MRC) we already
measured — which needs no pointing at all?"

This sim quantifies that. It models an idealised uniform linear array (λ/2
spacing) — its directive gain vs pointing error via the array factor — and a
pointing-error budget = platform angular rate × direction-update latency + the
angular resolution of the return-channel direction estimate. It then compares the
*realised* beam gain against the measured feedback-free STBC gain.

Idealisation caveat: real adapter antennas are not a calibrated λ/2 phased array
(the 8814's four antennas are packed, some internal), so this is an upper bound on
what beamforming could offer, not a prediction of a specific adapter.

CLI:
    uv run python beamsteer_feasibility_sim.py --n 4
    uv run python beamsteer_feasibility_sim.py --sweep
    uv run python beamsteer_feasibility_sim.py --self-test
"""
from __future__ import annotations

import argparse
import math

# The feedback-free alternative we actually measured (#132): STBC 2x1 coding gain
# (~2-3 dB), no pointing required. Beamforming must clear THIS to be worth it.
STBC_GAIN_DB = 2.5


def array_factor_db(n: int, theta_err_deg: float, spacing_wl: float = 0.5) -> float:
    """Directive gain (dB over a single element) of an N-element uniform linear
    array steered on-axis, evaluated at a pointing error theta_err. Peak is
    10·log10(N) on-axis; the array factor rolls it off toward the nulls."""
    if n <= 1:
        return 0.0
    peak_db = 10.0 * math.log10(n)
    psi = 2.0 * math.pi * spacing_wl * math.sin(math.radians(theta_err_deg))
    # normalised array factor |sin(N psi/2) / (N sin(psi/2))|
    denom = n * math.sin(psi / 2.0)
    if abs(denom) < 1e-12:
        af = 1.0
    else:
        af = abs(math.sin(n * psi / 2.0) / denom)
    return peak_db + 20.0 * math.log10(max(af, 1e-6))


def pointing_error_deg(rate_deg_s: float, update_latency_s: float,
                       csi_res_deg: float) -> float:
    """Pointing error budget: how far the true direction drifts between direction
    updates (rate x latency) plus the coarse return-channel angular resolution."""
    return rate_deg_s * update_latency_s + csi_res_deg


def realised_beam_gain_db(n: int, rate_deg_s: float, update_latency_s: float,
                          csi_res_deg: float) -> float:
    return array_factor_db(n, pointing_error_deg(rate_deg_s, update_latency_s,
                                                 csi_res_deg))


def verdict(beam_db: float) -> str:
    if beam_db > STBC_GAIN_DB + 1.0:
        return "BEAMFORM (clears STBC with margin)"
    if beam_db > STBC_GAIN_DB:
        return "marginal (~= STBC, not worth the CSI/pointing complexity)"
    return "USE DIVERSITY (STBC beats the pointed beam — no pointing needed)"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=4, help="array elements")
    ap.add_argument("--rate-deg-s", type=float, default=90.0,
                    help="platform angular rate (deg/s) — how fast the direction changes")
    ap.add_argument("--latency-s", type=float, default=0.2,
                    help="direction-update latency over the return channel (s)")
    ap.add_argument("--csi-res-deg", type=float, default=10.0,
                    help="angular resolution of the return-channel direction estimate")
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        return self_test()

    if args.sweep:
        print(f"N={args.n}: peak beam gain {10*math.log10(args.n):.1f} dB "
              f"(vs STBC {STBC_GAIN_DB} dB, no pointing). Realised gain vs "
              f"pointing error:")
        print(f"{'θ_err°':>7} {'beam dB':>8}  {'verdict'}")
        for te in (0, 5, 10, 15, 20, 30, 45):
            g = array_factor_db(args.n, te)
            print(f"{te:>7} {g:>8.1f}  {verdict(g)}")
        print("\nθ_err ≈ rate×latency + CSI-resolution. A fast platform with a "
              "slow/coarse\nreturn-channel direction blows the pointing budget "
              "long before the beam pays.")
        return 0

    te = pointing_error_deg(args.rate_deg_s, args.latency_s, args.csi_res_deg)
    g = realised_beam_gain_db(args.n, args.rate_deg_s, args.latency_s, args.csi_res_deg)
    print(f"N={args.n}, rate={args.rate_deg_s}°/s, latency={args.latency_s}s, "
          f"CSI-res={args.csi_res_deg}° → pointing error {te:.1f}°")
    print(f"peak beam gain {10*math.log10(args.n):.1f} dB → realised {g:.1f} dB "
          f"(STBC {STBC_GAIN_DB} dB)")
    print(f"verdict: {verdict(g)}")
    return 0


def self_test() -> int:
    print("=== beamsteer_feasibility_sim self-test ===")
    ok = True

    # on-axis peak is 10log10(N)
    c = abs(array_factor_db(4, 0.0) - 10 * math.log10(4)) < 1e-6
    ok &= c
    print(f"[{'ok' if c else 'FAIL'}] N=4 on-axis peak = {array_factor_db(4,0):.2f} dB (=6.02)")

    # gain falls monotonically off-axis (within the main lobe)
    c = array_factor_db(4, 0) > array_factor_db(4, 10) > array_factor_db(4, 20)
    ok &= c
    print(f"[{'ok' if c else 'FAIL'}] gain rolls off with pointing error "
          f"(0°={array_factor_db(4,0):.1f} 10°={array_factor_db(4,10):.1f} 20°={array_factor_db(4,20):.1f})")

    # pointing budget grows with rate and latency
    c = (pointing_error_deg(180, 0.3, 10) > pointing_error_deg(30, 0.3, 10)
         > pointing_error_deg(30, 0.05, 10))
    ok &= c
    print(f"[{'ok' if c else 'FAIL'}] pointing error grows with rate & latency")

    # the headline: a fast platform + slow coarse direction -> diversity wins;
    # a slow/known-geometry case -> beamforming clears STBC.
    fast = realised_beam_gain_db(4, rate_deg_s=120, update_latency_s=0.3, csi_res_deg=15)
    slow = realised_beam_gain_db(4, rate_deg_s=5, update_latency_s=0.05, csi_res_deg=3)
    c = fast < STBC_GAIN_DB and slow > STBC_GAIN_DB
    ok &= c
    print(f"[{'ok' if c else 'FAIL'}] fast/coarse={fast:.1f}dB (< STBC) vs "
          f"slow/precise={slow:.1f}dB (> STBC)")

    print("=== PASS ===" if ok else "=== FAIL ===")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
