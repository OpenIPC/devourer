#!/usr/bin/env python3
"""Per-tone interference localizer from VHT beamforming reports.

The finest frequency-resolution sensor devourer has: where the channel-wide
energy sweep (DEVOURER_RX_SWEEP) resolves to the 20 MHz channel grid, the
self-sounding beamforming report carries a *per-subcarrier* picture — an MU
report's per-tone SNR curve and the per-tone cross-frame psi variance. A
narrowband interferer sitting on a few subcarriers shows up as a localized
notch in the per-tone SNR and/or a spike in per-tone psi variance, which this
script thresholds (robust median/MAD) to flag and locate the interferer to a
subcarrier group (~312 kHz * Ng) — far finer than a channel.

Input: `<devourer-bf-report-raw>HEX` lines (WiFiDriverDemo DEVOURER_BF_DETECT_
REPORT=4) on stdin or a file. Reuses tools/bf_report_decode.py's analyzer.

Two modes:
  single    (default) — outlier detection within one capture: flag tones whose
            SNR is depressed (or psi-var elevated) far from the band's own
            median. Robust to an interferer covering a minority of tones.
  --baseline FILE — differential: compare a clean baseline capture against the
            interferer capture and flag tones whose SNR dropped / psi-var rose.
            More sensitive; needs a toggled interferer.

Usage:
  WiFiDriverDemo ... DEVOURER_BF_DETECT_REPORT=4 | tests/rx_tone_localize.py \
      --channel 100 --bw 20
  tests/rx_tone_localize.py interferer.txt --baseline clean.txt --channel 100
"""
from __future__ import annotations

import argparse
import os
import statistics
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
import bf_report_decode as bf  # noqa: E402


def load(path_or_none, max_frames):
    src = open(path_or_none) if path_or_none else sys.stdin
    frames = bf.read_frames(src, max_frames)
    if not frames:
        return None
    return bf.analyze_frames(frames)


def tone_freqs(ns, bw_code, center_mhz):
    """Map subcarrier-group index 0..ns-1 to an absolute MHz. The report groups
    the occupied band into ns bins; map them linearly across the bandwidth
    centred on center_mhz (a coarse but honest mapping — the report does not
    carry the exact 802.11 tone allocation). Resolution = bw/ns MHz per bin."""
    bw_mhz = 20 << bw_code
    if center_mhz is None:
        # No channel given — report offsets from band centre.
        return [(k - (ns - 1) / 2.0) * bw_mhz / ns for k in range(ns)]
    return [center_mhz + (k - (ns - 1) / 2.0) * bw_mhz / ns for k in range(ns)]


def mad(xs, med):
    return statistics.median([abs(x - med) for x in xs]) or 1e-9


def flag_high(vals, k):
    """Indices whose value is > k MADs ABOVE the median (variance spikes)."""
    med = statistics.median(vals)
    m = mad(vals, med)
    return [i for i, v in enumerate(vals) if v - med > k * m]


def flag_low(vals, k):
    """Indices whose value is > k MADs BELOW the median (SNR notches)."""
    med = statistics.median(vals)
    m = mad(vals, med)
    return [i for i, v in enumerate(vals) if med - v > k * m]


def contiguous(indices):
    """Group a sorted index list into contiguous [lo,hi] runs."""
    runs = []
    for i in sorted(indices):
        if runs and i == runs[-1][1] + 1:
            runs[-1][1] = i
        else:
            runs.append([i, i])
    return runs


def bar(vals, width=52):
    lo, hi = min(vals), max(vals)
    span = (hi - lo) or 1.0
    ramp = " .:-=+*#%@"
    return "".join(ramp[min(len(ramp) - 1, int((v - lo) / span * (len(ramp) - 1)))]
                   for v in vals), lo, hi


def self_test() -> int:
    """Deterministic validation of the detection + frequency-mapping math on
    synthetic per-tone arrays — independent of the (marginal, bench-limited) RF
    regime. A narrowband interferer is modelled as a psi-variance spike / SNR
    notch on a contiguous group of tones; assert we flag exactly that group and
    map it to the right frequency."""
    ns, bw_code, ch = 52, 0, 100          # 20 MHz on ch100 (5500 MHz center)
    center = 5000 + 5 * ch
    freqs = tone_freqs(ns, bw_code, center)
    fails = []

    # 1) variance spike at tones 40..42 (an interferer near +5.6 MHz).
    var = [0.010] * ns
    for k in (40, 41, 42):
        var[k] = 0.20
    hot = flag_high(var, 5.0)
    if hot != [40, 41, 42]:
        fails.append(f"variance spike: flagged {hot}, want [40,41,42]")
    runs = contiguous(hot)
    if runs != [[40, 42]]:
        fails.append(f"contiguous: {runs}, want [[40,42]]")
    cen = (freqs[40] + freqs[42]) / 2.0
    if not (5505.0 <= cen <= 5507.0):
        fails.append(f"freq map: group centre {cen:.2f} MHz, want ~5506")

    # 2) SNR notch at tones 10..11 (interferer near -6 MHz).
    snr = [25.0] * ns
    snr[10] = snr[11] = 8.0
    dip = flag_low(snr, 4.0)
    if dip != [10, 11]:
        fails.append(f"SNR notch: flagged {dip}, want [10,11]")

    # 3) clean arrays flag nothing.
    if flag_high([0.01] * ns, 5.0) or flag_low([25.0] * ns, 4.0):
        fails.append("clean arrays produced a false positive")

    # 4) two separate interferers -> two groups.
    v2 = [0.01] * ns
    for k in (5, 6, 30):
        v2[k] = 0.3
    if contiguous(flag_high(v2, 5.0)) != [[5, 6], [30, 30]]:
        fails.append(f"two-group: {contiguous(flag_high(v2, 5.0))}")

    # 5) resolution is sub-channel (bw/ns).
    res_khz = (20 << bw_code) * 1000.0 / ns
    if not (300 <= res_khz <= 400):
        fails.append(f"resolution {res_khz:.0f} kHz not ~385")

    if fails:
        print("SELF-TEST FAILED:")
        for f in fails:
            print("  -", f)
        return 1
    print(f"SELF-TEST PASSED: variance spike + SNR notch localized to the "
          f"correct tone groups; {res_khz:.0f} kHz/bin sub-channel resolution; "
          f"clean arrays clean; multi-group split works.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("infile", nargs="?", help="interferer capture (default stdin)")
    ap.add_argument("--baseline", help="clean baseline capture for differential mode")
    ap.add_argument("--channel", type=int, default=None,
                    help="center channel (for absolute MHz in the report)")
    ap.add_argument("--center-mhz", type=float, default=None,
                    help="center frequency MHz (overrides --channel mapping)")
    ap.add_argument("--max-frames", type=int, default=400)
    ap.add_argument("--snr-k", type=float, default=4.0,
                    help="flag a tone when SNR is this many MADs below the median")
    ap.add_argument("--var-k", type=float, default=5.0,
                    help="flag a tone when psi-var is this many MADs above median")
    ap.add_argument("--self-test", action="store_true",
                    help="deterministic check of the detection+localization math "
                         "(no hardware) — synthetic notch/spike at a known tone")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    a = load(args.infile, args.max_frames)
    if a is None:
        print("no beamforming reports on input", file=sys.stderr)
        return 1

    ns = a["ns"]
    rows = a["rows"]                          # (k, psi, phi, ratio, psi_var)
    psi_var = [r[4] for r in rows]
    snr = a["mu_snr_db"]                      # per-tone SNR (MU) or None

    center = args.center_mhz
    if center is None and args.channel is not None:
        # 5 GHz: f = 5000 + 5*ch; 2.4 GHz: f = 2407 + 5*ch (ch14 = 2484).
        center = (5000 + 5 * args.channel if args.channel >= 15
                  else 2407 + 5 * args.channel)
    freqs = tone_freqs(ns, a["bw"], center)
    res_khz = (20 << a["bw"]) * 1000.0 / ns
    funit = "MHz" if center is not None else "MHz-off-center"

    print(f"# per-tone localizer: SA={a['sa']} BW={20 << a['bw']}MHz Ng={a['ng']} "
          f"Ns={ns} ({res_khz:.0f} kHz/bin) reports={a['nfr']} "
          f"mu_reports={a['mu_reports']}")

    flagged = set()

    # --- psi cross-frame variance (always available) ---
    vhot = flag_high(psi_var, args.var_k)
    vline, vlo, vhi = bar(psi_var)
    print(f"\n# per-tone psi cross-frame variance [{vlo:.4f}..{vhi:.4f}] "
          f"(spikes = interferer-perturbed channel estimate):")
    print(f"  var: {vline}")
    if vhot:
        flagged.update(vhot)

    # --- MU per-tone SNR notch ---
    if snr is not None and len(snr) >= 4:
        m = min(ns, len(snr))
        s = snr[:m]
        sdip = flag_low(s, args.snr_k)
        sline, slo, shi = bar(s)
        print(f"\n# per-tone SNR dB [{slo:.1f}..{shi:.1f}] "
              f"(notch = interferer raising the noise floor):")
        print(f"  snr: {sline}")
        if sdip:
            flagged.update(sdip)
    else:
        print("\n# no MU per-tone SNR series (arm the beamformee with "
              "DEVOURER_BF_ARM_BFEE_MU=1 for the SNR notch) — using psi-var only")

    # --- differential against a clean baseline ---
    if args.baseline:
        b = load(args.baseline, args.max_frames)
        if b is None:
            print("# baseline had no reports — skipping differential",
                  file=sys.stderr)
        elif b["ns"] == ns:
            bvar = [r[4] for r in b["rows"]]
            dvar = [psi_var[k] - bvar[k] for k in range(ns)]
            drise = flag_high(dvar, args.var_k)
            dline, dlo, dhi = bar(dvar)
            print(f"\n# differential psi-var (interferer - baseline) "
                  f"[{dlo:.4f}..{dhi:.4f}]:")
            print(f"  d:   {dline}")
            flagged.update(drise)
            if b["mu_snr_db"] and snr:
                mm = min(len(snr), len(b["mu_snr_db"]), ns)
                dsnr = [snr[k] - b["mu_snr_db"][k] for k in range(mm)]
                ddip = [k for k in range(mm) if dsnr[k] < -3.0]  # >3 dB drop
                flagged.update(ddip)
        else:
            print("# baseline geometry differs — skipping differential",
                  file=sys.stderr)

    # --- verdict ---
    print()
    if not flagged:
        print("VERDICT: no localized interferer — per-tone metrics uniform "
              "(clean band, or interferer covers the whole channel).")
        return 0
    runs = contiguous(flagged)
    print(f"VERDICT: interferer(s) localized to {len(runs)} tone group(s):")
    for lo, hi in runs:
        f_lo, f_hi = freqs[lo], freqs[hi]
        span = abs(f_hi - f_lo) + res_khz / 1000.0
        cen = (f_lo + f_hi) / 2.0
        print(f"  tones {lo}..{hi}  ~{cen:.2f} {funit} "
              f"(span ~{span:.2f} MHz, {hi - lo + 1} bins)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
