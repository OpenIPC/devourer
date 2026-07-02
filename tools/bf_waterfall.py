#!/usr/bin/env python3
"""Live per-subcarrier waterfall from beamforming self-sounding reports.

Pipe WiFiDriverDemo's raw report stream into this for a scrolling, truecolor
per-tone spectrogram in the terminal — the channel across frequency (X) and
time (Y):

    WiFiDriverDemo ... DEVOURER_BF_DETECT_REPORT=4 | tools/bf_waterfall.py

- MU reports (DEVOURER_BF_ARM_BFEE_MU=1): each row is the per-subcarrier SNR;
  the colour ramp doubles as the modulation a rate-adaptive link would pick
  (blue QPSK … red 256-QAM).
- SU reports: each row is the per-tone |h_B/h_A| channel shape.

Reports arrive thousands/sec; rows are aggregated over `--interval` seconds so
the scroll is readable. Ctrl-C to stop.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bf_report_decode as bf


# --- colour ---------------------------------------------------------------
def lerp(a, b, t):
    return tuple(int(a[i] + (b[i] - a[i]) * t) for i in range(3))


# turbo-ish ramp: dark blue -> cyan -> green -> yellow -> orange -> red -> white
_STOPS = [(20, 20, 90), (0, 130, 200), (0, 200, 140), (200, 210, 0),
          (235, 110, 0), (220, 20, 30), (255, 255, 255)]


def ramp(t):
    t = 0.0 if t < 0 else 1.0 if t > 1 else t
    x = t * (len(_STOPS) - 1)
    i = int(x)
    if i >= len(_STOPS) - 1:
        return _STOPS[-1]
    return lerp(_STOPS[i], _STOPS[i + 1], x - i)


def cell(rgb, ch="  "):
    return f"\x1b[48;2;{rgb[0]};{rgb[1]};{rgb[2]}m{ch}\x1b[0m"


# QAM legend keyed to SNR (matches bf_report_decode.snr_to_qam thresholds).
_QAM_BANDS = [(30, "256-QAM"), (24, "64-QAM"), (18, "16-QAM"),
              (11, "QPSK"), (6, "BPSK"), (-99, "off")]


def qam_for(db):
    for thr, name in _QAM_BANDS:
        if db >= thr:
            return name
    return "off"


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--interval", type=float, default=0.15,
                    help="seconds per waterfall row (report-aggregation window)")
    ap.add_argument("--snr-min", type=float, default=None,
                    help="colour-scale floor dB (default: auto)")
    ap.add_argument("--snr-max", type=float, default=None,
                    help="colour-scale ceiling dB (default: auto)")
    ap.add_argument("--operating-snr", type=float, default=None,
                    help="re-centre measured per-tone SNR to this mean dB "
                         "(models a weaker link so the QAM ramp spreads)")
    ap.add_argument("--width", type=int, default=2,
                    help="terminal columns per subcarrier (default 2)")
    args = ap.parse_args()

    ns = None            # subcarrier count of the report stream (set on first)
    vbytes = None        # V-angle byte count (to locate the MU SNR field)
    mode = None          # 'MU' or 'SU'
    bw = 0
    acc, nrow = None, 0  # per-tone accumulator for the current row
    t_row = time.time()
    printed_header = False
    rows_emitted = 0
    lo_auto, hi_auto = 1e9, -1e9

    def emit_header(n):
        # frequency ruler + colour/QAM legend
        span = 20 << bw
        left = "  freq→ "
        ruler = ""
        for k in range(n):
            ruler += ("|" if k % 6 == 0 else " ") * args.width
        print(f"\x1b[1m  devourer beamforming waterfall — "
              f"{mode} report, {n} subcarrier groups across {span} MHz\x1b[0m")
        if mode == "MU":
            leg = "  ".join(cell(ramp(i / 5)) + " " + name
                            for i, (_, name) in enumerate(reversed(_QAM_BANDS[:-1])))
            print("  scale (low→high SNR / QPSK→256-QAM): " + leg)
        else:
            print("  scale: |h_B/h_A|  blue = antenna A stronger, "
                  "red = antenna B stronger")
        print(left + ruler)

    def flush_row():
        nonlocal acc, nrow, rows_emitted, printed_header, lo_auto, hi_auto
        if not acc or nrow == 0:
            return
        vals = [a / nrow for a in acc]
        if mode == "MU":
            dbs = [bf.u8_snr_db(v) for v in vals]
            if args.operating_snr is not None:
                sh = args.operating_snr - sum(dbs) / len(dbs)
                dbs = [d + sh for d in dbs]
            series = dbs
        else:
            series = vals    # |h_B/h_A|
        lo = args.snr_min if args.snr_min is not None else min(series)
        hi = args.snr_max if args.snr_max is not None else max(series)
        lo_auto, hi_auto = min(lo_auto, min(series)), max(hi_auto, max(series))
        if args.snr_min is None and mode == "MU":
            lo, hi = 8.0, 30.0             # fixed SNR scale = fixed QAM colours
        span = (hi - lo) or 1.0
        if not printed_header:
            emit_header(len(series)); printed_header = True
        bar = "".join(cell(ramp((v - lo) / span), " " * args.width)
                      for v in series)
        if mode == "MU":
            mn, mx = min(series), max(series)
            gutter = f"{qam_for(mn)[:4]:>4}→{qam_for(mx)[:4]:<4} "
        else:
            gutter = f"{min(series):.2f}-{max(series):.2f} "
        sys.stdout.write("  " + gutter + bar + "\n")
        sys.stdout.flush()
        acc = [0.0] * len(series); nrow = 0
        rows_emitted += 1

    try:
        for line in sys.stdin:
            if "<devourer-bf-report-raw>" in line:
                line = line.split("<devourer-bf-report-raw>", 1)[1]
            f = bf.parse_frame(line.strip())
            if not f:
                continue
            if ns is None:
                bw = f["bw"]
                ns = bf.NS_TABLE.get(bw, {}).get(f["ng"])
                mode = "MU" if f["feedback"] else "SU"
                vbytes = (ns * 10 + 7) // 8
            # extract this report's per-tone series
            if mode == "MU":
                f["raw"] = bytes.fromhex(line.strip()) if "raw" not in f else f["raw"]
                s = bf.parse_mu_snr(f, ns, vbytes)
            else:
                # SU: reuse the decoder's fixed compact split (bphi=6,bpsi=4)
                layout = bf.angle_layout(f["nr"], f["nc"])
                dec = bf.decode_angles(f["angle_bytes"][:vbytes], ns,
                                       len(layout), 6, 4, layout)
                s = ([__import__("math").tan(d["psi"][0]) for d in dec]
                     if dec else None)
            if not s:
                continue
            if acc is None:
                acc = [0.0] * len(s)
            n = min(len(acc), len(s))
            for k in range(n):
                acc[k] += s[k]
            nrow += 1
            if time.time() - t_row >= args.interval:
                flush_row(); t_row = time.time()
    except KeyboardInterrupt:
        pass
    flush_row()   # final partial row (e.g. end of a file replay)
    if rows_emitted == 0:
        sys.stderr.write("bf_waterfall: no reports decoded — is the sounder + "
                         "armed beamformee running on this channel?\n")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
