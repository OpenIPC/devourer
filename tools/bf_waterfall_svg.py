#!/usr/bin/env python3
"""Render a per-subcarrier SNR waterfall from captured MU beamforming reports
to a standalone SVG (for docs — GitHub renders it, and it stays diff-able).

    tools/bf_waterfall_svg.py capture.txt -o docs/img/bf_waterfall.svg \
        --operating-snr 28

Each SVG row is one report (time, top→bottom); each column is one subcarrier
group (frequency); colour is the per-tone SNR mapped to the modulation a
rate-adaptive link would pick (blue QPSK … red 256-QAM).
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bf_report_decode as bf
from bf_waterfall import ramp, qam_for, _QAM_BANDS


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("infile")
    ap.add_argument("-o", "--out", default="bf_waterfall.svg")
    ap.add_argument("--rows", type=int, default=120, help="time rows to render")
    ap.add_argument("--operating-snr", type=float, default=None)
    ap.add_argument("--snr-lo", type=float, default=15.0)
    ap.add_argument("--snr-hi", type=float, default=55.0)
    args = ap.parse_args()

    frames = []
    for line in open(args.infile):
        if "<devourer-bf-report-raw>" in line:
            line = line.split("<devourer-bf-report-raw>", 1)[1]
        f = bf.parse_frame(line.strip())
        if f and f["feedback"]:
            frames.append(f)
    if not frames:
        sys.stderr.write("no MU reports in capture\n"); return 1

    bw, ng = frames[0]["bw"], frames[0]["ng"]
    ns = bf.NS_TABLE[bw][ng]
    vbytes = (ns * 10 + 7) // 8
    series = [s for s in (bf.parse_mu_snr(f, ns, vbytes) for f in frames) if s]
    ncol = max(set(len(s) for s in series), key=[len(s) for s in series].count)
    series = [s[:ncol] for s in series if len(s) >= ncol][:args.rows]

    # global mean for the operating-point shift
    allv = [bf.u8_snr_db(v) for s in series for v in s]
    shift = (args.operating_snr - sum(allv) / len(allv)
             if args.operating_snr is not None else 0.0)

    # geometry
    cw, ch = 26, 5            # cell width/height px
    mL, mT, mR, mB = 70, 70, 30, 78
    title = (f'devourer — per-subcarrier SNR waterfall '
             f'(8822CU MU self-sounding, {20 << bw} MHz)')
    W = max(mL + ncol * cw + mR, 20 + int(len(title) * 9.3) + 20)
    H = mT + len(series) * ch + mB
    lo, hi = args.snr_lo, args.snr_hi

    def rgb(v):
        r, g, b = ramp((v - lo) / (hi - lo))
        return f"#{r:02x}{g:02x}{b:02x}"

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
           f'font-family="monospace" font-size="12">']
    out.append(f'<rect width="{W}" height="{H}" fill="#0b0e14"/>')
    out.append(f'<text x="20" y="26" fill="#e6e6e6" font-size="15" '
               f'font-weight="bold">{title}</text>')
    op = (f'  ·  re-centred to {args.operating_snr:.0f} dB mean'
          if args.operating_snr is not None else '')
    out.append(f'<text x="{mL}" y="44" fill="#9aa5b1">'
               f'{len(series)} reports (time ↓) × {ncol} subcarrier groups '
               f'(frequency →){op}</text>')

    # cells
    for r, s in enumerate(series):
        y = mT + r * ch
        for c, u8 in enumerate(s):
            v = bf.u8_snr_db(u8) + shift
            # width/height +1 so adjacent cells overlap — kills the 1px
            # anti-aliased seams that otherwise show the dark background.
            out.append(f'<rect x="{mL + c * cw}" y="{y}" width="{cw + 1}" '
                       f'height="{ch + 1}" fill="{rgb(v)}"/>')

    # axes
    span = 20 << bw
    for c in range(0, ncol, 4):
        foff = (c - ncol / 2) * (span / ncol)
        out.append(f'<text x="{mL + c * cw + cw/2}" y="{mT - 6}" '
                   f'fill="#9aa5b1" text-anchor="middle">{foff:+.0f}</text>')
    out.append(f'<text x="{mL + ncol*cw/2}" y="{H - 46}" fill="#9aa5b1" '
               f'text-anchor="middle">frequency offset from channel centre '
               f'(MHz)</text>')
    out.append(f'<text x="20" y="{mT + len(series)*ch/2}" fill="#9aa5b1" '
               f'transform="rotate(-90 20 {mT + len(series)*ch/2})" '
               f'text-anchor="middle">time →</text>')

    # legend: QAM bands
    bands = list(reversed(_QAM_BANDS[:-1]))    # low→high
    lw = 88
    x0 = mL
    ly = H - 26
    out.append(f'<text x="{x0}" y="{ly - 8}" fill="#9aa5b1">'
               f'per-tone modulation (SNR → QAM):</text>')
    for i, (thr, name) in enumerate(bands):
        r, g, b = ramp(i / (len(bands) - 1))
        x = x0 + i * lw
        out.append(f'<rect x="{x}" y="{ly}" width="18" height="14" '
                   f'fill="#{r:02x}{g:02x}{b:02x}"/>')
        out.append(f'<text x="{x + 22}" y="{ly + 12}" fill="#e6e6e6">'
                   f'{name}</text>')
    out.append('</svg>')
    with open(args.out, "w") as fh:
        fh.write("\n".join(out))
    print(f"wrote {args.out} ({W}x{H}, {len(series)}x{ncol} cells)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
