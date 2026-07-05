#!/usr/bin/env python3
"""Animated spectrum analyzer — a bare CW tone vs a modulated carrier, in the
DEVOURER live-monitor style.

    tools/spectrum_compare_gif.py -o docs/img/spectrum_compare.gif

A quiet channel, then a CW tone rises (all the energy at ONE frequency — a single
spike, ~zero bandwidth), then a modulated carrier rises (the energy spread across
the whole 20 MHz — a flat OFDM block). This is the difference between
DEVOURER_CW_TONE (a narrowband probe / interferer) and DEVOURER_CONT_TX (a
full-channel stimulus). The trace levels are the real ones a USRP B210 measured
on ch100: a −25 dB noise floor, the tone spiking ~+18 dB above it, the modulated
block ~+28 dB and ~20 MHz wide. Needs Pillow.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, GRID, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

FLOOR = -25.0      # measured noise floor (dB)
TONE_PK = -7.0     # measured CW-tone peak
MOD_PK = 3.0       # measured modulated-block level
NBIN = 200         # frequency bins across the display
SPAN = 30.0        # MHz shown (channel-relative, -15..+15)


def floor_trace(seed):
    import random
    r = random.Random(seed)
    return [FLOOR + r.uniform(-1.2, 1.2) for _ in range(NBIN)]


def freq_of(i):
    return (i / (NBIN - 1) - 0.5) * SPAN


def tone_shape(i):
    """narrow Gaussian spike at centre (~0.4 MHz wide)."""
    f = freq_of(i)
    return math.exp(-(f / 0.5) ** 2)


def mod_shape(i):
    """flat ~20 MHz block with steep OFDM shoulders + slight ripple."""
    f = freq_of(i)
    edge = 10.0
    roll = 1.0 / (1.0 + math.exp((abs(f) - edge) * 3.0))   # brick-ish
    ripple = 1.0 + 0.06 * math.sin(f * 2.1)
    return roll * ripple


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="spectrum_compare.gif")
    ap.add_argument("--per", type=int, default=14, help="frames per phase")
    ap.add_argument("--ms", type=int, default=95)
    ap.add_argument("--channel", type=int, default=100)
    args = ap.parse_args()

    base = floor_trace(7)
    tshape = [tone_shape(i) for i in range(NBIN)]
    mshape = [mod_shape(i) for i in range(NBIN)]

    def smooth(t):
        return t * t * (3 - 2 * t)

    # scenario: (label, tone_level, mod_level) keyframes, eased between
    KEYS = [("QUIET", 0, 0), ("CW TONE", 1, 0), ("QUIET", 0, 0),
            ("MODULATED", 0, 1), ("QUIET", 0, 0)]

    seq = []  # (trace[], label, tone_lv, mod_lv)
    for k in range(len(KEYS) - 1):
        (l0, t0, m0), (l1, t1, m1) = KEYS[k], KEYS[k + 1]
        for fi in range(args.per):
            e = smooth(fi / args.per)
            tl = t0 + (t1 - t0) * e
            ml = m0 + (m1 - m0) * e
            lbl = l1 if e > 0.5 else l0
            tr = []
            for i in range(NBIN):
                lin = 10 ** (base[i] / 10)
                lin += tl * 10 ** ((TONE_PK) / 10) * tshape[i]
                lin += ml * 10 ** ((MOD_PK) / 10) * mshape[i]
                tr.append(10 * math.log10(lin))
            seq.append((tr, lbl, tl, ml))

    padL, padT, padB = 34, 92, 52
    grid_w, grid_h = 620, 300
    panelW = 200
    W = padL + grid_w + 26 + panelW
    H = padT + grid_h + padB
    base_y = padT + grid_h
    lo_db, hi_db = -30.0, 8.0

    def yof(db):
        return base_y - int((db - lo_db) / (hi_db - lo_db) * grid_h)

    imgs = []
    for fi, (tr, lbl, tl, ml) in enumerate(seq):
        img, d = new_frame(W, H)
        chrome(d, W, H, "SPECTRUM ANALYZER",
               f"ch {args.channel} · {5000 + 5*args.channel} MHz · 30 MHz span · "
               "power vs frequency", fi)

        # grid + dB axis
        for gd in range(-30, 9, 10):
            y = yof(gd)
            d.line([padL, y, padL + grid_w, y], fill=GRID)
            d.text((padL - 2, y - 6), f"{gd:+d}", font=font(10), fill=DIM)
        # 20 MHz channel edges
        for fq in (-10, 10):
            x = padL + int((fq / SPAN + 0.5) * grid_w)
            d.line([x, padT, x, base_y], fill=(40, 60, 80))
        d.text((padL + grid_w // 2 - 60, padT + 4), "20 MHz channel",
               font=font(10), fill=(70, 100, 130))

        # filled spectrum trace
        col = AMBER if tl > 0.3 else (CYAN if ml > 0.3 else (90, 130, 150))
        pts = [(padL + int(i / (NBIN - 1) * grid_w), yof(tr[i]))
               for i in range(NBIN)]
        d.polygon([(padL, base_y)] + pts + [(padL + grid_w, base_y)],
                  fill=(col[0] // 6, col[1] // 6, col[2] // 6))
        d.line(pts, fill=col, width=2)

        # frequency axis
        for fq in range(-12, 13, 6):
            x = padL + int((fq / SPAN + 0.5) * grid_w)
            d.text((x - 8, base_y + 6), f"{fq:+d}", font=font(10), fill=DIM)
        d.text((padL, base_y + 26), "frequency offset from channel centre (MHz)",
               font=font(11), fill=DIM)

        # readout
        x0 = padL + grid_w + 24
        peak = max(tr)
        # crude occupied-BW: span of bins within 10 dB of peak, above floor+6
        occ = sum(1 for v in tr if v > peak - 10 and v > FLOOR + 6) / NBIN * SPAN
        d.text((x0, padT - 2), "LIVE READOUT", font=font(12), fill=CYAN)
        y = padT + 24

        def line(lb, val, c=INK):
            nonlocal y
            d.text((x0, y), lb, font=font(11), fill=DIM)
            d.text((x0 + 96, y - 3), val, font=font(15, True), fill=c)
            y += 30

        typ = "CW TONE" if tl > 0.5 else ("MODULATED" if ml > 0.5 else "— none —")
        tc = AMBER if tl > 0.5 else (CYAN if ml > 0.5 else DIM)
        line("signal", lbl, tc)
        line("peak", f"{peak:+4.1f} dB")
        line("occupied", f"{occ:4.1f} MHz", tc if occ > 0.5 or tl > 0.5 else DIM)
        y += 8
        note = ("all energy at ONE\nfrequency — a spike,\n~zero bandwidth"
                if tl > 0.5 else
                "energy spread over\nthe whole channel —\na flat OFDM block"
                if ml > 0.5 else
                "just the noise floor")
        for ln in note.split("\n"):
            d.text((x0, y), ln, font=font(11), fill=INK); y += 16
        y += 12
        d.text((x0, y), "CW_TONE = probe", font=font(10), fill=AMBER); y += 15
        d.text((x0, y), "CONT_TX = stimulus", font=font(10), fill=CYAN)
        imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms)
    return 0


if __name__ == "__main__":
    sys.exit(main())
