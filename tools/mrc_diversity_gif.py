#!/usr/bin/env python3
"""Animated antenna-diversity (MRC) monitor — 'why the number of receive chains
is a fade-state lever, not a range lever', in the DEVOURER live-monitor style.

    tools/mrc_diversity_gif.py -o docs/img/mrc_diversity.gif

Two receive antennas each see a fading signal (deep dips as multipath comes and
goes). When the platform is STILL the two antennas see almost the *same* channel
— they fade together, so combining them (maximal-ratio combining) barely fills
the dips and the extra chain is mostly wasted baseline power. Under MOTION the
antennas decorrelate — when one is in a fade the other usually isn't — so the
combined signal fills the deep fades and delivery jumps. That is why the
energy-min controller opens chains up under motion and collapses to one when
still: it adapts to the *fade state*, which the motion sensor reports. Needs
Pillow.
"""
from __future__ import annotations

import argparse
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, GRID, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

NT = 900          # precomputed timeline length
FLOOR_DB = -12.0  # delivery threshold (below = outage)


def smooth_noise(rnd, n, k=18):
    """Filtered Gaussian -> a slow fading envelope factor in [~0,1]."""
    x = [rnd.gauss(0, 1) for _ in range(n + k)]
    out = []
    for i in range(n):
        out.append(sum(x[i:i + k]) / k)
    return out


def env_db(a):
    """Rayleigh-ish envelope in dB from two quadrature fading components."""
    return 10 * math.log10(max(1e-3, a))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="mrc_diversity.gif")
    ap.add_argument("--frames", type=int, default=56)
    ap.add_argument("--win", type=int, default=150, help="samples shown")
    ap.add_argument("--ms", type=int, default=95)
    args = ap.parse_args()
    rnd = random.Random(0x11)

    # Build fading power (linear) for a shared component + two independent ones.
    shared = [abs(v) for v in smooth_noise(rnd, NT, 14)]
    indA = [abs(v) for v in smooth_noise(rnd, NT, 12)]
    indB = [abs(v) for v in smooth_noise(rnd, NT, 12)]

    def branch(t, ind, rho):
        # correlation rho: high -> tracks the shared fade; low -> independent
        p = (rho * shared[t] + (1 - rho) * ind[t])
        return 3.0 * p * p            # power (linear), scaled

    imgs = []
    padL, padT, padB = 34, 92, 46
    gw, gh = 560, 300
    panelW = 250
    W = padL + gw + 24 + panelW
    H = padT + gh + padB
    base_y = padT + gh
    lo, hi = -28.0, 8.0

    def yof(db):
        return base_y - int((db - lo) / (hi - lo) * gh)

    # accumulate outage counts over the whole run (per phase)
    for fi in range(args.frames):
        # first ~40% STATIC (rho high), then MOVING (rho low), eased
        u = fi / args.frames
        moving = u > 0.45
        rho = 0.9 if not moving else 0.12
        t0 = int(u * (NT - args.win))

        img, d = new_frame(W, H)
        chrome(d, W, H, "ANTENNA DIVERSITY · MRC",
               "two antennas fading over time · combined = maximal-ratio "
               "combining · dips below the line = outages", fi)

        d.rectangle([padL, padT, padL + gw, base_y], outline=(0, 70, 80))
        for gd in range(-24, 9, 8):
            y = yof(gd)
            d.line([padL, y, padL + gw, y], fill=GRID)
            d.text((padL - 2, y - 6), f"{gd:+d}", font=font(10), fill=DIM)
        # delivery threshold
        ty = yof(FLOOR_DB)
        d.line([padL, ty, padL + gw, ty], fill=(150, 90, 60))
        d.text((padL + gw - 78, ty - 14), "delivery floor", font=font(10),
               fill=(180, 120, 80))

        ptsA, ptsB, ptsC = [], [], []
        out1 = out2 = 0
        for i in range(args.win):
            t = t0 + i
            pa, pb = branch(t, indA, rho), branch(t, indB, rho)
            da, db = env_db(pa), env_db(pb)
            dc = env_db(pa + pb)              # MRC: branch powers add
            x = padL + int(i / (args.win - 1) * gw)
            ptsA.append((x, yof(da)))
            ptsB.append((x, yof(db)))
            ptsC.append((x, yof(dc)))
            if da < FLOOR_DB:
                out1 += 1
            if dc < FLOOR_DB:
                out2 += 1
        d.line(ptsA, fill=(70, 95, 120), width=1)
        d.line(ptsB, fill=(95, 80, 120), width=1)
        d.line(ptsC, fill=CYAN, width=2)
        # mark combined outages in red
        for (x, y) in ptsC:
            if y > ty:
                d.ellipse([x - 2, ty - 2, x + 2, ty + 2], fill=WARN)

        d.text((padL, base_y + 8), "time →   (signal level, dB)",
               font=font(11), fill=DIM)

        # readout
        x0 = padL + gw + 22
        d.text((x0, padT - 2), "LIVE READOUT", font=font(12), fill=CYAN)
        y = padT + 22
        st = "MOVING" if moving else "STILL"
        sc = OK if moving else AMBER
        d.rectangle([x0, y, x0 + 200, y + 30], outline=sc, width=2)
        d.ellipse([x0 + 8, y + 10, x0 + 18, y + 20], fill=sc)
        d.text((x0 + 28, y + 6), f"PLATFORM {st}", font=font(14, True), fill=sc)
        y += 44

        def line(lbl, val, c=INK):
            nonlocal y
            d.text((x0, y), lbl, font=font(11), fill=DIM)
            d.text((x0 + 128, y - 3), val, font=font(15, True), fill=c)
            y += 28

        corr = int(rho * 100)
        line("antenna corr", f"{corr:3d} %", AMBER if corr > 50 else OK)
        p1 = int(100 * out1 / args.win)
        p2 = int(100 * out2 / args.win)
        line("1 chain out", f"{p1:3d} %", WARN if p1 > 5 else OK)
        line("2 chains out", f"{p2:3d} %", WARN if p2 > 5 else OK)
        y += 8
        # verdict
        if moving:
            v, vc = "chains PAY — combining fills the fades", OK
        else:
            v, vc = "chains barely help — collapse to one", AMBER
        for ln in _wrap(v, 26):
            d.text((x0, y), ln, font=font(11), fill=vc); y += 15
        y += 8
        # legend
        for col, lbl in ((( 70, 95, 120), "antenna A"),
                         ((95, 80, 120), "antenna B"), (CYAN, "combined (MRC)")):
            d.rectangle([x0, y, x0 + 14, y + 4], fill=col)
            d.text((x0 + 22, y - 5), lbl, font=font(11), fill=INK); y += 18
        imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms, colors=32)
    return 0


def _wrap(s, n):
    words, line, out = s.split(), "", []
    for w in words:
        if len(line) + len(w) + 1 > n:
            out.append(line); line = w
        else:
            line = (line + " " + w).strip()
    if line:
        out.append(line)
    return out


if __name__ == "__main__":
    sys.exit(main())
