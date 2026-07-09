#!/usr/bin/env python3
"""Animated burst-level bandwidth TDMA — 'the link breathes between a robust
narrowband burst and a wide throughput burst', in the DEVOURER live-monitor
style.

    tools/tdma_schedule_gif.py -o docs/img/tdma_schedule.gif

A time-frequency view: time scrolls left, frequency (the 20 MHz channel span) up
the side. The transmitter alternates BURSTS — a narrowband burst (a thin central
band, ~10 MHz, carrying the critical frames at a robust rate) and a wide burst
(the full 20 MHz block, carrying bulk frames at a fast MCS). Both ends flip
bandwidth together in a fraction of a millisecond (FastSetBandwidth), so the
occupied width "breathes" over time. The example is examples/tdma. Needs Pillow.
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, GRID, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

NB_CELLS = 6      # narrowband burst length (time cells)
WIDE_CELLS = 8    # wide burst length
PERIOD = NB_CELLS + WIDE_CELLS
NROWS = 24        # frequency rows = the 20 MHz span
NB_ROWS = 10      # narrowband occupies a ~10 MHz central band


def phase_nb(t: int) -> bool:
    """True during the narrowband burst of the period containing time cell t."""
    return (t % PERIOD) < NB_CELLS


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="tdma_schedule.gif")
    ap.add_argument("--frames", type=int, default=56)
    ap.add_argument("--win", type=int, default=34, help="time cells shown")
    ap.add_argument("--ms", type=int, default=110)
    args = ap.parse_args()

    padL, padT, padB = 60, 96, 52
    gw, gh = 560, 300
    panelW = 214
    W = padL + gw + 24 + panelW
    H = padT + gh + padB
    cellw = gw / args.win
    cellh = gh / NROWS
    r_lo = (NROWS - NB_ROWS) // 2
    r_hi = r_lo + NB_ROWS  # central narrowband band [r_lo, r_hi)

    def cell_x(ti):
        return padL + ti * cellw

    def row_y(r):
        # r=0 at the bottom of the span, r=NROWS at the top
        return padT + (NROWS - r) * cellh

    imgs = []
    for fi in range(args.frames):
        img, d = new_frame(W, H)
        chrome(d, W, H, "BANDWIDTH TDMA",
               "time → · frequency ↑ · the link alternates a narrowband "
               "(critical) and a wide (bulk) burst", fi)

        d.rectangle([padL, padT, padL + gw, padT + gh], outline=(0, 70, 80))
        # frequency-span bracket + label
        d.line([padL - 20, padT, padL - 20, padT + gh], fill=(0, 70, 80))
        d.text((6, padT + gh // 2 - 30), "2\n0\nM\nH\nz".replace("\n", ""),
               font=font(10), fill=DIM)
        # faint band guides marking the narrowband slice edges
        for r in (r_lo, r_hi):
            y = row_y(r)
            d.line([padL, y, padL + gw, y], fill=GRID)

        # draw the breathing occupancy across the visible window
        cur_nb = True
        for ti in range(args.win):
            t = fi + ti
            nb = phase_nb(t)
            newest = ti == args.win - 1
            fade = 0.32 + 0.68 * (ti / args.win)
            x = cell_x(ti)
            if nb:
                base = AMBER
                y0, y1 = row_y(r_hi), row_y(r_lo)
            else:
                base = CYAN
                y0, y1 = row_y(NROWS), row_y(0)
            col = tuple(int(c * fade) for c in base)
            d.rectangle([x + 1.5, y0 + 2, x + cellw - 1.5, y1 - 2], fill=col)
            # a faint packet texture: two brighter ticks per cell
            tick = tuple(min(255, int(c * min(1.0, fade + 0.25))) for c in base)
            midx = x + cellw / 2
            d.line([midx, y0 + 4, midx, y1 - 4], fill=tick)
            if newest:
                cur_nb = nb
                d.rectangle([x + 1, y0 + 1, x + cellw - 1, y1 - 1],
                            outline=INK, width=2)

        d.text((padL, padT + gh + 8),
               "time →   (narrowband burst = critical @ robust rate · "
               "wide burst = bulk @ fast MCS)", font=font(11), fill=DIM)

        # readout panel
        x0 = padL + gw + 22
        d.text((x0, padT - 2), "LIVE READOUT", font=font(12), fill=CYAN)
        y = padT + 24

        def line(lbl, val, c=INK):
            nonlocal y
            d.text((x0, y), lbl, font=font(11), fill=DIM)
            d.text((x0 + 96, y - 3), val, font=font(16, True), fill=c)
            y += 30

        acc = AMBER if cur_nb else CYAN
        line("phase", "NARROWBAND" if cur_nb else "WIDE", acc)
        line("width", "10 MHz" if cur_nb else "20 MHz", acc)
        line("carrying", "CRITICAL" if cur_nb else "BULK", acc)
        line("rate", "6M robust" if cur_nb else "MCS7 fast", DIM)
        y += 6
        st = "ROBUST LINK" if cur_nb else "HIGH THROUGHPUT"
        d.rectangle([x0, y, x0 + 190, y + 30], outline=acc, width=2)
        d.ellipse([x0 + 8, y + 10, x0 + 18, y + 20], fill=acc)
        d.text((x0 + 28, y + 8), st, font=font(13, True), fill=acc)
        y += 46
        for ln in ("TX + RX flip together", "in ~0.2 ms (one 0x8ac", "write) — a receiver",
                   "decodes one width at a", "time, so the schedule", "is burst-level."):
            d.text((x0, y), ln, font=font(11), fill=INK); y += 15
        imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms, colors=48)
    return 0


if __name__ == "__main__":
    sys.exit(main())
