#!/usr/bin/env python3
"""Animated burst-level bandwidth TDMA — two stations (TX + RX) flipping
bandwidth in lockstep under a shared GPS clock, in the DEVOURER live-monitor
style.

    tools/tdma_schedule_gif.py -o docs/img/tdma_schedule.gif

The picture a newcomer needs: a transmitter and a receiver, a shared clock (GPS
1-PPS) that ticks both at once, and a schedule of BURSTS. On each tick both ends
flip together — a narrowband burst (thin, amber; critical frames, robust rate)
then a wide burst (full, cyan; bulk frames, fast rate) — and the flip itself
costs only ~0.2 ms (one baseband re-clock), versus ~90 ms for a full retune.
That speed is what makes the alternation practical. The runnable version is
examples/tdma. Needs Pillow.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, GRID, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

NB_CELLS = 6       # narrowband burst length (schedule cells)
WIDE_CELLS = 8     # wide burst length
PERIOD = NB_CELLS + WIDE_CELLS
NPER = 2           # periods drawn on the schedule strip
TOTAL = NPER * PERIOD
BOUNDS = [b for k in range(NPER + 1)
          for b in (k * PERIOD, k * PERIOD + NB_CELLS)][: 2 * NPER + 1]


def phase_nb(cell: int) -> bool:
    return (cell % PERIOD) < NB_CELLS


def dash(d, x0, y0, x1, y1, col, on=7, off=6, w=1):
    n = int(math.hypot(x1 - x0, y1 - y0) // (on + off)) + 1
    for i in range(n):
        a = i * (on + off) / max(1, (n * (on + off)))
        b = min(1.0, (i * (on + off) + on) / max(1, (n * (on + off))))
        d.line([x0 + (x1 - x0) * a, y0 + (y1 - y0) * a,
                x0 + (x1 - x0) * b, y0 + (y1 - y0) * b], fill=col, width=w)


def antenna(d, cx, cy, col):
    d.line([cx, cy, cx, cy + 26], fill=col, width=2)          # mast
    d.polygon([(cx - 6, cy), (cx + 6, cy), (cx, cy - 8)], fill=col)
    for r in (9, 15):                                        # radiating arcs
        d.arc([cx - r, cy - r - 4, cx + r, cy + r - 4], 210, 330, fill=col)


def satellite(d, cx, cy, col):
    d.rectangle([cx - 12, cy - 8, cx + 12, cy + 8], outline=col, width=2)
    for sx in (-30, 18):                                     # solar panels
        d.rectangle([cx + sx, cy - 6, cx + sx + 12, cy + 6], outline=col)
        d.line([cx + sx + 6, cy - 6, cx + sx + 6, cy + 6], fill=col)
    d.line([cx - 18, cy, cx - 12, cy], fill=col)
    d.line([cx + 12, cy, cx + 18, cy], fill=col)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="tdma_schedule.gif")
    ap.add_argument("--frames", type=int, default=56)
    ap.add_argument("--ms", type=int, default=95)
    args = ap.parse_args()

    W, H = 920, 548
    # --- fixed geometry -----------------------------------------------------
    sat = (W // 2, 96)
    tx = dict(box=(40, 150, 415, 320), ant=(150, 150))
    rx = dict(box=(505, 150, 880, 320), ant=(770, 150))
    strip = (44, 372, 876, 436)          # schedule timeline
    sw = float(strip[2] - strip[0]) / TOTAL

    def mode_panel(d, box, nb, sending, cx_ant, label, cls_txt, cls_col):
        x0, y0, x1, y1 = box
        d.rectangle(box, outline=(0, 70, 80))
        antenna(d, cx_ant, y0, INK)
        d.text((x0 + 14, y0 + 12), label, font=font(17, True), fill=CYAN)
        # occupied-bandwidth bar: narrow+amber (NB) vs full+cyan (wide)
        bar_y = y0 + 66
        span = x1 - x0 - 40
        col = AMBER if nb else CYAN
        w = int(span * (0.36 if nb else 0.94))
        bx0 = x0 + 20 + (span - w) // 2
        d.rectangle([x0 + 20, bar_y, x0 + 20 + span, bar_y + 34],
                    outline=GRID)
        d.rectangle([bx0, bar_y, bx0 + w, bar_y + 34], fill=col)
        d.text((x0 + 20, bar_y + 44), "10 MHz" if nb else "20 MHz",
               font=font(15, True), fill=col)
        d.text((x1 - 128, bar_y + 46), "occupied width", font=font(10), fill=DIM)
        verb = "sending" if sending else "decoding"
        d.text((x0 + 20, y1 - 34), f"{verb}", font=font(11), fill=DIM)
        d.text((x0 + 96, y1 - 37), cls_txt, font=font(15, True), fill=cls_col)
        if not sending:
            d.ellipse([x1 - 34, y1 - 36, x1 - 16, y1 - 18], outline=OK, width=2)
            d.line([x1 - 30, y1 - 27, x1 - 26, y1 - 22], fill=OK, width=2)
            d.line([x1 - 26, y1 - 22, x1 - 19, y1 - 32], fill=OK, width=2)

    imgs = []
    for fi in range(args.frames):
        p = (fi / args.frames) * TOTAL          # continuous position (cells)
        cell = int(p) % PERIOD
        nb = phase_nb(int(p))
        last_b = max(b for b in BOUNDS if b <= p + 1e-6)
        d_since = p - last_b                     # cells since last flip
        flipping = d_since < 1.6                 # a tick just happened

        img, d = new_frame(W, H)
        chrome(d, W, H, "BANDWIDTH TDMA",
               "two stations, one clock: TX and RX flip bandwidth together each "
               "burst", fi)

        # --- GPS shared clock + sync pulse ---------------------------------
        satellite(d, sat[0], sat[1], OK if flipping else DIM)
        d.text((sat[0] + 44, sat[1] - 6), "GPS · shared 1-PPS clock",
               font=font(11), fill=OK if flipping else DIM)
        for dest in (tx["ant"], rx["ant"]):
            dash(d, sat[0], sat[1] + 12, dest[0], dest[1] - 12,
                 (0, 90, 70) if not flipping else OK)
            if flipping:                          # pulse travels to the stations
                fr = min(1.0, d_since / 1.4)
                px = sat[0] + (dest[0] - sat[0]) * fr
                py = (sat[1] + 12) + (dest[1] - 12 - sat[1] - 12) * fr
                d.ellipse([px - 4, py - 4, px + 4, py + 4], fill=OK)
        if flipping:
            d.text((sat[0] - 22, sat[1] + 26), "TICK", font=font(11, True), fill=OK)

        # --- TX / RX stations ----------------------------------------------
        cls_txt = "CRITICAL" if nb else "BULK"
        cls_col = AMBER if nb else CYAN
        mode_panel(d, tx["box"], nb, True, tx["ant"][0], "TX", cls_txt, cls_col)
        mode_panel(d, rx["box"], nb, False, rx["ant"][0], "RX", cls_txt, cls_col)

        # a frame in flight across the air gap (TX -> RX), colour = class
        gy = 250
        fx = 415 + ((fi * 22) % 90)
        d.rectangle([fx, gy - 6, fx + 26, gy + 6], fill=cls_col)
        d.polygon([(fx + 26, gy - 8), (fx + 40, gy), (fx + 26, gy + 8)], fill=cls_col)

        # --- schedule strip + playhead -------------------------------------
        sx0, sy0, sx1, sy1 = strip
        d.text((sx0, sy0 - 20), "SCHEDULE  (repeats — both ends run it)",
               font=font(11), fill=CYAN)
        for c in range(TOTAL):
            cnb = phase_nb(c)
            x = sx0 + c * sw
            base = AMBER if cnb else CYAN
            col = tuple(int(v * 0.42) for v in base)
            d.rectangle([x + 1, sy0, x + sw - 1, sy1], fill=col)
        for b in BOUNDS:                            # burst boundaries
            d.line([sx0 + b * sw, sy0 - 4, sx0 + b * sw, sy1 + 4], fill=(0, 70, 80))
        phx = sx0 + p * sw                          # the 'now' playhead
        d.line([phx, sy0 - 8, phx, sy1 + 8], fill=INK, width=2)
        d.polygon([(phx - 5, sy0 - 12), (phx + 5, sy0 - 12), (phx, sy0 - 4)], fill=INK)
        if flipping:
            d.text((min(phx + 6, sx1 - 66), sy0 + 6), "flip 0.2 ms",
                   font=font(11, True), fill=OK)
        d.text((sx0, sy1 + 8),
               "amber = narrowband burst (critical, robust) · "
               "cyan = wide burst (bulk, fast)", font=font(10), fill=DIM)

        # --- switch-cost gauge (the achievement) ---------------------------
        gx, gyb = 44, 468
        d.text((gx, gyb), "SWITCH COST", font=font(11), fill=CYAN)
        d.text((gx + 118, gyb), "FastSetBandwidth  0.2 ms", font=font(11, True), fill=OK)
        d.rectangle([gx + 320, gyb + 1, gx + 322, gyb + 13], fill=OK)   # 0.2 ms
        d.text((gx + 118, gyb + 18), "full SetMonitorChannel  90 ms",
               font=font(11), fill=DIM)
        d.rectangle([gx + 320, gyb + 19, gx + 320 + 300, gyb + 31], fill=(60, 40, 40))
        d.text((gx + 118 + 300 + 26, gyb + 18), "~450×", font=font(11, True), fill=WARN)
        imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms, colors=48)
    return 0


if __name__ == "__main__":
    sys.exit(main())
