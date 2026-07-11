#!/usr/bin/env python3
"""Animated "warehouse cellular" loop for docs/multi-ap-cellular.md — what the
shared PTP timebase buys at the multi-AP level, in the DEVOURER monitor style.

Top: the wired side — PTP grandmaster + scheduler on an Ethernet backhaul,
with two APs hanging off it. Middle: the floor — two overlapping coverage
cells and a robot. Bottom: the network-wide TDMA slot ruler (one clock for
every cell) with a sweeping "now" cursor.

Acts (cycled by the loop):
  1  UNCOORDINATED   both cells contend for every slot; the overlap zone —
                     where the robot sits — is a collision zone
  2  ICIC            the scheduler assigns orthogonal slots over the backhaul;
                     the cell edge goes clean, the robot gets protected slots
  3  HANDOVER        the robot drives A -> B: per-AP RSSI crosses over, the
                     scheduler moves its slots to B between one slot and the
                     next (make-before-break) — the ghost bar shows the
                     ~100 ms hole an ordinary Wi-Fi roam would punch

    tools/warehouse_cellular_gif.py -o docs/img/warehouse_cellular.gif

Needs Pillow.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, GRID, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

W, H = 920, 700
ACTS = [(0, 26), (26, 52), (52, 104)]
TOTAL = ACTS[-1][1]
GREEN = OK
A_COL, B_COL = CYAN, GREEN
AX, BX, CY, R = 300, 640, 300, 185       # cell centers / floor row / radius
RULER = (44, 500, 876, 560)              # slot ruler box
NSLOT = 16
HO_FRAME = 78                            # handover instant (act-3 frame)


def act_of(fr):
    for i, (a, b) in enumerate(ACTS):
        if a <= fr < b:
            return i, (fr - a) / max(1, b - a - 1)
    return len(ACTS) - 1, 1.0


def ap_glyph(d, x, y, col, label, ch):
    d.line([x, y, x, y - 22], fill=col, width=2)
    d.polygon([(x - 7, y - 22), (x + 7, y - 22), (x, y - 32)], fill=col)
    for r in (10, 17):
        d.arc([x - r, y - r - 34, x + r, y + r - 34], 210, 330, fill=col)
    d.text((x - 28, y + 6), label, font=font(12, True), fill=col)
    d.text((x - 28, y + 22), ch, font=font(10), fill=DIM)


def robot(d, x, y, col):
    d.rectangle([x - 10, y - 8, x + 10, y + 8], outline=col, width=2)
    for wx in (-7, 7):
        d.ellipse([x + wx - 3, y + 6, x + wx + 3, y + 12], outline=col)
    d.line([x, y - 8, x, y - 16], fill=col)
    d.ellipse([x - 2, y - 20, x + 2, y - 16], outline=col)


def rssi_bars(d, x, y, da, db):
    """Two mini RSSI bars by distance to each AP."""
    for i, (dist, col, tag) in enumerate(((da, A_COL, "A"), (db, B_COL, "B"))):
        s = max(0.08, min(1.0, 1.6 - dist / R))
        bx = x + 16, y - 14 + i * 14
        d.text((bx[0], bx[1] - 2), tag, font=font(9), fill=col)
        d.rectangle([bx[0] + 12, bx[1], bx[0] + 12 + int(44 * s), bx[1] + 7],
                    fill=col)
        d.rectangle([bx[0] + 12, bx[1], bx[0] + 56, bx[1] + 7], outline=GRID)


def main() -> int:
    ap_ = argparse.ArgumentParser(description=__doc__)
    ap_.add_argument("-o", "--out", default="warehouse_cellular.gif")
    ap_.add_argument("--ms", type=int, default=115)
    args = ap_.parse_args()

    captions = [
        ("1/3  UNCOORDINATED", "two cells, one contention domain — the overlap "
                               "where the robot works is a collision zone", WARN),
        ("2/3  ICIC OVER THE BACKHAUL", "the scheduler splits the slot grid "
                                        "between cells — orthogonal at the edge "
                                        "(µs guards: shared clock)", CYAN),
        ("3/3  MAKE-BEFORE-BREAK HANDOVER", "RSSI crosses over -> the scheduler "
                                            "reassigns the robot's slots to B — "
                                            "valid clock, zero gap", OK),
    ]

    imgs = []
    for fr in range(TOTAL):
        act, t = act_of(fr)
        im, d = new_frame(W, H)
        chrome(d, W, H, "WAREHOUSE CELLULAR — ONE CLOCK, MANY CELLS", "", fr)

        # --- wired backhaul band ------------------------------------------------
        by = 108
        d.rectangle([28, 84, W - 28, 152], outline=GRID)
        d.text((44, 90), "WIRED BACKHAUL", font=font(12, True), fill=INK)
        d.text((44, 108), "PTP GM +", font=font(11), fill=INK)
        d.text((44, 122), "SCHEDULER", font=font(11), fill=INK)
        d.line([150, by + 18, W - 60, by + 18], fill=INK, width=2)  # the bus
        for x, col in ((AX, A_COL), (BX, B_COL)):
            d.line([x, by + 18, x, 168], fill=col, width=1)
        # scheduler pulses on the bus (acts 2+3, and the HO command)
        if act >= 1:
            px = 150 + ((fr * 37) % (W - 220))
            d.ellipse([px - 3, by + 15, px + 3, by + 21], fill=AMBER)
        if act == 2 and abs(fr - HO_FRAME) < 3:
            d.text((BX - 60, by - 2), ">> HO: robot -> B", font=font(11, True),
                   fill=AMBER)

        # --- floor: cells + robot ------------------------------------------------
        d.rectangle([28, 168, W - 28, 468], outline=GRID)
        for cx, col in ((AX, A_COL), (BX, B_COL)):
            d.ellipse([cx - R, CY - R + 40, cx + R, CY + R - 40], outline=col)
        ap_glyph(d, AX, CY - 6, A_COL, "AP A", "ch 36")
        ap_glyph(d, BX, CY - 6, B_COL, "AP B", "ch 44")

        # robot position: acts 1-2 works the overlap; act 3 drives A -> B
        if act < 2:
            rx = 470 + 14 * math.sin(fr / 3.5)
        else:
            rx = 330 + (620 - 330) * t
        ry = CY + 92
        served_b = act == 2 and fr >= HO_FRAME
        rcol = B_COL if served_b else A_COL
        robot(d, rx, ry, rcol)
        da, db = abs(rx - AX), abs(rx - BX)
        rssi_bars(d, rx, ry, da, db)

        # collisions in the overlap (act 1): both cells hit the robot at once
        if act == 0 and fr % 3 == 0:
            for cx, col in ((AX, A_COL), (BX, B_COL)):
                d.line([cx, CY - 40, rx, ry - 22], fill=col, width=1)
            d.text((rx - 34, ry + 26), "collision", font=font(11, True), fill=WARN)
            d.ellipse([rx - 16, ry - 30, rx + 16, ry + 2], outline=WARN)
        # clean serving link otherwise
        if act >= 1:
            sx = BX if served_b else AX
            d.line([sx, CY - 40, rx, ry - 22], fill=rcol, width=1)

        # --- the network-wide slot ruler ------------------------------------------
        x0, y0, x1, y1 = RULER
        d.rectangle([x0 - 8, y0 - 26, x1 + 8, y1 + 34], outline=GRID)
        d.text((x0, y0 - 22), "network-wide slot grid (one clock, every cell)",
               font=font(11), fill=DIM)
        sw = (x1 - x0) / NSLOT
        now = (fr * 2) % NSLOT
        for s in range(NSLOT):
            sx0, sx1 = x0 + s * sw, x0 + (s + 1) * sw - 3
            if act == 0:
                # both cells claim everything: split fill, collisions at "now"
                d.rectangle([sx0, y0, (sx0 + sx1) / 2, y1], fill=(0, 60, 66))
                d.rectangle([(sx0 + sx1) / 2, y0, sx1, y1], fill=(18, 66, 40))
                if s == now:
                    d.rectangle([sx0, y0, sx1, y1], outline=WARN, width=2)
            else:
                own_b = (s // 2) % 2 == 1        # pairs alternate A/B
                col = (18, 66, 40) if own_b else (0, 60, 66)
                d.rectangle([sx0, y0, sx1, y1], fill=col)
                # the robot's slots (an A pair before HO, a B pair after)
                mine = (s in (4, 5)) if not served_b else (s in (6, 7))
                if mine:
                    d.rectangle([sx0, y0, sx1, y1], outline=rcol, width=2)
                    d.text((sx0 + 4, y1 - 16), "UL" if s % 2 else "DL",
                           font=font(9), fill=rcol)
                if s == now:
                    d.rectangle([sx0, y0, sx1, y1], outline=INK)
        d.text((x0, y1 + 6), "A slots", font=font(10), fill=A_COL)
        d.text((x0 + 70, y1 + 6), "B slots", font=font(10), fill=B_COL)
        # legacy-roam ghost (act 3, after HO): the gap we do NOT take
        if act == 2:
            gx0 = x0 + 8 * sw
            if fr >= HO_FRAME:
                d.rectangle([gx0, y1 + 18, gx0 + 5 * sw, y1 + 28], outline=DIM)
                d.text((gx0 + 6, y1 + 18), "ordinary Wi-Fi roam: ~100 ms dead",
                       font=font(10), fill=DIM)
                d.text((x0 + 140, y1 + 6), "handover gap here: 0 slots",
                       font=font(10, True), fill=OK)

        cap, sub, col = captions[act]
        d.text((28, 54), cap, font=font(14, True), fill=col)
        d.text((300, 57), sub, font=font(11), fill=DIM)

        imgs.append(im)

    save_gif(imgs, args.out, ms=args.ms)
    print(f"wrote {args.out} ({len(imgs)} frames)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
