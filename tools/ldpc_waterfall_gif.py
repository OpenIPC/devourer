#!/usr/bin/env python3
"""Animated LDPC coding gain — 'what the error-correcting code buys', in the
DEVOURER live-monitor style.

    tools/ldpc_waterfall_gif.py -o docs/img/ldpc_waterfall.gif

Section 3 of the RF primer walks a block of bits through the transmit pipeline
and mentions forward error correction in passing. This is the payoff: sweep the
transmit power down until the link falls over, and plot delivery against power
for the two codes 802.11 offers. Both curves fall off a cliff — that shape is
why it is called a waterfall — but the stronger code's cliff sits to the left.
The horizontal distance between them, read at the 10 %-delivery crossing, is the
coding gain: about 3 dB at MCS7 / 20 MHz on the bench.

The second half is the part that decides whether you can use it: LDPC only pays
if the receiver decodes it, and on this hardware that is not uniform. The truth
table is bench-derived (src/AdapterCaps.h), deliberately not the vendor driver's
interop-advertisement policy, which claims none of the 11ac parts can.

The curve SHAPE here is illustrative; the measured quantity is the gap, and the
figure says so on its face. Needs Pillow.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, GRID, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

STEPS = 32          # TX power indices swept, 0.25 dB apart -> 8 dB of travel
GAIN_STEPS = 12     # 12 quarter-dB steps = the ~3 dB the bench recorded
BCC_MID, SHARP = 20.0, 2.2

# chip, HT-LDPC, VHT-LDPC, per-frame flag  (src/AdapterCaps.h + the HAL tables)
TRUTH = [
    ("RTL8812A / 8811A", True, True, True),
    ("RTL8814A", True, True, False),
    ("RTL8821A", True, False, True),
    ("RTL8822B / 8821C", True, True, True),
    ("RTL8822C / 8822E", True, True, True),
]


def delivery(p, mid):
    return 1.0 / (1.0 + math.exp(-(p - mid) / SHARP))


def crossing(mid, frac=0.10):
    """The power index where the curve passes `frac` delivery."""
    return mid + SHARP * math.log(frac / (1.0 - frac))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="ldpc_waterfall.gif")
    ap.add_argument("--hold", type=int, default=18)
    ap.add_argument("--ms", type=int, default=110)
    args = ap.parse_args()

    padL, padT = 92, 100
    pw, ph = 500, 212
    panelW = 316
    W = padL + pw + 32 + panelW
    H = padT + ph + 128
    ldpc_mid = BCC_MID - GAIN_STEPS

    def px(p):
        return padL + (p / STEPS) * pw

    def py(v):
        return padT + ph - v * ph

    DRAW, MARK = 30, 12          # frames to draw the curves / show the gap
    total = DRAW + MARK + len(TRUTH) * 4

    imgs = []
    for fi in range(total + args.hold):
        drawn = min(1.0, (fi + 1) / DRAW)
        marked = fi >= DRAW
        rows = max(0, (fi - DRAW - MARK) // 4 + 1) if fi >= DRAW + MARK else 0
        img, d = new_frame(W, H)
        chrome(d, W, H, "LDPC CODING GAIN",
               "sweep the power down until the link breaks — the stronger code "
               "breaks later", fi)

        # axes
        d.rectangle([padL, padT, padL + pw, padT + ph], outline=(0, 70, 80))
        for v, lbl in ((1.0, "100%"), (0.5, "50%"), (0.10, "10%"), (0.0, "0")):
            yy = py(v)
            d.line([padL, yy, padL + pw, yy],
                   fill=(60, 50, 30) if v == 0.10 else GRID)
            d.text((padL - 42, yy - 7), lbl, font=font(10),
                   fill=AMBER if v == 0.10 else DIM)
        d.text((padL - 78, padT + ph / 2 - 24), "d\ne\nl\ni\nv\ne\nr\ny",
               font=font(10), fill=DIM)
        d.text((padL, padT + ph + 10),
               "TX power index  (one step = 0.25 dB)  →", font=font(11), fill=DIM)

        # the two waterfalls
        for mid, col, tag in ((BCC_MID, AMBER, "BCC"), (ldpc_mid, CYAN, "LDPC")):
            pts = []
            n = int(STEPS * drawn * 4)
            for j in range(max(2, n)):
                p = j / 4.0
                pts.append((px(p), py(delivery(p, mid))))
            if len(pts) > 1:
                d.line(pts, fill=col, width=3)
            if drawn > 0.55:
                d.text((px(mid) + 10, py(delivery(mid, mid)) - 20), tag,
                       font=font(12, True), fill=col)

        # the gap at the 10% crossing — the thing that was actually measured
        if marked:
            cb, cl = crossing(BCC_MID), crossing(ldpc_mid)
            yy = py(0.10)
            for c, col in ((cb, AMBER), (cl, CYAN)):
                d.line([px(c), yy - 9, px(c), yy + 9], fill=col, width=2)
            d.line([px(cl), yy - 22, px(cb), yy - 22], fill=INK)
            for c, dx in ((cl, 5), (cb, -5)):
                d.line([px(c), yy - 22, px(c) + dx, yy - 26], fill=INK)
                d.line([px(c), yy - 22, px(c) + dx, yy - 18], fill=INK)
            mid_x = (px(cl) + px(cb)) / 2
            d.rectangle([mid_x - 30, yy - 41, mid_x + 30, yy - 25], fill=(8, 11, 18))
            d.text((mid_x - 26, yy - 40), "~3 dB", font=font(13, True), fill=INK)

        d.text((padL, padT + ph + 30),
               "curve shape illustrative — the measured quantity is this gap, "
               "at MCS7 / 20 MHz", font=font(10), fill=(70, 84, 104))
        d.text((padL, padT + ph + 46),
               "harness: tests/ldpc_waterfall.sh", font=font(10), fill=(70, 84, 104))

        # ---- who can actually decode it ------------------------------------
        x0 = padL + pw + 30
        d.text((x0, padT - 26), "…IF THE RECEIVER DECODES IT", font=font(12),
               fill=CYAN)
        d.text((x0 + 168, padT - 4), "HT", font=font(10), fill=DIM)
        d.text((x0 + 204, padT - 4), "VHT", font=font(10), fill=DIM)
        d.text((x0 + 248, padT - 4), "flag", font=font(10), fill=DIM)
        y = padT + 16
        for i, (chip, ht, vht, flag) in enumerate(TRUTH):
            if i >= rows:
                break
            d.text((x0, y), chip, font=font(11), fill=INK)
            for dx, ok in ((172, ht), (210, vht), (254, flag)):
                col = OK if ok else WARN
                d.ellipse([x0 + dx, y + 2, x0 + dx + 10, y + 12], fill=col)
            y += 24
        if rows >= len(TRUTH):
            y += 6
            for ln in ("green = the baseband decodes it.",
                       "the 8821A misses VHT-LDPC only —",
                       "HT is fine. the 8814A decodes",
                       "both but reports no per-frame",
                       "flag, so you cannot see it in",
                       "the receive telemetry.",
                       "",
                       "bench-derived, not the vendor",
                       "driver's advertised policy —",
                       "which claims none of these can."):
                d.text((x0, y), ln, font=font(10), fill=DIM if ln.startswith(
                    ("bench", "which", "driver")) else INK)
                y += 14

        imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms, colors=48)
    return 0


if __name__ == "__main__":
    sys.exit(main())
