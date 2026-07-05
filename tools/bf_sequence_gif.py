#!/usr/bin/env python3
"""Animated beamforming self-sounding sequence — how two adapters measure the
channel between them, in the DEVOURER live-monitor style.

    tools/bf_sequence_gif.py -o docs/img/bf_sequence.gif

The sounder announces (NDPA), sends a known waveform on every subcarrier (NDP),
the beamformee measures the per-subcarrier channel it arrived through, and sends
back a compressed CSI report. That report is where the per-subcarrier SNR
waterfall, the per-tone interference localizer, and the motion sensor all come
from — and with two adapters you own, you play both roles yourself
(self-sounding). Needs Pillow.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, GRID, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

# (key, caption, direction) direction: +1 down (sounder->bfee), -1 up, 0 measure
STEPS = [
    ("NDPA", "NDPA — 'I'm about to sound you' (announcement)", +1, AMBER),
    ("NDP", "NDP — a known waveform on every subcarrier", +1, CYAN),
    ("MEASURE", "beamformee measures the per-subcarrier channel H(k)", 0, OK),
    ("CSI REPORT", "compressed CSI report — per-tone angles + SNR", -1, (180, 130, 235)),
]


def device(d, x, y, w, h, label, col, active):
    oc = col if active else (60, 74, 92)
    d.rectangle([x, y, x + w, y + h], outline=oc, width=2)
    d.text((x + 12, y + 10), label, font=font(14, True), fill=col if active else DIM)
    # antennas
    for ax in (x + w - 40, x + w - 22):
        d.line([ax, y, ax, y - 14], fill=oc, width=2)
        d.ellipse([ax - 3, y - 20, ax + 3, y - 14], outline=oc)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="bf_sequence.gif")
    ap.add_argument("--per", type=int, default=13)
    ap.add_argument("--ms", type=int, default=95)
    args = ap.parse_args()

    padL, padT = 34, 96
    panelW = 236
    devW, devH = 360, 66
    airTop, airBot = padT + 40, padT + 40 + 210
    W = padL + devW + 40 + panelW
    H = airBot + devH + 96
    devX = padL + 20

    imgs = []
    for si, (key, cap, direction, col) in enumerate(STEPS):
        for fi in range(args.per):
            t = fi / args.per
            gi = si * args.per + fi
            img, d = new_frame(W, H)
            chrome(d, W, H, "BEAMFORMING SELF-SOUNDING",
                   "two adapters measure the channel between them · you play "
                   "both roles", gi)

            snd_active = direction >= 0 or key == "NDPA"
            bfe_active = direction <= 0 or key == "MEASURE"
            device(d, devX, padT + 12, devW, devH, "SOUNDER  (beamformer)",
                   CYAN, si <= 1)
            device(d, devX, airBot, devW, devH, "BEAMFORMEE  (client)",
                   OK, si >= 2)
            # the air gap
            gy0, gy1 = padT + 12 + devH, airBot
            cxa = devX + devW // 2
            d.line([cxa, gy0, cxa, gy1], fill=(26, 36, 50))
            # label on the far left of the gap — clear of the flying packet and
            # the readout panel on the right
            d.text((devX + 4, (gy0 + gy1) // 2 - 6), "air gap", font=font(10),
                   fill=(60, 78, 96))

            if direction != 0:
                # packet flying along the air gap
                if direction > 0:
                    py = gy0 + t * (gy1 - gy0 - 26)
                else:
                    py = gy1 - 26 - t * (gy1 - gy0 - 26)
                d.rectangle([cxa - 54, py, cxa + 54, py + 24], fill=col,
                            outline=INK)
                d.text((cxa - 46, py + 4), key, font=font(12, True), fill=(10, 12, 18))
                # arrowhead
                ay = py + (24 if direction > 0 else 0)
                d.polygon([(cxa - 6, ay + direction * 8), (cxa + 6, ay + direction * 8),
                           (cxa, ay + direction * 16)], fill=col)
            else:
                # MEASURE: per-subcarrier bars pulsing at the beamformee
                for k in range(24):
                    bx = devX + 60 + k * 11
                    hh = int((6 + 20 * abs(math.sin(k * 0.5 + gi * 0.4))) *
                             (0.4 + 0.6 * t))
                    d.rectangle([bx, airBot - 8 - hh, bx + 8, airBot - 8],
                                fill=OK)
                d.text((devX + 60, airBot - 8 - 44), "H(k) per subcarrier",
                       font=font(10), fill=OK)

            # caption
            d.text((padL, H - 40), cap, font=font(13, True), fill=col)

            # step tracker
            x0 = padL + devW + 36
            d.text((x0, padT - 2), "SEQUENCE", font=font(12), fill=CYAN)
            y = padT + 22
            for j, (k2, _, _, c2) in enumerate(STEPS):
                on = j == si
                d.ellipse([x0, y + 2, x0 + 12, y + 14],
                          fill=c2 if on else (40, 52, 68))
                d.text((x0 + 22, y), f"{j+1}. {k2}",
                       font=font(13, True) if on else font(12),
                       fill=INK if on else DIM)
                y += 30
            y += 10
            d.text((x0, y), "the report feeds:", font=font(11), fill=DIM); y += 18
            for f in ("· SNR waterfall", "· interference localizer",
                      "· motion sensor"):
                d.text((x0, y), f, font=font(11), fill=INK); y += 16
            imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms, colors=64)
    return 0


if __name__ == "__main__":
    sys.exit(main())
