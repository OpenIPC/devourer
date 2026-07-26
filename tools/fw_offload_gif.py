#!/usr/bin/env python3
"""Animated firmware IO-offload — 'who walks the registers', in the DEVOURER
live-monitor style.

    tools/fw_offload_gif.py -o docs/img/fw_offload.gif

Section 6 of the driver primer introduces H2C as a message pipe. This is the
same pipe used as a *speed* technique. Changing channel means writing a short
list of registers. Done the ordinary way each write is a USB vendor-control
transfer, and the host pays a bus round-trip per write — the registers are
cheap, the bus is not. Done as an offload, the host sends one H2C carrying the
whole list and the chip's own CPU replays it locally, with nothing crossing the
bus in between.

The honest half: this frees the host, not the radio. The synthesizer still has
to settle, and that floor does not move no matter who wrote the registers.
Needs Pillow.
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, GRID, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

NW = 10          # register writes the channel switch needs
ACT = 34         # frames to play one lane through


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="fw_offload.gif")
    ap.add_argument("--hold", type=int, default=20)
    ap.add_argument("--ms", type=int, default=110)
    args = ap.parse_args()

    padL, padT = 104, 100
    gw = 548
    hA = padT + 24
    cA = hA + 80
    hB = cA + 74
    cB = hB + 80
    panelW = 300
    W = padL + gw + 30 + panelW
    H = cB + 92
    bandH = 26

    def wx(i, span=0.62, start=0.05):
        return padL + gw * (start + span * (i / max(1, NW - 1)))

    imgs = []
    for fi in range(ACT + args.hold):
        prog = min(1.0, (fi + 1) / ACT)
        cut = padL + gw * prog
        img, d = new_frame(W, H)
        chrome(d, W, H, "FIRMWARE IO-OFFLOAD",
               "the registers are cheap; the bus is not — so send the list "
               "once and let the chip replay it", fi)

        for (hy, cy, title, tcol) in (
                (hA, cA, "HOST DRIVES — one USB round-trip per register", AMBER),
                (hB, cB, "OFFLOADED — one H2C carries the whole write list", OK)):
            d.text((padL, hy - 32), title, font=font(11), fill=tcol)
            # the thin bar above HOST is how long the host itself is occupied
            d.text((padL - 54, hy - 15), "busy", font=font(10), fill=(70, 84, 104))
            for by, lbl, col in ((hy, "HOST", CYAN), (cy, "CHIP", DIM)):
                d.rectangle([padL, by, padL + gw, by + bandH], outline=(0, 70, 80))
                d.text((padL - 54, by + 6), lbl, font=font(10), fill=col)
            # the bus, drawn as the space the arrows have to cross
            d.text((padL - 54, (hy + cy) / 2 - 4), "bus", font=font(10),
                   fill=(70, 84, 104))
            d.line([padL, (hy + cy) / 2 + bandH / 2, padL + gw,
                    (hy + cy) / 2 + bandH / 2], fill=GRID)

        # ---- lane A: a round trip per write --------------------------------
        busy_to = padL
        for i in range(NW):
            x = wx(i)
            if x > cut:
                break
            busy_to = max(busy_to, x + 16)
            d.rectangle([x - 5, hA + 4, x + 5, hA + bandH - 4], fill=AMBER)
            d.line([x, hA + bandH, x, cA], fill=(120, 100, 40))
            d.rectangle([x - 4, cA + 6, x + 4, cA + bandH - 6], fill=(120, 100, 40))
            d.line([x + 9, cA, x + 9, hA + bandH], fill=(70, 60, 30))
        if busy_to > padL:
            bar_end = min(busy_to, cut)
            d.rectangle([padL + 2, hA - 9, bar_end, hA - 5], fill=AMBER)


        # ---- lane B: one message, then a local replay ----------------------
        h2c_x0, h2c_x1 = wx(0) - 8, wx(0) + 54
        if cut > h2c_x0:
            x1 = min(h2c_x1, cut)
            d.rectangle([h2c_x0, hB + 4, x1, hB + bandH - 4], fill=OK)
            if x1 - h2c_x0 > 40:
                d.text((h2c_x0 + 5, hB + 7), "H2C", font=font(11, True),
                       fill=(8, 11, 18))
            d.rectangle([padL + 2, hB - 9, x1, hB - 5], fill=OK)
        if cut > h2c_x1:
            d.line([h2c_x1 - 6, hB + bandH, h2c_x1 - 6, cB], fill=(40, 140, 96))
            for i in range(NW):   # the same writes, replayed inside the chip
                x = h2c_x1 + 8 + i * 13
                if x > cut or x > padL + gw - 10:
                    break
                d.rectangle([x, cB + 6, x + 7, cB + bandH - 6], fill=OK)
            if prog > 0.6:
                d.text((h2c_x1 + 8, cB + bandH + 6),
                       "replayed on-chip — nothing crosses the bus",
                       font=font(10), fill=OK)


        # ---- the floor that offload cannot move ----------------------------
        if prog > 0.72:
            fx = padL + gw * 0.84   # the floor applies to both lanes
            d.rectangle([fx, hA - 12, padL + gw, cB + bandH + 2],
                        outline=(90, 78, 30))
            hx = fx + 5
            while hx < padL + gw:
                d.line([hx, hA - 12, hx - 8, cB + bandH + 2], fill=(52, 46, 26))
                hx += 14
            tx0 = padL + gw * 0.30
            d.text((tx0, cB + bandH + 22), "and then the synthesizer still has "
                   "to settle — same for both", font=font(10), fill=AMBER)
            d.line([tx0 + 320, cB + bandH + 26, fx, cB + bandH + 26],
                   fill=(90, 78, 30))

        # ---- readout -------------------------------------------------------
        x0 = padL + gw + 28
        d.text((x0, padT - 26), "MEASURED — 8822B CHANNEL SWITCH", font=font(12),
               fill=CYAN)
        y = padT
        for lbl, val, col in (
                ("full re-init", "~65 ms", WARN),
                ("host fast retune", "~2.5 ms", AMBER),
                ("firmware H2C 0x1D", "1.03 ms", OK)):
            d.text((x0, y), lbl, font=font(11), fill=DIM)
            d.text((x0 + 152, y - 3), val, font=font(14, True), fill=col)
            y += 28
        d.text((x0, y), "median dead air, on-air", font=font(10),
               fill=(70, 84, 104))
        y += 30

        d.text((x0, y), "SAME LEVER, KESTREL", font=font(12), fill=CYAN)
        y += 24
        for lbl, val, col in (("host cost", "9 ms → 0.15 ms", OK),
                              ("time to usable", "~5 ms → ~1.7 ms", AMBER),
                              ("settle floor", "~1.5 ms", WARN)):
            d.text((x0, y), lbl, font=font(11), fill=DIM)
            d.text((x0 + 152, y - 3), val, font=font(12, True), fill=col)
            y += 26
        y += 8
        for ln in ("twenty round-trips collapse", "into one bulk-OUT — but the",
                   "radio is unmoved. offload buys", "host time, and only the part",
                   "of the dead air that was the", "bus's fault."):
            d.text((x0, y), ln, font=font(10), fill=INK)
            y += 14

        imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms, colors=48)
    return 0


if __name__ == "__main__":
    sys.exit(main())
