#!/usr/bin/env python3
"""Animated frequency-hopping pattern — 'spreading a link across the band for
diversity', in the DEVOURER live-monitor style.

    tools/hop_pattern_gif.py -o docs/img/hop_pattern.gif

A time–frequency view: time scrolls left, frequency (channels) up the side. The
transmitter hops channel to channel each dwell, so its energy is spread across
the band instead of parked on one frequency. A narrowband interferer sitting on
one channel (the red row) only clips the occasional hop that lands on it — every
other hop escapes it. Per-packet hopping (DEVOURER_HOP_*) doubles as a
frequency-diversity interleaver for the outer FEC. Needs Pillow.
"""
from __future__ import annotations

import argparse
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, GRID, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

CHANS = [36, 40, 44, 48, 52, 56, 60, 64]   # a 5 GHz hop set
JAM = 52                                    # interferer parks here


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="hop_pattern.gif")
    ap.add_argument("--frames", type=int, default=56)
    ap.add_argument("--win", type=int, default=34, help="time cells shown")
    ap.add_argument("--ms", type=int, default=110)
    args = ap.parse_args()
    rnd = random.Random(0x40)

    # a long pseudo-random hop schedule over the channel set
    NT = args.frames + args.win + 4
    sched = []
    last = -1
    for _ in range(NT):
        c = rnd.randrange(len(CHANS))
        while c == last:
            c = rnd.randrange(len(CHANS))
        sched.append(c)
        last = c

    nC = len(CHANS)
    padL, padT, padB = 60, 96, 52
    gw, gh = 560, 300
    panelW = 214
    W = padL + gw + 24 + panelW
    H = padT + gh + padB
    cellw = gw / args.win
    cellh = gh / nC

    def cell_xy(ti, ci):
        x = padL + ti * cellw
        y = padT + (nC - 1 - ci) * cellh
        return x, y

    imgs = []
    collisions = 0
    for fi in range(args.frames):
        img, d = new_frame(W, H)
        chrome(d, W, H, "FREQUENCY HOPPING",
               "time → · channel ↑ · the TX spreads across the band; the red "
               "row is a parked interferer", fi)

        d.rectangle([padL, padT, padL + gw, padT + gh], outline=(0, 70, 80))
        # channel axis + interferer row
        for ci, ch in enumerate(CHANS):
            _, y = cell_xy(0, ci)
            d.text((padL - 34, y + cellh / 2 - 7), f"{ch}", font=font(11),
                   fill=WARN if ch == JAM else DIM)
            if ch == JAM:
                d.rectangle([padL, y, padL + gw, y + cellh], fill=(40, 12, 12))
            d.line([padL, y, padL + gw, y], fill=GRID)
        d.text((6, padT + gh // 2 - 20), "c\nh\na\nn".replace("\n", ""),
               font=font(10), fill=DIM)

        # draw the hop trail across the visible window
        cur_ci = None
        for ti in range(args.win):
            t = fi + ti
            ci = sched[t]
            x, y = cell_xy(ti, ci)
            newest = ti == args.win - 1
            fade = 0.35 + 0.65 * (ti / args.win)
            on_jam = CHANS[ci] == JAM
            base = WARN if on_jam else CYAN
            col = tuple(int(c * fade) for c in base)
            d.rectangle([x + 2, y + 3, x + cellw - 2, y + cellh - 3], fill=col)
            if newest:
                cur_ci = ci
                d.rectangle([x + 1, y + 2, x + cellw - 1, y + cellh - 2],
                            outline=INK, width=2)
                if on_jam:
                    collisions += 1
            # link successive hops with a faint line
            if ti > 0:
                pt = sched[t - 1]
                x0, y0 = cell_xy(ti - 1, pt)
                d.line([x0 + cellw / 2, y0 + cellh / 2, x + cellw / 2,
                        y + cellh / 2], fill=(30, 44, 60))

        d.text((padL, padT + gh + 8), "time →   (each cell = one dwell)",
               font=font(11), fill=DIM)

        # readout
        x0 = padL + gw + 22
        d.text((x0, padT - 2), "LIVE READOUT", font=font(12), fill=CYAN)
        y = padT + 24

        def line(lbl, val, c=INK):
            nonlocal y
            d.text((x0, y), lbl, font=font(11), fill=DIM)
            d.text((x0 + 96, y - 3), val, font=font(16, True), fill=c)
            y += 30

        cur = CHANS[cur_ci]
        nxt = CHANS[sched[fi + args.win]]
        on_jam = cur == JAM
        line("channel", f"{cur}", WARN if on_jam else CYAN)
        line("next", f"{nxt}", DIM)
        line("interferer", f"ch {JAM}", WARN)
        y += 6
        st = "HOP CLIPPED" if on_jam else "CLEAR HOP"
        sc = WARN if on_jam else OK
        d.rectangle([x0, y, x0 + 190, y + 30], outline=sc, width=2)
        d.ellipse([x0 + 8, y + 10, x0 + 18, y + 20], fill=sc)
        d.text((x0 + 28, y + 6), st, font=font(14, True), fill=sc)
        y += 46
        for ln in ("only the hop that lands", "on ch 52 is lost — the",
                   "rest escape. hopping =", "frequency diversity for", "the FEC."):
            d.text((x0, y), ln, font=font(11), fill=INK); y += 15
        imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms, colors=48)
    return 0


if __name__ == "__main__":
    sys.exit(main())
