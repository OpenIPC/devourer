#!/usr/bin/env python3
"""Animated chip anatomy — 'five machines in one package', in the DEVOURER
live-monitor style (docs/driver-primer.md §1).

    tools/chip_anatomy_gif.py -o docs/img/chip_anatomy.gif

A USB Wi-Fi dongle is one chip with five machines chained like an assembly
line: USB interface → MAC (with the WCPU firmware core inside) → BB (digital
PHY) → RF (analog PHY) → PA/LNA front end → antenna. The digital blocks
(MAC+BB) and the analog block (RF) are frequently separate silicon dies in one
package — the D-die and the A-die. Phase 1 walks a TX frame left→right through
every machine; phase 2 walks an RX frame back right→left. Needs Pillow.
"""
from __future__ import annotations

import argparse
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

# (name, width, tx description, rx description)
BLOCKS = [
    ("USB IF", 108, "frame arrives from the host over a bulk endpoint",
     "aggregated into a bulk transfer, back up to the host"),
    ("MAC", 148, "queue, TX descriptor, ACK timing — the WCPU runs here",
     "RX filter, address match, RX descriptor — a frame again"),
    ("BB", 116, "digital PHY: scramble, FEC, QAM, IFFT — the OFDM modem",
     "AGC, FFT, equalize, demap, decode — samples back to bits"),
    ("RF", 116, "synthesizer + mixers move baseband to channel frequency",
     "mixers bring the channel frequency down to baseband"),
    ("PA/LNA", 116, "the PA gives the TX signal its final watts",
     "the LNA gives the faint signal its first clean boost"),
    ("ANT", 88, "…and it radiates", "a faint frame arrives at the antenna"),
]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="chip_anatomy.gif")
    ap.add_argument("--per", type=int, default=8)
    ap.add_argument("--ms", type=int, default=110)
    args = ap.parse_args()
    rnd = random.Random(0xC1)

    W, H = 900, 452
    padL = 34
    n = len(BLOCKS)
    row_y, box_h = 168, 84
    gap = 14

    # block x positions
    xs, x = [], padL
    for name, bw, _, _ in BLOCKS:
        xs.append((x, x + bw))
        x += bw + gap
    centers = [(a + b) // 2 for a, b in xs]

    imgs = []
    phases = [
        (0, "PHASE 1 — TX: host frame -> radio wave  (left to right)"),
        (1, "PHASE 2 — RX: radio wave -> host frame  (right to left)"),
    ]
    for phase, phase_label in phases:
        order = list(range(n)) if phase == 0 else list(range(n - 1, -1, -1))
        for si, blk in enumerate(order):
            for fi in range(args.per):
                gi = (phase * n + si) * args.per + fi
                img, d = new_frame(W, H)
                chrome(d, W, H, "CHIP ANATOMY",
                       "five machines in one package: USB IF > MAC(+WCPU) > "
                       "BB > RF > PA/LNA > antenna", gi)

                # die brackets above the row: D-die = MAC+BB, A-die = RF
                for (i0, i1, lbl, col) in ((1, 2, "D-die (digital)", CYAN),
                                           (3, 3, "A-die (analog)", AMBER)):
                    bx0, bx1 = xs[i0][0], xs[i1][1]
                    by = row_y - 22
                    dc = tuple(c // 2 for c in col)
                    d.line([bx0, by, bx1, by], fill=dc, width=1)
                    d.line([bx0, by, bx0, by + 8], fill=dc, width=1)
                    d.line([bx1, by, bx1, by + 8], fill=dc, width=1)
                    tw = d.textlength(lbl, font=font(10))
                    d.text(((bx0 + bx1) // 2 - tw // 2, by - 15), lbl,
                           font=font(10), fill=col)
                # one-package outline around everything but the antenna
                d.rectangle([xs[0][0] - 8, row_y - 44, xs[4][1] + 8,
                             row_y + box_h + 10], outline=(35, 45, 60))
                d.text((xs[0][0], row_y + box_h + 16), "one chip package",
                       font=font(10), fill=(70, 90, 110))

                # blocks
                active = blk
                for j, (name, bw, _, _) in enumerate(BLOCKS):
                    x0, x1 = xs[j]
                    on = j == active
                    passed = (order.index(j) < si)
                    oc = CYAN if on else (OK if passed else (55, 70, 88))
                    tc = INK if on else (OK if passed else DIM)
                    if name == "ANT":  # antenna is a symbol, not a box
                        cx = centers[j]
                        ac = oc
                        d.line([cx, row_y + 10, cx, row_y + box_h - 6],
                               fill=ac, width=2)
                        d.line([cx - 12, row_y + 26, cx, row_y + 10],
                               fill=ac, width=2)
                        d.line([cx + 12, row_y + 26, cx, row_y + 10],
                               fill=ac, width=2)
                        if on:
                            for r in (12, 20, 28):
                                rr = r + (gi % 3) * 2
                                d.arc([cx + 4, row_y + 10 - rr,
                                       cx + 4 + 2 * rr, row_y + 10 + rr],
                                      140, 220, fill=AMBER)
                        d.text((cx - 20, row_y + box_h - 20), "ANT",
                               font=font(11, True), fill=tc)
                        continue
                    d.rectangle([x0, row_y, x1, row_y + box_h],
                                outline=oc, width=2 if on else 1)
                    d.text((x0 + 8, row_y + 6), name, font=font(12, True),
                           fill=tc)
                    if name == "MAC":  # small WCPU core inside the MAC
                        wc = AMBER if on else (90, 80, 40)
                        d.rectangle([x0 + 10, row_y + 42, x0 + 74,
                                     row_y + box_h - 10], outline=wc)
                        d.text((x0 + 16, row_y + 48), "WCPU", font=font(10),
                               fill=wc)
                    if name == "BB":
                        d.text((x0 + 8, row_y + 24), "FFT/EQ", font=font(10),
                               fill=(70, 90, 110))
                    if name == "RF":
                        d.text((x0 + 8, row_y + 24), "synth", font=font(10),
                               fill=(70, 90, 110))
                    # arrow to next block
                    d.line([x1, row_y + box_h // 2, x1 + gap,
                            row_y + box_h // 2],
                           fill=OK if passed and not on else (55, 70, 88),
                           width=2)

                # flowing frame token: sits at the active block, glides to
                # the next one during the second half of the stage
                cx0 = centers[blk]
                nxt = order[si + 1] if si + 1 < n else blk
                t = fi / max(1, args.per - 1)
                glide = max(0.0, (t - 0.5) * 2)
                tx = cx0 + (centers[nxt] - cx0) * glide
                ty = row_y + box_h + 36
                pr = 6 + 2 * math.sin(gi * 0.9)
                d.ellipse([tx - pr, ty - pr, tx + pr, ty + pr], fill=AMBER)
                d.text((tx - 22, ty + 12), "frame", font=font(10), fill=AMBER)

                # captions
                cy = row_y + box_h + 78
                d.text((padL, cy), phase_label, font=font(12, True),
                       fill=WARN if phase else OK)
                name, _, txd, rxd = BLOCKS[blk]
                desc = txd if phase == 0 else rxd
                d.text((padL, cy + 30), f"{si+1}. {name} — {desc}",
                       font=font(13, True), fill=CYAN)
                d.text((padL, cy + 56),
                       "digital dies (MAC+BB) and analog die (RF) share one "
                       "package — the D-die / A-die split of the primer",
                       font=font(11), fill=DIM)
                imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms, colors=56)
    return 0


if __name__ == "__main__":
    sys.exit(main())
