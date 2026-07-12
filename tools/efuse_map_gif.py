#!/usr/bin/env python3
"""Animated efuse decode — physical chain to logical map, in the DEVOURER
live-monitor style (docs/driver-primer.md §4).

    tools/efuse_map_gif.py -o docs/img/efuse_map.gif

The efuse is a one-time-programmable memory burned at the factory: not a flat
structure but a chain of headers, each saying 'the next few bytes belong at
logical offset N, and only these words are present'. The animation walks the
chain entry by entry — header highlighted and decoded on the left, its data
bytes flying into the fixed-layout logical map on the right (MAC address,
crystal trim, TX power base, RFE type, thermal baseline). Fusing is
0→1-bits-only, so the final entry is an appended *patch* that overrides an
earlier slot — how updates work on write-once silicon. Needs Pillow.
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

# region tints for the logical-map cells
T_MAC = (0, 62, 72)
T_XTAL = (72, 60, 18)
T_PWR = (16, 60, 40)
T_MISC = (58, 38, 66)

# physical chain: (hdr hex, decode text, logical offset, data bytes, patch?)
ENTRIES = [
    ("07", "hdr: offset 0x00, words 1111",
     0x00, ["00", "E0", "4C", "88", "12", "AA", "00", "03"], False),
    ("18", "hdr: offset 0x08, words 0001", 0x08, ["20", "00"], False),
    ("29", "hdr: offset 0x10, words 1111",
     0x10, ["2D", "2D", "2C", "2A", "28", "26", "24", "22"], False),
    ("3C", "hdr: offset 0x18, words 0011",
     0x18, ["02", "00", "1A", "00"], False),
    ("18", "hdr: offset 0x08, words 0001  (PATCH)",
     0x08, ["1E", "00"], True),
]
# logical rows shown in the grid (sparse map — most of it is empty)
ROWS = (0x00, 0x08, 0x10, 0x18)
REGION = {}  # logical offset -> tint
for o in range(0x00, 0x06):
    REGION[o] = T_MAC
REGION[0x08] = T_XTAL
for o in range(0x10, 0x18):
    REGION[o] = T_PWR
REGION[0x18] = T_MISC
REGION[0x1A] = T_MISC

LEGEND = [  # (text, tint, appears after entry idx)
    ("MAC ADDR      0x00-0x05", T_MAC, 0),
    ("XTAL TRIM     0x08", T_XTAL, 1),
    ("TX PWR BASE   0x10-0x17", T_PWR, 2),
    ("RFE TYPE 0x18 THERMAL 0x1A", T_MISC, 3),
]

CAPTIONS = [
    "each header says where the next bytes belong",
    "the logical map materializes slot by slot",
    "per-rate TX power base — this unit's calibration",
    "RFE type + thermal baseline — board options",
    "0->1-only fusing: updates are appended patches",
]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="efuse_map.gif")
    ap.add_argument("--per", type=int, default=17)
    ap.add_argument("--tail", type=int, default=8)
    ap.add_argument("--ms", type=int, default=110)
    args = ap.parse_args()
    rnd = random.Random(0xEF)

    W, H = 900, 452
    # left: physical chain
    lx0 = 40
    ent_y0, ent_dy = 118, 56
    # right: logical grid
    gx0, gy0 = 508, 118
    cw, ch = 44, 32

    def cell_xy(off):
        row = ROWS.index(off & ~0x07)
        col = off & 0x07
        return gx0 + col * cw, gy0 + row * ch

    filled = {}  # logical offset -> hex value (mutates as we replay)
    imgs = []
    n_ent = len(ENTRIES)
    total_states = []  # (entry idx, frame-in-entry) plus tail
    for ei in range(n_ent):
        for fi in range(args.per):
            total_states.append((ei, fi))
    for fi in range(args.tail):
        total_states.append((n_ent, fi))

    for gi, (ei, fi) in enumerate(total_states):
        done = ei >= n_ent
        # recompute fill state deterministically for this frame
        filled = {}
        patched = set()
        for k in range(n_ent):
            hdr, dec, off, data, patch = ENTRIES[k]
            if k < ei:
                nby = len(data)
            elif k == ei and not done:
                # bytes land during frames 5..12
                nby = max(0, min(len(data), int((fi - 5) / 8 * len(data) + 1)
                                 if fi >= 5 else 0))
            else:
                nby = 0
            for b in range(nby):
                o = off + b
                if o in REGION:
                    if o in filled and patch:
                        patched.add(o)
                    filled[o] = data[b]

        img, d = new_frame(W, H)
        chrome(d, W, H, "EFUSE",
               "the chip's birth certificate: a burned chain of headers -> "
               "a fixed logical map", gi)

        # ---- left: physical chain
        d.text((lx0, 92), "PHYSICAL EFUSE (burned bytes)", font=font(11, True),
               fill=INK)
        for k, (hdr, dec, off, data, patch) in enumerate(ENTRIES):
            y = ent_y0 + k * ent_dy
            active = k == ei and not done
            past = k < ei or done
            oc = CYAN if active else ((55, 70, 88) if not past else (0, 90, 100))
            d.rectangle([lx0, y, lx0 + 420, y + 24], outline=oc,
                        width=2 if active else 1)
            hc = AMBER if (active or past) else DIM
            d.text((lx0 + 8, y + 5), hdr, font=font(12, True), fill=hc)
            d.text((lx0 + 34, y + 5), "|", font=font(12), fill=(55, 70, 88))
            for b, hx in enumerate(data):
                bc = INK if (active or past) else DIM
                d.text((lx0 + 50 + b * 30, y + 5), hx, font=font(12),
                       fill=bc)
            # decode line under the active/past entry
            if active and fi >= 2:
                d.text((lx0 + 8, y + 28), dec, font=font(11, True),
                       fill=WARN if patch else AMBER)
            elif past:
                d.text((lx0 + 8, y + 28), dec, font=font(10),
                       fill=(90, 75, 40) if not patch else (110, 50, 40))

        # ---- right: logical map grid
        d.text((gx0, 92), "LOGICAL MAP", font=font(11, True), fill=INK)
        for r, base in enumerate(ROWS):
            d.text((gx0 - 44, gy0 + r * ch + 9), f"0x{base:02X}",
                   font=font(10), fill=DIM)
            for c in range(8):
                o = base + c
                x, y = cell_xy(o)
                tint = REGION.get(o)
                if o in filled:
                    flash = (o in patched and not done and ei == n_ent - 1
                             and gi % 2 == 0)
                    oc = WARN if (o in patched) else (100, 120, 140)
                    d.rectangle([x, y, x + cw - 4, y + ch - 4],
                                fill=tint, outline=oc,
                                width=2 if flash else 1)
                    d.text((x + 12, y + 8), filled[o], font=font(11, True),
                           fill=INK)
                else:
                    d.rectangle([x, y, x + cw - 4, y + ch - 4],
                                outline=(35, 45, 60))
                    d.text((x + 12, y + 8), "--", font=font(11),
                           fill=(50, 62, 78))
        # flying bytes: from the active entry line to their target cells
        if not done and 5 <= fi <= 12:
            hdr, dec, off, data, patch = ENTRIES[ei]
            t = (fi - 5) / 7
            b = min(len(data) - 1, int(t * len(data)))
            sx = lx0 + 50 + b * 30
            sy = ent_y0 + ei * ent_dy + 12
            txc, tyc = cell_xy(off + b) if (off + b) in REGION \
                else cell_xy(off)
            frac = (fi - 5) / 7 % 1
            mx = sx + (txc + 12 - sx) * frac
            my = sy + (tyc + 10 - sy) * frac
            d.text((mx, my), data[b], font=font(11, True), fill=AMBER)

        # legend under the grid
        ly = gy0 + len(ROWS) * ch + 14
        shown = 0
        for text, tint, after in LEGEND:
            if ei > after or done:
                d.rectangle([gx0, ly + shown * 20 + 2,
                             gx0 + 12, ly + shown * 20 + 14], fill=tint)
                d.text((gx0 + 20, ly + shown * 20), text, font=font(10),
                       fill=DIM)
                shown += 1
        if done or (ei == n_ent - 1 and fi > 12):
            d.text((gx0, ly + shown * 20 + 2),
                   "XTAL 0x20 -> 0x1E: later entry wins", font=font(10, True),
                   fill=WARN)

        # ---- caption
        cap = CAPTIONS[min(ei, len(CAPTIONS) - 1)]
        d.text((lx0, 414), cap, font=font(13, True), fill=CYAN)
        imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms, colors=64)
    return 0


if __name__ == "__main__":
    sys.exit(main())
