#!/usr/bin/env python3
"""Animated 'two doors to the radio' — LSSI vs the Wi-Fi 6 RF windows, in the
DEVOURER live-monitor style.

    tools/lssi_rf_windows_gif.py -o docs/img/lssi_rf_windows.gif

The RF dies keep a private register space (8-bit addresses, 20-bit data, one
copy per path) that is not memory-mapped. Left panel: the 11ac path — the host
writes one BB register (0xc90 path A / 0xe90 path B) and LSSI clocks the packed
addr+data across a 3-wire serial bus, bit by bit. Right panel: the Wi-Fi 6
path — the d-die registers appear through the DDV window apertures at
0xe000/0xf000 (plain writes), while a-die registers go through the DAV serial
command register 0x370; an address BIT(16) picks the door. Needs Pillow.
"""
from __future__ import annotations

import argparse
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, GRID, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

NB = 28  # 8-bit addr + 20-bit data

# stages: (frames, key, caption)
STAGES = [
    (12, "pack", "LSSI 1/2 — host packs 8-bit addr + 20-bit data into one BB write (0xc90)"),
    (30, "shift", "LSSI 2/2 — the BB clocks all 28 bits serially into the RF die: slow, write-mostly"),
    (6,  "land", "LSSI 2/2 — ...and the RF register finally latches"),
    (18, "ddv", "Wi-Fi 6 DDV — the d-die window at 0xe000/0xf000: RF regs become plain writes"),
    (18, "dav", "Wi-Fi 6 DAV — BIT(16) routes the write through serial cmd 0x370 into the a-die"),
    (12, "final", "one radio, two dies, two doors — DDV is a window, DAV is LSSI's descendant"),
]

D_REGS = ["0x00", "0x18", "0x1F", "0x56", "0x8F", "0xEF"]
A_REGS = ["0x00", "0x18", "0x55", "0xDF"]


def dimmed(c, on):
    return c if on else tuple(v // 3 for v in c)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="lssi_rf_windows.gif")
    ap.add_argument("--ms", type=int, default=110)
    args = ap.parse_args()
    rnd = random.Random(0xC90)
    bits = [rnd.randint(0, 1) for _ in range(NB)]
    word_hex = "0x18/0x08C40"  # addr 0x18, data 0x08C40 (illustrative)

    W, H = 900, 520
    mid = 452
    imgs = []

    total = sum(s[0] for s in STAGES)
    for gi in range(total):
        # locate stage
        acc, si, ft = 0, 0, 0.0
        for k, (nf, key, cap) in enumerate(STAGES):
            if gi < acc + nf:
                si, ft = k, (gi - acc) / nf
                break
            acc += nf
        key, cap = STAGES[si][1], STAGES[si][2]
        left_on = key in ("pack", "shift", "land", "final")
        right_on = key in ("ddv", "dav", "final")

        img, d = new_frame(W, H)
        chrome(d, W, H, "TWO DOORS TO THE RADIO",
               "RF registers: 8-bit addr, 20-bit data, per path — never "
               "memory-mapped", gi)
        d.line([mid, 78, mid, H - 54], fill=(0, 60, 70), width=1)

        # ================= LEFT: LSSI =====================================
        lc = lambda c: dimmed(c, left_on)
        d.text((40, 84), "LSSI - 11ac parts", font=font(13, True), fill=lc(CYAN))
        d.text((40, 104), "(8812A .. 8822C)", font=font(10), fill=lc(DIM))

        # host box
        d.rectangle([40, 126, 428, 158], outline=lc((0, 90, 100)))
        d.text((50, 134), "host: one BB write  0xc90 (path A) / 0xe90 (B)",
               font=font(11), fill=lc(INK))

        # shift register cells
        cw, cy = 13, 200
        cx0 = 40
        # how many bits have been packed / shifted out
        packed = NB if si > 0 else int(ft * NB) + 1
        shifted = 0
        if key == "shift":
            shifted = int(ft * NB)
        elif si > 1:
            shifted = NB
        d.text((cx0, cy - 22), "8-bit addr", font=font(10), fill=lc(AMBER))
        d.text((cx0 + 8 * cw + 12, cy - 22), "20-bit data", font=font(10),
               fill=lc(CYAN))
        for k in range(NB):
            x = cx0 + k * cw + (12 if k >= 8 else 0)
            oc = lc(AMBER if k < 8 else CYAN)
            d.rectangle([x, cy, x + cw - 2, cy + 18], outline=oc)
            # cell content: filled during pack, drains from the right as shifted
            has_bit = k < packed and k < NB - shifted
            if has_bit:
                d.text((x + 3, cy + 3), str(bits[k]), font=font(11),
                       fill=lc(INK))
        if key == "pack":
            d.text((cx0, cy + 30), f"packing {word_hex}  ({packed}/28 bits)",
                   font=font(11), fill=lc(AMBER))
        elif key == "shift":
            d.text((cx0, cy + 30), f"bit {shifted}/28", font=font(11),
                   fill=lc(AMBER))

        # 3-wire bus: CLK / DATA / EN going down to the RF die
        wire_x = cx0 + NB * cw + 12 + 6   # exit at the right end of the cells
        die_y0 = 380
        for wi, wname in enumerate(("CLK", "DATA", "EN")):
            wx = 150 + wi * 90
            d.text((wx - 14, 252), wname, font=font(10), fill=lc(DIM))
            d.line([wx + 14, 250 + 9, wx + 14, die_y0], fill=lc((50, 70, 90)),
                   width=1)
        # clock square wave beside CLK wire (ticks while shifting)
        if key == "shift":
            px, py = 120, 292
            pts = []
            for k in range(20):
                lvl = ((k + shifted) // 2) % 2
                pts.append((px + k * 4, py + (0 if lvl else 12)))
                pts.append((px + (k + 1) * 4, py + (0 if lvl else 12)))
            d.line(pts, fill=lc(OK), width=1)
            # data pulse travelling down the DATA wire
            pulse_y = 262 + int((die_y0 - 276) * (ft * NB - shifted))
            cur_bit = bits[NB - 1 - shifted] if shifted < NB else 0
            d.ellipse([240 + 14 - 4, pulse_y - 4, 240 + 14 + 4, pulse_y + 4],
                      fill=lc(CYAN if cur_bit else (70, 95, 120)))
            d.text((262, pulse_y - 7), str(cur_bit), font=font(11, True),
                   fill=lc(CYAN))

        # RF die box
        d.rectangle([110, die_y0, 400, die_y0 + 72], outline=lc(CYAN),
                    width=2 if key == "land" else 1)
        d.text((122, die_y0 + 8), "RF die - path A", font=font(12, True),
               fill=lc(INK))
        got = key == "land" or si > 2
        d.text((122, die_y0 + 34),
               "RF[0x18] = 0x08C40" if got else "RF[0x18] = -----",
               font=font(12), fill=lc(OK if got else DIM))
        if key == "land" and ft < 0.6:
            d.rectangle([112, die_y0 + 30, 398, die_y0 + 52],
                        outline=lc(OK))

        # ================= RIGHT: RF WINDOWS ==============================
        rc = lambda c: dimmed(c, right_on)
        d.text((468, 84), "RF WINDOWS - Wi-Fi 6", font=font(13, True),
               fill=rc(CYAN))
        d.text((468, 104), "(8852B/8852C: two dies)", font=font(10), fill=rc(DIM))

        # address token with BIT(16)
        tok_y = 128
        d.text((468, tok_y + 4), "write", font=font(10), fill=rc(DIM))
        bit16 = key == "dav"
        d.rectangle([510, tok_y, 592, tok_y + 22], outline=rc((0, 90, 100)))
        d.text((518, tok_y + 4), "reg 0x18", font=font(11), fill=rc(INK))
        # the BIT(16) routing field gets its own highlighted box
        d.rectangle([600, tok_y, 700, tok_y + 22],
                    outline=rc(AMBER if right_on else DIM))
        d.text((608, tok_y + 4), f"BIT(16)={1 if bit16 else 0}",
               font=font(11), fill=rc(INK))
        d.text((708, tok_y + 4), "-> DAV" if bit16 else "-> DDV",
               font=font(11, True), fill=rc(AMBER if bit16 else OK))

        # DAV serial-cmd box (route into the a-die)
        dav_y = 168
        d.rectangle([470, dav_y, 640, dav_y + 24],
                    outline=rc(AMBER if key == "dav" else (0, 90, 100)))
        d.text((478, dav_y + 5), "serial cmd BB 0x370", font=font(11),
               fill=rc(INK if key == "dav" else DIM))
        d.text((690, dav_y + 5), "direct (window)", font=font(10),
               fill=rc(OK if key == "ddv" else DIM))

        # the two dies
        die_top, die_h = 212, 196
        row_h = die_h // (len(D_REGS) + 1)
        # A-die
        ax0, ax1 = 470, 640
        d.rectangle([ax0, die_top, ax1, die_top + die_h],
                    outline=rc((50, 70, 90)))
        d.text((ax0 + 8, die_top + 4), "A-die (analog)", font=font(11, True),
               fill=rc(INK))
        for k, r in enumerate(A_REGS):
            ry = die_top + 26 + k * row_h
            hot = key == "dav" and r == "0x18" and ft > 0.55
            d.rectangle([ax0 + 8, ry, ax1 - 8, ry + row_h - 6],
                        outline=rc(AMBER if hot else GRID),
                        fill=(60, 50, 12) if hot and right_on else None)
            d.text((ax0 + 16, ry + 4), f"RF {r}", font=font(10),
                   fill=rc(INK if hot else DIM))
        # D-die
        dx0, dx1 = 692, 862
        d.rectangle([dx0, die_top, dx1, die_top + die_h],
                    outline=rc((50, 70, 90)))
        d.text((dx0 + 8, die_top + 4), "D-die (digital)", font=font(11, True),
               fill=rc(INK))
        hot_i = int(ft * len(D_REGS)) if key == "ddv" else 1
        for k, r in enumerate(D_REGS):
            ry = die_top + 26 + k * row_h
            hot = key == "ddv" and k == hot_i
            d.rectangle([dx0 + 8, ry, dx1 - 8, ry + row_h - 6],
                        outline=rc(OK if hot else GRID),
                        fill=(10, 50, 30) if hot and right_on else None)
            d.text((dx0 + 16, ry + 4), f"RF {r}", font=font(10),
                   fill=rc(INK if hot else DIM))
        # DDV sliding window frame over the d-die column
        if right_on:
            wy = die_top + 26 + (hot_i if key == "ddv" else 1) * row_h - 3
            d.rectangle([dx0 + 3, wy, dx1 - 3, wy + row_h],
                        outline=rc(OK), width=2)
            d.text((dx0 + 8, die_top + die_h + 6),
                   "DDV window 0xe000/0xf000", font=font(10),
                   fill=rc(OK))
        # DAV pulse: token -> 0x370 box -> a-die (mini serial echo)
        if key == "dav":
            # ticks inside the 0x370 box while transferring
            for k in range(6):
                tx = 478 + k * 26
                lvl = (k + gi) % 2
                d.line([tx, dav_y + 28 + (0 if lvl else 5),
                        tx + 26, dav_y + 28 + (0 if lvl else 5)],
                       fill=rc(AMBER), width=1)
            if ft < 0.55:
                py = tok_y + 24 + int((dav_y - tok_y - 24) * (ft / 0.55))
                d.ellipse([550 - 4, py - 4, 550 + 4, py + 4], fill=rc(AMBER))
            d.text((470, die_top + die_h + 6),
                   "DAV: a-die regs, via serial cmd", font=font(10),
                   fill=rc(AMBER))

        # ---- caption ------------------------------------------------------
        d.text((40, H - 42), cap, font=font(13, True),
               fill=CYAN if key != "final" else OK)
        imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms, colors=56)
    return 0


if __name__ == "__main__":
    sys.exit(main())
