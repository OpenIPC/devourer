#!/usr/bin/env python3
"""Animated register access — 'two planes, one cable', in the DEVOURER
live-monitor style (docs/driver-primer.md §2).

    tools/register_access_gif.py -o docs/img/register_access.gif

Strip away every abstraction and a driver does two things: it reads/writes
registers via USB vendor CONTROL transfers (a URB with the address in its
setup fields), and it moves frames over BULK endpoints. The animation shows a
host on the left, the chip's register map on the right (MAC low space, BB
above 0x800, and the second 64 KiB page the Wi-Fi 6 BB spills into — RF is
not in this map at all), with small URBs crossing the control lane while
larger frames stream continuously on the bulk lane below. Needs Pillow.
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

# register-map regions: (label, addr text, y0, y1)
REG_MAC = ("MAC", "0x0000-0x07FF", 100, 158)
REG_BB = ("BB", "0x0800-0xFFFF", 158, 262)
REG_BB2 = ("BB window", "+0x10000 (wIndex=1)", 288, 356)

# access sequence: (op, region, addr, note, value)
ACCESSES = [
    ("WR", 0, "0x0608", "RX filter: open for monitor mode", None),
    ("RD", 0, "0x00FC", "chip id", "0x13"),
    ("WR", 1, "0x08AC", "BB: ADC clock divider", None),
    ("RD", 1, "0x0C50", "BB: AGC gain index", "0x40"),
    ("WR", 2, "0x1_0100", "Wi-Fi 6 BB: second 64 KiB page", None),
    ("RD", 2, "0x1_1704", "Wi-Fi 6 BB readback", "0xEA5A"),
]

CAPTIONS = [
    "registers ride control transfers — a URB per read/write",
    "frames ride bulk endpoints — the data plane never stops",
    "two planes, one cable",
]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="register_access.gif")
    ap.add_argument("--per", type=int, default=15)
    ap.add_argument("--ms", type=int, default=110)
    args = ap.parse_args()
    rnd = random.Random(0x52)

    W, H = 900, 452
    host_x0, host_x1 = 40, 196
    host_y0, host_y1 = 100, 356
    map_x0, map_x1 = 656, 862
    lane_x0, lane_x1 = host_x1 + 6, map_x0 - 6
    ctrl_y = 150
    bulk_y = 322

    imgs = []
    total = len(ACCESSES) * args.per
    for ai, (op, reg, addr, note, val) in enumerate(ACCESSES):
        for fi in range(args.per):
            gi = ai * args.per + fi
            img, d = new_frame(W, H)
            chrome(d, W, H, "REGISTER ACCESS",
                   "the only lever the host has: control transfers for "
                   "registers, bulk transfers for frames", gi)

            # ---- host box
            d.rectangle([host_x0, host_y0, host_x1, host_y1],
                        outline=(0, 110, 120), width=2)
            d.text((host_x0 + 14, host_y0 + 10), "HOST", font=font(14, True),
                   fill=INK)
            for k, line in enumerate(("driver", "(RtlAdapter)", "",
                                      "libusb /", "kernel USB", "stack")):
                d.text((host_x0 + 14, host_y0 + 44 + k * 18), line,
                       font=font(11), fill=DIM)
            d.text((host_x0 + 14, host_y1 - 26), "one USB cable",
                   font=font(10), fill=(70, 90, 110))

            # ---- register map strip
            d.text((map_x0, 82), "REGISTER MAP", font=font(11, True),
                   fill=INK)
            regions = (REG_MAC, REG_BB, REG_BB2)
            for ri, (lbl, at, y0, y1) in enumerate(regions):
                hit = ri == reg and 5 <= fi <= 9
                oc = AMBER if hit else (0, 110, 120)
                d.rectangle([map_x0, y0, map_x1, y1], outline=oc,
                            width=2 if hit else 1)
                d.text((map_x0 + 10, y0 + 6), lbl, font=font(12, True),
                       fill=INK if hit else DIM)
                d.text((map_x0 + 10, y0 + 24), at, font=font(10),
                       fill=AMBER if hit else (70, 90, 110))
            # the second page hangs off the first map
            d.text((map_x0, 268), "page 2 —", font=font(10),
                   fill=(70, 90, 110))
            d.line([map_x1 - 20, 262, map_x1 - 20, 288],
                   fill=(55, 70, 88), width=1)
            d.text((map_x0, 366), "RF dies: NOT in this map", font=font(11),
                   fill=WARN)
            d.text((map_x0, 384), "(reached indirectly, §9)", font=font(10),
                   fill=DIM)

            # ---- CONTROL lane
            d.text((lane_x0 + 8, ctrl_y - 52), "CONTROL  (vendor requests)",
                   font=font(11, True), fill=CYAN)
            d.line([lane_x0, ctrl_y, lane_x1, ctrl_y],
                   fill=(30, 45, 58), width=1)
            # URB out (frames 0..5), region flash (5..9), read return (9..14)
            reg_yc = (regions[reg][2] + regions[reg][3]) // 2
            if fi <= 5:
                t = fi / 5
                px = lane_x0 + (lane_x1 - lane_x0 - 30) * t
                d.rectangle([px, ctrl_y - 8, px + 30, ctrl_y + 8],
                            fill=(0, 70, 80), outline=CYAN)
                d.text((px + 4, ctrl_y - 6), "URB", font=font(10), fill=INK)
                d.text((px - 10, ctrl_y - 28), f"{op} {addr}",
                       font=font(11, True), fill=AMBER)
            elif fi <= 9:
                # connector from lane into the flashing region
                d.line([lane_x1, ctrl_y, map_x0, reg_yc], fill=AMBER,
                       width=2)
                d.text((lane_x1 - 150, ctrl_y - 28), f"{op} {addr}",
                       font=font(11, True), fill=AMBER)
            elif op == "RD":
                t = (fi - 9) / 5
                px = lane_x1 - 30 - (lane_x1 - lane_x0 - 30) * t
                d.rectangle([px, ctrl_y - 8, px + 30, ctrl_y + 8],
                            fill=(10, 55, 35), outline=OK)
                d.text((px + 8, ctrl_y - 6), "<-", font=font(10), fill=INK)
                d.text((px - 10, ctrl_y - 28), f"data {val}",
                       font=font(11, True), fill=OK)
            else:
                d.text((lane_x0 + 8, ctrl_y - 28), "write done — no reply",
                       font=font(11), fill=DIM)
            # current access annotation under the control lane
            d.text((lane_x0 + 8, ctrl_y + 16),
                   f"{'write' if op == 'WR' else 'read '} {addr} — {note}",
                   font=font(10), fill=DIM)

            # ---- BULK lane: frames stream continuously
            d.text((lane_x0 + 8, bulk_y - 34), "BULK  (frame endpoints)",
                   font=font(11, True), fill=OK)
            d.line([lane_x0, bulk_y, lane_x1, bulk_y],
                   fill=(30, 45, 58), width=1)
            span = lane_x1 - lane_x0 - 70
            for k in range(3):
                px = lane_x0 + ((gi * 21 + k * (span // 3)) % span)
                d.rectangle([px, bulk_y - 11, px + 64, bulk_y + 11],
                            fill=(8, 40, 26), outline=OK)
                d.text((px + 8, bulk_y - 7), "FRAME", font=font(10),
                       fill=INK)
            d.text((lane_x0 + 8, bulk_y + 16),
                   "TX/RX 802.11 frames — many per URB (RX aggregation)",
                   font=font(10), fill=DIM)

            # ---- caption
            d.text((40, 410), CAPTIONS[ai % 3], font=font(13, True),
                   fill=CYAN)
            imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms, colors=56)
    return 0


if __name__ == "__main__":
    sys.exit(main())
