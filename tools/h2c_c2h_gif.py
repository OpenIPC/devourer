#!/usr/bin/env python3
"""Animated H2C / C2H mailboxes — 'talking to a running firmware', in the
DEVOURER live-monitor style.

    tools/h2c_c2h_gif.py -o docs/img/h2c_c2h.gif

H2C (host-to-CPU) is the command mailbox: small fixed-format boxes the host
fills and the firmware consumes — power-save mode, rate mask, coex slots.
C2H (CPU-to-host) is the return path: the firmware injects event frames into
the ordinary RX stream, so they ride the same pipe as received 802.11 frames.
The finale is firmware IO-offload: one big H2C write-list crosses the bus and
unpacks inside the chip into a burst of local register writes — one transfer,
a thousand writes. Needs Pillow.
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

W, H = 900, 470
HOST = (40, 96, 258, 400)            # host box
CHIP = (642, 96, 862, 400)           # WCPU / chip box
LANE_X0, LANE_X1 = HOST[2], CHIP[0]  # the bus between them
H2C_Y = 150                          # H2C lane centreline
C2H_Y = 268                          # C2H lane centreline
REG_Y0, REG_Y1 = 330, 358            # register strip inside the chip
CAP_Y = 432

H2C_CMDS = ["PWR MODE", "RATE MASK", "COEX SLOT"]
C2H_EVENTS = {2: "TX REPORT", 6: "CAL DONE", 10: "COEX TLM"}
N_STREAM = 13                        # items circulating in the C2H stream


def cmd_box(d, cx, cy, label, w=104, h=26, hot=True):
    col = AMBER if hot else (90, 110, 135)
    d.rectangle([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2],
                outline=col, fill=(30, 26, 10) if hot else (16, 20, 30),
                width=2)
    tw = d.textlength(label, font=font(11, True))
    d.text((cx - tw / 2, cy - 7), label, font=font(11, True),
           fill=AMBER if hot else INK)


def draw_static(d, tick, chip_flash, phase):
    # host box
    d.rectangle(HOST, outline=(0, 110, 120), width=2)
    d.text((HOST[0] + 14, HOST[1] + 10), "HOST", font=font(14, True),
           fill=CYAN)
    d.text((HOST[0] + 14, HOST[1] + 32), "(devourer / driver)", font=font(10),
           fill=DIM)
    # chip box (flash outline when the fw consumes a command)
    cc = AMBER if chip_flash else (0, 110, 120)
    d.rectangle(CHIP, outline=cc, width=3 if chip_flash else 2)
    d.text((CHIP[0] + 14, CHIP[1] + 10), "WCPU", font=font(14, True),
           fill=CYAN)
    d.text((CHIP[0] + 14, CHIP[1] + 32), "(running firmware)", font=font(10),
           fill=DIM)

    # H2C lane
    d.text((LANE_X0 + 12, H2C_Y - 44), "H2C — command mailbox",
           font=font(12, True), fill=INK)
    d.line([LANE_X0, H2C_Y + 22, LANE_X1, H2C_Y + 22], fill=(0, 70, 80))
    ax = LANE_X1 - 14
    d.line([LANE_X0, H2C_Y - 26, LANE_X1, H2C_Y - 26], fill=(0, 70, 80))
    d.polygon([(ax, H2C_Y + 17), (ax, H2C_Y + 27), (ax + 10, H2C_Y + 22)],
              fill=(0, 120, 130))

    # C2H lane
    d.text((LANE_X0 + 12, C2H_Y - 44), "C2H — events inside the RX stream",
           font=font(12, True), fill=INK)
    d.line([LANE_X0, C2H_Y - 26, LANE_X1, C2H_Y - 26], fill=(0, 70, 80))
    d.line([LANE_X0, C2H_Y + 22, LANE_X1, C2H_Y + 22], fill=(0, 70, 80))
    d.polygon([(LANE_X0 + 4, C2H_Y + 17), (LANE_X0 + 4, C2H_Y + 27),
               (LANE_X0 - 6, C2H_Y + 22)], fill=(0, 120, 130))

    # register strip inside the chip
    d.text((CHIP[0] + 14, REG_Y1 + 8), "registers", font=font(10), fill=DIM)
    d.rectangle([CHIP[0] + 14, REG_Y0, CHIP[2] - 14, REG_Y1],
                outline=(70, 90, 110))


def draw_reg_cells(d, lit):
    """The register strip's cells; `lit` = how many are written already."""
    x0, x1 = CHIP[0] + 16, CHIP[2] - 16
    n = 16
    cw = (x1 - x0) / n
    for k in range(n):
        cx = x0 + k * cw
        if k < lit:
            d.rectangle([cx + 1, REG_Y0 + 3, cx + cw - 2, REG_Y1 - 3],
                        fill=OK)
        else:
            d.rectangle([cx + 1, REG_Y0 + 3, cx + cw - 2, REG_Y1 - 3],
                        outline=(45, 60, 75))


def draw_c2h_stream(d, f, rnd_marks):
    """Small RX frames flowing chip → host; a few are C2H events (amber)."""
    span = LANE_X1 - LANE_X0 + 140
    xs = []
    for k in range(N_STREAM):
        off = k * (span / N_STREAM)
        x = LANE_X1 - ((f * 11 + off) % span) + 40
        xs.append(x)
    events = []                       # (x, half-width) — drawn last, on top
    for k, x in enumerate(xs):
        if k not in C2H_EVENTS:
            continue
        label = C2H_EVENTS[k]
        w = 12 + 8 * len(label)
        if LANE_X0 - w / 2 < x < LANE_X1 + 4:
            events.append((x, w / 2, label))
    for k, x in enumerate(xs):        # plain RX frames, skip near events
        if k in C2H_EVENTS or x < LANE_X0 - 30 or x > LANE_X1 + 4:
            continue
        if any(abs(x - ex) < ew + 22 for ex, ew, _ in events):
            continue
        d.rectangle([x - 13, C2H_Y - 9, x + 13, C2H_Y + 9],
                    outline=(70, 95, 120), fill=(14, 20, 30))
        d.text((x - 8, C2H_Y - 6), "RX", font=font(10),
               fill=(110, 135, 160))
    for ex, ew, label in events:
        xx = max(min(ex, LANE_X1 - ew - 2), LANE_X0 + ew + 2)
        cmd_box(d, xx, C2H_Y, label, w=ew * 2, h=22, hot=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="h2c_c2h.gif")
    ap.add_argument("--ms", type=int, default=110)
    args = ap.parse_args()
    rnd = random.Random(0xC2)
    sparks = [(rnd.random(), rnd.random()) for _ in range(80)]

    PER = 10                       # frames per H2C command crossing
    PH_A = len(H2C_CMDS) * PER     # 30 — commands cross
    PH_B = 28                      # events ride the RX stream
    PH_C_CROSS = 13                # write-list crosses
    PH_C_BURST = 26                # unpack into register writes
    total = PH_A + PH_B + PH_C_CROSS + PH_C_BURST + 5

    imgs = []
    for f in range(total):
        img, d = new_frame(W, H)
        chrome(d, W, H, "H2C / C2H — TALKING TO THE FIRMWARE",
               "H2C: host -> fw commands   C2H: fw events ride the RX stream  "
               " finale: fw IO-offload", f)

        chip_flash = False
        caption = ""
        pending = list(H2C_CMDS)
        big_cross = None
        lit = 0

        if f < PH_A:                                   # --- phase A: H2C
            ci, cf = divmod(f, PER)
            t = cf / (PER - 1)
            pending = H2C_CMDS[ci + 1:]
            chip_flash = t > 0.85
            caption = ("1. H2C: the host fills a small command box — the "
                       "firmware consumes it  (%s)" % H2C_CMDS[ci])
        elif f < PH_A + PH_B:                          # --- phase B: C2H
            caption = ("2. C2H: firmware events (TX reports, cal-done, coex "
                       "telemetry) ride among normal RX frames")
            pending = []
        elif f < PH_A + PH_B + PH_C_CROSS:             # --- phase C: cross
            t = (f - PH_A - PH_B) / (PH_C_CROSS - 1)
            big_cross = t
            pending = []
            caption = ("3. fw IO-offload: pack a whole register write-list "
                       "into one H2C payload")
        else:                                          # --- phase C: burst
            bf = f - PH_A - PH_B - PH_C_CROSS
            lit = min(16, 1 + bf * 16 // (PH_C_BURST - 6))
            pending = []
            chip_flash = bf < 3
            caption = ("4. one transfer, a thousand writes: the firmware "
                       "replays the list locally — fw IO-offload")

        draw_static(d, f, chip_flash, 0)
        draw_reg_cells(d, lit)

        # host-side pending command queue (phase A only)
        if f < PH_A:
            d.text((HOST[0] + 14, H2C_Y - 44 + 118), "queued:",
                   font=font(10), fill=DIM)
            for q, name in enumerate(pending):
                cmd_box(d, HOST[0] + 78, 290 + q * 36, name, hot=False)

        # the crossing H2C command (phase A)
        if f < PH_A:
            ci, cf = divmod(f, PER)
            t = cf / (PER - 1)
            x = LANE_X0 + 60 + t * (LANE_X1 - LANE_X0 - 120)
            cmd_box(d, x, H2C_Y, H2C_CMDS[ci])

        # the C2H stream runs through phases B..end (dim during A)
        if f >= PH_A:
            draw_c2h_stream(d, f, sparks)

        # phase C: the big write-list box
        if big_cross is not None:
            x = LANE_X0 + 90 + big_cross * (LANE_X1 - LANE_X0 - 180)
            cmd_box(d, x, H2C_Y, "WRITE-LIST (N=1500)", w=182, h=36)

        # phase C: spark burst from the WCPU core toward the register strip
        if lit:
            bf = f - PH_A - PH_B - PH_C_CROSS
            cx0, cy0 = (CHIP[0] + CHIP[2]) / 2, CHIP[1] + 96
            d.text((CHIP[0] + 14, CHIP[1] + 58), "unpacking...",
                   font=font(11), fill=AMBER)
            for si in range(min(len(sparks), 6 + bf * 3)):
                u, v = sparks[si]
                ph = (bf * 0.31 + v) % 1.0
                sx = cx0 + (u - 0.5) * 170
                sy = cy0 + ph * (REG_Y0 - 8 - cy0)
                r = 2 if v < 0.7 else 3
                d.ellipse([sx - r, sy - r, sx + r, sy + r], fill=AMBER)

        d.text((40, CAP_Y), caption, font=font(13, True), fill=CYAN)
        imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms, colors=56)
    return 0


if __name__ == "__main__":
    sys.exit(main())
