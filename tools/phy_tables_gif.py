#!/usr/bin/env python3
"""Animated PHY register-table walker — 'one generated table serves every
board', in the DEVOURER live-monitor style.

    tools/phy_tables_gif.py -o docs/img/phy_tables.gif

The vendor ships the whole BB/RF bring-up as long generated tables of
(address, value) pairs, interleaved with check_positive condition rows
("the next block applies only if cut == C && rfe == 2"). The walker holds the
board identity (cut / rfe_type / interface) and evaluates each gate: rows in a
matching block fly into the chip's register map, rows in a failed block bounce
off the closed gate. The animation scrolls a table tape past that gate so you
can watch one table serve two different boards. Needs Pillow.
"""
from __future__ import annotations

import argparse
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, GRID, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

# Register-map regions on the chip strip (label, address hint).
REGIONS = [
    ("BB PHY_REG", "0x800.."),
    ("AGC / GAIN", "0x81C.."),
    ("RADIO A", "RF path A"),
    ("RADIO B", "RF path B"),
    ("MAC", "0x000.."),
]

# The table tape: ("row", addr, value, region_idx) is a plain write,
# ("cond", text, verdict) opens/closes the gate, verdict in {open, closed,
# neutral}.  Values are illustrative but formatted like the real tables.
TAPE = [
    ("row", "0x800", "0x8020D410", 0),
    ("row", "0x804", "0x080112E0", 0),
    ("row", "0x808", "0x0E028233", 0),
    ("row", "0x818", "0x03204390", 0),
    ("row", "0x82C", "0x002083DD", 0),
    ("cond", "IF cut==C && rfe==2", "closed"),
    ("row", "0x840", "0x1F0E0F0F", None),
    ("row", "0x844", "0x00000000", None),
    ("row", "0x848", "0x2C028233", None),
    ("cond", "ELSE", "open"),
    ("row", "0x840", "0x0F0E0F0F", 0),
    ("row", "0x848", "0x0E028233", 0),
    ("cond", "END", "neutral"),
    ("row", "0x81C", "0xFF000003", 1),
    ("row", "0x81C", "0xFE020003", 1),
    ("cond", "IF rfe==1 && usb", "open"),
    ("row", "RF_A 0x18", "0x08400", 2),
    ("row", "RF_A 0x56", "0x51CF3", 2),
    ("row", "RF_A 0xEF", "0x00200", 2),
    ("cond", "END", "neutral"),
    ("row", "RF_B 0x18", "0x08400", 3),
    ("row", "0x520", "0x2E7A2E7A", 4),
]

# caption per tape index range (start, text)
CAPTIONS = [
    (0, "1. plain rows stream through the gate — (addr, value) register writes"),
    (5, "2. IF cut==C && rfe==2 — this board is cut B / rfe 1: gate CLOSED, rows skipped"),
    (9, "3. ELSE — the other branch is the one that matches: gate re-opens"),
    (13, "4. unconditional AGC rows — the receiver's gain staircase"),
    (15, "5. IF rfe==1 && usb — MATCHES this identity: a different block, gate stays OPEN"),
    (20, "6. radio B + MAC rows land — the bring-up quartet completes"),
]
FINAL = "one generated table serves every board — check_positive gating"


def caption_for(idx):
    cur = CAPTIONS[0][1]
    for start, text in CAPTIONS:
        if idx >= start:
            cur = text
    return cur


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="phy_tables.gif")
    ap.add_argument("--per", type=int, default=4, help="frames per tape row")
    ap.add_argument("--ms", type=int, default=110)
    args = ap.parse_args()
    rnd = random.Random(0x812A)  # jitter for bounce debris only

    W, H = 900, 520
    tape_x, tape_w = 40, 268            # scrolling tape column
    gate_x0, gate_x1 = 330, 396         # the check_positive gate
    gate_y = 268                        # tape row being evaluated sits here
    row_dy = 27
    strip_x0, strip_x1 = 566, 866       # chip register-map strip
    strip_y0, strip_y1 = 96, 436
    region_h = (strip_y1 - strip_y0) // len(REGIONS)
    id_x0, id_y0 = 418, 96              # identity panel

    P = args.per
    n_rows = len(TAPE)
    hold = 12
    total = n_rows * P + hold

    # pre-walk the tape to know gate state *while each row is processed*
    gate_state = []                     # per tape index: open?  (rows only)
    applied_after = []                  # cumulative counters after row i
    skipped_after = []
    g, ap_c, sk_c = True, 0, 0
    for kind, *rest in TAPE:
        if kind == "cond":
            g = rest[1] != "closed"
            gate_state.append(g)
        else:
            gate_state.append(g)
            if g:
                ap_c += 1
            else:
                sk_c += 1
        applied_after.append(ap_c)
        skipped_after.append(sk_c)

    imgs = []
    for gi in range(total):
        cur = min(gi // P, n_rows - 1)
        ft = (gi % P) / P if gi < n_rows * P else 1.0
        done = gi >= n_rows * P
        img, d = new_frame(W, H)
        chrome(d, W, H, "PHY TABLE WALKER",
               "generated (addr,value) tape  >>  check_positive gate  >>  "
               "BB/RF register map", gi)

        # ---- chip register-map strip -------------------------------------
        d.text((strip_x0, strip_y0 - 16), "chip register map", font=font(11),
               fill=DIM)
        flash_region = None
        row = TAPE[cur]
        if not done and row[0] == "row" and gate_state[cur] and ft > 0.72:
            flash_region = row[3]
        for ri, (name, hint) in enumerate(REGIONS):
            ry = strip_y0 + ri * region_h
            hot = ri == flash_region
            # regions already touched glow faintly
            touched = any(t[0] == "row" and t[3] == ri and gate_state[k]
                          for k, t in enumerate(TAPE[:cur]))
            fillc = (0, 66, 72) if hot else ((14, 26, 34) if touched else None)
            if fillc:
                d.rectangle([strip_x0 + 1, ry + 3, strip_x1 - 1,
                             ry + region_h - 3], fill=fillc)
            d.rectangle([strip_x0, ry + 2, strip_x1, ry + region_h - 2],
                        outline=CYAN if hot else (50, 66, 84),
                        width=2 if hot else 1)
            d.text((strip_x0 + 10, ry + 8), name, font=font(12, True),
                   fill=INK if (hot or touched) else DIM)
            d.text((strip_x0 + 10, ry + 26), hint, font=font(10), fill=DIM)

        # ---- identity panel ----------------------------------------------
        d.rectangle([id_x0, id_y0, id_x0 + 128, id_y0 + 96],
                    outline=(0, 90, 100))
        d.text((id_x0 + 8, id_y0 + 6), "IDENTITY", font=font(11, True),
               fill=CYAN)
        for k, line in enumerate(("cut  = B", "rfe  = 1", "intf = usb")):
            d.text((id_x0 + 8, id_y0 + 28 + 20 * k), line, font=font(12),
                   fill=INK)

        # ---- gate ----------------------------------------------------------
        open_now = gate_state[cur]
        gc = OK if open_now else WARN
        d.text((gate_x0 - 4, gate_y - 66), "check_positive", font=font(10),
               fill=DIM)
        d.line([gate_x0, gate_y - 46, gate_x0, gate_y + 46], fill=gc, width=3)
        d.line([gate_x1, gate_y - 46, gate_x1, gate_y + 46], fill=gc, width=3)
        if open_now:
            d.text((gate_x0 + 14, gate_y + 52), "OPEN", font=font(11, True),
                   fill=OK)
        else:  # a bar across the gap
            d.rectangle([gate_x0, gate_y - 9, gate_x1, gate_y + 9], fill=WARN)
            d.text((gate_x0 + 6, gate_y - 7), "CLOSED", font=font(11, True),
                   fill=(20, 8, 8))

        # ---- counters ------------------------------------------------------
        idx = cur if done else max(cur - 1, 0)
        d.text((gate_x0 - 4, gate_y + 78),
               f"applied {applied_after[cur] if done else applied_after[idx] if cur else 0:2d}",
               font=font(12, True), fill=OK)
        d.text((gate_x0 - 4, gate_y + 98),
               f"skipped {skipped_after[cur] if done else skipped_after[idx] if cur else 0:2d}",
               font=font(12, True), fill=WARN)

        # ---- tape (upcoming rows scroll up toward the gate) -----------------
        d.text((tape_x, strip_y0 - 16), "generated table (tape)", font=font(11),
               fill=DIM)
        d.rectangle([tape_x - 6, strip_y0, tape_x + tape_w, strip_y1],
                    outline=(0, 70, 80))
        for k in range(cur + 1, min(cur + 7, n_rows)):
            ry = gate_y + (k - cur) * row_dy - int(ft * row_dy) - 8
            if ry > strip_y1 - 20:
                continue
            t = TAPE[k]
            if t[0] == "cond":
                d.text((tape_x + 4, ry), t[1], font=font(12, True), fill=AMBER)
            else:
                d.text((tape_x + 4, ry), f"{t[1]:<10s} {t[2]}", font=font(12),
                       fill=(90, 110, 135))

        # ---- the row being evaluated ----------------------------------------
        if not done:
            if row[0] == "cond":
                # condition row parks at the gate in amber, gate reacts
                d.text((tape_x + 4, gate_y - 8), row[1], font=font(13, True),
                       fill=AMBER)
                d.line([tape_x + tape_w, gate_y, gate_x0 - 4, gate_y],
                       fill=AMBER, width=1)
            else:
                label = f"{row[1]:<10s} {row[2]}"
                if gate_state[cur]:
                    # fly right through the gate into its region
                    tx0, ty0 = tape_x + 4, gate_y - 8
                    reg_cy = strip_y0 + row[3] * region_h + region_h // 2 - 8
                    fx = tx0 + (strip_x0 + 12 - tx0) * ft
                    fy = ty0 + (reg_cy - ty0) * ft
                    d.text((fx, fy), label, font=font(12, True),
                           fill=CYAN if ft < 0.72 else OK)
                else:
                    # approach the closed gate, bounce back and fade
                    if ft < 0.45:
                        fx = tape_x + 4 + (gate_x0 - 60 - tape_x) * (ft / 0.45)
                        col = INK
                    else:
                        bt = (ft - 0.45) / 0.55
                        fx = (gate_x0 - 60) - 90 * bt
                        f = max(0.15, 1 - bt)
                        col = (int(240 * f), int(90 * f), int(70 * f))
                        for _ in range(3):  # debris sparks off the bar
                            sx = gate_x0 - 4 - rnd.randint(0, 26)
                            sy = gate_y + rnd.randint(-22, 22)
                            d.ellipse([sx - 1, sy - 1, sx + 1, sy + 1],
                                      fill=col)
                    d.text((fx, gate_y - 8 - (12 * ft if ft > 0.45 else 0)),
                           label, font=font(12, True), fill=col)

        # ---- caption --------------------------------------------------------
        cap = FINAL if done else caption_for(cur)
        d.text((tape_x - 6, H - 42), cap, font=font(13, True),
               fill=CYAN if not done else OK)
        imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms, colors=56)
    return 0


if __name__ == "__main__":
    sys.exit(main())
