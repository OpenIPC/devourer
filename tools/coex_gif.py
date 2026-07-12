#!/usr/bin/env python3
"""Animated coexistence arbitration — 'two radios, one antenna', in the
DEVOURER live-monitor style.

    tools/coex_gif.py -o docs/img/coex.gif

One antenna feeds a Wi-Fi/BT combo chip; a hardware TDMA arbiter time-slices
it between the two radios, slot by slot — the granted radio's path lights up,
the loser is blocked. The policy lives in firmware, steered over H2C and
reporting back over C2H. Three phases: BT idle (Wi-Fi owns nearly every slot),
a BT voice link appears (the firmware swaps the policy table and BT wins its
periodic slots), then an external LTE modem asserts the coex handshake and
2.4 GHz slots get blanked outright. Keep the firmware's coex state fed, or it
will keep the antenna. Needs Pillow.
"""
from __future__ import annotations

import argparse
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, GRID, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

NSLOT = 26
BLUE = (90, 150, 245)           # BT
LTEC = (170, 120, 235)          # LTE

# (key, frames, caption)
PHASES = [
    ("a", 26, "(a) BT idle — Wi-Fi owns almost every antenna slot"),
    ("b", 32, "(b) BT voice link — firmware swaps the policy table, "
              "BT wins its slots"),
    ("c", 32, "(c) LTE coex — the modem asserts the handshake, "
              "2.4 GHz slots get blanked"),
    ("d", 12, "shared spectrum, negotiated time — keep the firmware's coex "
              "state fed or it will keep the antenna"),
]


def slot_owner(phase, s):
    """'W' wifi, 'B' bt, 'X' blanked-for-LTE."""
    if phase == "a":
        return "B" if s in (7, 19) else "W"      # BT page scan only
    if phase == "b":
        return "W" if s % 3 == 0 else "B"        # voice: BT wins 2 of 3
    # phase c/d: voice pattern + LTE-protected slots
    if s % 9 in (4, 5):
        return "X"
    return "W" if s % 3 == 0 else "B"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="coex.gif")
    ap.add_argument("--ms", type=int, default=110)
    args = ap.parse_args()

    W, H = 900, 512
    padL = 34
    # left half: antenna + combo chip + LTE box
    antx, anty = 260, 118                      # antenna tip
    juncy = 178                                 # feed junction
    chip = [70, 208, 500, 290]                  # combo chip outline
    wifi = [92, 226, 218, 272]
    bt = [352, 226, 478, 272]
    lte = [44, 108, 164, 152]                   # external modem (dashed)
    # right half: host + fw policy panel
    host = [560, 104, 866, 140]
    pol = [560, 168, 866, 288]
    # arbiter bar
    bX, bY, bW, bH = 60, 330, 780, 40

    imgs = []
    gi = 0
    for (phase, nfr, caption) in PHASES:
        for fi in range(nfr):
            img, d = new_frame(W, H)
            chrome(d, W, H, "COEX ARBITRATION",
                   "one antenna, two radios: a fw policy machine grants TDMA "
                   "slots — H2C steers it, C2H reports back", gi)

            cur = (fi * 2) % NSLOT              # cursor slot this frame
            owner = slot_owner(phase, cur)
            has_lte = phase in ("c", "d")
            pol_new = phase == "b" and fi < 12   # table just swapped

            # ---- antenna ------------------------------------------------
            d.line([antx, anty + 14, antx, juncy], fill=INK, width=2)
            d.line([antx - 14, anty, antx, anty + 14], fill=INK, width=2)
            d.line([antx + 14, anty, antx, anty + 14], fill=INK, width=2)
            if owner != "X":
                r = 8 + (gi % 3) * 5
                d.arc([antx - r, anty - r + 6, antx + r, anty + r + 6],
                      200, 340, fill=AMBER)
            d.text((antx + 24, anty - 2), "one antenna", font=font(10),
                   fill=DIM)
            if owner == "X":
                d.text((antx + 24, anty + 12), "(blanked)", font=font(10),
                       fill=LTEC)

            # ---- combo chip + radios --------------------------------------
            d.rectangle(chip, outline=(0, 90, 100))
            d.text((chip[0] + 6, chip[3] + 4), "WiFi/BT combo chip",
                   font=font(10), fill=DIM)
            won = {"W": owner == "W", "B": owner == "B"}
            for box, key, name, col in ((wifi, "W", "WIFI", CYAN),
                                        (bt, "B", "BT", BLUE)):
                lit = won[key]
                oc = col if lit else (55, 70, 88)
                d.rectangle(box, outline=oc, width=2 if lit else 1)
                d.text((box[0] + 12, box[1] + 14), name,
                       font=font(13, True), fill=col if lit else DIM)
                if lit:
                    d.text((box[0] + 62, box[1] + 16), "TX/RX",
                           font=font(10), fill=col)
                # path to the antenna junction
                mx = (box[0] + box[2]) // 2
                pc = col if lit else (40, 52, 66)
                d.line([mx, box[1], mx, juncy], fill=pc,
                       width=3 if lit else 1)
                d.line([mx, juncy, antx, juncy], fill=pc,
                       width=3 if lit else 1)
                if not lit and owner != "X":
                    d.text((mx + 6, juncy + 16), "blocked", font=font(10),
                           fill=(90, 70, 70))
            d.line([antx, juncy, antx, juncy - 0], fill=INK)
            # arbiter tag inside the chip
            d.text((252, 234), "TDMA", font=font(10, True), fill=AMBER)
            d.text((246, 248), "arbiter", font=font(10), fill=AMBER)

            # ---- external LTE modem ---------------------------------------
            if has_lte:
                # dashed box
                x0, y0, x1, y1 = lte
                for x in range(x0, x1, 8):
                    d.line([x, y0, min(x + 4, x1), y0], fill=LTEC)
                    d.line([x, y1, min(x + 4, x1), y1], fill=LTEC)
                for y in range(y0, y1, 8):
                    d.line([x0, y, x0, min(y + 4, y1)], fill=LTEC)
                    d.line([x1, y, x1, min(y + 4, y1)], fill=LTEC)
                d.text((x0 + 14, y0 + 6), "LTE modem", font=font(11, True),
                       fill=LTEC)
                d.text((x0 + 14, y0 + 22), "(external)", font=font(10),
                       fill=DIM)
                # handshake line down to the chip edge
                hx = x0 + 26
                on = (gi % 4) < 2
                d.line([hx, y1, hx, chip[1]], fill=LTEC,
                       width=3 if on else 1)
                d.text((hx + 8, y1 + 16), "handshake", font=font(10, True),
                       fill=LTEC)
                d.text((hx + 8, y1 + 30), "BUSY", font=font(10, True),
                       fill=LTEC)

            # ---- host + H2C/C2H + fw policy table -------------------------
            d.rectangle(host, outline=INK)
            d.text((host[0] + 12, host[1] + 10), "host driver",
                   font=font(12, True), fill=INK)
            d.rectangle(pol, outline=(0, 90, 100))
            d.text((pol[0] + 12, pol[1] + 8), "FW COEX POLICY",
                   font=font(11, True), fill=CYAN)
            # H2C down / C2H up arrows between host and policy panel
            hx1, hx2 = pol[0] + 90, pol[0] + 210
            pulse = (gi % 4) < 2
            h2c_hot = pol_new or pulse
            d.line([hx1, host[3], hx1, pol[1]], fill=AMBER if h2c_hot else DIM,
                   width=2 if h2c_hot else 1)
            d.polygon([(hx1, pol[1]), (hx1 - 4, pol[1] - 7),
                       (hx1 + 4, pol[1] - 7)],
                      fill=AMBER if h2c_hot else DIM)
            d.text((hx1 - 78, host[3] + 6), "H2C policy", font=font(10),
                   fill=AMBER if h2c_hot else DIM)
            c2h_hot = not pulse
            d.line([hx2, pol[1], hx2, host[3]], fill=OK if c2h_hot else DIM,
                   width=2 if c2h_hot else 1)
            d.polygon([(hx2, host[3]), (hx2 - 4, host[3] + 7),
                       (hx2 + 4, host[3] + 7)], fill=OK if c2h_hot else DIM)
            d.text((hx2 + 10, host[3] + 6), "C2H telemetry", font=font(10),
                   fill=OK if c2h_hot else DIM)

            # policy rows (phase-dependent, changed rows flash amber)
            if phase == "a":
                rows = [("BT profile", "idle", False),
                        ("policy case", "WLAN only", False),
                        ("slot ratio", "W 92% : B 8%", False),
                        ("tdma table", "0x61", False)]
            else:
                rows = [("BT profile", "HFP voice", pol_new),
                        ("policy case", "BT-audio priority", pol_new),
                        ("slot ratio", "W 33% : B 67%", pol_new),
                        ("tdma table", "0x65", pol_new)]
                if has_lte:
                    rows.append(("LTE", "protect 2.4 GHz", phase == "c"
                                 and fi < 12))
            for j, (k, v, hot) in enumerate(rows):
                ry = pol[1] + 30 + j * 18
                d.text((pol[0] + 12, ry), k, font=font(11), fill=DIM)
                d.text((pol[0] + 130, ry), v, font=font(11, True),
                       fill=AMBER if hot else INK)

            # ---- TDMA arbiter bar ------------------------------------------
            d.text((bX, bY - 18), "TDMA slot grants →", font=font(11),
                   fill=DIM)
            sw = bW / NSLOT
            for s in range(NSLOT):
                o = slot_owner(phase, s)
                x0 = bX + s * sw
                col = {"W": (0, 120, 130), "B": (55, 85, 150),
                       "X": (32, 26, 44)}[o]
                d.rectangle([x0 + 1, bY, x0 + sw - 1, bY + bH], fill=col,
                            outline=LTEC if o == "X" else None)
                lab = {"W": "W", "B": "B", "X": "·"}[o]
                lc = {"W": CYAN, "B": BLUE, "X": (100, 100, 120)}[o]
                d.text((x0 + sw / 2 - 4, bY + 13), lab, font=font(12, True),
                       fill=lc)
            # cursor
            cx0 = bX + cur * sw
            d.rectangle([cx0, bY - 3, cx0 + sw, bY + bH + 3],
                        outline=INK, width=2)
            # legend
            ly = bY + bH + 12
            for lx, col, txt in ((bX, CYAN, "Wi-Fi slot"),
                                 (bX + 130, BLUE, "BT slot"),
                                 (bX + 240, LTEC, "blanked for LTE")):
                d.rectangle([lx, ly + 2, lx + 10, ly + 12], fill=col)
                d.text((lx + 16, ly), txt, font=font(10), fill=DIM)

            # ---- caption -----------------------------------------------------
            d.text((padL, H - 32), caption, font=font(12, True), fill=CYAN)
            imgs.append(img)
            gi += 1

    save_gif(imgs, args.out, ms=args.ms, colors=64)
    return 0


if __name__ == "__main__":
    sys.exit(main())
