#!/usr/bin/env python3
"""Animated 802.11ax Trigger frames — 'built, aired, read back', in the DEVOURER
live-monitor style.

    tools/he_ofdma_trigger_gif.py -o docs/img/he_ofdma_trigger.gif

What devourer does with 802.11ax uplink scheduling, exactly: it composes a
Trigger frame from a per-user grant table — who, which resource unit, what MCS,
how many spatial streams, what receive power to aim for — puts it on the air,
and an independent monitor decodes it back with every commanded parameter
intact. That round trip is the validated capability, and it makes the adapter a
usable instrument for 11ax work: arbitrary, exactly-specified Trigger frames on
demand.

Three columns, because that is what the measurement is: what was asked for, what
the frame actually looks like, and what came back off the air. The footer states
the boundary in one line — the frame is exact, but the hardware-timed reply
belongs to firmware these client parts do not ship — without spending half the
canvas miming it. Needs Pillow.
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, GRID, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

# who, AID, which resource unit, MCS, spatial streams, target RSSI
USERS = [
    ("UE-A", 1, "52-tone #1", "MCS4", 1, -60),
    ("UE-B", 2, "52-tone #2", "MCS2", 1, -62),
    ("UE-C", 3, "52-tone #3", "MCS6", 1, -58),
]
FIELDS = [
    ("frame control", "0x24 — Trigger", AMBER),
    ("duration", "", DIM),
    ("RA", "broadcast", DIM),
    ("TA", "our MAC", DIM),
    ("common info", "BW · GI/LTF · AP power", CYAN),
    ("user info", "UE-A · RU · MCS · NSS · RSSI", CYAN),
    ("user info", "UE-B · RU · MCS · NSS · RSSI", CYAN),
    ("user info", "UE-C · RU · MCS · NSS · RSSI", CYAN),
    ("FCS", "", DIM),
]

P1, P2, P3 = 10, 30, 50      # compose / assemble + air / decode


def check(d, x, y, col):
    d.line([x, y + 5, x + 4, y + 9], fill=col, width=2)
    d.line([x + 4, y + 9, x + 11, y], fill=col, width=2)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="he_ofdma_trigger.gif")
    ap.add_argument("--hold", type=int, default=20)
    ap.add_argument("--ms", type=int, default=110)
    args = ap.parse_args()

    padT = 106
    ax, aw = 46, 252
    bx, bw = 330, 300
    cx, cw = 672, 268
    W = cx + cw + 34
    H = 476

    imgs = []
    for fi in range(P3 + args.hold):
        n_cmd = max(0, min(len(USERS) + 1, (fi * (len(USERS) + 1)) // P1))
        n_fld = 0 if fi < P1 else min(len(FIELDS),
                                      ((fi - P1) * len(FIELDS)) // (P2 - P1))
        n_dec = 0 if fi < P2 else min(len(USERS) + 1,
                                      ((fi - P2) * (len(USERS) + 1)) // (P3 - P2))
        aired = fi >= P2
        img, d = new_frame(W, H)
        chrome(d, W, H, "802.11ax TRIGGER FRAMES — BUILT, AIRED, READ BACK",
               "arbitrary per-user grants composed on the host, transmitted, "
               "and decoded off the air unchanged", fi)

        def col_head(x, w, text, colr):
            d.text((x, padT - 26), text, font=font(12), fill=colr)
            d.line([x, padT - 8, x + w, padT - 8], fill=(0, 70, 80))

        # ---- what was asked for --------------------------------------------
        col_head(ax, aw, "COMMANDED — the grant table", CYAN)
        y = padT + 4
        if n_cmd > 0:
            d.text((ax, y), "AP TX power", font=font(11), fill=DIM)
            d.text((ax + 148, y), "20 dBm", font=font(11, True), fill=INK)
        y += 26
        for i, (who, aid, ru, mcs, nss, rssi) in enumerate(USERS):
            if i + 1 >= n_cmd:
                break
            d.rectangle([ax, y, ax + aw, y + 52], outline=(0, 60, 72))
            d.text((ax + 8, y + 6), who, font=font(12, True), fill=CYAN)
            d.text((ax + 62, y + 8), f"AID {aid}", font=font(10), fill=DIM)
            d.text((ax + 8, y + 24), f"RU {ru}", font=font(10), fill=INK)
            d.text((ax + 8, y + 37), f"{mcs} · NSS {nss} · target {rssi} dBm",
                   font=font(10), fill=INK)
            y += 58

        # ---- the frame that carries it -------------------------------------
        col_head(bx, bw, "THE FRAME ON AIR", OK if aired else AMBER)
        y = padT + 4
        for i, (name, note, colr) in enumerate(FIELDS):
            if i >= n_fld:
                break
            d.rectangle([bx, y, bx + bw, y + 20], outline=(0, 60, 72),
                        fill=(16, 22, 30) if colr is DIM else None)
            d.text((bx + 8, y + 3), name, font=font(10), fill=colr)
            if note:
                d.text((bx + 106, y + 3), note, font=font(10), fill=DIM)
            y += 23
        if n_fld >= len(FIELDS):
            y += 6
            d.text((bx, y), "the channel's slices, as granted", font=font(10),
                   fill=DIM)
            y += 16
            slot = bw / 4
            for i in range(4):
                sx = bx + i * slot
                on = i < len(USERS)
                d.rectangle([sx + 1, y, sx + slot - 2, y + 26],
                            fill=CYAN if on else (24, 30, 40))
                d.text((sx + 10, y + 7), USERS[i][0] if on else "—",
                       font=font(11, True), fill=(8, 11, 18) if on else DIM)
            y += 34
            if aired:
                d.text((bx, y), "transmitted on the ordinary management path",
                       font=font(10), fill=OK)

        # ---- what came back --------------------------------------------------
        col_head(cx, cw, "DECODED — independent monitor", OK if n_dec else DIM)
        y = padT + 4
        if n_dec > 0:
            d.text((cx, y), "AP TX power", font=font(11), fill=DIM)
            d.text((cx + 130, y), "20 dBm", font=font(11, True), fill=INK)
            check(d, cx + 214, y + 1, OK)
        y += 26
        for i, (who, aid, ru, mcs, nss, rssi) in enumerate(USERS):
            if i + 1 >= n_dec:
                break
            d.rectangle([cx, y, cx + cw, y + 52], outline=(0, 60, 72))
            d.text((cx + 8, y + 6), who, font=font(12, True), fill=OK)
            d.text((cx + 62, y + 8), f"AID {aid}", font=font(10), fill=DIM)
            d.text((cx + 8, y + 24), f"RU {ru}", font=font(10), fill=INK)
            d.text((cx + 8, y + 37), f"{mcs} · NSS {nss} · target {rssi} dBm",
                   font=font(10), fill=INK)
            check(d, cx + cw - 26, y + 20, OK)
            y += 58

        if n_dec > len(USERS):
            d.rectangle([cx, y + 6, cx + cw, y + 38], outline=OK, width=2)
            d.ellipse([cx + 10, y + 16, cx + 20, y + 26], fill=OK)
            d.text((cx + 30, y + 12), "EVERY PARAMETER MATCHES",
                   font=font(12, True), fill=OK)

        # ---- the one line that keeps it honest -------------------------------
        if n_dec > len(USERS):
            d.line([ax, H - 54, W - 34, H - 54], fill=(0, 55, 66))
            d.text((ax, H - 44), "the frame is exact. the hardware-timed reply "
                   "is not ours to schedule — only a firmware-scheduled trigger "
                   "arms the receiving", font=font(10), fill=DIM)
            d.text((ax, H - 30), "MAC's SIFS window, and the shipped client "
                   "firmware does not air one.", font=font(10), fill=DIM)

        imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms, colors=48)
    return 0


if __name__ == "__main__":
    sys.exit(main())
