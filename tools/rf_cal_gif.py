#!/usr/bin/env python3
"""Animated RF calibration — 'halrf's per-boot rituals', in the DEVOURER
live-monitor style.

    tools/rf_cal_gif.py -o docs/img/rf_cal.gif

Split view: an IQ constellation (16-QAM) on the left, a spectrum strip on the
right. The chip starts uncalibrated — the lattice is smeared and skewed by IQ
gain/phase imbalance, biased off-center by DAC offsets, and the RX mixers leak
a tall DC spike mid-spectrum. Then the per-boot rituals run in bring-up order,
each looping TX into RX on a tiny chip inset: DACK trims the DAC offsets (the
cloud de-biases), RX-DCK collapses the DC spike, IQK unwinds the gain/phase
skew (the lattice snaps square, the image tone shrinks). It ends on the loop
that never ends: thermal tracking re-trims TX gain as the chip heats. Needs
Pillow.
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

# (name, frames, loopback-active, caption)
STAGES = [
    ("UNCAL", 12, False,
     "power-on: raw analog — IQ mismatch smears the lattice, the mixers leak DC"),
    ("DACK", 16, True,
     "1. DACK — trim the TX DACs' offset and gain: the cloud de-biases"),
    ("RX-DCK", 16, True,
     "2. RX-DCK — null the RX mixer DC leak: the mid-spectrum spike collapses"),
    ("IQK", 20, True,
     "3. IQK — TX looped into RX, image tone measured: the lattice snaps square"),
    ("THERMAL", 26, False,
     "not a one-shot: thermal tracking re-trims as the chip heats"),
]


def smooth(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="rf_cal.gif")
    ap.add_argument("--ms", type=int, default=110)
    args = ap.parse_args()
    rnd = random.Random(0x8852)

    W, H = 900, 500
    padL = 34
    # constellation panel (left, square)
    cG = 320
    cX, cY = padL, 104
    ccx, ccy = cX + cG // 2, cY + cG // 2
    scale = cG / 2 / 1.85
    # spectrum strip (right, top)
    sX, sY, sW, sH = 430, 104, 436, 130
    # chip inset with loopback (right, middle)
    iX, iY, iW, iH = 430, 258, 250, 122
    # checklist (right of inset)
    kX, kY = 706, 258

    # ideal 16-QAM (unit-ish coordinates)
    ideal = [((2 * i - 3) / 3.0, (2 * q - 3) / 3.0)
             for i in range(4) for q in range(4)]
    # fixed noise per dot so only the impairment transform changes per frame
    dots = [(ix, iy, rnd.gauss(0, 1), rnd.gauss(0, 1))
            for (ix, iy) in ideal for _ in range(9)]
    # fixed spectrum noise floor shape
    floor = [rnd.uniform(0.0, 1.0) for _ in range(sW)]

    imgs = []
    gi = 0
    for si, (sname, nfr, loop_on, caption) in enumerate(STAGES):
        for fi in range(nfr):
            t = smooth((fi + 1) / nfr)          # progress within this stage
            # impairment levels: 1 = full error, 0 = calibrated out
            dc = 1.0 if si < 1 else (1.0 - t if si == 1 else 0.0)
            spike = 1.0 if si < 2 else (1.0 - t if si == 2 else 0.0)
            skew = 1.0 if si < 3 else (1.0 - t if si == 3 else 0.0)
            heat = t if si == 4 else 0.0        # thermal ticker 0..1

            img, d = new_frame(W, H)
            chrome(d, W, H, "RF CALIBRATION",
                   "halrf per-boot rituals: DACK → RX-DCK → IQK, then thermal "
                   "tracking forever after", gi)

            # ---- constellation panel -------------------------------------
            d.rectangle([cX, cY, cX + cG, cY + cG], outline=(0, 70, 80))
            d.line([ccx, cY, ccx, cY + cG], fill=GRID)
            d.line([cX, ccy, cX + cG, ccy], fill=GRID)
            d.text((cX + cG - 14, ccy + 4), "I", font=font(11), fill=DIM)
            d.text((ccx + 5, cY + 3), "Q", font=font(11), fill=DIM)
            d.text((cX + 6, cY - 16), "16-QAM · RX'd through own loopback",
                   font=font(11), fill=DIM)
            # ideal lattice crosses
            for (px, py) in ideal:
                x, y = ccx + px * scale, ccy - py * scale
                d.line([x - 3, y, x + 3, y], fill=(60, 80, 100))
                d.line([x, y - 3, x, y + 3], fill=(60, 80, 100))
            # received dots: gain imbalance + phase skew + DC bias + noise
            g_i = 1.0 + 0.24 * skew
            eps = 0.30 * skew
            dcx, dcy = 0.22 * dc, 0.15 * dc
            sig = 0.030 + 0.028 * skew
            wob = 0.010 * heat * math.sin(gi * 0.7)   # tiny thermal breathing
            for (ix, iy, nx, ny) in dots:
                x0 = ix * (g_i + wob) + eps * iy + dcx + nx * sig
                y0 = iy + dcy + ny * sig
                x = max(cX + 4, min(cX + cG - 4, ccx + x0 * scale))
                y = max(cY + 4, min(cY + cG - 4, ccy - y0 * scale))
                bad = skew > 0.45 or dc > 0.45
                col = WARN if bad else (AMBER if skew + dc > 0.05 else OK)
                d.ellipse([x - 2, y - 2, x + 2, y + 2], fill=col)
            if si == 0:
                d.text((cX + 8, cY + cG - 20), "smeared + off-center",
                       font=font(11, True), fill=WARN)
            elif si == 4:
                d.text((cX + 8, cY + cG - 20), "clean lattice",
                       font=font(11, True), fill=OK)

            # ---- spectrum strip ------------------------------------------
            d.rectangle([sX, sY, sX + sW, sY + sH], outline=(0, 70, 80))
            d.text((sX + 6, sY - 16), "loopback spectrum", font=font(11),
                   fill=DIM)
            base = sY + sH - 12
            sig_x = int(sW * 0.72)              # wanted tone
            img_x = int(sW * 0.28)              # IQ image (mirror)
            dc_x = sW // 2                      # DC bin
            pts = []
            for x in range(sW):
                h = 8 + 5 * floor[x]
                for (cx0, amp, wd) in ((sig_x, 88, 7.0),
                                       (img_x, 62 * skew, 7.0),
                                       (dc_x, 96 * spike, 3.5)):
                    h += amp * math.exp(-((x - cx0) / wd) ** 2)
                pts.append((sX + x, base - min(sH - 20, h)))
            d.line(pts, fill=CYAN, width=1)
            d.line([sX + dc_x, sY + 6, sX + dc_x, base], fill=(30, 40, 54))
            d.text((sX + sig_x - 22, base - 112), "signal", font=font(10),
                   fill=OK)
            if spike > 0.12:
                d.text((sX + dc_x + 6, sY + 8), "DC spike", font=font(10),
                       fill=WARN)
            if skew > 0.12:
                d.text((sX + img_x - 20, base - 96), "image", font=font(10),
                       fill=AMBER)
            if spike < 0.12 and skew < 0.12:
                d.text((sX + 8, sY + 8), "flat — image + DC gone",
                       font=font(10), fill=OK)

            # ---- chip inset: TX -> RX loopback path ----------------------
            d.rectangle([iX, iY, iX + iW, iY + iH], outline=(0, 70, 80))
            d.text((iX + 8, iY + 6), "chip (RF loopback)", font=font(10),
                   fill=DIM)
            txb = [iX + 16, iY + 34, iX + 76, iY + 60]
            rxb = [iX + 16, iY + 82, iX + 76, iY + 108]
            d.rectangle(txb, outline=INK)
            d.text((txb[0] + 16, txb[1] + 5), "TX", font=font(11, True),
                   fill=INK)
            d.rectangle(rxb, outline=INK)
            d.text((rxb[0] + 16, rxb[1] + 5), "RX", font=font(11, True),
                   fill=INK)
            lc = AMBER if loop_on else (45, 58, 74)
            lx = iX + 120
            d.line([txb[2], iY + 47, lx, iY + 47], fill=lc, width=2)
            d.line([lx, iY + 47, lx, iY + 95], fill=lc, width=2)
            d.line([lx, iY + 95, rxb[2], iY + 95], fill=lc, width=2)
            d.polygon([(rxb[2] + 8, iY + 95), (rxb[2] + 16, iY + 90),
                       (rxb[2] + 16, iY + 100)], fill=lc)
            if loop_on:
                # a pulse dot running around the loop
                path = [(txb[2] + k, iY + 47) for k in range(lx - txb[2])] + \
                       [(lx, iY + 47 + k) for k in range(48)] + \
                       [(lx - k, iY + 95) for k in range(lx - rxb[2])]
                px, py = path[(gi * 9) % len(path)]
                d.ellipse([px - 3, py - 3, px + 3, py + 3], fill=AMBER)
                d.text((iX + 138, iY + 62), "TX→RX", font=font(10, True),
                       fill=AMBER)
                d.text((iX + 138, iY + 76), "loopback", font=font(10),
                       fill=AMBER)
            else:
                d.text((iX + 138, iY + 62), "loop idle", font=font(10),
                       fill=DIM)

            # ---- checklist ------------------------------------------------
            d.text((kX, kY), "CAL SEQUENCE", font=font(11, True), fill=CYAN)
            for j, nm in enumerate(("DACK", "RX-DCK", "IQK", "THERMAL")):
                ry = kY + 22 + j * 24
                run = (j + 1) == si or (j == 3 and si == 4)
                done = (j + 1) < si
                col = CYAN if run else (OK if done else DIM)
                mark = "▶" if run else ("✓" if done else "·")
                d.text((kX, ry), f"{mark} {nm}", font=font(12, run or done),
                       fill=col)

            # ---- thermal ticker (finale) ---------------------------------
            ty = iY + iH + 22
            d.text((iX, ty), "thermal meter", font=font(10), fill=DIM)
            bx0, bx1 = iX + 118, iX + 356
            d.rectangle([bx0, ty, bx1, ty + 12], outline=(0, 70, 80))
            fillw = int((bx1 - bx0 - 2) * (0.25 + 0.6 * heat))
            hc = OK if heat < 0.5 else AMBER
            d.rectangle([bx0 + 1, ty + 1, bx0 + 1 + fillw, ty + 11], fill=hc)
            d.text((bx1 + 10, ty), f"Δ{int(heat * 8):d}", font=font(11, True),
                   fill=hc)
            if si == 4 and heat > 0.55:
                d.text((iX + 118, ty + 18),
                       "Δ crossed a step — TX gain re-trimmed (−1)",
                       font=font(10, True), fill=AMBER)

            # ---- caption --------------------------------------------------
            d.text((padL, H - 34), caption, font=font(13, True), fill=CYAN)
            imgs.append(img)
            gi += 1

    save_gif(imgs, args.out, ms=args.ms, colors=56)
    return 0


if __name__ == "__main__":
    sys.exit(main())
