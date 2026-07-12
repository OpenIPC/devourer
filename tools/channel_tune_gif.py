#!/usr/bin/env python3
"""Animated channel tune — 'RF18, band, bandwidth, LCK', in the DEVOURER
live-monitor style.

    tools/channel_tune_gif.py -o docs/img/channel_tune.gif

A channel switch is a small ballet across the layers: widen the BB filter mask
around the target, swap in the band's AGC table, pack channel + band + BW into
RF register 0x18 (written into every RF path — and on two-die parts through
both DAV and DDV), let the synthesizer needle overshoot and settle, then run
LCK to re-derive the VCO tuning constants before trusting the lock. The
animation stages exactly that recipe, ch 36 -> ch 149 @ 80 MHz. Needs Pillow.
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

# stages: (frames, key, caption)
STAGES = [
    (8,  "idle", "parked on ch 36 - a channel switch is a ballet, not one register"),
    (16, "filt", "1. BB filter: widen the mask 20 -> 40 -> 80 MHz for the new bandwidth"),
    (14, "agc",  "2. AGC table swap - load the 5 GHz gain staircase (a s8 table, not the 2G one)"),
    (26, "rf18", "3. RF18: channel+band+BW packed, written per path - both dies (DAV+DDV)"),
    (20, "lck",  "LCK: re-derive the VCO tuning constants before trusting the synth"),
    (12, "done", "locked: ch 149 / 5745 MHz / 80 MHz - now the synth can be trusted"),
]

CHANS = [36, 40, 44, 48, 52, 56, 60, 64, 100, 108, 116, 124, 132, 140, 149, 157, 165]
LABELED = {36, 52, 64, 100, 124, 149, 165}
CH_FROM, CH_TO = 36, 149

# RF18 bit groups (illustrative layout, 20 bits): [7:0] CHANNEL,
# [13:10] BW, [17:16] BAND
GROUPS = [("CHANNEL", 0, 8, CYAN), ("BW", 10, 4, OK), ("BAND", 16, 2, AMBER)]
OLD = {"CHANNEL": "00100100", "BW": "0000", "BAND": "01"}
NEW = {"CHANNEL": "10010101", "BW": "1100", "BAND": "10"}


def ch_x(ch, x0, x1):
    i = CHANS.index(min(CHANS, key=lambda c: abs(c - ch)))
    # position by list index (even spacing reads better than true frequency)
    f = (CHANS.index(ch) if ch in CHANS else i) / (len(CHANS) - 1)
    return x0 + f * (x1 - x0)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="channel_tune.gif")
    ap.add_argument("--ms", type=int, default=110)
    args = ap.parse_args()
    rnd = random.Random(0x18)

    W, H = 900, 520
    comb_x0, comb_x1, comb_y = 70, 840, 150
    f18_y, cell_w = 236, 27
    f18_x0 = (W - 20 * cell_w) // 2
    side_y = 330

    imgs = []
    total = sum(s[0] for s in STAGES)
    for gi in range(total):
        acc, si, ft = 0, 0, 0.0
        for k, (nf, key, cap) in enumerate(STAGES):
            if gi < acc + nf:
                si, ft = k, (gi - acc) / nf
                break
            acc += nf
        key, cap = STAGES[si][1], STAGES[si][2]
        s_of = lambda name: [s[1] for s in STAGES].index(name)
        after = lambda name: si > s_of(name)
        img, d = new_frame(W, H)
        chrome(d, W, H, "CHANNEL TUNE",
               "BB filter -> AGC table -> RF18 into every path -> LCK", gi)

        # ================= band comb + synthesizer needle =================
        d.text((40, comb_y + 30), "5 GHz band", font=font(11), fill=DIM)
        d.line([comb_x0 - 10, comb_y, comb_x1 + 10, comb_y],
               fill=(50, 70, 90), width=2)
        for ch in CHANS:
            x = ch_x(ch, comb_x0, comb_x1)
            d.line([x, comb_y - 6, x, comb_y + 6], fill=(70, 95, 120), width=1)
            if ch in LABELED:
                d.text((x - 10, comb_y + 12), str(ch), font=font(10), fill=DIM)

        # needle position: parked -> damped-overshoot sweep during rf18 -> target
        if si < s_of("rf18"):
            pos = float(CH_FROM_IDX := CHANS.index(CH_FROM))
        elif key == "rf18":
            t = ft
            a, b = CHANS.index(CH_FROM), CHANS.index(CH_TO)
            pos = b + (a - b) * math.exp(-5.0 * t) * math.cos(2 * math.pi * 1.2 * t)
            pos = max(0.0, min(pos, len(CHANS) - 1.0))
        else:
            pos = float(CHANS.index(CH_TO))
        nx = comb_x0 + (pos / (len(CHANS) - 1)) * (comb_x1 - comb_x0)
        settled = si >= s_of("lck")
        ncol = OK if si == len(STAGES) - 1 else (CYAN if key == "rf18" or settled else DIM)
        d.polygon([(nx - 7, comb_y - 26), (nx + 7, comb_y - 26), (nx, comb_y - 8)],
                  fill=ncol)
        d.line([nx, comb_y - 8, nx, comb_y + 8], fill=ncol, width=2)
        # readout above the needle (clamped inside the comb)
        cur_ch = CHANS[min(range(len(CHANS)), key=lambda i: abs(i - pos))]
        rtxt = f"ch {cur_ch}  {5000 + 5 * cur_ch} MHz"
        rx = min(max(nx - 55, comb_x0), comb_x1 - 130)
        d.text((rx, comb_y - 44), rtxt, font=font(11, True), fill=ncol)
        # BB filter brackets around the target channel on the comb
        bw_now = 20
        if key == "filt":
            bw_now = (20, 40, 80)[min(2, int(ft * 3))]
        elif after("filt"):
            bw_now = 80
        tx_ = ch_x(CH_TO, comb_x0, comb_x1)
        half = {20: 14, 40: 28, 80: 56}[bw_now] // 1
        bc = AMBER if key == "filt" else ((60, 70, 40) if after("filt") else GRID)
        d.line([tx_ - half, comb_y + 26, tx_ - half, comb_y + 34], fill=bc, width=2)
        d.line([tx_ - half, comb_y + 34, tx_ + half, comb_y + 34], fill=bc, width=1)
        d.line([tx_ + half, comb_y + 26, tx_ + half, comb_y + 34], fill=bc, width=2)
        if key == "filt" or after("filt"):
            d.text((tx_ - 28, comb_y + 40), f"{bw_now} MHz", font=font(10), fill=bc)

        # ================= RF18 field diagram ==============================
        d.text((f18_x0, f18_y - 20), "RF reg 0x18 (RF18) - 20 bits, per RF path",
               font=font(11), fill=DIM)
        filled = 0
        if key == "rf18":
            filled = int(ft * 3) + 1        # groups fill one after another
        elif after("rf18"):
            filled = 3
        for b in range(20):
            x = f18_x0 + (19 - b) * cell_w  # bit 19 leftmost
            grp = next((g for g in GROUPS if g[1] <= b < g[1] + g[2]), None)
            oc = grp[3] if grp else GRID
            d.rectangle([x, f18_y, x + cell_w - 3, f18_y + 24], outline=oc)
            if grp:
                name, lo, ln, col = grp
                gi_ = GROUPS.index(grp)
                src = NEW if gi_ < filled else OLD
                bit = src[name][ln - 1 - (b - lo)]
                hot = key == "rf18" and gi_ == filled - 1
                d.text((x + 9, f18_y + 5), bit, font=font(12, hot),
                       fill=INK if gi_ < filled else (70, 95, 120))
            else:
                d.text((x + 9, f18_y + 5), "0", font=font(12), fill=(45, 60, 78))
        for name, lo, ln, col in GROUPS:
            xl = f18_x0 + (19 - (lo + ln - 1)) * cell_w
            d.text((xl + 2, f18_y + 32), name, font=font(10), fill=col)
        if key == "rf18":
            d.text((f18_x0, f18_y + 54),
                   f"write path A ... path B{'   [DAV ok] [DDV ok]' if ft > 0.6 else ''}",
                   font=font(11), fill=CYAN)
        elif after("rf18"):
            d.text((f18_x0, f18_y + 54), "written: path A + path B (DAV + DDV)",
                   font=font(11), fill=(60, 130, 100))

        # ================= side elements: AGC swap + LCK ===================
        # AGC swap (left)
        agc_on = key == "agc"
        agc_done = after("agc")
        ac = AMBER if agc_on else ((60, 70, 40) if agc_done else GRID)
        d.rectangle([70, side_y, 190, side_y + 88], outline=ac)
        d.text((80, side_y + 6), "AGC table", font=font(11, True),
               fill=INK if (agc_on or agc_done) else DIM)
        # staircase icon: old (dim) vs new (bright) drawn as steps
        swap = ft if agc_on else (1.0 if agc_done else 0.0)
        for stp in range(5):
            sy = side_y + 70 - stp * 9
            sx = 82 + stp * 18
            newish = stp / 5.0 < swap
            d.line([sx, sy, sx + 18, sy],
                   fill=(OK if newish else (70, 95, 120)) if (agc_on or agc_done)
                   else GRID, width=2)
        d.text((80, side_y + 94), "5G gain staircase" if (agc_done or swap > 0.5)
               else "2G gain staircase", font=font(10), fill=ac)

        # RF paths note (middle)
        d.rectangle([250, side_y, 470, side_y + 88], outline=GRID)
        d.text((260, side_y + 6), "RF paths", font=font(11, True), fill=DIM)
        for pi, pname in enumerate(("path A", "path B")):
            py = side_y + 32 + pi * 26
            got = after("rf18") or (key == "rf18" and ft > (0.4 + 0.3 * pi))
            d.text((260, py), pname, font=font(11), fill=INK if got else DIM)
            d.text((330, py), "RF18 = 0x2C395" if got else "RF18 = 0x24024",
                   font=font(11), fill=OK if got else (70, 95, 120))

        # LCK lock indicator (right)
        lck_on = key == "lck"
        lck_done = si == len(STAGES) - 1
        lc = CYAN if lck_on else (OK if lck_done else GRID)
        d.rectangle([530, side_y, 840, side_y + 88], outline=lc)
        d.text((542, side_y + 6), "synthesizer PLL", font=font(11, True),
               fill=INK if (lck_on or lck_done) else DIM)
        cx, cy = 570, side_y + 54
        if lck_on:
            # oscillate red/green while the VCO cal hunts, settle at the end
            hunt = math.sin(ft * math.pi * 6) * (1 - ft)
            locked = ft > 0.8
            col = OK if locked or hunt > 0 else WARN
            d.ellipse([cx - 12, cy - 12, cx + 12, cy + 12], fill=col)
            d.text((cx + 24, cy - 8), "LOCK" if locked else "cal...",
                   font=font(12, True), fill=col)
            d.text((cx + 130, cy - 8), f"VCO const {int(ft * 8):d}/8",
                   font=font(11), fill=DIM)
        elif lck_done:
            d.ellipse([cx - 12, cy - 12, cx + 12, cy + 12], fill=OK)
            d.text((cx + 24, cy - 8), "LOCK OK", font=font(12, True), fill=OK)
            d.text((542, side_y + 70), "thermal delta will re-arm LCK",
                   font=font(10), fill=DIM)
        else:
            d.ellipse([cx - 12, cy - 12, cx + 12, cy + 12], outline=GRID)
            d.text((cx + 24, cy - 8), "unlocked", font=font(11), fill=DIM)

        # ---- caption ------------------------------------------------------
        d.text((70, H - 42), cap, font=font(13, True),
               fill=OK if key == "done" else CYAN)
        imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms, colors=56)
    return 0


if __name__ == "__main__":
    sys.exit(main())
