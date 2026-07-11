#!/usr/bin/env python3
"""Animated "one clock for many radios" chain for docs/rf-primer.md — the
time-distribution machinery as a four-act loop in the DEVOURER monitor style,
including the Wi-Fi MAC internals that make the radio hop work.

Three lanes, each a clock rendered as a tick comb (one tick per beacon
interval) plus a live counter. The AP and STATION lanes open up to show the
silicon path:

  AP:      TSF counter -> TBTT comparator (TSF mod interval == off -> fire)
           -> reserved-page beacon, whose TIMESTAMP field is written with the
           live TSF as the frame passes to the antenna (egress stamping)
  STATION: antenna -> tsfl latch (the local TSF frozen in hardware at frame
           arrival) -> the (beacon_ts, tsfl) pair feeding the linear fit

Acts (cycled by the loop):
  1  FREE-RUNNING   crystals drift: the AP and station combs slide off the grid
  2  PTP (wire)     hardware-timestamped sync pulses discipline the AP TSF
  3  BEACONS (air)  hardware egress stamp + hardware arrival latch pull the
                    station in (ghost dots: where software stamps would land)
  4  TBTT PIN       the comparator's offset snaps to the disciplined grid
                    (PinBeaconTbtt) while the TSF counter runs unbroken

    tools/timesync_chain_gif.py -o docs/img/timesync_chain.gif

Needs Pillow.
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

W, H = 920, 700
LANES = {  # y-band per lane
    "gm": (86, 168),
    "ap": (188, 388),
    "sta": (408, 572),
}
LX0, LX1 = 210, 880          # comb x-span
TICK_PX = 74                 # px per beacon interval on the comb
ACTS = [(0, 22), (22, 44), (44, 82), (82, 108)]   # frame ranges per act
TOTAL = ACTS[-1][1]
BLUE = (90, 140, 255)


def act_of(fr):
    for i, (a, b) in enumerate(ACTS):
        if a <= fr < b:
            return i, (fr - a) / max(1, b - a - 1)
    return len(ACTS) - 1, 1.0


def lane_box(d, key, label, sub, col):
    y0, y1 = LANES[key]
    d.rectangle([28, y0, W - 28, y1], outline=GRID, width=1)
    d.text((44, y0 + 10), label, font=font(15, True), fill=col)
    d.text((44, y0 + 32), sub, font=font(11), fill=DIM)


def comb(d, key, ytop, offset_px, col, flags=None, flag_col=None):
    """Tick comb near ytop: one tick per interval, slid by offset_px."""
    cy0, cy1 = ytop, ytop + 26
    for k in range(-1, int((LX1 - LX0) / TICK_PX) + 2):
        x = LX0 + k * TICK_PX + (offset_px % TICK_PX)
        if not (LX0 - 2 <= x <= LX1 + 2):
            continue
        d.line([x, cy0, x, cy1], fill=col, width=2)
        if flags is not None:
            fx = x + flags
            if LX0 - 2 <= fx <= LX1 + 2:
                d.polygon([(fx, cy0 - 2), (fx + 9, cy0 - 8), (fx, cy0 - 14)],
                          fill=flag_col or AMBER)


def block(d, x0, y0, x1, y1, title, col, lit=False):
    if lit:
        d.rectangle([x0 - 2, y0 - 2, x1 + 2, y1 + 2], outline=col)
    d.rectangle([x0, y0, x1, y1], outline=col if lit else DIM)
    d.text((x0 + 8, y0 + 5), title, font=font(11, True),
           fill=col if lit else DIM)
    return (x0, y0, x1, y1)


def arrow(d, x0, y, x1, col):
    d.line([x0, y, x1 - 7, y], fill=col, width=1)
    d.polygon([(x1 - 7, y - 4), (x1, y), (x1 - 7, y + 4)], fill=col)


def antenna(d, cx, cy, col):
    d.line([cx, cy + 18, cx, cy], fill=col, width=2)
    d.polygon([(cx - 6, cy + 2), (cx + 6, cy + 2), (cx, cy - 7)], fill=col)
    for r in (9, 15):
        d.arc([cx - r, cy - r - 6, cx + r, cy + r - 6], 220, 320, fill=col)


def main() -> int:
    ap_ = argparse.ArgumentParser(description=__doc__)
    ap_.add_argument("-o", "--out", default="timesync_chain.gif")
    ap_.add_argument("--ms", type=int, default=115)
    args = ap_.parse_args()

    rng = random.Random(7)
    imgs = []
    ap_drift, sta_drift = 1.35, -1.05
    ap_off = 0.0     # AP comb offset vs the GM grid (px)
    sta_off = 26.0   # station comb offset vs the AP comb (px)
    tbtt_flag = 30.0  # beacon-schedule offset vs the AP comb (act 4 pins to 0)
    resid_hist = []

    captions = [
        ("1/4  FREE-RUNNING", "every crystal keeps its own time — tens of ppm "
                              "apart, they drift milliseconds within minutes", WARN),
        ("2/4  PTP OVER THE WIRE", "hardware timestamps at the Ethernet PHY "
                                   "discipline the AP's TSF counter to ~300 ns", CYAN),
        ("3/4  BEACONS OVER THE AIR", "TSF stamped INTO the frame at egress; the "
                                      "station latches its clock at arrival — "
                                      "silicon on both ends", OK),
        ("4/4  THE TBTT PIN", "PinBeaconTbtt moves only the comparator's offset — "
                              "the TSF counter the servo reads runs unbroken", AMBER),
    ]

    for fr in range(TOTAL):
        act, t = act_of(fr)
        im, d = new_frame(W, H)
        chrome(d, W, H, "TIME DISTRIBUTION — ONE CLOCK FOR MANY RADIOS", "", fr)

        lane_box(d, "gm", "GRANDMASTER", "Intel I226 PHC (IEEE 1588, wired)", INK)
        lane_box(d, "ap", "AP / MASTER",
                 "8821CE — inside the Wi-Fi MAC", CYAN)
        lane_box(d, "sta", "STATION",
                 "any devourer RX — inside its MAC", OK)

        # --- clock dynamics per act ------------------------------------------
        if act == 0:
            ap_off += ap_drift
            sta_off += sta_drift
        elif act == 1:
            ap_off *= 0.82
            sta_off += sta_drift * 0.7
        elif act == 2:
            ap_off *= 0.9
            sta_off *= 0.80
        else:
            ap_off *= 0.9
            sta_off *= 0.9
            tbtt_flag *= 0.78

        base = 1_000_000 + fr * 102_400
        ap_us = base + ap_off * 320
        sta_us = base + (ap_off + sta_off) * 320

        # --- GRANDMASTER lane --------------------------------------------------
        comb(d, "gm", LANES["gm"][1] - 42, 0, INK)
        d.text((W - 250, LANES["gm"][0] + 10), f"{base/1e6:12.6f} s",
               font=font(15, True), fill=INK)

        # --- AP lane: MAC internals ---------------------------------------------
        ay0, ay1 = LANES["ap"]
        beat = fr % 4                 # beacon cadence for the frame animation
        firing = act >= 2 and beat in (0, 1, 2)
        # TSF counter block (the register the wire servo disciplines)
        b_tsf = block(d, 60, ay0 + 56, 240, ay0 + 96, "TSF COUNTER", CYAN, lit=True)
        d.text((b_tsf[0] + 8, b_tsf[1] + 21), f"{ap_us:12.0f} us",
               font=font(12, True), fill=CYAN)
        # TBTT comparator
        b_cmp = block(d, 280, ay0 + 56, 500, ay0 + 96, "TBTT COMPARE",
                      AMBER if firing and beat == 0 else CYAN,
                      lit=firing and beat == 0)
        d.text((b_cmp[0] + 8, b_cmp[1] + 21),
               f"TSF%interval=={int(abs(tbtt_flag))*14:d}", font=font(11), fill=DIM)
        # Reserved page (the stored beacon frame)
        b_rp = block(d, 540, ay0 + 56, 700, ay0 + 96, "RSVD PAGE", CYAN)
        d.text((b_rp[0] + 8, b_rp[1] + 21), "beacon frame", font=font(11), fill=DIM)
        arrow(d, 240, ay0 + 76, 280, DIM)
        arrow(d, 500, ay0 + 76, 540, AMBER if firing and beat == 0 else DIM)
        antenna(d, 790, ay0 + 62, CYAN if firing else DIM)
        # the frame in flight: rsvd page -> antenna, TS field filled mid-way
        if firing:
            fx = 700 + (770 - 700) * (beat + t) / 3.0
            d.rectangle([fx, ay0 + 66, fx + 46, ay0 + 88], outline=OK)
            stamped = fx > 726
            d.rectangle([fx + 4, ay0 + 70, fx + 24, ay0 + 84],
                        outline=OK if stamped else DIM)
            d.text((fx + 7, ay0 + 72), "TS", font=font(9),
                   fill=OK if stamped else DIM)
            if 720 < fx < 740:   # the egress-stamp moment
                d.line([b_tsf[2] - 60, ay0 + 96, fx + 12, ay0 + 70],
                       fill=OK, width=1)
                d.text((fx - 40, ay0 + 100), "TSF written at egress",
                       font=font(10), fill=OK)
        # AP comb + pinned flags at the lane bottom
        comb(d, "ap", ay1 - 42, ap_off, CYAN,
             flags=tbtt_flag if act >= 2 else None, flag_col=AMBER)
        d.text((W - 250, ay0 + 10), f"{ap_us/1e6:12.6f} s",
               font=font(15, True), fill=CYAN)
        if act == 3 and t < 0.4:
            d.text((60, ay1 - 66), "PIN: comparator offset -> grid "
                                   "(TSF untouched)", font=font(11, True),
                   fill=AMBER)

        # --- air gap: beacon pulse + software ghost ------------------------------
        if act >= 2:
            ph = (fr % 4) / 3.0
            x = 790
            y = ay1 + (LANES["sta"][0] - ay1) * min(1.0, ph)
            d.ellipse([x - 4, y + 6, x + 4, y + 14], fill=OK)
            d.text((x + 8, y + 2), "beacon", font=font(9), fill=OK)
            gx = x + rng.uniform(-26, 26)
            d.ellipse([gx - 3, y + 8, gx + 3, y + 14], outline=DIM)

        # --- STATION lane: MAC internals -------------------------------------------
        sy0, sy1 = LANES["sta"]
        antenna(d, 790, sy0 + 62, OK if act >= 2 else DIM)
        latch_hit = act >= 2 and beat == 3
        b_lat = block(d, 540, sy0 + 56, 700, sy0 + 96, "tsfl LATCH",
                      OK if latch_hit else DIM, lit=latch_hit)
        d.text((b_lat[0] + 8, b_lat[1] + 21),
               f"{sta_us:12.0f} us" if latch_hit else "frozen at arrival",
               font=font(11, True) if latch_hit else font(11),
               fill=OK if latch_hit else DIM)
        b_fit = block(d, 280, sy0 + 56, 500, sy0 + 96, "FIT  m = a*t + b",
                      OK if act >= 2 else DIM, lit=act >= 2)
        d.text((b_fit[0] + 8, b_fit[1] + 21),
               "(beacon TS, tsfl) pairs", font=font(11), fill=DIM)
        b_clk = block(d, 60, sy0 + 56, 240, sy0 + 96, "LOCAL TSF", OK, lit=True)
        d.text((b_clk[0] + 8, b_clk[1] + 21), f"{sta_us:12.0f} us",
               font=font(12, True), fill=OK)
        arrow(d, 540, sy0 + 76, 500, OK if latch_hit else DIM)
        arrow(d, 280, sy0 + 76, 240, OK if act >= 2 else DIM)
        comb(d, "sta", sy1 - 42, ap_off + sta_off, OK)
        d.text((W - 250, sy0 + 10), f"{sta_us/1e6:12.6f} s",
               font=font(15, True), fill=OK)

        # --- wire sync pulses (act 2) --------------------------------------------
        if act == 1 and (fr % 5) in (0, 1, 2):
            x = 150
            y_from, y_to = LANES["gm"][1] - 20, LANES["ap"][0] + 56
            tt = ((fr % 5) + t) / 3.0
            y = y_from + (y_to - y_from) * tt
            d.ellipse([x - 4, y - 4, x + 4, y + 4], fill=CYAN)
            d.line([x, y_from, x, y], fill=CYAN, width=1)
            if tt < 0.6:
                d.text((x + 8, y - 6), "sync (hw ts)", font=font(10), fill=CYAN)

        # --- residual sparkline -------------------------------------------------------
        resid = abs(ap_off + sta_off) * 12 + rng.uniform(0, 1.5)
        resid_hist.append(min(resid, 400))
        y0, y1 = 596, 648
        d.rectangle([28, y0, W - 28, y1], outline=GRID)
        d.text((44, y0 + 4), "station vs wire clock", font=font(11), fill=DIM)
        pts = resid_hist[-90:]
        for i, r in enumerate(pts):
            x = 200 + i * 7
            if x > W - 40:
                break
            hgt = min(44, 4 + 40 * math.log10(1 + r) / math.log10(401))
            d.line([x, y1 - 2, x, y1 - 2 - hgt],
                   fill=OK if r < 5 else (AMBER if r < 60 else WARN))
        d.text((W - 250, y0 + 4),
               f"residual ~{max(0.3, resid):7.1f} us", font=font(12, True),
               fill=OK if resid < 5 else (AMBER if resid < 60 else WARN))

        cap, sub, col = captions[act]
        d.text((28, 54), cap, font=font(14, True), fill=col)
        d.text((248, 57), sub, font=font(11), fill=DIM)

        imgs.append(im)

    save_gif(imgs, args.out, ms=args.ms)
    print(f"wrote {args.out} ({len(imgs)} frames)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
