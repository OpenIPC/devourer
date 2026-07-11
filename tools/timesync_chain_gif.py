#!/usr/bin/env python3
"""Animated "one clock for many radios" chain for docs/rf-primer.md — the
time-distribution machinery as a four-act loop in the DEVOURER monitor style.

Three lanes, each a clock rendered as a tick comb (one tick per beacon
interval) plus a live counter:

  GRANDMASTER  the wired IEEE-1588 reference (Intel I226 PHC) — the fixed grid
  AP (master)  the Wi-Fi chip's TSF on the PCIe AP (8821CE)
  STATION      a receiver across the air, syncing from beacons alone

Acts (cycled by the loop):
  1  FREE-RUNNING   crystals drift: the AP and station combs slide off the grid
  2  PTP (wire)     hardware-timestamped sync pulses discipline the AP clock
  3  BEACONS (air)  the MAC stamps the live TSF into each beacon AT the antenna;
                    the station's fit pulls its comb onto the AP's — a ghosted
                    software-stamped alternative scatters to show why hardware
                    egress stamping matters
  4  TBTT PIN       the beacon *schedule* snaps onto the disciplined clock
                    (PinBeaconTbtt) while the clock trace runs unbroken

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

W, H = 920, 548
LANES = {  # y-band per lane
    "gm": (86, 196),
    "ap": (216, 326),
    "sta": (346, 456),
}
LX0, LX1 = 210, 880          # comb x-span
TICK_PX = 74                 # px per beacon interval on the comb
ACTS = [(0, 22), (22, 44), (44, 78), (78, 104)]   # frame ranges per act
TOTAL = ACTS[-1][1]


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


def comb(d, key, offset_px, col, jitter=0.0, flags=None, flag_col=None):
    """Tick comb: one tick per interval, slid by offset_px. Optional TBTT
    flags drawn at tick+flag offset."""
    y0, y1 = LANES[key]
    cy0, cy1 = y1 - 42, y1 - 12
    for k in range(-1, int((LX1 - LX0) / TICK_PX) + 2):
        x = LX0 + k * TICK_PX + (offset_px % TICK_PX)
        if not (LX0 - 2 <= x <= LX1 + 2):
            continue
        j = random.uniform(-jitter, jitter)
        d.line([x + j, cy0, x + j, cy1], fill=col, width=2)
        if flags is not None:
            fx = x + flags
            if LX0 - 2 <= fx <= LX1 + 2:
                d.polygon([(fx, cy0 - 2), (fx + 9, cy0 - 8), (fx, cy0 - 14)],
                          fill=flag_col or AMBER)


def counter(d, key, us, col):
    y0, _ = LANES[key]
    d.text((W - 250, y0 + 10), f"{us/1e6:12.6f} s", font=font(15, True), fill=col)


def pulse(d, x, y_from, y_to, t, col, label=None):
    """A sync/beacon pulse travelling between lanes (t in 0..1)."""
    y = y_from + (y_to - y_from) * t
    d.ellipse([x - 4, y - 4, x + 4, y + 4], fill=col)
    d.line([x, y_from, x, y], fill=col, width=1)
    if label and t < 0.5:
        d.text((x + 8, y - 6), label, font=font(10), fill=col)


def main() -> int:
    ap_ = argparse.ArgumentParser(description=__doc__)
    ap_.add_argument("-o", "--out", default="timesync_chain.gif")
    ap_.add_argument("--ms", type=int, default=110)
    args = ap_.parse_args()

    rng = random.Random(7)
    imgs = []
    # drift rates in px/frame (visualized ppm)
    ap_drift, sta_drift = 1.35, -1.05
    ap_off = 0.0     # AP comb offset vs the GM grid (px; 0 = locked)
    sta_off = 26.0   # station comb offset vs the AP comb
    tbtt_flag = 30.0  # beacon-schedule offset vs the AP comb (act 4 pins it to 0)
    resid_hist = []

    captions = [
        ("1/4  FREE-RUNNING", "every crystal keeps its own time — ±20 ppm apart, "
                              "they drift ~2 ms every 100 s", WARN),
        ("2/4  PTP OVER THE WIRE", "hardware timestamps at the Ethernet PHY "
                                   "discipline the AP clock to ~300 ns", CYAN),
        ("3/4  BEACONS OVER THE AIR", "the Wi-Fi MAC stamps the live TSF into each "
                                      "beacon AT the antenna — the station locks to "
                                      "~0.3 µs (software stamps: the grey scatter)", OK),
        ("4/4  THE TBTT PIN", "PinBeaconTbtt snaps the beacon schedule onto the "
                              "disciplined clock — the clock trace never breaks", AMBER),
    ]

    for fr in range(TOTAL):
        act, t = act_of(fr)
        im, d = new_frame(W, H)
        chrome(d, W, H, "TIME DISTRIBUTION — ONE CLOCK FOR MANY RADIOS", "", fr)

        # --- lane frames ----------------------------------------------------
        lane_box(d, "gm", "GRANDMASTER", "Intel I226 PHC (IEEE 1588, wired)", INK)
        lane_box(d, "ap", "AP / MASTER", "8821CE Wi-Fi TSF (PCIe, monitor mode)", CYAN)
        lane_box(d, "sta", "STATION", "any devourer RX — beacons only, no wire", OK)

        # --- clock dynamics per act -----------------------------------------
        if act == 0:
            ap_off += ap_drift
            sta_off += sta_drift
        elif act == 1:
            # PTP servo: pull AP offset to 0 exponentially; station still drifts
            ap_off *= 0.82
            sta_off += sta_drift * 0.7
        elif act == 2:
            ap_off *= 0.9  # held by the servo
            sta_off *= 0.80  # beacon fit pulls the station in
        else:
            ap_off *= 0.9
            sta_off *= 0.9
            tbtt_flag *= 0.78  # the pin walks the schedule onto the grid

        # --- combs ------------------------------------------------------------
        comb(d, "gm", 0, INK)
        comb(d, "ap", ap_off, CYAN,
             flags=tbtt_flag if act >= 2 else None, flag_col=AMBER)
        comb(d, "sta", ap_off + sta_off, OK,
             jitter=0.6 if act >= 2 else 0.0)

        # --- counters (µs, exaggerated offsets for display) -------------------
        base = 1_000_000 + fr * 102_400
        counter(d, "gm", base, INK)
        counter(d, "ap", base + ap_off * 320, CYAN)
        counter(d, "sta", base + (ap_off + sta_off) * 320, OK)

        # --- pulses ------------------------------------------------------------
        if act == 1 and (fr % 5) in (0, 1, 2):
            x = LX0 + 120
            pulse(d, x, LANES["gm"][1] - 26, LANES["ap"][0] + 26,
                  ((fr % 5) + t) / 3.0, CYAN, "sync (hw ts)")
        if act >= 2:
            ph = (fr % 4) / 3.0
            x = LX0 + 320 + (fr // 4 % 3) * 110
            pulse(d, x, LANES["ap"][1] - 26, LANES["sta"][0] + 26, ph, OK,
                  "beacon+TSF")
            # ghost: where a software-stamped beacon would land (scattered)
            gx = x + rng.uniform(-26, 26)
            gy = LANES["sta"][0] + 26 + rng.uniform(-3, 9)
            d.ellipse([gx - 3, gy - 3, gx + 3, gy + 3], outline=DIM)

        # --- act-4 pin marker ---------------------------------------------------
        if act == 3 and t < 0.35:
            y0 = LANES["ap"][0]
            d.text((LX0 + 8, y0 + 10), "PIN", font=font(14, True), fill=AMBER)
            d.text((LX0 + 46, y0 + 12),
                   "(TSF-preserving: schedule moves, clock doesn't)",
                   font=font(10), fill=AMBER)

        # --- residual sparkline ---------------------------------------------------
        resid = abs(ap_off + sta_off) * 12 + rng.uniform(0, 1.5)  # display µs
        resid_hist.append(min(resid, 400))
        y0, y1 = 470, 520
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

        # --- caption (on the chrome's subtitle line) --------------------------------
        cap, sub, col = captions[act]
        d.text((28, 54), cap, font=font(14, True), fill=col)
        d.text((248, 57), sub, font=font(11), fill=DIM)

        imgs.append(im)

    save_gif(imgs, args.out, ms=args.ms)
    print(f"wrote {args.out} ({len(imgs)} frames)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
