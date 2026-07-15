#!/usr/bin/env python3
"""Animated DIG tracking loop — 'BB DM, AGC and the initial-gain staircase',
in the DEVOURER live-monitor style.

    tools/dig_loop_gif.py -o docs/img/dig_loop.gif

A scrolling time series: the noise floor drifts, then a wideband interferer
arrives and leaves. The DIG staircase (the AGC's starting gain, the IGI value)
tracks it — but only at the ~2 s watchdog cadence and with hysteresis, so it
lags. While the floor sits above the threshold the false-alarm counter strip
burns red (the receiver triggers on noise); when the floor drops and DIG is
still parked high, a weak frame slips past undetected. Side gauges show the
live IGI readout and CCA busy %. This is phydm's original job description.
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

FS = 10                 # samples per second
TICK = 2 * FS           # watchdog tick every ~2 s
MARGIN = 6              # DIG aims IGI threshold ~6 dB above the floor


def build_timeline(rnd, T):
    """Per-sample (floor dBm, igi-threshold dBm, false-alarm rate 0..1)."""
    floor = []
    for i in range(T):
        base = -62.0 + 1.5 * math.sin(i / 37.0)          # slow drift
        if 140 <= i < 225:                                # interferer on
            k = min(1.0, (i - 140) / 5.0)                 # sharp rise
            base += 14.0 * k
        elif 225 <= i < 231:                              # sharp fall
            base += 14.0 * (1.0 - (i - 225) / 6.0)
        floor.append(base + rnd.uniform(-0.7, 0.7))

    igi, thr = [], 0x2A                                   # IGI in hw units
    for i in range(T):
        if i % TICK == 0 and i > 0:
            target = floor[i] + MARGIN                    # wanted threshold
            cur = -96 + thr                               # IGI unit -> dBm
            if target - cur > 3:                          # hysteresis band
                thr = min(0x3E, thr + 4)                  # step up fast
            elif cur - target > 8:
                thr = max(0x1C, thr - 4)                  # step down slower
            elif cur - target > 5:
                thr = max(0x1C, thr - 2)
        igi.append(thr)

    fa = []
    for i in range(T):
        gap = floor[i] - (-96 + igi[i] - 4)               # floor vs threshold
        v = max(0.0, min(1.0, gap / 8.0 + 0.06))
        fa.append(v + rnd.uniform(0, 0.05))
    return floor, igi, fa


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="dig_loop.gif")
    ap.add_argument("--ms", type=int, default=105)
    ap.add_argument("--frames", type=int, default=100)
    args = ap.parse_args()
    rnd = random.Random(0xD16)

    N, STEP = 120, 2                          # window samples, samples/frame
    T = N + args.frames * STEP
    floor, igi, fa = build_timeline(rnd, T)
    # weak-frame events: (sample, rssi dBm) — arrive just after the interferer
    weak = [(245, -52.0), (305, -52.5)]

    W, H = 900, 500
    padL = 34
    pX, pY, pW, pH = padL, 110, 620, 200      # main dB panel
    fX, fY, fW, fH = padL, 368, 620, 56       # false-alarm strip
    gX = 690                                   # gauges column
    DBTOP, DBBOT = -38.0, -72.0

    def ydb(v):
        v = max(DBBOT, min(DBTOP, v))
        return pY + (DBTOP - v) / (DBTOP - DBBOT) * pH

    imgs = []
    for fi in range(args.frames):
        s0 = fi * STEP                        # window = samples [s0, s0+N)
        img, d = new_frame(W, H)
        chrome(d, W, H, "DIG TRACKING LOOP",
               "BB dynamic mechanisms: the ~2 s watchdog steps the AGC's "
               "initial gain (IGI) after the noise floor", fi)

        # ---- main panel: floor trace + DIG staircase ----------------------
        d.rectangle([pX, pY, pX + pW, pY + pH], outline=(0, 70, 80))
        for db in (-40, -50, -60, -70):
            y = ydb(db)
            d.line([pX + 1, y, pX + pW - 1, y], fill=GRID)
            d.text((pX + pW + 6, y - 6), f"{db}", font=font(10), fill=DIM)
        d.text((pX, pY - 16), "noise floor (dBm) vs DIG threshold",
               font=font(11), fill=DIM)

        def sx(i):                            # sample index -> panel x
            return pX + (i - s0) / (N - 1) * pW

        fpts = [(sx(i), ydb(floor[i])) for i in range(s0, s0 + N)]
        d.line(fpts, fill=INK, width=1)
        tpts = [(sx(i), ydb(-96 + igi[i] - 4)) for i in range(s0, s0 + N)]
        d.line(tpts, fill=AMBER, width=2)
        # legend (top-left of panel, kept clear of traces)
        d.line([pX + 10, pY + 12, pX + 30, pY + 12], fill=INK)
        d.text((pX + 36, pY + 6), "noise floor", font=font(10), fill=DIM)
        d.line([pX + 130, pY + 12, pX + 150, pY + 12], fill=AMBER, width=2)
        d.text((pX + 156, pY + 6), "DIG threshold (IGI)", font=font(10),
               fill=AMBER)

        # watchdog tick markers on the time axis
        for i in range(s0 - s0 % TICK, s0 + N, TICK):
            if i < s0:
                continue
            x = sx(i)
            d.polygon([(x, pY + pH + 4), (x - 4, pY + pH + 12),
                       (x + 4, pY + pH + 12)], fill=CYAN)
        d.text((pX, pY + pH + 16), "▲ watchdog tick (~2 s) — DIG only steps "
               "here", font=font(10), fill=DIM)

        # weak-frame events: caught if rssi clears the DIG threshold
        for wj, (wi, rssi) in enumerate(weak):
            if not (s0 <= wi < s0 + N):
                continue
            x, y = sx(wi), ydb(rssi)
            thr_dbm = -96 + igi[wi] - 4
            missed = rssi < thr_dbm
            col = WARN if missed else OK
            d.line([x, y - 6, x, y + 6], fill=col, width=2)
            d.ellipse([x - 3, y - 3, x + 3, y + 3], outline=col, width=2)
            lbl = "weak frame MISSED" if missed else "weak frame caught"
            lx = max(pX + 6, min(x - 60, pX + pW - 140))
            ly = y - 24 if wj % 2 == 0 else y + 14
            d.text((lx, ly), lbl, font=font(10, True), fill=col)

        # ---- false-alarm strip --------------------------------------------
        d.rectangle([fX, fY, fX + fW, fY + fH], outline=(0, 70, 80))
        d.text((fX, fY - 16), "false-alarm counter (CCA triggers on noise)",
               font=font(11), fill=DIM)
        for i in range(s0, s0 + N, 2):
            v = fa[i]
            x = sx(i)
            bh = int(v * (fH - 8))
            col = WARN if v > 0.45 else ((200, 140, 60) if v > 0.22 else
                                         (40, 90, 70))
            d.rectangle([x, fY + fH - 4 - bh, x + 3, fY + fH - 4], fill=col)

        # ---- side gauges ---------------------------------------------------
        cur = s0 + N - 1
        d.text((gX, pY - 16), "LIVE READOUT", font=font(12), fill=CYAN)
        y = pY + 8

        def gauge(lbl, val, col=INK):
            nonlocal y
            d.text((gX, y), lbl, font=font(11), fill=DIM)
            d.text((gX, y + 14), val, font=font(18, True), fill=col)
            y += 52

        favg = sum(fa[cur - 6:cur + 1]) / 7
        cca = min(96, int(8 + 80 * max(0.0, (floor[cur] + 63) / 16)))
        gauge("IGI (initial gain)", f"0x{igi[cur]:02X}", AMBER)
        gauge("noise floor", f"{floor[cur]:5.1f} dBm")
        gauge("false alarms", f"{int(favg * 800):4d}/s",
              WARN if favg > 0.45 else (AMBER if favg > 0.22 else OK))
        gauge("CCA busy", f"{cca:3d} %",
              WARN if cca > 60 else (AMBER if cca > 30 else OK))
        # status pill
        missed_now = any(s0 <= wi < s0 + N and rs < (-96 + igi[wi] - 4)
                         for wi, rs in weak)
        if favg > 0.45:
            st, sc = "FALSE-ALARM STORM", WARN
        elif missed_now and floor[cur] < -58 and igi[cur] > 0x30:
            st, sc = "DEAF: IGI HIGH", AMBER
        else:
            st, sc = "TRACKING", OK
        d.rectangle([gX, y, gX + 176, y + 28], outline=sc, width=2)
        d.text((gX + 10, y + 6), st, font=font(12, True), fill=sc)

        # ---- caption ---------------------------------------------------------
        t_now = cur
        if t_now < 142:
            cap = "quiet band — DIG parks the initial gain just above the floor"
        elif t_now < 185:
            cap = "interferer raises the floor — false alarms burn until the " \
                  "watchdog steps DIG up"
        elif t_now < 233:
            cap = "DIG stepped up: false alarms settle — bought by giving up " \
                  "sensitivity"
        elif t_now < 302:
            cap = "interferer gone, but DIG lags high — a weak frame slips past"
        else:
            cap = "DIG watches noise + false alarms, and steps the AGC's " \
                  "starting gain"
        d.text((padL, H - 34), cap, font=font(13, True), fill=CYAN)
        imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms, colors=56)
    return 0


if __name__ == "__main__":
    sys.exit(main())
