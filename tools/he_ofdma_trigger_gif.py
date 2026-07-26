#!/usr/bin/env python3
"""Animated 802.11ax trigger-based uplink — 'the AP says who talks, and where',
in the DEVOURER live-monitor style.

    tools/he_ofdma_trigger_gif.py -o docs/img/he_ofdma_trigger.gif

Two lanes sharing one time axis. The top lane is the standard: an AP sends a
Trigger frame naming, per station, a resource unit and an MCS; exactly one SIFS
later those stations answer simultaneously, each inside its own slice of the
channel's subcarriers. The bottom lane is what a userspace driver on shipped
client firmware actually achieves: the Trigger goes out and decodes with the
exact commanded parameters — and nothing answers, because a host-injected frame
does not arm the MAC's SIFS-scheduled receive window, so the station never gets
its hardware timing cue.

That gap is the teaching content, not a footnote: it is the line between putting
a correct frame on the air and owning the hardware schedule. Needs Pillow.
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, GRID, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

RUS = ["52-tone", "52-tone", "52-tone", "52-tone"]
GRANTS = [("UE-A", "MCS4"), ("UE-B", "MCS2"), ("UE-C", "MCS6"), (None, None)]

T_TRIG = (0.02, 0.34)    # the Trigger frame, full bandwidth, legacy OFDM
T_SIFS = (0.34, 0.42)    # the fixed ~16 us the standard allows for the answer
T_RESP = (0.42, 0.86)    # the trigger-based PPDU, or the silence where it isn't


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="he_ofdma_trigger.gif")
    ap.add_argument("--frames", type=int, default=52)
    ap.add_argument("--hold", type=int, default=18)
    ap.add_argument("--ms", type=int, default=110)
    args = ap.parse_args()

    nR = len(RUS)
    padL, padT = 96, 96
    gw, rh = 600, 25
    laneH = nR * rh
    gapH = 54
    panelW = 236
    W = padL + gw + 24 + panelW
    H = padT + 20 + laneH + gapH + 20 + laneH + 60
    yA = padT + 20
    yB = yA + laneH + gapH + 20

    def xt(f):
        return padL + f * gw

    def blk(d, y0, t0, t1, col, cut, label=None, dashed=False):
        """One time-frequency block, revealed only left of the playhead."""
        x0, x1 = xt(t0), min(xt(t1), cut)
        if x1 <= x0:
            return
        if dashed:
            d.rectangle([x0, y0 + 3, x1, y0 + rh - 3], outline=col)
            step = 7
            hx = x0 + 3
            while hx < x1 - 2:
                d.line([hx, y0 + rh / 2, min(hx + 3, x1 - 2), y0 + rh / 2], fill=col)
                hx += step
        else:
            d.rectangle([x0 + 1, y0 + 3, x1, y0 + rh - 3], fill=col)
        if label and x1 - x0 > 46:
            d.text((x0 + 7, y0 + rh / 2 - 7), label, font=font(11),
                   fill=(8, 11, 18) if not dashed else col)

    imgs = []
    for fi in range(args.frames + args.hold):
        prog = min(1.0, (fi + 1) / args.frames)
        cut = xt(prog)
        img, d = new_frame(W, H)
        chrome(d, W, H, "802.11ax — TRIGGER-BASED UPLINK",
               "one 20 MHz channel, 242 subcarriers, sliced into resource units "
               "— the AP grants each station one slice and an MCS", fi)

        for lane, (y, title, tcol) in enumerate((
                (yA, "THE STANDARD — a compliant station answers at trigger + SIFS", OK),
                (yB, "HOST-INJECTED, SHIPPED CLIENT FIRMWARE — what actually airs", AMBER))):
            d.text((padL, y - 17), title, font=font(11), fill=tcol)
            d.rectangle([padL, y, padL + gw, y + laneH], outline=(0, 70, 80))
            for i, ru in enumerate(RUS):
                ry = y + i * rh
                d.line([padL, ry, padL + gw, ry], fill=GRID)
                if lane == 0:
                    d.text((padL - 74, ry + rh / 2 - 7), ru, font=font(10), fill=DIM)

            # the Trigger itself — full bandwidth, so it spans every RU row
            x0, x1 = xt(T_TRIG[0]), min(xt(T_TRIG[1]), cut)
            if x1 > x0:
                d.rectangle([x0 + 1, y + 3, x1, y + laneH - 3], fill=(120, 100, 32))
                if x1 - x0 > 90:
                    d.text((x0 + 10, y + laneH / 2 - 14), "AP TRIGGER",
                           font=font(12, True), fill=(8, 11, 18))
                    d.text((x0 + 10, y + laneH / 2 + 2),
                           "who / which RU / what MCS", font=font(10),
                           fill=(8, 11, 18))

            # the SIFS the answer is owed
            if cut > xt(T_SIFS[0]):
                sx = xt((T_SIFS[0] + T_SIFS[1]) / 2)
                yy = y
                while yy < y + laneH:
                    d.line([sx, yy, sx, yy + 4], fill=DIM)
                    yy += 8
                if lane == 0:
                    d.text((sx - 16, y + laneH + 4), "SIFS", font=font(10), fill=DIM)

            # the response — or the silence where the standard puts one
            for i, (who, mcs) in enumerate(GRANTS):
                ry = y + i * rh
                if who is None:   # a slice the AP granted to nobody this round
                    if lane == 0 and cut > xt(T_RESP[0] + 0.06):
                        d.text((xt(T_RESP[0]) + 7, ry + rh / 2 - 7), "ungranted",
                               font=font(10), fill=(70, 84, 104))
                    continue
                if lane == 0:
                    blk(d, ry, T_RESP[0], T_RESP[1], CYAN, cut, f"{who}  {mcs}")
                else:
                    blk(d, ry, T_RESP[0], T_RESP[1], (70, 84, 104), cut,
                        None, dashed=True)
            if lane == 0 and cut > xt(T_RESP[0] + 0.06):
                d.text((xt(T_RESP[0]) + 6, y + laneH + 4),
                       "all three at once — one PPDU, three talkers",
                       font=font(10), fill=OK)
            if lane == 1 and cut > xt(T_RESP[0] + 0.06):
                d.text((xt(T_RESP[0]) + 6, y + laneH + 4),
                       "no TB PPDU — the SIFS receive window was never armed",
                       font=font(10), fill=WARN)

        # the playhead
        if prog < 1.0:
            d.line([cut, yA - 6, cut, yB + laneH + 6], fill=(60, 90, 110))

        # readout
        x0 = padL + gw + 22
        d.text((x0, padT - 2), "LIVE READOUT", font=font(12), fill=CYAN)
        y = padT + 24

        def line(lbl, val, c=INK):
            nonlocal y
            d.text((x0, y), lbl, font=font(11), fill=DIM)
            d.text((x0 + 104, y - 3), val, font=font(14, True), fill=c)
            y += 28

        aired = prog > T_TRIG[1]
        answered = prog > T_RESP[0] + 0.06
        line("trigger", "ON AIR" if aired else "…", OK if aired else DIM)
        line("decoded as", "EXACT" if aired else "…", OK if aired else DIM)
        line("granted", "3 users" if aired else "…", INK if aired else DIM)
        line("TB PPDU", "NONE" if answered else "…", WARN if answered else DIM)
        y += 4

        if not aired:
            cap = ("802.11ax moves uplink", "scheduling into the MAC.",
                   "the AP names who talks,", "in which RU, at what MCS.")
        elif not answered:
            cap = ("the frame airs on the", "ordinary management path,",
                   "so any monitor decodes", "it — AID, RU, MCS, NSS", "all exact.")
        else:
            cap = ("but the answer is", "hardware-timed. injecting",
                   "the trigger from the host", "never arms the MAC's SIFS",
                   "receive window, so no", "station is cued to reply.")
        for ln in cap:
            d.text((x0, y), ln, font=font(11), fill=INK)
            y += 15

        imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms, colors=48)
    return 0


if __name__ == "__main__":
    sys.exit(main())
