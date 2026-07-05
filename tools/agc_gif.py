#!/usr/bin/env python3
"""Animated AGC monitor — 'why a strong signal makes the receiver go deaf', in
the DEVOURER live-monitor style.

    tools/agc_gif.py -o docs/img/agc_saturation.gif

The receiver's automatic gain control (AGC) turns its gain down as the incoming
signal gets stronger, to keep the ADC in its sweet spot. But the gain has a
floor: when a signal is *too* strong — a co-located carrier inches away — the AGC
runs out of attenuation, the ADC input exceeds full scale, and the waveform clips
flat against the rails. A clipped waveform can't be demodulated: the RX goes
deaf. That is the 'collapse' the frame-free energy sensor sees when a strong
interferer arrives (frames and CCA fall toward zero) — the flip side of the
'spike' a moderate one causes. Needs Pillow.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, GRID, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

TARGET = 0.42     # ADC level the AGC aims for (fraction of full scale)
GMIN, GMAX = 0.16, 6.0
NS = 220          # waveform samples across the scope


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="agc_saturation.gif")
    ap.add_argument("--frames", type=int, default=64)
    ap.add_argument("--ms", type=int, default=85)
    args = ap.parse_args()

    padL, padT, padB = 34, 92, 46
    scope_w, scope_h = 500, 300
    panelW = 300
    W = padL + scope_w + 24 + panelW
    H = padT + scope_h + padB
    cy = padT + scope_h // 2
    rail = scope_h // 2 - 10          # full-scale rail offset from centre

    imgs = []
    for fi in range(args.frames):
        # interferer approaches and recedes: input level (full-scale units at
        # unity gain) sweeps low -> very high -> low.
        u = fi / args.frames
        env = 0.5 - 0.5 * math.cos(2 * math.pi * u)          # 0..1..0
        inp = 0.05 * math.exp(env * 5.3)                     # 0.05 .. ~10 (peak clips)
        gain = min(GMAX, max(GMIN, TARGET / inp))            # AGC decision
        adc = inp * gain                                     # ADC input level
        clipping = adc > 1.0
        floored = gain <= GMIN + 1e-3

        img, d = new_frame(W, H)
        chrome(d, W, H, "AGC / ADC MONITOR",
               "the receiver turns gain down as signal rises — until the gain "
               "floor, where the ADC clips", fi)

        # scope box + rails
        d.rectangle([padL, padT, padL + scope_w, padT + scope_h],
                    outline=(0, 70, 80))
        for sgn in (1, -1):
            yr = cy - sgn * rail
            d.line([padL, yr, padL + scope_w, yr], fill=(90, 40, 40))
        d.text((padL + 6, cy - rail - 14), "+full scale", font=font(10),
               fill=(150, 80, 80))
        d.text((padL + 6, cy + rail + 2), "−full scale", font=font(10),
               fill=(150, 80, 80))
        # saturation zones (above/below rails) tinted when clipping
        if clipping:
            d.rectangle([padL + 1, padT + 1, padL + scope_w - 1, cy - rail],
                        fill=(40, 12, 12))
            d.rectangle([padL + 1, cy + rail, padL + scope_w - 1,
                         padT + scope_h - 1], fill=(40, 12, 12))
        # the ADC waveform (sine at `adc` amplitude, clipped at the rails)
        pts = []
        for i in range(NS):
            ph = i / NS * 6 * math.pi + fi * 0.35
            s = math.sin(ph) * adc
            s = max(-1.0, min(1.0, s))                       # clip
            x = padL + int(i / (NS - 1) * scope_w)
            pts.append((x, cy - int(s * rail)))
        wcol = WARN if clipping else OK
        d.line(pts, fill=wcol, width=2)

        # readout panel
        x0 = padL + scope_w + 22
        d.text((x0, padT - 2), "LIVE READOUT", font=font(12), fill=CYAN)
        y = padT + 22

        def bar(lbl, frac, col, txt):
            nonlocal y
            d.text((x0, y), lbl, font=font(11), fill=DIM)
            d.text((x0 + 150, y - 1), txt, font=font(13, True), fill=col)
            y += 16
            bw = 260
            d.rectangle([x0, y, x0 + bw, y + 12], outline=(40, 52, 68))
            d.rectangle([x0, y, x0 + int(bw * max(0, min(1, frac))), y + 12],
                        fill=col)
            y += 26

        in_db = 20 * math.log10(inp / 0.05)                  # relative dB
        g_db = 20 * math.log10(gain)
        bar("input signal", inp / 10.5, AMBER if inp > 1 else CYAN,
            f"{in_db:+4.0f} dB")
        bar("AGC gain", (gain - GMIN) / (GMAX - GMIN),
            WARN if floored else OK, f"{g_db:+4.0f} dB")
        bar("ADC level", min(1.0, adc), WARN if clipping else OK,
            "CLIP" if clipping else f"{int(adc*100):d} %")

        # status pill
        if clipping:
            st, sc, sub = "SATURATED — RX DEAF", WARN, "energy sensor sees a COLLAPSE"
        elif floored:
            st, sc, sub = "GAIN FLOORED", AMBER, "no headroom left"
        else:
            st, sc, sub = "LOCKED", OK, "ADC in its sweet spot"
        y += 6
        d.rectangle([x0, y, x0 + 286, y + 32], outline=sc, width=2)
        d.ellipse([x0 + 9, y + 11, x0 + 20, y + 22], fill=sc)
        d.text((x0 + 30, y + 8), st, font=font(14, True), fill=sc)
        y += 40
        d.text((x0, y), sub, font=font(11), fill=INK)

        d.text((padL, padT + scope_h + 20),
               "a moderate interferer → CCA spike;  a strong one → AGC floors, "
               "RX collapses", font=font(11), fill=DIM)
        imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms)
    return 0


if __name__ == "__main__":
    sys.exit(main())
