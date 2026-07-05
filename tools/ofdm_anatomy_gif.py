#!/usr/bin/env python3
"""Animated OFDM channel anatomy — 'what a subcarrier / a channel actually is',
in the DEVOURER live-monitor style.

    tools/ofdm_anatomy_gif.py -o docs/img/ofdm_anatomy.gif

A 20 MHz channel is 64 subcarriers 312.5 kHz apart: a DC null in the middle,
pilot tones the receiver tracks, data tones that carry the bits, and guard bins
at the edges. A cursor sweeps across naming each region; a side ticker cycles the
bandwidth options — 20/40/80 MHz add more 312.5 kHz tones, while the 5/10 MHz
narrowband re-clocks to *closer* spacing to fit a thin channel. This is the
coordinate system the per-subcarrier waterfall, the NHM buckets, and the tone
mask all live in. Needs Pillow.
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, GRID, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

# 802.11ac 20 MHz: 64 FFT bins, indices -32..+31. Occupied -28..+28 (56 tones:
# 52 data + 4 pilot), pilots at ±7 / ±21, DC null at 0, guard beyond ±28.
PILOTS = {7, 21}


def region(k):
    if k == 0:
        return "dc"
    if abs(k) > 28:
        return "guard"
    if abs(k) in PILOTS:
        return "pilot"
    return "data"


# bandwidth ticker: (label, FFT, data tones, spacing kHz, note)
BWS = [
    ("20 MHz", 64, 52, 312.5, "the baseline"),
    ("40 MHz", 128, 108, 312.5, "2× the tones"),
    ("80 MHz", 256, 234, 312.5, "4× the tones"),
    ("10 MHz", 64, 52, 156.25, "re-clocked narrower"),
    ("5 MHz", 64, 52, 78.125, "narrowest — most robust"),
]

COL = {"data": CYAN, "pilot": AMBER, "guard": (70, 84, 100), "dc": WARN}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="ofdm_anatomy.gif")
    ap.add_argument("--ms", type=int, default=110)
    args = ap.parse_args()

    padL, padT, padB = 34, 96, 128
    grid_w, grid_h = 560, 260
    panelW = 210
    W = padL + grid_w + 28 + panelW
    H = padT + grid_h + padB
    N = 64
    binw = grid_w / N
    base_y = padT + grid_h

    imgs = []
    frames = N + 6            # sweep across all bins, then a short hold
    for fi in range(frames):
        cur_k = min(fi, N - 1) - 32      # cursor index -32..+31
        img, d = new_frame(W, H)
        chrome(d, W, H, "OFDM CHANNEL ANATOMY",
               "a 20 MHz channel = 64 subcarriers · 312.5 kHz apart · "
               "the grid the per-tone sensors live in", fi)

        # subcarrier comb
        heights = {"data": 0.82, "pilot": 1.0, "guard": 0.16, "dc": 0.0}
        for i in range(N):
            k = i - 32
            reg = region(k)
            x = padL + i * binw
            h = int(heights[reg] * grid_h)
            col = COL[reg]
            if reg == "dc":
                d.line([x + binw / 2, base_y - 8, x + binw / 2, base_y], fill=col)
            else:
                d.rectangle([x + 1, base_y - h, x + binw - 1, base_y], fill=col)
            if k == cur_k:            # cursor highlight
                d.rectangle([x, padT, x + binw, base_y], outline=CYAN, width=1)
        d.rectangle([padL, padT, padL + grid_w, base_y], outline=(0, 70, 80))
        # centre / DC marker
        d.text((padL + grid_w / 2 - 14, base_y + 6), "DC", font=font(11), fill=WARN)
        # frequency axis
        for k in (-28, -21, -14, -7, 7, 14, 21, 28):
            x = padL + (k + 32) * binw
            d.text((x - 12, base_y + 22), f"{k*312.5/1000:+.1f}",
                   font=font(11), fill=DIM)
        d.text((padL, base_y + 40), "frequency offset from centre (MHz)  ·  "
               "each bar = one 312.5 kHz subcarrier", font=font(11), fill=DIM)

        # cursor callout
        reg = region(cur_k)
        names = {"data": "DATA subcarrier — carries the coded bits",
                 "pilot": "PILOT — a known tone; tracks phase/frequency drift",
                 "guard": "GUARD — empty edge bin (spectral mask / roll-off)",
                 "dc": "DC NULL — the centre bin is left empty (LO leakage)"}
        cc = COL[reg]
        d.text((padL, padT - 26), f"tone {cur_k:+d}  ·  "
               f"{cur_k*312.5/1000:+.3f} MHz", font=font(12, True), fill=cc)
        d.text((padL + 220, padT - 25), names[reg], font=font(11), fill=INK)

        # legend (a clear row below the frequency caption)
        lx, ly = padL, base_y + 64
        for reg2, lbl in (("data", "data ×52"), ("pilot", "pilot ×4"),
                          ("dc", "DC null"), ("guard", "guard")):
            d.rectangle([lx, ly, lx + 14, ly + 11], fill=COL[reg2])
            d.text((lx + 20, ly - 1), lbl, font=font(11), fill=INK)
            lx += 150

        # bandwidth ticker (cycles ~ every 12 frames)
        x0 = padL + grid_w + 26
        d.text((x0, padT - 2), "BANDWIDTH", font=font(12), fill=CYAN)
        bi = (fi // 8) % len(BWS)
        y = padT + 22
        for j, (lbl, fft, dat, sp, note) in enumerate(BWS):
            on = j == bi
            d.text((x0, y), ("▸ " if on else "  ") + lbl,
                   font=font(13, True), fill=CYAN if on else DIM)
            if on:
                d.text((x0 + 14, y + 20),
                       f"{fft}-pt FFT · {dat} data", font=font(11), fill=INK)
                d.text((x0 + 14, y + 36),
                       f"{sp:.4g} kHz spacing", font=font(11), fill=INK)
                d.text((x0 + 14, y + 52), note, font=font(11), fill=OK)
                y += 52
            y += 24
        imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms)
    return 0


if __name__ == "__main__":
    sys.exit(main())
