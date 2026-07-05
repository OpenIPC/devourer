#!/usr/bin/env python3
"""Animated IQ-constellation monitor — 'what MCS actually is, and why SNR
matters', in the DEVOURER live-monitor style.

    tools/constellation_gif.py -o docs/img/constellation.gif

A fixed link SNR (slowly wobbling, like a real fading channel) while the
modulation steps up QPSK -> 16-QAM -> 64-QAM -> 256-QAM. Each received symbol is
an ideal constellation point plus Gaussian noise sized by the SNR (textbook AWGN,
EVM = 1/sqrt(SNR)); a dot is green if it still lands in the right decision cell,
red if the noise pushed it across a boundary into a bit error. The lesson: on the
*same* channel, low-order modulation has margin to spare while high-order packs
the points so tight the same noise breaks it — exactly the boundary the
MCS-headroom probe measures. Needs Pillow.
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

# (name, M, required-SNR dB for ~1e-3 uncoded BER)
MODS = [("QPSK", 4, 9.0), ("16-QAM", 16, 16.0),
        ("64-QAM", 64, 22.0), ("256-QAM", 256, 28.0)]


def constellation(M):
    """Unit-average-power square M-QAM points."""
    s = int(round(math.sqrt(M)))
    pts = [(2 * i - (s - 1), 2 * q - (s - 1)) for i in range(s) for q in range(s)]
    norm = math.sqrt(sum(x * x + y * y for x, y in pts) / len(pts))
    return [(x / norm, y / norm) for x, y in pts], s, norm


def nearest_idx(pt, s, norm):
    """Index of the nearest constellation point (grid quantise)."""
    def q(v):
        i = round((v * norm + (s - 1)) / 2)
        return max(0, min(s - 1, i))
    return q(pt[0]) * s + q(pt[1])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="constellation.gif")
    ap.add_argument("--per-mod", type=int, default=16)
    ap.add_argument("--pts", type=int, default=520)
    ap.add_argument("--ms", type=int, default=95)
    ap.add_argument("--snr", type=float, default=24.0, help="mean link SNR dB")
    args = ap.parse_args()
    rnd = random.Random(0xC0)

    padL, padT, padB = 34, 92, 40
    grid = 388
    panelW = 250
    W = padL + grid + 30 + panelW
    H = padT + grid + padB
    cx, cy = padL + grid // 2, padT + grid // 2
    R = grid // 2 - 8               # IQ plane radius (|point| ~ 1 -> 0.7*R-ish)
    scale = R / 1.45

    imgs = []
    nseg = len(MODS)
    total = nseg * args.per_mod
    for fi in range(total):
        seg = fi // args.per_mod
        name, M, req = MODS[seg]
        pts, s, norm = constellation(M)
        # link SNR wobbles like a fading channel
        snr_db = args.snr + 3.0 * math.sin(fi / 5.0)
        snr_lin = 10 ** (snr_db / 10)
        sigma = math.sqrt(1.0 / (2 * snr_lin))     # per-component AWGN
        evm = 100.0 / math.sqrt(snr_lin)
        margin = snr_db - req

        img, d = new_frame(W, H)
        chrome(d, W, H, "CONSTELLATION MONITOR",
               f"AWGN model · EVM = 1/√SNR · a dot crossing a cell boundary "
               f"= one bit error", fi)

        # IQ plane frame + axes
        d.rectangle([padL, padT, padL + grid, padT + grid], outline=(0, 70, 80))
        d.line([cx, padT, cx, padT + grid], fill=GRID)
        d.line([padL, cy, padL + grid, cy], fill=GRID)
        d.text((padL + grid - 12, cy + 4), "I", font=font(11), fill=DIM)
        d.text((cx + 5, padT + 2), "Q", font=font(11), fill=DIM)
        # decision gridlines (cell boundaries)
        for k in range(1, s):
            off = (2 * k - s) / norm * scale
            d.line([cx + off, padT + 6, cx + off, padT + grid - 6], fill=(18, 26, 38))
            d.line([padL + 6, cy + off, padL + grid - 6, cy + off], fill=(18, 26, 38))
        # ideal points (dim crosses)
        for (px, py) in pts:
            ix, iy = cx + px * scale, cy - py * scale
            d.line([ix - 3, iy, ix + 3, iy], fill=(70, 90, 110))
            d.line([ix, iy - 3, ix, iy + 3], fill=(70, 90, 110))
        # received symbols
        errs = 0
        for _ in range(args.pts):
            ideal = rnd.randrange(M)
            ix0, iy0 = pts[ideal]
            rx = (ix0 + rnd.gauss(0, sigma), iy0 + rnd.gauss(0, sigma))
            bad = nearest_idx(rx, s, norm) != ideal
            errs += bad
            x, y = cx + rx[0] * scale, cy - rx[1] * scale
            col = WARN if bad else OK
            d.ellipse([x - 1.5, y - 1.5, x + 1.5, y + 1.5], fill=col)
        ber = errs / args.pts

        # readout
        x0 = padL + grid + 26
        d.text((x0, padT - 2), "LIVE READOUT", font=font(12), fill=CYAN)
        y = padT + 24

        def line(lbl, val, col=INK):
            nonlocal y
            d.text((x0, y), lbl, font=font(11), fill=DIM)
            d.text((x0 + 108, y - 3), val, font=font(16, True), fill=col)
            y += 30

        line("modulation", name, CYAN)
        line("link SNR", f"{snr_db:4.1f} dB")
        line("needs", f"{req:4.1f} dB", DIM)
        line("margin", f"{margin:+4.1f} dB", OK if margin > 3 else
             (AMBER if margin > 0 else WARN))
        line("EVM", f"{evm:4.1f} %")
        line("sym errors", f"{errs}/{args.pts}", OK if errs == 0 else WARN)

        # status pill
        if margin > 3 and ber < 0.002:
            st, sc = "LOCKED", OK
        elif margin > 0:
            st, sc = "MARGINAL", AMBER
        else:
            st, sc = "LOST", WARN
        y += 6
        d.rectangle([x0, y, x0 + 210, y + 30], outline=sc, width=2)
        d.ellipse([x0 + 8, y + 10, x0 + 18, y + 20], fill=sc)
        d.text((x0 + 28, y + 6), st, font=font(16, True), fill=sc)
        y += 48
        d.text((x0, y), f"{M} points — {int(math.log2(M))} bits/symbol",
               font=font(11), fill=DIM)

        d.text((padL, H - 22),
               "same channel · climb the modulation until the points pack too "
               "tight for the noise", font=font(11), fill=DIM)
        imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms)
    return 0


if __name__ == "__main__":
    sys.exit(main())
