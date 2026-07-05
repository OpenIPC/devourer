#!/usr/bin/env python3
"""Render an animated NHM (noise-histogram) monitor as a GIF — the frame-free
in-band power distribution, styled as a live spectrum-monitor UI for the docs
(sibling of tools/bf_waterfall_gif.py).

    tools/nhm_histogram_gif.py -o docs/img/nhm_histogram.gif

NHM bins received power into 12 IGI-referenced buckets (low power on the left,
high on the right). This scripts a scenario — an ambient noise floor, then a
narrowband interferer that rises, saturates, and clears — interpolating between
the *real histograms devourer measured* (baseline peaking around bucket 5, a
co-located CW tone shifting the mass up to bucket 8, a strong carrier saturating
into bucket 11). The point it makes visually: as an interferer grows, the
histogram's mass marches into the hot (right) buckets — the whole detection
signal, with no received frame required. Needs Pillow.
"""
from __future__ import annotations

import argparse
import math
import os
import random
import sys

from PIL import Image, ImageDraw, ImageFont

FONT = "/usr/share/fonts/TTF/DejaVuSansMono.ttf"
FONTB = "/usr/share/fonts/TTF/DejaVuSansMono-Bold.ttf"
NB = 12  # NHM buckets


def font(sz, bold=False):
    try:
        return ImageFont.truetype(FONTB if bold else FONT, sz)
    except OSError:
        return ImageFont.load_default()


# Keyframe histograms (real measured shapes, normalised to ~255 total). Each is a
# 12-bucket distribution; the animation eases between consecutive keyframes.
#            b0 b1 b2 b3  b4   b5   b6  b7   b8  b9 b10 b11   phase label / status
KEYS = [
    ([0, 0, 2, 9, 70, 150, 22, 3, 1, 0, 0, 0], "AMBIENT NOISE FLOOR", "CLEAR"),
    ([0, 0, 1, 5, 40, 120, 55, 18, 6, 2, 1, 0], "INTERFERER RISING", "RISING"),
    ([0, 0, 0, 0, 2, 10, 30, 60, 150, 40, 8, 2], "INTERFERER IN-BAND", "DETECTED"),
    ([0, 0, 0, 0, 0, 0, 2, 8, 22, 55, 90, 210], "STRONG / SATURATING", "DETECTED"),
    ([0, 0, 1, 4, 30, 95, 70, 28, 10, 4, 1, 0], "CLEARING", "RISING"),
    ([0, 0, 2, 9, 70, 150, 22, 3, 1, 0, 0, 0], "AMBIENT NOISE FLOOR", "CLEAR"),
]

# Bucket colour ramp: cool green (low power / quiet) -> amber -> hot red (high
# power / interferer). Energy marching right = energy going hot.
def bucket_rgb(k):
    t = k / (NB - 1)
    if t < 0.5:
        u = t / 0.5
        return (int(40 + u * 180), int(200 + u * 20), int(120 - u * 60))
    u = (t - 0.5) / 0.5
    return (int(220 + u * 30), int(220 - u * 150), int(60 - u * 20))


def ease(a, b, t):
    s = t * t * (3 - 2 * t)  # smoothstep
    return a + (b - a) * s


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="nhm_histogram.gif")
    ap.add_argument("--per-key", type=int, default=13, help="frames per segment")
    ap.add_argument("--ms", type=int, default=90)
    ap.add_argument("--colors", type=int, default=128)
    ap.add_argument("--channel", type=int, default=36)
    args = ap.parse_args()

    rnd = random.Random(0x9E)  # deterministic per-frame liveness jitter

    # Build the per-frame histogram sequence by easing across the keyframes.
    seq = []  # (hist[12], phase, status)
    for ki in range(len(KEYS) - 1):
        h0, ph0, st0 = KEYS[ki]
        h1, ph1, st1 = KEYS[ki + 1]
        for fi in range(args.per_key):
            t = fi / args.per_key
            hist = [max(0.0, ease(h0[b], h1[b], t)) for b in range(NB)]
            # small multiplicative shimmer so it reads as a live meter
            hist = [v * (0.90 + 0.14 * rnd.random()) for v in hist]
            seq.append((hist, ph1 if t > 0.5 else ph0, st1 if t > 0.5 else st0))

    # layout
    barw, gap = 46, 8
    padL, padR, padT, padB = 34, 224, 96, 66
    grid_w = NB * barw + (NB - 1) * gap
    grid_h = 300
    W = padL + grid_w + padR
    H = padT + grid_h + padB
    cyan, dim, ink, bg = (0, 220, 235), (120, 140, 165), (225, 232, 240), (8, 11, 18)
    warn, ok = (240, 90, 70), (70, 220, 140)
    f_title, f_lab, f_big, f_sm = font(19, True), font(12), font(16, True), font(11)
    hmax = 255.0

    def status_col(st):
        return {"CLEAR": ok, "RISING": (235, 200, 70), "DETECTED": warn}[st]

    imgs = []
    for fi, (hist, phase, status) in enumerate(seq):
        img = Image.new("RGB", (W, H), bg)
        d = ImageDraw.Draw(img)

        # glowing panel border
        for i, a in enumerate((40, 90, 160)):
            d.rectangle([6 - i, 6 - i, W - 7 + i, H - 7 + i],
                        outline=(0, a, a), width=1)
        # header
        d.text((padL, 20), "DEVOURER", font=f_title, fill=cyan)
        d.text((padL + 116, 23), "IN-BAND POWER MONITOR · NHM", font=f_lab, fill=ink)
        if fi % 6 < 4:
            d.ellipse([W - 96, 22, W - 86, 32], fill=warn)
            d.text((W - 80, 21), "LIVE", font=f_lab, fill=(240, 90, 90))
        d.line([padL, 46, W - 20, 46], fill=(0, 70, 80), width=1)
        d.text((padL, 54),
               f"ch {args.channel} · {5000 + 5*args.channel} MHz · frame-free · "
               f"12 IGI-referenced power buckets", font=f_sm, fill=dim)

        base_y = padT + grid_h
        peak = max(range(NB), key=lambda b: hist[b])
        total = sum(hist) or 1.0
        # gridlines
        for gy in range(0, 5):
            y = base_y - int(gy / 4 * grid_h)
            d.line([padL, y, padL + grid_w, y], fill=(22, 30, 42), width=1)
        # bars
        for b in range(NB):
            x0 = padL + b * (barw + gap)
            hpx = int(min(hist[b], hmax) / hmax * grid_h)
            col = bucket_rgb(b)
            if b == peak:  # highlight the peak bucket
                d.rectangle([x0 - 2, base_y - hpx - 2, x0 + barw + 2, base_y],
                            outline=cyan, width=2)
            d.rectangle([x0, base_y - hpx, x0 + barw, base_y], fill=col)
            d.text((x0 + barw // 2 - 4, base_y + 6), f"{b}", font=f_sm, fill=dim)
        # axis
        d.text((padL, H - 22), "quiet ◂ noise floor", font=f_sm, fill=dim)
        d.text((padL + grid_w - 118, H - 22), "loud / interferer ▸",
               font=f_sm, fill=warn)

        # right readout panel
        x0 = padL + grid_w + 24
        hot = int(100 * sum(hist[8:]) / total)  # fraction in the hot (top) band
        centroid = sum(b * hist[b] for b in range(NB)) / total
        d.text((x0, padT - 2), "LIVE READOUT", font=f_lab, fill=cyan)
        y = padT + 24

        def line(lbl, val, col=ink):
            nonlocal y
            d.text((x0, y), lbl, font=f_sm, fill=dim)
            d.text((x0 + 96, y - 3), val, font=f_big, fill=col)
            y += 30

        line("peak bucket", f"{peak}", bucket_rgb(peak))
        line("centroid", f"{centroid:4.1f}", bucket_rgb(int(round(centroid))))
        line("hot band", f"{hot:3d} %", warn if hot > 25 else dim)
        # status pill
        y += 4
        sc = status_col(status)
        d.rectangle([x0, y, x0 + 190, y + 30], outline=sc, width=2)
        d.ellipse([x0 + 8, y + 10, x0 + 18, y + 20], fill=sc)
        d.text((x0 + 28, y + 6), status, font=f_big, fill=sc)
        y += 46
        d.text((x0, y), "phase", font=f_sm, fill=dim); y += 15
        d.text((x0, y), phase, font=f_lab, fill=ink); y += 30

        # colour legend
        d.text((x0, y), "POWER BAND", font=f_lab, fill=cyan); y += 18
        for lbl, bb in (("floor", 1), ("mid", 6), ("hot", 10)):
            r, g, b = bucket_rgb(bb)
            d.rectangle([x0, y, x0 + 14, y + 11], fill=(r, g, b))
            d.text((x0 + 22, y - 1), lbl, font=f_sm, fill=ink)
            y += 17

        imgs.append(img)

    # one global palette (no dither) so unchanged pixels stay bit-identical
    sample = imgs[:: max(1, len(imgs) // 8)]
    montage = Image.new("RGB", (W, H * len(sample)))
    for i, im in enumerate(sample):
        montage.paste(im, (0, i * H))
    pal = montage.quantize(colors=args.colors, method=Image.MEDIANCUT)
    quant = [im.quantize(palette=pal, dither=Image.Dither.NONE) for im in imgs]
    quant[0].save(args.out, save_all=True, append_images=quant[1:],
                  duration=args.ms, loop=0, optimize=False, disposal=1)
    kb = os.path.getsize(args.out) / 1024
    print(f"wrote {args.out}  {W}x{H}  {len(imgs)} frames  {kb:.0f} KB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
