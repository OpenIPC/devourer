#!/usr/bin/env python3
"""Shared 'DEVOURER live monitor' chrome for the animated documentation diagrams
(constellation / OFDM anatomy / spectrum / AGC), matching the look of the
per-subcarrier waterfall and NHM histogram GIFs. Pillow only."""
from __future__ import annotations

import os

from PIL import Image, ImageDraw, ImageFont

FONT = "/usr/share/fonts/TTF/DejaVuSansMono.ttf"
FONTB = "/usr/share/fonts/TTF/DejaVuSansMono-Bold.ttf"

# House palette (shared with bf_waterfall_gif.py / nhm_histogram_gif.py).
BG = (8, 11, 18)
CYAN = (0, 220, 235)
INK = (225, 232, 240)
DIM = (120, 140, 165)
WARN = (240, 90, 70)
OK = (70, 220, 140)
AMBER = (235, 200, 70)
GRID = (22, 30, 42)


def font(sz, bold=False):
    try:
        return ImageFont.truetype(FONTB if bold else FONT, sz)
    except OSError:
        return ImageFont.load_default()


def new_frame(W, H):
    img = Image.new("RGB", (W, H), BG)
    return img, ImageDraw.Draw(img)


def chrome(d, W, H, title, subtitle, tick):
    """Glowing panel border + DEVOURER header + blinking LIVE + subtitle rule."""
    for i, a in enumerate((40, 90, 160)):
        d.rectangle([6 - i, 6 - i, W - 7 + i, H - 7 + i],
                    outline=(0, a, a), width=1)
    d.text((34, 20), "DEVOURER", font=font(19, True), fill=CYAN)
    d.text((34 + 116, 23), title, font=font(12), fill=INK)
    if tick % 6 < 4:
        d.ellipse([W - 96, 22, W - 86, 32], fill=WARN)
        d.text((W - 80, 21), "LIVE", font=font(12), fill=(240, 90, 90))
    d.line([34, 46, W - 20, 46], fill=(0, 70, 80), width=1)
    d.text((34, 54), subtitle, font=font(11), fill=DIM)


def save_gif(imgs, out, ms=90, colors=128):
    """Save with one global palette (no dither) so unchanged pixels stay
    bit-identical across frames — no 'trembling' text."""
    W, H = imgs[0].size
    sample = imgs[:: max(1, len(imgs) // 8)]
    montage = Image.new("RGB", (W, H * len(sample)))
    for i, im in enumerate(sample):
        montage.paste(im, (0, i * H))
    pal = montage.quantize(colors=colors, method=Image.MEDIANCUT)
    quant = [im.quantize(palette=pal, dither=Image.Dither.NONE) for im in imgs]
    quant[0].save(out, save_all=True, append_images=quant[1:],
                  duration=ms, loop=0, optimize=False, disposal=1)
    kb = os.path.getsize(out) / 1024
    print(f"wrote {out}  {W}x{H}  {len(imgs)} frames  {kb:.0f} KB")
