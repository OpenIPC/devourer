#!/usr/bin/env python3
"""Render an animated per-subcarrier SNR waterfall (GIF) from captured MU
beamforming reports — styled as a live spectrum-monitor UI for the docs.

    tools/bf_waterfall_gif.py capture.txt -o docs/img/bf_waterfall.gif

Each frame scrolls one report in from the top (newest) downward, as if the
reports were streaming live. Colour = per-tone SNR / the modulation a
rate-adaptive link would pick. Needs Pillow.
"""
from __future__ import annotations

import argparse
import os
import sys

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bf_report_decode as bf
from bf_waterfall import ramp, qam_for, _QAM_BANDS

FONT = "/usr/share/fonts/TTF/DejaVuSansMono.ttf"
FONTB = "/usr/share/fonts/TTF/DejaVuSansMono-Bold.ttf"


def font(sz, bold=False):
    try:
        return ImageFont.truetype(FONTB if bold else FONT, sz)
    except OSError:
        return ImageFont.load_default()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("infile")
    ap.add_argument("-o", "--out", default="bf_waterfall.gif")
    ap.add_argument("--frames", type=int, default=64)
    ap.add_argument("--visible-rows", type=int, default=40)
    ap.add_argument("--snr-lo", type=float, default=8.0)
    ap.add_argument("--snr-hi", type=float, default=30.0)
    ap.add_argument("--ms", type=int, default=90, help="frame duration (ms)")
    ap.add_argument("--colors", type=int, default=128, help="GIF palette size")
    ap.add_argument("--channel", type=int, default=100)
    args = ap.parse_args()

    # decode per-tone SNR (dB) for every report
    frames_in = [bf.parse_frame(l.split("raw>")[1].strip())
                 for l in open(args.infile) if "raw>" in l]
    frames_in = [f for f in frames_in if f and f["feedback"]]
    if not frames_in:
        sys.stderr.write("no MU reports in capture\n"); return 1
    bw, ng = frames_in[0]["bw"], frames_in[0]["ng"]
    ns = bf.NS_TABLE[bw][ng]
    vb = (ns * 10 + 7) // 8
    rows = [[bf.u8_snr_db(v) for v in s]
            for s in (bf.parse_mu_snr(f, ns, vb) for f in frames_in) if s]
    ncol = max(set(len(r) for r in rows), key=[len(r) for r in rows].count)
    rows = [r[:ncol] for r in rows if len(r) >= ncol]

    # layout
    cw, ch = 24, 9
    padL, padR, padT, padB = 24, 210, 92, 64
    grid_w, grid_h = ncol * cw, args.visible_rows * ch
    W = padL + grid_w + padR
    H = padT + grid_h + padB
    lo, hi = args.snr_lo, args.snr_hi
    cyan, dim, ink, bg = (0, 220, 235), (120, 140, 165), (225, 232, 240), (8, 11, 18)
    f_title, f_lab, f_big, f_sm = font(19, True), font(12), font(15, True), font(11)

    def snr_rgb(db):
        return ramp((db - lo) / (hi - lo))

    span = 20 << bw

    def draw_chrome(img, d, tick, top_row):
        # glowing panel border
        for i, a in enumerate((40, 90, 160)):
            d.rectangle([6 - i, 6 - i, W - 7 + i, H - 7 + i],
                        outline=(0, a, a), width=1)
        # header
        d.text((padL, 20), "DEVOURER", font=f_title, fill=cyan)
        d.text((padL + 116, 23), "PER-SUBCARRIER SPECTRUM MONITOR",
               font=f_lab, fill=ink)
        # blinking LIVE
        if tick % 6 < 4:
            d.ellipse([W - 96, 22, W - 86, 32], fill=(240, 70, 70))
            d.text((W - 80, 21), "LIVE", font=f_lab, fill=(240, 90, 90))
        d.line([padL, 44, W - padR + grid_w if False else W - 20, 44],
               fill=(0, 70, 80), width=1)
        d.text((padL, 52),
               f"ch {args.channel} · {5000 + 5*args.channel} MHz · "
               f"{span} MHz · VHT MU sounding · {ncol} carrier groups",
               font=f_sm, fill=dim)
        # frequency axis
        for c in range(0, ncol, 4):
            foff = (c - ncol / 2) * (span / ncol)
            x = padL + c * cw + cw // 2
            d.text((x - 8, padT - 16), f"{foff:+.0f}", font=f_sm, fill=dim)
        d.text((padL, H - 20), "frequency offset (MHz)  ·  time scrolls ↓",
               font=f_sm, fill=dim)

    def draw_readout(d, cur):
        x0 = padL + grid_w + 22
        mn, mx = min(cur), max(cur)
        d.text((x0, padT - 2), "LIVE READOUT", font=f_lab, fill=cyan)
        y = padT + 22
        def line(lbl, val, col=ink):
            nonlocal y
            d.text((x0, y), lbl, font=f_sm, fill=dim)
            d.text((x0 + 92, y - 3), val, font=f_big, fill=col)
            y += 30
        line("peak SNR", f"{mx:4.1f} dB", snr_rgb(mx))
        line("min  SNR", f"{mn:4.1f} dB", snr_rgb(mn))
        line("best mod", qam_for(mx).strip(), snr_rgb(mx))
        line("worst mod", qam_for(mn).strip(), snr_rgb(mn))
        # mini per-tone bar (current profile)
        y += 6
        d.text((x0, y), "current profile", font=f_sm, fill=dim); y += 16
        bw_px = 176
        for c, db in enumerate(cur):
            h = int((db - lo) / (hi - lo) * 46)
            h = max(1, min(46, h))
            bx = x0 + int(c / ncol * bw_px)
            d.rectangle([bx, y + 46 - h, bx + max(2, bw_px // ncol - 1),
                         y + 46], fill=snr_rgb(db))
        y += 62
        # QAM legend
        d.text((x0, y), "SNR → MODULATION", font=f_lab, fill=cyan); y += 18
        for i, (_, name) in enumerate(reversed(_QAM_BANDS[:-1])):
            r, g, b = ramp(i / 4)
            d.rectangle([x0, y, x0 + 14, y + 11], fill=(r, g, b))
            d.text((x0 + 22, y - 1), name, font=f_sm, fill=ink)
            y += 17

    imgs = []
    n = len(rows)
    for fi in range(args.frames):
        img = Image.new("RGB", (W, H), bg)
        d = ImageDraw.Draw(img)
        # newest report index for this frame
        top = fi % n
        cur = rows[top]
        draw_chrome(img, d, fi, top)
        # waterfall: row 0 = newest (top), fading downward
        for vr in range(args.visible_rows):
            r = rows[(top - vr) % n]
            y = padT + vr * ch
            fade = 1.0 - 0.45 * (vr / args.visible_rows)
            for c, db in enumerate(r):
                rr, gg, bb = snr_rgb(db)
                d.rectangle([padL + c * cw, y, padL + c * cw + cw,
                             y + ch], fill=(int(rr*fade), int(gg*fade), int(bb*fade)))
        # bright scan edge on the newest row
        d.rectangle([padL, padT, padL + grid_w, padT + 2], fill=cyan)
        draw_readout(d, cur)
        imgs.append(img.convert("P", palette=Image.ADAPTIVE, colors=args.colors))

    imgs[0].save(args.out, save_all=True, append_images=imgs[1:],
                 duration=args.ms, loop=0, optimize=True, disposal=2)
    kb = os.path.getsize(args.out) / 1024
    print(f"wrote {args.out}  {W}x{H}  {len(imgs)} frames  {kb:.0f} KB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
