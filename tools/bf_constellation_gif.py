#!/usr/bin/env python3
"""Animated per-tone channel-direction constellation from SU beamforming
reports — a live spectrum-console GIF for the docs.

An SU report gives, per subcarrier, the complex ratio between the beamformer's
two TX antennas:  h_B/h_A = |h_B/h_A| * e^{j*phi}  (from the Givens angles
psi, phi:  |h_B/h_A| = tan(psi), arg = phi). This plots all subcarriers as
points on the complex plane — the unit circle is "antennas balanced", inside =
antenna A stronger, outside = antenna B stronger, angle = relative phase. The
tones form an arc (the channel signature) that shimmers with the real
report-to-report jitter; a bright marker sweeps tone-by-tone through frequency.

    tools/bf_constellation_gif.py su_capture.txt -o docs/img/bf_constellation.gif

Needs Pillow.
"""
from __future__ import annotations

import argparse
import colorsys
import math
import os
import sys

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bf_report_decode as bf

FONT = "/usr/share/fonts/TTF/DejaVuSansMono.ttf"
FONTB = "/usr/share/fonts/TTF/DejaVuSansMono-Bold.ttf"


def font(sz, bold=False):
    try:
        return ImageFont.truetype(FONTB if bold else FONT, sz)
    except OSError:
        return ImageFont.load_default()


def freq_rgb(t):
    """Rainbow across the band (t in 0..1): hue sweep, bright."""
    r, g, b = colorsys.hsv_to_rgb(0.66 * (1 - t), 0.85, 1.0)
    return (int(r * 255), int(g * 255), int(b * 255))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("infile")
    ap.add_argument("-o", "--out", default="bf_constellation.gif")
    ap.add_argument("--frames", type=int, default=52)
    ap.add_argument("--ms", type=int, default=110)
    ap.add_argument("--colors", type=int, default=160)
    ap.add_argument("--channel", type=int, default=100)
    ap.add_argument("--rmax", type=float, default=2.0, help="plot radius = this ratio")
    args = ap.parse_args()

    frames_in = [bf.parse_frame(l.split("raw>")[1].strip())
                 for l in open(args.infile) if "raw>" in l]
    frames_in = [f for f in frames_in if f and not f["feedback"]]
    if not frames_in:
        sys.stderr.write("no SU reports in capture\n"); return 1
    f0 = frames_in[0]
    ns = bf.NS_TABLE[f0["bw"]][f0["ng"]]
    lay = bf.angle_layout(f0["nr"], f0["nc"])
    vb = (ns * 10 + 7) // 8

    def zrow(f):
        dec = bf.decode_angles(f["angle_bytes"][:vb], ns, len(lay), 6, 4, lay)
        if not dec:
            return None
        row = [(math.tan(d["psi"][0]), d["phi"][0]) for d in dec]
        # Remove the per-report common phase: the beamformee's absolute phase
        # reference drifts report-to-report (an arbitrary global rotation), so
        # only the phase *across tones* is real channel. Subtract the vector-mean
        # phase so the arc keeps its shape (the frequency phase-ramp = delay)
        # and only the true per-tone jitter animates — not the reference spin.
        cx_ = sum(m * math.cos(p) for m, p in row)
        cy_ = sum(m * math.sin(p) for m, p in row)
        ref = math.atan2(cy_, cx_)
        return [(m, p - ref) for m, p in row]
    rows = [z for z in (zrow(f) for f in frames_in) if z]
    n = len(rows)
    span = 20 << f0["bw"]
    _allm = sorted(m for r in rows for (m, _) in r)
    rmax = max(1.2, min(2.6, _allm[int(0.90 * len(_allm))] * 1.25))

    # layout
    W, H = 720, 560
    cx, cy, R = 262, 300, 232
    bg = (8, 11, 18)
    cyan, dim, ink = (0, 220, 235), (120, 140, 165), (225, 232, 240)
    f_title, f_lab, f_big, f_sm = font(19, True), font(12), font(16, True), font(11)

    def pt(mag, phi):
        rr = min(mag, rmax) / rmax * R
        return cx + rr * math.cos(phi), cy - rr * math.sin(phi)

    imgs = []
    for fi in range(args.frames):
        img = Image.new("RGB", (W, H), bg)
        d = ImageDraw.Draw(img)
        rep = rows[fi % n]
        swept = fi % ns                     # tone the marker highlights

        # glow border
        for i, a in enumerate((40, 90, 160)):
            d.rectangle([6 - i, 6 - i, W - 7 + i, H - 7 + i],
                        outline=(0, a, a), width=1)
        # header
        d.text((24, 20), "DEVOURER", font=f_title, fill=cyan)
        d.text((140, 23), "PER-TONE CHANNEL-DIRECTION CONSTELLATION",
               font=f_lab, fill=ink)
        if fi % 6 < 4:
            d.ellipse([W - 96, 22, W - 86, 32], fill=(240, 70, 70))
            d.text((W - 80, 21), "LIVE", font=f_lab, fill=(240, 90, 90))
        d.text((24, 52), f"ch {args.channel} · {5000 + 5*args.channel} MHz · "
               f"{span} MHz · VHT SU sounding · h_B / h_A per subcarrier",
               font=f_sm, fill=dim)

        # polar grid
        for ring in (0.5, 1.0, 1.5, 2.0, 2.5):
            rr = ring / rmax * R
            if rr > R: continue
            col = (0, 120, 130) if ring == 1.0 else (28, 40, 55)
            wdt = 2 if ring == 1.0 else 1
            d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], outline=col, width=wdt)
            d.text((cx + rr - 8, cy + 3), f"{ring:.1f}", font=f_sm,
                   fill=(60, 80, 100))
        for ang in range(0, 360, 30):        # radial spokes
            a = math.radians(ang)
            d.line([cx, cy, cx + R * math.cos(a), cy - R * math.sin(a)],
                   fill=(22, 32, 46), width=1)
        d.text((cx + R + 4, cy - 6), "0°", font=f_sm, fill=(60, 80, 100))
        d.text((cx + 4, cy - R + 2), "90°", font=f_sm, fill=(60, 80, 100))
        d.text((cx - 66, cy - 4), "balanced →", font=f_sm, fill=(0, 110, 120))

        # motion trails: last few reports, faint
        for back in range(6, 0, -1):
            pr = rows[(fi - back) % n]
            fade = 0.10 + 0.06 * (6 - back)
            for k, (m, p) in enumerate(pr):
                x, y = pt(m, p)
                r0, g0, b0 = freq_rgb(k / (ns - 1))
                d.point([(x, y)], fill=(int(r0 * fade), int(g0 * fade),
                                        int(b0 * fade)))

        # arc connecting adjacent tones of the current report
        poly = [pt(m, p) for (m, p) in rep]
        for k in range(len(poly) - 1):
            r0, g0, b0 = freq_rgb(k / (ns - 1))
            d.line([poly[k], poly[k + 1]],
                   fill=(r0 // 2, g0 // 2, b0 // 2), width=1)
        # tone points (rainbow by frequency)
        for k, (m, p) in enumerate(rep):
            x, y = pt(m, p)
            c = freq_rgb(k / (ns - 1))
            d.ellipse([x - 3, y - 3, x + 3, y + 3], fill=c)

        # swept marker: phasor from origin + ring
        m, p = rep[swept]
        x, y = pt(m, p)
        d.line([cx, cy, x, y], fill=(255, 255, 255), width=1)
        d.ellipse([x - 6, y - 6, x + 6, y + 6], outline=(255, 255, 255), width=2)

        # readout panel
        x0 = 540
        foff = (swept - ns / 2) * (span / ns)
        dom = "antenna B" if m > 1.05 else "antenna A" if m < 0.95 else "balanced"
        domc = freq_rgb(swept / (ns - 1))
        d.text((x0, 84), "SWEEP READOUT", font=f_lab, fill=cyan)
        yy = 108
        def line(lbl, val, col=ink):
            nonlocal yy
            d.text((x0, yy), lbl, font=f_sm, fill=dim)
            d.text((x0, yy + 13), val, font=f_big, fill=col); yy += 46
        line("subcarrier", f"#{swept}  {foff:+.1f} MHz", domc)
        line("|h_B/h_A|", f"{m:4.2f}", domc)
        line("rel phase", f"{(math.degrees(p)+180)%360-180:+5.0f}°")
        line("stronger", dom, domc)

        # frequency colour bar
        by0 = 348
        d.text((x0, by0 - 16), "subcarrier (frequency)", font=f_sm, fill=dim)
        for i in range(140):
            d.line([x0 + i, by0, x0 + i, by0 + 12], fill=freq_rgb(i / 139))
        d.text((x0, by0 + 16), f"{-span/2:+.0f}", font=f_sm, fill=dim)
        d.text((x0 + 108, by0 + 16), f"{span/2:+.0f} MHz", font=f_sm, fill=dim)

        d.text((x0, 430), "unit circle = TX antennas", font=f_sm, fill=dim)
        d.text((x0, 444), "balanced; inside = A, ", font=f_sm, fill=dim)
        d.text((x0, 458), "outside = B stronger", font=f_sm, fill=dim)

        imgs.append(img)

    # single global palette (avoid per-frame palette shimmer), no dither
    sample = imgs[:: max(1, len(imgs) // 8)]
    montage = Image.new("RGB", (W, H * len(sample)))
    for i, im in enumerate(sample):
        montage.paste(im, (0, i * H))
    pal = montage.quantize(colors=args.colors, method=Image.MEDIANCUT)
    quant = [im.quantize(palette=pal, dither=Image.Dither.NONE) for im in imgs]
    quant[0].save(args.out, save_all=True, append_images=quant[1:],
                  duration=args.ms, loop=0, optimize=False, disposal=1)
    print(f"wrote {args.out}  {W}x{H}  {len(imgs)} frames  "
          f"{os.path.getsize(args.out)/1024:.0f} KB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
