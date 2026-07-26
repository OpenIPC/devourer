#!/usr/bin/env python3
"""Animated HE extended range — 'the range ladder', in the DEVOURER
live-monitor style.

    tools/he_extended_range_gif.py -o docs/img/he_extended_range.gif

802.11ax has a low-rate corner built for reach, and it stacks in three steps.
ER SU repeats the signalling field so the preamble is acquired at lower SNR;
the 106-tone variant pours the same transmit power into half as many tones; and
DCM sends every symbol twice, on two tones far enough apart that one narrow
fade cannot take both copies. Each step is worth roughly 3 dB, and each is paid
for in rate.

The figure climbs the ladder one rung at a time: the tone comb changes shape,
the link-budget bar grows, the rate bar shrinks, and the last act shows a fade
eating one DCM copy while the other survives. Needs Pillow.
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, GRID, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

NB = 48          # tone groups drawn across the 20 MHz channel
FADE = (26, 31)  # the narrow fade that lands in the last act

# name, cumulative dB, relative rate, constraint
LADDER = [
    ("HE SU  MCS0", 0.0, 1.00, "the baseline — full 242-tone RU"),
    ("+ ER 242-tone", 3.0, 1.00, "MCS 0-2, NSS 1"),
    ("+ ER 106-tone", 6.0, 0.50, "MCS 0, NSS 1"),
    ("+ DCM", 9.0, 0.25, "MCS 0/1; excludes STBC"),
]
ACT = 15         # frames per rung


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="he_extended_range.gif")
    ap.add_argument("--hold", type=int, default=18)
    ap.add_argument("--ms", type=int, default=110)
    args = ap.parse_args()

    padL, padT = 96, 96
    gw = 600
    combY, combH = padT + 26, 96
    barY = combY + combH + 62
    panelW = 236
    W = padL + gw + 24 + panelW
    H = barY + 128
    bw = gw / NB

    imgs = []
    total = len(LADDER) * ACT
    for fi in range(total + args.hold):
        k = min(len(LADDER) - 1, fi // ACT)
        sub = min(1.0, (fi - k * ACT + 1) / ACT) if fi < total else 1.0
        name, gain, rate, constraint = LADDER[k]
        er106 = k >= 2
        dcm = k >= 3
        img, d = new_frame(W, H)
        chrome(d, W, H, "802.11ax — EXTENDED RANGE",
               "three stackable steps that trade rate for reach, and "
               "interoperate with any 802.11ax device", fi)

        # ---- the tone comb -------------------------------------------------
        d.text((padL, combY - 20), "the channel's tones — where the power goes",
               font=font(11), fill=DIM)
        d.line([padL, combY + combH, padL + gw, combY + combH], fill=(0, 70, 80))
        for i in range(NB):
            x = padL + i * bw
            active = (i >= NB // 2) if er106 else True
            faded = dcm and FADE[0] <= i <= FADE[1]
            if not active:
                d.rectangle([x + 1, combY + combH - 6, x + bw - 1, combY + combH],
                            fill=(24, 30, 40))
                continue
            # concentrating the same power into half the tones raises each one
            hgt = combH * (0.80 if er106 else 0.46)
            col = CYAN
            if faded:
                hgt *= 0.12
                col = WARN
            d.rectangle([x + 1, combY + combH - hgt, x + bw - 1, combY + combH],
                        fill=col)
        if er106:
            d.text((padL + 6, combY + combH - 16), "no power here",
                   font=font(10), fill=(70, 84, 104))
            d.text((padL + gw * 0.42, combY + combH + 6),
                   "upper half only — twice the power per tone",
                   font=font(10), fill=OK)

        # DCM: link the tone pairs that carry the same bits
        if dcm:
            q = NB // 8
            for i in range(NB // 2, NB // 2 + q):
                if i + q >= NB:
                    break
                xa = padL + i * bw + bw / 2
                xb = padL + (i + q) * bw + bw / 2
                d.line([xa, combY - 2, xa, combY - 8], fill=AMBER)
                d.line([xb, combY - 2, xb, combY - 8], fill=AMBER)
                d.line([xa, combY - 8, xb, combY - 8], fill=AMBER)
            d.text((padL + gw / 2 + 6, combY - 34),
                   "same bits on two tones, far apart", font=font(10), fill=AMBER)
            if FADE[0] >= NB // 2:
                d.text((padL + gw * 0.42, combY + combH + 22),
                       "a narrow fade takes one copy — the other decodes",
                       font=font(10), fill=WARN)

        # ---- link budget and rate -----------------------------------------
        def bar(y, label, frac, col, value):
            d.text((padL - 74, y + 2), label, font=font(10), fill=DIM)
            d.rectangle([padL, y, padL + gw * 0.62, y + 20], outline=(0, 70, 80))
            d.rectangle([padL + 1, y + 1, padL + 1 + (gw * 0.62 - 2) * frac,
                         y + 19], fill=col)
            d.text((padL + gw * 0.62 + 12, y + 2), value, font=font(12, True),
                   fill=col)

        shown = gain - 3.0 * (1 - sub) if k else 0.0
        bar(barY, "link budget", max(0.0, shown) / 12.0, OK, f"+{shown:.0f} dB")
        bar(barY + 34, "rate", rate, AMBER if rate < 1 else CYAN,
            f"x{rate:.2f}".replace("x0.25", "x1/4").replace("x0.50", "x1/2")
            .replace("x1.00", "x1"))

        # the ladder itself, rungs lit as we climb
        ly = barY + 74
        for i, (n, g, _, _) in enumerate(LADDER):
            on = i <= k
            d.text((padL + i * 152, ly), ("● " if on else "○ ") + n,
                   font=font(11, True) if i == k else font(11),
                   fill=OK if i == k else (INK if on else (60, 72, 90)))

        # ---- readout -------------------------------------------------------
        x0 = padL + gw + 22
        d.text((x0, padT - 2), "LIVE READOUT", font=font(12), fill=CYAN)
        y = padT + 24

        def line(lbl, val, c=INK):
            nonlocal y
            d.text((x0, y), lbl, font=font(11), fill=DIM)
            d.text((x0 + 96, y - 3), val, font=font(14, True), fill=c)
            y += 28

        line("mode", name.replace("+ ", ""), OK if k else INK)
        line("vs HE SU", f"+{gain:.0f} dB" if gain else "—", OK if gain else DIM)
        line("rate", "x1" if rate == 1 else ("x1/2" if rate == 0.5 else "x1/4"),
             AMBER if rate < 1 else INK)
        line("bandwidth", "20 MHz", DIM)
        y += 4
        d.text((x0, y), constraint, font=font(10), fill=DIM)
        y += 22

        caps = (
            ("the starting point: one", "full-width 20 MHz RU,", "BPSK, nothing stacked."),
            ("ER SU repeats the", "signalling field, so the", "preamble survives a",
             "weaker signal. rate is", "unchanged."),
            ("the 106-tone variant", "spends the same power", "on half the tones —",
             "each one gets twice as", "much."),
            ("DCM sends every symbol", "twice, far apart in", "frequency. it is the",
             "only rung that pays in", "rate for payload, not", "preamble, gain."),
        )
        for ln in caps[k]:
            d.text((x0, y), ln, font=font(11), fill=INK)
            y += 15

        imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms, colors=48)
    return 0


if __name__ == "__main__":
    sys.exit(main())
