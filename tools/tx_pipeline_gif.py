#!/usr/bin/env python3
"""Animated TX pipeline — 'how a packet becomes a radio waveform', in the
DEVOURER live-monitor style.

    tools/tx_pipeline_gif.py -o docs/img/tx_pipeline.gif

A block of bits is scrambled, forward-error-coded, interleaved, mapped onto the
subcarrier constellations, turned into a time-domain OFDM symbol by an inverse
FFT, given a cyclic-prefix guard, and pushed out the DAC to the antenna. The
animation walks a data block through each stage, showing what it looks like along
the way — raw bits, constellation points, then the waveform. This is the chain
behind DEVOURER_TX_RATE and the modulated carrier. Needs Pillow.
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

# (short, kind, description)
STAGES = [
    ("BITS", "bits", "raw data bits from the MAC"),
    ("SCRAMBLE", "bits", "whiten — break up long runs of 0s/1s"),
    ("FEC", "coded", "forward error correction — add redundancy"),
    ("INTERLEAVE", "bits", "spread bits so a fade hits many codewords lightly"),
    ("QAM MAP", "qam", "bits → constellation points, one per subcarrier"),
    ("IFFT", "wave", "subcarriers → one time-domain OFDM symbol"),
    ("+CP", "cp", "copy the tail to the front — a guard vs echoes"),
    ("RF / DAC", "rf", "up-convert and radiate"),
]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="tx_pipeline.gif")
    ap.add_argument("--per", type=int, default=9)
    ap.add_argument("--ms", type=int, default=110)
    args = ap.parse_args()
    rnd = random.Random(0x7A)
    bits = [rnd.randint(0, 1) for _ in range(48)]

    padL, padT = 34, 92
    W, H = 900, 452
    n = len(STAGES)
    bw = (W - 2 * padL) // n
    row_y = padT + 6
    box_h = 54
    detail_y = row_y + box_h + 40
    detail_h = 190

    imgs = []
    for si in range(n):
        for fi in range(args.per):
            gi = si * args.per + fi
            img, d = new_frame(W, H)
            chrome(d, W, H, "TX PIPELINE",
                   "a block of bits → scramble → FEC → interleave → QAM → IFFT → "
                   "cyclic prefix → RF", gi)

            # stage row
            for j, (name, kind, _) in enumerate(STAGES):
                x = padL + j * bw
                on = j == si
                done = j < si
                oc = CYAN if on else (OK if done else (55, 70, 88))
                d.rectangle([x + 4, row_y, x + bw - 4, row_y + box_h],
                            outline=oc, width=2 if on else 1)
                d.text((x + 12, row_y + 8), name, font=font(11, True),
                       fill=INK if on else (DIM if not done else OK))
                if j < n - 1:
                    ax = x + bw - 4
                    ac = OK if done else (55, 70, 88)
                    d.line([ax, row_y + box_h // 2, ax + 8, row_y + box_h // 2],
                           fill=ac, width=2)
            # flowing token (pulses every frame so nothing gets merged away)
            tx = padL + (si + 0.5) * bw
            pr = 5 + 2 * math.sin(gi * 0.9)
            d.ellipse([tx - pr, row_y + box_h + 8, tx + pr, row_y + box_h + 8 + 2 * pr],
                      fill=AMBER)
            # data particles flowing along the pipeline baseline
            for p in range(6):
                px = padL + ((gi * 14 + p * 150) % (W - 2 * padL))
                d.ellipse([px - 2, row_y - 12, px + 2, row_y - 8],
                          fill=(0, 120, 130))

            # detail panel
            name, kind, desc = STAGES[si]
            d.rectangle([padL, detail_y, W - padL, detail_y + detail_h],
                        outline=(0, 70, 80))
            cx0, cyc = padL + 40, detail_y + detail_h // 2

            if kind in ("bits", "coded"):
                nb = 48 if kind == "bits" else 64
                seq = bits if kind != "coded" else bits + bits[:16]
                for k in range(min(nb, 56)):
                    bx = cx0 + k * 14
                    v = seq[k % len(seq)]
                    d.text((bx, cyc - 8), str(v), font=font(15, True),
                           fill=CYAN if v else (70, 95, 120))
                if kind == "coded":
                    d.text((cx0, cyc + 26), "48 bits → 64 (rate-3/4 example)",
                           font=font(11), fill=OK)
            elif kind == "qam":
                # a 16-QAM constellation, centred; caption below it (no overlap)
                g = detail_h - 70
                ox, oy = cx0 + 180, detail_y + 22
                d.text((cx0, detail_y + 14), "16-QAM", font=font(11), fill=DIM)
                for iq in range(4):
                    for qq in range(4):
                        px = ox + iq * (g // 4)
                        py = oy + qq * (g // 4)
                        d.ellipse([px - 3, py - 3, px + 3, py + 3], fill=CYAN)
                d.text((cx0, detail_y + detail_h - 26),
                       "each subcarrier picks one constellation point",
                       font=font(12), fill=INK)
            else:
                # waveform (sum of a few sines = an OFDM symbol)
                pts = []
                span = W - 2 * padL - 260
                for k in range(span):
                    ph = k / span
                    v = (math.sin(ph * 22) + 0.6 * math.sin(ph * 41 + 1) +
                         0.4 * math.sin(ph * 7 + gi * 0.3))
                    pts.append((cx0 + 200 + k, cyc - int(v * 34)))
                col = OK if kind != "rf" else AMBER
                if kind == "cp":
                    # highlight the copied prefix chunk
                    d.rectangle([cx0 + 200, cyc - 44, cx0 + 200 + span // 6,
                                 cyc + 44], fill=(40, 40, 14))
                    d.text((cx0 + 200, cyc - 62), "cyclic prefix",
                           font=font(10), fill=AMBER)
                d.line(pts, fill=col, width=2)
                if kind == "rf":
                    for r in (10, 20, 30):
                        d.arc([cx0 + 60 - r, cyc - r, cx0 + 60 + r, cyc + r],
                              300, 60, fill=AMBER)
                    d.line([cx0 + 60, cyc - 34, cx0 + 60, cyc + 20], fill=AMBER,
                           width=2)
                d.text((cx0, detail_y + 12), "OFDM symbol (time domain)",
                       font=font(11), fill=DIM)

            # caption
            d.text((padL, detail_y + detail_h + 18),
                   f"{si+1}. {name} — {desc}", font=font(13, True), fill=CYAN)
            imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms, colors=48)
    return 0


if __name__ == "__main__":
    sys.exit(main())
