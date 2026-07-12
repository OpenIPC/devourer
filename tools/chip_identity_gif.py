#!/usr/bin/env python3
"""Animated chip identity — 'one driver binary, gated at runtime', in the
DEVOURER live-monitor style (docs/driver-primer.md §3).

    tools/chip_identity_gif.py -o docs/img/chip_identity.gif

One vendor driver serves a whole family of boards and learns which one it is
standing on at runtime: chip id + cut from identity registers, RFE type and
board options from the efuse. The animation walks four boards — same driver,
different silicon and front-end wiring — filling an identity readout per board
and flipping the config gates (per-RFE table blocks, per-cut fixes, firmware
image, AGC table variant) that this identity drives. Needs Pillow.
"""
from __future__ import annotations

import argparse
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

# boards: (marketing name, chip id, cut, rfe, chains, n front-end doodads,
#          gate states) — gates: (rfe=1 block, B-cut fix, fw image, agc table)
BOARDS = [
    ("dongle A", "0x08 (8812A)", "B", 0, "2T2R", 2,
     (None, "B-cut RF fix", "8812a_fw.bin", "type 0")),
    ("dongle B", "0x0A (8822B)", "D", 1, "2T2R", 3,
     ("rfe=1 block", None, "8822b_fw.bin", "type 1")),
    ("dongle C", "0x09 (8821C)", "A", 2, "1T1R", 1,
     (None, None, "8821c_fw.bin", "type 2")),
    ("dongle D", "0x51 (8852B)", "C", 5, "2T2R", 4,
     (None, None, "8852b Ccut RAM", "type 5")),
]
GATES = ("PHY TABLE BLOCK", "PER-CUT FIX", "FIRMWARE IMAGE", "AGC TABLE")
FIELDS = ("chip id", "cut", "rfe type", "chains")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="chip_identity.gif")
    ap.add_argument("--per", type=int, default=22)
    ap.add_argument("--ms", type=int, default=110)
    args = ap.parse_args()
    rnd = random.Random(0x1D)

    W, H = 900, 452
    bx0, bw, bgap = 40, 190, 22
    board_y0, board_y1 = 92, 178
    id_y0, id_y1 = 196, 300
    gate_y0, gate_y1 = 316, 388

    # deterministic doodad layouts per board (front-end parts differ)
    doodads = []
    for _, _, _, _, _, nd, _ in BOARDS:
        pts = [(80 + rnd.randint(0, 68), 30 + rnd.randint(0, 34))
               for _ in range(nd)]
        doodads.append(pts)

    imgs = []
    for bi, (bname, cid, cut, rfe, chains, nd, gates) in enumerate(BOARDS):
        for fi in range(args.per):
            gi = bi * args.per + fi
            img, d = new_frame(W, H)
            chrome(d, W, H, "CHIP IDENTITY",
                   "one driver, many chips: chip id + cut + RFE type gate "
                   "everything at runtime", gi)

            # ---- boards row
            for j, (nm, _, _, _, _, ndj, _) in enumerate(BOARDS):
                x0 = bx0 + j * (bw + bgap)
                on = j == bi
                oc = CYAN if on else (55, 70, 88)
                d.rectangle([x0, board_y0, x0 + bw, board_y1],
                            outline=oc, width=2 if on else 1)
                d.text((x0 + 8, board_y0 + 6), nm, font=font(11, True),
                       fill=INK if on else DIM)
                # the same chip on every board
                cc = AMBER if on else (90, 80, 40)
                d.rectangle([x0 + 12, board_y0 + 28, x0 + 68, board_y0 + 62],
                            outline=cc)
                d.text((x0 + 20, board_y0 + 38), "chip", font=font(10),
                       fill=cc)
                # different front-end doodads (PA/LNA/switch parts)
                for (dx, dy) in doodads[j]:
                    dc = OK if on else (40, 70, 55)
                    d.rectangle([x0 + dx, board_y0 + dy,
                                 x0 + dx + 16, board_y0 + dy + 10],
                                outline=dc)
                # antenna stub
                ax = x0 + bw - 18
                d.line([ax, board_y0 + 30, ax, board_y0 + 58],
                       fill=oc, width=1)
                d.line([ax - 6, board_y0 + 38, ax, board_y0 + 30],
                       fill=oc, width=1)

            # ---- identity readout (typed in progressively)
            d.rectangle([bx0, id_y0, bx0 + 470, id_y1],
                        outline=(0, 110, 120))
            d.text((bx0 + 12, id_y0 + 8), "IDENTITY READOUT",
                   font=font(11, True), fill=CYAN)
            vals = (cid, f"{cut}-cut", str(rfe), chains)
            reveal = min(1.0, fi / (args.per * 0.5))  # fills in first half
            nshow = int(reveal * len(FIELDS) + 0.999)
            for k, (fld, v) in enumerate(zip(FIELDS, vals)):
                y = id_y0 + 30 + k * 18
                d.text((bx0 + 12, y), f"{fld:<9}:", font=font(11), fill=DIM)
                if k < nshow:
                    d.text((bx0 + 100, y), v, font=font(11, True), fill=INK)
                elif k == nshow:
                    d.text((bx0 + 100, y), "_" if gi % 2 else " ",
                           font=font(11, True), fill=AMBER)
            # where identity comes from
            sx = bx0 + 494
            d.rectangle([sx, id_y0, W - 38, id_y1], outline=(35, 45, 60))
            d.text((sx + 12, id_y0 + 8), "READ FROM", font=font(11, True),
                   fill=DIM)
            for k, line in enumerate(("SYS_CFG reg : chip id, cut",
                                      "efuse (§4)  : RFE type,",
                                      "              board options",
                                      "vendor: IS_C_CUT(), rfe_type==2")):
                d.text((sx + 12, id_y0 + 32 + k * 17), line, font=font(10),
                       fill=DIM)

            # ---- config gates
            open_now = fi >= args.per * 0.55  # flip after identity is read
            for k, gname in enumerate(GATES):
                x0 = bx0 + k * (bw + bgap)
                gv = gates[k]
                is_open = open_now and gv is not None
                oc = OK if is_open else (55, 70, 88)
                d.rectangle([x0, gate_y0, x0 + bw, gate_y1],
                            outline=oc, width=2 if is_open else 1)
                d.text((x0 + 8, gate_y0 + 6), gname, font=font(10, True),
                       fill=INK if is_open else DIM)
                if not open_now:
                    d.text((x0 + 8, gate_y0 + 28), "...", font=font(11),
                           fill=DIM)
                elif is_open:
                    d.text((x0 + 8, gate_y0 + 26), "OPEN", font=font(10, True),
                           fill=OK)
                    d.text((x0 + 8, gate_y0 + 44), gv, font=font(10),
                           fill=AMBER)
                else:
                    d.text((x0 + 8, gate_y0 + 26), "closed", font=font(10),
                           fill=(70, 90, 110))
                    d.text((x0 + 8, gate_y0 + 44), "skipped", font=font(10),
                           fill=(70, 90, 110))

            # ---- caption
            if bi == len(BOARDS) - 1 and open_now:
                cap = "one driver binary — every board gated at runtime"
            elif not open_now:
                cap = (f"board {bi+1}/4: reading identity registers + efuse "
                       f"on {bname}")
            else:
                cap = (f"board {bi+1}/4: {cid.split()[1]} {cut}-cut rfe={rfe}"
                       f" — gates flipped for this exact board")
            d.text((bx0, 412), cap, font=font(13, True), fill=CYAN)
            imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms, colors=56)
    return 0


if __name__ == "__main__":
    sys.exit(main())
