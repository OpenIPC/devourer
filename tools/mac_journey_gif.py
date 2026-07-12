#!/usr/bin/env python3
"""Animated MAC journey — 'a packet's trip through DMAC and CMAC', in the
DEVOURER live-monitor style.

    tools/mac_journey_gif.py -o docs/img/mac_journey.gif

The MAC has two halves: the DMAC (data MAC — the plumbing: DLE page allocator,
DMA engines, HFC flow-control credits) and the CMAC (control MAC — the protocol
brain: queue selection, ACK machinery, RX filters). The animation follows one
TX frame down (host → USB bulk → TX descriptor → DLE pages + HFC credits →
CMAC → antenna) and then the RX direction up (antenna → CMAC filter, which
drops one frame → RX-DMA aggregation buffer batching frames into one bulk URB
→ host). Needs Pillow.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

W, H = 900, 505
# horizontal bands (y0, y1, name, note)
BANDS = [
    (88, 126, "HOST", "driver / devourer"),
    (132, 162, "USB", "bulk endpoints"),
    (168, 292, "DMAC", "plumbing: DLE pages,"),
    (298, 368, "CMAC", "protocol brain:"),
    (374, 438, "RF", "antenna"),
]
BAND_NOTE2 = {"DMAC": "DMA, HFC credits", "CMAC": "queues, ACK, filters"}
TXL, RXL = 330, 770                  # TX lane (down) / RX lane (up) centres
# DMAC internals
HFC_X = 210
PAGE_X0, PAGE_Y0 = 255, 190         # DLE page grid
PCOLS, PROWS, PSZ = 6, 3, 26
AGG_X0, AGG_X1 = 560, 800           # RX-DMA aggregation buffer
AGG_Y0, AGG_Y1 = 226, 262
CAP_Y = 464

BAND_MID = {n: (y0 + y1) // 2 for y0, y1, n, _ in BANDS}


def draw_bands(d, tx_phase):
    for y0, y1, name, note in BANDS:
        d.line([34, y0, W - 34, y0], fill=(22, 34, 46))
        d.line([34, y1, W - 34, y1], fill=(22, 34, 46))
        col = CYAN if name in ("DMAC", "CMAC") else INK
        d.text((44, y0 + 4), name, font=font(13, True), fill=col)
        d.text((44, y0 + 24), note, font=font(10), fill=DIM)
        if name in BAND_NOTE2:
            d.text((44, y0 + 38), BAND_NOTE2[name], font=font(10), fill=DIM)
    # lane direction hints (kept clear of the tokens' travel corridor)
    d.text((TXL - 96, 92), "TX", font=font(12, True), fill=AMBER)
    d.polygon([(TXL - 74, 96), (TXL - 66, 96), (TXL - 70, 106)], fill=AMBER)
    d.text((RXL - 116, 92), "RX", font=font(12, True), fill=OK)
    d.polygon([(RXL - 94, 106), (RXL - 86, 106), (RXL - 90, 96)], fill=OK)


def draw_hfc(d, credits, total=8):
    x0, x1 = HFC_X, HFC_X + 26
    y1, y0 = 282, 186
    d.text((x0 - 2, y0 - 16), "HFC", font=font(10), fill=DIM)
    seg = (y1 - y0) / total
    for k in range(total):
        yy = y1 - (k + 1) * seg
        if k < credits:
            d.rectangle([x0, yy + 2, x1, yy + seg - 2], fill=(0, 110, 120))
        else:
            d.rectangle([x0, yy + 2, x1, yy + seg - 2],
                        outline=(45, 60, 75))
    d.text((x0 + 2, y1 + 4), f"{credits}/{total}", font=font(10),
           fill=INK if credits > total // 2 else AMBER)


def draw_pages(d, used):
    d.text((PAGE_X0, PAGE_Y0 - 16), "DLE page pool", font=font(10), fill=DIM)
    for k in range(PCOLS * PROWS):
        r, c = divmod(k, PCOLS)
        x = PAGE_X0 + c * (PSZ + 4)
        y = PAGE_Y0 + r * (PSZ + 4)
        if k in used:
            d.rectangle([x, y, x + PSZ, y + PSZ], fill=(120, 90, 10),
                        outline=AMBER)
        else:
            d.rectangle([x, y, x + PSZ, y + PSZ], outline=(45, 60, 75))


def draw_agg(d, filled, hot=False):
    d.text((AGG_X0, AGG_Y0 - 16), "RX-DMA aggregation buffer", font=font(10),
           fill=DIM)
    d.rectangle([AGG_X0, AGG_Y0, AGG_X1, AGG_Y1],
                outline=OK if hot else (70, 90, 110))
    n = 4
    sw = (AGG_X1 - AGG_X0) / n
    for k in range(n):
        x = AGG_X0 + k * sw
        if k < filled:
            d.rectangle([x + 4, AGG_Y0 + 5, x + sw - 4, AGG_Y1 - 5],
                        fill=(0, 90, 60), outline=OK)
            d.text((x + 12, AGG_Y0 + 9), f"frm{k}", font=font(10), fill=OK)
        else:
            d.rectangle([x + 4, AGG_Y0 + 5, x + sw - 4, AGG_Y1 - 5],
                        outline=(45, 60, 75))


def draw_cmac_boxes(d, tx_hot, rx_hot):
    d.rectangle([255, 308, 470, 358], outline=AMBER if tx_hot else (70, 90, 110),
                width=2 if tx_hot else 1)
    d.text((266, 314), "queue select / ACK", font=font(11, True),
           fill=AMBER if tx_hot else DIM)
    d.text((266, 334), "EDCA access, retries", font=font(10), fill=DIM)
    d.rectangle([560, 308, 800, 358], outline=OK if rx_hot else (70, 90, 110),
                width=2 if rx_hot else 1)
    d.text((572, 314), "RX filter / addr match", font=font(11, True),
           fill=OK if rx_hot else DIM)
    d.text((572, 334), "monitor mode opens this up", font=font(10), fill=DIM)


def draw_frame_token(d, cx, cy, label="FRAME", desc=True, col=AMBER, w=88):
    d.rectangle([cx - w / 2, cy - 11, cx + w / 2, cy + 11],
                outline=col, fill=(16, 20, 30), width=2)
    if desc:                                   # TX-descriptor header segment
        d.rectangle([cx - w / 2 + 2, cy - 9, cx - w / 2 + 22, cy + 9],
                    fill=(120, 90, 10))
        d.text((cx - w / 2 + 3, cy - 6), "TD", font=font(10, True), fill=INK)
        d.text((cx - w / 2 + 28, cy - 7), label, font=font(11, True), fill=col)
    else:
        tw = d.textlength(label, font=font(11, True))
        d.text((cx - tw / 2, cy - 7), label, font=font(11, True), fill=col)


def draw_antenna(d, x, radiate, tick):
    y = 430
    d.line([x, y, x, y - 34], fill=INK, width=2)
    d.line([x - 10, y - 22, x, y - 34], fill=INK, width=2)
    d.line([x + 10, y - 22, x, y - 34], fill=INK, width=2)
    if radiate:
        for r in (10, 18, 26):
            rr = r + (tick % 3) * 2
            d.arc([x - rr, y - 40 - rr, x + rr, y - 40 + rr], 200, 340,
                  fill=AMBER)


def lerp(a, b, t):
    return a + (b - a) * t


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="mac_journey.gif")
    ap.add_argument("--ms", type=int, default=105)
    args = ap.parse_args()

    imgs = []

    # ---------------- TX phase (frames 0..55) ----------------------------
    TX_STEPS = [   # (frames, y-from, y-to, caption)
        (7, BAND_MID["HOST"], BAND_MID["HOST"],
         "1. TX: the host prepends a TX descriptor (TD): rate, power, queue"),
        (7, BAND_MID["HOST"], BAND_MID["USB"],
         "2. one bulk URB carries descriptor + frame across USB"),
        (14, BAND_MID["USB"], 230,
         "3. DMAC: the DLE allocates pages for the frame — HFC credits must "
         "cover it first"),
        (10, 230, 332,
         "4. CMAC: queue selection, EDCA channel access, ACK machinery"),
        (9, 332, 408,
         "5. on air — pages free, HFC credits refill"),
        (5, 408, 408,
         "5. on air — pages free, HFC credits refill"),
    ]
    tick = 0
    for si, (nf, yf, yt, cap) in enumerate(TX_STEPS):
        for f in range(nf):
            t = f / max(nf - 1, 1)
            img, d = new_frame(W, H)
            chrome(d, W, H, "MAC JOURNEY — DMAC / CMAC",
                   "TX: host -> USB -> DMAC (DLE pages, HFC) -> CMAC -> air   "
                   "then RX back up through filter + RX-DMA agg", tick)
            draw_bands(d, True)

            in_dmac = si == 2
            after_dmac = si >= 3
            done_dmac = si >= 4
            pages = set()
            credits = 8
            if in_dmac:
                npg = min(4, 1 + int(t * 4))
                pages = set(range(npg))
                credits = 8 - npg
            elif si == 3:
                pages = set(range(4))
                credits = 4
            elif done_dmac:
                back = min(4, int(t * 5)) if si == 4 else 4
                pages = set(range(4 - back))
                credits = 4 + back
            draw_hfc(d, credits)
            draw_pages(d, pages)
            draw_agg(d, 0)
            draw_cmac_boxes(d, tx_hot=si == 3, rx_hot=False)
            radiate = si >= 4 and (si == 5 or t > 0.4)
            draw_antenna(d, TXL, radiate, tick)
            draw_antenna(d, RXL, False, tick)

            if not radiate:
                cy = lerp(yf, yt, t)
                draw_frame_token(d, TXL, cy)
            if si == 2:
                d.text((368, 174), "host may only push while credits "
                       "cover the frame", font=font(10), fill=AMBER)
            d.text((40, CAP_Y), cap, font=font(13, True), fill=CYAN)
            imgs.append(img)
            tick += 1

    # ---------------- RX phase (frames ..110) -----------------------------
    # four frames rise from the antenna; #1 is dropped by the CMAC filter;
    # survivors collect in the aggregation buffer; one URB goes up.
    RX_RISE, RX_AGG, RX_URB = 26, 10, 16
    y_ant, y_flt, y_agg = 408, 332, (AGG_Y0 + AGG_Y1) // 2
    starts = [0, 5, 10, 15]           # per-frame launch offsets
    for f in range(RX_RISE + RX_AGG + RX_URB):
        img, d = new_frame(W, H)
        chrome(d, W, H, "MAC JOURNEY — DMAC / CMAC",
               "TX: host -> USB -> DMAC (DLE pages, HFC) -> CMAC -> air   "
               "then RX back up through filter + RX-DMA agg", tick)
        draw_bands(d, False)
        draw_hfc(d, 8)
        draw_pages(d, set())

        filled = 0
        drop_flash = False
        for k, s in enumerate(starts):
            lf = f - s                              # local frame index
            if lf < 0:
                continue
            rise = 8                                # antenna -> filter
            hop = 6                                 # filter -> agg buffer
            if lf < rise:
                cy = lerp(y_ant, y_flt, lf / (rise - 1))
                draw_frame_token(d, RXL, cy, f"rx{k}", desc=False, col=OK,
                                 w=52)
            elif k == 1 and lf < rise + 4:          # the filtered-out frame
                drop_flash = True
                d.line([RXL - 12, y_flt - 12, RXL + 12, y_flt + 12],
                       fill=WARN, width=3)
                d.line([RXL - 12, y_flt + 12, RXL + 12, y_flt - 12],
                       fill=WARN, width=3)
                d.text((RXL - 130, 361), "dropped by filter",
                       font=font(10), fill=WARN)
            elif k != 1:
                if lf < rise + hop:
                    u = (lf - rise) / (hop - 1)
                    cy = lerp(y_flt, y_agg, u)
                    draw_frame_token(d, RXL, cy, f"rx{k}", desc=False,
                                     col=OK, w=52)
                else:
                    filled += 1
        filled = min(filled, 3)

        urb_f = f - (RX_RISE + RX_AGG)
        agg_hot = filled == 3 and urb_f < 0
        draw_agg(d, 0 if urb_f >= 3 else filled, hot=agg_hot)
        draw_cmac_boxes(d, tx_hot=False, rx_hot=f < RX_RISE)
        draw_antenna(d, TXL, False, tick)
        draw_antenna(d, RXL, f < 16, tick)

        if urb_f >= 0:                              # one URB up to the host
            t = min(urb_f / (RX_URB - 4), 1.0)
            cy = lerp(y_agg - 30, BAND_MID["HOST"], t)
            draw_frame_token(d, RXL, cy, "URB: 3 frames", desc=False,
                             col=CYAN, w=120)
            cap = ("8. one bulk URB completion carries many frames up to "
                   "the host")
        elif f < RX_RISE:
            cap = ("6. RX: the CMAC filter judges each frame — one fails "
                   "address match and is dropped")
        else:
            cap = ("7. RX-DMA batches survivors into the aggregation buffer")
        d.text((40, CAP_Y), cap, font=font(13, True), fill=CYAN)
        imgs.append(img)
        tick += 1

    save_gif(imgs, args.out, ms=args.ms, colors=56)
    return 0


if __name__ == "__main__":
    sys.exit(main())
