#!/usr/bin/env python3
"""Animated FWDL — 'installing the firmware into the WCPU', in the DEVOURER
live-monitor style.

    tools/fwdl_gif.py -o docs/img/fwdl.gif

The firmware blob ships inside the driver. FWDL parses it into sections (a
header, then instruction- and data-memory images), streams each section through
the chip's download window into the WCPU's IMEM/DMEM banks, verifies a
checksum, releases the WCPU from reset, and polls a ready latch until the
firmware announces itself alive. The animation walks the happy path — if any
stage failed, the chip would simply be silent. Needs Pillow.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

# (label, bank) — bank None = header, consumed at the window.
SECTIONS = [
    ("HDR",     None),
    ("IMEM S0", "imem"),
    ("IMEM S1", "imem"),
    ("DMEM S0", "dmem"),
    ("DMEM S1", "dmem"),
]

W, H = 900, 470
SEC_X0, SEC_X1 = 40, 250          # left column: blob / section boxes
SEC_H, SEC_GAP = 38, 10
SEC_Y0 = 116
WIN_X0, WIN_X1, WIN_Y0, WIN_Y1 = 318, 428, 200, 244   # download window
CPU_X0, CPU_X1, CPU_Y0, CPU_Y1 = 470, 862, 100, 388   # WCPU box
BAR_X0, BAR_X1 = 588, 832
IMEM_Y0, IMEM_Y1 = 146, 168
DMEM_Y0, DMEM_Y1 = 186, 208
CHK_Y0, CHK_Y1 = 232, 260
CORE = (500, 286, 664, 348)
LATCH = (694, 286, 832, 348)
CAP_Y = 432


def sec_rect(i):
    y = SEC_Y0 + i * (SEC_H + SEC_GAP)
    return (SEC_X0, y, SEC_X1, y + SEC_H)


def lerp(a, b, t):
    return a + (b - a) * t


def draw_section(d, cx, cy, label, hot=False):
    w, h = (SEC_X1 - SEC_X0) / 2, SEC_H / 2
    d.rectangle([cx - w, cy - h, cx + w, cy + h],
                outline=AMBER if hot else (90, 110, 135),
                fill=(30, 26, 10) if hot else (16, 20, 30), width=2)
    d.text((cx - w + 10, cy - 8), label, font=font(12, True),
           fill=AMBER if hot else INK)


def draw_bank(d, name, y0, y1, frac, active):
    d.text((508, y0 + 2), name, font=font(12, True),
           fill=CYAN if active else DIM)
    d.rectangle([BAR_X0, y0, BAR_X1, y1], outline=(70, 90, 110))
    if frac > 0:
        d.rectangle([BAR_X0 + 2, y0 + 2,
                     BAR_X0 + 2 + (BAR_X1 - BAR_X0 - 4) * min(frac, 1.0),
                     y1 - 2], fill=(0, 110, 120) if frac < 1 else OK)
    d.text((BAR_X1 - 44, y0 + 3), f"{int(frac * 100):3d}%", font=font(11),
           fill=INK if frac > 0 else DIM)


def draw_tick(d, x, y, col=OK):
    d.line([x, y + 5, x + 5, y + 10], fill=col, width=3)
    d.line([x + 5, y + 10, x + 14, y - 2], fill=col, width=3)


def draw_scene(d, tick, *, split, moving, done_secs, imem, dmem,
               chk, core_on, ready, poll_x, caption):
    """One frame of the whole scene.  moving = (label, cx, cy, hot) or None."""
    # ---- left: blob or the split section stack -------------------------
    d.text((SEC_X0, 92), "firmware blob (driver)", font=font(11), fill=DIM)
    if not split:
        top, bot = sec_rect(0), sec_rect(len(SECTIONS) - 1)
        d.rectangle([SEC_X0, top[1], SEC_X1, bot[3]],
                    outline=(90, 110, 135), fill=(16, 20, 30), width=2)
        d.text((SEC_X0 + 14, (top[1] + bot[3]) // 2 - 8), "RTL FW IMAGE",
               font=font(13, True), fill=INK)
    else:
        for i, (label, _) in enumerate(SECTIONS):
            if i in done_secs or (moving and moving[0] == label):
                x0, y0, x1, y1 = sec_rect(i)          # gone: ghost outline
                d.rectangle([x0, y0, x1, y1], outline=(38, 48, 62))
                continue
            x0, y0, x1, y1 = sec_rect(i)
            draw_section(d, (x0 + x1) / 2, (y0 + y1) / 2, label)

    # ---- middle: download window ---------------------------------------
    d.text((WIN_X0, WIN_Y0 - 22), "download window", font=font(11), fill=DIM)
    win_hot = moving is not None
    d.rectangle([WIN_X0, WIN_Y0, WIN_X1, WIN_Y1],
                outline=CYAN if win_hot else (70, 90, 110), width=2)
    for k in range(3):                                 # port slats
        yy = WIN_Y0 + 10 + k * 12
        d.line([WIN_X0 + 12, yy, WIN_X1 - 12, yy],
               fill=(0, 120, 130) if win_hot else (40, 55, 70), width=2)

    # ---- right: WCPU box ------------------------------------------------
    d.rectangle([CPU_X0, CPU_Y0, CPU_X1, CPU_Y1], outline=(0, 110, 120),
                width=2)
    d.text((CPU_X0 + 14, CPU_Y0 + 10), "WCPU", font=font(14, True), fill=CYAN)
    d.text((CPU_X0 + 76, CPU_Y0 + 13), "(MCU inside the MAC)", font=font(10),
           fill=DIM)
    draw_bank(d, "IMEM", IMEM_Y0, IMEM_Y1, imem,
              moving is not None and moving[3] == "imem")
    draw_bank(d, "DMEM", DMEM_Y0, DMEM_Y1, dmem,
              moving is not None and moving[3] == "dmem")

    # checksum row
    ch_col = OK if chk else (70, 90, 110)
    d.rectangle([500, CHK_Y0, BAR_X1, CHK_Y1], outline=ch_col, width=2)
    d.text((512, CHK_Y0 + 6), "CHECKSUM", font=font(12, True),
           fill=OK if chk else DIM)
    if chk:
        draw_tick(d, 620, CHK_Y0 + 8)
        d.text((648, CHK_Y0 + 6), "OK", font=font(12, True), fill=OK)
    else:
        d.text((620, CHK_Y0 + 6), "pending...", font=font(11), fill=DIM)

    # WCPU core
    cc = CYAN if core_on else (55, 70, 88)
    d.rectangle(CORE, outline=cc, width=2,
                fill=(0, 44, 50) if core_on else (14, 18, 26))
    d.text((CORE[0] + 12, CORE[1] + 8), "WCPU CORE", font=font(12, True),
           fill=CYAN if core_on else DIM)
    d.text((CORE[0] + 12, CORE[1] + 30),
           "RUNNING" if core_on else "IN RESET", font=font(11),
           fill=CYAN if core_on else WARN)

    # FW READY latch
    if ready == 2:
        lc, led, txt = OK, OK, "SET"
    elif ready == 1:
        blink = tick % 4 < 2
        lc = (70, 90, 110)
        led = AMBER if blink else (60, 55, 25)
        txt = "poll..."
    else:
        lc, led, txt = (70, 90, 110), (40, 50, 62), "clear"
    d.rectangle(LATCH, outline=lc, width=2)
    d.text((LATCH[0] + 12, LATCH[1] + 8), "FW READY", font=font(12, True),
           fill=OK if ready == 2 else DIM)
    d.ellipse([LATCH[0] + 14, LATCH[1] + 32, LATCH[0] + 28, LATCH[1] + 46],
              fill=led)
    d.text((LATCH[0] + 38, LATCH[1] + 31), txt, font=font(11),
           fill=OK if ready == 2 else DIM)
    if poll_x is not None:                       # host poll pulse (dashed)
        py = (LATCH[1] + LATCH[3]) // 2 + 62
        for xx in range(SEC_X0, CPU_X0 - 16, 14):
            d.line([xx, py, xx + 7, py], fill=(0, 80, 90))
        d.ellipse([poll_x - 4, py - 4, poll_x + 4, py + 4], fill=AMBER)
        d.text((SEC_X0, py + 10), "host polls the ready bit over USB",
               font=font(10), fill=DIM)

    # flying section
    if moving:
        label, cx, cy, _bank = moving
        draw_section(d, cx, cy, label, hot=True)

    # caption
    d.text((SEC_X0, CAP_Y), caption, font=font(13, True), fill=CYAN)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="fwdl.gif")
    ap.add_argument("--ms", type=int, default=100)
    args = ap.parse_args()

    imgs = []
    tick = [0]

    def frame(**kw):
        img, d = new_frame(W, H)
        chrome(d, W, H, "FWDL — FIRMWARE DOWNLOAD",
               "blob -> sections -> download window -> IMEM/DMEM -> checksum "
               "-> release from reset -> ready", tick[0])
        draw_scene(d, tick[0], **kw)
        imgs.append(img)
        tick[0] += 1

    base = dict(split=False, moving=None, done_secs=set(), imem=0.0, dmem=0.0,
                chk=False, core_on=False, ready=0, poll_x=None)

    # stage 1 — the blob, whole
    for _ in range(8):
        frame(**base, caption="1. the firmware image ships inside the driver "
                              "(a C array in the vendor tree)")
    # stage 2 — parse into sections
    base["split"] = True
    for _ in range(10):
        frame(**base, caption="2. parse the blob: header + sections bound for "
                              "IMEM (code) and DMEM (data)")

    # stage 3 — stream each section through the download window
    per = 9
    fills = {"imem": 0.0, "dmem": 0.0}
    done = set()
    for i, (label, bank) in enumerate(SECTIONS):
        sx0, sy0, sx1, sy1 = sec_rect(i)
        scx, scy = (sx0 + sx1) / 2, (sy0 + sy1) / 2
        wcx, wcy = (WIN_X0 + WIN_X1) / 2, (WIN_Y0 + WIN_Y1) / 2
        if bank == "imem":
            tcx, tcy = (BAR_X0 + BAR_X1) / 2, (IMEM_Y0 + IMEM_Y1) / 2
        elif bank == "dmem":
            tcx, tcy = (BAR_X0 + BAR_X1) / 2, (DMEM_Y0 + DMEM_Y1) / 2
        else:
            tcx, tcy = wcx, wcy
        for f in range(per):
            t = f / (per - 1)
            if t <= 0.5 or bank is None:
                u = min(t / 0.5, 1.0)
                cx, cy = lerp(scx, wcx, u), lerp(scy, wcy, u)
            else:
                u = (t - 0.5) / 0.5
                cx, cy = lerp(wcx, tcx, u), lerp(wcy, tcy, u)
                fills[bank] = min(fills[bank] + 0.5 / (per / 2), (0.5 * sum(
                    1 for l, b in SECTIONS if b == bank and
                    (l in done or l == label))))
            cap = ("3. stream section %s through the download window"
                   % label) if bank else \
                  "3. header first — it tells the loader where each section goes"
            frame(**{**base, "moving": (label, cx, cy, bank),
                     "done_secs": done, "imem": fills["imem"],
                     "dmem": fills["dmem"]}, caption=cap)
        done = done | {i}
        base.update(done_secs=done, imem=fills["imem"], dmem=fills["dmem"])

    base.update(imem=1.0, dmem=1.0)
    # stage 4 — checksum
    for f in range(10):
        base["chk"] = f >= 3
        frame(**base, caption="4. verify the checksum over what landed in "
                              "IMEM/DMEM")
    # stage 5 — release from reset
    for f in range(10):
        base["core_on"] = f >= 3
        frame(**base, caption="5. release the WCPU from reset — the core "
                              "starts executing IMEM")
    # stage 6 — poll ready
    for f in range(16):
        base["ready"] = 1 if f < 10 else 2
        px = SEC_X0 + (f % 10) * ((CPU_X0 - 30 - SEC_X0) / 9)
        base["poll_x"] = px if f < 10 else CPU_X0 - 30
        cap = ("6. poll the FW READY latch..." if f < 10 else
               "6. firmware announces itself alive — the chip is now a "
               "running computer")
        frame(**base, caption=cap)
    # hold the finale
    for _ in range(6):
        frame(**base, caption="6. firmware announces itself alive — the chip "
                              "is now a running computer")

    save_gif(imgs, args.out, ms=args.ms, colors=56)
    return 0


if __name__ == "__main__":
    sys.exit(main())
