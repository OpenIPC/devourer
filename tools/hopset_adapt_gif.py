#!/usr/bin/env python3
"""Animated adaptive hopset — 'the band heals', in the DEVOURER live-monitor
style. The companion to hop_pattern_gif.py: same channel set, same parked
interferer, but run long enough to show what the link does about it.

    tools/hopset_adapt_gif.py -o docs/img/hopset_adapt.gif

Same time-frequency view, except the time axis is fixed and fills in left to
right, so the final frame holds the whole story. Three acts: the hops that land
on the interferer's channel are clipped and the receiver scores them; the
transmitter — the schedule authority — issues an authenticated commit naming a
future slot, and both ends swap there; afterwards that channel is no longer
scheduled, except for the occasional keyed probe that goes back to see whether
it recovered. Needs Pillow.
"""
from __future__ import annotations

import argparse
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, GRID, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

CHANS = [36, 40, 44, 48, 52, 56, 60, 64]   # the same 5 GHz hop set
JAM = 52                                    # interferer parks here
JAM_CI = CHANS.index(JAM)

NT = 44          # dwell cells on the fixed time axis
COMMIT_T = 18    # the committed activation slot
COMMIT_HOLD = 6  # frames the commit caption stays up
PROBE_T = 36     # a keyed recovery probe revisits the excluded channel


def schedule(rnd):
    """Keyed-looking draw over the active set, re-keyed at the activation
    slot: before it every channel is a candidate, after it the excluded one is
    only ever reached by the probe."""
    sched, last = [], -1
    for t in range(NT):
        if t == PROBE_T:
            sched.append(JAM_CI)
            last = JAM_CI
            continue
        pool = [c for c in range(len(CHANS))
                if c != last and (t < COMMIT_T or c != JAM_CI)]
        c = rnd.choice(pool)
        sched.append(c)
        last = c
    return sched


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="hopset_adapt.gif")
    ap.add_argument("--hold", type=int, default=16, help="frames on the full picture")
    ap.add_argument("--ms", type=int, default=110)
    args = ap.parse_args()

    rnd = random.Random(0x265)
    sched = schedule(rnd)
    # make the learning act unmistakable: seed a few extra hits on the jammed row
    for t in (2, 7, 13):
        if sched[t] != JAM_CI and sched[t - 1] != JAM_CI and sched[t + 1] != JAM_CI:
            sched[t] = JAM_CI

    nC = len(CHANS)
    padL, padT, padB = 60, 96, 62
    cellw, gh = 14.0, 300
    gw = int(NT * cellw)
    panelW = 214
    W = padL + gw + 24 + panelW
    H = padT + gh + padB
    cellh = gh / nC

    def cell_xy(ti, ci):
        return padL + ti * cellw, padT + (nC - 1 - ci) * cellh

    jam_y = cell_xy(0, JAM_CI)[1]
    commit_x = padL + COMMIT_T * cellw

    imgs = []
    for fi in range(NT + args.hold):
        cursor = min(fi, NT - 1)
        adapted = cursor >= COMMIT_T
        img, d = new_frame(W, H)
        chrome(d, W, H, "ADAPTIVE HOPSET",
               "the receiver scores what it decodes; the transmitter commits "
               "the new schedule — the jammed channel stops being visited", fi)

        d.rectangle([padL, padT, padL + gw, padT + gh], outline=(0, 70, 80))

        # channel axis; the interferer's row is tinted until it goes dark
        for ci, ch in enumerate(CHANS):
            _, y = cell_xy(0, ci)
            d.text((padL - 34, y + cellh / 2 - 7), f"{ch}", font=font(11),
                   fill=WARN if ch == JAM else DIM)
            if ch == JAM:
                d.rectangle([padL, y, padL + gw, y + cellh], fill=(40, 12, 12))
            d.line([padL, y, padL + gw, y], fill=GRID)

        # the excluded span of that row goes dark + hatched as time passes it
        if adapted:
            xr = padL + min(cursor + 1, NT) * cellw
            d.rectangle([commit_x, jam_y, xr, jam_y + cellh], fill=(16, 19, 26))
            hx = commit_x + 6
            while hx < xr:
                d.line([hx, jam_y + 5, hx - 6, jam_y + cellh - 5], fill=(30, 38, 50))
                hx += 12
            if cursor > COMMIT_T + 5:
                tx, ty = commit_x + 16, jam_y + cellh / 2 - 7
                d.rectangle([tx - 5, ty - 3, tx + 92, ty + 15], fill=(16, 19, 26))
                d.text((tx, ty), "not scheduled", font=font(11), fill=DIM)

        # the revealed hop trace
        clipped = 0
        for ti in range(cursor + 1):
            ci = sched[ti]
            x, y = cell_xy(ti, ci)
            probe = ti == PROBE_T
            hit = ci == JAM_CI and not probe
            if hit:
                clipped += 1
            col = AMBER if probe else (WARN if hit else CYAN)
            fade = 0.45 + 0.55 * (ti / max(1, cursor)) if ti < cursor else 1.0
            d.rectangle([x + 2, y + 4, x + cellw - 2, y + cellh - 4],
                        fill=tuple(int(c * fade) for c in col))
            if hit:  # struck through — this dwell was lost
                d.line([x + 2, y + 4, x + cellw - 2, y + cellh - 4], fill=INK)
                d.line([x + 2, y + cellh - 4, x + cellw - 2, y + 4], fill=INK)
            if probe:  # has to read against the hatched, excluded row
                d.rectangle([x + 1, y + 3, x + cellw - 1, y + cellh - 3],
                            outline=AMBER, width=2)
            if ti == cursor and fi < NT:
                d.rectangle([x + 1, y + 3, x + cellw - 1, y + cellh - 3],
                            outline=INK, width=2)
            if ti:
                x0, y0 = cell_xy(ti - 1, sched[ti - 1])
                d.line([x0 + cellw / 2, y0 + cellh / 2,
                        x + cellw / 2, y + cellh / 2], fill=(30, 44, 60))

        # the activation boundary — both ends swap exactly here
        if adapted:
            d.line([commit_x, padT, commit_x, padT + gh], fill=AMBER, width=2)
            d.text((commit_x - 46, 76), "commit → activate", font=font(11),
                   fill=AMBER)
        if cursor >= PROBE_T:
            px = padL + PROBE_T * cellw
            d.line([px + cellw / 2, 90, px + cellw / 2, jam_y - 2], fill=(90, 78, 30))
            d.text((px - 24, 76), "keyed probe", font=font(11), fill=AMBER)

        d.text((padL, padT + gh + 8), "time →   (each cell = one dwell)",
               font=font(11), fill=DIM)

        # act brackets under the axis
        by = padT + gh + 30

        def bracket(t0, t1, label, col):
            x0, x1 = padL + t0 * cellw, padL + t1 * cellw
            d.line([x0, by, x1, by], fill=col)
            d.line([x0, by - 4, x0, by], fill=col)
            d.line([x1, by - 4, x1, by], fill=col)
            d.text(((x0 + x1) / 2 - 3.2 * len(label), by + 5), label,
                   font=font(11), fill=col)

        bracket(0, COMMIT_T, "LEARNING", WARN if not adapted else DIM)
        if adapted:
            bracket(COMMIT_T, min(cursor + 1, NT), "ADAPTED", OK)

        # readout
        x0 = padL + gw + 22
        d.text((x0, padT - 2), "LIVE READOUT", font=font(12), fill=CYAN)
        y = padT + 24

        def line(lbl, val, c=INK):
            nonlocal y
            d.text((x0, y), lbl, font=font(11), fill=DIM)
            d.text((x0 + 96, y - 3), val, font=font(15, True), fill=c)
            y += 30

        mask = "".join("0" if (adapted and ci == JAM_CI) else "1"
                       for ci in range(nC))
        line("generation", "2" if adapted else "1", AMBER if adapted else INK)
        line("hop set", mask, OK if adapted else INK)
        line("active", f"{nC - 1 if adapted else nC}/{nC}", INK)
        line("floor", "3", DIM)
        line("dwells lost", f"{clipped}", WARN if clipped else OK)

        if not adapted:
            st, sc = "LEARNING", WARN
        elif cursor < COMMIT_T + COMMIT_HOLD:
            st, sc = "COMMITTED", AMBER
        else:
            st, sc = "ADAPTED", OK
        d.rectangle([x0, y, x0 + 190, y + 30], outline=sc, width=2)
        d.ellipse([x0 + 8, y + 10, x0 + 18, y + 20], fill=sc)
        d.text((x0 + 28, y + 6), st, font=font(14, True), fill=sc)
        y += 44

        if st == "LEARNING":
            cap = ("the receiver scores every", "dwell it decodes and asks",
                   "for ch 52 to be dropped —", "it proposes, it never acts.")
        elif st == "COMMITTED":
            cap = ("the transmitter owns the", "schedule: a signed commit",
                   "names a future slot, so", "both ends swap together.")
        else:
            cap = ("ch 52 is no longer", "scheduled — and a keyed",
                   "probe still revisits it,", "so exclusion can be undone.")
        for ln in cap:
            d.text((x0, y), ln, font=font(11), fill=INK)
            y += 15

        imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms, colors=48)
    return 0


if __name__ == "__main__":
    sys.exit(main())
