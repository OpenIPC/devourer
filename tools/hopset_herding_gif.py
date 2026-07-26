#!/usr/bin/env python3
"""Animated anti-herding — 'the floor is not negotiable', in the DEVOURER
live-monitor style. The counterweight to hopset_adapt_gif.py: what the same
machinery does when the interferer is not furniture but an adversary that
follows the exclusions.

    tools/hopset_herding_gif.py -o docs/img/hopset_herding.gif

Four channels and a floor of three — the geometry of the bench run this is
drawn from. The link excludes the jammed channel and recovers, exactly as in
the stationary case. Then the interferer moves onto a channel that is still
active, delivery collapses again, the receiver proposes a second exclusion —
and the transmitter refuses it, because granting it would leave two channels
and one more move would leave one. The link eats the damage on three channels
instead of being walked down onto one of the attacker's choosing. Losses keep
climbing to the last frame: this ending is a pass, not a win, and the picture
says so. Needs Pillow.
"""
from __future__ import annotations

import argparse
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, GRID, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

CHANS = [36, 40, 44, 48]   # the 4-channel base the herding run was measured on
FLOOR = 3                  # the schedule never shrinks past this
JAM_A, JAM_B = 1, 2        # the interferer starts on ch 40, moves to ch 44

NT = 48
COMMIT_T = 14   # ch 40 excluded here
MOVE_T = 28     # the interferer follows, onto ch 44
REFUSE_T = 38   # the second proposal is answered with a refusal


def schedule(rnd):
    sched, last = [], -1
    for t in range(NT):
        pool = [c for c in range(len(CHANS))
                if c != last and (t < COMMIT_T or c != JAM_A)]
        c = rnd.choice(pool)
        sched.append(c)
        last = c
    return sched


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="hopset_herding.gif")
    ap.add_argument("--hold", type=int, default=16)
    ap.add_argument("--ms", type=int, default=110)
    args = ap.parse_args()

    sched = schedule(random.Random(0x11e))
    # make each act legible: guarantee hits early, and again after the move
    for t in (3, 9):
        if sched[t - 1] != JAM_A and sched[t + 1] != JAM_A:
            sched[t] = JAM_A
    for t in (MOVE_T + 2, MOVE_T + 7, MOVE_T + 13, MOVE_T + 17):
        if sched[t - 1] != JAM_B and sched[t + 1] != JAM_B:
            sched[t] = JAM_B

    nC = len(CHANS)
    padL, padT, padB = 60, 96, 62
    cellw, gh = 13.0, 260
    gw = int(NT * cellw)
    panelW = 230
    W = padL + gw + 24 + panelW
    H = padT + gh + padB
    cellh = gh / nC

    def cell_xy(ti, ci):
        return padL + ti * cellw, padT + (nC - 1 - ci) * cellh

    def tx(t):
        return padL + t * cellw

    imgs = []
    for fi in range(NT + args.hold):
        cursor = min(fi, NT - 1)
        seen = min(cursor + 1, NT)
        excluded = cursor >= COMMIT_T
        moved = cursor >= MOVE_T
        refused = cursor >= REFUSE_T
        img, d = new_frame(W, H)
        chrome(d, W, H, "ANTI-HERDING",
               "an interferer that follows the exclusions — the schedule "
               "refuses to shrink past its floor", fi)

        d.rectangle([padL, padT, padL + gw, padT + gh], outline=(0, 70, 80))

        jam_ci = JAM_B if moved else JAM_A
        for ci, ch in enumerate(CHANS):
            _, y = cell_xy(0, ci)
            d.text((padL - 34, y + cellh / 2 - 7), f"{ch}", font=font(11),
                   fill=WARN if ci == jam_ci else DIM)
            d.line([padL, y, padL + gw, y], fill=GRID)

        # where the interferer has been, span by span
        ya = cell_xy(0, JAM_A)[1]
        d.rectangle([padL, ya, tx(min(seen, MOVE_T)), ya + cellh], fill=(40, 12, 12))
        if moved:
            yb = cell_xy(0, JAM_B)[1]
            d.rectangle([tx(MOVE_T), yb, tx(seen), yb + cellh], fill=(40, 12, 12))

        # ch 40 is excluded from the commit on — dark, hatched, never visited
        if excluded:
            xr = tx(seen)
            d.rectangle([tx(COMMIT_T), ya, xr, ya + cellh], fill=(16, 19, 26))
            hx = tx(COMMIT_T) + 6
            while hx < xr:
                d.line([hx, ya + 6, hx - 6, ya + cellh - 6], fill=(30, 38, 50))
                hx += 12
            if cursor > COMMIT_T + 5:
                lx, ly = tx(COMMIT_T) + 16, ya + cellh / 2 - 7
                d.rectangle([lx - 5, ly - 3, lx + 92, ly + 15], fill=(16, 19, 26))
                d.text((lx, ly), "not scheduled", font=font(11), fill=DIM)

        # the trace
        lost = 0
        for ti in range(seen):
            ci = sched[ti]
            x, y = cell_xy(ti, ci)
            hit = ci == (JAM_B if ti >= MOVE_T else JAM_A)
            if hit:
                lost += 1
            fade = 0.45 + 0.55 * (ti / max(1, cursor)) if ti < cursor else 1.0
            d.rectangle([x + 2, y + 8, x + cellw - 2, y + cellh - 8],
                        fill=tuple(int(c * fade) for c in (WARN if hit else CYAN)))
            if hit:
                d.line([x + 2, y + 8, x + cellw - 2, y + cellh - 8], fill=INK)
                d.line([x + 2, y + cellh - 8, x + cellw - 2, y + 8], fill=INK)
            if ti == cursor and fi < NT:
                d.rectangle([x + 1, y + 7, x + cellw - 1, y + cellh - 7],
                            outline=INK, width=2)
            if ti:
                x0, y0 = cell_xy(ti - 1, sched[ti - 1])
                d.line([x0 + cellw / 2, y0 + cellh / 2,
                        x + cellw / 2, y + cellh / 2], fill=(30, 44, 60))

        # the three moments, marked on the axis
        if excluded:
            d.line([tx(COMMIT_T), padT, tx(COMMIT_T), padT + gh], fill=AMBER, width=2)
            d.text((tx(COMMIT_T) - 30, 76), "exclude ch 40", font=font(11), fill=AMBER)
        if moved:
            mx = tx(MOVE_T)
            yy = padT
            while yy < padT + gh:   # dashed: the adversary acts, we did not
                d.line([mx, yy, mx, yy + 5], fill=WARN)
                yy += 10
            d.text((mx - 42, 76), "interferer follows", font=font(11), fill=WARN)
        if refused:
            d.line([tx(REFUSE_T), padT, tx(REFUSE_T), padT + gh], fill=(120, 100, 40))
            d.text((tx(REFUSE_T) - 34, 76), "proposal refused", font=font(11),
                   fill=AMBER)

        d.text((padL, padT + gh + 8), "time →   (each cell = one dwell)",
               font=font(11), fill=DIM)

        by = padT + gh + 30

        def bracket(t0, t1, label, col):
            x0, x1 = tx(t0), tx(t1)
            d.line([x0, by, x1, by], fill=col)
            d.line([x0, by - 4, x0, by], fill=col)
            d.line([x1, by - 4, x1, by], fill=col)
            d.text(((x0 + x1) / 2 - 3.2 * len(label), by + 5), label,
                   font=font(11), fill=col)

        bracket(0, COMMIT_T, "JAMMED", WARN if not excluded else DIM)
        if excluded:
            bracket(COMMIT_T, min(seen, MOVE_T), "ADAPTED",
                    OK if not moved else DIM)
        if moved:
            bracket(MOVE_T, seen, "HELD — 3 CHANNELS, ALL OF THEM KEPT", AMBER)

        # readout
        x0 = padL + gw + 22
        d.text((x0, padT - 2), "LIVE READOUT", font=font(12), fill=CYAN)
        y = padT + 24

        def line(lbl, val, c=INK):
            nonlocal y
            d.text((x0, y), lbl, font=font(11), fill=DIM)
            d.text((x0 + 100, y - 3), val, font=font(15, True), fill=c)
            y += 30

        act = nC - 1 if excluded else nC
        line("hop set", "1011" if excluded else "1111", OK if excluded else INK)
        line("active", f"{act}/{nC}", AMBER if act == FLOOR else INK)
        line("floor", f"{FLOOR}", AMBER if act == FLOOR else DIM)
        line("interferer", f"ch {CHANS[jam_ci]}", WARN)
        line("dwells lost", f"{lost}", WARN if lost else OK)

        if refused:
            st, sc = "HELD AT FLOOR", AMBER
        elif moved:
            st, sc = "JAMMED AGAIN", WARN
        elif excluded:
            st, sc = "ADAPTED", OK
        else:
            st, sc = "LEARNING", WARN
        d.rectangle([x0, y, x0 + 200, y + 30], outline=sc, width=2)
        d.ellipse([x0 + 8, y + 10, x0 + 18, y + 20], fill=sc)
        d.text((x0 + 28, y + 6), st, font=font(14, True), fill=sc)
        y += 44

        if st == "LEARNING":
            cap = ("ch 40 keeps losing", "dwells — the receiver",
                   "asks for it to go.")
        elif st == "ADAPTED":
            cap = ("ch 40 is out, and the", "link is clean again —",
                   "so far this is the", "stationary case.")
        elif st == "JAMMED AGAIN":
            cap = ("but it followed. now", "ch 44 is jammed, and",
                   "the receiver wants to", "exclude that too.")
        else:
            cap = ("refused. granting it", "leaves 2, and the next",
                   "move leaves 1. better", "to bleed on three than",
                   "be herded onto one.")
        for ln in cap:
            d.text((x0, y), ln, font=font(11), fill=INK)
            y += 15

        imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms, colors=48)
    return 0


if __name__ == "__main__":
    sys.exit(main())
