#!/usr/bin/env python3
"""Animated adaptive hopset — 'the band heals', in the DEVOURER live-monitor
style. The stationary half of a pair: hopset_herding_gif.py draws the same eight
channels and the same opening act, and ends the other way.

    tools/hopset_adapt_gif.py -o docs/img/hopset_adapt.gif

The hop set and the interferer's channel are the ones from hop_pattern.gif, so
the three figures of the section read as one story. A stationary occupant sits
on ch 52: the receiver scores the dwells it
loses and asks for the channel to be dropped, the transmitter commits the new
schedule at a named future slot, and the channel stops being visited. Then the
occupant goes away — and nothing would ever know, except that keyed recovery
probes go back and look anyway. Three clean probes and the channel is back in
the set.

Deliberately scoped to a *stationary* occupant, which is what most interference
is (a fixed AP, another link, a microwave) and the only case where exclusion is
an unambiguous win. An interferer that follows the exclusions has a different
ending; that is the companion figure, not this one. Needs Pillow.
"""
from __future__ import annotations

import argparse
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, GRID, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

CHANS = [36, 40, 44, 48, 52, 56, 60, 64]   # the hop set of the section's first figure
JAM = 4                    # the occupant sits on ch 52, where it sat there too

NT = 64
COMMIT_T = 16              # ch 52 excluded here
STOP_T = 30                # the occupant goes away
PROBES = (36, 44, 52)      # keyed recovery probes revisit the excluded channel
RESTORE_T = 58             # three clean probes, and it is back in the set


def schedule(rnd):
    sched, last = [], -1
    for t in range(NT):
        if t in PROBES:
            sched.append(JAM)
            last = JAM
            continue
        excluded = COMMIT_T <= t < RESTORE_T
        pool = [c for c in range(len(CHANS))
                if c != last and not (excluded and c == JAM)]
        c = rnd.choice(pool)
        sched.append(c)
        last = c
    return sched


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-o", "--out", default="hopset_adapt.gif")
    ap.add_argument("--hold", type=int, default=16)
    ap.add_argument("--ms", type=int, default=110)
    args = ap.parse_args()

    sched = schedule(random.Random(0x265))
    for t in (3, 9):   # make the opening act unmistakable
        if sched[t - 1] != JAM and sched[t + 1] != JAM:
            sched[t] = JAM

    nC = len(CHANS)
    padL, padT, padB = 60, 96, 62
    cellw, gh = 11.0, 300
    gw = int(NT * cellw)
    panelW = 230
    W = padL + gw + 24 + panelW
    H = padT + gh + padB
    cellh = gh / nC

    def cell_xy(ti, ci):
        return padL + ti * cellw, padT + (nC - 1 - ci) * cellh

    def tx(t):
        return padL + t * cellw

    jam_y = cell_xy(0, JAM)[1]

    imgs = []
    for fi in range(NT + args.hold):
        cursor = min(fi, NT - 1)
        seen = min(cursor + 1, NT)
        excluded = cursor >= COMMIT_T
        stopped = cursor >= STOP_T
        restored = cursor >= RESTORE_T
        img, d = new_frame(W, H)
        chrome(d, W, H, "ADAPTIVE HOPSET",
               "a stationary occupant on ch 52 — the receiver scores what it "
               "decodes, the transmitter commits, the channel stops being "
               "visited", fi)

        d.rectangle([padL, padT, padL + gw, padT + gh], outline=(0, 70, 80))
        for ci, ch in enumerate(CHANS):
            _, y = cell_xy(0, ci)
            d.text((padL - 34, y + cellh / 2 - 7), f"{ch}", font=font(11),
                   fill=WARN if (ci == JAM and not stopped) else DIM)
            d.line([padL, y, padL + gw, y], fill=GRID)

        # the occupant's span, and the excluded span laid over it
        d.rectangle([padL, jam_y, tx(min(seen, STOP_T)), jam_y + cellh],
                    fill=(40, 12, 12))
        if excluded:
            xr = tx(min(seen, RESTORE_T))
            d.rectangle([tx(COMMIT_T), jam_y, xr, jam_y + cellh], fill=(16, 19, 26))
            hx = tx(COMMIT_T) + 6
            while hx < xr:
                d.line([hx, jam_y + 6, hx - 6, jam_y + cellh - 6], fill=(30, 38, 50))
                hx += 12
            if cursor > COMMIT_T + 5:
                lx, ly = tx(COMMIT_T) + 16, jam_y + cellh / 2 - 7
                d.rectangle([lx - 5, ly - 3, lx + 92, ly + 15], fill=(16, 19, 26))
                d.text((lx, ly), "not scheduled", font=font(11), fill=DIM)

        # the trace
        lost = 0
        for ti in range(seen):
            ci = sched[ti]
            x, y = cell_xy(ti, ci)
            probe = ti in PROBES
            hit = ci == JAM and ti < STOP_T and not probe
            if hit:
                lost += 1
            fade = 0.45 + 0.55 * (ti / max(1, cursor)) if ti < cursor else 1.0
            col = AMBER if probe else (WARN if hit else CYAN)
            d.rectangle([x + 2, y + 8, x + cellw - 2, y + cellh - 8],
                        fill=tuple(int(c * fade) for c in col))
            if hit:
                d.line([x + 2, y + 8, x + cellw - 2, y + cellh - 8], fill=INK)
                d.line([x + 2, y + cellh - 8, x + cellw - 2, y + 8], fill=INK)
            if probe:  # has to read against the hatched, excluded row
                d.rectangle([x + 1, y + 7, x + cellw - 1, y + cellh - 7],
                            outline=AMBER, width=2)
                d.line([x + cellw / 2, y + cellh - 6, x + cellw / 2,
                        padT + gh + 4], fill=(90, 78, 30))
            if ti == cursor and fi < NT:
                d.rectangle([x + 1, y + 7, x + cellw - 1, y + cellh - 7],
                            outline=INK, width=2)
            if ti:
                x0, y0 = cell_xy(ti - 1, sched[ti - 1])
                d.line([x0 + cellw / 2, y0 + cellh / 2,
                        x + cellw / 2, y + cellh / 2], fill=(30, 44, 60))

        # the moments, marked on the axis
        if excluded:
            d.line([tx(COMMIT_T), padT, tx(COMMIT_T), padT + gh], fill=AMBER, width=2)
            d.text((tx(COMMIT_T) - 30, 76), "exclude ch 52", font=font(11), fill=AMBER)
        if stopped:
            mx = tx(STOP_T)
            yy = padT
            while yy < padT + gh:
                d.line([mx, yy, mx, yy + 5], fill=DIM)
                yy += 10
            d.text((mx - 34, 76), "occupant stops", font=font(11), fill=DIM)
        if restored:
            d.line([tx(RESTORE_T), padT, tx(RESTORE_T), padT + gh], fill=OK, width=2)
            d.text((tx(RESTORE_T) - 52, 76), "ch 52 restored", font=font(11), fill=OK)

        d.text((padL, padT + gh + 8), "time →   (each cell = one dwell)",
               font=font(11), fill=DIM)
        if cursor >= PROBES[0]:
            d.text((tx(PROBES[0]) - 12, padT + gh + 8), "↑ keyed recovery probes",
                   font=font(11), fill=AMBER)

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
            bracket(COMMIT_T, min(seen, RESTORE_T), "ADAPTED",
                    OK if not restored else DIM)
        if restored:
            bracket(RESTORE_T, seen, "WHOLE AGAIN", OK)

        # readout
        x0 = padL + gw + 22
        d.text((x0, padT - 2), "LIVE READOUT", font=font(12), fill=CYAN)
        y = padT + 24

        def line(lbl, val, c=INK):
            nonlocal y
            d.text((x0, y), lbl, font=font(11), fill=DIM)
            d.text((x0 + 100, y - 3), val, font=font(15, True), fill=c)
            y += 30

        gen = 3 if restored else (2 if excluded else 1)
        out = excluded and not restored
        mask = "".join("0" if (out and ci == JAM) else "1" for ci in range(nC))
        done = sum(1 for p in PROBES if cursor >= p)
        line("generation", f"{gen}", AMBER if excluded else INK)
        line("hop set", mask, OK)
        line("active", f"{nC - 1 if out else nC}/{nC}", AMBER if out else OK)
        line("probes", f"{done}/{len(PROBES)}" if excluded else "—",
             AMBER if 0 < done < len(PROBES) else (OK if done else DIM))
        line("dwells lost", f"{lost}", WARN if lost else OK)

        if restored:
            st, sc = "RESTORED", OK
        elif stopped:
            st, sc = "PROBING", AMBER
        elif excluded:
            st, sc = "ADAPTED", OK
        else:
            st, sc = "LEARNING", WARN
        d.rectangle([x0, y, x0 + 200, y + 30], outline=sc, width=2)
        d.ellipse([x0 + 8, y + 10, x0 + 18, y + 20], fill=sc)
        d.text((x0 + 28, y + 6), st, font=font(14, True), fill=sc)
        y += 44

        if st == "LEARNING":
            cap = ("ch 52 keeps losing", "dwells — the receiver",
                   "asks for it to go.")
        elif st == "ADAPTED":
            cap = ("ch 52 is out, committed", "at a named slot so both",
                   "ends swap together.")
        elif st == "PROBING":
            cap = ("the occupant left, and", "nothing would ever know —",
                   "so keyed probes go back", "and look anyway.")
        else:
            cap = ("three clean probes and", "ch 52 is back in the set.",
                   "exclusion is evidence,", "not a one-way ratchet.")
        for ln in cap:
            d.text((x0, y), ln, font=font(11), fill=INK)
            y += 15

        imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms, colors=48)
    return 0


if __name__ == "__main__":
    sys.exit(main())
