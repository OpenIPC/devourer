#!/usr/bin/env python3
"""Animated anti-herding — 'the floor is not negotiable', in the DEVOURER
live-monitor style. The counterweight to hopset_adapt_gif.py: same eight
channels, same opening act, and what the machinery does when the interferer is
not furniture but an adversary that follows the exclusions.

    tools/hopset_herding_gif.py -o docs/img/hopset_herding.gif

The link excludes the jammed channel and recovers, exactly as in the stationary
case. Then the interferer moves onto a channel that is still active, and the
receiver — correctly, on the evidence it has — asks for that one too. And again.
Every one of those requests is individually reasonable, which is the whole
problem: granted without limit they walk the schedule down to a single frequency
of the attacker's choosing. So the schedule stops shrinking at its floor, the
last proposal is refused, and the link spends the rest of the run bleeding on
three channels rather than being parked on one. The loss counter climbs to the
final frame, because it does: this ending is a pass, not a win.

The floor of three is the shipped one. The walk-down is the policy's behaviour
carried out to the limit; the measured herding run used a four-channel base,
where the floor binds after a single exclusion. Needs Pillow.
"""
from __future__ import annotations

import argparse
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from monitor_style import (AMBER, CYAN, DIM, GRID, INK, OK, WARN, chrome, font,
                           new_frame, save_gif)

CHANS = [36, 40, 44, 48, 52, 56, 60, 64]
FLOOR = 3        # the schedule never shrinks past this
NT = 64
STAGE = 11       # dwells the interferer spends on a channel before it is dropped

# where the interferer sits, stage by stage: it starts on ch 52 — the channel it
# occupies in both of the section's other figures — and then follows.
JAM_SEQ = [4, 2, 7, 1, 5, 3]          # 52, 44, 64, 40, 56, then 48
COMMITS = [STAGE * (k + 1) for k in range(5)]   # 11, 22, 33, 44, 55
REFUSE_T = 59                          # the sixth request, answered with a no


def jam_at(t):
    return JAM_SEQ[min(t // STAGE, len(JAM_SEQ) - 1)]


def excluded_at(t):
    """The channels dropped by dwell t — one per commit, in the order the
    interferer forced them."""
    return {JAM_SEQ[k] for k, c in enumerate(COMMITS) if t >= c}


def schedule(rnd):
    sched, last = [], -1
    for t in range(NT):
        out = excluded_at(t)
        pool = [c for c in range(len(CHANS)) if c != last and c not in out]
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
    # every stage has to visibly bleed, or the walk-down looks unmotivated
    for k in range(len(JAM_SEQ)):
        j = JAM_SEQ[k]
        for off in (2, 5, 8):
            t = k * STAGE + off
            if t < NT - 1 and sched[t - 1] != j and sched[t + 1] != j:
                sched[t] = j

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

    imgs = []
    for fi in range(NT + args.hold):
        cursor = min(fi, NT - 1)
        seen = min(cursor + 1, NT)
        out = excluded_at(cursor)
        jam_ci = jam_at(cursor)
        refused = cursor >= REFUSE_T
        img, d = new_frame(W, H)
        chrome(d, W, H, "ANTI-HERDING",
               "an interferer that follows every exclusion — each request is "
               "reasonable, and granting them all would park the link on one "
               "channel", fi)

        d.rectangle([padL, padT, padL + gw, padT + gh], outline=(0, 70, 80))
        for ci, ch in enumerate(CHANS):
            _, y = cell_xy(0, ci)
            d.text((padL - 34, y + cellh / 2 - 7), f"{ch}", font=font(11),
                   fill=WARN if ci == jam_ci else (DIM if ci not in out else (60, 72, 90)))
            d.line([padL, y, padL + gw, y], fill=GRID)

        # every span the interferer has occupied
        for k, j in enumerate(JAM_SEQ):
            t0 = k * STAGE
            t1 = min(seen, (k + 1) * STAGE if k + 1 < len(JAM_SEQ) else NT)
            if t1 <= t0:
                continue
            _, y = cell_xy(0, j)
            d.rectangle([tx(t0), y, tx(t1), y + cellh], fill=(40, 12, 12))

        # and every channel it has cost us, dark from its commit onward
        for k, c in enumerate(COMMITS):
            if cursor < c:
                continue
            j = JAM_SEQ[k]
            _, y = cell_xy(0, j)
            xr = tx(seen)
            d.rectangle([tx(c), y, xr, y + cellh], fill=(16, 19, 26))
            hx = tx(c) + 6
            while hx < xr:
                d.line([hx, y + 5, hx - 6, y + cellh - 5], fill=(30, 38, 50))
                hx += 12

        # the trace
        lost = 0
        for ti in range(seen):
            ci = sched[ti]
            x, y = cell_xy(ti, ci)
            hit = ci == jam_at(ti)
            if hit:
                lost += 1
            fade = 0.45 + 0.55 * (ti / max(1, cursor)) if ti < cursor else 1.0
            d.rectangle([x + 2, y + 6, x + cellw - 2, y + cellh - 6],
                        fill=tuple(int(c * fade) for c in (WARN if hit else CYAN)))
            if hit:
                d.line([x + 2, y + 6, x + cellw - 2, y + cellh - 6], fill=INK)
                d.line([x + 2, y + cellh - 6, x + cellw - 2, y + 6], fill=INK)
            if ti == cursor and fi < NT:
                d.rectangle([x + 1, y + 5, x + cellw - 1, y + cellh - 5],
                            outline=INK, width=2)
            if ti:
                x0, y0 = cell_xy(ti - 1, sched[ti - 1])
                d.line([x0 + cellw / 2, y0 + cellh / 2,
                        x + cellw / 2, y + cellh / 2], fill=(30, 44, 60))

        # one amber line per granted exclusion — the staircase down to the floor
        for k, c in enumerate(COMMITS):
            if cursor < c:
                continue
            d.line([tx(c), padT, tx(c), padT + gh], fill=AMBER, width=1)
            d.text((tx(c) - 12, 78), f"gen {k + 2}", font=font(10), fill=AMBER)
        if refused:
            rx = tx(REFUSE_T)
            d.line([rx, padT, rx, padT + gh], fill=WARN, width=2)
            # inside the grid, clear of the gen labels crowding the top band
            d.rectangle([rx + 4, padT + 6, rx + 74, padT + 24], fill=(8, 11, 18))
            d.text((rx + 8, padT + 7), "REFUSED", font=font(11, True), fill=WARN)

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

        walked = min(seen, COMMITS[-1])
        bracket(0, walked, "WALKED DOWN — 8 ACTIVE, THEN 7, 6, 5, 4",
                WARN if not refused else DIM)
        if seen > COMMITS[-1]:
            bracket(COMMITS[-1], seen, "HELD AT 3", AMBER)

        # readout
        x0 = padL + gw + 22
        d.text((x0, padT - 2), "LIVE READOUT", font=font(12), fill=CYAN)
        y = padT + 24

        def line(lbl, val, c=INK):
            nonlocal y
            d.text((x0, y), lbl, font=font(11), fill=DIM)
            d.text((x0 + 100, y - 3), val, font=font(15, True), fill=c)
            y += 30

        act = nC - len(out)
        mask = "".join("0" if ci in out else "1" for ci in range(nC))
        line("generation", f"{len(out) + 1}", AMBER if out else INK)
        line("hop set", mask, AMBER if act == FLOOR else OK)
        line("active", f"{act}/{nC}", AMBER if act == FLOOR else INK)
        line("floor", f"{FLOOR}", AMBER if act == FLOOR else DIM)
        line("interferer", f"ch {CHANS[jam_ci]}", WARN)
        line("dwells lost", f"{lost}", WARN if lost else OK)

        if refused:
            st, sc = "HELD AT FLOOR", AMBER
        elif act == FLOOR:
            st, sc = "JAMMED AGAIN", WARN
        elif out:
            st, sc = "FOLLOWED", WARN
        else:
            st, sc = "LEARNING", WARN
        d.rectangle([x0, y, x0 + 200, y + 30], outline=sc, width=2)
        d.ellipse([x0 + 8, y + 10, x0 + 18, y + 20], fill=sc)
        d.text((x0 + 28, y + 6), st, font=font(14, True), fill=sc)
        y += 44

        if st == "LEARNING":
            cap = ("ch 52 keeps losing", "dwells — the receiver",
                   "asks for it to go.")
        elif st == "FOLLOWED":
            cap = ("granted — and it moved", "onto another active",
                   "channel. and another.", "every request is fair;",
                   "the pattern is not.")
        elif st == "JAMMED AGAIN":
            cap = ("three channels left, and", "the interferer is sitting",
                   "on one of them again.")
        else:
            cap = ("refused. the floor is", "not negotiable — better",
                   "to bleed on three than", "be herded onto one.")
        for ln in cap:
            d.text((x0, y), ln, font=font(11), fill=INK)
            y += 15

        imgs.append(img)

    save_gif(imgs, args.out, ms=args.ms, colors=48)
    return 0


if __name__ == "__main__":
    sys.exit(main())
