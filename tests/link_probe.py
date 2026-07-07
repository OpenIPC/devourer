#!/usr/bin/env python3
"""Active link-probe analyzer — turn a swept continuous-TX stimulus + the ground
station's RX telemetry into a margin-vs-lever curve and a recommended operating
point.

The emitter (txdemo with DEVOURER_CONT_TX + a DEVOURER_TX_PWR ramp)
emits one `txpwr.set` event (index=N) per step; the ground station
(rxdemo with DEVOURER_RX_ENERGY_MS) emits `rx.energy` (per-frame
SNR aggregate) and `rx.nhm` (frame-free power histogram) events. Both are
captured with a host-side arrival timestamp (prefixed `<epoch> `), so this aligns
them by wall-clock — no cross-process clock sync — binning each ground sample
into the emitter step window it falls in.

For each swept level it reports the ground's mean SNR / NHM-peak, then picks the
operating point: the *minimum* TX-power index whose ground SNR clears
--target-snr (the energy-min reflex: least power that holds the margin).

The same analyzer serves the MCS-headroom axis: when the emitter steps its rate
(DEVOURER_TX_MCS_SWEEP) it emits `tx.contx` events (mcs=<spec>) instead, and
this reports per-MCS ground SNR + frame delivery and picks the *highest* rate that
still clears the floor (the highest modulation the link holds). The axis is auto-
detected from whichever marker the emitter log carries.

    tests/link_probe.py --emit emit.log --ground ground.log --target-snr 20
"""
from __future__ import annotations

import argparse
import re
import statistics
import sys

from devourer_events import parse_event

TS = re.compile(r"^(\d+\.\d+)\s+(.*)$")


def read_stamped(path):
    """Yield (t_epoch, text) for lines prefixed with a host arrival timestamp."""
    out = []
    with open(path) as f:
        for line in f:
            m = TS.match(line.strip())
            if m:
                out.append((float(m.group(1)), m.group(2)))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--emit", required=True, help="emitter log (timestamped)")
    ap.add_argument("--ground", required=True, help="ground RX log (timestamped)")
    ap.add_argument("--target-snr", type=float, default=None,
                    help="pick the cheapest level whose ground SNR clears this")
    ap.add_argument("--settle-ms", type=float, default=150,
                    help="drop ground samples this soon after a step (settling)")
    args = ap.parse_args()

    emit = read_stamped(args.emit)
    ground = read_stamped(args.ground)

    # Auto-detect the swept axis from the emitter markers.
    steps = []           # [(t_start, label), ...]
    axis = None
    for t, text in emit:
        ev = parse_event(text)
        if ev is None:
            continue
        if ev["ev"] == "txpwr.set":
            axis = axis or "power"
            steps.append((t, str(ev["index"])))
        elif ev["ev"] == "tx.contx":
            axis = axis or "mcs"
            steps.append((t, str(ev["mcs"])))
    if not steps:
        print("no step markers in emitter log — set DEVOURER_TX_PWR_START/STOP/STEP "
              "(power axis) or DEVOURER_TX_MCS_SWEEP (MCS axis)", file=sys.stderr)
        return 1

    # Ground samples: (t, snr_mean, frames, nhm_peak).
    samples = []
    last_nhm = None
    for t, text in ground:
        ev = parse_event(text)
        if ev is None:
            continue
        if ev["ev"] == "rx.nhm":
            last_nhm = ev.get("peak")
            continue
        if ev["ev"] == "rx.energy":
            samples.append((t, ev.get("snr_mean"),
                            ev.get("frames") or 0, last_nhm))

    # Assign each ground sample to the step window it falls in, accumulating by
    # label (the sweep may cycle through the levels more than once).
    agg = {}             # label -> {snr:[], nhm:[], frames:int}
    order = []           # unique labels in first-seen (sweep) order
    for i, (t0, label) in enumerate(steps):
        t1 = steps[i + 1][0] if i + 1 < len(steps) else float("inf")
        win = [(s, fr, n) for (t, s, fr, n) in samples
               if t0 + args.settle_ms / 1000.0 <= t < t1]
        if label not in agg:
            agg[label] = {"snr": [], "nhm": [], "frames": 0}
            order.append(label)
        agg[label]["snr"] += [s for (s, fr, n) in win if s is not None and fr]
        agg[label]["nhm"] += [n for (s, fr, n) in win if n is not None]
        agg[label]["frames"] += sum(fr for (s, fr, n) in win if fr)

    rows = []            # (label, snr, nhm, frames, n) in sweep order
    for label in order:
        a = agg[label]
        if not a["snr"] and not a["nhm"]:
            continue
        rows.append((label,
                     statistics.mean(a["snr"]) if a["snr"] else None,
                     statistics.median(a["nhm"]) if a["nhm"] else None,
                     a["frames"], len(a["snr"])))

    if not rows:
        print("no ground samples aligned to any step window — check that both "
              "sides ran concurrently and the RX heard the emitter", file=sys.stderr)
        return 1

    col = "index" if axis == "power" else "MCS"
    print(f"# link probe ({axis} axis): {len(rows)} levels, {len(samples)} "
          f"ground samples")
    print(f"# {col:>8} {'gnd_SNR':>8} {'frames':>7} {'nhm_peak':>8} {'n':>4}")
    for label, snr, nhm, frames, n in rows:
        s = f"{snr:6.1f}" if snr is not None else "   -  "
        nn = f"{nhm}" if nhm is not None else "-"
        print(f"  {label:>8} {s:>8} {frames:>7} {nn:>8} {n:>4}")

    # Thermal-budget overlay: the emitter's PA thermal-meter delta over the sweep
    # (DEVOURER_THERMAL_POLL_MS on the emitter). Continuous stepping at full duty
    # is the worst-case heat; this reports how far the PA drifted, bounding the
    # power/duty a controller may sustain (the drone's local safety override).
    raws = [int(ev["raw"]) for _, text in emit
            for ev in [parse_event(text, "thermal")] if ev]
    if raws:
        print(f"\n# PA thermal (emitter raw meter): {raws[0]} -> {raws[-1]} "
              f"units over the sweep (range {min(raws)}..{max(raws)}, "
              f"+{max(raws) - min(raws)}); ~1.5-2 C/unit. Bounds the sustainable "
              f"power/duty.")

    if args.target_snr is None:
        return 0

    ok = [(label, snr) for label, snr, _, _, _ in rows
          if snr is not None and snr >= args.target_snr]
    if axis == "power":
        # Cheapest power that clears the floor (levels are numeric indices).
        if ok:
            best = min(ok, key=lambda r: int(r[0]))
            print(f"\nRECOMMEND: TX-power index {best[0]} — the minimum that clears "
                  f"the {args.target_snr:.0f} dB SNR floor (ground SNR "
                  f"{best[1]:.1f} dB). Energy-min: least power that holds margin.")
        else:
            top = max(rows, key=lambda r: (r[1] if r[1] is not None else -1e9))
            print(f"\nRECOMMEND: no level clears {args.target_snr:.0f} dB — best is "
                  f"index {top[0]} at {top[1]:.1f} dB. Link too weak for this rate; "
                  f"drop MCS or raise power ceiling.")
    else:
        # Highest MCS that still holds the floor (list order = ascending rate).
        held = [label for (label, snr, _, frames, _) in rows
                if snr is not None and snr >= args.target_snr and frames]
        if held:
            print(f"\nRECOMMEND: {held[-1]} — the highest swept rate whose ground "
                  f"SNR clears the {args.target_snr:.0f} dB floor. Energy-min: ride "
                  f"the fastest modulation the link holds.")
        else:
            print(f"\nRECOMMEND: no swept rate clears {args.target_snr:.0f} dB — the "
                  f"link can't hold these MCS; sweep lower rates or raise power.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
