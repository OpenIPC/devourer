#!/usr/bin/env python3
"""Render a coarse RX spectrum map from a live DEVOURER_RX_SWEEP log.

Parses the `rx.energy` events (with a `ch` field) the sensor emits (one per
dwell),
aggregates the frame-free CCA-OFDM energy per channel across sweep rounds, and
draws an ASCII energy-vs-frequency bar chart. Flags the peak bin (a spike
interferer) and, when the floor is clearly non-zero, the deepest dip (a 1T1R
part whose AGC saturates and goes deaf under a strong co-located carrier).

No per-tone CSI is available on this silicon (see docs/rx-spectrum-sensing.md);
the resolution is the channel grid — 20 MHz on the 2.4/5 GHz plan, ~5 MHz on
Jaguar3 with --nb-bw 5.
"""
import argparse
import statistics
import sys

from devourer_events import iter_events


def parse(path):
    """channel -> list of cca_ofdm samples (ints), in file order."""
    per_ch = {}
    with open(path) as f:
        for ev in iter_events(f, ev="rx.energy"):
            if "ch" not in ev or ev["ch"] is None:
                continue
            v = ev.get("cca_ofdm")
            if v is None:
                continue
            per_ch.setdefault(int(ev["ch"]), []).append(int(v))
    return per_ch


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", required=True)
    ap.add_argument("--nb-bw", default=None, help="bin bandwidth (MHz), for the header only")
    args = ap.parse_args()

    per_ch = parse(args.log)
    if not per_ch:
        print("no rx.energy samples with ch= in log", file=sys.stderr)
        return 1

    chans = sorted(per_ch)
    # Drop the first sample per channel (first dwell after a retune can catch the
    # AGC still settling); keep the rest, median for robustness.
    energy = {}
    for ch in chans:
        s = per_ch[ch]
        s = s[1:] if len(s) > 1 else s
        energy[ch] = statistics.median(s)

    peak_ch = max(energy, key=energy.__getitem__)
    peak = energy[peak_ch]
    floor = statistics.median(list(energy.values()))
    scale = peak if peak > 0 else 1
    width = 50

    bw = f"{args.nb_bw} MHz" if args.nb_bw else "channel-grid"
    print(f"RX spectrum map ({bw} bins, cca_ofdm median over rounds)\n")
    for ch in chans:
        e = energy[ch]
        bar = "#" * int(round(width * e / scale))
        mark = ""
        if ch == peak_ch and peak > 1.5 * max(floor, 1):
            mark = "  <-- PEAK (interferer)"
        print(f"ch {ch:>3} | {bar:<{width}} {int(e):>8}{mark}")

    # A dip is only meaningful against a clearly non-zero floor (the collapse
    # signature — a saturated 1T1R receiver).
    dip_ch = min(energy, key=energy.__getitem__)
    if floor >= 30 and energy[dip_ch] * 2.5 <= floor:
        print(f"\ndip: ch {dip_ch} (energy {int(energy[dip_ch])} vs floor "
              f"{int(floor)}) — possible AGC-saturation collapse (1T1R)")
    print(f"\npeak ch {peak_ch}  floor {int(floor)}  peak {int(peak)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
