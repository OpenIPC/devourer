#!/usr/bin/env python3
"""Active link-probe analyzer — turn a swept continuous-TX stimulus + the ground
station's RX telemetry into a margin-vs-lever curve and a recommended operating
point.

The emitter (WiFiDriverTxDemo with DEVOURER_CONT_TX + a DEVOURER_TX_PWR ramp)
prints one `<devourer-txpwr>index=N` marker per step; the ground station
(WiFiDriverDemo with DEVOURER_RX_ENERGY_MS) prints `<devourer-energy>` (per-frame
SNR aggregate) and `<devourer-nhm>` (frame-free power histogram) lines. Both are
captured with a host-side arrival timestamp (prefixed `<epoch> `), so this aligns
them by wall-clock — no cross-process clock sync — binning each ground sample
into the emitter step window it falls in.

For each swept level it reports the ground's mean SNR / NHM-peak, then picks the
operating point: the *minimum* TX-power index whose ground SNR clears
--target-snr (the energy-min reflex: least power that holds the margin). The
same harness serves the MCS axis when the emitter steps MCS instead of power.

    tests/link_probe.py --emit emit.log --ground ground.log --target-snr 20
"""
from __future__ import annotations

import argparse
import re
import statistics
import sys

TS = re.compile(r"^(\d+\.\d+)\s+(.*)$")
STEP = re.compile(r"<devourer-txpwr>index=(\d+)")
ENERGY = re.compile(r"<devourer-energy>(.*)")
NHM = re.compile(r"<devourer-nhm>.*peak=(\d+)")
KV = re.compile(r"(\w+)=(-?\d+)")


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

    # Build step windows: [(t_start, index), ...] from the emitter markers.
    steps = []
    for t, text in emit:
        m = STEP.search(text)
        if m:
            steps.append((t, int(m.group(1))))
    if not steps:
        print("no <devourer-txpwr>index= markers in emitter log — was the power "
              "ramp (DEVOURER_TX_PWR_START/STOP/STEP) set?", file=sys.stderr)
        return 1

    # Ground samples: (t, snr_mean, frames, nhm_peak).
    samples = []
    last_nhm = None
    for t, text in ground:
        mn = NHM.search(text)
        if mn:
            last_nhm = int(mn.group(1))
            continue
        me = ENERGY.search(text)
        if me:
            kv = dict((k, int(v)) for k, v in KV.findall(me.group(1)))
            samples.append((t, kv.get("snr_mean"), kv.get("frames", 0), last_nhm))

    # Assign each ground sample to the step window it falls in.
    rows = []
    for i, (t0, idx) in enumerate(steps):
        t1 = steps[i + 1][0] if i + 1 < len(steps) else float("inf")
        snrs = [s for (t, s, fr, _) in samples
                if t0 + args.settle_ms / 1000.0 <= t < t1 and s is not None and fr]
        nhms = [n for (t, s, fr, n) in samples
                if t0 + args.settle_ms / 1000.0 <= t < t1 and n is not None]
        if not snrs and not nhms:
            continue
        rows.append((idx,
                     statistics.mean(snrs) if snrs else None,
                     statistics.median(nhms) if nhms else None,
                     len(snrs)))

    if not rows:
        print("no ground samples aligned to any step window — check that both "
              "sides ran concurrently and the RX heard the emitter", file=sys.stderr)
        return 1

    print(f"# link probe: {len(rows)} levels, {len(samples)} ground samples")
    print(f"# {'index':>5} {'gnd_SNR':>8} {'nhm_peak':>8} {'n':>4}")
    for idx, snr, nhm, n in rows:
        s = f"{snr:6.1f}" if snr is not None else "   -  "
        nn = f"{nhm}" if nhm is not None else "-"
        print(f"  {idx:5d} {s:>8} {nn:>8} {n:>4}")

    if args.target_snr is not None:
        ok = [(idx, snr) for idx, snr, _, _ in rows
              if snr is not None and snr >= args.target_snr]
        if ok:
            best = min(ok, key=lambda r: r[0])
            print(f"\nRECOMMEND: TX-power index {best[0]} — the minimum that clears "
                  f"the {args.target_snr:.0f} dB SNR floor (ground SNR "
                  f"{best[1]:.1f} dB). Energy-min: least power that holds margin.")
        else:
            top = max(rows, key=lambda r: (r[1] if r[1] is not None else -1e9))
            print(f"\nRECOMMEND: no level clears {args.target_snr:.0f} dB — best is "
                  f"index {top[0]} at {top[1]:.1f} dB. Link too weak for this rate; "
                  f"drop MCS or raise power ceiling.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
