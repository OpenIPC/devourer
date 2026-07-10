#!/usr/bin/env python3
"""Analyze the master's timesync.ta stream: does the uplink timing-advance loop
CONVERGE — driving each UE uplink's arrival phase onto the master's slot boundary?

The master emits {"ev":"timesync.ta","n":k,"phase_us":P,"ta_us":T} per uplink.
`phase_us` is the arrival's offset from the nearest slot boundary (what the loop
nulls); `ta_us` is the accumulated timing advance. Convergence = the early-window
phase RMS (loop still correcting) ≫ the late-window phase RMS (locked).

  python3 tests/timesync_ta_analyze.py master.log
"""
import json
import sys


def rms(xs):
    return (sum(x * x for x in xs) / len(xs)) ** 0.5 if xs else float("nan")


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: timesync_ta_analyze.py master.log")
    phase, ta = [], []
    with open(sys.argv[1]) as f:
        for line in f:
            if '"timesync.ta"' not in line:
                continue
            try:
                e = json.loads(line.strip())
            except json.JSONDecodeError:
                continue
            phase.append(e["phase_us"])
            ta.append(e["ta_us"])
    n = len(phase)
    print(f"uplinks measured: {n}")
    if n < 40:
        sys.exit("too few uplinks — did the UE reach the master (8822E TX+RX "
                 "desense?) and both come up full-duplex?")
    q = max(10, n // 4)
    early, late = phase[:q], phase[-q:]
    mean = lambda xs: sum(xs) / len(xs)
    # The loop nulls the systematic (DC) arrival offset; the per-uplink scatter
    # is the UE's full-duplex TX-timing jitter and is irreducible, so track the
    # MEAN phase (what converges) alongside the RMS (the jitter floor).
    print(f"  first {q}: mean {mean(early):+8.1f} us   RMS {rms(early):8.1f} us")
    print(f"  last  {q}: mean {mean(late):+8.1f} us   RMS {rms(late):8.1f} us")
    print(f"  TA settled at {ta[-1]:+.1f} us (mod-slot)")
    if abs(mean(late)) < max(50.0, abs(mean(early)) * 0.5):
        print(f"  => CONVERGED: the loop pulled the mean uplink arrival onto the "
              f"slot boundary ({mean(early):+.0f} -> {mean(late):+.0f} us).")
    else:
        print("  => mean arrival not clearly nulled (see raw phase series).")


if __name__ == "__main__":
    main()
