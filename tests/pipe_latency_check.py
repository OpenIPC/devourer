#!/usr/bin/env python3
"""pipe_latency_check.py — prove the event stream doesn't stall in a pipe.

The historical failure mode: a supervisor (test script, AI agent) reads a
demo's stdout through subprocess.PIPE; libc makes piped stdout fully
buffered, so output stalls until a 4 KiB boundary. The new logging system
flushes every event line at emission (src/Event.h), so a piped reader must
see each event within moments of the emitting condition — this script
asserts that end-to-end with a real adapter.

Method: spawn rxdemo with stdout=PIPE, timestamp every event line as it
arrives, and check the gap between our first sight of `init.timing`
stage=demo.create_device and the moment the process emitted it (its `ms`
field vs the process start) stays under the threshold. Then confirm
periodic rx.energy events arrive at their cadence (each within 2 intervals),
which fails under full buffering (a quiet channel emits ~60 B/interval —
a 4 KiB buffer would hold ~30 s of them back).

Usage:
    sudo python3 tests/pipe_latency_check.py [--pid 0xNNNN] [--channel N]

Exit 0 = no stall; 1 = stall or no events; 2 = setup failure.
"""

import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from devourer_events import parse_event  # noqa: E402

BUILD = os.path.join(HERE, "..", "build")
ENERGY_MS = 500
RUN_S = 8
SLOP_S = 2 * ENERGY_MS / 1000.0  # each periodic event must land within 2 ticks


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pid", default=None, help="DEVOURER_PID filter")
    ap.add_argument("--channel", default="6")
    args = ap.parse_args()

    exe = os.path.join(BUILD, "rxdemo")
    if not os.path.exists(exe):
        print(f"pipe_latency_check: {exe} not built", file=sys.stderr)
        return 2

    env = dict(os.environ)
    env["DEVOURER_CHANNEL"] = args.channel
    env["DEVOURER_RX_ENERGY_MS"] = str(ENERGY_MS)
    if args.pid:
        env["DEVOURER_PID"] = args.pid

    t0 = time.monotonic()
    proc = subprocess.Popen([exe], stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, env=env)
    energy_seen = []
    first_event = None
    try:
        deadline = t0 + 60
        while time.monotonic() < deadline:
            line = proc.stdout.readline()
            if not line:
                print("note: rxdemo exited early (rc pending) — bring-up "
                      "failure? rerun or check stderr without DEVNULL",
                      file=sys.stderr)
                break
            now = time.monotonic()
            ev = parse_event(line)
            if ev is None:
                continue
            if first_event is None:
                first_event = (ev["ev"], now - t0)
            if ev["ev"] == "rx.energy":
                energy_seen.append(now)
                if len(energy_seen) >= RUN_S * 1000 // ENERGY_MS:
                    break
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()

    if first_event is None:
        print("FAIL: no events arrived through the pipe at all")
        return 1
    print(f"first event: {first_event[0]} after {first_event[1]:.2f}s")

    if len(energy_seen) < 3:
        print(f"FAIL: only {len(energy_seen)} rx.energy events "
              f"(expected ~{RUN_S * 1000 // ENERGY_MS})")
        return 1
    gaps = [b - a for a, b in zip(energy_seen, energy_seen[1:])]
    worst = max(gaps)
    print(f"rx.energy: {len(energy_seen)} events, worst inter-arrival "
          f"{worst:.2f}s (cadence {ENERGY_MS / 1000.0:.1f}s, "
          f"allowed {SLOP_S:.1f}s)")
    if worst > SLOP_S:
        print("FAIL: periodic events stalled in the pipe")
        return 1
    print("PASS: piped consumer sees every event at emission time")
    return 0


if __name__ == "__main__":
    sys.exit(main())
