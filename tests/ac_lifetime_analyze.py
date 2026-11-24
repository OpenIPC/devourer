#!/usr/bin/env python3
"""Per-arm delivered age for tests/ac_lifetime_onair.sh.

Reads the run directory that script leaves: `marks` (one "<n> <arm> <ns>"
line per arm, then an end mark), `wit.jsonl` and `tx-<n>.jsonl`, every event
line prefixed with the host monotonic time it arrived at.

A frame's submit time is interpolated between txdemo's tx.frame anchors (it
emits one in 500), and its age is the witness arrival of its last copy minus
that. Frames outside the anchored range are not scored: their submit time
would be an extrapolation. Exit 0 = every lifetime arm bounded and the
control unbounded, 1 = a lifetime arm failed (a transmitter that did not
finish its frames included), 2 = the run cannot be judged (no lifetime arm,
no control, nothing heard).
"""
import argparse
import bisect
import json
import os
import sys


def stamped(path, ev):
    """(t_ns, event) for each `ev` line of a stamped JSONL file."""
    tag = '"ev":"%s"' % ev
    with open(path, errors="replace") as f:
        for line in f:
            if tag not in line:
                continue
            ts, _, body = line.partition(" ")
            try:
                yield int(ts), json.loads(body)
            except ValueError:
                continue


def arm_lifetime_ms(arm):
    life = arm.split(":")[-1]
    return None if life == "off" else int(life) / 1000.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--tol-ms", type=float, default=15.0)
    ap.add_argument("--frames", type=int, default=0,
                    help="frames each arm was asked to send (0 = unchecked)")
    a = ap.parse_args()

    marks = [l.split() for l in open(os.path.join(a.out, "marks"))]
    wit = [(t, e["pctr"]) for t, e in
           stamped(os.path.join(a.out, "wit.jsonl"), "rx.seq")
           if not e.get("crc")]

    rows = []  # (arm, lifetime_ms, p50, p99, max, failed sends)
    incomplete = []  # lifetime arms whose transmitter did not finish
    print(f"{'arm':<14}{'sent':>6}{'fail':>6}{'tx_s':>7}{'heard':>7}"
          f"{'copies':>8}{'age_p50':>10}{'age_p99':>10}{'age_max':>10}"
          "   (ms, last copy)")
    for i in range(len(marks) - 1):
        n, arm, t0 = marks[i][0], marks[i][1], int(marks[i][2])
        t1 = int(marks[i + 1][2])
        path = os.path.join(a.out, f"tx-{n}.jsonl")
        anchors = [(e["n"], t) for t, e in stamped(path, "tx.frame")]
        failed = 0
        for _, e in stamped(path, "tx.stats"):
            failed = e["failed"]
        try:
            rc = int(open(os.path.join(a.out, f"tx-{n}.rc")).read())
        except (OSError, ValueError):
            rc = 0
        done = anchors[-1][0] if anchors else 0
        # The last anchor is the last multiple of 500 at or below the bound.
        if arm_lifetime_ms(arm) is not None and (
                rc != 0 or done < a.frames - a.frames % 500):
            incomplete.append(f"{arm}: txdemo exit {rc}, {done} of "
                              f"{a.frames} frames")
        if len(anchors) < 2:
            print(f"{arm:<14} no tx.frame anchors (the DUT sent nothing)")
            rows.append((arm, arm_lifetime_ms(arm), None, None, None, failed))
            continue
        counts = [c for c, _ in anchors]

        def submit_ns(pctr):
            # Counter p is the (p+1)-th frame; anchor n means n frames done.
            k = pctr + 1
            j = min(max(bisect.bisect_left(counts, k), 1), len(counts) - 1)
            (n0, ta), (n1, tb) = anchors[j - 1], anchors[j]
            return ta + (tb - ta) * (k - n0) / (n1 - n0)

        last, copies = {}, {}
        for t, p in wit:
            if t0 <= t < t1 and counts[0] <= p + 1 <= counts[-1]:
                last[p] = t
                copies[p] = copies.get(p, 0) + 1
        if not last:
            print(f"{arm:<14} the witness heard nothing")
            rows.append((arm, arm_lifetime_ms(arm), None, None, None, failed))
            continue
        ages = sorted((last[p] - submit_ns(p)) / 1e6 for p in last)

        def q(f):
            return ages[min(len(ages) - 1, int(f * len(ages)))]

        tx_s = (anchors[-1][1] - anchors[0][1]) / 1e9
        print(f"{arm:<14}{counts[-1]:>6}{failed:>6}{tx_s:>7.1f}"
              f"{len(last):>7}{sum(copies.values()) / len(copies):>8.1f}"
              f"{q(.5):>10.1f}{q(.99):>10.1f}{ages[-1]:>10.1f}")
        rows.append((arm, arm_lifetime_ms(arm), q(.5), q(.99), ages[-1], failed))

    lifetimes = [r[1] for r in rows if r[1] is not None]
    if not lifetimes:
        print("[aclife] ABORT: no lifetime arm, nothing to judge")
        return 2
    # Every control must be clearly older than each bound it is compared
    # with. A control whose sends started failing counts too: the queue
    # filled, which is the unbounded case at its worst (on some parts txdemo
    # gives up before a second anchor, so there is no age to print).
    controls = [r for r in rows if r[1] is None]
    unbounded = [r for r in controls
                 if r[5] > 0 or (r[3] is not None and
                                 r[3] >= 1.5 * max(lifetimes))]
    if not controls or len(unbounded) < len(controls):
        print("[aclife] ABORT: a knobs-off arm is missing or not clearly "
              "older than the lifetime - the peer ACKs or the retry chain is "
              "short")
        return 2
    bad = len(incomplete)
    for line in incomplete:
        print(f"[aclife] FAIL {line}")
    for arm, life, p50, p99, _, _ in rows:
        if life is None:
            continue
        if p50 is None:
            print(f"[aclife] FAIL {arm}: nothing measured")
            bad += 1
        elif p99 > life + a.tol_ms or abs(p50 - life) > a.tol_ms:
            print(f"[aclife] FAIL {arm}: median {p50:.1f} ms, p99 {p99:.1f} ms "
                  f"against a {life:.0f} ms lifetime (tolerance {a.tol_ms:g})")
            bad += 1
    if bad:
        return 1
    print(f"[aclife] PASS: {len(lifetimes)} lifetime arm(s) bounded, "
          f"{len(controls)} control arm(s) unbounded")
    return 0


if __name__ == "__main__":
    sys.exit(main())
