#!/usr/bin/env python3
"""Join two timesync SLAVE logs by beacon seq and report the inter-slave sync
error — how well two UEs agree on the master (eNB) clock, each derived purely
from the over-the-air beacons with no host clock or GPS.

Each slave emits {"ev":"timesync.lock","seq":N,"pred_master":X,"resid_us":R,...}
per beacon once its fit is warm. For beacons BOTH slaves predicted, |predA-predB|
is the inter-slave error (the common master stamp jitter cancels). Each slave's
own resid_us is its absolute lock error (bounded by that master jitter).

  python3 tests/timesync_analyze.py slaveA.log slaveB.log
"""
import json
import statistics
import sys


def load(path):
    """seq -> (pred_master, resid_us) for timesync.lock events."""
    out = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if '"timesync.lock"' not in line:
                continue
            try:
                e = json.loads(line)
            except json.JSONDecodeError:
                continue
            out[e["seq"]] = (e["pred_master"], e["resid_us"])
    return out


def rms(xs):
    return (sum(x * x for x in xs) / len(xs)) ** 0.5 if xs else float("nan")


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: timesync_analyze.py slaveA.log slaveB.log")
    A, B = load(sys.argv[1]), load(sys.argv[2])
    common = sorted(set(A) & set(B))
    print(f"slave A locked beacons: {len(A)}")
    print(f"slave B locked beacons: {len(B)}")
    print(f"common beacons        : {len(common)}")
    if len(common) < 10:
        sys.exit("too few common beacons — were both slaves in range of the master?")

    # Per-slave absolute lock error (vs the master's own broadcast).
    for name, S in (("A", A), ("B", B)):
        r = [abs(v[1]) for v in S.values()]
        print(f"  slave {name} lock error : RMS {rms(r):7.2f} us   max {max(r):7.2f} us")

    # Inter-slave agreement: the two UEs' independent estimates of the same
    # beacon's master TSF. This is the headline — the common master jitter cancels.
    diffs = [A[s][0] - B[s][0] for s in common]
    ad = [abs(d) for d in diffs]
    print(f"\n  inter-slave sync error: RMS {rms(diffs):7.2f} us   "
          f"max {max(ad):7.2f} us   mean {statistics.mean(diffs):+7.2f} us")
    print(f"  => two UEs agree on the master (eNB) clock to this, over the air, "
          f"no GPS.")


if __name__ == "__main__":
    main()
