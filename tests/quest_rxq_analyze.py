#!/usr/bin/env python3
# /// script
# requires-python = ">=3.9"
# ///
"""Analyse issue-330 Quest RX-ring captures: per-mode delivery (from the 802.11
seq_num sequence, which increments per aired frame) plus ring-health telemetry
(cb_max_us consumer stalls, min_armed depth, resubmit_fail, empties). Compares
RX-ring servicing modes under the same TX flood — the differential that isolates
host-side ring loss from the (mode-invariant) RF baseline."""
import glob
import json
import sys


def load(path):
    seq, ring = [], []
    for line in open(path):
        if '"ev":"rx.seq"' in line:
            try:
                seq.append(json.loads(line))
            except Exception:
                pass
        elif '"ev":"rx.ring"' in line:
            try:
                ring.append(json.loads(line))
            except Exception:
                pass
    return seq, ring


def delivery_from_seqnum(seq):
    """802.11 seq_num counts every aired frame and wraps at 4096. Unfold the
    wraps into a monotonic stream, then delivery = received / span."""
    sn = [e["seq"] for e in seq if "seq" in e]
    if len(sn) < 2:
        return None
    unfolded, base, prev = [], 0, sn[0]
    for v in sn:
        if v < prev - 2048:        # forward wrap
            base += 4096
        elif v > prev + 2048:      # backward blip (reorder) — ignore the jump
            base -= 4096
        unfolded.append(base + v)
        prev = v
    lo, hi = min(unfolded), max(unfolded)
    span = hi - lo + 1
    got = len(set(unfolded))
    return got, span, 100.0 * got / span if span else 0.0


def ring_stats(ring):
    if not ring:
        return {}
    cb = [r.get("cb_max_us", 0) for r in ring]
    ma = [r.get("min_armed", 0) for r in ring]
    comp = [r.get("completions", 0) for r in ring]
    return {
        "cb_max_us": max(cb),
        "cb_p50_us": sorted(cb)[len(cb) // 2],
        "min_armed": min(ma),
        "resubmit_fail": max(r.get("resubmit_fail", 0) for r in ring),
        "empties": max(r.get("empties", 0) for r in ring),
        "completions": (max(comp) - min(comp)) if comp else 0,
    }


def main(paths):
    rows = []
    for p in sorted(paths):
        seq, ring = load(p)
        mode = p.split("rxq_m_")[-1].replace(".jsonl", "").replace("_", "-") \
            if "rxq_m_" in p else p.split("/")[-1]
        d = delivery_from_seqnum(seq)
        rs = ring_stats(ring)
        rows.append((mode, len(seq), d, rs))

    print(f"{'mode':<14}{'rx.seq':>8}{'got/span':>14}{'deliv%':>8}"
          f"{'cbmax_us':>9}{'cbp50':>7}{'minArm':>7}{'resubF':>7}{'compl':>7}")
    for mode, nseq, d, rs in rows:
        gs = f"{d[0]}/{d[1]}" if d else "-"
        dv = f"{d[2]:.1f}" if d else "-"
        print(f"{mode:<14}{nseq:>8}{gs:>14}{dv:>8}"
              f"{rs.get('cb_max_us','-'):>9}{rs.get('cb_p50_us','-'):>7}"
              f"{rs.get('min_armed','-'):>7}{rs.get('resubmit_fail','-'):>7}"
              f"{rs.get('completions','-'):>7}")
    if len(rows) > 1 and all(r[2] for r in rows):
        base = rows[0]
        print(f"\nbaseline={base[0]} deliv={base[2][2]:.1f}%")
        for mode, _, d, _ in rows[1:]:
            print(f"  {mode:<14} Δdeliv = {d[2] - base[2][2]:+.1f} pts")


if __name__ == "__main__":
    args = sys.argv[1:]
    paths = [x for a in args for x in glob.glob(a)] if args else []
    if not paths:
        print("usage: quest_rxq_analyze.py <rxq_*.jsonl ...>")
        sys.exit(1)
    main(paths)
