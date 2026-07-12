#!/usr/bin/env python3
"""Analyze an ldpc_waterfall.sh points.jsonl: per-encoding delivery-vs-index
curves and the interpolated horizontal shift between them = measured LDPC
coding gain in dB.

  python3 tests/ldpc_waterfall.py /tmp/devourer-ldpc-waterfall/points.jsonl \
      [--step-qdb 2] [--thresholds 0.5,0.9]

--step-qdb: qdB per TXAGC index step (2 = 0.5 dB on Jaguar1/2, 1 = 0.25 dB
on Jaguar3 — read it off the emitter's txpwr.caps event if unsure).

The gain is read where each curve crosses a delivery-ratio threshold
(linear interpolation between bracketing points). Points where the curve is
non-monotonic near-field garbage (AGC saturation at high index) only matter
if they bracket a threshold — sweep the noise-limited regime.
"""
import argparse
import json
import sys
from collections import defaultdict


def crossing(curve: list[tuple[int, float]], thr: float):
    """First index (interpolated) where delivery ratio rises through thr.
    curve: sorted [(idx, ratio)]."""
    prev_i, prev_r = None, None
    for i, r in curve:
        if prev_r is not None and prev_r < thr <= r:
            # linear interp between (prev_i, prev_r) and (i, r)
            return prev_i + (thr - prev_r) * (i - prev_i) / (r - prev_r)
        prev_i, prev_r = i, r
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("points", help="points.jsonl from ldpc_waterfall.sh")
    ap.add_argument("--step-qdb", type=float, default=2.0,
                    help="qdB per TXAGC index (default 2 = 0.5 dB, Jaguar1/2)")
    ap.add_argument("--thresholds", default="0.5,0.9",
                    help="delivery-ratio crossings to compare (default 0.5,0.9)")
    args = ap.parse_args()

    curves: dict[str, list[tuple[int, float, int, int]]] = defaultdict(list)
    for line in open(args.points, errors="replace"):
        line = line.strip()
        if not line:
            continue
        p = json.loads(line)
        sent, delivered = p["sent"], p["delivered"]
        if sent <= 0:
            print(f"! dropping point enc={p['enc']} idx={p['idx']}: "
                  f"sent={sent} (no final tx.stats — TX died?)",
                  file=sys.stderr)
            continue
        ratio = min(1.0, delivered / sent)
        curves[p["enc"]].append((p["idx"], ratio, sent, delivered))

    for enc, pts in curves.items():
        pts.sort()
        print(f"\n== {enc}")
        print("  idx | sent  | delivered | ratio")
        for idx, ratio, sent, delivered in pts:
            bar = "#" * int(ratio * 40)
            print(f"  {idx:3d} | {sent:5d} | {delivered:9d} | {ratio:5.1%} {bar}")

    thrs = [float(t) for t in args.thresholds.split(",")]
    encs = sorted(curves)
    if len(encs) < 2:
        print("\n(need two encodings for a gain readout)")
        return
    # Convention: compare each /LDPC spec against its base spec.
    for enc in encs:
        if not enc.upper().endswith("/LDPC"):
            continue
        base = enc[: -len("/LDPC")]
        if base not in curves:
            continue
        print(f"\n== gain: {enc} vs {base} (step {args.step_qdb / 4:.2f} dB/idx)")
        for thr in thrs:
            cb = crossing([(i, r) for i, r, *_ in sorted(curves[base])], thr)
            cl = crossing([(i, r) for i, r, *_ in sorted(curves[enc])], thr)
            if cb is None or cl is None:
                print(f"  @{thr:.0%} delivery: not bracketed "
                      f"(base={cb}, ldpc={cl}) — widen/lower the sweep")
                continue
            gain_db = (cb - cl) * args.step_qdb / 4.0
            print(f"  @{thr:.0%} delivery: base idx {cb:.1f} vs ldpc idx "
                  f"{cl:.1f} -> LDPC gain {gain_db:+.2f} dB")


if __name__ == "__main__":
    main()
