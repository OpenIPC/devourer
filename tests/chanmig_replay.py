#!/usr/bin/env python3
"""Offline replay + shadow-mode confusion matrix for the channel-recommendation
engine (#277).

Operates on the JSONL a live advise-mode `chanscout` already emitted
(channel.recommend / channel.hold, with survey.dwell for context) plus the
primary receiver's log. It does NOT re-implement the policy — that would let a
python shadow drift from the shipping C++ engine — it tallies the engine's own
decisions against an INDEPENDENT impairment label derived from the primary
delivery stream.

Shadow confusion (the honest 4 cells computable from one trace):
  recommend + link impaired   -> escape offered when needed
  recommend + link healthy    -> false move (the rate to drive toward zero)
  hold      + link impaired    -> held during impairment
        ...with a candidate available -> a possible miss
        ...nowhere better / cooldown  -> correctly held
  hold      + link healthy     -> correctly held

Whether a recommended DESTINATION actually delivered better cannot be known
from a single shadow trace — that needs the manual-move validation on a
HELD-OUT run (never tune and score thresholds on the same captures). This tool
reports the false-move rate and the miss candidates; the destination-improved
cell is filled by the paired manual-move experiment documented alongside.

  python3 tests/chanmig_replay.py --scout scout.jsonl \
      [--primary primary.jsonl] [--confusion] [--window 2.0]
"""
import argparse
import json
import sys


def events(path, names=None):
    out = []
    try:
        with open(path, errors="replace") as f:
            for line in f:
                if '"ev":"' not in line:
                    continue
                try:
                    ev = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if names is None or ev.get("ev") in names:
                    out.append(ev)
    except OSError as e:
        print(f"cannot read {path}: {e}", file=sys.stderr)
        sys.exit(2)
    return out


def primary_impairment_series(primary, window_s, loss_thresh):
    """Rolling loss ratio over rx.txhit seq gaps, keyed on the receiver's own
    monotonic uptime (rx.quality carries `t`; rx.txhit does not, so we bucket
    txhits between quality marks)."""
    series = []           # (t_ms, impaired_bool)
    delivered = expected = 0
    prev = None
    for ev in primary:
        e = ev.get("ev")
        if e == "rx.txhit":
            s = ev.get("seq")
            if s is None:
                continue
            delivered += 1
            if prev is not None:
                gap = (s - prev) & 0xFFF
                expected += gap if 0 < gap < 2048 else 1
            else:
                expected += 1
            prev = s
        elif e in ("rx.quality", "link.health"):
            t = ev.get("t", 0)
            loss = 1 - delivered / expected if expected else 0.0
            verdict = ev.get("verdict", "")
            # channel-attributable impairment only (weak/saturated excluded)
            impaired = (loss > loss_thresh and
                        verdict not in ("WEAK", "SATURATED", "NO_SIGNAL"))
            series.append((t, impaired, loss, verdict))
            delivered = expected = 0
            prev = None
    return series


def nearest_impaired(series, t_ms, window_ms):
    """Was the primary link impaired within +/- window of a decision time?
    Decisions carry the scout's monotonic t; the two processes' clocks differ,
    so when no primary t aligns we fall back to the decision's own
    active_domain (which the engine derived from the same primary feed)."""
    best = None
    for (t, impaired, loss, verdict) in series:
        if abs(t - t_ms) <= window_ms:
            if best is None or abs(t - t_ms) < abs(best[0] - t_ms):
                best = (t, impaired, loss, verdict)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scout", required=True)
    ap.add_argument("--primary")
    ap.add_argument("--confusion", action="store_true")
    ap.add_argument("--window", type=float, default=2.0,
                    help="decision<->primary correlation window (s)")
    ap.add_argument("--loss-thresh", type=float, default=0.05)
    args = ap.parse_args()

    decisions = events(args.scout, ("channel.recommend", "channel.hold"))
    print(f"{len(decisions)} decisions in {args.scout}")
    by_reason = {}
    recommends = 0
    for d in decisions:
        r = d.get("reason", "?")
        by_reason[r] = by_reason.get(r, 0) + 1
        if d["ev"] == "channel.recommend":
            recommends += 1
    print(f"  recommends={recommends} holds={len(decisions)-recommends}")
    for r, n in sorted(by_reason.items(), key=lambda kv: -kv[1]):
        print(f"    {n:5d}  {r}")

    if not args.confusion:
        return

    # Independent impairment label. Prefer the primary log; fall back to the
    # decision's own active_domain field when clocks don't align.
    series = []
    if args.primary:
        series = primary_impairment_series(
            events(args.primary, ("rx.txhit", "rx.quality", "link.health")),
            args.window, args.loss_thresh)
        print(f"primary impairment series: {len(series)} windows, "
              f"{sum(1 for s in series if s[1])} impaired")

    cells = {"rec_impaired": 0, "rec_healthy": 0,
             "hold_impaired_miss": 0, "hold_impaired_ok": 0,
             "hold_healthy": 0, "unlabeled": 0}
    # reasons that mean "held but a move was genuinely available"
    MISS_REASONS = {"HoldCooldown", "HoldImprovementMargin"}
    for d in decisions:
        t = d.get("t", 0)
        lab = nearest_impaired(series, t, int(args.window * 1000)) if series else None
        if lab is not None:
            impaired = lab[1]
        elif "active_domain" in d:
            impaired = d["active_domain"] == "channel"
        else:
            cells["unlabeled"] += 1
            continue
        if d["ev"] == "channel.recommend":
            cells["rec_impaired" if impaired else "rec_healthy"] += 1
        else:
            if impaired:
                miss = d.get("reason") in MISS_REASONS
                cells["hold_impaired_miss" if miss else "hold_impaired_ok"] += 1
            else:
                cells["hold_healthy"] += 1

    print("\nshadow confusion (single-trace, destination-improved needs the "
          "held-out manual-move run):")
    print(f"  recommend + impaired   {cells['rec_impaired']:5d}  (escape offered)")
    print(f"  recommend + healthy    {cells['rec_healthy']:5d}  "
          f"{'<-- FALSE MOVES' if cells['rec_healthy'] else '(none)'}")
    print(f"  hold + impaired (move avail) {cells['hold_impaired_miss']:5d}  (possible miss)")
    print(f"  hold + impaired (no better)  {cells['hold_impaired_ok']:5d}  (correctly held)")
    print(f"  hold + healthy         {cells['hold_healthy']:5d}  (correctly held)")
    if cells["unlabeled"]:
        print(f"  unlabeled              {cells['unlabeled']:5d}")
    total_rec = cells["rec_impaired"] + cells["rec_healthy"]
    if total_rec:
        fmr = 100 * cells["rec_healthy"] / total_rec
        print(f"\nfalse-move rate among recommendations: {fmr:.1f}%")


if __name__ == "__main__":
    main()
