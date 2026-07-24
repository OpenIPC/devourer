#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = []
# ///
"""Analyse an rxq_starve.sh capture for issue #330.

Reads one or more capture directories (each holding rx.jsonl + meta.json) and
reports, per run:

  * delivery = distinct rx.seq counters received / the TX counter range they
    span (ground truth from the payload counter — independent of tx.stats),
  * the gap-width histogram (a host FIFO overflow drops a whole aggregate, so
    its gaps cluster at k*pkt_cnt; an RF loss drops single frames, width ~1),
  * the rx.ring armed-depth telemetry (min depth, fraction of windows the depth
    collapsed to 0, worst inline-consume latency),
  * the discriminator verdict: a loss cluster is HOST-STARVATION when the
    armed depth collapsed to 0 in the same host-time window as the gap; it is
    RF/other when the depth held up.

Usage:  tests/rxq_analyze.py DIR [DIR2 ...]
        (multiple dirs print a comparison table — e.g. async vs a fix mode)
"""
import json
import os
import sys
from collections import Counter


def load(path):
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or not line.startswith("{"):
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                pass  # a torn final line under FLUSH=0; skip it
    return rows


def analyse(dirpath):
    rx = load(os.path.join(dirpath, "rx.jsonl"))
    meta = {}
    mp = os.path.join(dirpath, "meta.json")
    if os.path.exists(mp):
        try:
            meta = json.loads(open(mp).read().replace("\n", ""))
        except json.JSONDecodeError:
            pass

    seq = [e for e in rx if e.get("ev") == "rx.seq"]
    ring = [e for e in rx if e.get("ev") == "rx.ring"]

    out = {"dir": dirpath, "meta": meta, "n_seq": len(seq), "n_ring": len(ring)}
    if len(seq) < 2:
        out["error"] = "too few rx.seq frames (link down / wrong SA / TX dead)"
        return out

    # --- delivery from the payload counter --------------------------------
    ctrs = sorted({e["pctr"] for e in seq})
    lo, hi = ctrs[0], ctrs[-1]
    span = hi - lo + 1
    got = len(ctrs)
    out["pctr_lo"], out["pctr_hi"], out["span"], out["received"] = lo, hi, span, got
    out["delivery"] = got / span if span else 0.0
    out["lost"] = span - got

    # --- gaps (runs of consecutive missing counters) ----------------------
    # keyed by t (host ms) of the frame that FOLLOWS the gap, so we can align
    # each gap to the rx.ring depth telemetry.
    t_by_ctr = {}
    for e in seq:
        t_by_ctr.setdefault(e["pctr"], e.get("t"))
    gaps = []  # (width, t_after)
    for i in range(1, len(ctrs)):
        d = ctrs[i] - ctrs[i - 1]
        if d > 1:
            gaps.append((d - 1, t_by_ctr.get(ctrs[i])))
    out["n_gaps"] = len(gaps)
    widths = Counter(w for w, _ in gaps)
    out["gap_width_hist"] = dict(sorted(widths.items()))
    out["gap_w1"] = widths.get(1, 0)
    out["gap_wmulti"] = sum(c for w, c in widths.items() if w > 1)

    # --- ring depth telemetry --------------------------------------------
    # empties/completions is the primary, poll-loop-independent signal: the
    # fraction of URB completions that drained the ring to zero posted URBs.
    # RF loss leaves the ring armed (frames don't arrive) -> ~0; host
    # starvation drains it (frames out-race resubmit) -> high. Catastrophic RF
    # ALSO makes wide gaps, so gap width alone can't be the trigger — the empty
    # rate is what separates "consumer/pump was the bottleneck" from "the link
    # was".
    if ring:
        out["cb_max_us"] = max(r.get("cb_max_us", 0) for r in ring)
        out["resubmit_fail"] = max(r.get("resubmit_fail", 0) for r in ring)
        comp = max((r.get("completions", 0) for r in ring), default=0)
        emp = max((r.get("empties", 0) for r in ring), default=0)
        out["completions"], out["empties"] = comp, emp
        out["empty_rate"] = emp / comp if comp else 0.0
        out["ring_min"] = min((r.get("min_armed", -1) for r in ring), default=-1)
        # collapse intervals for the secondary gap-coincidence corroboration
        collapse_iv = [(ring[i - 1].get("t", 0) if i else 0, r.get("t", 0))
                       for i, r in enumerate(ring) if r.get("min_armed", -1) == 0]
    else:
        collapse_iv = []
        out["cb_max_us"] = out["resubmit_fail"] = out["ring_min"] = None
        out["completions"] = out["empties"] = 0
        out["empty_rate"] = None

    def in_collapse(t):
        return t is not None and any(a <= t <= b for a, b in collapse_iv)

    out["gaps_in_collapse"] = sum(1 for _, t in gaps if in_collapse(t))

    # --- stall-window attribution (RF-robust) -----------------------------
    # An injected consumer stall shows as a >=STALL_MS gap in the rx.seq host-t
    # stream; the pctr jump ACROSS that gap is the loss that stall caused. RF
    # loss is time-uniform, so it lands in the non-stall remainder. This
    # separates the host effect from a high RF floor without needing a clean
    # link: spsc-fat keeps the pump armed during a consumer stall (frames queued,
    # no pctr jump) while async/reorder drop them (big jump).
    STALL_T_MS = 12
    seq_t = [(e.get("t"), e["pctr"]) for e in seq if e.get("t") is not None]
    seq_t.sort()
    stall_loss = stalls = 0
    for i in range(1, len(seq_t)):
        dt = seq_t[i][0] - seq_t[i - 1][0]
        if dt >= STALL_T_MS:
            stalls += 1
            stall_loss += max(0, seq_t[i][1] - seq_t[i - 1][1] - 1)
    out["stalls"] = stalls
    out["stall_loss"] = stall_loss
    out["nonstall_loss"] = max(0, out["lost"] - stall_loss)

    # Single-run verdict. cb_max_us is the robust consumer-stall signal: a
    # healthy parser consumes in ~100-200us, so a cb_max of milliseconds means
    # the pump thread stalled in on_data and could not re-arm the ring — the
    # host was the bottleneck, not the link. (RF loss leaves cb_max small.)
    # This catches the injected-spin model; pump PREEMPTION with a fast consumer
    # (e.g. the Quest compositor) shows small cb_max and is proven instead by
    # the DIFFERENTIAL below: same RF, delivery responds to the host lever.
    CB_STALL_US = 5000
    cbmax = out.get("cb_max_us") or 0
    if out["lost"] == 0:
        out["verdict"] = "CLEAN"
    elif cbmax >= CB_STALL_US:
        out["verdict"] = "HOST_STARVATION"
    else:
        out["verdict"] = "RF_OR_OTHER"
    return out


def fmt(o):
    if "error" in o:
        return f"  {os.path.basename(o['dir'])}: {o['error']} (n_seq={o['n_seq']})"
    m = o.get("meta", {})
    cfg = (f"mode={m.get('mode','?')} urbs={m.get('urbs','?')} "
           f"spin={m.get('sink_spin_us','?')}us "
           f"burst={m.get('burst_on_ms','?')}/{m.get('burst_off_ms','?')}ms "
           f"busy={m.get('busy_threads','?')} taskset={m.get('taskset','') or '-'}")
    lines = [
        f"  {os.path.basename(o['dir'])}  [{cfg}]",
        f"    delivery   {o['delivery']*100:6.2f}%   "
        f"received {o['received']}/{o['span']}  lost {o['lost']}",
        f"    gaps       {o['n_gaps']}  (width1={o['gap_w1']} multi={o['gap_wmulti']})  "
        f"hist={o['gap_width_hist']}",
    ]
    if o.get("empty_rate") is not None:
        lines.append(
            f"    ring       empty_rate={o['empty_rate']*100:5.1f}%  "
            f"(empties {o['empties']}/{o['completions']})  "
            f"min_armed={o['ring_min']}  cb_max={o['cb_max_us']}us  "
            f"resubmit_fail={o['resubmit_fail']}")
    if o.get("stalls"):
        lines.append(
            f"    stalls     {o['stalls']} detected  "
            f"stall_loss={o['stall_loss']}  nonstall(RF)_loss={o['nonstall_loss']}")
    lines.append(f"    VERDICT    {o['verdict']}")
    return "\n".join(lines)


def main(argv):
    dirs = argv[1:]
    if not dirs:
        print(__doc__)
        return 2
    results = [analyse(d) for d in dirs]
    for o in results:
        print(fmt(o))
        # machine-readable one-liner for scripting
        slim = {k: o.get(k) for k in (
            "delivery", "received", "span", "lost", "n_gaps", "gap_w1",
            "gap_wmulti", "ring_min", "empty_rate", "empties", "completions",
            "cb_max_us", "resubmit_fail", "gaps_in_collapse", "verdict")}
        slim["ev"] = "rxq.result"
        slim["dir"] = o["dir"]
        print(json.dumps(slim))

    # --- differential: host-induced loss vs the best-delivery run ----------
    # With the RF link held constant across runs, the drop in delivery from the
    # best run is loss the HOST caused (ring depth / consumer cost / mode). This
    # is the definitive proof — cleaner than any single-run classifier.
    ok = [o for o in results if "delivery" in o]
    if len(ok) >= 2:
        base = max(ok, key=lambda o: o["delivery"])
        print(f"\n  DIFFERENTIAL (baseline = {os.path.basename(base['dir'])}, "
              f"{base['delivery']*100:.2f}% delivery):")
        for o in ok:
            d = (base["delivery"] - o["delivery"]) * 100
            tag = "" if o is base else f"  host-induced loss +{d:.2f} pts"
            print(f"    {os.path.basename(o['dir']):28s} {o['delivery']*100:6.2f}%"
                  f"  cb_max={o.get('cb_max_us')}us{tag}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
