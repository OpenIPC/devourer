#!/usr/bin/env python3
"""arq_e2e_analyze.py — join the three ARQ end-to-end ledgers.

Inputs (from tests/arq_e2e_delivery.sh):
  --dut    duplex JSONL: rx.seq (host delivery ledger), stream.ctl (phase
           markers in file order), rx.ring (ring telemetry)
  --drone  txdemo JSONL: tx.report (per-frame CCX: ok/retries/tag), tx.stats
  --wit    rxdemo JSONL: rx.seq (independent witness ledger)

The drone's halmac tx.report echoes the descriptor SW_DEFINE low byte ("tag"),
stamped from the same per-frame counter txdemo writes into the payload
("pctr"), so tag == pctr (mod 256). Reports arrive in send order; unwrap the
tag into a relative index, then search the 256-aligned base offset that best
matches the observed pctr sets (the true base reconstructs the delivered set
almost perfectly; a wrong base matches ~1/256 at random).

Verdict per phase: of the frames the drone believes DELIVERED (ok=1), how many
never reached the DUT host ledger — split by retries and by whether the
independent witness decoded them (witness-yes = loss inside the DUT chain).
"""
import argparse
import json
import sys
from collections import defaultdict


def iter_ev(path, names):
    want = tuple('{"ev":"%s"' % n for n in names)
    with open(path, errors="replace") as f:
        for idx, line in enumerate(f):
            if not line.startswith(want):
                continue
            try:
                yield idx, json.loads(line)
            except json.JSONDecodeError:
                continue


def load_rx_ledger(path):
    """pctr -> list of dicts (file order); plus event stream for phases."""
    ledger = defaultdict(list)
    ctls = []          # file line idx of each stream.ctl (phase starts)
    rings = []         # (line_idx, ev) for rx.ring
    for idx, ev in iter_ev(path, ("rx.seq", "stream.ctl", "rx.ring")):
        e = ev.get("ev")
        if e == "rx.seq":
            ledger[int(ev["pctr"])].append(
                {"idx": idx, "t": ev.get("t"), "crc": ev.get("crc", 0)})
        elif e == "stream.ctl":
            ctls.append(idx)
        else:
            rings.append((idx, ev))
    return ledger, ctls, rings


def load_reports(path):
    reports = []       # file order: dict(tag, ok, retries, missed)
    submitted = 0
    for _, ev in iter_ev(path, ("tx.report", "tx.stats")):
        if ev.get("ev") == "tx.stats":
            submitted = max(submitted, int(ev.get("submitted", 0)))
            continue
        if "tag" not in ev:      # non-halmac format: tag join impossible
            continue
        reports.append({"tag": int(ev["tag"]), "ok": bool(ev.get("ok")),
                        "retries": int(ev.get("retries", 0)),
                        "missed": int(ev.get("missed", 0))})
    return reports, submitted


def unwrap_tags(reports):
    """Relative frame index per report from mod-256 tag deltas."""
    rel, r = [], 0
    for j, rp in enumerate(reports):
        if j:
            r += (rp["tag"] - reports[j - 1]["tag"]) % 256
        rel.append(r)
    return rel


def find_base(reports, rel, pctr_sets, max_pctr):
    """base = tag0 + 256*m maximizing delivered-set overlap."""
    if not reports:
        return None, 0, 0
    tag0 = reports[0]["tag"]
    ok_rel = [rel[j] for j, rp in enumerate(reports) if rp["ok"]]
    # Membership tests run against the ledgers' own key views — materializing
    # a union would copy ~1M ints per analysis for nothing.
    d0, d1 = pctr_sets
    best, second, best_m = -1, -1, 0
    m_hi = (max_pctr + 4096) // 256 + 2
    # Reports are send-ordered and dense, so the first report's absolute index
    # sits near the earliest delivered pctr — search a window around it first
    # with full precision. The exhaustive fallback scores on a SAMPLE of the
    # reports (a wrong base matches ~1/256 at random, so 2k samples separate
    # right from wrong decisively) and only the winner is re-scored in full —
    # otherwise non-joining logs cost O(m_hi * n_ok) and the analyzer hangs
    # instead of failing fast.
    lo_pctr = min((min(d0) if d0 else 0), (min(d1) if d1 else 0))
    near = range(max(0, (lo_pctr - 2048)) // 256,
                 min(m_hi, (lo_pctr + 2048) // 256 + 1))
    sample = ok_rel[:2000]
    for candidates, pool in ((near, ok_rel), (range(m_hi), sample)):
        for m in candidates:
            base = tag0 + 256 * m
            score = sum(1 for r in pool
                        if (base + r) in d0 or (base + r) in d1)
            if score > best:
                second, best, best_m = best, score, m
            elif score > second:
                second = score
        if best >= max(10, len(pool) // 2):
            break  # strong hit — skip / end the wider scan
        best, second = -1, -1  # sampled scores are not comparable to full ones
    full = sum(1 for r in ok_rel
               if (tag0 + 256 * best_m + r) in d0
               or (tag0 + 256 * best_m + r) in d1)
    return tag0 + 256 * best_m - rel[0], full, second


def phase_names(phases, cycles):
    per = [p.strip() for p in phases.split(",") if p.strip()]
    return [f"{p}" for _ in range(cycles) for p in per]


def phase_label(p, names):
    if p == -1:
        return "warmup"
    if p == -3:
        return "boundary"  # undelivered, nearest neighbours straddle phases
    return names[p] if p is not None and 0 <= p < len(names) else f"?{p}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dut", required=True)
    ap.add_argument("--drone", required=True)
    ap.add_argument("--wit", required=True)
    ap.add_argument("--phases", default="6M:10,idle,6M:30,idle")
    ap.add_argument("--cycles", type=int, default=5)
    ap.add_argument("--detail", type=int, default=30,
                    help="max acked-undelivered rows to print")
    a = ap.parse_args()

    dut, ctls, rings = load_rx_ledger(a.dut)
    wit, _, _ = load_rx_ledger(a.wit)
    reports, submitted = load_reports(a.drone)
    if not reports:
        print("VERDICT: NO tx.report events with tags — nothing to join")
        return 3

    rel = unwrap_tags(reports)
    max_pctr = max(max(dut, default=0), max(wit, default=0))
    base, best, second = find_base(reports, rel,
                                   (dut.keys(), wit.keys()), max_pctr)
    n_ok = sum(1 for r in reports if r["ok"])
    print(f"# reports={len(reports)} ok={n_ok} submitted={submitted} "
          f"dut_pctrs={len(dut)} wit_pctrs={len(wit)}")
    # At ~100% delivery every 256-shift matches nearly everything, so the
    # runner-up margin alone can't certify the base; the per-frame ledger
    # identity printed after the join (dut_pctrs vs ok+drop_dlvd-au) is the
    # real alignment proof.
    margin_ok = second < 0 or best > 2 * max(second, 1) or \
        (best - second) > max(20, n_ok // 100)
    print(f"# tag-align: base={base} matched={best}/{n_ok} "
          f"(runner-up {second}) — "
          f"{'OK' if margin_ok else 'WEAK-MARGIN (see ledger identity below)'}")
    if best < max(10, n_ok // 4):
        print("VERDICT: tag alignment failed — ledgers don't join; "
              "check SA gates / report coverage")
        return 3

    # Phase attribution: each DUT rx.seq line idx -> phase via ctl markers.
    names = phase_names(a.phases, a.cycles)
    if len(ctls) != len(names):
        print(f"# WARNING: {len(ctls)} stream.ctl markers vs "
              f"{len(names)} expected phases — trailing phases truncated")
        names = names[:len(ctls)]

    def phase_of_idx(idx):
        p = -1
        for i, c in enumerate(ctls):
            if idx >= c:
                p = i
            else:
                break
        return p            # -1 = warmup

    # Delivered frame -> phase (first delivery's line idx).
    k_phase = {}
    for k, hits in dut.items():
        k_phase[k] = phase_of_idx(hits[0]["idx"])
    delivered_sorted = sorted(k_phase)

    def attribute(k):
        """Phase of an UNdelivered frame k: nearest delivered neighbours.
        Neighbours straddling a phase edge get the distinct 'boundary' bucket
        (-3) — silently picking a side would bias per-phase counts."""
        import bisect
        i = bisect.bisect_left(delivered_sorted, k)
        lo = delivered_sorted[i - 1] if i else None
        hi = delivered_sorted[i] if i < len(delivered_sorted) else None
        if lo is None and hi is None:
            return -1
        if lo is None:
            return k_phase[hi]
        if hi is None:
            return k_phase[lo]
        pl, ph = k_phase[lo], k_phase[hi]
        return pl if pl == ph else -3

    # Ring telemetry per phase: min armed depth + empties/completions deltas.
    ring_by_phase = defaultdict(lambda: {"min_armed": None, "empties": 0,
                                         "completions": 0})
    prev = {}
    for idx, ev in rings:
        p = phase_of_idx(idx)
        st = ring_by_phase[p]
        ma = ev.get("min_armed")
        if ma is not None:
            st["min_armed"] = ma if st["min_armed"] is None else min(
                st["min_armed"], ma)
        for f in ("empties", "completions"):
            v = int(ev.get(f, 0))
            st[f] += max(0, v - prev.get(f, v))
            prev[f] = v

    # The join. Frames in the last TAIL_GUARD indices of the run are excluded
    # from the headline: the receivers' event streams end by process kill, so
    # a missing pctr there is indistinguishable from stream-tail truncation
    # (measured: 19 consecutive "losses" at the exact end of a run that were
    # an unflushed-stdout artifact, not RF or USB).
    TAIL_GUARD = 512
    max_index = max(base + rel[-1], max(dut, default=0), max(wit, default=0))
    tail_cutoff = max_index - TAIL_GUARD
    # On a run shorter than 2x the guard the tail window swallows most of the
    # frames and a NOT-REPRODUCED verdict would be vacuous — refuse to conclude
    # rather than silently reclassify real losses as truncation.
    short_run = max_index < 2 * TAIL_GUARD
    if short_run:
        print(f"# WARNING: run spans only {max_index + 1} frame indices "
              f"(< 2x TAIL_GUARD={TAIL_GUARD}) — verdict will be "
              f"INCONCLUSIVE-SHORT-RUN")
    # Per-frame reconciliation: under BlockAck a retransmitted MPDU can emit
    # MULTIPLE reports for the same tag (data_tx_cnt incrementing per attempt
    # — vendor-confirmed per-MPDU semantics). The frame's verdict is its LAST
    # report's state, its retries the maximum seen; a per-frame-ACK run has
    # exactly one report per frame, so this reduces to the identity there.
    frames = {}
    for j, rp in enumerate(reports):
        k = base + rel[j]
        f = frames.get(k)
        if f is None:
            frames[k] = {"ok": rp["ok"], "retries": rp["retries"],
                         "reports": 1}
        else:
            f["ok"] = rp["ok"]  # last report wins
            f["retries"] = max(f["retries"], rp["retries"])
            f["reports"] += 1
    multi = sum(1 for f in frames.values() if f["reports"] > 1)
    if multi:
        print(f"# BA reconciliation: {multi} frames carried multiple reports "
              f"(last-state-wins, max-retries)")

    per = defaultdict(lambda: defaultdict(int))
    detail = []
    for k in sorted(frames):
        rp = frames[k]
        in_dut = k in dut
        in_wit = k in wit
        p = k_phase.get(k) if in_dut else attribute(k)
        st = per[p]
        st["reports"] += 1
        st["retries_sum"] += rp["retries"]
        if rp["ok"]:
            st["ok"] += 1
            if rp["retries"] > 0:
                st["ok_retried"] += 1
            if not in_dut:
                if k >= tail_cutoff:
                    st["tail_suspect"] += 1
                    continue
                st["acked_undelivered"] += 1
                if rp["retries"] > 0:
                    st["au_retried"] += 1
                if in_wit:
                    st["au_witnessed"] += 1
                if len(detail) < a.detail:
                    detail.append((k, rp["retries"], p, in_wit))
        else:
            st["dropped"] += 1
            if in_dut:
                st["dropped_but_delivered"] += 1
    for k, hits in dut.items():
        if len(hits) > 1:
            per[k_phase[k]]["dup_delivered"] += 1

    print()
    hdr = (f"{'phase':>10} {'reports':>8} {'ok%':>6} {'ok_rtry':>8} "
           f"{'ACKED_UNDELIV':>14} {'au_rtry':>8} {'au_wit':>7} "
           f"{'drop':>6} {'drop_dlvd':>9} {'dupD':>5} "
           f"{'min_armed':>9} {'empties':>8}")
    print(hdr)
    total_au = 0
    for p in sorted(per):
        st = per[p]
        name = phase_label(p, names)
        okp = 100.0 * st["ok"] / st["reports"] if st["reports"] else 0.0
        rg = ring_by_phase.get(p, {})
        total_au += st["acked_undelivered"]
        print(f"{name:>10} {st['reports']:>8} {okp:>6.1f} "
              f"{st['ok_retried']:>8} {st['acked_undelivered']:>14} "
              f"{st['au_retried']:>8} {st['au_witnessed']:>7} "
              f"{st['dropped']:>6} {st['dropped_but_delivered']:>9} "
              f"{st['dup_delivered']:>5} "
              f"{str(rg.get('min_armed', '-')):>9} "
              f"{rg.get('empties', 0):>8}")

    print()
    if detail:
        print("first acked-undelivered frames (k, retries, phase, witnessed):")
        for k, r, p, w in detail:
            name = phase_label(p, names)
            print(f"  k={k} retries={r} phase={name} wit={'Y' if w else 'n'}")
    # Ledger identity: every ok'd frame must be delivered (minus the au set),
    # plus the ok=0-but-delivered strays. Holding to a few frames certifies
    # both the base alignment and the ledgers themselves.
    tot_dd = sum(st["dropped_but_delivered"] for st in per.values())
    tot_tail = sum(st["tail_suspect"] for st in per.values())
    # Reconciled-frame ok, not raw report ok: under BA one frame can carry
    # several reports and the identity is a per-FRAME conservation law.
    ok_frames = sum(1 for f in frames.values() if f["ok"])
    ident = ok_frames - total_au - tot_tail + tot_dd
    print(f"ledger identity: ok - acked_undelivered - tail_suspect "
          f"+ dropped_but_delivered = {ident} vs dut_pctrs = {len(dut)} "
          f"(delta {len(dut) - ident})")
    if tot_tail:
        print(f"tail_suspect: {tot_tail} ok'd frames missing within the last "
              f"{TAIL_GUARD} indices — excluded (stream-end truncation)")
    if total_au > 0:
        verdict = "REPRODUCED"
    elif short_run:
        verdict = "INCONCLUSIVE-SHORT-RUN"
    else:
        verdict = "NOT-REPRODUCED"
    print(f"\nVERDICT: {verdict} — acked_undelivered={total_au} across all "
          f"phases (drone said ok, DUT host never delivered)")
    # separators: machine events must be the grep-able {"ev":"name",...} form
    # (docs/logging.md) — default json.dumps inserts spaces.
    # flush: the one machine-event line lands atomically even if a consumer
    # reads the stream live. The human report around it is deliberately on
    # stdout — this is an offline report generator (the harness tees it into
    # report.txt), same contract as rxq_analyze.py / pp109_starve_analyze.py,
    # not a demo's runtime event plane.
    # Self-describing counts: "reports"/"ok_reports" are raw tx.report events,
    # "ok_frames" is the per-tag reconciled frame count the identity uses —
    # equal at one report per frame, divergent once BA multi-reports exist.
    print(json.dumps({"ev": "arqe2e.verdict", "verdict": verdict,
                      "acked_undelivered": total_au,
                      "tail_suspect": tot_tail,
                      "reports": len(reports),
                      "ok_reports": sum(1 for rp in reports if rp["ok"]),
                      "ok_frames": ok_frames,
                      "multi_report_frames": multi,
                      "dut_pctrs": len(dut), "wit_pctrs": len(wit),
                      "base": base}, separators=(",", ":")), flush=True)
    return 3 if verdict == "INCONCLUSIVE-SHORT-RUN" else 0


if __name__ == "__main__":
    sys.exit(main())
