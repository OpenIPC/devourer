#!/usr/bin/env python3
"""cell_2ue_analyze.py - KPIs for an examples/cell run (M1, #255).

Inputs are the JSONL streams of the DU, each UE and the witness:

  cell_2ue_analyze.py --du du.jsonl --ue ue1.jsonl --ue ue2.jsonl \
      [--witness w.jsonl] [--settle-s 10]

Reported as distributions, per the EPIC #254 KPI rule:
  UL (the DU's own measurement): per UE, arrivals vs superframes elapsed, the
     arrival error against the UL target (rms/p99/max) and the in-slot ratio,
     after the first --settle-s seconds (TA convergence).
  DL (each UE's measurement): frames received vs sent over the superframes
     the UE says it was listening (cell.ue.done listen_from_sf..listen_to_sf),
     in-slot ratio. Every UE the DU's grid schedules must report: a missing
     or empty UE stream fails the DL verdict.
  witness: every cell frame mapped onto the DU clock (a robust line through
     the DU beacons' egress TSF vs the witness RX stamp, so every frame maps
     to its own air start): a frame is in its own slot only if its whole air
     interval [start, start + airtime] lies inside that slot; overlaps (two frames whose air intervals intersect, from
     6 Mb/s airtime), and per-UE DL lateness (air start - DL slot start).
     Judged from --settle-s after the first UE uplink to the last one (while
     no UE runs, the DU's ACKed DL is retried and the retries leave the slot),
     on first copies; DL retransmissions in that window count separately.
  harq (M2, when the streams carry it): per UE, distinct DL frames the DU
     originated (attempt 1) in the UE's listening window vs those the UE
     received as new (its HARQ duplicate suppression), retransmissions, the
     DU's HARQ counters and the DL rate over time (cell.la).
The verdict line (ev cell.verdict) is in_slot == 1.0 in both directions and
zero witness overlaps. A witness log that was supplied but cannot be mapped
(fewer than 8 DU beacons) is an error and fails the verdict.

--selftest runs the analysis on a synthetic run (no files).
"""
import argparse
import heapq
import json
import math
import sys


def load(path):
    out = []
    with open(path, errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line.startswith("{"):
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return out


def pct(v, q):
    if not v:
        return float("nan")
    s = sorted(v)
    k = min(len(s) - 1, max(0, int(math.ceil(q * len(s))) - 1))
    return s[k]


def rms(v):
    return math.sqrt(sum(x * x for x in v) / len(v)) if v else float("nan")


def airtime_us(mpdu_len, rate=6):
    """802.11a OFDM airtime of an MPDU (incl. the 4-byte FCS)."""
    ndbps = 4 * rate
    bits = 16 + 8 * (mpdu_len + 4) + 6
    return 20 + 4 * -(-bits // ndbps)


def robust_line(xs, ys):
    """y = a*x + b by least squares, then refit without residuals > 5*MAD."""
    def fit(px, py):
        n = len(px)
        mx, my = sum(px) / n, sum(py) / n
        sxx = sum((x - mx) ** 2 for x in px)
        a = sum((x - mx) * (y - my) for x, y in zip(px, py)) / sxx if sxx else 1.0
        return a, my - a * mx
    a, b = fit(xs, ys)
    res = [y - (a * x + b) for x, y in zip(xs, ys)]
    med = sorted(abs(r) for r in res)[len(res) // 2]
    keep = [(x, y) for x, y, r in zip(xs, ys, res) if abs(r) <= max(5 * med, 2.0)]
    if len(keep) >= 8:
        a, b = fit([k[0] for k in keep], [k[1] for k in keep])
    return a, b


def count_overlaps(intervals):
    """Number of intersecting PAIRS among (start, duration) intervals: a sweep
    keeping every earlier interval still on air, so one long frame that
    overlaps several later ones counts each pair."""
    active, n = [], 0  # min-heap of end times
    for t, d in sorted(intervals):
        while active and active[0] <= t:
            heapq.heappop(active)
        n += len(active)
        heapq.heappush(active, t + d)
    return n


def unwrap32(vals):
    out, hi, prev = [], 0, None
    for v in vals:
        if prev is not None and v < prev - (1 << 31):
            hi += 1 << 32
        out.append(v + hi)
        prev = v
    return out


def analyze(du, ues, witness, settle_s):
    slots = sorted((e for e in du if e.get("ev") == "cell.slot"), key=lambda e: e["idx"])
    start = next((e for e in du if e.get("ev") == "cell.du.start"), None)
    if not slots or not start:
        return {"error": "DU stream has no cell.du.start / cell.slot"}
    P = start["period_us"]
    rep = {"period_us": P, "ul": {}, "dl": {}, "witness": None}

    def slot_of(phase):
        for s in slots:
            if s["start_us"] <= phase < s["start_us"] + s["len_us"]:
                return s
        return None

    # UL, from the DU.
    ul = [e for e in du if e.get("ev") == "cell.ul"]
    if ul:
        sf0 = min(e["sf"] for e in ul)
        settle_sf = int(settle_s * 1e6 / P)
        for ue in sorted({e["ue"] for e in ul}):
            mine = [e for e in ul if e["ue"] == ue]
            late = [e for e in mine if e["sf"] >= sf0 + settle_sf]
            sfs = {e["sf"] for e in late}
            span = (max(sfs) - min(sfs) + 1) if sfs else 0
            errs = [e["err_us"] for e in late]
            rep["ul"][str(ue)] = {
                "arrivals": len(late), "superframes": span,
                "missed": span - len(sfs) if span else 0,
                "dup": len(late) - len(sfs),
                "err_rms_us": round(rms(errs), 2),
                "err_p99_abs_us": pct([abs(x) for x in errs], 0.99),
                "err_max_abs_us": max((abs(x) for x in errs), default=float("nan")),
                "in_slot": round(sum(e["in_slot"] for e in late) / len(late), 4) if late else 0.0,
            }

    # DL, from the UEs; sent = DU submissions within the UE's listening span.
    dl_sent = [e for e in du if e.get("ev") == "cell.dl" and e.get("ok")]
    late_sub = [e["late_us"] for e in du if e.get("ev") == "cell.dl" and "late_us" in e]
    rep["dl_submit_late_us"] = {"p50": pct(late_sub, 0.5), "p99": pct(late_sub, 0.99),
                               "max": max(late_sub, default=float("nan"))}
    expected = sorted({s["ue"] for s in slots if s["kind"] == 1})
    by_ue = {}
    for stream in ues:
        ident = [e["ue"] for e in stream if e.get("ev") in ("cell.dl.rx", "cell.ue.done", "cell.ue.epoch")]
        if ident:
            by_ue[ident[0]] = stream
    for ue in expected:
        stream = by_ue.get(ue)
        if stream is None:
            rep["dl"][str(ue)] = {"error": "no stream for this UE", "in_slot": 0.0}
            continue
        rx = [e for e in stream if e.get("ev") == "cell.dl.rx"]
        done = next((e for e in stream if e.get("ev") == "cell.ue.done"), None)
        if done and done.get("listen_from_sf", -1) >= 0 and done.get("listen_to_sf", -1) >= 0:
            lo, hi = done["listen_from_sf"], done["listen_to_sf"]
        elif rx:
            lo, hi = min(e["sf"] for e in rx), max(e["sf"] for e in rx)
        else:
            lo, hi = 0, -1
        sent = sum(1 for e in dl_sent if e["ue"] == ue and lo <= e["sf"] <= hi)
        got = [e for e in rx if lo <= e["sf"] <= hi]
        rep["dl"][str(ue)] = {
            "sent": sent, "received": len(got),
            "in_slot": round(sum(e["in_slot"] for e in got) / len(got), 4) if got else 0.0,
            "phase_p50_us": pct([e["phase_us"] for e in got], 0.5),
        }

    # HARQ / delivery (M2).
    # successful submissions only: a refused send_packet put nothing on air
    dl_all = [e for e in du if e.get("ev") == "cell.dl" and "attempt" in e and e.get("ok")]
    if dl_all:
        rep["harq"] = {}
        for ue in expected:
            stream = by_ue.get(ue, [])
            done = next((e for e in stream if e.get("ev") == "cell.ue.done"), None)
            if done and done.get("listen_from_sf", -1) >= 0:
                lo, hi = done["listen_from_sf"], done["listen_to_sf"]
            else:
                lo, hi = 0, -1
            orig = {e["seq"] for e in dl_all if e["ue"] == ue and e["attempt"] == 1
                    and lo <= e["sf"] <= hi}
            got = {e["seq"] for e in stream if e.get("ev") == "cell.dl.rx" and e.get("new")}
            dups = sum(1 for e in stream if e.get("ev") == "cell.dl.rx" and e.get("new") == 0)
            retx = sum(1 for e in dl_all if e["ue"] == ue and e["attempt"] > 1
                       and lo <= e["sf"] <= hi)
            st = next((e for e in du if e.get("ev") == "cell.harq" and e["ue"] == ue), {})
            rates = [(e["rate"]) for e in du if e.get("ev") == "cell.la" and e["ue"] == ue]
            rep["harq"][str(ue)] = {
                "originated": len(orig), "delivered": len(orig & got),
                "delivery": round(len(orig & got) / len(orig), 4) if orig else 0.0,
                "retx": retx, "dup_rx": dups,
                "dropped": st.get("dropped"), "harq_on": st.get("harq"),
                "rate_first": dl_all[0]["rate"] if dl_all else None,
                "rate_last": st.get("rate"),
                "rate_steps": [r for i, r in enumerate(rates) if i == 0 or r != rates[i - 1]],
            }

    # Witness: onto the DU clock via the DU beacons.
    if witness:
        w = [e for e in witness if e.get("ev") == "cell.w"]
        t = unwrap32([e["tsfl"] for e in w])
        for e, v in zip(w, t):
            e["t"] = v
        b = [e for e in w if e["k"] == "bcn" and e.get("egress")]
        if len(b) < 8:
            rep["witness"] = {"error": f"witness has {len(b)} DU beacons; 8 needed to map it"}
        else:
            # DU egress = a * witness_t + c: the line maps every frame's
            # witness stamp to its air start on the DU clock (the stamp's
            # latch point within a PPDU is the same for all frames).
            a, c = robust_line([e["t"] for e in b], [e["egress"] for e in b])
            ul_t = [a * e["t"] + c for e in w if e["k"] == "ul"]
            t_settle = (min(ul_t) + settle_s * 1e6) if ul_t else float("-inf")
            t_end = max(ul_t) if ul_t else float("inf")
            frames = []
            dl_late = {}
            seen_dl = set()
            retx = {}
            for e in w:
                du_t = a * e["t"] + c
                if du_t < t_settle or du_t > t_end:
                    continue
                if e["k"] == "dl":
                    key = (e["st"], e["seq"])
                    if key in seen_dl:
                        retx[e["st"]] = retx.get(e["st"], 0) + 1
                        continue
                    seen_dl.add(key)
                air = airtime_us(e["len"])
                s = slot_of((du_t + air / 2.0) % P)
                if s is not None:
                    # the whole air interval must sit inside the slot
                    off = (du_t - s["start_us"]) % P
                    if off > P / 2:
                        off -= P
                    if off < 0 or off + air > s["len_us"]:
                        s = None
                if e["k"] == "bcn":
                    want = ("bcn", 0)
                elif e["k"] == "dl":
                    want = ("dl", e["st"])
                else:
                    want = ("ul", e["st"])
                kind = {0: "bcn", 1: "dl", 3: "ul"}.get(s["kind"], "other") if s else "idle"
                ok = s is not None and kind == want[0] and (want[0] == "bcn" or s["ue"] == want[1])
                frames.append((du_t, air, e["k"], e["st"], ok))
                if e["k"] == "dl":
                    own = next((x for x in slots if x["kind"] == 1 and x["ue"] == e["st"]), None)
                    if own:
                        late = (du_t - own["start_us"]) % P
                        if late > P / 2:
                            late -= P
                        dl_late.setdefault(e["st"], []).append(late)
            frames.sort()
            overlaps = count_overlaps([(f[0], f[1]) for f in frames])
            by = {}
            for _, _, k, st, ok in frames:
                key = f"{k}{st if k != 'bcn' else ''}"
                n, g = by.get(key, (0, 0))
                by[key] = (n + 1, g + (1 if ok else 0))
            rep["witness"] = {
                "frames": len(frames), "overlaps": overlaps,
                "dl_retx": {str(k): v for k, v in sorted(retx.items())},
                "in_own_slot": {k: round(g / n, 4) for k, (n, g) in sorted(by.items())},
                "counts": {k: n for k, (n, _) in sorted(by.items())},
                "dl_late_us": {str(k): {"p50": round(pct(v, 0.5)), "p90": round(pct(v, 0.9)),
                                        "p99": round(pct(v, 0.99)), "max": round(max(v))}
                               for k, v in sorted(dl_late.items())},
            }

    ul_ok = all(v["in_slot"] == 1.0 for v in rep["ul"].values()) and len(rep["ul"]) > 0
    dl_ok = (all(v.get("in_slot") == 1.0 and v.get("received", 0) > 0 for v in rep["dl"].values())
             and len(rep["dl"]) > 0)
    w_ok = (rep["witness"] is None if witness is None
            else rep["witness"] is not None and "error" not in rep["witness"]
            and rep["witness"]["overlaps"] == 0)
    rep["verdict"] = {"ul_in_slot": ul_ok, "dl_in_slot": dl_ok, "no_overlaps": w_ok,
                      "pass": ul_ok and dl_ok and w_ok}
    return rep


def selftest():
    # One long frame over two later ones that do not overlap each other: 2 pairs.
    assert count_overlaps([(0, 100), (10, 20), (50, 20)]) == 2
    assert count_overlaps([(0, 10), (10, 10)]) == 0  # touching is not overlapping
    assert count_overlaps([(0, 30), (5, 30), (10, 30)]) == 3
    P = 102400
    slots = [(0, 0, 0, 1500), (1, 1, 1, 4000), (2, 1, 2, 4000), (3, 2, 0, 1000),
             (4, 3, 1, 2000), (5, 3, 2, 2000)]
    starts = {}
    acc = 0
    du = [{"ev": "cell.du.start", "period_us": P}]
    for idx, kind, ue, ln in slots:
        du.append({"ev": "cell.slot", "idx": idx, "kind": kind, "ue": ue,
                   "start_us": acc, "len_us": ln})
        starts[(kind, ue)] = acc
        acc += ln
    w = []
    ue_streams = [[], []]
    for sf in range(200):
        base = sf * P
        w.append({"ev": "cell.w", "k": "bcn", "st": 0, "tsfl": base + 22 + 5000,
                  "egress": base + 22, "len": 90})  # departs 22 us after TBTT
        for ue in (1, 2):
            du.append({"ev": "cell.dl", "ue": ue, "sf": sf, "ok": 1, "late_us": 3})
            du.append({"ev": "cell.ul", "ue": ue, "sf": sf, "err_us": (sf % 3) - 1,
                       "in_slot": 1})
            ue_streams[ue - 1].append({"ev": "cell.dl.rx", "ue": ue, "sf": sf,
                                       "phase_us": starts[(1, ue)] + 100, "in_slot": 1})
            w.append({"ev": "cell.w", "k": "dl", "st": ue, "seq": 2 * sf + ue,
                      "tsfl": base + starts[(1, ue)] + 100 + 5000 + 22, "len": 130})
            w.append({"ev": "cell.w", "k": "ul", "st": ue,
                      "tsfl": base + starts[(3, ue)] + 500 + 5000 + 22, "len": 50})
    r = analyze(du, ue_streams, w, settle_s=1)
    assert r["verdict"]["pass"], r
    assert r["ul"]["1"]["missed"] == 0 and r["ul"]["1"]["err_max_abs_us"] == 1, r["ul"]
    assert r["witness"]["overlaps"] == 0 and r["witness"]["in_own_slot"]["ul2"] == 1.0, r
    assert r["dl"]["2"]["received"] == 200 and r["dl"]["2"]["sent"] == 200
    assert r["witness"]["in_own_slot"]["bcn"] == 1.0, r["witness"]
    assert r["witness"]["dl_late_us"]["1"]["p50"] == 122, r["witness"]
    # A retried DL copy after settle is counted as a retransmission, not judged.
    mid = next(e for e in w[len(w) // 2:] if e["k"] == "dl" and e["st"] == 2)
    w_rt = w + [dict(mid, tsfl=mid["tsfl"] + 30000)]
    r3 = analyze(du, ue_streams, w_rt, settle_s=1)
    assert r3["witness"]["dl_retx"] == {"2": 1} and r3["witness"]["in_own_slot"]["dl2"] == 1.0, r3
    # DL sent counts only the superframes the UE was listening.
    r2 = analyze(du, [s[50:] for s in ue_streams], w, settle_s=1)
    assert r2["dl"]["1"]["sent"] == 150 and r2["dl"]["1"]["received"] == 150, r2["dl"]
    # The UE's own listening window wins: a lost FINAL frame is counted.
    s1 = ue_streams[0][:-1] + [{"ev": "cell.ue.done", "ue": 1, "listen_from_sf": 0,
                                "listen_to_sf": 199}]
    r4 = analyze(du, [s1, ue_streams[1]], w, settle_s=1)
    # HARQ section: 10 originated frames, the UE got 8 as new (one twice).
    du_h = [e for e in du if e.get("ev") != "cell.dl"]
    ue_h = []
    for sq in range(10):
        du_h.append({"ev": "cell.dl", "ue": 1, "sf": sq, "seq": sq, "attempt": 1, "rate": 54, "ok": 1})
        if sq in (3, 7):
            du_h.append({"ev": "cell.dl", "ue": 1, "sf": sq + 1, "seq": sq, "attempt": 2,
                         "rate": 54, "ok": 1})
        if sq not in (3, 7):
            ue_h.append({"ev": "cell.dl.rx", "ue": 1, "sf": sq, "seq": sq, "new": 1,
                         "phase_us": 1600, "in_slot": 1})
    ue_h.append({"ev": "cell.dl.rx", "ue": 1, "sf": 5, "seq": 5, "new": 0, "phase_us": 1600,
                 "in_slot": 1})
    ue_h.append({"ev": "cell.ue.done", "ue": 1, "listen_from_sf": 0, "listen_to_sf": 20})
    du_h.append({"ev": "cell.dl", "ue": 1, "sf": 11, "seq": 10, "attempt": 1, "rate": 54, "ok": 0})
    du_h.append({"ev": "cell.harq", "ue": 1, "harq": 1, "dropped": 2, "rate": 24})
    du_h.append({"ev": "cell.la", "ue": 1, "rate": 54})
    du_h.append({"ev": "cell.la", "ue": 1, "rate": 36})
    du_h.append({"ev": "cell.la", "ue": 1, "rate": 36})
    du_h.append({"ev": "cell.la", "ue": 1, "rate": 24})
    rh = analyze(du_h, [ue_h, ue_streams[1]], None, settle_s=1)["harq"]["1"]
    assert rh["originated"] == 10 and rh["delivered"] == 8 and rh["delivery"] == 0.8, rh
    assert rh["retx"] == 2 and rh["dup_rx"] == 1 and rh["rate_steps"] == [54, 36, 24], rh
    assert r4["dl"]["1"]["sent"] == 200 and r4["dl"]["1"]["received"] == 199, r4["dl"]
    # A UE the grid schedules but whose stream is missing fails the DL verdict.
    r5 = analyze(du, [ue_streams[0]], w, settle_s=1)
    assert "error" in r5["dl"]["2"] and not r5["verdict"]["dl_in_slot"], r5["dl"]
    # A supplied witness that cannot be mapped is an error, not a pass.
    r6 = analyze(du, ue_streams, [e for e in w if e["k"] != "bcn"], settle_s=1)
    assert "error" in r6["witness"] and not r6["verdict"]["pass"], r6["witness"]
    # A frame whose midpoint is in its slot but which runs past the slot end fails.
    sl = next(x for x in du if x.get("ev") == "cell.slot" and x["kind"] == 3 and x["ue"] == 2)
    w7 = [dict(e) for e in w]
    for e in w7:
        if e["k"] == "ul" and e["st"] == 2:
            e["tsfl"] += sl["len_us"] - 500 - 30  # starts 30 us before the slot end
    r7 = analyze(du, ue_streams, w7, settle_s=1)
    assert r7["witness"]["in_own_slot"]["ul2"] == 0.0, r7["witness"]
    # Put UE2's uplink into UE1's slot: the witness must flag it and an overlap.
    for e in w:
        if e["k"] == "ul" and e["st"] == 2:
            e["tsfl"] -= 2000 - 10
    r = analyze(du, ue_streams, w, settle_s=1)
    assert r["witness"]["in_own_slot"]["ul2"] == 0.0, r["witness"]
    assert r["witness"]["overlaps"] > 0 and not r["verdict"]["pass"], r["witness"]
    # A missed uplink superframe is counted.
    du2 = [e for e in du if not (e.get("ev") == "cell.ul" and e["ue"] == 1 and e["sf"] == 150)]
    r = analyze(du2, ue_streams, None, settle_s=1)
    assert r["ul"]["1"]["missed"] == 1, r["ul"]
    print("cell_2ue_analyze selftest: OK")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--du")
    ap.add_argument("--ue", action="append", default=[])
    ap.add_argument("--witness")
    ap.add_argument("--settle-s", type=float, default=10.0)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        selftest()
        return 0
    if not a.du:
        ap.error("--du is required")
    rep = analyze(load(a.du), [load(u) for u in a.ue],
                  load(a.witness) if a.witness else None, a.settle_s)
    print(json.dumps({"ev": "cell.report", **rep}, indent=1))
    print(json.dumps({"ev": "cell.verdict", **rep.get("verdict", {})}))
    return 0 if rep.get("verdict", {}).get("pass") else 1


if __name__ == "__main__":
    sys.exit(main())
