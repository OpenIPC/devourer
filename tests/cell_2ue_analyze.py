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
     the UE was listening, in-slot ratio.
  witness: every cell frame mapped onto the DU clock (a robust line through
     the DU beacons' egress TSF vs the witness RX stamp, so every frame maps
     to its own air start), judged by its mid-air instant: frames outside
     their own slot, overlaps (two frames whose air intervals intersect, from
     6 Mb/s airtime), and per-UE DL lateness (air start - DL slot start).
     Judged from --settle-s after the first UE uplink to the last one (while
     no UE runs, the DU's ACKed DL is retried and the retries leave the slot),
     on first copies; DL retransmissions in that window count separately.
The verdict line (ev cell.verdict) is in_slot == 1.0 in both directions and
zero witness overlaps.

--selftest runs the analysis on a synthetic run (no files).
"""
import argparse
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
    late_sub = [e["late_us"] for e in du if e.get("ev") == "cell.dl"]
    rep["dl_submit_late_us"] = {"p50": pct(late_sub, 0.5), "p99": pct(late_sub, 0.99),
                               "max": max(late_sub, default=float("nan"))}
    for stream in ues:
        rx = [e for e in stream if e.get("ev") == "cell.dl.rx"]
        if not rx:
            continue
        ue = rx[0]["ue"]
        lo, hi = min(e["sf"] for e in rx), max(e["sf"] for e in rx)
        sent = sum(1 for e in dl_sent if e["ue"] == ue and lo <= e["sf"] <= hi)
        rep["dl"][str(ue)] = {
            "sent": sent, "received": len(rx),
            "in_slot": round(sum(e["in_slot"] for e in rx) / len(rx), 4),
            "phase_p50_us": pct([e["phase_us"] for e in rx], 0.5),
        }

    # Witness: onto the DU clock via the DU beacons.
    if witness:
        w = [e for e in witness if e.get("ev") == "cell.w"]
        t = unwrap32([e["tsfl"] for e in w])
        for e, v in zip(w, t):
            e["t"] = v
        b = [e for e in w if e["k"] == "bcn" and e.get("egress")]
        if len(b) >= 8:
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
            overlaps = sum(1 for (t0, d0, *_), (t1, *_r) in zip(frames, frames[1:]) if t1 < t0 + d0)
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
    dl_ok = all(v["in_slot"] == 1.0 for v in rep["dl"].values()) and len(rep["dl"]) > 0
    w_ok = rep["witness"] is None or rep["witness"]["overlaps"] == 0
    rep["verdict"] = {"ul_in_slot": ul_ok, "dl_in_slot": dl_ok, "no_overlaps": w_ok,
                      "pass": ul_ok and dl_ok and w_ok}
    return rep


def selftest():
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
