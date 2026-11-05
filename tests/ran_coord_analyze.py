#!/usr/bin/env python3
"""ran_coord_analyze.py - KPIs for an M5 (#259) coordination run: two (or
more) co-channel examples/cell DUs under the ran-controller CU.

  ran_coord_analyze.py --cell 1:du1.jsonl:ue1.jsonl --cell 2:du2.jsonl:ue2.jsonl \
      [--witness w.jsonl] [--cu cu.jsonl] [--settle-sf 20]

Each cell's KPIs are judged over its COORDINATED window: from the network
instant its CU schedule took effect (the DU's cell.f1.applied effective_sf,
its own superframe count) + --settle-sf, to the last superframe its UE
listened (cell.ue.done listen_to_sf). The UEs a cell must serve are those of
its last published grant map in the window (cell.grant ok=1): one with no
measurement fails its KPI.
  dl: per UE, frames the DU sent first (attempt 1, ok) in the window vs those
      the UE received on that first attempt; retransmissions the DU sent;
      the DU's DL submit lateness (p50/p90/p99/max).
  ul: per UE, uplinks the DU heard in the window vs superframes in it, and
      the arrival error against the UL target (p50/p90/p99/max |err|).
  instant: the network superframe each DU applied its schedule on (one
      network-wide instant when every DU names the same one).
  control (with --cu): per schedule, superframes from the CU sending it to
      each DU's staged and applied acknowledgement.
  witness (one receiver for every cell): cross-cell air overlaps between
      frames it decoded (6 Mb/s airtime, conservative) and the fraction of
      superframes with one. Each cell's window is mapped onto the witness
      clock through that cell's own beacons (their egress TSF is its clock);
      the judged span is where every cell's window overlaps. A collided
      frame often does not decode at all, so this is a lower bound; the
      delivery KPIs are the main measure.
The verdict (ev ran.verdict): every expected UE's first-attempt DL delivery
and UL ratio at least --min-ratio, every DU on the same instant, and no
witness cross-cell overlap. Output: one JSON event per line (ran.cell,
ran.witness, ran.control, ran.verdict).

--selftest runs the analysis on a synthetic run (no files).
"""
import argparse
import json
import sys

P = 102400


def load(path):
    out = []
    with open(path, errors="replace") as f:
        for line in f:
            line = line.strip()
            if line.startswith("{"):
                try:
                    out.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    return out


def airtime_us(n):
    return 20 + 4 * ((16 + 8 * (n + 4) + 6 + 23) // 24)  # 6 Mb/s, + FCS


def unwrap32(v):
    out, base, prev = [], 0, None
    for x in v:
        if prev is not None and x < prev - (1 << 31):
            base += 1 << 32
        out.append(x + base)
        prev = x
    return out


def robust_line(x, y):
    n = len(x)
    mx, my = sum(x) / n, sum(y) / n
    sxx = sum((a - mx) ** 2 for a in x)
    a = sum((p - mx) * (q - my) for p, q in zip(x, y)) / sxx if sxx else 1.0
    c = my - a * mx
    r = sorted(abs(q - (a * p + c)) for p, q in zip(x, y))
    cut = r[int(0.8 * (n - 1))]
    keep = [(p, q) for p, q in zip(x, y) if abs(q - (a * p + c)) <= cut]
    if len(keep) >= 2 and len(keep) < n:
        return robust_line([p for p, _ in keep], [q for _, q in keep])
    return a, c


def dist(v):
    if not v:
        return None
    s = sorted(v)
    q = lambda f: s[min(len(s) - 1, int(f * (len(s) - 1) + 0.5))]
    return {"n": len(s), "p50": q(0.5), "p90": q(0.9), "p99": q(0.99), "max": s[-1]}


def cell_kpis(cell, du, ue, settle_sf):
    rep = {"cell": cell}
    app = [e for e in du if e.get("ev") == "cell.f1.applied"]
    done = next((e for e in ue if e.get("ev") == "cell.ue.done"), None)
    if not app or not done:
        rep["error"] = "no cell.f1.applied in the DU stream" if not app else "no cell.ue.done"
        return rep
    eff = app[-1]["effective_sf"]
    lo, hi = eff + settle_sf, done["listen_to_sf"]
    rep.update(instant_net_sf=app[-1]["effective_net_sf"], version=app[-1]["version"],
               window_sf=[lo, hi], dl={}, ul={})
    if hi <= lo:
        rep["error"] = "empty window"
        return rep
    grants = [e for e in du if e.get("ev") == "cell.grant" and e.get("ok")
              and e.get("effective_sf", 0) <= hi]
    expected = sorted(int(k) for k in grants[-1]["ta"]) if grants else []
    rep["expected_ues"] = expected
    sent = {}
    retx = {}
    late = {}
    for e in du:
        if e.get("ev") == "cell.dl" and lo <= e["sf"] <= hi and e.get("ok"):
            late.setdefault(e["ue"], []).append(e.get("late_us", 0))
            if e["attempt"] == 1:
                sent.setdefault(e["ue"], set()).add(e["seq"])
            else:
                retx[e["ue"]] = retx.get(e["ue"], 0) + 1
    got = {}
    for e in ue:
        if e.get("ev") == "cell.dl.rx" and e.get("attempt") == 1 and lo <= e["sf"] <= hi:
            got.setdefault(e["ue"], set()).add(e["seq"])
    for u, s in sorted(sent.items()):
        g = got.get(u, set()) & s
        rep["dl"][str(u)] = {"sent_first": len(s), "rx_first": len(g),
                             "ratio": round(len(g) / len(s), 4) if s else 0.0,
                             "retx": retx.get(u, 0), "late_us": dist(late.get(u, []))}
    ul = {}
    err = {}
    for e in du:
        if e.get("ev") == "cell.ul" and lo <= e["sf"] <= hi:
            ul.setdefault(e["ue"], set()).add(e["sf"])
            if "err_us" in e:
                err.setdefault(e["ue"], []).append(abs(e["err_us"]))
    for u, s in sorted(ul.items()):
        rep["ul"][str(u)] = {"heard_sf": len(s), "window_sf": hi - lo + 1,
                             "ratio": round(len(s) / (hi - lo + 1), 4),
                             "abs_err_us": dist(err.get(u, []))}
    for u in expected:  # a UE the cell must serve but no measurement of it
        rep["dl"].setdefault(str(u), {"error": "no DL measured", "ratio": 0.0})
        rep["ul"].setdefault(str(u), {"error": "no UL measured", "ratio": 0.0})
    return rep


def control_kpis(cu):
    """Per schedule: superframes from the CU sending it to each ack."""
    sends = {(e["du"], e["version"]): e["net_sf"] for e in cu if e.get("ev") == "ran.schedule"}
    staged, applied = [], []
    for e in cu:
        if e.get("ev") != "ran.ack" or (e["du"], e["version"]) not in sends:
            continue
        d = e["net_sf"] - sends[(e["du"], e["version"])]
        (staged if e["status"] == "staged" else applied if e["status"] == "applied" else []).append(d)
    return {"schedules": len(sends), "to_staged_sf": dist(staged), "to_applied_sf": dist(applied),
            "rejected": sum(1 for e in cu if e.get("ev") == "ran.ack"
                            and e["status"] in ("stale", "late", "invalid"))}


def analyze(cells, witness=None, settle_sf=20, min_ratio=0.98, cu=None):
    rep = {"cells": [cell_kpis(c, du, ue, settle_sf) for c, du, ue in cells], "witness": None}
    if cu is not None:
        rep["control"] = control_kpis(cu)
    ok_cells = [c for c in rep["cells"] if "error" not in c]
    instants = {c["instant_net_sf"] for c in ok_cells}
    rep["one_instant"] = len(ok_cells) == len(cells) and len(instants) == 1
    if witness is not None and len(ok_cells) == len(cells):
        w = [e for e in witness if e.get("ev") == "cell.w"]
        t = unwrap32([e["tsfl"] for e in w])
        for e, v in zip(w, t):
            e["t"] = v
        # Each cell's window on the witness clock, through its own beacons.
        span, err = [float("-inf"), float("inf")], None
        for (cid, _, _), ck in zip(cells, rep["cells"]):
            b = [e for e in w if e["k"] == "bcn" and e.get("cell") == cid and e.get("egress")]
            if len(b) < 8:
                err = f"{len(b)} beacons of cell {cid}; 8 needed to map it"
                break
            a, c = robust_line([e["t"] for e in b], [e["egress"] for e in b])
            lo, hi = ck["window_sf"]
            span[0] = max(span[0], (lo * P - c) / a)
            span[1] = min(span[1], ((hi + 1) * P - c) / a)
        if err:
            rep["witness"] = {"error": err}
        else:
            frames = []
            for e in w:
                if span[0] <= e["t"] <= span[1]:
                    frames.append((e["t"], airtime_us(e["len"]), e.get("cell"), e["k"], e["st"]))
            frames.sort()
            cross, pairs, hit_sf = 0, {}, set()
            for i, f in enumerate(frames):
                for g in frames[i + 1:]:
                    if g[0] >= f[0] + f[1]:
                        break
                    if g[2] != f[2]:
                        cross += 1
                        hit_sf.add(int((f[0] - span[0]) // P))
                        k = "+".join(sorted([f"{f[2]}:{f[3]}{f[4] or ''}", f"{g[2]}:{g[3]}{g[4] or ''}"]))
                        pairs[k] = pairs.get(k, 0) + 1
            counts = {}
            for f in frames:
                k = f"{f[2]}:{f[3]}{f[4] or ''}"
                counts[k] = counts.get(k, 0) + 1
            n_sf = max(1, int((span[1] - span[0]) // P))
            rep["witness"] = {"frames": len(frames), "cross_cell_overlaps": cross,
                              "superframes": n_sf,
                              "overlapped_sf_fraction": round(len(hit_sf) / n_sf, 4),
                              "overlap_pairs": dict(sorted(pairs.items())),
                              "counts": dict(sorted(counts.items()))}
    deliv = all(v["ratio"] >= min_ratio for c in ok_cells for v in c["dl"].values()) and \
        all(c["dl"] and c["expected_ues"] for c in ok_cells)
    ul = all(v["ratio"] >= min_ratio for c in ok_cells for v in c["ul"].values()) and \
        all(c["ul"] and c["expected_ues"] for c in ok_cells)
    w_ok = rep["witness"] is None or ("error" not in rep["witness"]
                                       and rep["witness"]["cross_cell_overlaps"] == 0)
    rep["verdict"] = {"dl_delivery": deliv, "ul_ratio": ul, "one_instant": rep["one_instant"],
                      "no_cross_overlaps": w_ok,
                      "pass": deliv and ul and rep["one_instant"] and w_ok
                      and len(ok_cells) == len(cells)}
    return rep


def selftest():
    def run(collide, missing_ue=False):
        cells = []
        witness = []
        for cell, phase in ((1, 0), (2, 1500)):
            du = [{"ev": "cell.f1.applied", "version": 1, "effective_net_sf": 5000,
                   "effective_sf": 100},
                  {"ev": "cell.grant", "ok": 1, "effective_sf": 100,
                   "ta": {"1": [0, 24], **({"2": [0, 24]} if missing_ue else {})}}]
            ue = [{"ev": "cell.ue.done", "listen_to_sf": 400}]
            for sf in range(100, 401):
                du.append({"ev": "cell.dl", "ue": 1, "sf": sf, "seq": sf, "attempt": 1, "ok": 1})
                du.append({"ev": "cell.ul", "ue": 1, "sf": sf})
                lost = collide and cell == 1 and sf % 2 == 0
                if not lost:
                    ue.append({"ev": "cell.dl.rx", "ue": 1, "sf": sf, "seq": sf, "attempt": 1})
                t0 = sf * P
                # egress: the cell's own TSF at its TBTT (its local superframe)
                witness.append({"ev": "cell.w", "k": "bcn", "st": 0, "cell": cell,
                                "tsfl": (t0 + phase) & 0xffffffff, "egress": t0, "len": 120})
                # Planned (the rig plan): A's UE in position 2 (DL at 5500),
                # B's in position 3 (DL at 1500 + 9500). Uncoordinated, both
                # sit in position 1: A's DL lands on B's beacon at 1500.
                pos = (2 if cell == 1 else 3) if not collide else 1
                dl_at = t0 + phase + 1500 + 4000 * (pos - 1)
                witness.append({"ev": "cell.w", "k": "dl", "st": 1, "cell": cell,
                                "tsfl": dl_at & 0xffffffff, "len": 100})
            cells.append((cell, du, ue))
        witness.sort(key=lambda e: e["tsfl"])
        return analyze(cells, witness, settle_sf=10)
    clean = run(False)
    bad = run(True)
    silent = run(False, missing_ue=True)  # a granted UE 2 that never reports
    ok = (clean["verdict"]["pass"] and not bad["verdict"]["pass"]
          and bad["witness"]["cross_cell_overlaps"] > 0
          and bad["witness"]["overlapped_sf_fraction"] > 0.9
          and bad["cells"][0]["dl"]["1"]["ratio"] < 0.6
          and not silent["verdict"]["pass"] and "error" in silent["cells"][0]["ul"]["2"])
    print(json.dumps({"ev": "ran.selftest", "pass": ok,
                      "clean": clean["verdict"], "collide": bad["verdict"]}))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cell", action="append", default=[], help="CELL:du.jsonl:ue.jsonl")
    ap.add_argument("--witness")
    ap.add_argument("--cu", help="the CU's JSONL (control-plane latency)")
    ap.add_argument("--settle-sf", type=int, default=20)
    ap.add_argument("--min-ratio", type=float, default=0.98)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if len(a.cell) < 2:
        ap.error("two or more --cell")
    cells = []
    for spec in a.cell:
        c, d, u = spec.split(":", 2)
        cells.append((int(c), load(d), load(u)))
    rep = analyze(cells, load(a.witness) if a.witness else None, a.settle_sf, a.min_ratio,
                  load(a.cu) if a.cu else None)
    for c in rep["cells"]:
        print(json.dumps({"ev": "ran.cell", **c}))
    if rep["witness"] is not None:
        print(json.dumps({"ev": "ran.witness", **rep["witness"]}))
    if "control" in rep:
        print(json.dumps({"ev": "ran.control", **rep["control"]}))
    print(json.dumps({"ev": "ran.verdict", **rep["verdict"]}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
