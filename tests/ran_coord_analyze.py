#!/usr/bin/env python3
"""ran_coord_analyze.py - KPIs for an M5 (#259) coordination run: two (or
more) co-channel examples/cell DUs under the ran-controller CU.

  ran_coord_analyze.py --cell 1:du1.jsonl:ue1.jsonl --cell 2:du2.jsonl:ue2.jsonl \
      [--witness w.jsonl] [--cu cu.jsonl] [--settle-sf 20]

Each cell's KPIs are judged over its COORDINATED window: from the network
instant its CU schedule took effect (the DU's cell.f1.applied effective_sf,
its own superframe count) + --settle-sf, to the last superframe its UE
listened (cell.ue.done listen_to_sf).
  dl: per UE, frames the DU sent first (attempt 1, ok) in the window vs those
      the UE received on that first attempt; retransmissions the DU sent.
  ul: per UE, uplinks the DU heard in the window vs superframes in it.
  instant: the network superframe each DU applied its schedule on (one
      network-wide instant when every DU names the same one).
  witness (one receiver for every cell): cross-cell air overlaps between
      frames it decoded (6 Mb/s airtime, conservative), over the window
      mapped onto the witness clock through cell 1's beacons (their egress
      TSF is cell 1's clock). A collided frame often does not decode at all,
      so this is a lower bound; the delivery KPIs are the main measure.
The verdict (ev ran.verdict): every UE's first-attempt DL delivery and UL
ratio at least --min-ratio, every DU on the same instant, and no witness
cross-cell overlap.

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
    sent = {}
    retx = {}
    for e in du:
        if e.get("ev") == "cell.dl" and lo <= e["sf"] <= hi and e.get("ok"):
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
                             "retx": retx.get(u, 0)}
    ul = {}
    for e in du:
        if e.get("ev") == "cell.ul" and lo <= e["sf"] <= hi:
            ul.setdefault(e["ue"], set()).add(e["sf"])
    for u, s in sorted(ul.items()):
        rep["ul"][str(u)] = {"heard_sf": len(s), "window_sf": hi - lo + 1,
                             "ratio": round(len(s) / (hi - lo + 1), 4)}
    return rep


def analyze(cells, witness=None, settle_sf=20, min_ratio=0.98):
    rep = {"cells": [cell_kpis(c, du, ue, settle_sf) for c, du, ue in cells], "witness": None}
    ok_cells = [c for c in rep["cells"] if "error" not in c]
    instants = {c["instant_net_sf"] for c in ok_cells}
    rep["one_instant"] = len(ok_cells) == len(cells) and len(instants) == 1
    if witness is not None and ok_cells:
        w = [e for e in witness if e.get("ev") == "cell.w"]
        t = unwrap32([e["tsfl"] for e in w])
        for e, v in zip(w, t):
            e["t"] = v
        c1 = cells[0][0]
        b = [e for e in w if e["k"] == "bcn" and e.get("cell") == c1 and e.get("egress")]
        if len(b) < 8:
            rep["witness"] = {"error": f"{len(b)} beacons of cell {c1}; 8 needed to map it"}
        else:
            a, c = robust_line([e["t"] for e in b], [e["egress"] for e in b])
            lo, hi = rep["cells"][0]["window_sf"] if "window_sf" in rep["cells"][0] else (0, -1)
            frames = []
            for e in w:
                du1 = a * e["t"] + c  # cell 1 TSF
                if lo * P <= du1 <= (hi + 1) * P:
                    frames.append((e["t"], airtime_us(e["len"]), e.get("cell"), e["k"], e["st"]))
            frames.sort()
            cross, pairs = 0, {}
            for i, f in enumerate(frames):
                for g in frames[i + 1:]:
                    if g[0] >= f[0] + f[1]:
                        break
                    if g[2] != f[2]:
                        cross += 1
                        k = "+".join(sorted([f"{f[2]}:{f[3]}{f[4] or ''}", f"{g[2]}:{g[3]}{g[4] or ''}"]))
                        pairs[k] = pairs.get(k, 0) + 1
            counts = {}
            for f in frames:
                k = f"{f[2]}:{f[3]}{f[4] or ''}"
                counts[k] = counts.get(k, 0) + 1
            rep["witness"] = {"frames": len(frames), "cross_cell_overlaps": cross,
                              "overlap_pairs": dict(sorted(pairs.items())),
                              "counts": dict(sorted(counts.items()))}
    deliv = all(v["ratio"] >= min_ratio for c in ok_cells for v in c["dl"].values()) and \
        all(c["dl"] for c in ok_cells)
    ul = all(v["ratio"] >= min_ratio for c in ok_cells for v in c["ul"].values()) and \
        all(c["ul"] for c in ok_cells)
    w_ok = rep["witness"] is None or ("error" not in rep["witness"]
                                       and rep["witness"]["cross_cell_overlaps"] == 0)
    rep["verdict"] = {"dl_delivery": deliv, "ul_ratio": ul, "one_instant": rep["one_instant"],
                      "no_cross_overlaps": w_ok,
                      "pass": deliv and ul and rep["one_instant"] and w_ok
                      and len(ok_cells) == len(cells)}
    return rep


def selftest():
    def run(collide):
        cells = []
        witness = []
        for cell, phase in ((1, 0), (2, 1500)):
            du = [{"ev": "cell.f1.applied", "version": 1, "effective_net_sf": 5000,
                   "effective_sf": 100}]
            ue = [{"ev": "cell.ue.done", "listen_to_sf": 400}]
            for sf in range(100, 401):
                du.append({"ev": "cell.dl", "ue": 1, "sf": sf, "seq": sf, "attempt": 1, "ok": 1})
                du.append({"ev": "cell.ul", "ue": 1, "sf": sf})
                lost = collide and cell == 1 and sf % 2 == 0
                if not lost:
                    ue.append({"ev": "cell.dl.rx", "ue": 1, "sf": sf, "seq": sf, "attempt": 1})
                t0 = sf * P
                witness.append({"ev": "cell.w", "k": "bcn", "st": 0, "cell": cell,
                                "tsfl": (t0 + phase) & 0xffffffff, "egress": t0 + phase if cell == 1 else 0,
                                "len": 120})
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
    ok = (clean["verdict"]["pass"] and not bad["verdict"]["pass"]
          and bad["witness"]["cross_cell_overlaps"] > 0
          and bad["cells"][0]["dl"]["1"]["ratio"] < 0.6)
    print(json.dumps({"ev": "ran.selftest", "pass": ok,
                      "clean": clean["verdict"], "collide": bad["verdict"]}))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cell", action="append", default=[], help="CELL:du.jsonl:ue.jsonl")
    ap.add_argument("--witness")
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
    rep = analyze(cells, load(a.witness) if a.witness else None, a.settle_sf, a.min_ratio)
    print(json.dumps(rep, indent=1))
    print(json.dumps({"ev": "ran.verdict", **rep["verdict"]}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
