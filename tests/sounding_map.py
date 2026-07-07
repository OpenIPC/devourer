#!/usr/bin/env python3
"""Coarse H(f) map from a two-ended sounding sweep (issue #149).

Parses the RX side's per-dwell `rx.energy` events (with `ch`, `frames`,
`rssi_mean`, `rssi_max`, `snr_mean`, `evm_mean` fields; rxdemo with
DEVOURER_RX_SWEEP + DEVOURER_RX_AGG_SA=canon, driven by tests/sounding_sweep.sh)
and renders the recovered per-bin link map.

Headline metric: the per-bin **median of rssi_max** over live dwells. During a
dwell on bin k the prober is co-channel only ~1/nbins of the time; decoded
off-channel bleed drags rssi_mean down, but bleed decodes weaker — never
stronger — so the dwell's rssi_max is the bleed-robust probe level. rssi_mean /
snr / evm ride along as columns.

Flags per bin:
  NOTCH  median rssi_max >= --notch-db below the across-bin median (frames>0)
  DEAD   zero probe frames in every dwell while other bins have hits

--sdr-csv: rank-correlate the recovered map against a wideband SDR's per-bin
integrated power (tests/hop_rx_probe.py --bin-power-csv) — Spearman rho over
the common live bins.

--expand "SPEC": print the channel list for a SweepSpec bin spec (the shell
orchestrator's helper; mirrors src/SweepSpec.h).
"""
from __future__ import annotations

import argparse
import statistics
import sys

from devourer_events import iter_events


def expand_spec(spec: str) -> list[int]:
    """Mirror of devourer::parse_sweep_spec (src/SweepSpec.h; guarded there by
    ctest sweep_spec_math): channels, channel ranges A-B/S, MHz ranges."""
    out: list[int] = []

    def freq_to_chan(mhz: int) -> int:
        if mhz == 2484:
            return 14
        if 2412 <= mhz <= 2472:
            return (mhz - 2407) // 5
        if 5000 <= mhz <= 5895:
            return (mhz - 5000) // 5
        return 0

    for tok in (spec or "").split(","):
        tok = tok.strip()
        if not tok:
            continue
        step = 1
        if "/" in tok:
            tok, s = tok.split("/", 1)
            try:
                step = int(s, 0)
            except ValueError:
                step = 1
        step = max(1, step)
        dash = tok.find("-", 1)
        if dash < 0:
            try:
                ch = int(tok, 0)
            except ValueError:
                continue
            if ch > 0:
                out.append(ch)
            continue
        try:
            a, b = int(tok[:dash], 0), int(tok[dash + 1:], 0)
        except ValueError:
            continue
        if a <= 0 or b < a:
            continue
        if a >= 1000 and step < 5:
            step = 5
        for v in range(a, b + 1, step):
            ch = freq_to_chan(v) if a >= 1000 else v
            if ch > 0:
                out.append(ch)
    return out


def chan_to_mhz(ch: int) -> int:
    if ch == 14:
        return 2484
    return 2407 + 5 * ch if ch <= 14 else 5000 + 5 * ch


def spearman(x: list[float], y: list[float]) -> float:
    """Spearman rank correlation (no scipy dependency; average ranks on ties)."""
    def ranks(v: list[float]) -> list[float]:
        order = sorted(range(len(v)), key=lambda i: v[i])
        r = [0.0] * len(v)
        i = 0
        while i < len(order):
            j = i
            while j + 1 < len(order) and v[order[j + 1]] == v[order[i]]:
                j += 1
            avg = (i + j) / 2.0 + 1.0
            for k in range(i, j + 1):
                r[order[k]] = avg
            i = j + 1
        return r

    rx, ry = ranks(x), ranks(y)
    mx, my = statistics.mean(rx), statistics.mean(ry)
    num = sum((a - mx) * (b - my) for a, b in zip(rx, ry))
    dx = sum((a - mx) ** 2 for a in rx) ** 0.5
    dy = sum((b - my) ** 2 for b in ry) ** 0.5
    return num / (dx * dy) if dx > 0 and dy > 0 else 0.0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", help="RX sweep log (rx.energy events with ch=)")
    ap.add_argument("--expand", metavar="SPEC",
                    help="print the channel list for a SweepSpec and exit")
    ap.add_argument("--notch-db", type=float, default=6.0,
                    help="dB below the across-bin median to flag NOTCH")
    ap.add_argument("--min-dwells", type=int, default=2,
                    help="min live dwells for a bin to count as sounded")
    ap.add_argument("--sdr-csv",
                    help="per-bin SDR power (hop_rx_probe.py --bin-power-csv) "
                         "to rank-correlate against")
    args = ap.parse_args()

    if args.expand is not None:
        print(" ".join(str(c) for c in expand_spec(args.expand)))
        return 0
    if not args.log:
        ap.error("--log (or --expand) required")

    # Per-bin dwell records; the first dwell per bin is dropped (partial dwell
    # while the loops phase-align + AGC settle — same convention as
    # rx_spectrum_sweep.py).
    dwells: dict[int, list[dict[str, float]]] = {}
    for ev in iter_events(open(args.log, errors="replace"), ev="rx.energy"):
        # keep the numeric fields only (drop the ev name and any null sensors)
        kv = {k: v for k, v in ev.items() if isinstance(v, (int, float))}
        if "ch" not in kv or "frames" not in kv:
            continue  # no sweep bin / per-dwell frame stats (steady-state emitter)
        dwells.setdefault(int(kv["ch"]), []).append(kv)
    dwells = {ch: d[1:] if len(d) > 1 else d for ch, d in dwells.items()}
    if not dwells:
        print("no sounding samples in log (need rx.energy events with ch and "
              "frames)", file=sys.stderr)
        return 1

    rows = []
    for ch in sorted(dwells):
        d = dwells[ch]
        live = [x for x in d if x.get("frames", 0) > 0]
        rec = {
            "ch": ch,
            "dwells": len(d),
            "live": len(live),
            "hits": statistics.median([x["frames"] for x in live]) if live else 0,
            "rssi_max": statistics.median([x["rssi_max"] for x in live]) if live else None,
            "rssi_mean": statistics.median([x["rssi_mean"] for x in live]) if live else None,
            "snr_mean": statistics.median([x["snr_mean"] for x in live]) if live else None,
            "evm_mean": statistics.median([x["evm_mean"] for x in live
                                           if x.get("evm_mean", 0) != 0]) if any(
                x.get("evm_mean", 0) != 0 for x in live) else None,
        }
        rows.append(rec)

    sounded = [r for r in rows if r["live"] >= args.min_dwells]
    levels = [r["rssi_max"] for r in sounded if r["rssi_max"] is not None]
    med = statistics.median(levels) if levels else 0
    any_live = any(r["live"] for r in rows)

    print("=== sounding map (per-bin median over dwells; headline = rssi_max,"
          " bleed-robust) ===")
    lo = min(levels) - 3 if levels else 0
    hi = max(levels) if levels else 1
    span = max(1, hi - lo)
    width = 40
    for r in rows:
        ch, mhz = r["ch"], chan_to_mhz(r["ch"])
        flag = ""
        if r["live"] == 0:
            flag = " DEAD" if any_live else ""
        elif r["rssi_max"] is not None and levels and \
                r["rssi_max"] <= med - args.notch_db:
            flag = " NOTCH"
        def fmt(v):  # medians come back int or float; render uniformly
            return "   -" if v is None else f"{v:>4g}"
        if r["rssi_max"] is None:
            bar = ""
        else:
            bar = "#" * max(1, round(width * (r["rssi_max"] - lo) / span))
        print(f"  ch {ch:>3} ({mhz:>4} MHz) | {bar:<{width}} "
              f"rssi_max={fmt(r['rssi_max'])} rssi_mean={fmt(r['rssi_mean'])} "
              f"snr={fmt(r['snr_mean'])} evm={fmt(r['evm_mean'])} "
              f"hits={fmt(r['hits'])} live={r['live']}/{r['dwells']}{flag}")

    n_dead = sum(1 for r in rows if r["live"] == 0 and any_live)
    n_notch = sum(1 for r in rows if r["live"] > 0 and r["rssi_max"] is not None
                  and levels and r["rssi_max"] <= med - args.notch_db)
    print(f"\nbins={len(rows)} sounded={len(sounded)} dead={n_dead} "
          f"notched={n_notch} median_rssi_max={med}")

    if args.sdr_csv:
        sdr = {}
        with open(args.sdr_csv) as f:
            next(f)  # header
            for ln in f:
                parts = ln.strip().split(",")
                if len(parts) >= 2:
                    sdr[int(parts[0])] = float(parts[1])
        common = [r for r in sounded if r["ch"] in sdr and r["rssi_max"] is not None]
        vals = [float(r["rssi_max"]) for r in common]
        spread = (max(vals) - min(vals)) if vals else 0.0
        if len(common) < 3:
            print(f"sdr-crosscheck: only {len(common)} common live bins — "
                  "need >=3 for a rank correlation")
        elif spread <= 2.0:
            # A flat map (bench near-field, no frequency selectivity) has no
            # rank structure to correlate — rho on ties+noise is meaningless.
            print(f"sdr-crosscheck: map is flat (rssi_max spread {spread:g}) "
                  "— nothing to rank-correlate; both ends see a flat channel")
        else:
            rho = spearman(vals, [sdr[r["ch"]] for r in common])
            verdict = "MATCH" if rho >= 0.5 else "WEAK" if rho > 0 else "MISMATCH"
            print(f"sdr-crosscheck bins={len(common)} spearman_rho={rho:.2f} "
                  f"-> {verdict}")

    if not any_live:
        print("\nSOUNDING: FAIL — no probe frames on any bin (prober down, "
              "wrong SA filter, or ends on different bands?)")
        return 1
    print("\nSOUNDING: map recovered.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
