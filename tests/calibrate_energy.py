#!/usr/bin/env python3
"""Fit the adaptive-link energy model from a thermal-gain sweep -> energy_calib.json.

The model (energy_model.py) needs `P_circuit` (W, paid over the whole frame slot)
and `P_pa[idx]` (W, paid on-air, per TXAGC index 0..63). This consumes the CSV
that tests/thermal_gain_sweep.py already produces:
    index, raw, baseline, delta, status, dbfs
across a TXAGC sweep (--start/--stop/--step), and fits:
    P_pa[idx] ∝ alpha*10^(dbfs(idx)/10)  +  beta*delta(idx)
                \___radiated proxy___/      \__PA-dissipation proxy__/
interpolated to all 64 indices (monotone), then scaled. "Model now, meter later":
without --meter the result is RELATIVE (anchored to the nominal P_pa range, so the
controller picks the same operating points); with --meter CSV (columns:
index,watts of measured DC bus power) it is ABSOLUTE (sets metered_watts=true and
P_circuit from the zero-/low-idx intercept).

  ./calibrate_energy.py --sweep-csv sweep-8812-ch6-*.csv [--meter meter.csv] \
      --out energy_calib.json
Run the sweep first:  sudo python3 thermal_gain_sweep.py --ht --start 0 --stop 63 --step 3
"""
import argparse
import csv
import json
import os
import sys

sys.path.insert(0, os.path.expanduser("~/git/devourer/tools/precoder"))
import energy_model  # noqa: E402


def read_sweep(path):
    """CSV -> {idx: (mean_dbfs_or_None, mean_delta_or_None)} averaged per index."""
    rows = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                idx = int(r["index"])
            except (KeyError, ValueError):
                continue
            dbfs = r.get("dbfs");  dbfs = float(dbfs) if dbfs not in (None, "", "none") else None
            dlt = r.get("delta");  dlt = float(dlt) if dlt not in (None, "", "none") else None
            a, b, n = rows.get(idx, (0.0, 0.0, 0))
            rows[idx] = (a + (dbfs or 0.0), b + (dlt or 0.0), n + 1)
    return {k: (a / n if n else None, b / n if n else None) for k, (a, b, n) in rows.items()}


def fit_pa_curve(sweep, alpha=1.0, beta=0.02):
    """Per-index PA proxy from radiated (10^dbfs/10) + dissipation (delta);
    interpolate to all 64 indices, force monotone non-decreasing."""
    swept = sorted(sweep)
    raw = {}
    for idx in swept:
        dbfs, dlt = sweep[idx]
        rad = 10 ** (dbfs / 10.0) if dbfs is not None else 0.0
        raw[idx] = alpha * rad + beta * (dlt or 0.0)
    # linear interpolation across the full 0..63 grid
    pa = []
    for idx in range(64):
        if idx in raw:
            pa.append(raw[idx]); continue
        lo = max((i for i in swept if i <= idx), default=swept[0])
        hi = min((i for i in swept if i >= idx), default=swept[-1])
        if lo == hi:
            pa.append(raw[lo])
        else:
            t = (idx - lo) / (hi - lo)
            pa.append(raw[lo] + t * (raw[hi] - raw[lo]))
    for i in range(1, 64):              # monotone non-decreasing
        pa[i] = max(pa[i], pa[i - 1])
    return pa


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--sweep-csv", required=True)
    p.add_argument("--meter", help="optional DC-meter CSV: index,watts (absolute anchor)")
    p.add_argument("--out", default="energy_calib.json")
    a = p.parse_args()

    sweep = read_sweep(a.sweep_csv)
    if not sweep:
        sys.exit("no usable rows in sweep CSV")
    pa = fit_pa_curve(sweep)

    nominal = energy_model.DEFAULT_CALIB
    metered = False
    p_circuit = nominal["p_circuit_w"]
    if a.meter:
        metered = True
        with open(a.meter) as f:
            m = {int(r["index"]): float(r["watts"]) for r in csv.DictReader(f)}
        lo_idx = min(m)
        p_circuit = m[lo_idx]                      # low-idx DC ~= circuit floor
        # scale the PA proxy to absolute Watts: anchor span to (max-min) meter delta
        span = (m[max(m)] - m[lo_idx])
        pa_span = pa[-1] - pa[0] or 1.0
        pa = [p_circuit and (x - pa[0]) / pa_span * span for x in pa]
    else:
        # relative: rescale the proxy curve into the nominal P_pa range so the
        # controller's operating-point choices are unchanged in absolute terms.
        nom = nominal["p_pa_w"]
        lo, hi = pa[0], pa[-1] or 1.0
        pa = [nom[0] + (x - lo) / (hi - lo or 1.0) * (nom[-1] - nom[0]) for x in pa]

    out = dict(nominal)
    out.update({"source": "metered" if metered else "calibrated",
                "metered_watts": metered, "p_circuit_w": round(p_circuit, 4),
                "p_pa_w": [round(x, 5) for x in pa]})
    with open(a.out, "w") as f:
        json.dump(out, f, indent=2)
    print(f"[calibrate-energy] wrote {a.out} source={out['source']} "
          f"P_circuit={out['p_circuit_w']}W P_pa[0/63]={out['p_pa_w'][0]}/{out['p_pa_w'][63]}W")


if __name__ == "__main__":
    main()
