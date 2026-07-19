#!/usr/bin/env python3
"""Offline analyzer for kernel channel-switch bench runs (issue: standard-
driver FHSS baseline). No root, no hardware — pure parsing + numpy.

Input: a run directory produced by tests/kchansw_bench.py, one subdirectory
per config, each holding:

    meta.json        config descriptor (prim, channels, oracle roles,
                     probe_name→role map, fingerprint mode)
    control.jsonl    kchansw.req / kchansw.done records (host mono_ns)
    trace.txt        ftrace text (trace_clock=mono) incl. KCHANSW markers
    oracle_a.jsonl   {"host_mono_ns":..,"raw":"<rxdemo JSONL>"} per line
    oracle_b.jsonl   same for the second oracle
    inject.jsonl     inj.tx records (P1-style configs)

Timeline: one clock everywhere (CLOCK_MONOTONIC). ftrace timestamps are
directly comparable to mono_ns; oracle frames get sub-ms placement via a
robust per-oracle linear fit host_mono_ns ≈ a + b·tsfl (tsfl = the chip's
MAC-latched µs RX timestamp, unwrapped mod 2^32), so stdout-pipe jitter on
the reader stamps averages out instead of polluting per-frame times.

Outputs (into the run dir): switches.csv.gz (per-switch records across all
configs), summary.json (distributions + fit diagnostics + gate verdicts),
summary.md (tables in the docs/performance-tuning.md style).

Gate rule (headline = on-air dead time p99 per config):
    < 5 ms  → advance ; 5–20 ms → slow-slot-only ; > 20 ms → control

`--self-test` fabricates a synthetic run with known truth (dead time, fit
slope, losses) and asserts the pipeline recovers it — run it before spending
any hardware time, and in CI-less environments after edits.
"""

import argparse
import csv
import gzip
import io
import json
import math
import os
import re
import struct
import sys

import numpy as np

MAGIC_HEX = "4b435357"  # 'KCSW'

GATE_ADVANCE_NS = 5e6
GATE_SLOW_NS = 20e6

# ---------------------------------------------------------------------------
# Parsing.
# ---------------------------------------------------------------------------

# `  comm-pid [cpu] flags 12345.678901: event_name: details`
_TRACE_RE = re.compile(
    r"^\s*(?P<comm>.+?)-(?P<pid>\d+)\s+\[(?P<cpu>\d+)\]\s+\S+\s+"
    r"(?P<ts>\d+\.\d+):\s+(?P<name>[\w.]+):\s?(?P<detail>.*)$")

_MARKER_RE = re.compile(
    r"KCHANSW i=(?P<i>\d+) prim=(?P<prim>\S+) cfg=(?P<cfg>\S+) "
    r"from=(?P<from>\S+) to=(?P<to>\S+) mono=(?P<mono>\d+)")


def parse_trace(path):
    """Yield dicts {ts_ns, pid, name, detail} plus markers {..., marker:{}}."""
    events = []
    with open(path, "r", errors="replace") as f:
        for line in f:
            m = _TRACE_RE.match(line)
            if not m:
                continue
            ev = {
                "ts_ns": int(round(float(m.group("ts")) * 1e9)),
                "pid": int(m.group("pid")),
                "name": m.group("name"),
                "detail": m.group("detail"),
            }
            if ev["name"] == "tracing_mark_write":
                mm = _MARKER_RE.search(ev["detail"])
                if mm:
                    ev["marker"] = {
                        "i": int(mm.group("i")),
                        "prim": mm.group("prim"),
                        "cfg": mm.group("cfg"),
                        "from": mm.group("from"),
                        "to": mm.group("to"),
                        "mono_ns": int(mm.group("mono")),
                    }
            events.append(ev)
    return events


def load_jsonl(path):
    out = []
    if not os.path.exists(path):
        return out
    with open(path, "r", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or not line.startswith("{"):
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return out


def oracle_frames(path):
    """Parse a stamped oracle log into frame records:
    {host_mono_ns, tsfl_us, len, crc, seq, ctr, send_ns, nonce} — ctr/send_ns
    only when the KCSW fingerprint is present in the body."""
    frames = []
    for rec in load_jsonl(path):
        raw = rec.get("raw")
        host = rec.get("host_mono_ns")
        if raw is None or host is None:
            continue
        try:
            ev = json.loads(raw)
        except json.JSONDecodeError:
            continue
        if ev.get("ev") != "rx.frame":
            continue
        fr = {
            "host_mono_ns": int(host),
            "tsfl_us": int(ev.get("tsfl", 0)),
            "len": int(ev.get("len", 0)),
            "crc": int(ev.get("crc", 0)),
            "seq": int(ev.get("seq", 0)),
            "ctr": None,
            "send_ns": None,
            "nonce": None,
            "body": ev.get("body", ""),
        }
        body = fr["body"]
        if isinstance(body, str) and body.lower().startswith(MAGIC_HEX):
            try:
                blob = bytes.fromhex(body[:40])
                _, nonce, ctr, send_ns = struct.unpack("<4sIIQ", blob)
                fr["nonce"], fr["ctr"], fr["send_ns"] = nonce, ctr, send_ns
            except (ValueError, struct.error):
                pass
        frames.append(fr)
    return frames


# ---------------------------------------------------------------------------
# tsfl → host-mono fit.
# ---------------------------------------------------------------------------

def unwrap_tsfl(frames):
    """Unwrap the 32-bit µs tsfl in host-stamp order (adds 2^32 per wrap)."""
    off = 0
    prev = None
    for fr in sorted(frames, key=lambda f: f["host_mono_ns"]):
        t = fr["tsfl_us"]
        if prev is not None and t < prev - (1 << 31):
            off += 1 << 32
        prev = t
        fr["tsfl_unwrapped_us"] = t + off
    return frames


def fit_tsfl(frames, max_pairs=800):
    """Robust linear fit host_mono_ns ≈ a + b*tsfl_unwrapped_us.

    Theil-Sen slope on a decimated subset, then least-squares on inliers
    (|residual| < 3*1.4826*MAD, floored at 50 µs). Returns dict with a, b,
    residual std (ns), inlier fraction — or ok=False when too thin."""
    pts = [(f["tsfl_unwrapped_us"], f["host_mono_ns"]) for f in frames]
    if len(pts) < 20:
        return {"ok": False, "n": len(pts), "reason": "too-few-frames"}
    x = np.array([p[0] for p in pts], dtype=np.float64)
    y = np.array([p[1] for p in pts], dtype=np.float64)
    if len(x) > max_pairs:
        idx = np.linspace(0, len(x) - 1, max_pairs).astype(int)
        xs, ys = x[idx], y[idx]
    else:
        xs, ys = x, y
    dx = xs[None, :] - xs[:, None]
    dy = ys[None, :] - ys[:, None]
    mask = np.triu(np.abs(dx) > 1e3, k=1)  # pairs ≥1 ms apart in tsfl
    if not mask.any():
        return {"ok": False, "n": len(pts), "reason": "no-spread"}
    b = float(np.median(dy[mask] / dx[mask]))
    a = float(np.median(y - b * x))
    resid = y - (a + b * x)
    mad = float(np.median(np.abs(resid - np.median(resid))))
    thr = max(3 * 1.4826 * mad, 50e3)  # ≥50 µs floor
    inl = np.abs(resid - np.median(resid)) < thr
    if inl.sum() >= 20:
        A = np.vstack([x[inl], np.ones(inl.sum())]).T
        sol, *_ = np.linalg.lstsq(A, y[inl], rcond=None)
        b, a = float(sol[0]), float(sol[1])
        resid = y - (a + b * x)
    r_std = float(np.std(resid[inl])) if inl.sum() else float("inf")
    slope_err_ppm = (b / 1e3 - 1.0) * 1e6  # vs nominal 1000 ns/µs
    # Gate on MAPPING stability, not stamp noise: per-frame fitted times come
    # from tsfl (exact µs), so reader-stamp jitter only widens the residual —
    # it does not propagate into fitted placements. A broken tsfl stream
    # (wraps, stalls) shows up as huge residuals or an insane slope.
    return {
        "ok": r_std <= 2e6 and abs(slope_err_ppm) < 1000,
        "n": len(pts), "a": a, "b": b,
        "residual_std_ns": r_std,
        "slope_err_ppm": slope_err_ppm,
        "inlier_frac": float(inl.mean()),
    }


def fitted_ns(fit, fr):
    return fit["a"] + fit["b"] * fr["tsfl_unwrapped_us"]


# ---------------------------------------------------------------------------
# Per-switch extraction.
# ---------------------------------------------------------------------------

CSV_COLS = [
    "cfg", "prim", "i", "warmup", "from_ch", "to_ch", "direction",
    "t_req_ns", "rdev_entry_ns", "rdev_ret_ns", "drv_entry_ns", "drv_ret_ns",
    "chip_entry_ns", "chip_ret_ns", "h2c_n", "h2c_first_ns", "h2c_last_ns",
    "qstop_ns", "qwake_ns", "ready_ns", "expired_ns", "notify_ns",
    "last_src_air_ns", "first_dst_air_ns", "first_back_air_ns", "dead_ns",
    "src_last_ctr", "dst_first_ctr", "gap_frames", "dup", "inj_errs",
    "ok", "fail_reason",
]


def _first(events, name, after=None, pid=None):
    for ev in events:
        if ev["name"] != name:
            continue
        if after is not None and ev["ts_ns"] < after:
            continue
        if pid is not None and ev["pid"] != pid:
            continue
        return ev
    return None


def _last(events, name, before=None):
    out = None
    for ev in events:
        if ev["name"] != name:
            continue
        if before is not None and ev["ts_ns"] >= before:
            continue
        out = ev
    return out


def extract_switches(meta, trace, frames_by_ch, fits_by_ch, inject):
    """Per-switch records for one config. frames_by_ch maps channel →
    (frames sorted by fitted host time, fit)."""
    rdev_entry_name = meta.get("rdev_entry", "rdev_set_monitor_channel")
    probe_map = meta.get("probe_map", {})
    drv_p = probe_map.get("drv")
    chip_p = probe_map.get("chip")
    h2c_p = probe_map.get("h2c")

    markers = [ev for ev in trace if ev.get("marker")
               and ev["marker"]["cfg"] == meta["cfg"]]
    markers.sort(key=lambda ev: ev["ts_ns"])
    inj_times = [(r["mono_ns"], r.get("errno", 0)) for r in inject
                 if r.get("ev") == "inj.tx"]
    inj_times.sort()

    rows = []
    for k, mk in enumerate(markers):
        w0 = mk["ts_ns"]
        w1 = markers[k + 1]["ts_ns"] if k + 1 < len(markers) else w0 + int(2e9)
        win = [ev for ev in trace if w0 <= ev["ts_ns"] < w1]
        mkm = mk["marker"]
        row = {c: "" for c in CSV_COLS}
        row.update({
            "cfg": meta["cfg"], "prim": mkm["prim"], "i": mkm["i"],
            "warmup": 1 if mkm["i"] < meta.get("warmup", 0) else 0,
            "from_ch": mkm["from"], "to_ch": mkm["to"],
            "direction": f"{mkm['from']}>{mkm['to']}",
            "t_req_ns": mkm["mono_ns"],
            "ok": 1, "fail_reason": "",
        })

        entry = _first(win, rdev_entry_name)
        if entry:
            row["rdev_entry_ns"] = entry["ts_ns"]
            for ret_name in ("rdev_return_int", "rdev_return_void"):
                ret = _first(win, ret_name, after=entry["ts_ns"],
                             pid=entry["pid"])
                if ret:
                    row["rdev_ret_ns"] = ret["ts_ns"]
                    break
        for role, col_e, col_r in ((drv_p, "drv_entry_ns", "drv_ret_ns"),
                                   (chip_p, "chip_entry_ns", "chip_ret_ns")):
            if not role:
                continue
            e = _first(win, role)
            if e:
                row[col_e] = e["ts_ns"]
                r = _first(win, role + "_ret", after=e["ts_ns"])
                if r:
                    row[col_r] = r["ts_ns"]
        if h2c_p:
            h2cs = [ev for ev in win if ev["name"] == h2c_p]
            h2c_rets = [ev for ev in win if ev["name"] == h2c_p + "_ret"]
            row["h2c_n"] = len(h2cs)
            if h2cs:
                row["h2c_first_ns"] = h2cs[0]["ts_ns"]
            if h2c_rets:
                row["h2c_last_ns"] = h2c_rets[-1]["ts_ns"]
        qs = _first(win, "stop_queue")
        qw = _last(win, "wake_queue")
        if qs:
            row["qstop_ns"] = qs["ts_ns"]
        if qw:
            row["qwake_ns"] = qw["ts_ns"]
        for name, col in (("cfg80211_ready_on_channel", "ready_ns"),
                          ("cfg80211_ready_on_channel_expired", "expired_ns"),
                          ("cfg80211_ch_switch_notify", "notify_ns")):
            ev = _first(win, name)
            if ev:
                row[col] = ev["ts_ns"]

        # --- on-air evidence -------------------------------------------------
        src = frames_by_ch.get(str(mkm["from"]))
        dst = frames_by_ch.get(str(mkm["to"]))
        first_dst = None
        if dst is not None:
            dfr, dfit = dst
            for fr in dfr:
                t = fitted_ns(dfit, fr)
                if w0 <= t < w1:
                    first_dst = (t, fr)
                    break
        last_src = None
        if src is not None and first_dst is not None:
            sfr, sfit = src
            for fr in sfr:
                t = fitted_ns(sfit, fr)
                if t >= first_dst[0]:
                    break
                if w0 - int(50e6) <= t:  # allow frames just before the marker
                    last_src = (t, fr)
        if first_dst:
            row["first_dst_air_ns"] = int(first_dst[0])
            if first_dst[1]["ctr"] is not None:
                row["dst_first_ctr"] = first_dst[1]["ctr"]
        if last_src:
            row["last_src_air_ns"] = int(last_src[0])
            if last_src[1]["ctr"] is not None:
                row["src_last_ctr"] = last_src[1]["ctr"]
        if first_dst and last_src:
            row["dead_ns"] = int(first_dst[0] - last_src[0])
            if (row["src_last_ctr"] != "" and row["dst_first_ctr"] != ""
                    and row["dst_first_ctr"] > row["src_last_ctr"]):
                row["gap_frames"] = row["dst_first_ctr"] - row["src_last_ctr"] - 1
        elif meta.get("need_rf", True):
            row["ok"] = 0
            row["fail_reason"] = "no_rf_evidence"

        # ROC return-to-base: first source-channel frame after expiry.
        if row["expired_ns"] != "" and src is not None:
            sfr, sfit = src
            for fr in sfr:
                t = fitted_ns(sfit, fr)
                if t > row["expired_ns"]:
                    row["first_back_air_ns"] = int(t)
                    break

        # Duplicates: same counter twice on the dst oracle inside the window.
        if dst is not None:
            seen, dups = set(), 0
            dfr, dfit = dst
            for fr in dfr:
                t = fitted_ns(dfit, fr)
                if not (w0 <= t < w1) or fr["ctr"] is None:
                    continue
                if fr["ctr"] in seen:
                    dups += 1
                seen.add(fr["ctr"])
            row["dup"] = dups
        if inj_times:
            row["inj_errs"] = sum(1 for t, e in inj_times
                                  if w0 <= t < w1 and e != 0)
        rows.append(row)
    return rows


# ---------------------------------------------------------------------------
# Stats + gates + reports.
# ---------------------------------------------------------------------------

def _dist(vals):
    if not vals:
        return None
    a = np.array(vals, dtype=np.float64)
    return {
        "n": len(vals),
        "median_ms": float(np.median(a)) / 1e6,
        "p90_ms": float(np.percentile(a, 90)) / 1e6,
        "p99_ms": float(np.percentile(a, 99)) / 1e6,
        "max_ms": float(np.max(a)) / 1e6,
    }


def gate(p99_ns):
    if p99_ns is None:
        return "no-data"
    if p99_ns < GATE_ADVANCE_NS:
        return "advance"
    if p99_ns <= GATE_SLOW_NS:
        return "slow-slot-only"
    return "control"


def summarize(rows):
    """Distributions per (cfg): host-cmd, driver, chip, h2c span, queue span,
    dead time, total; plus failure/loss accounting and the gate verdict."""
    cfgs = {}
    for row in rows:
        if row["warmup"]:
            continue
        cfgs.setdefault(row["cfg"], []).append(row)
    out = {}
    for cfg, rws in cfgs.items():
        ok = [r for r in rws if r["ok"] == 1]

        def span(c1, c2):
            return [r[c2] - r[c1] for r in ok
                    if r[c1] != "" and r[c2] != "" and r[c2] >= r[c1]]

        dead = [r["dead_ns"] for r in ok if r["dead_ns"] != ""]
        total = [r["first_dst_air_ns"] - r["t_req_ns"] for r in ok
                 if r["first_dst_air_ns"] != ""]
        d = _dist(dead)
        out[cfg] = {
            "switches": len(rws),
            "ok": len(ok),
            "failures": {
                r["fail_reason"]: sum(1 for x in rws
                                      if x["fail_reason"] == r["fail_reason"])
                for r in rws if r["fail_reason"]
            },
            "host_cmd": _dist(span("rdev_entry_ns", "rdev_ret_ns")),
            "driver": _dist(span("drv_entry_ns", "drv_ret_ns")),
            "chip": _dist(span("chip_entry_ns", "chip_ret_ns")),
            "h2c_span": _dist(span("h2c_first_ns", "h2c_last_ns")),
            "queue_stopped": _dist(span("qstop_ns", "qwake_ns")),
            "dead_air": d,
            "total_req_to_air": _dist(total),
            "gap_frames_total": sum(r["gap_frames"] for r in ok
                                    if r["gap_frames"] != ""),
            "dup_total": sum(r["dup"] for r in ok if r["dup"] != ""),
            "inj_errs_total": sum(r["inj_errs"] for r in ok
                                  if r["inj_errs"] != ""),
            "gate": gate(d["p99_ms"] * 1e6 if d else None),
        }
    return out


def render_md(summary, fits):
    buf = io.StringIO()
    buf.write("| config | n | dead-air med | p90 | p99 | max | host-cmd med "
              "| driver med | queue med | gap | fail | gate |\n")
    buf.write("|---|---|---|---|---|---|---|---|---|---|---|---|\n")

    def ms(d, k="median_ms"):
        return f"{d[k]:.2f} ms" if d else "—"

    for cfg in sorted(summary):
        s = summary[cfg]
        fails = sum(s["failures"].values()) if s["failures"] else 0
        buf.write(
            f"| {cfg} | {s['ok']}/{s['switches']} | {ms(s['dead_air'])} | "
            f"{ms(s['dead_air'], 'p90_ms')} | {ms(s['dead_air'], 'p99_ms')} | "
            f"{ms(s['dead_air'], 'max_ms')} | {ms(s['host_cmd'])} | "
            f"{ms(s['driver'])} | {ms(s['queue_stopped'])} | "
            f"{s['gap_frames_total']} | {fails} | **{s['gate']}** |\n")
    buf.write("\nOracle clock fits (host_mono ≈ a + b·tsfl):\n\n")
    for name, fit in fits.items():
        if not fit:
            continue
        if fit.get("ok"):
            buf.write(f"- {name}: residual σ {fit['residual_std_ns']/1e3:.0f} µs, "
                      f"slope err {fit['slope_err_ppm']:.0f} ppm, "
                      f"n={fit['n']}, inliers {fit['inlier_frac']*100:.0f}%\n")
        else:
            buf.write(f"- {name}: FIT BAD ({fit.get('reason', 'residual gate')}, "
                      f"n={fit.get('n')})\n")
    return buf.getvalue()


# ---------------------------------------------------------------------------
# Run-dir walker.
# ---------------------------------------------------------------------------

def analyze_run(run_dir):
    all_rows = []
    fits = {}
    for cfg_dir in sorted(os.listdir(run_dir)):
        d = os.path.join(run_dir, cfg_dir)
        meta_path = os.path.join(d, "meta.json")
        if not os.path.isdir(d) or not os.path.exists(meta_path):
            continue
        with open(meta_path) as f:
            meta = json.load(f)
        trace = parse_trace(os.path.join(d, "trace.txt"))
        inject = load_jsonl(os.path.join(d, "inject.jsonl"))
        frames_by_ch = {}
        for role in ("a", "b"):
            ch = str(meta.get(f"oracle_{role}_ch", ""))
            path = os.path.join(d, f"oracle_{role}.jsonl")
            if not ch or not os.path.exists(path):
                continue
            frames = unwrap_tsfl(oracle_frames(path))
            # Fingerprint filter: KCSW counter or (CSA/scan) plain SA stream.
            if meta.get("fingerprint", "kcsw") == "kcsw":
                frames = [fr for fr in frames if fr["ctr"] is not None]
            fit = fit_tsfl(frames)
            fits[f"{meta['cfg']}/oracle_{role}(ch{ch})"] = fit
            if fit.get("ok"):
                frames.sort(key=lambda fr: fitted_ns(fit, fr))
                frames_by_ch[ch] = (frames, fit)
        rows = extract_switches(meta, trace, frames_by_ch, fits, inject)
        # fit_bad configs keep trace columns but lose oracle-derived ones.
        all_rows.extend(rows)
    return all_rows, fits


def write_outputs(run_dir, rows, fits):
    csv_path = os.path.join(run_dir, "switches.csv.gz")
    with gzip.open(csv_path, "wt", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_COLS)
        w.writeheader()
        for row in rows:
            w.writerow(row)
    summary = summarize(rows)
    with open(os.path.join(run_dir, "summary.json"), "w") as f:
        json.dump({"configs": summary, "fits": fits}, f, indent=2)
    md = render_md(summary, fits)
    with open(os.path.join(run_dir, "summary.md"), "w") as f:
        f.write(md)
    return summary, md


# ---------------------------------------------------------------------------
# Self-test: synthetic run with known truth.
# ---------------------------------------------------------------------------

def _selftest(tmp):
    """Fabricate a 30-switch P1-like config: dead time 25 ms (→ gate
    `control`), 2 kHz injector, tsfl slope exactly 1000 ns/µs + offset,
    ±120 µs stamp jitter, 3 warmups, one no-RF-evidence failure."""
    rng = np.random.default_rng(7)
    cfg = os.path.join(tmp, "a36-40")
    os.makedirs(cfg, exist_ok=True)
    n_sw, warmup, dead_ns = 30, 3, int(25e6)
    period = int(5e5)  # 2 kHz
    t0 = int(100e9)
    dwell = int(150e6)

    meta = {"cfg": "a36-40", "prim": "set_channel", "from": 36, "to": 40,
            "warmup": warmup, "oracle_a_ch": 36, "oracle_b_ch": 40,
            "rdev_entry": "rdev_set_monitor_channel",
            "probe_map": {"drv": "kc_drv_set_channel",
                          "chip": "kc_chip_set_channel", "h2c": "kc_h2c"},
            "fingerprint": "kcsw", "need_rf": True}
    with open(os.path.join(cfg, "meta.json"), "w") as f:
        json.dump(meta, f)

    # Oracle clocks: tsfl_us = (host_ns - off)/1000 with distinct offsets.
    offs = {36: int(11e9), 40: int(23e9)}

    def mk_frame(ch, host_ns, ctr):
        tsfl = ((host_ns - offs[ch]) // 1000) % (1 << 32)
        body = struct.pack("<4sIIQ", b"KCSW", 42, ctr, host_ns).hex()
        raw = json.dumps({"ev": "rx.frame", "len": 96, "crc": 0, "seq": ctr & 0xFFF,
                          "tsfl": int(tsfl), "body": body})
        stamp = int(host_ns + rng.integers(3e5, 12e5))  # pipe latency+jitter
        return {"host_mono_ns": stamp, "raw": raw}

    oa, ob, trace_lines, control, inject = [], [], [], [], []
    ctr = 0
    # The no-RF failure must be the LAST switch: any earlier one gets
    # "rescued" by the next switch's pre-dwell stream on the same channel
    # (showing up as a huge dead time, not missing evidence — which is
    # itself the realistic signature of a wedged-then-recovered switch).
    fail_i = n_sw - 1

    def tl(ts_ns, name, detail=""):
        trace_lines.append(f" bench-1000  [001] ..... "
                           f"{ts_ns/1e9:.6f}: {name}: {detail}")

    t = t0
    for i in range(n_sw):
        frm, to = (36, 40) if i % 2 == 0 else (40, 36)
        # pre-switch stream on `frm` for one dwell.
        for _ in range(dwell // period):
            ctr += 1
            inject.append({"ev": "inj.tx", "n": ctr, "mono_ns": t, "errno": 0})
            (oa if frm == 36 else ob).append(mk_frame(frm, t, ctr))
            t += period
        req = t
        tl(req + 40_000, "tracing_mark_write",
           f"KCHANSW i={i} prim=set_channel cfg=a36-40 from={frm} to={to} "
           f"mono={req}")
        control.append({"ev": "kchansw.req", "i": i, "mono_ns": req})
        tl(req + int(2.2e6), "rdev_set_monitor_channel", "phy0")
        tl(req + int(2.5e6), "kc_drv_set_channel", "(rtw_set_channel)")
        tl(req + int(3.0e6), "kc_h2c", "(h2c)")
        tl(req + int(3.2e6), "kc_h2c_ret", "(h2c)")
        tl(req + int(9.0e6), "kc_chip_set_channel", "(chip)")
        tl(req + int(19.0e6), "kc_chip_set_channel_ret", "(chip)")
        tl(req + int(20.0e6), "kc_drv_set_channel_ret", "(rtw_set_channel)")
        tl(req + int(21.0e6), "rdev_return_int", "ret=0")
        tl(req + int(1.0e6), "stop_queue", "queue=0")
        tl(req + int(23.0e6), "wake_queue", "queue=0")
        # dead air: frames sent during it are lost.
        lost = dead_ns // period
        ctr += lost
        t += dead_ns
        # post-switch stream on `to` (skipped for the injected failure case).
        if i != fail_i:
            for _ in range(dwell // period):
                ctr += 1
                inject.append({"ev": "inj.tx", "n": ctr, "mono_ns": t,
                               "errno": 0})
                (oa if to == 36 else ob).append(mk_frame(to, t, ctr))
                t += period
        else:
            t += dwell

    for path, recs in (("oracle_a.jsonl", oa), ("oracle_b.jsonl", ob),
                       ("control.jsonl", control), ("inject.jsonl", inject)):
        with open(os.path.join(cfg, path), "w") as f:
            for r in recs:
                f.write(json.dumps(r) + "\n")
    with open(os.path.join(cfg, "trace.txt"), "w") as f:
        f.write("\n".join(trace_lines) + "\n")

    rows, fits = analyze_run(tmp)
    summary, _ = write_outputs(tmp, rows, fits)
    s = summary["a36-40"]

    def approx(v, want, tol):
        return v is not None and abs(v - want) <= tol

    checks = [
        ("switch rows", len(rows) == n_sw),
        ("one no-RF failure", s["failures"].get("no_rf_evidence", 0) == 1
         and s["ok"] == n_sw - warmup - 1),
        # dead-air median: truth 25 ms; quantization = one injector period.
        ("dead-air median ≈ 25 ms",
         approx(s["dead_air"]["median_ms"], 25.0, 1.5)),
        ("gate = control", s["gate"] == "control"),
        ("host-cmd ≈ 18.8 ms",
         approx(s["host_cmd"]["median_ms"], 18.8, 0.5)),
        ("driver ≈ 17.5 ms", approx(s["driver"]["median_ms"], 17.5, 0.5)),
        ("queue-stopped ≈ 22 ms",
         approx(s["queue_stopped"]["median_ms"], 22.0, 0.5)),
        ("gap accounting", s["gap_frames_total"] ==
         (n_sw - warmup - 1) * (dead_ns // period)),
        ("both fits ok", all(f.get("ok") for f in fits.values())),
        ("fit slope ppm sane", all(abs(f["slope_err_ppm"]) < 200
                                   for f in fits.values() if f.get("ok"))),
    ]
    failed = [name for name, ok in checks if not ok]
    for name, ok in checks:
        print(f"  [{'ok' if ok else 'FAIL'}] {name}")
    if failed:
        print(json.dumps(s, indent=2))
        return 1
    print("kchansw_analyze --self-test: all checks passed")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run_dir", nargs="?", help="bench run directory")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        import tempfile
        with tempfile.TemporaryDirectory(prefix="kchansw-selftest-") as tmp:
            return _selftest(tmp)
    if not args.run_dir:
        ap.print_help()
        return 1
    rows, fits = analyze_run(args.run_dir)
    if not rows:
        sys.stderr.write("no per-switch rows extracted — is this a run dir?\n")
        return 1
    summary, md = write_outputs(args.run_dir, rows, fits)
    print(md)
    print(f"[{len(rows)} switches → {os.path.join(args.run_dir, 'switches.csv.gz')}]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
