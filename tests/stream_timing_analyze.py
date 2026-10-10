#!/usr/bin/env python3
"""stream_timing_analyze.py — summarize one rxdemo JSONL capture of the stream
timing telemetry (src/StreamTelemetry.h): per-frame rx.frame fields `tel`,
`depth`, `c2s_us`, `cap`, `lat_us`, `c2a_us`, and the per-marker `rx.timing`.

    python3 tests/stream_timing_analyze.py rx.jsonl            # summary line (JSON)
    python3 tests/stream_timing_analyze.py rx.jsonl --expect-delay 10:20
        # the producer slept 20 ms between CAPTURE_TS and every 10th record:
        # those frames' c2s must stand 20 ms (+-TOL) above the others' median,
        # and about 1 in 10 frames must be such a frame.
    --min-lat-frames N   require at least N frames carrying lat_us (default 100)
    --tol-ms MS          tolerance for --expect-delay (default 6)
    --no-abs             do not require lat_us (a part/mode with no egress pairs)
    --warmup-s S         drop the first S seconds of telemetry frames (default 4):
                         the producer fills the stdin pipe while streamtx brings the
                         chip up, so the first ~1000 records carry stale stamps — a
                         true backlog reading, but not the steady state under test

Exit 0 = every check passed, 1 = a check failed, 3 = too little data. The
summary is one JSON object on stdout (ev = stream_timing.summary) so a script
can grep it like any other event line.
"""
import argparse
import json
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from devourer_events import iter_events  # noqa: E402


def pct(v, q):
    if not v:
        return None
    s = sorted(v)
    i = min(len(s) - 1, max(0, int(round(q * (len(s) - 1)))))
    return s[i]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rx_jsonl")
    ap.add_argument("--expect-delay", default=None, metavar="N:MS")
    ap.add_argument("--tol-ms", type=float, default=6.0)
    ap.add_argument("--min-lat-frames", type=int, default=100)
    ap.add_argument("--no-abs", action="store_true")
    ap.add_argument("--warmup-s", type=float, default=4.0)
    args = ap.parse_args(argv)

    frames, markers = [], []
    with open(args.rx_jsonl, "rb") as f:
        for e in iter_events(f):
            ev = e.get("ev")
            if ev == "rx.frame" and e.get("sa") == "57427505d600":
                frames.append(e)
            elif ev == "rx.timing":
                markers.append(e)

    tel = [e for e in frames if e.get("tel") == 1]
    warm = 0
    if tel and args.warmup_s > 0:
        t0 = tel[0]["tsfl"]
        keep = [e for e in tel if ((e["tsfl"] - t0) & 0xffffffff) >= args.warmup_s * 1e6]
        warm = len(tel) - len(keep)
        tel = keep
    lat = [e["lat_us"] for e in tel if "lat_us" in e]
    c2a = [e["c2a_us"] for e in tel if "c2a_us" in e]
    c2s = [e["c2s_us"] for e in tel]
    cap = [e["c2s_us"] for e in tel if e.get("cap") == 1]
    depth = [e.get("depth", 0) for e in tel]
    out = {
        "ev": "stream_timing.summary",
        "frames": len(frames), "tel": len(tel), "warmup_dropped": warm, "lat_frames": len(lat),
        "cap_frames": len(cap), "markers": len(markers),
        "lat_p50_us": pct(lat, 0.5), "lat_p99_us": pct(lat, 0.99),
        "lat_max_us": max(lat) if lat else None,
        "lat_min_us": min(lat) if lat else None,
        "c2s_p50_us": pct(c2s, 0.5), "c2s_p99_us": pct(c2s, 0.99),
        "c2a_p50_us": pct(c2a, 0.5), "c2a_p99_us": pct(c2a, 0.99),
        "depth_max": max(depth) if depth else None,
        "pairs": markers[-1].get("pairs") if markers else None,
        "rx_resid_us": markers[-1].get("rx_resid_us") if markers else None,
        "presp_stamped": markers[-1].get("presp_stamped") if markers else None,
        "beacon": markers[-1].get("beacon") if markers else None,
        "tx_async": markers[-1].get("tx_async") if markers else None,
        "tw_p50_us": pct([m["tw_p50_us"] for m in markers], 0.5) if markers else None,
        "tw_max_us": max((m["tw_max_us"] for m in markers), default=None),
        "tq_max_us": max((m["tq_max_us"] for m in markers), default=None),
        "checks": {},
    }

    ok = True
    if len(tel) < args.min_lat_frames:
        print(json.dumps(out)); print(f"TOO LITTLE DATA: {len(tel)} telemetry frames", file=sys.stderr)
        return 3
    if not args.no_abs:
        c = len(lat) >= args.min_lat_frames
        out["checks"]["abs_latency_present"] = c
        ok &= c
        if lat:
            # Submit->air can only be negative by the fit error; a tail of
            # large negatives means the clock mapping is wrong.
            c = pct(lat, 0.01) > -2000
            out["checks"]["no_negative_tail"] = c
            ok &= c
    if args.expect_delay:
        n, ms = args.expect_delay.split(":")
        n, ms = int(n), float(ms)
        if not cap:
            out["checks"]["delay_step"] = False
            ok = False
        else:
            base = pct(cap, 0.5)
            hi = [v for v in cap if v - base > (ms * 1000) / 2]
            frac = len(hi) / len(cap)
            step = (statistics.median(hi) - base) / 1000.0 if hi else None
            out["delay_frac"] = frac
            out["delay_step_ms"] = step
            c_frac = abs(frac - 1.0 / n) < 0.5 / n
            c_step = step is not None and abs(step - ms) <= args.tol_ms
            out["checks"]["delay_fraction"] = c_frac
            out["checks"]["delay_step"] = c_step
            ok &= c_frac and c_step
    print(json.dumps(out))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
