#!/usr/bin/env python3
# beacon_update_analyze.py — M0 contract 2 detector: turn a beacon_update_probe
# TX stream (bcn.update calls) + witness stream (bcn.rx per aired beacon) into
# the dynamic-grant-delivery verdict.
#
# The witness's tsfl (hardware RX timestamp) reconstructs the TBTT grid:
# consecutive-beacon deltas cluster at interval_tu*1024 µs; a delta of k
# periods means k-1 grid slots carried no decoded beacon. A missing slot is
# EITHER a real skip (the chip didn't air) or witness RX loss — indistinguishable
# per-slot, so the verdict separates the loss rate NEAR updates (within
# ±attrib_periods of an update call) from the BACKGROUND rate away from them:
# the excess near updates is the update's real cost.
#   skips_per_update  = near-update missing slots / updates  (background-corrected)
#   dup               = same TBTT slot decoded twice
#   late              = beacon >lateness_us off its grid slot
#   stale/latency     = per update, host_ns delta from the update call to the
#                       first aired beacon carrying the NEW version (both
#                       processes share one host CLOCK_MONOTONIC)
#   torn              = crc_ok=false frames (a partial content swap)
#   ver_regress       = version going backward (old content re-airing)
#
# Usage: python3 beacon_update_analyze.py <witness.jsonl> <probe.jsonl>
#            [--gen NAME] [--interval-tu 100]
#        python3 beacon_update_analyze.py --selftest
import json, sys

def load(path, ev):
    out = []
    for line in open(path):
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            r = json.loads(line)
        except Exception:
            continue
        if r.get("ev") == ev:
            out.append(r)
    return out

def unwrap32(xs):
    out, hi, prev = [], 0, None
    for x in xs:
        if prev is not None and x < prev - (1 << 31):
            hi += 1 << 32
        out.append(x + hi); prev = x
    return out

def pct(sorted_xs, p):
    if not sorted_xs:
        return 0.0
    k = min(len(sorted_xs) - 1, max(0, int(round(p / 100.0 * (len(sorted_xs) - 1)))))
    return sorted_xs[k]

def analyze(rx, updates, gen="", interval_tu=100, attrib_periods=3,
            lateness_us=2000):
    period = interval_tu * 1024
    if len(rx) < 10:
        return {"ev": "bcn.verdict", "gen": gen, "error": "too few beacons",
                "n_rx": len(rx)}
    tsf = unwrap32([r["tsfl"] for r in rx])

    # Grid walk over consecutive decodes: slot deltas, dups, lateness.
    missing = []        # (tsf_of_gap_start, n_missing_slots)
    dups = 0
    late = 0
    phases = []
    for i in range(1, len(tsf)):
        d = tsf[i] - tsf[i - 1]
        k = round(d / period)
        if k == 0:
            dups += 1
            continue
        off = d - k * period
        phases.append(off)
        if abs(off) > lateness_us:
            late += 1
        if k > 1:
            missing.append((tsf[i - 1], k - 1))

    # host_ns <-> tsf line through the decoded beacons (same host for both
    # streams, so update host_ns maps onto the witness grid through this fit).
    hs = [r["host_ns"] / 1000.0 for r in rx]  # µs
    n = len(tsf)
    mt = sum(tsf) / n; mh = sum(hs) / n
    sxx = sum((t - mt) ** 2 for t in tsf)
    sxy = sum((t - mt) * (h - mh) for t, h in zip(tsf, hs))
    a = sxy / sxx if sxx else 1.0
    b = mh - a * mt
    def host_us_of_tsf(t):
        return a * t + b

    # Near-update vs background missing-slot attribution.
    upd_us = [u["host_ns"] / 1000.0 for u in updates]
    near_missing = 0
    far_missing = 0
    win = attrib_periods * period
    for t0, k in missing:
        h0 = host_us_of_tsf(t0)
        if any(abs(h0 - u) <= win for u in upd_us):
            near_missing += k
        else:
            far_missing += k
    total_slots = round((tsf[-1] - tsf[0]) / period) + 1
    far_slots = max(1, total_slots - len(upd_us) * (2 * attrib_periods + 1))
    bg_rate = far_missing / far_slots
    n_upd = max(1, len(upd_us))
    # Background-corrected: excess missing slots near updates, per update.
    skips_per_update = max(0.0, (near_missing - bg_rate *
                                 len(upd_us) * (2 * attrib_periods + 1)) / n_upd)

    # Update -> first-new-version-on-air latency; version regressions.
    lat_us = []
    regress = 0
    prev_ver = None
    for r in rx:
        if prev_ver is not None and r["ver"] < prev_ver:
            regress += 1
        prev_ver = r["ver"]
    for u in updates:
        first = next((r for r in rx if r["ver"] >= u["ver"] and
                      r["host_ns"] >= u["host_ns"]), None)
        if first is not None:
            lat_us.append((first["host_ns"] - u["host_ns"]) / 1000.0)
    lat_us.sort()

    torn = sum(1 for r in rx if not r.get("crc_ok", True))
    v = {"ev": "bcn.verdict", "gen": gen, "n_rx": len(rx),
         "n_updates": len(upd_us),
         "updates_ok": sum(1 for u in updates if u.get("ok", False)),
         "skips_per_update": round(skips_per_update, 2),
         "bg_loss_rate": round(bg_rate, 4),
         "near_missing": near_missing, "far_missing": far_missing,
         "dups": dups, "late": late, "torn": torn, "ver_regress": regress,
         "lat_p50_ms": round(pct(lat_us, 50) / 1000.0, 1) if lat_us else None,
         "lat_p99_ms": round(pct(lat_us, 99) / 1000.0, 1) if lat_us else None,
         "lat_max_ms": round(lat_us[-1] / 1000.0, 1) if lat_us else None,
         "updates_seen_on_air": len(lat_us)}
    # Go/no-go: every update aired, no torn frames, no regressions, and the
    # per-update cost within the documented steer bound (<= 1 skipped beacon
    # + measurement slack).
    v["dynamic_grants_ok"] = bool(
        len(lat_us) == len(upd_us) and torn == 0 and regress == 0 and
        skips_per_update <= 1.5)
    return v

def selftest():
    period = 102400  # 100 TU in µs
    fails = 0
    def check(cond, msg):
        nonlocal fails
        if not cond:
            print("FAIL:", msg); fails += 1

    # Clean run: 200 beacons, updates every 20 periods, one skipped slot per
    # update, new version airs on the next decoded beacon.
    rx, updates = [], []
    ver = 1
    t = 0
    host0 = 5_000_000_000  # ns
    skip_next = False
    for i in range(200):
        if i and i % 20 == 0:
            ver += 1
            updates.append({"ver": ver, "ok": True,
                            "host_ns": host0 + t * 1000 - 5_000_000})
            skip_next = True
        if skip_next:
            skip_next = False   # the update's one skipped beacon
        else:
            rx.append({"seq": i & 0xfff, "tsfl": t % (1 << 32), "ver": ver,
                       "crc_ok": True, "host_ns": host0 + t * 1000})
        t += period
    v = analyze(rx, updates, gen="selftest-clean")
    check(v["dynamic_grants_ok"], "clean run passes")
    check(0.5 <= v["skips_per_update"] <= 1.5,
          "one skip per update measured (got %s)" % v["skips_per_update"])
    check(v["torn"] == 0 and v["ver_regress"] == 0, "clean run: no torn/regress")
    check(v["updates_seen_on_air"] == len(updates), "every update airs")
    check(v["lat_p50_ms"] is not None and v["lat_p50_ms"] < 3 * period / 1000.0,
          "latency ~ a couple periods")

    # Torn frame + version regression must fail the verdict.
    rx2 = [dict(r) for r in rx]
    rx2[50]["crc_ok"] = False
    v2 = analyze(rx2, updates, gen="selftest-torn")
    check(v2["torn"] == 1 and not v2["dynamic_grants_ok"], "torn frame fails")
    rx3 = [dict(r) for r in rx]
    rx3[60]["ver"] = 1
    v3 = analyze(rx3, updates, gen="selftest-regress")
    check(v3["ver_regress"] >= 1 and not v3["dynamic_grants_ok"],
          "version regression fails")

    # Background witness loss away from updates must NOT count against updates.
    rx4 = [r for i, r in enumerate(rx) if i % 20 != 7]  # periodic far loss
    v4 = analyze(rx4, updates, gen="selftest-bgloss")
    check(v4["skips_per_update"] <= 1.5,
          "background loss not attributed to updates (got %s)" %
          v4["skips_per_update"])

    # 32-bit tsfl wrap mid-run parses fine.
    rx5 = [dict(r, tsfl=(r["tsfl"] + (1 << 32) - 50 * period) % (1 << 32))
           for r in rx]
    v5 = analyze(rx5, updates, gen="selftest-wrap")
    check(v5["n_rx"] == len(rx5) and v5["dups"] == 0, "tsfl wrap handled")

    print("beacon_update_analyze selftest:",
          "%d FAILURE(S)" % fails if fails else "all passed")
    return 1 if fails else 0

def main():
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    args = sys.argv[1:]
    gen, interval_tu = "", 100
    if "--gen" in args:
        i = args.index("--gen"); gen = args[i + 1]; del args[i:i + 2]
    if "--interval-tu" in args:
        i = args.index("--interval-tu"); interval_tu = int(args[i + 1]); del args[i:i + 2]
    rx = load(args[0], "bcn.rx")
    updates = load(args[1], "bcn.update")
    v = analyze(rx, updates, gen=gen, interval_tu=interval_tu)
    print(json.dumps(v))
    sys.exit(0 if v.get("dynamic_grants_ok") else 1)

if __name__ == "__main__":
    main()
