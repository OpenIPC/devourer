#!/usr/bin/env python3
# txegress_analyze.py — quantify the "read TSF after send to approximate egress"
# submit-to-air jitter from txegress_witness JSONL.
#
# Each record has a TX stamp (tx_sw = the transmitter's software ReadTsf, OR
# tx_hw = the MAC-inserted beacon egress TSF) and rx_tsfl (this receiver's
# hardware RX timestamp = a low-jitter proxy for true on-air time). Both TX
# stamps are in the transmitter's clock; rx_tsfl is in the receiver's clock, so
# a straight line rx_tsfl = a*tx + b absorbs the constant offset, the two
# crystals' rate difference, and the fixed propagation delay. What's LEFT — the
# residual — is the per-frame jitter of that TX stamp versus true air.
#
# Two numbers per stamp:
#   raw   — RMS over ALL frames. On a busy channel this is dominated by MAC
#           deferral / queueing outliers (a channel effect, not the transport):
#           the raw slope goes wild (implausible ppm) when a few frames are
#           milliseconds late.
#   floor — robust RMS after iterative 3*MAD outlier rejection: the tight cluster
#           of frames that aired immediately, i.e. the transport + MAC-pipeline
#           floor. Deferral only ever ADDS delay, so rejecting the positive tail
#           recovers the true transport jitter. A sane crystal ppm (~tens of ppm
#           or less) confirms the robust fit locked onto the real line.
#
# The M0 DL-departure contract rides the same fit: TD v2 frames also carry
# tx_host_ns (the transmitter's steady_clock immediately before send_packet),
# and the residual TAIL of the host-clock fit is the guard a host-side slot
# scheduler must budget. --transport labels the run and --slot-us sets the
# go/no-go bound; both emit a machine-readable txeg.verdict JSONL line with
# p50/p90/p99/p99.9/max of the residuals against the robust line (guard_us =
# p99.9 - p50 over ALL frames, i.e. deferral tail included).
#
# Usage: python3 txegress_analyze.py <witness.jsonl> [--transport NAME] [--slot-us N]
import json, sys, math

def unwrap32(xs):
    out, hi, prev = [], 0, None
    for x in xs:
        if prev is not None and x < prev - (1 << 31):
            hi += 1 << 32
        out.append(x + hi); prev = x
    return out

def fit(tx, rx, idx):
    n = len(idx)
    mx = sum(tx[i] for i in idx) / n
    my = sum(rx[i] for i in idx) / n
    sxx = sum((tx[i] - mx) ** 2 for i in idx)
    sxy = sum((tx[i] - mx) * (rx[i] - my) for i in idx)
    a = sxy / sxx if sxx else 1.0
    return a, my - a * mx

def resid(tx, rx, idx, a, b):
    return [rx[i] - (a * tx[i] + b) for i in idx]

def rms_p2p(res):
    n = len(res)
    return math.sqrt(sum(r * r for r in res) / n), (max(res) - min(res))

def robust_fit(tx, rx):
    """Iterative 3*MAD outlier rejection -> (slope, inlier_idx)."""
    idx = list(range(len(tx)))
    for _ in range(6):
        a, b = fit(tx, rx, idx)
        res = resid(tx, rx, idx, a, b)
        s = sorted(res); med = s[len(s) // 2]
        ab = sorted(abs(r - med) for r in res); mad = ab[len(ab) // 2] * 1.4826 or 1.0
        keep = [idx[k] for k in range(len(idx)) if abs(res[k] - med) < 3 * mad]
        if len(keep) == len(idx):
            break
        idx = keep
    return idx

def pct(sorted_xs, p):
    if not sorted_xs:
        return 0.0
    k = min(len(sorted_xs) - 1, max(0, int(round(p / 100.0 * (len(sorted_xs) - 1)))))
    return sorted_xs[k]

def main():
    args = sys.argv[1:]
    transport, slot_us = "", None
    if "--transport" in args:
        i = args.index("--transport"); transport = args[i + 1]; del args[i:i + 2]
    if "--slot-us" in args:
        i = args.index("--slot-us"); slot_us = float(args[i + 1]); del args[i:i + 2]
    recs = []
    for line in open(args[0]):
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            recs.append(json.loads(line))
        except Exception:
            continue
    recs = [r for r in recs if r.get("ev") == "txeg"][3:]   # drop warm-up
    if len(recs) < 20:
        print("too few frames (%d)" % len(recs)); return 1

    rx = unwrap32([r["rx_tsfl"] for r in recs])
    for label, key, scale in (("software (ReadTsf near send)", "tx_sw", 1.0),
                              ("hardware (MAC beacon egress)", "tx_hw", 1.0),
                              ("host (steady_clock at send)", "tx_host_ns", 1e-3)):
        sub = [(r[key] * scale, x) for r, x in zip(recs, rx) if r.get(key, 0) > 0]
        if len(sub) < 20:
            print("%-30s : n=%d (skipped)" % (label, len(sub))); continue
        tx = [s[0] for s in sub]; rxs = [s[1] for s in sub]
        n = len(tx)

        a0, b0 = fit(tx, rxs, list(range(n)))
        raw_rms, _ = rms_p2p(resid(tx, rxs, list(range(n)), a0, b0))

        idx = robust_fit(tx, rxs)
        a, b = fit(tx, rxs, idx)
        fl_rms, fl_p2p = rms_p2p(resid(tx, rxs, idx, a, b))
        ppm = (a - 1.0) * 1e6   # tx pre-scaled to µs, so slope ~1 for all stamps
        print("%-30s : n=%4d  raw RMS=%8.1f µs  |  floor=%7.1f µs "
              "(%d inliers, p2p %.1f µs)  xtal %+6.1f ppm"
              % (label, n, raw_rms, fl_rms, len(idx), fl_p2p, ppm))

        if key == "tx_host_ns":
            # The DL-departure guard: residuals of ALL frames against the robust
            # line (deferral tail included — a scheduler eats deferral too),
            # measured from the median so the guard is "how much later than
            # nominal can a frame air".
            res = sorted(resid(tx, rxs, list(range(n)), a, b))
            med = pct(res, 50)
            ps = {p: pct(res, p) - med for p in (50, 90, 99, 99.9)}
            guard = ps[99.9]
            v = {"ev": "txeg.verdict", "transport": transport, "n": n,
                 "floor_rms_us": round(fl_rms, 1),
                 "p50_us": round(ps[50], 1), "p90_us": round(ps[90], 1),
                 "p99_us": round(ps[99], 1), "p999_us": round(ps[99.9], 1),
                 "max_us": round(res[-1] - med, 1),
                 "guard_us": round(guard, 1),
                 "xtal_ppm": round(ppm, 1)}
            if slot_us is not None:
                v["slot_us"] = slot_us
                v["fine_dl_slots"] = bool(guard <= slot_us)
            print(json.dumps(v))

    print("\nfloor = the transport + MAC-pipeline submit-to-air jitter (deferral")
    print("outliers rejected); raw includes channel deferral. The txeg.verdict")
    print("line is the M0 DL-departure contract: guard_us = host-clock p99.9")
    print("residual above the median, i.e. the submission-ahead margin a slot")
    print("scheduler must budget on this transport.")
    return 0

if __name__ == "__main__":
    sys.exit(main())
