#!/usr/bin/env python3
# txegress_analyze.py — quantify the "read TSF after send to approximate egress"
# micro-option from txegress_witness JSONL.
#
# Each record has a TX stamp (tx_sw = the transmitter's software ReadTsf, OR
# tx_hw = the MAC-inserted beacon egress TSF) and rx_tsfl (this receiver's
# hardware RX timestamp = a low-jitter proxy for true on-air time). Both TX
# stamps are in the transmitter's clock; rx_tsfl is in the receiver's clock, so
# a straight line rx_tsfl = a*tx + b absorbs the constant offset, the two
# crystals' rate difference, and the fixed propagation delay. What's LEFT — the
# residual RMS — is the per-frame jitter of that TX stamp versus true air.
#
#   software-stamp residual = jitter the read-TSF-after-send micro-option injects
#   hardware-stamp residual = the MAC egress-timestamp floor (what a real TX HW
#                             timestamp would look like)
#
# Usage: python3 txegress_analyze.py <witness.jsonl>
import json, sys, math

def unwrap32(xs):
    out, hi, prev = [], 0, None
    for x in xs:
        if prev is not None and x < prev - (1 << 31):
            hi += 1 << 32
        out.append(x + hi); prev = x
    return out

def fit_resid(tx, rx):
    # least-squares rx = a*tx + b; return residual RMS (ns) and slope (ppm dev)
    n = len(tx)
    mx = sum(tx) / n; my = sum(rx) / n
    sxx = sum((t - mx) ** 2 for t in tx)
    sxy = sum((t - mx) * (r - my) for t, r in zip(tx, rx))
    a = sxy / sxx; b = my - a * mx
    res = [r - (a * t + b) for t, r in zip(tx, rx)]   # residual in rx-clock µs
    rms_us = math.sqrt(sum(r * r for r in res) / n)
    # rx_tsfl and tx are both µs units; residual is µs -> ns
    p2p = max(res) - min(res)
    return rms_us * 1000.0, p2p * 1000.0, a

def main():
    recs = []
    for line in open(sys.argv[1]):
        line = line.strip()
        if not line.startswith("{"): continue
        try: recs.append(json.loads(line))
        except Exception: continue
    recs = [r for r in recs if r.get("ev") == "txeg"]
    # drop the first few (warm-up) and any with a zero stamp
    recs = recs[3:]
    if len(recs) < 20:
        print("too few frames (%d)" % len(recs)); return

    rx = unwrap32([r["rx_tsfl"] for r in recs])
    for label, key in (("software (ReadTsf near send)", "tx_sw"),
                       ("hardware (MAC beacon egress)", "tx_hw")):
        sub = [(r[key], x) for r, x in zip(recs, rx) if r.get(key, 0) > 0]
        if len(sub) < 20:
            print("%-32s : n=%d (skipped)" % (label, len(sub))); continue
        tx = [s[0] for s in sub]; rxs = [s[1] for s in sub]
        rms, p2p, slope = fit_resid(tx, rxs)
        ppm = (slope - 1.0) * 1e6
        print("%-32s : n=%4d  residual RMS = %8.3f µs  (p2p %8.3f µs)  xtal %+7.1f ppm"
              % (label, len(sub), rms / 1000.0, p2p / 1000.0, ppm))

    print("\nThe software residual is the jitter 'read TSF after send' injects vs true")
    print("air; the hardware residual is the MAC egress-timestamp floor. The gap is")
    print("why the shortcut is not a substitute for a real TX-egress timestamp.")

if __name__ == "__main__":
    main()
