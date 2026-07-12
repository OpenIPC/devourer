#!/usr/bin/env python3
"""Burst-duration histogram via USRP — fingerprint what the TX actually airs.

A frame's airtime is set by its PHY rate, so the burst-length distribution
identifies the on-air rate without decoding: a 148-byte beacon is ~80us at
HT MCS3/20, ~66us at MCS4, ~230us as legacy 6M, ~1.2ms as CCK 1M. Run while
txdemo emits at a pinned rate with a fixed gap; compare the measured median
burst against the expected airtime (issue #238 MCS4+ bisect).

  sudo python3 tests/sdr_burst_len.py --freq 5180e6 --secs 4
"""
from __future__ import annotations
import argparse, sys
import numpy as np

try:
    import uhd
except ImportError:
    sys.stderr.write("need UHD python (system pkg) — run with system python3\n")
    sys.exit(2)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--freq", type=float, default=5180e6)
    ap.add_argument("--rate", type=float, default=25e6)
    ap.add_argument("--gain", type=float, default=60)
    ap.add_argument("--secs", type=float, default=4)
    ap.add_argument("--thr-db", type=float, default=8.0,
                    help="burst threshold above noise floor")
    a = ap.parse_args()

    usrp = uhd.usrp.MultiUSRP("")
    usrp.set_rx_rate(a.rate)
    usrp.set_rx_freq(uhd.types.TuneRequest(a.freq))
    usrp.set_rx_gain(a.gain)
    n = int(a.secs * a.rate)
    buf = usrp.recv_num_samps(n, a.freq, a.rate, [0], a.gain)[0]

    # Envelope at 1us resolution.
    dec = max(1, int(a.rate // 1_000_000))
    p = np.abs(buf) ** 2
    us = p[: len(p) // dec * dec].reshape(-1, dec).mean(axis=1)
    pdb = 10 * np.log10(us + 1e-15)
    noise = np.percentile(pdb, 20)
    on = pdb > (noise + a.thr_db)

    # Burst lengths in us (merge <=2us gaps: OFDM cyclic dips).
    bursts, run, gap = [], 0, 0
    for v in on:
        if v:
            run += gap + 1
            gap = 0
        elif run:
            gap += 1
            if gap > 2:
                bursts.append(run)
                run, gap = 0, 0
    if run:
        bursts.append(run)
    bursts = [b for b in bursts if b >= 8]  # drop spikes
    if not bursts:
        print("no bursts detected (noise=%.1f dB)" % noise)
        return 1
    arr = np.array(bursts)
    hist = {}
    for b in arr:
        hist[10 * int(b // 10)] = hist.get(10 * int(b // 10), 0) + 1
    top = sorted(hist.items(), key=lambda kv: -kv[1])[:6]
    print(
        "bursts=%d median=%dus p10=%dus p90=%dus noise=%.1fdB thr=%.1fdB"
        % (len(arr), int(np.median(arr)), int(np.percentile(arr, 10)),
           int(np.percentile(arr, 90)), noise, noise + a.thr_db)
    )
    print("top bins (us:count):", " ".join(f"{k}us:{v}" for k, v in top))
    return 0


if __name__ == "__main__":
    sys.exit(main())
