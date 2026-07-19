#!/usr/bin/env python3
"""B210 energy-gap probe for the kernel channel-switch bench.

Answers the decode-oracle bias question: the rxdemo oracles time a switch by
FIRST DECODED FRAME; RF energy lands earlier (AGC settle, preamble sync,
partial frames). This probe watches ONE of the two hop channels with the
B210 while the bench's alternating switch loop runs, computes per-window
in-band power, extracts the OFF-gap duration distribution (channel silent
while the DUT dwells on the other channel + both transitions), and reports
it for comparison against the same cycle's decode-derived OFF gaps from
kchansw_analyze — no cross-clock alignment needed, distributions compare
directly on independent clocks.

Partial-band trick: sampling 4-8 MS/s inside a 20 MHz OFDM signal captures
a proportional slice of its power — plenty for on/off detection — while
staying well inside the python-streaming rate the B210 sustains without
overflows (full 56 MS/s needs a compiled consumer; see repo SDR lore).

Usage:  tests/.venv/bin/python tests/kchansw_b210_gap.py \
            --freq 5200e6 --rate 8e6 --secs 20 --out /tmp/b210_gap.json
"""

import argparse
import json
import sys

import numpy as np


def capture(freq, rate, secs, gain):
    import uhd
    usrp = uhd.usrp.MultiUSRP("type=b200")
    usrp.set_rx_rate(rate)
    usrp.set_rx_freq(uhd.libpyuhd.types.tune_request(freq))
    usrp.set_rx_gain(gain)
    st_args = uhd.usrp.StreamArgs("fc32", "sc16")
    st_args.channels = [0]
    rx = usrp.get_rx_stream(st_args)
    md = uhd.types.RXMetadata()
    n_total = int(rate * secs)
    buf = np.empty(n_total, dtype=np.complex64)
    chunk = np.empty(rx.get_max_num_samps() * 16, dtype=np.complex64)
    cmd = uhd.types.StreamCMD(uhd.types.StreamMode.start_cont)
    cmd.stream_now = True
    rx.issue_stream_cmd(cmd)
    got = 0
    overflows = 0
    while got < n_total:
        n = rx.recv(chunk, md, 1.0)
        if md.error_code == uhd.types.RXMetadataErrorCode.overflow:
            overflows += 1
            continue
        if md.error_code != uhd.types.RXMetadataErrorCode.none:
            sys.stderr.write(f"rx error: {md.error_code}\n")
            break
        take = min(n, n_total - got)
        buf[got:got + take] = chunk[:take]
        got += take
    rx.issue_stream_cmd(uhd.types.StreamCMD(uhd.types.StreamMode.stop_cont))
    return buf[:got], overflows


def gaps_from_power(buf, rate, win_s=0.5e-3):
    """Per-window power → on/off threshold → OFF-gap durations (ms)."""
    win = int(rate * win_s)
    n = len(buf) // win
    p = (np.abs(buf[:n * win].reshape(n, win)) ** 2).mean(axis=1)
    pdb = 10 * np.log10(p + 1e-12)
    # Threshold halfway between the two modes (quiet floor vs on-air).
    lo, hi = np.percentile(pdb, 10), np.percentile(pdb, 90)
    thr = (lo + hi) / 2
    on = pdb > thr
    # OFF-run lengths, ignoring the truncated first/last runs.
    gaps = []
    run = 0
    started = False
    for v in on:
        if v:
            if started and run:
                gaps.append(run * win_s * 1e3)
            run = 0
            started = True
        else:
            run += 1
    return np.array(gaps), float(thr), float(hi - lo)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--freq", type=float, default=5200e6)
    ap.add_argument("--rate", type=float, default=8e6)
    ap.add_argument("--secs", type=float, default=20.0)
    ap.add_argument("--gain", type=float, default=45.0)
    ap.add_argument("--min-gap-ms", type=float, default=5.0,
                    help="ignore shorter OFF runs (inter-frame spacing)")
    ap.add_argument("--max-gap-ms", type=float, default=2000.0,
                    help="ignore longer OFF runs (bench idle, recovery)")
    ap.add_argument("--out", default="/tmp/b210_gap.json")
    args = ap.parse_args()

    buf, overflows = capture(args.freq, args.rate, args.secs, args.gain)
    gaps, thr, contrast_db = gaps_from_power(buf, args.rate)
    sel = gaps[(gaps >= args.min_gap_ms) & (gaps <= args.max_gap_ms)]
    result = {
        "freq_mhz": args.freq / 1e6, "rate_msps": args.rate / 1e6,
        "secs_captured": len(buf) / args.rate, "overflows": overflows,
        "threshold_dbfs": thr, "contrast_db": contrast_db,
        "gaps_all": len(gaps), "gaps_selected": len(sel),
        "gap_ms": {
            "median": float(np.median(sel)) if len(sel) else None,
            "p10": float(np.percentile(sel, 10)) if len(sel) else None,
            "p90": float(np.percentile(sel, 90)) if len(sel) else None,
            "min": float(sel.min()) if len(sel) else None,
            "max": float(sel.max()) if len(sel) else None,
        },
    }
    with open(args.out, "w") as f:
        json.dump(result, f, indent=2)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
