#!/usr/bin/env python3
"""B210 burst-pair probe for the NDPA->NDP hardware-sounding experiment.

Streams IQ from a UHD device, detects TX bursts by envelope threshold, and
reports each burst's duration plus the gap to the previous burst. The
signature of a working hardware sounding sequence is a *pair* of bursts per
injected NDPA — the NDPA PPDU, a SIFS (16 us), then the MAC-generated NDP —
where the baseline (descriptor NDPA bit off) shows isolated single bursts at
the txdemo inter-frame gap (~2 ms).

Output (line-buffered):
    bf-probe burst t=<s> dur_us=<f> gap_prev_us=<f>
    bf-probe summary bursts=<n> pairs=<n> singles=<n> pair_frac=<f> \
        median_pair_gap_us=<f> median_dur_us=<f>

A "pair" = consecutive bursts with gap in [4, 60] us (SIFS-ish, well below
the inter-injection gap). Power thresholding is relative: 12 dB above the
per-chunk median (idle floor), bursts 10..400 us kept.

Run:  .venv/bin/python bf_ndpa_probe.py --freq 5500e6 --rate 10e6 --duration 10
"""
from __future__ import annotations

import argparse
import sys

import numpy as np

try:
    import uhd
except ImportError:
    sys.stderr.write(
        "bf_ndpa_probe: `import uhd` failed. Use the tests/.venv created with "
        "`uv venv --system-site-packages` (UHD python is a system package).\n"
    )
    raise


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--freq", type=float, default=5500e6)
    ap.add_argument("--rate", type=float, default=10e6)
    ap.add_argument("--gain", type=float, default=40.0)
    ap.add_argument("--antenna", default="RX2")
    ap.add_argument("--args", default="", help="UHD device args")
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--thresh-db", type=float, default=12.0,
                    help="burst threshold above per-chunk median floor (dB)")
    ap.add_argument("--pair-gap-us", type=float, nargs=2, default=[4.0, 60.0],
                    help="gap window (us) classifying a burst pair")
    args = ap.parse_args()

    usrp = uhd.usrp.MultiUSRP(args.args)
    usrp.set_rx_rate(args.rate)
    usrp.set_rx_freq(uhd.types.TuneRequest(args.freq))
    usrp.set_rx_gain(args.gain)
    usrp.set_rx_antenna(args.antenna)

    st_args = uhd.usrp.StreamArgs("fc32", "sc16")
    rx = usrp.get_rx_stream(st_args)
    md = uhd.types.RXMetadata()
    cmd = uhd.types.StreamCMD(uhd.types.StreamMode.start_cont)
    cmd.stream_now = True
    rx.issue_stream_cmd(cmd)

    chunk = int(args.rate / 10)  # 100 ms
    buf = np.empty(chunk, dtype=np.complex64)
    us_per_samp = 1e6 / args.rate

    total = 0
    in_burst = False
    burst_start = 0  # absolute sample index
    hang = 0
    HANG_MAX = max(1, int(2.0 / us_per_samp))  # 2 us dropout tolerance
    last_burst_end = None
    last_burst_t = None
    bursts = []  # (abs_start, dur_us, gap_prev_us)

    n_target = int(args.duration * args.rate)
    while total < n_target:
        n = rx.recv(buf, md, timeout=2.0)
        if n == 0:
            continue
        if md.error_code not in (uhd.types.RXMetadataErrorCode.none,
                                 uhd.types.RXMetadataErrorCode.overflow):
            sys.stderr.write(f"bf_ndpa_probe: rx error {md.error_code}\n")
        x = buf[:n]
        p = (x.real * x.real + x.imag * x.imag)
        floor = float(np.median(p)) + 1e-12
        thr = floor * (10.0 ** (args.thresh_db / 10.0))
        above = p > thr
        for i in range(n):
            if above[i]:
                if not in_burst:
                    in_burst = True
                    burst_start = total + i
                hang = 0
            elif in_burst:
                hang += 1
                if hang > HANG_MAX:
                    end = total + i - hang
                    dur_us = (end - burst_start) * us_per_samp
                    gap_us = ((burst_start - last_burst_end) * us_per_samp
                              if last_burst_end is not None else -1.0)
                    if 10.0 <= dur_us <= 400.0:
                        t = burst_start / args.rate
                        bursts.append((burst_start, dur_us, gap_us))
                        print(f"bf-probe burst t={t:.6f} dur_us={dur_us:.1f} "
                              f"gap_prev_us={gap_us:.1f}", flush=True)
                        last_burst_end = end
                    in_burst = False
                    hang = 0
        total += n

    stop = uhd.types.StreamCMD(uhd.types.StreamMode.stop_cont)
    rx.issue_stream_cmd(stop)

    lo, hi = args.pair_gap_us
    pair_gaps = [g for (_, _, g) in bursts if lo <= g <= hi]
    n_pairs = len(pair_gaps)
    n_singles = len(bursts) - 2 * n_pairs
    durs = [d for (_, d, _) in bursts]
    frac = (2.0 * n_pairs / len(bursts)) if bursts else 0.0
    med_gap = float(np.median(pair_gaps)) if pair_gaps else -1.0
    med_dur = float(np.median(durs)) if durs else -1.0
    print(f"bf-probe summary bursts={len(bursts)} pairs={n_pairs} "
          f"singles={n_singles} pair_frac={frac:.3f} "
          f"median_pair_gap_us={med_gap:.1f} median_dur_us={med_dur:.1f}",
          flush=True)

    # Chain histogram: group consecutive bursts whose gap <= hi into chains.
    # len-2 chain = NDPA+NDP; len-3 = NDPA+NDP+CSI-report (beamformee reply).
    chains = {}
    cur = 1
    for (_, _, g) in bursts[1:]:
        if 0 <= g <= hi:
            cur += 1
        else:
            chains[cur] = chains.get(cur, 0) + 1
            cur = 1
    chains[cur] = chains.get(cur, 0) + 1
    hist = " ".join(f"len{k}={v}" for k, v in sorted(chains.items()))
    n3 = sum(v for k, v in chains.items() if k >= 3)
    total_chains = sum(chains.values())
    print(f"bf-probe chains total={total_chains} {hist} "
          f"triple_frac={n3 / total_chains if total_chains else 0.0:.3f}",
          flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
