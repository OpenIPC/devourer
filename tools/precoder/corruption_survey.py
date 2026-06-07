#!/usr/bin/env python3
"""Corruption-pattern survey for FEC design.

Reads `<devourer-corrupt-any>` lines (emitted by `WiFiDriverDemo` with
`DEVOURER_RX_DUMP_ALL=1` + `DEVOURER_RX_KEEP_CORRUPTED=1`) and aggregates
descriptive statistics about the corruption rate, frame-size distribution,
and link-quality distribution — the empirical inputs for choosing the
right stream-layer FEC scheme.

Workflow:

    sudo DEVOURER_PID=0x0120 DEVOURER_VID=0x2357 DEVOURER_CHANNEL=6 \
         DEVOURER_RX_DUMP_ALL=1 DEVOURER_RX_KEEP_CORRUPTED=1 \
         ./build/WiFiDriverDemo | \
        python3 tools/precoder/corruption_survey.py [--duration 300]

The tool reads stdin until EOF or `--duration` seconds have elapsed, then
prints a report with:

* Headline counts (total / chip-clean / chip-corrupt; corruption rate %)
* Frame-size distribution (clean and corrupt separately)
* Mean / median phy metrics (RSSI / EVM / SNR) for clean and corrupt
  populations
* BER-by-SNR-bucket equivalent in this context (per-bucket corruption
  rate, since we don't have ground truth for arbitrary frames)
* Burst-vs-isolated stats: per-second time series of corruption events
  to see whether errors cluster (suggests interference) or spread evenly
  (suggests sustained marginal SNR)

What this tells the FEC designer:

* If `chip-corrupt` is dominated by a single bucket (e.g. all-or-nothing
  near sync threshold), inter-frame FEC (Reed-Solomon across N frames
  with K parity frames) is the right strategy — recovers from K dropped
  frames, light overhead.
* If corruption rate scales smoothly with SNR and frames have correctable
  partial corruption (per-frame BER < 1%), intra-frame FEC (interleaving
  + light parity) is viable on top.
* If errors burst (rate >5×average in 1-second windows), the inter-frame
  FEC block size N has to be large enough that a burst doesn't exceed K
  losses inside one block.
"""

from __future__ import annotations

import argparse
import collections
import re
import select
import statistics
import sys
import time
from typing import Optional

_CORRUPT_ANY_RE = re.compile(
    r"<devourer-corrupt-any>len=(?P<len>\d+)\s+"
    r"crc_err=(?P<crc_err>\d+)\s+icv_err=(?P<icv_err>\d+)\s+"
    r"rate=(?P<rate>\d+)\s+"
    r"rssi=(?P<rssi_a>-?\d+),(?P<rssi_b>-?\d+)\s+"
    r"evm=(?P<evm_a>-?\d+),(?P<evm_b>-?\d+)\s+"
    r"snr=(?P<snr_a>-?\d+),(?P<snr_b>-?\d+)"
)


def _len_bucket(n: int) -> str:
    """802.11-friendly length buckets."""
    if n < 64:
        return "  <64 B  (ack/cts/control)"
    if n < 256:
        return " 64-255  (mgmt/short data)"
    if n < 768:
        return "256-767  (data/probe-resp)"
    if n < 1500:
        return "768-1499 (data/aggreg)"
    return ">=1500   (data/jumbo/aggreg)"


def _snr_bucket(s: int) -> str:
    base = (s // 5) * 5
    return f"{base:>3d}-{base + 5} dB"


def _effective_snr(snr_a: int, snr_b: int) -> int:
    """Same convention as corruption_analysis.py: max picks the active path
    on 1T1R (B reads 0) and the stronger path on 2T2R single-stream."""
    return max(snr_a, snr_b)


def _print_dist(title: str, counter: collections.Counter, total: int) -> None:
    print(f"\n{title}:")
    print(f"  {'bucket':<30s} {'count':>8s} {'%':>6s}")
    for bucket, count in sorted(counter.items()):
        pct = 100.0 * count / max(1, total)
        print(f"  {bucket:<30s} {count:>8d} {pct:>5.1f}%")


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--duration", type=float, default=0.0,
                    help="seconds to collect before reporting (default 0 = EOF)")
    ap.add_argument("--bucket-secs", type=float, default=1.0,
                    help="time-bucket size for the burst/isolation analysis")
    args = ap.parse_args(argv)

    total = 0
    n_clean = 0
    n_corrupt = 0
    by_rate = collections.Counter()
    by_rate_corrupt = collections.Counter()
    by_len_clean = collections.Counter()
    by_len_corrupt = collections.Counter()
    rssi_clean: list[int] = []
    rssi_corrupt: list[int] = []
    snr_clean: list[int] = []
    snr_corrupt: list[int] = []
    evm_clean: list[int] = []
    evm_corrupt: list[int] = []
    # Per-SNR-bucket: total frames + corrupted frames → corruption rate.
    bucket_total: collections.Counter = collections.Counter()
    bucket_corrupt: collections.Counter = collections.Counter()
    # Per-time-bucket corruption count for burst analysis.
    time_corrupt: collections.Counter = collections.Counter()
    time_total: collections.Counter = collections.Counter()

    start = time.monotonic()
    deadline = start + args.duration if args.duration > 0 else None
    no_snr = 0

    for line in sys.stdin:
        m = _CORRUPT_ANY_RE.search(line)
        if not m:
            continue
        now = time.monotonic()
        if deadline is not None and now > deadline:
            break
        total += 1
        crc_err = int(m.group("crc_err"))
        icv_err = int(m.group("icv_err"))
        corrupted = bool(crc_err or icv_err)
        plen = int(m.group("len"))
        rate = int(m.group("rate"))
        rssi_a = int(m.group("rssi_a")); rssi_b = int(m.group("rssi_b"))
        evm_a = int(m.group("evm_a"));   evm_b = int(m.group("evm_b"))
        snr_a = int(m.group("snr_a"));   snr_b = int(m.group("snr_b"))
        # The chip only populates evm/snr for OFDM data frames; for CCK ACKs
        # and short mgmt frames both paths read 0. Treat (0,0) as "no
        # measurement" rather than "0 dB" so we don't artificially fill the
        # 0-5 dB SNR bucket with frames that have no measurement at all.
        snr_present = not (snr_a == 0 and snr_b == 0)
        evm_present = not (evm_a == 0 and evm_b == 0)
        eff_snr = _effective_snr(snr_a, snr_b) if snr_present else None
        eff_rssi = max(rssi_a, rssi_b)
        eff_evm = min(evm_a, evm_b) if evm_present else None

        by_rate[rate] += 1
        if corrupted:
            by_rate_corrupt[rate] += 1
            n_corrupt += 1
            by_len_corrupt[_len_bucket(plen)] += 1
            rssi_corrupt.append(eff_rssi)
            if eff_snr is not None: snr_corrupt.append(eff_snr)
            if eff_evm is not None: evm_corrupt.append(eff_evm)
        else:
            n_clean += 1
            by_len_clean[_len_bucket(plen)] += 1
            rssi_clean.append(eff_rssi)
            if eff_snr is not None: snr_clean.append(eff_snr)
            if eff_evm is not None: evm_clean.append(eff_evm)

        if eff_snr is None:
            no_snr += 1
        else:
            bucket = _snr_bucket(eff_snr)
            bucket_total[bucket] += 1
            if corrupted:
                bucket_corrupt[bucket] += 1
        # The temporal bucketing is only meaningful for live captures; if
        # we're reading a pre-captured file all lines arrive at ~the same
        # instant. The report block guards on having at least 2 buckets.
        tb = int((now - start) / args.bucket_secs)
        time_total[tb] += 1
        if corrupted:
            time_corrupt[tb] += 1

    elapsed = max(1e-9, time.monotonic() - start)
    if total == 0:
        sys.stderr.write("survey: no <devourer-corrupt-any> lines parsed; "
                         "is DEVOURER_RX_DUMP_ALL set?\n")
        return 1

    realtime = elapsed > 1.0  # only meaningful when we ran live
    print(f"=== corruption survey ({total} frames"
          + (f", {elapsed:.1f}s live" if realtime else ", file/pipe")
          + ") ===")
    print(f"chip-clean       : {n_clean:6d} ({100.0 * n_clean / total:5.1f}%)")
    print(f"chip-corrupt     : {n_corrupt:6d} ({100.0 * n_corrupt / total:5.1f}%)")
    print(f"corruption rate  : {100.0 * n_corrupt / total:.2f}%")
    if realtime:
        print(f"frame rate       : {total / elapsed:.1f} fps")
    print(f"no-phy-measurement: {no_snr:6d}  "
          f"(CCK/short frames where chip didn't populate snr/evm)")

    # Corruption rate broken down by PHY rate index. Clean traffic is often
    # dominated by CCK ACKs and beacons which always decode well, so the
    # headline corruption rate understates what the FEC layer faces at the
    # OFDM rate the stream link actually uses (index 4 = legacy 6M).
    rate_names = {0: "1M CCK", 1: "2M CCK", 2: "5.5M CCK", 3: "11M CCK",
                  4: "6M OFDM", 5: "9M OFDM", 6: "12M OFDM", 7: "18M OFDM",
                  8: "24M OFDM", 9: "36M OFDM", 10: "48M OFDM", 11: "54M OFDM"}
    print(f"\nCorruption rate by DESC_RATE:")
    print(f"  {'idx':>4s} {'name':<12s} {'count':>8s} {'%':>6s}   "
          f"{'corrupt':>8s} {'rate':>7s}")
    for r in sorted(by_rate):
        name = rate_names.get(r, f"MCS{r - 12}" if r >= 12 else f"rate{r}")
        pct = 100.0 * by_rate[r] / total
        crpt = by_rate_corrupt.get(r, 0)
        cr_pct = 100.0 * crpt / max(1, by_rate[r])
        print(f"  0x{r:02x} {name:<12s} {by_rate[r]:>8d} {pct:>5.1f}%   "
              f"{crpt:>8d} {cr_pct:>5.1f}%")

    _print_dist("Frame-size distribution (chip-clean)", by_len_clean, n_clean)
    if n_corrupt:
        _print_dist("Frame-size distribution (chip-corrupt)",
                    by_len_corrupt, n_corrupt)

    def _stats(name: str, xs_clean: list[int], xs_corrupt: list[int]) -> None:
        def _stat(xs: list[int]) -> str:
            if not xs:
                return "n=0"
            xs = sorted(xs)
            n = len(xs)
            return (f"n={n} min={xs[0]} p25={xs[n // 4]} "
                    f"med={xs[n // 2]} p75={xs[(3 * n) // 4]} max={xs[-1]} "
                    f"mean={statistics.fmean(xs):+.1f}")
        print(f"\n{name}:")
        print(f"  chip-clean    : {_stat(xs_clean)}")
        print(f"  chip-corrupt  : {_stat(xs_corrupt)}")

    _stats("RSSI (stronger path)", rssi_clean, rssi_corrupt)
    _stats("EVM  (weaker path; more negative is better)",
           evm_clean, evm_corrupt)
    _stats("SNR  (stronger path, dB)", snr_clean, snr_corrupt)

    print(f"\nCorruption rate by SNR bucket (stronger path, 5-dB buckets):")
    print(f"  bucket       frames   corrupt   rate")
    for bucket in sorted(bucket_total):
        n = bucket_total[bucket]
        c = bucket_corrupt[bucket]
        print(f"  {bucket:>11s}   {n:6d}   {c:7d}   {100.0 * c / max(1, n):5.1f}%")

    if realtime and time_total:
        print(f"\nTemporal distribution ({args.bucket_secs:.1f}-s buckets, "
              f"top 10 buckets by corruption count):")
        print(f"  t-start    total   corrupt   rate")
        top = sorted(time_corrupt.items(), key=lambda kv: -kv[1])[:10]
        for tb, c in top:
            t = tb * args.bucket_secs
            n = time_total[tb]
            print(f"  {t:6.1f}s   {n:6d}   {c:7d}   "
                  f"{100.0 * c / max(1, n):5.1f}%")
        # Burst-vs-baseline heuristic
        rates = [time_corrupt[tb] / max(1, time_total[tb])
                 for tb in time_total]
        if rates:
            med_rate = statistics.median(rates)
            max_rate = max(rates)
            print(f"  median 1-s corruption rate: {100 * med_rate:.1f}%")
            print(f"  peak   1-s corruption rate: {100 * max_rate:.1f}%")
            if med_rate > 0 and max_rate > 5 * med_rate:
                print(f"  → BURSTY: peak is >5× median, sustained interference "
                      f"or coverage edge events. FEC block size should be "
                      f"large enough to span typical bursts.")
            elif med_rate < 0.01:
                print(f"  → CLEAN: well-conditioned link; FEC overhead can be "
                      f"low / opportunistic.")
            else:
                print(f"  → SUSTAINED: corruption is evenly distributed; light "
                      f"per-frame FEC + cross-frame parity should suffice.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
