#!/usr/bin/env python3
"""Post-ARQ residual gap analysis for FEC dimensioning.

For each recorded arq_e2e run directory: reconstruct the undelivered set
(report ok=0, plus unreported-and-undelivered — conservative, slightly
overcounts), then measure the RUN-LENGTH distribution of consecutive
undelivered frame indices. That is the quantity wfb-style block FEC cares
about: a (K,N) block recovers up to N-K losses per block window, so the
residual gap-length percentiles map directly onto the N-K needed.

Caveats the numbers carry: the mapping printed is SINGLE-gap coverage per
block window (two gaps landing in one window need their sum); the ledgers are
near-field bench data, so the residual is collision/stall structure, not
range-fade; and the undelivered set includes frames with no report verdict at
all (report coverage is load-dependent), which errs toward larger residuals —
the safe direction for dimensioning.

  python3 tests/arq_fec_dimension.py /tmp/arq-e2e/<run> [<run>...]
"""
import argparse
import json
from collections import Counter

TAIL_GUARD = 512  # mirror arq_e2e_analyze: stream-end truncation window


def undelivered(rundir):
    dut = set()
    with open(f"{rundir}/dut.jsonl", errors="replace") as f:
        for line in f:
            if line.startswith('{"ev":"rx.seq"'):
                try:
                    dut.add(json.loads(line)["pctr"])
                except Exception:
                    pass
    rep = {}
    prev = None
    r = 0
    with open(f"{rundir}/drone.jsonl", errors="replace") as f:
        for line in f:
            if not line.startswith('{"ev":"tx.report"'):
                continue
            try:
                ev = json.loads(line)
            except Exception:
                continue
            t = ev.get("tag")
            if t is None:
                continue
            if prev is not None:
                r += (t - prev) % 256
            prev = t
            rep[r] = bool(ev.get("ok"))
    if not rep or not dut:
        raise SystemExit(
            f"{rundir}: {'no tagged tx.report events' if not rep else ''}"
            f"{' and ' if not rep and not dut else ''}"
            f"{'no rx.seq ledger' if not dut else ''} — an empty ledger would "
            f"count every frame as undelivered (J1-format reports carry no "
            f"tag; this tool needs a halmac TX side)")
    hi = max(max(dut, default=0), r)
    lo_cut = min(dut) if dut else 0  # drone frames before the DUT RX was up
    hi_cut = hi - TAIL_GUARD
    miss = [k for k in range(lo_cut, hi_cut)
            if k not in dut and rep.get(k) is not True]
    gaps = []
    run = 0
    prev_k = None
    for k in miss:
        if prev_k is not None and k == prev_k + 1:
            run += 1
        else:
            if run:
                gaps.append(run)
            run = 1
        prev_k = k
    if run:
        gaps.append(run)
    return gaps, len(miss), max(1, hi_cut - lo_cut)


def pct(sorted_g, p):
    """Nearest-rank percentile: ceil(p*n)-1, 0-based. int(p*n) would be
    biased one rank upward (P50 of 4 elements landing on the 3rd)."""
    if not sorted_g:
        return 0
    import math
    return sorted_g[min(len(sorted_g) - 1,
                        max(0, math.ceil(p * len(sorted_g)) - 1))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--k", type=int, nargs="*", default=[8, 12],
                    help="FEC block K values to map the residual onto")
    a = ap.parse_args()
    for rundir in a.runs:
        gaps, n_miss, span = undelivered(rundir)
        g = sorted(gaps)
        hist = Counter(gaps)
        top = ", ".join(f"{k}x{v}" for k, v in sorted(hist.items())[:8])
        name = rundir.rstrip("/").split("/")[-1]
        print(f"\n== {name}: undelivered {n_miss}/{span} "
              f"({100.0 * n_miss / span:.2f}%), {len(g)} gaps")
        print(f"   lengths: [{top}{', ...' if len(hist) > 8 else ''}]")
        print(f"   P50={pct(g, .5)} P99={pct(g, .99)} "
              f"P99.9={pct(g, .999)} max={g[-1] if g else 0}")
        need = pct(g, .999)
        for K in a.k:
            if need:
                print(f"   K={K}: N-K >= {need} to cover the P99.9 single gap "
                      f"-> N={K + need} (rate {K / (K + need):.2f})")
            else:
                print(f"   K={K}: residual ~gap-free")


if __name__ == "__main__":
    main()
