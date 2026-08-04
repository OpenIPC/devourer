#!/usr/bin/env python3
"""tx.report coverage attribution — where do the missing reports go?

Per recorded arq_e2e run: coverage (reports/submitted), the achieved report
rate, and the RUN-LENGTH histogram of unreported frames between consecutive
reports (tag-unwrap deltas; the unwrap is trustworthy whenever the run's
ledger join matched — see arq_e2e_analyze).

How to read the gap shape: interleaved 1–2-frame gaps are per-report
emission-time drops at the fw/C2H-mailbox (a rate limiter); large contiguous
gaps would be transport-batch loss (a dropped C2H aggregate loses dozens of
reports at once). Bench-measured on the 8812CU (Jaguar3): the fw's CCX
emission saturates at ~1.3k reports/s — above that, excess reports are
dropped individually and interleaved (99.96% of unreported frames in gaps
<= 2), and the CCX MISSED_RPT_NUM field is stuffed with a constant (4), so
the fw offers no drop accounting of its own (layout verified identical
across the 8822B/8822C/8822E vendor headers; the constant is fw behaviour,
not a parse bug). Consequence for accounting layers: coverage is
~min(1, ceiling/fps) — either keep the demanded report rate under the
ceiling (sample SPE_RPT 1-in-N) or treat report-less frames as "unknown".

  python3 tests/txrpt_coverage_attrib.py /tmp/arq-e2e/<run> [...]
"""
import json
import sys
from collections import Counter


def analyze(rundir):
    n = 0
    submitted = 0
    prev_tag = None
    gaps = Counter()
    missed_vals = Counter()
    t_first = t_last = None
    with open(f"{rundir}/drone.jsonl", errors="replace") as f:
        for line in f:
            if line.startswith('{"ev":"tx.stats"'):
                try:
                    submitted = max(submitted,
                                    json.loads(line).get("submitted", 0))
                except Exception:
                    pass
                continue
            if not line.startswith('{"ev":"tx.report"'):
                continue
            try:
                ev = json.loads(line)
            except Exception:
                continue
            tag = ev.get("tag")
            if tag is None:
                continue
            n += 1
            missed_vals[int(ev.get("missed", 0))] += 1
            t = ev.get("t")
            if t is not None:
                t_first = t if t_first is None else t_first
                t_last = t
            if prev_tag is not None:
                d = (tag - prev_tag) % 256
                if d > 1:
                    gaps[d - 1] += 1
            prev_tag = tag
    if n == 0:
        raise SystemExit(f"{rundir}: no tagged tx.report events "
                         f"(J1-format reports carry no tag)")
    name = rundir.rstrip("/").split("/")[-1]
    cov = 100.0 * n / max(1, submitted)
    dur_s = (t_last - t_first) / 1000.0 if (t_first is not None and
                                            t_last and t_last > t_first) else 0
    rate = n / dur_s if dur_s else 0
    unrep = sum(k * v for k, v in gaps.items())
    small = sum(k * v for k, v in gaps.items() if k <= 2)
    top = ", ".join(f"{k}x{v}" for k, v in sorted(gaps.items())[:8])
    print(f"\n== {name}: submitted={submitted} reports={n} "
          f"coverage={cov:.1f}%"
          + (f" achieved={rate:.0f} rpt/s" if rate else ""))
    print(f"   unreported={unrep} gap-hist [{top}"
          f"{', ...' if len(gaps) > 8 else ''}] "
          f"max={max(gaps) if gaps else 0} "
          f"in-gaps<=2: {100.0 * small / max(1, unrep):.1f}%")
    if len(missed_vals) == 1:
        (mv, _), = missed_vals.items()
        print(f"   missed field: constant {mv} on every report "
              f"(fw drop-accounting unavailable)")
    else:
        print(f"   missed field values: {dict(sorted(missed_vals.items()))}")


for rd in sys.argv[1:]:
    analyze(rd)
