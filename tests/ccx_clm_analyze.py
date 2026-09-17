#!/usr/bin/env python3
"""Reduce tests/ccx_clm_probe.sh logs to a per-arm table and a verdict.

The question is never "is this number high" but "does this number separate the
arms by more than the noise between repetitions of the same arm". So every arm
reports its median across reps AND the spread of its per-rep medians; a
separation smaller than that spread is not a finding.
"""
from __future__ import annotations

import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from devourer_events import iter_events  # noqa: E402

# rx.energy fields, plus the hopset.sense spellings for the TX-session arms.
RX_FIELDS = ["clm", "nhm_env", "fa_ofdm", "cca_ofdm", "igi", "frames"]
TX_FIELDS = ["clm", "nhm_env", "nhm_busy", "fa_ofdm", "cca_ofdm", "igi"]

ARMS = [
    ("quiet", "rx", "nothing transmitting"),
    ("sdr", "rx", "non-802.11 narrowband carrier"),
    ("wifi", "rx", "devourer 802.11 transmitter"),
    ("txsess", "tx", "sensor transmitting, clean"),
    ("txsessjam", "tx", "sensor transmitting, carrier present"),
]


def collect(path: Path, ev_name: str, fields: list[str]) -> dict[str, list[float]]:
    """Every non-null sample of each field in one log."""
    out: dict[str, list[float]] = {f: [] for f in fields}
    try:
        text = path.read_text(errors="replace")
    except OSError:
        return out
    for ev in iter_events(text.splitlines()):
        if ev.get("ev") != ev_name:
            continue
        for f in fields:
            v = ev.get(f)
            if isinstance(v, (int, float)):
                out[f].append(float(v))
    return out


def med(xs: list[float]) -> float | None:
    return statistics.median(xs) if xs else None


def fmt(v: float | None) -> str:
    return "-" if v is None else f"{v:.0f}"


def main() -> int:
    out_dir = Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/devourer-ccx-clm")
    # arm -> field -> list of per-rep medians
    table: dict[str, dict[str, list[float]]] = {}
    nreps: dict[str, int] = {}

    for arm, kind, _desc in ARMS:
        ev_name = "rx.energy" if kind == "rx" else "hopset.sense"
        fields = RX_FIELDS if kind == "rx" else TX_FIELDS
        per_rep: dict[str, list[float]] = {f: [] for f in fields}
        reps = 0
        for log in sorted(out_dir.glob(f"{arm}_*.log")):
            samples = collect(log, ev_name, fields)
            if not any(samples.values()):
                continue
            reps += 1
            for f in fields:
                m = med(samples[f])
                if m is not None:
                    per_rep[f].append(m)
        table[arm] = per_rep
        nreps[arm] = reps

    if not any(nreps.values()):
        print(f"no usable logs under {out_dir}")
        return 1

    print("Per-arm median across reps (spread = range of the per-rep medians).")
    print("A separation smaller than the spread is not a finding.\n")
    for arm, kind, desc in ARMS:
        per_rep = table[arm]
        if not nreps[arm]:
            print(f"{arm:<12} -- no data")
            continue
        print(f"{arm:<12} n={nreps[arm]} rep(s)   {desc}")
        for f, vals in per_rep.items():
            if not vals:
                print(f"    {f:<10} -")
                continue
            spread = max(vals) - min(vals)
            print(f"    {f:<10} {fmt(med(vals)):>6}"
                  f"   spread {spread:.0f}   reps {[round(v) for v in vals]}")
        print()

    def arm_med(arm: str, field: str) -> float | None:
        return med(table.get(arm, {}).get(field, []))

    def arm_spread(arm: str, field: str) -> float:
        vals = table.get(arm, {}).get(field, [])
        return (max(vals) - min(vals)) if len(vals) > 1 else 0.0

    print("== verdict ==")

    # 1. Does nhm_env avoid railing on a quiet channel? That is the defect that
    #    kept the raw histogram out of the scoring engine.
    q_env = arm_med("quiet", "nhm_env")
    if q_env is None:
        print("  nhm_env  : no quiet-arm data")
    elif q_env >= 95:
        print(f"  nhm_env  : RAILS on a quiet channel ({q_env:.0f}%) -- no better"
              " than the naive busy%, not usable for scoring")
    else:
        print(f"  nhm_env  : quiet floor {q_env:.0f}% (does not rail)")

    # 2. THE cell. A non-802.11 carrier must move a sensor that decodes no
    #    frames. Judge each candidate against its own quiet-arm spread.
    for field in ("nhm_env", "clm", "fa_ofdm", "cca_ofdm"):
        q, s = arm_med("quiet", field), arm_med("sdr", field)
        if q is None or s is None:
            continue
        noise = max(arm_spread("quiet", field), arm_spread("sdr", field), 1.0)
        delta = s - q
        tag = "SEPARATES" if abs(delta) > noise else "within noise"
        print(f"  sdr arm  : {field:<9} quiet {q:>6.0f} -> sdr {s:>6.0f}"
              f"  (delta {delta:+.0f}, noise {noise:.0f})  {tag}")

    f_sdr = arm_med("sdr", "frames")
    f_quiet = arm_med("quiet", "frames")
    if f_sdr is not None:
        note = ("a frame-counting survey sees this channel as free"
                if f_sdr < 5 else
                "ambient 802.11 present -- the 'sniffer blind' case is NOT"
                " cleanly tested on this channel")
        print(f"  sdr arm  : decoded frames {f_sdr:.0f}"
              f" (quiet arm {fmt(f_quiet)}) -- {note}")

    # 3. CLM must rise on real 802.11 too, or it is not measuring airtime.
    q_clm, w_clm = arm_med("quiet", "clm"), arm_med("wifi", "clm")
    if q_clm is not None and w_clm is not None:
        noise = max(arm_spread("quiet", "clm"), arm_spread("wifi", "clm"), 1.0)
        tag = "SEPARATES" if (w_clm - q_clm) > noise else "within noise"
        print(f"  wifi arm : clm       quiet {q_clm:>6.0f} -> wifi {w_clm:>6.0f}"
              f"  (delta {w_clm - q_clm:+.0f}, noise {noise:.0f})  {tag}")

    # 4. The TX-session question: are the counters alive there at all?
    for field in ("clm", "fa_ofdm", "cca_ofdm", "igi"):
        vals = table.get("txsess", {}).get(field, []) + \
               table.get("txsessjam", {}).get(field, [])
        if not vals:
            print(f"  tx sess  : {field:<9} never emitted")
            continue
        if len(set(round(v) for v in vals)) == 1 and round(vals[0]) == 0:
            print(f"  tx sess  : {field:<9} pinned at 0 -- INERT in a TX session")
        else:
            t, tj = arm_med("txsess", field), arm_med("txsessjam", field)
            print(f"  tx sess  : {field:<9} clean {fmt(t):>6} -> carrier {fmt(tj):>6}"
                  "  ALIVE")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
