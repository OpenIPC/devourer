#!/usr/bin/env python3
"""Scout-impact soak verdict: is primary video delivery statistically
unchanged with the scout running?

The primary receiver (rxdemo) THROTTLES its rx.txhit event to every ~100th
canonical frame, but each event carries the cumulative `hits` counter (the
RX's own canonical-frame count) and the frame's 12-bit `seq`. So delivery is
measured from the `hits` progression, and loss from the (Δseq − Δhits) gap
between consecutive events — never by counting events (which would read the
throttle as 99% loss). Duration comes from the periodic rx.energy windows
(~500 ms cadence), so arms of different lengths compare on RATE, not totals.

Usage:
  python3 tests/chanmig_soak_analyze.py \
      --scout-arm primary-a.jsonl --control-arm primary-b.jsonl \
      [--scout-log scout-a.jsonl] [--energy-ms 500]
"""
import argparse
import json
import sys


def load(path, names):
    out = []
    try:
        with open(path, errors="replace") as f:
            for line in f:
                if '"ev":"' not in line:
                    continue
                try:
                    ev = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if ev.get("ev") in names:
                    out.append(ev)
    except OSError as e:
        print(f"cannot read {path}: {e}")
        sys.exit(2)
    return out


def arm_stats(name, path, energy_ms):
    evs = load(path, ("rx.txhit", "rx.energy"))
    txhits = [e for e in evs if e.get("ev") == "rx.txhit"]
    n_energy = sum(1 for e in evs if e.get("ev") == "rx.energy")
    duration_s = max(n_energy * energy_ms / 1000.0, 1.0)

    delivered = txhits[-1]["hits"] if txhits else 0
    # loss from consecutive (hits, seq) deltas, 12-bit seq wrap-aware
    exp = dlv = 0
    prev = None
    for e in txhits:
        h, s = e.get("hits"), e.get("seq")
        if h is None or s is None:
            continue
        if prev is not None:
            dh = h - prev[0]
            ds = (s - prev[1]) & 0xFFF
            if dh > 0 and 0 < ds < 4096 and ds >= dh * 0.5:
                dlv += dh
                exp += ds
        prev = (h, s)
    loss = 1.0 - dlv / exp if exp else 0.0
    rate = delivered / duration_s
    # near-field / verdict context
    verdicts = {}
    for e in load(path, ("link.health",)):
        verdicts[e.get("verdict", "?")] = verdicts.get(e.get("verdict", "?"), 0) + 1
    top = max(verdicts, key=verdicts.get) if verdicts else "-"
    print(f"[{name}] delivered={delivered} loss={100*loss:.2f}% "
          f"dur={duration_s:.0f}s rate={rate:.1f}/s verdict~{top}")
    return {"delivered": delivered, "loss": loss, "rate": rate,
            "duration": duration_s}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scout-arm", required=True)
    ap.add_argument("--control-arm", required=True)
    ap.add_argument("--scout-log")
    ap.add_argument("--energy-ms", type=int, default=500)
    args = ap.parse_args()

    a = arm_stats("scout-on ", args.scout_arm, args.energy_ms)
    b = arm_stats("control  ", args.control_arm, args.energy_ms)

    fails = 0

    def check(ok, msg):
        nonlocal fails
        print(("  ok  " if ok else "  FAIL") + " " + msg)
        if not ok:
            fails += 1

    check(a["delivered"] >= 5000 and b["delivered"] >= 5000,
          "both arms carried enough frames to judge")
    rel = (a["rate"] - b["rate"]) / b["rate"] if b["rate"] else 0.0
    print(f"  rate delta (scout vs control) = {100*rel:+.1f}%  "
          f"loss {100*a['loss']:.2f}% vs {100*b['loss']:.2f}%")
    # Conservative in both directions: a real degradation must fail; run-to-run
    # noise must not. The delivery rate is the dominant signal (a scout that
    # steals USB/RF bandwidth drops the primary's rate).
    check(rel > -0.10, "scout-arm delivery rate within 10% of control")
    check(a["loss"] - b["loss"] < 0.03,
          "scout-arm loss within 3 percentage points of control")

    if args.scout_log:
        dwells = wedged = 0
        chans = set()
        try:
            with open(args.scout_log, errors="replace") as f:
                for line in f:
                    if '"ev":"survey.dwell"' in line:
                        dwells += 1
                        try:
                            chans.add(json.loads(line)["chan"])
                        except (json.JSONDecodeError, KeyError):
                            pass
                    elif '"ev":"scout.health"' in line and '"wedged"' in line:
                        wedged += 1
        except OSError:
            pass
        print(f"[scout] {dwells} dwells over {sorted(chans)}")
        check(dwells > 0, "scout actually scanned during arm A")
        check(wedged == 0, "scout never wedged")

    print("PASS: primary delivery statistically unchanged"
          if fails == 0 else f"{fails} FAILURES")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
