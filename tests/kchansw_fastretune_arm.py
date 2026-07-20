#!/usr/bin/env python3
"""FastRetune control arm for the channel-switch offload comparison.

Same instruments as the kchansw kernel benches — two host-side rxdemo
oracles with tsfl→host-mono fits — but the DUT runs devourer's own
FastRetune hop loop (txdemo, DEVOURER_HOP_CHANNELS + _FAST), so the
distributions are directly comparable with the kernel rows: dead air per
switch = first frame decoded on the destination minus last frame decoded
on the source.

txdemo hops autonomously (no per-switch request records), so switches are
segmented from the oracle streams themselves: order all fitted frames,
every change of decode channel is one switch. With 150 ms dwells and
~1 ms retunes the alternation edges are unambiguous.

Usage (root, T3U free of kernel drivers):
  sudo tests/.venv/bin/python tests/kchansw_fastretune_arm.py \
      --out /tmp/devourer-kchansw-exp2-fastretune \
      --channels 36,40 --rounds 550 --dwell-frames 300
"""

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kchansw_bench as kb          # noqa: E402  (Child, spawn_oracle, log)
import kchansw_analyze as ka        # noqa: E402  (frames, fits)
import regress                      # noqa: E402

import numpy as np                  # noqa: E402


def run_capture(a):
    out = a.out
    os.makedirs(out, exist_ok=True)
    ch_a, ch_b = (int(c) for c in a.channels.split(","))
    dut = kb.find_dut(a.dut_pid)
    oa = kb.spawn_oracle("a", kb.find_dut(a.oracle_a_pid), ch_a, "canon", out)
    ob = kb.spawn_oracle("b", kb.find_dut(a.oracle_b_pid), ch_b, "canon", out)
    regress.detach_from_host_kernel(dut)
    env = dict(os.environ)
    env.update({
        "DEVOURER_VID": f"0x{dut.vid}", "DEVOURER_PID": f"0x{dut.pid}",
        "DEVOURER_EVENTS": "stdout", "DEVOURER_LOG_LEVEL": "warn",
        "DEVOURER_CHANNEL": str(ch_a),
        "DEVOURER_HOP_CHANNELS": a.channels,
        "DEVOURER_HOP_DWELL_FRAMES": str(a.dwell_frames),
        "DEVOURER_HOP_ROUNDS": str(a.rounds),
        "DEVOURER_HOP_FAST": "1",
        "DEVOURER_HOP_PROF": "1",
        "DEVOURER_TX_GAP_US": str(a.tx_gap_us),
    })
    if a.hop_bw:
        env["DEVOURER_HOP_BW"] = a.hop_bw
    tx = kb.Child("txdemo", [str(kb.REPO / "build" / "txdemo")],
                  os.path.join(out, "txdemo.jsonl"), env=env, stamp=True,
                  stderr_path=os.path.join(out, "txdemo.stderr"))
    kb.log(f"txdemo hopping {a.channels} ×{a.rounds} rounds, "
           f"dwell {a.dwell_frames} frames @ {1e6 / a.tx_gap_us:.0f} fps")
    budget = a.rounds * 2 * (a.dwell_frames * a.tx_gap_us / 1e6 + 0.05) + 60
    end = time.monotonic() + budget
    while time.monotonic() < end and tx.alive():
        time.sleep(2)
    alive = tx.alive()
    tx.stop()
    oa.stop()
    ob.stop()
    with open(os.path.join(out, "meta.json"), "w") as f:
        json.dump({"cfg": f"fastretune-{ch_a}-{ch_b}", "from": ch_a,
                   "to": ch_b, "rounds": a.rounds,
                   "dwell_frames": a.dwell_frames,
                   "tx_gap_us": a.tx_gap_us, "hop_bw": a.hop_bw,
                   "tx_overran": alive}, f, indent=1)
    kb.log(f"capture done (txdemo overran budget: {alive})")


def analyze(a):
    out = a.out
    ch_a, ch_b = (str(c) for c in a.channels.split(","))
    streams = {}
    for role, ch in (("a", ch_a), ("b", ch_b)):
        frames = ka.unwrap_tsfl(
            ka.oracle_frames(os.path.join(out, f"oracle_{role}.jsonl")))
        fit = ka.fit_tsfl(frames)
        if not fit.get("ok"):
            print(f"oracle_{role}: FIT BAD {fit}")
            return 1
        for fr in frames:
            fr["t"] = ka.fitted_ns(fit, fr)
        frames.sort(key=lambda fr: fr["t"])
        streams[ch] = frames
        print(f"oracle_{role}(ch{ch}): n={len(frames)} "
              f"σ={fit['residual_std_ns'] / 1e3:.0f}µs "
              f"slope_err={fit['slope_err_ppm']:.0f}ppm")
    merged = [(fr["t"], ch) for ch, frs in streams.items() for fr in frs]
    merged.sort()
    # Collapse to dwell runs; drop micro-runs (isolated cross-channel
    # decodes would otherwise split one dwell in two). Dead air = gap
    # between consecutive surviving runs.
    dead = []
    edges = []
    cur = None
    for t, ch in merged:
        if ch != cur:
            edges.append([ch, t, t])
            cur = ch
        else:
            edges[-1][2] = t
    # Filter out micro-runs (isolated cross-channel decodes).
    min_run_ns = a.dwell_ms_floor * 1e6
    edges = [e for e in edges if e[2] - e[1] >= min_run_ns]
    for k in range(1, len(edges)):
        dead.append(edges[k][1] - edges[k - 1][2])
    d = np.array(dead, dtype=float) / 1e6
    d = d[(d >= 0) & (d < 1000)]
    res = {
        "n_switches": int(len(d)),
        "dead_ms": {"median": float(np.median(d)),
                    "p90": float(np.percentile(d, 90)),
                    "p99": float(np.percentile(d, 99)),
                    "max": float(d.max())},
    }
    print(json.dumps(res, indent=2))
    with open(os.path.join(out, "fastretune_summary.json"), "w") as f:
        json.dump(res, f, indent=2)
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("mode", choices=["run", "analyze", "both"])
    ap.add_argument("--out", required=True)
    ap.add_argument("--channels", default="36,40")
    ap.add_argument("--rounds", type=int, default=550)
    ap.add_argument("--dwell-frames", type=int, default=300)
    ap.add_argument("--tx-gap-us", type=int, default=500)
    ap.add_argument("--hop-bw", default="")
    ap.add_argument("--dut-pid", default="2357:012d")
    ap.add_argument("--oracle-a-pid", default="2357:0120")
    ap.add_argument("--oracle-b-pid", default="0bda:c812")
    ap.add_argument("--min-run", type=int, default=10)
    ap.add_argument("--dwell-ms-floor", type=float, default=20.0,
                    help="dwell runs shorter than this are decode blips")
    a = ap.parse_args()
    if a.mode in ("run", "both"):
        if os.geteuid() != 0:
            sys.stderr.write("run mode needs root\n")
            return 2
        regress._install_cleanup_handlers()
        run_capture(a)
        sudo_user = os.environ.get("SUDO_USER")
        if sudo_user:
            import subprocess
            subprocess.run(["chown", "-R", f"{sudo_user}:", a.out],
                           capture_output=True)
    if a.mode in ("analyze", "both"):
        return analyze(a)
    return 0


if __name__ == "__main__":
    sys.exit(main())
