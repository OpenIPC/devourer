#!/usr/bin/env python3
"""Thermal-vs-TX-gain experiment orchestrator.

Drives one continuous WiFiDriverTxDemo TX session on an RTL88xxAU adapter while
ramping the absolute TXAGC index up over time (DEVOURER_TX_PWR_* knobs), and
captures three interleaved streams:

  * `<devourer-txpwr>index=N`            — the gain step the demo just applied
  * `<devourer-thermal>raw=.. delta=..`  — the chip's own thermal meter
  * `sdr-power dbfs=..`                  — an independent USRP receive-power probe

Every line from both subprocesses is stamped with a host-side monotonic
timestamp (ms since orchestrator start), so the three streams share one clock
with no cross-process alignment needed. Output: a merged CSV plus a printed
per-gain-index summary that answers:

  - does the thermal `delta` climb over time under sustained TX? (degrades?)
  - does `delta` track the TX gain index? (chip-meter response)
  - does the SDR power track the gain index? (is the override real on-air?)

Safety: if the chip reports thermal status=critical the TX session is stopped
immediately to protect the PA.

Run (needs root for USB claim; SDR needs no root):
    sudo python3 thermal_gain_sweep.py --channel 6 --start 8 --stop 40 \
        --step 4 --step-ms 30000
"""
from __future__ import annotations

import argparse
import csv
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

import regress  # reuse DUT discovery + process-hygiene helpers

TXPWR_RE = re.compile(r"<devourer-txpwr>index=(\d+)")
THERMAL_RE = re.compile(
    r"<devourer-thermal>raw=(\d+) baseline=(\d+|none)(?: delta=([+-]?\d+))? status=(\w+)"
)
SDR_RE = re.compile(r"sdr-power dbfs=([+-]?[\d.]+)")

# UHD channel-center frequencies (MHz) for the channels we use.
CH_FREQ_MHZ = {
    1: 2412, 6: 2437, 11: 2462,
    36: 5180, 44: 5220, 100: 5500, 149: 5745, 161: 5805,
}


class Sample:
    __slots__ = ("t_ms", "source", "index", "raw", "baseline", "delta",
                 "status", "dbfs")

    def __init__(self, t_ms, source):
        self.t_ms = t_ms
        self.source = source
        self.index = None
        self.raw = None
        self.baseline = None
        self.delta = None
        self.status = None
        self.dbfs = None


def _devourer_env(dut: regress.Dut, channel: int, ramp: dict) -> dict:
    env = os.environ.copy()
    env["DEVOURER_VID"] = f"0x{dut.vid}"
    env["DEVOURER_PID"] = f"0x{dut.pid}"
    env["DEVOURER_CHANNEL"] = str(channel)
    env["DEVOURER_TX_PWR_START"] = str(ramp["start"])
    env["DEVOURER_TX_PWR_STOP"] = str(ramp["stop"])
    env["DEVOURER_TX_PWR_STEP"] = str(ramp["step"])
    env["DEVOURER_TX_PWR_STEP_MS"] = str(ramp["step_ms"])
    env["DEVOURER_THERMAL_POLL_MS"] = str(ramp["thermal_ms"])
    env["DEVOURER_TX_GAP_US"] = str(ramp["gap_us"])
    if ramp["ht"]:
        # Transmit HT/OFDM instead of the 1M-CCK default — CCK output power is
        # decoupled from the per-rate TXAGC base, so the gain knob only shows
        # up on-air for OFDM/HT/VHT rates.
        env["DEVOURER_TX_HT_MCS"] = "1"
    # Keep a high warn threshold so the demo's own back-off message doesn't
    # spam; we judge from the data, and abort on `critical` ourselves.
    env["DEVOURER_THERMAL_WARN_DELTA"] = "100"
    return env


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--pid", default="8812", help="DUT PID (hex, no 0x)")
    ap.add_argument("--channel", type=int, default=6)
    ap.add_argument("--start", type=int, default=8, help="start TXAGC index")
    ap.add_argument("--stop", type=int, default=40, help="stop TXAGC index")
    ap.add_argument("--step", type=int, default=4)
    ap.add_argument("--step-ms", type=int, default=30000, help="dwell per level")
    ap.add_argument("--thermal-ms", type=int, default=1000)
    ap.add_argument("--gap-us", type=int, default=2000,
                    help="inter-frame gap (us); lower = higher duty/heat "
                         "(default 2000 ~= 500fps)")
    ap.add_argument("--ht", action="store_true",
                    help="transmit HT/OFDM (gain knob only affects on-air power "
                         "for OFDM/HT — CCK is decoupled)")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="total seconds (0 = derive from the ramp + 10s tail)")
    ap.add_argument("--sdr-freq", type=float, default=0.0,
                    help="SDR center freq Hz (0 = derive from --channel)")
    ap.add_argument("--sdr-rate", type=float, default=4e6)
    ap.add_argument("--sdr-gain", type=float, default=40.0)
    ap.add_argument("--no-sdr", action="store_true", help="thermal meter only")
    ap.add_argument("--outdir", default="/tmp/devourer-thermal-sweep")
    args = ap.parse_args()

    devourer_root = Path(__file__).resolve().parent.parent
    txdemo = devourer_root / "build" / "WiFiDriverTxDemo"
    if not txdemo.exists():
        sys.exit(f"{txdemo} not built — run `cmake --build build` first")

    duts = [d for d in regress.discover_duts() if d.pid == args.pid]
    if not duts:
        sys.exit(f"no DUT with PID {args.pid} found (plugged in?)")
    dut = duts[0]
    drv = regress.host_kernel_driver_for_dut(dut)
    if drv:
        sys.exit(f"{dut.chipset} is bound to kernel driver {drv!r}; "
                 f"unbind it first (see tests/regress.py detach helpers)")

    if args.duration > 0:
        total_s = args.duration
    else:
        n_steps = max(1, (args.stop - args.start + args.step - 1) // args.step + 1)
        total_s = n_steps * (args.step_ms / 1000.0) + 10.0

    sdr_freq = args.sdr_freq or CH_FREQ_MHZ.get(args.channel, 0) * 1e6
    if not args.no_sdr and sdr_freq <= 0:
        sys.exit(f"unknown center freq for channel {args.channel}; pass --sdr-freq")

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    csv_path = outdir / f"sweep-{args.pid}-ch{args.channel}-{stamp}.csv"

    regress._install_cleanup_handlers()

    print(f"# DUT: {dut.chipset} ({dut.vidpid}) ch{args.channel}")
    print(f"# ramp: index {args.start}..{args.stop} step {args.step} "
          f"every {args.step_ms}ms  | total ~{total_s:.0f}s")
    print(f"# SDR: {'off' if args.no_sdr else f'{sdr_freq/1e6:.1f}MHz gain {args.sdr_gain}dB'}")
    print(f"# CSV: {csv_path}\n")

    samples: list[Sample] = []
    samples_lock = threading.Lock()
    t0 = time.monotonic()
    abort = threading.Event()

    def now_ms() -> float:
        return (time.monotonic() - t0) * 1000.0

    def reader(proc: subprocess.Popen, source: str):
        assert proc.stdout is not None
        for raw_line in proc.stdout:
            line = raw_line.rstrip("\n")
            t = now_ms()
            s = None
            m = TXPWR_RE.search(line)
            if m:
                s = Sample(t, "txpwr")
                s.index = int(m.group(1))
            if s is None:
                m = THERMAL_RE.search(line)
                if m:
                    s = Sample(t, "thermal")
                    s.raw = int(m.group(1))
                    s.baseline = None if m.group(2) == "none" else int(m.group(2))
                    s.delta = int(m.group(3)) if m.group(3) is not None else None
                    s.status = m.group(4)
                    if s.status == "critical":
                        print(f"\n!! thermal critical at t={t/1000:.1f}s — aborting TX")
                        abort.set()
            if s is None:
                m = SDR_RE.search(line)
                if m:
                    s = Sample(t, "sdr")
                    s.dbfs = float(m.group(1))
            if s is not None:
                with samples_lock:
                    samples.append(s)

    env = _devourer_env(dut, args.channel, {
        "start": args.start, "stop": args.stop, "step": args.step,
        "step_ms": args.step_ms, "thermal_ms": args.thermal_ms,
        "gap_us": args.gap_us, "ht": args.ht,
    })
    tx_log = open(outdir / f"txdemo-{stamp}.log", "w")
    tx_proc = regress._register_local_proc(subprocess.Popen(
        [str(txdemo)], env=env,
        stdout=subprocess.PIPE, stderr=tx_log, text=True,
        preexec_fn=regress._child_preexec,
    ))
    threads = [threading.Thread(target=reader, args=(tx_proc, "tx"), daemon=True)]

    sdr_proc = None
    if not args.no_sdr:
        sdr_log = open(outdir / f"sdr-{stamp}.log", "w")
        sdr_proc = regress._register_local_proc(subprocess.Popen(
            [sys.executable, str(devourer_root / "tests" / "sdr_power_probe.py"),
             "--freq", str(sdr_freq), "--rate", str(args.sdr_rate),
             "--gain", str(args.sdr_gain)],
            stdout=subprocess.PIPE, stderr=sdr_log, text=True,
            preexec_fn=regress._child_preexec,
        ))
        threads.append(threading.Thread(target=reader, args=(sdr_proc, "sdr"),
                                        daemon=True))
    for t in threads:
        t.start()

    deadline = t0 + total_s
    try:
        while time.monotonic() < deadline and not abort.is_set():
            if tx_proc.poll() is not None:
                print("!! txdemo exited early — check txdemo log")
                break
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        for p in (tx_proc, sdr_proc):
            if p is not None:
                regress._terminate(p)
                regress._unregister_local_proc(p)
        time.sleep(0.3)

    with samples_lock:
        rows = sorted(samples, key=lambda s: s.t_ms)
    _write_csv(csv_path, rows)
    _summarize(rows)
    print(f"\nCSV written: {csv_path}")
    return 0


def _write_csv(path: Path, rows: list[Sample]) -> None:
    cur_index = None
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ms", "source", "gain_index", "raw", "baseline",
                    "delta", "status", "dbfs"])
        for s in rows:
            if s.source == "txpwr":
                cur_index = s.index
            w.writerow([f"{s.t_ms:.1f}", s.source,
                        cur_index if cur_index is not None else "",
                        s.raw if s.raw is not None else "",
                        s.baseline if s.baseline is not None else "",
                        s.delta if s.delta is not None else "",
                        s.status or "",
                        f"{s.dbfs:.2f}" if s.dbfs is not None else ""])


def _summarize(rows: list[Sample]) -> None:
    # forward-fill the current gain index onto every thermal/sdr sample
    cur = None
    per_index: dict[int, dict] = {}
    for s in rows:
        if s.source == "txpwr":
            cur = s.index
            per_index.setdefault(cur, {"deltas": [], "dbfs": [], "t0": s.t_ms,
                                       "t1": s.t_ms})
            continue
        if cur is None:
            continue
        b = per_index.setdefault(cur, {"deltas": [], "dbfs": [], "t0": s.t_ms,
                                       "t1": s.t_ms})
        b["t1"] = s.t_ms
        if s.source == "thermal" and s.delta is not None:
            b["deltas"].append((s.t_ms, s.delta, s.raw))
        elif s.source == "sdr" and s.dbfs is not None:
            b["dbfs"].append(s.dbfs)

    if not per_index:
        print("no gain-index transitions captured (did the demo emit "
              "<devourer-txpwr>?)")
        return

    print("\n=== per gain-index summary ===")
    print(f"{'idx':>4} {'dwell_s':>7} {'raw_first':>9} {'raw_last':>8} "
          f"{'d_first':>7} {'d_last':>6} {'d_min':>5} {'d_max':>5} "
          f"{'sdr_dbfs_mean':>13} {'sdr_n':>5}")
    index_means = []
    for idx in sorted(per_index):
        b = per_index[idx]
        ds = b["deltas"]
        dwell = (b["t1"] - b["t0"]) / 1000.0
        if ds:
            d_first, d_last = ds[0][1], ds[-1][1]
            raw_first, raw_last = ds[0][2], ds[-1][2]
            d_min = min(d for _, d, _ in ds)
            d_max = max(d for _, d, _ in ds)
            d_mean = sum(d for _, d, _ in ds) / len(ds)
        else:
            d_first = d_last = raw_first = raw_last = d_min = d_max = d_mean = None
        sdr_mean = (sum(b["dbfs"]) / len(b["dbfs"])) if b["dbfs"] else None
        index_means.append((idx, d_mean, sdr_mean))

        def fmt(v, p=""):
            return f"{v}{p}" if v is not None else "-"

        print(f"{idx:>4} {dwell:>7.1f} {fmt(raw_first):>9} {fmt(raw_last):>8} "
              f"{fmt(d_first):>7} {fmt(d_last):>6} {fmt(d_min):>5} {fmt(d_max):>5} "
              f"{(f'{sdr_mean:.2f}' if sdr_mean is not None else '-'):>13} "
              f"{len(b['dbfs']):>5}")

    # Verdicts ----------------------------------------------------------------
    print("\n=== verdicts ===")
    deltas_all = [(s.t_ms, s.delta) for s in rows
                  if s.source == "thermal" and s.delta is not None]
    if deltas_all:
        first_d = deltas_all[0][1]
        last_d = deltas_all[-1][1]
        max_d = max(d for _, d in deltas_all)
        span_s = (deltas_all[-1][0] - deltas_all[0][0]) / 1000.0
        print(f"thermal delta over whole run: first={first_d:+d} last={last_d:+d} "
              f"max={max_d:+d} over {span_s:.0f}s")
        if last_d > first_d:
            print(f"  -> DEGRADES OVER TIME: delta rose {last_d - first_d} units "
                  f"under sustained TX")
        else:
            print(f"  -> stable/no rise: delta did not increase over the run")

    valid_means = [(i, dm) for i, dm, _ in index_means if dm is not None]
    if len(valid_means) >= 2:
        rising = all(valid_means[k][1] <= valid_means[k + 1][1] + 1e-9
                     for k in range(len(valid_means) - 1))
        lo, hi = valid_means[0][1], valid_means[-1][1]
        print(f"thermal delta vs gain index: {lo:.1f} (idx {valid_means[0][0]}) "
              f"-> {hi:.1f} (idx {valid_means[-1][0]}); "
              f"{'monotonic rise' if rising else 'non-monotonic'} "
              f"(net {hi - lo:+.1f})")

    sdr_means = [(i, sm) for i, _, sm in index_means if sm is not None]
    if len(sdr_means) >= 2:
        lo, hi = sdr_means[0][1], sdr_means[-1][1]
        rising = all(sdr_means[k][1] <= sdr_means[k + 1][1] + 0.5
                     for k in range(len(sdr_means) - 1))
        print(f"SDR power vs gain index: {lo:.1f} (idx {sdr_means[0][0]}) -> "
              f"{hi:.1f}dBFS (idx {sdr_means[-1][0]}); net {hi - lo:+.1f}dB "
              f"-> {'override IS reaching the PA' if hi - lo > 1.0 else 'NO clear on-air change'}")


if __name__ == "__main__":
    raise SystemExit(main())
