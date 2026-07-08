#!/usr/bin/env python3
"""SDR on-air validation for Jaguar2 thermal TX-power tracking.

The chip reaches its thermal plateau (delta ~6-8 on a bus-powered dongle) within
seconds, so a cold->hot power sweep has too little delta spread. Instead this
measures the compensation DIRECTLY, on-air, at a matched hot steady state:

  1. Pre-heat the PA to steady state (txdemo, tracking OFF).
  2. Measure on-air power P_off with tracking OFF (USRP gated dBFS).
  3. Restart txdemo with tracking ON (chip stays hot), let the ~2 s tick ramp
     its swing to the plateau, measure P_on.

At the same hot state the tracking-ON pass adds the delta-swing compensation the
tracking-OFF pass omits, so P_on - P_off is exactly the PA gain droop the
tracker restores on-air. It is cross-checked against the swing index the driver
logged (8822B TXAGC step = 0.5 dB), so the number is grounded, not just a delta.

A single USRP probe streams for the whole run (one clean acquisition — a
back-to-back re-open can fail to reacquire and read ~0).

Run via tests/run_jaguar2_thermal_track_sdr.sh (uv venv w/ system uhd + sudo).
"""
import argparse
import os
import re
import statistics
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import regress  # noqa: E402
from devourer_events import parse_event  # noqa: E402

SDR_RE = re.compile(r"sdr-power dbfs=([+-]?[\d.]+)")
GATED_RE = re.compile(r"gated_dbfs=([+-]?[\d.]+)")
SWING_RE = re.compile(r"thermal-track:.* d=(\d+) swing=([+-]?\d+)")
CH_FREQ_MHZ = {1: 2412, 6: 2437, 11: 2462, 36: 5180, 44: 5220,
               100: 5500, 149: 5745, 161: 5805}


class Sdr:
    """One long-lived USRP power probe; samples tagged with monotonic time."""
    def __init__(self, freq, rate, gain, log):
        self.samples = []            # (t_mono, dbfs)
        self.lock = threading.Lock()
        self.streaming = threading.Event()
        self.proc = regress._register_local_proc(subprocess.Popen(
            [sys.executable, str(HERE / "sdr_power_probe.py"),
             "--freq", str(freq), "--rate", str(rate), "--gain", str(gain),
             "--gated"], stdout=subprocess.PIPE, stderr=log, text=True,
            preexec_fn=regress._child_preexec))
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for raw in self.proc.stdout:
            self.streaming.set()
            m = GATED_RE.search(raw) or SDR_RE.search(raw)
            if m:
                with self.lock:
                    self.samples.append((time.monotonic(), float(m.group(1))))

    def window(self, t_start, t_end):
        with self.lock:
            return [db for t, db in self.samples if t_start <= t <= t_end]

    def stop(self):
        self.proc.terminate()


def run_txdemo(track_on, args, txdemo, log):
    env = dict(os.environ,
               DEVOURER_VID=args.vid, DEVOURER_PID=args.pid,
               DEVOURER_CHANNEL=str(args.channel),
               DEVOURER_TX_RATE=args.rate, DEVOURER_TX_GAP_US="0",
               DEVOURER_THERMAL_POLL_MS=str(args.thermal_ms),
               DEVOURER_THERMAL_TRACK=("1" if track_on else "0"))
    deltas = []
    proc = regress._register_local_proc(subprocess.Popen(
        [str(txdemo)], env=env, stdout=subprocess.PIPE, stderr=log, text=True,
        preexec_fn=regress._child_preexec))

    def read():
        for raw in proc.stdout:
            ev = parse_event(raw.rstrip("\n"))
            if ev and ev.get("ev") == "thermal" and ev.get("delta") is not None:
                deltas.append(int(ev["delta"]))
    threading.Thread(target=read, daemon=True).start()
    return proc, deltas


def kill(proc, comm=None):
    proc.terminate()
    time.sleep(0.4)
    if comm:
        subprocess.run(["pkill", "-x", comm], check=False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vid", default="0x2357")          # T3U 8822B
    ap.add_argument("--pid", default="0x012d")
    ap.add_argument("--channel", type=int, default=6)
    ap.add_argument("--rate", default="6M")
    ap.add_argument("--preheat", type=float, default=70)  # reach steady state
    ap.add_argument("--settle", type=float, default=18)   # swing ramp after ON restart
    ap.add_argument("--meas", type=float, default=25)     # SDR averaging window
    ap.add_argument("--thermal-ms", type=int, default=500)
    ap.add_argument("--sdr-freq", type=float, default=0)
    ap.add_argument("--sdr-rate", type=float, default=25e6)
    ap.add_argument("--sdr-gain", type=float, default=40.0)
    ap.add_argument("--outdir", default="/tmp/tt_sdr2")
    args = ap.parse_args()

    txdemo = HERE.parent / "build" / "txdemo"
    if not txdemo.exists():
        sys.exit("txdemo not built")
    sdr_freq = args.sdr_freq or CH_FREQ_MHZ.get(args.channel, 0) * 1e6
    if sdr_freq <= 0:
        sys.exit(f"no freq for channel {args.channel}; pass --sdr-freq")
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    regress._install_cleanup_handlers()

    print(f"# DUT {args.vid}:{args.pid} ch{args.channel} {args.rate} | "
          f"SDR {sdr_freq/1e6:.1f}MHz rate {args.sdr_rate/1e6:.0f}M\n")

    sdr = Sdr(sdr_freq, args.sdr_rate, args.sdr_gain,
              open(outdir / f"sdr-{stamp}.log", "w"))
    if not sdr.streaming.wait(timeout=25):
        sys.exit("SDR probe never streamed")
    time.sleep(1.0)

    # ---- Phase 1: tracking OFF, pre-heat then measure ------------------------
    off_log = open(outdir / f"tx-off-{stamp}.log", "w")
    tx_off, d_off = run_txdemo(False, args, txdemo, off_log)
    print(f"-- OFF: pre-heating {args.preheat:.0f}s to steady state --", flush=True)
    time.sleep(args.preheat)
    t0 = time.monotonic()
    print(f"-- OFF: measuring on-air power {args.meas:.0f}s --", flush=True)
    time.sleep(args.meas)
    p_off = sdr.window(t0, time.monotonic())
    delta_off = statistics.median(d_off[-20:]) if d_off else None
    kill(tx_off, "txdemo")

    # ---- Phase 2: tracking ON, chip still hot, settle then measure -----------
    on_log_path = outdir / f"tx-on-{stamp}.log"
    on_log = open(on_log_path, "w")
    tx_on, d_on = run_txdemo(True, args, txdemo, on_log)
    print(f"-- ON: restarting hot + settling {args.settle:.0f}s (swing ramps) --",
          flush=True)
    time.sleep(args.settle)
    t1 = time.monotonic()
    print(f"-- ON: measuring on-air power {args.meas:.0f}s --", flush=True)
    time.sleep(args.meas)
    p_on = sdr.window(t1, time.monotonic())
    delta_on = statistics.median(d_on[-20:]) if d_on else None
    kill(tx_on, "txdemo")
    sdr.stop()
    subprocess.run(["pkill", "-f", "tests/sdr_power_probe.py"], check=False)

    # swing the ON tick actually applied at the measured delta
    swings = []
    for line in open(on_log_path):
        m = SWING_RE.search(line)
        if m:
            swings.append(int(m.group(2)))
    swing_hot = max(swings) if swings else 0

    print("\n================ VERDICT ================")
    if len(p_off) < 5 or len(p_on) < 5:
        sys.exit(f"INCONCLUSIVE: too few SDR samples (off={len(p_off)} on={len(p_on)})")
    m_off, m_on = statistics.fmean(p_off), statistics.fmean(p_on)
    sd_off = statistics.pstdev(p_off) if len(p_off) > 1 else 0
    sd_on = statistics.pstdev(p_on) if len(p_on) > 1 else 0
    gain = m_on - m_off
    expect = swing_hot * 0.5  # 8822B TXAGC step = 0.5 dB
    print(f"  OFF hot (Δ~{delta_off}): {m_off:.2f} dBFS  (sd {sd_off:.2f}, n {len(p_off)})")
    print(f"  ON  hot (Δ~{delta_on}): {m_on:.2f} dBFS  (sd {sd_on:.2f}, n {len(p_on)})")
    print(f"  on-air gain from tracking = {gain:+.2f} dB")
    print(f"  swing applied (ON) = {swing_hot} idx  => expected ~{expect:+.2f} dB")
    print("-----------------------------------------")
    # PASS: tracking measurably raises hot on-air power (restoring the droop),
    # by an amount consistent with the swing it logged.
    if gain >= 1.0 and gain >= 0.4 * expect and swing_hot >= 2:
        print(f"RESULT: PASS — at a matched hot state the tracker adds {gain:+.2f} dB "
              f"on-air (swing {swing_hot} idx ~ {expect:+.1f} dB expected), i.e. it "
              f"restores the PA droop that OFF leaves uncorrected.")
        sys.exit(0)
    if swing_hot < 2:
        print(f"RESULT: WEAK — chip barely heated (swing {swing_hot} idx); raise "
              f"--preheat or lower --rate.")
        sys.exit(1)
    print(f"RESULT: FAIL — expected ~{expect:+.1f} dB of on-air gain, measured "
          f"{gain:+.2f} dB (compensation not reaching the air).")
    sys.exit(1)


if __name__ == "__main__":
    main()
