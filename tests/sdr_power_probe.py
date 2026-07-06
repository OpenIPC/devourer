#!/usr/bin/env python3
"""USRP (UHD) receive-power probe for the thermal-vs-TX-gain experiment.

Tunes a UHD device (B200/B210/LibreSDR) to a fixed frequency and streams RX
samples continuously, emitting one windowed power reading per `--window`
samples as:

    sdr-power dbfs=<float> rms=<float> n=<int>

Power is **uncalibrated, relative dBFS** (10*log10(mean(|x|^2)) of the
normalised complex samples) — UHD's `recv` returns fc32 in roughly [-1, 1], so
this tracks *changes* in received power (e.g. as the 8812AU's TX gain ramps),
not an absolute dBm. Pair it with a fixed RX gain and a fixed geometry so the
trend is meaningful.

Stdout is line-buffered and each line is flushed; the orchestrator
(thermal_gain_sweep.py) stamps every line with a host-side arrival time, so no
cross-process clock alignment is needed here.

Run standalone to sanity-check the SDR:
    python3 sdr_power_probe.py --freq 2437e6 --rate 4e6 --gain 40 --duration 10
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

try:
    import uhd
except ImportError:
    sys.stderr.write(
        "sdr_power_probe: `import uhd` failed. UHD's Python module is a system "
        "package, not pip — create the venv with `uv venv --system-site-packages` "
        "(or run this script with the system python3).\n"
    )
    raise


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--freq", type=float, default=2437e6, help="center freq (Hz)")
    ap.add_argument("--rate", type=float, default=4e6, help="sample rate (Hz)")
    ap.add_argument("--gain", type=float, default=40.0, help="RX gain (dB)")
    ap.add_argument("--antenna", default="RX2", help="RX antenna port")
    ap.add_argument("--args", default="", help="UHD device args (e.g. type=b200)")
    ap.add_argument("--window", type=int, default=0,
                    help="samples per power reading (0 = ~rate/20, ~50ms)")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="seconds to run (0 = until killed)")
    ap.add_argument("--gated", action="store_true",
                    help="also report gated_dbfs: mean power of the 256-sample "
                         "blocks above noise+margin — duty-independent (frames "
                         "gate on, idle gaps gate off), so a framed feed reads "
                         "the same as continuous TX")
    ap.add_argument("--gate-margin-db", type=float, default=10.0,
                    help="gate threshold above the window's 10th-percentile "
                         "block power (default 10 dB)")
    args = ap.parse_args()

    window = args.window if args.window > 0 else max(1024, int(args.rate / 20))

    usrp = uhd.usrp.MultiUSRP(args.args)
    usrp.set_rx_rate(args.rate)
    usrp.set_rx_freq(uhd.types.TuneRequest(args.freq))
    usrp.set_rx_gain(args.gain)
    try:
        usrp.set_rx_antenna(args.antenna)
    except Exception:
        pass  # some clones expose only one port

    sys.stderr.write(
        f"sdr_power_probe: freq={args.freq/1e6:.3f}MHz rate={args.rate/1e6:.3f}Msps "
        f"gain={args.gain}dB window={window} actual_rate={usrp.get_rx_rate()/1e6:.3f}Msps\n"
    )
    sys.stderr.flush()

    st_args = uhd.usrp.StreamArgs("fc32", "sc16")
    st_args.channels = [0]
    rx = usrp.get_rx_stream(st_args)
    buf = np.zeros((1, rx.get_max_num_samps()), dtype=np.complex64)

    md = uhd.types.RXMetadata()
    stream_cmd = uhd.types.StreamCMD(uhd.types.StreamMode.start_cont)
    stream_cmd.stream_now = True
    rx.issue_stream_cmd(stream_cmd)

    acc_sumsq = 0.0
    acc_n = 0
    block_pows: list[float] = []  # per-256-sample block mean-squares (--gated)
    GATE_BLK = 256
    t_end = time.monotonic() + args.duration if args.duration > 0 else None
    try:
        while True:
            if t_end is not None and time.monotonic() >= t_end:
                break
            n = rx.recv(buf, md, 1.0)
            if md.error_code != uhd.types.RXMetadataErrorCode.none:
                # overflow ('O') is benign for a power probe; keep going
                if md.error_code != uhd.types.RXMetadataErrorCode.overflow:
                    sys.stderr.write(f"sdr_power_probe: rx error {md.error_code}\n")
                    sys.stderr.flush()
                continue
            if n <= 0:
                continue
            samples = buf[0, :n]
            acc_sumsq += float(np.sum((samples.real.astype(np.float64) ** 2)
                                      + (samples.imag.astype(np.float64) ** 2)))
            acc_n += n
            if args.gated:
                m = n - (n % GATE_BLK)
                if m > 0:
                    blocks = samples[:m].reshape(-1, GATE_BLK).astype(np.complex128)
                    # Per-block DC removal: the B210's DC-offset residual sits
                    # at band center (where a center-tuned DUT also sits) and
                    # reads as a constant ~-42 dBFS pedestal that ignores RX
                    # gain — measured pinning the low cells of a TXAGC sweep.
                    # OFDM carries no energy at DC, so subtracting the block
                    # mean removes the spur without touching signal power.
                    blocks = blocks - blocks.mean(axis=1, keepdims=True)
                    p = (np.abs(blocks) ** 2).mean(axis=1)
                    block_pows.extend(p.tolist())
            if acc_n >= window:
                meansq = acc_sumsq / acc_n
                rms = math.sqrt(meansq)
                dbfs = 10.0 * math.log10(meansq) if meansq > 0 else -200.0
                extra = ""
                if args.gated and block_pows:
                    arr = np.asarray(block_pows)
                    lo = float(np.percentile(arr, 10))
                    hi = float(arr.max())
                    if lo <= 0 or hi / max(lo, 1e-30) < 2.0:
                        # Uniform window (all-on continuous TX, or all-idle):
                        # nothing to gate — the plain mean IS the level.
                        g_meansq, frac = meansq, 1.0
                    else:
                        thr = lo * (10.0 ** (args.gate_margin_db / 10.0))
                        on = arr[arr > thr]
                        if on.size:
                            g_meansq, frac = float(on.mean()), on.size / arr.size
                        else:
                            g_meansq, frac = meansq, 1.0
                    g_dbfs = 10.0 * math.log10(g_meansq) if g_meansq > 0 else -200.0
                    extra = f" gated_dbfs={g_dbfs:.2f} gated_frac={frac:.3f}"
                    block_pows.clear()
                print(f"sdr-power dbfs={dbfs:.2f} rms={rms:.6f} n={acc_n}{extra}",
                      flush=True)
                acc_sumsq = 0.0
                acc_n = 0
    except KeyboardInterrupt:
        pass
    finally:
        stop = uhd.types.StreamCMD(uhd.types.StreamMode.stop_cont)
        rx.issue_stream_cmd(stop)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
