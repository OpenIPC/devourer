#!/usr/bin/env python3
"""USRP (UHD) spectral probe for the Jaguar-1 CW single-tone (DEVOURER_CW_TONE).

Captures one IQ window from a UHD device (B200/B210/LibreSDR), computes a Welch
PSD, and quantifies the emitted carrier so the tone can be trusted as an
instrument before it is used as a narrowband interferer (issue #165):

  * peak-bin frequency OFFSET from the SDR center (kHz) — should be ~0 when the
    SDR is tuned to the tone's channel-center frequency,
  * carrier-to-spur ratio (dB) — peak PSD vs the strongest bin at least
    --guard-khz away from the peak (the worst spur/image/residual),
  * peak power (relative dBFS) — for a gain-vs-power sweep (uncalibrated, like
    sdr_power_probe.py — tracks CHANGES, not absolute dBm),
  * noise floor (median PSD, dBFS).

With --drift-secs > 0 it instead takes back-to-back short captures across that
span and reports the peak-bin wander (min/max/std, kHz) — the cert-style
frequency-hold check.

Emits one JSON object per line to stdout (and to --json if given), so the
orchestrator (cw_tone_sdr.sh) can aggregate the sweep. Example:

    python3 cw_tone_probe.py --freq 2437e6 --rate 10e6 --gain 40 \\
        --label 8812au_ch6_g16 --json /tmp/cw.jsonl
"""
from __future__ import annotations

import argparse
import json
import sys
import time

import numpy as np

try:
    import uhd
except ImportError:
    sys.stderr.write(
        "cw_tone_probe: `import uhd` failed. UHD's Python module is a system "
        "package, not pip — create the venv with `uv venv --system-site-packages` "
        "(or run this script with the system python3).\n"
    )
    raise


def _open(args) -> "uhd.usrp.MultiUSRP":
    usrp = uhd.usrp.MultiUSRP(args.args)
    usrp.set_rx_rate(args.rate)
    usrp.set_rx_freq(uhd.types.TuneRequest(args.freq))
    usrp.set_rx_gain(args.gain)
    try:
        usrp.set_rx_antenna(args.antenna)
    except Exception:
        pass  # some clones expose only one port
    return usrp


def _capture(usrp, rate: float, nsamp: int) -> np.ndarray:
    """Stream `nsamp` contiguous samples; overflow-tolerant, drops nothing that
    matters for a stationary-tone PSD (we just need one clean window)."""
    st_args = uhd.usrp.StreamArgs("fc32", "sc16")
    st_args.channels = [0]
    rx = usrp.get_rx_stream(st_args)
    buf = np.zeros((1, rx.get_max_num_samps()), dtype=np.complex64)
    md = uhd.types.RXMetadata()
    cmd = uhd.types.StreamCMD(uhd.types.StreamMode.start_cont)
    cmd.stream_now = True
    rx.issue_stream_cmd(cmd)
    out = np.empty(nsamp, dtype=np.complex64)
    got = 0
    try:
        while got < nsamp:
            n = rx.recv(buf, md, 1.0)
            if md.error_code not in (
                uhd.types.RXMetadataErrorCode.none,
                uhd.types.RXMetadataErrorCode.overflow,
            ):
                continue
            if n <= 0:
                continue
            take = min(n, nsamp - got)
            out[got:got + take] = buf[0, :take]
            got += take
    finally:
        rx.issue_stream_cmd(uhd.types.StreamCMD(uhd.types.StreamMode.stop_cont))
    return out


def _welch_psd(x: np.ndarray, nfft: int) -> np.ndarray:
    """Averaged (Welch) complex PSD, fft-shifted so index 0 = -rate/2. Returns
    power (linear, arbitrary scale — consistent across captures at fixed gain)."""
    win = np.hanning(nfft).astype(np.float64)
    wnorm = float(np.sum(win ** 2))
    nseg = max(1, len(x) // nfft)
    acc = np.zeros(nfft, dtype=np.float64)
    for i in range(nseg):
        seg = x[i * nfft:(i + 1) * nfft]
        if len(seg) < nfft:
            break
        sp = np.fft.fftshift(np.fft.fft(seg.astype(np.complex64) * win))
        acc += (np.abs(sp) ** 2)
    return acc / (nseg * wnorm)


def _analyse(x: np.ndarray, rate: float, nfft: int, guard_khz: float) -> dict:
    psd = _welch_psd(x, nfft)
    freqs_hz = (np.arange(nfft) - nfft // 2) * (rate / nfft)  # bin center offsets
    peak_i = int(np.argmax(psd))
    peak_off_khz = float(freqs_hz[peak_i] / 1e3)
    bin_khz = (rate / nfft) / 1e3
    guard_bins = max(1, int(round(guard_khz / bin_khz)))
    # Spur = strongest bin outside +-guard around the peak.
    mask = np.ones(nfft, dtype=bool)
    lo = max(0, peak_i - guard_bins)
    hi = min(nfft, peak_i + guard_bins + 1)
    mask[lo:hi] = False
    spur_i = int(np.argmax(np.where(mask, psd, 0.0)))
    peak_db = 10.0 * np.log10(psd[peak_i] + 1e-30)
    spur_db = 10.0 * np.log10(psd[spur_i] + 1e-30)
    floor_db = 10.0 * np.log10(np.median(psd) + 1e-30)
    return {
        "peak_offset_khz": round(peak_off_khz, 2),
        "peak_dbfs": round(float(peak_db), 2),
        "spur_dbfs": round(float(spur_db), 2),
        "spur_offset_khz": round(float(freqs_hz[spur_i] / 1e3), 2),
        "c2s_db": round(float(peak_db - spur_db), 2),
        "floor_dbfs": round(float(floor_db), 2),
        "peak_to_floor_db": round(float(peak_db - floor_db), 2),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--freq", type=float, required=True, help="SDR center freq (Hz)")
    ap.add_argument("--rate", type=float, default=10e6, help="sample rate (Hz)")
    ap.add_argument("--gain", type=float, default=40.0, help="RX gain (dB)")
    ap.add_argument("--antenna", default="RX2", help="RX antenna port")
    ap.add_argument("--args", default="", help="UHD device args (e.g. type=b200)")
    ap.add_argument("--nfft", type=int, default=8192, help="FFT / PSD resolution")
    ap.add_argument("--secs", type=float, default=0.2, help="capture length (s)")
    ap.add_argument("--guard-khz", type=float, default=500.0,
                    help="guard band around the peak for the spur search (kHz)")
    ap.add_argument("--drift-secs", type=float, default=0.0,
                    help=">0: back-to-back captures over this span, report drift")
    ap.add_argument("--drift-window", type=float, default=0.05,
                    help="per-capture length in drift mode (s)")
    ap.add_argument("--label", default="", help="tag echoed into the JSON")
    ap.add_argument("--json", default="", help="append JSON lines to this file too")
    args = ap.parse_args()

    usrp = _open(args)
    actual = usrp.get_rx_rate()
    sys.stderr.write(
        f"cw_tone_probe: freq={args.freq/1e6:.4f}MHz rate={actual/1e6:.3f}Msps "
        f"gain={args.gain}dB nfft={args.nfft} label={args.label!r}\n")
    sys.stderr.flush()

    def emit(rec: dict) -> None:
        rec = {"label": args.label, "center_mhz": round(args.freq / 1e6, 4),
               "rate_msps": round(actual / 1e6, 3), **rec}
        line = json.dumps(rec)
        print(line, flush=True)
        if args.json:
            with open(args.json, "a") as f:
                f.write(line + "\n")

    if args.drift_secs > 0:
        nsamp = max(args.nfft, int(actual * args.drift_window))
        offs = []
        t_end = time.monotonic() + args.drift_secs
        n = 0
        while time.monotonic() < t_end:
            x = _capture(usrp, actual, nsamp)
            a = _analyse(x, actual, args.nfft, args.guard_khz)
            offs.append(a["peak_offset_khz"])
            n += 1
        offs_a = np.asarray(offs, dtype=np.float64)
        emit({"mode": "drift", "n_captures": n,
              "drift_secs": args.drift_secs,
              "peak_offset_khz_min": round(float(offs_a.min()), 2),
              "peak_offset_khz_max": round(float(offs_a.max()), 2),
              "peak_offset_khz_span": round(float(offs_a.max() - offs_a.min()), 2),
              "peak_offset_khz_std": round(float(offs_a.std()), 3)})
        return 0

    nsamp = max(args.nfft, int(actual * args.secs))
    x = _capture(usrp, actual, nsamp)
    emit({"mode": "psd", **_analyse(x, actual, args.nfft, args.guard_khz)})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
