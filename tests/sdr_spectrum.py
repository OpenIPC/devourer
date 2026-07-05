#!/usr/bin/env python3
"""USRP (UHD) spectrum-shape probe — tells a modulated wideband signal from a
bare CW tone. Captures IQ at a center frequency, computes a Welch PSD, and
prints a coarse ASCII spectrum plus two discriminators:

    occ_mhz   — occupied bandwidth (span of bins within 10 dB of the peak)
    peakiness — peak_bin_power / median_bin_power (dB); a single CW tone is a
                tall narrow spike (high peakiness, occ_mhz ~ 0); a modulated
                OFDM waveform fills the channel (low peakiness, occ_mhz ~ BW).

Uncalibrated relative dBFS, like sdr_power_probe.py. Pair with a fixed geometry.

    python3 sdr_spectrum.py --freq 5180e6 --rate 25e6 --gain 40 --duration 2
"""
from __future__ import annotations

import argparse
import sys

import numpy as np

try:
    import uhd
except ImportError:
    sys.stderr.write("sdr_spectrum: `import uhd` failed — use the "
                     "--system-site-packages venv (see tests/pyproject.toml).\n")
    raise


def capture(freq, rate, gain, antenna, dev_args, nsamp):
    usrp = uhd.usrp.MultiUSRP(dev_args)
    usrp.set_rx_rate(rate)
    usrp.set_rx_freq(uhd.types.TuneRequest(freq))
    usrp.set_rx_gain(gain)
    usrp.set_rx_antenna(antenna)
    st_args = uhd.usrp.StreamArgs("fc32", "sc16")
    st_args.channels = [0]
    rx = usrp.get_rx_stream(st_args)
    md = uhd.types.RXMetadata()
    buf = np.zeros((1, rx.get_max_num_samps()), dtype=np.complex64)
    out = np.empty(nsamp, dtype=np.complex64)
    got = 0
    sc = uhd.types.StreamCMD(uhd.types.StreamMode.start_cont)
    sc.stream_now = True
    rx.issue_stream_cmd(sc)
    while got < nsamp:
        n = rx.recv(buf, md)
        if md.error_code != uhd.types.RXMetadataErrorCode.none or n == 0:
            continue
        take = min(n, nsamp - got)
        out[got:got + take] = buf[0, :take]
        got += take
    rx.issue_stream_cmd(uhd.types.StreamCMD(uhd.types.StreamMode.stop_cont))
    return out


def welch_psd(x, rate, nfft=1024):
    win = np.hanning(nfft)
    step = nfft // 2
    acc = np.zeros(nfft)
    k = 0
    for i in range(0, len(x) - nfft, step):
        seg = x[i:i + nfft] * win
        acc += np.abs(np.fft.fftshift(np.fft.fft(seg))) ** 2
        k += 1
    psd = acc / max(k, 1)
    # Null the DC bins — the B210 has a fixed LO-leakage / DC-offset spike at the
    # tuned center that would masquerade as a tone. Replace the center +-2 bins
    # with the local median so occupancy/peakiness reflect the real signal.
    c = nfft // 2
    psd[c - 2:c + 3] = np.median(psd)
    psd_db = 10 * np.log10(psd + 1e-12)
    freqs = np.fft.fftshift(np.fft.fftfreq(nfft, 1.0 / rate))
    return freqs, psd_db


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--freq", type=float, default=5180e6)
    ap.add_argument("--rate", type=float, default=25e6)
    ap.add_argument("--gain", type=float, default=40.0)
    ap.add_argument("--antenna", default="RX2")
    ap.add_argument("--args", default="type=b200")
    ap.add_argument("--duration", type=float, default=1.5)
    ap.add_argument("--label", default="")
    args = ap.parse_args()

    nsamp = max(1 << 16, int(args.rate * args.duration))
    x = capture(args.freq, args.rate, args.gain, args.antenna, args.args, nsamp)
    freqs, psd = welch_psd(x, args.rate)

    peak = psd.max()
    med = np.median(psd)
    peakiness = peak - med
    # Occupied bandwidth: contiguous-ish span of bins within 10 dB of peak.
    mask = psd >= peak - 10.0
    occ_hz = (freqs[mask].max() - freqs[mask].min()) if mask.any() else 0.0
    occ_mhz = occ_hz / 1e6

    # Coarse ASCII spectrum (downsample to ~60 columns).
    cols = 60
    binw = len(psd) // cols
    ds = np.array([psd[i * binw:(i + 1) * binw].max() for i in range(cols)])
    lo, hi = ds.min(), ds.max()
    span = (hi - lo) or 1.0
    ramp = " .:-=+*#%@"
    line = "".join(ramp[min(len(ramp) - 1, int((v - lo) / span * (len(ramp) - 1)))]
                    for v in ds)
    lbl = f"[{args.label}] " if args.label else ""
    print(f"{lbl}{args.freq/1e6:.0f} MHz, {args.rate/1e6:.0f} MS/s")
    print(f"  {line}")
    print(f"  peakiness={peakiness:.1f} dB  occ_bw={occ_mhz:.1f} MHz  "
          f"(tone: high peakiness/low occ; modulated: low peakiness/wide occ)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
