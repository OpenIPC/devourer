#!/usr/bin/env python3
"""Occupied-bandwidth measurement via USRP spectrum capture.

Run a single-rate flood on the DUT, point this at the channel, and it reports
the 99%-power occupied bandwidth (the regulatory OBW definition) plus the
-20 dBr bandwidth (first 802.11 spectral-mask breakpoint) from a
frame-gated average power spectrum.

Method: capture at `--rate` (span = rate), gate out inter-frame gaps by
per-block energy (same floor+margin rule sdr_duty.py uses), average the PSD
over ON blocks only, notch the +/-150 kHz around DC (B2xx LO leakage), then
integrate outward from the center until 99% of the in-span power is enclosed.
The span must comfortably exceed the signal (25 Msps for a 20 MHz PPDU,
50 Msps for 40 MHz) or the OBW integral is clipped and reads low.

An uncalibrated B210 is fine here: OBW and dBr points are relative measures
within one capture. Absolute dBm stays out of scope.

  sudo python3 tests/sdr_obw.py --freq 5180e6 --rate 25e6 --secs 4
"""
import argparse
import time

import numpy as np
import uhd  # type: ignore

import uhd_select

FFT = 1024


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--freq", type=float, required=True)
    ap.add_argument("--rate", type=float, default=25e6, help="USRP sample rate = span")
    ap.add_argument("--gain", type=float, default=40.0)
    ap.add_argument("--secs", type=float, default=4.0)
    ap.add_argument("--margin-db", type=float, default=8.0,
                    help="ON-block threshold = noise floor + margin")
    ap.add_argument("--occ", type=float, default=99.0,
                    help="occupied-power percentage (default 99)")
    args = ap.parse_args()

    usrp = uhd.usrp.MultiUSRP(uhd_select.device_args())
    usrp.set_rx_rate(args.rate)
    usrp.set_rx_freq(uhd.types.TuneRequest(args.freq))
    usrp.set_rx_gain(args.gain)
    st = uhd.usrp.StreamArgs("fc32", "sc16"); st.channels = [0]
    rx = usrp.get_rx_stream(st)
    buf = np.zeros((1, rx.get_max_num_samps()), dtype=np.complex64)
    md = uhd.types.RXMetadata()
    cmd = uhd.types.StreamCMD(uhd.types.StreamMode.start_cont)
    cmd.stream_now = True
    rx.issue_stream_cmd(cmd)

    blocks = []       # per-FFT-block mean power (for the ON/OFF gate)
    spectra = []      # per-FFT-block |X|^2 (gated later)
    win = np.hanning(FFT).astype(np.float32)
    wnorm = float((win ** 2).sum())
    t_end = time.monotonic() + args.secs
    tail = np.zeros(0, dtype=np.complex64)
    try:
        while time.monotonic() < t_end:
            n = rx.recv(buf, md, 1.0)
            if md.error_code != uhd.types.RXMetadataErrorCode.none or n <= 0:
                continue
            x = np.concatenate([tail, buf[0, :n]])
            nb = (len(x) // FFT) * FFT
            tail = x[nb:]
            if not nb:
                continue
            xb = x[:nb].reshape(-1, FFT)
            sx = np.fft.fftshift(np.abs(np.fft.fft(xb * win, axis=1)) ** 2,
                                 axes=1) / wnorm
            spectra.append(sx.astype(np.float32))
            blocks.append(sx.mean(axis=1))
    finally:
        rx.issue_stream_cmd(uhd.types.StreamCMD(uhd.types.StreamMode.stop_cont))

    if not spectra:
        print("sdr-obw: no samples")
        return 1
    sx = np.concatenate(spectra)
    bp = 10 * np.log10(np.concatenate(blocks) + 1e-12)
    floor = np.percentile(bp, 1)
    on = bp > floor + args.margin_db
    if on.sum() < 100:
        print(f"sdr-obw: only {int(on.sum())} ON blocks "
              f"(floor {floor:.1f} dB) — is the flood running?")
        return 1
    psd = sx[on].mean(axis=0)

    binw = args.rate / FFT
    freqs = (np.arange(FFT) - FFT // 2) * binw
    dc = np.abs(freqs) < 150e3
    psd[dc] = np.interp(np.flatnonzero(dc), np.flatnonzero(~dc), psd[~dc])

    # OBW: grow a window outward from the power centroid bin until it holds
    # `occ`% of the total in-span power.
    total = psd.sum()
    lo = hi = int(np.argmax(np.cumsum(psd) >= total / 2))
    acc = psd[lo]
    while acc < total * args.occ / 100 and (lo > 0 or hi < FFT - 1):
        left = psd[lo - 1] if lo > 0 else -1.0
        right = psd[hi + 1] if hi < FFT - 1 else -1.0
        if left >= right:
            lo -= 1; acc += psd[lo]
        else:
            hi += 1; acc += psd[hi]
    obw = (hi - lo + 1) * binw

    # -20 dBr bandwidth: outermost bins within 20 dB of the in-band peak
    # (peak = 95th percentile of the PSD, so a spur can't set the reference).
    ref = np.percentile(10 * np.log10(psd + 1e-18), 95)
    above = np.flatnonzero(10 * np.log10(psd + 1e-18) > ref - 20)
    bw20 = (above[-1] - above[0] + 1) * binw if len(above) else 0.0

    print(f"sdr-obw: freq={args.freq/1e6:.0f}MHz span={args.rate/1e6:.0f}MHz "
          f"on_blocks={int(on.sum())}/{len(on)} "
          f"obw{args.occ:.0f}={obw/1e6:.2f}MHz bw-20dBr={bw20/1e6:.2f}MHz "
          f"center_off={(freqs[lo]+freqs[hi])/2/1e6:+.2f}MHz")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
