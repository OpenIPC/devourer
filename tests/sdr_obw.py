#!/usr/bin/env python3
"""Occupied-bandwidth measurement via USRP spectrum capture.

Run a single-rate flood on the DUT, point this at the channel, and it reports
the 99%-power occupied bandwidth (equal-tail: the band between the 0.5% and
99.5% cumulative-power edges, the regulatory OBW definition) plus the
-20 dBr bandwidth (first 802.11 spectral-mask breakpoint) from a
frame-gated average power spectrum.

Method: capture at `--rate` (span = rate). The first `--warmup` seconds
establish the idle floor (1st percentile of per-FFT-block power — the
inter-frame gaps), then the remaining capture streams into a bounded
accumulator: blocks above floor+margin contribute their spectrum, everything
else is dropped, so memory stays O(FFT) regardless of rate or duration. The
+/-150 kHz around DC is notched (B2xx LO leakage) before any metric. The
-20 dBr reference is the peak of a 5-bin median-smoothed PSD, so a single-bin
spur can neither set nor widen it. The span must comfortably exceed the
signal (25 Msps for a 20 MHz PPDU, 50 Msps for 40 MHz) or the OBW integral
is clipped and reads low.

The radio is resolved through uhd_select (--args / DEVOURER_UHD_ARGS /
tests/.uhd_args) and the opened device's serial is printed with the result —
this bench has two B210s sharing one USB id. An uncalibrated B210 is fine
here: OBW and dBr points are relative measures within one capture; absolute
dBm stays out of scope.

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
    ap.add_argument("--warmup", type=float, default=0.5,
                    help="seconds used to establish the idle floor before "
                         "spectra start accumulating")
    ap.add_argument("--margin-db", type=float, default=8.0,
                    help="ON-block threshold = noise floor + margin")
    ap.add_argument("--top-db", type=float, default=None,
                    help="strong-block gate: keep only blocks within this "
                         "many dB of the strongest block. For busy channels "
                         "where a near-field DUT is much louder than ambient "
                         "— ambient 20 MHz frames are WIDER than a narrowband "
                         "signal and inflate the OBW integral (measured: a "
                         "clean 8.3 MHz emission read 16 MHz through ambient)")
    ap.add_argument("--occ", type=float, default=99.0,
                    help="occupied-power percentage (default 99)")
    ap.add_argument("--args", default=None,
                    help="UHD device args (e.g. serial=XXXX); default resolves "
                         "via DEVOURER_UHD_ARGS / tests/.uhd_args")
    args = ap.parse_args()

    usrp = uhd.usrp.MultiUSRP(uhd_select.device_args(args.args))
    serial = usrp.get_usrp_rx_info(0).get("mboard_serial", "?")
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

    win = np.hanning(FFT).astype(np.float32)
    wnorm = float((win ** 2).sum())
    warm_powers = []          # per-block mean power during warmup
    all_powers = []           # per-block mean power, whole capture (floor sanity)
    # ON spectra accumulate into 1 dB power bins (index = floor(block dB),
    # clamped to [-90, -1]) so the --top-db strong-block selection can be
    # made after the peak is known while memory stays O(bins x FFT).
    NBINS = 90
    bin_sum = np.zeros((NBINS, FFT), dtype=np.float64)
    bin_cnt = np.zeros(NBINS, dtype=np.int64)
    on_blocks = total_blocks = 0
    floor = None
    thr = np.inf
    t0 = time.monotonic()
    t_end = t0 + args.secs
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
            bp = sx.mean(axis=1)
            all_powers.append(bp.astype(np.float32))
            total_blocks += len(bp)
            if floor is None:
                warm_powers.append(bp.astype(np.float32))
                if time.monotonic() - t0 >= args.warmup:
                    wp = 10 * np.log10(np.concatenate(warm_powers) + 1e-12)
                    floor = float(np.percentile(wp, 1))
                    thr = 10 ** ((floor + args.margin_db) / 10)
                continue  # warmup blocks establish the floor, nothing else
            on = bp > thr
            if on.any():
                bpdb = 10 * np.log10(bp[on] + 1e-12)
                idx = np.clip(np.floor(bpdb).astype(int) + NBINS, 0,
                              NBINS - 1)
                np.add.at(bin_sum, idx, sx[on])
                np.add.at(bin_cnt, idx, 1)
                on_blocks += int(on.sum())
    finally:
        rx.issue_stream_cmd(uhd.types.StreamCMD(uhd.types.StreamMode.stop_cont))

    if floor is None or total_blocks == 0:
        print("sdr-obw: no samples")
        return 1
    if on_blocks < 100:
        print(f"sdr-obw: only {on_blocks} ON blocks "
              f"(floor {floor:.1f} dB) — is the flood running?")
        return 1
    full_floor = float(np.percentile(
        10 * np.log10(np.concatenate(all_powers) + 1e-12), 1))
    if abs(full_floor - floor) > 3:
        print(f"sdr-obw: WARNING warmup floor {floor:.1f} dB vs whole-capture "
              f"{full_floor:.1f} dB — floor drifted, re-run")
    used = bin_cnt > 0
    if args.top_db is not None:
        peak_bin = int(np.flatnonzero(used).max())
        used &= np.arange(NBINS) > peak_bin - args.top_db
    kept = int(bin_cnt[used].sum())
    if args.top_db is not None and kept < on_blocks:
        print(f"sdr-obw: top-db gate kept {kept}/{on_blocks} ON blocks "
              f"(within {args.top_db:.0f} dB of the strongest)")
    psd = bin_sum[used].sum(axis=0) / kept

    binw = args.rate / FFT
    freqs = (np.arange(FFT) - FFT // 2) * binw
    dc = np.abs(freqs) < 150e3
    psd[dc] = np.interp(np.flatnonzero(dc), np.flatnonzero(~dc), psd[~dc])

    # OBW, equal-tail: the band between the (100-occ)/2 and 100-(100-occ)/2
    # cumulative-power percentiles, so each side excludes the same tail.
    c = np.cumsum(psd)
    total = c[-1]
    tail_frac = (100.0 - args.occ) / 200.0
    lo = int(np.searchsorted(c, total * tail_frac))
    hi = int(np.searchsorted(c, total * (1.0 - tail_frac)))
    hi = min(hi, FFT - 1)
    obw = (hi - lo + 1) * binw

    # -20 dBr bandwidth on a 5-bin median-smoothed PSD: the smoothing keeps a
    # single-bin spur from setting the reference peak or widening the result.
    sm = np.copy(psd)
    for i in range(2, FFT - 2):
        sm[i] = np.median(psd[i - 2:i + 3])
    smdb = 10 * np.log10(sm + 1e-18)
    ref = float(smdb.max())
    above = np.flatnonzero(smdb > ref - 20)
    bw20 = (above[-1] - above[0] + 1) * binw if len(above) else 0.0

    print(f"sdr-obw: serial={serial} freq={args.freq/1e6:.0f}MHz "
          f"span={args.rate/1e6:.0f}MHz on_blocks={on_blocks}/{total_blocks} "
          f"obw{args.occ:.0f}={obw/1e6:.2f}MHz bw-20dBr={bw20/1e6:.2f}MHz "
          f"center_off={(freqs[lo]+freqs[hi])/2/1e6:+.2f}MHz")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
