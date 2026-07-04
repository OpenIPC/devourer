#!/usr/bin/env python3
"""SDR on-air power / occupied-bandwidth probe for the Jaguar-3 TX bring-up.

Captures a short IQ window from the USRP B210 at a center frequency and reports
mean in-band power plus the 99%-power occupied bandwidth (Welch PSD). Used as the
on-air ground truth for:

  * confirming 20 MHz TX is actually radiating (mean-power rise vs a silent
    baseline at the same center freq).
  * confirming narrowband: a 10 MHz channel should show ~half the occupied
    bandwidth of a 20 MHz one, 5 MHz ~a quarter. Radiotap can't see this; the
    SDR is the only witness.

NB: the UHD Python binding is a system package (pacman `uhd`), not pip/uv
installable, so this script intentionally runs under the system interpreter and
is the one test script outside the uv-managed Python subtree.

  python3 tests/sdr_tx_probe.py --freq 5180e6 --label baseline
  python3 tests/sdr_tx_probe.py --freq 5180e6 --label tx --json /tmp/tx.json
"""
import argparse
import json
import sys

import numpy as np
import uhd


def measure(freq, rate, gain, nsamps, median=False):
    usrp = uhd.usrp.MultiUSRP("")
    # recv_num_samps tunes, sets rate+gain, streams, and returns a complex64
    # array shaped (channels, nsamps). One channel (RX A).
    samps = usrp.recv_num_samps(int(nsamps), float(freq), float(rate), [0], float(gain))
    x = samps[0].astype(np.complex64)

    # Drop the leading transient (AGC / tune settle) before measuring.
    x = x[len(x) // 8:]
    mean_power = float(np.mean(np.abs(x) ** 2))
    mean_db = 10.0 * np.log10(mean_power + 1e-20)

    # Welch PSD for occupied bandwidth (99% of total power, two-sided).
    nfft = 4096
    win = np.hanning(nfft)
    nseg = len(x) // nfft
    if median:
        # Median across segments instead of mean: suppresses bursty, low-duty
        # ambient (an AP beaconing through the capture) so a high-duty DUT
        # (DEVOURER_TX_GAP_US=0) dominates the estimate. Use for differential
        # A/Bs on a band with neighbours.
        segs = np.empty((nseg, nfft), dtype=np.float32)
        for i in range(nseg):
            seg = x[i * nfft:(i + 1) * nfft] * win
            segs[i] = np.abs(np.fft.fftshift(np.fft.fft(seg))) ** 2
        psd = np.median(segs, axis=0).astype(np.float64)
    else:
        acc = np.zeros(nfft)
        for i in range(nseg):
            seg = x[i * nfft:(i + 1) * nfft] * win
            acc += np.abs(np.fft.fftshift(np.fft.fft(seg))) ** 2
        psd = acc / max(nseg, 1)
    freqs = np.fft.fftshift(np.fft.fftfreq(nfft, d=1.0 / rate))

    total = psd.sum()
    if total > 0:
        # Occupied BW: smallest contiguous band holding 99% of power, centered
        # on the power centroid via a cumulative-from-edges trim.
        csum = np.cumsum(psd)
        lo = np.searchsorted(csum, 0.005 * total)
        hi = np.searchsorted(csum, 0.995 * total)
        occ_bw = float(freqs[min(hi, nfft - 1)] - freqs[max(lo, 0)])
    else:
        occ_bw = 0.0

    return {
        "freq_hz": float(freq),
        "rate_hz": float(rate),
        "gain_db": float(gain),
        "nsamps": int(len(x)),
        "mean_power_db": round(mean_db, 2),
        "occupied_bw_hz": round(occ_bw, 0),
    }, freqs, psd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--freq", default="5180e6", help="center freq Hz (ch36=5180e6)")
    ap.add_argument("--rate", default="23.04e6", help="sample rate Hz")
    ap.add_argument("--gain", default="40", help="RX gain dB")
    ap.add_argument("--nsamps", default="4e6", help="samples to capture")
    ap.add_argument("--label", default="probe")
    ap.add_argument("--json", default="", help="append result as JSON to this file")
    ap.add_argument("--psd-out", default="", help="save [freqs;psd] to this .npy")
    ap.add_argument("--psd-median", action="store_true",
                    help="median PSD across segments (suppresses bursty ambient)")
    a = ap.parse_args()

    r, freqs, psd = measure(float(a.freq), float(a.rate), float(a.gain),
                            float(a.nsamps), median=a.psd_median)
    r["label"] = a.label
    if a.psd_out:
        np.save(a.psd_out, np.vstack([freqs, psd]))
    line = (f"[sdr:{a.label}] f={r['freq_hz']/1e6:.1f}MHz "
            f"power={r['mean_power_db']:.2f}dB occ_bw={r['occupied_bw_hz']/1e6:.2f}MHz "
            f"({r['nsamps']} samps @ {r['rate_hz']/1e6:.2f}Msps gain {r['gain_db']:.0f})")
    print(line, file=sys.stderr)
    if a.json:
        with open(a.json, "a") as f:
            f.write(json.dumps(r) + "\n")


if __name__ == "__main__":
    main()
