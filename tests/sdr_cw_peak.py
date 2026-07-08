#!/usr/bin/env python3
"""Report the CW-tone peak frequency offset (Hz) from the tuned center — a
direct readout of the chip LO, hence of a crystal-cap trim (tests/
xtal_cfo_sweep.sh). Parabolic-interpolated for sub-bin resolution. USRP B210.

  python3 tests/sdr_cw_peak.py 5220e6
"""
import sys
import numpy as np
import uhd

freq = float(sys.argv[1])
rate = 46.08e6
gain = 50
n = int(8e5)

u = uhd.usrp.MultiUSRP()
s = u.recv_num_samps(n, freq, rate, [0], gain)[0]
s = s - np.mean(s)  # drop the DC/LO-leakage bin so it can't win the argmax

nfft = 1 << 20
win = s[:nfft] * np.hanning(min(len(s), nfft))
if len(win) < nfft:
    win = np.concatenate([win, np.zeros(nfft - len(win))])
spec = np.abs(np.fft.fftshift(np.fft.fft(win)))
freqs = np.fft.fftshift(np.fft.fftfreq(nfft, 1.0 / rate))

pk = int(np.argmax(spec))
d = 0.0
if 0 < pk < nfft - 1:
    a, b, c = spec[pk - 1], spec[pk], spec[pk + 1]
    denom = a - 2 * b + c
    if denom:
        d = 0.5 * (a - c) / denom
print(f"{freqs[pk] + d * rate / nfft:.0f}")
