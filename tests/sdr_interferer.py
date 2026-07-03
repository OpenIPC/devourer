#!/usr/bin/env python3
"""USRP B210 calibrated co-channel interferer — a reproducible SNR knob.

When a conducted/attenuator setup isn't available (fixed antennas, adapters
inches apart with tens of dB of link margin), lowering chip TX power alone
can't reach the "received-but-FCS-failed" salvage regime. This instead raises
the noise floor at the receiver by transmitting band-filling AWGN (or a CW
tone) on the Wi-Fi channel from the B210. The FCS-failure rate then becomes a
reproducible function of ONE number — the USRP TX gain — with nothing moved.

Reproducible by construction: the noise is drawn from a fixed-seed NumPy RNG,
so a given --tx-gain reproduces the same interference run to run. Sweep
--tx-gain to dial the corruption rate; lock it for a rig setpoint.

  # ch6 (2.437 GHz) AWGN at 70 dB gain for 20 s:
  sudo python3 sdr_interferer.py --channel 6 --tx-gain 70 --secs 20
  # run until killed (the on-air harness backgrounds it):
  sudo python3 sdr_interferer.py --channel 6 --tx-gain 70

SAFETY: this radiates on a live Wi-Fi channel. Start at LOW gain and raise it —
too much and the receiver is fully jammed (0 frames) instead of partially
corrupted. Use only on a bench you control.
"""
from __future__ import annotations

import argparse
import signal
import sys
import time


def chan_to_freq(channel: int) -> float:
    """2.4 GHz: ch1..14 → 2407 + 5*ch MHz (ch6 = 2437). 5 GHz: 5000 + 5*ch
    (ch36 = 5180)."""
    if channel <= 14:
        return (2407 + 5 * channel) * 1e6
    return (5000 + 5 * channel) * 1e6


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--args", default="", help="UHD device args (e.g. type=b200)")
    ap.add_argument("--channel", type=int, default=None,
                    help="Wi-Fi channel (sets --freq); 2.4 GHz 1-14 / 5 GHz 36+")
    ap.add_argument("--freq", type=float, default=None, help="center freq Hz")
    ap.add_argument("--rate", type=float, default=20e6,
                    help="sample rate Hz = interference bandwidth (default 20e6)")
    ap.add_argument("--tx-gain", type=float, default=60.0,
                    help="USRP TX gain dB — THE reproducible SNR knob")
    ap.add_argument("--antenna", default="TX/RX")
    ap.add_argument("--amplitude", type=float, default=0.3,
                    help="digital backoff 0..1 (keep fixed; use --tx-gain to vary power)")
    ap.add_argument("--mode", default="noise", choices=("noise", "cw"))
    ap.add_argument("--seed", type=int, default=1234,
                    help="RNG seed — fixes the noise so runs are reproducible")
    ap.add_argument("--secs", type=float, default=0.0,
                    help="run time; 0 = until SIGINT/SIGTERM")
    ap.add_argument("--fade-coherence", type=float, default=0.0,
                    help="if >0, modulate interference power with a time-correlated "
                         "(AR1) envelope of this coherence time (s) -> the victim "
                         "link sees correlated SNR fades (for fade-SLA validation)")
    ap.add_argument("--fade-depth-db", type=float, default=8.0,
                    help="std of the interference power (dB) in the fade envelope; "
                         "deeper = harder fades. Mean power is unchanged (matched A/B)")
    args = ap.parse_args(argv)

    if args.freq is None:
        if args.channel is None:
            ap.error("need --freq or --channel")
        args.freq = chan_to_freq(args.channel)

    try:
        import numpy as np
        import uhd
    except ImportError as e:
        sys.stderr.write(f"sdr_interferer: import failed ({e}); UHD+NumPy "
                         "must be importable (system uhd, as in sdr_power_probe.py)\n")
        return 2

    usrp = uhd.usrp.MultiUSRP(args.args)
    usrp.set_tx_rate(args.rate)
    usrp.set_tx_freq(uhd.types.TuneRequest(args.freq))
    usrp.set_tx_gain(args.tx_gain)
    try:
        usrp.set_tx_antenna(args.antenna)
    except Exception:
        pass

    st = uhd.usrp.StreamArgs("fc32", "sc16")
    st.channels = [0]
    tx = usrp.get_tx_stream(st)
    md = uhd.types.TXMetadata()
    md.start_of_burst = True
    md.end_of_burst = False
    md.has_time_spec = False

    nsamps = 8192
    rng = np.random.default_rng(args.seed)
    if args.mode == "cw":
        # +2.5 MHz tone within the band — a deterministic, strong interferer.
        t = np.arange(nsamps) / args.rate
        tone = (args.amplitude * np.exp(2j * np.pi * 2.5e6 * t)).astype(np.complex64)
    else:
        # Pre-generated noise ring: drawing fresh Gaussians per 8192-sample
        # buffer in Python cannot keep 20 MS/s fed — the TX stream starves
        # (solid 'U' underflow spam) and the jam radiates only in bursts.
        # 64 fixed-seed buffers cycle with a ~26 ms period at 20 MS/s: still
        # band-filling AWGN to the victim PHY, still reproducible per seed.
        NRING = 64
        ring = [((rng.standard_normal(nsamps) + 1j * rng.standard_normal(nsamps))
                 / np.sqrt(2)).astype(np.complex64) for _ in range(NRING)]
        ring_scaled = [(args.amplitude * b).astype(np.complex64) for b in ring]

    sys.stderr.write(
        f"[interferer] freq={args.freq/1e9:.4f} GHz rate={args.rate/1e6:g} MS/s "
        f"gain={args.tx_gain:g} dB mode={args.mode} amp={args.amplitude} "
        f"seed={args.seed}\n")
    sys.stderr.flush()

    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))

    # Time-correlated fading envelope: an AR(1) process modulates the interference
    # power, so the victim link sees correlated SNR fades. Base amplitude is
    # auto-scaled so the loudest (~3-sigma) fade peak stays under the DAC ceiling.
    fade_on = args.fade_coherence > 0.0
    if fade_on:
        buf_dt = nsamps / args.rate
        fade_rho = float(np.exp(-buf_dt / args.fade_coherence))
        fade_g = 0.0
        # log-normal interference-power envelope with UNIT MEAN, so a fading run's
        # mean power equals a static run's at the same --tx-gain (a matched A/B).
        # fade_depth_db is the std of the interference power in dB.
        fade_sigma = args.fade_depth_db * (np.log(10.0) / 10.0)
        marker_every = max(1, int(0.1 / max(1e-6, buf_dt)))   # ~10 Hz markers

    t0 = time.monotonic()
    sent = 0
    nbuf = 0
    while not stop["flag"]:
        if args.secs and (time.monotonic() - t0) >= args.secs:
            break
        if fade_on:
            fade_g = fade_rho * fade_g + (1.0 - fade_rho ** 2) ** 0.5 * rng.standard_normal()
            # unit-mean log-normal power -> correlated deep fades, mean matched to static
            power = float(np.exp(fade_sigma * fade_g - 0.5 * fade_sigma ** 2))
            amp = min(0.95, args.amplitude * power ** 0.5)
            if nbuf % marker_every == 0:
                sys.stderr.write(f"<interferer-fade>t={time.monotonic()-t0:.2f} "
                                 f"powr={power:.2f} amp={amp:.3f}\n")
                sys.stderr.flush()
        else:
            amp = args.amplitude
        if args.mode == "cw":
            buf = (tone * (amp / max(1e-9, args.amplitude))).astype(np.complex64)
        elif fade_on:
            buf = (amp * ring[nbuf % NRING]).astype(np.complex64)
        else:
            buf = ring_scaled[nbuf % NRING]
        tx.send(buf.reshape(1, -1), md)
        md.start_of_burst = False
        sent += nsamps
        nbuf += 1

    md.end_of_burst = True
    try:
        tx.send(np.zeros((1, 1), dtype=np.complex64), md)
    except Exception:
        pass
    sys.stderr.write(f"[interferer] stopped ({sent} samples)\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
