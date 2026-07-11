#!/usr/bin/env python3
"""Full-duplex follower jammer (USRP B210) for the M5 dwell-threshold experiment.

The B210 is a 2x2 (one AD9361, two RX + two TX frontends), so it senses and jams
AT THE SAME TIME — no time-multiplexing. This follower:

  * TX frontend (TX/RX port): a worker thread continuously radiates a jam burst
    on the current target channel, retuning (set_tx_freq) the instant the target
    changes.
  * RX frontend (RX2 port): the main loop takes a ~0.3 ms wideband burst (one
    FFT covers the whole hopset), finds the strongest hopset channel OTHER than
    the one it is currently jamming (its own TX would otherwise win), and moves
    the jam there. That excluded-self detection is what lets it chase.

Two strategies:
  reactive   (--predict off, the threat to a KEYED TX): jam where the TX was
             last sensed. A hit needs the TX still there after the sense+retune
             latency L; once the slot dwell D < L the keyed TX has already jumped
             to an unpredictable channel, so following breaks and delivery
             recovers.
  predictive (--predict seq, the threat to a SEQUENTIAL TX): the public order is
             round-robin, so the follower jams the NEXT channel it expects —
             anticipating cancels its own latency, so it holds to far smaller D.

The gap between the two dwell thresholds is exactly what a keyed permutation
buys. Each retune is logged as a `follow.jam` event (t_us, sense_ch, jam_ch,
retune_us). Run under sudo (UHD).
"""
from __future__ import annotations

import argparse
import json
import signal
import sys
import threading
import time

import numpy as np


def chan_to_freq(ch: int) -> float:
    return (2407 + 5 * ch) * 1e6 if ch <= 14 else (5000 + 5 * ch) * 1e6


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--args", default="", help="UHD device args")
    ap.add_argument("--channels", default="36,40,44,48")
    ap.add_argument("--center", type=float, default=0.0,
                    help="RX sense center Hz (0 = hopset midpoint)")
    ap.add_argument("--rate", type=float, default=60e6,
                    help="wideband sense rate (must span the hopset; 61.44 Msps "
                         "covers a 60 MHz 5 GHz hopset like 36..48)")
    ap.add_argument("--jam-rate", type=float, default=20e6)
    ap.add_argument("--rx-gain", type=float, default=45.0)
    ap.add_argument("--tx-gain", type=float, default=89.0)
    ap.add_argument("--amplitude", type=float, default=0.9)
    ap.add_argument("--jam-signal", default="cw", choices=("cw", "noise"),
                    help="cw = concentrated tone (strongest denial, matches the "
                         "parked-jammer calibration); noise = band-filling")
    ap.add_argument("--nfft", type=int, default=512)
    ap.add_argument("--min-snr-db", type=float, default=6.0)
    ap.add_argument("--predict", default="off", choices=("off", "seq"))
    ap.add_argument("--secs", type=float, default=20.0)
    ap.add_argument("--log", default="/tmp/devourer-follow.jsonl")
    args = ap.parse_args(argv)

    import uhd

    chans = [int(c) for c in args.channels.split(",")]
    n = len(chans)
    freqs = [chan_to_freq(c) for c in chans]
    center = args.center or (min(freqs) + max(freqs)) / 2.0
    nfft = args.nfft
    bin_hz = args.rate / nfft
    ch_bin = [int(round((f - center) / bin_hz)) + nfft // 2 for f in freqs]
    win = np.hanning(nfft).astype(np.float32)
    for c, b in zip(chans, ch_bin):
        if not (0 <= b < nfft):
            sys.stderr.write(
                f"ch{c} maps to bin {b} outside [0,{nfft}) — raise --rate to "
                f"span the hopset\n")
            return 2

    u = uhd.usrp.MultiUSRP(args.args)
    u.set_rx_rate(args.rate)
    u.set_tx_rate(args.jam_rate)
    u.set_rx_gain(args.rx_gain)
    u.set_tx_gain(args.tx_gain)
    u.set_rx_freq(uhd.types.TuneRequest(center))
    try:
        u.set_rx_antenna("RX2")
        u.set_tx_antenna("TX/RX")
    except Exception:
        pass
    rxs = u.get_rx_stream(uhd.usrp.StreamArgs("fc32", "sc16"))
    txs = u.get_tx_stream(uhd.usrp.StreamArgs("fc32", "sc16"))

    if args.jam_signal == "cw":
        # A tone at +2.5 MHz within the jammed channel — deterministic, all its
        # power in one bin (the parked-jammer denial used CW too).
        t = np.arange(60000) / args.jam_rate
        jam = (args.amplitude * np.exp(2j * np.pi * 2.5e6 * t)).astype(np.complex64)
    else:
        jam = (args.amplitude *
               (np.random.default_rng(0xBEEF).standard_normal(20000) +
                1j * np.random.default_rng(0xF00D).standard_normal(20000))
               / np.sqrt(2)).astype(np.complex64)

    # Single-threaded full-duplex: send a jam burst, then sense, then (if the TX
    # moved) retune — all in one thread. UHD's MultiUSRP is NOT safe for a
    # concurrent tune while another thread streams (it aborts on an internal
    # lcm-overflow assert), so control-plane ops stay serialized here. The RX
    # and TX frontends still run simultaneously; only the python calls serialize.
    stop = {"flag": False}
    stop_h = lambda *_: stop.update(flag=True)
    signal.signal(signal.SIGINT, stop_h)
    signal.signal(signal.SIGTERM, stop_h)

    tx_md = uhd.types.TXMetadata()
    tx_md.start_of_burst = True
    tx_md.end_of_burst = False
    u.set_tx_freq(uhd.types.TuneRequest(freqs[0]))
    jam_idx = 0
    retune_us = 0

    md = uhd.types.RXMetadata()
    buf = np.zeros(nfft, dtype=np.complex64)
    cmd = uhd.types.StreamCMD(uhd.types.StreamMode.num_done)
    cmd.num_samps = nfft
    cmd.stream_now = True
    log_fh = open(args.log, "w")
    sys.stderr.write(
        f"[follower] FULL-DUPLEX center={center/1e6:.1f}MHz hopset={chans} "
        f"predict={args.predict} tx_gain={args.tx_gain}\n")
    sys.stderr.flush()

    t0 = time.monotonic()
    nj = 0
    while not stop["flag"] and time.monotonic() - t0 < args.secs:
        # Radiate a jam burst on the current target (full-duplex with the sense).
        txs.send(jam, tx_md)
        tx_md.start_of_burst = False
        rxs.issue_stream_cmd(cmd)
        got = 0
        while got < nfft:
            k = rxs.recv(buf[got:], md, 0.1)
            if k == 0:
                break
            got += k
        if got < nfft:
            continue
        psd = np.abs(np.fft.fftshift(np.fft.fft(buf * win))) ** 2
        med = float(np.median(psd)) + 1e-20
        jam_now = jam_idx
        # Strongest hopset channel that ISN'T the one we're jamming (our own TX
        # dominates its channel) -> that's the TX to chase.
        best, best_ex = -1, args.min_snr_db
        for ci, b in enumerate(ch_bin):
            if ci == jam_now:
                continue
            lo, hi = max(0, b - 6), min(nfft, b + 6)
            ex = 10.0 * np.log10(float(psd[lo:hi].max()) / med)
            if ex > best_ex:
                best_ex, best = ex, ci
        if best < 0:
            continue
        target = (best + 1) % n if args.predict == "seq" else best
        if target != jam_now:
            t = time.monotonic()
            u.set_tx_freq(uhd.types.TuneRequest(freqs[target]))
            retune_us = int((time.monotonic() - t) * 1e6)
            jam_idx = target
            nj += 1
            log_fh.write(json.dumps({
                "ev": "follow.jam", "t_us": int((time.monotonic() - t0) * 1e6),
                "sense_ch": chans[best], "jam_ch": chans[target],
                "retune_us": retune_us}) + "\n")

    log_fh.close()
    dt = time.monotonic() - t0
    sys.stderr.write(
        f"[follower] {nj} retunes in {dt:.1f}s, last retune {retune_us}us\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
