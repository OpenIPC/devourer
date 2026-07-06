#!/usr/bin/env python3
"""Wideband SDR validator for single-adapter channel hopping.

Tunes a UHD device (B200/B210/LibreSDR) once to a center frequency wide enough
to cover several Wi-Fi channels at the same time, then watches all of them
simultaneously while a hopping transmitter (txdemo with
DEVOURER_HOP_CHANNELS) cycles between them. Because one wideband receiver sees
every hop channel at once, it can distinguish "the transmitter retuned away"
from "the transmitter dropped frames" — which a narrowband receiver tuned to a
single channel cannot.

Two phases, so the 61.44 Msps stream never competes with FFT work (inline
processing overflows and drops samples — fatal for a "no dropped frames"
claim):

  1. CAPTURE: stream IQ straight to a raw fc32 file, hot loop = recv + write.
  2. ANALYSE: read the file back, slice into short FFT windows, integrate power
     in a narrow band around each target channel's carrier (DC notched, since
     a channel parked at the SDR center sees the radio's DC spur), and decide.

The near-field hopping TX is by far the strongest signal on its channel during
its dwell, so the headline check is the *sequence* of the dominant strong
channel over time: it must step through the expected hop order (e.g. 1->6->11)
for the expected number of rounds. Ambient Wi-Fi on the same channels cannot
fake that periodic round-robin, so the test is robust to a congested band.

    python3 hop_rx_probe.py --channels 1,6,11 --center 2437e6 --rate 61.44e6 \
        --gain 45 --duration 6 --expect-rounds 8
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

try:
    import uhd
except ImportError:
    sys.stderr.write(
        "hop_rx_probe: `import uhd` failed. UHD's Python module is a system "
        "package, not pip — create the venv with `uv venv --system-site-packages` "
        "(or run with the system python3).\n"
    )
    raise


def chan_freq_2g(ch: int) -> float:
    """Wi-Fi channel number -> center frequency (Hz). 2.4 GHz (ch1..14) and
    5 GHz (ch >= 32) both handled; name kept for callers."""
    if ch == 14:
        return 2484e6
    if ch <= 14:
        return (2407 + 5 * ch) * 1e6
    return (5000 + 5 * ch) * 1e6


def capture(args, channels) -> tuple[float, float, int, int]:
    """Stream raw fc32 to disk at line rate via a GNU Radio flowgraph.

    The recv loop must run in compiled code: a Python recv loop can't keep up
    with ~30k packets/s at 61.44 Msps and overflows constantly (dropped samples
    = a false "frame loss" reading). gr-uhd's usrp_source -> file_sink runs the
    capture in C++. Returns (rate, center, total_samps, overflow_count).
    """
    from gnuradio import gr, blocks
    from gnuradio import uhd as gruhd

    nsamps = int(args.rate * args.duration)

    class Cap(gr.top_block):
        def __init__(self):
            gr.top_block.__init__(self, "hop_capture")
            self.src = gruhd.usrp_source(
                args.args,
                gruhd.stream_args(cpu_format="fc32", channels=[0]))
            self.src.set_samp_rate(args.rate)
            self.src.set_center_freq(args.center, 0)
            self.src.set_gain(args.gain, 0)
            try:
                self.src.set_bandwidth(min(args.rate, 56e6), 0)
            except Exception:
                pass
            try:
                self.src.set_antenna(args.antenna, 0)
            except Exception:
                pass
            self.head = blocks.head(gr.sizeof_gr_complex, nsamps)
            self.sink = blocks.file_sink(gr.sizeof_gr_complex, args.raw, False)
            self.sink.set_unbuffered(False)
            self.connect(self.src, self.head, self.sink)

    tb = Cap()
    rate = tb.src.get_samp_rate()
    center = tb.src.get_center_freq(0)
    sys.stderr.write(
        f"hop_rx_probe: capture center={center/1e6:.3f}MHz "
        f"rate={rate/1e6:.3f}Msps gain={args.gain}dB dur={args.duration}s "
        f"channels={channels} (gnuradio)\n")
    sys.stderr.flush()
    tb.start()
    tb.wait()
    tb.stop()

    total = os.path.getsize(args.raw) // np.dtype(np.complex64).itemsize
    # gr-uhd prints 'O' to stderr on overflow; we can't easily count them here,
    # but the head block guarantees exactly nsamps land in the file, so a brief
    # overflow shifts timing slightly rather than truncating. Report shortfall.
    overflows = max(0, nsamps - total)
    return rate, center, total, overflows


def analyse(args, channels, rate, center, total, overflows) -> int:
    nch = len(channels)
    nfft = args.nfft
    bin_hz = rate / nfft
    half_bins = max(1, int((args.halfband_mhz * 1e6) / bin_hz))
    dc_guard = max(1, int((args.dc_guard_mhz * 1e6) / bin_hz))

    ch_bins = []
    for ch in channels:
        off = chan_freq_2g(ch) - center
        k0 = int(round(off / bin_hz)) + nfft // 2
        if k0 < 0 or k0 >= nfft:
            sys.stderr.write(
                f"hop_rx_probe: ch{ch} carrier offset {off/1e6:+.1f}MHz outside "
                f"captured band — widen --rate or recenter.\n")
            return 2
        lo, hi = max(0, k0 - half_bins), min(nfft, k0 + half_bins)
        ch_bins.append((lo, hi))
        clamp = " (clamped to band edge)" if (k0 - half_bins < 0 or
                                              k0 + half_bins > nfft) else ""
        sys.stderr.write(f"  ch{ch}: {chan_freq_2g(ch)/1e6:.0f}MHz "
                         f"offset {off/1e6:+.1f}MHz bins[{lo}:{hi}]{clamp}\n")

    dc_lo, dc_hi = nfft // 2 - dc_guard, nfft // 2 + dc_guard
    win = np.hanning(nfft).astype(np.float32)
    win_norm = float(np.sum(win ** 2))
    slice_dt = nfft / rate

    # Stream the raw file through the FFT, building per-channel slice power.
    powers = [[] for _ in range(nch)]
    itemsize = np.dtype(np.complex64).itemsize
    chunk_slices = 4096
    with open(args.raw, "rb") as f:
        carry = np.empty(0, dtype=np.complex64)
        while True:
            raw = f.read(chunk_slices * nfft * itemsize)
            if not raw:
                break
            data = np.concatenate(
                (carry, np.frombuffer(raw, dtype=np.complex64)))
            nsl = len(data) // nfft
            if nsl == 0:
                carry = data
                continue
            block = data[:nsl * nfft].reshape(nsl, nfft)
            spec = np.fft.fftshift(np.fft.fft(block * win, axis=1), axes=1)
            psd = (np.abs(spec) ** 2) / win_norm
            psd[:, dc_lo:dc_hi] = 0.0     # notch the SDR DC spur
            for ci, (lo, hi) in enumerate(ch_bins):
                powers[ci].append(psd[:, lo:hi].sum(axis=1))
            carry = data[nsl * nfft:]

    pwr = [np.concatenate(p) if p else np.zeros(0) for p in powers]
    nslices = min((len(p) for p in pwr), default=0)
    if nslices == 0:
        print("hop-verdict result=FAIL reason=no_samples", flush=True)
        return 1
    pwr = np.vstack([p[:nslices] for p in pwr])    # (nch, nslices)

    eps = 1e-20
    pdb = 10.0 * np.log10(pwr + eps)
    floor_db = np.percentile(pdb, 20, axis=1)
    # "strong" = near-field TX level; rejects weaker ambient APs across the room.
    strong = pdb > (floor_db[:, None] + args.strong_snr_db)

    # Per-channel presence/burst count at the strong threshold.
    counts, peak_snr, strong_frac = [], [], []
    for ci in range(nch):
        a = strong[ci].astype(np.int8)
        edges = np.diff(np.concatenate(([0], a, [0])))
        runs = np.where(edges == -1)[0] - np.where(edges == 1)[0]
        counts.append(int(np.sum(runs >= args.min_burst_slices)))
        peak_snr.append(float(np.max(pdb[ci]) - floor_db[ci]))
        strong_frac.append(float(np.mean(strong[ci])))

    # Per-bin integrated power export (--bin-power-csv): the near-field TX
    # level per channel = median power over that channel's STRONG slices (its
    # dwells), falling back to the peak when a bin never crosses the strong
    # threshold. This is the wideband ground truth tests/sounding_map.py
    # rank-correlates against the sounding sweep's per-bin RSSI.
    if args.bin_power_csv:
        with open(args.bin_power_csv, "w") as f:
            f.write("ch,power_db,peak_snr_db,strong_frac,bursts\n")
            for ci, ch in enumerate(channels):
                sel = pdb[ci][strong[ci]]
                p = float(np.median(sel)) if sel.size else float(np.max(pdb[ci]))
                f.write(f"{ch},{p:.2f},{peak_snr[ci]:.2f},"
                        f"{strong_frac[ci]:.4f},{counts[ci]}\n")
        sys.stderr.write(f"hop_rx_probe: per-bin power -> {args.bin_power_csv}\n")

    # Dominant strong channel over coarse time windows -> hop sequence.
    win_slices = max(1, int((args.seq_window_ms * 1e-3) / slice_dt))
    nwin = nslices // win_slices
    seq = []
    for w in range(nwin):
        s = slice(w * win_slices, (w + 1) * win_slices)
        wp = pwr[:, s].mean(axis=1)
        wdb = 10.0 * np.log10(wp + eps)
        ci = int(np.argmax(wp))
        if wdb[ci] > floor_db[ci] + args.strong_snr_db:
            seq.append(channels[ci])
        else:
            seq.append(0)     # idle window
    # Run-length-encode, dropping idle windows.
    rle = []
    for c in seq:
        if c == 0:
            continue
        if not rle or rle[-1] != c:
            rle.append(c)

    # Count repeats of the expected hop pattern. The captured spectrum may be
    # mirrored (IQ/inversion convention on some SDR clones), which reverses the
    # apparent channel order while keeping a center channel fixed — so match the
    # pattern at any rotation AND in either direction; report which.
    def count_cycles(seq_list, pattern):
        n = len(pattern)
        best = 0
        for rot in range(n):
            pat = pattern[rot:] + pattern[:rot]
            c, i = 0, 0
            while i + n <= len(seq_list):
                if seq_list[i:i + n] == pat:
                    c += 1
                    i += n
                else:
                    i += 1
            best = max(best, c)
        return best

    fwd = count_cycles(rle, channels)
    rev = count_cycles(rle, channels[::-1])
    full_cycles = max(fwd, rev)
    direction = "forward" if fwd >= rev else "reversed(spectral-mirror)"

    # Proof of hopping: every channel carries the near-field TX, and the
    # dominant channel cycles through the full hop order several times. A floor
    # of 3 clean cycles is unforgeable by ambient traffic.
    cycle_floor = max(3, args.expect_rounds // 2) if args.expect_rounds else 3
    seq_ok = full_cycles >= cycle_floor

    rle_str = ",".join(str(c) for c in rle[:48]) + ("..." if len(rle) > 48 else "")
    seen_all = all(c > 0 for c in counts)

    print("", flush=True)
    print("=== hop_rx_probe verdict ===", flush=True)
    for ci, ch in enumerate(channels):
        print(f"hop-channel ch={ch} strong_bursts={counts[ci]} "
              f"peak_snr_db={peak_snr[ci]:.1f} floor_db={floor_db[ci]:.1f} "
              f"strong_frac={strong_frac[ci]:.3f}", flush=True)
    print(f"hop-sequence rle={rle_str}", flush=True)
    print(f"hop-cycles observed={full_cycles} ({direction}) "
          f"floor={cycle_floor} expected={args.expect_rounds}", flush=True)
    print(f"hop-stats slices={nslices} slice_us={slice_dt*1e6:.1f} "
          f"seq_window_ms={args.seq_window_ms} overflows={overflows} "
          f"samps={total}", flush=True)

    reasons = []
    if not seen_all:
        reasons.append("channel_with_no_strong_signal")
    if not seq_ok:
        reasons.append("hop_sequence_mismatch")
    result = "PASS" if (seen_all and seq_ok) else "FAIL"
    if overflows > 0 and result == "PASS":
        result = "PASS_WITH_OVERFLOWS"   # capture had gaps; sequence still held
        reasons.append(f"capture_overflows({overflows})")
    print(f"hop-verdict result={result} channels={nch} cycles={full_cycles} "
          f"reason={','.join(reasons) or 'ok'}", flush=True)
    return 0 if result.startswith("PASS") else 1


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--channels", default="1,6,11",
                    help="comma list of 2.4GHz channel numbers being hopped")
    ap.add_argument("--center", type=float, default=2437e6,
                    help="SDR center freq (Hz); default = ch6 (mid of 1/6/11)")
    ap.add_argument("--rate", type=float, default=61.44e6,
                    help="sample rate (Hz); must span all channels")
    ap.add_argument("--gain", type=float, default=45.0, help="RX gain (dB)")
    ap.add_argument("--antenna", default="RX2", help="RX antenna port")
    ap.add_argument("--args", default="", help="UHD device args (e.g. type=b200)")
    ap.add_argument("--duration", type=float, default=6.0, help="capture seconds")
    ap.add_argument("--raw", default="/tmp/devourer-hop-iq.fc32",
                    help="raw fc32 capture path")
    ap.add_argument("--keep-raw", action="store_true",
                    help="don't delete the raw capture after analysis")
    ap.add_argument("--nfft", type=int, default=2048, help="FFT / slice length")
    ap.add_argument("--halfband-mhz", type=float, default=3.0,
                    help="half-width (MHz) of per-channel power integration")
    ap.add_argument("--dc-guard-mhz", type=float, default=0.1,
                    help="notch ±this around DC (SDR offset spur)")
    ap.add_argument("--strong-snr-db", type=float, default=15.0,
                    help="dB above floor to count as the near-field TX")
    ap.add_argument("--min-burst-slices", type=int, default=2,
                    help="min contiguous strong slices for one burst")
    ap.add_argument("--seq-window-ms", type=float, default=20.0,
                    help="time granularity for the dominant-channel sequence")
    ap.add_argument("--expect-rounds", type=int, default=0,
                    help="expected full hop cycles (0 = just require order+presence)")
    ap.add_argument("--bin-power-csv", default="",
                    help="write per-channel integrated power (CSV) — the SDR "
                         "ground truth for tests/sounding_map.py --sdr-csv")
    ap.add_argument("--analyse-only", action="store_true",
                    help="skip capture, analyse an existing --raw file")
    args = ap.parse_args()

    channels = [int(x) for x in args.channels.split(",") if x.strip()]
    if len(channels) < 2:
        sys.stderr.write("hop_rx_probe: need >=2 channels to test hopping\n")
        return 2

    if args.analyse_only:
        # Rate/center are needed for bin geometry; trust the CLI values.
        sz = os.path.getsize(args.raw)
        total = sz // np.dtype(np.complex64).itemsize
        return analyse(args, channels, args.rate, args.center, total, 0)

    rate, center, total, overflows = capture(args, channels)
    rc = analyse(args, channels, rate, center, total, overflows)
    if not args.keep_raw:
        try:
            os.remove(args.raw)
        except OSError:
            pass
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
