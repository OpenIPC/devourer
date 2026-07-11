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
With --seed the expected order is the keyed SipHash schedule instead of a fixed
round-robin; the aligner tolerates dropped dwells (a hop channel at the band
edge of the wide capture can roll off below the strong threshold — the same
slot the two-adapter link decodes fine) and spurious windows, so a single weak
dwell no longer sinks the whole match.

    python3 hop_rx_probe.py --channels 1,6,11 --center 2437e6 --rate 61.44e6 \
        --gain 45 --duration 6 --expect-rounds 8
    python3 hop_rx_probe.py --self-test   # offline aligner check, no SDR/UHD
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

def _rotl(x, b): return ((x << b) | (x >> (64-b))) & 0xffffffffffffffff
def siphash24(key: bytes, msg: bytes) -> int:
    def rd(p): return int.from_bytes(p, "little")
    k0,k1=rd(key[:8]),rd(key[8:]); v0=0x736f6d6570736575^k0;v1=0x646f72616e646f6d^k1;v2=0x6c7967656e657261^k0;v3=0x7465646279746573^k1
    def rnd():
        nonlocal v0,v1,v2,v3
        v0=(v0+v1)&0xffffffffffffffff;v1=_rotl(v1,13)^v0;v0=_rotl(v0,32);v2=(v2+v3)&0xffffffffffffffff;v3=_rotl(v3,16)^v2;v0=(v0+v3)&0xffffffffffffffff;v3=_rotl(v3,21)^v0;v2=(v2+v1)&0xffffffffffffffff;v1=_rotl(v1,17)^v2;v2=_rotl(v2,32)
    end=len(msg)&~7
    for off in range(0,end,8):
        m=rd(msg[off:off+8]);v3^=m;rnd();rnd();v0^=m
    b=len(msg)<<56
    for i,x in enumerate(msg[end:]): b|=x<<(8*i)
    v3^=b;rnd();rnd();v0^=b;v2^=0xff
    for _ in range(4):rnd()
    return v0^v1^v2^v3

def keyed_sequence(seed: str, channels, rounds: int):
    s=seed[2:] if seed.lower().startswith("0x") else seed
    if not 1 <= len(s) <= 32: raise ValueError("seed must be 1..32 hex digits")
    key=bytes.fromhex(s.zfill(32)); out=[]
    for r in range(rounds):
        p=list(range(len(channels))); counter=0
        for i in range(len(p),1,-1):
            limit=0xffffffffffffffff-(0xffffffffffffffff%i)
            while True:
                x=siphash24(key,b"H"+r.to_bytes(8,"little")+counter.to_bytes(8,"little"));counter+=1
                if x<limit:break
            j=x%i;p[i-1],p[j]=p[j],p[i-1]
        out.extend(channels[i] for i in p)
    return out


def keyed_align(rle, expected, nch, drop_tol=None, ins_tol=2):
    """Align an observed dwell sequence to the keyed schedule, tolerant to
    dropped and spurious dwells.

    The wideband capture legitimately loses dwells: a hop channel near the
    band edge of the ~61 Msms capture sits in the SDR's anti-alias / front-end
    roll-off, so its dwell can fall below the strong-signal threshold and get
    RLE'd to idle — a *dropped* dwell, not a schedule violation (the two-adapter
    link test decodes >99% of those same slots). A momentary ambient win inserts
    a *spurious* dwell. Both must be tolerated, or a single glitch truncates the
    entire match.

    So instead of an exact contiguous prefix anchored at rle[0] (which any early
    drop kills), slide over every start offset in `expected` and greedily walk,
    resyncing on a mismatch by skipping up to `drop_tol` expected entries (a run
    of dropped dwells) or up to `ins_tol` observed entries (spurious windows).
    Returns (matched, span, offset): matched dwells, expected-index span
    covered, and the winning offset.
    """
    if drop_tol is None:
        drop_tol = nch                     # a whole round of drops still resyncs
    best = (0, 0, 0)
    ne = len(expected)
    for off in range(max(1, ne - nch)):
        ei, ri, matched = off, 0, 0
        while ri < len(rle) and ei < ne:
            if rle[ri] == expected[ei]:
                matched += 1; ri += 1; ei += 1
                continue
            # resync: is this observed dwell a later expected dwell (things were
            # dropped between), or is it spurious (skip it)? Prefer the smaller
            # jump so a genuine drop isn't misread as a long spurious run.
            skip_e = next((d for d in range(1, drop_tol + 1)
                           if ei + d < ne and rle[ri] == expected[ei + d]), None)
            skip_r = next((d for d in range(1, ins_tol + 1)
                           if ri + d < len(rle) and rle[ri + d] == expected[ei]),
                          None)
            if skip_e is not None and (skip_r is None or skip_e <= skip_r):
                ei += skip_e
            elif skip_r is not None:
                ri += skip_r
            else:
                break                      # lost sync — this offset is done
        if matched > best[0]:
            best = (matched, ei - off, off)
    return best


def mirror_map(channels):
    """Spectral-mirror channel involution. Some SDR clones present an
    IQ-inverted spectrum, reflecting channels about the capture center; sorting
    by carrier frequency and swapping rank i <-> n-1-i is that map. Applied to
    the observed dwells so the keyed order still lines up under inversion."""
    order = sorted(range(len(channels)), key=lambda i: chan_freq_2g(channels[i]))
    return {channels[order[r]]: channels[order[len(order) - 1 - r]]
            for r in range(len(order))}


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
    try:
        from gnuradio import gr, blocks
        from gnuradio import uhd as gruhd
    except ImportError:
        sys.stderr.write(
            "hop_rx_probe: GNU Radio (gr-uhd) not importable. It is a system "
            "package, not pip — create the venv with `uv venv "
            "--system-site-packages` (or run with the system python3). "
            "Analysis-only (--analyse-only) and --self-test do not need it.\n")
        raise

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
        # Use a high percentile, not the mean: the near-field TX is bursty
        # (frames spaced by the inter-frame gap), so a 20 ms window is only
        # part-filled. Averaging dilutes a real dwell below a steady band-edge
        # pedestal (near-field splatter / IQ-image raises the edge channels'
        # floor ~9 dB on the B210), which then wins every window and collapses
        # the sequence. The peak power in the window tracks the dwell instead.
        wp = np.percentile(pwr[:, s], args.seq_pct, axis=1)
        wdb = 10.0 * np.log10(wp + eps)
        # Receiver response and ambient floor differ by channel, especially
        # near a wide capture's band edge. Compare excess above each channel's
        # own floor instead of raw watts or the loudest bin wins every dwell.
        excess_db = wdb - floor_db
        ci = int(np.argmax(excess_db))
        if excess_db[ci] > args.strong_snr_db:
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

    keyed_frac = 1.0
    keyed_span = 0
    if args.seed:
        # Generate enough keyed rounds to cover the whole observed sequence plus
        # slack for the sliding offset, even if the capture ran longer than the
        # expected round count.
        rounds_needed = max(args.expect_rounds + 4, 64, len(rle) // nch + 8)
        expected = keyed_sequence(args.seed, channels, rounds_needed)
        fwd = keyed_align(rle, expected, nch)
        mir = mirror_map(channels)
        rev = keyed_align([mir[c] for c in rle], expected, nch)
        matched, keyed_span, _ = fwd if fwd[0] >= rev[0] else rev
        direction = "keyed" if fwd[0] >= rev[0] else "keyed(spectral-mirror)"
        full_cycles = matched // nch
        keyed_frac = matched / keyed_span if keyed_span else 0.0
    else:
        fwd = count_cycles(rle, channels); rev = count_cycles(rle, channels[::-1])
        full_cycles = max(fwd, rev)
        direction = "forward" if fwd >= rev else "reversed(spectral-mirror)"

    # Proof of hopping: every channel carries the near-field TX, and the
    # dominant channel cycles through the full hop order several times. A floor
    # of 3 clean cycles is unforgeable by ambient traffic.
    cycle_floor = max(3, args.expect_rounds // 2) if args.expect_rounds else 3
    # For the keyed schedule also require the aligned span to be mostly matched,
    # so a long low-quality alignment (chance matches at a wrong offset) can't
    # inflate the cycle count. Dropped/spurious dwells are tolerated up to
    # 1-match_frac of the span.
    frac_ok = (not args.seed) or keyed_frac >= args.match_frac
    seq_ok = full_cycles >= cycle_floor and frac_ok

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
    if args.seed:
        print(f"hop-keyed matched_dwells={full_cycles * nch} span={keyed_span} "
              f"match_frac={keyed_frac:.3f} min_frac={args.match_frac}",
              flush=True)
    print(f"hop-stats slices={nslices} slice_us={slice_dt*1e6:.1f} "
          f"seq_window_ms={args.seq_window_ms} overflows={overflows} "
          f"samps={total}", flush=True)

    reasons = []
    if not seen_all:
        reasons.append("channel_with_no_strong_signal")
    if not seq_ok:
        reasons.append("hop_sequence_mismatch" if frac_ok
                       else f"hop_low_match_fraction({keyed_frac:.2f})")
    result = "PASS" if (seen_all and seq_ok) else "FAIL"
    if overflows > 0 and result == "PASS":
        result = "PASS_WITH_OVERFLOWS"   # capture had gaps; sequence still held
        reasons.append(f"capture_overflows({overflows})")
    print(f"hop-verdict result={result} channels={nch} cycles={full_cycles} "
          f"reason={','.join(reasons) or 'ok'}", flush=True)
    return 0 if result.startswith("PASS") else 1


def self_test() -> int:
    """Offline proof that the keyed aligner survives the failure the SDR run
    hit: weak edge-channel dwells omitted (drops) plus a few misclassified
    windows (spurious/substituted), with and without spectral mirroring. Runs
    without UHD or an SDR. Also shows the old anchored-contiguous matcher fails
    the same input, so this stays a regression guard."""
    seed = "00112233445566778899aabbccddeeff"
    channels = [36, 40, 44, 48]
    nch = len(channels)
    rounds = 12
    truth = keyed_sequence(seed, channels, rounds)

    # Deterministic (no RNG dependence) corruption: drop every 8th dwell (a weak
    # edge dwell omitted) and substitute every 17th with an ambient channel.
    edge = {36, 48}
    observed = []
    for i, c in enumerate(truth):
        if i % 8 == 0 and c in edge:
            continue                                   # dropped weak edge dwell
        observed.append(40 if (i % 17 == 0 and c != 40) else c)
    # RLE, as analyse() does, so adjacent survivors don't merge spuriously.
    rle = []
    for c in observed:
        if not rle or rle[-1] != c:
            rle.append(c)

    fails = 0
    exp = keyed_sequence(seed, channels, rounds + 4)

    matched, span, _ = keyed_align(rle, exp, nch)
    frac = matched / span if span else 0.0
    cyc = matched // nch
    ok = cyc >= rounds - 2 and frac >= 0.6
    print(f"self-test forward: cycles={cyc}/{rounds} matched={matched} "
          f"span={span} frac={frac:.3f} -> {'OK' if ok else 'FAIL'}")
    fails += not ok

    mir = mirror_map(channels)
    rle_m = [mir[c] for c in rle]                       # simulate IQ inversion
    fm, _, _ = keyed_align(rle_m, exp, nch)             # naive forward: poor
    rm, sm, _ = keyed_align([mir[c] for c in rle_m], exp, nch)  # de-mirrored
    fracm = rm / sm if sm else 0.0
    okm = (rm // nch) >= rounds - 2 and fracm >= 0.6 and rm > fm
    print(f"self-test mirror:  demirrored_cycles={rm // nch}/{rounds} "
          f"frac={fracm:.3f} naive_matched={fm} -> {'OK' if okm else 'FAIL'}")
    fails += not okm

    # The OLD matcher: exact contiguous prefix anchored at rle[0]. Must undercount
    # (that's the bug this fix removes) — the first drop truncates it.
    old_best = 0
    for off in range(len(exp)):
        n = 0
        while n < len(rle) and off + n < len(exp) and rle[n] == exp[off + n]:
            n += 1
        old_best = max(old_best, n)
    old_cyc = old_best // nch
    regressed = old_cyc < rounds - 2
    print(f"self-test old-matcher: cycles={old_cyc}/{rounds} "
          f"(expected to undercount) -> {'OK' if regressed else 'FAIL'}")
    fails += not regressed

    print(f"self-test result={'PASS' if not fails else 'FAIL'}")
    return 1 if fails else 0


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
    ap.add_argument("--seq-pct", type=float, default=90.0,
                    help="percentile of in-window power used for the dominant "
                         "channel (high = track the bursty TX peak, not the mean)")
    ap.add_argument("--expect-rounds", type=int, default=0,
                    help="expected full hop cycles (0 = just require order+presence)")
    ap.add_argument("--seed", default="", help="keyed schedule seed (same hex as DEVOURER_HOP_SEED)")
    ap.add_argument("--match-frac", type=float, default=0.6,
                    help="keyed mode: min matched fraction of the aligned span "
                         "(tolerates dropped/spurious dwells up to 1-this)")
    ap.add_argument("--self-test", action="store_true",
                    help="run the offline keyed-aligner self-test and exit "
                         "(no SDR/UHD needed)")
    ap.add_argument("--bin-power-csv", default="",
                    help="write per-channel integrated power (CSV) — the SDR "
                         "ground truth for tests/sounding_map.py --sdr-csv")
    ap.add_argument("--analyse-only", action="store_true",
                    help="skip capture, analyse an existing --raw file")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

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
