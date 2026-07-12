#!/usr/bin/env python3
"""SDR cross-check: chip LA-capture H(k) vs a B210-derived estimate.

Two subcommands:

  capture  — record raw complex64 IQ from the B210 at 20 Msps on a Wi-Fi
             channel (the same grid the chip's LA capture uses):
                 sudo python3 la_sdr_compare.py capture --channel 6 \
                     --secs 0.5 --out /tmp/sdr.c64
  compare  — run the SAME L-LTF pipeline (tools/la_csi.py) over both
             captures and correlate per-tone |H(k)|:
                 python3 la_sdr_compare.py compare --sdr /tmp/sdr.c64 \
                     --chip /tmp/la.bin [--min-corr 0.9]

The SDR stream holds many frames; every detected L-LTF yields one
estimate and the per-tone MEDIAN |H| is the SDR-side answer (robust to
the odd foreign beacon on a quiet bench channel). The chip capture is a
one-shot crcok trigger, so it holds the one frame that fired it.

Interpretation caveat (bench truth, MEASURED): the two receivers sit at
different antennas and see INDEPENDENT frequency-selective fading —
chip-vs-SDR |H(k)| correlation over the air came out ~0 while each side
is strongly self-consistent (chip-vs-chip 0.70 across captures of mixed
transmitters; SDR same-TX cluster 0.9). The `compare` subcommand remains
as the diagnostic that measured this; the DEFINITIVE cross-validation is
the notch protocol below.

Notch protocol (txltf / check-notch): the B210 transmits repeated L-LTF
bursts with a chosen set of tones ZEROED — a deep deterministic spectral
feature that multipath cannot mimic or mask. The chip captures with a
CCA trigger and its H(k) must show those exact tones >= --min-notch dB
below the median of the untouched tones. Asymmetric notch sets also
catch tone-mapping / spectral-inversion errors end to end.

    sudo python3 la_sdr_compare.py txltf --channel 6 \
        --notch=-21,-20,-19,7,8,9 --secs 40 &
    DEVOURER_LA_CAPTURE=cca/20M/dma0/port:0x880/t100 build/rxdemo ...
    python3 la_sdr_compare.py check-notch --chip /tmp/la.bin \
        --notch=-21,-20,-19,7,8,9
"""

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "tools"))
import la_csi  # noqa: E402
import la_decode  # noqa: E402


def chan_to_freq(ch):
    return (2407 + 5 * ch if ch <= 14 else 5000 + 5 * ch) * 1e6


def do_capture(args):
    import uhd
    usrp = uhd.usrp.MultiUSRP(args.args)
    rate = 20e6
    usrp.set_rx_rate(rate)
    usrp.set_rx_freq(uhd.types.TuneRequest(chan_to_freq(args.channel)))
    usrp.set_rx_gain(args.gain)
    usrp.set_rx_antenna("RX2")
    st_args = uhd.usrp.StreamArgs("fc32", "sc16")
    st_args.channels = [0]
    rx = usrp.get_rx_stream(st_args)
    md = uhd.types.RXMetadata()
    buf = np.zeros((1, rx.get_max_num_samps()), dtype=np.complex64)
    nsamp = int(rate * args.secs)
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
    out.tofile(args.out)
    print(f"wrote {nsamp} samples -> {args.out}", file=sys.stderr)
    return 0


def h_estimates(x, max_ltfs=40, chunk=65536):
    """Every L-LTF found in the stream -> list of |H| arrays (52 tones)."""
    nz = la_csi.LTF_SEQ != 0
    ests = []
    i = 0
    while i + 8192 < len(x) and len(ests) < max_ltfs:
        seg = x[i:i + chunk]
        start, cfo = la_csi.locate_lltf(seg)
        if start is None:
            i += chunk - 256
            continue
        if start + 128 <= len(seg):
            h = la_csi.estimate_h(seg, start, cfo)
            mag = np.abs(h[nz])
            if mag.max() > 0:
                ests.append(mag / np.median(mag))  # scale-free shape
        i += start + 160  # skip past this LTF, keep scanning
    return ests


def do_compare(args):
    nz = la_csi.LTF_SEQ != 0
    tones = la_csi.TONES[nz]

    # Chip side: one-shot LA capture.
    meta, words = la_decode.read_dump(args.chip)
    xc = la_decode.LAYOUTS[args.layout][1](words)
    xc = xc - np.mean(xc)
    start, cfo = la_csi.locate_lltf(xc)
    if start is None:
        print("FAIL: no L-LTF in the chip capture", file=sys.stderr)
        return 1
    hc = np.abs(la_csi.estimate_h(xc, start, cfo)[nz])
    hc = hc / np.median(hc)

    # SDR side: median shape over every detected LTF.
    xs = np.fromfile(args.sdr, dtype=np.complex64)
    xs = xs - np.mean(xs)
    ests = h_estimates(xs)
    if not ests:
        print("FAIL: no L-LTF in the SDR capture", file=sys.stderr)
        return 1
    hs = np.median(np.array(ests), axis=0)

    hc_db = 20 * np.log10(hc + 1e-9)
    hs_db = 20 * np.log10(hs + 1e-9)
    corr = float(np.corrcoef(hc_db, hs_db)[0, 1])
    rmse = float(np.sqrt(np.mean((hc_db - hs_db) ** 2)))

    for k, c, s in zip(tones, hc_db, hs_db):
        print(json.dumps({"tone": int(k), "chip_db": round(float(c), 2),
                          "sdr_db": round(float(s), 2)}))
    print(f"# chip LTF@{start}, SDR LTFs used: {len(ests)}", file=sys.stderr)
    print(f"# |H(k)| dB correlation = {corr:.3f}, rmse = {rmse:.2f} dB",
          file=sys.stderr)
    ok = corr >= args.min_corr
    print(f"# {'PASS' if ok else 'FAIL'} (threshold {args.min_corr})",
          file=sys.stderr)
    return 0 if ok else 1


def parse_notch(s):
    return [int(t) for t in s.split(",") if t]


def do_txltf(args):
    """B210: repeated L-LTF bursts with the notched tones zeroed."""
    import uhd
    notch = set(parse_notch(args.notch))
    f = la_csi.ltf_freq64().copy()
    for k in notch:
        f[k % 64] = 0
    ref = np.fft.ifft(f)
    # The settling preamble IS the LTF: 10 repeated periods (~32 us). The
    # RX AGC slews during the first few; any interior 128-sample window
    # is a valid estimation window (the train is 64-periodic), and the
    # locator naturally picks the cleanest one. (An L-STF preamble was
    # tried and rejected: its 16-sample comb also repeats at lag 64 and
    # its tone set (±4k) aliases straight into the estimate.)
    burst = np.tile(ref, 10)
    one = np.concatenate([burst, np.zeros(2048, dtype=complex)])
    one = args.amplitude * one / np.max(np.abs(one))
    # Long precomputed buffer (~40 bursts / 4.6 ms per send) — per-burst
    # send calls underrun the B210 from python and nothing airs.
    frame = np.tile(one, 40).astype(np.complex64)
    usrp = uhd.usrp.MultiUSRP(args.args)
    rate = 20e6
    usrp.set_tx_rate(rate)
    usrp.set_tx_freq(uhd.types.TuneRequest(chan_to_freq(args.channel)))
    usrp.set_tx_gain(args.gain)
    st_args = uhd.usrp.StreamArgs("fc32", "sc16")
    st_args.channels = [0]
    tx = usrp.get_tx_stream(st_args)
    md = uhd.types.TXMetadata()
    md.start_of_burst = True
    reps = int(args.secs * rate / len(frame))
    print(f"txltf: ch {args.channel}, notch {sorted(notch)}, "
          f"{reps} bursts", file=sys.stderr)
    for _ in range(reps):
        tx.send(frame, md)
        md.start_of_burst = False
    md.end_of_burst = True
    tx.send(np.zeros(0, dtype=np.complex64), md)
    return 0


def do_check_notch(args):
    notch = parse_notch(args.notch)
    nz = la_csi.LTF_SEQ != 0
    tones = la_csi.TONES[nz]
    meta, words = la_decode.read_dump(args.chip)
    x = la_decode.LAYOUTS[args.layout][1](words)
    x = x - np.mean(x)
    # Average |H| over every LTF burst in the ring (txltf repeats one
    # burst every ~114 us, so an 8 K-sample capture holds several) —
    # noise-limited notch depth improves with each burst averaged.
    mags = []
    start0 = cfo0 = None
    chunk = 2400  # one txltf burst period (~2288) + margin
    for i in range(0, max(1, len(x) - 192), chunk - 300):
        seg = x[i:i + chunk]
        start, cfo = la_csi.locate_lltf(seg, min_metric=0.5)
        if start is None or i + start + 128 > len(x):
            continue
        # txltf transmits an LTF *train* — refine CFO over every clean
        # repetition (a 2-symbol estimate's few-kHz noise leaks ~-8 dB
        # into the tones adjacent to a deep notch).
        cfo_r = la_csi.refine_cfo(seg, start)
        if cfo_r is not None:
            cfo = cfo_r
        if start0 is None:
            start0, cfo0 = i + start, cfo
        mags.append(np.abs(la_csi.estimate_h(seg, start, cfo)[nz]))
    if not mags:
        print("FAIL: no L-LTF in the chip capture", file=sys.stderr)
        return 1
    start, cfo = start0, cfo0
    h_db = 20 * np.log10(np.mean(np.array(mags), axis=0) + 1e-9)
    is_notch = np.isin(tones, notch)
    # Depth vs the LOCAL neighbours (nearest clean tones on each side):
    # genuine multipath ripple is smooth across adjacent tones (delay
    # spread << symbol time) while the imposed TX notch is not — a global
    # median reference conflates real channel dips with the notch.
    depth = []
    for idx in np.where(is_notch)[0]:
        neigh = [j for j in range(max(0, idx - 3), min(len(tones), idx + 4))
                 if not is_notch[j]]
        depth.append(float(np.mean(h_db[neigh])) - h_db[idx])
    depth = np.array(depth)
    for k, d in zip(tones[is_notch], depth):
        print(json.dumps({"tone": int(k), "notch_depth_db": round(float(d), 1)}))
    ok = bool(np.all(depth >= args.min_notch))
    # Context: the clean tones' own ripple (real channel + noise).
    clean_med = float(np.median(h_db[~is_notch]))
    clean_min = float(np.min(h_db[~is_notch]))
    print(f"# LTF@{start} cfo={cfo * 20e3:+.1f} kHz bursts={len(mags)} | "
          f"notch depth min={depth.min():.1f} dB (need >= {args.min_notch}), "
          f"clean-tone ripple={clean_med - clean_min:.1f} dB", file=sys.stderr)
    print(f"# {'PASS' if ok else 'FAIL'}", file=sys.stderr)
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("capture")
    c.add_argument("--channel", type=int, default=6)
    c.add_argument("--secs", type=float, default=0.5)
    c.add_argument("--gain", type=float, default=50)
    c.add_argument("--args", default="")
    c.add_argument("--out", required=True)
    m = sub.add_parser("compare")
    m.add_argument("--sdr", required=True, help="complex64 file (capture)")
    m.add_argument("--chip", required=True, help="DVLA dump")
    m.add_argument("--layout", default=la_decode.DEFAULT_LAYOUT)
    m.add_argument("--min-corr", type=float, default=0.9)
    t = sub.add_parser("txltf")
    t.add_argument("--channel", type=int, default=6)
    t.add_argument("--notch", required=True,
                   help="comma-separated tone indices to zero, e.g. -21,-20,7")
    t.add_argument("--secs", type=float, default=40)
    t.add_argument("--gain", type=float, default=60)
    t.add_argument("--amplitude", type=float, default=0.5)
    t.add_argument("--args", default="")
    n = sub.add_parser("check-notch")
    n.add_argument("--chip", required=True)
    n.add_argument("--notch", required=True)
    n.add_argument("--layout", default=la_decode.DEFAULT_LAYOUT)
    n.add_argument("--min-notch", type=float, default=10.0)
    args = ap.parse_args()
    sys.exit({"capture": do_capture, "compare": do_compare,
              "txltf": do_txltf, "check-notch": do_check_notch}[args.cmd](args))


if __name__ == "__main__":
    main()
