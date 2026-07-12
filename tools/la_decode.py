#!/usr/bin/env python3
"""LA-mode capture decoder — unpack + score the packed IQ sample layout.

Input: the "DVLA" dump rxdemo writes for DEVOURER_LA_CAPTURE (32-byte
header + little-endian u64 records, one per 8-byte LA bus word; see
src/LaCapture.h). The vendor C code never documents how I/Q are packed in
the 64-bit word (it depends on dma_type and the chip's dialect), so this
tool carries a set of CANDIDATE layouts and scores them empirically:
capture a CW tone at a known offset (tests/sdr_interferer.py --mode cw)
and the correct layout is the one whose FFT shows a single sharp line at
that offset — wrong bit-slicing scrambles the phase trajectory and smears
the tone into the floor.

    la_decode.py raw     cap.bin              # vendor "%08x%08x" parity dump
    la_decode.py score   cap.bin [--offset-mhz 2.0]
    la_decode.py unpack  cap.bin --layout iq12_l --out iq.c64
    la_decode.py plot    cap.bin --layout iq12_l --out psd.png

Scores: peak-to-median PSD ratio in dB (higher = sharper line = better
candidate); with --offset-mhz the peak must land within ±0.25 MHz of the
expected bin, else the layout scores 0.

CONFIRMED (tests/la_cw_score.sh, B210 CW at +1/+4/+5 MHz): at
dma_type=0 / dbg_port=0x880 every LA-capable chip packs ONE complex
sample per 64-bit word as 12-bit two's complement with I=[11:0],
Q=[23:12] of data_l — layout "qi12_l" — at exactly the configured
smp_rate (20 Msps verified; residual +0.05 MHz = the crystals' CFO).
Scores: 8822BU 23.6 dB, 8814AU 19.1 dB, 8821C 36.8 dB, 8822CU 50.8 dB.
JGR3 (8822C/E) additionally carries the SECOND RX path in data_h as
I=[55:44], Q=[43:32] ("iq12_h", 47.3 dB, opposite I/Q order) — a
two-chain capture in one shot.
"""

DEFAULT_LAYOUT = "qi12_l"

import argparse
import struct
import sys

import numpy as np

RATE_MHZ = {0: 80.0, 1: 40.0, 2: 20.0, 3: 10.0, 4: 5.0, 5: 2.5, 6: 1.25,
            7: 160.0}


def read_dump(path):
    """Return (meta dict, np.uint64 words[])."""
    with open(path, "rb") as f:
        hdr = f.read(32)
        if hdr[:4] != b"DVLA":
            sys.exit(f"{path}: not a DVLA dump")
        meta = {
            "version": hdr[4],
            "trig_mode": hdr[5],
            "mac_sig": hdr[6],
            "smp_rate": hdr[7],
            "dma_type": hdr[8],
            "wrap": hdr[9],
            "n": struct.unpack_from("<I", hdr, 12)[0],
            "finish_addr": struct.unpack_from("<I", hdr, 16)[0],
            "trigger_time_us": struct.unpack_from("<I", hdr, 20)[0],
        }
        meta["fs_mhz"] = RATE_MHZ.get(meta["smp_rate"], 20.0)
        words = np.fromfile(f, dtype="<u8", count=meta["n"])
    return meta, words


def _s(field, bits):
    """Two's-complement sign-extend an unsigned field array."""
    field = field.astype(np.int64)
    sign = 1 << (bits - 1)
    return (field ^ sign) - sign


def _slice(words, lsb, bits):
    return ((words >> np.uint64(lsb)) & np.uint64((1 << bits) - 1))


# Candidate layouts: name -> (description, unpack(words) -> complex64[]).
# The low dword of each u64 is data_l (the word at the 8-byte address), the
# high dword data_h (at +4) — vendor prints "%08x%08x" as {h}{l}.
def _iq(words, i_lsb, q_lsb, bits):
    i = _s(_slice(words, i_lsb, bits), bits).astype(np.float32)
    q = _s(_slice(words, q_lsb, bits), bits).astype(np.float32)
    return (i + 1j * q).astype(np.complex64)


LAYOUTS = {
    # 24-bit-populated low dword (8822B/8814A/8821C at dma0/port 0x880):
    "iq12_l": ("I=[23:12] Q=[11:0] of data_l",
               lambda w: _iq(w, 12, 0, 12)),
    "qi12_l": ("Q=[23:12] I=[11:0] of data_l",
               lambda w: _iq(w, 0, 12, 12)),
    "iq11_l": ("I=[21:11] Q=[10:0] of data_l",
               lambda w: _iq(w, 11, 0, 11)),
    "qi11_l": ("Q=[21:11] I=[10:0] of data_l",
               lambda w: _iq(w, 0, 11, 11)),
    "iq10_l": ("I=[19:10] Q=[9:0] of data_l",
               lambda w: _iq(w, 10, 0, 10)),
    # High-dword variants (JGR3 populates both dwords):
    "iq12_h": ("I=[55:44] Q=[43:32] (data_h)",
               lambda w: _iq(w, 44, 32, 12)),
    "qi12_h": ("Q=[55:44] I=[43:32] (data_h)",
               lambda w: _iq(w, 32, 44, 12)),
    "iq16_l": ("I=[31:16] Q=[15:0] of data_l",
               lambda w: _iq(w, 16, 0, 16)),
    "qi16_l": ("Q=[31:16] I=[15:0] of data_l",
               lambda w: _iq(w, 0, 16, 16)),
}


def psd(x, nfft=4096):
    """Averaged periodogram, fftshifted; returns (freq_norm, psd)."""
    n = min(len(x), nfft)
    if n < 256:
        sys.exit("capture too short")
    segs = len(x) // n
    acc = np.zeros(n)
    win = np.hanning(n)
    for k in range(segs):
        seg = x[k * n:(k + 1) * n] * win
        acc += np.abs(np.fft.fft(seg)) ** 2
    acc /= segs
    return np.fft.fftshift(np.fft.fftfreq(n)), np.fft.fftshift(acc)


def score_layout(x, fs_mhz, offset_mhz=None, tol_mhz=0.25):
    """Peak-to-median PSD in dB (0 if the peak misses the expected bin)."""
    x = x - np.mean(x)  # kill DC so a DC spur can't win
    f, p = psd(x)
    med = np.median(p) + 1e-12
    pk = int(np.argmax(p))
    pk_mhz = f[pk] * fs_mhz
    s = 10 * np.log10(p[pk] / med)
    if offset_mhz is not None and abs(pk_mhz - offset_mhz) > tol_mhz:
        return 0.0, pk_mhz
    return s, pk_mhz


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["raw", "score", "unpack", "plot"])
    ap.add_argument("dump")
    ap.add_argument("--layout", help="layout name (unpack/plot)")
    ap.add_argument("--offset-mhz", type=float, default=None,
                    help="expected CW offset from channel center (score)")
    ap.add_argument("--out", help="output file (unpack: complex64; plot: png)")
    ap.add_argument("--nfft", type=int, default=4096)
    args = ap.parse_args()

    meta, words = read_dump(args.dump)
    print(f"# {args.dump}: n={meta['n']} fs={meta['fs_mhz']}M "
          f"dma={meta['dma_type']} wrap={meta['wrap']} "
          f"trig={meta['trig_mode']}/{meta['mac_sig']}", file=sys.stderr)

    if args.cmd == "raw":
        # Byte-parity with the vendor's is_la_print "%08x%08x" dump.
        for w in words:
            print(f"{int(w) >> 32:08x}{int(w) & 0xffffffff:08x}")
        return

    if args.cmd == "score":
        rows = []
        for name, (desc, fn) in LAYOUTS.items():
            x = fn(words)
            s, pk = score_layout(x, meta["fs_mhz"], args.offset_mhz)
            rows.append((s, name, desc, pk))
        rows.sort(reverse=True)
        print(f"{'layout':10s} {'score_db':>8s} {'peak_mhz':>9s}  description")
        for s, name, desc, pk in rows:
            print(f"{name:10s} {s:8.1f} {pk:9.2f}  {desc}")
        return

    layout = args.layout or DEFAULT_LAYOUT
    if layout not in LAYOUTS:
        sys.exit(f"unknown layout, one of: {', '.join(LAYOUTS)}")
    x = LAYOUTS[layout][1](words)

    if args.cmd == "unpack":
        out = args.out or (args.dump + ".c64")
        x.tofile(out)
        print(f"wrote {len(x)} complex64 samples -> {out}", file=sys.stderr)
        return

    if args.cmd == "plot":
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        f, p = psd(x - np.mean(x), args.nfft)
        fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 7))
        a1.plot(f * meta["fs_mhz"], 10 * np.log10(p + 1e-12))
        a1.set_xlabel("offset MHz")
        a1.set_ylabel("PSD dB")
        a1.set_title(f"{args.dump} layout={layout} fs={meta['fs_mhz']}M")
        a1.grid(True, alpha=0.3)
        n = min(2000, len(x))
        a2.plot(np.abs(x[:n]))
        a2.set_xlabel("sample")
        a2.set_ylabel("|x|")
        a2.grid(True, alpha=0.3)
        out = args.out or (args.dump + ".png")
        fig.savefig(out, dpi=100, bbox_inches="tight")
        print(f"wrote {out}", file=sys.stderr)


if __name__ == "__main__":
    main()
