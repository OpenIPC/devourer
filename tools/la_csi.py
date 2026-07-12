#!/usr/bin/env python3
"""Per-tone channel estimate H(k) from an LA-mode capture (issue #150).

Locates an 802.11 L-LTF (legacy long training field) in the complex
baseband stream of a DVLA capture (tools/la_decode.py unpacking, default
layout qi12_l), corrects CFO from the two LTF repetitions, FFTs both
64-sample symbols, and divides by the known ±26-tone sequence — the full
per-subcarrier channel estimate no Jaguar chip exports through a normal
host path.

    la_csi.py cap.bin [--layout qi12_l] [--json out.jsonl] [--plot h.png]
    la_csi.py --selftest        # synthetic L-LTF through a 2-tap channel

Requires a capture at 20 Msps of a 20 MHz channel (the standard L-LTF
sample grid): DEVOURER_LA_CAPTURE=crcok/20M/dma0/port:0x880 — the CRC-OK
trigger fires at frame *end*, and the wrapped ring holds the whole frame
including the preamble.

Detection: 64-lag autocorrelation plateau (the LTF's two identical
periods), refined by cross-correlation against the reference symbol.
Output: one JSON line per tone {tone, mag, mag_db, phase_rad} plus a
summary. Selftest: 2-tap channel + AWGN, asserts |H(k)| correlation
against the analytic channel > 0.99 (exits nonzero on failure — wired as
the la_csi_math ctest).
"""

import argparse
import json
import os
import sys

try:
    import numpy as np
except ImportError:
    # ctest SKIP_RETURN_CODE — CI runners without numpy skip, not fail.
    print("numpy not available — skipping", file=sys.stderr)
    sys.exit(77)

# L_{-26..-1}, 0 (DC), L_{+1..+26} — IEEE 802.11-2016 19.3.10.10.
LTF_SEQ = np.array(
    [1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1, -1, 1, 1, -1,
     1, -1, 1, 1, 1, 1,
     0,
     1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, -1, 1,
     -1, 1, -1, 1, 1, 1, 1], dtype=float)
TONES = np.arange(-26, 27)  # tone indices matching LTF_SEQ (incl. DC)


def ltf_freq64():
    """The L-LTF mapped onto a 64-bin FFT grid (bin = tone mod 64)."""
    f = np.zeros(64, dtype=complex)
    for k, v in zip(TONES, LTF_SEQ):
        f[k % 64] = v
    return f


def ltf_ref_time():
    """One 64-sample time-domain LTF period at 20 Msps."""
    return np.fft.ifft(ltf_freq64())


def locate_lltf(x, min_metric=0.6):
    """Find the start of the first LTF 64-sample period.

    Returns (start_index, cfo_norm) or (None, 0). cfo_norm is the
    frequency offset in cycles/sample estimated from the phase drift
    between the two repetitions."""
    n = len(x)
    if n < 192:
        return None, 0.0
    # 64-lag autocorrelation plateau (two identical periods).
    lag = 64
    c = x[:n - lag] * np.conj(x[lag:])
    p = np.abs(x[lag:]) ** 2
    win = 64
    ker = np.ones(win)
    num = np.convolve(c, ker, "valid")
    den = np.convolve(p, ker, "valid") + 1e-12
    metric = np.abs(num) / den
    # Candidate plateau peaks, best first.
    ref = ltf_ref_time()
    best = None
    for idx in np.argsort(metric)[::-1][:2000]:
        if metric[idx] < min_metric:
            break
        # Refine with cross-correlation against the reference in a small
        # neighbourhood (plateau peaks are timing-ambiguous by design).
        lo = max(0, idx - 96)
        hi = min(n - 128, idx + 96)
        if hi <= lo:
            continue
        seg = x[lo:hi + 128]
        cc = np.abs(np.correlate(seg, ref, "valid"))
        k = int(np.argmax(cc))
        start = lo + k
        if start + 128 > n:
            continue
        # Sanity: the two 64-blocks must actually repeat.
        a, b = x[start:start + 64], x[start + 64:start + 128]
        rep = np.abs(np.vdot(a, b)) / (np.linalg.norm(a) * np.linalg.norm(b)
                                       + 1e-12)
        if rep < 0.5:
            continue
        # Reject the L-STF: its 16-sample periodicity also repeats at lag
        # 64 (and its tone comb aliases into an LTF-looking estimate). A
        # true LTF has LOW 16-lag self-similarity.
        blk = x[start:start + 128]
        r16 = np.abs(np.vdot(blk[:-16], blk[16:])) / (
            np.linalg.norm(blk[:-16]) * np.linalg.norm(blk[16:]) + 1e-12)
        if r16 > 0.8:
            continue
        best = (start, np.vdot(a, b))
        break
    if best is None:
        return None, 0.0
    start, acc = best
    cfo_norm = np.angle(acc) / (2 * np.pi * 64)
    return start, cfo_norm


def refine_cfo(x, start, max_periods=8, min_rep=0.5):
    """Re-estimate CFO over as many consecutive 64-sample repetitions as
    the signal provides (LTF-train captures carry ~10) — the 2-symbol
    estimate's few-kHz noise leaks measurably into adjacent tones."""
    acc = 0.0 + 0.0j
    used = 0
    for k in range(max_periods):
        a = x[start + 64 * k:start + 64 * (k + 1)]
        b = x[start + 64 * (k + 1):start + 64 * (k + 2)]
        if len(b) < 64:
            break
        rep = np.abs(np.vdot(a, b)) / (np.linalg.norm(a) * np.linalg.norm(b)
                                       + 1e-12)
        if rep < min_rep:
            break
        acc += np.vdot(a, b)
        used += 1
    if used == 0:
        return None
    return np.angle(acc) / (2 * np.pi * 64)


def estimate_h(x, start, cfo_norm, backoff=4):
    """CFO-correct, FFT both LTF symbols, average, divide by the tones.

    The FFT window is pulled `backoff` samples INTO the guard interval:
    the GI2 is a cyclic prefix of the same symbol, so the shift is a pure
    linear phase across tones (harmless for |H|) while absorbing channel
    delay spread that would otherwise leak inter-symbol energy into deep
    nulls (measured on the notch-validation protocol)."""
    start = max(0, start - backoff)
    n = np.arange(start, start + 128)
    seg = x[start:start + 128] * np.exp(-2j * np.pi * cfo_norm * n)
    s1 = np.fft.fft(seg[:64])
    s2 = np.fft.fft(seg[64:128])
    avg = (s1 + s2) / 2
    h = np.zeros(len(TONES), dtype=complex)
    for i, (k, v) in enumerate(zip(TONES, LTF_SEQ)):
        if v != 0:
            h[i] = avg[k % 64] / v
    return h


def run(x, args):
    start, cfo = locate_lltf(x)
    if start is None:
        print("no L-LTF found in capture", file=sys.stderr)
        return 1
    h = estimate_h(x, start, cfo)
    nz = LTF_SEQ != 0
    mag = np.abs(h[nz])
    print(f"# L-LTF at sample {start}, CFO {cfo * 20e6 / 1e3:+.1f} kHz, "
          f"|H| mean {mag.mean():.1f} spread "
          f"{20 * np.log10(mag.max() / max(mag.min(), 1e-9)):.1f} dB",
          file=sys.stderr)
    out = open(args.json, "w") if args.json else sys.stdout
    for k, hv, seq in zip(TONES, h, LTF_SEQ):
        if seq == 0:
            continue
        out.write(json.dumps({
            "tone": int(k),
            "mag": float(np.abs(hv)),
            "mag_db": float(20 * np.log10(np.abs(hv) + 1e-12)),
            "phase_rad": float(np.angle(hv)),
        }) + "\n")
    if args.json:
        out.close()
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        a1.stem(TONES[nz], 20 * np.log10(np.abs(h[nz]) + 1e-12))
        a1.set_ylabel("|H(k)| dB")
        a1.grid(True, alpha=0.3)
        a2.stem(TONES[nz], np.angle(h[nz]))
        a2.set_ylabel("arg H(k) rad")
        a2.set_xlabel("subcarrier")
        a2.grid(True, alpha=0.3)
        fig.savefig(args.plot, dpi=100, bbox_inches="tight")
        print(f"wrote {args.plot}", file=sys.stderr)
    return 0


def selftest():
    """Synthetic L-LTF through a known 2-tap channel + AWGN."""
    rng = np.random.default_rng(7)
    ref = ltf_ref_time()
    # GI2 (32 samples) + two periods, embedded in noise padding.
    ltf = np.concatenate([ref[-32:], ref, ref])
    x = np.concatenate([
        0.05 * (rng.standard_normal(700) + 1j * rng.standard_normal(700)),
        ltf,
        0.05 * (rng.standard_normal(700) + 1j * rng.standard_normal(700)),
    ])
    taps = np.array([1.0, 0.55 * np.exp(1j * 1.1)])
    y = np.convolve(x, taps, "full")[:len(x)]
    cfo_true = 45e3 / 20e6  # 45 kHz at 20 Msps
    y = y * np.exp(2j * np.pi * cfo_true * np.arange(len(y)))
    y += 0.01 * (rng.standard_normal(len(y)) + 1j * rng.standard_normal(len(y)))

    start, cfo = locate_lltf(y)
    assert start is not None, "selftest: L-LTF not found"
    cfo_err_khz = abs(cfo - cfo_true) * 20e3
    assert cfo_err_khz < 2.0, f"selftest: CFO error {cfo_err_khz:.2f} kHz"
    h = estimate_h(y, start, cfo)

    h_true64 = np.fft.fft(np.concatenate([taps, np.zeros(62)]))
    nz = LTF_SEQ != 0
    h_true = np.array([h_true64[k % 64] for k in TONES])[nz]
    corr = np.corrcoef(np.abs(h[nz]), np.abs(h_true))[0, 1]
    print(f"selftest: LTF@{start} cfo_err={cfo_err_khz:.2f} kHz "
          f"|H| corr={corr:.4f}")
    assert corr > 0.99, f"selftest: |H(k)| correlation {corr:.4f} <= 0.99"
    print("selftest: PASS")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dump", nargs="?")
    ap.add_argument("--layout", default=None)
    ap.add_argument("--json", help="write per-tone JSONL here (default stdout)")
    ap.add_argument("--plot", help="write |H|/phase PNG here")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        sys.exit(selftest())
    if not args.dump:
        ap.error("dump file required (or --selftest)")

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import la_decode
    meta, words = la_decode.read_dump(args.dump)
    if abs(meta["fs_mhz"] - 20.0) > 0.01:
        print(f"WARNING: capture at {meta['fs_mhz']} Msps — L-LTF math "
              "assumes the standard 20 Msps grid", file=sys.stderr)
    layout = args.layout or la_decode.DEFAULT_LAYOUT
    x = la_decode.LAYOUTS[layout][1](words)
    x = x - np.mean(x)
    sys.exit(run(x, args))


if __name__ == "__main__":
    main()
