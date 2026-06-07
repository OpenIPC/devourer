#!/usr/bin/env python3
"""Phase-B RF verification for the precoder PoC: per-subcarrier IQ inspection.

GATING: run this only after Phase A (byte-level round-trip via a partner
monitor adapter) passes — proving per-subcarrier control is pointless if the
scrambler/BCC/interleaver model is wrong, and the SDR stack is the expensive
part to stand up.

What it does: take a 20 MHz baseband IQ capture of a precoder frame, recover
the OFDM data-subcarrier constellation symbol-by-symbol (FFT, channel-estimate
off the L-LTF, pilot phase-track, BPSK slice), and compare it to the target
pattern the encoder was asked for. Pattern in == pattern out is the proof.

SDR NOTE (correcting the plan): an RTL-SDR tops out near ~2.4 MS/s, so it
CANNOT capture a 20 MHz 802.11 signal — you need a >=20 MS/s SDR (HackRF @
20 Msps, USRP, LimeSDR, bladeRF). Capture is therefore done via SoapySDR
(device-agnostic) or from a pre-recorded IQ file. The OFDM math below is the
reusable core; full on-air frame synchronisation is scaffolded and flagged
where it needs bench validation.

Runnable without hardware:
    uv run python fft_capture.py --self-test     # synthesise -> demod -> verify

With a capture file (complex64 little-endian, 20 Msps, channel-centred):
    uv run python fft_capture.py --iq frame.cf32 --pattern target.txt --phy ht
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

import encode_subcarriers as enc

FFT_N = 64          # 20 MHz OFDM
CP_LEN = 16         # long guard interval
SYM_LEN = FFT_N + CP_LEN
PILOTS = (-21, -7, 7, 21)


def data_subcarrier_offsets(phy: enc.PhyParams) -> np.ndarray:
    """FFT bin offsets (-N..+N) carrying DATA, in ascending = logical order.

    This is the order the interleaver maps coded bits onto, so element k of the
    encoder's per-symbol target corresponds to this list's element k. Excludes
    DC (0), the four pilots, and the guard bands.
    """
    edge = 28 if phy.n_sd == 52 else 26  # HT uses +-28, legacy +-26
    offs = [k for k in range(-edge, edge + 1) if k != 0 and k not in PILOTS]
    assert len(offs) == phy.n_sd, (len(offs), phy.n_sd)
    return np.array(offs, dtype=int)


def _bin_index(offset: int) -> int:
    """Map an FFT bin offset (-32..31) to a numpy fft output index (0..63)."""
    return offset % FFT_N


def modulate_symbol(data_pm1: np.ndarray, phy: enc.PhyParams,
                    pilot_pm1: float = 1.0) -> np.ndarray:
    """Build one time-domain OFDM symbol (CP + 64 samples) from data subcarriers.

    Inverse of `demod_symbol`; used by --self-test and as ground truth. Pilots
    are set to a constant BPSK value (good enough for the self-test's equaliser).
    """
    freq = np.zeros(FFT_N, dtype=complex)
    for off, val in zip(data_subcarrier_offsets(phy), data_pm1):
        freq[_bin_index(off)] = val
    for off in PILOTS:
        freq[_bin_index(off)] = pilot_pm1
    time = np.fft.ifft(freq) * FFT_N / np.sqrt(FFT_N)
    return np.concatenate([time[-CP_LEN:], time])  # prepend cyclic prefix


def demod_symbol(samples: np.ndarray, chan_est: np.ndarray | None = None
                 ) -> np.ndarray:
    """FFT one received OFDM symbol (drop CP) -> 64 equalised frequency bins."""
    assert len(samples) >= SYM_LEN
    body = samples[CP_LEN:SYM_LEN]
    freq = np.fft.fft(body) / np.sqrt(FFT_N)
    if chan_est is not None:
        with np.errstate(divide="ignore", invalid="ignore"):
            freq = np.where(np.abs(chan_est) > 1e-9, freq / chan_est, 0)
    return freq


def pilot_phase_correct(freq: np.ndarray, ref_pm1: float = 1.0) -> np.ndarray:
    """Derotate a symbol's bins by the average pilot phase error (residual CFO/SFO)."""
    pil = np.array([freq[_bin_index(p)] for p in PILOTS])
    err = np.angle(np.sum(pil * np.conj(ref_pm1)))
    return freq * np.exp(-1j * err)


def slice_data(freq: np.ndarray, phy: enc.PhyParams) -> np.ndarray:
    """BPSK hard-slice the data subcarriers of an equalised symbol -> +-1."""
    offs = data_subcarrier_offsets(phy)
    vals = np.array([freq[_bin_index(o)] for o in offs])
    return np.where(vals.real >= 0, 1, -1).astype(np.int8)


def compare(recovered: np.ndarray, target: np.ndarray) -> dict:
    recovered = np.atleast_2d(recovered)
    target = np.atleast_2d(target)
    n = min(len(recovered), len(target))
    recovered, target = recovered[:n], target[:n]
    mism = int(np.count_nonzero(recovered != target))
    total = target.size
    return {"symbols": n, "subcarriers": total, "mismatches": mism,
            "match_pct": 100.0 * (total - mism) / total if total else 0.0}


# --------------------------------------------------------------------------- #
# IQ input
# --------------------------------------------------------------------------- #
def load_iq(path: str) -> np.ndarray:
    """Load complex baseband. .cf32 = float32 I,Q interleaved; .cs16 = int16."""
    if path.endswith(".cs16"):
        raw = np.fromfile(path, dtype=np.int16).astype(np.float32) / 32768.0
        return raw[0::2] + 1j * raw[1::2]
    raw = np.fromfile(path, dtype=np.float32)
    return (raw[0::2] + 1j * raw[1::2]).astype(complex)


def find_symbol_starts(iq: np.ndarray, n_sym: int, threshold: float = 0.0
                       ) -> int:
    """Coarse energy-based burst start (first sample above threshold).

    NB: a production receiver syncs on the L-STF/L-LTF (autocorrelation +
    cross-correlation) for sample-accurate timing and CFO; this energy gate is
    a placeholder that needs bench validation on real captures. For the
    --self-test path the start is known exactly.
    """
    if threshold <= 0:
        threshold = 0.5 * np.mean(np.abs(iq) ** 2)
    above = np.where(np.abs(iq) ** 2 > threshold)[0]
    return int(above[0]) if len(above) else 0


# --------------------------------------------------------------------------- #
# Self-test: synthesise -> demod -> verify (no hardware)
# --------------------------------------------------------------------------- #
def self_test(phy: enc.PhyParams, snr_db: float = 25.0, n_sym: int = 3,
              seed_rng: int = 0) -> int:
    rng = np.random.default_rng(seed_rng)
    target = rng.choice([1, -1], size=(n_sym, phy.n_sd)).astype(np.int8)

    # Build a faux PPDU: one LTF symbol (all data bins = +1 for a flat channel
    # estimate) followed by the data symbols, with a benign multipath-ish gain.
    ltf = modulate_symbol(np.ones(phy.n_sd), phy)
    syms = [modulate_symbol(t, phy) for t in target]
    tx = np.concatenate([ltf, *syms])

    chan = 0.8 * np.exp(1j * 0.4)  # flat complex channel gain
    rx = tx * chan
    sig_p = np.mean(np.abs(rx) ** 2)
    noise = (rng.standard_normal(len(rx)) + 1j * rng.standard_normal(len(rx)))
    rx = rx + noise * np.sqrt(sig_p / (2 * 10 ** (snr_db / 10)))

    # Channel estimate from the LTF symbol (known all-ones data bins + pilots).
    ltf_freq = demod_symbol(rx[:SYM_LEN])
    ref_freq = np.fft.fft(ltf[CP_LEN:SYM_LEN]) / np.sqrt(FFT_N)
    with np.errstate(divide="ignore", invalid="ignore"):
        chan_est = np.where(np.abs(ref_freq) > 1e-9, ltf_freq / ref_freq, 1.0)

    recovered = []
    for s in range(n_sym):
        start = SYM_LEN * (1 + s)
        freq = demod_symbol(rx[start:start + SYM_LEN], chan_est)
        freq = pilot_phase_correct(freq)
        recovered.append(slice_data(freq, phy))
    recovered = np.array(recovered, dtype=np.int8)

    rep = compare(recovered, target)
    ok = rep["mismatches"] == 0
    print(f"[self-test] {phy.name} snr={snr_db}dB symbols={n_sym}: "
          f"{rep['match_pct']:.1f}% subcarriers recovered "
          f"({rep['mismatches']} mismatch(es)) -> {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def main(argv: "list[str] | None" = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--self-test", action="store_true",
                    help="synthesise an OFDM frame and verify the demod (no SDR)")
    ap.add_argument("--iq", help="capture file (.cf32 float32 I/Q, or .cs16)")
    ap.add_argument("--pattern", help="+-1 target pattern to compare against")
    ap.add_argument("--phy", choices=("ht", "legacy"), default="legacy")
    ap.add_argument("--n-sym", type=int, default=3, help="data symbols to demod")
    ap.add_argument("--snr-db", type=float, default=25.0, help="self-test SNR")
    args = ap.parse_args(argv)
    phy = enc.phy_params(args.phy)

    if args.self_test:
        return self_test(phy, snr_db=args.snr_db, n_sym=args.n_sym)

    if not args.iq:
        print("need --iq <capture> (or --self-test). SDR live capture via "
              "SoapySDR is a documented TODO — record to a .cf32 first with "
              "your >=20 Msps SDR's own tooling.", file=sys.stderr)
        return 2

    iq = load_iq(args.iq)
    start = find_symbol_starts(iq, args.n_sym)
    # The L-LTF is two repeats; use the second as the channel reference. This
    # alignment is the part that needs bench validation (see find_symbol_starts).
    chan_est = demod_symbol(iq[start:start + SYM_LEN])
    recovered = []
    for s in range(args.n_sym):
        seg = iq[start + SYM_LEN * (1 + s):start + SYM_LEN * (2 + s)]
        if len(seg) < SYM_LEN:
            break
        freq = pilot_phase_correct(demod_symbol(seg, chan_est))
        recovered.append(slice_data(freq, phy))
    recovered = np.array(recovered, dtype=np.int8)
    print(f"[fft] demodulated {len(recovered)} symbol(s) from {args.iq}")

    if args.pattern:
        target = enc.parse_pattern_file(args.pattern, phy.n_sd)
        rep = compare(recovered, target)
        print(f"[fft] vs target: {rep['match_pct']:.1f}% subcarriers match "
              f"({rep['mismatches']}/{rep['subcarriers']} mismatched over "
              f"{rep['symbols']} symbol(s))")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
