#!/usr/bin/env python3
"""Pre-modulator subcarrier encoder for devourer.

The Realtek Jaguar TX pipeline for a single-stream BPSK / BCC frame is fully
deterministic:

    PSDU bytes -> scrambler (x^7+x^4+1) -> BCC K=7 (133,171) r=1/2
               -> interleaver(MCS,BW) -> BPSK map (0->+1, 1->-1)
               -> pilot insert -> IFFT -> CP -> DAC

Because every stage above the constellation map is an invertible, memoryless-
or-linear transform, we can run the chain *backwards*: pick what each OFDM
data subcarrier should transmit (a +-1 BPSK target), and solve for the PSDU
bytes that make the chip emit it. The chip needs no firmware change — it
faithfully encodes whatever bytes we hand it.

This module implements the forward model AND its exact inverse, so the
round-trip (target -> encode -> emulate chip -> constellation) is bit-exact for
any target that is representable by the rate-1/2 code (see `bcc_viterbi_preimage`
for why only a 2^k subspace is reachable; non-representable targets get the
nearest valid pattern + a reported Hamming distance).

PHY MODE — legacy vs HT  (IMPORTANT)
------------------------------------
The originating plan's prose ("48 data + 4 pilot", "24 input bits") describes
*legacy* 802.11a OFDM (6 Mbps BPSK). It also said to select the rate via the HT
radiotap MCS field (`DEVOURER_TX_MCS=0`) — but HT MCS 0 is a DIFFERENT
numerology (52 SD, 26 info bits, 13x4 interleaver) AND, decisively,
`RtlJaguarDevice::send_packet` never wires the HT MCS *index* into the TX rate:
an HT-MCS frame with no RATE field transmits at the MGN_1M default = 1 Mbps
CCK, which is DSSS — no OFDM subcarriers at all. So the working OFDM-BPSK path
is legacy 6 Mbps (honoured via the radiotap RATE field), which is what
`PrecoderDemo` transmits. Both numerologies are implemented:

    --phy legacy   (default)  N_SD=48  N_CBPS=48  N_DBPS=24  interleaver 16x3
    --phy ht                  N_SD=52  N_CBPS=52  N_DBPS=26  interleaver 13x4

`legacy` is the default to match `PrecoderDemo`'s 6 Mbps OFDM frame (and the
plan's prose). `--phy ht` is correct math but won't reach the air until
send_packet's HT-MCS-index gap is fixed AND PrecoderDemo emits an HT radiotap.

SCOPE / ASSUMPTIONS (documented, deliberate for a PoC)
------------------------------------------------------
* Single spatial stream, BPSK, rate 1/2, BCC (not LDPC), long GI, 20 MHz.
  These are the locked-in scope decisions from the plan.
* The encoder's contract is purely bit-domain: "given a target for a
  contiguous run of OFDM *data* symbols, starting from BCC state 0 and
  scrambler phase `offset`, produce the bytes that reproduce it." It does NOT
  model the 16-bit SERVICE prefix, the MAC header, the tail/pad bits, or where
  the controlled symbol physically lands in a real MPDU. Those are an
  *application* concern: see README.md for the offset arithmetic needed to
  place a fully-controlled symbol in a real frame (a symbol that straddles the
  fixed MAC header is only partially controllable).
* Byte packing is LSB-first within each octet (the 802.11 PPDU convention:
  the first PSDU bit is bit 0 of octet 0).
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass

import numpy as np

# Convolutional code generators, K=7, rate 1/2 (IEEE 802.11 §17.3.5.6).
# Octal 133/171; output order is A (133) then B (171) per input bit.
G0 = 0o133  # 0b1011011
G1 = 0o171  # 0b1111001

# Scrambler seed used in the IEEE worked example (Annex I) and referenced by
# the plan's CLI example. 1011101b. Any nonzero 7-bit value is valid; the chip
# picks its own per-frame (discover it with seed_probe.py).
DEFAULT_SEED = 0x5D


# --------------------------------------------------------------------------- #
# PHY parameters
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class PhyParams:
    """OFDM / coding parameters for one (PHY, MCS, BW) point.

    For BPSK 1-stream: N_CBPS == N_SD (one coded bit per data subcarrier) and
    N_DBPS == N_CBPS // 2 (rate 1/2). The interleaver reduces to its first
    permutation for BPSK (the second permutation is identity when s=1, and the
    HT third permutation is identity for a single spatial stream).
    """

    name: str
    n_sd: int      # data subcarriers per OFDM symbol
    n_cbps: int    # coded bits per symbol
    n_dbps: int    # data (info) bits per symbol
    n_bpscs: int    # coded bits per subcarrier per stream (BPSK = 1)
    i_col: int     # interleaver columns
    i_row: int     # interleaver rows
    n_ss: int = 1  # spatial streams (PoC = 1)


_LEGACY_BPSK = PhyParams(
    name="legacy-6Mbps-BPSK", n_sd=48, n_cbps=48, n_dbps=24, n_bpscs=1,
    i_col=16, i_row=3,
)
_HT_MCS0 = PhyParams(
    name="ht-mcs0-BPSK", n_sd=52, n_cbps=52, n_dbps=26, n_bpscs=1,
    i_col=13, i_row=4,
)


def phy_params(mode: str = "legacy", mcs: int = 0, bw: int = 20) -> PhyParams:
    """Resolve PHY parameters. PoC supports BPSK rate-1/2 at 20 MHz only."""
    if bw != 20:
        raise NotImplementedError(f"only 20 MHz supported in this PoC (got {bw})")
    if mode == "legacy":
        return _LEGACY_BPSK
    if mode == "ht":
        if mcs != 0:
            raise NotImplementedError(
                f"only HT MCS 0 (BPSK r=1/2) supported (got MCS {mcs}); "
                "higher MCS use QAM and/or >1 stream, out of PoC scope"
            )
        return _HT_MCS0
    raise ValueError(f"unknown phy mode {mode!r} (expected 'ht' or 'legacy')")


# --------------------------------------------------------------------------- #
# Scrambler — x^7 + x^4 + 1
# --------------------------------------------------------------------------- #
def scrambler_sequence(seed: int, n: int) -> np.ndarray:
    """`n` bits of the 802.11 scrambling sequence for 7-bit LFSR `seed`.

    Taps at x7 (bit 6) and x4 (bit 3); feedback shifts into bit 0. The pure
    LFSR output therefore satisfies the recurrence o[m] = o[m-4] ^ o[m-7]
    (= the polynomial 1 + x^4 + x^7) and is maximal-length (period 127).
    """
    reg = seed & 0x7F
    out = np.empty(n, dtype=np.uint8)
    for i in range(n):
        fb = ((reg >> 6) ^ (reg >> 3)) & 1
        out[i] = fb
        reg = ((reg << 1) | fb) & 0x7F
    return out


def apply_scrambler(bits: np.ndarray, seed: int, offset: int = 0) -> np.ndarray:
    """XOR `bits` with the scrambler sequence starting at phase `offset`.

    The scrambler is its own inverse, so this is both `scramble` and
    `descramble`. `offset` is the bit position of `bits[0]` within the frame's
    scrambled stream (the scrambler runs continuously from the SERVICE field).
    """
    bits = np.asarray(bits, dtype=np.uint8)
    seq = scrambler_sequence(seed, offset + len(bits))[offset:]
    return (bits ^ seq).astype(np.uint8)


# `descramble` is identical to scrambling (XOR); alias for readability at the
# call sites that conceptually "remove" the chip's scrambling.
descramble = apply_scrambler


# --------------------------------------------------------------------------- #
# Binary convolutional code (BCC), K=7, (133,171), rate 1/2
# --------------------------------------------------------------------------- #
def _parity(x: int) -> int:
    return bin(x).count("1") & 1


def bcc_encode(info_bits: np.ndarray, init_state: int = 0) -> np.ndarray:
    """Encode info bits with the 802.11 K=7 (133,171) r=1/2 code.

    Returns 2*len(info_bits) coded bits in A0,B0,A1,B1,... order. `init_state`
    is the 6-bit encoder state (the 6 preceding input bits); 0 = the all-zero
    start state used at the head of an 802.11 frame (no tail-biting). A nonzero
    `init_state` continues a stream mid-frame — see `bcc_final_state`.
    """
    info_bits = np.asarray(info_bits, dtype=np.uint8)
    out = np.empty(2 * len(info_bits), dtype=np.uint8)
    state = init_state & 0x3F
    for t, u in enumerate(info_bits):
        w = (int(u) << 6) | state  # 7-bit window: bit6 = current input
        out[2 * t] = _parity(w & G0)
        out[2 * t + 1] = _parity(w & G1)
        state = w >> 1
    return out


def bcc_final_state(info_bits: np.ndarray, init_state: int = 0) -> int:
    """The 6-bit BCC encoder state after encoding `info_bits` from `init_state`.

    Use this to compute the state entering a mid-frame symbol: encode the
    scrambled SERVICE field + MAC-header bits that precede the controlled
    region, then pass the result as `entry_state` to `encode_pattern`.
    """
    info_bits = np.asarray(info_bits, dtype=np.uint8)
    state = init_state & 0x3F
    for u in info_bits:
        state = ((int(u) << 6) | state) >> 1
    return state


# Precomputed trellis: for (state, input) -> (next_state, expected (A,B)).
def _build_trellis() -> tuple[np.ndarray, np.ndarray]:
    next_state = np.empty((64, 2), dtype=np.int64)
    out_ab = np.empty((64, 2, 2), dtype=np.uint8)
    for state in range(64):
        for u in (0, 1):
            w = (u << 6) | state
            next_state[state, u] = w >> 1
            out_ab[state, u, 0] = _parity(w & G0)
            out_ab[state, u, 1] = _parity(w & G1)
    return next_state, out_ab


_TRELLIS_NEXT, _TRELLIS_OUT = _build_trellis()


def bcc_viterbi_preimage(
    coded_bits: np.ndarray, entry_state: int = 0
) -> tuple[np.ndarray, int]:
    """Find the info bits whose BCC encoding is closest to `coded_bits`.

    Hard-decision Viterbi over the 64-state trellis, starting at `entry_state`
    (0 = frame head), no termination (free end state). Returns
    (info_bits, hamming_distance).

    A rate-1/2 BCC is a (2k, k) linear code, so only 2^k of the 2^(2k) coded
    patterns are exactly representable. When `coded_bits` is one of them the
    distance is 0 and re-encoding the returned info reproduces it exactly;
    otherwise the result is the maximum-likelihood (nearest) codeword's input.
    """
    coded_bits = np.asarray(coded_bits, dtype=np.uint8)
    if len(coded_bits) % 2 != 0:
        raise ValueError("coded_bits length must be even (rate 1/2)")
    n_pairs = len(coded_bits) // 2

    INF = 1 << 30
    metric = np.full(64, INF, dtype=np.int64)
    metric[entry_state & 0x3F] = 0
    back_state = np.empty((n_pairs, 64), dtype=np.int64)
    back_input = np.empty((n_pairs, 64), dtype=np.uint8)

    for t in range(n_pairs):
        a = int(coded_bits[2 * t])
        b = int(coded_bits[2 * t + 1])
        new_metric = np.full(64, INF, dtype=np.int64)
        for state in range(64):
            m = metric[state]
            if m >= INF:
                continue
            for u in (0, 1):
                ea = _TRELLIS_OUT[state, u, 0]
                eb = _TRELLIS_OUT[state, u, 1]
                bm = (ea ^ a) + (eb ^ b)
                ns = int(_TRELLIS_NEXT[state, u])
                cost = m + bm
                if cost < new_metric[ns]:
                    new_metric[ns] = cost
                    back_state[t, ns] = state
                    back_input[t, ns] = u
        metric = new_metric

    end = int(np.argmin(metric))
    dist = int(metric[end])
    info = np.empty(n_pairs, dtype=np.uint8)
    s = end
    for t in range(n_pairs - 1, -1, -1):
        info[t] = back_input[t, s]
        s = int(back_state[t, s])
    return info, dist


# Plan's name for the inverse step.
bcc_preimage = bcc_viterbi_preimage


# --------------------------------------------------------------------------- #
# Interleaver
# --------------------------------------------------------------------------- #
def interleaver_perm(p: PhyParams) -> np.ndarray:
    """Permutation `perm[k] = j`: coded-bit index k -> subcarrier index j.

    First permutation (block, i_col x i_row) followed by the second
    permutation. For BPSK (n_bpscs=1) s = max(n_bpscs//2, 1) = 1, so the second
    permutation is the identity; the formula is kept general for clarity.
    """
    k = np.arange(p.n_cbps)
    i = p.i_row * (k % p.i_col) + (k // p.i_col)
    s = max(p.n_bpscs // 2, 1)
    j = s * (i // s) + (i + p.n_cbps - ((p.i_col * i) // p.n_cbps)) % s
    if p.n_ss != 1:
        raise NotImplementedError("HT frequency rotation (n_ss>1) out of scope")
    return j.astype(np.int64)


def interleave(coded: np.ndarray, p: PhyParams) -> np.ndarray:
    """coded bits (BCC output order) -> subcarrier-order bits."""
    coded = np.asarray(coded, dtype=np.uint8)
    perm = interleaver_perm(p)
    sub = np.empty(p.n_cbps, dtype=np.uint8)
    sub[perm] = coded
    return sub


def deinterleave(sub: np.ndarray, p: PhyParams) -> np.ndarray:
    """subcarrier-order bits -> coded bits (BCC output order)."""
    sub = np.asarray(sub, dtype=np.uint8)
    return sub[interleaver_perm(p)]


# --------------------------------------------------------------------------- #
# BPSK constellation
# --------------------------------------------------------------------------- #
def bpsk_map(bits: np.ndarray) -> np.ndarray:
    """0 -> +1, 1 -> -1."""
    return (1 - 2 * np.asarray(bits, dtype=np.int8)).astype(np.int8)


def bpsk_demap(target_pm1: np.ndarray) -> np.ndarray:
    """+1 -> 0, -1 -> 1 (sign slicer)."""
    return (np.asarray(target_pm1) < 0).astype(np.uint8)


# --------------------------------------------------------------------------- #
# Byte <-> bit packing (LSB-first within each octet, 802.11 PPDU order)
# --------------------------------------------------------------------------- #
def bits_to_bytes(bits: np.ndarray) -> bytes:
    bits = np.asarray(bits, dtype=np.uint8)
    pad = (-len(bits)) % 8
    if pad:
        bits = np.concatenate([bits, np.zeros(pad, dtype=np.uint8)])
    return np.packbits(bits, bitorder="little").tobytes()


def bytes_to_bits(data: bytes, n: int | None = None) -> np.ndarray:
    bits = np.unpackbits(np.frombuffer(data, dtype=np.uint8), bitorder="little")
    return bits if n is None else bits[:n]


# --------------------------------------------------------------------------- #
# High-level encode / emulate
# --------------------------------------------------------------------------- #
@dataclass
class EncodeResult:
    psdu_bits: np.ndarray   # the controllable info bits, descrambled (PSDU domain)
    psdu_bytes: bytes       # psdu_bits packed LSB-first (last octet zero-padded)
    hamming_distance: int   # subcarriers that differ from the target (0 = exact)
    representable: bool      # hamming_distance == 0
    n_sym: int
    phy: PhyParams
    seed: int
    offset: int


def encode_pattern(
    targets: np.ndarray,
    seed: int = DEFAULT_SEED,
    phy: PhyParams = _LEGACY_BPSK,
    offset: int = 0,
    strict: bool = False,
    entry_state: int = 0,
) -> EncodeResult:
    """Invert the pipeline: target BPSK subcarriers -> PSDU bytes.

    `targets` is shape (n_sym, N_SD) of +-1 (or a flat (N_SD,) for one symbol).
    `entry_state` is the BCC state entering the controlled region (0 at frame
    head; compute mid-frame values with `bcc_final_state`). `offset` is the
    scrambler phase of the first emitted bit. Returns an EncodeResult; if
    `strict` and the target is not exactly representable by the rate-1/2 code,
    raises ValueError instead of returning the nearest pattern.
    """
    targets = np.atleast_2d(np.asarray(targets))
    if targets.shape[1] != phy.n_sd:
        raise ValueError(
            f"each symbol must have {phy.n_sd} subcarriers for {phy.name}, "
            f"got {targets.shape[1]}"
        )

    coded_all = np.concatenate(
        [deinterleave(bpsk_demap(sym), phy) for sym in targets]
    )
    info_scrambled, dist = bcc_viterbi_preimage(coded_all, entry_state)
    if strict and dist != 0:
        raise ValueError(
            f"target not exactly representable by the rate-1/2 code "
            f"(nearest differs in {dist} subcarrier(s)); drop --strict to "
            f"accept the nearest pattern"
        )

    psdu_bits = apply_scrambler(info_scrambled, seed, offset)
    return EncodeResult(
        psdu_bits=psdu_bits,
        psdu_bytes=bits_to_bytes(psdu_bits),
        hamming_distance=dist,
        representable=(dist == 0),
        n_sym=targets.shape[0],
        phy=phy,
        seed=seed,
        offset=offset,
    )


def emulate_chip(
    psdu_bits: np.ndarray,
    seed: int,
    phy: PhyParams,
    n_sym: int,
    offset: int = 0,
    entry_state: int = 0,
) -> np.ndarray:
    """Forward model: PSDU info bits -> per-symbol BPSK subcarriers (+-1).

    The exact inverse of `encode_pattern` (for a representable target). Used by
    the round-trip / smoke tests and as a software stand-in for the chip.
    Returns shape (n_sym, N_SD).
    """
    info_scrambled = apply_scrambler(psdu_bits, seed, offset)
    coded = bcc_encode(info_scrambled, entry_state)
    expected = n_sym * phy.n_cbps
    if len(coded) < expected:
        raise ValueError(
            f"need {expected} coded bits for {n_sym} symbol(s), got {len(coded)}"
        )
    out = []
    for s in range(n_sym):
        chunk = coded[s * phy.n_cbps:(s + 1) * phy.n_cbps]
        out.append(bpsk_map(interleave(chunk, phy)))
    return np.array(out, dtype=np.int8)


# --------------------------------------------------------------------------- #
# Pattern file IO
# --------------------------------------------------------------------------- #
def parse_pattern_file(path: str, n_sd: int) -> np.ndarray:
    """Read a +-1 pattern file. Each non-comment token is +1/-1 (or 1/-1).

    Tokens may be whitespace- or newline-separated. The total count must be a
    positive multiple of `n_sd`; the result is reshaped to (n_sym, n_sd).
    """
    vals: list[int] = []
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            for tok in line.replace(",", " ").split():
                v = int(tok)
                if v not in (1, -1):
                    raise ValueError(f"{path}: token {tok!r} is not +1 or -1")
                vals.append(v)
    if not vals or len(vals) % n_sd != 0:
        raise ValueError(
            f"{path}: got {len(vals)} values, expected a positive multiple of "
            f"N_SD={n_sd}"
        )
    return np.array(vals, dtype=np.int8).reshape(-1, n_sd)


def alternating_pattern(n_sd: int, n_sym: int = 1) -> np.ndarray:
    """The plan's example pattern: [+1, -1, +1, -1, ...]. Generally NOT a
    codeword, so it exercises the nearest-match path."""
    row = np.where(np.arange(n_sd) % 2 == 0, 1, -1).astype(np.int8)
    return np.tile(row, (n_sym, 1))


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def _seed_int(s: str) -> int:
    v = int(s, 0)
    if not (0 <= v <= 0x7F):
        raise argparse.ArgumentTypeError("scrambler seed must be a 7-bit value (0..127)")
    return v


def build_argparser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="Encode a target OFDM subcarrier pattern into PSDU bytes by "
                    "inverting the 802.11 BPSK/BCC TX pipeline."
    )
    ap.add_argument("--pattern", help="file of +-1 values (N_SD per symbol). "
                    "Omit to use the built-in alternating example.")
    ap.add_argument("--scrambler-seed", type=_seed_int, default=DEFAULT_SEED,
                    help=f"chip scrambler seed, 7-bit (default 0x{DEFAULT_SEED:02x}); "
                         "discover the real one with seed_probe.py")
    ap.add_argument("--phy", choices=("ht", "legacy"), default="legacy",
                    help="legacy = 802.11a/g 6 Mbps BPSK (48 SD, default, "
                         "matches PrecoderDemo's on-air rate); ht = HT MCS0 "
                         "(52 SD, correct math but not reachable on air — see "
                         "module docstring)")
    ap.add_argument("--mcs", type=int, default=0,
                    help="HT MCS index (only 0 supported)")
    ap.add_argument("--offset", type=int, default=0,
                    help="bit offset of the first emitted bit within the frame's "
                         "scrambled stream (default 0; see README for real-frame "
                         "placement)")
    ap.add_argument("--strict", action="store_true",
                    help="fail if the target is not exactly representable instead "
                         "of emitting the nearest pattern")
    ap.add_argument("--psdu-out", help="write packed PSDU bytes here")
    ap.add_argument("--bruteforce", action="store_true",
                    help="emit 128 PSDUs (one per candidate seed) to "
                         "<psdu-out>.seed_NN.bin — single-adapter seed-search "
                         "fallback (see seed_probe.py --mode bruteforce)")
    return ap


def main(argv: list[str] | None = None) -> int:
    args = build_argparser().parse_args(argv)
    phy = phy_params(args.phy, args.mcs)

    if args.pattern:
        targets = parse_pattern_file(args.pattern, phy.n_sd)
    else:
        targets = alternating_pattern(phy.n_sd)
        print(f"[encode] no --pattern given; using built-in alternating "
              f"example ({phy.n_sd} subcarriers)", file=sys.stderr)

    if args.bruteforce:
        if not args.psdu_out:
            print("--bruteforce requires --psdu-out (used as a filename prefix)",
                  file=sys.stderr)
            return 2
        worst = 0
        for seed in range(128):
            res = encode_pattern(targets, seed=seed, phy=phy, offset=args.offset)
            worst = max(worst, res.hamming_distance)
            with open(f"{args.psdu_out}.seed_{seed:02x}.bin", "wb") as fh:
                fh.write(res.psdu_bytes)
        print(f"[encode] wrote 128 candidate PSDUs to {args.psdu_out}.seed_*.bin "
              f"({phy.name}, {targets.shape[0]} symbol(s), "
              f"max nearest-distance {worst})", file=sys.stderr)
        return 0

    res = encode_pattern(targets, seed=args.scrambler_seed, phy=phy,
                         offset=args.offset, strict=args.strict)
    status = "EXACT" if res.representable else f"NEAREST (+{res.hamming_distance})"
    print(f"[encode] {phy.name} seed=0x{res.seed:02x} offset={res.offset} "
          f"symbols={res.n_sym} info_bits={len(res.psdu_bits)} "
          f"bytes={len(res.psdu_bytes)} -> {status}", file=sys.stderr)
    if not res.representable:
        print(f"[encode] target not a codeword: emitted pattern differs from the "
              f"request in {res.hamming_distance} subcarrier(s) (use --strict to "
              f"refuse)", file=sys.stderr)

    if args.psdu_out:
        with open(args.psdu_out, "wb") as fh:
            fh.write(res.psdu_bytes)
        print(f"[encode] wrote {len(res.psdu_bytes)} bytes to {args.psdu_out}",
              file=sys.stderr)
    else:
        sys.stdout.buffer.write(res.psdu_bytes)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
