"""Soft-reliability outer FEC — the SDR-RX-only half of fused FEC.

The fused-FEC outer layer as shipped (`stream_fec_rs.py` + `fec_subblock.py`)
is **hard**: a sub-block either passes its CRC (a clean symbol) or it is erased,
and an erasure-only Reed-Solomon recovers the block from any K survivors. That
binary erasure is all a *chip* receiver can offer — its decoder hands you bytes,
not confidences.

An **SDR** receiver has more: the soft demapper computes a per-coded-bit LLR and
throws it away at the hard-decision slice (`frame_equalizer_impl.cc::
ht_data_symbol`, `sym = eqd[i] * derot` — the soft constellation point is right
there). This module is the outer code that *uses* that confidence:

  * The production RS is **erasure-only** (Vandermonde / zfec construction): it
    cannot correct a symbol whose location it does not already know. So a symbol
    with one flipped byte and no CRC to flag it silently poisons the inverse.
  * This module adds a **BCH-form RS** (generator polynomial, consecutive
    spectral zeros) with **errors-and-erasures** decoding — Berlekamp-Massey
    seeded by the erasure locator, Chien search, Forney. It corrects
    `2·errors + erasures ≤ N−K` symbols.
  * The **soft** part: per-symbol reliability (min |LLR| over the symbol's bits —
    a symbol is wrong iff its weakest bit flipped) ranks the symbols. Marking the
    least-reliable symbols as *erasures* turns each into a cost of 1 instead of 2
    in that budget, **doubling** the effective correction vs. an errors-only
    decode. A GMD sweep tries successive erasure counts and keeps the decode that
    re-checks clean.

FINDING (this module's verdict — `--mode overhead`): for the fused-FEC's real
parameters the soft outer code does **NOT** pay off. Two regimes:

  * Against a per-symbol **CRC** that hands the erasure-only RS perfect erasure
    locations, binary CRC-erasure BEATS soft-GMD — a CRC is a near-perfect,
    cheap erasure flag, and erasure-RS corrects up to N−K of them, while soft
    GMD only guesses by reliability and can be fooled by a confidently-wrong
    symbol. Soft beats only the *no-side-info* errors-only decode.
  * The soft scheme's lone structural edge is **overhead** — no per-symbol CRC,
    so those bytes become extra parity. But that only flips the result when the
    CRC overhead is large: with 32-byte symbols (CRC ~12.5%) CRC-erasure wins
    decisively; soft pulls ahead only near ~50% overhead (tiny ~4-byte symbols),
    a regime the video link never runs in.

So the SDR's genuinely-useful soft information lives at the **inner** decoder
(soft Viterbi — measured ~2× more surviving sub-blocks per corrupt frame, and
+485 frames recovered to clean CRC over real air), NOT the outer code. The chip
already does soft inner decode in hardware *and* exposes the same CRC erasure
flag via KEEP_CORRUPTED, so there is no outer-code advantage to be had from an
SDR here. This module is the reference that establishes that — and the
errors-and-erasures RS it adds (the production codec is erasure-only) is reusable
in its own right.

Reliability model: each codeword symbol's 8 bits are sent BPSK over AWGN; the
per-bit soft metric LLR = 2·y/σ² is what the SDR demapper produces post-equalize.
A live realisation would also need a soft-OUTPUT inner decoder (SOVA/BCJR) to
propagate bit confidence to per-byte reliability — an integration cost that, given
the negative finding above, is not worth paying.
"""

from __future__ import annotations

import argparse
import math
import random
from dataclasses import dataclass

# Reuse the project's GF(2^8): primitive poly 0x11d, alpha = 2 (matches
# stream_fec_rs / zfec / wfb-ng), so the two RS codecs share a field.
from stream_fec_rs import _GF_EXP, _GF_LOG  # noqa: E402

GF_GEN = 2  # primitive element alpha
FCR = 0     # first consecutive root exponent (roots alpha^0 .. alpha^(nsym-1))


# --------------------------------------------------------------------------- #
# GF(2^8) scalar + polynomial arithmetic (polys: index 0 = highest degree).
# --------------------------------------------------------------------------- #
def gf_mul(a: int, b: int) -> int:
    if a == 0 or b == 0:
        return 0
    return _GF_EXP[_GF_LOG[a] + _GF_LOG[b]]


def gf_inv(a: int) -> int:
    return _GF_EXP[255 - _GF_LOG[a]]


def gf_pow(a: int, e: int) -> int:
    if e == 0:
        return 1
    if a == 0:
        return 0
    return _GF_EXP[(_GF_LOG[a] * e) % 255]


def gf_poly_mul(p: list[int], q: list[int]) -> list[int]:
    r = [0] * (len(p) + len(q) - 1)
    for j in range(len(q)):
        qj = q[j]
        if qj == 0:
            continue
        lq = _GF_LOG[qj]
        for i in range(len(p)):
            pi = p[i]
            if pi:
                r[i + j] ^= _GF_EXP[lq + _GF_LOG[pi]]
    return r


def gf_poly_add(p: list[int], q: list[int]) -> list[int]:
    r = [0] * max(len(p), len(q))
    for i in range(len(p)):
        r[i + len(r) - len(p)] = p[i]
    for i in range(len(q)):
        r[i + len(r) - len(q)] ^= q[i]
    return r


def gf_poly_eval(p: list[int], x: int) -> int:
    y = p[0]
    for i in range(1, len(p)):
        y = gf_mul(y, x) ^ p[i]
    return y


# --------------------------------------------------------------------------- #
# Encoder
# --------------------------------------------------------------------------- #
_GEN_CACHE: dict[int, list[int]] = {}


def rs_generator_poly(nsym: int) -> list[int]:
    g = _GEN_CACHE.get(nsym)
    if g is not None:
        return g
    g = [1]
    for i in range(nsym):
        g = gf_poly_mul(g, [1, gf_pow(GF_GEN, i + FCR)])
    _GEN_CACHE[nsym] = g
    return g


def rs_encode_msg(msg_in: list[int], nsym: int) -> list[int]:
    """Systematic encode: returns msg_in followed by nsym parity symbols."""
    gen = rs_generator_poly(nsym)
    msg_out = list(msg_in) + [0] * (len(gen) - 1)
    for i in range(len(msg_in)):
        coef = msg_out[i]
        if coef != 0:
            lc = _GF_LOG[coef]
            for j in range(1, len(gen)):
                gj = gen[j]
                if gj:
                    msg_out[i + j] ^= _GF_EXP[lc + _GF_LOG[gj]]
    msg_out[:len(msg_in)] = msg_in
    return msg_out


# --------------------------------------------------------------------------- #
# Decoder — syndromes, errata locator, Berlekamp-Massey, Chien, Forney.
# Ported from the canonical "Reed-Solomon codes for coders" algorithms onto the
# project GF, fcr=0, generator=2.
# --------------------------------------------------------------------------- #
def rs_calc_syndromes(msg: list[int], nsym: int) -> list[int]:
    return [0] + [gf_poly_eval(msg, gf_pow(GF_GEN, i + FCR)) for i in range(nsym)]


def rs_find_errata_locator(e_pos: list[int]) -> list[int]:
    e_loc = [1]
    for i in e_pos:
        e_loc = gf_poly_mul(e_loc, gf_poly_add([1], [gf_pow(GF_GEN, i), 0]))
    return e_loc


def rs_find_error_evaluator(synd: list[int], err_loc: list[int],
                            nsym: int) -> list[int]:
    remainder = gf_poly_mul(synd, err_loc)
    remainder = remainder[len(remainder) - (nsym + 1):]
    return remainder


def rs_find_error_locator(synd: list[int], nsym: int,
                          erase_loc: list[int] | None = None,
                          erase_count: int = 0) -> list[int]:
    """Berlekamp-Massey, optionally seeded by an erasure locator."""
    if erase_loc is not None:
        err_loc = list(erase_loc)
        old_loc = list(erase_loc)
    else:
        err_loc = [1]
        old_loc = [1]

    synd_shift = 0
    if len(synd) > nsym:
        synd_shift = len(synd) - nsym

    for i in range(nsym - erase_count):
        if erase_loc is not None:
            K = erase_count + i + synd_shift
        else:
            K = i + synd_shift

        delta = synd[K]
        for j in range(1, len(err_loc)):
            delta ^= gf_mul(err_loc[len(err_loc) - (j + 1)], synd[K - j])

        old_loc = old_loc + [0]

        if delta != 0:
            if len(old_loc) > len(err_loc):
                new_loc = [gf_mul(c, delta) for c in old_loc]
                old_loc = [gf_mul(c, gf_inv(delta)) for c in err_loc]
                err_loc = new_loc
            err_loc = gf_poly_add(err_loc, [gf_mul(c, delta) for c in old_loc])

    while len(err_loc) and err_loc[0] == 0:
        del err_loc[0]
    errs = len(err_loc) - 1
    if (errs - erase_count) * 2 + erase_count > nsym:
        raise ValueError("too many errors to correct")
    return err_loc


def rs_find_errors(err_loc: list[int], nmess: int) -> list[int]:
    errs = len(err_loc) - 1
    err_pos = []
    for i in range(nmess):
        if gf_poly_eval(err_loc, gf_pow(GF_GEN, i)) == 0:
            err_pos.append(nmess - 1 - i)
    if len(err_pos) != errs:
        raise ValueError("too many (or few) errors found by Chien search")
    return err_pos


def rs_forney_syndromes(synd: list[int], pos: list[int],
                        nmess: int) -> list[int]:
    erase_pos_reversed = [nmess - 1 - p for p in pos]
    fsynd = list(synd[1:])
    for i in range(len(pos)):
        x = gf_pow(GF_GEN, erase_pos_reversed[i])
        for j in range(len(fsynd) - 1):
            fsynd[j] = gf_mul(fsynd[j], x) ^ fsynd[j + 1]
    return fsynd


def rs_correct_errata(msg: list[int], synd: list[int],
                      err_pos: list[int]) -> list[int]:
    coef_pos = [len(msg) - 1 - p for p in err_pos]
    err_loc = rs_find_errata_locator(coef_pos)
    err_eval = rs_find_error_evaluator(synd[::-1], err_loc,
                                       len(err_loc) - 1)[::-1]

    X = []
    for i in range(len(coef_pos)):
        ll = 255 - coef_pos[i]
        X.append(gf_pow(GF_GEN, -ll % 255))

    E = [0] * len(msg)
    Xlength = len(X)
    for i, Xi in enumerate(X):
        Xi_inv = gf_inv(Xi)
        err_loc_prime_tmp = []
        for j in range(Xlength):
            if j != i:
                err_loc_prime_tmp.append(1 ^ gf_mul(Xi_inv, X[j]))
        err_loc_prime = 1
        for coef in err_loc_prime_tmp:
            err_loc_prime = gf_mul(err_loc_prime, coef)
        y = gf_poly_eval(err_eval[::-1], Xi_inv)
        y = gf_mul(gf_pow(Xi, 1 - FCR), y)
        if err_loc_prime == 0:
            raise ValueError("could not find error magnitude")
        magnitude = gf_mul(y, gf_inv(err_loc_prime))
        E[err_pos[i]] = magnitude

    return gf_poly_add(msg, E)


def rs_correct_msg(msg_in: list[int], nsym: int,
                   erase_pos: list[int] | None = None) -> list[int]:
    """Full errors-and-erasures decode. Returns the corrected codeword.

    Raises ValueError if the (errors, erasures) load exceeds the budget or the
    correction does not re-check clean.
    """
    if len(msg_in) > 255:
        raise ValueError("message too long")
    msg_out = list(msg_in)
    erase_pos = list(erase_pos) if erase_pos else []
    for e in erase_pos:
        msg_out[e] = 0
    if len(erase_pos) > nsym:
        raise ValueError("too many erasures to correct")

    synd = rs_calc_syndromes(msg_out, nsym)
    if max(synd) == 0:
        return msg_out  # already clean

    fsynd = rs_forney_syndromes(synd, erase_pos, len(msg_out))
    err_loc = rs_find_error_locator(fsynd, nsym, erase_count=len(erase_pos))
    err_pos = rs_find_errors(err_loc[::-1], len(msg_out))
    if not err_pos and not erase_pos:
        raise ValueError("could not locate errors")

    msg_out = rs_correct_errata(msg_out, synd, erase_pos + err_pos)
    if max(rs_calc_syndromes(msg_out, nsym)) != 0:
        raise ValueError("decode did not re-check clean")
    return msg_out


# --------------------------------------------------------------------------- #
# Soft GMD: use per-symbol reliability to pick erasures, sweeping the count.
# --------------------------------------------------------------------------- #
def soft_decode(received: list[int], nsym: int,
                reliability: list[float],
                max_extra_erasures: int | None = None) -> tuple[list[int], int]:
    """Generalised-minimum-distance decode.

    `reliability[i]` is the confidence in symbol i (higher = more reliable; e.g.
    min |LLR| over its bits). Sweep e = 0,2,4,… extra erasures applied to the
    LEAST-reliable symbols (the classic GMD even-erasure ladder) and return the
    first decode that re-checks clean. Returns (codeword, erasures_used).
    Raises ValueError if no erasure count yields a clean decode.
    """
    order = sorted(range(len(received)), key=lambda i: reliability[i])
    if max_extra_erasures is None:
        max_extra_erasures = nsym
    # GMD ladder: 0, 1, 2, ... up to the budget. (Even-only is the textbook
    # ladder; stepping by 1 is strictly more thorough and cheap here.)
    for e in range(0, min(max_extra_erasures, nsym) + 1):
        erase = order[:e]
        try:
            out = rs_correct_msg(received, nsym, erase_pos=erase)
            return out, e
        except ValueError:
            continue
    raise ValueError("GMD exhausted: no erasure count decoded cleanly")


# --------------------------------------------------------------------------- #
# Channel simulation — 8-bit RS symbols, each bit BPSK over AWGN.
# --------------------------------------------------------------------------- #
@dataclass
class BlockOutcome:
    received: list[int]
    reliability: list[float]
    true_codeword: list[int]
    corrupt_positions: list[int]


def simulate_block(codeword: list[int], snr_db: float,
                   rng: random.Random) -> BlockOutcome:
    """Send each symbol's 8 bits BPSK (+1/-1) over AWGN at snr_db (Eb/N0 per
    bit). Hard-slice to recover the received symbol; reliability = min |LLR|."""
    # Eb/N0: bit energy 1, noise variance sigma^2 = 1/(2*EbN0).
    ebn0 = 10 ** (snr_db / 10.0)
    sigma = math.sqrt(1.0 / (2.0 * ebn0))
    received = []
    reliability = []
    corrupt = []
    for idx, sym in enumerate(codeword):
        rsym = 0
        min_llr = math.inf
        for b in range(8):
            bit = (sym >> b) & 1
            tx = 1.0 if bit else -1.0
            y = tx + rng.gauss(0.0, sigma)
            llr = 2.0 * y / (sigma * sigma)      # SDR soft metric
            hard = 1 if y > 0 else 0
            rsym |= hard << b
            if abs(llr) < min_llr:
                min_llr = abs(llr)
        received.append(rsym)
        reliability.append(min_llr)
        if rsym != sym:
            corrupt.append(idx)
    return BlockOutcome(received, reliability, codeword, corrupt)


# --------------------------------------------------------------------------- #
# The three outer-decoder strategies, on the SAME received block.
# --------------------------------------------------------------------------- #
def decode_errors_only(o: BlockOutcome, nsym: int) -> bool:
    """Hard RS, no side info: BM corrects up to floor(nsym/2) symbol errors."""
    try:
        out = rs_correct_msg(o.received, nsym, erase_pos=[])
        return out == o.true_codeword
    except ValueError:
        return False


def decode_crc_erasure(o: BlockOutcome, nsym: int) -> bool:
    """Binary CRC erasure (SBI-style): every corrupt symbol is known-erased (a
    per-symbol CRC flags it exactly). Erasure-only RS recovers iff #corrupt <=
    nsym. This is the *upper bound* of the hard scheme — a real CRC also costs
    payload overhead, which this idealisation ignores (favouring the baseline)."""
    if len(o.corrupt_positions) > nsym:
        return False
    try:
        out = rs_correct_msg(o.received, nsym, erase_pos=o.corrupt_positions)
        return out == o.true_codeword
    except ValueError:
        return False


def decode_soft(o: BlockOutcome, nsym: int) -> bool:
    """Soft-reliability GMD: rank by min|LLR|, sweep erasures, errors+erasures."""
    try:
        out, _ = soft_decode(o.received, nsym, o.reliability)
        return out == o.true_codeword
    except ValueError:
        return False


# --------------------------------------------------------------------------- #
# CLI: sweep SNR, compare the three on identical blocks.
# --------------------------------------------------------------------------- #
def run_sweep(k: int, nsym: int, trials: int, snrs: list[float],
              seed: int) -> list[tuple]:
    n = k + nsym
    rng = random.Random(seed)
    rows = []
    for snr in snrs:
        won = {"errors_only": 0, "crc_erasure": 0, "soft": 0}
        for _ in range(trials):
            msg = [rng.randrange(256) for _ in range(k)]
            cw = rs_encode_msg(msg, nsym)
            o = simulate_block(cw, snr, rng)
            won["errors_only"] += decode_errors_only(o, nsym)
            won["crc_erasure"] += decode_crc_erasure(o, nsym)
            won["soft"] += decode_soft(o, nsym)
        rows.append((snr, won["errors_only"] / trials,
                     won["crc_erasure"] / trials, won["soft"] / trials))
    return rows, n


def run_overhead_fair_sweep(k: int, s_data: int, s_crc: int, n_crc: int,
                            trials: int, snrs: list[float],
                            seed: int) -> list[tuple]:
    """Fixed on-air BYTE budget. The CRC scheme spends s_crc bytes/symbol on a
    per-symbol CRC (perfect erasure detection); the soft scheme spends the same
    bytes on EXTRA PARITY (more symbols, no detection, soft-ranked erasure)."""
    airtime = n_crc * (s_data + s_crc)
    n_soft = airtime // s_data
    nsym_crc = n_crc - k
    nsym_soft = n_soft - k
    rng = random.Random(seed)

    def corrupt_symbol(snr: float) -> tuple[bool, float]:
        ebn0 = 10 ** (snr / 10.0)
        sigma = math.sqrt(1.0 / (2.0 * ebn0))
        bad, mn = False, math.inf
        for _ in range(s_data * 8):
            y = 1.0 + rng.gauss(0.0, sigma)
            if y <= 0.0:
                bad = True
            mn = min(mn, abs(2.0 * y / (sigma * sigma)))
        return bad, mn

    rows = []
    for snr in snrs:
        crc_ok = soft_ok = 0
        for _ in range(trials):
            corrupt = sum(corrupt_symbol(snr)[0] for _ in range(n_crc))
            crc_ok += (corrupt <= nsym_crc)
            msg = [rng.randrange(256) for _ in range(k)]
            cw = rs_encode_msg(msg, nsym_soft)
            recv, rel = [], []
            for sym in cw:
                bad, mn = corrupt_symbol(snr)
                recv.append(sym ^ rng.randrange(1, 256) if bad else sym)
                rel.append(mn)
            try:
                out, _ = soft_decode(recv, nsym_soft, rel)
                soft_ok += (out == cw)
            except ValueError:
                pass
        rows.append((snr, crc_ok / trials, soft_ok / trials))
    return rows, n_crc, n_soft, nsym_crc, nsym_soft


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--mode", choices=["genie", "overhead"], default="overhead",
                   help="genie: CRC gets perfect free erasure flags (soft loses); "
                        "overhead: fixed airtime, CRC pays per-symbol bytes (fair)")
    p.add_argument("--k", type=int, default=8, help="source symbols/block")
    p.add_argument("--nsym", type=int, default=8,
                   help="parity symbols (N-K) — genie mode")
    p.add_argument("--s-data", type=int, default=32,
                   help="payload bytes/symbol — overhead mode")
    p.add_argument("--s-crc", type=int, default=4,
                   help="CRC bytes/symbol — overhead mode")
    p.add_argument("--n-crc", type=int, default=12,
                   help="CRC-scheme codeword length — overhead mode")
    p.add_argument("--trials", type=int, default=400)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--snr", type=float, nargs="*", default=[3, 4, 5, 6, 7, 8])
    a = p.parse_args()

    if a.mode == "genie":
        rows, n = run_sweep(a.k, a.nsym, a.trials, a.snr, a.seed)
        print(f"[genie] RS({n},{a.k}) GF(2^8) parity={a.nsym}; a CRC gives perfect "
              f"FREE erasure flags. {a.trials} blocks/SNR")
        print(f"{'Eb/N0':>6} | {'errors-only':>11} | {'CRC-erasure':>11} | "
              f"{'SOFT-GMD':>9} | soft-vs-best")
        print("-" * 62)
        for snr, eo, crc, soft in rows:
            print(f"{snr:6.1f} | {eo:11.3f} | {crc:11.3f} | {soft:9.3f} | "
                  f"{soft - max(eo, crc):+.3f}")
        return 0

    rows, n_crc, n_soft, nc, nso = run_overhead_fair_sweep(
        a.k, a.s_data, a.s_crc, a.n_crc, a.trials, a.snr, a.seed)
    ov = a.s_crc / (a.s_data + a.s_crc)
    print(f"[overhead-fair] CRC overhead {ov:.0%} (s_data={a.s_data}B, "
          f"s_crc={a.s_crc}B). Fixed airtime: CRC RS({n_crc},{a.k}) parity={nc} "
          f"vs SOFT RS({n_soft},{a.k}) parity={nso}. {a.trials} blocks/SNR")
    print(f"{'Eb/N0':>6} | {'CRC-erasure':>11} | {'SOFT-GMD':>9} | soft-vs-crc")
    print("-" * 48)
    for snr, crc, soft in rows:
        print(f"{snr:6.1f} | {crc:11.3f} | {soft:9.3f} | {soft - crc:+.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
