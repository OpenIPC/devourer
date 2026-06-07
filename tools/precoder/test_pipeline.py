"""Unit tests for the pre-modulator encoder pipeline.

Pure software — no hardware, no libusb. Runs under the uv project env:

    cd tools/precoder && uv run pytest

The known-answer tests pin the actual standard:
  * scrambler -> recurrence o[m]=o[m-4]^o[m-7] (polynomial x^7+x^4+1) + maximal
    length (period 127, 64 ones per period);
  * BCC -> impulse response equals the (133,171) generators;
  * interleaver -> bijection + self-inverse;
  * full pipeline -> bit-exact round-trip for representable targets, and the
    nearest-pattern Hamming-distance contract for arbitrary ones.
"""

from __future__ import annotations

import numpy as np
import pytest

import encode_subcarriers as enc

PHYS = [enc.phy_params("ht"), enc.phy_params("legacy")]
PHY_IDS = ["ht", "legacy"]


# --------------------------------------------------------------------------- #
# Scrambler
# --------------------------------------------------------------------------- #
def test_scrambler_recurrence_pins_polynomial():
    # o[m] = o[m-4] ^ o[m-7] is exactly the polynomial 1 + x^4 + x^7.
    seq = enc.scrambler_sequence(0x5D, 300)
    m = np.arange(7, len(seq))
    assert np.array_equal(seq[m], seq[m - 4] ^ seq[m - 7])


@pytest.mark.parametrize("seed", [1, 0x5D, 0x7F, 42])
def test_scrambler_maximal_length(seed):
    seq = enc.scrambler_sequence(seed, 254)
    # Period exactly 127 (prime) and 64 ones per period => maximal m-sequence.
    assert np.array_equal(seq[:127], seq[127:254])
    assert int(seq[:127].sum()) == 64


def test_scrambler_is_self_inverse():
    rng = np.random.default_rng(0)
    bits = rng.integers(0, 2, size=200, dtype=np.uint8)
    once = enc.apply_scrambler(bits, 0x5D)
    twice = enc.apply_scrambler(once, 0x5D)
    assert np.array_equal(bits, twice)
    assert not np.array_equal(bits, once)  # it actually did something


def test_scrambler_offset_matches_slice():
    full = enc.scrambler_sequence(0x5D, 100)
    bits = np.zeros(20, dtype=np.uint8)
    # scrambling zeros yields the sequence itself; offset must select a window.
    assert np.array_equal(enc.apply_scrambler(bits, 0x5D, offset=33), full[33:53])


# --------------------------------------------------------------------------- #
# BCC
# --------------------------------------------------------------------------- #
def test_bcc_impulse_response_equals_generators():
    info = np.zeros(7, dtype=np.uint8)
    info[0] = 1
    coded = enc.bcc_encode(info)
    a_stream = coded[0::2]
    b_stream = coded[1::2]
    # 0o133 = 1011011, 0o171 = 1111001 (MSB first).
    assert np.array_equal(a_stream, [1, 0, 1, 1, 0, 1, 1])
    assert np.array_equal(b_stream, [1, 1, 1, 1, 0, 0, 1])


def test_bcc_viterbi_recovers_clean_codeword():
    rng = np.random.default_rng(1)
    info = rng.integers(0, 2, size=60, dtype=np.uint8)
    coded = enc.bcc_encode(info)
    recovered, dist = enc.bcc_viterbi_preimage(coded)
    assert dist == 0
    assert np.array_equal(recovered, info)


def test_bcc_viterbi_corrects_single_error():
    rng = np.random.default_rng(2)
    info = rng.integers(0, 2, size=80, dtype=np.uint8)
    coded = enc.bcc_encode(info).copy()
    coded[10] ^= 1  # inject one bit error mid-stream
    recovered, dist = enc.bcc_viterbi_preimage(coded)
    # K=7 free distance is 10, so a single error is well within correction.
    assert np.array_equal(recovered, info)
    assert dist == 1


# --------------------------------------------------------------------------- #
# Interleaver
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("p", PHYS, ids=PHY_IDS)
def test_interleaver_is_a_bijection(p):
    perm = enc.interleaver_perm(p)
    assert np.array_equal(np.sort(perm), np.arange(p.n_cbps))


@pytest.mark.parametrize("p", PHYS, ids=PHY_IDS)
def test_interleave_deinterleave_inverse(p):
    rng = np.random.default_rng(3)
    coded = rng.integers(0, 2, size=p.n_cbps, dtype=np.uint8)
    assert np.array_equal(enc.deinterleave(enc.interleave(coded, p), p), coded)


# --------------------------------------------------------------------------- #
# BPSK + byte packing
# --------------------------------------------------------------------------- #
def test_bpsk_map_demap_inverse():
    bits = np.array([0, 1, 0, 1, 1, 0], dtype=np.uint8)
    assert np.array_equal(enc.bpsk_map(bits), [1, -1, 1, -1, -1, 1])
    assert np.array_equal(enc.bpsk_demap(enc.bpsk_map(bits)), bits)


def test_bits_bytes_roundtrip():
    rng = np.random.default_rng(4)
    bits = rng.integers(0, 2, size=26, dtype=np.uint8)  # not a byte multiple
    data = enc.bits_to_bytes(bits)
    assert np.array_equal(enc.bytes_to_bits(data, n=len(bits)), bits)


# --------------------------------------------------------------------------- #
# Full pipeline round-trip
# --------------------------------------------------------------------------- #
def _representable_target(p, seed, n_sym, offset, rng):
    """Build a target that IS reachable by the code: pick random info, run the
    forward model, use its constellation as the target."""
    info_bits = rng.integers(0, 2, size=n_sym * p.n_dbps, dtype=np.uint8)
    psdu_bits = enc.apply_scrambler(info_bits, seed, offset)  # info domain -> PSDU
    target = enc.emulate_chip(psdu_bits, seed, p, n_sym, offset)
    return target, psdu_bits


@pytest.mark.parametrize("p", PHYS, ids=PHY_IDS)
@pytest.mark.parametrize("n_sym", [1, 3])
@pytest.mark.parametrize("offset", [0, 16])
def test_representable_roundtrip_is_exact(p, n_sym, offset):
    rng = np.random.default_rng(5)
    seed = 0x5D
    target, psdu_bits = _representable_target(p, seed, n_sym, offset, rng)

    res = enc.encode_pattern(target, seed=seed, phy=p, offset=offset)
    assert res.representable
    assert res.hamming_distance == 0
    assert np.array_equal(res.psdu_bits, psdu_bits)

    # The chip emulation of our bytes reproduces the requested constellation.
    back = enc.emulate_chip(res.psdu_bits, seed, p, n_sym, offset)
    assert np.array_equal(back, target)


@pytest.mark.parametrize("p", PHYS, ids=PHY_IDS)
def test_arbitrary_target_nearest_distance_contract(p):
    rng = np.random.default_rng(6)
    seed = 0x11
    target = rng.choice([1, -1], size=(2, p.n_sd)).astype(np.int8)

    res = enc.encode_pattern(target, seed=seed, phy=p, offset=0)
    # A random target is essentially never a codeword (2^-N_DBPS per symbol).
    assert res.hamming_distance > 0
    back = enc.emulate_chip(res.psdu_bits, seed, p, n_sym=2, offset=0)
    # The emulated pattern differs from the target in exactly H subcarriers.
    assert int(np.count_nonzero(back != target)) == res.hamming_distance


def test_bcc_final_state_matches_continuation():
    rng = np.random.default_rng(8)
    prefix = rng.integers(0, 2, size=200, dtype=np.uint8)
    tail = rng.integers(0, 2, size=60, dtype=np.uint8)
    # Encoding prefix+tail in one shot == encoding tail from prefix's final state.
    joined = enc.bcc_encode(np.concatenate([prefix, tail]))
    state = enc.bcc_final_state(prefix)
    continued = enc.bcc_encode(tail, init_state=state)
    assert np.array_equal(joined[2 * len(prefix):], continued)


@pytest.mark.parametrize("p", PHYS, ids=PHY_IDS)
def test_mid_frame_entry_state_roundtrip(p):
    # Exact control of a symbol that follows a SERVICE field + MAC header: the
    # BCC state entering it is non-zero, derived from the preceding bits.
    rng = np.random.default_rng(9)
    seed = 0x5D
    prefix_scrambled = rng.integers(0, 2, size=16 + 24 * 8, dtype=np.uint8)
    entry = enc.bcc_final_state(prefix_scrambled)
    assert entry != 0  # the whole point: header leaves a non-zero state

    info = rng.integers(0, 2, size=2 * p.n_dbps, dtype=np.uint8)
    psdu = enc.apply_scrambler(info, seed, offset=len(prefix_scrambled))
    target = enc.emulate_chip(psdu, seed, p, 2,
                              offset=len(prefix_scrambled), entry_state=entry)

    res = enc.encode_pattern(target, seed=seed, phy=p,
                             offset=len(prefix_scrambled), entry_state=entry)
    assert res.representable
    assert np.array_equal(res.psdu_bits, psdu)


def test_strict_refuses_non_representable():
    p = enc.phy_params("ht")
    rng = np.random.default_rng(7)
    target = rng.choice([1, -1], size=p.n_sd).astype(np.int8)
    with pytest.raises(ValueError):
        enc.encode_pattern(target, phy=p, strict=True)


def test_alternating_example_runs_for_both_phys():
    for p in PHYS:
        target = enc.alternating_pattern(p.n_sd)
        res = enc.encode_pattern(target, phy=p)  # non-strict: nearest is fine
        assert len(res.psdu_bits) == p.n_dbps
        back = enc.emulate_chip(res.psdu_bits, res.seed, p, n_sym=1)
        assert int(np.count_nonzero(back != target)) == res.hamming_distance
