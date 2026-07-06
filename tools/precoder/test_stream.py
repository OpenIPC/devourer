"""Tests for the stream-framing layer (`stream.py`).

Covers:
  * CRC-16-CCITT against a pinned KAT (catches polynomial / init regressions);
  * BCC + interleaver linearity — `compute_generator` is the actual Jacobian
    of the pipeline in info bits and `compute_state_offset` is the affine
    constant for a given entry state;
  * GF(2) RREF solver — particular satisfies Ax=b, null basis is in ker(A),
    and the encode/decode free-column convention is symmetric;
  * Byte-mode and shape-mode envelope round-trips, including non-default
    scrambler offset / BCC entry_state (mid-frame placement);
  * Shape-mode subcarriers actually honour the pin pattern when the encoded
    body is fed back through `emulate_chip`;
  * `pack_stream` / `unpack_stream` end-to-end byte conservation, dedup, gap
    detection;
  * Inconsistent / over-constrained shape masks are rejected.
"""

from __future__ import annotations

import numpy as np
import pytest

import encode_subcarriers as enc
import stream


PHY = enc._LEGACY_BPSK


# --------------------------------------------------------------------------- #
# CRC-16-CCITT KAT
# --------------------------------------------------------------------------- #
def test_crc16_ccitt_kat_123456789():
    # Canonical CRC-16-CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflect, xorout 0)
    # of the string "123456789" -> 0x29B1.
    assert stream.crc16_ccitt(b"123456789") == 0x29B1


def test_crc16_ccitt_empty_is_init():
    assert stream.crc16_ccitt(b"") == 0xFFFF


def test_crc16_ccitt_detects_single_bit_flip():
    rng = np.random.default_rng(0)
    data = bytes(rng.integers(0, 256, size=64, dtype=np.uint8).tolist())
    bad = bytearray(data)
    bad[7] ^= 0x10
    assert stream.crc16_ccitt(data) != stream.crc16_ccitt(bytes(bad))


# --------------------------------------------------------------------------- #
# Generator matrix
# --------------------------------------------------------------------------- #
def test_generator_matrix_shape():
    M = stream.compute_generator(PHY)
    assert M.shape == (PHY.n_cbps, PHY.n_dbps)
    assert M.dtype == np.uint8
    assert set(np.unique(M).tolist()) <= {0, 1}


def test_generator_matrix_is_bcc_jacobian():
    # M @ info ⊕ b(entry_state) == interleave(bcc_encode(info, entry_state))
    rng = np.random.default_rng(1)
    M = stream.compute_generator(PHY)
    for entry_state in (0, 7, 0x2A, 0x3F):
        b = stream.compute_state_offset(entry_state, PHY)
        for _ in range(5):
            info = rng.integers(0, 2, size=PHY.n_dbps, dtype=np.uint8)
            coded = enc.bcc_encode(info, init_state=entry_state)
            sub_direct = enc.interleave(coded, PHY)
            sub_via_M = ((M @ info) ^ b) & 1
            assert np.array_equal(sub_direct, sub_via_M)


def test_state_offset_state_zero_is_zero():
    # bcc_encode(0, state=0) -> 0; interleave(0) -> 0.
    assert not stream.compute_state_offset(0, PHY).any()


# --------------------------------------------------------------------------- #
# GF(2) solver
# --------------------------------------------------------------------------- #
def test_gf2_solve_empty_constraints_returns_identity_basis():
    A = np.zeros((0, 5), dtype=np.uint8)
    b = np.zeros(0, dtype=np.uint8)
    sol = stream.gf2_solve(A, b)
    assert sol.consistent
    assert sol.n_free == 5
    assert np.array_equal(sol.null_basis, np.eye(5, dtype=np.uint8))
    assert np.array_equal(sol.free_cols, np.arange(5))


def test_gf2_solve_particular_satisfies_constraints():
    rng = np.random.default_rng(2)
    for _ in range(20):
        m = int(rng.integers(1, 20))
        n = int(rng.integers(m, m + 8))
        A = rng.integers(0, 2, size=(m, n), dtype=np.uint8)
        x = rng.integers(0, 2, size=n, dtype=np.uint8)
        b = (A @ x) & 1
        sol = stream.gf2_solve(A, b)
        assert sol.consistent
        assert np.array_equal((A @ sol.particular) & 1, b)


def test_gf2_solve_null_basis_is_in_kernel():
    rng = np.random.default_rng(3)
    A = rng.integers(0, 2, size=(10, 24), dtype=np.uint8)
    b = rng.integers(0, 2, size=10, dtype=np.uint8)
    sol = stream.gf2_solve(A, b)
    assert sol.consistent
    for j in range(sol.n_free):
        assert not ((A @ sol.null_basis[:, j]) & 1).any()


def test_gf2_solve_free_cols_convention_round_trip():
    # data[j] = (particular ⊕ null @ data)[free_cols[j]] ⊕ particular[free_cols[j]]
    rng = np.random.default_rng(4)
    A = rng.integers(0, 2, size=(12, 24), dtype=np.uint8)
    b = rng.integers(0, 2, size=12, dtype=np.uint8)
    sol = stream.gf2_solve(A, b)
    assert sol.consistent
    data = rng.integers(0, 2, size=sol.n_free, dtype=np.uint8)
    x = (sol.particular ^ ((sol.null_basis @ data) & 1)).astype(np.uint8)
    delta = (x ^ sol.particular) & 1
    recovered = delta[sol.free_cols].astype(np.uint8)
    assert np.array_equal(recovered, data)


def test_gf2_solve_detects_inconsistency():
    A = np.array([[1, 1], [1, 1]], dtype=np.uint8)
    b = np.array([0, 1], dtype=np.uint8)
    sol = stream.gf2_solve(A, b)
    assert not sol.consistent


def test_gf2_rank_matches_dim():
    rng = np.random.default_rng(5)
    A = rng.integers(0, 2, size=(20, 24), dtype=np.uint8)
    r = stream.gf2_rank(A)
    sol = stream.gf2_solve(A, np.zeros(A.shape[0], dtype=np.uint8))
    assert r == A.shape[1] - sol.n_free
    assert 0 <= r <= min(A.shape)


# --------------------------------------------------------------------------- #
# Shape normalisation
# --------------------------------------------------------------------------- #
def test_normalise_shape_dict_and_array_agree():
    arr = np.zeros(PHY.n_sd, dtype=np.int8)
    arr[3] = 1
    arr[17] = -1
    arr[40] = 1
    idx_a, vals_a = stream.normalise_shape(arr, PHY.n_sd)
    idx_d, vals_d = stream.normalise_shape({3: 1, 17: -1, 40: 1}, PHY.n_sd)
    assert np.array_equal(idx_a, idx_d)
    assert np.array_equal(vals_a, vals_d)


def test_normalise_shape_rejects_bad_values():
    with pytest.raises(ValueError):
        stream.normalise_shape({0: 2}, PHY.n_sd)
    with pytest.raises(ValueError):
        stream.normalise_shape({999: 1}, PHY.n_sd)


def test_normalise_shape_none_is_empty():
    idx, vals = stream.normalise_shape(None, PHY.n_sd)
    assert idx.size == 0 and vals.size == 0


# --------------------------------------------------------------------------- #
# Envelope round-trip
# --------------------------------------------------------------------------- #
def test_envelope_round_trip_default_body():
    f = stream.StreamFrame(seq=0x1234, total=7, payload=b"hello, world!!")
    body = f.envelope_bytes(stream.DEFAULT_BODY_BYTES)
    assert len(body) == stream.DEFAULT_BODY_BYTES
    out = stream.parse_envelope(body)
    assert out == f


def test_envelope_rejects_bad_magic():
    f = stream.StreamFrame(seq=1, total=1, payload=b"abc")
    body = bytearray(f.envelope_bytes(24))
    body[0] ^= 0xFF
    assert stream.parse_envelope(bytes(body)) is None


def test_envelope_rejects_bad_crc():
    f = stream.StreamFrame(seq=1, total=1, payload=b"abc")
    body = bytearray(f.envelope_bytes(24))
    # CRC sits at HEADER_LEN + plen .. HEADER_LEN + plen + TRAILER_LEN; the
    # trailing pad after it is ignored by the parser. Flip a CRC byte to
    # isolate the CRC check from the (separately-tested) trailer behaviour.
    body[stream.HEADER_LEN + len(f.payload)] ^= 0x01
    assert stream.parse_envelope(bytes(body)) is None


def test_envelope_ignores_trailing_chip_bytes():
    # Simulates the chip-side body dump: 4-byte FCS + assorted trailer past
    # the envelope. The parser must still recover the frame from PLEN alone.
    f = stream.StreamFrame(seq=0x4242, total=9, payload=b"hello stream!!")
    body = f.envelope_bytes(24)
    chip_view = body + b"\xde\xad\xbe\xef" + b"\x00" * 24
    out = stream.parse_envelope(chip_view)
    assert out == f


def test_envelope_rejects_oversized_plen():
    f = stream.StreamFrame(seq=0, total=0, payload=b"x" * 4)
    body = bytearray(f.envelope_bytes(24))
    # Bump PLEN past the body size, recompute CRC so we isolate the PLEN check.
    body[6:8] = (200).to_bytes(2, "little")
    crc = stream.crc16_ccitt(bytes(body[:-2]))
    body[-2:] = crc.to_bytes(2, "little")
    assert stream.parse_envelope(bytes(body)) is None


def test_envelope_rejects_oversized_payload():
    f = stream.StreamFrame(seq=0, total=0, payload=b"x" * 100)
    with pytest.raises(ValueError):
        f.envelope_bytes(24)


# --------------------------------------------------------------------------- #
# Byte-mode encode / decode
# --------------------------------------------------------------------------- #
def test_byte_mode_encode_decode_round_trip():
    f = stream.StreamFrame(seq=42, total=3, payload=b"streamtest!!aa")
    body, layout = stream.encode_body(f)
    assert layout.body_bytes == stream.DEFAULT_BODY_BYTES
    assert layout.capacity_per_sym == PHY.n_dbps
    assert layout.payload_capacity == stream.DEFAULT_BODY_BYTES - stream.ENVELOPE_LEN
    out = stream.decode_body(body)
    assert out == f


def test_byte_mode_respects_custom_body_size():
    f = stream.StreamFrame(seq=1, total=1, payload=b"")
    body, layout = stream.encode_body(f, body_bytes=48)
    assert layout.body_bytes == 48
    assert layout.n_sym == 16  # 48 bytes / 3 bytes per legacy OFDM symbol
    assert stream.decode_body(body) == f


def test_byte_mode_rejects_misaligned_body_size():
    f = stream.StreamFrame(seq=0, total=0, payload=b"")
    with pytest.raises(ValueError):
        stream.encode_body(f, body_bytes=25)  # not a multiple of 3 bytes/symbol


# --------------------------------------------------------------------------- #
# Shape-mode encode / decode + on-air subcarrier verification
# --------------------------------------------------------------------------- #
def _check_shape_honoured(body: bytes, shape: dict, seed: int,
                          offset: int, entry_state: int, n_sym: int):
    """Feed `body` through emulate_chip and verify the pinned subcarriers
    match the requested ±1 values for every body symbol."""
    psdu_bits = enc.bytes_to_bits(body)[: n_sym * PHY.n_dbps]
    sub = enc.emulate_chip(psdu_bits, seed, PHY, n_sym,
                           offset=offset, entry_state=entry_state)
    for sc, want in shape.items():
        assert np.all(sub[:, sc] == want), \
            f"subcarrier {sc}: want {want}, got {sub[:, sc].tolist()}"


def test_shape_mode_round_trip_simple_pin():
    f = stream.StreamFrame(seq=0xABCD, total=2, payload=b"abc")
    shape = {0: +1, 10: -1, 20: +1}
    body, layout = stream.encode_body(f, shape=shape)
    assert layout.capacity_per_sym == PHY.n_dbps - len(shape)
    assert layout.n_sym >= (stream.ENVELOPE_LEN + 3) * 8 // layout.capacity_per_sym
    out = stream.decode_body(body, shape=shape)
    assert out == f
    _check_shape_honoured(body, shape, enc.DEFAULT_SEED, 0, 0, layout.n_sym)


def test_shape_mode_full_array_spec():
    arr = np.zeros(PHY.n_sd, dtype=np.int8)
    arr[5] = +1
    arr[6] = -1
    arr[30] = +1
    f = stream.StreamFrame(seq=1, total=1, payload=b"x")
    body, layout = stream.encode_body(f, shape=arr)
    out = stream.decode_body(body, shape=arr)
    assert out == f
    shape_dict = {int(i): int(v) for i, v in zip(np.nonzero(arr)[0], arr[arr != 0])}
    _check_shape_honoured(body, shape_dict, enc.DEFAULT_SEED, 0, 0, layout.n_sym)


def test_shape_mode_alternating_quarter_pattern():
    # Pin every fourth subcarrier on an alternating ±1 (12 constraints).
    shape = {k: (+1 if (k // 4) % 2 == 0 else -1) for k in range(0, PHY.n_sd, 4)}
    f = stream.StreamFrame(seq=9, total=10, payload=b"hi")
    body, layout = stream.encode_body(f, shape=shape)
    assert layout.capacity_per_sym == PHY.n_dbps - stream.gf2_rank(
        stream.compute_generator(PHY)[np.array(sorted(shape.keys())), :]
    )
    out = stream.decode_body(body, shape=shape)
    assert out == f
    _check_shape_honoured(body, shape, enc.DEFAULT_SEED, 0, 0, layout.n_sym)


def test_shape_mode_mid_frame_offset_and_entry_state():
    # The README's body offset for precoder: SERVICE(16) + 24B header.
    rng = np.random.default_rng(6)
    prefix = rng.integers(0, 2, size=16 + 24 * 8, dtype=np.uint8)
    entry_state = enc.bcc_final_state(prefix)
    offset = len(prefix)

    shape = {2: -1, 14: +1, 33: -1}
    f = stream.StreamFrame(seq=7, total=0, payload=b"mid")
    body, layout = stream.encode_body(
        f, shape=shape, offset=offset, entry_state=entry_state
    )
    out = stream.decode_body(
        body, shape=shape, offset=offset, entry_state=entry_state
    )
    assert out == f
    _check_shape_honoured(body, shape, enc.DEFAULT_SEED, offset, entry_state,
                          layout.n_sym)


def test_shape_mode_rejects_inconsistent_pins():
    # Pin two subcarriers that share the same coded-bit row of M to opposite
    # values — the constraint becomes 0 = 1 mod 2 at that row.
    M = stream.compute_generator(PHY)
    # find two rows of M that are identical (extremely common for BCC+interleaver).
    pairs = []
    for i in range(M.shape[0]):
        for j in range(i + 1, M.shape[0]):
            if np.array_equal(M[i, :], M[j, :]):
                # `b(entry_state)` evaluated at state=0 is zero, so for a pair
                # of subcarriers with the same M-row the chip emits
                # sub[i] == sub[j] when scrambler XOR is also equal at those
                # subcarrier positions. We force inconsistency at the M[S,:]
                # level by requesting opposite ±1: rhs becomes 0 vs 1.
                pairs.append((i, j))
                break
        if pairs:
            break
    if not pairs:
        pytest.skip("M has no duplicate rows — no easy inconsistency to force")
    i, j = pairs[0]
    shape = {i: +1, j: -1}
    f = stream.StreamFrame(seq=0, total=0, payload=b"")
    with pytest.raises(ValueError):
        stream.encode_body(f, shape=shape)


def test_shape_mode_full_rank_pin_is_rejected():
    # Pin enough subcarriers to drive rank to 24 — no room for framing bits.
    M = stream.compute_generator(PHY)
    # Greedy independent-row pick to reach rank 24.
    chosen: list[int] = []
    A = np.zeros((0, PHY.n_dbps), dtype=np.uint8)
    for r in range(M.shape[0]):
        cand = np.vstack([A, M[r, :]])
        if stream.gf2_rank(cand) > stream.gf2_rank(A):
            chosen.append(r)
            A = cand
            if stream.gf2_rank(A) == PHY.n_dbps:
                break
    assert stream.gf2_rank(A) == PHY.n_dbps
    shape = {sc: +1 for sc in chosen}
    f = stream.StreamFrame(seq=0, total=0, payload=b"")
    with pytest.raises(ValueError, match="rank=24"):
        stream.encode_body(f, shape=shape)


# --------------------------------------------------------------------------- #
# Stream helpers
# --------------------------------------------------------------------------- #
def test_pack_stream_chunks_at_mtu():
    data = bytes(range(50))
    frames = stream.pack_stream(data, mtu=14)
    assert len(frames) == 4  # 14 + 14 + 14 + 8
    assert frames[0].seq == 0 and frames[-1].seq == 3
    assert all(f.total == 4 for f in frames)
    assert b"".join(f.payload for f in frames) == data


def test_pack_stream_seq_start_and_wraparound():
    data = bytes(range(30))
    frames = stream.pack_stream(data, mtu=14, seq_start=0xFFFE)
    assert [f.seq for f in frames] == [0xFFFE, 0xFFFF, 0x0000]


def test_pack_stream_empty():
    assert stream.pack_stream(b"", mtu=14) == []


def test_unpack_stream_drops_duplicates_and_stops_at_total():
    frames = [
        stream.StreamFrame(seq=0, total=3, payload=b"AAA"),
        stream.StreamFrame(seq=2, total=3, payload=b"CCC"),
        stream.StreamFrame(seq=0, total=3, payload=b"AAA"),  # dup
        stream.StreamFrame(seq=1, total=3, payload=b"BBB"),
        stream.StreamFrame(seq=4, total=3, payload=b"ignored"),  # past total
    ]
    out = list(stream.unpack_stream(frames))
    assert out == [(0, b"AAA"), (2, b"CCC"), (1, b"BBB")]


def test_pack_unpack_round_trip_reorders_by_seq_at_caller():
    data = bytes(range(64))
    frames = stream.pack_stream(data, mtu=14)
    received = sorted(stream.unpack_stream(frames), key=lambda kv: kv[0])
    assert b"".join(payload for _, payload in received) == data


# --------------------------------------------------------------------------- #
# End-to-end: stream over byte-mode bodies
# --------------------------------------------------------------------------- #
def test_byte_mode_end_to_end_byte_stream():
    rng = np.random.default_rng(7)
    data = bytes(rng.integers(0, 256, size=200, dtype=np.uint8).tolist())
    frames = stream.pack_stream(data)
    bodies = [stream.encode_body(f)[0] for f in frames]
    decoded = [stream.decode_body(b) for b in bodies]
    assert all(d is not None for d in decoded)
    out = sorted(((d.seq, d.payload) for d in decoded), key=lambda kv: kv[0])
    assert b"".join(p for _, p in out) == data


def test_shape_mode_end_to_end_byte_stream():
    rng = np.random.default_rng(8)
    data = bytes(rng.integers(0, 256, size=120, dtype=np.uint8).tolist())
    shape = {0: +1, 8: -1, 16: +1, 24: -1, 32: +1}
    frames = stream.pack_stream(data)
    bodies_and_layouts = [stream.encode_body(f, shape=shape) for f in frames]
    decoded = [stream.decode_body(b, shape=shape) for b, _ in bodies_and_layouts]
    assert all(d is not None for d in decoded)
    out = sorted(((d.seq, d.payload) for d in decoded), key=lambda kv: kv[0])
    assert b"".join(p for _, p in out) == data
    # And every body honours the shape on-air.
    for (body, layout), f in zip(bodies_and_layouts, frames):
        _check_shape_honoured(body, shape, enc.DEFAULT_SEED, 0, 0, layout.n_sym)
