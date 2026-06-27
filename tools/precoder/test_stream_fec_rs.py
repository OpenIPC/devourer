"""Tests for the Reed-Solomon outer scheme (`stream_fec_rs.py`).

Covers MDS recovery (any K of N reconstruct), the erasure limit (exactly R
recoverable, R+1 not), systematic passthrough, partial-block flush, block-id
wrap, and the RS+SBI fusion (RS as the outer code under sub-block salvage).
"""

from __future__ import annotations

import random

import pytest

import fec_subblock
import stream_fec
import stream_fec_rs
from stream_fec import FecConfig, make_decoder, make_encoder


def _encode(packets: list[bytes], cfg: FecConfig) -> list[bytes]:
    enc = make_encoder(cfg)
    envs: list[bytes] = []
    for p in packets:
        envs += enc.add_packet(p)
    envs += enc.flush()
    return envs


def _decode(envs: list[bytes], cfg: FecConfig) -> list[bytes]:
    dec = make_decoder(cfg)
    out: list[bytes] = []
    for e in envs:
        out += dec.add_symbol(e)
    return out


# --------------------------------------------------------------------------- #
# GF(2^8) sanity
# --------------------------------------------------------------------------- #
def test_gf_field_axioms():
    for a in range(1, 256):
        assert stream_fec_rs._mul(a, stream_fec_rs._inv(a)) == 1
    # distributivity spot-check
    assert stream_fec_rs._mul(0x53, 0xCA) == stream_fec_rs._mul(0xCA, 0x53)


def test_encoding_matrix_is_systematic():
    A = stream_fec_rs._encoding_matrix(8, 12)
    for i in range(8):
        assert A[i] == [1 if j == i else 0 for j in range(8)]


# --------------------------------------------------------------------------- #
# MDS recovery
# --------------------------------------------------------------------------- #
def test_round_trip_no_loss():
    cfg = FecConfig(k=8, symbol_size=64, overhead=0.5, scheme="rs")  # N=12
    pkts = [f"msg-{i:02d}-".encode() * 4 for i in range(8)]
    assert _decode(_encode(pkts, cfg), cfg) == pkts


def test_any_k_of_n_reconstruct():
    """MDS: every K-subset of the N symbols reconstructs the block."""
    import itertools
    cfg = FecConfig(k=4, symbol_size=48, overhead=0.5, scheme="rs")  # N=6
    pkts = [f"p{i}".encode() * 10 for i in range(4)]
    envs = _encode(pkts, cfg)
    assert len(envs) == 6
    for keep in itertools.combinations(range(6), 4):
        subset = [envs[i] for i in keep]
        assert _decode(subset, cfg) == pkts, f"subset {keep} failed"


def test_erasure_limit_R_ok_Rplus1_fails():
    cfg = FecConfig(k=6, symbol_size=64, overhead=0.5, scheme="rs")  # N=9, R=3
    pkts = [f"data-{i}".encode() * 6 for i in range(6)]
    envs = _encode(pkts, cfg)
    assert len(envs) == 9
    rng = random.Random(7)

    # Drop exactly R=3 -> recovers.
    e = envs[:]
    rng.shuffle(e)
    assert _decode(e[3:], cfg) == pkts

    # Drop R+1=4 -> only 5 < K=6 symbols -> cannot recover.
    assert _decode(e[4:], cfg) == []


def test_partial_flush_kreal():
    cfg = FecConfig(k=8, symbol_size=64, overhead=0.5, scheme="rs")
    pkts = [f"only-{i}".encode() * 3 for i in range(3)]  # < 1 symbol each
    # 3 tiny packets pack into 1 source symbol -> kreal=1.
    assert _decode(_encode(pkts, cfg), cfg) == pkts


def test_block_id_wrap():
    cfg = FecConfig(k=2, symbol_size=32, overhead=0.5, scheme="rs")
    enc = make_encoder(cfg)
    enc._block_id = 0xFFFF
    dec = make_decoder(cfg)
    out: list[bytes] = []
    for i in range(6):  # several blocks across the wrap
        for e in enc.add_packet(f"x{i}".encode() * 8):
            out += dec.add_symbol(e)
    for e in enc.flush():
        out += dec.add_symbol(e)
    assert out == [f"x{i}".encode() * 8 for i in range(6)]


def test_rejects_oversized_n():
    with pytest.raises(ValueError):
        make_encoder(FecConfig(k=200, symbol_size=32, overhead=1.0, scheme="rs"))


# --------------------------------------------------------------------------- #
# RS + SBI fusion
# --------------------------------------------------------------------------- #
def test_rs_with_sbi_under_ubiquitous_ber():
    cfg = FecConfig(k=8, symbol_size=64, overhead=0.5, scheme="rs")  # N=12, R=4
    pkts = [f"frame-{i:02d} payload".encode() for i in range(8)]
    envs = _encode(pkts, cfg)
    env_size = len(envs[0])
    assert all(len(e) == env_size for e in envs)

    bodies = fec_subblock.pack(envs, block_payload=env_size, blocks_per_body=3)
    rng = random.Random(99)
    dec = make_decoder(cfg)
    out: list[bytes] = []
    failed = blocks = 0
    for body in bodies:
        # Localized hit: corrupt one sub-block per frame (fails its FCS).
        b = bytearray(body)
        stride = 2 + env_size
        n = (len(b) - fec_subblock.SBI_HDR_LEN) // stride
        j = rng.randrange(n)
        base = fec_subblock.SBI_HDR_LEN + j * stride + 2
        b[base + rng.randrange(env_size)] ^= 0xFF
        res = fec_subblock.unpack(bytes(b), block_payload=env_size)
        blocks += res.n_blocks
        failed += res.n_failed
        for env in res.survivors:
            out += dec.add_symbol(env)

    # ~1/3 sub-blocks lost (4 of 12 here) — within the R=4 erasure budget.
    assert 0 < failed <= 4
    assert out == pkts
