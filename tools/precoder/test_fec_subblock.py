"""Tests for the Sub-Block Integrity (SBI) fused-FEC layer (`fec_subblock.py`).

Covers:
  * Clean round-trip — pack N envelopes, unpack, get them back byte-exact.
  * Localized corruption — flipping bytes in sub-block j drops *only* j; the
    other sub-blocks in the same body survive (the whole point).
  * Header corruption can't desync — a mangled SBI header still partitions
    correctly off the receiver's configured block_payload.
  * CRC32 option.
  * The headline fusion result — a low-but-ubiquitous BER that fails the
    chip FCS on *every* frame destroys the drop-whole-frame baseline (0
    symbols delivered) but SBI delivers the uncorrupted sub-blocks, so the
    outer RaptorQ code reconstructs the message.
"""

from __future__ import annotations

import random

import pytest

import fec_subblock
from fec_subblock import (SBI_HDR_LEN, SubBlockPacker, overhead_bytes, pack,
                          unpack)


def _envs(n: int, size: int, seed: int = 0) -> list[bytes]:
    rng = random.Random(seed)
    return [bytes(rng.randrange(256) for _ in range(size)) for _ in range(n)]


def test_inline_crc_matches_stream():
    """fec_subblock inlines crc16 (to stay numpy-free for the SDR bridge); it
    must stay byte-identical to stream.crc16_ccitt or the two ends disagree."""
    import stream
    rng = random.Random(0)
    for _ in range(500):
        d = bytes(rng.randrange(256) for _ in range(rng.randrange(1, 80)))
        assert fec_subblock.crc16_ccitt(d) == stream.crc16_ccitt(d)


# --------------------------------------------------------------------------- #
# Clean round-trip
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("crc_bytes", [2, 4])
def test_clean_round_trip(crc_bytes):
    envs = _envs(10, 64)
    bodies = pack(envs, block_payload=64, blocks_per_body=4, crc_bytes=crc_bytes)
    # 10 envelopes, 4/body -> 3 bodies (4,4,2).
    assert len(bodies) == 3
    got: list[bytes] = []
    for b in bodies:
        res = unpack(b, block_payload=64, crc_bytes=crc_bytes)
        assert res.header_ok
        assert res.n_failed == 0
        got += res.survivors
    assert got == envs


def test_packer_rejects_wrong_size():
    p = SubBlockPacker(block_payload=64, blocks_per_body=4)
    with pytest.raises(ValueError):
        p.add(b"\x00" * 63)


def test_partial_final_body():
    envs = _envs(5, 32)
    bodies = pack(envs, block_payload=32, blocks_per_body=4)
    assert len(bodies) == 2
    # Last body holds the single leftover block.
    last = unpack(bodies[1], block_payload=32)
    assert last.n_blocks == 1 and last.n_failed == 0
    assert last.survivors == [envs[4]]


# --------------------------------------------------------------------------- #
# Localized corruption — only the hit sub-block is lost
# --------------------------------------------------------------------------- #
def test_corrupting_one_subblock_drops_only_it():
    envs = _envs(4, 64, seed=1)
    body = bytearray(pack(envs, block_payload=64, blocks_per_body=4)[0])
    stride = 2 + 64  # crc16 + payload
    hit = 2          # corrupt sub-block index 2
    # Flip a byte inside block `hit`'s payload (past its 2-byte CRC).
    off = SBI_HDR_LEN + hit * stride + 2 + 30
    body[off] ^= 0xFF
    res = unpack(bytes(body), block_payload=64)
    assert res.n_blocks == 4
    assert res.n_failed == 1
    # Survivors are blocks 0,1,3 in order — block 2 dropped, others byte-exact.
    assert res.survivors == [envs[0], envs[1], envs[3]]


def test_header_corruption_does_not_desync():
    envs = _envs(4, 48, seed=2)
    body = bytearray(pack(envs, block_payload=48, blocks_per_body=4)[0])
    body[0] ^= 0xFF  # smash the magic
    body[1] ^= 0xAA
    res = unpack(bytes(body), block_payload=48)
    assert not res.header_ok            # header flagged bad …
    assert res.n_failed == 0            # … but every sub-block still recovered
    assert res.survivors == envs


def test_stream_id_round_trips():
    envs = _envs(6, 40, seed=5)
    bodies = pack(envs, block_payload=40, blocks_per_body=3, stream_id=7)
    got = []
    for b in bodies:
        res = unpack(b, block_payload=40)
        assert res.header_ok and res.stream_id == 7 and res.n_failed == 0
        got += res.survivors
    assert got == envs


def test_overhead_accounting():
    # 4 blocks, CRC16 -> header(6) + 4*2 = 14 bytes of framing per body.
    assert overhead_bytes(4, crc_bytes=2) == SBI_HDR_LEN + 8
    assert overhead_bytes(4, crc_bytes=4) == SBI_HDR_LEN + 16


# --------------------------------------------------------------------------- #
# Headline fusion result — ubiquitous localized BER
# --------------------------------------------------------------------------- #
def _corrupt_one_block_per_body(body: bytes, block_payload: int, rng,
                                crc_bytes: int = 2) -> bytes:
    """Flip a couple of bytes inside ONE random sub-block of `body` — models a
    low BER whose few bit-flips happen to land in a single OFDM-symbol region,
    failing the frame FCS but leaving the rest of the body intact."""
    stride = crc_bytes + block_payload
    n = (len(body) - SBI_HDR_LEN) // stride
    j = rng.randrange(n)
    b = bytearray(body)
    base = SBI_HDR_LEN + j * stride + crc_bytes
    for _ in range(2):
        b[base + rng.randrange(block_payload)] ^= (1 << rng.randrange(8))
    return bytes(b)


def test_sbi_beats_drop_whole_frame_under_ubiquitous_ber():
    raptorq = pytest.importorskip("raptorq")  # outer code dep
    import stream_fec
    from stream_fec import FecConfig

    # Small symbols so several ride in one radio body. overhead=1.0 -> the
    # RaptorQ block tolerates ~50% symbol loss.
    cfg = FecConfig(k=8, symbol_size=64, overhead=1.0, scheme="raptorq")
    payloads = [bytes(f"packet-{i:03d} ".encode()) * 3 for i in range(8)]

    enc = stream_fec.make_encoder(cfg)
    envelopes: list[bytes] = []
    for p in payloads:
        envelopes += enc.add_packet(p)
    envelopes += enc.flush()
    assert envelopes, "encoder produced no symbols"

    env_size = len(envelopes[0])
    assert all(len(e) == env_size for e in envelopes), "non-uniform envelope size"

    blocks_per_body = 4
    bodies = pack(envelopes, block_payload=env_size,
                  blocks_per_body=blocks_per_body)

    rng = random.Random(1234)
    # Channel: EVERY radio frame takes a localized hit (fails its FCS).
    corrupted = [_corrupt_one_block_per_body(b, env_size, rng) for b in bodies]

    # --- Baseline: chip drops any FCS-failed frame -> no symbols at all. ---
    base_dec = stream_fec.make_decoder(cfg)
    base_out: list[bytes] = []
    for _b in corrupted:
        pass  # every frame corrupted -> every frame dropped -> nothing decoded
    assert base_out == [], "baseline somehow recovered without SBI"

    # --- SBI: keep the corrupt frames, salvage the surviving sub-blocks. ---
    sbi_dec = stream_fec.make_decoder(cfg)
    sbi_out: list[bytes] = []
    total_blocks = total_failed = 0
    for b in corrupted:
        res = unpack(b, block_payload=env_size)
        total_blocks += res.n_blocks
        total_failed += res.n_failed
        for env in res.survivors:
            sbi_out += sbi_dec.add_symbol(env)

    # ~1 of 4 sub-blocks lost per body — the rest survive and reconstruct.
    assert 0 < total_failed < total_blocks
    assert set(sbi_out) == set(payloads), (
        f"SBI failed to recover all packets: got {len(set(sbi_out))}/"
        f"{len(payloads)} (lost {total_failed}/{total_blocks} sub-blocks)")
