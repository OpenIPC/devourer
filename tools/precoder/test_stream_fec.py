"""Tests for the RaptorQ FEC layer (`stream_fec.py`).

Cover:
  * Round-trip with no symbol loss (full reconstruction).
  * Loss tolerance at 30/50/70% random symbol drop (R/K = 1 default → 50% is
    the design point; 30% must succeed every time, 70% is expected to fail).
  * Concatenation packing — small + medium packets share symbols correctly.
  * Partial-block flush — KREAL < K, decoder emits exactly KREAL worth of
    packets, no trailing garbage from padded zero symbols.
  * Block ID wrap — encoder rolls past 65535 without colliding live blocks.
  * MTU enforcement — oversized packet → ValueError.
  * Unrecoverable-block bookkeeping — expire_blocks_older_than counts them.
"""

from __future__ import annotations

import random
import time

import pytest

import stream_fec
from stream_fec import FecConfig, FecDecoder, FecEncoder


def _round_trip(packets: list[bytes], cfg: FecConfig,
                 drop_pct: float = 0.0,
                 seed: int = 0,
                 flush: bool = True) -> list[bytes]:
    """TX-encode `packets` into envelopes, randomly drop `drop_pct`, RX-decode.
    Returns the recovered packet stream in arrival order. Per-block."""
    enc = FecEncoder(cfg)
    dec = FecDecoder(cfg)
    envelopes: list[bytes] = []
    for p in packets:
        envelopes += enc.add_packet(p)
    if flush:
        envelopes += enc.flush()
    rng = random.Random(seed)
    rng.shuffle(envelopes)
    keep = envelopes[int(len(envelopes) * drop_pct):]
    out: list[bytes] = []
    for env in keep:
        out += dec.add_symbol(env)
    return out


# --------------------------------------------------------------------------- #
# Config sanity
# --------------------------------------------------------------------------- #
def test_config_rejects_bad_args():
    with pytest.raises(ValueError):
        FecConfig(k=0)
    with pytest.raises(ValueError):
        FecConfig(k=256)
    with pytest.raises(ValueError):
        FecConfig(symbol_size=1)
    with pytest.raises(ValueError):
        FecConfig(overhead=-0.1)


def test_repair_count_is_ceil_k_times_overhead():
    assert FecConfig(k=16, overhead=1.0).repair_count == 16
    assert FecConfig(k=16, overhead=0.5).repair_count == 8
    assert FecConfig(k=16, overhead=0.51).repair_count == 9
    assert FecConfig(k=16, overhead=2.0).repair_count == 32


# --------------------------------------------------------------------------- #
# Round-trip
# --------------------------------------------------------------------------- #
def test_no_loss_round_trip_recovers_every_packet():
    cfg = FecConfig(k=8, symbol_size=200, overhead=0.5)
    rng = random.Random(0xC0FFEE)
    pkts = [bytes(rng.randint(0, 255) for _ in range(rng.randint(20, 180)))
            for _ in range(20)]
    out = _round_trip(pkts, cfg, drop_pct=0.0, seed=1)
    assert out == pkts


@pytest.mark.parametrize("drop_pct", [0.0, 0.2, 0.4])
def test_recovers_within_design_loss(drop_pct):
    """At R/K = 1 (overhead 1.0) cberner's raptorq needs K_effective+ε
    symbols per block where K_effective ≈ K + 1 (small overhead the lib
    adds for OTI alignment). With K=8 + R=8 = 16 envelopes per block we
    have ~7 symbols of headroom, so 40% loss is reliably recoverable;
    above that we're in the regime where the bridge's FEC needs higher
    overhead. Asymptotic K → ∞ gives the textbook ~50% loss tolerance
    at R/K = 1 — see the 70%-loss test for the unrecoverable-block side.

    The decoder emits packets per-block when each block crosses the decode
    threshold, so cross-block envelope order can reorder packets in groups
    of K — within a block, order is preserved. We assert set-equality on
    the bytes."""
    cfg = FecConfig(k=8, symbol_size=200, overhead=1.0)
    rng = random.Random(0xBEEF + int(drop_pct * 100))
    pkts = [
        i.to_bytes(2, "little") + bytes(
            rng.randint(0, 255) for _ in range(rng.randint(50, 178)))
        for i in range(16)
    ]
    out = _round_trip(pkts, cfg, drop_pct=drop_pct, seed=int(drop_pct * 10))
    assert sorted(out) == sorted(pkts), (
        f"drop={drop_pct}: got {len(out)}/{len(pkts)} packets, "
        f"set-equal={set(out) == set(pkts)}"
    )


def test_50pct_loss_recoverable_at_higher_overhead():
    """The default overhead R/K=1 is the recommended middle of the survey's
    loss range; raising to R/K=2 buys reliable decode at 50% loss for the
    small-K regime where cberner's lib needs K+1 packets."""
    cfg = FecConfig(k=8, symbol_size=200, overhead=2.0)
    rng = random.Random(0xFADE)
    pkts = [bytes(rng.randint(0, 255) for _ in range(rng.randint(50, 178)))
            for _ in range(16)]
    out = _round_trip(pkts, cfg, drop_pct=0.5, seed=42)
    assert sorted(out) == sorted(pkts)


def test_70_percent_loss_is_unrecoverable_with_default_overhead():
    """Documents the expectation: at 70% drop and R/K = 1, decode fails.
    The bridge's job is to count unrecoverable blocks via the decoder's
    expire_blocks_older_than API, not to silently lose packets."""
    cfg = FecConfig(k=8, symbol_size=200, overhead=1.0)
    rng = random.Random(0xDEAD)
    pkts = [bytes(rng.randint(0, 255) for _ in range(rng.randint(20, 180)))
            for _ in range(16)]
    enc = FecEncoder(cfg)
    dec = FecDecoder(cfg)
    envelopes = []
    for p in pkts:
        envelopes += enc.add_packet(p)
    envelopes += enc.flush()
    rng.shuffle(envelopes)
    keep = envelopes[int(len(envelopes) * 0.7):]
    out = []
    for env in keep:
        out += dec.add_symbol(env)
    # At 70% drop expect 0 or 1 blocks decoded (random; structural — RaptorQ
    # has small probability of decoding with significantly fewer than K
    # symbols, but the typical case fails).
    assert len(out) < len(pkts)
    # Aging-out the still-in-flight blocks moves them into the unrecoverable
    # counter — that's the bridge's signal.
    dec.expire_blocks_older_than(-1.0)
    assert dec.blocks_unrecoverable >= 1


# --------------------------------------------------------------------------- #
# Concatenation packing
# --------------------------------------------------------------------------- #
def test_many_small_packets_share_one_symbol():
    """32 packets of 30 B with symbol_size=200 must pack ~6 packets per
    symbol (30 + 2-byte prefix = 32 B per slot, 6 fit in 200 with 8 B left
    for zero pad). K=8 → 32 packets should fit in much less than 32 symbols."""
    cfg = FecConfig(k=8, symbol_size=200, overhead=1.0)
    rng = random.Random(0xCAFE)
    pkts = [bytes(rng.randint(0, 255) for _ in range(30)) for _ in range(32)]
    enc = FecEncoder(cfg)
    envelopes = []
    for p in pkts:
        envelopes += enc.add_packet(p)
    envelopes += enc.flush()
    # Decoder gives back exactly the input order.
    dec = FecDecoder(cfg)
    out = []
    for env in envelopes:
        out += dec.add_symbol(env)
    assert out == pkts
    # Symbol-efficiency: at most ceil(32/6) = 6 source symbols needed for 32
    # packets. Two blocks (K=8) means 16 source slots, plenty of headroom;
    # the encoder should have produced exactly 1 block.
    assert enc.blocks_encoded == 1


def test_mixed_sizes_round_trip_preserves_order():
    cfg = FecConfig(k=4, symbol_size=200, overhead=1.0)
    pkts = [
        bytes([0x11] * 30), bytes([0x22] * 100), bytes([0x33] * 10),
        bytes([0x44] * 180), bytes([0x55] * 5),  bytes([0x66] * 50),
    ]
    out = _round_trip(pkts, cfg, drop_pct=0.0)
    assert out == pkts


def test_max_size_packet_fits_in_its_own_symbol():
    cfg = FecConfig(k=2, symbol_size=200, overhead=1.0)
    pkts = [bytes([0xA1]) * cfg.max_packet_size,
            bytes([0xB2]) * cfg.max_packet_size]
    out = _round_trip(pkts, cfg, drop_pct=0.0)
    assert out == pkts


def test_oversized_packet_raises():
    cfg = FecConfig(k=4, symbol_size=200)
    enc = FecEncoder(cfg)
    with pytest.raises(ValueError):
        enc.add_packet(bytes(cfg.max_packet_size + 1))


# --------------------------------------------------------------------------- #
# Partial / flush
# --------------------------------------------------------------------------- #
def test_partial_block_flush_emits_exact_kreal_packets():
    cfg = FecConfig(k=8, symbol_size=200, overhead=1.0)
    pkts = [bytes([i + 1]) * 50 for i in range(3)]  # 3 packets, well under K
    out = _round_trip(pkts, cfg, drop_pct=0.0)
    assert out == pkts


def test_unflushed_partial_block_emits_nothing():
    cfg = FecConfig(k=8, symbol_size=200, overhead=1.0)
    enc = FecEncoder(cfg)
    dec = FecDecoder(cfg)
    pkts = [bytes([0x55]) * 50 for _ in range(3)]
    envelopes = []
    for p in pkts:
        envelopes += enc.add_packet(p)
    # No flush() — envelopes should be empty, decoder should produce nothing.
    assert envelopes == []
    out = []
    for env in envelopes:
        out += dec.add_symbol(env)
    assert out == []


# --------------------------------------------------------------------------- #
# Block ID wrap + multi-block sequencing
# --------------------------------------------------------------------------- #
def test_block_id_wraps_without_aliasing_in_flight():
    """Drive the encoder through 65538 blocks with K=2 to wrap u16. The
    decoder should see each block id at most once at a time."""
    cfg = FecConfig(k=2, symbol_size=200, overhead=0.5)
    enc = FecEncoder(cfg)
    dec = FecDecoder(cfg)
    pkts_in: list[bytes] = []
    pkts_out: list[bytes] = []
    for i in range(65540):
        pkt = bytes([(i & 0xFF), ((i >> 8) & 0xFF)] * 50)  # 100 B
        pkts_in.append(pkt)
        envelopes = enc.add_packet(pkt)
        for env in envelopes:
            pkts_out += dec.add_symbol(env)
    pkts_out += [p for env in enc.flush() for p in dec.add_symbol(env)]
    # Some blocks may decode out of order or after later ones; what we care
    # about is no aliasing damage — every input packet emerges intact.
    assert pkts_out == pkts_in


def test_multiple_blocks_interleaved_at_decoder():
    """Out-of-order envelope arrival across blocks must still decode each
    block correctly."""
    cfg = FecConfig(k=4, symbol_size=200, overhead=1.0)
    rng = random.Random(7)
    pkts = [bytes([i + 1]) * 50 for i in range(12)]  # 3 blocks
    enc = FecEncoder(cfg)
    dec = FecDecoder(cfg)
    envelopes = []
    for p in pkts:
        envelopes += enc.add_packet(p)
    envelopes += enc.flush()
    rng.shuffle(envelopes)
    out = []
    for env in envelopes:
        out += dec.add_symbol(env)
    # Per-block order is preserved by the codec; across blocks the decode
    # order depends on which block reached the K-symbol threshold first.
    # Sort by first-byte tag, which uniquely identifies each input packet.
    out_by_tag = sorted(out, key=lambda b: b[0])
    pkts_by_tag = sorted(pkts, key=lambda b: b[0])
    assert out_by_tag == pkts_by_tag


# --------------------------------------------------------------------------- #
# Expiry / bookkeeping
# --------------------------------------------------------------------------- #
def test_expire_blocks_older_than_drops_stale():
    cfg = FecConfig(k=8, symbol_size=200, overhead=0.5)
    enc = FecEncoder(cfg)
    dec = FecDecoder(cfg)
    # Need packets that each occupy their own symbol so we trigger encoding.
    # With symbol_size=200 and max_packet_size=198, use ~190-byte packets.
    pkts = [bytes([i + 1]) * 190 for i in range(cfg.k)]
    envelopes = []
    for p in pkts:
        envelopes += enc.add_packet(p)
    envelopes += enc.flush()  # ensure the block is encoded
    assert envelopes, "encoder produced no envelopes — block not encoded"
    # Feed only one envelope (decoder won't have enough to decode).
    dec.add_symbol(envelopes[0])
    assert dec.in_flight_blocks == 1
    n = dec.expire_blocks_older_than(-1.0)  # negative age = expire everything
    assert n == 1
    assert dec.blocks_unrecoverable == 1
    assert dec.in_flight_blocks == 0


def test_decoder_drops_symbol_with_wrong_config():
    cfg = FecConfig(k=8, symbol_size=200)
    dec = FecDecoder(cfg)
    # Hand-craft an envelope with a different k.
    # _pack_header now lives in stream_fec_raptorq (scheme-specific header).
    import stream_fec_raptorq
    bad_header = stream_fec_raptorq._pack_header(
        16, kreal=16, symbol_size=200, block_id=0)
    out = dec.add_symbol(bad_header + b"\x00" * 200)
    assert out == []
    assert dec.symbols_dropped_bad_cfg == 1


def test_decoder_ignores_garbage_envelope():
    cfg = FecConfig(k=8, symbol_size=200)
    dec = FecDecoder(cfg)
    assert dec.add_symbol(b"\x00\x00not a fec frame") == []
    assert dec.add_symbol(b"") == []
    assert dec.symbols_in == 0
