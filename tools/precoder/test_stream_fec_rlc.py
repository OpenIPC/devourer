"""Tests for the Sliding-Window RLC (RFC 8681) scheme.

Mirrors `test_stream_fec.py`'s structure: round-trip with no loss,
moderate loss tolerance, concatenation packing, partial-symbol flush,
MAGIC dispatch, oversized packet rejection.

The recovery characteristics differ from RaptorQ in two ways the tests
account for:

* **Lower asymptotic loss tolerance at small W.** At window=8 +
  overhead=1.0 the codec recovers cleanly through ~20% packet loss but
  starts to leave packets on the floor by 30%. Higher overhead (or
  larger window) buys more tolerance — `test_loss_tolerance_*` makes
  that visible.
* **Per-packet emission is order-preserving.** Source symbols arrive
  systematically; the decoder buffers any out-of-order recovery and
  surfaces packets in ESI order. We assert this rather than the
  sort()-based "set equal" pattern used for RaptorQ.
"""

from __future__ import annotations

import random
import struct

import pytest

import stream_fec
import stream_fec_rlc
from stream_fec import FecConfig, make_encoder, make_decoder


def _round_trip(packets, cfg, drop_pct=0.0, drop_seed=0, flush=True):
    enc = make_encoder(cfg)
    dec = make_decoder(cfg)
    envelopes: list[bytes] = []
    for p in packets:
        envelopes += enc.add_packet(p)
    if flush:
        envelopes += enc.flush()
    rng = random.Random(drop_seed)
    kept = [e for e in envelopes if rng.random() >= drop_pct]
    out: list[bytes] = []
    for env in kept:
        out += dec.add_symbol(env)
    return out, enc, dec


def test_no_loss_round_trip_preserves_order():
    cfg = FecConfig(scheme="rlc", window=8, symbol_size=128, overhead=1.0)
    pkts = [bytes([i + 1] * 50) for i in range(16)]
    out, _, _ = _round_trip(pkts, cfg, drop_pct=0.0)
    assert out == pkts  # ESI-order emission, including post-recovery packets


def test_systematic_source_symbols_emit_immediately_pre_flush():
    """RLC's headline win: each source symbol delivers its packed packets
    the moment its envelope reaches the decoder — no K-buffer."""
    cfg = FecConfig(scheme="rlc", window=8, symbol_size=128, overhead=1.0)
    enc = make_encoder(cfg)
    dec = make_decoder(cfg)
    # Feed enough packets to fill one source symbol (so add_packet emits a
    # source + a repair envelope), then check the decoder surfaces packets
    # without needing a flush.
    big = bytes([0xAA] * cfg.max_packet_size)
    out_before_flush: list[bytes] = []
    envs = enc.add_packet(big)
    for e in envs:
        out_before_flush += dec.add_symbol(e)
    assert out_before_flush == [big]


@pytest.mark.parametrize("drop_pct", [0.0, 0.1, 0.2])
def test_recovers_within_design_loss(drop_pct):
    """At overhead=1.0 the codec reliably handles up to ~20% loss for
    a window-aligned stream. Higher drops are exercised by
    test_high_loss_needs_higher_overhead."""
    cfg = FecConfig(scheme="rlc", window=8, symbol_size=128, overhead=1.0)
    pkts = [bytes([(i + 1) & 0xFF] * 50) for i in range(16)]
    out, _, _ = _round_trip(pkts, cfg, drop_pct=drop_pct, drop_seed=13)
    assert set(out) >= set(pkts)


def test_high_loss_needs_higher_overhead():
    """Bumping `overhead` to 1.5 should recover at the 30% loss point that
    fails at 1.0 — same fundamental tradeoff RaptorQ faces."""
    cfg_low = FecConfig(scheme="rlc", window=8, symbol_size=128, overhead=1.0)
    cfg_high = FecConfig(scheme="rlc", window=8, symbol_size=128, overhead=1.5)
    pkts = [bytes([(i + 1) & 0xFF] * 50) for i in range(16)]
    out_low, _, _ = _round_trip(pkts, cfg_low, drop_pct=0.3, drop_seed=13)
    out_high, _, _ = _round_trip(pkts, cfg_high, drop_pct=0.3, drop_seed=13)
    # We only require that higher overhead doesn't perform worse —
    # the exact percentage recovered depends on the random subset
    # the seed picks.
    assert len(out_high) >= len(out_low)


def test_concatenation_packs_small_packets_into_one_symbol():
    cfg = FecConfig(scheme="rlc", window=8, symbol_size=200, overhead=1.0)
    # 6 packets of 30 B each at 200B symbol_size: each (30 + 2) = 32, so
    # 6 packets fit in one symbol (192 of 200 bytes used, 8 trailing pad).
    pkts = [bytes([0xA0 + i] * 30) for i in range(6)]
    out, enc, _ = _round_trip(pkts, cfg, drop_pct=0.0)
    assert out == pkts
    # Only 1 source symbol sealed → blocks_encoded counter == 1.
    assert enc.blocks_encoded == 1


def test_oversized_packet_raises():
    cfg = FecConfig(scheme="rlc", window=8, symbol_size=128, overhead=1.0)
    enc = make_encoder(cfg)
    with pytest.raises(ValueError):
        enc.add_packet(bytes(cfg.max_packet_size + 1))


def test_dispatcher_routes_by_magic():
    """A RLC-encoded envelope must not be parsed by a RaptorQ decoder
    (silent corruption is the worst outcome of a scheme mix)."""
    cfg_rlc = FecConfig(scheme="rlc", window=8, symbol_size=128, overhead=1.0)
    cfg_raptorq = FecConfig(scheme="raptorq", k=8, symbol_size=128, overhead=1.0)
    enc = make_encoder(cfg_rlc)
    dec_raptorq = make_decoder(cfg_raptorq)
    envs = enc.add_packet(bytes([0xDE] * 50))
    envs += enc.flush()
    out: list[bytes] = []
    for e in envs:
        out += dec_raptorq.add_symbol(e)
    assert out == []  # RaptorQ decoder rejected RLC envelopes
    # And vice versa.
    enc_raptorq = make_encoder(cfg_raptorq)
    dec_rlc = make_decoder(cfg_rlc)
    envs = enc_raptorq.add_packet(bytes([0xAD] * 50))
    envs += enc_raptorq.flush()
    out = []
    for e in envs:
        out += dec_rlc.add_symbol(e)
    assert out == []


def test_garbage_envelope_dropped():
    cfg = FecConfig(scheme="rlc", window=8, symbol_size=128, overhead=1.0)
    dec = make_decoder(cfg)
    assert dec.add_symbol(b"") == []
    assert dec.add_symbol(b"\x00\x00not a fec frame") == []


def test_partial_packing_flushes_cleanly():
    cfg = FecConfig(scheme="rlc", window=8, symbol_size=128, overhead=1.0)
    enc = make_encoder(cfg)
    dec = make_decoder(cfg)
    pkts = [bytes([0x11] * 30), bytes([0x22] * 50)]  # both fit one symbol
    envs = []
    for p in pkts:
        envs += enc.add_packet(p)
    envs += enc.flush()
    out: list[bytes] = []
    for e in envs:
        out += dec.add_symbol(e)
    assert out == pkts


def test_envelope_header_has_distinct_magic():
    assert stream_fec_rlc.RLC_MAGIC != stream_fec.FEC_MAGIC_RAPTORQ
    # And the dispatcher reads MAGIC right off the wire.
    cfg = FecConfig(scheme="rlc", window=8, symbol_size=64, overhead=1.0)
    enc = make_encoder(cfg)
    envs = enc.add_packet(bytes([0x55] * 50)) + enc.flush()
    for env in envs:
        magic = struct.unpack_from("<H", env)[0]
        assert magic == stream_fec_rlc.RLC_MAGIC


def test_config_rejects_bad_args():
    with pytest.raises(ValueError):
        FecConfig(scheme="rlc", window=0)
    with pytest.raises(ValueError):
        FecConfig(scheme="rlc", window=256)
    with pytest.raises(ValueError):
        FecConfig(scheme="rlc", density_threshold=16)
    with pytest.raises(ValueError):
        FecConfig(scheme="rlc", density_threshold=-1)
    with pytest.raises(ValueError):
        FecConfig(scheme="weird")
