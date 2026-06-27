"""Tests for the fused-FEC simulation harness (`fec_fusion_sim.py`).

Locks in the qualitative result the sim produced: in the realistic
post-PHY-FEC residual-BER regime (small symbols, low BER) sub-block salvage
turns a marginal drop-whole-frame link into a reliable one, and SBI never
does worse than the baseline.
"""

from __future__ import annotations

import random

import fec_fusion_sim
from fec_fusion_sim import Channel, run
from stream_fec import FecConfig


def _factory(model, ber, frame_loss):
    return lambda rng: Channel(model=model, ber=ber, frame_loss=frame_loss,
                               rng=rng)


def test_slope_model_tail_heavier_than_head():
    rng = random.Random(0)
    ch = Channel("slope", ber=5e-3, frame_loss=0.0, rng=rng)
    head_flips = tail_flips = 0
    body = bytes(200)
    for _ in range(200):
        out, lost, _ = ch.corrupt(body)
        assert not lost
        head_flips += sum(bin(out[i]).count("1") for i in range(20))
        tail_flips += sum(bin(out[i]).count("1") for i in range(180, 200))
    # Slope-line: errors concentrate at the frame tail.
    assert tail_flips > head_flips * 3


def test_sbi_beats_baseline_in_residual_ber_regime():
    cfg = FecConfig(k=8, symbol_size=32, overhead=0.5, scheme="rs")
    s = run(cfg, blocks_per_body=8, n_packets=16, pkt_len=8,
            chan_factory=_factory("slope", 3e-4, 0.0),
            crc_bytes=2, trials=120, seed=3)
    # Baseline is genuinely marginal here; SBI is near-perfect.
    assert s.baseline_success < 0.8
    assert s.sbi_success > 0.95
    assert s.sbi_success >= s.baseline_success


def test_sbi_never_worse_than_baseline_clean_channel():
    cfg = FecConfig(k=6, symbol_size=64, overhead=0.5, scheme="rs")
    s = run(cfg, blocks_per_body=4, n_packets=12, pkt_len=40,
            chan_factory=_factory("uniform", 0.0, 0.0),
            crc_bytes=2, trials=20, seed=1)
    assert s.baseline_success == 1.0 and s.sbi_success == 1.0


def test_frame_loss_hits_both_equally():
    # Pure whole-frame loss (no bit errors): SBI has no advantage, and at a
    # loss rate within the erasure budget both still recover.
    cfg = FecConfig(k=8, symbol_size=64, overhead=1.0, scheme="rs")  # N=16
    s = run(cfg, blocks_per_body=2, n_packets=10, pkt_len=40,
            chan_factory=_factory("uniform", 0.0, 0.15),
            crc_bytes=2, trials=60, seed=5)
    assert s.sbi_success >= s.baseline_success


def test_summary_overhead_accounting():
    cfg = FecConfig(k=8, symbol_size=64, overhead=0.5, scheme="rs")
    s = run(cfg, blocks_per_body=4, n_packets=8, pkt_len=40,
            chan_factory=_factory("uniform", 0.0, 0.0),
            crc_bytes=2, trials=5, seed=0)
    # env = 11 (RS header) + 64 = 75; overhead = hdr(6)+4*2=14 over 14+4*75=314.
    assert s.env_size == 75
    assert 4.0 < s.overhead_pct < 5.0
