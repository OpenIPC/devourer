"""Tests for the frequency-diversity hop sim (hop_diversity_sim.py)."""
import random

from stream_fec import FecConfig
import hop_diversity_sim as hd


def _cfg(k=8, overhead=0.5, symbol_size=64):
    return FecConfig(k=k, symbol_size=symbol_size, overhead=overhead, scheme="rs")


def test_hop_recovers_single_channel_loss_when_repair_sufficient():
    # K=8, overhead 0.5 -> R=4, N=12, ceil(12/3)=4, so R==need -> full recovery.
    cfg = _cfg()
    res = hd.montecarlo(cfg, n_ch=3, trials=60, per=0.0, n_packets=24, seed=7)
    assert res["hop"] == 1.0


def test_nohop_cannot_survive_single_channel_loss():
    # Whole blocks ride one channel; with multiple blocks a single dead channel
    # always takes some, so end-to-end recovery never completes.
    cfg = _cfg()
    res = hd.montecarlo(cfg, n_ch=3, trials=60, per=0.0, n_packets=24, seed=7)
    assert res["nohop"] == 0.0


def test_hop_fails_when_repair_below_threshold():
    # overhead 0.25 -> R=2 < ceil(N/3)=4: a dead channel erases more than the
    # parity can cover, so hop cannot recover either.
    cfg = _cfg(overhead=0.25)
    assert cfg.repair_count < -(-(cfg.k + cfg.repair_count) // 3)
    res = hd.montecarlo(cfg, n_ch=3, trials=60, per=0.0, n_packets=24, seed=7)
    assert res["hop"] == 0.0


def test_hop_threshold_matches_ceil_formula():
    # The repair needed for 100% single-channel-loss recovery is ceil(N/N_ch).
    for n_ch in (2, 3, 4):
        cfg = _cfg(k=8, overhead=1.0)          # R=8, N=16: generous
        need = -(-(cfg.k + cfg.repair_count) // n_ch)
        assert cfg.repair_count >= need
        res = hd.montecarlo(cfg, n_ch=n_ch, trials=40, per=0.0,
                            n_packets=16, seed=3)
        assert res["hop"] == 1.0


def test_single_trial_recovers_exact_packets():
    cfg = _cfg()
    rng = random.Random(1)
    packets = hd._gen_packets(rng, 24, cfg.max_packet_size)
    ok = hd.run_trial(cfg, packets, n_ch=3, mode="hop",
                      dead_channels={1}, per=0.0, rng=rng)
    assert ok
