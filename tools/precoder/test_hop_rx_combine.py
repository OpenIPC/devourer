"""Tests for the diversity-RX combiner (hop_rx_combine.py)."""
import random

from stream_fec import FecConfig
import hop_rx_combine as hc


def _cfg(k=8, overhead=0.5, symbol_size=64):
    return FecConfig(k=k, symbol_size=symbol_size, overhead=overhead, scheme="rs")


def test_all_receivers_up_recovers():
    cfg = _cfg()
    assert hc.montecarlo(cfg, n_ch=3, trials=40, per=0.0, n_packets=24,
                         seed=2, dead_count=0) == 1.0


def test_one_dead_receiver_recovers_with_enough_repair():
    # K=8, overhead 0.5 -> R=4 == ceil(N/3); one dead receiver is recoverable.
    cfg = _cfg()
    assert hc.montecarlo(cfg, n_ch=3, trials=40, per=0.0, n_packets=24,
                         seed=2, dead_count=1) == 1.0


def test_one_dead_receiver_fails_without_enough_repair():
    cfg = _cfg(overhead=0.25)            # R=2 < ceil(N/3)=4
    assert hc.montecarlo(cfg, n_ch=3, trials=40, per=0.0, n_packets=24,
                         seed=2, dead_count=1) == 0.0


def test_two_dead_receivers_need_more_repair():
    # With 4 receivers and 2 dead, ~half the symbols are gone -> need R ~ N/2.
    cfg = _cfg(overhead=1.0)             # R=8, N=16: survives 2-of-4 down
    assert hc.montecarlo(cfg, n_ch=4, trials=30, per=0.0, n_packets=16,
                         seed=5, dead_count=2) == 1.0


def test_combiner_dedups_duplicate_symbols_across_receivers():
    # The same symbol arriving on two receivers must not break decode.
    cfg = _cfg()
    rng = random.Random(1)
    packets = [bytes([i]) +
               bytes(rng.getrandbits(8) for _ in range(cfg.max_packet_size - 1))
               for i in range(8)]
    syms = hc._encode_symbols(cfg, packets)
    # Feed every symbol on TWO receivers (full duplication).
    recovered, used = hc.combine_streams([syms, list(syms)], cfg)
    assert set(recovered) == set(packets)
    assert sum(used) == 2 * len(syms)
