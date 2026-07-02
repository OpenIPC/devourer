"""Tests for the space-frequency diversity sim (space_freq_diversity_sim.py)."""
from stream_fec import FecConfig
import space_freq_diversity_sim as sf


def _cfg(k=8, overhead=0.5, symbol_size=64):
    return FecConfig(k=k, symbol_size=symbol_size, overhead=overhead, scheme="rs")


def test_n_eff_limits():
    # Independent branches -> full order; fully correlated -> one branch.
    assert abs(sf.n_eff(4, 0.0) - 4.0) < 1e-9
    assert abs(sf.n_eff(4, 1.0) - 1.0) < 1e-9
    assert 1.0 < sf.n_eff(4, 0.5) < 4.0


def test_spatial_reduces_channel_outage_only_when_decorrelated():
    # p_channel_dead = p^N_eff: at rho=0 it is p^n_ant; at rho=1 it stays p.
    p = 0.4
    assert abs(sf.p_channel_dead(p, 4, 1.0) - p) < 1e-9
    assert sf.p_channel_dead(p, 4, 0.0) < p          # decorrelated helps
    assert sf.p_channel_dead(p, 4, 0.1) < sf.p_channel_dead(p, 4, 0.9)


def test_spacefreq_dominates_both_single_axes():
    # p_branch 0.25: low enough that hopping spreads a recoverable number of
    # channel deaths, so the axes separate cleanly (at ~0.5 freq≈none — hopping
    # only helps when the per-channel outage is recoverable).
    cfg = _cfg()
    r = sf.montecarlo(cfg, n_ch=3, n_ant=4, rho=0.1, p_branch=0.25, per=0.0,
                      n_packets=24, trials=300, seed=1)
    assert r["spacefreq"] == max(r.values())
    assert r["spacefreq"] >= r["freq"] - 0.02
    assert r["spacefreq"] >= r["space"] - 0.02
    assert r["freq"] > r["none"]


def test_frequency_axis_is_correlation_independent():
    # Hopping-only recovery must not depend on the spatial correlation rho.
    cfg = _cfg()
    lo = sf.montecarlo(cfg, 3, 4, 0.1, 0.4, 0.0, 24, 300, 5)["freq"]
    hi = sf.montecarlo(cfg, 3, 4, 0.9, 0.4, 0.0, 24, 300, 5)["freq"]
    assert lo == hi


def test_spatial_axis_helps_more_when_decorrelated():
    # The hardware finding: combining pays under motion (low rho), not static.
    cfg = _cfg()
    lo = sf.montecarlo(cfg, 1, 4, 0.1, 0.5, 0.0, 24, 300, 2)["space"]
    hi = sf.montecarlo(cfg, 1, 4, 0.9, 0.5, 0.0, 24, 300, 2)["space"]
    assert lo > hi
