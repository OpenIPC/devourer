"""Tests for the active-chain-count energy sim (chain_count_energy_sim.py)."""
from energy_model import TxPoint, load_calibration
import chain_count_energy_sim as cc


def _setup():
    return load_calibration(), TxPoint(mcs=3, txagc=32), 4e6


def test_n_eff_limits():
    assert abs(cc.n_eff(4, 0.0) - 4.0) < 1e-9
    assert abs(cc.n_eff(4, 1.0) - 1.0) < 1e-9
    assert 1.0 < cc.n_eff(4, 0.5) < 4.0


def test_delivery_improves_only_when_decorrelated():
    # correlated antennas barely lift delivery; decorrelated lift it a lot
    assert abs(cc.deliver_multichain(0.6, 4, 1.0) - 0.6) < 1e-9   # rho=1: no help
    assert cc.deliver_multichain(0.6, 4, 0.1) > 0.95
    assert cc.deliver_multichain(0.6, 4, 0.1) > cc.deliver_multichain(0.6, 4, 0.9)


def test_static_prefers_single_chain():
    cal, p, src = _setup()
    best, eb = cc.optimal_chains(p, src, 0.25, 1024, 0.6, 0.9, cal)
    assert best == 1
    # and energy strictly increases with each added chain when diversity is nil
    assert eb[0] < eb[1] < eb[2] < eb[3]


def test_mobile_prefers_more_chains():
    cal, p, src = _setup()
    best_static, _ = cc.optimal_chains(p, src, 0.25, 1024, 0.6, 0.9, cal)
    best_mobile, _ = cc.optimal_chains(p, src, 0.25, 1024, 0.6, 0.1, cal)
    assert best_mobile > best_static


def test_more_marginal_link_opens_more_chains():
    # at a deeper marginal link, more chains earn their baseline
    cal, p, src = _setup()
    best_mid, _ = cc.optimal_chains(p, src, 0.25, 1024, 0.6, 0.1, cal)
    best_deep, _ = cc.optimal_chains(p, src, 0.25, 1024, 0.35, 0.1, cal)
    assert best_deep >= best_mid
    assert best_deep == 4    # a deep marginal + decorrelated link wants all four


def test_cheaper_chains_shift_optimum_up():
    cal, p, src = _setup()
    n_expensive, _ = cc.optimal_chains(p, src, 0.25, 1024, 0.6, 0.1, cal, p_chain_w=0.15)
    n_cheap, _ = cc.optimal_chains(p, src, 0.25, 1024, 0.6, 0.1, cal, p_chain_w=0.05)
    assert n_cheap >= n_expensive
