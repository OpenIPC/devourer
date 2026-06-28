"""Tests for the adaptive-link energy model (`energy_model.py`).

Pins the energy thesis the controller relies on: to carry a fixed video bitrate,
higher MCS / less FEC = less airtime = less energy/bit; losing blocks raises it;
TX power (TXAGC) is a WEAK energy lever vs MCS; and an MCS too slow to carry the
bitrate is infeasible.
"""

from __future__ import annotations

import energy_model as em

SRC = 4e6   # 4 Mbps video
L = 1024


def _eb(mcs, txagc, overhead=0.25, pdel=1.0, src=SRC):
    return em.energy_per_delivered_bit(em.TxPoint(mcs=mcs, txagc=txagc), src,
                                       overhead, L, pdel, em.load_calibration())


def test_phy_rates():
    assert em.phy_rate_mbps("ht", 0) == 6.5
    assert em.phy_rate_mbps("ht", 7) == 65.0
    assert em.phy_rate_mbps("ht", 7, bw=40) == 135.0
    assert abs(em.phy_rate_mbps("ht", 7, sgi=True) - 65.0 * 10 / 9) < 1e-6
    assert em.phy_rate_mbps("legacy", 54) == 54.0


def test_higher_mcs_lowers_energy_per_bit_clean():
    """Core thesis: carrying a fixed bitrate, higher MCS = less airtime = less
    energy/bit (monotone)."""
    eb = [_eb(m, 32) for m in range(8)]
    assert eb == sorted(eb, reverse=True)
    assert eb[0] > eb[7]


def test_losing_blocks_is_expensive():
    full = _eb(5, 32, pdel=1.0)
    half = _eb(5, 32, pdel=0.5)
    assert abs(half - 2 * full) < 1e-12                    # E_bit ~ 1/p_deliver
    assert _eb(5, 32, pdel=0.0) == float("inf")


def test_more_overhead_costs_energy():
    assert _eb(5, 32, overhead=1.00) > _eb(5, 32, overhead=0.25)


def test_txpower_is_a_weak_energy_lever():
    """Raising TXAGC across its whole range moves energy/bit far less than the
    swing across the MCS range (guards 'power is cheap for link margin')."""
    pwr_cost = _eb(5, 63) - _eb(5, 0)        # full TXAGC range, fixed MCS
    mcs_swing = _eb(0, 8) - _eb(7, 8)        # full MCS range, fixed low TXAGC
    assert pwr_cost > 0
    assert pwr_cost < mcs_swing


def test_too_slow_mcs_is_infeasible():
    """An MCS whose effective rate can't carry src*(1+ov) is +inf (can't fit).
    Effective rate is well below the PHY rate at small payloads (preamble tax)."""
    assert _eb(0, 32, overhead=0.0, src=30e6) == float("inf")   # 30 Mbps >> MCS0
    assert _eb(7, 32, overhead=0.0, src=30e6) < float("inf")    # MCS7 carries it


def test_airtime_drops_with_mcs():
    cal = em.load_calibration()
    af = [em.airtime_fraction(em.TxPoint(mcs=m), SRC, 0.25, L, cal) for m in range(8)]
    assert af == sorted(af, reverse=True)


def test_calibration_overlay(tmp_path):
    import json
    p = tmp_path / "cal.json"
    p.write_text(json.dumps({"source": "metered", "metered_watts": True,
                             "p_baseline_w": 0.9}))
    cal = em.load_calibration(str(p))
    assert cal.source == "metered" and cal.metered_watts and cal.p_baseline_w == 0.9
    assert len(cal.p_pa_w) == 64
