"""Tests for the beam-steer feasibility sim (beamsteer_feasibility_sim.py)."""
import math
import beamsteer_feasibility_sim as bs


def test_onaxis_peak_is_array_gain():
    for n in (2, 3, 4, 8):
        assert abs(bs.array_factor_db(n, 0.0) - 10 * math.log10(n)) < 1e-6


def test_gain_rolls_off_within_main_lobe():
    assert bs.array_factor_db(4, 0) > bs.array_factor_db(4, 8) > bs.array_factor_db(4, 14)


def test_pointing_budget_grows_with_rate_and_latency():
    assert bs.pointing_error_deg(180, 0.3, 5) > bs.pointing_error_deg(30, 0.3, 5)
    assert bs.pointing_error_deg(30, 0.3, 5) > bs.pointing_error_deg(30, 0.05, 5)
    assert bs.pointing_error_deg(0, 0, 7) == 7        # pure CSI resolution floor


def test_fast_moving_source_loses_to_diversity():
    # a fast platform + slow coarse direction blows the pointing budget
    g = bs.realised_beam_gain_db(4, rate_deg_s=90, update_latency_s=0.2,
                                 csi_res_deg=10)
    assert g < bs.STBC_GAIN_DB
    assert bs.verdict(g).startswith("USE DIVERSITY")


def test_slow_known_geometry_can_beamform():
    # a near-stationary / precisely-known geometry keeps the pointing tight
    g = bs.realised_beam_gain_db(4, rate_deg_s=5, update_latency_s=0.05,
                                 csi_res_deg=3)
    assert g > bs.STBC_GAIN_DB
    assert "BEAMFORM" in bs.verdict(g)
