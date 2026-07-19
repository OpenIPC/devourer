"""Tests for the MCS-probe A/B sim (`mcs_probe_ab_sim.py`).

Pins the headline the feature exists for: a model that is optimistically wrong
about one rate traps the model-only controller (it promotes in and its own
belief keeps it there), while probe evidence keeps the probing controller out —
SLA held, energy per delivered bit orders of magnitude apart. The clean-channel
control quantifies the honest cost: bounded probe airtime and a bounded energy
premium from the evidence-conservative operating point.
"""

from __future__ import annotations

import mcs_probe_ab_sim as ab


def _static(bias):
    return ab.run_scenario("static", n_frames=9000, bias=bias, seed=1,
                           trials=250)


def test_biased_model_traps_model_arm_probes_hold_sla():
    arms = _static(bias={5: -8.0})
    model, probe = arms["model"], arms["probe"]
    # the model arm enters the lie and stays (its cur_ok shares the wrong
    # belief); the probe arm may touch it transiently (cold picks are
    # model-trusted, and a video-loss reset re-picks cold), but the measured
    # active-row delivery condemns and blocks it within seconds
    assert model.biased_row_ms is not None
    assert model.biased_frames > 4000
    assert probe.biased_frames < 500
    assert probe.delivery >= 0.95
    assert model.delivery < 0.5
    # delivered-bit energy: the collapse makes the model arm ruinous
    assert probe.e_bit_nj < model.e_bit_nj / 10
    # probing did not add churn
    assert probe.commits <= model.commits + 3


def test_ambush_cold_capture_self_heals_with_probes():
    """No acquisition segment: the ungated cold pick lands BOTH arms on the
    biased row. The probe arm's measured active-row delivery condemns it
    within seconds (trip -> block -> fast-down; the block outlives the escape
    so it is not re-picked), while the model-only arm — whose belief is the
    thing that is wrong — stays for the whole run."""
    arms = ab.run_scenario("ambush", n_frames=9000, bias={5: -8.0}, seed=1,
                           trials=250)
    model, probe = arms["model"], arms["probe"]
    assert model.biased_row_ms is not None
    assert probe.biased_row_ms is not None        # cold-captured too...
    assert probe.biased_frames < 1500             # ...but escapes in seconds
    assert model.biased_frames > 4000             # stuck for most of the run
    assert probe.first_block_ms is not None
    assert probe.delivery >= 0.9
    assert model.delivery < 0.5
    assert probe.e_bit_nj < model.e_bit_nj / 5


def test_clean_channel_probe_cost_is_bounded():
    arms = _static(bias=None)
    model, probe = arms["model"], arms["probe"]
    # a well-calibrated model needs no probes: both arms hold the SLA...
    assert model.delivery >= 0.97 and probe.delivery >= 0.97
    # ...and the probe arm's cost is bounded: ~3% scheduled duty on airtime
    # and a bounded energy premium from its evidence-conservative operating
    # point (promotions wait for proof at the incumbent's power)
    assert probe.probe_airtime_frac < 0.06
    assert probe.e_bit_nj < model.e_bit_nj * 1.6
    # probe bookkeeping is alive (attempted and priced)
    assert probe.probe_frames > 50
    assert model.probe_frames == 0
