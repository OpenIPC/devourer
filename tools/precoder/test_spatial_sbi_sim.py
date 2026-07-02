"""Tests for the spatial-diversity SBI-precondition sim (spatial_sbi_sim.py)."""
import spatial_sbi_sim as sb


def _mc(n, rho, snr=10.0, trials=20000, seed=3):
    return sb.montecarlo(n, rho, snr, trials, seed)


def test_classify_regimes():
    assert sb.classify(sb.CLEAN_DB + 1) == "clean"
    assert sb.classify((sb.CLEAN_DB + sb.DEEP_DB) / 2) == "localized"
    assert sb.classify(sb.DEEP_DB - 1) == "lost"


def test_fractions_sum_to_one():
    r = _mc(4, 0.2)
    assert abs(r["clean"] + r["localized"] + r["lost"] - 1.0) < 1e-9


def test_decorrelated_diversity_collapses_frame_wide_loss():
    r1 = _mc(1, 0.0)
    r4 = _mc(4, 0.1)
    assert r4["lost"] < r1["lost"] * 0.6        # frame-wide bin much smaller
    assert r4["sbi_salvageable_of_corrupt"] > r1["sbi_salvageable_of_corrupt"]


def test_static_correlated_barely_helps():
    # matches the measured "static MRC ~= nil": correlated combining leaves the
    # frame-wide loss almost where a single chain had it.
    r1 = _mc(1, 0.0)
    r4_hi = _mc(4, 0.9)
    assert abs(r4_hi["lost"] - r1["lost"]) < 0.05


def test_precondition_improves_with_decorrelation():
    # SBI-salvageable-of-corrupt rises as antennas decorrelate (mobility)
    lo = _mc(4, 0.1)["sbi_salvageable_of_corrupt"]
    mid = _mc(4, 0.5)["sbi_salvageable_of_corrupt"]
    hi = _mc(4, 0.9)["sbi_salvageable_of_corrupt"]
    assert lo > mid > hi


def test_matched_mean_keeps_clean_fraction_roughly_constant():
    # the benefit is moving frames lost->localized, not raising the clean bin
    r1 = _mc(1, 0.0)
    r4 = _mc(4, 0.1)
    assert abs(r4["clean"] - r1["clean"]) < 0.12
