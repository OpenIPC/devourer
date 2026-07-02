"""Tests for the per-SVC-layer spatial UEP sim (svc_spatial_uep_sim.py)."""
import svc_spatial_uep_sim as uep


def test_stbc_lifts_phy_delivery():
    # STBC shifts the MCS threshold down -> more delivery at a fixed SNR
    for snr in (8.0, 10.0, 12.0):
        assert uep.phy_delivery(snr, 5, True) >= uep.phy_delivery(snr, 5, False)


def test_binom_cdf_bounds():
    assert uep._binom_cdf(0, 10, 0.0) == 1.0      # no loss -> always within budget
    assert uep._binom_cdf(0, 10, 1.0) == 0.0      # all lost, 0 repair -> fail
    assert 0.0 < uep._binom_cdf(2, 12, 0.3) < 1.0


def test_staircase_importance_ordering():
    # at a mid SNR the delivery order follows layer importance
    for snr in (10.0, 12.0, 14.0, 16.0):
        ds = [uep.layer_delivery(snr, L) for L in uep.DEFAULT_LADDER]
        assert ds[0] >= ds[1] >= ds[2] >= ds[3]


def test_critical_survives_deeper_than_enhancement():
    s_crit = uep.survival_snr(uep.DEFAULT_LADDER[0])
    s_t2 = uep.survival_snr(uep.DEFAULT_LADDER[3])
    assert s_crit < s_t2 - 10.0        # critical holds many dB deeper


def test_spatial_axis_extends_survival():
    # the third UEP knob: STBC pushes the critical layer's survival lower
    crit = uep.DEFAULT_LADDER[0]
    s_on = uep.survival_snr(crit, stbc_enabled=True)
    s_off = uep.survival_snr(crit, stbc_enabled=False)
    assert s_on < s_off
    assert abs((s_off - s_on) - uep.STBC_GAIN_DB) < 0.3   # ~= the coding gain


def test_enhancement_has_no_spatial_axis():
    # T1/T2 carry no STBC, so toggling the spatial axis doesn't change them
    for L in uep.DEFAULT_LADDER[2:]:
        assert uep.survival_snr(L, stbc_enabled=True) == uep.survival_snr(L, stbc_enabled=False)
