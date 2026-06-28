"""Tests for the energy-min adaptive controller (`controller.py`) + the closed-loop
sim headline.

Pins: higher SNR -> higher MCS / lower power (energy-min); feedback-loss failsafe;
no profile flapping under noisy SNR; enhancement layers shed when infeasible; and
the sim shows a real energy saving vs an over-provisioned fixed baseline while
holding the delivery SLA.
"""

from __future__ import annotations

import os
import random
import sys

import energy_model as em
import link_model as lm
import op_table
from controller import Controller, ControllerConfig


def _ctrl(**kw):
    return Controller(lm.LinkModel(trials=400), em.load_calibration(),
                      ControllerConfig(**kw))


def test_higher_snr_picks_higher_mcs_lower_power():
    c = _ctrl(target=0.99, allow_shed=False)
    # report a strong link (high received SNR at a mid TXAGC) vs a weak one
    strong = c.update(40.0, 32, now_ms=0)
    c2 = _ctrl(target=0.99, allow_shed=False)
    weak = c2.update(8.0, 32, now_ms=0)
    assert strong.mcs > weak.mcs                      # energy-min rides high MCS when it can
    assert strong.txagc <= weak.txagc                 # and spends less power when close


def test_failsafe_on_feedback_timeout():
    c = _ctrl(target=0.99, allow_shed=False, feedback_timeout_ms=1000)
    c.update(30.0, 32, now_ms=0)
    assert c.on_tick(now_ms=500) is not None and c.on_tick(500).mcs != 0   # still good
    fs = c.on_tick(now_ms=2000)                       # > timeout
    assert fs is op_table.MAX_RANGE                   # max-range failsafe


def test_no_flap_under_noisy_snr():
    """Noisy SNR straddling a threshold must not cause continuous MCS/FEC churn.
    (TXAGC tracks SNR freely — it's the cheap power lever; only MCS/overhead
    changes are disruptive, so those are the flap metric.)"""
    c = _ctrl(target=0.99, allow_shed=False)
    rng = random.Random(0)
    prev = None
    changes = 0
    for t in range(300):
        snr = 18.0 + rng.uniform(-2.5, 2.5)           # jitter around a waterfall edge
        op = c.update(snr, op_table.MAX_RANGE.txagc if prev is None else prev.txagc,
                      now_ms=t * 100)
        if prev and (op.mcs, op.overhead) != (prev.mcs, prev.overhead):
            changes += 1
        prev = op
    assert changes < 30                               # hysteresis bounds MCS/FEC churn


def test_enhancement_layer_sheds_when_infeasible():
    # an enhancement layer (allow_shed) at an SNR so low even max power can't reach
    c = _ctrl(target=0.99, allow_shed=True, src_bitrate_bps=8e6)
    op = c.update(-25.0, 0, now_ms=0)                 # nothing clears even at txagc 63
    assert op is None                                 # shed, not failsafe


def test_base_layer_never_sheds():
    c = _ctrl(target=0.99, allow_shed=False)
    op = c.update(-25.0, 0, now_ms=0)
    assert op is op_table.MAX_RANGE                   # base falls to max-range, not None


def test_sim_energy_savings_headline():
    sys.path.insert(0, os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "..", "tests")))
    import sim_loop
    r = sim_loop.main()
    assert r["save_vs_robust"] >= 0.25                # vs over-provisioned set-and-forget
    assert r["save_vs_emin"] > 0.0                    # adapting beats energy-aware-static too
    assert r["delivery"] >= 0.98                      # SLA held (target 0.99, MC noise)
    assert r["changes"] < 100                         # no flapping over 200 ticks


def _drive_pl(ctrl, pl_seq, calib, txagc0=32):
    """Drive a controller with a free-SNR (path-loss) sequence, feeding back the
    reported SNR at the previously applied TXAGC (like the real loop), so the
    controller's recovered path loss — and its variance — are uncontaminated by
    its own power moves."""
    txagc, op = txagc0, None
    for t, pl in enumerate(pl_seq):
        reported = pl + calib.gain_db(txagc)
        op = ctrl.update(reported, txagc, now_ms=t * 100)
        if op is not None:
            txagc = op.txagc
    return op


def test_fade_margin_off_is_behaviourally_unchanged():
    """Default config (fade_margin_k=0): the tracked variance has zero effect on
    the chosen op — the opt-in fade margin never changes shipped behaviour."""
    calib = em.load_calibration()
    seq = [18.0 + (8.0 if i % 2 else -8.0) for i in range(40)]   # volatile
    a = _drive_pl(_ctrl(target=0.99, allow_shed=False, fade_margin_k=0.0), seq, calib)
    b = _drive_pl(_ctrl(target=0.99, allow_shed=False, fade_margin_k=0.0), seq, calib)
    assert (a.mcs, a.overhead, a.txagc) == (b.mcs, b.overhead, b.txagc)


def test_fade_margin_adds_power_headroom_not_airtime():
    """With the fade margin enabled and a volatile path loss, the controller adds
    TXAGC headroom (more power) while KEEPING the energy-min MCS/FEC — the margin
    must not be spent on airtime (lower MCS / heavier FEC), which would overload
    the channel during fades. This is the validated fix for fading under-delivery."""
    calib = em.load_calibration()
    seq = [18.0 + (8.0 if i % 2 else -8.0) for i in range(40)]
    base = _drive_pl(_ctrl(target=0.99, allow_shed=False, fade_margin_k=0.0), seq, calib)
    fade = _drive_pl(_ctrl(target=0.99, allow_shed=False, fade_margin_k=1.5), seq, calib)
    assert fade.txagc > base.txagc                       # more power
    assert fade.mcs == base.mcs and fade.overhead == base.overhead   # same airtime


def test_fade_variance_tracked_only_when_volatile():
    calib = em.load_calibration()
    volatile = _ctrl(target=0.99, fade_margin_k=1.0)
    _drive_pl(volatile, [18.0 + (6.0 if i % 2 else -6.0) for i in range(40)], calib)
    steady = _ctrl(target=0.99, fade_margin_k=1.0)
    _drive_pl(steady, [18.0] * 40, calib)
    assert volatile.pl_var_ema > 1.0 and steady.pl_var_ema < 0.5


def test_svc_uep_graceful_staircase():
    """As SNR drops, enhancement layers (T2 then T1) shed first; base (critical/T0)
    holds longest — the UEP graceful-degradation staircase."""
    from controller import SvcController
    svc = SvcController(lm.LinkModel(trials=300), em.load_calibration())
    # high SNR: all four layers active
    _, _, active_hi = svc.update(40.0, 32, now_ms=0)
    assert active_hi == [0, 1, 2, 3]
    # walk SNR down; record how many layers survive at each step
    counts = []
    for i, snr in enumerate([40, 25, 18, 12, 6, 0, -8]):
        # fresh bank per SNR to avoid hysteresis/hold coupling the sweep
        s = SvcController(lm.LinkModel(trials=300), em.load_calibration())
        _, _, act = s.update(float(snr), 32, now_ms=0)
        counts.append(len(act))
    assert counts == sorted(counts, reverse=True)        # monotonically sheds
    assert counts[0] == 4 and counts[-1] <= 2            # all active high; base-only low


def test_svc_shared_power_is_max_over_active():
    from controller import SvcController
    svc = SvcController(lm.LinkModel(trials=300), em.load_calibration())
    ops, shared, active = svc.update(10.0, 32, now_ms=0)
    present = [ops[s].txagc for s in active]
    assert shared == max(present)                         # one PA: max over active layers
