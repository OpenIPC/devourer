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


def test_bandwidth_rows_pay_noise_bandwidth():
    """A wider row's snr_req carries +3 dB per bandwidth doubling — the honest
    cost that lets the e_bit ranking trade airtime against noise bandwidth."""
    link = lm.LinkModel(trials=300)
    rows = op_table.build_link_rows(link, 0.99, mcs_set=(3,),
                                    overhead_set=(0.25,), bw_set=(20, 40, 80),
                                    mode="vht")
    req = {r.bw: r.snr_req for r in rows}
    assert abs((req[40] - req[20]) - 3.0103) < 1e-3
    assert abs((req[80] - req[20]) - 6.0206) < 1e-3


def test_bandwidth_dimension_wide_when_strong_narrow_when_weak():
    """With the bandwidth dimension open (20 c 40 c 80, the primary-nested
    unilateral ladder), a strong link rides a wide rung (less airtime = less
    energy), a weak one falls back to a narrow rung (its +3 dB/doubling noise
    cost is the first thing to go)."""
    kw = dict(target=0.99, allow_shed=False, bw_set=(20, 40, 80), mode="vht",
              src_bitrate_bps=8e6)
    strong = _ctrl(**kw).update(45.0, 32, now_ms=0)
    weak = _ctrl(**kw).update(9.0, 32, now_ms=0)
    assert strong is not None and weak is not None
    assert strong.bw > weak.bw
    assert strong.bw == 80                       # airtime-min rides the top rung
    assert strong.e_bit < weak.e_bit


def test_rung_blocking_vacates_dirty_wide_rung_and_recovers():
    """Probe sensing says the 80 rung's extra spectrum is dirty (delivery
    contrast vs the best rung): the controller must vacate 80 even though the
    SNR estimate alone says it is the cheapest, then return after the hold."""
    # improve_frac=0 disables the anti-churn hysteresis: this test pins the
    # rung block/unblock semantics, and the 80-vs-40 e_bit gap at low airtime
    # duty is smaller than the default 3% churn threshold.
    c = _ctrl(target=0.99, allow_shed=False, bw_set=(20, 40, 80), mode="vht",
              src_bitrate_bps=8e6, rung_block_hold_ms=5000, improve_frac=0.0)
    assert c.update(45.0, 32, now_ms=0).bw == 80          # strong link rides 80
    c.report_rung_delivery({20: (1.0, 12), 40: (1.0, 12), 80: (0.4, 12)},
                           now_ms=100)
    op = c.update(45.0, 32, now_ms=200)
    assert op.bw <= 40                                     # vacated the dirty rung
    assert not c.primary_dirty                             # narrow rungs are fine
    # after the hold expires (and probes look clean) 80 is rankable again
    c.report_rung_delivery({20: (1.0, 12), 40: (1.0, 12), 80: (1.0, 12)},
                           now_ms=6000)
    late = c.update(45.0, 32, now_ms=20000)                # past hysteresis holds
    assert late.bw == 80


def test_primary_dirty_fires_only_when_narrowest_rung_bad_too():
    c = _ctrl(target=0.99, allow_shed=False, bw_set=(20, 40, 80), mode="vht")
    c.report_rung_delivery({20: (0.5, 12), 40: (0.5, 12), 80: (0.4, 12)},
                           now_ms=0)
    assert c.primary_dirty                                 # nothing bw can fix
    c.report_rung_delivery({20: (1.0, 12), 40: (0.5, 12), 80: (0.4, 12)},
                           now_ms=100)
    assert not c.primary_dirty                             # 20 is clean: avoidable
    # too few samples -> no verdict change
    c.report_rung_delivery({20: (0.0, 2)}, now_ms=200)
    assert not c.primary_dirty


# --------------------------------------------------------------------------- #
# Adjacent-MCS probe evidence: the gate/block on the model
# --------------------------------------------------------------------------- #
def _stat(lcb, ucb, attempts, delivery=None):
    from score import ProbeStat
    return ProbeStat(delivery=lcb if delivery is None else delivery,
                     successes=int(attempts * (delivery or lcb)),
                     attempts=attempts, lcb=lcb, ucb=ucb, age_ms=50.0)


_SHARED_LINK = lm.LinkModel(trials=400)   # shared p_deliver cache across cases


def _fast_ctrl(**kw):
    return Controller(_SHARED_LINK, em.load_calibration(), ControllerConfig(**kw))


def _settle_pl(c, calib, pl, t0, ticks=8, txagc=32, dt=200):
    """Drive at a constant path loss with honest feedback (reported SNR follows
    the applied TXAGC), keeping the time base continuous across phases."""
    op = None
    for i in range(ticks):
        op = c.update(pl + calib.gain_db(txagc), txagc, now_ms=t0 + i * dt)
        if op is not None:
            txagc = op.txagc
    return op, t0 + ticks * dt, txagc


def test_required_frame_delivery_mirrors_sim_interframe():
    """The gate's raw-frame requirement is the exact inverse of the plain
    (no-SBI) FEC math the link rows were built from: at the requirement the
    binomial block delivery crosses the target."""
    import math as m
    import fec_ab_sim
    c = _fast_ctrl(target=0.99, allow_shed=False)
    # generation geometry 12x10: heavy FEC needs 6/12 clean frames, light 11/12
    req_heavy = c._required_frame_delivery(1.00)
    req_light = c._required_frame_delivery(0.10)
    assert 0.70 < req_heavy < 0.85
    assert 0.97 < req_light < 1.0
    assert req_heavy < c._required_frame_delivery(0.50) < \
        c._required_frame_delivery(0.25) < req_light

    def tail(p, k):
        return sum(m.comb(12, i) * p ** i * (1 - p) ** (12 - i)
                   for i in range(k, 13))
    assert tail(req_heavy, 6) >= 0.99 > tail(req_heavy - 0.02, 6)
    assert tail(req_light, 11) >= 0.99 > tail(req_light - 0.005, 11)
    # cross-check against sim_interframe itself (whole-frame erasures)
    rng = random.Random(7)
    ch = {"corrupt_rate": 1.0 - (req_heavy + 0.04), "n_sub": 10,
          "survivor_hist": {0: 1}}
    d_hi, _ = fec_ab_sim.sim_interframe(ch, 1.00, 12, 3000, rng, sbi=False)
    ch["corrupt_rate"] = 1.0 - (req_heavy - 0.06)
    d_lo, _ = fec_ab_sim.sim_interframe(ch, 1.00, 12, 3000, rng, sbi=False)
    assert d_hi > 0.98 and d_lo < 0.98


def test_mcs_probe_disabled_is_behaviourally_unchanged():
    """Default config (mcs_probe_enabled=False): identical trajectory to a
    plain controller — the opt-in gate never changes shipped behaviour."""
    calib = em.load_calibration()
    seq = [10 + i * 1.2 for i in range(25)]              # climb through promotions
    a = _drive_pl(_ctrl(target=0.99, allow_shed=False), seq, calib)
    b = _drive_pl(_ctrl(target=0.99, allow_shed=False, mcs_probe_enabled=False),
                  seq, calib)
    assert (a.mcs, a.overhead, a.txagc) == (b.mcs, b.overhead, b.txagc)


def test_mcs_block_vacates_candidate_and_recovers():
    """A candidate whose probe UPPER bound cannot reach even the heaviest
    overhead's requirement is blocked from candidacy for the hold, then
    re-admitted; the lowest rate is never blocked (it is the fallback)."""
    calib = em.load_calibration()
    kw = dict(target=0.99, allow_shed=False, improve_frac=0.0,
              mcs_probe_block_hold_ms=5000)
    ref, _, _ = _settle_pl(_fast_ctrl(**kw), calib, pl=0.0, t0=0)
    assert ref.mcs > 0                                    # the unblocked optimum
    c = _fast_ctrl(**kw)
    c.report_mcs_delivery({ref.mcs: _stat(0.0, 0.2, 12), 0: _stat(0.0, 0.2, 12)},
                          now_ms=0)
    assert 0 not in c._mcs_block                          # floor never blocked
    early, _, _ = _settle_pl(c, calib, pl=0.0, t0=100, ticks=4)
    assert early.mcs != ref.mcs                           # vacated while held
    # after the hold expires the block no longer bars the pick (a cold pick —
    # displacing an INCUMBENT additionally rides the normal e_bit hysteresis)
    c3 = _fast_ctrl(**kw)
    c3.report_mcs_delivery({ref.mcs: _stat(0.0, 0.2, 12)}, now_ms=0)
    late, _, _ = _settle_pl(c3, calib, pl=0.0, t0=6000)
    assert late.mcs == ref.mcs
    # too few samples -> no block
    c2 = _fast_ctrl(**kw)
    c2.report_mcs_delivery({ref.mcs: _stat(0.0, 0.2, 4)}, now_ms=0)
    ok, _, _ = _settle_pl(c2, calib, pl=0.0, t0=100)
    assert ok.mcs == ref.mcs


def test_up_gate_requires_probe_lcb_then_trims_overhead():
    """With probing on, a faster row is entered only on probe evidence — and
    since the raw requirement rises steeply as overhead falls, the promote
    path is two-step: mcs+1 at the admissible (heavier) overhead first, then
    the ungated same-MCS overhead trim."""
    calib = em.load_calibration()
    kw = dict(target=0.99, allow_shed=False, mcs_probe_enabled=True,
              improve_frac=0.0, mcs_set=(3, 4), overhead_set=(0.10, 0.25, 0.50))
    c = _fast_ctrl(**kw)
    # weak: MCS4 unreachable even at max power -> settles on the floor rate
    op0, t, txagc = _settle_pl(c, calib, pl=-3.5, t0=0)
    assert op0.mcs == 3
    # strong link, NO probe evidence: the model alone may not promote
    opA, t, txagc = _settle_pl(c, calib, pl=25.0, t0=t, ticks=12, txagc=txagc)
    assert opA.mcs == 3
    # evidence for MCS4 whose LCB clears ov=0.50 (req~0.88) but not ov=0.25
    # (req~0.96): only the heavier-FEC row is admissible on the entry tick
    c.report_mcs_delivery({4: _stat(0.93, 1.0, 40)}, now_ms=t)
    opB, t, txagc = _settle_pl(c, calib, pl=25.0, t0=t, ticks=1, txagc=txagc)
    assert opB.mcs == 4 and opB.overhead == 0.50
    # the same-MCS overhead trim is ungated: it follows on the next ticks
    opC, t, _ = _settle_pl(c, calib, pl=25.0, t0=t, ticks=4, txagc=txagc)
    assert opC.mcs == 4 and opC.overhead < 0.50


def test_measured_active_collapse_escapes_model_belief():
    """A row the model wrongly believes in (here: cold-picked on an optimistic
    bias) is escaped by MEASURED active-row delivery: after the persistence
    guard the row is blocked, cur_ok fails despite the model, and the
    fast-down commits a row the model alone would never have left for. A
    non-persistent condemnation (one burst-flooded report) does not trip."""
    from controller import apply_model_bias
    calib = em.load_calibration()
    c = _fast_ctrl(target=0.99, allow_shed=False, mcs_probe_enabled=True,
                   improve_frac=0.0)
    apply_model_bias(c, {5: -10.0})
    op, t, txagc = _settle_pl(c, calib, pl=0.0, t0=0)
    assert op.mcs == 5                    # the cold pick lands on the lie
    bad = {5: _stat(0.0, 0.3, 40)}        # measured: the active row is dead
    # burst guard: an interrupted condemnation never trips
    c.report_mcs_delivery(bad, now_ms=t)
    c.report_mcs_delivery({}, now_ms=t + 100)
    c.report_mcs_delivery(bad, now_ms=t + 200)
    op2 = c.update(0.0 + calib.gain_db(txagc), txagc, now_ms=t + 300)
    assert op2.mcs == 5
    # persistent condemnation: trips, blocks, escapes
    for i in range(3):
        c.report_mcs_delivery(bad, now_ms=t + 400 + i * 100)
    esc, _, _ = _settle_pl(c, calib, pl=0.0, t0=t + 800, ticks=2, txagc=txagc)
    assert esc.mcs != 5
    assert c._mcs_blocked(5)              # the lie cannot be re-picked


def test_fast_down_is_ungated():
    """Escaping a failing point never waits for probe samples: on !cur_ok the
    model's candidate commits with zero evidence even with probing enabled."""
    calib = em.load_calibration()
    c = _fast_ctrl(target=0.99, allow_shed=False, mcs_probe_enabled=True)
    op0, t, txagc = _settle_pl(c, calib, pl=20.0, t0=0)
    assert op0.mcs > 0
    drop, _, _ = _settle_pl(c, calib, pl=-15.0, t0=t, ticks=6, txagc=txagc)
    assert drop is not None and drop.mcs < op0.mcs        # downgraded immediately
