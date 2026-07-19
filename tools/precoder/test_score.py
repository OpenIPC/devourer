"""Tests for the VRX scoring window (`score.py`)."""

from __future__ import annotations

from score import ScoreWindow, ScoreConfig


def _fill(w, n, rssi, snr, crc_err=False, start_seq=0, t0=0.0, dt=0.01):
    for i in range(n):
        w.add_frame(rssi, snr, crc_err, start_seq + i, t0 + i * dt)


def test_score_rises_with_snr():
    lo = ScoreWindow(); _fill(lo, 30, -70, 8)
    hi = ScoreWindow(); _fill(hi, 30, -50, 28)
    assert lo.score() < hi.score()
    assert 1000 <= lo.score() <= 2000 and 1000 <= hi.score() <= 2000


def test_seq_gap_loss_wrap_and_gaps():
    # contiguous arrival straddling a 12-bit wrap -> NO loss (the old sort+max-min
    # mis-read any wrap-straddling window as ~100% loss)
    w = ScoreWindow(ScoreConfig(window_s=100.0))
    for i, s in enumerate([4094, 4095, 0, 1, 2]):
        w.add_frame(-50, 20, False, s, i * 0.001)
    assert w.seq_gap_loss() < 1e-9
    # a real gap: seqs 0,1,3,4 -> span 5, 4 received -> 0.20 loss
    g = ScoreWindow(ScoreConfig(window_s=100.0))
    for i, s in enumerate([0, 1, 3, 4]):
        g.add_frame(-50, 20, False, s, i * 0.001)
    assert abs(g.seq_gap_loss() - 0.2) < 1e-9


def test_snr_estimate_is_windowed_mean():
    w = ScoreWindow(ScoreConfig(window_s=10.0))
    _fill(w, 10, -60, 20)
    assert abs(w.snr_estimate() - 20.0) < 1e-6


def test_window_evicts_old_frames():
    w = ScoreWindow(ScoreConfig(window_s=0.05))
    _fill(w, 10, -60, 20, t0=0.0, dt=0.01)        # spans 0..0.09 s
    w.add_frame(-60, 5, False, 100, now_s=1.0)    # far future -> evicts all but this
    assert w.n() == 1 and abs(w.snr_estimate() - 5.0) < 1e-6


def test_residual_loss_penalizes_score():
    w = ScoreWindow(); _fill(w, 30, -50, 28)
    clean = w.score(residual_loss=0.0)
    lossy = w.score(residual_loss=0.3)
    assert lossy < clean                          # decoder-experienced loss drags the score


def test_seq_gap_loss():
    w = ScoreWindow(ScoreConfig(window_s=100))
    # 5 frames spanning seq 0..9 -> ~50% missing
    for i, s in enumerate([0, 2, 4, 6, 9]):
        w.add_frame(-60, 20, False, s, now_s=i * 0.01)
    assert 0.3 < w.seq_gap_loss() < 0.6


def test_ack_seq_tracks_max():
    w = ScoreWindow()
    _fill(w, 5, -60, 20, start_seq=100)
    assert w.ack_seq() == 104


def test_rung_window_attributes_losses_by_probe_schedule():
    """Kill exactly the seqs whose probe rung is 80 MHz: the 80 rung's delivery
    collapses while 20/40 stay perfect — per-rung sensing from seq gaps alone."""
    import rc_proto as rp
    from score import RungWindow
    bw_set = (20, 40, 80)
    rw = RungWindow(bw_set)
    for s in range(0, 32 * 12):
        if rp.probe_bw(s, bw_set) == 80:
            continue                       # lost on air -> a gap at the VRX
        rw.add_seq(s)
    st = rw.stats()
    assert st[20][0] == 1.0 and st[40][0] == 1.0
    assert st[80][0] == 0.0 and st[80][1] >= 8


# --------------------------------------------------------------------------- #
# Wilson bounds + McsProbeWindow
# --------------------------------------------------------------------------- #
def test_wilson_bounds_known_values():
    from score import wilson_bounds
    z = 1.2816
    assert wilson_bounds(0, 0, z) == (0.0, 1.0)          # no evidence at all
    lcb, ucb = wilson_bounds(12, 12, z)
    # a perfect record caps at n/(n+z^2): "0 failures in 12" is NOT proof of 99%
    assert abs(lcb - 12 / (12 + z * z)) < 1e-9
    assert ucb == 1.0 and lcb < 0.89
    lcb, ucb = wilson_bounds(5, 10, z)
    assert lcb < 0.5 < ucb
    l31, _ = wilson_bounds(31, 31, z)
    assert l31 > lcb                                     # more evidence -> tighter


MCS_SET = tuple(range(8))


def _mcs_stream(win, seqs, sel=4, mode="ht", lost=(), t0=0.0, dt_ms=4.0,
                rate_of=None):
    """Feed a seq stream: probe seqs fly their candidate, others the selected
    rate; seqs in `lost` are skipped (a gap at the VRX)."""
    import rc_proto as rp
    t = t0
    for s in seqs:
        t += dt_ms
        if s in lost:
            continue
        flown = rp.probe_mcs(s, sel, MCS_SET)
        flown = sel if flown is None else flown
        rate = (mode, flown) if rate_of is None else rate_of(s)
        win.add_seq(s, t, rate=rate, crc_err=False)
    return t


def test_mcs_probe_window_attributes_by_schedule():
    """Kill exactly the up-probe seqs: the up candidate's delivery collapses
    while the down candidate stays perfect — sensing from gaps + verified
    receptions alone."""
    import rc_proto as rp
    from score import McsProbeWindow
    win = McsProbeWindow(MCS_SET)
    win.set_selected("ht", 4, 0.0)
    lost = {s for s in range(64 * 16) if rp.probe_mcs(s, 4, MCS_SET) == 5}
    t = _mcs_stream(win, range(64 * 16), lost=lost)
    st = win.stats(t)
    assert st[5].delivery == 0.0 and st[5].attempts >= 12
    assert st[3].delivery == 1.0 and st[3].attempts >= 12
    assert st[5].ucb < 0.3 and st[3].lcb > 0.8


def test_mcs_probe_window_epoch_gating():
    """Gap losses are attributed only while a non-probe frame has confirmed the
    VTX flies the commanded rate; a mismatch un-confirms."""
    import rc_proto as rp
    from score import McsProbeWindow
    lost = {s for s in range(256) if rp.probe_mcs(s, 4, MCS_SET) is not None}
    # never confirmed: every non-probe frame decodes at the WRONG rate
    win = McsProbeWindow(MCS_SET)
    win.set_selected("ht", 4, 0.0)
    t = _mcs_stream(win, range(256), lost=lost, rate_of=lambda s: ("ht", 0))
    assert win.stats(t) == {}
    # confirmed: same losses now count as candidate failures
    win2 = McsProbeWindow(MCS_SET)
    win2.set_selected("ht", 4, 0.0)
    t = _mcs_stream(win2, range(256), lost=lost)
    st = win2.stats(t)
    assert st[5].attempts >= 3 and st[5].delivery == 0.0
    # un-confirm: after a mismatching non-probe frame, attribution stops
    win3 = McsProbeWindow(MCS_SET)
    win3.set_selected("ht", 4, 0.0)
    win3.add_seq(0, 1.0, rate=("ht", 4))       # confirm
    win3.add_seq(1, 2.0, rate=("ht", 0))       # VTX visibly elsewhere -> unconfirm
    win3.add_seq(6, 3.0, rate=("ht", 4))       # gap walks the lost up-probe seq 4
    assert 5 not in win3.stats(4.0)


def test_mcs_probe_window_rate_verify_and_crc():
    from score import McsProbeWindow
    # a received probe-slot frame with a MISMATCHED rate is ignored, not failed:
    # the candidate never verifiably flew (lag, suppressed probe, parse fallback)
    win = McsProbeWindow(MCS_SET)
    win.set_selected("ht", 4, 0.0)
    win.add_seq(3, 1.0, rate=("ht", 4))        # confirm epoch
    win.add_seq(4, 2.0, rate=("ht", 4))        # up-probe slot flew at selected
    assert 5 not in win.stats(3.0)
    # rate-less frames are dropped entirely (fail-inert without telemetry)
    win.add_seq(68, 3.0, rate=None)            # up-probe slot, no rate info
    assert 5 not in win.stats(4.0)
    # a crc_err frame never advances the gap walk (its body seq is untrusted
    # bits) but its PHY-decoded rate is descriptor truth: a corrupt frame at a
    # tracked rate is that rate's measured failure, pushed seq-free — the
    # evidence that keeps flowing when the stream is crc-flooded
    win2 = McsProbeWindow(MCS_SET)
    win2.set_selected("ht", 4, 0.0)
    win2.add_seq(3, 1.0, rate=("ht", 4))       # confirm; last_seq = 3
    win2.add_seq(999, 2.0, rate=("ht", 4), crc_err=True)   # active-rate failure
    win2.add_seq(998, 2.1, rate=("ht", 5), crc_err=True)   # candidate failure
    win2.add_seq(997, 2.2, rate=("ht", 0), crc_err=True)   # untracked: ignored
    win2.add_seq(996, 2.3, rate=None, crc_err=True)        # rate-less: ignored
    win2.add_seq(5, 3.0, rate=("ht", 4))       # gap 3->5 walks the lost seq 4
    st = win2.stats(4.0)
    assert st[5].attempts == 2 and st[5].delivery == 0.0   # gap walk + crc
    assert (4 in st) and not st[4].delivery == 1.0         # active crc counted
    assert 0 not in st


def test_mcs_probe_window_tracks_active_row():
    """The selected row's delivery is measured from the main stream (free
    evidence — this is what lets the controller escape a rate its model
    wrongly believes in); bandwidth-probe slots are bandwidth evidence and
    never touch it."""
    import rc_proto as rp
    from score import McsProbeWindow
    bw_set = (20, 40, 80)
    win = McsProbeWindow(MCS_SET, bw_set=bw_set)
    win.set_selected("ht", 4, 0.0)
    t = _mcs_stream(win, range(256))
    st = win.stats(t)
    assert st[4].delivery == 1.0
    assert st[4].attempts > st[5].attempts        # main stream >> probe duty
    # every bw-probe seq killed: the ACTIVE row stays clean (their loss is
    # about the rung's spectrum, not the rate)
    lost = {s for s in range(256, 512) if rp.probe_bw(s, bw_set) is not None}
    t = _mcs_stream(win, range(256, 512), lost=lost, t0=t)
    assert win.stats(t)[4].delivery == 1.0
    # the active (non-probe) seqs killed: the selected row's delivery
    # collapses while the probe slots keep verifying
    lost = {s for s in range(512, 768)
            if rp.probe_mcs(s, 4, MCS_SET) is None
            and rp.probe_bw(s, bw_set) is None}
    t = _mcs_stream(win, range(512, 768), lost=lost, t0=t)
    st = win.stats(t)
    assert st[4].delivery < 0.2 and st[4].ucb < 0.5
    assert st[5].delivery == 1.0


def test_mcs_probe_window_wrap_age_and_reset():
    import rc_proto as rp
    from score import McsProbeWindow
    win = McsProbeWindow(MCS_SET, max_age_ms=8000.0)
    win.set_selected("ht", 4, 0.0)
    # contiguous stream straddling the 12-bit wrap: no spurious attribution
    t = _mcs_stream(win, [4094, 4095, 0, 1, 2], t0=0.0)
    assert all(s.delivery == 1.0 for s in win.stats(t).values())
    # probes keep landing on the right candidates after the wrap
    t = _mcs_stream(win, range(3, 130), t0=t)
    st = win.stats(t)
    assert st[5].attempts >= 2 and st[3].attempts >= 2
    # age expiry: far-future stats() prunes everything
    assert win.stats(t + 9000.0) == {}
    # a profile change blanks the window (evidence is context-bound)
    t = _mcs_stream(win, range(200, 400), t0=0.0)
    assert win.stats(t) != {}
    win.set_selected("ht", 5, t)
    assert win.stats(t) == {}
