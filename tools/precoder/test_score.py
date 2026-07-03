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
