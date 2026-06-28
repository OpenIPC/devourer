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
