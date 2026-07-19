"""Tests for the adaptive-link orchestrator (`adaptive_link.py`).

Drives the VTX<->VRX closed loop in-process (no radio) through a fly-out-and-back
channel and asserts the loop holds delivery while power tracks range; plus the
VTX-side RCF application and failsafe.
"""

from __future__ import annotations

import adaptive_link as al
import rc_proto as rp
import rendezvous as rz


def test_closed_loop_holds_delivery_and_tracks_power():
    r = al.selftest()
    assert r["avg_deliver"] >= 0.95                 # SLA held through the flight
    assert r["far_txagc"] >= r["close_txagc"] + 25  # much more power far than close
    assert r["far_txagc"] >= 32                      # clearly cranks power at range
    assert r["close_txagc"] <= 16                    # backs off when close


def test_op_to_ladder_and_overhead_mapping():
    class Op:
        mcs, bw, overhead = 3, 20, 0.5
    spec = al.op_to_ladder(Op())
    assert "CRIT=MCS3/20" in spec and "T2=MCS5/20" in spec   # base robust, enh steps up

    class VhtOp:                       # bandwidth-dimension rung (VHT row)
        mcs, bw, overhead, mode = 3, 80, 0.5, "vht"
    vspec = al.op_to_ladder(VhtOp())
    assert "CRIT=VHT1SS_MCS3/80" in vspec and "T2=VHT1SS_MCS5/80" in vspec
    assert al.overhead_to_16ths(0.25) == 4
    assert al.overhead_to_16ths(1.0) == 16
    assert al.overhead_to_16ths(0.0) == 1                    # clamped to >=1


def test_vtx_applies_rcf():
    vtx = al.AdaptiveVtx(0xABCD)
    rcf = rp.pack_rcf(rp.Rcf(vtx_id=0xABCD, pwr_idx=40, fec_overhead_16ths=8))
    vtx.on_rc_frame(rcf, now_ms=0)
    assert vtx.state.txagc == 40 and abs(vtx.state.overhead - 0.5) < 1e-9
    assert vtx.rz.state == rz.RC_OK and not vtx.state.failsafe


def test_vtx_ignores_rcf_for_other_vtx():
    vtx = al.AdaptiveVtx(0xABCD)
    vtx.on_rc_frame(rp.pack_rcf(rp.Rcf(vtx_id=0x1111, pwr_idx=10)), now_ms=0)
    assert vtx.state.txagc == 32                     # unchanged (default)


def test_vtx_failsafe_on_rc_loss():
    vtx = al.AdaptiveVtx(0xABCD, rz.VtxConfig(vtx_id=0xABCD, fallback_ms=100))
    vtx.on_rc_frame(rp.pack_rcf(rp.Rcf(vtx_id=0xABCD, pwr_idx=10)), now_ms=0)
    assert vtx.step(50) == rz.A_TX_VIDEO
    assert vtx.step(200) == rz.A_FAILSAFE            # > fallback_ms
    assert vtx.state.failsafe and vtx.state.txagc == 63   # max-range power


def test_vrx_answers_disc_handshake():
    # VTX in discovery hears a VRX DISC -> answers DISC_ACK and resumes
    vtx = al.AdaptiveVtx(0xABCD, rz.VtxConfig(vtx_id=0xABCD, fallback_ms=100,
                                              grace_ms=100))
    vtx.on_rc_frame(rp.pack_rcf(rp.Rcf(vtx_id=0xABCD)), 0)
    t = 1
    while vtx.rz.state != rz.DISCOVERY and t < 1000:
        vtx.step(t); t += 5
    disc = rp.pack_disc(rp.Disc(vtx_id=0xABCD, vrx_nonce=42, op_channel=36))
    # advance to a listen window
    while not vtx.rz.listening(t):
        vtx.step(t); t += 5
    ack = vtx.on_rc_frame(disc, t)
    assert ack is not None and rp.frame_type(ack) == rp.T_DISC_ACK
    assert vtx.rz.state == rz.RC_OK and vtx.rz.op_channel == 36


def test_vtx_decodes_v2_profile_to_vht_ladder_and_probes():
    vtx = al.AdaptiveVtx(0xABCD, bw_set=(20, 40, 80))
    rcf = rp.pack_rcf(rp.Rcf(vtx_id=0xABCD,
                             profile=rp.encode_profile("vht", 2, 80),
                             pwr_idx=40, fec_overhead_16ths=8))
    vtx.on_rc_frame(rcf, now_ms=0)
    assert "CRIT=VHT1SS_MCS2/80" in vtx.state.ladder
    # shared probe schedule: slots 0/8/16 fly the rungs, others ride the op bw
    assert vtx.probe_bw_for_seq(0) == 20
    assert vtx.probe_bw_for_seq(8) == 40
    assert vtx.probe_bw_for_seq(16) == 80
    assert vtx.probe_bw_for_seq(1) is None
    # legacy v1 profile byte (raw MCS) still yields the old HT/20 ladder
    rcf1 = rp.pack_rcf(rp.Rcf(vtx_id=0xABCD, profile=3, pwr_idx=40,
                              fec_overhead_16ths=8))
    vtx.on_rc_frame(rcf1, now_ms=100)
    assert "CRIT=MCS3/20" in vtx.state.ladder


def test_rate_spec_ht_and_vht():
    assert al.rate_spec("ht", 4, 20) == "MCS4/20"
    assert al.rate_spec("vht", 3, 80) == "VHT1SS_MCS3/80"


def test_vtx_tracks_commanded_rate_fields():
    """TxState carries the commanded (mode, mcs, bw) as fields — the SET_RATE
    plumbing and probe schedule read these, never re-parse the ladder string
    (which broke on VHT ladders)."""
    vtx = al.AdaptiveVtx(0xABCD)
    vtx.on_rc_frame(rp.pack_rcf(rp.Rcf(vtx_id=0xABCD,
                                       profile=rp.encode_profile("vht", 2, 80))), 0)
    assert (vtx.state.mode, vtx.state.mcs, vtx.state.bw) == ("vht", 2, 80)
    assert al.rate_spec(vtx.state.mode, vtx.state.mcs, vtx.state.bw) == \
        "VHT1SS_MCS2/80"


def test_vtx_mcs_probe_schedule_and_suppression():
    """probe_mcs_for_seq flies the shared schedule only in the steady TX-video
    state; up-probes never ride protected frames, an explicit suspend, or
    failsafe; counters expose the attempted probes per candidate."""
    vtx = al.AdaptiveVtx(0xABCD, rz.VtxConfig(vtx_id=0xABCD, fallback_ms=100),
                         mcs_probe=True)
    # before any feedback/step: not in TX-video -> no probes
    assert vtx.probe_mcs_for_seq(4) is None
    vtx.on_rc_frame(rp.pack_rcf(rp.Rcf(vtx_id=0xABCD,
                                       profile=rp.encode_profile("ht", 4, 20))), 0)
    assert vtx.step(10) == rz.A_TX_VIDEO
    assert vtx.probe_mcs_for_seq(4) == 5          # up slot
    assert vtx.probe_mcs_for_seq(20) == 3         # down slot
    assert vtx.probe_mcs_for_seq(5) is None       # off-slot
    # protected frames never carry the speculative up-probe; down stays allowed
    assert vtx.probe_mcs_for_seq(4 + 64, protected=True) is None
    assert vtx.probe_mcs_for_seq(20 + 64, protected=True) == 3
    # explicit back-off hook (thermal/congestion): same asymmetry
    vtx.suspend_upward_probes(True)
    assert vtx.probe_mcs_for_seq(4 + 128) is None
    assert vtx.probe_mcs_for_seq(20 + 128) == 3
    vtx.suspend_upward_probes(False)
    assert vtx.probe_counters == {5: 1, 3: 3}
    # failsafe clamps probing entirely (and the failsafe rate fields match the
    # failsafe ladder, not the last commanded profile)
    assert vtx.step(500) == rz.A_FAILSAFE
    assert (vtx.state.mode, vtx.state.mcs, vtx.state.bw) == ("ht", 0, 20)
    assert vtx.probe_mcs_for_seq(4 + 192) is None
    assert vtx.probe_mcs_for_seq(20 + 192) is None


def test_vrx_mcs_probe_stats_feed_controller_and_block():
    """Closed VRX-side loop with probing on: up-probe seqs go missing (their
    rate is too fragile on the real channel) -> the estimator attributes the
    losses to the candidate, and the controller blocks it; a profile change
    then blanks the evidence (context-bound)."""
    import energy_model as em
    import link_model as lm
    from controller import ControllerConfig
    link, calib = lm.LinkModel(trials=300), em.load_calibration()
    vrx = al.AdaptiveVrx(link, calib, 0xABCD,
                         ControllerConfig(target=0.99, allow_shed=False,
                                          mcs_probe_enabled=True))
    state = {"seq": 0}

    def run(n_frames, t0, pl, lose_up=False):
        t = t0
        for _ in range(n_frames):
            seq = state["seq"]
            state["seq"] = (seq + 1) & 0xFFF      # continuous across phases
            t += 4.0
            sel = vrx.cur_op.mcs
            # honest feedback: the reported SNR follows the applied power
            snr = pl + calib.gain_db(vrx._cur_txagc)
            cand = rp.probe_mcs(seq, sel, vrx.ctrl.cfg.mcs_set)
            flown = sel if cand is None else cand
            if lose_up and cand is not None and cand > sel:
                pass                              # lost on air -> a gap
            else:
                vrx.on_video(rssi=-52.0, snr=snr, crc_err=False, seq=seq,
                             now_ms=t, rate=("ht", flown))
            vrx.step(t)
        return t

    t = run(400, t0=0, pl=0.0)                    # settle the operating point
    sel = vrx.cur_op.mcs
    assert 0 < sel < 7
    t = run(1400, t0=t, pl=0.0, lose_up=True)
    assert vrx.cur_op.mcs == sel                  # blocked/ungated: no promotion
    st = vrx.mcs_win.stats(t)
    # a couple of pre-loss successes may linger inside max_age; the verdict
    # lives in the confidence bound, not the point estimate
    assert st[sel + 1].attempts >= 12 and st[sel + 1].ucb < 0.5
    assert st[sel - 1].delivery == 1.0            # down-probes keep arriving
    assert vrx.ctrl._mcs_blocked(sel + 1)         # empirically bad: blocked
    # a commanded-rate change resets the evidence window
    vrx.mcs_win.set_selected("ht", sel - 1, t)
    assert vrx.mcs_win.stats(t) == {}


def test_vrx_video_loss_resets_to_max_range():
    """Total video silence -> the VRX drops its command to MAX_RANGE and
    forgets the committed point (an operating point it cannot hear starves the
    very estimator that could condemn it — the frozen-command deadlock), while
    the measured MCS blocks survive so reacquisition cannot re-pick a
    condemned row. RCFs stopping is also what lets the VTX-side failsafe fire."""
    import energy_model as em
    import link_model as lm
    import op_table
    from controller import ControllerConfig
    link, calib = lm.LinkModel(trials=300), em.load_calibration()
    vrx = al.AdaptiveVrx(link, calib, 0xABCD,
                         ControllerConfig(target=0.99, allow_shed=False,
                                          mcs_probe_enabled=True))
    for i in range(30):
        vrx.on_video(rssi=-50.0, snr=25.0, crc_err=False, seq=i, now_ms=i * 4.0,
                     rate=("ht", vrx.cur_op.mcs))
        vrx.step(i * 4.0)
    assert vrx.cur_op.mcs > 0             # settled on a real rate
    vrx.ctrl._mcs_block[7] = 1 << 40      # a held measured block
    fb = vrx.step(5000.0)                 # > link_lost_ms of silence
    assert fb is not None and rp.frame_type(fb) == rp.T_DISC
    assert vrx.cur_op is op_table.MAX_RANGE
    assert vrx.ctrl.cur is None
    assert 7 in vrx.ctrl._mcs_block       # measured evidence survives


def test_vrx_senses_rungs_and_flags_primary_dirty():
    """Closed VRX-side loop: video seqs arrive with the 80-rung probes missing
    -> the controller vacates 80; then ALL probe rungs go missing while video
    still flows -> primary_dirty escalation fires."""
    import energy_model as em
    import link_model as lm
    from controller import ControllerConfig
    link, calib = lm.LinkModel(trials=300), em.load_calibration()
    vrx = al.AdaptiveVrx(link, calib, 0xABCD,
                         ControllerConfig(target=0.99, allow_shed=False,
                                          bw_set=(20, 40, 80), mode="vht",
                                          src_bitrate_bps=8e6))

    def run(secs_of_frames, lost_rungs, t0):
        t = t0
        for s in range(secs_of_frames):
            seq = s % 4096
            if rp.probe_bw(seq, (20, 40, 80)) in lost_rungs:
                continue
            t = t0 + s * 2.0
            vrx.on_video(rssi=-52.0, snr=38.0, crc_err=False, seq=seq, now_ms=t)
            fb = vrx.step(t)
        return t

    t = run(1024, lost_rungs=(), t0=0)
    assert vrx.cur_op.bw == 80                       # clean: rides the top rung
    t = run(1024, lost_rungs=(80,), t0=t + 10)
    assert vrx.cur_op.bw <= 40                       # dirty 80 rung vacated
    assert not vrx.primary_dirty
    run(1024, lost_rungs=(20, 40, 80), t0=t + 10)
    assert vrx.primary_dirty                         # nothing bandwidth can fix
