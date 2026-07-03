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
