"""Adaptive-link orchestrator — ties the controller, scorer, protocol, and
rendezvous into the VTX and VRX control loops.

This is the policy plane (the mechanical knobs live in the C++ duplex binary). The
two roles:

  AdaptiveVrx (ground): consumes the VTX's video frames (`rx.frame` events:
    RSSI/SNR/crc/seq), scores the link, runs the energy-min controller to pick the
    operating point, and emits RCF feedback (or DISC beacons when the VTX is
    lost) ~10 Hz.
  AdaptiveVtx (drone): consumes RCF/DISC, applies the operating point to the live
    knobs (TX power, per-layer MCS ladder, FEC overhead), and runs the failsafe +
    discovery state machine.

The radio I/O (a subprocess duplex) is abstracted; `selftest()` wires a
VTX and a VRX through a simulated channel (link_model) so the whole closed loop is
deterministically testable without hardware. The CLI (`--role vtx|vrx`) wires the
real duplex binary; that path is exercised on-air by tests/adaptive_onair.sh.
"""

from __future__ import annotations

from dataclasses import dataclass

import rc_proto as rp
import rendezvous as rz
from controller import Controller, ControllerConfig
from score import ScoreWindow, ScoreConfig, RungWindow, McsProbeWindow
import op_table


def ladder_spec(mode: str, mcs: int, bw: int) -> str:
    """DEVOURER_SVC_LADDER spec for a (mode, base-MCS, bw) operating point.
    Base/critical fly the chosen (robust) MCS; enhancement steps up from there.
    VHT rows (the bandwidth-dimension rungs, incl. 80 MHz) map to VHT1SS_MCSn
    rate strings; the bandwidth rides every layer's /bw suffix — per-packet
    and unilateral while it stays on the RX's primary
    (tests/rx80_narrow_tx_probe.sh). Shared by the VRX-side op mapping and the
    VTX-side RCF profile decode so both ends build the identical ladder."""
    top, name = (8, "VHT1SS_MCS") if mode == "vht" else (7, "MCS")
    m = max(0, min(top, mcs))
    e = min(top, m + 2)
    return (f"CRIT={name}{m}/{bw};T0={name}{m}/{bw};"
            f"T1={name}{min(top, m + 1)}/{bw};T2={name}{e}/{bw}")


def op_to_ladder(op) -> str:
    """Map a controller OpPoint to a DEVOURER_SVC_LADDER spec."""
    return ladder_spec(getattr(op, "mode", "ht"), op.mcs, op.bw)


def rate_spec(mode: str, mcs: int, bw: int) -> str:
    """One rate as a SET_RATE / DEVOURER_TX_RATE spec string. The /bw suffix is
    always explicit so a probe frame and its restore pin the commanded
    bandwidth rather than inheriting whatever the session last flew."""
    name = "VHT1SS_MCS" if mode == "vht" else "MCS"
    return f"{name}{mcs}/{bw}"


def overhead_to_16ths(ov: float) -> int:
    return max(1, min(16, round(ov * 16)))


# --------------------------------------------------------------------------- #
# VRX (ground)
# --------------------------------------------------------------------------- #
class AdaptiveVrx:
    def __init__(self, link, calib, vtx_id: int,
                 ctrl_cfg: ControllerConfig | None = None,
                 score_cfg: ScoreConfig | None = None,
                 op_channel: int = 6, feedback_period_ms: int = 100):
        self.vtx_id = vtx_id
        self.ctrl = Controller(link, calib, ctrl_cfg or ControllerConfig())
        self.win = ScoreWindow(score_cfg)
        # Per-rung delivery sensing (with the bandwidth dimension open): every
        # video seq — received or gap-inferred-lost — is attributed to its
        # probe rung by the shared schedule, and the stats feed the
        # controller's rung blocker / primary-dirty detector each feedback
        # tick. `primary_dirty` (property below) is the escalation signal the
        # embedding app turns into a coordinated channel move.
        bw_set = self.ctrl.cfg.bw_set
        self.rungs = RungWindow(bw_set) if bw_set else None
        # Adjacent-MCS probe evidence (opt-in): the selected rate follows the
        # controller's committed op, so the window resets itself on every
        # commanded (mode, mcs, bw) transition — see _track_selected().
        cfg = self.ctrl.cfg
        self.mcs_win = McsProbeWindow(
            cfg.mcs_set, mode=cfg.mode,
            samples_per_mcs=cfg.mcs_probe_window_samples,
            max_age_ms=cfg.mcs_probe_max_age_ms,
            confidence=cfg.mcs_probe_confidence,
            bw_set=cfg.bw_set) if cfg.mcs_probe_enabled else None
        self._sel_key: tuple | None = None
        self.rz = rz.VrxRendezvous(rz.VrxConfig(vtx_id=vtx_id, op_channel=op_channel))
        self.feedback_period_ms = feedback_period_ms
        self._last_fb_ms = -1 << 60
        self._seq = 0
        self.cur_op = op_table.MAX_RANGE
        self._cur_txagc = 32
        self._video_lost = False

    @property
    def primary_dirty(self) -> bool:
        return self.ctrl.primary_dirty

    def on_video(self, rssi: float, snr: float, crc_err: bool, seq: int,
                 now_ms: float, residual_loss: float | None = None,
                 rate: tuple | None = None) -> None:
        """`rate` is the frame's PHY-decoded (mode, mcs) when the event stream
        carries it (devourer_events.desc_rate_to_mcs) — the probe attribution
        is rate-verified, so without it the MCS window stays fail-inert."""
        self.win.add_frame(rssi, snr, crc_err, seq, now_ms / 1000.0)
        if self.rungs is not None:
            self.rungs.add_seq(seq)
        if self.mcs_win is not None:
            self.mcs_win.add_seq(seq, now_ms, rate=rate, crc_err=crc_err)
        self.rz.feed_video(now_ms)
        self._residual = residual_loss

    def _track_selected(self, now_ms: float) -> None:
        op = self.cur_op
        key = (getattr(op, "mode", "ht"), op.mcs, op.bw)
        if key != self._sel_key:
            self._sel_key = key
            if self.mcs_win is not None:
                self.mcs_win.set_selected(key[0], key[1], now_ms)

    def on_disc_ack(self, buf: bytes, now_ms: float) -> None:
        ack = rp.parse_disc_ack(buf)
        if ack:
            self.rz.feed_disc_ack(ack, now_ms)

    def step(self, now_ms: float) -> bytes | None:
        """Return a frame to TX back to the VTX (RCF or DISC), or None."""
        act = self.rz.tick(now_ms)
        if act == rz.A_BEACON:
            # Video gone entirely: an operating point we cannot hear is not
            # worth keeping, and with zero frames the estimator can never
            # condemn it — the frozen-command deadlock. Take the max-range
            # stance (the dual of the VTX's RC-loss failsafe; RCFs stopping is
            # what lets the VTX failsafe fire) and re-pick fresh on
            # reacquisition; measured MCS blocks survive the reset so the new
            # cold pick avoids the rows the evidence just condemned.
            if not self._video_lost:
                self._video_lost = True
                self.cur_op = op_table.MAX_RANGE
                self.ctrl.reset_operating_point()
                self._track_selected(now_ms)
            return rp.pack_disc(self.rz.beacon())
        if act != rz.A_TX_FEEDBACK:
            return None
        self._video_lost = False
        if now_ms - self._last_fb_ms < self.feedback_period_ms:
            return None
        self._last_fb_ms = now_ms
        if self.rungs is not None:
            self.ctrl.report_rung_delivery(self.rungs.stats(), now_ms)
        if self.mcs_win is not None:
            self.ctrl.report_mcs_delivery(self.mcs_win.stats(now_ms), now_ms)
        snr = self.win.snr_estimate()
        if snr is not None:
            op = self.ctrl.update(snr, self._cur_txagc, now_ms)
            if op is not None:
                self.cur_op = op
                self._cur_txagc = op.txagc
        self._track_selected(now_ms)
        self._seq = (self._seq + 1) & 0xFFFF
        # profile carries the GS-chosen operating point — mode/MCS/bw packed by
        # the shared v2 encoding; the VTX builds its SVC ladder from it
        # (base/critical at this MCS, enhancement steps up).
        rcf = rp.Rcf(vtx_id=self.vtx_id, seq=self._seq, ack_seq=self.win.ack_seq(),
                     profile=rp.encode_profile(getattr(self.cur_op, "mode", "ht"),
                                               self.cur_op.mcs, self.cur_op.bw),
                     score=self.win.score(getattr(self, "_residual", None)),
                     pwr_idx=self.cur_op.txagc,
                     fec_overhead_16ths=overhead_to_16ths(self.cur_op.overhead),
                     layer_delivery=(int(100 * (1 - self.win.seq_gap_loss())),))
        return rp.pack_rcf(rcf)


# --------------------------------------------------------------------------- #
# VTX (drone)
# --------------------------------------------------------------------------- #
@dataclass
class TxState:
    txagc: int = 32
    overhead: float = 0.25
    ladder: str = "CRIT=MCS1/20;T0=MCS2/20;T1=MCS4/20;T2=MCS5/20"
    failsafe: bool = False
    # The commanded base rate as fields (kept in lockstep with `ladder`): the
    # live SET_RATE plumbing and the MCS-probe schedule derive from these
    # rather than re-parsing the ladder string.
    mode: str = "ht"
    mcs: int = 1
    bw: int = 20


class AdaptiveVtx:
    def __init__(self, vtx_id: int, vtx_cfg: rz.VtxConfig | None = None,
                 failsafe_txagc: int = 63, failsafe_overhead: float = 1.0,
                 failsafe_ladder: str | None = None,
                 bw_set: tuple | None = None,
                 mcs_probe: bool = False, mcs_set: tuple = tuple(range(8))):
        self.vtx_id = vtx_id
        self.rz = rz.VtxRendezvous(vtx_cfg or rz.VtxConfig(vtx_id=vtx_id))
        self.state = TxState()
        self.failsafe_txagc = failsafe_txagc
        self.failsafe_overhead = failsafe_overhead
        self.failsafe_ladder = failsafe_ladder or \
            "CRIT=MCS0/20/LDPC;T0=MCS0/20/LDPC;T1=MCS0/20;T2=MCS0/20"
        # Bandwidth-probe rungs — must match the VRX controller's bw_set (a
        # config invariant, like the profile-table version). None = no probing.
        self.bw_set = tuple(sorted(bw_set)) if bw_set else None
        # Adjacent-MCS probing — mcs_set is the same config invariant as
        # bw_set; the schedule itself (rc_proto.probe_mcs) is versioned.
        self.mcs_probe = mcs_probe
        self.mcs_set = tuple(sorted(mcs_set))
        self.probe_counters: dict[int, int] = {}     # candidate mcs -> attempts
        self._last_act: str | None = None
        self._up_suspended = False

    def probe_bw_for_seq(self, seq: int) -> int | None:
        """Rung this video seq must fly at as a bandwidth probe (the shared
        schedule the VRX attributes against), else None -> the commanded op
        bandwidth. Injectors apply it as the frame's radiotap bandwidth —
        per-packet and unilateral (tests/rx80_narrow_tx_probe.sh)."""
        if self.bw_set is None or self.state.failsafe:
            return None
        return rp.probe_bw(seq, self.bw_set)

    def suspend_upward_probes(self, suspended: bool) -> None:
        """Hook for the embedding app's back-off signals (thermal, congestion,
        delivery collapse): an up-probe is speculative airtime the link may not
        afford, a down-probe stays useful (it tells 'drop MCS' apart from 'all
        rates failing')."""
        self._up_suspended = bool(suspended)

    def probe_mcs_for_seq(self, seq: int, protected: bool = False) -> int | None:
        """Candidate MCS this video seq must fly at as a rate probe (shared
        schedule, commanded bandwidth/power/FEC untouched), else None -> the
        commanded rate. `protected` marks a frame whose loss is not disposable
        (control, base-layer critical, IDR): it never carries an UP-probe —
        losing it to a speculative faster rate would damage exactly the traffic
        the SLA protects. Down-probes are more robust than the operating point
        and stay allowed. Probing pauses entirely outside the steady TX-video
        state (failsafe/listen/discovery): those regimes fly reach-first."""
        if not self.mcs_probe or self.state.failsafe \
                or self._last_act != rz.A_TX_VIDEO:
            return None
        cand = rp.probe_mcs(seq, self.state.mcs, self.mcs_set)
        if cand is None:
            return None
        if cand > self.state.mcs and (protected or self._up_suspended):
            return None
        self.probe_counters[cand] = self.probe_counters.get(cand, 0) + 1
        return cand

    def on_rc_frame(self, buf: bytes, now_ms: float) -> bytes | None:
        """Process an inbound control frame. Applies an RCF (updates TxState) or
        answers a DISC with a DISC_ACK to TX. Returns a frame to TX, or None."""
        t = rp.frame_type(buf)
        if t == rp.T_RCF:
            r = rp.parse_rcf(buf)
            if r is None or r.vtx_id != self.vtx_id:
                return None
            self.rz.feed_rc(now_ms)
            if r.pwr_idx != rp.PWR_NO_CHANGE:
                self.state.txagc = r.pwr_idx
            self.state.overhead = r.fec_overhead
            self.state.failsafe = False
            # Ladder follows the GS-chosen operating point in `profile`
            # (mode/MCS/bw, shared v2 encoding + shared ladder builder).
            mode, m, bw = rp.decode_profile(r.profile)
            self.state.ladder = ladder_spec(mode, m, bw)
            self.state.mode, self.state.mcs, self.state.bw = mode, m, bw
            return None
        if t == rp.T_DISC:
            d = rp.parse_disc(buf)
            if d is None:
                return None
            ack = self.rz.feed_disc(d, now_ms)
            return rp.pack_disc_ack(ack) if ack else None
        return None

    def step(self, now_ms: float) -> str:
        """Advance the failsafe/discovery SM; clamp TxState to failsafe when lost.
        Returns the rendezvous action (tx_video / failsafe / listen / idle)."""
        act = self.rz.tick(now_ms)
        self._last_act = act
        if act == rz.A_FAILSAFE:
            self.state = TxState(txagc=self.failsafe_txagc,
                                 overhead=self.failsafe_overhead,
                                 ladder=self.failsafe_ladder, failsafe=True,
                                 mode="ht", mcs=0, bw=20)   # match failsafe_ladder
        return act

    def apply_ladder_from_op(self, op) -> None:
        self.state.ladder = op_to_ladder(op)


# --------------------------------------------------------------------------- #
# In-process closed-loop self-test (no radio) — validates the control loop.
# --------------------------------------------------------------------------- #
def selftest(verbose: bool = False):
    """Wire a VTX and VRX through a simulated channel; sweep path loss and assert
    the loop holds delivery while the commanded TX power/FEC track the link."""
    import energy_model as em
    import link_model as lm

    link = lm.LinkModel(trials=400)
    calib = em.load_calibration()
    vtx = AdaptiveVtx(0xABCD)
    vrx = AdaptiveVrx(link, calib, 0xABCD,
                      ctrl_cfg=ControllerConfig(target=0.99, allow_shed=False,
                                                src_bitrate_bps=4e6))
    # path loss (free SNR at txagc 0) triangle: starts FAR, close at mid, far again
    sched = [35 - abs(t - 100) * 0.45 for t in range(200)]
    # the VTX transmits at the operating point the VRX last commanded (the whole
    # (mcs, txagc, overhead) applied atomically, with a uniform 1-tick lag).
    applied = vrx.cur_op
    deliveries, pwr_at = [], []
    for t, pl in enumerate(sched):
        now = t * 100
        recv = pl + calib.gain_db(applied.txagc)
        deliver = link.p_deliver(recv, applied.mcs, applied.overhead)
        deliveries.append(deliver)
        vrx.on_video(rssi=-50 + recv, snr=recv, crc_err=(deliver < 0.5), seq=t, now_ms=now)
        fb = vrx.step(now)
        if fb is not None:
            vtx.on_rc_frame(fb, now)              # VTX applies pwr/fec from RCF
            applied = vrx.cur_op                  # + the chosen MCS (GS-decides)
            pwr_at.append((t, applied.txagc))
        if verbose and t % 20 == 0:
            print(f"t={t} pl={pl:+5.1f} recv={recv:5.1f} MCS{applied.mcs} "
                  f"txagc={applied.txagc:2d} ov={applied.overhead} deliver={deliver:.3f}")
    warm = len(deliveries) // 10                   # ignore cold-start ticks
    avg_deliver = sum(deliveries[warm:]) / len(deliveries[warm:])
    close_txagc = min(p for _, p in pwr_at if 80 <= _ <= 120)   # closest stretch
    far_txagc = max(p for _, p in pwr_at if _ <= 20 or _ >= 180)  # far stretches
    return {"avg_deliver": avg_deliver, "close_txagc": close_txagc,
            "far_txagc": far_txagc, "n_cmds": len(pwr_at)}


# --------------------------------------------------------------------------- #
# Live I/O: drive a real duplex subprocess (exercised on-air by
# tests/adaptive_onair.sh). The classes above hold all the policy; this is plumbing.
# --------------------------------------------------------------------------- #
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tests"))
from devourer_events import parse_event, desc_rate_to_mcs  # noqa: E402


def ctl_frame(op: int, payload: bytes = b"") -> bytes:
    """Encode a stdin control TLV for the duplex binary's SET_* opcodes."""
    body = bytes([op]) + payload
    return struct.pack("<I", 0x80000000 | len(body)) + body


def psdu_frame(body: bytes) -> bytes:
    return struct.pack("<I", len(body)) + body


SET_PWR, SET_RATE, SET_CHAN = 1, 2, 3


def _parse_mcs_bias(spec: str) -> dict[int, float]:
    """'4:-6,5:-8' -> {4: -6.0, 5: -8.0} (per-MCS model snr_req shift, dB)."""
    out = {}
    for part in spec.split(","):
        m, db = part.split(":")
        out[int(m)] = float(db)
    return out


def _now_ms() -> float:
    import time
    return time.monotonic() * 1000.0


def run_vrx(proc, link, calib, vtx_id, channel, feedback_period_ms=100,
            mcs_probe=False, mcs_bias=None):
    """Read the VTX's video frames, score, run the controller, TX feedback."""
    vrx = AdaptiveVrx(link, calib, vtx_id,
                      ctrl_cfg=ControllerConfig(target=0.99, allow_shed=False,
                                                mcs_probe_enabled=mcs_probe),
                      op_channel=channel, feedback_period_ms=feedback_period_ms)
    if mcs_bias:
        from controller import apply_model_bias
        apply_model_bias(vrx.ctrl, mcs_bias)
    import threading

    def reader():
        for line in proc.stdout:
            ev = parse_event(line, ev="rx.frame")
            if ev is None:
                continue
            body = bytes.fromhex(ev.get("body") or "")
            if rp.frame_type(body) == rp.T_DISC_ACK:
                vrx.on_disc_ack(body, _now_ms())
                continue
            snr = max(ev["snr"])
            rssi = max(ev["rssi"])
            # App-level sequence: the VTX prefixes a 12-bit per-video-frame counter,
            # so gap-based loss is EXACT. (The 802.11 seq advances per transmitted
            # frame incl. retries, so it over-counts loss — see score.seq_gap_loss.)
            seq = (int.from_bytes(body[:2], "little") & 0xFFF
                   if len(body) >= 2 else ev["seq"])
            vrx.on_video(rssi, snr, ev["crc"] != 0, seq, _now_ms(),
                         rate=desc_rate_to_mcs(ev.get("rate")))
    threading.Thread(target=reader, daemon=True).start()
    import time
    last_log = 0.0
    while True:
        now = _now_ms()
        fb = vrx.step(now)
        if fb is not None:
            proc.stdin.write(psdu_frame(fb))
            proc.stdin.flush()
        if now - last_log > 1000:                # 1 Hz trajectory log (to stderr)
            last_log = now
            op = vrx.cur_op
            # per-candidate probe evidence: attempts@lcb — a silent line here
            # with probing on means the rate feed is dead, not the probes good
            probes = ""
            if vrx.mcs_win is not None:
                st = vrx.mcs_win.stats(now)
                probes = " probe[" + ",".join(
                    f"MCS{m}:{s.attempts}@{s.lcb:.2f}"
                    for m, s in sorted(st.items())) + "]"
            sys.stderr.write(
                f"<adaptive-vrx>state={vrx.rz.state} snr={vrx.win.snr_estimate()} "
                f"score={vrx.win.score()} deliv={1.0 - vrx.win.seq_gap_loss():.3f} "
                f"-> MCS{op.mcs} ov{op.overhead} "
                f"txagc{op.txagc} frames={vrx.win.n()}{probes}\n")
            sys.stderr.flush()
        time.sleep(0.01)


def run_vtx(proc, vtx_id, video_path, channel, mcs_probe=False):
    """Stream video; apply inbound RCF/DISC to the live knobs via control ops.

    All stdin writers share one lock, and each logical action goes down the
    pipe as ONE composed write: duplex's TX thread consumes stdin strictly in
    order, so a probe's SET_RATE / PSDU / SET_RATE-restore triple applies
    exactly to its frame — and the reader thread can't interleave a fresher
    commanded rate into the middle of it."""
    vtx = AdaptiveVtx(vtx_id, mcs_probe=mcs_probe)
    import threading
    wlock = threading.Lock()

    def _write(buf: bytes) -> None:
        with wlock:
            proc.stdin.write(buf)
            proc.stdin.flush()

    def _cmd_rate() -> bytes:
        return rate_spec(vtx.state.mode, vtx.state.mcs, vtx.state.bw).encode()

    def reader():
        for line in proc.stdout:
            ev = parse_event(line, ev="rx.frame")
            if ev is None:
                continue
            body = bytes.fromhex(ev.get("body") or "")
            t = rp.frame_type(body)
            if t not in (rp.T_RCF, rp.T_DISC):
                continue
            with wlock:                              # state + pipe change together
                ack = vtx.on_rc_frame(body, _now_ms())
                out = b""
                if ack is not None:                  # DISC_ACK to TX
                    out += psdu_frame(ack)
                if t == rp.T_RCF:                    # apply the new operating point
                    out += ctl_frame(SET_PWR, bytes([vtx.state.txagc]))
                    out += ctl_frame(SET_RATE, _cmd_rate())
                if out:
                    proc.stdin.write(out); proc.stdin.flush()
    threading.Thread(target=reader, daemon=True).start()

    import sys
    import time
    vid = open(video_path, "rb") if video_path else None
    app_seq = 0                                  # 12-bit per-video-frame counter
    last_log = 0.0
    cur_chan = channel
    prev_state = vtx.rz.state
    while True:
        now = _now_ms()
        act = vtx.step(now)
        # rendezvous channel switching: hop to the discovery channel on entering
        # DISCOVERY; hop to the agreed op channel once rendezvous completes.
        if vtx.rz.state != prev_state:
            want = (vtx.rz.cfg.discovery_channel if vtx.rz.state == rz.DISCOVERY
                    else vtx.rz.op_channel if vtx.rz.state == rz.RC_OK else cur_chan)
            if want != cur_chan:
                _write(ctl_frame(SET_CHAN, bytes([want, 0, 0])))  # 20 MHz
                cur_chan = want
            prev_state = vtx.rz.state
        if act == rz.A_FAILSAFE:
            _write(ctl_frame(SET_PWR, bytes([vtx.state.txagc])))
        if vid is not None and act in (rz.A_TX_VIDEO, rz.A_FAILSAFE):
            chunk = vid.read(1022)               # room for the 2-byte app-seq header
            if not chunk:
                vid.seek(0); chunk = vid.read(1022)
            psdu = psdu_frame(app_seq.to_bytes(2, "little") + chunk)
            with wlock:
                # The synthetic stream has no protected frame classes; an
                # embedding app passes protected=True for control/IDR bodies.
                cand = vtx.probe_mcs_for_seq(app_seq)
                if cand is not None:
                    probe = rate_spec(vtx.state.mode, cand, vtx.state.bw)
                    buf = (ctl_frame(SET_RATE, probe.encode()) + psdu
                           + ctl_frame(SET_RATE, _cmd_rate()))
                else:
                    buf = psdu
                proc.stdin.write(buf)
                proc.stdin.flush()
            app_seq = (app_seq + 1) & 0xFFF
        if now - last_log > 1000:
            last_log = now
            probes = (f" probes={dict(sorted(vtx.probe_counters.items()))}"
                      if vtx.mcs_probe else "")
            sys.stderr.write(f"<adaptive-vtx>state={vtx.rz.state} txagc={vtx.state.txagc} "
                             f"ov={vtx.state.overhead} ladder={vtx.state.ladder} "
                             f"failsafe={vtx.state.failsafe}{probes}\n")
            sys.stderr.flush()
        time.sleep(0.002)


def main():
    import argparse
    import os
    import subprocess
    import energy_model as em
    import link_model as lm

    ap = argparse.ArgumentParser(description="adaptive-link orchestrator")
    ap.add_argument("--role", choices=["vtx", "vrx", "selftest"], required=True)
    ap.add_argument("--pid", default="0x8812")
    ap.add_argument("--channel", type=int, default=6)
    ap.add_argument("--vtx-id", type=lambda x: int(x, 0), default=0xABCD)
    ap.add_argument("--video", help="VTX video source file (length-agnostic bytes)")
    ap.add_argument("--duplex", default=os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "..", "build", "duplex")))
    ap.add_argument("--link-calib"); ap.add_argument("--energy-calib")
    ap.add_argument("--feedback-ms", type=int, default=100,
                    help="VRX RCF feedback period (ms). Higher = fewer half-duplex "
                         "RX-blind windows on the ground (diagnostic knob)")
    ap.add_argument("--mcs-probe", action="store_true",
                    help="adjacent-MCS probing: the VTX flies the scheduled "
                         "probe seqs on the neighbour rates, the VRX gates "
                         "promotions on the measured candidate delivery. Must "
                         "be set on BOTH roles (schedule = protocol invariant)")
    ap.add_argument("--mcs-bias", type=_parse_mcs_bias, default=None,
                    metavar="M:DB,...",
                    help="VRX-only: shift the model's per-MCS snr_req (e.g. "
                         "'4:-6,5:-8'; negative = model optimistic) — the "
                         "deliberate miscalibration the probe A/B measures "
                         "against")
    a = ap.parse_args()

    if a.role == "selftest":
        import json
        print(json.dumps(selftest(verbose=True), indent=2))
        return

    env = dict(os.environ, DEVOURER_PID=a.pid, DEVOURER_CHANNEL=str(a.channel),
               DEVOURER_RX_KEEP_CORRUPTED="1")
    proc = subprocess.Popen([a.duplex], env=env, stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, bufsize=0)
    link = lm.LinkModel(calib_path=a.link_calib)
    calib = em.load_calibration(a.energy_calib)
    if a.role == "vrx":
        run_vrx(proc, link, calib, a.vtx_id, a.channel,
                feedback_period_ms=a.feedback_ms,
                mcs_probe=a.mcs_probe, mcs_bias=a.mcs_bias)
    else:
        run_vtx(proc, a.vtx_id, a.video, a.channel, mcs_probe=a.mcs_probe)


if __name__ == "__main__":
    main()
