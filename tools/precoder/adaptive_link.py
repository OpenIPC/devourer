"""Adaptive-link orchestrator — ties the controller, scorer, protocol, and
rendezvous into the VTX and VRX control loops.

This is the policy plane (the mechanical knobs live in the C++ duplex binary). The
two roles:

  AdaptiveVrx (ground): consumes the VTX's video frames (<devourer-stream> RSSI/
    SNR/crc/seq), scores the link, runs the energy-min controller to pick the
    operating point, and emits RCF feedback (or DISC beacons when the VTX is
    lost) ~10 Hz.
  AdaptiveVtx (drone): consumes RCF/DISC, applies the operating point to the live
    knobs (TX power, per-layer MCS ladder, FEC overhead), and runs the failsafe +
    discovery state machine.

The radio I/O (a subprocess StreamDuplexDemo) is abstracted; `selftest()` wires a
VTX and a VRX through a simulated channel (link_model) so the whole closed loop is
deterministically testable without hardware. The CLI (`--role vtx|vrx`) wires the
real duplex binary; that path is exercised on-air by tests/adaptive_onair.sh.
"""

from __future__ import annotations

from dataclasses import dataclass

import rc_proto as rp
import rendezvous as rz
from controller import Controller, ControllerConfig
from score import ScoreWindow, ScoreConfig
import op_table


def op_to_ladder(op) -> str:
    """Map a controller OpPoint to a DEVOURER_SVC_LADDER spec (per-layer MCS).
    Base/critical fly the chosen (robust) MCS; enhancement steps up from there."""
    m = op.mcs
    e = min(7, m + 2)
    bw = op.bw
    return (f"CRIT=MCS{m}/{bw};T0=MCS{m}/{bw};"
            f"T1=MCS{min(7, m + 1)}/{bw};T2=MCS{e}/{bw}")


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
        self.rz = rz.VrxRendezvous(rz.VrxConfig(vtx_id=vtx_id, op_channel=op_channel))
        self.feedback_period_ms = feedback_period_ms
        self._last_fb_ms = -1 << 60
        self._seq = 0
        self.cur_op = op_table.MAX_RANGE
        self._cur_txagc = 32

    def on_video(self, rssi: float, snr: float, crc_err: bool, seq: int,
                 now_ms: float, residual_loss: float | None = None) -> None:
        self.win.add_frame(rssi, snr, crc_err, seq, now_ms / 1000.0)
        self.rz.feed_video(now_ms)
        self._residual = residual_loss

    def on_disc_ack(self, buf: bytes, now_ms: float) -> None:
        ack = rp.parse_disc_ack(buf)
        if ack:
            self.rz.feed_disc_ack(ack, now_ms)

    def step(self, now_ms: float) -> bytes | None:
        """Return a frame to TX back to the VTX (RCF or DISC), or None."""
        act = self.rz.tick(now_ms)
        if act == rz.A_BEACON:
            return rp.pack_disc(self.rz.beacon())
        if act != rz.A_TX_FEEDBACK:
            return None
        if now_ms - self._last_fb_ms < self.feedback_period_ms:
            return None
        self._last_fb_ms = now_ms
        snr = self.win.snr_estimate()
        if snr is not None:
            op = self.ctrl.update(snr, self._cur_txagc, now_ms)
            if op is not None:
                self.cur_op = op
                self._cur_txagc = op.txagc
        self._seq = (self._seq + 1) & 0xFFFF
        # profile carries the GS-chosen base MCS (0..7); the VTX builds its SVC
        # ladder from it (base/critical at this MCS, enhancement steps up).
        rcf = rp.Rcf(vtx_id=self.vtx_id, seq=self._seq, ack_seq=self.win.ack_seq(),
                     profile=self.cur_op.mcs,
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


class AdaptiveVtx:
    def __init__(self, vtx_id: int, vtx_cfg: rz.VtxConfig | None = None,
                 failsafe_txagc: int = 63, failsafe_overhead: float = 1.0,
                 failsafe_ladder: str | None = None):
        self.vtx_id = vtx_id
        self.rz = rz.VtxRendezvous(vtx_cfg or rz.VtxConfig(vtx_id=vtx_id))
        self.state = TxState()
        self.failsafe_txagc = failsafe_txagc
        self.failsafe_overhead = failsafe_overhead
        self.failsafe_ladder = failsafe_ladder or \
            "CRIT=MCS0/20/LDPC;T0=MCS0/20/LDPC;T1=MCS0/20;T2=MCS0/20"

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
            # MCS ladder follows the GS-chosen base MCS in `profile`.
            m = max(0, min(7, r.profile))
            bw = 20
            self.state.ladder = (f"CRIT=MCS{m}/{bw};T0=MCS{m}/{bw};"
                                 f"T1=MCS{min(7, m + 1)}/{bw};T2=MCS{min(7, m + 2)}/{bw}")
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
        if act == rz.A_FAILSAFE:
            self.state = TxState(txagc=self.failsafe_txagc,
                                 overhead=self.failsafe_overhead,
                                 ladder=self.failsafe_ladder, failsafe=True)
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
# Live I/O: drive a real StreamDuplexDemo subprocess (exercised on-air by
# tests/adaptive_onair.sh). The classes above hold all the policy; this is plumbing.
# --------------------------------------------------------------------------- #
import re
import struct

_STREAM_RE = re.compile(
    rb"<devourer-stream>rate=(?P<rate>\d+).*?crc_err=(?P<crc>\d+).*?"
    rb"rssi=(?P<rssi>-?\d+),(?P<rssi2>-?\d+).*?snr=(?P<snr>-?\d+),(?P<snr2>-?\d+).*?"
    rb"seq=(?P<seq>\d+).*?body=(?P<body>[0-9a-fA-F]*)")


def ctl_frame(op: int, payload: bytes = b"") -> bytes:
    """Encode a stdin control TLV for the duplex binary's SET_* opcodes."""
    body = bytes([op]) + payload
    return struct.pack("<I", 0x80000000 | len(body)) + body


def psdu_frame(body: bytes) -> bytes:
    return struct.pack("<I", len(body)) + body


SET_PWR, SET_RATE, SET_CHAN = 1, 2, 3


def _now_ms() -> float:
    import time
    return time.monotonic() * 1000.0


def run_vrx(proc, link, calib, vtx_id, channel, feedback_period_ms=100):
    """Read the VTX's video frames, score, run the controller, TX feedback."""
    vrx = AdaptiveVrx(link, calib, vtx_id,
                      ctrl_cfg=ControllerConfig(target=0.99, allow_shed=False),
                      op_channel=channel, feedback_period_ms=feedback_period_ms)
    import threading

    def reader():
        for line in proc.stdout:
            m = _STREAM_RE.search(line)
            if not m:
                continue
            body = bytes.fromhex(m.group("body").decode())
            if rp.frame_type(body) == rp.T_DISC_ACK:
                vrx.on_disc_ack(body, _now_ms())
                continue
            snr = max(int(m.group("snr")), int(m.group("snr2")))
            rssi = max(int(m.group("rssi")), int(m.group("rssi2")))
            vrx.on_video(rssi, snr, int(m.group("crc")) != 0, int(m.group("seq")),
                         _now_ms())
    threading.Thread(target=reader, daemon=True).start()
    import sys
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
            sys.stderr.write(
                f"<adaptive-vrx>state={vrx.rz.state} snr={vrx.win.snr_estimate()} "
                f"score={vrx.win.score()} -> MCS{op.mcs} ov{op.overhead} "
                f"txagc{op.txagc} frames={vrx.win.n()}\n")
            sys.stderr.flush()
        time.sleep(0.01)


def run_vtx(proc, vtx_id, video_path, channel):
    """Stream video; apply inbound RCF/DISC to the live knobs via control ops."""
    vtx = AdaptiveVtx(vtx_id)
    import threading

    def reader():
        for line in proc.stdout:
            m = _STREAM_RE.search(line)
            if not m:
                continue
            body = bytes.fromhex(m.group("body").decode())
            t = rp.frame_type(body)
            if t not in (rp.T_RCF, rp.T_DISC):
                continue
            ack = vtx.on_rc_frame(body, _now_ms())
            if ack is not None:                      # DISC_ACK to TX
                proc.stdin.write(psdu_frame(ack)); proc.stdin.flush()
            if t == rp.T_RCF:                        # apply the new operating point
                proc.stdin.write(ctl_frame(SET_PWR, bytes([vtx.state.txagc])))
                base = f"MCS{vtx.state.ladder.split('CRIT=MCS')[1].split('/')[0]}"
                proc.stdin.write(ctl_frame(SET_RATE, base.encode()))
                proc.stdin.flush()
    threading.Thread(target=reader, daemon=True).start()

    import sys
    import time
    vid = open(video_path, "rb") if video_path else None
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
                proc.stdin.write(ctl_frame(SET_CHAN, bytes([want, 0, 0])))  # 20 MHz
                proc.stdin.flush()
                cur_chan = want
            prev_state = vtx.rz.state
        if act == rz.A_FAILSAFE:
            proc.stdin.write(ctl_frame(SET_PWR, bytes([vtx.state.txagc])))
            proc.stdin.flush()
        if vid is not None and act in (rz.A_TX_VIDEO, rz.A_FAILSAFE):
            chunk = vid.read(1024)
            if not chunk:
                vid.seek(0); chunk = vid.read(1024)
            proc.stdin.write(psdu_frame(chunk)); proc.stdin.flush()
        if now - last_log > 1000:
            last_log = now
            sys.stderr.write(f"<adaptive-vtx>state={vtx.rz.state} txagc={vtx.state.txagc} "
                             f"ov={vtx.state.overhead} ladder={vtx.state.ladder} "
                             f"failsafe={vtx.state.failsafe}\n")
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
        os.path.dirname(os.path.abspath(__file__)), "..", "..", "build", "StreamDuplexDemo")))
    ap.add_argument("--link-calib"); ap.add_argument("--energy-calib")
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
        run_vrx(proc, link, calib, a.vtx_id, a.channel)
    else:
        run_vtx(proc, a.vtx_id, a.video, a.channel)


if __name__ == "__main__":
    main()
