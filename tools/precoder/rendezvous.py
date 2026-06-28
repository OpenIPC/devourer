"""Session establishment + failsafe state machines (receiver-initiated rendezvous).

The user's design: when the VTX loses the command/feedback link, it doesn't keep
blasting — it falls back to max range, then begins LOW-DUTY LISTENING on a known
discovery channel; the VRX (wall-powered) BEACONS a frame addressed to that
specific VTX_ID on the discovery channel; on overlap they handshake and resume
the adaptive session. Classic asymmetric-duty rendezvous (cheap listener, fast
beaconer) — the listener's energy is what we protect.

Pure logic (no radio, no time source): the orchestrator/test pumps `tick(now_ms)`
and feeds events; each side returns an ACTION the I/O layer performs. This makes
the whole handshake + failsafe deterministically testable without hardware
(test_rendezvous.py).
"""

from __future__ import annotations

from dataclasses import dataclass

import rc_proto as rp

# VTX states
RC_OK = "RC_OK"            # adaptive session: TX video, apply feedback
RC_LOST = "RC_LOST"        # failsafe: max-range, still TX on current channel
DISCOVERY = "DISCOVERY"    # low-duty listen on the discovery channel

# VRX states
SESSION = "SESSION"        # hearing the VTX: command + TX feedback
BEACONING = "BEACONING"    # lost the VTX: beacon DISC on the discovery channel

# actions
A_TX_VIDEO = "tx_video"
A_FAILSAFE = "failsafe"
A_LISTEN = "listen"
A_IDLE = "idle"
A_TX_FEEDBACK = "tx_feedback"
A_BEACON = "beacon"


@dataclass
class VtxConfig:
    vtx_id: int
    fallback_ms: int = 1000          # no RC this long -> RC_LOST (failsafe)
    grace_ms: int = 3000             # RC_LOST this long -> DISCOVERY (assume drifted)
    listen_on_ms: int = 50           # discovery listen window
    listen_period_ms: int = 1000     # discovery listen period (5% duty default)
    listen_backoff_max_ms: int = 5000  # period grows the longer RC stays lost
    discovery_channel: int = 6


class VtxRendezvous:
    def __init__(self, cfg: VtxConfig):
        self.cfg = cfg
        self.state = RC_OK
        self.op_channel = cfg.discovery_channel
        self._last_rc_ms = 0
        self._lost_at_ms = None
        self._disc_at_ms = None

    def feed_rc(self, now_ms: float) -> None:
        """Any valid RC/feedback proves the link is alive."""
        self._last_rc_ms = now_ms
        if self.state != RC_OK:
            self.state = RC_OK
            self._lost_at_ms = self._disc_at_ms = None

    def _listen_period(self, now_ms: float) -> float:
        """Period grows with how long RC has been lost (save the most energy when
        the GS is probably gone), capped."""
        if self._disc_at_ms is None:
            return self.cfg.listen_period_ms
        elapsed = now_ms - self._disc_at_ms
        grow = self.cfg.listen_period_ms + elapsed / 4.0
        return min(self.cfg.listen_backoff_max_ms, grow)

    def listening(self, now_ms: float) -> bool:
        """In DISCOVERY, are we in a listen-on window right now?"""
        if self.state != DISCOVERY or self._disc_at_ms is None:
            return False
        phase = (now_ms - self._disc_at_ms) % self._listen_period(now_ms)
        return phase < self.cfg.listen_on_ms

    def tick(self, now_ms: float) -> str:
        if self.state == RC_OK:
            if now_ms - self._last_rc_ms > self.cfg.fallback_ms:
                self.state = RC_LOST
                self._lost_at_ms = now_ms
                return A_FAILSAFE
            return A_TX_VIDEO
        if self.state == RC_LOST:
            if now_ms - self._lost_at_ms > self.cfg.grace_ms:
                self.state = DISCOVERY
                self._disc_at_ms = now_ms
                return A_LISTEN if self.listening(now_ms) else A_IDLE
            return A_FAILSAFE                       # keep TXing max-range meanwhile
        # DISCOVERY
        return A_LISTEN if self.listening(now_ms) else A_IDLE

    def feed_disc(self, disc: "rp.Disc", now_ms: float) -> "rp.DiscAck | None":
        """A DISC heard while listening. If it targets us, reply DISC_ACK, switch
        to the op channel, and enter RC_OK."""
        if self.state not in (RC_LOST, DISCOVERY):
            return None
        if disc.vtx_id != self.cfg.vtx_id:
            return None                             # not for us — ignore
        self.op_channel = disc.op_channel
        self.state = RC_OK
        self._last_rc_ms = now_ms
        self._lost_at_ms = self._disc_at_ms = None
        return rp.DiscAck(vtx_id=self.cfg.vtx_id, vrx_nonce=disc.vrx_nonce,
                          chip_caps=0, agreed_channel=disc.op_channel,
                          agreed_width=disc.op_width)


@dataclass
class VrxConfig:
    vtx_id: int
    link_lost_ms: int = 1000         # no VTX video this long -> BEACONING
    beacon_period_ms: int = 20       # beacon fast (< VTX listen window) for overlap
    op_channel: int = 6
    discovery_channel: int = 6


class VrxRendezvous:
    def __init__(self, cfg: VrxConfig):
        self.cfg = cfg
        self.state = SESSION
        self._last_video_ms = 0
        self._last_beacon_ms = -1 << 60
        self._seq = 0
        self._nonce = (cfg.vtx_id * 2654435761) & 0xFFFFFFFF

    def feed_video(self, now_ms: float) -> None:
        self._last_video_ms = now_ms
        if self.state == BEACONING:
            self.state = SESSION                    # heard the VTX again

    def tick(self, now_ms: float) -> str:
        if self.state == SESSION:
            if now_ms - self._last_video_ms > self.cfg.link_lost_ms:
                self.state = BEACONING
            else:
                return A_TX_FEEDBACK
        # BEACONING: only the newer "VRX transmits only when it (re)hears the VTX"
        # safety doesn't apply during discovery — we are actively searching.
        if now_ms - self._last_beacon_ms >= self.cfg.beacon_period_ms:
            self._last_beacon_ms = now_ms
            return A_BEACON
        return A_IDLE

    def beacon(self) -> "rp.Disc":
        self._seq = (self._seq + 1) & 0xFFFF
        return rp.Disc(vtx_id=self.cfg.vtx_id, vrx_nonce=self._nonce,
                       op_channel=self.cfg.op_channel, seq=self._seq)

    def feed_disc_ack(self, ack: "rp.DiscAck", now_ms: float) -> bool:
        """VTX answered our beacon. Complete rendezvous -> SESSION."""
        if ack.vtx_id != self.cfg.vtx_id or ack.vrx_nonce != self._nonce:
            return False
        self.state = SESSION
        self._last_video_ms = now_ms               # expect video imminently
        return True
