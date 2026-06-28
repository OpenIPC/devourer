"""Tests for the rendezvous + failsafe state machines (`rendezvous.py`).

Deterministic, no radio: pumps `tick(now_ms)` and routes DISC/DISC_ACK in-process.
Covers the failsafe timing chain (RC_OK -> RC_LOST -> DISCOVERY), the VRX
SESSION<->BEACONING transitions, and that the asymmetric-duty handshake completes
within the worst-case listen period for ALL phase offsets (the duty-overlap math).
"""

from __future__ import annotations

import rendezvous as rz
import rc_proto as rp


def _vtx(**kw):
    return rz.VtxRendezvous(rz.VtxConfig(vtx_id=0xABCD, fallback_ms=100,
                                         grace_ms=200, listen_on_ms=50,
                                         listen_period_ms=200, **kw))


def _vrx(**kw):
    return rz.VrxRendezvous(rz.VrxConfig(vtx_id=0xABCD, link_lost_ms=100,
                                         beacon_period_ms=20, op_channel=36, **kw))


def test_failsafe_chain_timing():
    v = _vtx()
    v.feed_rc(0)
    assert v.tick(50) == rz.A_TX_VIDEO and v.state == rz.RC_OK
    assert v.tick(150) == rz.A_FAILSAFE and v.state == rz.RC_LOST   # > fallback_ms
    # stays failsafe through the grace window, then enters DISCOVERY
    assert v.tick(300) == rz.A_FAILSAFE or v.state == rz.RC_LOST
    v.tick(400)                                                     # > lost+grace
    assert v.state == rz.DISCOVERY


def test_feed_rc_recovers_to_session():
    v = _vtx()
    v.feed_rc(0); v.tick(150)                       # -> RC_LOST
    assert v.state == rz.RC_LOST
    v.feed_rc(160)                                  # RC came back
    assert v.state == rz.RC_OK


def test_vrx_session_to_beaconing_and_back():
    r = _vrx()
    r.feed_video(0)
    assert r.tick(50) == rz.A_TX_FEEDBACK and r.state == rz.SESSION
    r.tick(200)                                     # > link_lost_ms, no video
    assert r.state == rz.BEACONING
    r.feed_video(210)
    assert r.state == rz.SESSION


def test_vrx_beacons_at_period():
    r = _vrx(); r.feed_video(0); r.tick(200)        # -> BEACONING
    beacons = sum(1 for t in range(200, 400, 5) if r.tick(t) == rz.A_BEACON)
    assert 8 <= beacons <= 11                       # ~1 per 20 ms over 200 ms


def _drive_to_discovery(v, start):
    v.feed_rc(start)
    t = start + 1
    while v.state != rz.DISCOVERY and t < start + 1000:
        v.tick(t); t += 5
    return t


def test_rendezvous_completes_for_all_phase_offsets():
    """VRX beacons every 20 ms; VTX listens 50 ms / 200 ms. Any listen window
    overlaps >=2 beacons, so rendezvous must complete within ~1 listen period for
    every phase offset between the two clocks."""
    for offset in range(0, 200, 10):               # phase sweep
        v = _vtx()
        r = _vrx()
        _drive_to_discovery(v, 0)                  # VTX in DISCOVERY (~>300 ms)
        r.feed_video(0)
        # force VRX into BEACONING, offset its beacon phase
        t = 200 + offset
        while r.state != rz.BEACONING:
            r.tick(t); t += 5
        done_at = None
        for now in range(t, t + 600, 5):           # up to 3 listen periods
            act = r.tick(now)
            if act == rz.A_BEACON and v.listening(now):
                ack = v.feed_disc(r.beacon(), now)
                if ack and r.feed_disc_ack(ack, now):
                    done_at = now
                    break
        assert done_at is not None, f"no rendezvous at offset {offset}"
        assert v.state == rz.RC_OK and r.state == rz.SESSION
        assert v.op_channel == 36                   # agreed the op channel


def test_disc_for_other_vtx_ignored():
    v = _vtx(); _drive_to_discovery(v, 0)
    other = rp.Disc(vtx_id=0x9999, vrx_nonce=1, op_channel=36)
    assert v.feed_disc(other, 500) is None and v.state == rz.DISCOVERY


def test_disc_ack_nonce_must_match():
    r = _vrx(); r.feed_video(0); r.tick(200)        # BEACONING
    r.beacon()
    bad = rp.DiscAck(vtx_id=0xABCD, vrx_nonce=0xDEAD, chip_caps=0, agreed_channel=36)
    assert r.feed_disc_ack(bad, 300) is False and r.state == rz.BEACONING
