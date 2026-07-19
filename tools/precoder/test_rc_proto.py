"""Tests for the adaptive-link control protocol (`rc_proto.py`).

Round-trip every frame; CRC16 rejects bit-flips and truncation; a non-RC body is
not mistaken for a control frame. A corrupted command must be DROPPED (parse
-> None), never misapplied.
"""

from __future__ import annotations

import rc_proto as rp


def test_rcf_roundtrip():
    r = rp.Rcf(vtx_id=0xDEADBEEF, seq=1234, ack_seq=1200, profile=3, score=1750,
               pwr_idx=24, fec_overhead_16ths=4, flags=rp.F_AUTH_ADVISORY,
               layer_delivery=(100, 98, 80, 40))
    out = rp.parse_rcf(rp.pack_rcf(r))
    assert out is not None
    assert out.vtx_id == 0xDEADBEEF and out.seq == 1234 and out.ack_seq == 1200
    assert out.profile == 3 and out.score == 1750 and out.pwr_idx == 24
    assert out.fec_overhead_16ths == 4 and abs(out.fec_overhead - 0.25) < 1e-9
    assert out.flags == rp.F_AUTH_ADVISORY
    assert out.layer_delivery == (100, 98, 80, 40)


def test_rcf_crc_rejects_bitflip():
    buf = bytearray(rp.pack_rcf(rp.Rcf(vtx_id=1, profile=2)))
    buf[8] ^= 0x01                      # flip a payload bit
    assert rp.parse_rcf(bytes(buf)) is None


def test_rcf_rejects_truncation_and_garbage():
    buf = rp.pack_rcf(rp.Rcf(vtx_id=1))
    assert rp.parse_rcf(buf[:5]) is None
    assert rp.parse_rcf(b"\x00" * len(buf)) is None       # wrong magic
    assert rp.parse_rcf(b"video-body-not-rc") is None


def test_disc_roundtrip_and_crc():
    d = rp.Disc(vtx_id=0x1234, vrx_nonce=0x99887766, op_channel=36, op_width=40,
                init_profile=0, cap_bits=0x000F, seq=7)
    out = rp.parse_disc(rp.pack_disc(d))
    assert out and out.vtx_id == 0x1234 and out.vrx_nonce == 0x99887766
    assert out.op_channel == 36 and out.op_width == 40 and out.cap_bits == 0x000F
    bad = bytearray(rp.pack_disc(d)); bad[10] ^= 0xFF
    assert rp.parse_disc(bytes(bad)) is None


def test_disc_ack_roundtrip():
    a = rp.DiscAck(vtx_id=0x1234, vrx_nonce=0x99887766, chip_caps=0x0007,
                   agreed_channel=36, agreed_width=40, seq=8)
    out = rp.parse_disc_ack(rp.pack_disc_ack(a))
    assert out and out.vtx_id == 0x1234 and out.vrx_nonce == 0x99887766
    assert out.agreed_channel == 36 and out.chip_caps == 0x0007


def test_frame_type_discriminates():
    assert rp.frame_type(rp.pack_rcf(rp.Rcf())) == rp.T_RCF
    assert rp.frame_type(rp.pack_disc(rp.Disc(1, 2, 6))) == rp.T_DISC
    assert rp.frame_type(rp.pack_disc_ack(rp.DiscAck(1, 2, 0, 6))) == rp.T_DISC_ACK
    assert rp.frame_type(b"\xf5\xb0video") is None        # an SBI video body, not RC


def test_cross_parse_rejected():
    """A DISC must not parse as an RCF (type byte guards it)."""
    disc = rp.pack_disc(rp.Disc(vtx_id=1, vrx_nonce=2, op_channel=6))
    assert rp.parse_rcf(disc) is None
    assert rp.parse_disc_ack(disc) is None


def test_profile_table():
    assert len(rp.DEFAULT_PROFILE_TABLE) >= 2
    assert rp.MAX_RANGE_PROFILE == 0
    # index 0 = most robust (max power, heaviest FEC)
    assert rp.DEFAULT_PROFILE_TABLE[0].pwr_idx >= rp.DEFAULT_PROFILE_TABLE[-1].pwr_idx
    assert rp.DEFAULT_PROFILE_TABLE[0].fec_overhead >= rp.DEFAULT_PROFILE_TABLE[-1].fec_overhead


def test_profile_v2_encoding_roundtrip_and_legacy_compat():
    # v2 roundtrip across the bandwidth ladder
    for mode in ("ht", "vht"):
        for bw in (20, 40, 80):
            for mcs in (0, 3, 7):
                p = rp.encode_profile(mode, mcs, bw)
                assert rp.decode_profile(p) == (mode, mcs, bw)
    # legacy v1 bytes (raw MCS 0..7) decode as HT/20 — wire compatible
    for m in range(8):
        assert rp.decode_profile(m) == ("ht", m, 20)
    # and an HT/20 v2 encoding IS the legacy byte
    assert rp.encode_profile("ht", 5, 20) == 5


def test_probe_schedule_is_shared_and_low_duty():
    bw_set = (20, 40, 80)
    # slots 0/8/16 probe the sorted rungs; everything else rides the op bw
    assert rp.probe_bw(0, bw_set) == 20
    assert rp.probe_bw(8, bw_set) == 40
    assert rp.probe_bw(16, bw_set) == 80
    assert rp.probe_bw(1, bw_set) is None
    assert rp.probe_bw(32, bw_set) == 20          # periodic
    # duty: 3 probes per 32 seqs
    probes = sum(1 for s in range(320) if rp.probe_bw(s, bw_set) is not None)
    assert probes == 30
    # two-rung set only uses two slots
    assert rp.probe_bw(16, (20, 40)) is None


def test_mcs_probe_schedule_adjacent_and_boundaries():
    mcs_set = tuple(range(8))
    # slot 4 = the next rate above, slot 20 = the next below
    assert rp.probe_mcs(4, 4, mcs_set) == 5
    assert rp.probe_mcs(20, 4, mcs_set) == 3
    assert rp.probe_mcs(5, 4, mcs_set) is None          # off-slot
    assert rp.probe_mcs(4 + 64, 4, mcs_set) == 5        # periodic
    # boundaries of the set: nothing adjacent -> None
    assert rp.probe_mcs(4, 7, mcs_set) is None          # no rate above top
    assert rp.probe_mcs(20, 0, mcs_set) is None         # no rate below bottom
    # adjacency is within the SET, not selected±1 (sparse sets probe neighbours)
    sparse = (0, 2, 4, 6, 7)
    assert rp.probe_mcs(4, 4, sparse) == 6
    assert rp.probe_mcs(20, 4, sparse) == 2
    # selected not in the set: no well-defined neighbour -> never probe
    assert rp.probe_mcs(4, 3, sparse) is None
    assert rp.probe_mcs(20, 3, sparse) is None


def test_mcs_probe_schedule_disjoint_duty_and_wrap():
    bw_set = (20, 40, 80)
    mcs_set = tuple(range(8))
    # disjoint from the bandwidth-probe slots over the whole 12-bit seq space:
    # every probe observation has exactly one changed variable
    for s in range(4096):
        assert not (rp.probe_bw(s, bw_set) is not None
                    and rp.probe_mcs(s, 4, mcs_set) is not None)
    # duty: 2 probes (one up, one down) per 64 seqs
    probes = sum(1 for s in range(640) if rp.probe_mcs(s, 4, mcs_set) is not None)
    assert probes == 20
    # the period divides 4096, so the wrap is phase-safe: seq 4 and 4+4096k agree
    assert 4096 % rp.MCS_PROBE_PERIOD == 0
    assert rp.probe_mcs(4, 4, mcs_set) == rp.probe_mcs((4 + 4096) % 4096, 4, mcs_set)
