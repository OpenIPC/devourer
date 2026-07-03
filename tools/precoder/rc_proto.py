"""Adaptive-link control protocol — VRX->VTX feedback + rendezvous frames.

Three tiny control frames ride the same radio as the video (canonical SA, probe-
req header), distinguished from video bodies by a leading "RC" magic so the peer
routes them without SBI involvement (feedback is the REVERSE direction from the
SBI-framed video):

  RCF      — VRX -> VTX feedback, ~100 ms: the GS-authoritative profile + the
             alink-style 1000..2000 score + explicit power/FEC + per-layer
             delivery stats. CRC16 so a corrupted command is DROPPED not applied.
  DISC     — VRX -> VTX discovery beacon (rendezvous), addressed to a VTX_ID.
  DISC_ACK — VTX -> VRX reply that completes rendezvous + agrees the op channel.

Design = OpenIPC alink, dual objective: GS decides (AUTH=0, explicit PROFILE);
1 advisory bit reserved for a future drone-decides mode (AUTH=1, metrics only).
Pure + numpy-free; unit-tested in test_rc_proto.py.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

import fec_subblock  # reuse crc16_ccitt

RC_MAGIC = 0x5243           # "RC"
RC_VERSION = 1

# frame types
T_RCF = 1
T_DISC = 2
T_DISC_ACK = 3

# FLAGS bits
F_AUTH_ADVISORY = 0x01      # 0 = GS-decides (explicit PROFILE); 1 = drone-decides
F_FAILSAFE = 0x02           # VRX is in failsafe / lost the VTX
F_DISCOVERY = 0x04          # discovery context

PWR_NO_CHANGE = 0xFF        # PWR_IDX sentinel: leave TX power as-is


# --------------------------------------------------------------------------- #
# PROFILE byte encoding v2 — bandwidth-dimension aware.
# bits[3:0] = MCS (0..8), bits[5:4] = bw code (0=20, 1=40, 2=80), bit6 = VHT.
# Legacy v1 values 0..7 decode as HT/20, so a v1 receiver (which clamps the
# byte to 0..7) still gets a sane MCS from a v2 sender.
# --------------------------------------------------------------------------- #
_BW_CODE = {20: 0, 40: 1, 80: 2}
_BW_FROM_CODE = {v: k for k, v in _BW_CODE.items()}


def encode_profile(mode: str, mcs: int, bw: int = 20) -> int:
    """(mode, mcs, bw) -> wire PROFILE byte."""
    p = (mcs & 0x0F) | (_BW_CODE.get(bw, 0) << 4)
    if mode == "vht":
        p |= 0x40
    return p


def decode_profile(p: int) -> tuple[str, int, int]:
    """Wire PROFILE byte -> (mode, mcs, bw)."""
    mode = "vht" if p & 0x40 else "ht"
    bw = _BW_FROM_CODE.get((p >> 4) & 0x3, 20)
    mcs = p & 0x0F
    return mode, min(mcs, 8 if mode == "vht" else 7), bw


# --------------------------------------------------------------------------- #
# Bandwidth probe schedule — a PROTOCOL INVARIANT both ends derive from the
# video seq alone, so per-rung delivery sensing needs no extra wire fields:
# the VTX flies the scheduled seqs on the scheduled rung, the VRX attributes
# every (received or gap-inferred-lost) seq to its rung by the same rule.
# Slots 0/8/16 of each 32-seq cycle probe the sorted rungs of the (config-
# agreed) bw_set; every other seq rides the commanded operating bandwidth.
# ~3% duty per rung — cheap enough to leave always on.
# --------------------------------------------------------------------------- #
PROBE_PERIOD = 32
_PROBE_SLOTS = (0, 8, 16)


def probe_bw(seq: int, bw_set) -> int | None:
    """Bandwidth this video seq must fly at as a rung probe, else None."""
    rungs = sorted(bw_set)
    slot = seq % PROBE_PERIOD
    for i, s in enumerate(_PROBE_SLOTS):
        if slot == s and i < len(rungs):
            return rungs[i]
    return None


def _crc(buf: bytes) -> int:
    return fec_subblock.crc16_ccitt(buf)


# --------------------------------------------------------------------------- #
# Shared profile table (indices on the wire; both ends agree on a version).
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Profile:
    svc_ladder: str        # DEVOURER_SVC_LADDER-style spec (per-layer MCS)
    pwr_idx: int           # TXAGC 0..63
    fec_overhead: float    # outer-code overhead
    bw: int                # 20 | 40


PROFILE_TABLE_VERSION = 1
# Energy-ranked, max-range (index 0 = failsafe) -> max-quality. The live
# controller computes the actual operating point; this coarse table is what the
# wire PROFILE index and the discovery INIT_PROFILE reference.
DEFAULT_PROFILE_TABLE = [
    Profile("CRIT=MCS0/20/LDPC;T0=MCS0/20/LDPC;T1=MCS0/20;T2=MCS0/20", 63, 1.00, 20),  # 0 max range
    Profile("CRIT=MCS0/20/LDPC;T0=MCS1/20;T1=MCS2/20;T2=MCS2/20", 48, 0.75, 20),       # 1
    Profile("CRIT=MCS1/20/LDPC;T0=MCS2/20;T1=MCS4/20;T2=MCS4/20", 32, 0.50, 20),       # 2
    Profile("CRIT=MCS2/20;T0=MCS4/20;T1=MCS5/20;T2=MCS7/20/SGI", 20, 0.25, 20),        # 3
    Profile("CRIT=MCS4/20;T0=MCS5/20;T1=MCS7/20;T2=MCS7/40/SGI", 8, 0.10, 20),         # 4 max quality
]
MAX_RANGE_PROFILE = 0


# --------------------------------------------------------------------------- #
# RCF — feedback / command
# --------------------------------------------------------------------------- #
@dataclass
class Rcf:
    vtx_id: int = 0
    seq: int = 0
    ack_seq: int = 0           # highest VTX video seq the VRX decoded (round-trip + liveness)
    profile: int = 0
    score: int = 1000          # alink-compatible 1000..2000
    pwr_idx: int = PWR_NO_CHANGE
    fec_overhead_16ths: int = 4   # overhead in 1/16ths (4 = 0.25)
    flags: int = 0
    layer_delivery: tuple = ()    # per-layer delivery %, 0..100

    @property
    def fec_overhead(self) -> float:
        return self.fec_overhead_16ths / 16.0


_RCF_HEAD = "<HBBBIHHBHBBB"   # magic,ver,type,flags,vtx_id,seq,ack_seq,profile,score,pwr,fec16,n_layers


def pack_rcf(r: Rcf) -> bytes:
    layers = bytes(min(100, max(0, int(x))) for x in r.layer_delivery)
    head = struct.pack(_RCF_HEAD, RC_MAGIC, RC_VERSION, T_RCF, r.flags & 0xFF,
                       r.vtx_id & 0xFFFFFFFF, r.seq & 0xFFFF, r.ack_seq & 0xFFFF,
                       r.profile & 0xFF, r.score & 0xFFFF, r.pwr_idx & 0xFF,
                       r.fec_overhead_16ths & 0xFF, len(layers))
    body = head + layers
    return body + struct.pack("<H", _crc(body))


def parse_rcf(buf: bytes) -> Rcf | None:
    """Return Rcf, or None if not a valid RCF (drop — never misapply)."""
    n = struct.calcsize(_RCF_HEAD)
    if len(buf) < n + 2:
        return None
    magic, ver, typ, flags, vtx, seq, ack, prof, score, pwr, fec16, nl = \
        struct.unpack_from(_RCF_HEAD, buf)
    if magic != RC_MAGIC or ver != RC_VERSION or typ != T_RCF:
        return None
    if len(buf) < n + nl + 2:
        return None
    body = buf[:n + nl]
    crc, = struct.unpack_from("<H", buf, n + nl)
    if crc != _crc(body):
        return None
    return Rcf(vtx_id=vtx, seq=seq, ack_seq=ack, profile=prof, score=score,
              pwr_idx=pwr, fec_overhead_16ths=fec16, flags=flags,
              layer_delivery=tuple(buf[n:n + nl]))


# --------------------------------------------------------------------------- #
# DISC / DISC_ACK — rendezvous
# --------------------------------------------------------------------------- #
@dataclass
class Disc:
    vtx_id: int
    vrx_nonce: int
    op_channel: int
    op_width: int = 20
    table_ver: int = PROFILE_TABLE_VERSION
    init_profile: int = MAX_RANGE_PROFILE
    cap_bits: int = 0
    seq: int = 0


_DISC = "<HBBBIIBBBBHH"  # magic,ver,type,flags,vtx_id,nonce,chan,width,tblver,initprof,caps,seq


def pack_disc(d: Disc) -> bytes:
    body = struct.pack(_DISC, RC_MAGIC, RC_VERSION, T_DISC, F_DISCOVERY,
                       d.vtx_id & 0xFFFFFFFF, d.vrx_nonce & 0xFFFFFFFF,
                       d.op_channel & 0xFF, d.op_width & 0xFF, d.table_ver & 0xFF,
                       d.init_profile & 0xFF, d.cap_bits & 0xFFFF, d.seq & 0xFFFF)
    return body + struct.pack("<H", _crc(body))


def parse_disc(buf: bytes) -> Disc | None:
    n = struct.calcsize(_DISC)
    if len(buf) < n + 2:
        return None
    vals = struct.unpack_from(_DISC, buf)
    if vals[0] != RC_MAGIC or vals[1] != RC_VERSION or vals[2] != T_DISC:
        return None
    crc, = struct.unpack_from("<H", buf, n)
    if crc != _crc(buf[:n]):
        return None
    return Disc(vtx_id=vals[4], vrx_nonce=vals[5], op_channel=vals[6],
                op_width=vals[7], table_ver=vals[8], init_profile=vals[9],
                cap_bits=vals[10], seq=vals[11])


@dataclass
class DiscAck:
    vtx_id: int
    vrx_nonce: int             # echo of the DISC nonce
    chip_caps: int
    agreed_channel: int
    agreed_width: int = 20
    seq: int = 0


_DACK = "<HBBBIIHBBH"  # magic,ver,type,flags,vtx_id,nonce,caps,chan,width,seq


def pack_disc_ack(a: DiscAck) -> bytes:
    body = struct.pack(_DACK, RC_MAGIC, RC_VERSION, T_DISC_ACK, F_DISCOVERY,
                       a.vtx_id & 0xFFFFFFFF, a.vrx_nonce & 0xFFFFFFFF,
                       a.chip_caps & 0xFFFF, a.agreed_channel & 0xFF,
                       a.agreed_width & 0xFF, a.seq & 0xFFFF)
    return body + struct.pack("<H", _crc(body))


def parse_disc_ack(buf: bytes) -> DiscAck | None:
    n = struct.calcsize(_DACK)
    if len(buf) < n + 2:
        return None
    vals = struct.unpack_from(_DACK, buf)
    if vals[0] != RC_MAGIC or vals[1] != RC_VERSION or vals[2] != T_DISC_ACK:
        return None
    crc, = struct.unpack_from("<H", buf, n)
    if crc != _crc(buf[:n]):
        return None
    return DiscAck(vtx_id=vals[4], vrx_nonce=vals[5], chip_caps=vals[6],
                   agreed_channel=vals[7], agreed_width=vals[8], seq=vals[9])


def frame_type(buf: bytes) -> int | None:
    """Peek the RC frame type without full parse (None if not an RC frame)."""
    if len(buf) < 4:
        return None
    magic, ver, typ = struct.unpack_from("<HBB", buf)
    if magic != RC_MAGIC or ver != RC_VERSION:
        return None
    return typ
