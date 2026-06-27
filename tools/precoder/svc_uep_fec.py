"""Per-SVC-layer FEC-rate UEP — the application-FEC half of cross-layer UEP.

`txdemo/svc_tx_demo/svc_tx.h` already maps each HEVC temporal layer to a PHY
MCS (the PHY-rate half of unequal error protection). This adds the *outer-FEC-
rate* half: each temporal layer gets its own Reed-Solomon redundancy, so the
base/IDR layers — which `svc_tx.h` already flies at the most robust MCS — also
carry the heaviest erasure protection, and the enhancement layers carry the
least. That is the joint MCS+FEC UEP of Abdel-Khalek & Heath (JSAC 2012):
protection tracks perceptual importance on BOTH knobs at once, giving a
graceful-degradation staircase instead of one cliff.

Each layer is an independent FEC stream tagged with an SBI `stream_id`, so
losing all of T2 never stalls T0's decode, and a corrupted frame's surviving
sub-blocks (via `fec_subblock`) still feed the correct layer's decoder. The
receiver routes a body by the stream_id in its SBI header and unpacks with
that layer's *configured* envelope size (never the corruptible header field),
so a mangled stream_id merely drops the body rather than mis-decoding it.

This is the application-FEC sibling of the C++ `svc::LayerPolicy`; the two are
meant to be configured together (robust MCS + heavy FEC on the same layers).
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import fec_subblock  # noqa: E402
import stream_fec  # noqa: E402
from stream_fec import FecConfig  # noqa: E402


# --------------------------------------------------------------------------- #
# HEVC NAL classification — Python mirror of svc_tx.h:parse_hevc_nal
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class NalInfo:
    tid: int = 0
    critical: bool = False
    type: int = 0


def parse_hevc_nal(nal: bytes) -> NalInfo:
    """HEVC 2-byte NAL header: type=(b0>>1)&0x3F, tid=(b1&7)-1. Critical =
    IRAP slices (16..23) or parameter sets (VPS=32/SPS=33/PPS=34)."""
    if len(nal) < 2:
        return NalInfo()
    ntype = (nal[0] >> 1) & 0x3F
    tid = (nal[1] & 0x07) - 1
    if tid < 0:
        tid = 0
    critical = (16 <= ntype <= 23) or (32 <= ntype <= 34)
    return NalInfo(tid=tid, critical=critical, type=ntype)


# --------------------------------------------------------------------------- #
# Per-layer policy
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class UepLayer:
    fec: FecConfig
    blocks_per_body: int = 4


def _rs(k: int, overhead: float, symbol_size: int = 64) -> FecConfig:
    return FecConfig(k=k, symbol_size=symbol_size, overhead=overhead, scheme="rs")


@dataclass
class UepPolicy:
    """stream_id 0 = critical; 1.. = temporal layers T0, T1, … in order.
    `by_tid[i]` is the layer for temporal id i (clamped to the last entry)."""
    critical: UepLayer
    by_tid: list[UepLayer]

    def stream_for(self, info: NalInfo) -> int:
        if info.critical or not self.by_tid:
            return 0
        return 1 + min(info.tid, len(self.by_tid) - 1)

    def layer(self, stream_id: int) -> UepLayer:
        if stream_id == 0:
            return self.critical
        return self.by_tid[min(stream_id - 1, len(self.by_tid) - 1)]

    def stream_ids(self) -> list[int]:
        return [0] + [1 + i for i in range(len(self.by_tid))]


def default_uep_policy() -> UepPolicy:
    """FEC-rate ladder complementing svc_tx.h's default MCS ladder. Heaviest
    erasure protection on the layers that also fly at the most robust MCS.
        critical (IDR / VPS/SPS/PPS) : RS overhead 1.00  (N=16, tolerates 8/16 loss)
        T0 base                      : RS overhead 0.75  (N=14)
        T1                           : RS overhead 0.50  (N=12)
        T2                           : RS overhead 0.25  (N=10) — lightest
    """
    return UepPolicy(
        critical=UepLayer(_rs(8, 1.00)),
        by_tid=[
            UepLayer(_rs(8, 0.75)),
            UepLayer(_rs(8, 0.50)),
            UepLayer(_rs(8, 0.25)),
        ],
    )


def _env_size(cfg: FecConfig) -> int:
    """Envelope (outer-code symbol) size for a config — derived by a throwaway
    encode so we never hard-code a scheme's header length."""
    enc = stream_fec.make_encoder(cfg)
    envs: list[bytes] = []
    for i in range(cfg.k):
        envs += enc.add_packet(b"\x00" * min(8, cfg.max_packet_size))
    envs += enc.flush()
    return len(envs[0])


# --------------------------------------------------------------------------- #
# Encoder / decoder
# --------------------------------------------------------------------------- #
class SvcUepEncoder:
    """Routes each NAL to its temporal layer's FEC stream and SBI-packs it.
    `add_nal` returns a list of (stream_id, body) ready for the radio."""

    def __init__(self, policy: UepPolicy) -> None:
        self.policy = policy
        self._enc = {sid: stream_fec.make_encoder(policy.layer(sid).fec)
                     for sid in policy.stream_ids()}
        self._env = {sid: _env_size(policy.layer(sid).fec)
                     for sid in policy.stream_ids()}
        self._packer = {
            sid: fec_subblock.SubBlockPacker(
                self._env[sid], policy.layer(sid).blocks_per_body, stream_id=sid)
            for sid in policy.stream_ids()}

    def add_nal(self, nal: bytes) -> list[tuple[int, bytes]]:
        sid = self.policy.stream_for(parse_hevc_nal(nal))
        out: list[tuple[int, bytes]] = []
        for env in self._enc[sid].add_packet(nal):
            for body in self._packer[sid].add(env):
                out.append((sid, body))
        return out

    def flush(self) -> list[tuple[int, bytes]]:
        out: list[tuple[int, bytes]] = []
        for sid in self.policy.stream_ids():
            for env in self._enc[sid].flush():
                for body in self._packer[sid].add(env):
                    out.append((sid, body))
            for body in self._packer[sid].flush():
                out.append((sid, body))
        return out


class SvcUepDecoder:
    """Routes a received body by its SBI stream_id to that layer's FEC decoder,
    unpacking with the layer's *configured* envelope size."""

    def __init__(self, policy: UepPolicy) -> None:
        self.policy = policy
        self._dec = {sid: stream_fec.make_decoder(policy.layer(sid).fec)
                     for sid in policy.stream_ids()}
        self._env = {sid: _env_size(policy.layer(sid).fec)
                     for sid in policy.stream_ids()}
        self.bodies_routed = 0
        self.bodies_misrouted = 0

    def add_body(self, body: bytes) -> list[tuple[int, bytes]]:
        sid = fec_subblock.peek_stream_id(body)
        if sid is None or sid not in self._dec:
            self.bodies_misrouted += 1
            return []
        self.bodies_routed += 1
        res = fec_subblock.unpack(body, self._env[sid])
        out: list[tuple[int, bytes]] = []
        for env in res.survivors:
            for pkt in self._dec[sid].add_symbol(env):
                out.append((sid, pkt))
        return out

    def blocks_decoded(self, stream_id: int) -> int:
        return self._dec[stream_id].blocks_decoded
