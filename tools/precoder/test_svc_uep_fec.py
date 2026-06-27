"""Tests for per-SVC-layer FEC-rate UEP (`svc_uep_fec.py`).

The headline is the graceful-degradation staircase: under the SAME channel
loss, the heavily-protected base/critical layers fully recover while the
lightly-protected enhancement layers degrade — protection tracking importance.
"""

from __future__ import annotations

import random

import pytest

import svc_uep_fec
from svc_uep_fec import (NalInfo, SvcUepDecoder, SvcUepEncoder,
                         default_uep_policy, parse_hevc_nal)

IDR = 19      # IRAP -> critical
TRAIL = 1     # trailing picture
SPS = 33      # parameter set -> critical


def _nal(ntype: int, tid: int, payload: int = 60) -> bytes:
    # 62-byte NAL = 2 header + 60 payload -> exactly one 64-byte RS symbol.
    return bytes([(ntype << 1) & 0xFF, (tid + 1) & 0x07]) + bytes(payload)


# --------------------------------------------------------------------------- #
# NAL classification (mirror of svc_tx.h)
# --------------------------------------------------------------------------- #
def test_parse_hevc_nal():
    assert parse_hevc_nal(_nal(IDR, 0)) == NalInfo(tid=0, critical=True, type=IDR)
    assert parse_hevc_nal(_nal(SPS, 0)).critical
    n = parse_hevc_nal(_nal(TRAIL, 2))
    assert n.tid == 2 and not n.critical and n.type == TRAIL
    assert parse_hevc_nal(b"\x00") == NalInfo()  # malformed -> base


def test_stream_routing():
    p = default_uep_policy()
    assert p.stream_for(parse_hevc_nal(_nal(IDR, 2))) == 0   # critical regardless of tid
    assert p.stream_for(parse_hevc_nal(_nal(TRAIL, 0))) == 1
    assert p.stream_for(parse_hevc_nal(_nal(TRAIL, 1))) == 2
    assert p.stream_for(parse_hevc_nal(_nal(TRAIL, 6))) == 3  # tid>layers clamps to last


# --------------------------------------------------------------------------- #
# Clean round-trip
# --------------------------------------------------------------------------- #
def test_clean_round_trip_routes_per_layer():
    enc = SvcUepEncoder(default_uep_policy())
    nals = ([_nal(IDR, 0)] + [_nal(TRAIL, 0)] * 8 + [_nal(TRAIL, 1)] * 8 +
            [_nal(TRAIL, 2)] * 8) * 2
    bodies = []
    for n in nals:
        bodies += enc.add_nal(n)
    bodies += enc.flush()

    dec = SvcUepDecoder(default_uep_policy())
    recovered = {0: set(), 1: set(), 2: set(), 3: set()}
    for sid, body in bodies:
        for rsid, pkt in dec.add_body(body):
            recovered[rsid].add(pkt)
    # Every layer's NALs come back on their own stream, no loss.
    assert recovered[1] == {_nal(TRAIL, 0)}
    assert recovered[2] == {_nal(TRAIL, 1)}
    assert recovered[3] == {_nal(TRAIL, 2)}
    assert _nal(IDR, 0) in recovered[0]
    assert dec.bodies_misrouted == 0


# --------------------------------------------------------------------------- #
# The headline: graceful-degradation staircase under loss
# --------------------------------------------------------------------------- #
def test_uep_staircase_under_body_loss():
    policy = default_uep_policy()
    enc = SvcUepEncoder(policy)
    NB = 20  # RS blocks per layer
    bodies = []
    for _ in range(NB):
        for _ in range(8):
            bodies += enc.add_nal(_nal(IDR, 0))     # critical (sid 0)
        for _ in range(8):
            bodies += enc.add_nal(_nal(TRAIL, 0))   # T0 (sid 1)
        for _ in range(8):
            bodies += enc.add_nal(_nal(TRAIL, 1))   # T1 (sid 2)
        for _ in range(8):
            bodies += enc.add_nal(_nal(TRAIL, 2))   # T2 (sid 3)
    bodies += enc.flush()

    rng = random.Random(7)
    dec = SvcUepDecoder(policy)
    p = 0.30  # whole-body loss probability, same for every layer
    for sid, body in bodies:
        if rng.random() < p:
            continue
        dec.add_body(body)

    crit = dec.blocks_decoded(0)
    t0 = dec.blocks_decoded(1)
    t1 = dec.blocks_decoded(2)
    t2 = dec.blocks_decoded(3)

    # Monotone staircase: heavier FEC -> more blocks survive the same loss.
    assert crit >= t0 >= t1 >= t2
    # Heavy layers ride through; the lightest visibly degrades.
    assert crit >= int(0.8 * NB)
    assert t2 <= int(0.6 * NB)
    assert crit > t2  # the UEP point — base protected, enhancement sacrificed
