"""End-to-end SVC-HEVC UEP pipeline tests (`svc_pipeline.py`).

Drives a realistic synthetic HEVC stream (tests/gen_svc_nals.py) through the
full per-layer pipeline — classify -> per-layer FEC -> SBI framing -> per-layer
(MCS,SNR) channel -> SBI salvage -> FEC decode -> fragment reassembly — and
asserts the cross-layer-UEP contract: base/IDR delivered while enhancement
sheds first, a monotone staircase as SNR drops, and SBI sub-block salvage
beating whole-frame erasure.
"""

from __future__ import annotations

import os
import sys

import svc_pipeline as sp
from svc_uep_fec import parse_hevc_nal

sys.path.insert(0, os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "..", "tests")))
import gen_svc_nals  # noqa: E402

CRIT, T0, T1, T2 = 0, 1, 2, 3


def _nals(gops=8):
    return gen_svc_nals.gen_nals(gops=gops)


# --------------------------------------------------------------------------- #
# Synthetic source realism
# --------------------------------------------------------------------------- #
def test_synthetic_stream_layer_mix():
    """gops=8, idr_period=2 -> 4 IDR AUs (4 critical NALs each) + 4:8:16 T0/T1/T2
    per GOP. The byte sizes differ per class so downstream sees a real VBR mix."""
    pol = sp.pipeline_uep_policy()
    counts = {CRIT: 0, T0: 0, T1: 0, T2: 0}
    sizes = {CRIT: set(), T0: set(), T1: set(), T2: set()}
    for nal in _nals(8):
        sid = pol.stream_for(parse_hevc_nal(nal))
        counts[sid] += 1
        sizes[sid].add(len(nal))
    assert counts == {CRIT: 16, T0: 32, T1: 64, T2: 128}        # 4:8:16 + IDR AUs
    # IDR access units are far larger than enhancement frames (realistic VBR)
    assert max(sizes[CRIT]) > max(sizes[T2])


# --------------------------------------------------------------------------- #
# Clean channel — everything arrives
# --------------------------------------------------------------------------- #
def test_clean_channel_delivers_all_layers():
    r = sp.run_svc_pipeline(_nals(8), 40.0, seed=1)
    for sid in (CRIT, T0, T1, T2):
        assert r.delivery(sid) == 1.0


# --------------------------------------------------------------------------- #
# The headline: graceful-degradation staircase
# --------------------------------------------------------------------------- #
def test_uep_staircase_monotone_under_stress():
    r = sp.run_svc_pipeline(_nals(8), 16.0, seed=16)
    d = [r.delivery(s) for s in (CRIT, T0, T1, T2)]
    assert d == sorted(d, reverse=True)          # CRIT >= T0 >= T1 >= T2
    assert d[0] == 1.0                            # base/IDR fully protected
    assert d[0] > d[3]                            # enhancement sacrificed first


def test_base_holds_when_enhancement_collapses():
    # SNR low enough that the top enhancement layer is gone but base rides through
    r = sp.run_svc_pipeline(_nals(8), 10.0, seed=10)
    assert r.delivery(CRIT) >= 0.99              # base/IDR SLA (>=99%)
    assert r.delivery(T2) <= 0.05               # T2 collapsed
    assert r.delivery(CRIT) > r.delivery(T2)


def test_staircase_is_monotone_across_snr_sweep():
    """As SNR drops, no layer's delivery ever increases — the staircase only
    descends (each layer sheds and stays shed)."""
    snrs = [40, 30, 24, 20, 16, 12, 8, 4]
    curves = {s: [] for s in (CRIT, T0, T1, T2)}
    for snr in snrs:
        r = sp.run_svc_pipeline(_nals(8), float(snr), seed=100 + snr)
        for sid in curves:
            curves[sid].append(r.delivery(sid))
    for sid, c in curves.items():
        # allow Monte-Carlo wobble of one sub-block; trend must be non-increasing
        for a, b in zip(c, c[1:]):
            assert b <= a + 0.06, f"layer {sid} rose as SNR dropped: {c}"
    # and base outlasts enhancement: CRIT's area under the curve > T2's
    assert sum(curves[CRIT]) > sum(curves[T2])


# --------------------------------------------------------------------------- #
# Fused-FEC: SBI sub-block salvage beats whole-frame erasure
# --------------------------------------------------------------------------- #
def test_sbi_salvage_beats_whole_frame():
    nals = _nals(8)
    sbi = sp.run_svc_pipeline(nals, 18.0, sbi=True, seed=18)
    wf = sp.run_svc_pipeline(nals, 18.0, sbi=False, seed=18)
    tot_sbi = sum(l.delivered for l in sbi.layers.values())
    tot_wf = sum(l.delivered for l in wf.layers.values())
    assert tot_sbi > tot_wf                       # salvaging partial frames wins
    assert sbi.delivery(T1) > wf.delivery(T1)     # most visible on the marginal layer
    # base is fully protected either way (heavy FEC); salvage helps the margin
    assert sbi.delivery(CRIT) == wf.delivery(CRIT) == 1.0


# --------------------------------------------------------------------------- #
# Fragmentation round-trips real-sized NALs
# --------------------------------------------------------------------------- #
# --------------------------------------------------------------------------- #
# Closed-loop adaptive SVC: SvcController drives per-layer MCS + shedding + power
# --------------------------------------------------------------------------- #
def _sweep(reported_snrs, txagc=32):
    return [(s, sp.run_svc_pipeline_adaptive(_nals(8), float(s), txagc, seed=s & 0x7F))
            for s in reported_snrs]


def test_adaptive_backs_off_power_when_close():
    # strong reported link -> controller spends little power yet delivers all layers
    a = sp.run_svc_pipeline_adaptive(_nals(8), 40.0, 32, seed=1)
    assert a.active_sids == [CRIT, T0, T1, T2]
    assert a.shared_txagc <= 16
    for sid in (CRIT, T0, T1, T2):
        assert a.pipeline.delivery(sid) == 1.0


def test_adaptive_cranks_power_at_range():
    close = sp.run_svc_pipeline_adaptive(_nals(8), 40.0, 32, seed=2)
    far = sp.run_svc_pipeline_adaptive(_nals(8), 6.0, 32, seed=2)
    assert far.shared_txagc >= close.shared_txagc + 25   # much more power at range


def test_adaptive_sheds_enhancement_before_base():
    # a reported SNR where even max power can't carry every layer
    a = sp.run_svc_pipeline_adaptive(_nals(8), 4.0, 32, seed=3)
    assert CRIT in a.active_sids and T0 in a.active_sids   # base never shed
    assert T2 not in a.active_sids                          # enhancement shed first
    assert a.pipeline.delivery(CRIT) >= 0.99               # base SLA still met
    assert a.pipeline.delivery(T2) == 0.0                  # shed -> not delivered


def test_adaptive_base_sla_across_operational_range():
    """Across the operational range the controller holds the base/IDR SLA and
    sheds layers monotonically (the active set only shrinks as range grows)."""
    sweep = _sweep([40, 30, 22, 16, 10, 4])
    prev_active = 99
    for snr, a in sweep:
        assert a.pipeline.delivery(CRIT) >= 0.99, f"base SLA missed at rSNR {snr}"
        assert len(a.active_sids) <= prev_active             # sheds, never re-adds blindly
        prev_active = len(a.active_sids)
    # power rises monotonically as the link weakens (energy tracks range)
    txagcs = [a.shared_txagc for _s, a in sweep]
    assert txagcs == sorted(txagcs)


def test_large_nal_fragments_and_reassembles_intact():
    from svc_uep_fec import SvcUepDecoder, SvcUepEncoder
    pol = sp.pipeline_uep_policy()
    enc = SvcUepEncoder(pol, fragment=True)
    dec = SvcUepDecoder(pol, fragment=True)
    big_idr = gen_svc_nals.nal_bytes(gen_svc_nals.IDR_W_RADL, 0, 1800)
    bodies = enc.add_nal(big_idr) + enc.flush()
    out = []
    for sid, body in bodies:
        out += [nal for _sid, nal in dec.add_body(body)]
    assert big_idr in out                         # reassembled byte-identical
