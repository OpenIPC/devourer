"""End-to-end SVC-HEVC UEP pipeline simulator (software, no radio).

Stitches the whole cross-layer-UEP path together so it can be exercised in CI
against a *realistic* synthetic HEVC stream:

    synthetic HEVC NALs            (tests/gen_svc_nals.py — 4:8:16 T0/T1/T2 + IDR AUs)
      -> classify + per-layer FEC  (svc_uep_fec.SvcUepEncoder, fragment=True)
      -> SBI sub-block framing      (fec_subblock — one body = several CRC'd symbols)
      -> per-layer PHY channel      (this module — MCS ladder + SNR -> sub-block loss
                                     via link_model.synth_channel, the fec_ab_sim shape)
      -> SBI survivor salvage       (fec_subblock.unpack inside the decoder)
      -> per-layer FEC decode       (SvcUepDecoder, fragment reassembly)
      -> per-NAL delivery per layer

The two halves of unequal error protection are *both* applied per layer:
  - PHY MCS    (DEFAULT_SVC_MCS, mirroring examples/svctx/svc_tx.h default_policy)
  - outer FEC  (default_uep_policy — heavier RS overhead on the robust layers)
so a dropping SNR produces a graceful staircase (T2 sheds first, base/IDR last)
instead of one cliff. `run_svc_pipeline` returns per-layer delivery; `main()`
prints the staircase across an SNR sweep.
"""

from __future__ import annotations

import os
import random
import sys
from dataclasses import dataclass, field

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
_TESTS = os.path.normpath(os.path.join(_HERE, "..", "..", "tests"))
if _TESTS not in sys.path:
    sys.path.insert(0, _TESTS)

import fec_subblock  # noqa: E402
import link_model as lm  # noqa: E402
import svc_uep_fec  # noqa: E402
from stream_fec import FecConfig  # noqa: E402
from svc_uep_fec import (SvcUepDecoder, SvcUepEncoder, UepLayer, UepPolicy,  # noqa: E402
                         parse_hevc_nal)

import gen_svc_nals  # noqa: E402  (tests/)

# PHY-MCS half of UEP — the Python mirror of svc_tx.h default_policy()'s ladder:
#   critical (sid 0) MCS0   T0 (sid 1) MCS1   T1 (sid 2) MCS4   T2 (sid 3) MCS7
# Robust layers fly at the robust MCS *and* carry the heavy FEC overhead.
DEFAULT_SVC_MCS = {0: 0, 1: 1, 2: 4, 3: 7}

LAYER_NAME = {0: "CRIT", 1: "T0", 2: "T1", 3: "T2"}


def pipeline_uep_policy(symbol_size: int = 256) -> UepPolicy:
    """default_uep_policy's FEC-overhead ladder at a larger symbol size, so a
    realistic NAL fragments into a handful of symbols (not dozens) — keeps the
    CI sim fast while preserving the per-layer overhead ratio and SBI sub-block
    granularity."""
    def rs(overhead: float) -> FecConfig:
        return FecConfig(k=8, symbol_size=symbol_size, overhead=overhead, scheme="rs")
    return UepPolicy(
        critical=UepLayer(rs(1.00)),
        by_tid=[UepLayer(rs(0.75)), UepLayer(rs(0.50)), UepLayer(rs(0.25))],
    )


def _corrupt_body(body: bytes, block_payload: int, channel: dict,
                  rng: random.Random) -> bytes:
    """Apply one (MCS,SNR) channel draw to a radio body. With prob corrupt_rate
    the frame fails its FCS; each of its SBI sub-blocks then independently
    survives with the channel's mean surviving fraction (the localized-corruption
    model SBI exploits). A hit sub-block has a payload byte flipped so its CRC
    fails — exactly what `fec_subblock.unpack` erases."""
    if rng.random() >= channel["corrupt_rate"]:
        return body                                  # clean frame
    hist = channel["survivor_hist"]
    n_sub = channel["n_sub"]
    tot = sum(hist.values()) or 1
    f = sum(int(s) * v for s, v in hist.items()) / (n_sub * tot)  # P(sub-block ok)
    b = bytearray(body)
    stride = 2 + block_payload                       # crc16 + payload
    n_blocks = (len(body) - fec_subblock.SBI_HDR_LEN) // stride
    for i in range(n_blocks):
        if rng.random() >= f:                        # this sub-block is hit
            off = fec_subblock.SBI_HDR_LEN + i * stride + 2
            b[off] ^= 0xFF
    return bytes(b)


@dataclass
class LayerStat:
    sid: int
    sent: int = 0          # NALs offered to the encoder
    delivered: int = 0     # NALs recovered intact at the decoder
    bodies: int = 0        # radio bodies for this layer
    bodies_corrupted: int = 0

    @property
    def delivery(self) -> float:
        return self.delivered / self.sent if self.sent else 1.0


@dataclass
class PipelineResult:
    layers: dict[int, LayerStat] = field(default_factory=dict)

    def delivery(self, sid: int) -> float:
        return self.layers[sid].delivery

    def summary(self) -> str:
        return "  ".join(
            f"{LAYER_NAME.get(s, s)}={self.layers[s].delivery:.3f}"
            for s in sorted(self.layers))


def run_svc_pipeline(nals: list[bytes], snr_db: float,
                     policy: UepPolicy | None = None,
                     mcs_ladder: dict[int, int] | None = None,
                     link: lm.LinkModel | None = None,
                     sbi: bool = True, seed: int = 0) -> PipelineResult:
    """Push `nals` through the full per-layer UEP pipeline at a fixed link SNR.

    `sbi=True` salvages a corrupt frame's surviving sub-blocks (fused FEC);
    `sbi=False` models the legacy whole-frame-erasure path (a corrupt frame is
    dropped entirely) for the salvage A/B.
    """
    policy = policy or pipeline_uep_policy()
    mcs_ladder = mcs_ladder or DEFAULT_SVC_MCS
    link = link or lm.LinkModel(trials=1)
    rng = random.Random(seed)

    enc = SvcUepEncoder(policy, fragment=True)
    dec = SvcUepDecoder(policy, fragment=True)
    res = PipelineResult(layers={s: LayerStat(s) for s in policy.stream_ids()})

    bodies: list[tuple[int, bytes]] = []
    for nal in nals:
        sid = policy.stream_for(parse_hevc_nal(nal))
        res.layers[sid].sent += 1
        bodies += enc.add_nal(nal)
    bodies += enc.flush()

    for sid, body in bodies:
        st = res.layers[sid]
        st.bodies += 1
        mcs = mcs_ladder.get(sid, max(mcs_ladder.values()))
        ch = link.channel(mcs, snr_db)
        if sbi:
            cbody = _corrupt_body(body, enc._env[sid], ch, rng)
            if cbody != body:
                st.bodies_corrupted += 1
            for rsid, nal in dec.add_body(cbody):
                res.layers[rsid].delivered += 1
        else:
            # legacy: a frame that fails the FCS is a whole-body erasure
            if rng.random() < ch["corrupt_rate"]:
                st.bodies_corrupted += 1
                continue
            for rsid, nal in dec.add_body(body):
                res.layers[rsid].delivered += 1
    return res


@dataclass
class AdaptiveResult:
    pipeline: PipelineResult
    active_sids: list
    shared_txagc: int
    effective_snr: float
    per_layer_mcs: dict


def run_svc_pipeline_adaptive(nals: list[bytes], reported_snr: float,
                              reported_txagc: int, controller=None,
                              policy: UepPolicy | None = None,
                              link: lm.LinkModel | None = None,
                              seed: int = 0) -> AdaptiveResult:
    """Close the loop: a `SvcController` picks each layer's MCS, decides which
    layers to shed, and sets one shared TX power from the reported link sample;
    the pipeline then transmits only the active layers at the commanded MCS over
    the resulting effective SNR. Shed layers are not sent (0 delivered, by
    design — that airtime/energy is saved), so base/IDR SLA is met while
    enhancement degrades gracefully.
    """
    import energy_model as em
    from controller import SvcController

    policy = policy or pipeline_uep_policy()
    link = link or lm.LinkModel(trials=200)
    calib = em.load_calibration()
    controller = controller or SvcController(link, calib)

    ops, shared_txagc, active = controller.update(reported_snr, reported_txagc,
                                                  now_ms=0)
    path_loss = reported_snr - calib.gain_db(reported_txagc)
    eff_snr = path_loss + calib.gain_db(shared_txagc)
    mcs_ladder = {sid: ops[sid].mcs for sid in active}

    # only the active layers go on air; shed layers are dropped at the source
    sent = [n for n in nals
            if policy.stream_for(parse_hevc_nal(n)) in active]
    # decode at a fixed MCS ladder over the effective SNR (one shared txagc)
    sim_link = lm.LinkModel(trials=1)
    res = run_svc_pipeline(sent, eff_snr, policy=policy, mcs_ladder=mcs_ladder,
                           link=sim_link, seed=seed)
    # account shed layers as 0-delivered against their full offered count
    for nal in nals:
        sid = policy.stream_for(parse_hevc_nal(nal))
        if sid not in active:
            res.layers[sid].sent += 1
    return AdaptiveResult(pipeline=res, active_sids=active,
                          shared_txagc=shared_txagc, effective_snr=eff_snr,
                          per_layer_mcs=mcs_ladder)


def main() -> dict:
    gops = int(os.environ.get("SVC_GOPS", "8"))
    nals = gen_svc_nals.gen_nals(gops=gops)
    print(f"synthetic HEVC: {len(nals)} NALs over {gops} GOPs")
    rows = []
    for snr in [40, 28, 22, 18, 14, 10, 6, 2, -4]:
        r = run_svc_pipeline(nals, float(snr), seed=snr & 0x7F)
        rows.append((snr, r))
        print(f"  SNR {snr:+3d} dB : {r.summary()}")
    # headline number: at a marginal SNR, base holds while enhancement sheds
    mid = run_svc_pipeline(nals, 12.0, seed=3)
    return {"crit": mid.delivery(0), "t0": mid.delivery(1),
            "t1": mid.delivery(2), "t2": mid.delivery(3)}


if __name__ == "__main__":
    main()
