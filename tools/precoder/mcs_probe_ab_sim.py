"""A/B sim: model-only vs model+adjacent-MCS-probes under a miscalibrated model.

The controller's whole failure mode is a WRONG BELIEF: its link model says a
rate clears the SLA when the real channel disagrees. This sim splits the two —
the controller runs on a deliberately biased model (`controller.apply_model_
bias`, the same lever the on-air harness uses) while frame delivery comes from
the untouched nominal channel — and drives the full VTX<->VRX loop per frame
(~4 ms, so the probe-window arithmetic is realistic, unlike selftest's 100 ms
tick). Both arms see identical channel randomness (paired seeds).

What the arms show:
  * model-only: promotes into the optimistically-biased row and STAYS there —
    its cur_ok check believes the same wrong model, so nothing pulls it back
    (post-FEC delivery collapses while the model reads healthy);
  * probes on: the candidate's measured delivery never clears the promote
    gate's lower confidence bound, so the biased row is never entered (and its
    upper bound eventually blocks it outright).

Probe COST is priced with no special-casing: probes are real frames flying a
different rate, so their airtime, PA energy, and losses land in the same
energy / delivered-source-bit ledger as everything else. The honest caveat the
clean scenario quantifies: probes measure candidates AT THE INCUMBENT'S TXAGC,
so with probing on a promotion needs the faster rate to already work at the
current power — steady-state upward moves get rarer (delivery holds, energy
may idle slightly above the model-only optimum). Run `python mcs_probe_ab_sim
.py` for the report; test_mcs_probe_ab_sim.py pins the headline.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

import energy_model as em
import link_model as lm
import rc_proto as rp
import rendezvous as rz
from adaptive_link import AdaptiveVrx, AdaptiveVtx
from controller import ControllerConfig, apply_model_bias

VTX_ID = 0xABCD
PAYLOAD = 1024
DT_MS = 4.0                      # ~250 fps injection traffic
FEEDBACK_MS = 100
GEN_FRAMES = 12                  # FEC generation geometry (link_model default)
N_SUB = 10


# --------------------------------------------------------------------------- #
# Scenarios: path loss (free SNR at txagc 0) per frame index. Every scenario
# opens with a 4 s deep-range ACQUISITION segment — a real session starts at
# MAX_RANGE/discovery distance, and the controller's cold pick is deliberately
# model-trusted (ungated), so opening at close range would walk BOTH arms
# straight into an optimistically-biased row before a single probe has flown.
# --------------------------------------------------------------------------- #
ACQ_FRAMES = 1000
ACQ_PL = -16.0


def sc_static(i: int) -> float:
    return 2.0


def sc_triangle(i: int, n: int = 14000) -> float:
    """Fly out and back: near -> far -> near (post-acquisition)."""
    half = n // 2
    return 8.0 - 22.0 * (i if i < half else n - i) / half


def make_sc_fade(seed: int, base: float = 4.0, rho: float = 0.995,
                 sigma: float = 0.3):
    """AR(1) time-correlated fading (~0.8 s correlation, ~3 dB std)."""
    rng = random.Random(seed ^ 0xFADE)
    state = {"x": 0.0, "i": -1}

    def f(i: int) -> float:
        while state["i"] < i:                     # advance once per frame
            state["x"] = rho * state["x"] + rng.gauss(0.0, sigma)
            state["i"] += 1
        return base + state["x"]
    return f


def sc_burst(i: int) -> float:
    """Steady link with a 160 ms deep interference slam every 5 s (past the
    post-downgrade hold, so recovery climbs are part of the picture)."""
    return 4.0 - (20.0 if (i % 1250) < 40 else 0.0)


SCENARIOS = {
    "static": sc_static,
    "triangle": sc_triangle,
    "fade": None,                # built per seed
    "burst": sc_burst,
}


def with_acquisition(pl_fn):
    def f(i: int) -> float:
        return ACQ_PL if i < ACQ_FRAMES else pl_fn(i - ACQ_FRAMES)
    return f


@dataclass
class ArmResult:
    delivery: float              # post-FEC generation delivery (warm period)
    e_bit_nj: float              # realized nJ per delivered source bit
    commits: int                 # (mode, mcs, bw, overhead) changes (churn)
    biased_row_ms: float | None  # first time the biased MCS was flown as the op
    biased_frames: int           # TX frames spent commanded to the biased MCS
    first_block_ms: float | None  # first controller block on the biased MCS
    probe_frames: int
    probe_lost: int
    probe_airtime_frac: float
    settle_mcs: int


def run_arm(pl_fn, n_frames: int, probe: bool, bias: dict[int, float] | None,
            seed: int, trials: int = 300) -> ArmResult:
    truth = lm.LinkModel(trials=trials)           # the channel: nominal
    model = lm.LinkModel(trials=trials)           # the controller's belief
    calib = em.load_calibration()
    # sbi=False: the frame-level Bernoulli channel below has no sub-block
    # salvage, so the controller's belief must price FEC the same way — a
    # mixed-semantics pair would misread as miscalibration in BOTH arms.
    vrx = AdaptiveVrx(model, calib, VTX_ID,
                      ControllerConfig(target=0.99, allow_shed=False, sbi=False,
                                       mcs_probe_enabled=probe))
    if bias:
        apply_model_bias(vrx.ctrl, bias)
    vtx = AdaptiveVtx(VTX_ID, mcs_probe=probe)
    rng = random.Random(seed)
    biased_mcs = max(bias) if bias else None

    energy_j = 0.0
    delivered_bits = 0.0
    airtime_s = probe_air_s = 0.0
    probe_frames = probe_lost = 0
    commits = 0
    prev_key = None
    biased_row_ms = first_block_ms = None
    biased_frames = 0
    settle_mcs = 0
    gen_ok: list[bool] = []
    gen_ov = 0.25
    gens_total = gens_ok = 0
    warm = n_frames // 10
    seq = 0

    for i in range(n_frames):
        now = i * DT_MS
        act = vtx.step(now)
        energy_j += calib.p_baseline_w * DT_MS / 1000.0   # floor: always on
        if act in (rz.A_TX_VIDEO, rz.A_FAILSAFE):
            cand = vtx.probe_mcs_for_seq(seq)
            sel, mode = vtx.state.mcs, vtx.state.mode
            flown = sel if cand is None else cand
            txagc = vtx.state.txagc
            recv = pl_fn(i) + calib.gain_db(txagc)
            ok = rng.random() < 1.0 - truth.channel(flown, recv)["corrupt_rate"]

            t_on = calib.t_pre_us * 1e-6 + 8.0 * PAYLOAD / (
                em.phy_rate_mbps(mode, flown, vtx.state.bw) * 1e6)
            energy_j += calib.pa_w(txagc) * t_on
            airtime_s += t_on
            if cand is not None:
                probe_frames += 1
                probe_air_s += t_on
                probe_lost += 0 if ok else 1

            if ok:
                vrx.on_video(rssi=-52.0, snr=recv, crc_err=False, seq=seq,
                             now_ms=now, rate=(mode, flown))
            # FEC generation accounting at the overhead the generation started
            if not gen_ok:
                gen_ov = vtx.state.overhead
            gen_ok.append(ok)
            if len(gen_ok) == GEN_FRAMES:
                total = GEN_FRAMES * N_SUB
                src = max(1, min(total, round(total / (1.0 + gen_ov))))
                need = math.ceil(src / N_SUB)
                if i >= warm:
                    gens_total += 1
                    if sum(gen_ok) >= need:
                        gens_ok += 1
                        delivered_bits += 8.0 * PAYLOAD * GEN_FRAMES / (1.0 + gen_ov)
                gen_ok = []
            seq = (seq + 1) & 0xFFF

        fb = vrx.step(now)
        if fb is not None:
            out = vtx.on_rc_frame(fb, now)
            if out is not None:
                vrx.on_disc_ack(out, now)

        key = (vtx.state.mode, vtx.state.mcs, vtx.state.bw, vtx.state.overhead)
        if key != prev_key:
            if prev_key is not None and i >= warm:
                commits += 1
            prev_key = key
        if i == warm:
            settle_mcs = vtx.state.mcs
        if biased_mcs is not None:
            if vtx.state.mcs == biased_mcs and not vtx.state.failsafe:
                biased_frames += 1
                if biased_row_ms is None:
                    biased_row_ms = now
            if first_block_ms is None and vrx.ctrl._mcs_blocked(biased_mcs):
                first_block_ms = now

    return ArmResult(
        delivery=gens_ok / gens_total if gens_total else 0.0,
        e_bit_nj=(energy_j / delivered_bits * 1e9) if delivered_bits else
        float("inf"),
        commits=commits,
        biased_row_ms=biased_row_ms,
        biased_frames=biased_frames,
        first_block_ms=first_block_ms,
        probe_frames=probe_frames,
        probe_lost=probe_lost,
        probe_airtime_frac=probe_air_s / airtime_s if airtime_s else 0.0,
        settle_mcs=settle_mcs)


def run_scenario(name: str, n_frames: int = 15000, bias: dict | None = None,
                 seed: int = 1, trials: int = 300):
    """`ambush` = the static scenario WITHOUT the acquisition segment: the
    session opens at close range and the deliberately-ungated cold pick lands
    straight on the biased row — the case the measured active-row escape
    exists for."""
    arms = {}
    for probe in (False, True):
        if name == "fade":                        # paired fading realization
            f = make_sc_fade(seed)
        elif name == "triangle":
            f = lambda i: sc_triangle(i, n_frames - ACQ_FRAMES)   # noqa: E731
        else:
            f = SCENARIOS["static" if name == "ambush" else name]
        arms["probe" if probe else "model"] = run_arm(
            f if name == "ambush" else with_acquisition(f),
            n_frames, probe, bias, seed, trials)
    return arms


def main():
    bias = {5: -8.0}          # the model believes MCS5 needs 8 dB less than it does
    print(f"MCS-probe A/B — {DT_MS:g} ms frames, model bias {bias} "
          f"(clean scenario runs unbiased)\n")
    hdr = (f"{'scenario':<10} {'arm':<6} {'deliv':>6} {'nJ/bit':>8} "
           f"{'settle':>6} {'commits':>7} {'flew-bad':>9} {'bad-frm':>7} "
           f"{'blocked':>8} {'probe%':>7} {'p-lost':>6}")
    print(hdr)
    print("-" * len(hdr))
    for name in ("static", "ambush", "triangle", "fade", "burst"):
        for label, b in (("", bias), ("-clean", None)):
            if label == "-clean" and name != "static":
                continue          # one unbiased control is enough
            arms = run_scenario(name, bias=b)
            for arm, r in arms.items():
                fb = "-" if r.biased_row_ms is None else f"{r.biased_row_ms/1000:.1f}s"
                bl = "-" if r.first_block_ms is None else f"{r.first_block_ms/1000:.1f}s"
                print(f"{name + label:<10} {arm:<6} {r.delivery:>6.3f} "
                      f"{r.e_bit_nj:>8.2f} {'MCS' + str(r.settle_mcs):>6} "
                      f"{r.commits:>7d} {fb:>9} {r.biased_frames:>7d} "
                      f"{bl:>8} {r.probe_airtime_frac:>6.1%} {r.probe_lost:>6d}")
        print()


if __name__ == "__main__":
    main()
