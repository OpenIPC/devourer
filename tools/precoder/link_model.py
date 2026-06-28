"""Link model for the adaptive link — SNR -> block-delivery probability.

Turns an estimated link SNR + an operating point (MCS, FEC overhead) into
`P_deliver` (the probability a FEC block's source is recovered), which the energy
model divides into Joules to get energy-per-delivered-bit. It reuses the measured-
channel FEC accounting in `fec_ab_sim` verbatim — the SBI-vs-plain survivor model,
the CRC tax, `sim_interframe`/`sim_intra` — so the controller prices delivery and
airtime consistently.

Two layers:
  * a CHANNEL = (corrupt-frame rate, per-corrupt-frame sub-block survivor histogram)
    keyed by (MCS, SNR), exactly the structure of `fec_ab_sim.CHANNELS`;
  * `P_deliver(SNR, MCS, overhead)` = `fec_ab_sim.sim_interframe(channel, overhead)`.

CALIBRATION ("model now, meter later"): channels default to a documented NOMINAL
per-MCS FCS waterfall + survivor shape. `tests/calibrate_link.py` replaces them
with histograms measured by sweeping `tests/sdr_interferer.py` and reading
`fused_fec_link.FusedFecReceiver.report()`. The shape (energy-min picks the same
operating points) is right from the nominal model; absolute SNR thresholds move
once calibrated.
"""

from __future__ import annotations

import json
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import fec_ab_sim  # noqa: E402

# Nominal per-HT-MCS FCS-failure waterfall centres (dB, ~50% corrupt-frame rate
# for a ~1500 B frame) and slope. Higher MCS = more fragile = higher centre.
# OVERRIDE via calibrate_link.py. These set where each MCS "falls off".
NOMINAL_MCS_CENTER_DB = {0: 6.0, 1: 8.0, 2: 10.0, 3: 13.0, 4: 17.0, 5: 21.0,
                         6: 23.0, 7: 26.0}
NOMINAL_SLOPE_DB = 2.5
N_SUB = 10  # sub-blocks per body (matches fec_ab_sim / fec_subblock default body)


def _logistic(x: float) -> float:
    if x < -60:
        return 0.0
    if x > 60:
        return 1.0
    return 1.0 / (1.0 + math.exp(-x))


def synth_channel(mcs: int, snr_db: float,
                  center_db: dict[int, float] | None = None,
                  slope_db: float = NOMINAL_SLOPE_DB) -> dict:
    """Nominal (corrupt_rate, survivor_hist) for (MCS, SNR), fec_ab_sim-shaped.

    corrupt_rate rises as SNR drops below the MCS centre. Within a corrupt frame
    the surviving-sub-block fraction rises with SNR (localized near the edge,
    frame-wide far below) — the structure SBI exploits. Histogram = binomial
    around that mean, the conservative i.i.d. stance fec_ab_sim already takes.
    """
    centers = center_db or NOMINAL_MCS_CENTER_DB
    c = centers.get(mcs, 26.0)
    corrupt_rate = _logistic((c - snr_db) / slope_db)        # 0.5 at SNR=centre
    # mean surviving fraction of a corrupt frame's sub-blocks
    f = 0.55 + 0.05 * (snr_db - c)
    f = max(0.05, min(0.95, f))
    # binomial(N_SUB, f) histogram over corrupt frames (scaled to a count)
    hist: dict[int, int] = {}
    scale = 1000
    for s in range(N_SUB + 1):
        pmf = math.comb(N_SUB, s) * (f ** s) * ((1 - f) ** (N_SUB - s))
        hist[s] = max(0, round(pmf * scale))
    if sum(hist.values()) == 0:
        hist[round(f * N_SUB)] = 1
    return {"corrupt_rate": round(corrupt_rate, 4), "n_sub": N_SUB,
            "survivor_hist": {k: v for k, v in hist.items() if v > 0}}


class LinkModel:
    """SNR -> P_deliver, backed by a (MCS, SNR-bucket) channel table.

    Nominal channels are synthesised on demand; a calibration file replaces them
    with measured ones (keyed 'mcs:snr_bucket'). P_deliver is cached and uses a
    fixed RNG seed so the controller is deterministic.
    """

    def __init__(self, calib_path: str | None = None, frames_per_gen: int = 12,
                 trials: int = 2000, snr_bucket_db: float = 1.0):
        self.frames_per_gen = frames_per_gen
        self.trials = trials
        self.bucket = snr_bucket_db
        self._measured: dict[str, dict] = {}
        self._center = dict(NOMINAL_MCS_CENTER_DB)
        self._slope = NOMINAL_SLOPE_DB
        if calib_path:
            with open(calib_path) as f:
                d = json.load(f)
            self._measured = d.get("channels", {})
            self._center.update({int(k): v for k, v in d.get("centers", {}).items()})
            self._slope = d.get("slope_db", self._slope)
        self._deliver_cache: dict[tuple, float] = {}

    def _snr_bucket(self, snr_db: float) -> float:
        return round(snr_db / self.bucket) * self.bucket

    def channel(self, mcs: int, snr_db: float) -> dict:
        b = self._snr_bucket(snr_db)
        key = f"{mcs}:{b:g}"
        if key in self._measured:
            return self._measured[key]
        return synth_channel(mcs, b, self._center, self._slope)

    def p_deliver(self, snr_db: float, mcs: int, overhead: float,
                  sbi: bool = True) -> float:
        """Block-delivery probability at (SNR, MCS, FEC overhead). sbi=True uses
        the SBI sub-block salvage path (fused FEC); False = plain whole-frame."""
        b = self._snr_bucket(snr_db)
        key = (mcs, b, round(overhead, 3), sbi)
        if key in self._deliver_cache:
            return self._deliver_cache[key]
        ch = self.channel(mcs, b)
        rng = random.Random(0xA11CE ^ hash(key) & 0xFFFFFFFF)
        deliver, _ = fec_ab_sim.sim_interframe(ch, overhead, self.frames_per_gen,
                                               self.trials, rng, sbi=sbi)
        self._deliver_cache[key] = deliver
        return deliver

    def snr_required(self, mcs: int, overhead: float, target: float,
                     sbi: bool = True, lo: float = -5.0, hi: float = 40.0,
                     step: float = 0.5) -> float:
        """Lowest SNR (dB) where p_deliver >= target, scanning a grid. Returns
        hi+step (i.e. 'infeasible') if the target is never met."""
        snr = lo
        while snr <= hi:
            if self.p_deliver(snr, mcs, overhead, sbi) >= target:
                return snr
            snr += step
        return hi + step


if __name__ == "__main__":  # quick sanity dump
    lm = LinkModel(trials=800)
    print("HT MCS  SNR_req(99% deliver, ov=0.25)  SNR_req(ov=1.0)")
    for mcs in range(8):
        r25 = lm.snr_required(mcs, 0.25, 0.99)
        r100 = lm.snr_required(mcs, 1.00, 0.99)
        print(f"  MCS{mcs}: {r25:5.1f} dB        {r100:5.1f} dB")
