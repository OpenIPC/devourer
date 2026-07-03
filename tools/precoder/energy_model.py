"""Energy model for the adaptive link — energy-per-delivered-bit on the RTL8812AU.

The controller minimises energy/bit, where (per the bits-per-Joule literature)
**time-on-air is the dominant energy term and radiated power is a weak one**.
This module turns a transmit operating point (MCS, bandwidth, SGI, TXAGC index,
FEC k/overhead, payload size) plus a delivery probability into Joules-per-
delivered-bit:

    t_air   = 8*L / R_phy(MCS, BW, SGI)         # data on-air time
    t_on    = t_pre + t_air                       # whole frame on the air
    t_slot  = t_on + t_gap                         # frame period (incl. idle gap)
    E_slot  = P_circuit * t_slot + P_pa[idx] * t_on
    E_block = (K + R) * E_slot                     # R = repair frames
    E_bit   = E_block / (K * 8 * L * P_deliver)    # P_deliver from link_model

`P_circuit` (LO + baseband + USB PHY) is paid over the *whole* slot, so duty and
MCS both matter; `P_pa[idx]` is paid only while on the air. The split is the
reason high MCS (short t_air) saves energy and TXAGC barely moves it.

CALIBRATION ("model now, meter later"): the constants default to a documented
NOMINAL RTL8812AU-on-USB model. `tests/calibrate_energy.py` fits the real
`P_circuit` / `P_pa[idx]` from the thermal-gain + duty sweeps (and, when an inline
DC meter is present, anchors them to absolute Watts) and writes a JSON that
`load_calibration()` overlays. Relative energy *savings* are valid from the
nominal model alone; absolute Watts need the meter.

numpy-free (imports into the GNU Radio / orchestrator env like fec_subblock).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field

# --------------------------------------------------------------------------- #
# 802.11 PHY data rates (Mbps). HT/VHT = 1 spatial stream. HT MCS 0..7;
# VHT MCS 0..8 (MCS9 is invalid for 1SS/20 and 1SS with BCC constraints vary,
# and the link model has no SNR centre for it — 0..8 is the modelled set).
# VHT 80 MHz is the bandwidth rung HT lacks — the ladder the adaptive
# controller's bandwidth dimension rides (20 c 40 c 80, primary-nested).
# --------------------------------------------------------------------------- #
HT20_LGI_MBPS = [6.5, 13.0, 19.5, 26.0, 39.0, 52.0, 58.5, 65.0]
HT40_LGI_MBPS = [13.5, 27.0, 40.5, 54.0, 81.0, 108.0, 121.5, 135.0]
VHT_LGI_MBPS = {
    20: [6.5, 13.0, 19.5, 26.0, 39.0, 52.0, 58.5, 65.0, 78.0],
    40: [13.5, 27.0, 40.5, 54.0, 81.0, 108.0, 121.5, 135.0, 162.0],
    80: [29.3, 58.5, 87.8, 117.0, 175.5, 234.0, 263.3, 292.5, 351.0],
}
LEGACY_MBPS = {6: 6.0, 9: 9.0, 12: 12.0, 18: 18.0, 24: 24.0, 36: 36.0, 48: 48.0,
               54: 54.0}
SGI_FACTOR = 10.0 / 9.0  # short guard interval: 400 vs 800 ns -> x1.111


def phy_rate_mbps(mode: str, mcs: int, bw: int = 20, sgi: bool = False) -> float:
    """On-air PHY data rate. mode='ht' (mcs 0..7), 'vht' (1SS mcs 0..8,
    bw 20/40/80) or 'legacy' (mcs = Mbps)."""
    if mode == "ht":
        base = HT40_LGI_MBPS if bw == 40 else HT20_LGI_MBPS
        if not (0 <= mcs < len(base)):
            raise ValueError(f"HT mcs {mcs} out of range 0..7")
        r = base[mcs]
    elif mode == "vht":
        if bw not in VHT_LGI_MBPS:
            raise ValueError(f"VHT bw {bw} not one of 20/40/80")
        base = VHT_LGI_MBPS[bw]
        if not (0 <= mcs < len(base)):
            raise ValueError(f"VHT 1SS mcs {mcs} out of range 0..8")
        r = base[mcs]
    elif mode == "legacy":
        if mcs not in LEGACY_MBPS:
            raise ValueError(f"legacy rate {mcs} not a valid OFDM rate")
        r = LEGACY_MBPS[mcs]
    else:
        raise ValueError(f"unknown mode {mode!r}")
    return r * (SGI_FACTOR if sgi else 1.0)


def bw_noise_db(bw: int) -> float:
    """Extra noise power a receiver integrates at `bw` vs the 20 MHz reference:
    10*log10(bw/20) — +3 dB per bandwidth doubling. The op-table adds this to a
    row's required SNR so wider rows honestly pay their noise-bandwidth cost.
    The measured WIDE-RX penalty (an 80 MHz-tuned RX decoding a narrower frame
    on its primary, vs retuning) is 0 dB / <=1 dB on real silicon
    (tests/wide_rx_penalty_sweep.sh), so no extra term is added for rows
    narrower than the RX tune."""
    if bw <= 20:
        return 0.0
    return 10.0 * math.log10(bw / 20.0)


# --------------------------------------------------------------------------- #
# Nominal calibration. OVERRIDE with a metered JSON via load_calibration().
# Values are an RTL8812AU-on-USB-2 order-of-magnitude model, NOT measured:
#   p_circuit_w  ~ idle/RX bus power (LO, baseband, USB) ~ 1.2 W
#   p_pa_w[idx]  ~ incremental PA DC draw vs TXAGC index, ~0.05..0.6 W,
#                  compressing near the top (a deliberately WEAK energy lever)
#   t_pre_us     ~ HT-mixed preamble + SIG on-air before the data symbols
# --------------------------------------------------------------------------- #
def _nominal_pa_curve() -> list[float]:
    pa = []
    for idx in range(64):
        x = idx / 63.0
        # PA DC draw (W) ON-AIR. These USB adapters are PA-heavy at high power
        # (the 5 GHz Vbus-sag note), so the PA is a real chunk of the budget:
        # ~0.1 W at idx 0 up to ~1.5 W at idx 63, compressing near the top.
        pa.append(round(0.10 + 1.40 * (x ** 0.8), 4))
    return pa


def _nominal_gain_curve(g_max_db: float = 25.0) -> list[float]:
    """Radiated-power gain (dB) vs TXAGC index — the LINK lever. Concave
    (PA compresses near the top): 0 dB at idx 0, ~g_max at idx 63. Calibrated
    from the SDR `dbfs` sweep in calibrate_energy.py."""
    import math
    denom = 1.0 - math.exp(-3.0)
    return [round(g_max_db * (1.0 - math.exp(-3.0 * idx / 63.0)) / denom, 3)
            for idx in range(64)]


DEFAULT_CALIB = {
    "source": "nominal",          # set to "metered" by calibrate_energy.py
    "metered_watts": False,        # True once anchored to an inline DC meter
    "p_baseline_w": 0.7,          # LO+baseband+USB+RX, ALWAYS on (the floor)
    "p_pa_w": _nominal_pa_curve(),  # PA, paid only over airtime
    "txagc_gain_db": _nominal_gain_curve(),
    "t_pre_us": 40.0,             # per-frame preamble+SIG on-air overhead
}


@dataclass
class Calib:
    p_baseline_w: float
    p_pa_w: list[float]
    txagc_gain_db: list[float]
    t_pre_us: float
    source: str = "nominal"
    metered_watts: bool = False

    def pa_w(self, idx: int) -> float:
        return self.p_pa_w[max(0, min(63, int(idx)))]

    def gain_db(self, idx: int) -> float:
        """Radiated gain (dB) at TXAGC idx — how much received SNR it buys."""
        return self.txagc_gain_db[max(0, min(63, int(idx)))]

    def min_txagc_for_gain(self, need_db: float) -> int | None:
        """Smallest TXAGC index giving >= need_db radiated gain, or None."""
        for idx in range(64):
            if self.txagc_gain_db[idx] >= need_db:
                return idx
        return None


def load_calibration(path: str | None = None) -> Calib:
    """Nominal model overlaid by a calibration JSON (from calibrate_energy.py)."""
    d = dict(DEFAULT_CALIB)
    if path:
        with open(path) as f:
            d.update(json.load(f))
    return Calib(p_baseline_w=d["p_baseline_w"], p_pa_w=list(d["p_pa_w"]),
                 txagc_gain_db=list(d.get("txagc_gain_db", DEFAULT_CALIB["txagc_gain_db"])),
                 t_pre_us=d["t_pre_us"],
                 source=d.get("source", "nominal"),
                 metered_watts=bool(d.get("metered_watts", False)))


# --------------------------------------------------------------------------- #
# Operating point + energy
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class TxPoint:
    """One transmit operating point (a row of the controller's op-table)."""
    mode: str = "ht"          # 'ht' | 'legacy'
    mcs: int = 0              # HT 0..7, or legacy Mbps
    bw: int = 20             # 20 | 40
    sgi: bool = False
    txagc: int = 32          # TXAGC index 0..63 (SetTxPowerOverride)


# --------------------------------------------------------------------------- #
# Stream energy: to deliver a fixed video bitrate, the strategy sets the AIRTIME
# fraction (MCS + FEC) and the PA power (TXAGC). P_baseline is always on (the
# floor adaptation can't remove without sleeping); the savable energy is the
# airtime x P_pa term. This is the physically-correct framing for "energy to
# carry a video stream" — the per-frame/idle-gap view drowns the levers.
# --------------------------------------------------------------------------- #
def phy_rate_eff_bps(p: TxPoint, payload_bytes: int, calib: Calib) -> float:
    """Effective goodput (bits/s) including the per-frame preamble overhead."""
    r_bps = phy_rate_mbps(p.mode, p.mcs, p.bw, p.sgi) * 1e6
    t_air = (8.0 * payload_bytes) / r_bps
    t_on = calib.t_pre_us * 1e-6 + t_air
    return (8.0 * payload_bytes) / t_on


def airtime_fraction(p: TxPoint, src_bitrate_bps: float, overhead: float,
                     payload_bytes: int, calib: Calib) -> float:
    """Fraction of wall time the PA+TX path is on-air to carry `src_bitrate_bps`
    of source at this MCS/FEC. >1.0 means the channel can't carry it (infeasible)."""
    on_air_bps = src_bitrate_bps * (1.0 + overhead)
    return on_air_bps / phy_rate_eff_bps(p, payload_bytes, calib)


def avg_power_w(p: TxPoint, airtime_frac: float, calib: Calib) -> float:
    """Average DC power (W): baseline always on + PA only over airtime."""
    return calib.p_baseline_w + min(1.0, airtime_frac) * calib.pa_w(p.txagc)


def energy_per_delivered_bit(p: TxPoint, src_bitrate_bps: float, overhead: float,
                             payload_bytes: int, p_deliver: float,
                             calib: Calib) -> float:
    """J per delivered SOURCE bit, carrying `src_bitrate_bps` of video.

        E_bit = (P_baseline + airtime*P_pa) / (src_bitrate * P_deliver)

    Higher MCS / less FEC -> less airtime -> less PA energy; lower TXAGC -> less
    P_pa; lost blocks (p_deliver<1) -> smaller denominator -> energy EXPENSIVE
    (so energy-min avoids the FCS cliff). +inf if infeasible (airtime>1) or
    nothing delivered."""
    if p_deliver <= 0.0:
        return float("inf")
    af = airtime_fraction(p, src_bitrate_bps, overhead, payload_bytes, calib)
    if af > 1.0:
        return float("inf")              # channel can't carry the offered bitrate
    p_avg = avg_power_w(p, af, calib)
    return p_avg / (src_bitrate_bps * p_deliver)


if __name__ == "__main__":  # quick sanity dump
    cal = load_calibration()
    src = 4e6  # 4 Mbps video
    print(f"calibration source={cal.source} metered={cal.metered_watts}; "
          f"carrying {src/1e6:g} Mbps video, ov=0.25, txagc=32")
    for mcs in range(8):
        pt = TxPoint(mcs=mcs, txagc=32)
        af = airtime_fraction(pt, src, 0.25, 1024, cal)
        eb = energy_per_delivered_bit(pt, src, 0.25, 1024, 1.0, cal)
        print(f"  HT MCS{mcs}: rate={phy_rate_mbps('ht', mcs):.1f}Mbps "
              f"airtime={af:.3f} P_avg={avg_power_w(pt, af, cal):.2f}W "
              f"E/bit={eb*1e9:.2f} nJ (clean)")
