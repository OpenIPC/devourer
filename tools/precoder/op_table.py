"""Operating-point table for the adaptive controller.

A LINK ROW is a (MCS, FEC-overhead) pair with the received SNR it needs to clear
the delivery target (`snr_req`, from `link_model`). TX power (TXAGC) is NOT a row
dimension — the controller chooses, at runtime, the *minimum* TXAGC that supplies
`snr_req` at the current path loss, then ranks rows by energy-per-delivered-bit.
That is the joint rate+power energy-min: cheapest power per rate, then cheapest
rate. An OP POINT is a fully-resolved row (+ chosen TXAGC + e_bit).
"""

from __future__ import annotations

from dataclasses import dataclass

import energy_model as em


@dataclass(frozen=True)
class LinkRow:
    mode: str
    mcs: int
    bw: int
    sgi: bool
    overhead: float
    snr_req: float          # received SNR (dB) needed to meet the delivery target


@dataclass(frozen=True)
class OpPoint:
    mode: str
    mcs: int
    bw: int
    sgi: bool
    txagc: int
    overhead: float
    snr_req: float
    e_bit: float            # J per delivered source bit at the resolved operating point
    p_deliver: float

    def tx(self) -> "em.TxPoint":
        return em.TxPoint(self.mode, self.mcs, self.bw, self.sgi, self.txagc)


def build_link_rows(link, target: float, mcs_set=range(8),
                    overhead_set=(0.10, 0.25, 0.50, 0.75, 1.00),
                    bw: int = 20, sgi: bool = False, sbi: bool = True,
                    bw_set=None, mode: str = "ht") -> list[LinkRow]:
    """(bw, MCS, overhead) -> snr_req for the given delivery target. Computed once.

    `bw_set` opens the BANDWIDTH dimension (e.g. (20, 40, 80) with mode='vht'):
    one row per rung of the primary-nested ladder the TX can pick per-packet
    with no RX coordination (the RX parks at the widest rung —
    tests/rx80_narrow_tx_probe.sh). A wider row's snr_req carries its
    +3 dB/doubling noise-bandwidth cost (em.bw_noise_db), so the e_bit ranking
    trades airtime against noise bandwidth honestly; the measured wide-RX
    penalty for narrower-than-tune frames is 0 dB and adds nothing. Default
    (bw_set=None) keeps the single-bw behaviour."""
    rows = []
    for w in (bw_set if bw_set is not None else (bw,)):
        for mcs in mcs_set:
            for ov in overhead_set:
                req = link.snr_required(mcs, ov, target, sbi=sbi)
                rows.append(LinkRow(mode, mcs, w, sgi, ov, req + em.bw_noise_db(w)))
    return rows


def resolve(row: LinkRow, path_loss_db: float, calib, link,
            payload_bytes: int, src_bitrate_bps: float,
            margin_db: float = 0.0) -> OpPoint | None:
    """Pick the minimum TXAGC that supplies `row.snr_req + margin` at this path
    loss, price the row (e_bit may be +inf if this MCS is too slow to carry the
    bitrate). Returns None if no TXAGC reaches the required SNR."""
    need_gain = (row.snr_req + margin_db) - path_loss_db
    txagc = 0 if need_gain <= 0 else calib.min_txagc_for_gain(need_gain)
    if txagc is None:
        return None
    recv = path_loss_db + calib.gain_db(txagc)
    # recv is in the 20 MHz-noise reference frame (path loss + gain); the
    # demodulator at this row's bandwidth sees bw_noise_db less SNR — the same
    # term build_link_rows folded into snr_req, kept consistent here.
    pdel = link.p_deliver(recv - em.bw_noise_db(row.bw), row.mcs, row.overhead)
    eb = em.energy_per_delivered_bit(
        em.TxPoint(row.mode, row.mcs, row.bw, row.sgi, txagc),
        src_bitrate_bps, row.overhead, payload_bytes, pdel, calib)
    return OpPoint(row.mode, row.mcs, row.bw, row.sgi, txagc, row.overhead,
                   row.snr_req, eb, pdel)


MAX_RANGE = OpPoint("ht", 0, 20, False, 63, 1.00, 0.0, float("inf"), 0.0)
"""Failsafe: MCS0, 20 MHz, max power, heaviest FEC — energy irrelevant, reach only."""
