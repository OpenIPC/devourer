#!/usr/bin/env python3
"""Offline closed-loop A/B: energy-min adaptive controller vs a fixed profile.

Drives a "fly out and back" path-loss schedule (close -> far -> close) through the
controller + link model + energy model, and compares energy-per-delivered-bit and
delivery against a FIXED baseline profile sized for the worst-case range (the
"set it for worst case and forget" status quo). Headline: the adaptive link banks
the close-range headroom as Watts saved while holding the delivery SLA.

  uv run python ../tests/sim_loop.py            # from tools/precoder, or:
  PYTHONPATH=tools/precoder uv run python tests/sim_loop.py
"""
import os
import sys

sys.path.insert(0, os.path.expanduser("~/git/devourer/tools/precoder"))
import energy_model as em
import link_model as lm
import op_table
from controller import Controller, ControllerConfig


def path_loss_schedule(n_ticks: int, hi: float = 35.0, lo: float = -12.0):
    """Triangle: free-SNR (received SNR at TXAGC 0) close(hi) -> far(lo) -> close.
    Spans ~47 dB (more than the ~25 dB TXAGC range) so a real flight forces MCS
    adaptation, not just power tracking."""
    half = n_ticks // 2
    out = []
    for t in range(n_ticks):
        frac = t / half if t < half else (n_ticks - t) / (n_ticks - half)
        out.append(lo + (hi - lo) * frac)
    return out


def run(link, calib, cfg, schedule, fixed_op=None, dt_s=0.1):
    """Returns (energy_J, delivered_bits, offered_bits, list_of_ops). Each tick
    carries `cfg.src_bitrate_bps` of video for dt_s seconds at the chosen point;
    energy = P_avg*dt, delivered = src*P_deliver*dt."""
    import energy_model as em
    ctrl = Controller(link, calib, cfg) if fixed_op is None else None
    src = cfg.src_bitrate_bps
    energy = delivered = offered = 0.0
    prev_txagc = 32
    ops = []
    for t, pl in enumerate(schedule):
        if fixed_op is None:
            recv_reported = pl + calib.gain_db(prev_txagc)     # VRX feedback (1-tick lag)
            op = ctrl.update(recv_reported, prev_txagc, now_ms=t * 100)
            if op is None:                                     # shed layer (not base)
                prev_txagc = 0
                ops.append(None)
                offered += src * dt_s
                continue
        else:
            op = fixed_op
        true_recv = pl + calib.gain_db(op.txagc)
        af = em.airtime_fraction(op.tx(), src, op.overhead, cfg.payload_bytes, calib)
        # channel overload (can't carry the offered bitrate) delivers nothing
        deliver = 0.0 if af > 1.0 else link.p_deliver(true_recv, op.mcs, op.overhead)
        energy += em.avg_power_w(op.tx(), af, calib) * dt_s
        delivered += src * deliver * dt_s
        offered += src * dt_s
        prev_txagc = op.txagc
        ops.append(op)
    return energy, delivered, offered, ops


def robust_baseline(link, calib, cfg, worst_free_snr, margin_db=3.0):
    """The over-provisioned 'set-and-forget' profile a cautious operator flies:
    the most-robust row (lowest snr_req: low MCS + heavy FEC) resolved with a
    safety margin at the worst range -> high power. Guarantees the SLA across the
    whole flight, and wastes most of it when close."""
    rows = op_table.build_link_rows(link, cfg.target, cfg.mcs_set, cfg.overhead_set)
    # most-robust (lowest snr_req) row that is FEASIBLE (finite e_bit = can carry
    # the bitrate AND a TXAGC reaches its snr_req) at the worst range.
    feasible = []
    for r in sorted(rows, key=lambda r: r.snr_req):
        op = op_table.resolve(r, worst_free_snr, calib, link, cfg.payload_bytes,
                              cfg.src_bitrate_bps, margin_db)
        if op and op.e_bit < float("inf") and op.p_deliver >= cfg.target:
            feasible.append(op)
    return feasible[0] if feasible else op_table.MAX_RANGE


def main():
    link = lm.LinkModel(trials=1200)
    calib = em.load_calibration()
    cfg = ControllerConfig(target=0.99, k=8, payload_bytes=1024,
                           src_bitrate_bps=4e6, allow_shed=False)
    sched = path_loss_schedule(200)
    worst = min(sched)

    base_ctrl = Controller(link, calib, cfg)
    emin = base_ctrl._best(worst, cfg.margin_db) or op_table.MAX_RANGE   # energy-aware static
    robust = robust_baseline(link, calib, cfg, worst)                    # over-provisioned

    e_ad, d_ad, o_ad, ops = run(link, calib, cfg, sched)
    e_em, d_em, o_em, _ = run(link, calib, cfg, sched, fixed_op=emin)
    e_rb, d_rb, o_rb, _ = run(link, calib, cfg, sched, fixed_op=robust)

    def ebit(e, d):
        return e / d if d else float("inf")
    eb_ad, eb_em, eb_rb = ebit(e_ad, d_ad), ebit(e_em, d_em), ebit(e_rb, d_rb)
    save_em = 1 - eb_ad / eb_em
    save_rb = 1 - eb_ad / eb_rb
    # disruptive changes = MCS/FEC (TXAGC tracks freely; it's the cheap power lever)
    changes = sum(1 for a, b in zip(ops, ops[1:])
                  if a and b and (a.mcs, a.overhead) != (b.mcs, b.overhead))

    print(f"ADAPTIVE        : E/bit={eb_ad*1e9:6.1f} nJ  delivery={d_ad/o_ad:.3f}")
    print(f"FIXED energy-min: E/bit={eb_em*1e9:6.1f} nJ  delivery={d_em/o_em:.3f}  "
          f"(MCS{emin.mcs} ov{emin.overhead} txagc{emin.txagc})  -> SAVED {save_em*100:.1f}%")
    print(f"FIXED robust    : E/bit={eb_rb*1e9:6.1f} nJ  delivery={d_rb/o_rb:.3f}  "
          f"(MCS{robust.mcs} ov{robust.overhead} txagc{robust.txagc})  -> SAVED {save_rb*100:.1f}%")
    print(f"config changes over {len(sched)} ticks: {changes}")
    return {"save_vs_emin": save_em, "save_vs_robust": save_rb,
            "delivery": d_ad / o_ad, "changes": changes}


if __name__ == "__main__":
    main()


