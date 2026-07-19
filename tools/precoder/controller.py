"""Energy-minimising adaptive controller — the dual of OpenIPC alink.

alink picks the highest-quality profile a link budget allows. This picks the
**lowest energy-per-delivered-bit operating point that still meets the delivery
SLA** — bank link headroom as Watts saved, not as throughput. Per feedback tick
(~100 ms) it: smooths the VRX SNR, estimates path loss, and for every (MCS, FEC)
row chooses the minimum TXAGC that clears the row's `snr_req`, then takes the
row with the smallest e_bit. Joint rate+power: cheapest power per rate, cheapest
rate overall.

Hysteresis is asymmetric (alink-style): enter a cheaper/fragile row only with an
SNR margin and a rate-limit (slow up); fall to a more robust row immediately when
the current one stops clearing (fast down), then hold before climbing again.
Feedback loss -> MAX_RANGE failsafe. One controller per SVC layer (`stream_id`):
base layers get a high delivery target and may use expensive rows; enhancement
layers get a lower target and are SHED (return None) when no row clears — the
graceful UEP staircase.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import energy_model as em
import op_table
from op_table import OpPoint, build_link_rows, resolve, MAX_RANGE


@dataclass
class ControllerConfig:
    target: float = 0.99              # delivery SLA for this layer's blocks
    k: int = 8                        # FEC block source symbols (protocol/FEC)
    payload_bytes: int = 1024
    src_bitrate_bps: float = 4e6      # offered source (video) bitrate for this layer
    mcs_set: tuple = tuple(range(8))
    overhead_set: tuple = (0.10, 0.25, 0.50, 0.75, 1.00)
    bw: int = 20
    # Bandwidth dimension (opt-in, default None -> single-bw rows as before):
    # the primary-nested rungs the TX may pick per-packet while the RX parks at
    # the widest (e.g. bw_set=(20, 40, 80), mode='vht'). Rows pay their
    # +3 dB/doubling noise-bandwidth cost in snr_req; the e_bit ranking then
    # answers "drop bandwidth or drop MCS?" per tick.
    bw_set: tuple | None = None
    mode: str = "ht"
    sbi: bool = True
    ema_alpha: float = 0.3            # SNR EWMA weight when SNR is RISING (cautious)
    ema_alpha_down: float = 0.8       # ...when FALLING (react fast -> raise power in time)
    margin_db: float = 2.0            # hysteresis: extra SNR to ENTER a row (slow up)
    min_between_changes_ms: int = 150
    hold_after_downgrade_ms: int = 4000
    improve_frac: float = 0.03        # only switch up if >=3% cheaper (vs churn)
    feedback_timeout_ms: int = 1000   # no feedback this long -> failsafe
    allow_shed: bool = True           # enhancement layers may be deallocated
    # Variance-aware fade margin (OPT-IN, default OFF -> behaviour unchanged):
    # under time-correlated fading the energy-min point can't cover deep dips. The
    # margin adds `fade_margin_k` dB of TXAGC HEADROOM (power only) per dB of recent
    # path-loss std (clamped) AFTER the operating point is chosen — so the
    # MCS/FEC choice is unchanged (adding it to row selection instead would buy
    # robustness with airtime/FEC and overload the channel during fades). Variance
    # is measured on the path loss (TX power removed), so our own TXAGC moves don't
    # feed back into it.
    fade_margin_k: float = 0.0        # 0 = disabled
    fade_margin_max_db: float = 12.0  # clamp on the extra TXAGC headroom (dB)
    fade_var_alpha: float = 0.2       # EWMA weight for the path-loss variance
    # Per-rung interference sensing (with bw_set): report_rung_delivery()
    # BLOCKS a rung whose probed delivery falls this far below the best
    # rung's — detection by CONTRAST across rungs, not by model prediction, so
    # it fires on the interference signature (one rung's extra spectrum is
    # dirty) and not on plain path loss (all rungs sag together). The
    # narrowest rung is never blocked: it is the fallback; when IT also
    # underperforms alongside every other rung the primary itself is dirty
    # (`primary_dirty`), which no unilateral bandwidth choice can fix — the
    # escalation to a coordinated channel move belongs to the embedding app.
    rung_block_delta: float = 0.15
    rung_block_hold_ms: int = 5000
    rung_min_samples: int = 8
    # Live adjacent-MCS probes (OPT-IN, default OFF -> behaviour unchanged):
    # report_mcs_delivery() feeds measured candidate delivery (score.
    # McsProbeWindow.stats()) and the evidence corrects the MODEL, it does not
    # replace it. While the current point still works, a row with mcs ABOVE the
    # current one is admissible only when its probe LOWER confidence bound
    # clears the raw frame-delivery its FEC needs; a candidate whose UPPER
    # bound cannot reach even the heaviest overhead's requirement is blocked
    # outright for a hold. Emergency downgrades keep trusting the model — never
    # wait for probes to escape a failing point. The raw-delivery requirement
    # rises steeply as overhead falls (ov=0.10 needs ~0.99 per-frame), and a
    # Wilson lower bound caps at n/(n+z^2), so the only promote path is
    # mcs+1 at mid/heavy overhead first, then the ungated same-MCS overhead
    # trim — measured caution, by construction. Sample-rate dependence: one
    # up-probe per 64 frames, so min_samples=12 inside max_age needs sustained
    # >=~100 fps injection traffic; thresholds are sized for the streaming
    # (~250+ fps) use, not the selftest tick rate.
    mcs_probe_enabled: bool = False
    mcs_probe_min_samples: int = 12
    mcs_probe_window_samples: int = 64
    mcs_probe_max_age_ms: int = 8000
    mcs_probe_block_hold_ms: int = 5000
    mcs_probe_confidence: float = 0.90
    mcs_probe_margin: float = 0.0
    # The ACTIVE row's measured delivery (its samples are the main stream, so
    # they arrive at full frame rate) may condemn the operating point the
    # model still believes in — the SNR-blind failure modes (interference, a
    # miscalibrated row the cold pick landed on) show up only here. Tripping
    # is deliberately slower than a candidate block: the active evidence must
    # stay condemned for this many CONSECUTIVE feedback reports, so a burst
    # that briefly floods the fast-filling window rides through while a
    # persistent lie cannot. Candidates need no such guard — their samples
    # accrue at probe duty, so min_samples already spans seconds.
    mcs_probe_active_trip_ticks: int = 3


class Controller:
    def __init__(self, link, calib, cfg: ControllerConfig | None = None):
        self.link = link
        self.calib = calib
        self.cfg = cfg or ControllerConfig()
        self.rows = build_link_rows(link, self.cfg.target, self.cfg.mcs_set,
                                    self.cfg.overhead_set, self.cfg.bw,
                                    sbi=self.cfg.sbi, bw_set=self.cfg.bw_set,
                                    mode=self.cfg.mode)
        self.snr_ema: float | None = None
        self.pl_var_ema: float = 0.0        # EWMA of path-loss variance (fade depth)
        self.cur: OpPoint | None = None
        self.shed = False
        self._last_change_ms = -1 << 60
        self._last_downgrade_ms = -1 << 60
        self._last_feedback_ms = -1 << 60
        self._rung_block: dict[int, float] = {}   # bw -> blocked-until (ms)
        self._now_ms: float = -1 << 60            # last update() time, for _best
        self.primary_dirty = False                # all rungs bad incl. narrowest
        self._mcs_block: dict[int, float] = {}    # mcs -> blocked-until (ms)
        self._mcs_stats: dict = {}                # mcs -> score.ProbeStat
        self._frame_req_cache: dict[float, float] = {}
        self._active_trip = 0                     # consecutive condemned reports

    # --- estimation -------------------------------------------------------- #
    def _path_loss(self, reported_snr: float, reported_txagc: int) -> float:
        """Channel SNR with TX power removed: snr = path_loss + gain(txagc)."""
        return reported_snr - self.calib.gain_db(reported_txagc)

    def _rung_blocked(self, bw: int) -> bool:
        until = self._rung_block.get(bw)
        return until is not None and self._now_ms < until

    def report_rung_delivery(self, stats: dict[int, tuple[float, int]],
                             now_ms: float) -> None:
        """Feed per-rung probe delivery (bw -> (delivery, n), score.RungWindow
        .stats()). Blocks rungs whose delivery contrasts badly with the best
        rung's (interference on that rung's extra spectrum); sets
        `primary_dirty` when every sampled rung — the narrowest included —
        misses the target (nothing a bandwidth choice can fix)."""
        usable = {bw: d for bw, (d, n) in stats.items()
                  if n >= self.cfg.rung_min_samples}
        if not usable:
            return
        best = max(usable.values())
        narrowest = min(usable)
        for bw, d in usable.items():
            if bw != narrowest and d < best - self.cfg.rung_block_delta:
                self._rung_block[bw] = now_ms + self.cfg.rung_block_hold_ms
        self.primary_dirty = (len(usable) >= 2 and
                              all(d < self.cfg.target -
                                  self.cfg.rung_block_delta
                                  for d in usable.values()))

    # --- adjacent-MCS probe evidence ---------------------------------------- #
    def _mcs_blocked(self, mcs: int) -> bool:
        until = self._mcs_block.get(mcs)
        return until is not None and self._now_ms < until

    def _required_frame_delivery(self, overhead: float) -> float:
        """Raw FCS-clean frame delivery this overhead's FEC needs to hit the
        block-delivery target — the exact inverse of the plain (no-SBI) branch
        of fec_ab_sim.sim_interframe, on the same generation geometry the link
        rows were built from. Conservative on purpose: SBI salvage only makes
        the real requirement laxer."""
        key = round(overhead, 3)
        if key in self._frame_req_cache:
            return self._frame_req_cache[key]
        import link_model as _lm
        frames = getattr(self.link, "frames_per_gen", 12)
        n_sub = getattr(self.link, "n_sub", _lm.N_SUB)
        total = frames * n_sub
        src = max(1, min(total, round(total / (1.0 + overhead))))
        if src >= total:                 # no room for parity: unattainable
            self._frame_req_cache[key] = req = 1.01
            return req
        k_frames = math.ceil(src / n_sub)

        def block_ok(p: float) -> float:
            return sum(math.comb(frames, i) * p ** i * (1 - p) ** (frames - i)
                       for i in range(k_frames, frames + 1))

        lo, hi = 0.0, 1.0
        for _ in range(40):              # bisect the smallest clearing p
            mid = (lo + hi) / 2
            if block_ok(mid) >= self.cfg.target:
                hi = mid
            else:
                lo = mid
        self._frame_req_cache[key] = hi
        return hi

    def report_mcs_delivery(self, stats: dict, now_ms: float) -> None:
        """Feed per-MCS measured delivery (mcs -> score.ProbeStat: the adjacent
        candidates from the probe schedule plus the ACTIVE row from the main
        stream). An MCS whose UPPER confidence bound falls short of even the
        heaviest overhead's raw frame-delivery requirement cannot meet the SLA
        at any FEC — block it for a hold. The lowest rate is never blocked: it
        is the fallback. Candidates block on first sufficient evidence; the
        active row must stay condemned for `mcs_probe_active_trip_ticks`
        consecutive reports (its full-rate window fills in fractions of a
        second, so a single interference burst would otherwise read as a dead
        rate). A tripped active row makes cur_ok fail on the next decide (the
        measured fast-down the model cannot veto) and the block keeps _best
        from re-picking the row the model still believes in."""
        self._mcs_stats = dict(stats)
        req = self._required_frame_delivery(max(self.cfg.overhead_set))
        lowest = min(self.cfg.mcs_set)
        active = self.cur.mcs if self.cur is not None else None
        active_bad = False
        for mcs, st in stats.items():
            if mcs == lowest:
                continue
            if st.attempts >= self.cfg.mcs_probe_min_samples and st.ucb < req:
                if mcs == active:
                    active_bad = True
                    if self._active_trip + 1 < self.cfg.mcs_probe_active_trip_ticks:
                        continue
                self._mcs_block[mcs] = now_ms + self.cfg.mcs_probe_block_hold_ms
        self._active_trip = self._active_trip + 1 if active_bad else 0

    def _mcs_gate_ok(self, r) -> bool:
        """Probe gate for one row (only rows ABOVE the current MCS): admissible
        when fresh probe evidence's LOWER bound clears the row's raw
        frame-delivery requirement. Probes fly at the commanded bandwidth, so
        evidence does not transfer to another bw — those rows wait for the
        stepwise path (bw move at same MCS, then MCS with fresh evidence)."""
        if not self.cfg.mcs_probe_enabled or self.cur is None \
                or r.mcs <= self.cur.mcs:
            return True
        if r.bw != self.cur.bw:
            return False
        st = self._mcs_stats.get(r.mcs)
        if st is None or st.attempts < self.cfg.mcs_probe_min_samples:
            return False
        return st.lcb >= (self._required_frame_delivery(r.overhead)
                          + self.cfg.mcs_probe_margin)

    def _best(self, path_loss: float, margin: float,
              gate_up: bool = False) -> OpPoint | None:
        best = None
        for r in self.rows:
            if self._rung_blocked(r.bw) or self._mcs_blocked(r.mcs):
                continue
            if gate_up and not self._mcs_gate_ok(r):
                continue
            op = resolve(r, path_loss, self.calib, self.link,
                         self.cfg.payload_bytes, self.cfg.src_bitrate_bps, margin)
            # skip rows that can't clear the SLA OR can't carry the bitrate (e_bit=inf)
            if op is None or op.p_deliver < self.cfg.target or op.e_bit == float("inf"):
                continue
            if best is None or op.e_bit < best.e_bit:
                best = op
        return best

    # --- main tick --------------------------------------------------------- #
    def update(self, reported_snr: float, reported_txagc: int,
               now_ms: float) -> OpPoint | None:
        """Apply one VRX feedback sample; return the chosen op point (or None if
        this layer is shed)."""
        self._last_feedback_ms = now_ms
        self._now_ms = now_ms
        # EWMA the PATH LOSS (channel SNR with TX power removed), not the SNR —
        # SNR moves with our own TXAGC, path loss doesn't. Asymmetric: track a
        # worsening channel fast (raise power in time), an improving one slow. Also
        # track the path-loss variance (fade depth) for the optional fade margin.
        pl_inst = self._path_loss(reported_snr, reported_txagc)
        if self.snr_ema is None:
            self.snr_ema = pl_inst
        else:
            dev = pl_inst - self.snr_ema
            a = self.cfg.ema_alpha_down if pl_inst < self.snr_ema else self.cfg.ema_alpha
            self.snr_ema = (1 - a) * self.snr_ema + a * pl_inst
            b = self.cfg.fade_var_alpha
            self.pl_var_ema = (1 - b) * self.pl_var_ema + b * dev * dev
        path_loss = self.snr_ema
        op = self._decide(path_loss, now_ms)
        # Fade margin (opt-in) is applied as power-only TXAGC headroom here, so it
        # never perturbs the energy-min MCS/FEC choice above.
        return self._fade_headroom(op, path_loss)

    def _decide(self, path_loss: float, now_ms: float) -> OpPoint | None:
        """Energy-min operating-point decision (hysteresis, shed/failsafe)."""
        # Is the current pick still clearing (no margin = fast-down threshold)?
        cur_ok = False
        if self.cur is not None and not self.shed:
            cur_now = resolve(op_table.LinkRow(self.cur.mode, self.cur.mcs,
                              self.cur.bw, self.cur.sgi, self.cur.overhead,
                              self.cur.snr_req), path_loss, self.calib, self.link,
                              self.cfg.payload_bytes, self.cfg.src_bitrate_bps, 0.0)
            cur_ok = (cur_now is not None and cur_now.p_deliver >= self.cfg.target
                      and not self._rung_blocked(self.cur.bw)
                      and not self._mcs_blocked(self.cur.mcs))
            if cur_ok:
                self.cur = cur_now            # refresh txagc/e_bit at new path loss

        # Candidate with hysteresis. The probe gate rides only while the current
        # point still works — an emergency downgrade must take the model's word
        # (waiting for probe samples to escape a failing point would be
        # backwards), and the cold start (cur None) is model-trusted: a bad
        # first pick self-corrects through the existing fast-down, after which
        # the probes prevent re-entry.
        cand = self._best(path_loss, self.cfg.margin_db, gate_up=cur_ok)

        if cand is None:
            # nothing clears even with margin: keep current if it still works,
            # else shed (enhancement) or fall to MAX_RANGE (base).
            if cur_ok:
                return self.cur
            if self.cfg.allow_shed:
                self.shed = True
                self.cur = None
                return None
            self.cur = self._failsafe_point()
            return self.cur

        self.shed = False
        if self.cur is None or self.shed:
            return self._commit(cand, now_ms)

        # rate-limit
        if now_ms - self._last_change_ms < self.cfg.min_between_changes_ms and cur_ok:
            return self.cur
        is_upgrade = cand.e_bit < self.cur.e_bit
        if is_upgrade:
            # slow up: only to a meaningfully cheaper point, after the post-downgrade
            # hold, while the current point still works (it cleared with margin).
            if cur_ok and cand.e_bit > self.cur.e_bit * (1 - self.cfg.improve_frac):
                return self.cur
            if cur_ok and now_ms - self._last_downgrade_ms < self.cfg.hold_after_downgrade_ms:
                return self.cur
            return self._commit(cand, now_ms)
        # cand is same-or-more-expensive. Keep the current point unless it is
        # actually FAILING (fast down only on real degradation — no needless churn).
        if not cur_ok:
            self._last_downgrade_ms = now_ms
            return self._commit(cand, now_ms)
        return self.cur

    def on_tick(self, now_ms: float) -> OpPoint | None:
        """Call between feedback samples to enforce the failsafe timeout."""
        if now_ms - self._last_feedback_ms > self.cfg.feedback_timeout_ms:
            self.shed = False
            self.cur = self._failsafe_point()
        return self.cur

    def reset_operating_point(self) -> None:
        """Forget the committed point — the next update makes a fresh,
        model-trusted pick — while KEEPING the measured MCS blocks: a
        reacquisition cold pick must not walk straight back into a row whose
        measured delivery just condemned it. The VRX calls this when it loses
        the video entirely (an operating point it cannot hear is not worth
        keeping, and its own estimator is starved of the evidence to fix it)."""
        self.cur = None
        self.shed = False
        self._active_trip = 0

    def _failsafe_point(self) -> OpPoint:
        return MAX_RANGE

    def _commit(self, op: OpPoint, now_ms: float) -> OpPoint:
        # Probe evidence is bound to the (mode, bw) it was measured in — a
        # committed move to another context invalidates blocks and stats (the
        # VRX-side window resets itself on the same transition).
        if self.cur is None or (op.mode, op.bw) != (self.cur.mode, self.cur.bw):
            self._mcs_block.clear()
            self._mcs_stats = {}
        if self.cur is None or op.mcs != self.cur.mcs:
            self._active_trip = 0
        self.cur = op
        self._last_change_ms = now_ms
        return op

    def _fade_headroom(self, op: OpPoint | None, path_loss: float) -> OpPoint | None:
        """Add power-only TXAGC headroom for fade resilience (opt-in via
        fade_margin_k). Keeps the energy-min MCS/FEC choice and leaves `self.cur`
        (hysteresis state) untouched — only the op actually applied on-air gets the
        extra power, so deep fades are covered without spending airtime."""
        if op is None or self.cfg.fade_margin_k <= 0.0:
            return op
        headroom_db = min(self.cfg.fade_margin_max_db,
                          self.cfg.fade_margin_k * self.pl_var_ema ** 0.5)
        if headroom_db <= 0.0:
            return op
        new_txagc = self.calib.min_txagc_for_gain(
            self.calib.gain_db(op.txagc) + headroom_db)
        if new_txagc is None or new_txagc <= op.txagc:
            return op
        recv = path_loss + self.calib.gain_db(new_txagc)
        pdel = self.link.p_deliver(recv, op.mcs, op.overhead)
        eb = em.energy_per_delivered_bit(
            em.TxPoint(op.mode, op.mcs, op.bw, op.sgi, new_txagc),
            self.cfg.src_bitrate_bps, op.overhead, self.cfg.payload_bytes, pdel,
            self.calib)
        return OpPoint(op.mode, op.mcs, op.bw, op.sgi, new_txagc, op.overhead,
                       op.snr_req, eb, pdel)


class BiasedLink:
    """A link model whose per-MCS belief is shifted by `bias_db` dB (NEGATIVE =
    the model believes the rate works at that much less SNR than it does).
    The bias runs through BOTH faces of the model — `snr_required` (row
    thresholds) and `p_deliver` (the SLA check in resolve/cur_ok) — because a
    real miscalibration is a wrong belief, not just a wrong table entry: with
    only the rows shifted, resolve()'s own p_deliver veto would silently
    un-bias the controller."""

    def __init__(self, base, bias_db: dict[int, float]):
        self.base = base
        self.bias = {int(m): float(db) for m, db in bias_db.items()}
        self.frames_per_gen = getattr(base, "frames_per_gen", 12)

    def p_deliver(self, snr_db, mcs, overhead, sbi=True):
        return self.base.p_deliver(snr_db - self.bias.get(mcs, 0.0), mcs,
                                   overhead, sbi)

    def snr_required(self, mcs, overhead, target, sbi=True, **kw):
        return (self.base.snr_required(mcs, overhead, target, sbi=sbi, **kw)
                + self.bias.get(mcs, 0.0))


def apply_model_bias(ctrl: Controller, bias_db: dict[int, float]) -> None:
    """Deliberately miscalibrate a controller's model per MCS (dB; negative =
    optimistic) and rebuild its rows from the biased belief. The lever the
    probe A/B harnesses measure against — the probes' whole job is to catch
    exactly this."""
    ctrl.link = BiasedLink(ctrl.link, bias_db)
    ctrl.rows = build_link_rows(ctrl.link, ctrl.cfg.target, ctrl.cfg.mcs_set,
                                ctrl.cfg.overhead_set, ctrl.cfg.bw,
                                sbi=ctrl.cfg.sbi, bw_set=ctrl.cfg.bw_set,
                                mode=ctrl.cfg.mode)


# --------------------------------------------------------------------------- #
# SVC per-layer UEP: a bank of controllers, one per temporal layer. Base/IDR
# layers hold a high delivery target and never shed; enhancement layers carry a
# lower target, are restricted to cheap (high-MCS/low-FEC) rows, and SHED first
# under SNR stress — the graceful-degradation staircase. The PA is shared, so the
# commanded TX power is the max any active layer needs; per-layer MCS+FEC still
# differ (encoded in the SVC ladder).
# --------------------------------------------------------------------------- #
@dataclass
class LayerSpec:
    stream_id: int
    target: float
    allow_shed: bool
    src_bitrate_bps: float
    overhead_set: tuple = (0.10, 0.25, 0.50, 0.75, 1.00)


def default_svc_layers() -> list:
    """critical / T0 (base, protected) + T1 / T2 (enhancement, shed first).
    Mirrors svc_uep_fec.default_uep_policy's intent on the control side."""
    return [
        LayerSpec(0, target=0.999, allow_shed=False, src_bitrate_bps=0.5e6),  # critical/IDR
        LayerSpec(1, target=0.99, allow_shed=False, src_bitrate_bps=1.0e6),   # T0 base
        LayerSpec(2, target=0.95, allow_shed=True, src_bitrate_bps=1.5e6,
                  overhead_set=(0.10, 0.25, 0.50)),                            # T1
        LayerSpec(3, target=0.90, allow_shed=True, src_bitrate_bps=2.0e6,
                  overhead_set=(0.10, 0.25)),                                  # T2
    ]


class SvcController:
    def __init__(self, link, calib, layers: list | None = None, **ctrl_kw):
        self.layers = layers or default_svc_layers()
        self.ctrls = {}
        for L in self.layers:
            cfg = ControllerConfig(target=L.target, allow_shed=L.allow_shed,
                                   src_bitrate_bps=L.src_bitrate_bps,
                                   overhead_set=L.overhead_set, **ctrl_kw)
            self.ctrls[L.stream_id] = Controller(link, calib, cfg)

    def update(self, reported_snr: float, reported_txagc: int, now_ms: float):
        """Returns (ops_by_sid, shared_txagc, active_sids). A None op = shed layer.
        shared_txagc = the max TX power any active layer needs (one PA).
        Each layer controller tracks its own path-loss variance for the fade
        margin (when enabled), so no extra signal is threaded in."""
        ops = {sid: c.update(reported_snr, reported_txagc, now_ms)
               for sid, c in self.ctrls.items()}
        present = [op for op in ops.values() if op is not None]
        shared_txagc = max((op.txagc for op in present), default=63)
        active = sorted(sid for sid, op in ops.items() if op is not None)
        return ops, shared_txagc, active

    def on_tick(self, now_ms: float):
        for c in self.ctrls.values():
            c.on_tick(now_ms)
