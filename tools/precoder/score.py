"""VRX link-quality scoring over a sliding window of `rx.frame` events.

Feeds the controller an SNR estimate and produces the alink-compatible 1000..2000
score carried in the RCF (telemetry + a future drone-decides mode). The score
blends windowed RSSI/SNR (best chain) and is penalised by the loss the DECODER
actually experiences — i.e. POST-FEC residual loss, not the raw FCS-failure rate,
since SBI sub-block salvage recovers much of the apparent corruption (the key
insight: score the link the decoder sees).
"""

from __future__ import annotations

import math
import threading
from collections import deque
from dataclasses import dataclass, field


@dataclass
class ScoreConfig:
    window_s: float = 0.5
    rssi_lo: float = -80.0      # -> score 1000
    rssi_hi: float = -40.0      # -> score 2000
    snr_lo: float = 5.0         # dB -> 1000
    snr_hi: float = 30.0        # dB -> 2000
    rssi_weight: float = 0.3
    snr_weight: float = 0.7
    loss_penalty: float = 1000.0  # score points subtracted per unit residual loss


def _lin(x: float, lo: float, hi: float) -> float:
    if hi == lo:
        return 1000.0
    return 1000.0 + 1000.0 * max(0.0, min(1.0, (x - lo) / (hi - lo)))


class ScoreWindow:
    def __init__(self, cfg: ScoreConfig | None = None):
        self.cfg = cfg or ScoreConfig()
        self._frames: deque = deque()        # (t, rssi, snr, crc_err, seq)
        self._max_seq_seen = None

    def add_frame(self, rssi: float, snr: float, crc_err: bool, seq: int,
                  now_s: float) -> None:
        self._frames.append((now_s, rssi, snr, bool(crc_err), seq))
        self._max_seq_seen = seq if self._max_seq_seen is None else max(self._max_seq_seen, seq)
        cutoff = now_s - self.cfg.window_s
        while self._frames and self._frames[0][0] < cutoff:
            self._frames.popleft()

    def n(self) -> int:
        return len(self._frames)

    def snr_estimate(self) -> float | None:
        """Windowed mean SNR (best chain), for the controller. None if empty."""
        if not self._frames:
            return None
        return sum(f[2] for f in self._frames) / len(self._frames)

    def rssi_estimate(self) -> float | None:
        if not self._frames:
            return None
        return sum(f[1] for f in self._frames) / len(self._frames)

    def fcs_loss(self) -> float:
        """Raw fraction of windowed frames with crc_err."""
        if not self._frames:
            return 0.0
        return sum(1 for f in self._frames if f[3]) / len(self._frames)

    def seq_gap_loss(self) -> float:
        """Fraction of expected frames missing, from 802.11 sequence gaps.

        Unwraps the 12-bit sequence in ARRIVAL order (summing forward, wrap-safe
        deltas) so a 4096 wrap inside the window is not mistaken for a giant gap —
        the previous sort + (max-min) made any wrap-straddling window read as
        ~100% loss.

        CAVEAT: 802.11 seq advances per *transmitted* frame (retries, other
        traffic, fragments), not per delivered application frame, so this
        OVER-reports loss whenever the received seq stream isn't a clean
        per-video-frame counter. Treat it as an upper bound; the truer on-air
        delivery is received/sent frame counts (an app-level sequence in the
        payload would make this exact)."""
        if len(self._frames) < 2:
            return 0.0
        seqs = [f[4] for f in self._frames]        # arrival order (NOT sorted)
        span = 1
        for a, b in zip(seqs, seqs[1:]):
            span += (b - a) % 4096                 # forward, wrap-safe distance
        return max(0.0, 1.0 - len(seqs) / span) if span else 0.0

    def ack_seq(self) -> int:
        return self._max_seq_seen or 0

    def score(self, residual_loss: float | None = None) -> int:
        """alink-compatible 1000..2000. `residual_loss` = post-FEC loss fraction
        (from FusedFecReceiver); if None, falls back to seq-gap loss."""
        if not self._frames:
            return 1000
        rssi_s = _lin(self.rssi_estimate(), self.cfg.rssi_lo, self.cfg.rssi_hi)
        snr_s = _lin(self.snr_estimate(), self.cfg.snr_lo, self.cfg.snr_hi)
        s = self.cfg.rssi_weight * rssi_s + self.cfg.snr_weight * snr_s
        loss = self.seq_gap_loss() if residual_loss is None else residual_loss
        s -= self.cfg.loss_penalty * loss
        return int(max(1000, min(2000, s)))


class RungWindow:
    """Per-rung (bandwidth) delivery from the video seq stream.

    Both ends derive each seq's rung from the shared probe schedule
    (rc_proto.probe_bw), so no wire fields are needed: received seqs are
    attributed on arrival, missed seqs are inferred from the wrap-safe gaps
    between arrivals and attributed the same way. Only scheduled probe seqs
    are tracked — the commanded-op frames tell us nothing about the OTHER
    rungs, which is the whole point of probing.
    """

    def __init__(self, bw_set, samples_per_rung: int = 24):
        import rc_proto as _rp
        self._probe_bw = _rp.probe_bw
        self.bw_set = tuple(sorted(bw_set))
        self._hist: dict[int, deque] = {bw: deque(maxlen=samples_per_rung)
                                        for bw in self.bw_set}
        self._last_seq: int | None = None

    def _attribute(self, seq: int, ok: bool) -> None:
        bw = self._probe_bw(seq, self.bw_set)
        if bw is not None:
            self._hist[bw].append(ok)

    def add_seq(self, seq: int) -> None:
        """One RECEIVED video seq; the (wrap-safe) gap since the previous
        arrival is attributed as losses."""
        if self._last_seq is not None:
            gap = (seq - self._last_seq) % 4096
            # walk the missing seqs (bounded: a monster gap is a link outage,
            # not per-rung information — cap the walk at one probe period x4)
            for d in range(1, min(gap, 128)):
                self._attribute((self._last_seq + d) % 4096, ok=False)
        self._attribute(seq, ok=True)
        self._last_seq = seq

    def stats(self) -> dict[int, tuple[float, int]]:
        """bw -> (delivery fraction, n samples) for rungs with any samples."""
        out = {}
        for bw, h in self._hist.items():
            if h:
                out[bw] = (sum(h) / len(h), len(h))
        return out


# One-sided normal quantiles for the Wilson bounds (each bound is used as its
# own one-sided test: promote on the LOWER bound, block on the UPPER).
_Z_ONE_SIDED = {0.90: 1.2816, 0.95: 1.6449, 0.99: 2.3263}


def wilson_bounds(successes: int, n: int, z: float) -> tuple[float, float]:
    """Wilson-score interval for a binomial proportion — (lcb, ucb).

    Unlike the bare mean, the bounds stay honest at small n: a perfect record
    caps at lcb = n/(n+z²), so "0 failures in 12" cannot read as proof of 99%
    delivery. (0, n) -> (0, 1) when there is no evidence at all."""
    if n <= 0:
        return 0.0, 1.0
    ph = successes / n
    z2 = z * z
    denom = 1.0 + z2 / n
    center = ph + z2 / (2 * n)
    half = z * math.sqrt(ph * (1.0 - ph) / n + z2 / (4.0 * n * n))
    return (max(0.0, (center - half) / denom),
            min(1.0, (center + half) / denom))


@dataclass(frozen=True)
class ProbeStat:
    """Per-candidate-MCS probe evidence: raw FCS-clean frame delivery with
    Wilson confidence bounds and the staleness of the newest sample."""
    delivery: float
    successes: int
    attempts: int
    lcb: float
    ucb: float
    age_ms: float


class McsProbeWindow:
    """Per-MCS raw delivery from the scheduled rate probes AND the active row.

    The rc_proto.probe_mcs schedule names which seqs fly the adjacent rates, so
    like RungWindow no wire fields are needed — but rate evidence is regime-
    dependent (the candidates move with the commanded profile), so attribution
    is guarded three ways where RungWindow needs none:

      * successes are RATE-VERIFIED: a received probe-slot seq counts only when
        the PHY-decoded (mode, mcs) matches the expected candidate — a frame
        that demonstrably flew elsewhere (command lag, suppressed probe, spec
        parse fallback) is ignored, never mis-credited;
      * gap losses are EPOCH-GATED: attributed only while a non-probe frame has
        confirmed the VTX is flying the commanded rate (and a mismatch
        un-confirms), so losses around a profile transition or a failsafe
        overlap are dropped rather than blamed on the wrong candidate;
      * crc_err frames never advance the gap walk and never attribute BY SEQ —
        their body is untrusted bits — but their PHY-decoded rate is from the
        descriptor (pre-FCS), so a corrupt frame at the selected or an
        adjacent-candidate rate IS a rate-verified failure of that rate and is
        pushed seq-free. This is what keeps the evidence flowing in the
        crc-flooded regimes where escape speed matters most; the same lost
        frame may additionally be counted by the next clean frame's gap walk,
        an accepted over-weighting of visible corruption in the conservative
        direction.

    The SELECTED row is attributed by the same rules from every non-probe seq
    (bandwidth-probe slots excluded when `bw_set` is given — those fly the
    selected rate at another bandwidth, so their loss is bandwidth evidence,
    not rate evidence). That makes the active row's measured delivery free —
    the main stream is its probe — and it is what lets the controller escape
    an operating point its model wrongly believes in: SNR-blind failure
    (interference, a miscalibrated row) shows up here when no model does.

    set_selected() (any commanded (mode, mcs, bw) change) resets the window —
    evidence from another operating context must not authorize a promotion.
    Thread-safe: add_seq runs on the RX reader thread while stats()/
    set_selected() run on the control loop.
    """

    def __init__(self, mcs_set, mode: str = "ht", samples_per_mcs: int = 64,
                 max_age_ms: float = 8000.0, confidence: float = 0.90,
                 period: int | None = None, slots=None, bw_set=None):
        import rc_proto as _rp
        self._rp = _rp
        self.mcs_set = tuple(sorted(mcs_set))
        self._period = period if period is not None else _rp.MCS_PROBE_PERIOD
        self._slots = tuple(slots) if slots is not None else _rp._MCS_PROBE_SLOTS
        self._bw_set = tuple(sorted(bw_set)) if bw_set else None
        self._maxlen = samples_per_mcs
        self.max_age_ms = max_age_ms
        self._z = _Z_ONE_SIDED.get(round(confidence, 2), 1.2816)
        self._hist: dict[int, deque] = {}     # mcs -> deque[(t_ms, ok)]
        self._mode = mode
        self._selected: int | None = None
        self._epoch_confirmed = False
        self._last_seq: int | None = None
        self._lock = threading.Lock()

    def _probe(self, seq: int) -> int | None:
        return self._rp.probe_mcs(seq, self._selected, self.mcs_set,
                                  self._period, self._slots)

    def _slot_mcs(self, seq: int) -> int | None:
        """The MCS this seq is scheduled to fly, or None for a seq that is no
        rate evidence at all (a bandwidth-probe slot)."""
        cand = self._probe(seq)
        if cand is not None:
            return cand
        if self._bw_set is not None and \
                self._rp.probe_bw(seq, self._bw_set) is not None:
            return None
        return self._selected

    def _tracked(self) -> tuple:
        """The rates evidence is currently collected for: the selected row and
        its adjacent-in-set candidates."""
        rates = self.mcs_set
        if self._selected not in rates:
            return (self._selected,)
        i = rates.index(self._selected)
        out = [self._selected]
        if i + 1 < len(rates):
            out.append(rates[i + 1])
        if i > 0:
            out.append(rates[i - 1])
        return tuple(out)

    def set_selected(self, mode: str, mcs: int, now_ms: float) -> None:
        """Commanded operating point changed: blank the window (new epoch)."""
        with self._lock:
            self._mode = mode
            self._selected = mcs
            self._hist.clear()
            self._epoch_confirmed = False
            self._last_seq = None

    def _push(self, mcs: int, now_ms: float, ok: bool) -> None:
        h = self._hist.get(mcs)
        if h is None:
            h = self._hist[mcs] = deque(maxlen=self._maxlen)
        h.append((now_ms, ok))

    def add_seq(self, seq: int, now_ms: float, rate: tuple | None = None,
                crc_err: bool = False) -> None:
        """One received video seq. `rate` is the PHY-decoded (mode, mcs) of the
        frame (devourer_events.desc_rate_to_mcs), or None when unavailable —
        rate-less frames are dropped (fail-inert: no admission, no blocking)."""
        with self._lock:
            if self._selected is None:
                return
            if crc_err:
                # the descriptor rate survives body corruption: a corrupt
                # frame at a rate we track is that rate's measured failure
                if rate is not None and rate[0] == self._mode and \
                        rate[1] in self._tracked():
                    self._push(rate[1], now_ms, False)
                return
            if self._last_seq is not None and self._epoch_confirmed:
                gap = (seq - self._last_seq) % 4096
                # walk the missing seqs (bounded like RungWindow: a monster gap
                # is a link outage, not per-candidate information)
                for d in range(1, min(gap, 128)):
                    m = self._slot_mcs((self._last_seq + d) % 4096)
                    if m is not None:
                        self._push(m, now_ms, False)
            cand = self._probe(seq)
            if cand is None:
                if rate is not None:
                    self._epoch_confirmed = (rate == (self._mode, self._selected))
                    if self._epoch_confirmed and self._slot_mcs(seq) is not None:
                        self._push(self._selected, now_ms, True)
            elif rate is not None and rate == (self._mode, cand):
                self._push(cand, now_ms, True)
            self._last_seq = seq

    def stats(self, now_ms: float) -> dict[int, ProbeStat]:
        """mcs -> ProbeStat for candidates with any fresh samples."""
        out = {}
        with self._lock:
            for mcs, h in self._hist.items():
                while h and h[0][0] < now_ms - self.max_age_ms:
                    h.popleft()
                if not h:
                    continue
                n = len(h)
                s = sum(1 for _, ok in h if ok)
                lcb, ucb = wilson_bounds(s, n, self._z)
                out[mcs] = ProbeStat(delivery=s / n, successes=s, attempts=n,
                                     lcb=lcb, ucb=ucb,
                                     age_ms=now_ms - h[-1][0])
        return out
