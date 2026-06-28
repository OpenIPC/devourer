"""VRX link-quality scoring over a sliding window of <devourer-stream> frames.

Feeds the controller an SNR estimate and produces the alink-compatible 1000..2000
score carried in the RCF (telemetry + a future drone-decides mode). The score
blends windowed RSSI/SNR (best chain) and is penalised by the loss the DECODER
actually experiences — i.e. POST-FEC residual loss, not the raw FCS-failure rate,
since SBI sub-block salvage recovers much of the apparent corruption (the key
insight: score the link the decoder sees).
"""

from __future__ import annotations

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
