"""stdin control TLVs for the stream demos (examples/common/stream_stdin.h).

Imports only the standard library on purpose: adaptive_link.py and the
on-air orchestrators run under interpreters without numpy, while stream.py
(the body codec) needs it. stream.py re-exports these names for the
producers.
"""
from __future__ import annotations

import struct
import time

# A length word with the top bit set is a control TLV <op:u8><args>, not a
# PSDU. SET_* drive the duplex binary's live knobs; CAPTURE_TS tells streamtx
# / duplex the producer's capture time of the NEXT record (CLOCK_MONOTONIC ns,
# same host), which the TX turns into the per-frame capture->send field.
CTL_FLAG = 0x80000000
SET_PWR, SET_RATE, SET_CHAN, CAPTURE_TS = 1, 2, 3, 4


def ctl_frame(op: int, payload: bytes = b"") -> bytes:
    body = bytes([op]) + payload
    return struct.pack("<I", CTL_FLAG | len(body)) + body


def psdu_frame(body: bytes) -> bytes:
    return struct.pack("<I", len(body)) + body


def capture_ts_frame(ns: int | None = None) -> bytes:
    """CAPTURE_TS for the next record; ns defaults to time.monotonic_ns()."""
    if ns is None:
        ns = time.monotonic_ns()
    return ctl_frame(CAPTURE_TS, struct.pack("<Q", ns))


class CaptureStamper:
    """Prefix each record with CAPTURE_TS when enabled. `delay_every` /
    `delay_ms` inject a known producer delay between the stamp and the record
    on every Nth record — the on-air check that the TX's capture->send field
    tracks the producer, not the pipe (tests/stream_timing_onair.sh)."""

    def __init__(self, enabled: bool, delay_every: int = 0, delay_ms: float = 0.0):
        self.enabled = enabled
        self.delay_every = max(0, delay_every)
        self.delay_ms = delay_ms
        self.n = 0

    def prefix(self) -> bytes:
        if not self.enabled:
            return b""
        self.n += 1
        out = capture_ts_frame()
        if self.delay_every and self.n % self.delay_every == 0 and self.delay_ms > 0:
            time.sleep(self.delay_ms / 1000.0)
        return out


def add_capture_args(ap) -> None:
    ap.add_argument("--capture-ts", action="store_true",
                    help="prefix every record with a CAPTURE_TS control TLV "
                         "(stamped at emission; a real encoder stamps at capture)")
    ap.add_argument("--capture-delay", default=None, metavar="N:MS",
                    help="with --capture-ts: sleep MS ms between stamp and record "
                         "on every Nth record (validation of the TX-side c2s field)")


def capture_stamper_from_args(args) -> "CaptureStamper":
    every, ms = 0, 0.0
    if getattr(args, "capture_delay", None):
        n, m = args.capture_delay.split(":")
        every, ms = int(n), float(m)
    return CaptureStamper(bool(getattr(args, "capture_ts", False)), every, ms)
