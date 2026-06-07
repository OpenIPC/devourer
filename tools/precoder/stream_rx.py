#!/usr/bin/env python3
"""Stream RX driver — decodes stream frames out of WiFiDriverDemo's stdout.

Reads WiFiDriverDemo's stdout on this process's stdin, picks up the
`<devourer-stream>` lines emitted when DEVOURER_STREAM_OUT=1 is set, decodes
each body via `stream.decode_body` (optionally with a shape constraint),
re-orders by sequence number, and writes the decoded payload bytes to stdout.
Gaps and duplicates are logged to stderr.

Usage:
    DEVOURER_PID=0x8813 DEVOURER_CHANNEL=6 DEVOURER_STREAM_OUT=1 \
        ./build/WiFiDriverDemo | \
        DEVOURER_STREAM_SHAPE='0:+1,10:-1' \
        python3 tools/precoder/stream_rx.py > received.bin

The env knob set is symmetric with `stream_tx.py` (same shape spec,
DEVOURER_STREAM_SEED / OFFSET / ENTRY_STATE). With no shape set, the RX side
just parses the byte-mode envelope — it never decodes a shape its TX peer
didn't use.

Modes:
    --collect (default): buffer all decoded frames until EOF / --total / a
        gap-timeout, then emit payloads in sequence order. Bounded latency,
        correct ordering, lossless when no gaps.
    --realtime: emit each newly-decoded payload as it arrives; out-of-order
        arrivals appear out of order. Useful for live demos.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import time
from typing import Optional

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import stream  # noqa: E402

# Mirrors stream_tx.py's parser without re-importing it (kept side-effect free).
_STREAM_RE = re.compile(
    r"<devourer-stream>rate=(?P<rate>\d+) len=(?P<len>\d+) body=(?P<hex>[0-9a-fA-F]*)"
)


def parse_shape_env(s: str) -> Optional[dict]:
    if not s:
        return None
    out: dict[int, int] = {}
    for tok in s.split(","):
        tok = tok.strip()
        if not tok:
            continue
        if ":" not in tok:
            raise ValueError(f"shape token {tok!r} missing ':' (want idx:±1)")
        k, v = tok.split(":", 1)
        k = int(k.strip(), 0)
        v = v.strip()
        if v in ("+1", "+", "1"):
            sign = +1
        elif v in ("-1", "-"):
            sign = -1
        else:
            raise ValueError(f"shape value {v!r} (token {tok!r}); want ±1")
        if k in out:
            raise ValueError(f"duplicate subcarrier index {k} in shape")
        out[k] = sign
    return out or None


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    return int(raw, 0)


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--shape", default=None,
                    help="override DEVOURER_STREAM_SHAPE")
    ap.add_argument("--seed", type=lambda s: int(s, 0), default=None,
                    help="override DEVOURER_STREAM_SEED")
    ap.add_argument("--offset", type=int, default=None,
                    help="override DEVOURER_STREAM_OFFSET")
    ap.add_argument("--entry-state", type=lambda s: int(s, 0), default=None,
                    help="override DEVOURER_STREAM_ENTRY_STATE")
    ap.add_argument("--realtime", action="store_true",
                    help="emit decoded payloads as they arrive (no reorder)")
    ap.add_argument("--total", type=int, default=0,
                    help="stop after N distinct frames (default: rely on the "
                         "first frame's TOTAL field, fall back to EOF)")
    ap.add_argument("--idle-timeout", type=float, default=0.0,
                    help="exit when no new frame for this many seconds "
                         "(default 0 = no timeout)")
    args = ap.parse_args(argv)

    shape_raw = args.shape if args.shape is not None else os.environ.get(
        "DEVOURER_STREAM_SHAPE", "")
    shape = parse_shape_env(shape_raw)
    seed = args.seed if args.seed is not None else _env_int(
        "DEVOURER_STREAM_SEED", stream.DEFAULT_SEED)
    offset = args.offset if args.offset is not None else _env_int(
        "DEVOURER_STREAM_OFFSET", 0)
    entry_state = args.entry_state if args.entry_state is not None else _env_int(
        "DEVOURER_STREAM_ENTRY_STATE", 0)

    by_seq: dict[int, bytes] = {}
    total_target = args.total
    rx_total = 0
    dup_count = 0
    bad_count = 0
    rate_mismatch = 0
    last_event = time.monotonic()

    out_bytes = sys.stdout.buffer

    for line in sys.stdin:
        m = _STREAM_RE.search(line)
        if not m:
            continue
        rate = int(m.group("rate"))
        if rate != 0x04:
            # Tier-1 sanity: every shaped frame must fly as legacy 6M OFDM. A
            # 0x00 here means the chip downgraded us to 1M CCK and OFDM
            # subcarriers don't exist at all (see precoder README).
            rate_mismatch += 1
        body = bytes.fromhex(m.group("hex"))
        last_event = time.monotonic()

        frame = stream.decode_body(
            body, shape=shape, seed=seed, offset=offset, entry_state=entry_state,
        )
        if frame is None:
            bad_count += 1
            continue
        if frame.seq in by_seq:
            dup_count += 1
            continue
        by_seq[frame.seq] = frame.payload
        rx_total += 1
        if total_target == 0 and frame.total:
            total_target = frame.total
        if args.realtime:
            out_bytes.write(frame.payload)
            out_bytes.flush()

        if total_target and rx_total >= total_target:
            break
        if args.idle_timeout and (time.monotonic() - last_event) > args.idle_timeout:
            break

    if not args.realtime:
        # Emit in ascending-seq order. Gaps don't stop us — we write what we
        # have and report the gap on stderr; caller decides whether to retry.
        for seq in sorted(by_seq):
            out_bytes.write(by_seq[seq])
        out_bytes.flush()

    # Report.
    seen = sorted(by_seq)
    gaps: list[tuple[int, int]] = []
    if seen:
        for i in range(seen[0], seen[-1] + 1):
            if i not in by_seq:
                if gaps and gaps[-1][1] == i - 1:
                    gaps[-1] = (gaps[-1][0], i)
                else:
                    gaps.append((i, i))
    sys.stderr.write(
        f"stream_rx: rx={rx_total} unique frame(s), "
        f"dup={dup_count}, malformed={bad_count}, "
        f"rate-mismatch={rate_mismatch}"
    )
    if total_target:
        sys.stderr.write(f", target={total_target}")
    if gaps:
        sys.stderr.write(
            ", gaps=" + ",".join(
                f"{a}" if a == b else f"{a}-{b}" for a, b in gaps[:8]
            )
            + ("…" if len(gaps) > 8 else "")
        )
    sys.stderr.write("\n")
    return 0 if (rx_total > 0 and not gaps and rate_mismatch == 0) else 1


if __name__ == "__main__":
    raise SystemExit(main())
