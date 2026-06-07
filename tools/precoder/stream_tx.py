#!/usr/bin/env python3
"""Stream TX driver — wraps a byte stream into precoded PSDU bodies.

Reads bytes from stdin (or --input), chunks them via `stream.pack_stream`,
encodes each frame via `stream.encode_body` (optionally with a shape
constraint sourced from the environment), and writes a sequence of
length-prefixed PSDU bodies to stdout. Piped into the C++ StreamTxDemo:

    python3 tools/precoder/stream_tx.py < data.bin | ./build/StreamTxDemo

Wire format on stdout:  <u32_le length><length bytes of descrambled PSDU>

Environment knobs (all optional):
    DEVOURER_STREAM_SHAPE       e.g. "0:+1,10:-1,20:+1"  — per-subcarrier ±1
                                pin spec; absent / "" = byte mode.
    DEVOURER_STREAM_MTU         payload bytes per frame (default 14, matches
                                a 24-byte byte-mode body).
    DEVOURER_STREAM_BODY_BYTES  floor for body size (default 24); shape mode
                                may grow the body past this when the pin
                                rank reduces per-symbol capacity.
    DEVOURER_STREAM_SEED        scrambler seed for the *encoder's* model
                                (default 0x5D). Must match the chip's actual
                                seed to make the shaped subcarriers appear
                                on-air at the requested ±1; for byte-only
                                use the default works.
    DEVOURER_STREAM_OFFSET      scrambler-phase offset for the body's first
                                bit (default 0; the README's PrecoderDemo
                                placement is 208 = SERVICE(16) + MAC(24·8)).
                                Byte mode ignores this.
    DEVOURER_STREAM_ENTRY_STATE  BCC entry state at the body (default 0).
                                Byte mode ignores this.
    DEVOURER_STREAM_SEQ_START    starting sequence number (default 0).
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
from typing import Optional

# Allow running both via `uv run python tools/precoder/stream_tx.py`
# and directly from inside tools/precoder. When invoked from the repo root,
# add this script's directory to sys.path so `import stream` resolves.
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import stream  # noqa: E402


def parse_shape_env(s: str) -> Optional[dict]:
    """Parse "idx:±1,..." into {idx: ±1}. Empty / None → None (byte mode)."""
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
        # Accept +1 / -1 / + / - / 1 / -1.
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
    ap.add_argument("--input", default=None,
                    help="read bytes from this file instead of stdin")
    ap.add_argument("--shape", default=None,
                    help="override DEVOURER_STREAM_SHAPE")
    ap.add_argument("--mtu", type=int, default=None,
                    help="override DEVOURER_STREAM_MTU")
    ap.add_argument("--body-bytes", type=int, default=None,
                    help="override DEVOURER_STREAM_BODY_BYTES")
    ap.add_argument("--seed", type=lambda s: int(s, 0), default=None,
                    help="override DEVOURER_STREAM_SEED")
    ap.add_argument("--offset", type=int, default=None,
                    help="override DEVOURER_STREAM_OFFSET")
    ap.add_argument("--entry-state", type=lambda s: int(s, 0), default=None,
                    help="override DEVOURER_STREAM_ENTRY_STATE")
    ap.add_argument("--seq-start", type=int, default=None,
                    help="override DEVOURER_STREAM_SEQ_START")
    ap.add_argument("--repeat", type=int, default=1,
                    help="emit each encoded body this many times in sequence "
                         "(RX dedups by seq). Combats early-frame loss during "
                         "the RX adapter's warmup period.")
    ap.add_argument("--dump-bodies", default=None,
                    help="also write the concatenated bodies (without length "
                         "prefixes) to this path — for offline inspection")
    args = ap.parse_args(argv)

    shape_raw = args.shape if args.shape is not None else os.environ.get(
        "DEVOURER_STREAM_SHAPE", "")
    shape = parse_shape_env(shape_raw)
    mtu = args.mtu if args.mtu is not None else _env_int(
        "DEVOURER_STREAM_MTU", stream.DEFAULT_BODY_BYTES - stream.ENVELOPE_LEN)
    body_bytes = (args.body_bytes if args.body_bytes is not None
                  else _env_int("DEVOURER_STREAM_BODY_BYTES",
                                stream.DEFAULT_BODY_BYTES))
    seed = args.seed if args.seed is not None else _env_int(
        "DEVOURER_STREAM_SEED", stream.DEFAULT_SEED)
    offset = args.offset if args.offset is not None else _env_int(
        "DEVOURER_STREAM_OFFSET", 0)
    entry_state = args.entry_state if args.entry_state is not None else _env_int(
        "DEVOURER_STREAM_ENTRY_STATE", 0)
    seq_start = args.seq_start if args.seq_start is not None else _env_int(
        "DEVOURER_STREAM_SEQ_START", 0)

    src = sys.stdin.buffer if args.input is None else open(args.input, "rb")
    data = src.read()
    if args.input is not None:
        src.close()

    frames = stream.pack_stream(data, mtu=mtu, seq_start=seq_start)
    if not frames:
        sys.stderr.write("stream_tx: 0 bytes on input — no frames emitted\n")
        return 0

    out = sys.stdout.buffer
    dump = open(args.dump_bodies, "wb") if args.dump_bodies else None

    repeat = max(1, args.repeat)
    total_bytes = 0
    for f in frames:
        body, layout = stream.encode_body(
            f, shape=shape, body_bytes=body_bytes,
            seed=seed, offset=offset, entry_state=entry_state,
        )
        chunk = struct.pack("<I", len(body)) + body
        for _ in range(repeat):
            out.write(chunk)
            total_bytes += len(body)
        if dump is not None:
            dump.write(body)

    out.flush()
    if dump is not None:
        dump.close()

    # `layout` from the last encode_body call describes the geometry; logged
    # so the caller can sanity-check body size matches StreamTxDemo's max-psdu.
    sys.stderr.write(
        f"stream_tx: emitted {len(frames)} frame(s), {total_bytes} body bytes, "
        f"mode={'shape' if shape else 'byte'}, "
        f"body={layout.body_bytes}B/frame, "
        f"capacity/sym={layout.capacity_per_sym}b, "
        f"payload={len(frames[0].payload)}..{len(frames[-1].payload)}B\n"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
