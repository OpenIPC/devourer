#!/usr/bin/env python3
"""Fused-FEC TX driver — bytes → RS+SBI bodies → streamtx.

Reads a byte stream (stdin or --input), Reed-Solomon-encodes it and packs the
outer-code symbols into sub-block-integrity (SBI) bodies, then writes
length-prefixed PSDU bodies to stdout for the C++ streamtx:

    python3 fused_fec_tx.py --input data.bin | DEVOURER_PID=0x8812 ./build/streamtx

Wire format on stdout:  <u32_le length><length bytes of body>

The FEC parameters (--k/--symbol-size/--overhead/--blocks-per-body/--scheme)
MUST match fused_fec_rx.py on the receiver.
"""
from __future__ import annotations

import argparse
import os
import struct
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from fused_fec_link import FusedFecSender  # noqa: E402
from stream_fec import FecConfig  # noqa: E402


def add_fec_args(ap: argparse.ArgumentParser) -> None:
    ap.add_argument("--k", type=int, default=8)
    ap.add_argument("--symbol-size", type=int, default=64)
    ap.add_argument("--overhead", type=float, default=0.5)
    ap.add_argument("--blocks-per-body", type=int, default=4)
    ap.add_argument("--scheme", default="rs", choices=("rs", "raptorq", "rlc"))
    ap.add_argument("--crc-bytes", type=int, default=2, choices=(2, 4))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--input", default=None, help="read bytes from file (else stdin)")
    ap.add_argument("--repeat", type=int, default=1,
                    help="emit each body this many times (combats RX warmup loss)")
    add_fec_args(ap)
    args = ap.parse_args(argv)

    cfg = FecConfig(k=args.k, symbol_size=args.symbol_size,
                    overhead=args.overhead, scheme=args.scheme)
    snd = FusedFecSender(cfg, args.blocks_per_body, crc_bytes=args.crc_bytes)

    src = sys.stdin.buffer if args.input is None else open(args.input, "rb")
    data = src.read()
    if args.input is not None:
        src.close()

    bodies = snd.add_bytes(data) + snd.flush()
    out = sys.stdout.buffer
    nbytes = 0
    for body in bodies:
        chunk = struct.pack("<I", len(body)) + body
        for _ in range(max(1, args.repeat)):
            out.write(chunk)
            nbytes += len(body)
    out.flush()
    sys.stderr.write(
        f"fused_fec_tx: {len(data)} src bytes → {len(bodies)} bodies "
        f"({nbytes} body bytes, env={snd.env}B, {args.blocks_per_body} sub-blocks/body, "
        f"scheme={args.scheme} k={args.k} N={cfg.k + cfg.repair_count})\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
