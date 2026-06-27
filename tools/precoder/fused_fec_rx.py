#!/usr/bin/env python3
"""Fused-FEC RX driver — <devourer-stream> → SBI salvage → RS decode.

Reads WiFiDriverDemo's stdout (run it with DEVOURER_STREAM_OUT=1 AND
DEVOURER_RX_KEEP_CORRUPTED=1 so FCS-failed bodies are surfaced), salvages each
frame's CRC-valid sub-blocks, and Reed-Solomon-decodes the recovered symbols.
Recovered packet bytes go to stdout; the baseline-vs-SBI gain report to stderr.

    DEVOURER_PID=0x8821 DEVOURER_STREAM_OUT=1 DEVOURER_RX_KEEP_CORRUPTED=1 \
        ./build/WiFiDriverDemo | python3 fused_fec_rx.py > recovered.bin

FEC parameters MUST match fused_fec_tx.py.
"""
from __future__ import annotations

import argparse
import os
import re
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from fused_fec_link import FusedFecReceiver  # noqa: E402
from fused_fec_tx import add_fec_args  # noqa: E402  (shared FEC arg set)
from stream_fec import FecConfig  # noqa: E402

_STREAM_RE = re.compile(
    r"<devourer-stream>rate=(?P<rate>\d+)\s+len=(?P<len>\d+)"
    r"(?:\s+crc_err=(?P<crc_err>\d+))?"
    r"(?:\s+icv_err=(?P<icv_err>\d+))?"
    r".*?\s+body=(?P<hex>[0-9a-fA-F]*)")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_fec_args(ap)
    ap.add_argument("--idle-timeout", type=float, default=0.0,
                    help="exit when no new frame for this many seconds (0=off)")
    args = ap.parse_args(argv)

    cfg = FecConfig(k=args.k, symbol_size=args.symbol_size,
                    overhead=args.overhead, scheme=args.scheme)
    rcv = FusedFecReceiver(cfg, args.blocks_per_body, crc_bytes=args.crc_bytes)

    out = sys.stdout.buffer
    last = time.monotonic()
    for line in sys.stdin:
        m = _STREAM_RE.search(line)
        if not m:
            if args.idle_timeout and (time.monotonic() - last) > args.idle_timeout:
                break
            continue
        body = bytes.fromhex(m.group("hex"))
        crc_err = bool(int(m.group("crc_err") or 0)) or \
            bool(int(m.group("icv_err") or 0))
        for pkt in rcv.add_frame(body, crc_err):
            out.write(pkt)
        out.flush()
        last = time.monotonic()
        if args.idle_timeout and (time.monotonic() - last) > args.idle_timeout:
            break

    r = rcv.report()
    gain = r.sbi_blocks - r.base_blocks
    sys.stderr.write(
        f"fused_fec_rx: frames={r.frames_seen} corrupt={r.frames_corrupt} "
        f"sub-blocks={r.subblocks_total} salvaged={r.subblocks_salvaged}\n"
        f"fused_fec_rx: baseline blocks={r.base_blocks} pkts={r.base_packets}\n"
        f"fused_fec_rx: sbi      blocks={r.sbi_blocks} pkts={r.sbi_packets}\n"
        f"fused_fec_rx: FUSED-FEC GAIN = {gain} block(s) recovered that the "
        f"drop-whole-frame baseline lost\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
