#!/usr/bin/env python3
"""Generate length-prefixed synthetic HEVC NAL units with controlled
temporal_ids, to exercise SvcTxDemo's TID -> TxMode UEP mapping on air.

Output (stdout, binary):  <u32_le len><len bytes of NAL>  ...

HEVC NAL header (2 bytes):  byte0 = (nal_type << 1),  byte1 = (tid + 1).
Per GOP we emit a dyadic 3-temporal-layer hierarchical-P pattern plus an IDR:
  1 IDR (critical)  +  4 T0  +  8 T1  +  16 T2     (ratio 1:4:8:16)
so the witness rate histogram should be MCS0:MCS1:MCS4:MCS7 ~= 1:4:8:16
(the default_policy ladder in svc_tx.h).

  python3 tests/gen_svc_nals.py [GOPS] [PAYLOAD] | DEVOURER_PID=0x8812 ... ./build/SvcTxDemo
"""
import struct
import sys

GOPS = int(sys.argv[1]) if len(sys.argv) > 1 else 6
PAYLOAD = int(sys.argv[2]) if len(sys.argv) > 2 else 200

IDR_W_RADL = 19  # IRAP -> critical
TRAIL_R = 1      # referenced trailing picture


def nal(nal_type: int, tid: int) -> bytes:
    body = bytes([(nal_type << 1) & 0xFF, (tid + 1) & 0x07]) + bytes(PAYLOAD)
    return struct.pack("<I", len(body)) + body


out = sys.stdout.buffer
for _ in range(GOPS):
    out.write(nal(IDR_W_RADL, 0))            # critical
    for _ in range(4):
        out.write(nal(TRAIL_R, 0))           # T0  base
    for _ in range(8):
        out.write(nal(TRAIL_R, 1))           # T1
    for _ in range(16):
        out.write(nal(TRAIL_R, 2))           # T2
out.flush()
