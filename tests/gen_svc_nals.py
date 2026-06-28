#!/usr/bin/env python3
"""Generate length-prefixed synthetic HEVC NAL units with controlled
temporal_ids, to exercise the SVC-T UEP path (PHY MCS + outer FEC) on air and
in software.

Output (stdout, binary):  <u32_le len><len bytes of NAL>  ...

HEVC NAL header (2 bytes):  byte0 = (nal_type << 1),  byte1 = (tid + 1).
Per GOP we emit a dyadic 3-temporal-layer hierarchical-P pattern. Every
`idr_period` GOPs the access unit is led by parameter sets + an IDR so the
critical-layer share matches a real stream (small but unconditionally needed):
  per GOP:  4 T0 base  +  8 T1  +  16 T2          (ratio 4:8:16)
  per IDR AU additionally:  VPS + SPS + PPS + IDR (critical)

Frame sizes track real HEVC: IRAP/parameter-set NALs are large, base-layer
P-frames medium, enhancement frames small — so the airtime/energy and FEC
accounting downstream see a realistic byte mix, not a flat one.

  python3 tests/gen_svc_nals.py [GOPS] [PAYLOAD] | DEVOURER_PID=0x8812 ... ./build/SvcTxDemo
"""
import struct
import sys

# HEVC nal_unit_type values
IDR_W_RADL = 19   # IRAP        -> critical
TRAIL_R = 1       # referenced trailing picture
VPS_NUT = 32      # parameter sets -> critical
SPS_NUT = 33
PPS_NUT = 34

# Realistic per-class payload sizes (bytes, header excluded). IRAP/param-sets
# big, base medium, enhancement small — a coarse but representative VBR mix.
SZ_PARAM = 48     # VPS/SPS/PPS are tiny but critical
SZ_IDR = 1800     # intra access unit — the biggest frame
SZ_T0 = 700       # base-layer P
SZ_T1 = 320
SZ_T2 = 150


def nal_bytes(nal_type: int, tid: int, payload: int) -> bytes:
    """A bare HEVC NAL (2-byte header + `payload` deterministic body bytes)."""
    body = bytearray([(nal_type << 1) & 0xFF, (tid + 1) & 0x07])
    # deterministic, non-constant body so FEC/CRC see real-looking entropy
    body += bytes((nal_type * 31 + tid * 7 + i) & 0xFF for i in range(payload))
    return bytes(body)


def gen_nals(gops: int = 6, idr_period: int = 2,
             sizes: dict | None = None) -> list[bytes]:
    """Synthetic SVC-T HEVC NAL stream as a list of bare NALs (no length prefix
    — feed straight to `svc_uep_fec.SvcUepEncoder.add_nal`). One IDR access unit
    (param sets + IDR) every `idr_period` GOPs; dyadic 4:8:16 T0/T1/T2 per GOP.
    """
    s = {"param": SZ_PARAM, "idr": SZ_IDR, "t0": SZ_T0, "t1": SZ_T1, "t2": SZ_T2}
    if sizes:
        s.update(sizes)
    out: list[bytes] = []
    for g in range(gops):
        if g % idr_period == 0:
            out.append(nal_bytes(VPS_NUT, 0, s["param"]))   # critical
            out.append(nal_bytes(SPS_NUT, 0, s["param"]))   # critical
            out.append(nal_bytes(PPS_NUT, 0, s["param"]))   # critical
            out.append(nal_bytes(IDR_W_RADL, 0, s["idr"]))  # critical
        for _ in range(4):
            out.append(nal_bytes(TRAIL_R, 0, s["t0"]))      # T0 base
        for _ in range(8):
            out.append(nal_bytes(TRAIL_R, 1, s["t1"]))      # T1
        for _ in range(16):
            out.append(nal_bytes(TRAIL_R, 2, s["t2"]))      # T2
    return out


def _main() -> None:
    gops = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    # legacy positional PAYLOAD arg: flatten every class to that size
    sizes = None
    if len(sys.argv) > 2:
        p = int(sys.argv[2])
        sizes = {"param": p, "idr": p, "t0": p, "t1": p, "t2": p}
    out = sys.stdout.buffer
    for nal in gen_nals(gops, sizes=sizes):
        out.write(struct.pack("<I", len(nal)) + nal)
    out.flush()


if __name__ == "__main__":
    _main()
