#!/usr/bin/env python3
"""Decode the per-path OFDM/CCK TXAGC reference indices from a usbmon text
capture of an RTL8822E bring-up. Pairs with tests/eu_txagc_regdiff.sh.

The driver programs TX power via vendor control writes (bmRequestType 0x40,
bRequest 0x05): wValue = register address, data = little-endian value. The
8822e OFDM reference lives in 0x18e8[16:10] (path A) / 0x41e8 (path B); the CCK
reference in 0x18a0[22:16] / 0x41a0. The last write to each register wins, so we
must walk chronologically (a sort|uniq would lose ordering and the final value).

Usage: decode_txagc.py <usbmon-capture.txt> [<label>]
"""
import re
import sys


def writes(path):
    """Chronological (addr, value) list of 40/05 vendor writes."""
    out = []
    pat = re.compile(
        r" s 40 05 ([0-9a-f]{4}) ([0-9a-f]{4}) ([0-9a-f]{4}) [0-9]+ = ?([0-9a-f ]*)")
    for ln in open(path, errors="ignore"):
        m = pat.search(ln)
        if not m:
            continue
        addr = int(m.group(1), 16)
        raw = m.group(4).replace(" ", "")
        if len(raw) < 2:
            continue
        val = 0
        for i in range(0, min(len(raw), 8), 2):  # little-endian, up to 4 bytes
            val |= int(raw[i:i + 2], 16) << (4 * i)
        out.append((addr, val))
    return out


def last(ws, addr):
    v = None
    for a, d in ws:
        if a == addr:
            v = d
    return v


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    path = sys.argv[1]
    label = sys.argv[2] if len(sys.argv) > 2 else path
    ws = writes(path)
    a = last(ws, 0x18e8)
    b = last(ws, 0x41e8)
    ca = last(ws, 0x18a0)
    cb = last(ws, 0x41a0)
    print(f"{label}: {len(ws)} vendor writes")
    if a is not None and b is not None:
        ra, rb = (a >> 10) & 0x7f, (b >> 10) & 0x7f
        print(f"  OFDM ref  pathA(0x18e8)=0x{ra:02x}  pathB(0x41e8)=0x{rb:02x}  "
              f"delta(B-A)={rb - ra:+d}")
    if ca is not None and cb is not None:
        print(f"  CCK  ref  pathA(0x18a0)=0x{(ca >> 16) & 0x7f:02x}  "
              f"pathB(0x41a0)=0x{(cb >> 16) & 0x7f:02x}")


if __name__ == "__main__":
    main()
