#!/usr/bin/env python3
"""Parse <devourer-csi> lines emitted by rxdemo with
DEVOURER_RX_DUMP_CSI=hex,hex,... set and report which selectors look
like a real BB bus vs static status flags.

The C++ side runs the dbgport sweep on the first 8 canonical-SA frames.
This script groups the values per selector and reports:

  * which selectors changed across frames (= dynamic, possibly a bus)
  * which stayed constant (= static, usually a status flag or clock-domain bit)
  * per-bit toggle ratio (a high ratio across MANY bits suggests data;
    a high ratio on a few bits suggests counters or single-bit flags)

A real per-subcarrier IQ selector would show:
  * value changing on every frame (different RX content -> different IQ)
  * high toggle ratio on the low 16 bits (Q) AND the high 16 bits (I),
    since both should swing across all 16 bits worth of range
  * NO sticky bits in the upper byte (no busy / clock-domain marker)

Run:
  ./build/rxdemo > /tmp/csi.log 2>&1   # with DEVOURER_RX_DUMP_CSI set
  uv run python tools/precoder/csi_dump.py /tmp/csi.log
"""

from __future__ import annotations

import argparse
import re
import sys
from collections import defaultdict
from pathlib import Path

CSI_RE = re.compile(
    r"<devourer-csi>hit=(?P<hit>\d+)\s+selector=0x(?P<sel>[0-9a-fA-F]+)"
    r"\s+value=0x(?P<val>[0-9a-fA-F]+)"
)
WEDGE_RE = re.compile(r"<devourer-csi-wedged>after selector=0x([0-9a-fA-F]+)")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("log", type=Path, help="rxdemo stdout/stderr capture")
    args = p.parse_args()

    if not args.log.exists():
        print(f"no such file: {args.log}", file=sys.stderr)
        return 1

    per_selector: dict[int, list[int]] = defaultdict(list)
    wedged_after: int | None = None

    for line in args.log.read_text().splitlines():
        m = CSI_RE.search(line)
        if m:
            sel = int(m.group("sel"), 16)
            val = int(m.group("val"), 16)
            per_selector[sel].append(val)
            continue
        w = WEDGE_RE.search(line)
        if w:
            wedged_after = int(w.group(1), 16)

    if not per_selector:
        print("no <devourer-csi> lines found", file=sys.stderr)
        return 2

    print(f"# parsed {sum(len(v) for v in per_selector.values())} samples "
          f"across {len(per_selector)} selectors")
    if wedged_after is not None:
        print(f"# WEDGED after selector=0x{wedged_after:08x}")
    print()
    print(f"{'selector':>12}  {'n':>3}  {'unique':>6}  {'mean':>10}  "
          f"{'high32':>10}  {'low32':>10}  {'toggle/32':>9}  classification")

    for sel in sorted(per_selector):
        vs = per_selector[sel]
        n = len(vs)
        unique = len(set(vs))
        mean = sum(vs) / n
        # Stickiness: how many bits stayed constant across samples?
        and_mask = 0xFFFFFFFF
        or_mask = 0
        for v in vs:
            and_mask &= v
            or_mask |= v
        sticky_bits = bin(~(or_mask ^ and_mask) & 0xFFFFFFFF).count("1")
        toggle_bits = 32 - sticky_bits
        # Classify
        if unique == 1:
            cls = "STATIC (status flag / const)"
        elif toggle_bits >= 16:
            cls = "DYNAMIC (wide-bit bus — candidate)"
        elif toggle_bits >= 4:
            cls = "PARTIAL (counter or narrow field)"
        else:
            cls = "SINGLE-BIT (flag/clock)"
        print(f"  0x{sel:08x}  {n:>3}  {unique:>6}  {int(mean):>10}  "
              f"0x{(vs[0] >> 16) & 0xFFFF:>8x}  0x{vs[0] & 0xFFFF:>8x}  "
              f"{toggle_bits:>9}  {cls}")

    print()
    print("# Notes:")
    print("#  - 'DYNAMIC (wide-bit bus)' selectors are the IQ-candidate set;")
    print("#    cross-check against frame-by-frame variance by raising")
    print("#    DEVOURER_RX_DUMP_CSI sample count (edit kCsiMaxFrames in examples/rx/main.cpp).")
    print("#  - Per-subcarrier IQ would pack int16 i + int16 q in one u32;")
    print("#    high16 AND low16 both swing wide. A 'DYNAMIC' selector with")
    print("#    only the low 16 bits toggling is more likely a counter.")
    print("#  - Selector encoding for per-subcarrier capture (upstream phydm)")
    print("#    typically packs subcarrier index in the low bits of the selector.")
    print("#    Sweep adjacent values (0x40,0x41,0x42,...) to look for a")
    print("#    monotonic-index relationship.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
