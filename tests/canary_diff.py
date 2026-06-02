#!/usr/bin/env python3
"""canary_diff.py — diff two devourer/kernel canary dumps with
runtime-ephemeral register masking.

A clean way to compare:
  - `tools/canary_kernel_dump.sh <iface> <channel> [chip]` output
    (kernel side via iwpriv)
  - `DEVOURER_DUMP_CANARY=1 ./build/WiFiDriverDemo` block extracted
    by `awk '/DEVOURER_DUMP_CANARY \\(post channel-set ch=N\\)/,
    /END DEVOURER_DUMP_CANARY/' | sed 's/^<devourer>//'`

Both files contain `KIND 0xADDR = 0xVALUE` lines in the same order
(the kernel script mirrors devourer's emit order exactly per
`RadioManagementModule::phy_SwChnlAndSetBwMode8812`).

Usage:
    tests/canary_diff.py <kernel.canary> <devourer.canary> \\
        [--channel N] [--strict] [--show-clean]

Exit status:
    0  — no real init-drift divergence
    1  — divergences found in non-ephemeral registers
    2  — file parse error / register-set mismatch

Why this isn't just `diff`:

Several registers shift on every capture for reasons that aren't
init drift — they're runtime state the kernel or devourer keep
updating after init completes. Listing them as divergences would
drown out real bugs. The mask:

  - MAC 0x040, 0x550, 0x560: per-queue / beacon-window / TBTT
    counters that increment continuously.
  - RF[A] 0x42: thermal-meter sample register; reads vary with
    chip temperature so each capture shows a slightly different
    value. The thermal value is also the input to phydm's TX
    BB-swing tracking (see BB 0xc1c[31:21] below).
  - BB 0xc1c bits 31:21 / 0xe1c bits 31:21: TX BB-swing
    `tx_scaling_table_jaguar` index, written by `PowerTracking8812a`
    (and the kernel's phydm watchdog) based on the thermal-meter
    sample. Same drift class as RF[A] 0x42. Other bits of 0xc1c
    (AGC table select [11:8], static base bits) ARE checked.

There's also a known capture-state asymmetry: the kernel iface is
long-lived (CCK regs at 5G retain values written during prior 2.4G
activity), while devourer captures from a fresh process per run
(BB-init defaults). At 5G channels we therefore skip `BB 0xc20`
(rTxAGC_A_CCK11_CCK1_JAguar) because it's CCK-only — never written
at 5G by either side, but reflects different histories. Add more
to `CAPTURE_STATE_5G_ARTIFACTS` if new ones surface.
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

# Registers always masked (runtime ephemeral on both sides). Per-bit
# masks: if a register has only some bits that drift, give a bit mask
# that we should IGNORE — the diff compares (a & ~mask) vs (b & ~mask).
RUNTIME_EPHEMERAL: dict[tuple[str, int], int] = {
    # MAC counters that advance on every TBTT / queue tick.
    ("MAC", 0x040): 0xFFFFFFFF,
    ("MAC", 0x550): 0xFFFFFFFF,
    ("MAC", 0x560): 0xFFFFFFFF,
    # RF thermal-meter sample — varies with chip temperature.
    ("RF[A]", 0x42): 0xFFFFFFFF,
    ("RF[B]", 0x42): 0xFFFFFFFF,
    # BB TX-swing thermal pwrtrk — only bits 31:21 (the
    # tx_scaling_table_jaguar index) are thermal-tracked.
    ("BB", 0xc1c): 0xFFE00000,
    ("BB", 0xe1c): 0xFFE00000,
}

# Capture-state artifacts at 5GHz only — registers that aren't
# written at 5G but retain prior 2.4G state on a long-lived kernel
# iface, while devourer captures from a fresh process. Skip
# entirely when --channel > 14.
CAPTURE_STATE_5G_ARTIFACTS: set[tuple[str, int]] = {
    ("BB", 0xc20),  # rTxAGC_A_CCK11_CCK1_JAguar
    ("BB", 0xe20),  # path-B mirror
}

LINE_RE = re.compile(
    r"^(BB|MAC|RF\[[AB]\])\s+0x([0-9a-fA-F]+)\s*=\s*0x([0-9a-fA-F]+)\s*$"
)


@dataclass(frozen=True)
class Reading:
    kind: str  # "BB", "MAC", "RF[A]", "RF[B]"
    addr: int
    value: int


def parse_canary(path: Path) -> dict[tuple[str, int], int]:
    """Parse a canary file into {(kind, addr): value}. Skips lines
    outside the `=== DEVOURER_DUMP_CANARY ===` envelope (header,
    log noise, etc.)."""
    readings: dict[tuple[str, int], int] = {}
    in_block = False
    for raw in path.read_text().splitlines():
        if "DEVOURER_DUMP_CANARY (post channel-set" in raw:
            in_block = True
            continue
        if "END DEVOURER_DUMP_CANARY" in raw:
            in_block = False
            continue
        if not in_block:
            continue
        m = LINE_RE.match(raw.strip())
        if not m:
            continue
        kind, addr_s, val_s = m.groups()
        readings[(kind, int(addr_s, 16))] = int(val_s, 16)
    return readings


def diff(
    kernel: dict[tuple[str, int], int],
    devourer: dict[tuple[str, int], int],
    channel: int,
    strict: bool,
) -> tuple[list[tuple[tuple[str, int], int, int, str]], list[tuple[str, int]]]:
    """Returns (real_divergences, masked_divergences)."""
    is_5g = channel > 14
    real: list[tuple[tuple[str, int], int, int, str]] = []
    masked: list[tuple[str, int]] = []
    common_keys = set(kernel.keys()) & set(devourer.keys())
    for key in sorted(common_keys, key=lambda k: (k[0], k[1])):
        k_val = kernel[key]
        d_val = devourer[key]
        if is_5g and key in CAPTURE_STATE_5G_ARTIFACTS and not strict:
            if k_val != d_val:
                masked.append(key)
            continue
        ignore_mask = RUNTIME_EPHEMERAL.get(key, 0) if not strict else 0
        keep_mask = 0xFFFFFFFF & ~ignore_mask
        if (k_val & keep_mask) == (d_val & keep_mask):
            continue
        if ignore_mask:
            tag = f"masked-bits=0x{ignore_mask:x}"
        else:
            tag = "real"
        real.append((key, k_val, d_val, tag))
    return real, masked


def report_set_mismatch(
    kernel: dict[tuple[str, int], int],
    devourer: dict[tuple[str, int], int],
) -> bool:
    """Return True if the register sets disagree."""
    k_only = set(kernel) - set(devourer)
    d_only = set(devourer) - set(kernel)
    if not k_only and not d_only:
        return False
    if k_only:
        sys.stderr.write("Registers in kernel dump but not devourer:\n")
        for kind, addr in sorted(k_only):
            sys.stderr.write(f"  {kind} 0x{addr:x}\n")
    if d_only:
        sys.stderr.write("Registers in devourer dump but not kernel:\n")
        for kind, addr in sorted(d_only):
            sys.stderr.write(f"  {kind} 0x{addr:x}\n")
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("kernel", type=Path, help="kernel canary dump")
    ap.add_argument("devourer", type=Path, help="devourer canary dump")
    ap.add_argument("--channel", type=int, default=6,
                    help="channel the captures are from (controls 5GHz "
                         "capture-state masking)")
    ap.add_argument("--strict", action="store_true",
                    help="disable all masking — report every divergence")
    ap.add_argument("--show-clean", action="store_true",
                    help="print 'CLEAN' line even if no divergences")
    args = ap.parse_args()

    kernel = parse_canary(args.kernel)
    devourer = parse_canary(args.devourer)

    if not kernel:
        sys.stderr.write(f"no canary readings parsed from {args.kernel}\n")
        return 2
    if not devourer:
        sys.stderr.write(f"no canary readings parsed from {args.devourer}\n")
        return 2

    if report_set_mismatch(kernel, devourer):
        sys.stderr.write(
            "register sets diverge — kernel/devourer canary "
            "lists are out of sync\n")
        return 2

    real, masked = diff(kernel, devourer, args.channel, args.strict)

    if not real:
        if masked:
            print(f"Capture-state-artifact registers masked (5G only): "
                  f"{', '.join(f'{kind} 0x{addr:x}' for kind, addr in masked)}")
        if args.show_clean:
            print(f"CLEAN ({len(kernel)} regs compared, "
                  f"{len(masked)} masked at ch={args.channel})")
        return 0

    print(f"Canary diff ({len(kernel)} regs, ch={args.channel}):")
    width = max(len(f"{kind} 0x{addr:x}") for (kind, addr), *_ in real)
    print(f"  {'Register':<{width}}  Kernel       Devourer     Notes")
    print(f"  {'-'*width}  -----------  -----------  -----")
    for (kind, addr), k_val, d_val, tag in real:
        reg = f"{kind} 0x{addr:x}"
        notes = "" if tag == "real" else tag
        print(f"  {reg:<{width}}  0x{k_val:08x}  0x{d_val:08x}  {notes}")

    if masked:
        print()
        print(f"Capture-state-artifact registers masked (5G only): "
              f"{', '.join(f'{kind} 0x{addr:x}' for kind, addr in masked)}")

    print()
    print(f"FAIL: {len(real)} real divergence(s)")
    return 1


if __name__ == "__main__":
    sys.exit(main())
