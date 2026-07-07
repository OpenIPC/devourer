#!/usr/bin/env python3
"""canary_diff.py — diff two devourer/kernel canary dumps with
runtime-ephemeral register masking.

A clean way to compare:
  - `tools/canary_kernel_dump.sh <iface> <channel> [chip]` output
    (kernel side via iwpriv)
  - `DEVOURER_DUMP_CANARY=1 ./build/rxdemo` block. The dump is a
    human diagnostic and goes to **stderr** as `devourer [I] `-prefixed
    lines (stdout carries the JSONL event stream), so capture with
    `2> rx.err` (or `2>&1` past the demo) and extract with
    `awk '/DEVOURER_DUMP_CANARY \\(post channel-set ch=N\\)/,
    /END DEVOURER_DUMP_CANARY/' | sed -E 's/^devourer \\[[A-Z]\\] //'`
    — though the parser below strips the prefix itself, so raw
    stderr files work unstripped too.

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
updating after init completes, or calibration outputs that vary
between runs because the calibration samples a noisy signal.
Listing them as divergences would drown out real bugs. The mask:

  - MAC 0x040, 0x550, 0x560: per-queue / beacon-window / TBTT
    counters that increment continuously.
  - RF[A] 0x42: thermal-meter sample register; reads vary with
    chip temperature so each capture shows a slightly different
    value. The thermal value is also the input to phydm's TX
    BB-swing tracking (see BB 0xc1c[31:21] below).
  - RF[A] 0x00 / RF[B] 0x00 lower 16 bits: RF_AC LNA + analog
    config. Bits 4-9 are LNA gain index (phydm_rssi_monitor
    walks), bit 15 is LNA HW/SW control mode (phydm_lna_sat
    toggles). Upper bits 16-19 (RF mode: Standby/Normal/TX/RX)
    ARE checked. Devourer doesn't run the phydm RSSI / LNA-sat
    modules so it holds whatever the channel-set left.
  - BB 0xc1c / 0xe1c / 0x181c / 0x1a1c bits 31:21: TX BB-swing
    `tx_scaling_table_jaguar` index, written by `PowerTracking8812a`
    (and the kernel's phydm watchdog) based on the thermal-meter
    sample. Same drift class as RF[A] 0x42. Devourer's pwrtrk is
    gated to CHIP_8812 only, so on 8814 only the kernel walks
    path C/D — masking is still correct for the diff. Other bits
    of these registers (AGC table select [11:8], static base
    bits) ARE checked.
  - BB 0xc50, 0xe50, 0x1850, 0x1a50 bits 7:0: DIG IGI (path
    A/B/C/D Initial Gain). The kernel's phydm DIG watchdog walks
    the IGI value up and down each interrupt cycle based on
    received noise floor; devourer writes the 0x1c floor once at
    init and leaves it. Upper bits of the AGC core word are
    static config and ARE checked.
  - BB 0x8b0, 0xc10, 0xc14, 0xc90, 0xc94 (and 0xe10/0xe14/0xe90/0xe94
    path-B mirrors): IQK output coefficients. Both sides run IQK,
    but the tone sweep samples noise so the per-bit output varies
    between runs even on the same chip. Functional IQK correctness
    is validated by the on-air RX/TX matrix, not by canary diff.
  - MAC 0x100, 0x420, 0x4c8, 0x522, 0x610, 0x614: MAC operational-mode
    artifacts. Kernel iface runs as a fully-configured normal-mode
    driver (programs the iface MAC address, TX queue control, TBTT-
    prohibit timing, MAC port enable bits); devourer's monitor mode
    intentionally skips most of these. Not bugs — structural mode
    differences that would otherwise dominate the diff.

There's also a known capture-state asymmetry: the kernel iface is
long-lived (CCK regs at 5G retain values written during prior 2.4G
activity), while devourer captures from a fresh process per run
(BB-init defaults). At 5G channels we therefore skip the per-path
CCK TX-AGC registers — `rTxAGC_*_CCK11_CCK1_*` for paths A/B (8812 /
8821) and paths C/D (8814) — they're CCK-only, never written at
5G by either side, but reflect different histories. Add more to
`CAPTURE_STATE_5G_ARTIFACTS` if new ones surface.
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
    # RF_AC (mode + LNA + analog) — lower 16 bits are runtime-tracked
    # by phydm (LNA gain index in bits 4-9, LNA HW/SW mode in bit 15,
    # plus various biasing). Upper bits 16-19 are the RF mode field
    # (Standby/Normal/TX/RX) and ARE checked. Devourer lacks the
    # phydm_rssi_monitor + phydm_lna_sat modules that walk these
    # bits on kernel, so devourer holds whatever the channel-set
    # sequence left.
    ("RF[A]", 0x00): 0x0000FFFF,
    ("RF[B]", 0x00): 0x0000FFFF,
    # BB TX-swing thermal pwrtrk — only bits 31:21 (the
    # tx_scaling_table_jaguar index) are thermal-tracked. Devourer's
    # PowerTracking8812a is gated to CHIP_8812 only, so on 8814 the
    # kernel walks the path-C/D swing while devourer leaves it static
    # — same divergence class, same masking applies.
    ("BB", 0xc1c): 0xFFE00000,    # path A
    ("BB", 0xe1c): 0xFFE00000,    # path B
    ("BB", 0x181c): 0xFFE00000,   # path C (8814 only)
    ("BB", 0x1a1c): 0xFFE00000,   # path D (8814 only)
    # DIG IGI (bits 7:0) — kernel's phydm DIG watchdog continuously
    # walks the path-IGI based on RX noise floor; devourer writes
    # the 0x1c floor once and doesn't update.
    ("BB", 0xc50): 0x000000FF,    # path A
    ("BB", 0xe50): 0x000000FF,    # path B
    ("BB", 0x1850): 0x000000FF,   # path C (8814 only)
    ("BB", 0x1a50): 0x000000FF,   # path D (8814 only)
    # IQK output regs — calibration results vary run-to-run because
    # the tone sweep samples a noisy signal. Bit-exact match between
    # captures (or between devourer and kernel) isn't expected;
    # functional correctness is checked by the on-air RX/TX matrix.
    ("BB", 0x8b0): 0xFFFFFFFF,
    ("BB", 0xc10): 0xFFFFFFFF,    # path-A RX IQK fill
    ("BB", 0xc14): 0xFFFFFFFF,
    ("BB", 0xc90): 0xFFFFFFFF,    # path-A TX IQK matrix
    ("BB", 0xc94): 0xFFFFFFFF,
    ("BB", 0xe10): 0xFFFFFFFF,    # path-B RX IQK fill
    ("BB", 0xe14): 0xFFFFFFFF,
    ("BB", 0xe90): 0xFFFFFFFF,    # path-B TX IQK matrix
    ("BB", 0xe94): 0xFFFFFFFF,
    # MAC operational-mode artifacts — kernel runs the iface as a
    # fully-configured normal-mode driver (programs MAC address, TX
    # queue control, TBTT-prohibit timing, MAC port enable bits);
    # devourer runs monitor-mode-only and intentionally skips these
    # writes. Not bugs — structural mode differences.
    ("MAC", 0x100): 0xFFFFFFFF,   # REG_CR — TX/RXMACEN, port enable
    ("MAC", 0x420): 0xFFFFFFFF,   # REG_FWHW_TXQ_CTRL — TX queue cfg
    ("MAC", 0x4c8): 0xFFFFFFFF,   # REG_TBTT_PROHIBIT — beacon timing
    ("MAC", 0x522): 0xFFFFFFFF,   # REG_TXPAUSE — TX pause domains
    ("MAC", 0x610): 0xFFFFFFFF,   # REG_MACID  (MAC[3:0]) — kernel-only
    ("MAC", 0x614): 0xFFFFFFFF,   # REG_MACID+4 (MAC[5:4]) — kernel-only
    # 8814 init-time 8812 band-switch artifacts — HalModule's cold-init
    # calls `PHY_SwitchWirelessBand8812` directly for all chip families
    # (NOT through the chip-aware `phy_SwBand8812` dispatcher), so on
    # 8814 the 8812-specific writes still fire. Routing this call
    # through `PHY_SwitchWirelessBand8814A` instead corrupts the RF
    # SI/PI read interface (CCK+OFDM clock-gate cycle pre-IQK leaves
    # phy_RFSerialRead returning wrong-register values on path B),
    # so we accept the small-bit BB divergence over broken RF reads.
    # Specific bits the 8812 path forces vs BB-init / 8814 expected:
    ("BB", 0x808): 0x10000000,    # bit 28 — CCK enable (8812 path forces 1)
    ("BB", 0x82c): 0x00000001,    # bit 0  — AGC table tail
    ("BB", 0x830): 0x00024000,    # bits 14, 17 — rPwed_TH_Jaguar 5G bits
    ("BB", 0x834): 0x0000000c,    # bits 2, 3  — rBWIndication tail
}

# Capture-state artifacts at 5GHz only — registers that aren't
# written at 5G but retain prior 2.4G state on a long-lived kernel
# iface, while devourer captures from a fresh process. Skip
# entirely when --channel > 14.
CAPTURE_STATE_5G_ARTIFACTS: set[tuple[str, int]] = {
    ("BB", 0xc20),    # rTxAGC_A_CCK11_CCK1_JAguar      (path A)
    ("BB", 0xe20),    # rTxAGC_B_CCK11_CCK1_JAguar      (path B)
    ("BB", 0x1820),   # rTxAGC_C_CCK11_CCK1_Jaguar2     (path C, 8814)
    ("BB", 0x1a20),   # rTxAGC_D_CCK11_CCK1_Jaguar2     (path D, 8814)
}

LINE_RE = re.compile(
    r"^(BB|MAC|RF\[[AB]\])\s+0x([0-9a-fA-F]+)\s*=\s*0x([0-9a-fA-F]+)\s*$"
)

# Devourer human-diagnostic prefix on stderr: `devourer [I] ` (level letter
# T/D/I/W/E). Stripped liberally so an unfiltered stderr capture parses.
DIAG_PREFIX_RE = re.compile(r"^devourer \[[A-Z]\] ")


@dataclass(frozen=True)
class Reading:
    kind: str  # "BB", "MAC", "RF[A]", "RF[B]"
    addr: int
    value: int


def parse_canary(path: Path) -> dict[tuple[str, int], int]:
    """Parse a canary file into {(kind, addr): value}. Skips lines
    outside the `=== DEVOURER_DUMP_CANARY ===` envelope (header,
    log noise, etc.). Devourer-side lines carry the stderr
    `devourer [I] ` diagnostic prefix — stripped before matching, so
    both pre-stripped and raw stderr captures parse."""
    readings: dict[tuple[str, int], int] = {}
    in_block = False
    for raw in path.read_text().splitlines():
        raw = DIAG_PREFIX_RE.sub("", raw.strip())
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
