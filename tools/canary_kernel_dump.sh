#!/bin/bash
# canary_kernel_dump.sh — kernel-side half of the devourer cross-validation oracle (TODO T1).
#
# Pair with `DEVOURER_DUMP_CANARY=1` on the devourer side. Each captures the
# same set of BB/MAC/RF "canary" registers post-init at a chosen channel; the
# resulting plaintext files diff line-by-line. Any mismatch is a candidate
# devourer init-drift bug.
#
# Tested against `aircrack-ng/88XXau`'s `iwpriv` interface (SIOCDEVPRIVATE
# strings: `read 4,<addr>` for BB/MAC, `rfr <path> <addr>` for RF). The
# iwpriv `read` only takes width 1 or 4; we use 4 everywhere — for 1-byte
# MAC regs the upper bytes are read-zero and don't affect the low-byte
# diff.
#
# Usage:
#   sudo tools/canary_kernel_dump.sh <iface> <channel> > /tmp/krn.canary
#   # then on devourer side (the canary is a stderr diagnostic, prefixed
#   # "devourer [I] " — hence the 2>&1 and the prefix strip):
#   sudo env DEVOURER_VID=... DEVOURER_PID=... DEVOURER_CHANNEL=<channel> \
#       DEVOURER_DUMP_CANARY=1 ./build/txdemo 2>&1 \
#     | awk '/DEVOURER_DUMP_CANARY \(post channel-set ch=<channel>\)/,
#            /END DEVOURER_DUMP_CANARY/' \
#     | sed -E 's/^devourer \[[A-Z]\] //' \
#     > /tmp/dev.canary
#   diff /tmp/krn.canary /tmp/dev.canary
#   # (tests/canary_diff.py strips the prefix itself — raw captures are fine.)
#
# Iface is the wlx... or wlan? name the kernel driver enumerated. Run
# `iw dev` to find it after `modprobe 88XXau`.
#
# Expected divergence on a clean diff (do NOT chase as devourer bug):
#   BB 0x0c1c bits 31:21 — phydm TX BB-swing thermal compensation. The
#     kernel's phydm watchdog reads RF reg 0x42 thermal meter every 2s
#     and walks 0xc1c[31:21] up/down through `tx_scaling_table_jaguar`
#     (typical 0x21E/+0.5dB or 0x23E/+1.0dB on a warmed-up dongle).
#     Devourer keeps the BB-init value 0x200 (0 dB, table index 24).
#     Porting the full thermal-tracking watchdog (~2800 LOC) is out of
#     scope; the other bits of 0xc1c are byte-for-byte.

set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <iface> <channel> [chip]" >&2
  echo "  chip: 8812 (default) | 8821 | 8814" >&2
  exit 1
fi

IFACE="$1"
CHANNEL="$2"
CHIP="${3:-8812}"

case "$CHIP" in
  8812|8821|8814) ;;
  *)
    echo "unknown chip '$CHIP' — must be 8812, 8821 or 8814" >&2
    exit 1
    ;;
esac

# Per-chip path mask — mirrors the chip-aware dump in
# RadioManagementModule::phy_SwChnlAndSetBwMode8812. 8821 is 1T1R so
# path-B BB-AGC mirror + RF[B] reads return sentinel/default; skip
# them. 8814 has additional path-C/D BB-table state (RF[C]/RF[D] are
# HW write-only so RF is still A+B only).
HAS_PATHB=1
HAS_PATHCD=0
if [[ "$CHIP" = "8821" ]]; then HAS_PATHB=0; fi
if [[ "$CHIP" = "8814" ]]; then HAS_PATHCD=1; fi

if ! ip -o link show "$IFACE" >/dev/null 2>&1; then
  echo "iface '$IFACE' not found — did you modprobe 88XXau?" >&2
  exit 1
fi

# Put iface into monitor mode at the requested channel so post-init state is
# directly comparable to devourer's (which is also in monitor at the same
# channel).
iw dev "$IFACE" set type monitor 2>/dev/null || true
ip link set "$IFACE" up
iw dev "$IFACE" set channel "$CHANNEL"

readreg() {
  iwpriv "$IFACE" read 4,"$1" 2>&1 | grep -oP '0x[0-9A-Fa-f]+$' | head -1
}

rfread() {
  iwpriv "$IFACE" rfr "$1" "$2" 2>&1 | grep -oP '0x[0-9A-Fa-f]+$' | head -1
}

echo "=== DEVOURER_DUMP_CANARY (post channel-set ch=$CHANNEL) ==="

# Order matches `RadioManagementModule::phy_SwChnlAndSetBwMode8812`'s
# DEVOURER_DUMP_CANARY block exactly — keep the two in sync so the
# kernel + devourer captures line-diff cleanly.

# Shared anchors — PHY/AGC anchors common to every Jaguar chip.
for ADDR in 0x808 0x80c 0x82c 0x830 0x834 0x838 0x84c 0x860 0x8ac \
            0x8b0 0x8c4; do
  printf "BB %s = %s\n" "$ADDR" "$(readreg $ADDR)"
done

# Path-A BB: TX-AGC + AGC core + IQK/DPK output regs.
for ADDR in 0xc00 0xc1c 0xc20 0xc24 0xc28 0xc2c 0xc30 \
            0xc34 0xc38 0xc3c 0xc40 0xc50 0xc54 0xc60 0xc64 0xc68 \
            0xc6c 0xc70 \
            0xc10 0xc14 0xc90 0xc94; do
  printf "BB %s = %s\n" "$ADDR" "$(readreg $ADDR)"
done

# Path-B BB: TX-AGC mirror + IQK/DPK output. Skipped on 1T1R 8821.
if [[ "$HAS_PATHB" = "1" ]]; then
  for ADDR in 0xe1c 0xe20 0xe24 0xe28 0xe2c 0xe30 0xe34 0xe38 0xe3c \
              0xe40 0xe50 0xe54 \
              0xe10 0xe14 0xe90 0xe94; do
    printf "BB %s = %s\n" "$ADDR" "$(readreg $ADDR)"
  done
fi

# Path-C/D BB-AGC + BB-swing + IGI. 8814AU only.
if [[ "$HAS_PATHCD" = "1" ]]; then
  for ADDR in 0x1820 0x1824 0x1828 0x182c 0x1830 0x1834 0x1838 0x183c \
              0x1840 0x181c 0x1850 \
              0x1a20 0x1a24 0x1a28 0x1a2c 0x1a30 0x1a34 0x1a38 0x1a3c \
              0x1a40 0x1a1c 0x1a50; do
    printf "BB %s = %s\n" "$ADDR" "$(readreg $ADDR)"
  done
fi

# MAC anchors — chip-independent.
for ADDR in 0x040 0x0cf 0x0f0 0x100 0x102 0x420 0x4c8 0x508 \
            0x522 0x550 0x560 0x610 0x614; do
  printf "MAC %s = %s\n" "$ADDR" "$(readreg $ADDR)"
done

# RF: path A always; path B skipped on 1T1R; paths C/D never (write-only
# by HW design on 8814).
PATHS="0"
if [[ "$HAS_PATHB" = "1" ]]; then PATHS="0 1"; fi
for PATH_IDX in $PATHS; do
  PATH_LBL=$([ "$PATH_IDX" = "0" ] && echo "A" || echo "B")
  for RF in 0x00 0x05 0x18 0x42 0x65 0x8f; do
    printf "RF[%s] %s = %s\n" "$PATH_LBL" "$RF" "$(rfread $PATH_IDX $RF)"
  done
done

echo "=== END DEVOURER_DUMP_CANARY ==="
