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
#   # then on devourer side:
#   sudo env DEVOURER_VID=... DEVOURER_PID=... DEVOURER_CHANNEL=<channel> \
#       DEVOURER_DUMP_CANARY=1 ./build/WiFiDriverTxDemo 2>&1 \
#     | awk '/DEVOURER_DUMP_CANARY \(post channel-set ch=<channel>\)/,
#            /END DEVOURER_DUMP_CANARY/' \
#     | sed 's/^<devourer>//' \
#     > /tmp/dev.canary
#   diff /tmp/krn.canary /tmp/dev.canary
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
  echo "Usage: $0 <iface> <channel>" >&2
  exit 1
fi

IFACE="$1"
CHANNEL="$2"

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

# BB canary set — same list as `RadioManagementModule::phy_SwChnlAndSetBwMode8812`
# emits when DEVOURER_DUMP_CANARY=1 is set. Keep the two lists in sync.
for ADDR in 0x808 0x80c 0x82c 0x830 0x834 0x838 0x84c 0x860 0x8ac \
            0x8b0 0x8c4 0xc00 0xc1c 0xc20 0xc24 0xc28 0xc2c 0xc30 \
            0xc34 0xc38 0xc3c 0xc40 0xc50 0xc54 0xc60 0xc64 0xc68 \
            0xc6c 0xc70 0xc90 0xe1c 0xe50 0xe54; do
  printf "BB %s = %s\n" "$ADDR" "$(readreg $ADDR)"
done

for ADDR in 0x040 0x0cf 0x0f0 0x100 0x102 0x420 0x4c8 0x508 \
            0x522 0x550 0x560 0x610 0x614; do
  printf "MAC %s = %s\n" "$ADDR" "$(readreg $ADDR)"
done

for PATH_IDX in 0 1; do
  PATH_LBL=$([ "$PATH_IDX" = "0" ] && echo "A" || echo "B")
  for RF in 0x00 0x05 0x18 0x42 0x65 0x8f; do
    printf "RF[%s] %s = %s\n" "$PATH_LBL" "$RF" "$(rfread $PATH_IDX $RF)"
  done
done

echo "=== END DEVOURER_DUMP_CANARY ==="
