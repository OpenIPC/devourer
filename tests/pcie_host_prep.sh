#!/usr/bin/env bash
# pcie_host_prep.sh — hand a Realtek PCIe NIC to the devourer PCIe transport
# for a test session, and hand it back afterwards.
#
#   sudo tests/pcie_host_prep.sh <bdf> [--usb-driver rtw88_8812au]  # -> vfio-pci
#   sudo tests/pcie_host_prep.sh <bdf> --restore                    # -> kernel
#
# What binding does beyond tests/pcie_vfio_bind.sh: the in-tree rtw88/rtw89
# modules auto-probe (and firmware-download into) every Realtek device at each
# enumeration, and `modprobe -r` does not survive a re-enumeration — so the
# driver currently bound to <bdf> (and any --usb-driver named for a USB
# witness on the same host) is temp-blacklisted in a modprobe.d drop-in for
# the session, then unloaded. --restore removes the drop-in and re-probes.
# The AER correctable-error count for <bdf> is printed on both paths so a run
# can be bracketed (see docs: the 8822CE under rtw88 logs replay-timer
# timeouts in 5 s bursts; whether they continue under devourer is a
# bring-up tell, not something to mask).
set -euo pipefail

BDF="${1:?usage: $0 <bdf> [--restore] [--usb-driver MOD]}"
shift
MODE=bind
USB_DRIVERS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --restore) MODE=restore; shift ;;
    --usb-driver) USB_DRIVERS+=("$2"); shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done
[ "$(id -u)" = 0 ] || { echo "ERROR: run as root" >&2; exit 1; }
DEV="/sys/bus/pci/devices/$BDF"
[ -d "$DEV" ] || { echo "ERROR: no PCI device $BDF" >&2; exit 1; }
HERE="$(cd "$(dirname "$0")" && pwd)"
DROPIN=/etc/modprobe.d/devourer-pcie-session-blacklist.conf

aer_count() {
  journalctl -k -b 0 --no-pager 2>/dev/null | grep -c "$BDF: PCIe Bus Error" || true
}

current_driver() {
  if [ -e "$DEV/driver" ]; then basename "$(readlink -f "$DEV/driver")"; else echo none; fi
}

if [ "$MODE" = restore ]; then
  rm -f "$DROPIN"
  bash "$HERE/pcie_vfio_bind.sh" "$BDF" --restore
  for m in "${USB_DRIVERS[@]}"; do modprobe "$m" 2>/dev/null || true; done
  echo "AER correctable errors on $BDF so far: $(aer_count)"
  exit 0
fi

cur="$(current_driver)"
mods=("${USB_DRIVERS[@]}")
case "$cur" in
  vfio-pci|none) ;;
  *) mods+=("$cur") ;;
esac
if [ "${#mods[@]}" -gt 0 ]; then
  {
    echo "# devourer PCIe test session — removed by tests/pcie_host_prep.sh --restore"
    for m in "${mods[@]}"; do echo "blacklist $m"; done
  } > "$DROPIN"
  echo "temp-blacklisted: ${mods[*]} ($DROPIN)"
fi
echo "AER correctable errors on $BDF before bind: $(aer_count)"
bash "$HERE/pcie_vfio_bind.sh" "$BDF"
# The kernel driver's module stays loaded after the unbind; drop it so a
# USB witness re-enumeration does not re-probe through it.
for m in "${mods[@]}"; do
  if lsmod | grep -q "^$m "; then
    modprobe -r "$m" 2>/dev/null && echo "unloaded $m" || echo "note: $m still in use (left loaded)"
  fi
done
lsmod | grep -E "^rtw8[89]_" || true
