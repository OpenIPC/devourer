#!/usr/bin/env bash
# pcie_vfio_bind.sh — move a PCIe Realtek NIC between its kernel driver and
# vfio-pci for the devourer PCIe transport (pcieprobe / rxdemo DEVOURER_PCIE_BDF).
#
#   sudo tests/pcie_vfio_bind.sh 0000:01:00.0            # -> vfio-pci
#   sudo tests/pcie_vfio_bind.sh 0000:01:00.0 --restore  # -> kernel driver
#
# Uses driver_override (race-free vs. the in-tree rtw88 auto-probe: new_id
# would still let rtw88 grab the device on a rescan; driver_override pins it).
# --restore clears the override and reprobes, handing it back to rtw88.
set -euo pipefail

BDF="${1:?usage: $0 <bdf> [--restore]}"
MODE="${2:-bind}"
DEV="/sys/bus/pci/devices/$BDF"

[ -d "$DEV" ] || { echo "ERROR: no PCI device $BDF" >&2; exit 1; }
[ "$(id -u)" = 0 ] || { echo "ERROR: run as root" >&2; exit 1; }

current_driver() {
  basename "$(readlink -f "$DEV/driver" 2>/dev/null)" 2>/dev/null || echo none
}

if [ "$MODE" = "--restore" ]; then
  echo "" > "$DEV/driver_override"
  if [ -e "$DEV/driver" ]; then
    echo "$BDF" > "$DEV/driver/unbind"
  fi
  echo "$BDF" > /sys/bus/pci/drivers_probe
  echo "restored: $BDF -> $(current_driver)"
  exit 0
fi

modprobe vfio-pci

cur="$(current_driver)"
if [ "$cur" = vfio-pci ]; then
  echo "already bound: $BDF -> vfio-pci"
  exit 0
fi
if [ "$cur" != none ]; then
  echo "unbinding $BDF from $cur"
  echo "$BDF" > "$DEV/driver/unbind"
fi

echo vfio-pci > "$DEV/driver_override"
echo "$BDF" > /sys/bus/pci/drivers_probe

new="$(current_driver)"
[ "$new" = vfio-pci ] || { echo "ERROR: bind failed ($BDF -> $new)" >&2; exit 1; }

group="$(basename "$(readlink -f "$DEV/iommu_group")")"
echo "bound: $BDF -> vfio-pci (IOMMU group $group, /dev/vfio/$group)"
