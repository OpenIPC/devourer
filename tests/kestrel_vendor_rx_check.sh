#!/usr/bin/env bash
# kestrel_vendor_rx_check.sh — decisive RX-reception ground truth for the
# RTL8852BU. Brings up the vendor rtw89_8852bu as a station and runs an
# `iw scan`; a non-zero AP count proves the antenna + environment + hardware
# RX path are all live, so a devourer zero-RX result is a devourer bug (not a
# dead environment). Read-only w.r.t. devourer; restores the kernel binding on
# exit.
#
#   sudo tests/kestrel_vendor_rx_check.sh
set -u
cd "$(dirname "$0")/.."
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }
VID=35bc PID=0108

find_devdir() {
  local d
  for d in /sys/bus/usb/devices/*; do
    [ -f "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor")" = "$VID" ] && [ "$(cat "$d/idProduct")" = "$PID" ] \
      && { echo "$d"; return 0; }
  done
  return 1
}
iface_for() {
  local n dev; dev="$(readlink -f "$1")"
  for n in /sys/class/net/*; do
    [ -e "$n/device" ] || continue
    case "$(readlink -f "$n/device")" in "$dev"*) basename "$n"; return 0;; esac
  done
  return 1
}

IFACE=""
cleanup() {
  [ -n "$IFACE" ] && ip link set "$IFACE" down 2>/dev/null
  echo ">> restoring kernel binding"
  modprobe -r rtw89_8852bu 2>/dev/null || true
  modprobe rtw89_8852bu 2>/dev/null || true
}
trap cleanup EXIT

echo ">> VBUS-cycle for a clean cold state (chip retains devourer soft state)"
uhubctl -l 3-2.3 -p 3 -a cycle -d 2 >/dev/null 2>&1
sleep 6
echo ">> ensuring rtw89_8852bu is bound"
modprobe rtw89_8852bu 2>/dev/null || true
sleep 4
DEVDIR="$(find_devdir)" || { echo "FAIL: $VID:$PID not on bus"; exit 2; }
# rtw89_8852bu may not auto-probe after re-enum — bind the interface explicitly.
if ! iface_for "$DEVDIR" >/dev/null; then
  for i in "$DEVDIR":*; do
    [ -e "$i/bInterfaceNumber" ] || continue
    echo "$(basename "$i")" > /sys/bus/usb/drivers/rtw89_8852bu/bind 2>/dev/null || true
  done
  sleep 3
fi
IFACE="$(iface_for "$DEVDIR")" || { echo "FAIL: no netdev for $VID:$PID"; exit 2; }
echo "   iface=$IFACE"

ip link set "$IFACE" up 2>/dev/null
sleep 2
echo ">> scanning (up to 3 tries)"
BSS=0
for i in 1 2 3; do
  SCAN=$(iw dev "$IFACE" scan 2>/dev/null)
  N=$(echo "$SCAN" | grep -c "^BSS ")
  echo "   try $i: $N APs"
  [ "$N" -gt 0 ] && echo "$SCAN" | grep -E "freq:" | sort | uniq -c | sort -rn | head
  [ "$N" -gt "$BSS" ] && BSS=$N
  [ "$BSS" -gt 0 ] && break
  sleep 3
done
echo "=================================================================="
if [ "$BSS" -gt 0 ]; then
  echo "RESULT: vendor RX WORKS — $BSS APs visible. Environment+HW RX are live;"
  echo "        devourer's zero-RX is a devourer-side RX-DMA bug (task #15)."
else
  echo "RESULT: vendor sees ZERO APs — environment may be quiet or antenna dead."
  echo "        Re-check before blaming devourer RX."
fi
