#!/usr/bin/env bash
# kestrel_vendor_ifup_capture.sh — capture the VENDOR 8852bu.ko bringing the
# RTL8852BU (35bc:0108) interface UP. Unlike the module-load capture (probe =
# MAC init + FWDL only), ifup is where the vendor programs BB/RF via firmware
# offload (cmd_ofld H2C on CH12). The vendor .ko uses the SAME fw blob devourer
# extracts, so this is the true byte-reference for devourer's cmd_ofld framing.
#
#   sudo tests/kestrel_vendor_ifup_capture.sh [out.txt]
set -u
cd "$(dirname "$0")/.."
VID=35bc PID=0108
KO=reference/rtl8852bu/8852bu.ko
OUT=${1:-/tmp/kestrel_vendor_ifup.txt}
RAW=/tmp/kestrel_vendor_ifup_raw.txt
[ -f "$KO" ] || { echo "FAIL: $KO not built"; exit 2; }
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }

DEVDIR=""
for d in /sys/bus/usb/devices/*; do
  [ -f "$d/idVendor" ] || continue
  [ "$(cat "$d/idVendor")" = "$VID" ] && [ "$(cat "$d/idProduct")" = "$PID" ] \
    && { DEVDIR="$d"; break; }
done
[ -n "$DEVDIR" ] || { echo "FAIL: $VID:$PID not on the bus"; exit 2; }
SYS=$(basename "$DEVDIR"); BUS=$(cat "$DEVDIR/busnum"); DEVNUM=$(cat "$DEVDIR/devnum")
MON=/sys/kernel/debug/usb/usbmon/${BUS}u
modprobe usbmon 2>/dev/null

REBIND=()
CATPID=""
cleanup() {
  [ -n "$CATPID" ] && kill "$CATPID" 2>/dev/null
  rmmod 8852bu 2>/dev/null
  local pair drv iface
  for pair in "${REBIND[@]:-}"; do
    [ -n "$pair" ] || continue
    drv="${pair% *}"; iface="${pair#* }"
    echo "$iface" > "$drv/bind" 2>/dev/null && echo "rebound $iface"
  done
}
trap cleanup EXIT

# Detach the in-kernel driver from this device only.
for ifdir in "$DEVDIR":*; do
  [ -L "$ifdir/driver" ] || continue
  drv="$(readlink -f "$ifdir/driver")"; iface="$(basename "$ifdir")"
  echo "unbind $iface from $(basename "$drv")"
  echo "$iface" > "$drv/unbind"; REBIND+=("$drv $iface")
done
sleep 1

insmod "$KO" || { echo "FAIL: insmod"; exit 2; }
sleep 3
# Find the vendor netdev.
NET=""
for n in "$DEVDIR":*/net/*; do [ -e "$n" ] && { NET=$(basename "$n"); break; }; done
[ -n "$NET" ] || { echo "FAIL: no vendor netdev"; exit 2; }
echo "vendor netdev=$NET ; capturing ifup on $MON"

sh -c "cat $MON > $RAW" & CATPID=$!
sleep 0.5
ip link set "$NET" up
sleep 3
iw dev "$NET" scan >/dev/null 2>&1 || true
sleep 3
kill "$CATPID" 2>/dev/null; CATPID=""
ip link set "$NET" down 2>/dev/null
sleep 0.3

chown "$(id -u):$(id -g)" "$RAW" 2>/dev/null || true
grep -E ":0*${DEVNUM}:" "$RAW" > "$OUT" 2>/dev/null || cp "$RAW" "$OUT"
echo "captured $(wc -l < "$RAW") raw; $(wc -l < "$OUT") for devnum $DEVNUM -> $OUT"
echo "bulk-OUT: $(grep -cE '\bBo:' "$OUT" || true)"
