#!/usr/bin/env bash
# kestrel_self_capture.sh — usbmon capture of DEVOURER's own Kestrel bring-up
# (kestrelprobe phy), to diff the register-write stream against the vendor .ko
# capture and find what devourer does/omits around the runtime H2C.
#
# Usage: sudo tests/kestrel_self_capture.sh [OUT] [VID:PID]
set -euo pipefail
cd "$(dirname "$0")/.."
OUT="${1:-/tmp/kestrel_self.txt}"
ID="${2:-35bc:0108}"
VID="${ID%%:*}"; PID="${ID##*:}"

DEVDIR=""
for d in /sys/bus/usb/devices/*; do
  [ -f "$d/idVendor" ] || continue
  [ "$(cat "$d/idVendor")" = "$VID" ] && [ "$(cat "$d/idProduct")" = "$PID" ] && { DEVDIR="$d"; break; }
done
[ -n "$DEVDIR" ] || { echo "no $ID on bus"; exit 2; }
BUS=$(cat "$DEVDIR/busnum"); DEVNUM=$(cat "$DEVDIR/devnum")
MON="/sys/kernel/debug/usb/usbmon/$((10#$BUS))u"
[ -e "$MON" ] || modprobe usbmon
echo "device $ID bus=$BUS devnum=$DEVNUM mon=$MON"

RAW=$(mktemp)
cleanup() { kill "$CATPID" 2>/dev/null || true; rm -f "$RAW"; }
trap cleanup EXIT

# unbind any kernel driver on the interfaces
for i in "$DEVDIR":*; do
  [ -e "$i/driver" ] || continue
  echo "$(basename "$i")" > "$i/driver/unbind" 2>/dev/null || true
done

sh -c "cat '$MON' > '$RAW'" &
CATPID=$!
sleep 0.5
echo ">> running kestrelprobe phy (bring-up + failing H2C)"
DEVOURER_LOG_LEVEL=warn build/kestrelprobe phy "$ID" 2>&1 | grep -cE "bulk_send.*FAIL" | sed 's/^/   H2C fails: /' || true
sleep 0.5
pkill -9 -f "cat $MON" 2>/dev/null || true
kill "$CATPID" 2>/dev/null || true
sleep 0.3

# The probe re-enumerates (libusb_reset), so the devnum changes; keep the whole
# bus stream (this device is effectively the only traffic during the window).
cp "$RAW" "$OUT"
echo "captured $(wc -l < "$OUT") lines -> $OUT"
echo "ctrl writes: $(grep -cE 's 40 05' "$OUT" || true) ; bulk-out: $(grep -cE 'S Bo:' "$OUT" || true)"
