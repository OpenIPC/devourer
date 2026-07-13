#!/usr/bin/env bash
# kestrel_ifup_capture.sh — capture the vendor/in-tree driver bringing the
# RTL8852BU (35bc:0108) interface UP, which is when BB/RF get programmed via
# firmware offload (cmd_ofld H2C on CH12). The module-load capture only covers
# probe (MAC init + FWDL); the cmd_ofld batches only fire at ifup/scan.
#
# Emits the bulk-OUT stream to OUT for byte-diffing devourer's cmd_ofld framing
# (WD + fwcmd hdr + command dwords) against the ground truth.
#
#   sudo tests/kestrel_ifup_capture.sh [out.txt]
set -u
cd "$(dirname "$0")/.."
VID=35bc PID=0108
OUT=${1:-/tmp/kestrel_ifup.txt}
RAW=/tmp/kestrel_ifup_raw.txt

[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }

DEVDIR=""
for d in /sys/bus/usb/devices/*; do
  [ -f "$d/idVendor" ] || continue
  [ "$(cat "$d/idVendor")" = "$VID" ] && [ "$(cat "$d/idProduct")" = "$PID" ] \
    && { DEVDIR="$d"; break; }
done
[ -n "$DEVDIR" ] || { echo "FAIL: $VID:$PID not on the bus"; exit 2; }
SYS=$(basename "$DEVDIR")
BUS=$(cat "$DEVDIR/busnum"); DEVNUM=$(cat "$DEVDIR/devnum")
MON=/sys/kernel/debug/usb/usbmon/${BUS}u
modprobe usbmon 2>/dev/null

# Make sure the in-kernel rtw89_8852bu owns the interface (rebind if my earlier
# capture runs left it unbound).
IF="$DEVDIR:1.0"
if [ ! -e "$IF/driver" ]; then
  echo "1.0" > /dev/null 2>&1 || true
  echo "$(basename "$IF")" > /sys/bus/usb/drivers/rtw89_8852bu/bind 2>/dev/null || true
  sleep 2
fi

# Resolve the netdev.
NET=""
for n in "$DEVDIR":*/net/*; do [ -e "$n" ] && { NET=$(basename "$n"); break; }; done
[ -n "$NET" ] || { echo "FAIL: no netdev for $SYS (driver bound?)"; exit 2; }
echo "device $SYS bus=$BUS devnum=$DEVNUM netdev=$NET mon=$MON"

CATPID=""
cleanup() { [ -n "$CATPID" ] && kill "$CATPID" 2>/dev/null; ip link set "$NET" down 2>/dev/null; }
trap cleanup EXIT

# Down first so the up transition (and its BB/RF program) is fresh in-window.
ip link set "$NET" down 2>/dev/null; sleep 1
sh -c "cat $MON > $RAW" & CATPID=$!
sleep 0.5
echo ">> ip link set $NET up (triggers PHY bring-up + cmd_ofld)"
ip link set "$NET" up
sleep 3
# a scan forces a channel set (more cmd_ofld) if iw is present
iw dev "$NET" scan >/dev/null 2>&1 || true
sleep 3
kill "$CATPID" 2>/dev/null; CATPID=""
sleep 0.3

chown "$(id -u):$(id -g)" "$RAW" 2>/dev/null || true
grep -E ":0*${DEVNUM}:" "$RAW" > "$OUT" 2>/dev/null || cp "$RAW" "$OUT"
NBULK=$(grep -cE '\bBo:' "$OUT" 2>/dev/null || echo 0)
echo "captured $(wc -l < "$RAW") raw lines; $(wc -l < "$OUT") for devnum $DEVNUM -> $OUT"
echo "bulk-OUT URBs: $NBULK"
