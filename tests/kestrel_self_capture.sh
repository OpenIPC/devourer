#!/usr/bin/env bash
# kestrel_self_capture.sh — usbmon capture of DEVOURER's own Kestrel bring-up
# (rxdemo full monitor path: power -> FWDL -> MAC TRX -> BB/RF -> channel), to
# diff the register/H2C stream against the vendor .ko ifup capture with
# tests/kestrel_capture_diff.py.
#
# Usage: sudo [KESTREL_VBUS_LOC=3-2.3 KESTREL_VBUS_PORT=3] \
#          tests/kestrel_self_capture.sh [OUT] [SECS]
#
# KESTREL_VBUS_LOC/_PORT: optional uhubctl hub location/port for a VBUS
# power-cycle before the run (a stalled CH12 wedges the next soft re-init, so
# cold-start is the only trustworthy baseline).
set -u
cd "$(dirname "$0")/.."
VID=35bc PID=0108
OUT="${1:-/tmp/kestrel_self.txt}"
SECS="${2:-12}"
RAW=/tmp/kestrel_self_raw.txt

[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }
[ -x build/rxdemo ] || { echo "FAIL: build/rxdemo missing"; exit 2; }

# Optional VBUS cold-cycle (uhubctl), then wait for re-enumeration.
if [ -n "${KESTREL_VBUS_LOC:-}" ]; then
  echo ">> VBUS power-cycle via uhubctl -l $KESTREL_VBUS_LOC -p ${KESTREL_VBUS_PORT:-3}"
  uhubctl -l "$KESTREL_VBUS_LOC" -p "${KESTREL_VBUS_PORT:-3}" -a cycle -d 2 || \
    echo "WARN: uhubctl cycle failed (continuing warm)"
  sleep 3
fi

DEVDIR=""
for _ in $(seq 20); do
  for d in /sys/bus/usb/devices/*; do
    [ -f "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor")" = "$VID" ] && [ "$(cat "$d/idProduct")" = "$PID" ] \
      && { DEVDIR="$d"; break; }
  done
  [ -n "$DEVDIR" ] && break
  sleep 0.5
done
[ -n "$DEVDIR" ] || { echo "FAIL: $VID:$PID not on the bus"; exit 2; }
BUS=$(cat "$DEVDIR/busnum")
MON="/sys/kernel/debug/usb/usbmon/$((10#$BUS))u"
[ -e "$MON" ] || modprobe usbmon
echo "device $VID:$PID bus=$BUS mon=$MON"

REBIND=()
CATPID=""
cleanup() {
  [ -n "$CATPID" ] && kill "$CATPID" 2>/dev/null
  pkill -9 -x rxdemo 2>/dev/null
  local pair drv iface
  for pair in "${REBIND[@]:-}"; do
    [ -n "$pair" ] || continue
    drv="${pair% *}"; iface="${pair#* }"
    echo "$iface" > "$drv/bind" 2>/dev/null && echo "rebound $iface"
  done
}
trap cleanup EXIT

# Detach the in-kernel driver (rtw89) from this device only.
for ifdir in "$DEVDIR":*; do
  [ -L "$ifdir/driver" ] || continue
  drv="$(readlink -f "$ifdir/driver")"; iface="$(basename "$ifdir")"
  echo "unbind $iface from $(basename "$drv")"
  echo "$iface" > "$drv/unbind"; REBIND+=("$drv $iface")
done
sleep 1

sh -c "cat $MON > $RAW" & CATPID=$!
sleep 0.5
echo ">> running rxdemo bring-up for ${SECS}s (ch6)"
DEVOURER_VID=0x$VID DEVOURER_PID=0x$PID DEVOURER_CHANNEL=6 \
  timeout -k 2 "$SECS" build/rxdemo > /tmp/kestrel_self_events.jsonl \
  2> /tmp/kestrel_self.log || true
sleep 0.5
kill "$CATPID" 2>/dev/null
pkill -9 -f "cat $MON" 2>/dev/null   # the sh -c child; kill before any rebind
CATPID=""
sleep 0.3

# rxdemo resets the device (re-enumeration changes devnum); this device is the
# only meaningful traffic on the monitored bus during the window, keep it all.
chown "$(id -u):$(id -g)" "$RAW" 2>/dev/null || true
cp "$RAW" "$OUT"
echo "captured $(wc -l < "$OUT") lines -> $OUT (log: /tmp/kestrel_self.log)"
echo "ctrl-writes: $(grep -cE 's 40 05' "$OUT" || true) ; bulk-OUT: $(grep -cE 'S Bo:' "$OUT" || true)"
