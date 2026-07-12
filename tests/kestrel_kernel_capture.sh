#!/usr/bin/env bash
# Capture the vendor 8852bu.ko bring-up of the RTL8852BU (35bc:0108, the
# TX20U Nano — INVENTORY #13) as the ground-truth register + firmware-download
# sequence for the Kestrel devourer port (power-on -> DLE/HFC -> DLFW).
#
# This is the golden reference the M1 transcription is validated against:
#   * control-OUT writes (bmRequestType 0x40, bRequest 5) = register writes,
#     decoded as "addr = value" — diff against HalKestrel's power-on/DLE writes.
#   * bulk-OUT on the CH12 endpoint = the firmware section transfers.
#
# A fresh probe is triggered by insmod of the vendor module while usbmon
# records the device's bus. The in-kernel rtw89_8852bu is unbound first (and
# rebound on exit) so the vendor module is the sole driver during capture.
#
#   sudo tests/kestrel_kernel_capture.sh [out.txt]
set -u

cd "$(dirname "$0")/.."

VID=35bc PID=0108
KO=reference/rtl8852bu/8852bu.ko
OUT=${1:-/tmp/kestrel_kernel.txt}
RAW=/tmp/kestrel_kernel_raw.txt

[ -f "$KO" ] || { echo "FAIL: $KO not built (make -C reference/rtl8852bu)"; exit 2; }
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }

# Locate the device (sysfs node, bus, devnum).
DEVDIR=""
for d in /sys/bus/usb/devices/*; do
  [ -f "$d/idVendor" ] || continue
  [ "$(cat "$d/idVendor")" = "$VID" ] && [ "$(cat "$d/idProduct")" = "$PID" ] \
    && { DEVDIR="$d"; break; }
done
[ -n "$DEVDIR" ] || { echo "FAIL: $VID:$PID not on the bus"; exit 2; }
SYS=$(basename "$DEVDIR")
BUS=$(cat "$DEVDIR/busnum")
DEVNUM=$(cat "$DEVDIR/devnum")
MON=/sys/kernel/debug/usb/usbmon/${BUS}u

REBIND=() # "driver_path iface"
cleanup() {
  pkill -9 -f "cat $MON" 2>/dev/null
  rmmod 8852bu 2>/dev/null
  local pair drv iface
  for pair in "${REBIND[@]:-}"; do
    [ -n "$pair" ] || continue
    drv="${pair% *}"; iface="${pair#* }"
    echo "$iface" > "$drv/bind" 2>/dev/null && echo "rebound $iface"
  done
}
trap cleanup EXIT

modprobe usbmon
echo "device $SYS = bus $BUS devnum $DEVNUM ; monitor $MON"

# Detach any in-kernel driver (rtw89_8852bu) so the vendor module owns it.
for ifdir in "$DEVDIR":*; do
  [ -L "$ifdir/driver" ] || continue
  drv="$(readlink -f "$ifdir/driver")"
  iface="$(basename "$ifdir")"
  echo "unbinding $iface from $(basename "$drv")"
  echo "$iface" > "$drv/unbind"
  REBIND+=("$drv $iface")
done
sleep 1

# Capture across the vendor insmod = fresh power-on + DLE/HFC + firmware DL.
sh -c "cat $MON > $RAW" &
sleep 0.5
echo ">> insmod $KO (bring-up starts)"
insmod "$KO"
sleep 5
pkill -9 -f "cat $MON" 2>/dev/null
sleep 0.3

chown "$(id -u):$(id -g)" "$RAW" 2>/dev/null || true
# Keep only this device's URBs (the bus may carry other adapters).
grep -E ":0*${DEVNUM}:" "$RAW" > "$OUT" 2>/dev/null
[ -s "$OUT" ] || cp "$RAW" "$OUT"

NREG=$(grep -cE '\bCo:.* s 40 05 ' "$OUT" 2>/dev/null || echo 0)
NBULK=$(grep -cE '\bBo:' "$OUT" 2>/dev/null || echo 0)
echo "captured $(wc -l < "$RAW") raw lines; $(wc -l < "$OUT") for devnum $DEVNUM -> $OUT"
echo "control-OUT reg writes: $NREG ; bulk-OUT URBs: $NBULK"
echo "decode with: tests/decode_wseq.py $OUT   (or tools/usbmon_diff.py)"
