#!/usr/bin/env bash
# kestrelprobe_id.sh — Kestrel M0 hardware validation: the identity layer.
#
# Finds a Kestrel adapter (default: the TX20U Nano RTL8852BU, 35bc:0108),
# detaches the in-kernel rtw89 driver from its interfaces for the duration
# (sysfs unbind — NOT modprobe -r, which wouldn't survive a re-enumeration
# anyway; the interfaces are re-bound on exit), runs `kestrelprobe id`, and
# passes iff the kestrel.id event reports ok:true (die-id matches the
# PID-selected variant).
#
# Usage: sudo tests/kestrelprobe_id.sh [VID:PID]
#   sudo tests/kestrelprobe_id.sh              # 35bc:0108 (RTL8852BU)
#   sudo tests/kestrelprobe_id.sh 35bc:0101    # RTL8832CU TX50UH
#
# ZeroCD note: if lsusb shows 0bda:1a2b instead, the dongle is still in disk
# mode — usb_modeswitch flips it (usually automatic on modern distros).

set -euo pipefail

cd "$(dirname "$0")/.."

# First positional arg may be a stage (id|power|fw); default id. Second the ID.
STAGE="id"
case "${1:-}" in id|power|fw) STAGE="$1"; shift ;; esac
ID="${1:-35bc:0108}"
VID="${ID%%:*}"
PID="${ID##*:}"
PROBE="build/kestrelprobe"

[ -x "$PROBE" ] || { echo "FAIL: $PROBE not built (cmake --build build)"; exit 2; }
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root (sudo) for usbfs claim"; exit 2; }

# Locate the device's sysfs node by VID:PID.
DEVDIR=""
for d in /sys/bus/usb/devices/*; do
  [ -f "$d/idVendor" ] || continue
  if [ "$(cat "$d/idVendor")" = "$VID" ] && [ "$(cat "$d/idProduct")" = "$PID" ]; then
    DEVDIR="$d"
    break
  fi
done
if [ -z "$DEVDIR" ]; then
  echo "FAIL: no $ID on the bus (lsusb; ZeroCD 0bda:1a2b would mean un-modeswitched)"
  exit 2
fi
echo "found $ID at $DEVDIR"

# Detach any bound kernel driver from the device's interfaces; remember what
# was unbound so the trap can restore it.
UNBOUND=()  # "driver_path iface" pairs
cleanup() {
  local pair drv iface
  for pair in "${UNBOUND[@]:-}"; do
    [ -n "$pair" ] || continue
    drv="${pair% *}"; iface="${pair#* }"
    echo "$iface" > "$drv/bind" 2>/dev/null \
      && echo "rebound $iface -> $(basename "$drv")" \
      || echo "note: could not rebind $iface (re-plug restores it)"
  done
}
trap cleanup EXIT

for ifdir in "$DEVDIR":*; do
  [ -d "$ifdir" ] || continue
  if [ -L "$ifdir/driver" ]; then
    drv="$(readlink -f "$ifdir/driver")"
    iface="$(basename "$ifdir")"
    echo "unbinding $iface from $(basename "$drv")"
    echo "$iface" > "$drv/unbind"
    UNBOUND+=("$drv $iface")
  fi
done

# Run the probe; require the kestrel.id ok:true event on stdout (JSONL plane).
EVENT="kestrel.$STAGE"
set +e
OUT="$("$PROBE" "$STAGE" --vid "0x$VID" --pid "0x$PID" 2> >(sed 's/^/  /' >&2))"
RC=$?
set -e
echo "$OUT" | sed 's/^/  /'
if [ $RC -eq 0 ] && echo "$OUT" | grep -qF "\"ev\":\"$EVENT\"" \
   && echo "$OUT" | grep -F "\"ev\":\"$EVENT\"" | grep -qF '"ok":true'; then
  echo "PASS: kestrelprobe $STAGE on $ID"
  exit 0
fi
echo "FAIL: kestrelprobe $STAGE on $ID (rc=$RC)"
exit 1
