#!/usr/bin/env bash
# kestrel_vendor_ko_smoke.sh — prove the two Kestrel vendor kernel modules
# (reference/rtl8852bu -> 8852bu.ko, reference/rtl8852cu -> 8852cu.ko) actually
# *run* on this host kernel, not merely compile: insmod, watch the driver bind
# the adapter and register a wlan netdev, bring it up, then unload and restore
# the in-kernel rtw89 binding.
#
# These .ko are the kernel cells for the Kestrel regress rows and the usbmon
# golden reference for the M1 bring-up diff — a broken vendor build here has
# historically cost days on other chip families, hence a hard functional gate.
#
# Usage: sudo tests/kestrel_vendor_ko_smoke.sh [bu|cu|both]   (default: both)

set -euo pipefail

cd "$(dirname "$0")/.."
WHICH="${1:-both}"

[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }

find_devdir() { # $1=vid $2=pid -> sysfs device dir
  local d
  for d in /sys/bus/usb/devices/*; do
    [ -f "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor")" = "$1" ] && [ "$(cat "$d/idProduct")" = "$2" ] \
      && { echo "$d"; return 0; }
  done
  return 1
}

iface_for_devdir() { # netdev whose device path sits under $1
  local n
  for n in /sys/class/net/*; do
    [ -e "$n/device" ] || continue
    case "$(readlink -f "$n/device")" in
      "$(readlink -f "$1")"*) basename "$n"; return 0 ;;
    esac
  done
  return 1
}

LOADED_KO=""
REBIND=() # "driver_path iface" pairs to restore
cleanup() {
  set +e
  if [ -n "$LOADED_KO" ]; then
    rmmod "$LOADED_KO" 2>/dev/null && echo "  rmmod $LOADED_KO"
  fi
  local pair drv iface
  for pair in "${REBIND[@]:-}"; do
    [ -n "$pair" ] || continue
    drv="${pair% *}"; iface="${pair#* }"
    echo "$iface" > "$drv/bind" 2>/dev/null \
      && echo "  rebound $iface -> $(basename "$drv")"
  done
  REBIND=()
}
trap cleanup EXIT

smoke() { # $1=name $2=ko_path $3=vid $4=pid
  local name="$1" ko="$2" vid="$3" pid="$4" devdir ifdir drv iface wlan i
  echo "=== $name ($vid:$pid) ==="
  [ -f "$ko" ] || { echo "FAIL: $ko not built (make -C $(dirname "$ko"))"; return 1; }

  devdir="$(find_devdir "$vid" "$pid")" \
    || { echo "FAIL: $vid:$pid not on the bus"; return 1; }

  # Detach whatever in-kernel driver holds the interfaces (rtw89_*), remember
  # for restore.
  for ifdir in "$devdir":*; do
    [ -L "$ifdir/driver" ] || continue
    drv="$(readlink -f "$ifdir/driver")"
    iface="$(basename "$ifdir")"
    echo "  unbinding $iface from $(basename "$drv")"
    echo "$iface" > "$drv/unbind"
    REBIND+=("$drv $iface")
  done

  insmod "$ko" || { echo "FAIL: insmod $ko"; return 1; }
  LOADED_KO="$(basename "$ko" .ko)"
  echo "  insmod ok: $(modinfo -F version "$ko")"

  # The vendor module probes already-present devices on insmod; give the
  # netdev a moment to register.
  wlan=""
  for i in $(seq 1 20); do
    wlan="$(iface_for_devdir "$devdir" || true)"
    [ -n "$wlan" ] && break
    sleep 0.5
  done
  if [ -z "$wlan" ]; then
    echo "FAIL: no netdev appeared for $vid:$pid under $LOADED_KO (dmesg tail:)"
    dmesg | tail -15 | sed 's/^/    /'
    cleanup
    return 1
  fi
  echo "  netdev: $wlan"

  ip link set "$wlan" up || { echo "FAIL: ip link up $wlan"; cleanup; return 1; }
  # udev may rename the netdev (wlan0 -> wlpXsY...) right after registration —
  # re-resolve through sysfs before touching it again.
  wlan="$(iface_for_devdir "$devdir" || echo "$wlan")"
  echo "  link up ok ($wlan); driver: $(basename "$(readlink -f /sys/class/net/"$wlan"/device/driver 2>/dev/null || echo unknown)")"
  ip link set "$wlan" down 2>/dev/null || true

  cleanup   # rmmod + rebind rtw89 for this adapter
  trap cleanup EXIT  # re-arm (cleanup cleared its state)
  LOADED_KO=""
  echo "PASS: $name loads, binds $vid:$pid, registers $wlan on $(uname -r)"
}

RC=0
case "$WHICH" in
  bu|both) smoke rtl8852bu reference/rtl8852bu/8852bu.ko 35bc 0108 || RC=1 ;;&
  cu|both) smoke rtl8852cu reference/rtl8852cu/8852cu.ko 35bc 0101 || RC=1 ;;
  *) echo "usage: $0 [bu|cu|both]"; exit 2 ;;
esac
exit $RC
