#!/usr/bin/env bash
# rtl8733b_vendor_ko_smoke.sh — prove the pinned vendor RTL8733BU module
# (tests/rtl8733b_vendor_build.sh -> build/vendor-8733bu/src/8733bu.ko)
# actually RUNS on this kernel: insmod, watch it bind the adapter's Wi-Fi
# interface and register a netdev, bring the link up, put it in monitor mode
# on a channel, then unload. The .ko is the kernel cell for tests/regress.py
# on this family and the vendor-driver control docs/rtl8733b.md lacked.
#
# The combo module's Bluetooth interfaces (class e0, btusb) are left alone:
# only interfaces no driver holds, or a Wi-Fi driver holds, are touched.
#
# Usage: sudo tests/rtl8733b_vendor_ko_smoke.sh            # 0bda:b733
#        PID=f72b sudo tests/rtl8733b_vendor_ko_smoke.sh   # the Wi-Fi-only module
#        SYSFS=9-2 sudo tests/rtl8733b_vendor_ko_smoke.sh  # two same-model adapters: pin the port
set -euo pipefail
cd "$(dirname "$0")/.."
VID="${VID:-0bda}"; PID="${PID:-b733}"; CH="${CH:-36}"; SYSFS="${SYSFS:-}"
KO="${KO:-build/vendor-8733bu/src/8733bu.ko}"
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }
[ -f "$KO" ] || { echo "FAIL: $KO not built — run tests/rtl8733b_vendor_build.sh"; exit 2; }

# Every match, not the first: a bench with two same-model adapters must name
# the one it means (SYSFS=bus-port) before anything here unbinds a driver.
find_devdir() { local d hits=(); for d in /sys/bus/usb/devices/*; do [ -f "$d/idVendor" ] || continue
  [ "$(cat "$d/idVendor")" = "$1" ] && [ "$(cat "$d/idProduct")" = "$2" ] && hits+=("$d"); done
  if [ -n "$SYSFS" ]; then [ -d "/sys/bus/usb/devices/$SYSFS" ] && echo "/sys/bus/usb/devices/$SYSFS" && return 0; return 1; fi
  [ "${#hits[@]}" -eq 1 ] && { echo "${hits[0]}"; return 0; }
  [ "${#hits[@]}" -gt 1 ] && { echo "ambiguous: ${#hits[@]} adapters match $1:$2 — set SYSFS=<bus-port>: ${hits[*]##*/}" >&2; }
  return 1; }
iface_for_devdir() { local n; for n in /sys/class/net/*; do [ -e "$n/device" ] || continue
  case "$(readlink -f "$n/device")" in "$(readlink -f "$1")"*) basename "$n"; return 0;; esac; done; return 1; }

LOADED_KO=""; REBIND=()
cleanup() {
  set +e
  [ -n "$LOADED_KO" ] && rmmod "$LOADED_KO" 2>/dev/null && echo "  rmmod $LOADED_KO"
  local pair drv iface
  for pair in "${REBIND[@]:-}"; do [ -n "$pair" ] || continue
    drv="${pair% *}"; iface="${pair#* }"; echo "$iface" > "$drv/bind" 2>/dev/null && echo "  rebound $iface -> $(basename "$drv")"; done
  REBIND=()
}
trap cleanup EXIT

devdir="$(find_devdir "$VID" "$PID")" || { echo "FAIL: $VID:$PID not on the bus (or ambiguous — see above)"; exit 1; }
for ifdir in "$devdir":*; do
  [ -L "$ifdir/driver" ] || continue
  [ "$(cat "$ifdir/bInterfaceClass" 2>/dev/null)" = "e0" ] && continue   # Bluetooth stays with btusb
  drv="$(readlink -f "$ifdir/driver")"; iface="$(basename "$ifdir")"
  echo "  unbinding $iface from $(basename "$drv")"; echo "$iface" > "$drv/unbind"; REBIND+=("$drv $iface")
done

insmod "$KO" || { echo "FAIL: insmod $KO"; exit 1; }
LOADED_KO="$(basename "$KO" .ko)"
echo "  insmod ok: $(modinfo -F version "$KO")"
wlan=""
for i in $(seq 1 30); do wlan="$(iface_for_devdir "$devdir" || true)"; [ -n "$wlan" ] && break; sleep 0.5; done
[ -n "$wlan" ] || { echo "FAIL: no netdev for $VID:$PID under $LOADED_KO"; dmesg | tail -15 | sed 's/^/    /'; exit 1; }
echo "  netdev: $wlan (driver $(basename "$(readlink -f /sys/class/net/"$wlan"/device/driver)"))"
# udev may rename the netdev (wlan0 -> wlx...) right after registration, so
# re-resolve through sysfs before every command and give the rename a moment.
sleep 2
wlan="$(iface_for_devdir "$devdir" || echo "$wlan")"
ip link set "$wlan" down
wlan="$(iface_for_devdir "$devdir" || echo "$wlan")"
iw dev "$wlan" set type monitor || { echo "FAIL: monitor mode refused on $wlan"; dmesg | tail -8 | sed 's/^/    /'; exit 1; }
ip link set "$wlan" up || { echo "FAIL: ip link up $wlan"; exit 1; }
wlan="$(iface_for_devdir "$devdir" || echo "$wlan")"
iw dev "$wlan" set channel "$CH" || { echo "FAIL: set channel $CH"; exit 1; }
echo "  monitor mode on ch$CH: $(iw dev "$wlan" info | grep -E 'type|channel' | tr -s ' ' | tr '\n' ' ')"
ip link set "$wlan" down 2>/dev/null || true
echo "PASS: $LOADED_KO binds $VID:$PID, registers $wlan, monitor ch$CH on $(uname -r)"
