#!/usr/bin/env bash
# kestrel_golden_c2h_capture.sh — capture the in-kernel rtw89_8852bu bring-up on
# usbmon and extract its control-OUT register writes, so devourer's C2H-delivery
# gap (task #12: no async packet-C2H reaches the host) can be diffed against a
# known-good driver. READ-ONLY capture; binds the mainline rtw89_8852bu (normal
# STA — NOT the wedge-prone vendor monitor .ko) and unbinds on exit.
#
#   sudo tests/kestrel_golden_c2h_capture.sh [seconds]
set -u
cd "$(dirname "$0")/.."
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }
DUR=${1:-15}
TX_ID="35bc:0108"; TX_HUB="3-2.3"; TX_PORT="3"
 BUS=0 # the 8852BU sits on bus 3 (3-2.3.3)
CAP="/tmp/rtw89_8852bu_golden.mon"
MONPID=""
cleanup() {
  [ -n "$MONPID" ] && kill "$MONPID" 2>/dev/null
  # leave the chip unbound as we found it
  echo "35bc 0108" > /sys/bus/usb/drivers/rtw89_8852bu/remove_id 2>/dev/null || true
  d=$(for x in /sys/bus/usb/devices/*; do
        [ -f "$x/idVendor" ] && [ "$(cat "$x/idVendor")" = 35bc ] && \
        [ "$(cat "$x/idProduct")" = 0108 ] && echo "$x"; done)
  for i in "$d":*; do
    [ -L "$i/driver" ] && echo "$(basename "$i")" \
      > /sys/bus/usb/drivers/rtw89_8852bu/unbind 2>/dev/null || true
  done
}
trap cleanup EXIT

echo ">> VBUS-cycle for a clean cold bring-up"
uhubctl -l "$TX_HUB" -p "$TX_PORT" -a cycle -d 2 >/dev/null 2>&1
sleep 6

echo ">> enable usbmon"
modprobe usbmon 2>/dev/null || true
[ -e "/sys/kernel/debug/usb/usbmon/0u" ] || {
  mount -t debugfs none /sys/kernel/debug 2>/dev/null || true; }
[ -e "/sys/kernel/debug/usb/usbmon/0u" ] || { echo "FAIL: no usbmon/0u"; exit 2; }

echo ">> start capture -> $CAP"
timeout $((DUR + 5)) cat "/sys/kernel/debug/usb/usbmon/0u" > "$CAP" &
MONPID=$!
sleep 1

echo ">> bind rtw89_8852bu (mainline STA bring-up)"
# NB: rtw89_8852bu does NOT auto-probe here — bind the interface EXPLICITLY.
# It brings up the chip with the MAINLINE fw (rtw8852b_fw-1.bin, ~0.29.x), which
# differs from devourer's vendor v1.19 — so a C2H diff is indicative, not exact.
modprobe rtw89_8852bu 2>/dev/null || true
d=$(for x in /sys/bus/usb/devices/*; do
      [ -f "$x/idVendor" ] && [ "$(cat "$x/idVendor")" = 35bc ] && \
      [ "$(cat "$x/idProduct")" = 0108 ] && echo "$x"; done)
for i in "$d":*; do
  echo "$(basename "$i")" > /sys/bus/usb/drivers/rtw89_8852bu/bind 2>/dev/null || true
done
# CAVEAT: usbmon/0u may need the device to stay on ${BUS} after re-enum;
# if the capture is empty, re-check the bus (lsusb) and rfkill state.
sleep "$DUR"

kill "$MONPID" 2>/dev/null; MONPID=""
sleep 1
echo "=================================================================="
echo "capture lines: $(wc -l < "$CAP" 2>/dev/null || echo 0)"
echo "control-OUT register writes (s 40 05): $(grep -c ' s 40 05 ' "$CAP" 2>/dev/null || echo 0)"
echo "saved: $CAP"
