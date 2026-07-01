#!/usr/bin/env bash
# Run a devourer binary against the BL-M8812EU2 (RTL8812EU, 0bda:a81a) for
# Jaguar-3 EU (rtl8822e) bring-up. Unlike the c812 CU harness, the bare EU
# module has NO kernel driver bound, so there is no unbind/rebind — but it
# DOES need libusb_reset_device (SKIP_RESET stays unset) and an external 5V
# supply. On a crash the deep hub chain (3-2.3.3) is recovered by a uhubctl
# port power-cycle.
#
#   sudo tests/eu_devourer_run.sh <binary> [seconds] [usbmon_out]
# e.g. sudo tests/eu_devourer_run.sh build/WiFiDriverDemo 12 /tmp/eu_dev.txt
set -u

PID=${PID:-a81a}
SYS=${SYS:-}
if [ -z "$SYS" ]; then
  for d in /sys/bus/usb/devices/*/idProduct; do
    [ "$(cat "$d" 2>/dev/null)" = "$PID" ] && { SYS=$(basename "$(dirname "$d")"); break; }
  done
fi
if [ -z "$SYS" ] || [ ! -e "/sys/bus/usb/devices/$SYS" ]; then
  echo "ERROR: device PID=$PID not found on USB (power-cycle it?)" >&2; exit 1
fi
HUBLOC=${SYS%.*}      # 3-2.3.3 -> 3-2.3  (uhubctl hub location)
HUBPORT=${SYS##*.}    # 3-2.3.3 -> 3      (uhubctl port number)
echo "[harness] a81a at $SYS (uhubctl $HUBLOC port $HUBPORT)"
BIN=${1:-build/WiFiDriverDemo}
SECS=${2:-12}
MONOUT=${3:-}
COMM=$(basename "$BIN" | cut -c1-15)   # /proc comm is truncated to 15 chars
BUS=$(cat /sys/bus/usb/devices/$SYS/busnum)
DEVNUM=$(cat /sys/bus/usb/devices/$SYS/devnum)
MON=/sys/kernel/debug/usb/usbmon/${BUS}u

cleanup() {
  sudo pkill -9 -x "$COMM" 2>/dev/null
  [ -n "$MONOUT" ] && sudo pkill -9 -f "cat $MON" 2>/dev/null
  # A crashed bring-up can wedge the MAC; recover the bare module with an
  # electrical power-cycle of its port (it has no kernel driver to rebind).
  if [ ! -e "/sys/bus/usb/devices/$SYS" ]; then
    echo "[recover] $SYS gone — power-cycling $HUBLOC port $HUBPORT"
    sudo uhubctl -l "$HUBLOC" -p "$HUBPORT" -a cycle -d 2 >/dev/null 2>&1
    sleep 7
  fi
}
trap cleanup EXIT

if [ -n "$MONOUT" ]; then
  sudo modprobe usbmon
  sudo sh -c "cat $MON > ${MONOUT}.raw" & sleep 0.3
fi

echo "=== running $BIN for ${SECS}s against $SYS (devnum $DEVNUM) ==="
sudo env DEVOURER_VID=0x0bda DEVOURER_PID=0xa81a \
     DEVOURER_CHANNEL="${DEVOURER_CHANNEL:-36}" \
     DEVOURER_NB_BW="${DEVOURER_NB_BW:-}" \
     DEVOURER_TX_PWR="${DEVOURER_TX_PWR:-}" \
     DEVOURER_SKIP_RESET="${DEVOURER_SKIP_RESET:-}" \
     stdbuf -oL -eL timeout -k 5 "$SECS" "$BIN" 2>&1 | sed -E 's/^/[dev] /'

if [ -n "$MONOUT" ]; then
  sudo pkill -9 -f "cat $MON" 2>/dev/null; sleep 0.3
  sudo chown "$(id -u):$(id -g)" "${MONOUT}.raw" 2>/dev/null || true
  grep -E "Co:${BUS}:0*${DEVNUM}:" "${MONOUT}.raw" > "$MONOUT" 2>/dev/null || cp "${MONOUT}.raw" "$MONOUT"
  echo "usbmon -> $MONOUT ($(wc -l < "$MONOUT") dev lines)"
fi
