#!/usr/bin/env bash
# Run a devourer binary against the LB-LINK WDN1300H (RTL8812CU, 0bda:c812,
# INVENTORY #12) for Jaguar-3 bring-up: unbind the kernel rtw88_8822cu, run the
# binary with the Jaguar-3 family forced, optionally usbmon-capture, then rebind.
#
#   sudo tests/jaguar3_devourer_run.sh <binary> [seconds] [usbmon_out]
# e.g. sudo tests/jaguar3_devourer_run.sh build/WiFiDriverDemo 12 /tmp/j3_dev.txt
set -u

# Auto-detect the c812's sysfs node by PID (it moves between USB ports across
# power-cycles). Override with SYS=<node> if needed.
PID=${PID:-c812}
SYS=${SYS:-}
if [ -z "$SYS" ]; then
  # Retry the search: a prior run's recovery power-cycle can leave the DUT
  # still re-enumerating for several seconds.
  for _try in $(seq 15); do
    for d in /sys/bus/usb/devices/*/idProduct; do
      [ "$(cat "$d" 2>/dev/null)" = "$PID" ] && { SYS=$(basename "$(dirname "$d")"); break; }
    done
    [ -n "$SYS" ] && break
    sleep 1
  done
fi
if [ -z "$SYS" ] || [ ! -e "/sys/bus/usb/devices/$SYS" ]; then
  echo "ERROR: device PID=$PID not found on USB (power-cycle it?)" >&2; exit 1
fi
HUBLOC=${SYS%.*}      # 3-2.4 -> 3-2   (uhubctl hub location)
HUBPORT=${SYS##*.}    # 3-2.4 -> 4     (uhubctl port number)
echo "[harness] c812 at $SYS (uhubctl $HUBLOC port $HUBPORT)"
DRV=rtw88_8822cu
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
  echo "$SYS:1.0" | sudo tee /sys/bus/usb/drivers/$DRV/bind >/dev/null 2>&1
  sleep 3
  # devourer crashing mid-init wedges the MAC (kernel: "mac power on failed").
  # Recovery needs an electrical power-cycle, not just rebind. If the kernel
  # driver didn't take, uhubctl-cycle the c812's port (9-1 port 3) and rebind.
  if [ ! -e /sys/bus/usb/devices/$SYS/$SYS:1.0/driver ]; then
    echo "[recover] kernel rebind failed — power-cycling $HUBLOC port $HUBPORT"
    sudo uhubctl -l "$HUBLOC" -p "$HUBPORT" -a cycle -d 2 >/dev/null 2>&1
    sleep 7
  fi
}
trap cleanup EXIT

# Release the device from the kernel driver (cold de-init, as on Jaguar-1).
echo "$SYS:1.0" | sudo tee /sys/bus/usb/drivers/$DRV/unbind >/dev/null 2>&1; sleep 1

if [ -n "$MONOUT" ]; then
  sudo modprobe usbmon
  sudo sh -c "cat $MON > ${MONOUT}.raw" & sleep 0.3
fi

echo "=== running $BIN for ${SECS}s against $SYS (devnum $DEVNUM) ==="
sudo env DEVOURER_VID=0x0bda DEVOURER_PID="0x$PID" \
     DEVOURER_CHANNEL="${DEVOURER_CHANNEL:-36}" \
     DEVOURER_NB_BW="${DEVOURER_NB_BW:-}" \
     DEVOURER_NB_DAC="${DEVOURER_NB_DAC:-}" \
     DEVOURER_TX_PWR="${DEVOURER_TX_PWR:-}" \
     DEVOURER_TX_GAP_US="${DEVOURER_TX_GAP_US:-}" \
     DEVOURER_HOP_BW="${DEVOURER_HOP_BW:-}" \
     DEVOURER_HOP_OFFSET="${DEVOURER_HOP_OFFSET:-}" \
     DEVOURER_TX_RATE="${DEVOURER_TX_RATE:-}" \
     DEVOURER_TX_MAXFAIL="${DEVOURER_TX_MAXFAIL:-}" \
     DEVOURER_TX_PAYLOAD_BYTES="${DEVOURER_TX_PAYLOAD_BYTES:-}" \
     DEVOURER_SKIP_RESET="${DEVOURER_SKIP_RESET:-}" \
     stdbuf -oL -eL timeout -k 5 "$SECS" "$BIN" 2>&1 | sed -E 's/^/[dev] /'
# timeout sends SIGTERM at $SECS; the binary now catches it, runs a clean chip
# card-disable (Stop()), and exits. -k 5 escalates to SIGKILL only if that clean
# shutdown itself hangs past 5s, so a buggy de-init can't wedge the harness.

if [ -n "$MONOUT" ]; then
  sudo pkill -9 -f "cat $MON" 2>/dev/null; sleep 0.3
  sudo chown "$(id -u):$(id -g)" "${MONOUT}.raw" 2>/dev/null || true
  grep -E "Co:${BUS}:0*${DEVNUM}:" "${MONOUT}.raw" > "$MONOUT" 2>/dev/null || cp "${MONOUT}.raw" "$MONOUT"
  echo "usbmon -> $MONOUT ($(wc -l < "$MONOUT") dev lines, $(grep -cE '\bCo:.* s 40 05 ' "$MONOUT") reg writes)"
fi
