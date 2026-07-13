#!/usr/bin/env bash
# kestrel_tx_stall_probe.sh — isolate the ~103-frame mgmt-TX stall.
# Runs txdemo on the 8852BU at max duty (gap=0) and counts how many frames the
# bulk-OUT accepts before the first USB timeout (rc=-7). Pre-port_init the count
# pinned to ~103 deterministically; a clean run to the frame cap proves the CMAC
# PORT is now airing frames (pool drains). No witness needed — this measures the
# host-side stall directly.
#
#   sudo tests/kestrel_tx_stall_probe.sh [seconds]
set -u
cd "$(dirname "$0")/.."
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }
DUR=${1:-10}
TX_ID="35bc:0108"
LOG="/tmp/kestrel_tx_stall.log"
[ -x build/txdemo ] || { echo "FAIL: build txdemo"; exit 2; }

sysdir_for() {
  local vid=${1%%:*} pid=${1##*:} d
  for d in /sys/bus/usb/devices/*; do
    [ -f "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor")" = "$vid" ] && [ "$(cat "$d/idProduct")" = "$pid" ] \
      && { echo "$d"; return; }
  done
}
unbind_kernel() {
  local d i; d=$(sysdir_for "$1")
  [ -n "$d" ] && for i in "$d":*; do
    [ -L "$i/driver" ] && echo "$(basename "$i")" > "$(readlink -f "$i/driver")/unbind" 2>/dev/null || true
  done
}
TXPID=""
cleanup() {
  [ -n "$TXPID" ] && kill "$TXPID" 2>/dev/null
  pkill -9 -f "build/txdemo" 2>/dev/null || true
}
trap cleanup EXIT

unbind_kernel "$TX_ID"; sleep 1
echo ">> txdemo on the 8852BU at max duty (gap=0) for ${DUR}s"
env DEVOURER_VID=0x35bc DEVOURER_PID=0x0108 DEVOURER_CHANNEL=6 \
  DEVOURER_TX_GAP_US=0 DEVOURER_LOG_LEVEL=info DEVOURER_TX_RATE=6M \
  DEVOURER_EVENTS=stdout \
  build/txdemo >"$LOG" 2>&1 &
TXPID=$!
sleep "$DUR"
kill "$TXPID" 2>/dev/null; TXPID=""
sleep 1

SENT=$(grep -cE '"ev":"tx' "$LOG" 2>/dev/null || echo 0)
TIMEOUT=$(grep -cE "rc=-7|timeout|LIBUSB_ERROR_TIMEOUT|stall" "$LOG" 2>/dev/null || echo 0)
PORT=$(grep -c "CMAC port0 enabled" "$LOG" 2>/dev/null || echo 0)
echo "=================================================================="
echo "port0 enabled log lines : $PORT"
echo "tx events               : $SENT"
echo "USB timeout/stall lines : $TIMEOUT"
echo "  --- txdemo tail ---"; tail -15 "$LOG"
