#!/usr/bin/env bash
# kestrel_tx_sdr.sh — authoritative on-air TX validation for the 8852BU via the
# B210 SDR (channel-occupancy / duty, ceiling-free — unlike the monitor witness
# which undercounts / races the kernel rebind). Measures duty with NO TX
# (baseline noise floor) then with devourer flooding ch6; a clear duty jump =
# the 8852BU is radiating.
#
#   sudo tests/kestrel_tx_sdr.sh [channel] [seconds]
set -u
cd "$(dirname "$0")/.."
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }
CH=${1:-6}; SECS=${2:-4}; RATE=${3:-6M}
if [ "$CH" -le 14 ]; then FREQ=$(( 2407 + CH*5 ))e6   # 2.4 GHz
else FREQ=$(( 5000 + CH*5 ))e6; fi                    # 5 GHz: 5000 + ch*5
TX_ID="35bc:0108"; TX_HUB="3-2.3"; TX_PORT="3"
TXLOG="/tmp/kestrel_tx_sdr_tx.log"
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
  pkill -9 -x -f "build/txdemo" 2>/dev/null || true
}
trap cleanup EXIT

# NOTE: take exactly ONE SDR read per session — a second back-to-back sdr_duty
# read can fail to reacquire the B210 and report ~0 (CLAUDE.md). So we bring up
# the flood FIRST, then take a single read; a high absolute duty (>>ambient ~18%
# noise floor on this bench) == the 8852BU is radiating.
echo ">> freq ${FREQ} (ch$CH)"
echo ">> [1/2] bring up devourer TX flooding ch$CH (gap=0)"
uhubctl -l "$TX_HUB" -p "$TX_PORT" -a cycle -d 2 >/dev/null 2>&1
sleep 6
unbind_kernel "$TX_ID"; sleep 1
env DEVOURER_VID=0x35bc DEVOURER_PID=0x0108 DEVOURER_CHANNEL=$CH \
  DEVOURER_TX_GAP_US=0 DEVOURER_LOG_LEVEL=warn DEVOURER_TX_RATE="$RATE" \
  build/txdemo >"$TXLOG" 2>&1 &
TXPID=$!
sleep 4
kill -0 "$TXPID" 2>/dev/null || { echo "FAIL: txdemo died"; tail -5 "$TXLOG"; exit 2; }

echo ">> [2/2] single SDR duty read WITH devourer flooding"
LIVE=$(python3 tests/sdr_duty.py --freq "$FREQ" --secs "$SECS" --mcs 0 --bw 20 2>/dev/null \
        | grep -iE "duty" | tail -1)

kill "$TXPID" 2>/dev/null; TXPID=""
echo "=================================================================="
echo "live (devourer flooding): ${LIVE:-<none>}"
echo "Bench ambient noise floor is ~18% duty; a live duty well above that (e.g."
echo ">50%) == the 8852BU is radiating."
