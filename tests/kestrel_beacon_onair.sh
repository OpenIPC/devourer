#!/usr/bin/env bash
# kestrel_beacon_onair.sh — validate the AX HW beacon engine (StartBeacon).
# The 8852BU runs the timesync master in HW-beacon mode (InitWrite -> StartBeacon,
# no software send loop); an 8812AU devourer rxdemo witness counts beacons at the
# canonical SA. A HW beacon at 100 TU airs ~9.77 frames/s, so a multi-second run
# should register tens of hits — proof the MAC beacons on its own timer.
#
#   sudo tests/kestrel_beacon_onair.sh [channel] [seconds]
set -u
cd "$(dirname "$0")/.."
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }
CH=${1:-6}; DUR=${2:-10}
SA="57:42:75:05:d6:00"
TX_ID="35bc:0108"; TX_HUB="3-2.3"; TX_PORT="3"
RX_ID="${RX_ID:-}"
if [ -z "$RX_ID" ]; then
  for pid in 8812 c812 0811 a811; do
    lsusb -d "0bda:$pid" >/dev/null 2>&1 && { RX_ID="0bda:$pid"; break; }
  done
fi
[ -n "$RX_ID" ] || { echo "FAIL: no 8812AU witness present"; exit 2; }
echo ">> witness adapter: $RX_ID"
RXLOG="/tmp/kestrel_beacon_rx.jsonl"; TXLOG="/tmp/kestrel_beacon_tx.log"
[ -x build/timesync ] && [ -x build/rxdemo ] || { echo "FAIL: build timesync+rxdemo"; exit 2; }

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
RXPID=""; TXPID=""
cleanup() {
  [ -n "$TXPID" ] && kill "$TXPID" 2>/dev/null
  [ -n "$RXPID" ] && kill "$RXPID" 2>/dev/null
  pkill -9 -x -f "build/timesync" 2>/dev/null || true
  pkill -9 -x -f "build/rxdemo" 2>/dev/null || true
}
trap cleanup EXIT

echo ">> VBUS-cycling the 8852BU"
uhubctl -l "$TX_HUB" -p "$TX_PORT" -a cycle -d 2 >/dev/null 2>&1
sleep 6

unbind_kernel "$RX_ID"; sleep 1
echo ">> rxdemo witness on $RX_ID ch$CH"
DEVOURER_VID=0x${RX_ID%%:*} DEVOURER_PID=0x${RX_ID##*:} DEVOURER_CHANNEL=$CH \
  DEVOURER_EVENTS=stdout DEVOURER_LOG_LEVEL=warn build/rxdemo >"$RXLOG" 2>/dev/null &
RXPID=$!
sleep 5
kill -0 "$RXPID" 2>/dev/null || { echo "FAIL: witness died"; exit 2; }

unbind_kernel "$TX_ID"; sleep 1
echo ">> 8852BU timesync master HW beacon ch$CH ${DUR}s"
env DEVOURER_VID=0x35bc DEVOURER_PID=0x0108 DEVOURER_CHANNEL=$CH \
  DEVOURER_TSYNC_ROLE=master DEVOURER_TSYNC_HWBEACON=1 DEVOURER_TSYNC_SECS=$DUR \
  DEVOURER_LOG_LEVEL=info build/timesync >"$TXLOG" 2>&1 &
TXPID=$!
sleep $((DUR + 4))
kill "$TXPID" 2>/dev/null; TXPID=""
sleep 1; kill "$RXPID" 2>/dev/null; RXPID=""

BCN_ARMED=$(grep -c "HW beacon armed\|StartBeacon -> OK" "$TXLOG" 2>/dev/null || echo 0)
HITS=$(grep -c '"ev":"rx.txhit"' "$RXLOG" 2>/dev/null || echo 0)
echo "=================================================================="
echo "StartBeacon armed: $BCN_ARMED   witness rx.txhit (SA $SA): $HITS"
if [ "$BCN_ARMED" -lt 1 ]; then
  echo "RESULT: StartBeacon FAILED to arm"; tail -6 "$TXLOG"
elif [ "${HITS:-0}" -gt 0 ]; then
  echo "RESULT: PASS — the 8852BU HW-beacons on its own TBTT timer."
else
  echo "RESULT: armed but ZERO witnessed (witness flaky, or beacon not airing)."
  echo "  --- tx tail ---"; tail -6 "$TXLOG"
fi
