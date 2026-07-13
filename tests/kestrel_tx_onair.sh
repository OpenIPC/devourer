#!/usr/bin/env bash
# kestrel_tx_onair.sh — M4 TX first-light on-air validation.
# devourer TX on the RTL8852BU (35bc:0108, txdemo, InitWrite + canonical beacon
# SA 57:42:75:05:d6:00) -> devourer RX witness on the RTL8812AU (0bda:8812,
# rxdemo, counts rx.txhit on that SA). Mirror of kestrel_inject_sanity.sh with
# devourer on both sides. Passing = the 8852BU actually radiates devourer-pushed
# frames (the kernel-RX cell is already proven authoritative for this chip).
#
#   sudo tests/kestrel_tx_onair.sh [channel] [seconds]
set -u
cd "$(dirname "$0")/.."
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }
CH=${1:-6}; DUR=${2:-8}
SA="57:42:75:05:d6:00"
TX_ID="35bc:0108"      # RTL8852BU (devourer txdemo)
RX_ID="0bda:8812"      # RTL8812AU (devourer rxdemo witness)
TX_HUB="3-2.3"; TX_PORT="3"   # uhubctl VBUS map for the 8852BU
RXLOG="/tmp/kestrel_tx_rx.jsonl"; TXLOG="/tmp/kestrel_tx_tx.log"
[ -x build/txdemo ] && [ -x build/rxdemo ] || { echo "FAIL: build txdemo+rxdemo"; exit 2; }

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
  pkill -9 -f "build/txdemo" 2>/dev/null || true
  pkill -9 -f "build/rxdemo" 2>/dev/null || true
  # rebind rtw88 to the 8812AU witness so the rig is left as found
  local d i; d=$(sysdir_for "$RX_ID")
  [ -n "$d" ] && for i in "$d":*; do
    [ -d "$i" ] && echo "$(basename "$i")" > /sys/bus/usb/drivers/rtw88_8812au/bind 2>/dev/null || true
  done
}
trap cleanup EXIT

echo ">> VBUS-cycling the 8852BU TX ($TX_ID) for a clean devourer bring-up"
uhubctl -l "$TX_HUB" -p "$TX_PORT" -a cycle -d 2 >/dev/null 2>&1
sleep 6

# --- RX witness: 8812AU devourer rxdemo ---
unbind_kernel "$RX_ID"; sleep 1
echo ">> starting devourer rxdemo witness on the 8812AU (ch$CH)"
DEVOURER_VID=0x0bda DEVOURER_PID=0x8812 DEVOURER_CHANNEL=$CH \
  DEVOURER_EVENTS=stdout DEVOURER_LOG_LEVEL=warn \
  build/rxdemo >"$RXLOG" 2>/dev/null &
RXPID=$!
sleep 5
kill -0 "$RXPID" 2>/dev/null || { echo "FAIL: rxdemo witness died (see $RXLOG)"; exit 2; }

# --- TX: 8852BU devourer txdemo, canonical beacon ---
unbind_kernel "$TX_ID"; sleep 1
echo ">> devourer txdemo on the 8852BU for ${DUR}s (canonical beacon, 6M)"
DEVOURER_VID=0x35bc DEVOURER_PID=0x0108 DEVOURER_CHANNEL=$CH \
  DEVOURER_TX_GAP_US=2000 DEVOURER_LOG_LEVEL=info \
  build/txdemo >"$TXLOG" 2>&1 &
TXPID=$!
sleep "$DUR"
kill "$TXPID" 2>/dev/null; TXPID=""
sleep 1
kill "$RXPID" 2>/dev/null; RXPID=""

# --- verdict ---
TX_UP=$(grep -c "TX ready" "$TXLOG" 2>/dev/null || echo 0)
TX_SENT=$(grep -cE "send_packet|tx\.|sent" "$TXLOG" 2>/dev/null || echo "?")
HITS=$(grep -c '"ev":"rx.txhit"' "$RXLOG" 2>/dev/null || echo 0)
echo "=================================================================="
echo "M4 TX on-air (ch$CH, ${DUR}s):  txdemo TX-ready=$TX_UP"
echo "  8812AU witness rx.txhit (canonical SA $SA): $HITS"
if [ "$TX_UP" -lt 1 ]; then
  echo "RESULT: TX bring-up FAILED — see $TXLOG"; tail -5 "$TXLOG"
elif [ "${HITS:-0}" -gt 0 ]; then
  echo "RESULT: PASS — the 8852BU radiates devourer-pushed frames (M4 TX first light)."
else
  echo "RESULT: TX came up but ZERO frames witnessed — descriptor/endpoint/rate needs work."
  echo "  --- txdemo tail ---"; tail -8 "$TXLOG"
fi
