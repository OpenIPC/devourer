#!/usr/bin/env bash
# On-air confirmation for Jaguar2 thermal tracking: with the
# tracking tick running, the canonical txdemo beacon must still air and be
# received. TX = 8822B (T3U), RX = known-good 8812AU monitor, count rx.txhit.
#
# Usage: sudo tests/jaguar2_thermal_track_onair.sh [channel] [secs]
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
BUILD="$HERE/../build"
TX_VID=0x2357 TX_PID=0x012d   # T3U (8822B) transmitter
RX_VID=0x0bda RX_PID=0x8812   # 8812AU known-good monitor receiver
# Default ch6 @ 1M: max link budget for two close-in bus dongles, and it
# exercises the 2.4 GHz thermal swing-table branch. 5 GHz needs the SDR bench
# (the close-pair 5 GHz link budget is too tight for a monitor-RX txhit).
CH="${1:-6}" SECS="${2:-12}" RATE="${DEVOURER_TX_RATE:-1M}"

TXPID="" RXPID="" RXLOG=$(mktemp)
cleanup() {
  [[ -n "$TXPID" ]] && kill -INT "$TXPID" 2>/dev/null
  [[ -n "$RXPID" ]] && kill -INT "$RXPID" 2>/dev/null
  sleep 0.3; pkill -KILL -x txdemo 2>/dev/null; pkill -KILL -x rxdemo 2>/dev/null
  rm -f "$RXLOG"
}
trap cleanup EXIT INT TERM

echo "== RX 8812AU ch$CH =="
DEVOURER_VID=$RX_VID DEVOURER_PID=$RX_PID DEVOURER_CHANNEL=$CH \
  timeout $((SECS + 4)) "$BUILD/rxdemo" >"$RXLOG" 2>/dev/null &
RXPID=$!
sleep 3

echo "== TX 8822B ch$CH $RATE, thermal tracking ON =="
DEVOURER_VID=$TX_VID DEVOURER_PID=$TX_PID DEVOURER_CHANNEL=$CH \
  DEVOURER_TX_RATE=$RATE DEVOURER_TX_GAP_US=2000 DEVOURER_THERMAL_TRACK=1 \
  timeout "$SECS" "$BUILD/txdemo" >/dev/null 2>&1 &
TXPID=$!
wait "$TXPID" 2>/dev/null; TXPID=""
sleep 1
kill -INT "$RXPID" 2>/dev/null; wait "$RXPID" 2>/dev/null; RXPID=""

HITS=$(grep -c '"ev":"rx.txhit"' "$RXLOG")
echo "rx.txhit received: $HITS"
if [[ "$HITS" -ge 1 ]]; then
  echo "RESULT: PASS (canonical beacon aired + received with tracking live)"
else
  echo "RESULT: FAIL (no txhit — TX broke with tracking on)"; exit 1
fi
