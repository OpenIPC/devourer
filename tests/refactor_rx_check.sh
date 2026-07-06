#!/usr/bin/env bash
# Controlled devourer-TX -> devourer-RX smoke: one adapter injects the canonical
# beacon (SA 57:42:75:05:d6:00) while each target adapter receives on the same
# channel. Confirms the RX read+parse path (and factory dispatch) works per chip
# without needing the kernel driver or ambient traffic — used to validate the
# src/jaguar1 split refactor broke no chip's RX.
#
# Usage: sudo tests/refactor_rx_check.sh <TX_PID> <CH> <RX_PID> [RX_PID ...]
set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RX_BIN="$ROOT/build/rxdemo"
TX_BIN="$ROOT/build/txdemo"
DWELL=14            # seconds each RX cell listens
TX_PID="${1:?TX pid, e.g. 0x8812}"
CH="${2:?channel, e.g. 6}"
shift 2
CANONICAL_SA_HIT="devourer-tx-hit"

TX_PROC=""
cleanup() {
  [ -n "$TX_PROC" ] && kill "$TX_PROC" 2>/dev/null
  pkill -f "txdemo" 2>/dev/null
  pkill -f "rxdemo" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "== controlled RX check: TX=$TX_PID ch=$CH =="

for RX_PID in "$@"; do
  # start the beacon source
  DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="$CH" DEVOURER_TX_RATE=6M \
    "$TX_BIN" >/tmp/rxchk-tx.log 2>&1 &
  TX_PROC=$!
  sleep 3   # let TX init + start emitting

  RX_LOG="/tmp/rxchk-rx-${RX_PID//0x/}.log"
  timeout "$DWELL" env DEVOURER_PID="$RX_PID" DEVOURER_CHANNEL="$CH" \
    "$RX_BIN" >"$RX_LOG" 2>&1

  frames=$(grep -cE "devourer-stream|RX #|first_rx_frame" "$RX_LOG" 2>/dev/null)
  hits=$(grep -cE "$CANONICAL_SA_HIT" "$RX_LOG" 2>/dev/null)
  inited=$(grep -cE "monitor RX config applied|first_rx_frame|read loop|create_device" "$RX_LOG" 2>/dev/null)
  if [ "$hits" -gt 0 ]; then
    verdict="PASS (canonical beacon received: $hits hits)"
  elif [ "$frames" -gt 0 ]; then
    verdict="PASS (RX frames captured: $frames)"
  elif [ "$inited" -gt 0 ]; then
    verdict="INIT-OK / no-frame (init succeeded, saw no frame in ${DWELL}s)"
  else
    verdict="FAIL (no init)"
  fi
  printf "  RX %-8s ch %-3s -> %s\n" "$RX_PID" "$CH" "$verdict"

  kill "$TX_PROC" 2>/dev/null; TX_PROC=""
  pkill -f "txdemo" 2>/dev/null
  sleep 2
done
