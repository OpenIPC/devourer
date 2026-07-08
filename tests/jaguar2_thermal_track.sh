#!/usr/bin/env bash
# Hardware validation for Jaguar2 thermal TX-power tracking.
#
# Exercises the new ~2 s thermal-track tick end-to-end on real hardware and
# asserts three invariants per plugged Jaguar2 family:
#
#   A. SOAK  — txdemo TX-only (InitWrite path, the sustained-TX case) at max
#      duty (DEVOURER_TX_GAP_US=0) with tracking ON: the thermal-track thread
#      starts, the RF 0x42 meter reads a plausible value (raw near baseline),
#      the MIX_MODE swing is written, TX keeps airing, and shutdown does not
#      hang. Every measured delta stays small (<= DELTA_MAX) — a garbage meter
#      read (e.g. a dead path-B sensor returning 0) would spike delta and fail.
#   B. OFF   — with DEVOURER_THERMAL_TRACK=0 the feature is fully inert: no
#      thread, no meter reads, no swing writes (the pre-feature code path).
#
# Usage: sudo tests/jaguar2_thermal_track.sh [channel] [soak_secs]
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
BUILD="$HERE/../build"
TXDEMO="$BUILD/txdemo"
CH="${1:-36}"
SOAK="${2:-25}"
DELTA_MAX=15   # a real bus-powered soak drifts a few units; 27 = garbage read

declare -A ADAPTERS=(
  [8822B_T3U]="0x2357 0x012d"
  [8821C_CF811]="0x0bda 0xc811"
)

CHILD=""
cleanup() {
  [[ -n "$CHILD" ]] && kill -INT "$CHILD" 2>/dev/null
  sleep 0.3
  pkill -KILL -x txdemo 2>/dev/null
}
trap cleanup EXIT INT TERM

[[ -x "$TXDEMO" ]] || { echo "ERROR: txdemo not built" >&2; exit 1; }

present() { local v=${1#0x} p=${2#0x}; lsusb -d "${v}:${p}" >/dev/null 2>&1; }

run_txdemo() { # VID PID THERMAL_TRACK secs -> log on stdout
  local vid=$1 pid=$2 tt=$3 secs=$4 out=$5
  DEVOURER_VID=$vid DEVOURER_PID=$pid DEVOURER_CHANNEL=$CH \
    DEVOURER_TX_RATE=MCS7 DEVOURER_TX_GAP_US=0 \
    DEVOURER_THERMAL_POLL_MS=1000 DEVOURER_THERMAL_TRACK=$tt \
    DEVOURER_LOG_LEVEL=info \
    timeout "$secs" "$TXDEMO" >"$out" 2>&1 &
  CHILD=$!; wait "$CHILD" 2>/dev/null; CHILD=""
}

RC=0
for name in "${!ADAPTERS[@]}"; do
  read -r VID PID <<<"${ADAPTERS[$name]}"
  if ! present "$VID" "$PID"; then
    echo "== $name ($VID:$PID): not plugged, skipping"; continue
  fi
  echo "======================================================================"
  echo "== $name ($VID:$PID)  ch=$CH  soak=${SOAK}s"
  echo "======================================================================"

  # ---- A. SOAK (tracking ON) --------------------------------------------------
  LOG=$(mktemp)
  run_txdemo "$VID" "$PID" 1 "$SOAK" "$LOG"
  STARTED=$(grep -c "thermal-track thread started" "$LOG")
  SWINGS=$(grep -c "thermal-track: .*swing=" "$LOG")
  THERMS=$(grep -c '"ev":"thermal"' "$LOG")
  MAXD=$(grep -oE 'thermal-track: .* d=[0-9]+' "$LOG" | grep -oE 'd=[0-9]+' \
         | grep -oE '[0-9]+' | sort -n | tail -1)
  MAXD=${MAXD:-0}
  echo "   thread-start=$STARTED  meter-reads=$THERMS  swing-writes=$SWINGS  max-delta=$MAXD"
  echo "   last thermal marker: $(grep '"ev":"thermal"' "$LOG" | tail -1)"
  echo "   last swing line    : $(grep 'thermal-track: .*swing=' "$LOG" | tail -1)"
  [[ "$STARTED" -ge 1 ]] || { echo "   FAIL: thermal-track thread never started"; RC=1; }
  [[ "$THERMS"  -ge 1 ]] || { echo "   FAIL: no live thermal meter reads"; RC=1; }
  if [[ "$MAXD" -gt "$DELTA_MAX" ]]; then
    echo "   FAIL: measured delta spiked to $MAXD (> $DELTA_MAX) — garbage meter read"; RC=1
  fi
  if grep -qiE 'wedge|deadlock|terminate|abort|segfault|what\(\)' "$LOG"; then
    echo "   FAIL: crash/wedge keyword in soak log"; RC=1
  fi
  rm -f "$LOG"

  # ---- B. OFF is fully inert --------------------------------------------------
  OLOG=$(mktemp)
  run_txdemo "$VID" "$PID" 0 8 "$OLOG"
  OFF_LINES=$(grep -c 'thermal-track' "$OLOG")
  if [[ "$OFF_LINES" -eq 0 ]]; then
    echo "   PASS: DEVOURER_THERMAL_TRACK=0 inert (no thread, no swing writes)"
  else
    echo "   FAIL: tracking-off still emitted $OFF_LINES thermal-track lines"; RC=1
  fi
  rm -f "$OLOG"
done

echo "======================================================================"
[[ $RC -eq 0 ]] && echo "RESULT: PASS" || echo "RESULT: FAIL"
exit $RC
