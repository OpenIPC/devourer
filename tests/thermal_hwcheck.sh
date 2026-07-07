#!/usr/bin/env bash
# Hardware smoke-test for the thermal monitor probe.
#
# Runs txdemo against each plugged Jaguar adapter for a few seconds
# with DEVOURER_THERMAL_POLL_MS enabled, and prints the thermal event
# lines it emits ({"ev":"thermal",...} on stdout). Read-only w.r.t. the
# probe — this just confirms the thermal meter reads back a live,
# plausible value per chip.
#
# Usage: sudo tests/thermal_hwcheck.sh
set -u

BUILD_DIR="$(cd "$(dirname "$0")/.." && pwd)/build"
TXDEMO="$BUILD_DIR/txdemo"
RUN_SECS=6
POLL_MS=500          # ~ every 250 TX frames inline
WARN_DELTA=15

CHILD_PID=""
cleanup() {
  if [[ -n "$CHILD_PID" ]] && kill -0 "$CHILD_PID" 2>/dev/null; then
    kill -INT "$CHILD_PID" 2>/dev/null
    sleep 0.3
    kill -KILL "$CHILD_PID" 2>/dev/null
  fi
  # Backstop: reap any stray demo by exact comm name.
  pkill -KILL -x txdemo 2>/dev/null
}
trap cleanup EXIT INT TERM

if [[ ! -x "$TXDEMO" ]]; then
  echo "ERROR: $TXDEMO not built — run: cmake --build build -j" >&2
  exit 1
fi

# pid -> human label
declare -A CHIPS=(
  [0x8812]="RTL8812AU"
  [0x8813]="RTL8814AU"
)
# 8821AU is OEM-rebadged on the T2U Plus (2357:0120) — needs VID override.
declare -A VID_OVERRIDE=(
  [0x0120]="0x2357"
)
declare -A CHIPS_OEM=(
  [0x0120]="RTL8821AU (T2U Plus)"
)

run_one() {
  local pid="$1" label="$2" vid="${3:-0x0bda}"
  echo
  echo "==================================================================="
  echo "  $label  (VID=$vid PID=$pid)  — ${RUN_SECS}s"
  echo "==================================================================="
  local log
  log="$(mktemp)"
  DEVOURER_VID="$vid" DEVOURER_PID="$pid" \
    DEVOURER_THERMAL_POLL_MS="$POLL_MS" \
    DEVOURER_THERMAL_WARN_DELTA="$WARN_DELTA" \
    "$TXDEMO" >"$log" 2>&1 &
  CHILD_PID=$!
  sleep "$RUN_SECS"
  if kill -0 "$CHILD_PID" 2>/dev/null; then
    kill -INT "$CHILD_PID" 2>/dev/null; sleep 0.3
    kill -KILL "$CHILD_PID" 2>/dev/null
  fi
  wait "$CHILD_PID" 2>/dev/null
  CHILD_PID=""

  echo "--- thermal monitor lines ---"
  # thermal events land on stdout; the human diagnostics (devourer [I]/[W]
  # "thermal: ...") on stderr — the log captures both (2>&1).
  grep -E '"ev":"thermal"|thermal:|ThermalMeter|thermal monitor on' "$log" | head -20
  local n
  n="$(grep -c -F '"ev":"thermal"' "$log")"
  echo "--- ($n thermal event lines total) ---"
  if [[ "$n" -eq 0 ]]; then
    echo "    no thermal lines — tail of log for context:"
    tail -15 "$log" | sed 's/^/    /'
  fi
  rm -f "$log"
}

for pid in "${!CHIPS[@]}"; do
  run_one "$pid" "${CHIPS[$pid]}"
done
for pid in "${!CHIPS_OEM[@]}"; do
  run_one "$pid" "${CHIPS_OEM[$pid]}" "${VID_OVERRIDE[$pid]}"
done

echo
echo "done."
