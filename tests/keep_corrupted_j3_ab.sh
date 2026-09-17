#!/usr/bin/env bash
# A/B for DEVOURER_RX_KEEP_CORRUPTED on a Jaguar3 DUT: run rxdemo twice on the
# same channel (knob off, then on), count frames the host saw with the RX
# descriptor's crc_err bit set. Expected: 0 with the knob off (WMAC drops them),
# non-zero with it on — given any ambient traffic weak enough to fail FCS.
#
#   sudo tests/keep_corrupted_j3_ab.sh <pid-hex> [channel] [seconds]
set -euo pipefail
PID=${1:?usage: $0 <pid-hex e.g. 0xc812> [channel] [seconds]}
CH=${2:-6}
SECS=${3:-20}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
RX=$ROOT/build/rxdemo
OUT=${OUT:-/tmp/keep_corrupted_j3_ab}
mkdir -p "$OUT"
pids=()
cleanup() { for p in "${pids[@]:-}"; do [ -n "$p" ] && kill -INT "$p" 2>/dev/null || true; done; }
trap cleanup EXIT

run_cell() {
  local knob=$1
  local log=$OUT/pid${PID}_keep${knob}.jsonl
  # The off arm scrubs the knob from the environment (env -u) rather than
  # relying on the caller not having exported it; the on arm sets it. Events
  # are forced to stdout because that is the stream the counts come from.
  local env=(-u DEVOURER_RX_KEEP_CORRUPTED
             DEVOURER_PID="$PID" DEVOURER_CHANNEL="$CH" DEVOURER_RX_DUMP_ALL=1
             DEVOURER_EVENTS=stdout DEVOURER_LOG_LEVEL=warn)
  [ "$knob" = 1 ] && env+=(DEVOURER_RX_KEEP_CORRUPTED=1)
  env "${env[@]}" "$RX" >"$log" 2>"$log.err" &
  local p=$!; pids+=("$p")
  sleep "$SECS"
  # A cell whose rxdemo is no longer running did not measure anything (device
  # open / claim / bring-up failed) — refuse to print a zero as a result.
  if ! kill -0 "$p" 2>/dev/null; then
    local rc=0; wait "$p" 2>/dev/null || rc=$?
    echo "FAIL: pid=$PID keep_corrupted=$knob rxdemo exited early (rc=$rc); stderr tail:" >&2
    tail -n 5 "$log.err" >&2
    exit 1
  fi
  kill -INT "$p" 2>/dev/null || true; wait "$p" 2>/dev/null || true
  local total crc
  total=$(grep -c -F '"ev":"rx.corrupt"' "$log" || true)
  crc=$(grep -F '"ev":"rx.corrupt"' "$log" | grep -c -F '"crc":1' || true)
  echo "pid=$PID keep_corrupted=$knob ch=$CH secs=$SECS frames=$total crc_err=$crc"
}
run_cell 0
run_cell 1
