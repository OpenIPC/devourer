#!/usr/bin/env bash
# TX-side A/B of two devourer trees on one Jaguar3 DUT: N reps of a timed
# txdemo flood per tree, reporting per rep the frames submitted, the bulk-OUT
# failures and the failed synchronous register reads. Alternates trees rep by
# rep so a slow drift in the unit (warm state, ambient) lands on both sides.
#
#   sudo tests/j3_tx_flood_ab.sh <pid-hex> <channel> <treeA> <treeB> [reps] [secs]
set -euo pipefail
PID=${1:?pid}; CH=${2:?channel}; A=${3:?treeA}; B=${4:?treeB}; REPS=${5:-3}; SECS=${6:-15}
OUT=${OUT:-/tmp/j3_tx_flood_ab/pid${PID}_ch${CH}}; mkdir -p "$OUT"
run() {
  # Logs and lines are keyed by side (A/B) so two checkouts that share a
  # directory name cannot overwrite each other; the basename is context.
  local side=$1 tree=$2 rep=$3 n; n="$side:$(basename "$tree")"
  local log=$OUT/${side}_rep${rep}
  local rc=0
  env DEVOURER_PID="$PID" DEVOURER_CHANNEL="$CH" DEVOURER_LOG_LEVEL=info \
    timeout -s INT "$SECS" "$tree/build/txdemo" >"$log.jsonl" 2>"$log.err" || rc=$?
  # 124 = timeout fired (the normal end), 0/130 = txdemo took the INT itself.
  case $rc in 0|124|130) ;; *) echo "$n rep$rep: txdemo exited rc=$rc"; tail -3 "$log.err"; failed=1;; esac
  local submitted fail rd first
  # Submitted frames: the final tx.stats event ("final":1) carries the
  # cumulative tally; a run without one did not finish and is a failure.
  submitted=$(grep -F '"ev":"tx.stats"' "$log.jsonl" | grep -F '"final":1' | tail -n1 \
              | grep -oE '"submitted":[0-9]+' | grep -oE '[0-9]+$' || true)
  [ -n "$submitted" ] || { submitted='?'; echo "$n rep$rep: no final tx.stats event"; failed=1; }
  fail=$(grep -c 'bulk_send EP .* FAIL' "$log.err" || true)
  rd=$(grep -c 'rtw_read(' "$log.err" || true)
  first=$(grep -F '"ev":"init.timing"' "$log.jsonl" | grep -F '"stage":"txdemo.first_tx_submit"' \
          | grep -oE '"ms":[0-9]+' | grep -oE '[0-9]+$' | head -n1 || true)
  [ -n "$first" ] || { first='?'; echo "$n rep$rep: no txdemo.first_tx_submit event"; failed=1; }
  echo "$n rep$rep: first_tx_ms=$first submitted=$submitted bulk_fail=$fail read_fail=$rd"
}
failed=0
for r in $(seq 1 "$REPS"); do run A "$A" "$r"; run B "$B" "$r"; done
# Every rep is still reported, but a crashed or failed run fails the script.
exit "$failed"
