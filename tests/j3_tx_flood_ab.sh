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
  local tree=$1 rep=$2 n; n=$(basename "$tree")
  local log=$OUT/${n}_rep${rep}
  local rc=0
  env DEVOURER_PID="$PID" DEVOURER_CHANNEL="$CH" DEVOURER_LOG_LEVEL=info \
    timeout -s INT "$SECS" "$tree/build/txdemo" >"$log.jsonl" 2>"$log.err" || rc=$?
  # 124 = timeout fired (the normal end), 0/130 = txdemo took the INT itself.
  case $rc in 0|124|130) ;; *) echo "$n rep$rep: txdemo exited rc=$rc"; tail -3 "$log.err"; failed=1;; esac
  local tx fail rd first
  tx=$(grep -cF '"ev":"tx.' "$log.jsonl" || true)
  fail=$(grep -c 'bulk_send EP .* FAIL' "$log.err" || true)
  rd=$(grep -c 'rtw_read(' "$log.err" || true)
  first=$(grep -oE 'first_tx_submit","ms":[0-9]+' "$log.jsonl" | grep -oE '[0-9]+$' || echo '?')
  echo "$n rep$rep: first_tx_ms=$first tx_events=$tx bulk_fail=$fail read_fail=$rd"
}
failed=0
for r in $(seq 1 "$REPS"); do run "$A" "$r"; run "$B" "$r"; done
# Every rep is still reported, but a crashed or failed run fails the script.
exit "$failed"
