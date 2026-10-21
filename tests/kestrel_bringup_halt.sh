#!/usr/bin/env bash
# Kestrel bring-up firmware-halt count: N back-to-back txdemo bring-ups, each
# graded by the halt check at the end of bring-up (BringUpCheckingFwHalt logs
# "firmware halted during ..." per halted attempt). The halt it guards is the
# 8852C RX-DMA-stuck one (HAXI_IDCT 0x2 -> L1 0x999 -> fw assert 0x30000024)
# that an undrained bulk-IN during bring-up raises; see src/kestrel/CLAUDE.md.
#
# It is timing-sensitive: debug logging slows the H2C stream enough to hide
# it, so the bring-ups run at log level warn. A/B: build once as is and once
# with the start_wp_drain() call in BringUpMonitorOnce removed (BUILD=...).
#
#   sudo DEVOURER_PID=0x0101 bash tests/kestrel_bringup_halt.sh      # 8852C
#   sudo DEVOURER_PID=0x0108 N=20 bash tests/kestrel_bringup_halt.sh # 8852B
#
# Exit 0 when every run completed (txdemo exit 0) and no attempt halted, 1
# otherwise. A run that exits non-zero for any other reason (adapter absent,
# open or bring-up error, timeout) tested nothing and counts as an error.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD=${BUILD:-$ROOT/build}
OUT=${OUT:-/tmp/kestrel-bringup-halt}
N=${N:-30}
CH=${CH:-149}
VID=${DEVOURER_VID:-0x35bc}; PID=${DEVOURER_PID:-0x0101}
SUDO=${SUDO-sudo}
TXDEMO="$BUILD/txdemo"; [ -x "$TXDEMO" ] || TXDEMO="$BUILD/txdemo.exe"
mkdir -p "$OUT"

runs_halted=0 attempts_halted=0 runs_failed=0 runs_errored=0 attempts=0
for i in $(seq 1 "$N"); do
  log="$OUT/run$i.log"
  $SUDO env DEVOURER_VID=$VID DEVOURER_PID=$PID DEVOURER_CHANNEL=$CH \
       DEVOURER_TX_FRAMES=200 DEVOURER_LOG_LEVEL=warn DEVOURER_EVENTS=off \
       timeout 120 "$TXDEMO" >"$log" 2>&1
  rc=$?
  h=$(grep -c "firmware halted during" "$log" || true)
  f=$(grep -c "firmware halted on all" "$log" || true)
  attempts_halted=$((attempts_halted + h))
  [ "$h" -gt 0 ] && runs_halted=$((runs_halted + 1))
  note=""
  if [ "$f" -gt 0 ]; then
    attempts=$((attempts + h)); runs_failed=$((runs_failed + 1))
    note=" (all attempts halted)"
  elif [ "$rc" -ne 0 ]; then
    attempts=$((attempts + h)); runs_errored=$((runs_errored + 1))
    note=" (ERROR: txdemo failed without a halt; see $log)"
  else
    attempts=$((attempts + h + 1))
  fi
  echo "run $i: rc=$rc halted_attempts=$h$note"
  grep -h "FW-SER" "$log" | sed 's/^/    /' | sort -u
done
echo "=== $VID:$PID ch$CH: $runs_halted of $N runs halted at least once," \
     "$attempts_halted of $attempts attempts, $runs_failed runs failed,"      "$runs_errored runs errored"
[ "$attempts_halted" -eq 0 ] && [ "$runs_errored" -eq 0 ]
