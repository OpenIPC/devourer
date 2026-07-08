#!/usr/bin/env bash
# Jaguar1 closed-loop CFO tracker smoke (#217): confirm the loop actually fires
# on a plugged Jaguar1 die — reads the phy-status cfo_tail and runs the ~2 s
# cfo.track tick without crashing under RX load. Convergence needs a marginal
# narrowband TX partner; this only proves the plumbing executes on real silicon.
#
# Usage: sudo tests/jaguar1_cfo_track_smoke.sh <PID> [CHANNEL] [SECONDS]
#   PID     0x8812 (8812AU) | 0x8813 (8814AU) | ...
set -euo pipefail

PID="${1:?usage: $0 <PID> [channel] [seconds]}"
CH="${2:-6}"
DUR="${3:-12}"
# NB=10|5 narrows the RX passband (needs a matching TX partner to decode);
# NB=0 (default) leaves 20 MHz so ambient beacons feed the loop and prove the
# tick fires on real silicon.
NB="${NB:-0}"
RX="$(dirname "$0")/../build/rxdemo"
LOG="$(mktemp /tmp/j1-cfo-smoke.XXXXXX.log)"

cleanup() { pkill -9 -x rxdemo 2>/dev/null || true; }
trap cleanup EXIT

echo "== Jaguar1 CFO-track smoke: PID=$PID ch=$CH NB=$NB ${DUR}s =="
NB_ENV=()
[ "$NB" != "0" ] && NB_ENV=(DEVOURER_NB_BW="$NB")
env "${NB_ENV[@]}" \
DEVOURER_PID="$PID" \
DEVOURER_CHANNEL="$CH" \
DEVOURER_CFO_TRACK=1 \
DEVOURER_LOG_LEVEL="${DEVOURER_LOG_LEVEL:-info}" \
  timeout "${DUR}s" "$RX" >"$LOG" 2>&1 || true

echo "--- adapter.caps ---"
grep -F '"ev":"adapter.caps"' "$LOG" | head -1 || echo "(no caps event)"
echo "--- frames seen (rx.txhit / rx totals) ---"
grep -cF '"ev":"rx' "$LOG" || true
echo "--- cfo.track tick lines (need DEVOURER_LOG_LEVEL=debug for per-tick) ---"
if grep -F 'cfo.track' "$LOG"; then
  echo "PASS: closed-loop CFO tick fired on this die"
else
  echo "NOTE: no cfo.track line — either no frames decoded (no CFO error"
  echo "signal to tick on) or AFE base read failed. Full log: $LOG"
fi
echo "(log: $LOG)"
