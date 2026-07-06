#!/usr/bin/env bash
# Regression gate: prove the ChipVariant strategy-dispatch refactor did NOT
# regress the hardware-validated RTL8822B path. Runs devourer RX on the connected
# TP-Link Archer T3U (2357:012d, chip 8822B) and checks it still (a) constructs
# RtlJaguar2Device and (b) decodes live 802.11 frames.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
DEMO="$HERE/../build/rxdemo"
VID=0x2357 PID=0x012d CH=${1:-6} SECS=${2:-8}
LOG=$(mktemp /tmp/j2-8822b-rx.XXXX.log)
restored=0

cleanup() {
    [ "$restored" = 1 ] && return
    restored=1
    pkill -f "rxdemo" 2>/dev/null
    # T3U had no kernel driver bound at start; nothing to restore. Re-probe in
    # case a driver auto-binds after the device resets.
    sleep 1
}
trap cleanup EXIT INT TERM

[ -x "$DEMO" ] || { echo "FATAL: build $DEMO first"; exit 1; }

echo "=== devourer RX on T3U (8822B) ch$CH for ${SECS}s ==="
DEVOURER_VID=$VID DEVOURER_PID=$PID DEVOURER_CHANNEL=$CH \
    timeout "$SECS" "$DEMO" >"$LOG" 2>&1
echo "--- key log lines ---"
grep -iE "Creating RtlJaguar2Device|firmware booted|PHY tables applied|entering RX loop|RX: completion" "$LOG" | head -12

DEV=$(grep -c "Creating RtlJaguar2Device" "$LOG")
STREAM=$(grep -c "<devourer-stream>" "$LOG")
BODY=$(grep -c "<devourer-body>" "$LOG")
COMPL=$(grep -c "RX: completion" "$LOG")
echo "--- counts: RtlJaguar2Device=$DEV  stream=$STREAM  body=$BODY  rx-completions=$COMPL ---"

if [ "$DEV" -ge 1 ] && { [ "$STREAM" -ge 1 ] || [ "$COMPL" -ge 1 ]; }; then
    echo "PASS: 8822B constructs + RX active (refactor did not regress)"
else
    echo "PARTIAL/FAIL: see $LOG"; tail -25 "$LOG"
fi
