#!/usr/bin/env bash
# 8821C bring-up smoke test on the connected CF-811AC (RTL8811CU, 0bda:c811).
# Swaps out the in-kernel rtw88_8821cu, runs devourer, and reports how far the
# Jaguar2 C8821C bring-up gets (factory dispatch -> power-on -> chip-version ->
# DLFW -> ...). Restores the kernel driver on exit.
#
# Usage: sudo tests/jaguar2_8821c_bringup_smoke.sh [channel] [seconds]
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
DEMO="$HERE/../build/WiFiDriverDemo"
VID=0x0bda PID=0xc811 CH=${1:-6} SECS=${2:-10}
LOG=$(mktemp /tmp/j2-8821c.XXXX.log)
restored=0

cleanup() {
    [ "$restored" = 1 ] && return
    restored=1
    pkill -f "WiFiDriverDemo" 2>/dev/null
    echo "=== restore rtw88_8821cu ==="
    modprobe rtw88_8821cu 2>/dev/null
    sleep 1
}
trap cleanup EXIT INT TERM

[ -x "$DEMO" ] || { echo "FATAL: build $DEMO first"; exit 1; }

echo "=== unbind in-kernel rtw88_8821cu ==="
modprobe -r rtw88_8821cu 2>&1
sleep 1

echo "=== devourer on CF-811AC (8821C) ch$CH for ${SECS}s ==="
DEVOURER_VID=$VID DEVOURER_PID=$PID DEVOURER_CHANNEL=$CH \
    timeout "$SECS" "$DEMO" >"$LOG" 2>&1

echo "--- bring-up progress ---"
grep -iE "Creating RtlJaguar2Device|power-on|chip.version|cut=|rf_2t2r|firmware booted|DLFW|MAC cfg|PHY tables|channel set|IQK|RX enabled|entering RX loop|RX: completion|error|throw|fail" "$LOG" | head -30

DEV=$(grep -c "Creating RtlJaguar2Device C8821C" "$LOG")
STREAM=$(grep -c "<devourer-stream>" "$LOG")
COMPL=$(grep -c "RX: completion" "$LOG")
echo "--- counts: C8821C-construct=$DEV  stream=$STREAM  rx-completions=$COMPL ---"
if [ "$DEV" -ge 1 ]; then
    echo "M1 PASS: CF-811AC dispatches to RtlJaguar2Device C8821C"
else
    echo "M1 FAIL: did not construct C8821C — full log:"; tail -30 "$LOG"
fi
echo "(full log: $LOG)"
