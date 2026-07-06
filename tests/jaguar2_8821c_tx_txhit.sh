#!/usr/bin/env bash
# TX validation (functional): does the RTL8821C (CF-811AC) TRANSMIT on air?
# Inject the canonical txdemo beacon FROM the 8821C and receive on the known-good
# Jaguar-1 8821AU (2357:0120); each <devourer-tx-hit> on the RX side proves an
# 8821C-transmitted frame flew over the air. Restores kernel drivers on exit.
#
# Usage: sudo tests/jaguar2_8821c_tx_txhit.sh [channel] [seconds]
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
RXDEMO="$HERE/../build/rxdemo"
TXDEMO="$HERE/../build/txdemo"
CH=${1:-1} SECS=${2:-15}
TX_VID=0x0bda TX_PID=0xc811   # CF-811AC (8821C) transmitter
RX_VID=0x2357 RX_PID=0x0120   # T2U-Plus 8821AU (Jaguar1) receiver
RXLOG=$(mktemp /tmp/8821c-tx.XXXX.log)
restored=0

cleanup() {
    [ "$restored" = 1 ] && return
    restored=1
    pkill -f txdemo 2>/dev/null
    pkill -f rxdemo 2>/dev/null
    sleep 1
    modprobe rtw88_8821cu rtw88_8821au 2>/dev/null
    echo "=== kernel drivers restored ==="
}
trap cleanup EXIT INT TERM

[ -x "$RXDEMO" ] && [ -x "$TXDEMO" ] || { echo "FATAL: build demos first"; exit 1; }

echo "=== unbind kernel drivers ==="
modprobe -r rtw88_8821cu rtw88_8821au 2>&1
sleep 2

echo "=== start TX beacon on CF-811AC (8821C) ch$CH ${DEVOURER_TX_RATE:-1M} ==="
DEVOURER_VID=$TX_VID DEVOURER_PID=$TX_PID DEVOURER_CHANNEL=$CH \
    DEVOURER_TX_RATE=${DEVOURER_TX_RATE:-1M} timeout $((SECS + 5)) "$TXDEMO" \
    >/tmp/8821c-txside.log 2>&1 &
sleep 5  # let the 8821C TX bring-up finish + start injecting

echo "=== RX on 8821AU (Jaguar1) ch$CH for ${SECS}s ==="
DEVOURER_VID=$RX_VID DEVOURER_PID=$RX_PID DEVOURER_CHANNEL=$CH \
    timeout "$SECS" "$RXDEMO" >"$RXLOG" 2>&1

echo "--- TX side (8821C) ---"
grep -iE "ready for TX|Creating|LCK|channel set|error|throw" /tmp/8821c-txside.log | head -5
echo "--- RX side (8821AU) ---"
HITS=$(grep -c "<devourer-tx-hit>" "$RXLOG")
grep -E "<devourer-tx-hit>" "$RXLOG" | head -3
echo "$(grep -oE "RX loop exited \([0-9]+ frames, [0-9]+ reads" "$RXLOG" | head -1)"
echo "--- 8821C-TX tx-hits: $HITS ---"
if [ "$HITS" -ge 1 ]; then
    echo "FUNCTIONAL PASS: 8821C transmitted frames over the air"
else
    echo "TX not confirmed (no hit). TX log /tmp/8821c-txside.log, RX $RXLOG"
fi
