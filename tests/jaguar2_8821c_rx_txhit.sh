#!/usr/bin/env bash
# RX validation: confirm the RTL8821C (CF-811AC) decodes a KNOWN over-the-air
# frame. Inject the canonical txdemo beacon (SA 57:42:75:05:d6:00) from the T3U
# (RTL8822B, known-good TX) and RX on the 8821C; a <devourer-tx-hit> proves the
# 8821C RX decoded it end-to-end. Uses controlled traffic because this bench has
# no reliable ambient 2.4G. Restores kernel drivers on exit.
#
# Usage: sudo tests/jaguar2_8821c_rx_txhit.sh [channel] [seconds]
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
RXDEMO="$HERE/../build/WiFiDriverDemo"
TXDEMO="$HERE/../build/WiFiDriverTxDemo"
CH=${1:-1} SECS=${2:-15}
TX_VID=0x2357 TX_PID=0x012d   # T3U (8822B) transmitter
RX_VID=0x0bda RX_PID=0xc811   # CF-811AC (8821C) receiver
RXLOG=$(mktemp /tmp/8821c-rxhit.XXXX.log)
restored=0

cleanup() {
    [ "$restored" = 1 ] && return
    restored=1
    pkill -f WiFiDriverTxDemo 2>/dev/null
    pkill -f WiFiDriverDemo 2>/dev/null
    sleep 1
    modprobe rtw88_8821cu 2>/dev/null
    modprobe rtw88_8822bu 2>/dev/null
    echo "=== kernel drivers restored ==="
}
trap cleanup EXIT INT TERM

[ -x "$RXDEMO" ] && [ -x "$TXDEMO" ] || { echo "FATAL: build demos first"; exit 1; }

echo "=== unbind kernel drivers ==="
modprobe -r rtw88_8821cu rtw88_8822bu 2>&1
sleep 2

echo "=== start TX beacon on T3U (8822B) ch$CH ==="
DEVOURER_VID=$TX_VID DEVOURER_PID=$TX_PID DEVOURER_CHANNEL=$CH \
    DEVOURER_TX_RATE=1M timeout $((SECS + 4)) "$TXDEMO" >/tmp/8821c-tx.log 2>&1 &
sleep 4  # let TX bring-up + start injecting

echo "=== RX on CF-811AC (8821C) ch$CH for ${SECS}s ==="
DEVOURER_VID=$RX_VID DEVOURER_PID=$RX_PID DEVOURER_CHANNEL=$CH \
    timeout "$SECS" "$RXDEMO" >"$RXLOG" 2>&1

echo "--- TX side ---"; grep -iE "ready for TX|Creating|error" /tmp/8821c-tx.log | head -3
echo "--- RX result ---"
HITS=$(grep -c "<devourer-tx-hit>" "$RXLOG")
grep -E "<devourer-tx-hit>" "$RXLOG" | head -3
echo "$(grep -oE "RX loop exited \([0-9]+ frames, [0-9]+ reads" "$RXLOG" | head -1)"
echo "--- tx-hits: $HITS ---"
if [ "$HITS" -ge 1 ]; then
    echo "PASS: 8821C RX decoded the injected canonical beacon over the air"
else
    echo "INCONCLUSIVE: no tx-hit (check TX bring-up / channel); RX log $RXLOG"
fi
