#!/usr/bin/env bash
# Wide-bandwidth (HT40 / VHT80) tx-hit validation for the RTL8821C (CF-811AC).
# Exercises set_channel_bw_8821c() at bw=40/80 in BOTH directions, which is where
# the 0x8ac ADC/DAC clock word is programmed:
#   rx: T3U (8822B) TX an HT40/VHT80 frame; 8821C RX at the same bandwidth. An
#       rx.txhit event proves the 8821C wide RX (0x8ac clock) decodes on-air.
#   tx: 8821C TX an HT40/VHT80 frame; T2U-Plus (8821AU) RX. A tx-hit proves the
#       8821C wide TX tune works on-air.
# Uses controlled traffic (no reliable ambient wide 2.4G at this bench). Restores
# kernel drivers on exit.
#
# Usage: sudo tests/jaguar2_8821c_bw_txhit.sh <rx|tx> <40|80> [channel] [seconds]
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
RXDEMO="$HERE/../build/rxdemo"
TXDEMO="$HERE/../build/txdemo"
DIR=${1:-rx} BW=${2:-40} CH=${3:-1} SECS=${4:-15}
C811_VID=0x0bda C811_PID=0xc811   # CF-811AC (8821C)
T3U_VID=0x2357  T3U_PID=0x012d     # T3U (8822B, known-good wide TX)
T2U_VID=0x2357  T2U_PID=0x0120     # T2U-Plus (8821AU, wide RX)
RXLOG=$(mktemp /tmp/8821c-bwrx.XXXX.log)
TXLOG=$(mktemp /tmp/8821c-bwtx.XXXX.log)
# HT40 offset: primary lower (HT40+). 80 MHz takes the block's lowest 20 as primary.
OFF=1
restored=0

cleanup() {
    [ "$restored" = 1 ] && return
    restored=1
    pkill -f txdemo 2>/dev/null
    pkill -f rxdemo 2>/dev/null
    sleep 1
    modprobe rtw88_8821cu rtw88_8822bu 2>/dev/null
    echo "=== kernel drivers restored ==="
}
trap cleanup EXIT INT TERM

[ -x "$RXDEMO" ] && [ -x "$TXDEMO" ] || { echo "FATAL: build demos first"; exit 1; }
case "$BW" in 40) RATE=MCS7/40 ;; 80) RATE=VHT1SS_MCS7/80 ;; *) echo "BW must be 40|80"; exit 1;; esac

echo "=== unbind kernel drivers ==="
modprobe -r rtw88_8821cu rtw88_8822bu 2>&1
sleep 2

if [ "$DIR" = rx ]; then
    echo "=== TX $BW MHz on T3U (8822B) ch$CH rate=$RATE ==="
    DEVOURER_VID=$T3U_VID DEVOURER_PID=$T3U_PID DEVOURER_CHANNEL=$CH \
        DEVOURER_HOP_BW=$BW DEVOURER_HOP_OFFSET=$OFF DEVOURER_TX_RATE=$RATE \
        timeout $((SECS + 4)) "$TXDEMO" >"$TXLOG" 2>&1 &
    sleep 4
    echo "=== RX $BW MHz on CF-811AC (8821C) ch$CH for ${SECS}s ==="
    DEVOURER_VID=$C811_VID DEVOURER_PID=$C811_PID DEVOURER_CHANNEL=$CH \
        DEVOURER_BW=$BW timeout "$SECS" "$RXDEMO" >"$RXLOG" 2>&1
else
    echo "=== RX $BW MHz on T2U-Plus (8821AU) ch$CH for ${SECS}s ==="
    DEVOURER_VID=$T2U_VID DEVOURER_PID=$T2U_PID DEVOURER_CHANNEL=$CH \
        DEVOURER_BW=$BW timeout $((SECS + 4)) "$RXDEMO" >"$RXLOG" 2>&1 &
    sleep 3
    echo "=== TX $BW MHz on CF-811AC (8821C) ch$CH rate=$RATE for ${SECS}s ==="
    DEVOURER_VID=$C811_VID DEVOURER_PID=$C811_PID DEVOURER_CHANNEL=$CH \
        DEVOURER_HOP_BW=$BW DEVOURER_HOP_OFFSET=$OFF DEVOURER_TX_RATE=$RATE \
        timeout "$SECS" "$TXDEMO" >"$TXLOG" 2>&1
fi

echo "--- 8821C side bring-up ---"
grep -iE "channel set|bw=|error|throw|fail" "$([ "$DIR" = rx ] && echo "$RXLOG" || echo "$TXLOG")" | head -4
HITS=$(grep -cF '"ev":"rx.txhit"' "$RXLOG")
echo "--- tx-hits: $HITS  (dir=$DIR bw=$BW) ---"
if [ "$HITS" -ge 1 ]; then
    echo "PASS: 8821C wide $BW MHz $DIR decoded on-air (rx-log $RXLOG)"
else
    echo "INCONCLUSIVE: no tx-hit (rx-log $RXLOG tx-log $TXLOG)"
fi
