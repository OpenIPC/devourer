#!/usr/bin/env bash
# fast_bw_parity.sh — validate FastSetBandwidth (the lean same-channel
# 20<->5/10 MHz toggle) on a plugged adapter, three ways:
#   1. timing     — full SetMonitorChannel vs fast FastSetBandwidth (median us)
#   2. parity     — the BW re-clock registers (0x8ac / on J2 0x8c4/0x8c8) come
#                   out bit-for-bit identical to the full narrowband path
#   3. cross-RX   — with a narrowband TX partner, the RX decodes ONLY in the
#                   fast-toggled narrowband window (different clock domain from
#                   20 MHz), proving the fast switch re-clocks on-air
#
# Builds the two throwaway harnesses against the already-built static lib.
#
#   sudo tests/fast_bw_parity.sh <RX_VID> <RX_PID> [CH] [TX_PID] [NB]
# e.g. Jaguar2 8822B RX + 8812AU 10 MHz TX partner:
#   sudo tests/fast_bw_parity.sh 0x2357 0x012d 36 0x8812 10
set -uo pipefail
cd "$(dirname "$0")/.."

RX_VID="${1:-0x0bda}"
RX_PID="${2:?usage: $0 <RX_VID> <RX_PID> [ch] [tx_pid] [nb]}"
CH="${3:-36}"
TX_PID="${4:-0x8812}"   # narrowband TX partner (a second adapter)
NB="${5:-10}"
LIBS="$(pkg-config --cflags --libs libusb-1.0)"

build() { g++ -std=c++20 -O2 -Isrc -Iexamples/common "tests/$1.cpp" \
    examples/common/env_config.cpp build/libdevourer.a $LIBS -lpthread \
    -o "build/$1" || { echo "build $1 failed"; exit 1; }; }
build retune_bench
build fast_bw_rxcheck

cleanup() { pkill -9 -x retune_bench 2>/dev/null; pkill -9 -x fast_bw_rxcheck 2>/dev/null;
            pkill -9 -x txdemo 2>/dev/null; }
trap cleanup EXIT
cleanup; sleep 1

echo "############ 1. TIMING (full vs fast) ############"
sudo env DEVOURER_VID="$RX_VID" DEVOURER_PID="$RX_PID" DEVOURER_CHANNEL="$CH" \
    DEVOURER_LOG_LEVEL=silent build/retune_bench 15 2>/dev/null | grep -A6 "bandwidth-switch"
cleanup; sleep 1

echo
echo "############ 2. REGISTER PARITY (0x8ac full vs fast) ############"
sudo env DEVOURER_VID="$RX_VID" DEVOURER_PID="$RX_PID" DEVOURER_CHANNEL="$CH" \
    PARITY=1 DEVOURER_DUMP_CANARY=1 DEVOURER_LOG_LEVEL=info build/retune_bench 2>&1 |
  grep -iE "PARITY_MARK|BB 0x8ac |BB 0x8c4 |BB 0x8c8 "
cleanup; sleep 1

echo
echo "############ 3. CROSS-RX (fast-toggle decodes narrowband on-air) ############"
echo "TX partner $TX_PID at ${NB} MHz narrowband; RX $RX_PID toggles via fast path"
sudo env DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="$CH" DEVOURER_NB_BW="$NB" \
    DEVOURER_TX_GAP_US=0 DEVOURER_LOG_LEVEL=silent build/txdemo >/tmp/fbw-tx.log 2>&1 &
sleep 4
sudo env DEVOURER_VID="$RX_VID" DEVOURER_PID="$RX_PID" DEVOURER_CHANNEL="$CH" \
    DEVOURER_LOG_LEVEL=silent build/fast_bw_rxcheck "$NB" 2>/dev/null
