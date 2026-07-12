#!/usr/bin/env bash
# la_cw_score.sh — establish the LA-capture IQ packing empirically (issue
# #150). B210 radiates a CW at a known offset from the DUT's channel
# center; the DUT takes one LA capture; tools/la_decode.py scores every
# candidate bit-layout — the correct one shows a single sharp FFT line at
# the expected offset.
#
#   sudo tests/la_cw_score.sh <DUT_PID> [CH] [OFFSET_MHZ] [DUT_VID]
#   sudo tests/la_cw_score.sh 0x012d 6 2 0x2357     # 8822BU (T3U)
set -uo pipefail
cd "$(dirname "$0")/.."

DUT_PID="${1:?usage: $0 <DUT_PID> [ch] [offset_mhz] [dut_vid]}"
CH="${2:-6}"
OFF="${3:-2}"
DUT_VID="${4:-0x0bda}"
OUT=/tmp/la-cw
mkdir -p "$OUT"

# sdr_interferer's cw mode bakes a +2.5 MHz baseband tone into the buffer —
# ask for (center + OFF - 2.5) so the on-air tone lands at center + OFF.
FREQ=$(python3 -c "print((2407+5*$CH if $CH<=14 else 5000+5*$CH) + $OFF - 2.5)")

cleanup() { pkill -9 -x rxdemo 2>/dev/null; pkill -9 -f sdr_interferer 2>/dev/null; }
trap cleanup EXIT
cleanup; sleep 1

echo "== B210 CW at ${FREQ} MHz (ch $CH center + ${OFF} MHz) =="
sudo python3 tests/sdr_interferer.py --freq "${FREQ}e6" --mode cw \
    --tx-gain 60 --secs 60 >"$OUT/cw.log" 2>&1 &
sleep 6

echo "== DUT capture (manual trigger, 20 Msps) =="
sudo timeout 45 env DEVOURER_VID="$DUT_VID" DEVOURER_PID="$DUT_PID" \
    DEVOURER_CHANNEL="$CH" DEVOURER_LOG_LEVEL=info \
    DEVOURER_LA_CAPTURE=manual/20M/dma0/port:0x880 \
    DEVOURER_LA_OUT="$OUT/cw.bin" DEVOURER_LA_MAX=8192 \
    build/rxdemo >"$OUT/cw.jsonl" 2>"$OUT/rx.log" &
RXPID=$!
sleep 35; sudo kill -INT "$RXPID" 2>/dev/null; wait "$RXPID" 2>/dev/null
cleanup

grep -F '"ev":"la.capture"' "$OUT/cw.jsonl" || { echo "NO CAPTURE"; exit 1; }
echo
echo "== layout scores (expected peak at +${OFF} MHz) =="
python3 tools/la_decode.py score "$OUT/cw.bin" --offset-mhz "$OFF"
