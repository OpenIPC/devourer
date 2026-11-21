#!/usr/bin/env bash
# Does a single frame whose TX descriptor + MPDU is an exact USB bulk
# max-packet-size multiple reach the air?
#
# A synchronous bulk-OUT of an exact multiple ends without a short packet, so
# the chip never sees the end of the transfer unless the host pads the frame
# off the boundary or appends a zero-length packet. A frame lost this way is
# still counted as submitted with failed=0 - only a receiver shows it.
#
# Every frame of a cell has the same PSDU length (DEVOURER_TX_PAYLOAD_BYTES).
# LENS lists the boundary lengths and at least one control beside them:
# descriptor + PSDU must be a multiple of 512 (USB2) / 1024 (USB3) for a
# boundary length. Descriptor sizes: 48 on Jaguar2/3 and the RTL8733B, 40 on
# Jaguar1, 48 on the 8852B and 56 on the 8852C.
#
#   sudo DUT_PID=0xc812 tests/tx_bulk_boundary_onair.sh
#   sudo DUT_VID=0x35bc DUT_PID=0x0101 LENS="968 456 976" \
#        tests/tx_bulk_boundary_onair.sh
#
# A/B against another build: TX_BUILD=/path/to/other/build (the receiver
# always runs this tree's build). Exit 1 when a length delivers under MIN_PCT
# of the best length of the same run.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
RXDEMO="$ROOT/build/rxdemo"
TX_BUILD=${TX_BUILD:-$ROOT/build}
TXDEMO="$TX_BUILD/txdemo"

DUT_VID=${DUT_VID:-0x0bda} DUT_PID=${DUT_PID:?DUT_PID=0xNNNN}
GND_VID=${GND_VID:-0x2357} GND_PID=${GND_PID:-0x012d}
CH=${CH:-36}
FRAMES=${FRAMES:-1000}
LENS=${LENS:-"976 984"}
RATE=${RATE:-}
MIN_PCT=${MIN_PCT:-50}
OUT=${OUT:-/tmp/devourer-tx-bulk-boundary}

mkdir -p "$OUT"
RXPID=""
cleanup() {
  [ -n "$RXPID" ] && kill -TERM "$RXPID" 2>/dev/null
  pkill -x txdemo 2>/dev/null
  pkill -x rxdemo 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

cell() { # $1=len -> "hits submitted failed"
  local len="$1"
  local rxlog="$OUT/rx_$len.jsonl" txlog="$OUT/tx_$len.jsonl"
  env DEVOURER_VID="$GND_VID" DEVOURER_PID="$GND_PID" \
    DEVOURER_CHANNEL="$CH" DEVOURER_LOG_LEVEL=warn \
    "$RXDEMO" > "$rxlog" 2>"$OUT/rx_$len.err" &
  RXPID=$!
  sleep 7
  if ! kill -0 "$RXPID" 2>/dev/null; then
    echo "ground RX died (len $len) - see $OUT/rx_$len.err" >&2
    RXPID=""; echo "-1 0 0"; return
  fi
  env DEVOURER_VID="$DUT_VID" DEVOURER_PID="$DUT_PID" DEVOURER_CHANNEL="$CH" \
    DEVOURER_TX_PAYLOAD_BYTES="$len" DEVOURER_TX_FRAMES="$FRAMES" \
    DEVOURER_TX_GAP_US=2000 DEVOURER_LOG_LEVEL=warn \
    ${RATE:+DEVOURER_TX_RATE="$RATE"} \
    timeout --signal=TERM 60 "$TXDEMO" > "$txlog" 2>"$OUT/tx_$len.err"
  sleep 1
  kill -TERM "$RXPID" 2>/dev/null
  wait "$RXPID" 2>/dev/null
  RXPID=""
  local hits sub fail
  hits=$(grep -F '"ev":"rx.txhit"' "$rxlog" | grep -F '"final":1' | tail -1 |
         grep -o '"hits":[0-9]*' | cut -d: -f2)
  sub=$(grep -F '"ev":"tx.stats"' "$txlog" | tail -1 |
        grep -o '"submitted":[0-9]*' | cut -d: -f2)
  fail=$(grep -F '"ev":"tx.stats"' "$txlog" | tail -1 |
         grep -o '"failed":[0-9]*' | cut -d: -f2)
  echo "${hits:-0} ${sub:-0} ${fail:-0}"
}

echo "DUT $DUT_VID:$DUT_PID  tx build $TX_BUILD  ch $CH  $FRAMES frames/cell"
printf '%-6s %-6s %-10s %-7s\n' len hits submitted failed
best=0
declare -A got
for len in $LENS; do
  read -r h s f <<<"$(cell "$len")"
  printf '%-6s %-6s %-10s %-7s\n' "$len" "$h" "$s" "$f"
  got[$len]=$h
  [ "$h" -gt "$best" ] && best=$h
done

# No length delivering is a dead link or a dead DUT, not a boundary finding.
if [ "$best" -lt $((FRAMES / 4)) ]; then
  echo "NO DELIVERY at any length (best $best of $FRAMES) - check the pair" >&2
  exit 2
fi
rc=0
for len in $LENS; do
  if [ $((got[$len] * 100)) -lt $((best * MIN_PCT)) ]; then
    echo "FAIL: len $len delivered ${got[$len]} against a best of $best"
    rc=1
  fi
done
[ $rc -eq 0 ] && echo "PASS: every length within ${MIN_PCT}% of the best"
exit $rc
