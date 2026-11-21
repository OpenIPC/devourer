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
# boundary length. Descriptor sizes: 48 on Jaguar2/3, 40 on Jaguar1 and the
# RTL8733B, 48 on the 8852B and 56 on the 8852C. The default LENS is the
# Jaguar2/3 pair; every other family needs its own, as below.
#
#   sudo DUT_PID=0xc812 tests/tx_bulk_boundary_onair.sh
#   sudo DUT_PID=0xb733 LENS="984 472 976" tests/tx_bulk_boundary_onair.sh
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
# Back the DUT's TX power off (quarter-dB, negative). The bench 8812EU module
# (the 8822E die) drops off the bus about a second into TX at its default
# power and holds at -44, so that is the only power it was measured at here.
TX_PWR_OFFSET_QDB=${TX_PWR_OFFSET_QDB:-}
MIN_PCT=${MIN_PCT:-50}
OUT=${OUT:-/tmp/devourer-tx-bulk-boundary}

mkdir -p "$OUT"
# Only this run's own processes are ever signalled: another session's demos
# on other adapters are left alone. cell() runs in this shell, not a
# subshell, so the trap sees the PIDs.
RXPID="" TXPID=""
cleanup() {
  [ -n "$TXPID" ] && kill -TERM "$TXPID" 2>/dev/null
  [ -n "$RXPID" ] && kill -TERM "$RXPID" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

cell() { # $1=len -> CELL_HITS CELL_SUB CELL_FAIL
  CELL_HITS=-1 CELL_SUB=0 CELL_FAIL=0
  local len="$1"
  local rxlog="$OUT/rx_$len.jsonl" txlog="$OUT/tx_$len.jsonl"
  env DEVOURER_VID="$GND_VID" DEVOURER_PID="$GND_PID" \
    DEVOURER_CHANNEL="$CH" DEVOURER_LOG_LEVEL=warn \
    "$RXDEMO" > "$rxlog" 2>"$OUT/rx_$len.err" &
  RXPID=$!
  sleep 7
  if ! kill -0 "$RXPID" 2>/dev/null; then
    echo "ground RX died (len $len) - see $OUT/rx_$len.err" >&2
    RXPID=""; return
  fi
  env DEVOURER_VID="$DUT_VID" DEVOURER_PID="$DUT_PID" DEVOURER_CHANNEL="$CH" \
    DEVOURER_TX_PAYLOAD_BYTES="$len" DEVOURER_TX_FRAMES="$FRAMES" \
    DEVOURER_TX_GAP_US=2000 DEVOURER_LOG_LEVEL=warn \
    ${RATE:+DEVOURER_TX_RATE="$RATE"} \
    ${TX_PWR_OFFSET_QDB:+DEVOURER_TX_PWR_OFFSET_QDB="$TX_PWR_OFFSET_QDB"} \
    timeout --signal=TERM 60 "$TXDEMO" > "$txlog" 2>"$OUT/tx_$len.err" &
  TXPID=$!
  wait "$TXPID" 2>/dev/null
  TXPID=""
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
  CELL_HITS=${hits:-0} CELL_SUB=${sub:-0} CELL_FAIL=${fail:-0}
}

echo "DUT $DUT_VID:$DUT_PID  tx build $TX_BUILD  ch $CH  $FRAMES frames/cell"
printf '%-6s %-6s %-10s %-7s\n' len hits submitted failed
best=0
declare -A got
for len in $LENS; do
  cell "$len"
  printf '%-6s %-6s %-10s %-7s\n' "$len" "$CELL_HITS" "$CELL_SUB" "$CELL_FAIL"
  got[$len]=$CELL_HITS
  [ "$CELL_HITS" -gt "$best" ] && best=$CELL_HITS
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
