#!/usr/bin/env bash
# rx80_narrow_tx_probe.sh — can a receiver tuned at 80 MHz decode narrower
# frames sent on its primary sub-channels, WITHOUT retuning?
#
# Decides whether TX-side bandwidth adaptation (the "puncture by primary
# selection" lever from docs/pseudo-preamble-puncturing.md) is UNILATERAL —
# a VTX shrinking 80->40->20 frame-by-frame with no RX coordination — or
# needs a lockstep bandwidth change over the feedback channel first.
#
# Cells (TX bw varies, RX fixed at ch36 BW80; controls bracket the question):
#   80->80   control: known-good wide link
#   40->80   THE QUESTION (primary 40 of the block)
#   20->80   bonus: legacy-20 on the primary
#   40->40   control: proves the 40 MHz TX itself is healthy
#
# Usage: sudo tests/rx80_narrow_tx_probe.sh [TX_VID:PID] [RX_VID:PID] [DUR]
# Defaults: TX 2357:012d (8822BU T3U), RX 0bda:8813 (8814AU), 15 s cells.

set -u
cd "$(dirname "$0")/.."

TX_SPEC=${1:-2357:012d}
RX_SPEC=${2:-0bda:8813}
DUR=${3:-15}

TX_VID=${TX_SPEC%:*}; TX_PID=${TX_SPEC#*:}
RX_VID=${RX_SPEC%:*}; RX_PID=${RX_SPEC#*:}

LOGDIR=/tmp/devourer-rx80-narrow-tx
rm -rf "$LOGDIR"; mkdir -p "$LOGDIR"

cleanup() {
  pkill -INT -x txdemo 2>/dev/null
  pkill -INT -x rxdemo 2>/dev/null
  sleep 1
  pkill -KILL -x txdemo 2>/dev/null
  pkill -KILL -x rxdemo 2>/dev/null
}
trap cleanup EXIT INT TERM

# run_cell <name> <tx_bw 20|40|80> <tx_rate> <rx_bw 20|40|80>
run_cell() {
  local name=$1 tx_bw=$2 rate=$3 rx_bw=$4
  local rxlog="$LOGDIR/rx-$name.log" txlog="$LOGDIR/tx-$name.log"
  local tx_env=() rx_env=()
  [ "$tx_bw" != 20 ] && tx_env=(DEVOURER_HOP_BW="$tx_bw")
  [ "$rx_bw" != 20 ] && rx_env=(DEVOURER_BW="$rx_bw")

  env DEVOURER_VID="0x$TX_VID" DEVOURER_PID="0x$TX_PID" DEVOURER_CHANNEL=36 \
      "${tx_env[@]}" DEVOURER_TX_RATE="$rate" \
      timeout -s INT -k 5 $((DUR + 10)) ./build/txdemo >"$txlog" 2>&1 &
  local txpid=$!
  sleep 5

  env DEVOURER_VID="0x$RX_VID" DEVOURER_PID="0x$RX_PID" DEVOURER_CHANNEL=36 \
      "${rx_env[@]}" \
      timeout -s INT -k 5 "$DUR" ./build/rxdemo >"$rxlog" 2>&1
  wait "$txpid" 2>/dev/null
  sleep 5

  local hits
  hits=$(grep -c "devourer-tx-hit" "$rxlog")
  echo "$hits" >"$LOGDIR/$name.count"
  printf "  %-9s TX@%-2s -> RX@%-2s : %s hits\n" "$name" "$tx_bw" "$rx_bw" "$hits"
}

echo "rx80-narrow-tx probe: TX $TX_SPEC -> RX $RX_SPEC, ch36 block, ${DUR}s cells"
run_cell 80to80 80 VHT1SS_MCS0/80 80
run_cell 40to80 40 VHT1SS_MCS0/40 80
run_cell 20to80 20 6M             80
run_cell 40to40 40 VHT1SS_MCS0/40 40

A=$(cat "$LOGDIR/80to80.count"); B=$(cat "$LOGDIR/40to80.count")
C=$(cat "$LOGDIR/20to80.count"); D=$(cat "$LOGDIR/40to40.count")
echo
if [ "$A" -lt 20 ]; then
  echo "VERDICT: bench broken (80->80 control $A hits)"; exit 1
fi
if [ "$D" -lt 20 ]; then
  echo "VERDICT: 40 MHz TX itself broken (40->40 control $D hits)"; exit 1
fi
if [ "$B" -ge $((D / 2)) ]; then
  echo "VERDICT: UNILATERAL — RX@80 decodes primary-40 frames ($B vs $D retuned)."
  echo "         TX can shrink/grow bandwidth per-packet with no RX coordination."
else
  echo "VERDICT: LOCKSTEP — RX@80 misses primary-40 frames ($B vs $D retuned)."
  echo "         Bandwidth changes must ride the feedback channel."
fi
[ "$C" -ge $((D / 2)) ] \
  && echo "         (legacy-20 on the primary also decodes at RX@80: $C hits)" \
  || echo "         (legacy-20 at RX@80: only $C hits)"
