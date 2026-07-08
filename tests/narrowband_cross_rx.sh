#!/usr/bin/env bash
# narrowband_cross_rx.sh — does 5/10 MHz narrowband actually DEMODULATE across
# chip generations, not just narrow the TX lobe? Both ends re-clock to the same
# narrowband width; the RX end's frame count is the witness that the ADC
# re-clock receives a real NB signal (an SDR only proves the TX side).
#
# Cells: a 20 MHz control brackets the bench, then 10 and 5 MHz with both ends
# re-clocked. Defaults pair Jaguar3 TX (8812CU, whose narrowband is already
# SDR-validated) with Jaguar2 8821C RX (8811CU) — run the reverse direction
# too. The 8822BU (2357:012d) can be substituted but its NB is
# EXPERIMENTAL/gated (partial RX, no NB TX):
#
#   sudo tests/narrowband_cross_rx.sh                       # J3 c812 -> 8811CU
#   sudo tests/narrowband_cross_rx.sh 0bda:c811 0bda:c812   # 8811CU  -> J3 c812
#
# The 8821C TX at UNII needs a flat TXAGC (worldwide-min limit table clamps
# it to 0): TX_PWR=0x2d is applied automatically for a c811 TX.
#
# Usage: sudo tests/narrowband_cross_rx.sh [TX_VID:PID] [RX_VID:PID] [DUR] [CH]
set -u
cd "$(dirname "$0")/.."

TX_SPEC=${1:-0bda:c812}
RX_SPEC=${2:-0bda:c811}
DUR=${3:-15}
CH=${4:-44}

TX_VID=${TX_SPEC%:*}; TX_PID=${TX_SPEC#*:}
RX_VID=${RX_SPEC%:*}; RX_PID=${RX_SPEC#*:}

LOGDIR=/tmp/devourer-narrowband-cross-rx
rm -rf "$LOGDIR"; mkdir -p "$LOGDIR"

cleanup() {
  pkill -INT -x txdemo 2>/dev/null
  pkill -INT -x rxdemo 2>/dev/null
  sleep 1
  pkill -KILL -x txdemo 2>/dev/null
  pkill -KILL -x rxdemo 2>/dev/null
}
trap cleanup EXIT INT TERM

# Keep the in-tree rtw88 auto-probe off both DUTs (best effort; harness rigs
# usually blacklist these already).
for m in rtw88_8822bu rtw88_8821cu rtw88_8822cu rtw88_8822eu; do
  sudo modprobe -r "$m" 2>/dev/null
done

# run_cell <name> <nb_bw: "" for 20 MHz | 5 | 10>
run_cell() {
  local name=$1 nb=$2
  local rxlog="$LOGDIR/rx-$name.log" txlog="$LOGDIR/tx-$name.log"
  local nb_env=()
  [ -n "$nb" ] && nb_env=(DEVOURER_NB_BW="$nb")

  local tx_pwr=()
  [ "$TX_PID" = "c811" ] && tx_pwr=(DEVOURER_TX_PWR=0x2d)
  env DEVOURER_VID="0x$TX_VID" DEVOURER_PID="0x$TX_PID" DEVOURER_CHANNEL=$CH \
      "${nb_env[@]}" "${tx_pwr[@]}" \
      timeout -s INT -k 5 $((DUR * 3 + 20)) ./build/txdemo >"$txlog" 2>&1 &  # covers RX bring-up retries
  local txpid=$!
  sleep 7   # TX bring-up (DLFW + re-clock) before the RX window opens

  # 8821C DLFW is flaky — retry the RX bring-up until the RX loop starts.
  local try
  for try in 1 2 3; do
    env DEVOURER_VID="0x$RX_VID" DEVOURER_PID="0x$RX_PID" DEVOURER_CHANNEL=$CH \
        "${nb_env[@]}" \
        timeout -s INT -k 5 "$DUR" ./build/rxdemo >"$rxlog" 2>&1
    grep -q "entering RX loop" "$rxlog" && break
    echo "  ($name: RX bring-up failed, retry $try)"
  done
  wait "$txpid" 2>/dev/null
  sleep 3

  local hits
  hits=$(grep -cF '"ev":"rx.txhit"' "$rxlog")
  echo "$hits" >"$LOGDIR/$name.count"
  printf "  %-6s (%s MHz): %s hits\n" "$name" "${nb:-20}" "$hits"
}

echo "narrowband cross-RX: TX $TX_SPEC -> RX $RX_SPEC, ch$CH, ${DUR}s cells"
run_cell bw20 ""
run_cell bw10 10
run_cell bw5  5

C20=$(cat "$LOGDIR/bw20.count")
C10=$(cat "$LOGDIR/bw10.count")
C5=$(cat "$LOGDIR/bw5.count")
echo
if [ "$C20" -lt 20 ]; then
  echo "VERDICT: bench broken (20 MHz control $C20 hits) — fix before judging NB"
  exit 1
fi
# NB thresholds are deliberately loose: the ratio to the control absorbs
# bench-to-bench delivery variance; near-zero is the failure signature.
if [ "$C10" -ge $((C20 / 10)) ] && [ "$C10" -ge 10 ]; then V10=OK; else V10=FAIL; fi
if [ "$C5" -ge $((C20 / 10)) ] && [ "$C5" -ge 10 ]; then V5=OK; else V5=FAIL; fi
echo "VERDICT: 20M=$C20 (control), 10M=$C10 [$V10], 5M=$C5 [$V5]"
echo "logs: $LOGDIR"
[ "$V10" = OK ] && [ "$V5" = OK ]
