#!/usr/bin/env bash
# pseudo_puncture_onair.sh — the jammed-slice experiment behind
# docs/pseudo-preamble-puncturing.md.
#
# Question: with one 20 MHz slice of an 80 MHz channel jammed, how much of the
# Wi-Fi 7 "transmit around the dirty slice" win can 802.11ac silicon recover?
#
# Rig: two devourer DUTs on the ch36 80 MHz block (5170-5250, fc 5210) + a
# USRP B210 jamming the TOP 20 MHz slice (5230-5250, center 5240) with
# band-limited AWGN (tests/sdr_interferer.py — reproducible via --tx-gain).
#
# Cells (RX counts canonical-SA hits over a fixed window):
#   A clean-80   VHT80 TX, 80 MHz RX, no jam            — bench baseline
#   B jam-80     same + jammer                          — the damage
#   C jam-80-csi same + DEVOURER_RX_CSI_MASK=5230-5250  — RX-side "puncturing"
#   D jam-40     TX+RX drop to ch36 BW40 (clean half)   — TX-side avoidance
#
# Verdict logic: C > B means the receive-equalizer mask recovers real goodput
# under a jammed slice; D ≈ A (at half the PHY rate) is the avoidance bound.
#
# Usage:
#   sudo tests/pseudo_puncture_onair.sh [TX_VID:PID] [RX_VID:PID] [JAM_GAIN] [DUR]
# Defaults: TX 2357:012d (8822BU T3U — BW80 TX validated), RX 0bda:8813
# (8814AU), gain 60 dB, 15 s per cell. Sweep JAM_GAIN until cell B drops
# hard but not to zero (a fully-saturated front-end can't be masked back).

set -u
cd "$(dirname "$0")/.."

TX_SPEC=${1:-2357:012d}
RX_SPEC=${2:-0bda:8813}
JAM_GAIN=${3:-60}
DUR=${4:-15}

TX_VID=${TX_SPEC%:*}; TX_PID=${TX_SPEC#*:}
RX_VID=${RX_SPEC%:*}; RX_PID=${RX_SPEC#*:}

JAM_FREQ=5240e6   # center of the 5230-5250 slice (ch48 quarter of the block)
JAM_RATE=20e6
MASK_SPEC=5230-5250

LOGDIR=/tmp/devourer-pseudo-puncture
rm -rf "$LOGDIR"; mkdir -p "$LOGDIR"

JAM_PID=""
cleanup() {
  [ -n "$JAM_PID" ] && kill -INT "$JAM_PID" 2>/dev/null
  pkill -INT -x txdemo 2>/dev/null
  pkill -INT -x rxdemo 2>/dev/null
  pkill -INT -f sdr_interferer 2>/dev/null
  sleep 1
  pkill -KILL -x txdemo 2>/dev/null
  pkill -KILL -x rxdemo 2>/dev/null
  pkill -KILL -f sdr_interferer 2>/dev/null
}
trap cleanup EXIT INT TERM

jam_start() {
  python3 tests/sdr_interferer.py --freq "$JAM_FREQ" --rate "$JAM_RATE" \
    --tx-gain "$JAM_GAIN" >"$LOGDIR/jammer.log" 2>&1 &
  JAM_PID=$!
  sleep 3
  kill -0 "$JAM_PID" 2>/dev/null || {
    echo "FATAL: jammer died — $LOGDIR/jammer.log"; exit 2; }
}

jam_stop() {
  [ -n "$JAM_PID" ] && { kill -INT "$JAM_PID" 2>/dev/null; wait "$JAM_PID" 2>/dev/null; }
  JAM_PID=""
  sleep 1
}

# run_cell <name> <bw> <tx_rate> <rx_mask ('' = none)>
run_cell() {
  local name=$1 bw=$2 rate=$3 mask=$4
  local rxlog="$LOGDIR/rx-$name.log" txlog="$LOGDIR/tx-$name.log"
  local mask_env=()
  [ -n "$mask" ] && mask_env=(DEVOURER_RX_CSI_MASK="$mask")

  env DEVOURER_VID="0x$TX_VID" DEVOURER_PID="0x$TX_PID" DEVOURER_CHANNEL=36 \
      DEVOURER_HOP_BW="$bw" DEVOURER_TX_RATE="$rate" \
      timeout -s INT -k 5 $((DUR + 10)) ./build/txdemo >"$txlog" 2>&1 &
  local txpid=$!
  sleep 5  # TX bring-up (DLFW + cal)

  env DEVOURER_VID="0x$RX_VID" DEVOURER_PID="0x$RX_PID" DEVOURER_CHANNEL=36 \
      DEVOURER_BW="$bw" "${mask_env[@]}" \
      timeout -s INT -k 5 "$DUR" ./build/rxdemo >"$rxlog" 2>&1

  wait "$txpid" 2>/dev/null
  sleep 5  # warm-FW settle before the same dongles re-open

  local hits
  hits=$(grep -c "devourer-tx-hit" "$rxlog")
  echo "$hits" >"$LOGDIR/$name.count"
  echo "cell $name: $hits hits  (bw=$bw rate=$rate mask='${mask:-none}')"
}

echo "pseudo-puncture on-air: TX $TX_SPEC -> RX $RX_SPEC, jam ${JAM_FREQ} @ ${JAM_GAIN} dB, ${DUR}s cells"
echo

run_cell clean-80 80 VHT1SS_MCS0/80 ""

jam_start
run_cell jam-80     80 VHT1SS_MCS0/80 ""
run_cell jam-80-csi 80 VHT1SS_MCS0/80 "$MASK_SPEC"
run_cell jam-40     40 VHT1SS_MCS0/40 ""
jam_stop

A=$(cat "$LOGDIR/clean-80.count")
B=$(cat "$LOGDIR/jam-80.count")
C=$(cat "$LOGDIR/jam-80-csi.count")
D=$(cat "$LOGDIR/jam-40.count")

echo
echo "== results (canonical-SA hits / ${DUR}s) =="
printf "  %-28s %s\n" "A clean VHT80:" "$A"
printf "  %-28s %s\n" "B jammed VHT80:" "$B"
printf "  %-28s %s\n" "C jammed VHT80 + CSI mask:" "$C"
printf "  %-28s %s\n" "D jammed, 40 MHz avoidance:" "$D"
echo
if [ "$A" -lt 20 ]; then
  echo "VERDICT: bench broken (clean baseline $A hits) — fix the rig first"
  exit 1
fi
if [ "$B" -ge $((A * 7 / 10)) ]; then
  echo "VERDICT: jammer too weak (B=$B vs A=$A) — raise JAM_GAIN and re-run"
  exit 1
fi
echo "damage:            B/A = $((B * 100 / A))%"
echo "CSI-mask recovery: C/A = $((C * 100 / A))%  (uplift vs B: $((C - B)) hits)"
echo "40MHz avoidance:   D/A = $((D * 100 / A))%"
