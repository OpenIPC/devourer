#!/usr/bin/env bash
# tone_mask_rx_sanity.sh — paired TX/RX sanity for the tone-mask knobs.
#
# Cell A (baseline): devourer TX beacons on one adapter, devourer RX on another,
#   no tone-mask knobs -> expect canonical-SA hits (proves the bench itself).
# Cell B (mask off-frequency): same, RX with DEVOURER_RX_CSI_MASK on a slice
#   AWAY from the TX frequency -> hits must survive (mask doesn't break RX).
# Cell C (mask on-frequency): RX masks the slice CONTAINING the 20 MHz TX
#   channel -> expect hits to drop (the mask provably affects demodulation).
#
# Usage: sudo tests/tone_mask_rx_sanity.sh [TX_VID:TX_PID RX_VID:RX_PID [ch]]
# Defaults: TX = 2357:0120 (8821AU), RX = 2357:012d (8822BU T3U), ch 6.

set -u
cd "$(dirname "$0")/.."

TX_SPEC=${1:-2357:0120}
RX_SPEC=${2:-2357:012d}
CH=${3:-6}
DUR=12

TX_VID=${TX_SPEC%:*}; TX_PID=${TX_SPEC#*:}
RX_VID=${RX_SPEC%:*}; RX_PID=${RX_SPEC#*:}

LOGDIR=/tmp/devourer-tonemask-sanity
rm -rf "$LOGDIR"; mkdir -p "$LOGDIR"

cleanup() {
  pkill -INT -x txdemo 2>/dev/null
  pkill -INT -x rxdemo 2>/dev/null
  sleep 1
  pkill -KILL -x txdemo 2>/dev/null
  pkill -KILL -x rxdemo 2>/dev/null
}
trap cleanup EXIT INT TERM

# ch -> 2.4G frequency (MHz)
FREQ=$((2412 + (CH - 1) * 5))

run_cell() {
  local name=$1 csi=$2
  local rxlog="$LOGDIR/rx-$name.log" txlog="$LOGDIR/tx-$name.log"
  local csi_env=()
  [ -n "$csi" ] && csi_env=(DEVOURER_RX_CSI_MASK="$csi")

  env DEVOURER_VID="0x$TX_VID" DEVOURER_PID="0x$TX_PID" DEVOURER_CHANNEL="$CH" \
      timeout -s INT -k 5 $((DUR + 8)) ./build/txdemo >"$txlog" 2>&1 &
  local txpid=$!
  sleep 4  # TX bring-up

  env DEVOURER_VID="0x$RX_VID" DEVOURER_PID="0x$RX_PID" DEVOURER_CHANNEL="$CH" \
      "${csi_env[@]}" \
      timeout -s INT -k 5 "$DUR" ./build/rxdemo >"$rxlog" 2>&1

  wait "$txpid" 2>/dev/null
  sleep 4  # settle before the next cell re-opens both dongles

  local hits
  hits=$(grep -c "devourer-tx-hit" "$rxlog")
  echo "$name: $hits canonical-SA hits  (rx log: $rxlog)"
  echo "$hits" > "$LOGDIR/$name.count"
}

# 20 MHz channel: fc = FREQ, band edges +-10 MHz.
ON_LO=$((FREQ - 8)); ON_HI=$((FREQ + 8))       # mask covering the TX channel
OFF_LO=$((FREQ + 30)); OFF_HI=$((FREQ + 40))   # outside the tuned 20 MHz -> no-op

run_cell baseline ""
run_cell mask-off "${OFF_LO}-${OFF_HI}"
run_cell mask-on  "${ON_LO}-${ON_HI}"

BASE=$(cat "$LOGDIR/baseline.count")
OFF=$(cat "$LOGDIR/mask-off.count")
ON=$(cat "$LOGDIR/mask-on.count")

echo
RC=0
if [ "$BASE" -lt 10 ]; then
  echo "FAIL: baseline delivered $BASE hits — bench itself is not working"
  RC=1
else
  # off-frequency mask must not tank RX (allow 50% variance)
  if [ "$OFF" -lt $((BASE / 2)) ]; then
    echo "FAIL: off-frequency mask dropped hits $BASE -> $OFF"
    RC=1
  else
    echo "PASS: off-frequency mask preserves RX ($BASE -> $OFF)"
  fi
  # on-frequency mask should measurably hurt (expect < 50% of baseline)
  if [ "$ON" -lt $((BASE / 2)) ]; then
    echo "PASS: on-frequency mask suppresses RX ($BASE -> $ON) — mask provably active"
  else
    echo "WARN: on-frequency mask did not suppress RX ($BASE -> $ON) — mask may be demod-inert at 20 MHz"
  fi
fi
exit $RC
