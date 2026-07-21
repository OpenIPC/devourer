#!/usr/bin/env bash
# fw_table_offload.sh — A/B the Jaguar2/3 init-time firmware register IO-offload
# (DEVOURER_FW_TABLE_OFLD) against the direct-write path: cold-cycle the DUT,
# run init both ways, and report the init-time drop, the offload command count,
# the per-batch read-back verify verdict, and whether the RF LO locks.
#
#   sudo tests/fw_table_offload.sh <VID:PID> <uhubctl-loc> <port> [ofld_flags]
# e.g. sudo tests/fw_table_offload.sh 2357:012d 10 2 1
set -u

VIDPID="${1:?VID:PID}"; HUB="${2:?uhubctl hub loc}"; PORT="${3:?hub port}"
OFLD="${4:-1}"
VID="0x${VIDPID%%:*}"; PID="0x${VIDPID##*:}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
TXDEMO="$REPO/build/txdemo"

cleanup() { sudo pkill -9 -x txdemo 2>/dev/null; }
trap cleanup EXIT INT TERM

cold_cycle() {
  cleanup; sleep 1
  rm -f "/tmp/devourer-usb-${HUB}-${PORT}.lock" 2>/dev/null
  echo ">> VBUS cold-cycle hub $HUB port $PORT"
  uhubctl -l "$HUB" -p "$PORT" -a cycle -d 2 >/dev/null 2>&1
  sleep 4
}

run_init() {  # $1=ofld flag, $2=tag
  local out="/tmp/fwtof.$2"
  env DEVOURER_VID="$VID" DEVOURER_PID="$PID" DEVOURER_FW_TABLE_OFLD="$1" \
      DEVOURER_EVENTS=stdout DEVOURER_LOG_LEVEL=info DEVOURER_CHANNEL=36 \
      DEVOURER_TX_GAP_US=100000 \
      timeout 16 "$TXDEMO" >"$out.out" 2>"$out.err"
  echo "== $2 (FW_TABLE_OFLD=$1) =="
  grep -iE 'PHY tables applied|fw offload|readback|did not replay|LCK|channel set ch' "$out.err" | tail -6
  local ms
  ms=$(grep -F '"stage":"txdemo.init_write"' "$out.out" | grep -oE '"ms":[0-9]+' | head -1 | cut -d: -f2)
  echo "   init_write = ${ms:-N/A} ms"
  echo
}

cold_cycle; run_init 0 direct
cold_cycle; run_init "$OFLD" offload
