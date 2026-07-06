#!/usr/bin/env bash
# tone_mask_regcheck.sh — register-landing check for the DEVOURER_RX_CSI_MASK /
# DEVOURER_RX_NBI knobs (src/ToneMask.h) on every plugged Jaguar generation.
#
# Per DUT: run rxdemo briefly with a CSI-mask slice + an NBI notch
# inside the tuned channel, then assert from the demo log that
#   - the CSI mask enumerated >0 tones and the enable bit reads back set
#     (0x874[0] on 11ac, 0xC0C[3] on Jaguar-3), and
#   - the NBI notch armed (0x87C[13] on 11ac, 0x818[11] on Jaguar-3).
#
# This validates that the ported phydm register recipes land on real silicon;
# whether the mask *helps* under a jammed slice is the separate on-air
# experiment (tests/pseudo_puncture_onair.sh).
#
# Usage: sudo tests/tone_mask_regcheck.sh   (after cmake --build build)

set -u
cd "$(dirname "$0")/.."

DEMO=./build/rxdemo
[ -x "$DEMO" ] || { echo "FATAL: $DEMO not built"; exit 2; }

LOGDIR=/tmp/devourer-tonemask-regcheck
rm -rf "$LOGDIR"; mkdir -p "$LOGDIR"

cleanup() {
  # Exact-comm kill: only our demo binary, nothing else.
  pkill -INT -x rxdemo 2>/dev/null
  sleep 1
  pkill -KILL -x rxdemo 2>/dev/null
}
trap cleanup EXIT INT TERM

PASS=0; FAIL=0; SKIP=0

# run_cell <name> <vid> <pid> <family> <channel> <bw> <csi_spec> <nbi_mhz>
#   family: 11ac | jgr3  (selects which readback registers to assert)
run_cell() {
  local name=$1 vid=$2 pid=$3 fam=$4 ch=$5 bw=$6 csi=$7 nbi=$8
  local log="$LOGDIR/${name}.log"

  if ! lsusb -d "${vid}:${pid#0x}" >/dev/null 2>&1; then
    echo "SKIP  $name (${vid}:${pid#0x} not plugged)"
    SKIP=$((SKIP+1)); return
  fi

  local bw_env=()
  [ "$bw" != 20 ] && bw_env=(DEVOURER_BW="$bw")

  env DEVOURER_VID="0x$vid" DEVOURER_PID="$pid" \
      DEVOURER_CHANNEL="$ch" "${bw_env[@]}" \
      DEVOURER_RX_CSI_MASK="$csi" DEVOURER_RX_NBI="$nbi" \
      timeout -s INT -k 5 15 "$DEMO" >"$log" 2>&1
  sleep 5  # settle before the next cell — a back-to-back re-open of the same
           # dongle can fail DLFW when the firmware is still winding down

  local ok=1
  local csi_line nbi_line
  csi_line=$(grep -o "DEVOURER_RX_CSI_MASK: [0-9]* tones masked.*" "$log" | head -1)
  nbi_line=$(grep -o "DEVOURER_RX_NBI: notch at .*" "$log" | head -1)

  if [ -z "$csi_line" ]; then
    echo "FAIL  $name: no CSI-mask apply line"; ok=0
  else
    local ntones; ntones=$(echo "$csi_line" | grep -o "[0-9]* tones" | cut -d' ' -f1)
    [ "${ntones:-0}" -gt 0 ] || { echo "FAIL  $name: 0 tones masked"; ok=0; }
    if [ "$fam" = 11ac ]; then
      # 0x874 readback: bit0 must be set
      local r874; r874=$(echo "$csi_line" | grep -o "0x874=0x[0-9a-f]*" | cut -d= -f2)
      [ $(( ${r874:-0} & 1 )) -eq 1 ] || { echo "FAIL  $name: 0x874[0] not set ($r874)"; ok=0; }
      # at least one mask array dword nonzero
      echo "$csi_line" | sed 's/.*mask\[0x880\.\.0x89c\]=//' \
        | grep -qE "[1-9a-f]" \
        || { echo "FAIL  $name: mask arrays read back all-zero"; ok=0; }
    else
      local rc0c; rc0c=$(echo "$csi_line" | grep -o "0xc0c=0x[0-9a-f]*" | cut -d= -f2)
      [ $(( ${rc0c:-0} & 8 )) -eq 8 ] || { echo "FAIL  $name: 0xc0c[3] not set ($rc0c)"; ok=0; }
    fi
  fi

  if [ -z "$nbi_line" ]; then
    echo "FAIL  $name: no NBI apply line"; ok=0
  else
    if [ "$fam" = 11ac ]; then
      local r87c; r87c=$(echo "$nbi_line" | grep -o "0x87c=0x[0-9a-f]*" | cut -d= -f2)
      [ $(( ${r87c:-0} & 0x2000 )) -ne 0 ] || { echo "FAIL  $name: 0x87c[13] not set ($r87c)"; ok=0; }
    else
      local r818; r818=$(echo "$nbi_line" | grep -o "0x818=0x[0-9a-f]*" | cut -d= -f2)
      [ $(( ${r818:-0} & 0x800 )) -ne 0 ] || { echo "FAIL  $name: 0x818[11] not set ($r818)"; ok=0; }
    fi
  fi

  if [ "$ok" = 1 ]; then
    echo "PASS  $name: $csi_line"
    PASS=$((PASS+1))
  else
    echo "      log: $log"
    FAIL=$((FAIL+1))
  fi
}

# 2.4 GHz ch6 BW20 (fc 2437): mask 2440-2445 MHz, notch 2442 MHz.
run_cell 8821au-ac1     2357 0x0120 11ac 6 20 2440-2445 2442
run_cell 8814au-ac2     0bda 0x8813 11ac 6 20 2440-2445 2442
run_cell 8822bu-jaguar2 2357 0x012d 11ac 6 20 2440-2445 2442
run_cell 8812cu-jaguar3 0bda 0xc812 jgr3 6 20 2440-2445 2442
run_cell 8822eu-jaguar3 0bda 0xa81a jgr3 6 20 2440-2445 2442
# BW80 cell on the Jaguar2 (validated 20/40/80): ch36-48 block (fc 5210),
# mask the top 20 MHz slice, notch mid-slice.
run_cell 8822bu-bw80    2357 0x012d 11ac 36 80 5230-5250 5240

echo
echo "tone_mask_regcheck: $PASS pass, $FAIL fail, $SKIP skip (logs: $LOGDIR)"
[ "$FAIL" -eq 0 ]
