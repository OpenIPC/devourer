#!/usr/bin/env bash
# dwell1_parity.sh — register parity per target channel for the N-channel
# dwell-1 hop set, including AGC/sub-band bucket crossings (issue #273's
# "channel/bandwidth/calibration/... change as one logical generation").
#
# A fast hop (FastRetune) must leave the chip in the same channel/BW register
# state as a full SetMonitorChannel to that target, for EVERY channel the
# schedule visits — otherwise a dwell-1 frame on that channel would air with
# stale AGC/CFO/VCO/spur state. tests/hop_parity_check.sh proves this for one
# INIT->TARGET pair; this sweeps it over a target set that crosses the 8822B
# AGC buckets (UNII-1 = bucket 1, UNII-3 = bucket 3; both non-DFS) from a fixed
# UNII-1 init, so the cross-bucket hops (36->149 etc.) are exercised.
#
# Usage (root, 8822B DUT free of the kernel driver):
#   tests/dwell1_parity.sh
#   TX_PID=0x012d TX_VID=0x2357 INIT=36 TARGETS="40 44 48 149 157" \
#       tests/dwell1_parity.sh
set -u
cd "$(dirname "$0")/.."

TX_PID="${TX_PID:-0x012d}"     # 8822B (T3U)
TX_VID="${TX_VID:-0x2357}"
INIT="${INIT:-36}"            # UNII-1, bucket 1
# Same-bucket adjacents (40/44/48) + cross-bucket UNII-3 (149/157/165, bucket 3).
TARGETS="${TARGETS:-40 44 48 149 157 165}"
BW="${BW:-20}"

pass=0; fail=0; fails=""
for t in $TARGETS; do
  echo "========================================================"
  echo "== parity: fast hop $INIT -> $t (BW $BW) vs full set to $t"
  echo "========================================================"
  if TX_PID="$TX_PID" TX_VID="$TX_VID" INIT="$INIT" TARGET="$t" BW="$BW" \
       OFFSET=0 tests/hop_parity_check.sh 2>&1 | tail -6; then
    pass=$((pass + 1))
  else
    fail=$((fail + 1)); fails="$fails $t"
  fi
done

echo "========================================================"
echo "dwell1 register parity: $pass PASS, $fail FAIL${fails:+ (targets:$fails)}"
[ "$fail" -eq 0 ]
