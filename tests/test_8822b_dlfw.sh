#!/usr/bin/env bash
# Test RTL8812BU/8822BU (Jaguar2) firmware DLFW boot on the TP-Link Archer T3U.
# Power-cycles the chip via the powered hub (cold state — the chip retains
# firmware/register state across a soft re-init), then runs WiFiDriverDemo far
# enough to exercise power-on -> chip-version -> init_trx_cfg -> DLFW. Success =
# the "firmware booted" log line (DLFW bcn-valid latched, 0x80 == 0xC078).
set -euo pipefail

VID=0x2357
PID=0x012d
DEMO=./build/WiFiDriverDemo
RECOVER=/home/josephnef/git/lkl-wifi-poc/scripts/recover-chip.sh

cleanup() {
  pkill -x WiFiDriverDemo 2>/dev/null || true
}
trap cleanup EXIT

echo "=== power-cycle T3U (cold state) ==="
RECOVER_HUB_LOC=4-2.3 RECOVER_HUB_PORT=3 bash "$RECOVER" 2>&1 | tail -5 || {
  echo "recover script failed; falling back to whatever state the chip is in"
}
sleep 3

echo "=== lsusb ==="
lsusb | grep -iE '2357:012d|2357:0141' || echo "T3U not enumerated!"

echo "=== run WiFiDriverDemo (DLFW exercise) ==="
# The demo throws after DLFW at M4; we only need the DLFW log lines.
timeout 25 sudo DEVOURER_VID=$VID DEVOURER_PID=$PID DEVOURER_CHANNEL=6 \
  "$DEMO" 2>&1 | grep -iE 'chip|version|DLFW|firmware|boot|C078|bcn|fail|error|trx|power' \
  | head -40 || true
