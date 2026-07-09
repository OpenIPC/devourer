#!/usr/bin/env bash
# timesync uplink timing-advance bench (LTE TA): a full-duplex MASTER broadcasts
# beacons + phase-measures each UE uplink against its TSF slot grid, feeding back
# a timing advance; a full-duplex UE transmits one uplink per beacon, TA-
# corrected. The loop should drive uplink arrivals onto the slot boundary.
#
# BOTH nodes are full-duplex (InitWrite + StartRxLoop), so BOTH must be clean
# Jaguar2/3 TX+RX adapters. The 8822E desenses its RX in TX+RX mode, so a run
# with an 8822E node is best-effort (that node may hear little) — use two 8822B/
# 8822C for a clean result.
#
#   MASTER = 8822CU (0xc812, clean full-duplex)   UE = 8822EU (0xa81a, best-effort)
#
#   sudo tests/timesync_ta_demo.sh [secs] [channel] [slot_ms]
set -uo pipefail
cd "$(dirname "$0")/.."
SECS=${1:-30}; CH=${2:-36}; SLOT=${3:-20}
# HWBEACON=1 : the UE airs its uplink from the beacon engine and fine-steers the
# TBTT (AdjustBeaconTimingFine) — the CONVERGING closed loop. Needs a J3 UE (fine
# steering) + any full-duplex master (Jaguar1 8812AU works; it needs no steering).
HWBEACON=${HWBEACON:-0}
if [ "$HWBEACON" = 1 ]; then
  M_VID=${M_VID:-0x0bda}; M_PID=${M_PID:-0x8812}   # 8812AU full-duplex master
  U_VID=${U_VID:-0x0bda}; U_PID=${U_PID:-0xc812}   # 8822CU J3 UE (fine steer)
  INT=${INT:-100}                                   # slower UE beacon = fewer steers = tighter
  UE_EXTRA="DEVOURER_TSYNC_HWBEACON=1"
else
  M_VID=${M_VID:-0x0bda}; M_PID=${M_PID:-0xc812}
  U_VID=${U_VID:-0x0bda}; U_PID=${U_PID:-0xa81a}
  INT=${INT:-$SLOT}                                 # beacon interval = slot
  UE_EXTRA=""
fi

cleanup() { sudo pkill -9 -x timesync 2>/dev/null; }
trap cleanup EXIT
cleanup; sleep 2

LOGM=$(mktemp); LOGU=$(mktemp)
echo "master=$M_VID:$M_PID  ue=$U_VID:$U_PID  ch$CH  slot=${SLOT}ms  ${SECS}s (full-duplex)"

# Master: full-duplex, TA loop.
sudo env DEVOURER_VID=$M_VID DEVOURER_PID=$M_PID DEVOURER_CHANNEL=$CH \
    DEVOURER_TX_WITH_RX=thread DEVOURER_TSYNC_ROLE=master DEVOURER_TSYNC_UPLINK=1 \
    DEVOURER_TSYNC_INTERVAL_MS=$INT DEVOURER_TSYNC_SLOT_MS=$SLOT \
    DEVOURER_TSYNC_SECS=$((SECS + 3)) DEVOURER_LOG_LEVEL=info DEVOURER_EVENTS=stdout \
    ./build/timesync >"$LOGM" 2>>"$LOGM" &
sleep 4   # master up + beaconing before the UE starts locking

# UE: full-duplex. HWBEACON=1 => hardware-beacon uplink fine-steered (converges).
sudo env DEVOURER_VID=$U_VID DEVOURER_PID=$U_PID DEVOURER_CHANNEL=$CH $UE_EXTRA \
    DEVOURER_TX_WITH_RX=thread DEVOURER_TSYNC_ROLE=ue \
    DEVOURER_TSYNC_INTERVAL_MS=$INT DEVOURER_TSYNC_SLOT_MS=$SLOT \
    DEVOURER_TSYNC_SECS=$SECS DEVOURER_LOG_LEVEL=info \
    ./build/timesync >"$LOGU" 2>>"$LOGU" &

wait 2>/dev/null
sleep 1

echo "=== master(TA) summary ==="; grep -A5 "master(TA) summary" "$LOGM" || echo "(none)"
echo "=== ue summary ==="; grep -A4 "ue summary" "$LOGU" || echo "(none)"
echo "=== timing-advance convergence ==="
python3 tests/timesync_ta_analyze.py "$LOGM"
rm -f "$LOGM" "$LOGU"
