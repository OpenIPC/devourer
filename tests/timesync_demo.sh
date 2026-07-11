#!/usr/bin/env bash
# timesync bench: LTE-eNB-style over-the-air time distribution. One MASTER
# broadcasts its hardware TSF; two SLAVES lock to it from the beacons alone (no
# GPS, no host clock). Then join the slaves' lock streams by beacon seq to get
# the inter-slave (inter-UE) sync error.
#
#   MASTER = 8812AU (0x8812)      SLAVE_A = 8822CU (0xc812)
#                                 SLAVE_B = 8822BU (2357:012d)
# Slaves work on every generation (per-frame tsfl; J1 8821AU bench: 0.30 µs RMS
# lock); the defaults below are the J2/3 pair. Master can be any TX (ReadTsf).
#
#   sudo tests/timesync_demo.sh [secs] [channel] [interval_ms]
set -uo pipefail
cd "$(dirname "$0")/.."
SECS=${1:-30}; CH=${2:-36}; IVL=${3:-100}
MASTER_PID=${MASTER_PID:-0x8812}
A_VID=${A_VID:-0x0bda}; A_PID=${A_PID:-0xc812}
B_VID=${B_VID:-0x2357}; B_PID=${B_PID:-0x012d}

cleanup() { sudo pkill -9 -x timesync 2>/dev/null; }
trap cleanup EXIT
cleanup; sleep 2

LOGA=$(mktemp); LOGB=$(mktemp)
echo "master=$MASTER_PID  slaveA=$A_VID:$A_PID  slaveB=$B_VID:$B_PID  ch$CH  ${IVL}ms beacons  ${SECS}s"

# Master: TX-only sync beacons.
sudo env DEVOURER_PID=$MASTER_PID DEVOURER_CHANNEL=$CH DEVOURER_TSYNC_ROLE=master \
    DEVOURER_TSYNC_INTERVAL_MS=$IVL DEVOURER_TSYNC_SECS=$((SECS + 3)) \
    DEVOURER_LOG_LEVEL=silent DEVOURER_EVENTS=off ./build/timesync >/dev/null 2>&1 &
sleep 3   # let the master come up before the slaves start locking

# Two slaves: RX-only, each locking to the master.
sudo env DEVOURER_VID=$A_VID DEVOURER_PID=$A_PID DEVOURER_CHANNEL=$CH \
    DEVOURER_TSYNC_ROLE=slave DEVOURER_TSYNC_SECS=$SECS DEVOURER_LOG_LEVEL=info \
    ./build/timesync >"$LOGA" 2>>"$LOGA" &
sudo env DEVOURER_VID=$B_VID DEVOURER_PID=$B_PID DEVOURER_CHANNEL=$CH \
    DEVOURER_TSYNC_ROLE=slave DEVOURER_TSYNC_SECS=$SECS DEVOURER_LOG_LEVEL=info \
    ./build/timesync >"$LOGB" 2>>"$LOGB" &

wait 2>/dev/null
sleep 1

echo "=== slave A summary ==="; grep -A6 "slave summary" "$LOGA" || echo "(no summary)"
echo "=== slave B summary ==="; grep -A6 "slave summary" "$LOGB" || echo "(no summary)"
echo "=== inter-slave (inter-UE) sync ==="
python3 tests/timesync_analyze.py "$LOGA" "$LOGB"
rm -f "$LOGA" "$LOGB"
