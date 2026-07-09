#!/usr/bin/env bash
# tdma_tsf_test.sh — compare marker vs tsf sync in the tdma example, and sweep
# the guard. The tsf RX must be a Jaguar1 adapter (per-frame tsfl); the TX must
# be Jaguar1 too if you want the TX-TSF drift stamp (ReadTsf). Uses SUDO in the
# trap cleanup — the demo binaries run as root, so a plain pkill can't reap them.
#
#   sudo tests/tdma_tsf_test.sh <TX_PID> <RX_PID> <sync> <guard_ms> [ch] [nb] [sec]
set -uo pipefail
cd "$(dirname "$0")/.."
TX_PID="${1:-0x8813}"   # 8814AU (J1) TX
RX_PID="${2:-0x8812}"   # 8812AU (J1) RX — has tsfl
SYNC="${3:-tsf}"        # marker | tsf | wallclock
GUARD="${4:-20}"
CH="${5:-36}"
NB="${6:-10}"
SEC="${7:-15}"
BIN="$(pwd)/build/tdma"

TXLOG=$(mktemp)
cleanup() { sudo pkill -9 -x tdma 2>/dev/null || true; rm -f "$TXLOG"; }
trap cleanup EXIT
sudo pkill -9 -x tdma 2>/dev/null || true; sleep 2

# Robust bulk rate: MCS7 does not decode at 5 GHz on this bench (SNR); 6M does.
BULK="${BULK:-6M}"
BURST="${BURST:-100}"   # nb_ms = wide_ms = BURST (short bursts stress the anchor)
sudo env DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="$CH" DEVOURER_TDMA_ROLE=tx \
  DEVOURER_TDMA_NB="$NB" DEVOURER_TDMA_NB_MS="$BURST" DEVOURER_TDMA_WIDE_MS="$BURST" \
  DEVOURER_TDMA_BULK_RATE="$BULK" DEVOURER_TDMA_GAP_US=300 DEVOURER_LOG_LEVEL=silent \
  "$BIN" >"$TXLOG" 2>&1 &
sleep 4
echo "### sync=$SYNC guard=${GUARD}ms burst=${BURST}ms  (TX $TX_PID -> RX $RX_PID, ch$CH ${NB}MHz bulk=$BULK) ###"
sudo env DEVOURER_PID="$RX_PID" DEVOURER_CHANNEL="$CH" DEVOURER_TDMA_ROLE=rx-sync \
  DEVOURER_TDMA_SYNC="$SYNC" DEVOURER_TDMA_NB="$NB" DEVOURER_TDMA_NB_MS="$BURST" \
  DEVOURER_TDMA_WIDE_MS="$BURST" DEVOURER_TDMA_GUARD_MS="$GUARD" DEVOURER_LOG_LEVEL=silent \
  timeout -k 3 "$SEC" "$BIN" 2>&1 | grep -iE "delivered"
