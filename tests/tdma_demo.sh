#!/usr/bin/env bash
# tdma_demo.sh — drive the burst-level bandwidth-TDMA example (examples/tdma)
# through its three scenarios on plugged adapters and print the per-band /
# per-class decode tables. Reads: critical frames ride the narrowband burst,
# bulk frames the wide burst; a correct run shows critical+marker under the
# narrowband band and bulk under the wide band, ~0 off-diagonal.
#
#   sudo tests/tdma_demo.sh [TX_PID] [RX1_VID:RX1_PID] [RX2_VID:RX2_PID] [CH] [NB]
# defaults use this bench: TX 8812AU, RX1 8822CU, RX2 8822BU, ch36, NB=10.
set -uo pipefail
cd "$(dirname "$0")/.."

TX_PID="${1:-0x8812}"
RX1="${2:-0x0bda:0xc812}"     # narrowband / lockstep receiver
RX2="${3:-0x2357:0x012d}"     # wide receiver (mode 2)
CH="${4:-36}"
NB="${5:-10}"
RX1_VID="${RX1%:*}"; RX1_PID="${RX1#*:}"
RX2_VID="${RX2%:*}"; RX2_PID="${RX2#*:}"
BIN="$(pwd)/build/tdma"
[ -x "$BIN" ] || { echo "build/tdma missing — run: cmake --build build -j"; exit 1; }

SCHED=(DEVOURER_TDMA_NB="$NB" DEVOURER_TDMA_NB_MS=100 DEVOURER_TDMA_WIDE_MS=100 DEVOURER_TDMA_GAP_US=300)
TXLOG=$(mktemp); RXA=$(mktemp); RXB=$(mktemp)
cleanup() { pkill -9 -x tdma 2>/dev/null || true; rm -f "$TXLOG" "$RXA" "$RXB"; }
trap cleanup EXIT
pkill -9 -x tdma 2>/dev/null || true; sleep 1

start_tx() {
  sudo env DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="$CH" DEVOURER_TDMA_ROLE=tx \
    "${SCHED[@]}" DEVOURER_LOG_LEVEL=silent "$BIN" >"$TXLOG" 2>&1 &
  disown   # suppress the job-control "Killed" notice when cleanup ends it
  sleep 4
}
show() { grep -A3 'summary' "$1" | tail -3; }

echo "############ Mode 1 — lockstep, WALL-CLOCK sync (TX + rx-sync) ############"
start_tx
sudo env DEVOURER_VID="$RX1_VID" DEVOURER_PID="$RX1_PID" DEVOURER_CHANNEL="$CH" \
  DEVOURER_TDMA_ROLE=rx-sync DEVOURER_TDMA_SYNC=wallclock DEVOURER_TDMA_GUARD_MS=20 \
  "${SCHED[@]}" DEVOURER_LOG_LEVEL=silent timeout -k 3 12 "$BIN" 2>&1 | grep -iE 'summary|narrowband|^  wide'
cleanup; sleep 2

echo
echo "############ Mode 1 — lockstep, MARKER sync (no shared clock) ############"
start_tx
sudo env DEVOURER_VID="$RX1_VID" DEVOURER_PID="$RX1_PID" DEVOURER_CHANNEL="$CH" \
  DEVOURER_TDMA_ROLE=rx-sync DEVOURER_TDMA_SYNC=marker DEVOURER_TDMA_GUARD_MS=20 \
  "${SCHED[@]}" DEVOURER_LOG_LEVEL=silent timeout -k 3 12 "$BIN" 2>&1 | grep -iE 'summary|narrowband|^  wide'
cleanup; sleep 2

echo
echo "############ Mode 2 — dual-RX per-band (TX + camp-NB + camp-wide) ############"
start_tx
sudo env DEVOURER_VID="$RX1_VID" DEVOURER_PID="$RX1_PID" DEVOURER_CHANNEL="$CH" \
  DEVOURER_TDMA_ROLE=rx-camp DEVOURER_TDMA_CAMP="$NB" \
  "${SCHED[@]}" DEVOURER_LOG_LEVEL=silent timeout -k 3 12 "$BIN" >"$RXA" 2>&1 &
sudo env DEVOURER_VID="$RX2_VID" DEVOURER_PID="$RX2_PID" DEVOURER_CHANNEL="$CH" \
  DEVOURER_TDMA_ROLE=rx-camp DEVOURER_TDMA_CAMP=20 \
  "${SCHED[@]}" DEVOURER_LOG_LEVEL=silent timeout -k 3 12 "$BIN" >"$RXB" 2>&1 &
sleep 14
echo "-- RX-A camps ${NB} MHz narrowband (the robust critical link) --"; show "$RXA"
echo "-- RX-B camps 20 MHz wide (the bulk link) --"; show "$RXB"
echo
echo "PASS criteria: critical+marker land under 'narrowband', bulk under 'wide',"
echo "off-diagonal ~0 (a handful = frames caught mid-switch)."
