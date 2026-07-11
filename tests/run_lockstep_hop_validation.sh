#!/usr/bin/env bash
# Two-Realtek-adapter lockstep FHSS validation (hardware-only).
# Usage: TX_PID=0x8812 RX_PID=0xb82c tests/run_lockstep_hop_validation.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CHANNELS="${HOP_CHANNELS:-36,40,44,48}"
SEED="${HOP_SEED:-00112233445566778899aabbccddeeff}"
SLOT_MS="${HOP_SLOT_MS:-50}"
SECS="${SECS:-20}"
TX_PID="${TX_PID:?set TX_PID for the transmit adapter}"
RX_PID="${RX_PID:?set RX_PID for the receive adapter}"
OUT="${OUT:-/tmp/devourer-lockstep-hop}"
SUDO="${SUDO:-sudo -n}"
mkdir -p "$OUT"
cleanup(){ kill "${tx_proc:-}" "${rx_proc:-}" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
cmake --build "$ROOT/build" -j --target txdemo rxdemo >/dev/null
$SUDO env DEVOURER_PID="$RX_PID" DEVOURER_CHANNEL="${CHANNELS%%,*}" \
  DEVOURER_HOP_CHANNELS="$CHANNELS" DEVOURER_HOP_SEED="$SEED" \
  DEVOURER_HOP_SLOT_MS="$SLOT_MS" "$ROOT/build/rxdemo" >"$OUT/rx.log" 2>&1 & rx_proc=$!
$SUDO env DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="${CHANNELS%%,*}" \
  DEVOURER_HOP_CHANNELS="$CHANNELS" DEVOURER_HOP_SEED="$SEED" \
  DEVOURER_HOP_SLOT_MS="$SLOT_MS" DEVOURER_HOP_FAST=1 \
  DEVOURER_TX_GAP_US=2000 "$ROOT/build/txdemo" >"$OUT/tx.log" 2>&1 & tx_proc=$!
sleep "$SECS"
cleanup
grep -F '"ev":"hop.rx"' "$OUT/rx.log" || true
echo "logs: $OUT/tx.log $OUT/rx.log"
