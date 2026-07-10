#!/usr/bin/env bash
# txegress_bench.sh — benchmark the "read TSF after send to approximate egress"
# micro-option (research angle 6) that the literature never measured.
#
# A transmitter (8812AU) airs frames carrying its own TX timestamp; a witness
# receiver (8822BU) reads that stamp AND its hardware rx_tsfl from the same
# packet, so one box measures the TX stamp's jitter versus true on-air time.
# Two transmit modes:
#   SOFTWARE : timesync master embeds ReadTsf() near the send call (the micro-
#              option) into each frame's TD tag.
#   HARDWARE : timesync master airs a hardware beacon; the MAC inserts the true
#              egress TSF into the beacon body — the real TX-egress reference.
#
#   sudo tests/txegress_bench.sh
set -uo pipefail
cd "$(dirname "$0")/.."
TX_VID=${TX_VID:-0x0bda}; TX_PID=${TX_PID:-0x8812}     # 8812AU transmitter
RX_VID=${RX_VID:-0x2357}; RX_PID=${RX_PID:-0x012d}     # 8822BU witness (TP-Link)
CH=${CH:-6}; N=${N:-400}
SW=/tmp/txeg_sw.jsonl; HW=/tmp/txeg_hw.jsonl

cleanup(){ sudo pkill -9 -x timesync txegress_witness 2>/dev/null; }
trap cleanup EXIT
cleanup; sleep 1

run_mode() {
  local label="$1" out="$2"; shift 2
  echo "== $label =="
  sudo env DEVOURER_VID=$TX_VID DEVOURER_PID=$TX_PID DEVOURER_CHANNEL=$CH \
      DEVOURER_TSYNC_ROLE=master DEVOURER_TSYNC_INTERVAL_MS=10 DEVOURER_TSYNC_SECS=45 \
      "$@" ./build/timesync >/tmp/txeg_tx.log 2>&1 &
  sleep 4
  sudo env DEVOURER_VID=$RX_VID DEVOURER_PID=$RX_PID DEVOURER_CHANNEL=$CH \
      ./build/txegress_witness "$N" 2>/dev/null > "$out"
  sudo pkill -9 -x timesync 2>/dev/null; sleep 2
  echo "  frames: $(grep -c '"ev":"txeg"' "$out")"
}

run_mode "SOFTWARE (ReadTsf near send)" "$SW"
run_mode "HARDWARE (MAC beacon egress)" "$HW" DEVOURER_TSYNC_HWBEACON=1

echo
echo "==== SOFTWARE stamp vs true air ===="
python3 tests/txegress_analyze.py "$SW"
echo
echo "==== HARDWARE stamp vs true air ===="
python3 tests/txegress_analyze.py "$HW"
