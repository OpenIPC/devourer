#!/usr/bin/env bash
# Per-tone interferer localization from beamforming reports — the finest
# frequency sensor devourer has (sub-channel, vs the 20 MHz grid of
# rx_spectrum_sweep.sh). Runs the MU self-sounding rig and, optionally, parks a
# CW-tone interferer on a nearby channel so it lands off-centre within the
# sounded band; the sniffer's reports are localized by tests/rx_tone_localize.py.
#
#   sounder   : 8812AU   (MU NDPA)
#   beamformee: 8822CU   (MU per-subcarrier SNR report)
#   sniffer   : 8814AU   (captures reports -> localizer)
#   interferer: optional (--tone-pid) CW tone on --tone-channel
#
# Two-capture (differential) is the most sensitive: run once clean, once with
# the tone, and diff. This script captures a clean baseline then, if --tone-pid
# is given, an interferer capture, and runs the localizer in differential mode.
#
#   sudo tests/rx_tone_localize.sh --channel 100
#   sudo tests/rx_tone_localize.sh --channel 100 --tone-pid 0xc811 \
#        --tone-channel 102 --op-snr 28
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$HERE")"
TXDEMO="$ROOT/build/WiFiDriverTxDemo"
RXDEMO="$ROOT/build/WiFiDriverDemo"
LOC="$HERE/rx_tone_localize.py"

CHANNEL=100
BFER_MAC="57:42:75:05:d6:00"
BFEE_MAC="00:e0:4c:88:22:ce"
SOUNDER_PID=0x8812 BFEE_PID=0xc812 SNIFFER_PID=0x8813
TONE_VID=0x0bda TONE_PID="" TONE_CHANNEL="" TONE_GAIN=10
SECS=15 OUT=/tmp/devourer-tone-localize OP_SNR=""

while [ $# -gt 0 ]; do
  case "$1" in
    --channel) CHANNEL="$2"; shift 2 ;;
    --sounder-pid) SOUNDER_PID="$2"; shift 2 ;;
    --bfee-pid) BFEE_PID="$2"; shift 2 ;;
    --sniffer-pid) SNIFFER_PID="$2"; shift 2 ;;
    --tone-vid) TONE_VID="$2"; shift 2 ;;
    --tone-pid) TONE_PID="$2"; shift 2 ;;
    --tone-channel) TONE_CHANNEL="$2"; shift 2 ;;
    --tone-gain) TONE_GAIN="$2"; shift 2 ;;
    --secs) SECS="$2"; shift 2 ;;
    --op-snr) OP_SNR="$2"; shift 2 ;;
    --outdir) OUT="$2"; shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done
[ -z "$TONE_CHANNEL" ] && TONE_CHANNEL="$CHANNEL"

[ -x "$TXDEMO" ] && [ -x "$RXDEMO" ] || { echo "build demos first" >&2; exit 1; }
mkdir -p "$OUT"

cleanup() {
  for c in WiFiDriverTxDem WiFiDriverDemo; do pkill -x "$c" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM
cleanup; sleep 1

arm_rig() {
  env DEVOURER_PID="$BFEE_PID" DEVOURER_CHANNEL=$CHANNEL \
    DEVOURER_BF_ARM_BFEE="$BFER_MAC" DEVOURER_BF_ARM_BFEE_MU=1 \
    "$RXDEMO" > "$OUT/bfee.log" 2>&1 &
  for _ in $(seq 80); do grep -q "entering RX loop" "$OUT/bfee.log" && break; sleep 0.5; done
  env DEVOURER_PID="$SOUNDER_PID" DEVOURER_CHANNEL=$CHANNEL \
    DEVOURER_TX_RATE=VHT2SS_MCS0 DEVOURER_TX_NDPA_RA="$BFEE_MAC" \
    DEVOURER_TX_NDPA=1 DEVOURER_TX_NDPA_MU=1 DEVOURER_BF_ARM_SOUNDER=1 \
    "$TXDEMO" > "$OUT/sounder.log" 2>&1 &
  sleep 2
}

capture() { # $1 = output capture file
  timeout -sINT "$((SECS + 3))" env DEVOURER_PID="$SNIFFER_PID" \
    DEVOURER_CHANNEL=$CHANNEL DEVOURER_BF_DETECT_REPORT=4 "$RXDEMO" 2>/dev/null \
    | grep --line-buffered "devourer-bf-report-raw" > "$1" || true
}

echo "== arming MU self-sounding rig (ch $CHANNEL, bfee init ~20s) =="
arm_rig

echo "== clean baseline capture (${SECS}s) =="
capture "$OUT/clean.txt"
echo "  $(wc -l < "$OUT/clean.txt") reports"

if [ -n "$TONE_PID" ]; then
  echo "== interferer ON: $TONE_PID CW tone ch$TONE_CHANNEL gain $TONE_GAIN =="
  timeout -sINT "$((SECS + 20))" env DEVOURER_VID="$TONE_VID" \
    DEVOURER_PID="$TONE_PID" DEVOURER_CHANNEL="$TONE_CHANNEL" \
    DEVOURER_CW_TONE=1 DEVOURER_CW_TONE_GAIN="$TONE_GAIN" \
    "$TXDEMO" > "$OUT/tone.emit.log" 2>&1 &
  # CW bring-up can take ~6 s (IQK); poll up to 15 s for the arm line.
  for _ in $(seq 30); do
    grep -q "single-tone armed" "$OUT/tone.emit.log" && break; sleep 0.5
  done
  grep -q "single-tone armed" "$OUT/tone.emit.log" && echo "  tone armed" \
    || echo "  WARN: tone not armed (see $OUT/tone.emit.log)"
  capture "$OUT/interf.txt"
  echo "  $(wc -l < "$OUT/interf.txt") reports"
fi

cleanup

echo "== localize =="
loc_args=(--channel "$CHANNEL")
if [ -n "$TONE_PID" ] && [ -s "$OUT/interf.txt" ]; then
  python3 "$LOC" "$OUT/interf.txt" --baseline "$OUT/clean.txt" "${loc_args[@]}"
else
  python3 "$LOC" "$OUT/clean.txt" "${loc_args[@]}"
fi
