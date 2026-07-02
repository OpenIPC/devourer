#!/usr/bin/env bash
# Live per-subcarrier SNR waterfall demo. Runs the three-adapter MU
# self-sounding rig and pipes the sniffer's report stream into
# tools/bf_waterfall.py for a scrolling truecolor spectrogram:
#
#   8812AU  : sounder      (MU NDPA)
#   8822CU  : MU-beamformee (per-subcarrier delta-SNR report)
#   8814AU  : monitor sniffer  →  waterfall
#
# Usage: sudo tests/bf_waterfall.sh [operating_snr_db]
#   operating_snr_db (optional) re-centres the measured per-tone SNR so the
#   QAM colour ramp spreads on a strong bench link (e.g. 28).

set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$HERE")"
TXDEMO="$ROOT/build/WiFiDriverTxDemo"
RXDEMO="$ROOT/build/WiFiDriverDemo"
WF="$ROOT/tools/bf_waterfall.py"
CHANNEL=100
BFER_MAC="57:42:75:05:d6:00"
BFEE_MAC="00:e0:4c:88:22:ce"
OP_SNR="${1:-}"

[ -x "$TXDEMO" ] && [ -x "$RXDEMO" ] || { echo "build demos first"; exit 1; }

cleanup() {
    pkill -x WiFiDriverTxDemo 2>/dev/null
    pkill -x WiFiDriverDemo 2>/dev/null
    wait 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "== arming 8822CU MU-beamformee (init ~20s) =="
env DEVOURER_PID=0xc812 DEVOURER_CHANNEL=$CHANNEL \
    DEVOURER_BF_ARM_BFEE="$BFER_MAC" DEVOURER_BF_ARM_BFEE_MU=1 \
    "$RXDEMO" > /tmp/bf-wf-bfee.log 2>&1 &
for _ in $(seq 80); do grep -q "entering RX loop" /tmp/bf-wf-bfee.log && break; sleep 0.5; done

echo "== starting 8812AU MU sounder =="
env DEVOURER_PID=0x8812 DEVOURER_CHANNEL=$CHANNEL DEVOURER_TX_RATE=VHT2SS_MCS0 \
    DEVOURER_TX_NDPA_RA="$BFEE_MAC" DEVOURER_TX_NDPA=1 DEVOURER_TX_NDPA_MU=1 \
    DEVOURER_BF_ARM_SOUNDER=1 "$TXDEMO" > /tmp/bf-wf-tx.log 2>&1 &
sleep 2

echo "== waterfall (Ctrl-C to stop) =="
wf_args=()
[ -n "$OP_SNR" ] && wf_args=(--operating-snr "$OP_SNR")
env DEVOURER_PID=0x8813 DEVOURER_CHANNEL=$CHANNEL DEVOURER_BF_DETECT_REPORT=4 \
    "$RXDEMO" 2>/dev/null | /usr/bin/python3 "$WF" "${wf_args[@]}"
