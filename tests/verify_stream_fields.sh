#!/usr/bin/env bash
# M0 verification: confirm the <devourer-stream> RX line now carries the decoded
# PHY descriptor fields (bw/stbc/ldpc/sgi) added in demo/main.cpp.
#
# Two-adapter loopback, NO SDR: an 8821AU transmits the canonical-SA beacon
# (WiFiDriverTxDemo) while an 8812AU receives with DEVOURER_STREAM_OUT=1. The
# stream line only fires on canonical-SA frames, so a devourer TX is required.
#
#   TX: RTL8821AU (TP-Link Archer T2U Plus, 2357:0120)  --ch6-->  air
#   RX: RTL8812AU (0bda:8812), DEVOURER_STREAM_OUT=1
#
# Usage: sudo tests/verify_stream_fields.sh   (run from repo root, after a build)
set -u
CH=6
BIN=./build
TXPID=0x0120 ; TXVID=0x2357
RXPID=0x8812
OUT=$(mktemp /tmp/devourer-stream-verify.XXXXXX)
TXLOG=$(mktemp /tmp/devourer-tx-verify.XXXXXX)

cleanup() {
  # exact-comm kills so we never reap an unrelated process
  pkill -f "WiFiDriverTxDemo" 2>/dev/null
  pkill -f "WiFiDriverDemo"   2>/dev/null
  rm -f "$OUT" "$TXLOG"
}
trap cleanup EXIT INT TERM

echo "[verify] starting 8821AU canonical-SA TX on ch$CH ..."
DEVOURER_VID=$TXVID DEVOURER_PID=$TXPID DEVOURER_CHANNEL=$CH \
  "$BIN/WiFiDriverTxDemo" >"$TXLOG" 2>&1 &
sleep 8   # let the TX adapter init + start injecting

echo "[verify] starting 8812AU RX (STREAM_OUT) for 30s ..."
timeout 30 env DEVOURER_PID=$RXPID DEVOURER_CHANNEL=$CH DEVOURER_STREAM_OUT=1 \
  "$BIN/WiFiDriverDemo" 2>/dev/null >"$OUT"

N=$(grep -c devourer-stream "$OUT")
echo "[verify] <devourer-stream> lines: $N"
if [ "$N" -gt 0 ]; then
  echo "[verify] sample (body truncated):"
  grep -m2 devourer-stream "$OUT" | sed 's/body=.*/body=.../'
  if grep -q "bw=.*stbc=.*ldpc=.*sgi=" "$OUT"; then
    echo "[verify] RESULT: PASS — bw/stbc/ldpc/sgi present in stream line"
    exit 0
  fi
  echo "[verify] RESULT: FAIL — stream line present but new fields missing"
  exit 1
fi
echo "[verify] RESULT: no canonical-SA frames RX'd. Check both adapters are"
echo "         plugged, on ch$CH, and the TX log below:"
tail -15 "$TXLOG"
exit 1
