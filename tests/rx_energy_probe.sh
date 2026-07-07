#!/usr/bin/env bash
# RX energy / interferer-detection validation — the read side of DEVOURER_CW_TONE.
#
# Runs a two-adapter, no-SDR test: one adapter emits a CW tone (DEVOURER_CW_TONE)
# on a channel; a second adapter runs the RX demo with DEVOURER_RX_ENERGY_MS and
# reports the frame-free energy telemetry (rx.energy events). We capture the
# sensor's telemetry with the tone OFF (baseline) and ON, then assert the two are
# clearly separable (rx_energy_check.py). A strong co-located CW pushes the
# sensor's CCA counter far out of its ambient band — either a large spike (the
# CCA registers the carrier as busy) or a collapse toward zero (the carrier
# saturates the AGC and the RX goes deaf). Both are unambiguous detections.
#
# Requires two USB Wi-Fi adapters plugged in and sudo (USB claim).
#
#   sudo ./tests/rx_energy_probe.sh --sensor-pid 0xc812 \
#        --tone-pid 0x8812 --channel 6
#   sudo ./tests/rx_energy_probe.sh --sensor-vid 0x2357 --sensor-pid 0x0120 ...
#
# Cleanup: exact-comm kills of both demos on any exit.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
DEMO_RX="$ROOT/build/rxdemo"
DEMO_TX="$ROOT/build/txdemo"

SENSOR_VID=0x0bda SENSOR_PID=""
TONE_VID=0x0bda TONE_PID=""
CHANNEL=6 GAIN=8 ENERGY_MS=800 SECS=8 OUT=/tmp/devourer-rx-energy

while [ $# -gt 0 ]; do
  case "$1" in
    --sensor-vid) SENSOR_VID="$2"; shift 2 ;;
    --sensor-pid) SENSOR_PID="$2"; shift 2 ;;
    --tone-vid) TONE_VID="$2"; shift 2 ;;
    --tone-pid) TONE_PID="$2"; shift 2 ;;
    --channel) CHANNEL="$2"; shift 2 ;;
    --gain) GAIN="$2"; shift 2 ;;
    --energy-ms) ENERGY_MS="$2"; shift 2 ;;
    --secs) SECS="$2"; shift 2 ;;
    --outdir) OUT="$2"; shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done
[ -n "$SENSOR_PID" ] && [ -n "$TONE_PID" ] || {
  echo "need --sensor-pid and --tone-pid" >&2; exit 2; }

cleanup() { for c in rxdemo txdemo; do pkill -x "$c" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM

echo "== building =="
cmake --build "$ROOT/build" -j >/dev/null
mkdir -p "$OUT"
cleanup; sleep 2

sense() { # $1=logfile
  timeout -sINT "$((SECS + 3))" env DEVOURER_VID="$SENSOR_VID" \
    DEVOURER_PID="$SENSOR_PID" DEVOURER_CHANNEL="$CHANNEL" \
    DEVOURER_RX_ENERGY_MS="$ENERGY_MS" "$DEMO_RX" 2>/dev/null \
    | grep --line-buffered -F '"ev":"rx.energy"' > "$1" || true
}

echo "== baseline (tone OFF) =="
sense "$OUT/baseline.log"
sleep 2

echo "== tone ON ($TONE_PID ch$CHANNEL gain $GAIN) =="
timeout -sINT "$((SECS + 20))" env DEVOURER_VID="$TONE_VID" \
  DEVOURER_PID="$TONE_PID" DEVOURER_CHANNEL="$CHANNEL" DEVOURER_CW_TONE=1 \
  DEVOURER_CW_TONE_GAIN="$GAIN" "$DEMO_TX" >"$OUT/tone.emit.log" 2>&1 &
TX=$!
sleep 8
grep -q "single-tone armed" "$OUT/tone.emit.log" && echo "  tone armed" \
  || echo "  WARN: tone not armed (see $OUT/tone.emit.log)"
sense "$OUT/tone.log"
kill -INT "$TX" 2>/dev/null || true; wait "$TX" 2>/dev/null || true

echo "== verdict =="
python3 "$HERE/rx_energy_check.py" "$OUT/baseline.log" "$OUT/tone.log"
