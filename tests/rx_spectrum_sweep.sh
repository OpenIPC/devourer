#!/usr/bin/env bash
# Coarse RX spectrum map — a single live sweep. One sensor process cycles the
# listed channels (DEVOURER_RX_SWEEP), dwelling DEVOURER_RX_SWEEP_DWELL_MS on
# each and emitting one rx.energy event (with ch=N) per bin from the frame-free
# energy sensor. The devourer-native spectrum analyzer: no per-tone CSI (the
# silicon can't export it), just channel-wide energy per bin.
#
# The RX loop runs on a worker thread while the main thread retunes
# (SetMonitorChannel) between reads — uniform across all three generations
# (Jaguar2's SetMonitorChannel now retunes), so it's one process, not a relaunch
# per channel. Resolution = the channel grid: 20 MHz on the 2.4/5 GHz plan;
# ~5 MHz on Jaguar3 with --nb-bw 5 (the 2.4 GHz channels are 5 MHz apart, so
# stepping them at 5 MHz bandwidth gives 5 MHz bins).
#
#   sudo ./tests/rx_spectrum_sweep.sh --sensor-pid 0xc812 --channels 1,3,5,6,7,9,11
#   sudo ./tests/rx_spectrum_sweep.sh --sensor-pid 0xc812 --nb-bw 5 \
#        --channels 1,2,3,4,5,6,7,8,9,10,11,12,13
#
# Park a DEVOURER_CW_TONE on one channel (another adapter) first; the map should
# peak (or, on the 1T1R parts that saturate, dip) at that channel.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
DEMO_RX="$ROOT/build/rxdemo"
DEMO_TX="$ROOT/build/txdemo"

SENSOR_VID=0x0bda SENSOR_PID="" CHANNELS="1,6,11" DWELL_MS=300 ROUNDS=3
NB_BW="" OUT=/tmp/devourer-rx-sweep
# Optional second adapter parking a CW tone (self-validation): the map should
# peak (or, on a saturating 1T1R sensor, dip) at --tone-channel.
TONE_VID=0x0bda TONE_PID="" TONE_CHANNEL=6 TONE_GAIN=8

while [ $# -gt 0 ]; do
  case "$1" in
    --sensor-vid) SENSOR_VID="$2"; shift 2 ;;
    --sensor-pid) SENSOR_PID="$2"; shift 2 ;;
    --channels) CHANNELS="$2"; shift 2 ;;
    --dwell-ms) DWELL_MS="$2"; shift 2 ;;
    --rounds) ROUNDS="$2"; shift 2 ;;
    --nb-bw) NB_BW="$2"; shift 2 ;;
    --outdir) OUT="$2"; shift 2 ;;
    --tone-vid) TONE_VID="$2"; shift 2 ;;
    --tone-pid) TONE_PID="$2"; shift 2 ;;
    --tone-channel) TONE_CHANNEL="$2"; shift 2 ;;
    --tone-gain) TONE_GAIN="$2"; shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done
[ -n "$SENSOR_PID" ] || { echo "need --sensor-pid" >&2; exit 2; }

cleanup() { for c in rxdemo txdemo; do pkill -x "$c" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM

echo "== building =="
cmake --build "$ROOT/build" -j >/dev/null
mkdir -p "$OUT"; rm -f "$OUT"/sweep.log
cleanup; sleep 1

if [ -n "$TONE_PID" ]; then
  echo "== parking CW tone: $TONE_PID ch$TONE_CHANNEL gain $TONE_GAIN =="
  timeout -sINT 600 env DEVOURER_VID="$TONE_VID" DEVOURER_PID="$TONE_PID" \
    DEVOURER_CHANNEL="$TONE_CHANNEL" DEVOURER_CW_TONE=1 \
    DEVOURER_CW_TONE_GAIN="$TONE_GAIN" "$DEMO_TX" >"$OUT/tone.emit.log" 2>&1 &
  sleep 8
  grep -q "single-tone armed" "$OUT/tone.emit.log" && echo "  tone armed" \
    || echo "  WARN: tone not armed (see $OUT/tone.emit.log)"
fi

# nbins * rounds bins, ~DWELL_MS each, + ~4 s bring-up, + margin.
nbins="$(awk -F',' '{print NF}' <<< "$CHANNELS")"
secs="$(( (nbins * ROUNDS * DWELL_MS) / 1000 + 8 ))"

echo "== live sweep: $CHANNELS x $ROUNDS rounds, ${DWELL_MS}ms dwell (~${secs}s) =="
timeout -sINT "$secs" env DEVOURER_VID="$SENSOR_VID" \
  DEVOURER_PID="$SENSOR_PID" DEVOURER_RX_SWEEP="$CHANNELS" \
  DEVOURER_RX_SWEEP_DWELL_MS="$DWELL_MS" ${NB_BW:+DEVOURER_NB_BW="$NB_BW"} \
  "$DEMO_RX" 2>/dev/null | grep --line-buffered -F '"ev":"rx.energy"' \
  | tee "$OUT/sweep.log" || true
cleanup

echo "== spectrum map =="
python3 "$HERE/rx_spectrum_sweep.py" --log "$OUT/sweep.log" \
  ${NB_BW:+--nb-bw "$NB_BW"}
