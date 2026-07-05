#!/usr/bin/env bash
# Active link-probe — the adaptive-link building block. One adapter emits a
# modulated continuous-TX carrier (DEVOURER_CONT_TX) and sweeps its TX power in
# steps; a second adapter (the "ground station") reads per-frame SNR + the
# frame-free NHM power histogram at each step. tests/link_probe.py aligns the two
# by wall-clock and reports the margin-vs-power curve + the cheapest power that
# clears a target SNR (the energy-min reflex: least power that holds the link).
#
# The power sweep reuses the Jaguar1 TXAGC ramp (DEVOURER_TX_PWR_START/STOP/STEP),
# so the EMITTER must be a Jaguar1 part (8812AU/8814AU/8821AU). The ground can be
# any generation. Two adapters, no SDR.
#
#   sudo tests/link_probe.sh --emit-pid 0x8812 --ground-pid 0xc812 \
#        --channel 36 --rate MCS4 --pwr-start 4 --pwr-stop 40 --pwr-step 4 \
#        --step-ms 1500 --target-snr 20
#
# NB (bench): sweep the noise-limited (lower-power) regime for a monotonic SNR
# curve — pushing very high power with the two adapters co-located inches apart
# saturates the ground RX's AGC and the SNR collapses. A validated bench run
# (indices 2..18) climbed SNR ~2->7 dB monotonically and the analyzer picked the
# minimum index clearing the target.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
TXDEMO="$ROOT/build/WiFiDriverTxDemo"
RXDEMO="$ROOT/build/WiFiDriverDemo"

EMIT_VID=0x0bda EMIT_PID="" GROUND_VID=0x0bda GROUND_PID=""
CHANNEL=36 RATE=MCS4
PWR_START=4 PWR_STOP=40 PWR_STEP=4 STEP_MS=1500
TARGET_SNR=20 ENERGY_MS=300 OUT=/tmp/devourer-link-probe

while [ $# -gt 0 ]; do
  case "$1" in
    --emit-vid) EMIT_VID="$2"; shift 2 ;;
    --emit-pid) EMIT_PID="$2"; shift 2 ;;
    --ground-vid) GROUND_VID="$2"; shift 2 ;;
    --ground-pid) GROUND_PID="$2"; shift 2 ;;
    --channel) CHANNEL="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    --pwr-start) PWR_START="$2"; shift 2 ;;
    --pwr-stop) PWR_STOP="$2"; shift 2 ;;
    --pwr-step) PWR_STEP="$2"; shift 2 ;;
    --step-ms) STEP_MS="$2"; shift 2 ;;
    --target-snr) TARGET_SNR="$2"; shift 2 ;;
    --energy-ms) ENERGY_MS="$2"; shift 2 ;;
    --outdir) OUT="$2"; shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done
[ -n "$EMIT_PID" ] && [ -n "$GROUND_PID" ] || {
  echo "need --emit-pid (Jaguar1) and --ground-pid" >&2; exit 2; }

# Host-side line timestamper (no moreutils dependency).
TS='import sys,time
for l in sys.stdin:
    sys.stdout.write(f"{time.time():.3f} {l}"); sys.stdout.flush()'

cleanup() { for c in WiFiDriverTxDemo WiFiDriverDemo; do pkill -x "$c" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM

echo "== building =="
cmake --build "$ROOT/build" -j >/dev/null
mkdir -p "$OUT"; rm -f "$OUT"/emit.log "$OUT"/ground.log
cleanup; sleep 1

# Total sweep time: number of steps * step-ms, + bring-up + margin.
nsteps=$(( (PWR_STOP - PWR_START) / PWR_STEP + 1 ))
run_secs=$(( nsteps * STEP_MS / 1000 + 12 ))

echo "== ground station: $GROUND_PID ch$CHANNEL energy sensor =="
timeout -sINT "$((run_secs + 4))" env DEVOURER_VID="$GROUND_VID" \
  DEVOURER_PID="$GROUND_PID" DEVOURER_CHANNEL="$CHANNEL" \
  DEVOURER_RX_ENERGY_MS="$ENERGY_MS" "$RXDEMO" 2>/dev/null \
  | python3 -c "$TS" > "$OUT/ground.log" &
sleep 6   # let the ground RX come up before the emitter starts stepping

echo "== emitter: $EMIT_PID cont-TX $RATE, power $PWR_START..$PWR_STOP step $PWR_STEP =="
# The probe uses the plain modulated beacon feed at a steady rate + the TXAGC
# power ramp (decodable per-frame SNR at each step) — NOT the HW-continuous
# carrier (DEVOURER_CONT_TX), which is idle-hold and doesn't step power.
timeout -sINT "$run_secs" env DEVOURER_VID="$EMIT_VID" DEVOURER_PID="$EMIT_PID" \
  DEVOURER_CHANNEL="$CHANNEL" DEVOURER_TX_RATE="$RATE" \
  DEVOURER_TX_PWR_START="$PWR_START" \
  DEVOURER_TX_PWR_STOP="$PWR_STOP" DEVOURER_TX_PWR_STEP="$PWR_STEP" \
  DEVOURER_TX_PWR_STEP_MS="$STEP_MS" "$TXDEMO" 2>&1 \
  | python3 -c "$TS" > "$OUT/emit.log" || true

cleanup
echo "== margin-vs-power curve =="
python3 "$HERE/link_probe.py" --emit "$OUT/emit.log" --ground "$OUT/ground.log" \
  --target-snr "$TARGET_SNR"
