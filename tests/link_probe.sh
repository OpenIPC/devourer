#!/usr/bin/env bash
# Active link-probe — the adaptive-link building block. One adapter emits a
# modulated beacon feed and sweeps a lever (--axis power | mcs) in steps; a second
# adapter (the "ground station") reads per-frame SNR + the frame-free NHM power
# histogram at each step. tests/link_probe.py aligns the two by wall-clock and
# reports the margin-vs-lever curve + the operating point that meets a target:
#   power axis — the cheapest power that clears the SNR floor (least power that
#     holds the link); works on any emitter generation (the TXAGC ramp
#     DEVOURER_TX_PWR_START/STOP/STEP rides the runtime TX-power API).
#   mcs axis  — the highest rate whose ground SNR clears the floor (ride the
#     fastest modulation the link holds); works on any emitter generation.
# The ground can be any generation. Two adapters, no SDR. The emitter's PA thermal
# meter is polled and reported as a thermal-budget overlay.
#
#   sudo tests/link_probe.sh --emit-pid 0x8812 --ground-pid 0xc812 --channel 36 \
#        --axis power --rate MCS4 --pwr-start 4 --pwr-stop 40 --step-ms 1500
#   sudo tests/link_probe.sh --emit-pid 0x8812 --ground-pid 0xc812 --channel 36 \
#        --axis mcs --mcs-list MCS0,MCS2,MCS4,MCS6,MCS7 --step-ms 3000 --target-snr 10
#
# NB (bench): sweep the noise-limited (lower-power) regime for a monotonic SNR
# curve — pushing very high power with the two adapters co-located inches apart
# saturates the ground RX's AGC and the SNR collapses. A validated bench run
# (indices 2..18) climbed SNR ~2->7 dB monotonically and the analyzer picked the
# minimum index clearing the target.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
TXDEMO="$ROOT/build/txdemo"
RXDEMO="$ROOT/build/rxdemo"

EMIT_VID=0x0bda EMIT_PID="" GROUND_VID=0x0bda GROUND_PID=""
CHANNEL=36 RATE=MCS4
PWR_START=4 PWR_STOP=40 PWR_STEP=4 STEP_MS=1500
TARGET_SNR=20 ENERGY_MS=300 OUT=/tmp/devourer-link-probe
AXIS=power MCS_LIST="MCS0,MCS2,MCS4,MCS6,MCS7"

while [ $# -gt 0 ]; do
  case "$1" in
    --emit-vid) EMIT_VID="$2"; shift 2 ;;
    --emit-pid) EMIT_PID="$2"; shift 2 ;;
    --ground-vid) GROUND_VID="$2"; shift 2 ;;
    --ground-pid) GROUND_PID="$2"; shift 2 ;;
    --channel) CHANNEL="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    --axis) AXIS="$2"; shift 2 ;;           # power | mcs
    --mcs-list) MCS_LIST="$2"; shift 2 ;;
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
[ "$AXIS" = power ] || [ "$AXIS" = mcs ] || { echo "--axis power|mcs" >&2; exit 2; }
[ -n "$EMIT_PID" ] && [ -n "$GROUND_PID" ] || {
  echo "need --emit-pid and --ground-pid" >&2; exit 2; }

# Host-side line timestamper (no moreutils dependency).
TS='import sys,time
for l in sys.stdin:
    sys.stdout.write(f"{time.time():.3f} {l}"); sys.stdout.flush()'

cleanup() { for c in txdemo rxdemo; do pkill -x "$c" 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM

echo "== building =="
cmake --build "$ROOT/build" -j >/dev/null
mkdir -p "$OUT"; rm -f "$OUT"/emit.log "$OUT"/ground.log
cleanup; sleep 1

# Total sweep time: number of steps * step-ms, + bring-up + margin.
if [ "$AXIS" = power ]; then
  nsteps=$(( (PWR_STOP - PWR_START) / PWR_STEP + 1 ))
else
  nsteps=$(( $(awk -F',' '{print NF}' <<< "$MCS_LIST") ))
fi
run_secs=$(( nsteps * STEP_MS / 1000 + 12 ))

echo "== ground station: $GROUND_PID ch$CHANNEL energy sensor =="
timeout -sINT "$((run_secs + 4))" env DEVOURER_VID="$GROUND_VID" \
  DEVOURER_PID="$GROUND_PID" DEVOURER_CHANNEL="$CHANNEL" \
  DEVOURER_RX_ENERGY_MS="$ENERGY_MS" "$RXDEMO" 2>/dev/null \
  | python3 -c "$TS" > "$OUT/ground.log" &
sleep 6   # let the ground RX come up before the emitter starts stepping

# The probe uses the plain modulated beacon feed (decodable per-frame SNR at each
# step) — NOT the HW-continuous carrier (DEVOURER_CONT_TX), which is idle-hold and
# doesn't step. Power axis = the TXAGC ramp; MCS axis = the rate sweep.
if [ "$AXIS" = power ]; then
  echo "== emitter: $EMIT_PID $RATE, power $PWR_START..$PWR_STOP step $PWR_STEP =="
  emit_env=(DEVOURER_TX_RATE="$RATE" DEVOURER_TX_PWR_START="$PWR_START"
            DEVOURER_TX_PWR_STOP="$PWR_STOP" DEVOURER_TX_PWR_STEP="$PWR_STEP"
            DEVOURER_TX_PWR_STEP_MS="$STEP_MS")
else
  echo "== emitter: $EMIT_PID MCS sweep [$MCS_LIST] =="
  emit_env=(DEVOURER_TX_MCS_SWEEP="$MCS_LIST" DEVOURER_TX_MCS_STEP_MS="$STEP_MS")
fi
# DEVOURER_THERMAL_POLL_MS on the emitter feeds the thermal-budget overlay (the
# PA heats under the swept full-duty load; link_probe.py reports the drift).
timeout -sINT "$run_secs" env DEVOURER_VID="$EMIT_VID" DEVOURER_PID="$EMIT_PID" \
  DEVOURER_CHANNEL="$CHANNEL" DEVOURER_THERMAL_POLL_MS=1000 "${emit_env[@]}" \
  "$TXDEMO" 2>&1 | python3 -c "$TS" > "$OUT/emit.log" || true

cleanup
echo "== margin-vs-$AXIS curve =="
python3 "$HERE/link_probe.py" --emit "$OUT/emit.log" --ground "$OUT/ground.log" \
  --target-snr "$TARGET_SNR"
