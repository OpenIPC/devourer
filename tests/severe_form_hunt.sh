#!/usr/bin/env bash
# Hunt a deterministic reproduction of #348's SEVERE form: 64-QAM and up at 0%
# delivery while 16-QAM and below stay perfect, recovered only by removing power.
#
# The mild form (~5-7 points across repeated identical sessions) is barely
# outside this bench's noise — a single MCS7/20 probe has sd 1.8, so anything
# under ~4 points is not a finding. The severe form is a >20-point collapse and
# is unmistakable. Nothing has reproduced it since; the original sighting
# followed a long session that, unlike the fixed-config repro, kept CHANGING the
# radio: 2.4<->5 GHz bands, 20/40/80 bandwidths, flat max power indices, large
# offsets, and 5 GHz tunes that never delivered.
#
# So each arm here adds exactly ONE of those stressors on top of the same
# session loop, and every arm is measured the same way. That identifies the
# responsible stressor rather than just eventually breaking the chip.
#
# Sessions run with DEVOURER_TEARDOWN_POWER_DOWN=0 on purpose: this is the
# historical behaviour that produced the report, and it leaves the chip readable
# afterwards so examples/chipstate can capture the degraded state.
#
#   sudo REGRESS_VBUS_MAP="0bda:8812=3-2.3.4,3;2357:012d=10,2" \
#        tests/severe_form_hunt.sh
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
TXDEMO="$ROOT/build/txdemo"
RXDEMO="$ROOT/build/rxdemo"

DUT_VID=${DUT_VID:-0x0bda} DUT_PID=${DUT_PID:-0x8812}
GND_VID=${GND_VID:-0x2357} GND_PID=${GND_PID:-0x012d}
CH=${CH:-6}
RATE=${RATE:-MCS7/20}
CTRL=${CTRL:-MCS1/20}
SESSIONS=${SESSIONS:-8}     # stressor sessions per arm
PROBES=${PROBES:-3}         # repeats per measurement — must beat sd 1.8
SECS=${SECS:-5}
ARMS=${ARMS:-"baseline band bandwidth spur power failtune"}
OUT=${OUT:-/tmp/devourer-severe-hunt}

mkdir -p "$OUT"
RXLOG="$OUT/ground.rx.jsonl"
: > "$OUT/results.tsv"

RXPID=""
cleanup() {
  [ -n "$RXPID" ] && kill "$RXPID" 2>/dev/null
  pkill -x txdemo 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

cold_cycle() {
  local map=${REGRESS_VBUS_MAP:-} key entry hubport
  [ -n "$map" ] || { echo "REGRESS_VBUS_MAP unset" >&2; return 1; }
  key=$(printf '%s:%s' "${1#0x}" "${2#0x}")
  entry=$(tr ';' '\n' <<<"$map" | grep -i "^$key=" | head -1) || true
  [ -n "${entry:-}" ] || { echo "no VBUS entry for $key" >&2; return 1; }
  hubport=${entry#*=}
  sudo uhubctl -l "${hubport%,*}" -p "${hubport#*,}" -a cycle -d 3 >/dev/null 2>&1
  sleep 12
}

# One measurement = PROBES repeats, reported as mean. Echoes "mean_pct".
measure() {
  local rate="$1" tag="$2" i off0 off1 sent d sum=0 n=0
  for i in $(seq 1 "$PROBES"); do
    off0=$(stat -c%s "$RXLOG")
    sudo env DEVOURER_VID="$DUT_VID" DEVOURER_PID="$DUT_PID" \
      DEVOURER_CHANNEL="$CH" DEVOURER_TX_RATE="$rate" \
      DEVOURER_TX_GAP_US=2000 DEVOURER_TEARDOWN_POWER_DOWN=0 \
      DEVOURER_LOG_LEVEL=warn \
      timeout --signal=TERM $((SECS + 12)) "$TXDEMO" \
      > "$OUT/tx_${tag}_$i.jsonl" 2>/dev/null &
    local p=$!
    sleep $((SECS + 6))
    kill -TERM "$p" 2>/dev/null; wait "$p" 2>/dev/null
    sleep 1
    off1=$(stat -c%s "$RXLOG")
    sent=$(grep -o '"submitted":[0-9]*' "$OUT/tx_${tag}_$i.jsonl" | tail -1 | cut -d: -f2)
    d=$(python3 - "$RXLOG" "$off0" "$off1" <<'EOF'
import sys
sys.path.insert(0, "tests")
from devourer_events import parse_event
f = open(sys.argv[1], "rb"); f.seek(int(sys.argv[2]))
blob = f.read(int(sys.argv[3]) - int(sys.argv[2])).decode(errors="replace")
print(sum(int(ev.get("frames") or 0)
          for ev in (parse_event(l, "rx.energy") for l in blob.splitlines()) if ev))
EOF
)
    if [ "${sent:-0}" -gt 0 ] 2>/dev/null; then
      sum=$(python3 -c "print($sum + 100.0*$d/$sent)"); n=$((n + 1))
    fi
  done
  python3 -c "print('%.1f' % ($sum/$n) if $n else 'nan')"
}

# One stressor session. $1 = arm name. Each arm changes exactly one thing.
stress_session() {
  local arm="$1" e=()
  case "$arm" in
    baseline)   e=(DEVOURER_CHANNEL="$CH" DEVOURER_TX_RATE="$RATE") ;;
    band)       # alternate 2.4 <-> 5 GHz: re-runs IQK, rewrites the LOK
                if [ $((RANDOM_SEQ % 2)) -eq 0 ]; then
                  e=(DEVOURER_CHANNEL=36 DEVOURER_TX_RATE="$RATE")
                else
                  e=(DEVOURER_CHANNEL="$CH" DEVOURER_TX_RATE="$RATE")
                fi ;;
    bandwidth)  if [ $((RANDOM_SEQ % 2)) -eq 0 ]; then
                  e=(DEVOURER_CHANNEL="$CH" DEVOURER_HOP_BW=40 DEVOURER_TX_RATE=MCS7/40)
                else
                  e=(DEVOURER_CHANNEL="$CH" DEVOURER_TX_RATE="$RATE")
                fi ;;
    spur)       # ch13/14 arm the ADC-clock spur workaround (0x8ac/0x8c4)
                if [ $((RANDOM_SEQ % 2)) -eq 0 ]; then
                  e=(DEVOURER_CHANNEL=14 DEVOURER_TX_RATE=MCS4/20)
                else
                  e=(DEVOURER_CHANNEL="$CH" DEVOURER_TX_RATE="$RATE")
                fi ;;
    power)      if [ $((RANDOM_SEQ % 2)) -eq 0 ]; then
                  e=(DEVOURER_CHANNEL="$CH" DEVOURER_TX_RATE="$RATE" DEVOURER_TX_PWR=63)
                else
                  e=(DEVOURER_CHANNEL="$CH" DEVOURER_TX_RATE="$RATE"
                     DEVOURER_TX_PWR_OFFSET_QDB=48)
                fi ;;
    failtune)   # 5 GHz tunes that deliver nothing on this pair — the original
                # session was full of them
                e=(DEVOURER_CHANNEL=149 DEVOURER_TX_RATE="$RATE") ;;
    *) echo "unknown arm $arm" >&2; return 1 ;;
  esac
  sudo env DEVOURER_VID="$DUT_VID" DEVOURER_PID="$DUT_PID" "${e[@]}" \
    DEVOURER_TX_GAP_US=0 DEVOURER_TEARDOWN_POWER_DOWN=0 DEVOURER_LOG_LEVEL=warn \
    timeout --signal=TERM 14 "$TXDEMO" >/dev/null 2>&1
  sleep 1
}

start_ground() {
  [ -n "$RXPID" ] && { kill "$RXPID" 2>/dev/null; wait "$RXPID" 2>/dev/null; }
  DEVOURER_VID="$GND_VID" DEVOURER_PID="$GND_PID" DEVOURER_CHANNEL="$CH" \
  DEVOURER_RX_AGG_SA=canon DEVOURER_RX_ENERGY_MS=500 DEVOURER_LOG_LEVEL=warn \
  "$RXDEMO" >> "$RXLOG" 2>>"$OUT/ground.rx.stderr" &
  RXPID=$!
  sleep 9
  kill -0 "$RXPID" 2>/dev/null || { echo "ground RX died" >&2; exit 1; }
}

cold_cycle "$GND_VID" "$GND_PID" || exit 1
start_ground

echo "severe-form hunt — $SESSIONS stressor sessions/arm, $PROBES probes/measurement"
echo "  a hit is a >=20 point drop at $RATE with $CTRL intact (noise is ~+/-3)"
echo
printf '  %-11s %-9s %-9s %-9s %-9s  %s\n' \
    "arm" "cold" "after" "delta" "ctrl" "verdict"

for ARM in $ARMS; do
  # Recover BOTH ends before every arm, and restart the ground process with
  # them. Cycling only the DUT was a systematic flaw in the first version of
  # this harness: the ground station stayed up across every arm, so anything
  # that decayed on the receive side accumulated through the whole run and
  # showed up as a per-arm delta. It produced a convincing 32-point "hit" that
  # an independent receiver then refuted — the arms' own cold baselines had
  # been sliding (62 -> 32 -> 1.2 -> 1.8) the entire time.
  cold_cycle "$GND_VID" "$GND_PID" || exit 1
  cold_cycle "$DUT_VID" "$DUT_PID" || exit 1
  start_ground
  before=$(measure "$RATE" "${ARM}_cold")
  RANDOM_SEQ=0
  for _ in $(seq 1 "$SESSIONS"); do
    stress_session "$ARM"
    RANDOM_SEQ=$((RANDOM_SEQ + 1))
  done
  after=$(measure "$RATE" "${ARM}_after")
  ctrl=$(measure "$CTRL" "${ARM}_ctrl")
  read -r delta verdict <<<"$(python3 -c "
b,a='$before','$after'
try:
    d=float(a)-float(b)
    print('%+.1f' % d, 'HIT' if d<=-20 else ('mild' if d<=-4 else 'no effect'))
except ValueError:
    print('nan', 'INVALID')")"
  printf '  %-11s %-9s %-9s %-9s %-9s  %s\n' \
      "$ARM" "$before" "$after" "$delta" "$ctrl" "$verdict"
  printf '%s\t%s\t%s\t%s\t%s\n' "$ARM" "$before" "$after" "$ctrl" "$verdict" \
      >> "$OUT/results.tsv"
  # Capture the chip state while it is still powered and still degraded.
  sudo "$ROOT/build/chipstate" --pid "$DUT_PID" --vid "$DUT_VID" \
      > "$OUT/${ARM}_after.canary" 2>&1
done


# --- drift canary ------------------------------------------------------------
# Re-measure the very first arm's cold baseline at the END, both ends recovered
# exactly as they were at the start. Nothing about the run should have changed
# it. If it has, the reference moved underneath every arm and the per-arm deltas
# above are differences between two different rigs, not stressor effects.
#
# This is not hypothetical: the run that "found" a 32-point band-switch collapse
# had its arms' own cold baselines sliding 62 -> 32 -> 1.2 -> 1.8 -> 1.9 -> 0.7,
# because the GROUND STATION's receiver was dying. An independent receiver later
# showed the transmitter untouched, and the vendor driver failed on that ground
# adapter exactly as devourer did — the adapter, not the code. Without this
# check the run reports a confident, entirely wrong answer.
cold_cycle "$GND_VID" "$GND_PID" || exit 1
cold_cycle "$DUT_VID" "$DUT_PID" || exit 1
start_ground
first_arm=$(head -1 "$OUT/results.tsv" | cut -f1)
first_cold=$(head -1 "$OUT/results.tsv" | cut -f2)
final=$(measure "$RATE" drift_canary)
echo
python3 - "$first_arm" "$first_cold" "$final" <<'EOF'
import sys
arm, a, b = sys.argv[1], sys.argv[2], sys.argv[3]
try:
    d = float(b) - float(a)
except ValueError:
    print("  drift canary: INVALID (%s -> %s)" % (a, b)); raise SystemExit(1)
verdict = ("OK — the reference held" if abs(d) <= 4 else
           "!! RUN INVALID: the reference moved %+.1f points during the run, "
           "so every per-arm delta above is suspect. Check the GROUND station "
           "(tests/rx_vendor_ab.sh) before believing any of it." % d)
print("  drift canary: %s cold was %s, is now %s (%+.1f)  %s"
      % (arm, a, b, d, verdict))
raise SystemExit(0 if abs(d) <= 4 else 1)
EOF
canary_rc=$?

echo
echo "per-arm chip dumps: $OUT/*_after.canary"
echo "results: $OUT/results.tsv"
exit $canary_rc
