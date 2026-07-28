#!/usr/bin/env bash
# Is the high-rate delivery loss in the TRANSMITTER or the RECEIVER?
#
# Every measurement in this investigation so far reported "delivery", which is a
# property of the whole link. When it fell, the transmitter got the blame — the
# DUT was the thing being stressed and the thing being power-cycled. The ground
# station was set up once and left running for the entire run.
#
# Then a vendor-driver A/B judged by a THIRD receiver (an AR9271 sniffer) showed
# the very same "dead" transmitter delivering MCS7 at 62%, minutes after a run
# had reported 0%. The only difference was which receiver was listening.
#
# So this isolates the receiver, by recovering it in increasing steps while the
# transmitter is left completely alone:
#
#   1. fresh        - ground rxdemo just started
#   2. aged         - after the ground has been running a while under load
#   3. rx restart   - kill and restart the rxdemo PROCESS only (no power cycle):
#                     separates accumulated software/driver state from chip state
#   4. rx VBUS      - power-cycle the GROUND adapter (chip state)
#   5. tx VBUS      - power-cycle the TRANSMITTER, ground left as-is: the control.
#                     If this is what recovers delivery, the receiver is innocent.
#
# The robust control rate is measured at every step. A receiver losing high-order
# demodulation shows the test rate collapsing while the control stays pinned.
#
#   sudo REGRESS_VBUS_MAP="0bda:8812=3-2.3.4,3;2357:012d=10,2" \
#        tests/ground_rx_degradation.sh
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
AGE_SESSIONS=${AGE_SESSIONS:-8}   # load applied while the ground station ages
PROBES=${PROBES:-3}
SECS=${SECS:-5}
OUT=${OUT:-/tmp/devourer-ground-rx}

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

start_ground() {
  [ -n "$RXPID" ] && { kill "$RXPID" 2>/dev/null; wait "$RXPID" 2>/dev/null; }
  DEVOURER_VID="$GND_VID" DEVOURER_PID="$GND_PID" DEVOURER_CHANNEL="$CH" \
  DEVOURER_RX_AGG_SA=canon DEVOURER_RX_ENERGY_MS=500 DEVOURER_LOG_LEVEL=warn \
  "$RXDEMO" >> "$RXLOG" 2>>"$OUT/ground.rx.stderr" &
  RXPID=$!
  sleep 9
  kill -0 "$RXPID" 2>/dev/null || { echo "ground RX died" >&2; exit 1; }
}

measure() { # $1=rate $2=tag -> mean percent over PROBES
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

report() { printf '  %-24s %-9s %-9s\n' "$1" "$2" "$3"
           printf '%s\t%s\t%s\n' "$1" "$2" "$3" >> "$OUT/results.tsv"; }

# Both ends cold, so step 1 is a genuine fresh baseline.
cold_cycle "$GND_VID" "$GND_PID" || exit 1
cold_cycle "$DUT_VID" "$DUT_PID" || exit 1
start_ground

echo "is the loss in the transmitter or the receiver?"
echo "  transmitter is left untouched until the last step"
echo
printf '  %-24s %-9s %-9s\n' "step" "$RATE" "$CTRL"
report "1 fresh ground" "$(measure "$RATE" fresh)" "$(measure "$CTRL" fresh_c)"

echo "  (aging the ground station under $AGE_SESSIONS load sessions ...)"
for _ in $(seq 1 "$AGE_SESSIONS"); do
  sudo env DEVOURER_VID="$DUT_VID" DEVOURER_PID="$DUT_PID" \
    DEVOURER_CHANNEL="$CH" DEVOURER_TX_RATE="$RATE" DEVOURER_TX_GAP_US=0 \
    DEVOURER_TEARDOWN_POWER_DOWN=0 DEVOURER_LOG_LEVEL=warn \
    timeout --signal=TERM 14 "$TXDEMO" >/dev/null 2>&1
done
report "2 aged ground" "$(measure "$RATE" aged)" "$(measure "$CTRL" aged_c)"

start_ground   # process restart only — no power cycle
report "3 rx process restart" "$(measure "$RATE" rxproc)" "$(measure "$CTRL" rxproc_c)"

cold_cycle "$GND_VID" "$GND_PID" || exit 1
start_ground
report "4 rx VBUS cycle" "$(measure "$RATE" rxvbus)" "$(measure "$CTRL" rxvbus_c)"

cold_cycle "$DUT_VID" "$DUT_PID" || exit 1
report "5 tx VBUS cycle (control)" "$(measure "$RATE" txvbus)" "$(measure "$CTRL" txvbus_c)"

echo
echo "read: recovery at step 3 or 4 => the RECEIVER was the problem;"
echo "      recovery only at step 5 => the transmitter was."
echo "results: $OUT/results.tsv"
