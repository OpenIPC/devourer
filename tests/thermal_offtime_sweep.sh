#!/usr/bin/env bash
# Cooling, or state reset? Sweep how LONG the chip stays powered down.
#
# Every recovery observed so far removes power, and removing power does two
# things at once: it resets chip state and it lets the die cool. No experiment
# that simply power-cycles can tell those apart, which is why "it is thermal"
# was not proven by any of them.
#
# This separates them. Degrade the part, then power it down for N seconds and
# probe immediately on the way back up. The state reset is BIT-IDENTICAL for
# every N — same VBUS cycle, same enumeration, same bring-up, same calibration.
# The only thing that varies is how long the die had to cool.
#
#   delivery rises with N        -> cooling is doing the work (thermal)
#   delivery flat across N       -> the reset is doing the work (state)
#
# Off-times are swept in a deliberately non-monotonic order so a slow drift in
# the ambient channel cannot masquerade as the trend.
#
#   sudo REGRESS_VBUS_MAP="0bda:8812=3-2.3.4,3;2357:012d=10,2" \
#        tests/thermal_offtime_sweep.sh
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
HEAT_SECS=${HEAT_SECS:-90}          # max-duty soak before each off-time
OFF_TIMES=${OFF_TIMES:-"5 60 15 120 30"}   # non-monotonic on purpose
SECS=${SECS:-6}
OUT=${OUT:-/tmp/devourer-offtime-sweep}

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

hubport_for() { # $1=vid $2=pid -> "hub port"
  local map=${REGRESS_VBUS_MAP:-} key entry
  key=$(printf '%s:%s' "${1#0x}" "${2#0x}")
  entry=$(tr ';' '\n' <<<"$map" | grep -i "^$key=" | head -1) || return 1
  local hp=${entry#*=}
  printf '%s %s' "${hp%,*}" "${hp#*,}"
}

read -r DUT_HUB DUT_PORT <<<"$(hubport_for "$DUT_VID" "$DUT_PID")" || {
  echo "no VBUS map entry for DUT" >&2; exit 1; }

# Power the DUT off, hold for $1 seconds, power on. The hold is the variable
# under test; everything else about the cycle is identical every time.
power_off_for() {
  sudo uhubctl -l "$DUT_HUB" -p "$DUT_PORT" -a off >/dev/null 2>&1
  sleep "$1"
  sudo uhubctl -l "$DUT_HUB" -p "$DUT_PORT" -a on >/dev/null 2>&1
  sleep 8   # enumeration; identical for every arm
}

probe() { # $1=rate $2=tag -> "delivered/sent thermal"
  local rate="$1" tag="$2" off0 off1 sent delivered therm
  off0=$(stat -c%s "$RXLOG")
  sudo env DEVOURER_VID="$DUT_VID" DEVOURER_PID="$DUT_PID" \
    DEVOURER_CHANNEL="$CH" DEVOURER_TX_RATE="$rate" \
    DEVOURER_TX_GAP_US=2000 DEVOURER_THERMAL_POLL_MS=500 \
    DEVOURER_LOG_LEVEL=warn \
    timeout --signal=TERM $((SECS + 12)) "$TXDEMO" \
    > "$OUT/tx_$tag.jsonl" 2>"$OUT/tx_$tag.err" &
  local p=$!
  sleep $((SECS + 6))
  kill -TERM "$p" 2>/dev/null; wait "$p" 2>/dev/null
  sleep 1
  off1=$(stat -c%s "$RXLOG")
  sent=$(grep -o '"submitted":[0-9]*' "$OUT/tx_$tag.jsonl" | tail -1 | cut -d: -f2)
  # FIRST thermal sample of the session — the coldest point, i.e. what the die
  # actually cooled to during the off-time, before this probe reheats it.
  therm=$(grep -o '"raw":[0-9]*' "$OUT/tx_$tag.jsonl" | head -1 | cut -d: -f2)
  delivered=$(python3 - "$RXLOG" "$off0" "$off1" <<'EOF'
import sys
sys.path.insert(0, "tests")
from devourer_events import parse_event
f = open(sys.argv[1], "rb"); f.seek(int(sys.argv[2]))
blob = f.read(int(sys.argv[3]) - int(sys.argv[2])).decode(errors="replace")
print(sum(int(ev.get("frames") or 0)
          for ev in (parse_event(l, "rx.energy") for l in blob.splitlines()) if ev))
EOF
)
  printf '%s/%s %s' "${delivered:-0}" "${sent:-0}" "${therm:-0}"
}

heat() {
  sudo env DEVOURER_VID="$DUT_VID" DEVOURER_PID="$DUT_PID" \
    DEVOURER_CHANNEL="$CH" DEVOURER_TX_RATE="$RATE" DEVOURER_TX_PWR=63 \
    DEVOURER_TX_GAP_US=0 DEVOURER_TX_PAYLOAD_BYTES=1024 \
    DEVOURER_LOG_LEVEL=warn \
    timeout --signal=TERM $((HEAT_SECS + 10)) "$TXDEMO" >/dev/null 2>&1 &
  local p=$!
  sleep "$HEAT_SECS"
  kill -TERM "$p" 2>/dev/null; wait "$p" 2>/dev/null
}

# Ground station up once for the whole sweep — the reference must not move.
sudo uhubctl -l "$(hubport_for "$GND_VID" "$GND_PID" | cut -d' ' -f1)" \
     -p "$(hubport_for "$GND_VID" "$GND_PID" | cut -d' ' -f2)" \
     -a cycle -d 3 >/dev/null 2>&1
sleep 12
DEVOURER_VID="$GND_VID" DEVOURER_PID="$GND_PID" DEVOURER_CHANNEL="$CH" \
DEVOURER_RX_AGG_SA=canon DEVOURER_RX_ENERGY_MS=500 DEVOURER_LOG_LEVEL=warn \
"$RXDEMO" > "$RXLOG" 2>"$OUT/ground.rx.stderr" &
RXPID=$!
sleep 9
kill -0 "$RXPID" 2>/dev/null || { echo "ground RX died" >&2; exit 1; }

echo "off-time sweep — identical reset every arm, only cooling time varies"
echo "  DUT $DUT_VID:$DUT_PID  heat ${HEAT_SECS}s max-duty, then power off for N"
echo
printf '  %-10s %-10s %-10s %-8s\n' "off(s)" "$RATE" "$CTRL" "thermal"
for N in $OFF_TIMES; do
  heat
  power_off_for "$N"
  t=$(probe "$RATE" "off${N}")
  c=$(probe "$CTRL" "off${N}_ctrl")
  read -r t_ds t_th <<<"$t"
  read -r c_ds _ <<<"$c"
  pct=$(python3 -c "d,s='$t_ds'.split('/'); print('%.1f%%'%(100*int(d)/int(s)) if int(s) else 'n/a')")
  cpct=$(python3 -c "d,s='$c_ds'.split('/'); print('%.1f%%'%(100*int(d)/int(s)) if int(s) else 'n/a')")
  printf '  %-10s %-10s %-10s %-8s\n' "$N" "$pct" "$cpct" "$t_th"
  printf '%s\t%s\t%s\n' "$N" "$t" "$c" >> "$OUT/results.tsv"
done

echo
echo "read: delivery rising with off-time => cooling (thermal);"
echo "      delivery flat across off-time => the reset, not the cooling."
