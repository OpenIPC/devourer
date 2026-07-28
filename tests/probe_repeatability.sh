#!/usr/bin/env bash
# What is the run-to-run scatter of a single delivery probe, with NOTHING
# changed between runs?
#
# This is the measurement that should have come first. Every conclusion drawn
# from the warm-degradation work — the decay curve, the recovery, the thermal
# correlation — is a difference of a few points between probes. None of it means
# anything unless a probe repeated under identical conditions lands in a tighter
# band than the effect being claimed.
#
# So: one ground receiver up for the whole run, one DUT, N identical probes
# back to back, no power cycling, no config change, no re-tuning. Report the
# spread. Any claimed effect smaller than this band is not a finding.
#
#   sudo REGRESS_VBUS_MAP="0bda:8812=3-2.3.4,3;2357:012d=10,2" \
#        tests/probe_repeatability.sh
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
TXDEMO="$ROOT/build/txdemo"
RXDEMO="$ROOT/build/rxdemo"

DUT_VID=${DUT_VID:-0x0bda} DUT_PID=${DUT_PID:-0x8812}
GND_VID=${GND_VID:-0x2357} GND_PID=${GND_PID:-0x012d}
CH=${CH:-6}
RATES=${RATES:-"MCS7/20 MCS1/20"}
REPS=${REPS:-10}
SECS=${SECS:-6}
OUT=${OUT:-/tmp/devourer-probe-repeat}

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
  [ -n "$map" ] || return 0
  key=$(printf '%s:%s' "${1#0x}" "${2#0x}")
  entry=$(tr ';' '\n' <<<"$map" | grep -i "^$key=" | head -1) || return 0
  hubport=${entry#*=}
  sudo uhubctl -l "${hubport%,*}" -p "${hubport#*,}" -a cycle -d 3 >/dev/null 2>&1
  sleep 12
}

probe() { # $1=rate $2=tag -> "delivered sent thermal rssi"
  local rate="$1" tag="$2" off0 off1 sent therm
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
  therm=$(grep -o '"raw":[0-9]*' "$OUT/tx_$tag.jsonl" | tail -1 | cut -d: -f2)
  python3 - "$RXLOG" "$off0" "$off1" "${sent:-0}" "${therm:-0}" <<'EOF'
import sys
sys.path.insert(0, "tests")
from devourer_events import parse_event
f = open(sys.argv[1], "rb"); f.seek(int(sys.argv[2]))
blob = f.read(int(sys.argv[3]) - int(sys.argv[2])).decode(errors="replace")
frames, rssi = 0, []
for line in blob.splitlines():
    ev = parse_event(line, "rx.energy")
    if not ev:
        continue
    n = int(ev.get("frames") or 0)
    frames += n
    if n and ev.get("rssi_mean"):
        rssi.append(ev["rssi_mean"])
print("%d %s %s %.0f" % (frames, sys.argv[4], sys.argv[5],
                         sum(rssi) / len(rssi) if rssi else 0))
EOF
}

cold_cycle "$GND_VID" "$GND_PID"
DEVOURER_VID="$GND_VID" DEVOURER_PID="$GND_PID" DEVOURER_CHANNEL="$CH" \
DEVOURER_RX_AGG_SA=canon DEVOURER_RX_ENERGY_MS=500 DEVOURER_LOG_LEVEL=warn \
"$RXDEMO" > "$RXLOG" 2>"$OUT/ground.rx.stderr" &
RXPID=$!
sleep 9
kill -0 "$RXPID" 2>/dev/null || { echo "ground RX died" >&2; exit 1; }

cold_cycle "$DUT_VID" "$DUT_PID"

for RATE in $RATES; do
  echo
  echo "== $RATE, $REPS identical probes, nothing changed between them"
  printf '  %-5s %-9s %-8s %-6s\n' "rep" "delivery" "thermal" "rssi"
  for i in $(seq 1 "$REPS"); do
    read -r d s th rs <<<"$(probe "$RATE" "${RATE//\//-}_$i")"
    pct=$(python3 -c "print('%.1f' % (100*$d/$s) if $s else 0)")
    printf '  %-5s %-9s %-8s %-6s\n' "$i" "$pct%" "$th" "$rs"
    printf '%s\t%s\t%s\t%s\t%s\n' "$RATE" "$i" "$pct" "$th" "$rs" >> "$OUT/results.tsv"
  done
  python3 - "$OUT/results.tsv" "$RATE" <<'EOF'
import sys, statistics as st
rate = sys.argv[2]
v = [float(l.split("\t")[2]) for l in open(sys.argv[1])
     if l.split("\t")[0] == rate]
if len(v) > 1:
    sd = st.stdev(v)
    print("  -> mean %.1f%%  sd %.1f  range %.1f-%.1f  spread %.1f points"
          % (st.mean(v), sd, min(v), max(v), max(v) - min(v)))
    print("     an effect smaller than ~%.0f points (2 sd) is not detectable "
          "with a single probe" % (2 * sd))
EOF
done
