#!/usr/bin/env bash
# Reproduce the warm-session TX degradation: repeated devourer init/teardown
# cycles on one adapter progressively destroy high-order-constellation TX while
# leaving the low rates untouched, and only a VBUS power cycle recovers it.
#
# This matters beyond the bench. A field deployment (FPV link, drone) cannot
# power-cycle its adapter, so a long-running or restart-prone session silently
# loses its top rates with nothing in any log to say so.
#
# The signature is what makes this diagnosable: it is NOT a wedge. The adapter
# keeps enumerating, keeps accepting frames, keeps reporting every frame
# submitted, and keeps delivering the robust rates at full rate. Only the dense
# constellations stop decoding. That points at analog TX quality (calibration
# state — IQK/LCK/DPK — or power/thermal tracking) rather than the datapath,
# which would take the low rates down with it.
#
# Method: hold ONE ground receiver up for the whole run so the reference never
# moves, then alternate {probe} and {N warm devourer sessions} on the DUT and
# watch the probe decay. Two controls separate the candidate causes:
#   - idle with no power cycle  -> if this recovers it, the cause is thermal
#   - VBUS power cycle          -> known to recover it
#
#   sudo REGRESS_VBUS_MAP="0bda:8812=3-2.3.4,3;2357:012d=10,2" \
#        tests/warm_tx_degradation_repro.sh
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
TXDEMO="$ROOT/build/txdemo"
RXDEMO="$ROOT/build/rxdemo"

DUT_VID=${DUT_VID:-0x0bda} DUT_PID=${DUT_PID:-0x8812}
GND_VID=${GND_VID:-0x2357} GND_PID=${GND_PID:-0x012d}
CH=${CH:-6}
# The rate under test must be one the link carries comfortably when cold, or a
# decay curve cannot be distinguished from a marginal link. MCS7 measured ~60%
# cold on this pair. CTRL_RATE is the robust control that must keep working
# throughout — it is what shows the adapter is still transmitting at all.
RATE=${RATE:-MCS7/20}
CTRL_RATE=${CTRL_RATE:-MCS1/20}
WARM_PER_ROUND=${WARM_PER_ROUND:-3}
ROUNDS=${ROUNDS:-4}
SECS=${SECS:-5}
IDLE_SECS=${IDLE_SECS:-60}
OUT=${OUT:-/tmp/devourer-warm-degradation}

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

cold_cycle() { # $1=vid $2=pid
  local map=${REGRESS_VBUS_MAP:-} key entry hubport
  [ -n "$map" ] || { echo "REGRESS_VBUS_MAP unset — cannot cold-cycle" >&2; return 1; }
  key=$(printf '%s:%s' "${1#0x}" "${2#0x}")
  entry=$(tr ';' '\n' <<<"$map" | grep -i "^$key=" | head -1) || true
  [ -n "${entry:-}" ] || { echo "no VBUS map entry for $key" >&2; return 1; }
  hubport=${entry#*=}
  sudo uhubctl -l "${hubport%,*}" -p "${hubport#*,}" -a cycle -d 3 >/dev/null 2>&1
  sleep 12
}

# One devourer TX session at $1, measured against the standing ground receiver.
# Echoes "delivered/sent thermal_raw rssi evm".
#
# The three extra channels are what separate the candidate mechanisms, and the
# whole point of running this before changing any library code:
#   thermal_raw  the chip's own RF 0x42 meter. The adapter is never powered
#                down between sessions, so it never cools; a monotonic rise
#                that falls after a VBUS cycle means heat, not latched state.
#   rssi         falling RSSI means the transmitter is losing POWER; RSSI flat
#                while the high rates die means it is losing signal QUALITY
#                (EVM / phase noise) at unchanged power. Different fixes.
#   evm          the receiver's own quality estimate, as a cross-check on rssi.
probe() {
  local rate="$1" tag="$2"
  local off0 off1 sent delivered stats
  off0=$(stat -c%s "$RXLOG")
  sudo env DEVOURER_VID="$DUT_VID" DEVOURER_PID="$DUT_PID" \
    DEVOURER_CHANNEL="$CH" DEVOURER_TX_RATE="$rate" \
    DEVOURER_TX_GAP_US=2000 DEVOURER_THERMAL_POLL_MS="${THERM_MS:-1000}" \
    DEVOURER_LOG_LEVEL=warn \
    timeout --signal=TERM $((SECS + 12)) "$TXDEMO" \
    > "$OUT/tx_$tag.jsonl" 2>"$OUT/tx_$tag.err" &
  local p=$!
  sleep $((SECS + 6))
  kill -TERM "$p" 2>/dev/null; wait "$p" 2>/dev/null
  sleep 1
  off1=$(stat -c%s "$RXLOG")
  sent=$(grep -o '"submitted":[0-9]*' "$OUT/tx_$tag.jsonl" | tail -1 | cut -d: -f2)
  # Last thermal reading of the session — the warmest point, i.e. the one that
  # would matter if heat is the mechanism.
  local therm
  therm=$(grep -o '"raw":[0-9]*' "$OUT/tx_$tag.jsonl" | tail -1 | cut -d: -f2)
  stats=$(python3 - "$RXLOG" "$off0" "$off1" <<'EOF'
import sys
sys.path.insert(0, "tests")
from devourer_events import parse_event
f = open(sys.argv[1], "rb"); f.seek(int(sys.argv[2]))
blob = f.read(int(sys.argv[3]) - int(sys.argv[2])).decode(errors="replace")
frames = 0
rssi, evm = [], []
for line in blob.splitlines():
    ev = parse_event(line, "rx.energy")
    if not ev:
        continue
    n = int(ev.get("frames") or 0)
    frames += n
    # Average only over windows that actually carried frames — an empty window
    # reports rssi/evm as 0 and would drag the mean toward zero.
    if n:
        if ev.get("rssi_mean"):
            rssi.append(ev["rssi_mean"])
        if ev.get("evm_mean"):
            evm.append(ev["evm_mean"])
mean = lambda a: (sum(a) / len(a)) if a else 0
print("%d %.0f %.0f" % (frames, mean(rssi), mean(evm)))
EOF
)
  read -r delivered rssi evm <<<"$stats"
  printf '%s/%s %s %s %s' "${delivered:-0}" "${sent:-0}" "${therm:-0}" \
      "${rssi:-0}" "${evm:-0}"
}

# A warm session: full devourer bring-up + TX + teardown, no power cycle. This
# is the thing under test — exactly what a restart-prone deployment does.
#
# WARM_PWR / WARM_GAP let a run stress the session rather than just repeat it,
# to find what aggravates the decay. Plain repetition costs a few points; the
# catastrophic form (top rates at zero) was first seen after a long spell of
# max-power and max-duty sessions, so those are the first suspects.
warm_session() {
  local extra=()
  [ -n "${WARM_PWR:-}" ] && extra=(DEVOURER_TX_PWR="$WARM_PWR")
  sudo env DEVOURER_VID="$DUT_VID" DEVOURER_PID="$DUT_PID" \
    DEVOURER_CHANNEL="$CH" DEVOURER_TX_RATE="$RATE" "${extra[@]}" \
    DEVOURER_TX_GAP_US="${WARM_GAP:-2000}" DEVOURER_LOG_LEVEL=warn \
    timeout --signal=TERM 14 "$TXDEMO" >/dev/null 2>&1
  sleep 1
}

# $1=label, $2/$3 = probe output for the test and control rate
# ("delivered/sent thermal rssi evm").
report() {
  local label="$1" t="$2" c="$3"
  local t_ds t_therm t_rssi t_evm c_ds
  read -r t_ds t_therm t_rssi t_evm <<<"$t"
  read -r c_ds _ _ _ <<<"$c"
  local pct cpct
  pct=$(python3 -c "
d,s='$t_ds'.split('/')
print('%.1f%%' % (100*int(d)/int(s)) if int(s) else 'n/a')")
  cpct=$(python3 -c "
d,s='$c_ds'.split('/')
print('%.1f%%' % (100*int(d)/int(s)) if int(s) else 'n/a')")
  printf '  %-32s %-8s %-8s  therm=%-3s rssi=%-3s evm=%s\n' \
      "$label" "$pct" "$cpct" "$t_therm" "$t_rssi" "$t_evm"
  printf '%s\t%s\t%s\n' "$label" "$t" "$c" >> "$OUT/results.tsv"
}

echo "warm-session TX degradation repro"
echo "  DUT $DUT_VID:$DUT_PID   ground $GND_VID:$GND_PID   ch$CH"
echo "  test rate $RATE, control rate $CTRL_RATE, ${WARM_PER_ROUND} warm sessions/round"
echo

cold_cycle "$GND_VID" "$GND_PID" || exit 1
DEVOURER_VID="$GND_VID" DEVOURER_PID="$GND_PID" DEVOURER_CHANNEL="$CH" \
DEVOURER_RX_AGG_SA=canon DEVOURER_RX_ENERGY_MS=500 DEVOURER_LOG_LEVEL=warn \
"$RXDEMO" > "$RXLOG" 2>"$OUT/ground.rx.stderr" &
RXPID=$!
sleep 9
kill -0 "$RXPID" 2>/dev/null || { echo "ground RX died" >&2; exit 1; }

printf '  %-32s %-8s %-8s  %s\n' "stage" "$RATE" "$CTRL_RATE" "chip/link sensors"
cold_cycle "$DUT_VID" "$DUT_PID" || exit 1
report "cold baseline" "$(probe "$RATE" cold)" "$(probe "$CTRL_RATE" cold_ctrl)"

warm=0
for r in $(seq 1 "$ROUNDS"); do
  for _ in $(seq 1 "$WARM_PER_ROUND"); do warm_session; warm=$((warm + 1)); done
  report "after $warm warm sessions" \
      "$(probe "$RATE" "warm$warm")" "$(probe "$CTRL_RATE" "warm${warm}_ctrl")"
done

# Control 1: idle, no power cycle. Recovery here would mean thermal, and would
# also mean a field deployment could recover by simply pausing.
echo "  (idling ${IDLE_SECS}s with no power cycle ...)"
sleep "$IDLE_SECS"
report "after ${IDLE_SECS}s idle (no power cycle)" \
    "$(probe "$RATE" idle)" "$(probe "$CTRL_RATE" idle_ctrl)"

# Control 2: the known recovery.
cold_cycle "$DUT_VID" "$DUT_PID" || exit 1
report "after VBUS cold cycle" \
    "$(probe "$RATE" recovered)" "$(probe "$CTRL_RATE" recovered_ctrl)"

echo
echo "results: $OUT/results.tsv"
