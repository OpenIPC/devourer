#!/usr/bin/env bash
# Does chip TEMPERATURE cause the high-rate delivery loss, or does chip STATE?
#
# The obvious experiments cannot tell those apart. A VBUS cycle resets state AND
# cools the part. A card-disable at teardown resets state AND lets the part cool.
# Anything that recovers delivery by removing power is consistent with both
# stories, so none of it is proof.
#
# This is the discriminator: measure delivery WITHIN ONE UNINTERRUPTED SESSION.
# One bring-up, one channel set, one calibration, one TX config, never torn down
# and never re-inited. Across the run the only thing that changes is how hot the
# die is. If delivery decays as the thermal meter climbs, temperature is doing
# it, because nothing else moved.
#
# The control leg is the same session at a robust rate. Heating is identical;
# if the decay were something time-dependent but not thermal (a leaking counter,
# a drifting receiver, ambient), it would drag the control down too. A decay
# that appears ONLY at the dense constellation is the thermal signature.
#
#   sudo REGRESS_VBUS_MAP="0bda:8812=3-2.3.4,3;2357:012d=10,2" \
#        tests/thermal_causation_probe.sh
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
TXDEMO="$ROOT/build/txdemo"
RXDEMO="$ROOT/build/rxdemo"

DUT_VID=${DUT_VID:-0x0bda} DUT_PID=${DUT_PID:-0x8812}
GND_VID=${GND_VID:-0x2357} GND_PID=${GND_PID:-0x012d}
CH=${CH:-6}
RATES=${RATES:-"MCS7/20 MCS1/20"}   # first = test (near the cliff), second = control
SECS=${SECS:-240}                   # one continuous session per rate
BIN=${BIN:-20}                      # seconds per reporting bin
OUT=${OUT:-/tmp/devourer-thermal-causation}

mkdir -p "$OUT"
RXLOG="$OUT/ground.rx.jsonl"

RXPID=""
cleanup() {
  [ -n "$RXPID" ] && kill "$RXPID" 2>/dev/null
  pkill -x txdemo 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

cold_cycle() { # $1=vid $2=pid
  local map=${REGRESS_VBUS_MAP:-} key entry hubport
  [ -n "$map" ] || { echo "REGRESS_VBUS_MAP unset" >&2; return 1; }
  key=$(printf '%s:%s' "${1#0x}" "${2#0x}")
  entry=$(tr ';' '\n' <<<"$map" | grep -i "^$key=" | head -1) || true
  [ -n "${entry:-}" ] || { echo "no VBUS entry for $key" >&2; return 1; }
  hubport=${entry#*=}
  sudo uhubctl -l "${hubport%,*}" -p "${hubport#*,}" -a cycle -d 3 >/dev/null 2>&1
  sleep 12
}

cold_cycle "$GND_VID" "$GND_PID" || exit 1
DEVOURER_VID="$GND_VID" DEVOURER_PID="$GND_PID" DEVOURER_CHANNEL="$CH" \
DEVOURER_RX_AGG_SA=canon DEVOURER_RX_ENERGY_MS=1000 DEVOURER_LOG_LEVEL=warn \
"$RXDEMO" > "$RXLOG" 2>"$OUT/ground.rx.stderr" &
RXPID=$!
sleep 9
kill -0 "$RXPID" 2>/dev/null || { echo "ground RX died" >&2; exit 1; }

for RATE in $RATES; do
  TAG="${RATE//\//-}"
  echo
  echo "== single uninterrupted session: $RATE, ${SECS}s, max duty, ch$CH"
  echo "   (one bring-up, no re-init, no power cycle — only temperature moves)"
  cold_cycle "$DUT_VID" "$DUT_PID" || exit 1
  OFF0=$(stat -c%s "$RXLOG")
  sudo env DEVOURER_VID="$DUT_VID" DEVOURER_PID="$DUT_PID" \
    DEVOURER_CHANNEL="$CH" DEVOURER_TX_RATE="$RATE" \
    DEVOURER_TX_GAP_US=0 DEVOURER_TX_PAYLOAD_BYTES=1024 \
    DEVOURER_THERMAL_POLL_MS=1000 DEVOURER_LOG_LEVEL=warn \
    timeout --signal=TERM $((SECS + 15)) "$TXDEMO" \
    > "$OUT/tx_$TAG.jsonl" 2>"$OUT/tx_$TAG.err" &
  TXP=$!
  sleep $((SECS + 8))
  kill -TERM "$TXP" 2>/dev/null; wait "$TXP" 2>/dev/null
  sleep 1
  OFF1=$(stat -c%s "$RXLOG")

  python3 - "$RXLOG" "$OFF0" "$OFF1" "$OUT/tx_$TAG.jsonl" "$BIN" "$RATE" <<'EOF'
import sys
sys.path.insert(0, "tests")
from devourer_events import parse_event

rxlog, off0, off1, txlog, binsec, rate = sys.argv[1:7]
binsec = int(binsec)

f = open(rxlog, "rb"); f.seek(int(off0))
blob = f.read(int(off1) - int(off0)).decode(errors="replace")

# rx.energy carries a monotonic ms timestamp; bin delivered frames by it.
bins, t0 = {}, None
for line in blob.splitlines():
    ev = parse_event(line, "rx.energy")
    if not ev or ev.get("t") is None:
        continue
    t = int(ev["t"])
    if t0 is None:
        t0 = t
    bins.setdefault((t - t0) // (binsec * 1000), [0, 0])
    b = bins[(t - t0) // (binsec * 1000)]
    b[0] += int(ev.get("frames") or 0)
    b[1] += 1

# Thermal samples, binned the same way off the TX session's own clock.
th, n = {}, 0
for line in open(txlog, errors="replace"):
    ev = parse_event(line, "thermal")
    if not ev:
        continue
    th.setdefault(n // binsec, []).append(int(ev.get("raw") or 0))
    n += 1

print("  %-10s %10s %10s   %s" % ("t(s)", "frames/s", "thermal", ""))
ks = sorted(k for k in bins if bins[k][1] >= max(1, binsec // 2))
base = None
for k in ks:
    fr, wins = bins[k]
    rateps = fr / (wins * 1.0)          # rx.energy window = 1000 ms
    t = th.get(k, [])
    tm = sum(t) / len(t) if t else 0
    if base is None and rateps > 0:
        base = rateps
    rel = (rateps / base * 100) if base else 0
    bar = "#" * int(rel / 4)
    print("  %-10d %10.0f %10.0f   %5.1f%% %s" % (k * binsec, rateps, tm, rel, bar))
if base and ks:
    last = bins[ks[-1]]
    lastps = last[0] / last[1]
    print("  -> %s: %.0f -> %.0f frames/s over %ds (%.1f%% of start)"
          % (rate, base, lastps, ks[-1] * binsec, lastps / base * 100))
EOF
done
