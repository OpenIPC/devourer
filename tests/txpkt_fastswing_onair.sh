#!/usr/bin/env bash
# MEASURE the Jaguar1 fast BB-swing TX-power lever on-air
# (RtlJaguarDevice::FastSetTxPowerOffsetQdb): does the TxScale digital scaler
# (0xc1c/0xe1c[31:21], table steps of 0.5 dB) move radiated power by the
# requested amount? This is the 8812A/8821A compensating lever — global,
# per-burst, NOT per-packet (their descriptor has no per-frame power field);
# the 8814A also has it (4 paths) on top of its descriptor LUT.
#
# TX = the DUT at a FIXED rate/power, one DEVOURER_TX_FAST_PWR_QDB per cell;
# ground = a second devourer part reporting per-frame RSSI. Cells sweep
# {0, +2, -2, -4, -8} dB (qdb 0/+8/-8/-16/-32); the +2 ceiling is the vendor
# thermal cap the lever clamps to.
#
# Usage: sudo -v && tests/txpkt_fastswing_onair.sh [ground_pid]
#   TX_PID/TX_VID          DUT identity (default 8812AU 0bda:8812)
#   GROUND_PID/GROUND_VID  RSSI sensor identity (default 8822BU 2357:012d)
#   CH                     channel (default 36)
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${FASTSWING_OUT:-/tmp/devourer-txpkt-fastswing}"
CH="${CH:-36}"
TX_PID="${TX_PID:-0x8812}" TX_VID="${TX_VID:-0x0bda}"
GROUND_PID="${1:-0x012d}" GROUND_VID="${GROUND_VID:-0x2357}"
# qdb cells paired with their nominal dB (for the assertion).
QDBS="0 8 -8 -16 -32"
declare -A DB=( [0]=0 [8]=2 [-8]=-2 [-16]=-4 [-32]=-8 )
mkdir -p "$OUT"

cleanup() { pkill -x txdemo 2>/dev/null||true; pkill -x rxdemo 2>/dev/null||true; true; }
trap cleanup EXIT INT TERM
plugged() { lsusb -d "$(printf '%04x:%04x' "$2" "$1")" >/dev/null 2>&1; }
plugged "$TX_PID" "$TX_VID" || { echo "SKIP: TX DUT $TX_VID:$TX_PID not plugged"; exit 0; }
plugged "$GROUND_PID" "$GROUND_VID" || { echo "SKIP: ground $GROUND_PID not plugged"; exit 0; }

echo "== building =="
cmake --build "$ROOT/build" -j --target txdemo rxdemo >/dev/null || exit 1

echo "== ground RX ($GROUND_PID) on ch$CH =="
: >"$OUT/ground.log"
sudo -n env DEVOURER_PID="$GROUND_PID" DEVOURER_VID="$GROUND_VID" \
    DEVOURER_CHANNEL="$CH" DEVOURER_STREAM_OUT=1 \
    stdbuf -oL timeout 300 "$ROOT/build/rxdemo" 2>"$OUT/ground.err" \
    | while IFS= read -r l; do printf '%s %s\n' "$(date +%s.%N)" "$l"; done \
    >>"$OUT/ground.log" &
GJ=$!
sleep 12

: >"$OUT/cells.txt"
for qdb in $QDBS; do
    t0="$(date +%s.%N)"
    # Fixed rate MCS3, fixed base power, only the fast swing offset varies.
    sudo -n env DEVOURER_PID="$TX_PID" DEVOURER_VID="$TX_VID" DEVOURER_CHANNEL="$CH" \
        DEVOURER_TX_RATE=MCS3 DEVOURER_TX_PWR=40 DEVOURER_TX_FAST_PWR_QDB="$qdb" \
        DEVOURER_TX_GAP_US=1500 \
        timeout 12 "$ROOT/build/txdemo" >/dev/null 2>&1 || true
    t1="$(date +%s.%N)"
    echo "q$qdb ${DB[$qdb]} $t0 $t1" >>"$OUT/cells.txt"
    sleep 2
done
sudo -n pkill -x rxdemo 2>/dev/null; wait "$GJ" 2>/dev/null

python3 - "$OUT/ground.log" "$OUT/cells.txt" <<'PYEOF'
import re, statistics, sys
frames = []
rx = re.compile(r'^([0-9.]+) .*"ev":"rx\.frame".*"rssi":\[(-?\d+),')
for line in open(sys.argv[1], errors="replace"):
    m = rx.match(line)
    if m:
        frames.append((float(m.group(1)), int(m.group(2))))
def med(t0, t1):
    win = [r for (t, r) in frames if t0 + 6 <= t <= t1 - 1]
    return (int(statistics.median(win)), len(win)) if len(win) >= 15 else (None, len(win))

cells = []
for line in open(sys.argv[2]):
    label, db, t0, t1 = line.split(); db, t0, t1 = int(db), float(t0), float(t1)
    rssi, n = med(t0, t1)
    if rssi is None:
        print(f"{label} ({db:+d} dB): too few frames ({n})"); continue
    cells.append((db, rssi))
    print(f"<fastswing> nominal_db={db:+d} rssi_med={rssi} frames={n}")

base = next((r for (d, r) in cells if d == 0), None)
if base is None or len(cells) < 4:
    print("RESULT-FASTSWING inconclusive (no 0 dB baseline)"); sys.exit(1)
print(f"\nfast-swing baseline (0 dB) rssi = {base}")
moved = correct = total = 0
for db, rssi in cells:
    if db == 0:
        continue
    total += 1
    d = rssi - base
    moved += abs(d) >= 1
    correct += (db > 0 and d > 0) or (db < 0 and d < 0)
    print(f"  {db:+3d} dB: rssi {base}->{rssi} = {d:+d} raw")
if moved == 0:
    print("RESULT-FASTSWING INERT — BB-swing writes did not move on-air power")
    sys.exit(1)
if correct >= total - 1:
    print(f"RESULT-FASTSWING WORKS — {correct}/{total} cells correct direction")
    sys.exit(0)
print(f"RESULT-FASTSWING PARTIAL — {correct}/{total} correct sign"); sys.exit(1)
PYEOF
