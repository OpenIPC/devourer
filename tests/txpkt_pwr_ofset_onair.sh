#!/usr/bin/env bash
# MEASURE the per-packet TX-descriptor power lever on-air. Does
# the DUT's per-frame power mechanism actually move radiated power, and by the
# nominal dB the sweep requests? Family mechanisms behind the ONE env knob
# (DEVOURER_TX_PKT_OFSET, examples/tx fan-out):
#   Jaguar2 8822B/8821C — descriptor TXPWR_OFSET 3-bit hardware LUT
#     (0=none 1=-3 2=-7 3=-11 4=+3 5=+6 dB). The original, on-air-confirmed.
#   Jaguar1 8814A       — dword5 [30:28] descriptor field, same LUT expected.
#   Jaguar3 8822C/8822E — TXPWR_OFSET_TYPE bank selector; the step's nominal
#     dB is programmed into a 0x1e70 offset bank (TxPktPwrBanks.h) — slope
#     calibratable via DEVOURER_TXPKT_STEP_QDB (passed through when set).
#
# TX = the DUT at a FIXED rate/power, stepping DEVOURER_TX_PKT_OFSET; ground =
# a second devourer part reporting per-frame RSSI (the same chip-RSSI sensor
# the TX-power slope work uses, since the B210 front end limits on near-field
# frames). If the lever works, ground RSSI tracks the nominal dB per step.
#
# Usage: sudo -v && tests/txpkt_pwr_ofset_onair.sh [ground_pid]
#   TX_PID/TX_VID          DUT identity (default 8822BU T3U 2357:012d)
#   TX_BUS/TX_PORT         USB topology pin when VID:PID is ambiguous (8814A)
#   GROUND_PID/GROUND_VID  RSSI sensor identity (default 8812CU 0bda:c812)
#   CH                     channel (default 36; 8822E DUTs are 5 GHz-only TX)
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${TXPKT_OUT:-/tmp/devourer-txpkt-ofset}"
CH="${CH:-36}"
TX_PID="${TX_PID:-0x012d}" TX_VID="${TX_VID:-0x2357}"
TX_BUS="${TX_BUS:-}" TX_PORT="${TX_PORT:-}"
GROUND_PID="${1:-0xc812}" GROUND_VID="${GROUND_VID:-0x0bda}"
# Extra env for the TX cells (topology pin + J3 slope override when set).
TX_EXTRA=()
[ -n "$TX_BUS" ] && TX_EXTRA+=("DEVOURER_USB_BUS=$TX_BUS")
[ -n "$TX_PORT" ] && TX_EXTRA+=("DEVOURER_USB_PORT=$TX_PORT")
[ -n "${DEVOURER_TXPKT_STEP_QDB:-}" ] && TX_EXTRA+=("DEVOURER_TXPKT_STEP_QDB=$DEVOURER_TXPKT_STEP_QDB")
# LUT steps to sweep, paired with their nominal dB (for the assertion).
STEPS="0 4 5 1 2 3"
declare -A DB=( [0]=0 [4]=3 [5]=6 [1]=-3 [2]=-7 [3]=-11 )
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
for step in $STEPS; do
    t0="$(date +%s.%N)"
    # Fixed rate MCS3, fixed base power, only the per-packet offset varies.
    sudo -n env DEVOURER_PID="$TX_PID" DEVOURER_VID="$TX_VID" DEVOURER_CHANNEL="$CH" \
        DEVOURER_TX_RATE=MCS3 DEVOURER_TX_PWR=40 DEVOURER_TX_PKT_OFSET="$step" \
        DEVOURER_TX_GAP_US=1500 "${TX_EXTRA[@]}" \
        timeout 12 "$ROOT/build/txdemo" >/dev/null 2>&1 || true
    t1="$(date +%s.%N)"
    echo "$step ${DB[$step]} $t0 $t1" >>"$OUT/cells.txt"
    sleep 2
done
sudo -n pkill -x rxdemo 2>/dev/null; wait "$GJ" 2>/dev/null

# --- radiotap path: same effect via a per-packet DBM_TX_POWER field (not the
#     device default) — proves send_packet honours per-frame power. Two cells
#     at the LUT extremes; append to cells.txt with a 'r' tag column.
echo "== ground still up: radiotap DBM_TX_POWER cells ==" >&2
sudo -n env DEVOURER_PID="$GROUND_PID" DEVOURER_VID="$GROUND_VID" \
    DEVOURER_CHANNEL="$CH" DEVOURER_STREAM_OUT=1 \
    stdbuf -oL timeout 90 "$ROOT/build/rxdemo" 2>/dev/null \
    | while IFS= read -r l; do printf '%s %s\n' "$(date +%s.%N)" "$l"; done \
    >>"$OUT/ground.log" &
GJ2=$!
sleep 10
for pair in "0:0" "6:6" "-11:-11"; do
    db="${pair%%:*}"
    t0="$(date +%s.%N)"
    sudo -n env DEVOURER_PID="$TX_PID" DEVOURER_VID="$TX_VID" DEVOURER_CHANNEL="$CH" \
        DEVOURER_TX_RATE=MCS3 DEVOURER_TX_PWR=40 DEVOURER_TX_PKT_PWR_DB="$db" \
        DEVOURER_TX_GAP_US=1500 "${TX_EXTRA[@]}" \
        timeout 12 "$ROOT/build/txdemo" >/dev/null 2>&1 || true
    t1="$(date +%s.%N)"
    echo "r$db $db $t0 $t1" >>"$OUT/cells.txt"
    sleep 2
done
sudo -n pkill -x rxdemo 2>/dev/null; wait "$GJ2" 2>/dev/null

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

desc = []   # (step:int, db, rssi) — device-default path
rtap = []   # (db, rssi) — radiotap path
for line in open(sys.argv[2]):
    label, db, t0, t1 = line.split(); db, t0, t1 = int(db), float(t0), float(t1)
    rssi, n = med(t0, t1)
    if rssi is None:
        print(f"{label} ({db:+d} dB): too few frames ({n})"); continue
    if label.startswith("r"):
        rtap.append((db, rssi))
        print(f"<txpkt-radiotap> db={db:+d} rssi_med={rssi} frames={n}")
    else:
        desc.append((int(label), db, rssi))
        print(f"<txpkt> step={label} nominal_db={db:+d} rssi_med={rssi} frames={n}")

fail = 0
# --- device-default (descriptor LUT) path ---
base = next((r[2] for r in desc if r[0] == 0), None)
if base is None or len(desc) < 4:
    print("RESULT-DESC inconclusive (no step-0 baseline)"); fail = 1
else:
    print(f"\ndescriptor-default baseline (step 0) rssi = {base}")
    moved = correct = total = 0
    for step, db, rssi in desc:
        if step == 0:
            continue
        total += 1
        d = rssi - base
        moved += abs(d) >= 1
        correct += (db > 0 and d > 0) or (db < 0 and d < 0)
        print(f"  step {step} ({db:+3d} dB): rssi {base}->{rssi} = {d:+d} raw")
    if moved == 0:
        print("RESULT-DESC INERT — descriptor TXPWR_OFSET did not move on-air power")
        fail = 1
    elif correct >= total - 1:
        print(f"RESULT-DESC WORKS — {correct}/{total} steps correct direction")
    else:
        print(f"RESULT-DESC PARTIAL — {correct}/{total} correct sign"); fail = 1

# --- radiotap per-packet path ---
rbase = next((r[1] for r in rtap if r[0] == 0), None)
if rbase is None or len(rtap) < 3:
    print("RESULT-RADIOTAP skipped (no cells)")
else:
    print(f"\nradiotap DBM_TX_POWER baseline (0 dB) rssi = {rbase}")
    rcorrect = rtotal = 0
    for db, rssi in rtap:
        if db == 0:
            continue
        rtotal += 1
        d = rssi - rbase
        rcorrect += (db > 0 and d > 0) or (db < 0 and d < 0)
        print(f"  radiotap {db:+3d} dB: rssi {rbase}->{rssi} = {d:+d} raw")
    if rcorrect == rtotal and rtotal >= 2:
        print(f"RESULT-RADIOTAP WORKS — {rcorrect}/{rtotal} per-packet power via "
              f"radiotap DBM_TX_POWER moved on-air power correctly")
    else:
        print(f"RESULT-RADIOTAP FAIL — {rcorrect}/{rtotal} correct"); fail = 1

sys.exit(1 if fail else 0)
PYEOF
