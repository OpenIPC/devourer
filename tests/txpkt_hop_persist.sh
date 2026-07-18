#!/usr/bin/env bash
# PERSISTENCE check for the Jaguar3 per-packet power banks: the
# lever contract says power knobs stick across channel switches. One txdemo
# session hops 36<->40 via FastRetune while a +6 dB per-packet default is
# active; a second identical session runs at 0 dB. The ground (fixed ch36)
# only decodes the ch36 dwells — if a retune dropped the 0x1e70 banks, the
# offset session's ch36 median would collapse to the baseline instead of
# holding ~+6 dB above the 0 dB session.
#
# Usage: sudo -v && tests/txpkt_hop_persist.sh [ground_pid]
#   TX_PID/TX_VID (default 8812CU 0bda:c812), GROUND_* (default 8822BU).
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${TXPKT_OUT:-/tmp/devourer-txpkt-hop-persist}"
TX_PID="${TX_PID:-0xc812}" TX_VID="${TX_VID:-0x0bda}"
GROUND_PID="${1:-0x012d}" GROUND_VID="${GROUND_VID:-0x2357}"
mkdir -p "$OUT"

cleanup() { pkill -x txdemo 2>/dev/null||true; pkill -x rxdemo 2>/dev/null||true; true; }
trap cleanup EXIT INT TERM
plugged() { lsusb -d "$(printf '%04x:%04x' "$2" "$1")" >/dev/null 2>&1; }
plugged "$TX_PID" "$TX_VID" || { echo "SKIP: TX DUT $TX_VID:$TX_PID not plugged"; exit 0; }
plugged "$GROUND_PID" "$GROUND_VID" || { echo "SKIP: ground $GROUND_PID not plugged"; exit 0; }

cmake --build "$ROOT/build" -j --target txdemo rxdemo >/dev/null || exit 1

echo "== ground RX ($GROUND_PID) fixed on ch36 =="
: >"$OUT/ground.log"
sudo -n env DEVOURER_PID="$GROUND_PID" DEVOURER_VID="$GROUND_VID" \
    DEVOURER_CHANNEL=36 DEVOURER_STREAM_OUT=1 \
    stdbuf -oL timeout 120 "$ROOT/build/rxdemo" 2>"$OUT/ground.err" \
    | while IFS= read -r l; do printf '%s %s\n' "$(date +%s.%N)" "$l"; done \
    >>"$OUT/ground.log" &
GJ=$!
sleep 10

: >"$OUT/cells.txt"
for qdb in 24 0; do
    t0="$(date +%s.%N)"
    # 36<->40 FastRetune hops, ~200 frames per dwell, per-packet default $qdb.
    sudo -n env DEVOURER_PID="$TX_PID" DEVOURER_VID="$TX_VID" DEVOURER_CHANNEL=36 \
        DEVOURER_TX_RATE=MCS3 DEVOURER_TX_PWR=40 DEVOURER_TX_PKT_PWR_QDB="$qdb" \
        DEVOURER_HOP_CHANNELS=36,40 DEVOURER_HOP_DWELL_FRAMES=200 DEVOURER_HOP_FAST=1 \
        DEVOURER_TX_GAP_US=1500 \
        timeout 20 "$ROOT/build/txdemo" >/dev/null 2>&1 || true
    t1="$(date +%s.%N)"
    echo "$qdb $t0 $t1" >>"$OUT/cells.txt"
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
med = {}
for line in open(sys.argv[2]):
    qdb, t0, t1 = line.split(); qdb, t0, t1 = int(qdb), float(t0), float(t1)
    win = [r for (t, r) in frames if t0 + 4 <= t <= t1 - 1]
    if len(win) < 15:
        print(f"qdb={qdb}: too few ch36 frames ({len(win)})"); sys.exit(1)
    med[qdb] = int(statistics.median(win))
    print(f"<hop-persist> qdb={qdb:+d} ch36_rssi_med={med[qdb]} frames={len(win)}")
d = med[24] - med[0]
print(f"\n+6 dB session vs 0 dB session across FastRetune hops: {d:+d} raw")
if d >= 3:
    print("RESULT-PERSIST WORKS — banks survive FastRetune (offset held on-air)")
    sys.exit(0)
print("RESULT-PERSIST FAIL — offset lost across retunes")
sys.exit(1)
PYEOF
