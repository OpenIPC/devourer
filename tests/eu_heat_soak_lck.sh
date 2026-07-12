#!/usr/bin/env bash
# Heat-soak the 8812EU at max TX duty and observe the thermal/LCK runtime
# (issue #238 loop: validates the LCK synthesizer re-lock port live).
#   - EU: MCS7, GAP_US=0 (max duty) for SOAK_SECS.
#   - Ground (8822CU): samples EVM continuously; we compare the first vs last
#     2 minutes (EVM drift without LCK would grow with chip heating).
#   - EU stderr: pwr_track thermal/swing logs + "LCK re-lock" events.
#   sudo tests/eu_heat_soak_lck.sh [soak_secs]
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT=/tmp/eu-heat-soak
CH="${CH:-36}"
SOAK="${1:-1320}"
mkdir -p "$OUT"
cleanup(){ sudo -n pkill -x rxdemo 2>/dev/null; sudo -n pkill -x txdemo 2>/dev/null; true; }
trap cleanup EXIT INT TERM
cleanup; sleep 1

: >"$OUT/ground.log"
sudo -n env DEVOURER_PID=0xc812 DEVOURER_VID=0x0bda DEVOURER_CHANNEL="$CH" \
    DEVOURER_STREAM_OUT=1 \
    stdbuf -oL timeout $((SOAK+40)) "$ROOT/build/rxdemo" 2>"$OUT/ground.err" \
    | while IFS= read -r line; do printf '%s %s\n' "$(date +%s.%N)" "$line"; done \
    >>"$OUT/ground.log" &
GJ=$!
sleep 12

echo "== EU MCS7 max-duty soak for ${SOAK}s =="
sudo -n env DEVOURER_PID=0xa81a DEVOURER_VID=0x0bda DEVOURER_CHANNEL="$CH" \
    DEVOURER_TX_RATE=MCS7 DEVOURER_TX_GAP_US=0 \
    timeout "$SOAK" "$ROOT/build/txdemo" >/dev/null 2>"$OUT/eu.err" || true
sleep 1
cleanup; wait "$GJ" 2>/dev/null

echo "== thermal / LCK events =="
grep -E "pwr_track|LCK" "$OUT/eu.err" | tail -20
python3 - "$OUT/ground.log" "$SOAK" <<'PYEOF'
import json, statistics, sys
soak = float(sys.argv[2])
rows = []
for line in open(sys.argv[1], errors="replace"):
    if '"ev":"rx.frame"' not in line: continue
    ts,_,js = line.partition(" ")
    try: ev = json.loads(js)
    except ValueError: continue
    if ev.get("rate") != 19: continue
    rows.append((float(ts), ev["evm"][0]))
if not rows:
    print("RESULT FAIL: ground caught no EU MCS7 frames"); sys.exit(1)
t0 = rows[0][0]
head = [e for t,e in rows if t - t0 < 120]
tail = [e for t,e in rows if t - t0 > soak - 120]
mh = statistics.median(head) if head else 0
mt = statistics.median(tail) if tail else 0
print(f"frames={len(rows)}  EVM first2min={mh}  last2min={mt}  drift={mt-mh:+}")
print("RESULT PASS" if tail and abs(mt-mh) <= 4 else
      "RESULT WARN: EVM drifted >4 dB across the soak (LCK/thermal check)")
PYEOF
