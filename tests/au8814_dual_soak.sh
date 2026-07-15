#!/usr/bin/env bash
# Issue #110 disambiguation soak: 8814AU max-duty 5 GHz TX, measured
# CONTINUOUSLY by the 8822CU ground and PERIODICALLY by the B210 — when a
# low SDR reading appears, the per-window ground tallies say whether the
# transmitter actually collapsed (both drop) or the SDR read is bad (ground
# stays strong). Ground = truth anchor; SDR = the instrument under suspicion.
#   sudo tests/au8814_dual_soak.sh [rounds] [round_secs]
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ROUNDS="${1:-10}"
RSECS="${2:-45}"
OUT=/tmp/au8814-dualsoak
mkdir -p "$OUT"
cleanup(){ sudo -n pkill -x rxdemo 2>/dev/null; sudo -n pkill -x txdemo 2>/dev/null; true; }
trap cleanup EXIT INT TERM
cleanup; sleep 1

TOTAL=$((ROUNDS*RSECS + 40))
: >"$OUT/ground.log"
sudo -n env DEVOURER_PID=0xc812 DEVOURER_VID=0x0bda DEVOURER_CHANNEL=36 \
    DEVOURER_STREAM_OUT=1 timeout "$TOTAL" "$ROOT/build/rxdemo" \
    | while IFS= read -r line; do printf '%s %s\n' "$(date +%s)" "$line"; done \
    >>"$OUT/ground.log" 2>/dev/null &
GJ=$!; sleep 12
sudo -n env DEVOURER_PID=0x8813 DEVOURER_VID=0x0bda DEVOURER_CHANNEL=36 \
    DEVOURER_TX_RATE=MCS7 DEVOURER_TX_GAP_US=0 \
    timeout $((TOTAL-15)) "$ROOT/build/txdemo" >/dev/null 2>&1 &
TJ=$!; sleep 6

: >"$OUT/rounds.txt"
for i in $(seq 1 "$ROUNDS"); do
    t0=$(date +%s)
    duty=$(sudo -n python3 "$ROOT/tests/sdr_duty.py" --freq 5180e6 --rate 25e6 \
        --secs 4 --gain 60 --mcs 7 --bw 20 2>&1 | grep -oE "duty=[0-9.]+" | head -1)
    t1=$(date +%s)
    echo "$i $t0 $t1 ${duty:-duty=NA}" >>"$OUT/rounds.txt"
    sleep $((RSECS-8))
done
cleanup; wait "$GJ" "$TJ" 2>/dev/null

python3 - "$OUT/ground.log" "$OUT/rounds.txt" <<'PYEOF'
import json, statistics, sys
frames=[]
for line in open(sys.argv[1], errors="replace"):
    if '"ev":"rx.frame"' not in line: continue
    ts,_,js=line.partition(" ")
    try: ev=json.loads(js)
    except ValueError: continue
    if ev.get("rate")==19:
        frames.append((int(ts), ev["rssi"][0]))
print(f"{'round':>5} {'sdr':>10} {'ground fps':>10} {'medRSSI':>8}")
for line in open(sys.argv[2]):
    i, t0, t1, duty = line.split()
    t0, t1 = int(t0), int(t1)
    sel=[r for t,r in frames if t0<=t<=t1]
    fps=len(sel)/max(1,(t1-t0))
    med=statistics.median(sel) if sel else None
    print(f"{i:>5} {duty:>10} {fps:>10.0f} {str(med):>8}")
PYEOF
