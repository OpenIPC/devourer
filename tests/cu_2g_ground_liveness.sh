#!/usr/bin/env bash
# 8822CU 2.4 GHz RX liveness series (issue #238 loop): N fresh rxdemo
# bring-ups on ch6, each judged by an 8812AU MCS0 burst (ambient traffic is
# an unreliable liveness signal on quiet channels). Historically the CU
# ground was sporadically deaf on ch6 across bring-ups; the IGI-toggle port
# (now on both Jaguar3 variants) targets that stale-RF-mode mechanism.
#   sudo tests/cu_2g_ground_liveness.sh [reps]
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT=/tmp/cu-2g-liveness
CH="${CH:-6}"
REPS="${1:-5}"
mkdir -p "$OUT"
cleanup(){ sudo -n pkill -x rxdemo 2>/dev/null; sudo -n pkill -x txdemo 2>/dev/null; true; }
trap cleanup EXIT INT TERM
cleanup; sleep 1

alive=0
for i in $(seq 1 "$REPS"); do
    : >"$OUT/g$i.log"
    sudo -n env DEVOURER_PID=0xc812 DEVOURER_VID=0x0bda DEVOURER_CHANNEL="$CH" \
        DEVOURER_STREAM_OUT=1 \
        timeout 30 "$ROOT/build/rxdemo" >"$OUT/g$i.log" 2>"$OUT/g$i.err" &
    GJ=$!
    sleep 12
    sudo -n env DEVOURER_PID=0x8812 DEVOURER_VID=0x0bda DEVOURER_CHANNEL="$CH" \
        DEVOURER_TX_RATE=MCS0 DEVOURER_TX_GAP_US=1500 \
        timeout 10 "$ROOT/build/txdemo" >/dev/null 2>&1 || true
    sleep 1
    sudo -n pkill -x rxdemo 2>/dev/null; wait "$GJ" 2>/dev/null
    n=$(grep -c '"ev":"rx.frame"' "$OUT/g$i.log" || true); n=${n:-0}
    ok=$([ "$n" -gt 500 ] && echo ALIVE || echo DEAF)
    [ "$ok" = ALIVE ] && alive=$((alive+1))
    echo "bring-up $i: rx.frame=$n $ok"
    sleep 2
done
echo "== $alive/$REPS ground bring-ups alive on ch$CH =="
[ "$alive" -eq "$REPS" ] && echo "RESULT PASS" || { echo "RESULT FAIL"; exit 1; }
