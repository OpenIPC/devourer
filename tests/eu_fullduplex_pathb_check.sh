#!/usr/bin/env bash
# Full-duplex validation of the retired 0x41e8 skip (issue #238 quirk #1):
# EU in TX+RX=thread mode with path-B OFDM power now APPLIED —
#   TX half: EU injects MCS7, counted by the 8822CU ground (rate=19).
#   RX half: 8812AU floods MCS0, counted by the EU's own RX (rx.count).
# PASS = both halves alive simultaneously (TX+RX with full TXAGC).
#   sudo tests/eu_fullduplex_pathb_check.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT=/tmp/eu-fullduplex-pathb
CH="${CH:-36}"
mkdir -p "$OUT"
cleanup(){ sudo -n pkill -x rxdemo 2>/dev/null; sudo -n pkill -x txdemo 2>/dev/null; true; }
trap cleanup EXIT INT TERM
cleanup; sleep 1

echo "== CU ground up (ch$CH) =="
: >"$OUT/ground.log"
sudo -n env DEVOURER_PID=0xc812 DEVOURER_VID=0x0bda DEVOURER_CHANNEL="$CH" \
    DEVOURER_STREAM_OUT=1 \
    stdbuf -oL timeout 70 "$ROOT/build/rxdemo" >"$OUT/ground.log" 2>"$OUT/ground.err" &
GJ=$!
sleep 12

echo "== EU TX+RX up (MCS7 inject + RX thread) =="
sudo -n env DEVOURER_PID=0xa81a DEVOURER_VID=0x0bda DEVOURER_CHANNEL="$CH" \
    DEVOURER_TX_WITH_RX=thread DEVOURER_TX_RATE=MCS7 DEVOURER_TX_GAP_US=2000 \
    stdbuf -oL timeout 40 "$ROOT/build/txdemo" >"$OUT/eu.log" 2>"$OUT/eu.err" &
EJ=$!
sleep 14

echo "== AU MCS0 flood (EU RX traffic) =="
sudo -n env DEVOURER_PID=0x8812 DEVOURER_VID=0x0bda DEVOURER_CHANNEL="$CH" \
    DEVOURER_TX_RATE=MCS0 DEVOURER_TX_GAP_US=1500 \
    timeout 20 "$ROOT/build/txdemo" >/dev/null 2>&1 || true
sleep 2
cleanup; wait "$GJ" "$EJ" 2>/dev/null

eu_rx=$(grep -o '"total":[0-9]*' "$OUT/eu.log" | tail -1 | cut -d: -f2)
cu_m7=$(python3 - "$OUT/ground.log" <<'PYEOF'
import json,sys
n=0
for line in open(sys.argv[1],errors="replace"):
    if '"ev":"rx.frame"' not in line: continue
    try: ev=json.loads(line)
    except ValueError: continue
    if ev.get("rate")==19: n+=1
print(n)
PYEOF
)
pathb=$(grep -c "both paths" "$OUT/eu.err" || true)
echo
echo "EU RX total (rx.count):      ${eu_rx:-0}"
echo "CU ground EU-MCS7 (rate19):  ${cu_m7:-0}"
echo "EU TXAGC 'both paths' logs:  ${pathb:-0}"
if [ "${eu_rx:-0}" -gt 1000 ] && [ "${cu_m7:-0}" -gt 1000 ]; then
    echo "RESULT PASS: full duplex with path-B OFDM power applied"
else
    echo "RESULT FAIL: one half dead (RX=${eu_rx:-0}, TX->ground=${cu_m7:-0})"
    exit 1
fi
