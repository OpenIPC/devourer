#!/usr/bin/env bash
# Issue #110 re-baseline on current master: RTL8814AU 5 GHz TX health in ONE
# cold session — (a) computed TXAGC index (DEVOURER_LOG_TXPWR), (b) CU-ground
# decode (frames/EVM/RSSI), (c) a single SDR duty read (first read only — a
# second back-to-back read can fail to reacquire).
#   sudo tests/au8814_5g_baseline.sh [channel] [rate]
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CH="${1:-36}"
RATE="${2:-MCS7}"
FREQ=$((5000 + 5*CH))e6
OUT=/tmp/au8814-baseline
mkdir -p "$OUT"
cleanup(){ sudo -n pkill -x rxdemo 2>/dev/null; sudo -n pkill -x txdemo 2>/dev/null; true; }
trap cleanup EXIT INT TERM
cleanup; sleep 1

# VBUS cold the 8814 (rail-sag history on its hub branch).
SYS=""
for d in /sys/bus/usb/devices/*/idProduct; do
    [ "$(cat "$d" 2>/dev/null)" = "8813" ] && { SYS=$(basename "$(dirname "$d")"); break; }
done
[ -n "$SYS" ] || { echo "ERROR: 8813 not on USB"; exit 1; }
case "$SYS" in
    *.*) sudo -n uhubctl -l "${SYS%.*}" -p "${SYS##*.}" -a cycle -d 3 >/dev/null 2>&1 || \
             echo "WARN: uhubctl cycle failed";;
    *)   echo "WARN: 8814 on a root port — skipping VBUS cycle";;
esac
sleep 4; lsusb -d 0bda:8813 >/dev/null || { echo "ERROR: 8814 gone after cycle"; exit 1; }

echo "== CU ground up (ch$CH) =="
: >"$OUT/ground.log"
sudo -n env DEVOURER_PID=0xc812 DEVOURER_VID=0x0bda DEVOURER_CHANNEL="$CH" \
    DEVOURER_STREAM_OUT=1 timeout 60 "$ROOT/build/rxdemo" \
    >"$OUT/ground.log" 2>/dev/null &
GJ=$!; sleep 12

echo "== 8814 TX ($RATE ch$CH, max duty) =="
sudo -n env DEVOURER_PID=0x8813 DEVOURER_VID=0x0bda DEVOURER_CHANNEL="$CH" \
    DEVOURER_TX_RATE="$RATE" DEVOURER_TX_GAP_US=0 DEVOURER_LOG_TXPWR=1 \
    timeout 35 "$ROOT/build/txdemo" >/dev/null 2>"$OUT/tx.err" &
TJ=$!; sleep 8

echo "== single SDR duty read =="
sudo -n python3 "$ROOT/tests/sdr_duty.py" --freq "$FREQ" --rate 25e6 --secs 5 \
    --gain 60 --mcs 7 --bw 20 2>&1 | grep -iE "duty=" | head -1
wait "$TJ" 2>/dev/null; sleep 1
cleanup; wait "$GJ" 2>/dev/null

echo "== computed TXAGC (from DEVOURER_LOG_TXPWR) =="
grep -iE "txpwr|power.*index|idx" "$OUT/tx.err" | grep -vE "bulk|ready" | head -8
python3 - "$OUT/ground.log" <<'PYEOF'
import json, statistics as st
n=0; evm=[]; rssi=[]
for line in open(sys_argv1 := __import__('sys').argv[1], errors="replace"):
    if '"ev":"rx.frame"' not in line: continue
    try: ev=json.loads(line)
    except ValueError: continue
    if ev.get("rate")==19:
        n+=1; evm.append(ev["evm"][0]); rssi.append(ev["rssi"][0])
print(f"ground: MCS7 n={n} medEVM={st.median(evm) if evm else None} medRSSI={st.median(rssi) if rssi else None}")
PYEOF
