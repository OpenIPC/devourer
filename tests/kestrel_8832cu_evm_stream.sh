#!/usr/bin/env bash
# kestrel_8832cu_evm_stream.sh — TX-EVM for the RTL8832CU using HT *data* frames.
# txdemo only sends beacons (mgmt @ legacy — the monitor won't decode them at HT
# and legacy carries no EVM). streamtx sends canonical-SA data frames at
# DEVOURER_TX_RATE, so an HT rate (MCS7 = 64-QAM) gives EVM-bearing frames. The
# monitor is the 8822CU (c812) — a validated per-frame EVM reporter (rx.frame
# evm[0]). Attenuate (TXPWR) so the bench-adjacent monitor isn't saturated.
#
#   sudo tests/kestrel_8832cu_evm_stream.sh <label> [channel] [rate]
# Appends "<label> <n> <medEVM> <p10> <p90> <medRSSI>" to /tmp/kestrel-evm/summary
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT=/tmp/kestrel-evm; mkdir -p "$OUT"; chmod 777 "$OUT" 2>/dev/null || true
LABEL=${1:?need a label}
CH=${2:-36}; RATE=${3:-MCS7}; TXPWR=${TXPWR:-10}
MON_ID="0bda:c812"        # 8822CU monitor (per-frame EVM reporter)
TX_ID="35bc:0101"         # 8832CU DUT
GLOG="$OUT/gs-$LABEL.log"

sysdir_for() {
  local vid=${1%%:*} pid=${1##*:} d
  for d in /sys/bus/usb/devices/*; do
    [ -f "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor")" = "$vid" ] && [ "$(cat "$d/idProduct")" = "$pid" ] \
      && { echo "$d"; return; }
  done
}
unbind_kernel() {
  local d i; d=$(sysdir_for "$1")
  [ -n "$d" ] && for i in "$d":*; do
    [ -L "$i/driver" ] && echo "$(basename "$i")" > "$(readlink -f "$i/driver")/unbind" 2>/dev/null || true
  done
}
GJ=""; TXPID=""; FEEDPID=""
cleanup() {
  [ -n "$TXPID" ] && kill "$TXPID" 2>/dev/null || true
  [ -n "$FEEDPID" ] && kill "$FEEDPID" 2>/dev/null || true
  [ -n "$GJ" ] && kill "$GJ" 2>/dev/null || true
  pkill -9 -x -f "build/streamtx" 2>/dev/null || true
  pkill -9 -x -f "build/rxdemo" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
cleanup; sleep 1

sysdir_for "$MON_ID" >/dev/null || { echo "FAIL: monitor $MON_ID not on bus"; exit 2; }
sysdir_for "$TX_ID"  >/dev/null || { echo "FAIL: DUT $TX_ID not on bus"; exit 2; }
unbind_kernel "$MON_ID"; unbind_kernel "$TX_ID"; sleep 1

# 1) Monitor: c812, ch$CH, stream rx.frame (evm) timestamped.
: >"$GLOG"
env DEVOURER_VID=0x0bda DEVOURER_PID=0xc812 DEVOURER_CHANNEL="$CH" \
    DEVOURER_STREAM_OUT=1 DEVOURER_LOG_LEVEL=warn \
    stdbuf -oL timeout 42 "$ROOT/build/rxdemo" 2>"$OUT/gs-$LABEL.err" \
  | while IFS= read -r line; do printf '%s %s\n' "$(date +%s.%N)" "$line"; done \
  >>"$GLOG" &
GJ=$!
sleep 10

# 2) DUT: streamtx flooding HT-$RATE data frames (random 200B PSDUs) for ~16 s.
t0=$(date +%s.%N)
( python3 -c '
import os,sys,struct,time
end=time.time()+16
while time.time()<end:
    b=os.urandom(200)
    sys.stdout.buffer.write(struct.pack("<I",len(b))+b)
    sys.stdout.buffer.flush()
    time.sleep(0.0015)
' | env DEVOURER_VID=0x35bc DEVOURER_PID=0x0101 DEVOURER_CHANNEL="$CH" \
      DEVOURER_TX_RATE="$RATE" DEVOURER_TX_PWR="$TXPWR" DEVOURER_LOG_LEVEL=warn \
      "$ROOT/build/streamtx" >/dev/null 2>"$OUT/txs-$LABEL.err" ) &
TXPID=$!
wait "$TXPID" 2>/dev/null; TXPID=""
t1=$(date +%s.%N)
sleep 1
kill "$GJ" 2>/dev/null; wait "$GJ" 2>/dev/null; GJ=""

# 3) Median EVM over the flood window for the target HT rate.
python3 - "$GLOG" "$LABEL" "$RATE" "$t0" "$t1" <<'PYEOF'
import json, statistics, sys
glog, label, rate, t0, t1 = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4]), float(sys.argv[5])
WANT = {"MCS7": 19, "MCS0": 12, "MCS4": 16}.get(rate, 19)
ev = []
for line in open(glog, errors="replace"):
    if '"ev":"rx.frame"' not in line: continue
    ts,_,js = line.partition(" ")
    try: e = json.loads(js)
    except ValueError: continue
    if e.get("rate") != WANT: continue
    if not (t0+2 <= float(ts) <= t1-0.3): continue
    ev.append((e["evm"][0], e["rssi"][0]))
n = len(ev)
if not n:
    print(f"{label:<12} n=0  (no {rate} frames)")
    open("/tmp/kestrel-evm/summary","a").write(f"{label} 0 - - - -\n"); sys.exit(0)
evm = sorted(x[0] for x in ev); rssi = sorted(x[1] for x in ev)
med = statistics.median(evm); p10 = evm[n//10]; p90 = evm[(n*9)//10]
mr = statistics.median(rssi)
print(f"{label:<12} n={n:<5} medEVM={med:<6} p10={p10:<5} p90={p90:<5} medRSSI={mr}")
open("/tmp/kestrel-evm/summary","a").write(f"{label} {n} {med} {p10} {p90} {mr}\n")
PYEOF
