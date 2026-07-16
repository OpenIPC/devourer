#!/usr/bin/env bash
# kestrel_8832cu_evm.sh — TX-EVM measurement for the RTL8832CU (C8852C,
# 35bc:0101) to validate the halrf TX-PA calibrations (IQK/DPK/TSSI). A monitor
# adapter (8812AU, 0bda:8812) decodes the 8832CU's HT-MCS7 flood on ch36 and
# reports per-frame EVM (rx.frame event, evm[0]); we take the median over the
# flood window. Compare two builds (e.g. DPK-on vs IQK-only) WITHOUT moving the
# adapters — relative EVM is what matters, so geometry/power must stay fixed.
#
#   sudo tests/kestrel_8832cu_evm.sh <label> [channel] [rate]
# Appends "<label> <n> <medEVM> <p10> <p90> <medRSSI>" to /tmp/kestrel-evm/summary.
# TX_ID=35bc:0108 selects the 8852BU as the DUT instead (same monitor/method).
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT=/tmp/kestrel-evm; mkdir -p "$OUT"
LABEL=${1:?need a label, e.g. dpk-on}
CH=${2:-36}; RATE=${3:-MCS7}
MON_ID="0bda:8812"          # 8812AU monitor (well-validated EVM reporter)
TX_ID="${TX_ID:-35bc:0101}" # DUT (default 8832CU; env-overridable)
GLOG="$OUT/ground-$LABEL.log"

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
GJ=""; TXPID=""
cleanup() {
  [ -n "$TXPID" ] && kill "$TXPID" 2>/dev/null || true
  [ -n "$GJ" ] && kill "$GJ" 2>/dev/null || true
  pkill -9 -x -f "build/txdemo" 2>/dev/null || true
  pkill -9 -x -f "build/rxdemo" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
cleanup; sleep 1

sysdir_for "$MON_ID" >/dev/null || { echo "FAIL: monitor $MON_ID not on bus"; exit 2; }
sysdir_for "$TX_ID"  >/dev/null || { echo "FAIL: DUT $TX_ID not on bus"; exit 2; }
unbind_kernel "$MON_ID"; unbind_kernel "$TX_ID"; sleep 1

# 1) Ground monitor: 8812AU, ch$CH, stream rx.frame (evm) timestamped.
: >"$GLOG"
env DEVOURER_VID=0x0bda DEVOURER_PID=0x8812 DEVOURER_CHANNEL="$CH" \
    DEVOURER_STREAM_OUT=1 DEVOURER_LOG_LEVEL=warn \
    stdbuf -oL timeout 40 "$ROOT/build/rxdemo" 2>"$OUT/ground-$LABEL.err" \
  | while IFS= read -r line; do printf '%s %s\n' "$(date +%s.%N)" "$line"; done \
  >>"$GLOG" &
GJ=$!
sleep 10   # monitor settle

# 2) DUT flood: 8832CU HT-$RATE on ch$CH for ~16 s. Attenuate to TXPWR dBm so
# the bench-adjacent monitor lands in its linear range (full 20 dBm saturates
# it -> AGC can't lock -> 0 decode). Overridable via env TXPWR (default 6 dBm).
TXPWR=${TXPWR:-6}
t0=$(date +%s.%N)
env DEVOURER_VID=0x${TX_ID%%:*} DEVOURER_PID=0x${TX_ID##*:} DEVOURER_CHANNEL="$CH" \
    DEVOURER_TX_RATE="$RATE" DEVOURER_TX_GAP_US=1500 DEVOURER_TX_PWR="$TXPWR" \
    DEVOURER_LOG_LEVEL=warn \
    timeout 16 "$ROOT/build/txdemo" >/dev/null 2>"$OUT/tx-$LABEL.err" &
TXPID=$!
wait "$TXPID" 2>/dev/null; TXPID=""
t1=$(date +%s.%N)
sleep 1
kill "$GJ" 2>/dev/null; wait "$GJ" 2>/dev/null; GJ=""

# 3) Median EVM over the flood window for the target rate.
python3 - "$GLOG" "$LABEL" "$RATE" "$t0" "$t1" <<'PYEOF'
import json, statistics, sys
glog, label, rate, t0, t1 = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4]), float(sys.argv[5])
WANT = {"MCS7": 19, "MCS0": 12, "6M": 4}.get(rate, 19)
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
    print(f"{label:<10} n=0  (no {rate} frames decoded in window)")
    open("/tmp/kestrel-evm/summary","a").write(f"{label} 0 - - - -\n"); sys.exit(0)
evm = sorted(x[0] for x in ev); rssi = sorted(x[1] for x in ev)
med = statistics.median(evm); p10 = evm[n//10]; p90 = evm[(n*9)//10]
mr = statistics.median(rssi)
print(f"{label:<10} n={n:<5} medEVM={med:<6} p10={p10:<5} p90={p90:<5} medRSSI={mr}")
open("/tmp/kestrel-evm/summary","a").write(f"{label} {n} {med} {p10} {p90} {mr}\n")
PYEOF
