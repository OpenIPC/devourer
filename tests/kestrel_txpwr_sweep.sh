#!/usr/bin/env bash
# kestrel_txpwr_sweep.sh — TX-power validation for the RTL8852BU.
# Sweeps devourer's flat TX-power override (DEVOURER_TX_PWR -> the CMAC
# power-by-rate/limit tables at 0xD2C0/0xD2EC/0xD33C) and measures the received
# power slope on an independent devourer ground station (RTL8812AU rxdemo,
# per-frame rssi[] on Realtek phystatus). This is the project's validated
# TXAGC-slope method (the B210 saturates at close range — see
# txpwr_offset_onair.sh); at 500 fps the DUT's canonical-SA beacons dominate the
# median rssi in each cell window. Monotonically rising median rssi vs power
# code = the flat power control actually moves TX power.
#
#   sudo tests/kestrel_txpwr_sweep.sh [channel]
set -u
cd "$(dirname "$0")/.."
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }
CH=${1:-6}
TX_ID="35bc:0108"; RX_ID="0bda:8812"
TX_HUB="3-2.3"; TX_PORT="3"
CODES="0x04 0x10 0x28 0x40 0x70"
GLOG="/tmp/kestrel_txpwr_ground.log"; CELLS="/tmp/kestrel_txpwr_cells.txt"
[ -x build/txdemo ] && [ -x build/rxdemo ] || { echo "FAIL: build txdemo+rxdemo"; exit 2; }

sysdir_for() { local vid=${1%%:*} pid=${1##*:} d; for d in /sys/bus/usb/devices/*; do
  [ -f "$d/idVendor" ] || continue
  [ "$(cat "$d/idVendor")" = "$vid" ] && [ "$(cat "$d/idProduct")" = "$pid" ] && { echo "$d"; return; }; done; }
unbind_kernel() { local d i; d=$(sysdir_for "$1"); [ -n "$d" ] && for i in "$d":*; do
  [ -L "$i/driver" ] && echo "$(basename "$i")" > "$(readlink -f "$i/driver")/unbind" 2>/dev/null || true; done; }

GPID=""
cleanup() {
  [ -n "$GPID" ] && kill "$GPID" 2>/dev/null
  pkill -9 -f "build/txdemo" 2>/dev/null || true
  pkill -9 -f "build/rxdemo" 2>/dev/null || true
  local d i; d=$(sysdir_for "$RX_ID")
  [ -n "$d" ] && for i in "$d":*; do [ -d "$i" ] && echo "$(basename "$i")" > /sys/bus/usb/drivers/rtw88_8812au/bind 2>/dev/null || true; done
}
trap cleanup EXIT

echo ">> VBUS-cycling the 8852BU DUT for a clean devourer bring-up"
uhubctl -l "$TX_HUB" -p "$TX_PORT" -a cycle -d 2 >/dev/null 2>&1; sleep 6

# --- ground station: 8812AU rxdemo, epoch-stamped rx.frame stream ---
unbind_kernel "$RX_ID"; sleep 1
: >"$GLOG"; : >"$CELLS"
echo ">> ground rxdemo on the 8812AU (ch$CH)"
env DEVOURER_VID=0x0bda DEVOURER_PID=0x8812 DEVOURER_CHANNEL=$CH \
    DEVOURER_EVENTS=stdout DEVOURER_STREAM_OUT=1 DEVOURER_LOG_LEVEL=warn \
    stdbuf -oL build/rxdemo 2>/dev/null \
  | while IFS= read -r l; do printf '%s %s\n' "$(date +%s.%N)" "$l"; done >>"$GLOG" &
GPID=$!
sleep 6

# --- sweep DUT TX power ---
unbind_kernel "$TX_ID"; sleep 1
for code in $CODES; do
  t0=$(date +%s.%N)
  echo ">> TX cell: DEVOURER_TX_PWR=$code (12s)"
  env DEVOURER_VID=0x35bc DEVOURER_PID=0x0108 DEVOURER_CHANNEL=$CH \
      DEVOURER_TX_GAP_US=2000 DEVOURER_TX_PWR=$code DEVOURER_LOG_LEVEL=warn \
      timeout 12 build/txdemo >/tmp/kestrel_txpwr_cell_$code.log 2>&1 || true
  t1=$(date +%s.%N)
  echo "$code $t0 $t1" >>"$CELLS"
  sleep 1
done
kill "$GPID" 2>/dev/null; GPID=""

# --- fit: median rssi[A] per cell window (skip the first 5s of each cell) ---
python3 - "$GLOG" "$CELLS" <<'PYEOF'
import re, statistics, sys
glog, cells = sys.argv[1], sys.argv[2]
rx = re.compile(r'^([0-9.]+) .*"ev":"rx\.frame".*"rssi":\[(-?\d+),(-?\d+)\]')
frames = []
for ln in open(glog, errors="replace"):
    m = rx.match(ln)
    if m: frames.append((float(m.group(1)), int(m.group(2))))
print(f"total ground rx.frame with rssi: {len(frames)}")
rows = []
for ln in open(cells):
    code, t0, t1 = ln.split()
    t0, t1 = float(t0), float(t1)
    vals = [r for (ts, r) in frames if t0 + 5 <= ts <= t1 - 0.5]
    med = statistics.median(vals) if vals else None
    rows.append((code, med, len(vals)))
    print(f"  code {code}: median rssi[A] = {med}  (n={len(vals)})")
meds = [(int(c, 16), m) for c, m, n in rows if m is not None]
if len(meds) >= 3:
    lo, hi = meds[0][1], meds[-1][1]
    mono = all(meds[i][1] <= meds[i+1][1] + 1 for i in range(len(meds)-1))
    print("=" * 50)
    print(f"RESULT: rssi {lo} -> {hi} dB across codes {hex(meds[0][0])}..{hex(meds[-1][0])}, "
          f"{'MONOTONIC-RISING' if mono and hi > lo else 'FLAT/NON-MONOTONIC'}")
    print("PASS: flat TX-power control moves received power." if mono and hi > lo
          else "INCONCLUSIVE: no clear power slope (check geometry / saturation).")
else:
    print("RESULT: too few cells with frames — ground caught too little.")
PYEOF
