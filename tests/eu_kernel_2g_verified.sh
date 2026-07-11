#!/usr/bin/env bash
# Kernel ch6 TX ground truth with an IN-SESSION ground control (issue #238
# quirk #3): one CU ground bring-up hears (1) an 8812AU control burst, then
# (2) the vendor-kernel 8812EU injecting MCS0 — both on ch6. If (1) decodes
# and (2) doesn't, the kernel's own 2.4 GHz TX on this module is undecodable
# too (module property, not a devourer gap).
#   sudo tests/eu_kernel_2g_verified.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
KMOD=rtl88x2eu_ohd
CH=6
OUT=/tmp/eu-k2g-verified
mkdir -p "$OUT"
cleanup(){
    sudo -n pkill -x rxdemo 2>/dev/null || true
    sudo -n pkill -x txdemo 2>/dev/null || true
    sudo -n pkill -9 -f kernel_tx_inject 2>/dev/null || true
    sudo -n rmmod "$KMOD" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
sudo -n pkill -x rxdemo 2>/dev/null; sudo -n pkill -x txdemo 2>/dev/null; sleep 1

# Kernel EU up FIRST (VBUS cold; the CU ground stays untouched after).
SYS=""
for d in /sys/bus/usb/devices/*/idProduct; do
    [ "$(cat "$d" 2>/dev/null)" = "a81a" ] && { SYS=$(basename "$(dirname "$d")"); break; }
done
[ -n "$SYS" ] || { echo "ERROR: a81a not on USB"; exit 1; }
sudo -n rmmod "$KMOD" 2>/dev/null
sudo -n uhubctl -l "${SYS%.*}" -p "${SYS##*.}" -a cycle -d 3 >/dev/null 2>&1 || true
sleep 4
sudo -n insmod "$ROOT/reference/rtl88x2eu/${KMOD}.ko" 2>/dev/null || true
sleep 6
IF=""
for d in /sys/class/net/*; do
    drv=$(basename "$(readlink -f "$d/device/driver" 2>/dev/null)" 2>/dev/null)
    [ "$drv" = "$KMOD" ] && { IF=$(basename "$d"); break; }
done
[ -n "$IF" ] || { echo "ERROR: no $KMOD netdev"; exit 1; }
sudo -n ip link set "$IF" down
sudo -n iw dev "$IF" set monitor none
sudo -n ip link set "$IF" up
sudo -n iw dev "$IF" set channel "$CH"
sudo -n iw dev "$IF" set txpower fixed 500 2>/dev/null || true
sleep 2

: >"$OUT/ground.log"
sudo -n env DEVOURER_PID=0xc812 DEVOURER_VID=0x0bda DEVOURER_CHANNEL="$CH" \
    DEVOURER_STREAM_OUT=1 \
    stdbuf -oL timeout 120 "$ROOT/build/rxdemo" 2>"$OUT/ground.err" \
    | while IFS= read -r line; do printf '%s %s\n' "$(date +%s.%N)" "$line"; done \
    >>"$OUT/ground.log" &
GJ=$!
sleep 12

echo "== phase 1: AU control burst (MCS0, ch$CH) =="
t0=$(date +%s.%N)
sudo -n env DEVOURER_PID=0x8812 DEVOURER_VID=0x0bda DEVOURER_CHANNEL="$CH" \
    DEVOURER_TX_RATE=MCS0 DEVOURER_TX_GAP_US=1500 \
    timeout 10 "$ROOT/build/txdemo" >/dev/null 2>&1 || true
t1=$(date +%s.%N)
sleep 2
echo "== phase 2: kernel EU inject (MCS0, ch$CH) =="
t2=$(date +%s.%N)
sudo -n python3 "$ROOT/tests/kernel_tx_inject.py" "$IF" 0 148 10 \
    >"$OUT/ktx.log" 2>&1 || true
t3=$(date +%s.%N)
sleep 1
sudo -n pkill -x rxdemo 2>/dev/null; wait "$GJ" 2>/dev/null

python3 - "$OUT/ground.log" "$t0" "$t1" "$t2" "$t3" <<'PYEOF'
import json, statistics, sys
t0,t1,t2,t3 = map(float, sys.argv[2:6])
au=[]; eu=[]
for line in open(sys.argv[1], errors="replace"):
    if '"ev":"rx.frame"' not in line: continue
    ts,_,js=line.partition(" ")
    try: ev=json.loads(js)
    except ValueError: continue
    t=float(ts)
    if ev.get("rate")!=12: continue
    e=ev.get("evm",[0])[0]
    if t0<=t<=t1: au.append(e)
    elif t2<=t<=t3: eu.append(e)
def s(v): return f"n={len(v)} medEVM={int(statistics.median(v))}" if v else "n=0"
print(f"AU control : {s(au)}")
print(f"kernel EU  : {s(eu)}")
PYEOF
tail -1 "$OUT/ktx.log"
