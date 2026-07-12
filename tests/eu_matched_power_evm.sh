#!/usr/bin/env bash
# Matched-power TX EVM comparison, devourer vs vendor kernel, on the SAME
# 8812EU unit measured by the SAME 8822CU ground session (ch36/20M):
#   - devourer: MCS7 + MCS0 at DEVOURER_TX_PWR_OFFSET_QDB=-44 — lands the
#     OFDM TXAGC refs at A=31/B=40, bit-identical to the kernel's end state
#     at `iw set txpower fixed 500` (5 dBm) on this unit.
#   - kernel:   rtl88x2eu_ohd monitor ch36 @ fixed 500, injecting MCS7+MCS0.
# Output: per-cell clean-frame count + median/percentile ground EVM.
#   sudo tests/eu_matched_power_evm.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT=/tmp/eu-matched-evm
CH=36
KMOD=rtl88x2eu_ohd
mkdir -p "$OUT"
cleanup(){
    sudo -n pkill -x rxdemo 2>/dev/null || true
    sudo -n pkill -x txdemo 2>/dev/null || true
    sudo -n pkill -9 -f kernel_tx_inject 2>/dev/null || true
    sudo -n rmmod "$KMOD" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
cleanup; sleep 1

: >"$OUT/ground.log"
sudo -n env DEVOURER_PID=0xc812 DEVOURER_VID=0x0bda DEVOURER_CHANNEL="$CH" \
    DEVOURER_STREAM_OUT=1 \
    stdbuf -oL timeout 220 "$ROOT/build/rxdemo" 2>"$OUT/ground.err" \
    | while IFS= read -r line; do printf '%s %s\n' "$(date +%s.%N)" "$line"; done \
    >>"$OUT/ground.log" &
GJ=$!
sleep 12

: >"$OUT/cells.txt"
dev_cell() { # $1=label $2=rate
    t0=$(date +%s.%N)
    sudo -n env DEVOURER_PID=0xa81a DEVOURER_VID=0x0bda DEVOURER_CHANNEL="$CH" \
        DEVOURER_TX_RATE="$2" DEVOURER_TX_GAP_US=1500 \
        DEVOURER_TX_PWR_OFFSET_QDB=-44 \
        timeout 14 "$ROOT/build/txdemo" >/dev/null 2>"$OUT/tx-$1.err" || true
    t1=$(date +%s.%N)
    echo "$1 $2 $t0 $t1" >>"$OUT/cells.txt"
    sleep 2
}
echo "== devourer cells (offset -44 = kernel 5 dBm refs) =="
dev_cell dev-mcs7 MCS7
dev_cell dev-mcs0 MCS0

echo "== VBUS cold + kernel bind (monitor ch$CH @ fixed 500) =="
SYS=""
for d in /sys/bus/usb/devices/*/idProduct; do
    [ "$(cat "$d" 2>/dev/null)" = "a81a" ] && { SYS=$(basename "$(dirname "$d")"); break; }
done
[ -n "$SYS" ] || { echo "ERROR: a81a not on USB"; exit 1; }
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
sudo -n iw dev "$IF" set txpower fixed 500
sleep 2

k_cell() { # $1=label $2=mcs-index
    t0=$(date +%s.%N)
    sudo -n python3 "$ROOT/tests/kernel_tx_inject.py" "$IF" "$2" 148 12 \
        >"$OUT/ktx-$1.log" 2>&1 || true
    t1=$(date +%s.%N)
    echo "$1 mcs$2 $t0 $t1" >>"$OUT/cells.txt"
    sleep 2
}
echo "== kernel cells =="
k_cell ker-mcs7 7
k_cell ker-mcs0 0
sudo -n pkill -x rxdemo 2>/dev/null; wait "$GJ" 2>/dev/null

python3 - "$OUT/ground.log" "$OUT/cells.txt" <<'PYEOF'
import json, statistics, sys
RATE = {"MCS7": 19, "MCS0": 12, "mcs7": 19, "mcs0": 12}
frames = []
for line in open(sys.argv[1], errors="replace"):
    if '"ev":"rx.frame"' not in line: continue
    ts,_,js = line.partition(" ")
    try: ev = json.loads(js)
    except ValueError: continue
    frames.append((float(ts), ev.get("rate"), ev["evm"][0], ev["rssi"][0]))
print(f"\n{'cell':<12} {'n':>6} {'medEVM':>7} {'p10':>5} {'p90':>5} {'medRSSI':>8}")
for line in open(sys.argv[2]):
    label, rate, t0, t1 = line.split()
    t0, t1 = float(t0), float(t1)
    want = RATE[rate]
    e = sorted(x[2] for x in frames if t0+2 <= x[0] <= t1-0.3 and x[1] == want)
    r = sorted(x[3] for x in frames if t0+2 <= x[0] <= t1-0.3 and x[1] == want)
    if not e:
        print(f"{label:<12} {0:>6}")
        continue
    med = statistics.median(e)
    p10 = e[len(e)//10]; p90 = e[(len(e)*9)//10]
    print(f"{label:<12} {len(e):>6} {med:>7} {p10:>5} {p90:>5} {statistics.median(r):>8}")
PYEOF
