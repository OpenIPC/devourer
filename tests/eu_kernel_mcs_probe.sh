#!/usr/bin/env bash
# Ground truth for issue #238 MCS4+ on THIS 8812EU unit: does the VENDOR
# kernel driver (rtl88x2eu_ohd) inject decodable MCS4/MCS7, measured by the
# same devourer 8822CU ground that decodes devourer MCS0-3 and 8812AU MCS7?
# snokvist verified kernel-MCS7-clean on their unit; this checks ours.
#   sudo tests/eu_kernel_mcs_probe.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${EU_KMCS_OUT:-/tmp/devourer-eu-kernel-mcs}"
CH="${CH:-36}"
KMOD=rtl88x2eu_ohd
mkdir -p "$OUT"

cleanup() {
    sudo -n pkill -9 -f kernel_tx_inject 2>/dev/null || true
    sudo -n pkill -x rxdemo 2>/dev/null || true
    sudo -n rmmod "$KMOD" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sudo -n pkill -x txdemo 2>/dev/null; sudo -n pkill -x rxdemo 2>/dev/null
sleep 1

echo "== ground RX (0xc812, devourer) up on ch$CH =="
: >"$OUT/ground.log"
sudo -n env DEVOURER_PID=0xc812 DEVOURER_VID=0x0bda DEVOURER_CHANNEL="$CH" \
    DEVOURER_STREAM_OUT=1 \
    stdbuf -oL timeout 300 "$ROOT/build/rxdemo" 2>"$OUT/ground.err" \
    | while IFS= read -r line; do printf '%s %s\n' "$(date +%s.%N)" "$line"; done \
    >>"$OUT/ground.log" &
GJ=$!
sleep 12

echo "== VBUS cold-cycling the EU (chip retains devourer state; kernel FW-DL
   fails on a warm chip) =="
SYS=""
for d in /sys/bus/usb/devices/*/idProduct; do
    [ "$(cat "$d" 2>/dev/null)" = "a81a" ] && { SYS=$(basename "$(dirname "$d")"); break; }
done
[ -n "$SYS" ] || { echo "ERROR: a81a not on USB"; exit 1; }
HUBLOC=${SYS%.*}; HUBPORT=${SYS##*.}
sudo -n uhubctl -l "$HUBLOC" -p "$HUBPORT" -a cycle -d 3 >/dev/null 2>&1 || {
    echo "WARN: uhubctl cycle failed (continuing warm)"; }
sleep 4

echo "== binding $KMOD on the EU =="
lsmod | grep -q "^$KMOD" || sudo -n insmod "$ROOT/reference/rtl88x2eu/${KMOD}.ko"
sleep 6
IF=""
for d in /sys/class/net/*; do
    drv=$(basename "$(readlink -f "$d/device/driver" 2>/dev/null)" 2>/dev/null)
    [ "$drv" = "$KMOD" ] && { IF=$(basename "$d"); break; }
done
[ -n "$IF" ] || { echo "ERROR: no $KMOD netdev"; exit 1; }
echo "EU kernel netdev = $IF"
sudo -n ip link set "$IF" down
sudo -n iw dev "$IF" set monitor none
sudo -n ip link set "$IF" up
sudo -n iw dev "$IF" set channel "$CH"
# snokvist's clean-MCS7 kernel baseline ran at 5 dBm; the driver's monitor
# default on this unit measures ~0 duty, so pin the power explicitly.
sudo -n iw dev "$IF" set txpower fixed 500 2>/dev/null || \
    echo "WARN: set txpower failed (driver may not support it in monitor)"
sleep 1

: >"$OUT/cells.txt"
for mcs in 0 4 7; do
    echo "== kernel inject MCS$mcs =="
    t0="$(date +%s.%N)"
    sudo -n python3 "$ROOT/tests/kernel_tx_inject.py" "$IF" "$mcs" 148 10 \
        >"$OUT/ktx-mcs$mcs.log" 2>&1 || true
    t1="$(date +%s.%N)"
    echo "mcs$mcs $mcs $t0 $t1" >>"$OUT/cells.txt"
    sleep 2
done
sudo -n pkill -x rxdemo 2>/dev/null
wait "$GJ" 2>/dev/null

python3 - "$OUT/ground.log" "$OUT/cells.txt" <<'PYEOF'
import json, statistics, sys
RATE = {0: 12, 4: 16, 7: 19}
frames = []
for line in open(sys.argv[1], errors="replace"):
    if '"ev":"rx.frame"' not in line:
        continue
    ts, _, js = line.partition(" ")
    try:
        ev = json.loads(js)
    except ValueError:
        continue
    evm = ev.get("evm")
    frames.append((float(ts), ev.get("rate", -1),
                   evm[0] if isinstance(evm, list) and evm else 0))
print("\ncell        rate-matched  other  medEVM(dB)")
for line in open(sys.argv[2]):
    label, mcs, t0, t1 = line.split()
    t0, t1 = float(t0), float(t1)
    want = RATE[int(mcs)]
    win = [(r, e) for (t, r, e) in frames if t0 + 1 <= t <= t1 - 0.2]
    hit = [e for (r, e) in win if r == want]
    other = sum(1 for (r, _) in win if r != want)
    med = int(statistics.median(hit)) if hit else 0
    print(f"{label:<11} {len(hit):>12}  {other:>5}  {med:>10}")
PYEOF
