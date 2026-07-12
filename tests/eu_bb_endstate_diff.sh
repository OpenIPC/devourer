#!/usr/bin/env bash
# BB end-state diff: vendor kernel (monitor ch36 @ 5 dBm, pristine MCS7) vs
# devourer (same channel, MCS4+ garbled) on the 8812EU — issue #238.
# Kernel side reads via /proc read_reg while the driver is live; devourer side
# via DEVOURER_BB_DUMP at the end of InitWrite. Diff of 0x800..0x4ffc BB space
# = the candidate register set for the MCS4+ whole-PPDU corruption.
#   sudo tests/eu_bb_endstate_diff.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
KMOD=rtl88x2eu_ohd
CH="${CH:-36}"
OUT=/tmp/devourer-eu-endstate
mkdir -p "$OUT"

cleanup(){ sudo -n pkill -x txdemo 2>/dev/null || true; }
trap cleanup EXIT INT TERM

eu_sys(){
    SYS=""
    for d in /sys/bus/usb/devices/*/idProduct; do
        [ "$(cat "$d" 2>/dev/null)" = "a81a" ] && { SYS=$(basename "$(dirname "$d")"); break; }
    done
    [ -n "$SYS" ] || { echo "ERROR: a81a not on USB"; exit 1; }
}
vbus_cold(){
    eu_sys
    sudo -n uhubctl -l "${SYS%.*}" -p "${SYS##*.}" -a cycle -d 3 >/dev/null 2>&1 || \
        echo "WARN: uhubctl cycle failed"
    sleep 4
}

echo "== kernel side: cold bind + monitor ch$CH @5dBm =="
sudo -n rmmod "$KMOD" 2>/dev/null
vbus_cold
sudo -n insmod "$ROOT/reference/rtl88x2eu/${KMOD}.ko" 2>/dev/null || true
sleep 5
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
sleep 5  # settle: one watchdog/TSSI pass

PROC="/proc/net/$KMOD/$IF"
[ -e "$PROC/read_reg" ] || PROC="/proc/net/rtl88x2eu/$IF"
[ -e "$PROC/read_reg" ] || { echo "ERROR: no proc read_reg"; exit 1; }
echo "== dumping kernel MAC+BB 0x000..0x4ffc via $PROC/read_reg =="
sudo -n python3 - "$PROC/read_reg" >"$OUT/kernel.dump" <<'PYEOF'
import re, sys
proc = sys.argv[1]
for a in range(0x000, 0x5000, 4):
    with open(proc, "w") as f:
        f.write(f"{a:x} 4")
    with open(proc) as f:
        m = re.search(r"=0x(\w+)", f.read())
    print(f"0x{a:04x} 0x{int(m.group(1),16):08x}" if m else f"0x{a:04x} ERR")
PYEOF
echo "kernel dump: $(wc -l <"$OUT/kernel.dump") regs"
sudo -n rmmod "$KMOD" 2>/dev/null

echo "== devourer side: cold bring-up + BB dump =="
vbus_cold
sudo -n env DEVOURER_PID=0xa81a DEVOURER_VID=0x0bda DEVOURER_CHANNEL="$CH" \
    DEVOURER_TX_RATE=MCS7 DEVOURER_BB_DUMP=1 DEVOURER_TX_GAP_US=5000 \
    timeout 30 "$ROOT/build/txdemo" >/dev/null 2>"$OUT/devourer.err" || true
grep -oE "BBDUMP 0x[0-9a-f]{4} 0x[0-9a-f]{8} 0x[0-9a-f]{8} 0x[0-9a-f]{8} 0x[0-9a-f]{8}" \
    "$OUT/devourer.err" | while read -r _ base v0 v1 v2 v3; do
    b=$((base))
    printf "0x%04x %s\n0x%04x %s\n0x%04x %s\n0x%04x %s\n" \
        $b "$v0" $((b+4)) "$v1" $((b+8)) "$v2" $((b+12)) "$v3"
done >"$OUT/devourer.dump"
echo "devourer dump: $(wc -l <"$OUT/devourer.dump") regs"

echo "== diff (kernel vs devourer) =="
python3 - "$OUT/kernel.dump" "$OUT/devourer.dump" <<'PYEOF'
import sys
def load(p):
    d = {}
    for line in open(p):
        parts = line.split()
        if len(parts) == 2 and parts[1].startswith("0x"):
            d[int(parts[0], 16)] = int(parts[1], 16)
    return d
k, u = load(sys.argv[1]), load(sys.argv[2])
# Noisy/live registers to ignore: counters, AGC/IGI, IQK/DACK results, RF
# read windows change with 3-wire state; keep them but tag.
diffs = [(a, k[a], u[a]) for a in sorted(set(k) & set(u)) if k[a] != u[a]]
print(f"{len(diffs)} differing registers of {len(set(k) & set(u))}")
for a, kv, uv in diffs:
    tag = ""
    if 0x3c00 <= a < 0x4000: tag = " (RF-A win)"
    elif 0x4c00 <= a < 0x5000: tag = " (RF-B win)"
    print(f"0x{a:04x}: kernel=0x{kv:08x} devourer=0x{uv:08x}{tag}")
PYEOF
