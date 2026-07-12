#!/usr/bin/env bash
# Issue #267: BB end-state diff for the 8822CU at 80 MHz RX — vendor kernel
# (88x2cu_ohd, monitor, iw set freq 5180 80 5210, RX-verified against an AU
# VHT80 flood) vs devourer (rxdemo ch36 DEVOURER_BW=80, delivers nothing).
# Same methodology as tests/eu_bb_endstate_diff.sh.
#   sudo tests/cu_bb_endstate_diff_80.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
KMOD=88x2cu_ohd
OUT=/tmp/devourer-cu80-endstate
mkdir -p "$OUT"

cleanup(){
    sudo -n pkill -x rxdemo 2>/dev/null || true
    sudo -n pkill -x txdemo 2>/dev/null || true
    sudo -n pkill -x tcpdump 2>/dev/null || true
    true
}
trap cleanup EXIT INT TERM

cu_sys(){
    SYS=""
    for d in /sys/bus/usb/devices/*/idProduct; do
        [ "$(cat "$d" 2>/dev/null)" = "c812" ] && { SYS=$(basename "$(dirname "$d")"); break; }
    done
    [ -n "$SYS" ] || { echo "ERROR: c812 not on USB"; exit 1; }
}
vbus_cold(){
    cu_sys
    sudo -n uhubctl -l "${SYS%.*}" -p "${SYS##*.}" -a cycle -d 3 >/dev/null 2>&1 || \
        echo "WARN: uhubctl cycle failed"
    sleep 4
}

echo "== kernel side: vendor $KMOD, monitor 80 MHz (ch36/central 42) =="
# The in-tree rtw88 auto-probes every enumeration — remove it for the vendor
# bind (modprobe -r does not survive re-enumeration, so do it right before).
sudo -n rmmod "$KMOD" 2>/dev/null
sudo -n modprobe -r rtw88_8822cu 2>/dev/null
vbus_cold
sudo -n modprobe -r rtw88_8822cu 2>/dev/null
sudo -n insmod "$ROOT/reference/rtl88x2cu/${KMOD}.ko" 2>/dev/null || true
sleep 5
IF=""
for d in /sys/class/net/*; do
    drv=$(basename "$(readlink -f "$d/device/driver" 2>/dev/null)" 2>/dev/null)
    case "$drv" in *88x2cu*) IF=$(basename "$d"); break;; esac
done
[ -n "$IF" ] || { echo "ERROR: no $KMOD netdev (rtw88 raced the probe?)"; exit 1; }
sudo -n ip link set "$IF" down
sudo -n iw dev "$IF" set monitor none
sudo -n ip link set "$IF" up
sudo -n iw dev "$IF" set freq 5180 80 5210
sleep 2

echo "== kernel RX sanity: AU VHT80 flood vs tcpdump =="
sudo -n timeout 16 tcpdump -i "$IF" "wlan addr2 57:42:75:05:d6:00" \
    >/dev/null 2>"$OUT/ktcp.count" &
TD=$!; sleep 2
sudo -n env DEVOURER_PID=0x8812 DEVOURER_VID=0x0bda DEVOURER_CHANNEL=36 \
    DEVOURER_HOP_BW=80 DEVOURER_TX_RATE=VHT1SS_MCS7/80 DEVOURER_TX_GAP_US=2000 \
    timeout 10 "$ROOT/build/txdemo" >/dev/null 2>&1 || true
sleep 2; sudo -n pkill -x tcpdump 2>/dev/null; wait "$TD" 2>/dev/null
grep -m1 "captured" "$OUT/ktcp.count"

PROC="/proc/net/rtl$KMOD/$IF"
[ -e "$PROC/read_reg" ] || PROC="/proc/net/$KMOD/$IF"
[ -e "$PROC/read_reg" ] || { echo "ERROR: no $PROC/read_reg"; exit 1; }
echo "== dumping kernel MAC+BB 0x000..0x4ffc =="
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

echo "== devourer side: rxdemo ch36 BW=80 + BB dump =="
vbus_cold
sudo -n env DEVOURER_PID=0xc812 DEVOURER_VID=0x0bda DEVOURER_CHANNEL=36 \
    DEVOURER_BW=80 DEVOURER_BB_DUMP=1 \
    timeout 40 "$ROOT/build/rxdemo" >/dev/null 2>"$OUT/devourer.err" || true
grep -oE "BBDUMP 0x[0-9a-f]{4} 0x[0-9a-f]{8} 0x[0-9a-f]{8} 0x[0-9a-f]{8} 0x[0-9a-f]{8}" \
    "$OUT/devourer.err" | while read -r _ base v0 v1 v2 v3; do
    b=$((base))
    printf "0x%04x %s\n0x%04x %s\n0x%04x %s\n0x%04x %s\n" \
        $b "$v0" $((b+4)) "$v1" $((b+8)) "$v2" $((b+12)) "$v3"
done >"$OUT/devourer.dump"
echo "devourer dump: $(wc -l <"$OUT/devourer.dump") regs"

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
diffs = [(a, k[a], u[a]) for a in sorted(set(k) & set(u)) if k[a] != u[a]]
print(f"{len(diffs)} differing registers of {len(set(k) & set(u))}")
for a, kv, uv in diffs:
    tag = ""
    if 0x3c00 <= a < 0x4000: tag = " (RF-A win)"
    elif 0x4c00 <= a < 0x5000: tag = " (RF-B win)"
    print(f"0x{a:04x}: kernel=0x{kv:08x} devourer=0x{uv:08x}{tag}")
PYEOF
