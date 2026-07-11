#!/usr/bin/env bash
# Capture the EXACT register-write stream of `iw set txpower fixed 500` on the
# vendor rtl88x2eu_ohd (issue #238). This one command flips the unit from
# near-silent/garbled TX (kernel default power AND every devourer power state)
# to pristine MCS7 (EVM -60) — so its differential write set is precisely what
# devourer's TX-power path is missing. usbmon on the EU's bus, control writes
# (bRequest=0x05) parsed to "addr len data" lines.
#   sudo tests/eu_txpwr500_wseq.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
KMOD=rtl88x2eu_ohd
CH="${CH:-36}"
RAW=/tmp/eu_txpwr500_usbmon.txt
OUT=/tmp/eu_txpwr500_writes.txt

CATPID=""
cleanup(){ [ -n "$CATPID" ] && sudo -n kill "$CATPID" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
sudo -n modprobe usbmon 2>/dev/null || true

# Bind the vendor driver (VBUS cold first if needed).
IF=""
bindif(){
    IF=""
    for d in /sys/class/net/*; do
        drv=$(basename "$(readlink -f "$d/device/driver" 2>/dev/null)" 2>/dev/null)
        [ "$drv" = "$KMOD" ] && { IF=$(basename "$d"); return; }
    done
}
bindif
if [ -z "$IF" ]; then
    SYS=""
    for d in /sys/bus/usb/devices/*/idProduct; do
        [ "$(cat "$d" 2>/dev/null)" = "a81a" ] && { SYS=$(basename "$(dirname "$d")"); break; }
    done
    [ -n "$SYS" ] || { echo "ERROR: a81a not on USB"; exit 1; }
    sudo -n uhubctl -l "${SYS%.*}" -p "${SYS##*.}" -a cycle -d 3 >/dev/null 2>&1 || true
    sleep 4
    lsmod | grep -q "^$KMOD" || sudo -n insmod "$ROOT/reference/rtl88x2eu/${KMOD}.ko"
    sleep 6
    bindif
fi
[ -n "$IF" ] || { echo "ERROR: no $KMOD netdev"; exit 1; }
BUSNUM=$(cat "/sys/class/net/$IF/device/../busnum" 2>/dev/null || echo 3)
MON=/sys/kernel/debug/usb/usbmon/${BUSNUM}u
echo "EU netdev=$IF bus=$BUSNUM"

sudo -n ip link set "$IF" down
sudo -n iw dev "$IF" set monitor none
sudo -n ip link set "$IF" up
sudo -n iw dev "$IF" set channel "$CH"
sleep 2

sudo -n sh -c "cat $MON > $RAW" &
CATPID=$!
sleep 1
echo "== iw set txpower fixed 500 =="
sudo -n iw dev "$IF" set txpower fixed 500
sleep 2
cleanup
sleep 0.5

# Parse: control OUT writes "... s 40 05 vvvv iiii llll = data..." ->
# "0xADDR len data". wValue = register address (usbmon prints it LE-decoded).
python3 - "$RAW" >"$OUT" <<'PYEOF'
import re, sys
pat = re.compile(r" s 40 05 (\w{4}) (\w{4}) (\w{4})(?: = (.*))?$")
n = 0
for line in open(sys.argv[1], errors="replace"):
    m = pat.search(line.rstrip())
    if not m:
        continue
    addr = int(m.group(1), 16)
    length = int(m.group(3), 16)
    data = (m.group(4) or "").replace(" ", "")
    print(f"0x{addr:04x} {length} {data}")
    n += 1
sys.stderr.write(f"{n} control writes\n")
PYEOF
echo "writes -> $OUT ($(wc -l <"$OUT") lines)"
head -40 "$OUT"
