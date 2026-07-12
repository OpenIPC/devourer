#!/usr/bin/env bash
# Golden-init capture for the 8812EU (issue #238): record the vendor kernel's
# ENTIRE cold-init register-write stream (VBUS cold -> insmod -> monitor ch36
# -> txpower fixed 500) via usbmon, and emit DEVOURER_REPLAY_WSEQ files:
#   /tmp/eu_golden_full.wseq  - every control write (addr width value)
#   /tmp/eu_golden_bb.wseq    - BB/RF plane only (addr >= 0x800) — the safe
#                               replay set (no MAC power/DMA hazards)
# The kernel FW download rides bulk-out, so control-write capture naturally
# excludes it. This is the lever that found the 8822B RF18 bug.
#   sudo tests/eu_golden_capture.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
KMOD=rtl88x2eu_ohd
CH="${CH:-36}"
RAW=/tmp/eu_golden_usbmon.txt

CATPID=""
cleanup(){ [ -n "$CATPID" ] && sudo -n kill "$CATPID" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
sudo -n modprobe usbmon 2>/dev/null || true

# Locate the EU + VBUS cold-cycle it with the driver unloaded.
sudo -n rmmod "$KMOD" 2>/dev/null
SYS=""
for d in /sys/bus/usb/devices/*/idProduct; do
    [ "$(cat "$d" 2>/dev/null)" = "a81a" ] && { SYS=$(basename "$(dirname "$d")"); break; }
done
[ -n "$SYS" ] || { echo "ERROR: a81a not on USB"; exit 1; }
BUS=$(cat /sys/bus/usb/devices/$SYS/busnum)
sudo -n uhubctl -l "${SYS%.*}" -p "${SYS##*.}" -a cycle -d 3 >/dev/null 2>&1 || {
    echo "WARN: uhubctl cycle failed (continuing warm)"; }
sleep 4

MON=/sys/kernel/debug/usb/usbmon/${BUS}u
sudo -n sh -c ": > $RAW"
sudo -n sh -c "cat $MON > $RAW" &
CATPID=$!
sleep 1

echo "== insmod + monitor ch$CH + txpower 500 (capturing) =="
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
sleep 5   # let one watchdog/TSSI pass land in the capture
cleanup; CATPID=""
sleep 0.5

# The device number can change across the VBUS cycle — parse every device on
# the bus and let the write count sanity-check it.
python3 - "$RAW" <<'PYEOF'
import re, sys
pat = re.compile(r"S Co:\d+:0*(\d+):0 s 40 05 (\w+) (\w+) (\w+) \d+(?: = (\w+))?")
full = open("/tmp/eu_golden_full.wseq", "w")
bb = open("/tmp/eu_golden_bb.wseq", "w")
n = nb = 0
for line in open(sys.argv[1], errors="replace"):
    m = pat.search(line)
    if not m:
        continue
    addr = int(m.group(2), 16)
    wlen = int(m.group(4), 16)
    data = m.group(5) or ""
    if not data or wlen not in (1, 2, 4) or len(data) < 2 * wlen:
        continue
    # usbmon data is the raw wire byte stream (LE) — convert to a host value.
    val = int.from_bytes(bytes.fromhex(data[:2 * wlen]), "little")
    print(f"{addr:x} {wlen} {val:x}", file=full)
    n += 1
    if addr >= 0x800:
        print(f"{addr:x} {wlen} {val:x}", file=bb)
        nb += 1
sys.stderr.write(f"{n} writes -> eu_golden_full.wseq; {nb} BB/RF -> eu_golden_bb.wseq\n")
PYEOF
sudo -n rmmod "$KMOD" 2>/dev/null
echo "done (driver unloaded; VBUS-cycle the EU before a devourer replay run)"
