#!/usr/bin/env bash
# Read the RTL8821C (CF-811AC, 0bda:c811) SYS_CFG1/SYS_CFG2 registers to verify
# the real chip-id byte for the devourer factory. Unbinds the in-kernel
# rtw88_8821cu for the read, restores it on exit.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
VID=0x0bda PID=0xc811
restored=0
PROBE="$(mktemp -d)/chipid_probe"

cleanup() {
    [ "$restored" = 1 ] && return
    restored=1
    rm -f "$PROBE" 2>/dev/null
    echo "=== restore rtw88_8821cu ==="
    modprobe rtw88_8821cu 2>/dev/null
    sleep 1
}
trap cleanup EXIT INT TERM

echo "=== build probe ==="
cc -O2 -o "$PROBE" "$HERE/chipid_probe.c" $(pkg-config --cflags --libs libusb-1.0) || exit 1

echo "=== unbind in-kernel driver ==="
modprobe -r rtw88_8821cu 2>&1
sleep 1

echo "=== warm read (steady state) ==="
"$PROBE" "$VID" "$PID"
