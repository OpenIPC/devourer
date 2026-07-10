#!/usr/bin/env bash
# Hardware test for the out-of-tree morrownr 8821cu driver against the
# connected COMFAST CF-811AC (RTL8811CU, 0bda:c811).
#
# Swaps the in-kernel rtw88_8821cu for the freshly-built
# reference/8821cu/8821cu.ko submodule, confirms it binds and can scan
# (functional RX proof), then restores the in-kernel driver on exit no matter
# how we leave.
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(git -C "$HERE" rev-parse --show-toplevel)"
KO="$REPO/reference/8821cu/8821cu.ko"
USB_PATH="9-1.3"          # CF-811AC topology path (c811)
BASE_IFACE="wlp13s0u1u3"  # name under the in-kernel driver
restored=0

log() { printf '\n=== %s ===\n' "$*"; }

cleanup() {
    [ "$restored" = 1 ] && return
    restored=1
    log "CLEANUP: restoring in-kernel driver"
    rmmod 8821cu 2>/dev/null
    modprobe rtw88_8821cu 2>/dev/null
    sleep 2
    for n in /sys/class/net/*; do
        readlink -f "$n/device" 2>/dev/null | grep -q "$USB_PATH" && \
            echo "restored: $(basename "$n") -> $(basename "$(readlink "$n/device/driver" 2>/dev/null)")"
    done
}
trap cleanup EXIT INT TERM

find_iface() {  # echo the netdev currently at $USB_PATH, if any
    for n in /sys/class/net/*; do
        if readlink -f "$n/device" 2>/dev/null | grep -q "$USB_PATH"; then
            basename "$n"; return 0
        fi
    done
    return 1
}

[ -f "$KO" ] || { echo "FATAL: $KO not built"; exit 1; }

log "STEP 1: unbind in-kernel rtw88_8821cu"
modprobe -r rtw88_8821cu 2>&1
sleep 1
if find_iface >/dev/null; then echo "warn: iface still present after unload"; fi

log "STEP 2: insmod out-of-tree $KO"
insmod "$KO" || { echo "FATAL: insmod failed"; dmesg | tail -15; exit 1; }
sleep 3

log "STEP 3: confirm bind"
IFACE="$(find_iface)" || { echo "FATAL: no netdev appeared at $USB_PATH"; dmesg | tail -20; exit 1; }
DRV="$(basename "$(readlink "/sys/class/net/$IFACE/device/driver" 2>/dev/null)")"
echo "iface:  $IFACE"
echo "driver: $DRV"
echo "mac:    $(cat "/sys/class/net/$IFACE/address")"
[ "$DRV" = "8821cu" ] || echo "WARN: expected driver '8821cu', got '$DRV'"

log "STEP 4: bring up + scan (functional RX test)"
ip link set "$IFACE" up && sleep 2
ip -br link show "$IFACE"
echo "--- scan results (top APs by signal) ---"
timeout 25 iw dev "$IFACE" scan 2>&1 | \
    awk '/^BSS/{bss=$2} /SSID:/{ssid=$0} /signal:/{print $2, $3, bss, ssid}' | \
    sort -rn | head -15
N=$(timeout 20 iw dev "$IFACE" scan 2>/dev/null | grep -c '^BSS')
echo "APs seen this scan: $N"

log "STEP 5: driver dmesg"
dmesg | grep -iE "8821cu|rtl8821c|RTW" | tail -20

log "RESULT"
if [ "$DRV" = "8821cu" ] && [ "${N:-0}" -gt 0 ]; then
    echo "PASS: out-of-tree 8821cu bound to CF-811AC and scanned $N APs"
else
    echo "PARTIAL: bound=$([ "$DRV" = 8821cu ] && echo yes || echo no), APs=$N"
fi
