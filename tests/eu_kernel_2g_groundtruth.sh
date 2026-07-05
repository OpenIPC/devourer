#!/usr/bin/env bash
# Ground-truth check: does the OpenHD out-of-tree rtl88x2eu kernel driver
# receive on 2.4 GHz with THIS RTL8822EU (0bda:a81a)? Devourer's userspace
# path is deaf on 2.4 GHz (fine on 5 GHz). If the kernel driver hears ch6,
# the hardware is good and it's a devourer bug; the driver's monitor RX is
# also the reference register state.
#
#   sudo ./tests/eu_kernel_2g_groundtruth.sh [channel]
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
KO="$ROOT/reference/rtl88x2eu/rtl88x2eu_ohd.ko"
CH="${1:-6}"
SECS="${SECS:-12}"
IFACE=""

cleanup() {
    [ -n "$IFACE" ] && { sudo ip link set "$IFACE" down 2>/dev/null; }
    sudo pkill -x tcpdump 2>/dev/null || true
    sudo rmmod rtl88x2eu_ohd 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "== loading OHD rtl88x2eu (mon mode support) =="
sudo rmmod rtl88x2eu_ohd 2>/dev/null || true
sudo insmod "$KO" rtw_mon_ndev=1 2>/dev/null || sudo insmod "$KO" || {
    echo "insmod failed"; exit 1; }
sleep 3

# Find the interface the driver created for the a81a.
for i in $(ls /sys/class/net); do
    dev="/sys/class/net/$i/device"
    [ -e "$dev" ] || continue
    id=$(cat "$dev/../idProduct" 2>/dev/null || true)
    drv=$(basename "$(readlink -f "$dev/driver" 2>/dev/null)" 2>/dev/null || true)
    echo "  iface $i driver=$drv"
    case "$drv" in *88x2eu*|*8822*) IFACE="$i";; esac
done
# fall back: any new wlan/mon iface
[ -z "$IFACE" ] && IFACE=$(ls /sys/class/net | grep -iE "^wlx|^wlan|^mon" | head -1)
[ -z "$IFACE" ] && { echo "no EU interface found"; ip link; exit 1; }
echo "== EU interface: $IFACE =="

echo "== monitor mode on ch$CH =="
sudo ip link set "$IFACE" down
sudo iw dev "$IFACE" set type monitor 2>/dev/null || sudo iwconfig "$IFACE" mode monitor
sudo ip link set "$IFACE" up
sleep 1
sudo iw dev "$IFACE" set channel "$CH" 2>/dev/null || \
    sudo iwconfig "$IFACE" channel "$CH"
echo "  $(sudo iw dev "$IFACE" info 2>/dev/null | grep -E 'channel|type' | tr '\n' ' ')"

echo "== capturing ${SECS}s on $IFACE ch$CH =="
n=$(sudo timeout "$SECS" tcpdump -i "$IFACE" -nn -c 50 2>/dev/null | wc -l)
echo "== RESULT: $n frames captured on 2.4 GHz ch$CH =="
[ "$n" -gt 0 ] && echo "== HARDWARE OK — kernel hears 2.4 GHz => devourer bug ==" \
               || echo "== kernel also deaf — suspect hardware/antenna =="
