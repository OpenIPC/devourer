#!/usr/bin/env bash
# kestrel_8832cu_kernel_rx_ab.sh — decisive hardware-vs-software check for the
# 8832CU RX-sensitivity gap. devourer decodes ~10x fewer ambient frames than the
# 8852BU on ch36 (6 vs 65 / 10 s) even after the halrf_dack_8852c fix. This runs
# the SAME 8832CU dongle under the VENDOR rtl8852cu kernel driver in monitor mode
# and counts decoded frames over the same window. If the kernel driver also sees
# ~6, the gap is hardware/antenna (TX50UH front-end) and we stop chasing; if it
# sees ~65, it's a real devourer RX-cal gap (port halrf_rx_dck_8852c next).
#
# Prereq: the vendor .ko is built (make -C reference/rtl8852cu). The in-tree
# rtw89 has no USB 8852C driver, so there is no auto-probe conflict for 0101.
#
#   sudo tests/kestrel_8832cu_kernel_rx_ab.sh [channel] [seconds]
set -u
cd "$(dirname "$0")/.."
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }
CH=${1:-36}; SECS=${2:-10}
VID=35bc; PID=0101; HUB="3-2.3"; PORT="1"
KO=$(find reference/rtl8852cu -maxdepth 2 -iname '8852cu.ko' -o -iname 'rtl8852cu.ko' 2>/dev/null | head -1)
[ -n "$KO" ] || { echo "FAIL: vendor .ko not built (make -C reference/rtl8852cu)"; exit 2; }
MOD=$(basename "$KO" .ko)
PCAP=/tmp/kestrel_8832cu_kernel.pcap

netdev() { # the wlan/wlp netdev backed by our USB dev
  local d; for d in /sys/class/net/*/device; do
    local dev; dev=$(readlink -f "$d" 2>/dev/null)
    case "$dev" in *"/$HUB.1/"*|*"3-2.3.1"*) basename "$(dirname "$d")"; return;; esac
  done
  # fallback: any non-eth/non-lo iface that appeared
  for d in /sys/class/net/*; do
    local n; n=$(basename "$d"); case "$n" in wlan*|wlp*|wlx*) echo "$n"; return;; esac
  done
}
cleanup() {
  pkill -9 -x tcpdump 2>/dev/null || true
  rmmod "$MOD" 2>/dev/null || true
}
trap cleanup EXIT

echo ">> load vendor $MOD + bind the 8832CU"
rmmod "$MOD" 2>/dev/null || true
insmod "$KO" 2>/dev/null || modprobe "$MOD" 2>/dev/null || { echo "FAIL: insmod $KO"; exit 2; }
uhubctl -l "$HUB" -p "$PORT" -a cycle -d 2 >/dev/null 2>&1
sleep 10
IF=$(netdev)
[ -n "$IF" ] || { echo "FAIL: no netdev appeared (driver didn't bind)"; dmesg | tail -8; exit 2; }
echo "   netdev: $IF"

echo ">> monitor mode on ch$CH"
ip link set "$IF" down 2>/dev/null
iw dev "$IF" set type monitor 2>/dev/null || { echo "FAIL: monitor mode unsupported"; exit 2; }
ip link set "$IF" up 2>/dev/null
iw dev "$IF" set channel "$CH" 2>/dev/null || iw dev "$IF" set freq $(( CH<=14 ? 2407+CH*5 : 5000+CH*5 )) 2>/dev/null
sleep 1

echo ">> capture ${SECS}s"
timeout "$SECS" tcpdump -i "$IF" -w "$PCAP" -U 2>/dev/null || true
n=$(tcpdump -r "$PCAP" 2>/dev/null | wc -l)
echo "=================================================================="
echo "vendor-driver 8832CU monitor ch$CH ${SECS}s: $n frames"
echo "(devourer on the same dongle/window: ~6; the 8852BU: ~65)"
echo "   >> if ~6  -> HARDWARE/antenna, stop chasing the sensitivity gap"
echo "   >> if ~65 -> devourer software gap, port halrf_rx_dck_8852c next"
