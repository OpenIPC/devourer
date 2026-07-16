#!/usr/bin/env bash
# kestrel_env_calibrate.sh — ground-truth the ch36 ambient RX rate with the
# vendor RTL8832CU driver, to calibrate whether devourer's ~0.5 frame/s is
# deafness or just a quiet channel. Loads reference/rtl8852cu/8852cu.ko, puts
# the adapter in monitor mode on the given channel, counts frames for N s.
#
#   sudo tests/kestrel_env_calibrate.sh [channel] [seconds]   (default 36 10)
set -u
cd "$(dirname "$0")/.."
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }
CH="${1:-36}"; SECS="${2:-10}"
KO=reference/rtl8852cu/8852cu.ko
VID=35bc PID=0101
[ -f "$KO" ] || { echo "FAIL: $KO not built"; exit 2; }

MON=""; PCAP=/tmp/kestrel_env.pcap
cleanup() {
  pkill -9 tcpdump 2>/dev/null || true
  [ -n "$MON" ] && ip link set "$MON" down 2>/dev/null || true
  rmmod 8852cu 2>/dev/null || true
  rm -f "$PCAP"
}
trap cleanup EXIT

echo ">> loading vendor 8852cu.ko"
rmmod 8852cu 2>/dev/null || true
insmod "$KO" 2>/dev/null || { echo "FAIL: insmod"; exit 1; }
# wait for the netdev to appear for 35bc:0101
for i in $(seq 1 15); do
  for n in /sys/class/net/*; do
    [ -e "$n/device" ] || continue
    dev=$(readlink -f "$n/device")
    idv=$(cat "$dev/../idVendor" 2>/dev/null || cat "$dev/idVendor" 2>/dev/null)
    idp=$(cat "$dev/../idProduct" 2>/dev/null || cat "$dev/idProduct" 2>/dev/null)
    if [ "$idv" = "$VID" ] && [ "$idp" = "$PID" ]; then MON=$(basename "$n"); break; fi
  done
  [ -n "$MON" ] && break; sleep 1
done
[ -n "$MON" ] || { echo "FAIL: no netdev for $VID:$PID after insmod"; exit 1; }
echo ">> iface=$MON — setting monitor + ch$CH"
ip link set "$MON" down 2>/dev/null
iw dev "$MON" set type monitor 2>/dev/null || iw dev "$MON" set monitor none 2>/dev/null
ip link set "$MON" up 2>/dev/null
iw dev "$MON" set channel "$CH" 2>/dev/null || echo "   (set channel warn)"
sleep 1
echo ">> capturing ${SECS}s on ch$CH"
timeout "$SECS" tcpdump -i "$MON" -w "$PCAP" 2>/dev/null || true
N=$(tcpdump -r "$PCAP" 2>/dev/null | wc -l)
echo "=================================================================="
printf "VENDOR ground truth: %d frames in %ds on ch%s = %.1f frames/s\n" \
  "$N" "$SECS" "$CH" "$(echo "scale=1; $N/$SECS" | bc 2>/dev/null || echo "?")"
