#!/usr/bin/env bash
# Hardware-vs-software check: can the KERNEL driver (rtw88_8822cu) receive on the
# CU hardware? Put the CU netdev in monitor mode on ch1, tcpdump it while the
# 8812AU floods devourer beacons. If the kernel sees frames but devourer can't,
# the CU RX hardware/bench is fine and the gap is in devourer's Jaguar3 bring-up.
#
#   sudo tests/cu_kernel_rx_check.sh [channel] [seconds]
set -u
CH=${1:-1}
SECS=${2:-20}
CUIF=${CUIF:-wlp4s0u2u4}

find_sys() { for d in /sys/bus/usb/devices/*/idProduct; do
  [ "$(cat "$d" 2>/dev/null)" = "$1" ] && { basename "$(dirname "$d")"; return; }; done; }
TXS=$(find_sys 8812)
[ -z "$TXS" ] && { echo "no 8812AU"; exit 1; }

cleanup() {
  sudo pkill -9 -x WiFiDriverTxDe 2>/dev/null
  sudo pkill -9 -f "tcpdump -i ${CUIF}mon" 2>/dev/null
  sudo iw dev ${CUIF}mon del 2>/dev/null
  sudo ip link set "$CUIF" up 2>/dev/null
  echo "$TXS:1.0" | sudo tee /sys/bus/usb/drivers/rtw88_8812au/bind >/dev/null 2>&1
  sleep 1
}
trap cleanup EXIT

# CU -> monitor mode on ch$CH (kernel driver stays bound)
sudo ip link set "$CUIF" down 2>/dev/null
sudo iw dev "$CUIF" set monitor none 2>/dev/null || \
  sudo iw phy "$(cat /sys/class/net/$CUIF/phy80211/name)" interface add ${CUIF}mon type monitor 2>/dev/null
MONIF="$CUIF"
sudo ip link set "$MONIF" up 2>/dev/null
sudo iw dev "$MONIF" set channel "$CH" 2>/dev/null
echo "[harness] CU monitor iface=$MONIF ch=$CH; TX 8812AU=$TXS"
iw dev "$MONIF" info 2>/dev/null | grep -iE "type|channel" | sed 's/^/  /'

# 8812AU -> devourer beacon flood
echo "$TXS:1.0" | sudo tee /sys/bus/usb/drivers/rtw88_8812au/unbind >/dev/null 2>&1; sleep 1
sudo env DEVOURER_VID=0x0bda DEVOURER_PID=0x8812 DEVOURER_CHANNEL=$CH \
     stdbuf -oL timeout -k 5 "$SECS" build/WiFiDriverTxDemo >/tmp/cuk_tx.log 2>&1 &
sleep 5

echo "=== tcpdump on $MONIF for $((SECS-7))s ==="
sudo timeout $((SECS-7)) tcpdump -i "$MONIF" -nn -c 200 2>/dev/null | tee /tmp/cuk_rx.log | head -8
echo "==================== RESULT ===================="
TOT=$(wc -l < /tmp/cuk_rx.log)
SA=$(grep -ic "57:42:75:05:d6:00\|SA:57:42:75" /tmp/cuk_rx.log)
echo "kernel CU monitor frames captured : $TOT"
echo "  (canonical-SA 57:42:75 frames   : $SA)"
[ "$TOT" -gt 0 ] && echo ">>> CU HARDWARE RECEIVES via kernel — devourer bring-up gap <<<" \
                 || echo ">>> CU kernel monitor also 0 — hardware/bench/RF issue <<<"