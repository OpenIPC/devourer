#!/usr/bin/env bash
# jaguar2_tx_onair.sh — validate RTL8822BU (Archer T3U) on-air TX by sniffing
# devourer's canonical beacon (SA 57:42:75:05:d6:00) on a separate monitor
# adapter (host 8812au, a DIFFERENT chip — not the port target).
set -uo pipefail

CH=${1:-9}
RATE=${2:-MCS1}
SNIFF_IF=${SNIFF_IF:-wlp13s0u2}   # host 8812au in monitor mode
CANON_SA=57:42:75:05:d6:00
DUR=${DUR:-10}

cleanup() {
  sudo pkill -f "tcpdump -i $SNIFF_IF" 2>/dev/null
  sudo pkill -x WiFiDriverTxDemo 2>/dev/null
}
trap cleanup EXIT

echo "[tx] sniffer $SNIFF_IF -> monitor ch$CH"
sudo ip link set "$SNIFF_IF" down
sudo iw dev "$SNIFF_IF" set type monitor 2>/dev/null
sudo ip link set "$SNIFF_IF" up
sudo iw dev "$SNIFF_IF" set channel "$CH" 2>/dev/null
sleep 1

echo "[tx] starting tcpdump (filter SA $CANON_SA)"
sudo timeout $((DUR + 3)) tcpdump -i "$SNIFF_IF" -nn -e -c 20 \
  "wlan addr2 $CANON_SA" >/tmp/j2_tx_sniff.txt 2>/dev/null &
SNIFF_PID=$!
sleep 1

echo "[tx] recovering T3U + starting WiFiDriverTxDemo (ch$CH rate $RATE)"
RECOVER_HUB_LOC=4-2.3 RECOVER_HUB_PORT=3 bash /home/josephnef/git/lkl-wifi-poc/scripts/recover-chip.sh >/dev/null 2>&1
sleep 5
timeout "$DUR" sudo stdbuf -oL env \
  DEVOURER_VID=0x2357 DEVOURER_PID=0x012d DEVOURER_CHANNEL=$CH \
  DEVOURER_TX_RATE=$RATE ${DEVOURER_TX_PWR:+DEVOURER_TX_PWR=$DEVOURER_TX_PWR} \
  ./build/WiFiDriverTxDemo >/tmp/j2_tx_demo.txt 2>&1 || true

wait $SNIFF_PID 2>/dev/null
echo "=== TX demo tail ==="; grep -iE 'bulk|TX|ready for TX|error|send' /tmp/j2_tx_demo.txt | tail -5
echo "=== sniffer caught (canonical SA beacons) ==="
n=$(grep -c "$CANON_SA" /tmp/j2_tx_sniff.txt 2>/dev/null || echo 0)
echo "frames from $CANON_SA: $n"
head -4 /tmp/j2_tx_sniff.txt
[ "$n" -gt 0 ] && echo "[tx] ON-AIR TX CONFIRMED" || echo "[tx] no frames sniffed"
