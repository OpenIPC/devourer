#!/usr/bin/env bash
# Decisive RX diagnostic: while the 8812AU floods canonical beacons, run the CU
# (c812) RX demo and usbmon-capture the CU's bus. Answers whether the Jaguar3
# chip puts ANY bytes on bulk-IN (Bi ... > 0) — separating a chip-side RX-DMA
# failure (no Bi data) from a host/parse issue (Bi data the demo drops).
#
#   sudo tests/jaguar3_rx_usbmon.sh [channel] [seconds]
set -u
CH=${1:-1}
SECS=${2:-22}

find_sys() { for d in /sys/bus/usb/devices/*/idProduct; do
  [ "$(cat "$d" 2>/dev/null)" = "$1" ] && { basename "$(dirname "$d")"; return; }; done; }
TXS=$(find_sys 8812); RXS=$(find_sys c812)
[ -z "$TXS" ] && { echo "no 8812AU"; exit 1; }
[ -z "$RXS" ] && { echo "no c812"; exit 1; }
RXBUS=$(cat /sys/bus/usb/devices/$RXS/busnum)
RXDEV=$(cat /sys/bus/usb/devices/$RXS/devnum)
MON=/sys/kernel/debug/usb/usbmon/${RXBUS}u
echo "[harness] TX 8812AU=$TXS  RX c812=$RXS (bus $RXBUS dev $RXDEV)"

cleanup() {
  sudo pkill -9 -x WiFiDriverTxDem 2>/dev/null
  sudo pkill -9 -x WiFiDriverDemo 2>/dev/null
  sudo pkill -9 -f "cat $MON" 2>/dev/null
  echo "$TXS:1.0" | sudo tee /sys/bus/usb/drivers/rtw88_8812au/bind >/dev/null 2>&1
  echo "$RXS:1.0" | sudo tee /sys/bus/usb/drivers/rtw88_8822cu/bind >/dev/null 2>&1
  sleep 2
}
trap cleanup EXIT

sudo modprobe usbmon
echo "$TXS:1.0" | sudo tee /sys/bus/usb/drivers/rtw88_8812au/unbind >/dev/null 2>&1
echo "$RXS:1.0" | sudo tee /sys/bus/usb/drivers/rtw88_8822cu/unbind >/dev/null 2>&1
sleep 1

sudo env DEVOURER_VID=0x0bda DEVOURER_PID=0x8812 DEVOURER_CHANNEL=$CH \
     stdbuf -oL timeout -k 5 "$SECS" build/WiFiDriverTxDemo >/tmp/j3rx_tx.log 2>&1 &
sleep 5
sudo sh -c "cat $MON > /tmp/j3rx_mon.raw" & sleep 0.3
sudo env DEVOURER_VID=0x0bda DEVOURER_PID=0xc812 DEVOURER_CHANNEL=$CH \
     stdbuf -oL -eL timeout -k 5 $((SECS-7)) build/WiFiDriverDemo >/tmp/j3rx_rx.log 2>&1
sudo pkill -9 -f "cat $MON" 2>/dev/null; sleep 0.3
sudo chown "$(id -u):$(id -g)" /tmp/j3rx_mon.raw 2>/dev/null || true

echo "==================== RESULT ===================="
echo "demo RX pkt lines    : $(grep -c 'RX pkt' /tmp/j3rx_rx.log)"
# usbmon: Bi = bulk-in callback completions for the CU device. Count those that
# returned >0 bytes (the token after the status field on a 'Bi ... 0 N' line).
BI_TOTAL=$(grep -E "Bi:${RXBUS}:0*${RXDEV}:" /tmp/j3rx_mon.raw | wc -l)
BI_DATA=$(grep -E "Bi:${RXBUS}:0*${RXDEV}:.* 0 ([1-9][0-9]*) " /tmp/j3rx_mon.raw | wc -l)
echo "bulk-IN (Bi) URBs    : $BI_TOTAL"
echo "bulk-IN with data    : $BI_DATA"
if [ "$BI_DATA" -gt 0 ]; then
  echo ">>> chip DELIVERS RX bytes to bulk-IN (host/parse issue) <<<"
else
  echo ">>> chip delivers ZERO bulk-IN data (chip-side RX-DMA not running) <<<"
fi