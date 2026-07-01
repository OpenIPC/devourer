#!/usr/bin/env bash
# Capture devourer's register-write sequence (USB vendor control, bRequest=0x05)
# during InitWrite on channel 36, via usbmon bus 3. Output /tmp/eu_dev_usbmon.txt.
#   sudo tests/eu_usbmon_devourer.sh
set -u
BUS=3; MON=/sys/kernel/debug/usb/usbmon/${BUS}u; RAW=/tmp/eu_dev_usbmon.txt
cleanup(){ sudo pkill -9 -f "cat $MON" 2>/dev/null; sudo pkill -9 -x WiFiDriverTxDe 2>/dev/null; }
trap cleanup EXIT
for i in $(ls /sys/class/net | grep -E "^wl"); do sudo ip link set "$i" down 2>/dev/null; done
sudo rmmod rtl88x2eu_ohd 2>/dev/null; sleep 2
sudo cat "$MON" > "$RAW" 2>/dev/null &
sleep 1
sudo env DEVOURER_VID=0x0bda DEVOURER_PID=0xa81a DEVOURER_CHANNEL=36 \
  DEVOURER_TX_RATE=MCS7 stdbuf -oL timeout -k 5 10 build/WiFiDriverTxDemo \
  >/tmp/eu_dev_tx.log 2>&1
sleep 1
cleanup
echo "captured $(wc -l <"$RAW") lines; control writes: $(grep -cE ' s 40 05 ' "$RAW")"
