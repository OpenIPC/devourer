#!/usr/bin/env bash
# Capture the kernel rtl88x2eu_ohd register-write sequence (USB vendor control
# transfers, bRequest=0x05) during init + channel-36 set, via usbmon. The EU is
# on bus 3. Output: /tmp/eu_kernel_usbmon.txt (raw) — parse with parse helper.
#   sudo tests/eu_usbmon_capture.sh
set -u
BUS=3
MON=/sys/kernel/debug/usb/usbmon/${BUS}u
RAW=/tmp/eu_kernel_usbmon.txt

cleanup(){ sudo pkill -9 -f "cat $MON" 2>/dev/null; }
trap cleanup EXIT

# Start fresh: unload driver so we capture a clean cold init.
sudo pkill -9 -x txdemo 2>/dev/null
for i in $(ls /sys/class/net | grep -E "^wl"); do sudo ip link set "$i" down 2>/dev/null; done
sudo rmmod rtl88x2eu_ohd 2>/dev/null; sleep 2

# Begin capture.
sudo cat "$MON" > "$RAW" 2>/dev/null &
sleep 1

# Cold init + channel 36.
sudo insmod reference/rtl88x2eu/rtl88x2eu_ohd.ko 2>/dev/null || sudo modprobe rtl88x2eu_ohd 2>/dev/null
sleep 4
IF=$(ls /sys/class/net | grep -E "^wl" | head -1)
sudo ip link set "$IF" down 2>/dev/null
sudo iw dev "$IF" set monitor none 2>/dev/null
sudo ip link set "$IF" up 2>/dev/null
sudo iw dev "$IF" set channel 36 2>/dev/null
sleep 2

cleanup
echo "captured $(wc -l <"$RAW") usbmon lines -> $RAW"
echo "control writes (bRequest=05): $(grep -cE ' s 40 05 ' "$RAW")"
