#!/usr/bin/env bash
# Capture the kernel rtw88_8822cu bring-up of the LB-LINK WDN1300H
# (RTL8812CU, 0bda:c812 — INVENTORY.md #12) as the ground-truth register
# sequence for the Jaguar-3 devourer port (power-on -> DLFW -> channel set).
#
# A fresh probe is triggered by unbind+bind of the driver while usbmon records
# the device's bus. Decode the result with tests/usbmon_regdiff.py.
#
#   sudo tests/jaguar3_kernel_capture.sh [out.txt]
set -u

SYS=9-1.3                 # sysfs path of the WDN1300H (INVENTORY #12)
DRV=rtw88_8822cu          # in-kernel CU-family driver
OUT=${1:-/tmp/j3_kernel.txt}
RAW=/tmp/j3_kernel_raw.txt

BUS=$(cat /sys/bus/usb/devices/$SYS/busnum)
DEVNUM=$(cat /sys/bus/usb/devices/$SYS/devnum)
MON=/sys/kernel/debug/usb/usbmon/${BUS}u

cleanup() {
  # exact-comm kill of our capture cat, then guarantee the driver is bound
  sudo pkill -9 -f "cat $MON" 2>/dev/null
  echo "$SYS:1.0" | sudo tee /sys/bus/usb/drivers/$DRV/bind >/dev/null 2>&1
}
trap cleanup EXIT

sudo modprobe usbmon
echo "device $SYS = bus $BUS devnum $DEVNUM ; monitor $MON"

# Ensure bound first so the unbind below is a real de-init, then capture across
# a full unbind -> bind (= fresh power-on + firmware download + channel set).
echo "$SYS:1.0" | sudo tee /sys/bus/usb/drivers/$DRV/bind >/dev/null 2>&1; sleep 2

sudo sh -c "cat $MON > $RAW" &
sleep 0.5
echo "$SYS:1.0" | sudo tee /sys/bus/usb/drivers/$DRV/unbind >/dev/null; sleep 1
echo ">> rebind (init starts)"; echo "$SYS:1.0" | sudo tee /sys/bus/usb/drivers/$DRV/bind >/dev/null
sleep 5
sudo pkill -9 -f "cat $MON" 2>/dev/null
sleep 0.3

# Keep only this device's traffic (bus 9 also carries an 8812AU).
sudo chown "$(id -u):$(id -g)" "$RAW" 2>/dev/null || true
grep -E "Co:${BUS}:0*${DEVNUM}:" "$RAW" > "$OUT" 2>/dev/null
[ -s "$OUT" ] || cp "$RAW" "$OUT"
echo "captured $(wc -l < "$RAW") raw lines; $(wc -l < "$OUT") for devnum $DEVNUM -> $OUT"
echo "control-OUT reg writes: $(grep -cE '\bCo:.* s 40 05 ' "$OUT")"
