#!/usr/bin/env bash
# Capture devourer's control-register writes on a TRULY COLD c812 (RTL8812CU):
# disable kernel auto-probe so rtw88_8822cu never initializes the chip first,
# electrically power-cycle the port, then run devourer (which self-configures the
# USB device) under usbmon. Used to diff the structured source-port init vs the
# kernel-replay init when both start from an identical cold chip — isolating the
# RX-engine-start ordering gap.
#
#   sudo tests/jaguar3_cold_capture.sh <out.txt> [extra DEVOURER_ env...]
# e.g. sudo tests/jaguar3_cold_capture.sh /tmp/sc.txt DEVOURER_J3_STRUCTURED=1
set -u
SYS=9-1.3
DRV=rtw88_8822cu
OUT=${1:?usage: cold_capture.sh out.txt [env...]}
shift || true
EXTRA="$*"
BUS=9

cleanup() {
  echo 1 | sudo tee /sys/bus/usb/drivers_autoprobe >/dev/null 2>&1
  sudo pkill -9 -f "usbmon/${BUS}u" 2>/dev/null
}
trap cleanup EXIT

# 1) prevent the kernel from grabbing the chip, then cold power-cycle
echo 0 | sudo tee /sys/bus/usb/drivers_autoprobe >/dev/null
echo "$SYS:1.0" | sudo tee /sys/bus/usb/drivers/$DRV/unbind >/dev/null 2>&1
sudo uhubctl -l 9-1 -p 3 -a cycle -d 3 >/dev/null 2>&1
sleep 6

if [ ! -e /sys/bus/usb/devices/$SYS/devnum ]; then
  echo "ERROR: $SYS did not re-enumerate"; exit 1
fi
DEVNUM=$(cat /sys/bus/usb/devices/$SYS/devnum)
BOUND=$(ls /sys/bus/usb/devices/$SYS/$SYS:1.0/driver 2>/dev/null && echo yes || echo no)
echo "cold chip: devnum=$DEVNUM kernel-bound=$BOUND"

# 2) usbmon capture of bus 9 while devourer runs
sudo modprobe usbmon 2>/dev/null
sudo sh -c "cat /sys/kernel/debug/usb/usbmon/${BUS}u > ${OUT}.raw" &
sleep 0.4

sudo env $EXTRA DEVOURER_VID=0x0bda DEVOURER_PID=0xc812 \
     DEVOURER_CHANNEL=36 stdbuf -oL -eL timeout 9 build/rxdemo \
     > "${OUT}.log" 2>&1

sudo pkill -9 -f "usbmon/${BUS}u" 2>/dev/null
sleep 0.4
sudo chown "$(id -u)" "${OUT}.raw" "${OUT}.log" 2>/dev/null || true

# 3) filter control writes for our device
grep -E "Co:${BUS}:0*${DEVNUM}:" "${OUT}.raw" > "$OUT" 2>/dev/null
echo "raw=$(wc -l < ${OUT}.raw) filtered Co=$(wc -l < $OUT)"
echo "RX events: $(grep -cE 'async completion|RX: frame' ${OUT}.log)"
