#!/usr/bin/env bash
# Capture the kernel rtw88_8822cu bring-up of the WDN1300H (RTL8812CU,
# 0bda:c812) WITH the interface brought up in monitor mode on a channel — so
# the capture includes the RX datapath (CR/RCR enable, RQPN/page alloc, queue
# priority, channel set) that a plain unbind/bind probe omits.
#
#   sudo tests/jaguar3_kernel_capture_monitor.sh [out.txt] [channel]
set -u

SYS=9-1.3
DRV=rtw88_8822cu
OUT=${1:-/tmp/j3_kernel_mon.txt}
CH=${2:-36}
RAW=/tmp/j3_kernel_mon_raw.txt
BUS=$(cat /sys/bus/usb/devices/$SYS/busnum)
DEVNUM=$(cat /sys/bus/usb/devices/$SYS/devnum)
MON=/sys/kernel/debug/usb/usbmon/${BUS}u

iface() { basename "$(readlink -f /sys/bus/usb/devices/$SYS/$SYS:1.0/net/* 2>/dev/null)" 2>/dev/null; }

cleanup() {
  sudo pkill -9 -f "cat $MON" 2>/dev/null
  echo "$SYS:1.0" | sudo tee /sys/bus/usb/drivers/$DRV/bind >/dev/null 2>&1
}
trap cleanup EXIT

sudo modprobe usbmon
echo "device $SYS bus $BUS devnum $DEVNUM ; monitor $MON ; channel $CH"

# cold de-init, then capture across a full bind + monitor-up + channel-set.
echo "$SYS:1.0" | sudo tee /sys/bus/usb/drivers/$DRV/unbind >/dev/null 2>&1; sleep 1
sudo sh -c "cat $MON > $RAW" & sleep 0.3
echo "$SYS:1.0" | sudo tee /sys/bus/usb/drivers/$DRV/bind >/dev/null; sleep 4

IF=$(iface)
echo "kernel iface: ${IF:-<none>}"
if [ -n "$IF" ]; then
  sudo ip link set "$IF" down 2>/dev/null
  sudo iw dev "$IF" set monitor none 2>/dev/null || sudo iw dev "$IF" set type monitor 2>/dev/null
  sudo ip link set "$IF" up 2>/dev/null
  sudo iw dev "$IF" set channel "$CH" 2>/dev/null
  echo "iface state: $(iw dev "$IF" info 2>/dev/null | grep -oE 'type [a-z]+|channel [0-9]+' | tr '\n' ' ')"
  sleep 3
fi

sudo pkill -9 -f "cat $MON" 2>/dev/null; sleep 0.3
sudo chown "$(id -u):$(id -g)" "$RAW" 2>/dev/null || true
grep -E "Co:${BUS}:0*${DEVNUM}:" "$RAW" > "$OUT" 2>/dev/null
[ -s "$OUT" ] || cp "$RAW" "$OUT"
echo "captured $(wc -l < "$RAW") raw / $(wc -l < "$OUT") dev lines -> $OUT"
echo "reg writes: $(grep -cE '\bCo:.* s 40 05 ' "$OUT")"
