#!/usr/bin/env bash
# Fresh apples-to-apples kernel-driver (rtl88x2eu_ohd) on-air TX baseline for the
# EU, ch36 MCS7/20 1465B, measured on the same hub+SDR as devourer. Binds the
# module, selects the EU netdev by driver (NOT the 8814AU), runs the injector,
# measures SDR duty, and cleans up.
set -u
KMOD=rtl88x2eu_ohd

cleanup(){
  sudo pkill -9 -f kernel_tx_inject 2>/dev/null
}
trap cleanup EXIT

# Ensure no devourer userspace holds the device.
sudo pkill -9 -x txdemo 2>/dev/null
sudo pkill -9 -x rxdemo 2>/dev/null
sleep 1

# Bind the kernel module (built out-of-tree in reference/rtl88x2eu).
if ! lsmod | grep -q "^$KMOD"; then
  sudo insmod reference/rtl88x2eu/${KMOD}.ko 2>/dev/null
fi
sleep 3

# Find the EU netdev by its driver (rtl88x2eu_ohd), not by wl* order.
IF=""
for d in /sys/class/net/*; do
  drv=$(basename "$(readlink -f "$d/device/driver" 2>/dev/null)" 2>/dev/null)
  if [ "$drv" = "$KMOD" ]; then IF=$(basename "$d"); break; fi
done
if [ -z "$IF" ]; then
  echo "ERROR: no netdev bound to $KMOD found. lsusb / dmesg:"
  lsusb | grep -i 0bda
  dmesg | tail -5
  exit 1
fi
echo "EU kernel netdev = $IF (driver $KMOD)"

sudo ip link set "$IF" down 2>/dev/null
sudo iw dev "$IF" set monitor none 2>/dev/null
sudo ip link set "$IF" up 2>/dev/null
sudo iw dev "$IF" set channel 36 2>/dev/null
sleep 1

sudo python3 tests/kernel_tx_inject.py "$IF" 7 1465 12 >/tmp/eu_ktx.log 2>&1 &
sleep 4
duty=$(sudo python3 tests/sdr_duty.py --freq 5180e6 --rate 25e6 --secs 5 --gain 60 --mcs 7 --bw 20 2>&1 | grep -oiE "duty=[0-9.]+%.*on_air~=[0-9.]+Mbps" | head -1)
sudo pkill -9 -f kernel_tx_inject 2>/dev/null
echo "KERNEL EU 5G ch36 MCS7/20 1465B: $duty  | $(tail -1 /tmp/eu_ktx.log)"
