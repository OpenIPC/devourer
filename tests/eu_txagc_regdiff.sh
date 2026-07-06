#!/usr/bin/env bash
# Capture the USB vendor register-writes the KERNEL vs DEVOURER make to the EU
# (8822e) TXAGC power registers during a cold 5 GHz (ch36) bring-up, and diff
# them. Goal: find any TX-power register the kernel programs that devourer does
# not (or programs differently) — the source of the residual 5 GHz power delta.
#
# TXAGC registers of interest (8822e phydm_hal_api8822e.c):
#   0x18e8[16:10] OFDM/HT ref (path A)   0x41e8 (path B)
#   0x18a0[22:16] CCK ref     (path A)   0x41a0 (path B)
#   0x3a00-0x3a7c per-rate diff table
#   plus 5 GHz band scaling 0x0818/0x081c and power-by-rate 0x0c20-0x0c4c
set -u
HUB=3-2.3
BUS=3u
KMOD=rtl88x2eu_ohd
OUT=/tmp/eu_regdiff
mkdir -p $OUT

cleanup(){
  sudo pkill -9 -x txdemo 2>/dev/null
  sudo pkill -9 -f kernel_tx_inject 2>/dev/null
  sudo pkill -9 -f "cat /sys/kernel/debug/usb/usbmon" 2>/dev/null
}
trap cleanup EXIT

sudo modprobe usbmon 2>/dev/null
mountpoint -q /sys/kernel/debug || sudo mount -t debugfs none /sys/kernel/debug 2>/dev/null

cold_cycle(){
  cleanup
  sudo rmmod $KMOD 2>/dev/null; sleep 1
  sudo uhubctl -l $HUB -a off >/dev/null 2>&1; sleep 4
  sudo uhubctl -l $HUB -a on  >/dev/null 2>&1; sleep 6
}

echo "=== KERNEL cold ch36 capture ==="
cold_cycle
sudo timeout 20 cat /sys/kernel/debug/usb/usbmon/$BUS > $OUT/k_raw.txt &
CAP=$!
sudo insmod reference/rtl88x2eu/${KMOD}.ko 2>/dev/null; sleep 4
IF=""; for d in /sys/class/net/*; do
  [ "$(basename "$(readlink -f "$d/device/driver" 2>/dev/null)" 2>/dev/null)" = "$KMOD" ] && IF=$(basename "$d"); done
if [ -n "$IF" ]; then
  sudo ip link set "$IF" up; sudo iw "$IF" set type monitor 2>/dev/null
  sudo iw dev "$IF" set channel 36 HT20 2>/dev/null; sleep 1
  sudo python3 tests/kernel_tx_inject.py "$IF" 7 1465 6 >/dev/null 2>&1 || true
fi
sleep 1; sudo kill $CAP 2>/dev/null; wait $CAP 2>/dev/null
sudo rmmod $KMOD 2>/dev/null

echo "=== DEVOURER cold ch36 capture ==="
cold_cycle
sudo timeout 20 cat /sys/kernel/debug/usb/usbmon/$BUS > $OUT/d_raw.txt &
CAP=$!
sudo env DEVOURER_VID=0x0bda DEVOURER_PID=0xa81a DEVOURER_CHANNEL=36 \
  DEVOURER_TX_RATE=MCS7 DEVOURER_TX_GAP_US=2000 \
  stdbuf -oL timeout -k 3 8 build/txdemo >/dev/null 2>&1 || true
sudo kill $CAP 2>/dev/null; wait $CAP 2>/dev/null

echo ""
echo "=== per-path TXAGC reference (0x18e8/0x41e8 OFDM, 0x18a0/0x41a0 CCK) ==="
# Decode the final per-path reference indices chronologically (a flat/equal
# devourer pathB vs a higher/lower kernel pathB is the per-path power gap).
python3 tests/decode_txagc.py $OUT/k_raw.txt "KERNEL   ch36"
python3 tests/decode_txagc.py $OUT/d_raw.txt "DEVOURER ch36"
