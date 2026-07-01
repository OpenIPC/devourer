#!/usr/bin/env bash
# Kernel-driver on-air TX benchmark for the EU (rtl88x2eu_ohd), to compare vs
# devourer on the SAME hub/SDR. For each band: put the kernel netdev in monitor
# mode on the channel, flood MCS7/20 frames via raw injection, measure SDR duty.
#
#   sudo tests/kernel_tx_bench.sh <monitor_iface>
set -u
IF=${1:?usage: kernel_tx_bench.sh <iface>}
MCS=7; BW=20
# band: "label chan freq"
BANDS=("2.4GHz_ch6 6 2412e6" "UNII-1_ch36 36 5180e6" "UNII-2/3_ch149 149 5745e6")
# (ch6 SDR at 2437 in devourer bench; use 2437 to match)
BANDS=("2.4GHz_ch6 6 2437e6" "UNII-1_ch36 36 5180e6" "UNII-2/3_ch149 149 5745e6")

cleanup(){ sudo pkill -9 -f kernel_tx_inject 2>/dev/null; }
trap cleanup EXIT

sudo ip link set "$IF" down 2>/dev/null
sudo iw dev "$IF" set monitor none 2>/dev/null
sudo ip link set "$IF" up 2>/dev/null

echo "# kernel-driver ($IF) on-air TX, MCS$MCS/${BW}MHz"
for b in "${BANDS[@]}"; do
  set -- $b; label=$1; ch=$2; freq=$3
  sudo iw dev "$IF" set channel "$ch" 2>/dev/null
  sleep 1
  sudo python3 tests/kernel_tx_inject.py "$IF" $MCS 1465 12 >/tmp/ktx_$label.log 2>&1 &
  sleep 4
  duty=$(sudo python3 tests/sdr_duty.py --freq $freq --rate 25e6 --secs 5 --gain 60 --mcs $MCS --bw $BW 2>&1 | grep -oiE "duty=[0-9.]+%.*on_air~=[0-9.]+Mbps" | head -1)
  sudo pkill -9 -f kernel_tx_inject 2>/dev/null; sleep 1
  echo "  $label: $duty  | $(cat /tmp/ktx_$label.log 2>/dev/null | tail -1)"
done
