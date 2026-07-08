#!/usr/bin/env bash
# jaguar1_nb_divide_sweep.sh — characterize the 8812AU narrowband ADC/DAC
# clock-divide codes (0x8ac [9:8]/[21:20]) against the SDR. For a target
# width, sweep DEVOURER_NB_DAC 0..3 x DEVOURER_NB_ADC 0..3 and record the
# occupied bandwidth of each combo's continuous TX. The TX lobe width tracks
# the DAC clock; the goal code produces ~target MHz with a single clean lobe.
#
#   sudo tests/jaguar1_nb_divide_sweep.sh <5|10>
set -uo pipefail
cd "$(dirname "$0")/.."
BW=${1:-10}
VID=0x0bda PID=0x8812
CH=44 FREQ=5220e6 RATE=46.08e6
OUT=/tmp/j1_nb_sweep_$BW
rm -rf "$OUT"; mkdir -p "$OUT"

cleanup() { sudo pkill -x txdemo 2>/dev/null; }
trap cleanup EXIT INT TERM

probe() { # freq label -> prints occ_bw MHz
  sudo python3 tests/sdr_tx_probe.py --freq "$FREQ" --rate "$RATE" --gain 50 \
      --nsamps 5e6 --label "$1" 2>&1 |
    grep -oE 'occ_bw=[0-9.]+MHz' | head -1
}

echo "=== 8812AU ${BW} MHz narrowband ADC/DAC divide-code grid (occ_bw) ==="
printf "      %8s %8s %8s %8s   (DAC->)\n" 0 1 2 3
for adc in 0 1 2 3; do
  printf "ADC=%d" "$adc"
  for dac in 0 1 2 3; do
    sudo env DEVOURER_VID=$VID DEVOURER_PID=$PID DEVOURER_CHANNEL=$CH \
        DEVOURER_NB_BW=$BW DEVOURER_NB_ADC=$adc DEVOURER_NB_DAC=$dac \
        DEVOURER_TX_GAP_US=0 timeout -s INT -k 5 24 ./build/txdemo \
        >"$OUT/tx_a${adc}d${dac}.log" 2>&1 &
    sleep 9
    ob=$(probe "a${adc}d${dac}")
    printf " %8s" "${ob#occ_bw=}"
    sudo pkill -x txdemo 2>/dev/null; wait 2>/dev/null; sleep 1
  done
  printf "\n"
done
echo "target ~${BW} MHz single lobe; baseline noise reads ~45 MHz (full window)"
