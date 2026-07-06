#!/usr/bin/env bash
# Measure on-air TX duty (B210 SDR) for the CU (control) and the EU, to tell
# whether the EU's external front-end radiates. Channel 149 / 5745 MHz, MCS7/20.
#
#   sudo tests/eu_tx_sdr_check.sh
set -u
CH=149; FREQ=5745e6; SECS=5
PY=${PY:-python3}

find_sys() { for d in /sys/bus/usb/devices/*/idProduct; do
  [ "$(cat "$d" 2>/dev/null)" = "$1" ] && { basename "$(dirname "$d")"; return; }; done; }
CUS=$(find_sys c812); EUS=$(find_sys a81a)

run_tx() {  # $1=pid $2=sysfs $3=driver $4=label
  local pid=$1 sys=$2 drv=$3 label=$4
  [ -n "$drv" ] && { echo "$sys:1.0" | sudo tee /sys/bus/usb/drivers/$drv/unbind >/dev/null 2>&1; sleep 1; }
  sudo env DEVOURER_VID=0x0bda DEVOURER_PID=0x$pid DEVOURER_CHANNEL=$CH DEVOURER_TX_RATE=MCS7/20 \
       stdbuf -oL timeout -k 5 $((SECS+4)) build/txdemo >/tmp/eutx_$label.log 2>&1 &
  local txpid=$!
  sleep 4
  echo "--- $label TX duty @ ${FREQ}Hz ---"
  sudo $PY tests/sdr_duty.py --freq $FREQ --secs $SECS --mcs 7 --bw 20 2>&1 | grep -iE "duty|mbps|noise|error" | head
  sudo pkill -9 -x txdemo 2>/dev/null; wait $txpid 2>/dev/null
  [ -n "$drv" ] && echo "$sys:1.0" | sudo tee /sys/bus/usb/drivers/$drv/bind >/dev/null 2>&1
  sleep 2
  echo "   ($label sends: $(grep -c 'rc=1\|bulk_send.*OK' /tmp/eutx_$label.log), fails: $(grep -c 'Failed to send' /tmp/eutx_$label.log))"
}

trap 'sudo pkill -9 -x txdemo 2>/dev/null' EXIT
[ -n "$CUS" ] && run_tx c812 "$CUS" rtw88_8822cu CU-control
[ -n "$EUS" ] && run_tx a81a "$EUS" "" EU
echo "Done. duty >> noise = radiates; duty ~ noise = dark front-end."
