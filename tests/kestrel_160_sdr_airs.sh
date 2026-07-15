#!/usr/bin/env bash
# kestrel_160_sdr_airs.sh — does the 8832CU 160 MHz TX actually radiate?
# Floods HE-160 on the TX adapter (bus9) and measures B210 energy duty inside
# the 160 MHz block. Validates the method on the SDR-reachable 5G-160 block
# first, then measures the 6G-160 block's LOWER HALF (5945-6000 MHz, the part
# below the B210's 6 GHz ceiling; the 6G ch1/160 block spans 5945-6105 MHz).
# High duty under TX vs an idle baseline = the 160 MHz signal is on air.
#   sudo tests/kestrel_160_sdr_airs.sh
set -u
ROOT=/home/josephnef/git/devourer
TXBUS=9
cyc() { uhubctl -l 9-1 -p 3 -a cycle >/dev/null 2>&1; sleep 4
  for d in /sys/bus/usb/devices/*; do [ -f "$d/idVendor" ]||continue
    [ "$(cat $d/idVendor)" = 35bc ]&&[ "$(cat $d/idProduct)" = 0101 ]||continue
    for i in "$d":*; do [ -L "$i/driver" ]&&echo "$(basename "$i")">"$(readlink -f "$i/driver")/unbind" 2>/dev/null||true; done
  done; sleep 1; }

measure() { # $1=label $2=band('' |6) $3=ch $4=sdrfreq
  local label=$1 band=$2 ch=$3 f=$4
  echo "=== $label : TX 6G? band=${band:-5G} ch$ch/160  SDR@${f} ==="
  echo -n "  idle baseline: "
  python3 "$ROOT/tests/sdr_duty.py" --freq "$f" --bw 160 --mcs 7 --secs 3 2>/dev/null | grep -oE "duty=[0-9.]+%" || echo "(sdr fail)"
  cyc
  timeout -s KILL 16 env DEVOURER_VID=0x35bc DEVOURER_PID=0x0101 DEVOURER_USB_BUS=$TXBUS \
    ${band:+DEVOURER_BAND=$band} DEVOURER_CHANNEL=$ch DEVOURER_HOP_BW=160 \
    DEVOURER_TX_RATE=HE2SS_MCS7/160 DEVOURER_TX_PWR=22 DEVOURER_TX_GAP_US=0 \
    DEVOURER_LOG_LEVEL=warn "$ROOT/build/txdemo" >/dev/null 2>/dev/null &
  local txp=$!
  sleep 3
  echo -n "  TX flooding:   "
  python3 "$ROOT/tests/sdr_duty.py" --freq "$f" --bw 160 --mcs 7 --secs 4 2>/dev/null | grep -oE "duty=[0-9.]+%|on_air[^ ]*" || echo "(sdr fail)"
  kill -9 $txp 2>/dev/null; pkill -9 -x -f build/txdemo 2>/dev/null
  echo
}

# 5G ch36/160: block center ch50 = 5250 MHz (fully B210-reachable) — method check
measure "5G-160 (method check)" "" 36 5250e6
# 6G ch1/160: block spans 5945-6105; sample 5970 MHz (lower half, under 6 GHz)
measure "6G-160 (target)" 6 1 5970e6
