#!/usr/bin/env bash
# Discriminate why EU 5 GHz on-air TX is weak: is the chip's 5 GHz TXAGC left
# near zero (efuse/FW power-by-rate not applied) or is the front-end/PA gated?
#
# Three cells, all MCS7/20 saturating flood, SDR duty as ground truth:
#   1. 2G ch6   default TXAGC   (reference: EU 2G is known-working)
#   2. 5G ch36  default TXAGC   (the symptom)
#   3. 5G ch36  forced flat TXAGC override (DEVOURER_TX_PWR=0x45)
# If cell 3 >> cell 2 -> 5G TXAGC was low (need to program TX power for 5G).
# If cell 3 ~ cell 2  -> PA/RF gated on 5G (deeper).
#
#   sudo tests/eu_5g_txpwr_probe.sh
set -u
PY=${PY:-python3}
SECS=5
EU_VID=0x0bda; EU_PID=0xa81a

cleanup(){ sudo pkill -9 -x txdemo 2>/dev/null; }
trap cleanup EXIT

cell() { # $1=label $2=channel $3=freq $4=extra-env
  local label=$1 ch=$2 freq=$3 extra=$4
  cleanup; sleep 2
  sudo env DEVOURER_VID=$EU_VID DEVOURER_PID=$EU_PID DEVOURER_CHANNEL=$ch \
       DEVOURER_TX_RATE=MCS7 DEVOURER_TX_GAP_US=0 $extra \
       stdbuf -oL timeout -k 5 $((SECS+5)) build/txdemo \
       >/tmp/eu5gp_$label.log 2>&1 &
  local txpid=$!
  sleep 5
  echo "--- $label  ch$ch @ ${freq}  ($extra) ---"
  sudo $PY tests/sdr_duty.py --freq $freq --rate 25e6 --secs $SECS --gain 60 \
       --mcs 7 --bw 20 2>&1 | grep -iE "duty=" | head -1
  echo "   sends=$(grep -c 'bulk_send.*OK' /tmp/eu5gp_$label.log) fails=$(grep -c 'Failed to send' /tmp/eu5gp_$label.log)"
  cleanup; wait $txpid 2>/dev/null; sleep 1
}

sudo rmmod rtl88x2eu_ohd 2>/dev/null
cell 2g_default   6   2437e6 ""
cell 5g_default   36  5180e6 ""
cell 5g_override  36  5180e6 "DEVOURER_TX_PWR=0x45"
echo "Done."
