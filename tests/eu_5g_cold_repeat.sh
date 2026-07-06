#!/usr/bin/env bash
# Honest independent-sample test of EU 5 GHz TX: VBUS power-cycle the adapter
# before EVERY run so each is a true cold start (the chip retains state across a
# soft re-init, so back-to-back demo runs are NOT independent). N cold devourer
# runs, then N cold kernel runs, each measured on the SDR.
set -u
HUB=3-2.3          # hub the EU (a81a) sits on
FREQ=5180e6
N=${1:-4}
KMOD=rtl88x2eu_ohd

cleanup(){ sudo pkill -9 -x txdemo 2>/dev/null; sudo pkill -9 -f kernel_tx_inject 2>/dev/null; }
trap cleanup EXIT

cold_cycle(){ # full VBUS off/on of the EU's hub port
  sudo pkill -9 -x txdemo 2>/dev/null; sudo pkill -9 -f kernel_tx_inject 2>/dev/null
  sudo rmmod $KMOD 2>/dev/null; sleep 1
  sudo uhubctl -l $HUB -a off >/dev/null 2>&1; sleep 4
  sudo uhubctl -l $HUB -a on  >/dev/null 2>&1; sleep 6
}

sweep(){ # echo "6:xx 8:xx 10:xx 12:xx"
  local out=""
  for m in 6 8 10 12; do
    local d=$(sudo python3 tests/sdr_duty.py --freq $FREQ --rate 25e6 --secs 2 --gain 60 \
                --mcs 7 --bw 20 --margin-db $m 2>/dev/null | grep -oiE "duty=[0-9.]+%" | grep -oE "[0-9.]+")
    out="$out ${m}dB:${d}%"
  done
  echo "$out"
}

echo "=== $N COLD devourer EU 5G runs (power-cycled each) ==="
for i in $(seq 1 $N); do
  cold_cycle
  if ! lsusb | grep -qi a81a; then echo "  run $i: EU did not enumerate"; continue; fi
  sudo env DEVOURER_VID=0x0bda DEVOURER_PID=0xa81a DEVOURER_CHANNEL=36 \
    DEVOURER_TX_RATE=MCS7 DEVOURER_TX_GAP_US=0 DEVOURER_TX_PAYLOAD_BYTES=1465 \
    stdbuf -oL timeout -k 5 14 build/txdemo >/tmp/cold_dev_$i.log 2>&1 &
  sleep 6
  echo "  devourer cold run $i:$(sweep)"
  sudo pkill -9 -x txdemo 2>/dev/null; sleep 1
done

echo "=== $N COLD kernel EU 5G runs (power-cycled each) ==="
for i in $(seq 1 $N); do
  cold_cycle
  sudo insmod reference/rtl88x2eu/${KMOD}.ko 2>/dev/null; sleep 4
  IF=""; for d in /sys/class/net/*; do
    [ "$(basename "$(readlink -f "$d/device/driver" 2>/dev/null)" 2>/dev/null)" = "$KMOD" ] && IF=$(basename "$d"); done
  if [ -z "$IF" ]; then echo "  kernel run $i: no iface"; continue; fi
  sudo ip link set "$IF" up; sudo iw "$IF" set type monitor 2>/dev/null; sudo iw dev "$IF" set channel 36 HT20 2>/dev/null; sleep 1
  sudo python3 tests/kernel_tx_inject.py "$IF" 7 1465 14 >/tmp/cold_ktx_$i.log 2>&1 &
  sleep 4
  echo "  kernel cold run $i:$(sweep)"
  sudo pkill -9 -f kernel_tx_inject 2>/dev/null; sudo rmmod $KMOD 2>/dev/null; sleep 1
done
