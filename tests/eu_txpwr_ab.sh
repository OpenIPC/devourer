#!/usr/bin/env bash
# Rigorous devourer-vs-kernel MCS7 TX-power A/B for the EU, controlling for the
# USB rail: power-cycle the hub port before EACH side, take 3 SDR samples per
# margin, alternate A/B/A/B so rail drift shows up as within-side variance rather
# than a spurious between-side gap. MCS7/20 1465B, ch36.
set -u
KMOD=rtl88x2eu_ohd
HUB=3-2.3
FREQ=5180e6
cleanup(){ sudo pkill -9 -x WiFiDriverTxDem 2>/dev/null; sudo pkill -9 -f kernel_tx_inject 2>/dev/null; }
trap cleanup EXIT

cycle(){ sudo pkill -9 -x WiFiDriverTxDem 2>/dev/null; sudo pkill -9 -f kernel_tx_inject 2>/dev/null
         sudo rmmod $KMOD 2>/dev/null; sleep 1; sudo uhubctl -l $HUB -a cycle >/dev/null 2>&1; sleep 4; }

sweep(){ # prints "8:xx 10:xx 12:xx 14:xx"
  local out=""
  for m in 8 10 12 14; do
    local best=0
    for i in 1 2 3; do
      local d=$(sudo python3 tests/sdr_duty.py --freq $FREQ --rate 25e6 --secs 2 --gain 60 --mcs 7 --bw 20 --margin-db $m 2>/dev/null | grep -oiE "duty=[0-9.]+%" | grep -oE "[0-9.]+")
      # keep the max across samples (rail dips only lower it)
      awk "BEGIN{exit !($d>$best)}" && best=$d
    done
    out="$out ${m}dB:${best}%"
  done
  echo "$out"
}

meas_devourer(){
  cycle
  sudo env DEVOURER_VID=0x0bda DEVOURER_PID=0xa81a DEVOURER_CHANNEL=36 \
    DEVOURER_TX_RATE=MCS7 DEVOURER_TX_GAP_US=0 DEVOURER_TX_PAYLOAD_BYTES=1465 \
    stdbuf -oL timeout -k 5 30 build/WiFiDriverTxDemo >/dev/null 2>&1 &
  sleep 5
  echo "  devourer:$(sweep)"
  sudo pkill -9 -x WiFiDriverTxDem 2>/dev/null; sleep 1
}

meas_kernel(){
  cycle
  sudo insmod reference/rtl88x2eu/${KMOD}.ko 2>/dev/null; sleep 4
  local IF=""
  for d in /sys/class/net/*; do
    [ "$(basename "$(readlink -f "$d/device/driver" 2>/dev/null)" 2>/dev/null)" = "$KMOD" ] && IF=$(basename "$d")
  done
  [ -z "$IF" ] && { echo "  kernel: NO IFACE"; return; }
  sudo ip link set "$IF" up; sudo iw "$IF" set type monitor 2>/dev/null; sudo iw dev "$IF" set channel 36 HT20 2>/dev/null; sleep 1
  sudo python3 tests/kernel_tx_inject.py "$IF" 7 1465 30 >/tmp/eu_ktx.log 2>&1 &
  sleep 4
  echo "  kernel:  $(sweep)"
  sudo pkill -9 -f kernel_tx_inject 2>/dev/null; sudo rmmod $KMOD 2>/dev/null; sleep 1
}

echo "=== EU MCS7 TX-power A/B (fresh rail per side, max of 3 samples/margin) ==="
echo "[round 1]"; meas_devourer; meas_kernel
echo "[round 2]"; meas_kernel; meas_devourer
