#!/usr/bin/env bash
# kestrel_8832cu_tx_matrix.sh — full on-air TX matrix for the RTL8832CU
# (C8852C, 35bc:0101) via the B210 SDR. The 8832CU TX just came up (path-com
# routing + V1-descriptor rate word); this exercises it across the same span
# the 8852BU already passes — 2.4/5 GHz, 20/40/80 MHz, an HE rate, and 5/10 MHz
# narrowband — so a bandwidth- or rate-specific TX gap can't hide behind the
# single ch36/20MHz/6M smoke test. Each cell floods with devourer and takes ONE
# SDR duty read; a duty well above the ~18% ambient floor == radiating.
#
#   sudo tests/kestrel_8832cu_tx_matrix.sh
set -u
cd "$(dirname "$0")/.."
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }
TX_ID="35bc:0101"; TX_HUB="3-2.3"; TX_PORT="1"
TXLOG="/tmp/kestrel_8832cu_tx_matrix_tx.log"
[ -x build/txdemo ] || { echo "FAIL: build txdemo"; exit 2; }
THRESH=40   # duty % above which we call it radiating (ambient ~18%)

sysdir_for() {
  local vid=${1%%:*} pid=${1##*:} d
  for d in /sys/bus/usb/devices/*; do
    [ -f "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor")" = "$vid" ] && [ "$(cat "$d/idProduct")" = "$pid" ] \
      && { echo "$d"; return; }
  done
}
unbind_kernel() {
  local d i; d=$(sysdir_for "$1")
  [ -n "$d" ] && for i in "$d":*; do
    [ -L "$i/driver" ] && echo "$(basename "$i")" > "$(readlink -f "$i/driver")/unbind" 2>/dev/null || true
  done
}
TXPID=""
cleanup() { [ -n "$TXPID" ] && kill "$TXPID" 2>/dev/null; pkill -9 -x -f "build/txdemo" 2>/dev/null || true; }
trap cleanup EXIT

fails=0
# cell: $1=label $2=channel $3=rate $4=extra-env ("DEVOURER_HOP_BW=80" | "DEVOURER_NB_BW=5" | "")
cell() {
  local label="$1" ch="$2" rate="$3" extra="$4"
  local freq bwm=20
  if [ "$ch" -le 14 ]; then freq=$(( 2407 + ch*5 ))e6; else freq=$(( 5000 + ch*5 ))e6; fi
  case "$extra" in *HOP_BW=40*) bwm=40;; *HOP_BW=80*) bwm=80;; *NB_BW=5*) bwm=5;; *NB_BW=10*) bwm=10;; esac
  echo ">> $label (ch$ch ${bwm}MHz rate=$rate)"
  uhubctl -l "$TX_HUB" -p "$TX_PORT" -a cycle -d 2 >/dev/null 2>&1
  sleep 8
  sysdir_for "$TX_ID" >/dev/null || { echo "   FAIL: $TX_ID not on bus"; fails=$((fails+1)); return; }
  unbind_kernel "$TX_ID"; sleep 1
  # shellcheck disable=SC2086
  env DEVOURER_VID=0x35bc DEVOURER_PID=0x0101 DEVOURER_CHANNEL=$ch \
      DEVOURER_TX_GAP_US=0 DEVOURER_LOG_LEVEL=warn DEVOURER_TX_RATE="$rate" $extra \
      build/txdemo >"$TXLOG" 2>&1 &
  TXPID=$!
  sleep 4
  if ! kill -0 "$TXPID" 2>/dev/null; then
    echo "   FAIL: txdemo died"; tail -4 "$TXLOG"; fails=$((fails+1)); TXPID=""; return
  fi
  local nak; nak=$(grep -cE "bulk_send.*FAIL|not enabled|rc=-7" "$TXLOG"); nak=${nak:-0}
  local live; live=$(python3 tests/sdr_duty.py --freq "$freq" --secs 4 --mcs 0 --bw 20 2>/dev/null \
                     | grep -iE "duty" | tail -1)
  kill "$TXPID" 2>/dev/null; TXPID=""; sleep 1
  local duty; duty=$(printf '%s' "$live" | grep -oE "duty=[0-9.]+" | grep -oE "[0-9.]+")
  echo "   $live ; naks=$nak"
  if [ -z "$duty" ]; then echo "   FAIL: no SDR read"; fails=$((fails+1)); return; fi
  # bash has no float compare; integer-truncate the duty %
  local di=${duty%%.*}
  if [ "${di:-0}" -ge "$THRESH" ] && [ "$nak" -eq 0 ]; then echo "   PASS"; else echo "   FAIL"; fails=$((fails+1)); fi
}

cell "5GHz/20/6M"        36 6M                 ""
cell "5GHz/40/VHT-MCS0"  36 VHT1SS_MCS0/40     "DEVOURER_HOP_BW=40"
cell "5GHz/80/VHT-MCS0"  36 VHT1SS_MCS0/80     "DEVOURER_HOP_BW=80"
cell "5GHz/20/HE-MCS0"   36 HE1SS_MCS0/20      ""
cell "5GHz/5MHz-NB"      36 6M                 "DEVOURER_NB_BW=5"
cell "2.4GHz/20/6M"       6 6M                 ""

echo "=================================================================="
[ "$fails" -eq 0 ] && echo "RESULT: PASS — 8832CU TX radiates across the matrix" \
  || { echo "RESULT: FAIL — $fails cell(s)"; exit 1; }
