#!/usr/bin/env bash
# kestrel_narrowband_sdr.sh — validate RTL8852BU 5/10 MHz narrowband TX by
# measuring the on-air occupied bandwidth on the B210. devourer TXes a
# continuous OFDM stream at each width on a quiet 5 GHz channel; sdr_spectrum.py
# reports occ_bw. A working narrowband path shrinks the lobe ~4x (20->5) / ~2x
# (20->10). The 20 MHz cell is the control.
#
#   sudo tests/kestrel_narrowband_sdr.sh [channel] [freq_hz]
set -u
cd "$(dirname "$0")/.."
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }
CH=${1:-149}; FREQ=${2:-5745e6}
TX_HUB="3-2.3"; TX_PORT="3"; TXLOG=/tmp/kestrel_nb_tx.log
[ -x build/txdemo ] || { echo "FAIL: build txdemo"; exit 2; }

sysdir() { for d in /sys/bus/usb/devices/*; do
  [ -f "$d/idVendor" ] && [ "$(cat "$d/idVendor")" = 35bc ] &&
  [ "$(cat "$d/idProduct")" = 0108 ] && { echo "$d"; return; }; done; }
unbind() { local d; d=$(sysdir); [ -n "$d" ] && for i in "$d":*; do
  [ -L "$i/driver" ] && echo "$(basename "$i")" \
    > "$(readlink -f "$i/driver")/unbind" 2>/dev/null || true; done; }
TXPID=""
cleanup() { [ -n "$TXPID" ] && kill "$TXPID" 2>/dev/null;
  pkill -9 -f build/txdemo 2>/dev/null || true; }
trap cleanup EXIT

measure() { sudo python3 tests/sdr_spectrum.py --freq "$FREQ" --duration 1.5 \
  2>/dev/null | grep -oE "occ_bw=[0-9.]+ MHz"; }

echo ">> ambient check at $FREQ (want a quiet channel)"
uhubctl -l "$TX_HUB" -p "$TX_PORT" -a off -d 2 >/dev/null 2>&1; sleep 2
echo "   ambient: $(measure)"
uhubctl -l "$TX_HUB" -p "$TX_PORT" -a on  -d 2 >/dev/null 2>&1; sleep 5

cell() { # $1=label $2=NB_BW-env(empty=20)
  local nb="$2"
  echo ">> cell $1 (ch$CH)"
  uhubctl -l "$TX_HUB" -p "$TX_PORT" -a cycle -d 2 >/dev/null 2>&1; sleep 6
  unbind; sleep 1
  env DEVOURER_VID=0x35bc DEVOURER_PID=0x0108 DEVOURER_CHANNEL=$CH \
      DEVOURER_TX_GAP_US=0 DEVOURER_TX_RATE=6M DEVOURER_LOG_LEVEL=info \
      ${nb:+DEVOURER_NB_BW=$nb} build/txdemo >"$TXLOG" 2>&1 & TXPID=$!
  sleep 4
  grep -qE "TX ready|bulk_send EP" "$TXLOG" && echo "   TX up" || \
    { echo "   TX FAILED"; tail -4 "$TXLOG"; }
  echo "   $1: $(measure)"
  kill "$TXPID" 2>/dev/null; TXPID=""; sleep 1
}

cell "20MHz (control)" ""
cell "10MHz narrowband" "10"
cell "5MHz narrowband"  "5"
echo "=================================================================="
echo "PASS if the narrowband cells' occ_bw shrinks vs the 20 MHz control."
