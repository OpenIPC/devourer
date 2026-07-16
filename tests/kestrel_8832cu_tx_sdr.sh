#!/usr/bin/env bash
# kestrel_8832cu_tx_sdr.sh — authoritative on-air TX validation for the
# RTL8832CU (C8852C, 35bc:0101) via the B210 SDR (channel-occupancy / duty,
# ceiling-free). The 8852C needs the T-MAC path-com routing block
# (halbb_ctrl_tx_path_tmac_8852c, 0xD800..0xD82C) that the 8852B-derived
# bring-up omitted; without it the RF synth locks but nothing radiates. This
# floods ch$CH with devourer and takes a single SDR duty read — a clear jump
# above the ambient noise floor == the 8832CU is finally on air.
#
#   sudo tests/kestrel_8832cu_tx_sdr.sh [channel] [seconds] [rate]
set -u
cd "$(dirname "$0")/.."
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }
CH=${1:-36}; SECS=${2:-4}; RATE=${3:-6M}
if [ "$CH" -le 14 ]; then FREQ=$(( 2407 + CH*5 ))e6   # 2.4 GHz
else FREQ=$(( 5000 + CH*5 ))e6; fi                    # 5 GHz: 5000 + ch*5
TX_ID="35bc:0101"; TX_HUB="3-2.3"; TX_PORT="1"       # 8832CU VBUS map
TXLOG="/tmp/kestrel_8832cu_tx_sdr_tx.log"
[ -x build/txdemo ] || { echo "FAIL: build txdemo"; exit 2; }

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
cleanup() {
  [ -n "$TXPID" ] && kill "$TXPID" 2>/dev/null
  pkill -9 -x -f "build/txdemo" 2>/dev/null || true
}
trap cleanup EXIT

# ONE SDR read per session (a second back-to-back sdr_duty read can fail to
# reacquire the B210 and report ~0). Bring the flood up first, then read once.
echo ">> freq ${FREQ} (ch$CH)"
echo ">> [1/2] bring up devourer TX flooding ch$CH (gap=0)"
uhubctl -l "$TX_HUB" -p "$TX_PORT" -a cycle -d 2 >/dev/null 2>&1
# 8 s settle: the 8832CU is slow to re-enumerate after a VBUS cold cycle.
sleep 8
sysdir_for "$TX_ID" >/dev/null || { echo "FAIL: $TX_ID not on bus"; exit 2; }
unbind_kernel "$TX_ID"; sleep 1
env DEVOURER_VID=0x35bc DEVOURER_PID=0x0101 DEVOURER_CHANNEL=$CH \
  DEVOURER_TX_GAP_US=0 DEVOURER_LOG_LEVEL=warn DEVOURER_TX_RATE="$RATE" \
  build/txdemo >"$TXLOG" 2>&1 &
TXPID=$!
sleep 4
kill -0 "$TXPID" 2>/dev/null || { echo "FAIL: txdemo died"; tail -8 "$TXLOG"; exit 2; }

echo ">> [2/2] single SDR duty read WITH devourer flooding"
LIVE=$(python3 tests/sdr_duty.py --freq "$FREQ" --secs "$SECS" --mcs 0 --bw 20 2>/dev/null \
        | grep -iE "duty" | tail -1)

kill "$TXPID" 2>/dev/null; TXPID=""
echo "=================================================================="
echo "live (devourer flooding): ${LIVE:-<none>}"
echo "Bench ambient noise floor is ~18% duty; a live duty well above that (e.g."
echo ">50%) == the 8832CU is radiating (T-MAC path-com fix effective)."
