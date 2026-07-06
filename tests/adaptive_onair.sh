#!/usr/bin/env bash
# On-air closed-loop adaptive link: VTX (8812) <-> VRX (8821), both running
# duplex driven by adaptive_link.py. The VTX streams synthetic video and
# applies the VRX's RCF feedback (SET_PWR / SET_RATE) live; the VRX scores RSSI/SNR
# and commands the energy-min operating point. An optional B210 interferer
# (USE_INTERFERER=1) drives the link into the corrupt regime so the controller
# visibly drops to a more robust profile.
#
# WITNESS (no extra instrumentation): the *peer's* <devourer-stream> rate= changes
# when a SET_RATE lands, and rssi= rises when a SET_PWR raises power.
#
#   sudo bash tests/adaptive_onair.sh                 # steady-state adaptation
#   USE_INTERFERER=1 IGAIN=75 sudo bash tests/adaptive_onair.sh
#   RENDEZVOUS=1 sudo bash tests/adaptive_onair.sh    # VTX on wrong channel -> discovery
# SKIP_RAIL=1 after a clean boot.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PREC="$ROOT/tools/precoder"
REAL_HOME=$(getent passwd "${SUDO_USER:-$USER}" | cut -d: -f6)

VTX_PID=${VTX_PID:-0x8812}; VTX_VID=${VTX_VID:-0x0bda}; VTX=${VTX_SYSFS:-9-2}
VRX_PID=${VRX_PID:-0x0120}; VRX_VID=${VRX_VID:-0x2357}; VRX=${VRX_SYSFS:-9-1.4}
CH=${CH:-6}; SECS=${SECS:-30}; VTX_ID=${VTX_ID:-0xABCD}
USE_INTERFERER=${USE_INTERFERER:-}; IGAIN=${IGAIN:-75}; IMODE=${IMODE:-noise}
RENDEZVOUS=${RENDEZVOUS:-}

VTX_LOG=/tmp/adaptive_vtx.log; VRX_LOG=/tmp/adaptive_vrx.log
VIDEO=/tmp/adaptive_video.bin

KILL(){ sudo pkill -9 -f adaptive_link 2>/dev/null; sudo pkill -9 duplex 2>/dev/null
        sudo pkill -9 -f sdr_interferer 2>/dev/null; }
trap KILL EXIT

# free both Wi-Fi adapters (B210 is uhd-accessed)
for D in "$VTX" "$VRX"; do
  for i in /sys/bus/usb/devices/$D/$D:*; do
    ifc=$(basename "$i"); drv=$(readlink -f "$i/driver" 2>/dev/null)
    [ -n "$drv" ] && echo "$ifc" | sudo tee "$drv/unbind" >/dev/null 2>&1
  done
done; sleep 1

head -c 4000000 /dev/urandom > "$VIDEO"   # synthetic "video" bytes

if [ -n "$USE_INTERFERER" ]; then
  echo "=== B210 interferer ch$CH gain=$IGAIN mode=$IMODE (warming up) ==="
  sudo python3 "$ROOT/tests/sdr_interferer.py" --channel $CH --tx-gain "$IGAIN" \
       --rate 20e6 --mode "$IMODE" --secs $((SECS + 30)) >/tmp/intf.log 2>&1 &
  sleep 11
fi

# VRX channel: RENDEZVOUS puts the VTX on a WRONG channel so it must rediscover.
VTX_CH=$CH; [ -n "$RENDEZVOUS" ] && VTX_CH=$((CH + 5))

echo "=== VRX (8821) adaptive_link on ch$CH ==="
sudo env DEVOURER_VID=$VRX_VID DEVOURER_PID=$VRX_PID PYTHONPATH="$PREC" \
     python3 "$PREC/adaptive_link.py" --role vrx --pid $VRX_PID --channel $CH \
     --vtx-id $VTX_ID --duplex "$ROOT/build/duplex" >"$VRX_LOG" 2>&1 &
sleep 6
echo "=== VTX (8812) adaptive_link on ch$VTX_CH ==="
sudo env DEVOURER_VID=$VTX_VID DEVOURER_PID=$VTX_PID PYTHONPATH="$PREC" \
     python3 "$PREC/adaptive_link.py" --role vtx --pid $VTX_PID --channel $VTX_CH \
     --vtx-id $VTX_ID --video "$VIDEO" --duplex "$ROOT/build/duplex" >"$VTX_LOG" 2>&1 &

sleep "$SECS"; KILL; sleep 1

echo "=== RESULT ==="
echo "[vrx] video frames heard from VTX (rx hits): $(grep -oP 'rx hits=\K\d+' "$VRX_LOG" | tail -1)"
echo "[vtx] RCF applied: SET_PWR=$(grep -c 'ctl op=1' "$VTX_LOG") SET_RATE=$(grep -c 'ctl op=2' "$VTX_LOG")"
echo "[vrx] controller trajectory (1 Hz):"
grep '<adaptive-vrx>' "$VRX_LOG" | tail -8
echo "[vtx] applied-state trajectory (1 Hz):"
grep '<adaptive-vtx>' "$VTX_LOG" | tail -8
echo "logs: $VRX_LOG  $VTX_LOG"
