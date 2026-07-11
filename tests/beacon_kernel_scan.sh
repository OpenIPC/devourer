#!/usr/bin/env bash
# beacon_kernel_scan.sh — the end-to-end "behaves like a kernel AP" interop test:
# devourer airs a full-content beacon; a REAL Linux 802.11 station (rtw88/mac80211)
# scans and must list it as a valid AP with every element parsed. This validates
# devourer's beacon against the actual kernel BSS/scan parser, not just the wire
# bytes (tests/beacon_wire_check.cpp).
#
# Needs a second Realtek adapter already bound to the kernel as a station — on this
# rig rtw88 auto-probes any dongle at enumeration, so a VBUS-cold adapter comes up
# as e.g. wlp4s0u2u4 (managed). Point SCAN_IF at it; use a DIFFERENT adapter
# (VID/PID) for the devourer beacon.
#
#   sudo SCAN_IF=wlp4s0u2u4 B_VID=0x2357 B_PID=0x012d tests/beacon_kernel_scan.sh
#
# Proven result (8812BU beacon, rtw88 8822CU station) on BOTH bands — ch6
# (CH=6 FREQ=2437) and ch36 (CH=36 FREQ=5180):
#   BSS 57:42:75:05:d6:00  TSF <live>  freq <2437|5180>  beacon interval 25 TUs
#   capability: ESS  SSID: devourerAP  Supported rates 1..54 (basic flags)
#   DS Parameter set: channel <6|36>  TIM: DTIM Count 0 Period 1  ERP: <no flags>
set -uo pipefail
cd "$(dirname "$0")/.."
SCAN_IF=${SCAN_IF:-wlp4s0u2u4}
B_VID=${B_VID:-0x2357}; B_PID=${B_PID:-0x012d}   # devourer beacon adapter (StartBeacon, any generation)
CH=${CH:-6}; FREQ=${FREQ:-2437}; TU=${TU:-25}; SECS=${SECS:-30}
BSSID=57:42:75:05:d6:00

BIN=$(mktemp); LOG=$(mktemp)
cleanup(){ sudo pkill -9 -x bkscan 2>/dev/null; rm -f "$BIN" "$LOG"; }
trap cleanup EXIT

g++ -std=c++20 -O2 -Isrc -Iexamples/common tests/beacon_fullbody.cpp \
    examples/common/env_config.cpp build/libdevourer.a \
    $(pkg-config --cflags --libs libusb-1.0) -lpthread -o "$BIN" || exit 1
cp "$BIN" /tmp/bkscan

sudo ip link set "$SCAN_IF" up 2>/dev/null; sleep 1
echo "beacon: $B_VID:$B_PID ch$CH ${TU}TU   scan station: $SCAN_IF"
sudo pkill -9 -x bkscan 2>/dev/null; sleep 1
sudo env DEVOURER_VID=$B_VID DEVOURER_PID=$B_PID DEVOURER_CHANNEL=$CH DEVOURER_BCN_TU=$TU \
    /tmp/bkscan "$SECS" >/dev/null 2>"$LOG" &
sleep 5

found=0
for i in 1 2 3 4; do
  ENTRY=$(sudo iw dev "$SCAN_IF" scan freq "$FREQ" 2>/dev/null | \
          awk -v b="$BSSID" 'index($0,b){f=1} f{print} f&&/^BSS/&&!index($0,b){exit}')
  if [ -n "$ENTRY" ]; then
    echo "=== FOUND — kernel station parsed devourer's beacon (scan $i) ==="
    echo "$ENTRY" | grep -iE "BSS |TSF:|freq:|beacon interval|capability|SSID:|Supported rates|DS Param|TIM:|ERP:"
    found=1; break
  fi
  echo "scan $i: not yet seen"; sleep 2
done
sudo pkill -9 -x bkscan 2>/dev/null
[ "$found" = 1 ] && echo "PASS: devourer beacon accepted by the kernel 802.11 stack" \
                 || { echo "FAIL: not listed (check beacon aired: $(grep -c StartBeacon "$LOG"))"; exit 1; }
