#!/usr/bin/env bash
# ap_ping_demo.sh — the full "devourer behaves like a kernel AP" demo: bring
# devourer up as an open AP (tests/ap_responder.cpp), associate a REAL Linux
# station (rtw88) to it, and PING the AP over the air. Proven: 0% loss, ~2 ms RTT.
#
# Needs a second Realtek adapter bound to the kernel as a station (rtw88
# auto-probes a VBUS-cold dongle -> e.g. wlp4s0u2u4). AP adapter = a DIFFERENT
# J2/J3 devourer adapter (StartBeacon + full-duplex; 8812BU/8822CU work).
#
#   sudo STA_IF=wlp4s0u2u4 AP_VID=0x2357 AP_PID=0x012d tests/ap_ping_demo.sh
set -uo pipefail
cd "$(dirname "$0")/.."
STA_IF=${STA_IF:-wlp4s0u2u4}
AP_VID=${AP_VID:-0x2357}; AP_PID=${AP_PID:-0x012d}
CH=${CH:-6}; TU=${TU:-25}; APIP=192.168.99.1; STAIP=192.168.99.2

BIN=$(mktemp); WPA=$(mktemp); LOG=$(mktemp)
cleanup(){ sudo pkill -9 -x apr_ping wpa_supplicant 2>/dev/null
  sudo ip addr flush dev "$STA_IF" 2>/dev/null; sudo iw dev "$STA_IF" disconnect 2>/dev/null
  rm -f "$BIN" "$WPA" "$LOG"; }
trap cleanup EXIT

g++ -std=c++20 -O2 -Isrc -Iexamples/common tests/ap_responder.cpp \
    examples/common/env_config.cpp build/libdevourer.a \
    $(pkg-config --cflags --libs libusb-1.0) -lpthread -o "$BIN" || exit 1
cp "$BIN" /tmp/apr_ping
printf 'network={\n\tssid="devourerAP"\n\tkey_mgmt=NONE\n\tscan_ssid=1\n}\n' > "$WPA"

sudo pkill -9 -x apr_ping wpa_supplicant 2>/dev/null; sudo iw dev "$STA_IF" disconnect 2>/dev/null; sleep 2
echo "AP: $AP_VID:$AP_PID ch$CH ${TU}TU (BSSID 02:42:75:05:d6:00)   station: $STA_IF"
sudo env DEVOURER_VID=$AP_VID DEVOURER_PID=$AP_PID DEVOURER_CHANNEL=$CH DEVOURER_BCN_TU=$TU \
    DEVOURER_TX_WITH_RX=thread /tmp/apr_ping 45 2>"$LOG" &
sleep 5
sudo wpa_supplicant -i "$STA_IF" -c "$WPA" -B >/dev/null 2>&1
# poll until associated (up to ~15 s) before pinging
for i in $(seq 1 15); do
  L=$(sudo iw dev "$STA_IF" link 2>&1 | grep -oE 'Connected to [0-9a-f:]+') && { echo "-- link: $L (after ${i}s)"; break; }
  sleep 1
done
sudo ip addr flush dev "$STA_IF" 2>/dev/null; sudo ip addr add "$STAIP/24" dev "$STA_IF"
sudo ping -c 1 -W 2 -I "$STA_IF" "$APIP" >/dev/null 2>&1   # warm ARP
echo "-- ping $APIP from $STA_IF --"
sudo ping -c 6 -W 1 -I "$STA_IF" "$APIP" | tail -6
echo "-- AP handshake+data log --"
grep -iE "AUTH req|ASSOC req|data\(" "$LOG" | tail -4
