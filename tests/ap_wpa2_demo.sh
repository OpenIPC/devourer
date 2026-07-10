#!/usr/bin/env bash
# ap_wpa2_demo.sh — the full encrypted "devourer behaves like a kernel WPA2 AP"
# demo: bring devourer up as a WPA2-PSK AP (tests/ap_wpa2.cpp), associate a REAL
# Linux station (rtw88), lease an IP over ENCRYPTED DHCP, and PING over WPA2/CCMP.
# Proven (in pieces): 4-way "Key negotiation completed [PTK=CCMP GTK=CCMP]",
# dhcpcd "leased 192.168.99.2", ping 0% loss ~2.5 ms — all CCMP-encrypted.
#
# Needs a second Realtek adapter bound to the kernel as a station (rtw88
# auto-probes a VBUS-cold dongle -> e.g. wlp4s0u2u4). AP adapter = a DIFFERENT
# full-duplex J2/J3 devourer adapter (8812BU/8822CU). openssl (-lcrypto) required.
#
# NOTE: a single clean end-to-end run is flaky when the AP adapter has no VBUS
# reset (an xhci root-hub port degrades after many cycles) — prefer a
# VBUS-cyclable AP, and cold-cycle the station between runs.
#
#   sudo STA_IF=wlp4s0u2u4 AP_VID=0x2357 AP_PID=0x012d tests/ap_wpa2_demo.sh
set -uo pipefail
cd "$(dirname "$0")/.."
STA_IF=${STA_IF:-wlp4s0u2u4}
AP_VID=${AP_VID:-0x2357}; AP_PID=${AP_PID:-0x012d}
CH=${CH:-6}; TU=${TU:-25}; PSK=${PSK:-devourer123}; APIP=192.168.99.1

BIN=$(mktemp); WPA=$(mktemp); LOG=$(mktemp)
cleanup(){ sudo pkill -9 -x apw_demo wpa_supplicant dhcpcd 2>/dev/null
  sudo dhcpcd -x "$STA_IF" 2>/dev/null; sudo ip addr flush dev "$STA_IF" 2>/dev/null
  sudo iw dev "$STA_IF" disconnect 2>/dev/null; rm -f "$BIN" "$WPA" "$LOG"; }
trap cleanup EXIT

g++ -std=c++20 -O2 -Isrc -Iexamples/common tests/ap_wpa2.cpp \
    examples/common/env_config.cpp build/libdevourer.a \
    $(pkg-config --cflags --libs libusb-1.0) -lcrypto -lpthread -o "$BIN" || exit 1
cp "$BIN" /tmp/apw_demo
printf 'network={\n\tssid="devourerAP"\n\tpsk="%s"\n\tkey_mgmt=WPA-PSK\n\tproto=RSN\n\tpairwise=CCMP\n\tgroup=CCMP\n\tscan_ssid=1\n}\n' "$PSK" > "$WPA"

sudo pkill -9 -x apw_demo wpa_supplicant dhcpcd 2>/dev/null; sudo iw dev "$STA_IF" disconnect 2>/dev/null; sleep 2
echo "WPA2 AP: $AP_VID:$AP_PID ch$CH ${TU}TU PSK='$PSK'   station: $STA_IF"
sudo env DEVOURER_VID=$AP_VID DEVOURER_PID=$AP_PID DEVOURER_CHANNEL=$CH DEVOURER_WPA2_PSK="$PSK" \
    DEVOURER_BCN_TU=$TU DEVOURER_TX_WITH_RX=thread /tmp/apw_demo 55 2>"$LOG" &
sleep 6
sudo wpa_supplicant -i "$STA_IF" -c "$WPA" -B >/dev/null 2>&1
ok=0; for i in $(seq 1 16); do
  sudo wpa_cli -i "$STA_IF" status 2>/dev/null | grep -q COMPLETED && { echo "-- WPA2 key negotiation completed (${i}s)"; ok=1; break; }; sleep 1; done
if [ "$ok" != 1 ]; then echo "FAIL: 4-way did not complete (bench flaky / AP degraded)"; grep -iE "4-WAY|msg" "$LOG" | tail -3; exit 1; fi

sudo ip addr flush dev "$STA_IF" 2>/dev/null
echo "-- encrypted DHCP:"; sudo timeout 16 dhcpcd -4 -1 -t 14 "$STA_IF" 2>&1 | grep -iE "leased" | sed 's/^/     /'
echo "-- station IP: $(ip -4 -o addr show "$STA_IF" | grep -oE '192\.168\.99\.[0-9]+' | head -1)"
sudo ping -c 1 -W 2 -I "$STA_IF" "$APIP" >/dev/null 2>&1
echo "-- encrypted ping (WPA2/CCMP):"; sudo ping -c 6 -W 1 -I "$STA_IF" "$APIP" | tail -3
