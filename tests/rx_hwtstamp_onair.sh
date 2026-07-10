#!/usr/bin/env bash
# rx_hwtstamp_onair.sh — end-to-end on-air VALUE validation of the
# CONFIG_RTW_HWTSTAMP prototype (reference/rtl88x2cu): read the MAC RX TSF back
# through SO_TIMESTAMPING off real over-the-air traffic, no external AP/secrets.
#
# Closed loop, entirely on this rig:
#   AP      = a devourer OPEN AP (tests/ap_responder.cpp) on a J2/J3 adapter
#             (default 8822BU 2357:012d) — beacons + DHCP + ICMP responder.
#   STATION = the 8822CU (0bda:c812) driven by the PATCHED vendor driver.
# The station associates (open), gets a DHCP lease from devourer, pings the AP,
# and tests/rx_hwtstamp_probe reads the raw-hardware RX timestamp off the ICMP
# replies. Advancing ~µs-granular timestamps = the MAC TSF surfaced through the
# stock kernel API — the honest apples-vs-apples half of docs/timing-accuracy.md.
#
#   sudo tests/rx_hwtstamp_onair.sh
set -uo pipefail
cd "$(dirname "$0")/.."
KO=reference/rtl88x2cu/88x2cu_ohd.ko
STA_PID=c812                       # 8822CU station (patched driver)
AP_VID=${AP_VID:-0x2357}; AP_PID=${AP_PID:-0x012d}   # 8822BU devourer AP
CH=${CH:-6}; TU=${TU:-25}; APIP=192.168.99.1
STA_IF=""

log(){ echo "[hwts-onair] $*"; }
cleanup(){
  log "cleanup"
  sudo pkill -9 -x apr_hwts wpa_supplicant rx_hwtstamp_probe 2>/dev/null
  [ -n "$STA_IF" ] && { sudo ip addr flush dev "$STA_IF" 2>/dev/null; sudo iw dev "$STA_IF" disconnect 2>/dev/null; sudo dhcpcd -x "$STA_IF" 2>/dev/null; }
  sudo rmmod 88x2cu_ohd 2>/dev/null
  sudo modprobe rtw88_8822cu 2>/dev/null; sudo modprobe rtw88_8822bu 2>/dev/null
  rm -f /tmp/apr_hwts "$WPA" "$LOG" 2>/dev/null
}
WPA=$(mktemp); LOG=$(mktemp)
trap cleanup EXIT

# --- build probe + AP responder ---
mkdir -p build
cc -O2 -o build/rx_hwtstamp_probe tests/rx_hwtstamp_probe.c || { log "probe build failed"; exit 1; }
[ -f build/libdevourer.a ] || { log "build/libdevourer.a missing — cmake --build build first"; exit 1; }
g++ -std=c++20 -O2 -Isrc -Iexamples/common tests/ap_responder.cpp \
    examples/common/env_config.cpp build/libdevourer.a \
    $(pkg-config --cflags --libs libusb-1.0) -lpthread -o /tmp/apr_hwts || { log "ap build failed"; exit 1; }
[ -f "$KO" ] || { log "missing $KO — build the patched module first"; exit 1; }

# --- load patched driver on the station (8822CU) ---
sudo pkill -9 -x apr_hwts wpa_supplicant 2>/dev/null
log "loading patched $KO on 8822CU"
sudo modprobe -r rtw88_8822cu 2>/dev/null; sleep 1
sudo insmod "$KO" || { log "insmod failed"; exit 1; }
sleep 3
for i in $(seq 1 10); do
  STA_IF=$(for d in /sys/class/net/*; do drv=$(readlink -f "$d/device/driver" 2>/dev/null); case "$drv" in *88x2cu*) basename "$d";; esac; done | head -1)
  [ -n "$STA_IF" ] && break; sleep 1
done
[ -z "$STA_IF" ] && { log "no station netdev"; exit 1; }
log "station iface: $STA_IF"
sudo ip link set "$STA_IF" up; sleep 1
sudo ethtool -T "$STA_IF" 2>&1 | grep -qi hardware-receive && log "capability OK (hardware-receive advertised)" || { log "capability FAIL"; exit 1; }

# --- start devourer open AP on the 8822BU ---
printf 'network={\n\tssid="devourerAP"\n\tkey_mgmt=NONE\n\tscan_ssid=1\n}\n' > "$WPA"
log "starting devourer open AP $AP_VID:$AP_PID ch$CH"
sudo env DEVOURER_VID=$AP_VID DEVOURER_PID=$AP_PID DEVOURER_CHANNEL=$CH DEVOURER_BCN_TU=$TU \
    DEVOURER_TX_WITH_RX=thread /tmp/apr_hwts 60 2>"$LOG" &
sleep 5

# --- associate (open) + DHCP ---
sudo wpa_supplicant -i "$STA_IF" -c "$WPA" -B >/dev/null 2>&1
for i in $(seq 1 15); do
  sudo iw dev "$STA_IF" link 2>&1 | grep -qE 'Connected to' && { log "associated (after ${i}s)"; break; }
  sleep 1
done
sudo ip addr flush dev "$STA_IF" 2>/dev/null
if command -v dhcpcd >/dev/null; then sudo timeout 20 dhcpcd -4 -1 -t 18 "$STA_IF" 2>&1 | grep -iE "offered|leased" | sed 's/^/-- /'
else sudo ip addr add 192.168.99.2/24 dev "$STA_IF"; fi
log "station IP: $(ip -4 -o addr show "$STA_IF" | grep -oE '192\.168\.99\.[0-9]+' | head -1)"

# --- generate RX traffic (ICMP replies land as data frames on the station) ---
sudo ping -c1 -W2 -I "$STA_IF" "$APIP" >/dev/null 2>&1   # warm ARP
( sudo ping -i 0.05 -c 200 -W1 -I "$STA_IF" "$APIP" >/dev/null 2>&1 ) &

# --- read RX hardware timestamps off that traffic ---
log "=== rx_hwtstamp_probe $STA_IF ==="
sudo ./build/rx_hwtstamp_probe "$STA_IF" 20
RC=$?
log "on-air value check rc=$RC"
exit $RC
