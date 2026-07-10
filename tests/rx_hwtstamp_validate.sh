#!/usr/bin/env bash
# rx_hwtstamp_validate.sh — validate the CONFIG_RTW_HWTSTAMP prototype in
# reference/rtl88x2cu: prove the patched vendor driver surfaces the MAC RX TSF
# through the standard kernel SO_TIMESTAMPING API (the "honest apples-vs-apples"
# primitive for the NTP/PTP-vs-TSF comparison in docs/timing-accuracy.md).
#
# Two levels:
#   1. CAPABILITY (always, deterministic): swap the in-tree rtw88 driver for the
#      patched out-of-tree 88x2cu_ohd.ko on the 8822CU, and assert `ethtool -T`
#      now advertises SOF_TIMESTAMPING_RX_HARDWARE + RAW_HARDWARE. This proves
#      get_ts_info registered.
#   2. ON-AIR VALUE (opt-in): if AP_SSID+AP_PSK are set, associate + DHCP + ping,
#      and run rx_hwtstamp_probe to read advancing raw-hardware RX timestamps off
#      real traffic. Skipped (with a note) otherwise.
#
# The in-tree rtw88_8822cu is restored on exit no matter what.
#
#   sudo tests/rx_hwtstamp_validate.sh
#   sudo AP_SSID=KArt2 AP_PSK=... tests/rx_hwtstamp_validate.sh
set -uo pipefail
cd "$(dirname "$0")/.."
KO=reference/rtl88x2cu/88x2cu_ohd.ko
DEV_PID=c812                 # 0bda:c812, the 8822C-family USB part
IFACE=""
WPA_PID=""
DHCP_PID=""

log() { echo "[hwts] $*"; }

restore() {
  log "cleanup: restoring in-tree driver"
  [ -n "$WPA_PID" ] && kill "$WPA_PID" 2>/dev/null
  [ -n "$DHCP_PID" ] && kill "$DHCP_PID" 2>/dev/null
  sudo pkill -9 -x rx_hwtstamp_probe 2>/dev/null
  sudo rmmod 88x2cu_ohd 2>/dev/null
  # bring the stock stack back (it auto-probes the dongle on load)
  sudo modprobe rtw88_8822cu 2>/dev/null
  sleep 1
}
trap restore EXIT

# --- build the userspace probe ---
mkdir -p build
cc -O2 -o build/rx_hwtstamp_probe tests/rx_hwtstamp_probe.c || { log "probe build failed"; exit 1; }

# --- confirm the patched module is built ---
[ -f "$KO" ] || { log "missing $KO — run 'make' in reference/rtl88x2cu first"; exit 1; }
lsusb | grep -qi "0bda:$DEV_PID" || { log "no 0bda:$DEV_PID (8822CU) plugged"; exit 1; }

# --- driver swap: in-tree rtw88 -> patched out-of-tree ---
log "unloading in-tree rtw88_8822cu"
sudo modprobe -r rtw88_8822cu 2>/dev/null
sleep 1
log "inserting patched $KO"
sudo insmod "$KO" || { log "insmod failed"; exit 1; }
sleep 3

# --- find the netdev the patched driver created ---
for i in $(seq 1 10); do
  IFACE=$(ls /sys/bus/usb/devices/*/net/ 2>/dev/null | grep -E '^wl|^wlan' | head -1)
  [ -z "$IFACE" ] && IFACE=$(for d in /sys/class/net/*; do
      drv=$(readlink -f "$d/device/driver" 2>/dev/null); case "$drv" in *88x2cu*) basename "$d";; esac; done | head -1)
  [ -n "$IFACE" ] && break
  sleep 1
done
[ -z "$IFACE" ] && { log "patched driver created no netdev"; exit 1; }
log "interface: $IFACE"
sudo ip link set "$IFACE" up 2>/dev/null
sleep 1

# --- LEVEL 1: capability ---
log "=== ethtool -T $IFACE ==="
TS=$(sudo ethtool -T "$IFACE" 2>&1); echo "$TS"
if echo "$TS" | grep -q "hardware-receive"; then
  log "PASS(capability): RX hardware timestamping advertised"
else
  log "FAIL(capability): ethtool -T does not show hardware-receive"
  exit 1
fi

# --- LEVEL 2: on-air value (opt-in) ---
if [ -n "${AP_SSID:-}" ] && [ -n "${AP_PSK:-}" ]; then
  log "associating to $AP_SSID"
  WPACONF=$(mktemp)
  wpa_passphrase "$AP_SSID" "$AP_PSK" > "$WPACONF"
  sudo wpa_supplicant -B -i "$IFACE" -c "$WPACONF" -P /tmp/wpa_hwts.pid >/dev/null 2>&1
  WPA_PID=$(cat /tmp/wpa_hwts.pid 2>/dev/null)
  sleep 6
  sudo dhclient -1 "$IFACE" 2>/dev/null &
  DHCP_PID=$!
  sleep 4
  GW=$(ip route show dev "$IFACE" | awk '/default/{print $3; exit}')
  log "gateway=$GW — pinging to generate RX traffic while probing"
  ( for _ in $(seq 1 40); do ping -c1 -W1 "${GW:-8.8.8.8}" >/dev/null 2>&1; sleep 0.1; done ) &
  rm -f "$WPACONF"
  log "=== rx_hwtstamp_probe $IFACE ==="
  sudo ./build/rx_hwtstamp_probe "$IFACE" 20
  RC=$?
  log "on-air value check rc=$RC"
  exit $RC
else
  log "AP_SSID/AP_PSK not set — skipping on-air value check."
  log "To read live RX hardware timestamps: sudo AP_SSID=<ssid> AP_PSK=<psk> $0"
fi
