#!/usr/bin/env bash
# kestrel_he_onair.sh — M5 HE-rate on-air validation with an AX sniffer.
# devourer txdemo transmits an 802.11ax HE PPDU on the RTL8852BU (35bc:0108,
# DEVOURER_TX_RATE=HE<N>SS_MCS<M>), and the second AX adapter — the RTL8832CU
# (35bc:0101) under the vendor 8852cu.ko in monitor mode — decodes it. tshark
# reports the received PHY type: wlan_radio.phy == 11 (11ax) on the canonical SA
# is definitive proof the 8852BU aired a real HE PPDU (an 11ac witness cannot
# even detect an HE frame, hence the AX sniffer).
#
#   sudo tests/kestrel_he_onair.sh [channel] [rate] [seconds]
set -u
cd "$(dirname "$0")/.."
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }
CH=${1:-6}; RATE=${2:-HE1SS_MCS7/20}; DUR=${3:-8}
SA="57:42:75:05:d6:00"
TX_ID="35bc:0108"      # RTL8852BU (devourer txdemo)
SNIFF_ID="35bc:0101"   # RTL8832CU (vendor 8852cu.ko monitor sniffer)
TX_HUB="3-2.3"; TX_PORT="3"
KO="reference/rtl8852cu/8852cu.ko"
CAP="/tmp/kestrel_he_cap.txt"; TXLOG="/tmp/kestrel_he_tx.log"
[ -x build/txdemo ] || { echo "FAIL: build txdemo"; exit 2; }
[ -f "$KO" ] || { echo "FAIL: missing $KO"; exit 2; }

sysdir_for(){ local vid=${1%%:*} pid=${1##*:} d; for d in /sys/bus/usb/devices/*; do
  [ -f "$d/idVendor" ]||continue; [ "$(cat "$d/idVendor")" = "$vid" ]&&[ "$(cat "$d/idProduct")" = "$pid" ]&&{ echo "$d";return;};done; }
unbind_kernel(){ local d i; d=$(sysdir_for "$1"); [ -n "$d" ]&&for i in "$d":*; do
  [ -L "$i/driver" ]&&echo "$(basename "$i")">"$(readlink -f "$i/driver")/unbind" 2>/dev/null||true; done; }

MON=""; TSHARK_PID=""; TXPID=""
cleanup(){
  [ -n "$TXPID" ]&&kill "$TXPID" 2>/dev/null; pkill -9 -f build/txdemo 2>/dev/null||true
  [ -n "$TSHARK_PID" ]&&kill "$TSHARK_PID" 2>/dev/null
  [ -n "$MON" ]&&{ ip link set "$MON" down 2>/dev/null; iw dev "$MON" del 2>/dev/null; }
  rmmod 8852cu 2>/dev/null||true
}
trap cleanup EXIT

echo ">> VBUS-cycling the 8852BU TX"
uhubctl -l "$TX_HUB" -p "$TX_PORT" -a cycle -d 2 >/dev/null 2>&1; sleep 6

echo ">> loading vendor 8852cu.ko for the AX sniffer ($SNIFF_ID)"
rmmod 8852cu 2>/dev/null||true
insmod "$KO" rtw_monitor_overwrite=1 2>/dev/null || insmod "$KO" 2>/dev/null || { echo "FAIL: insmod 8852cu"; exit 2; }
sleep 5
# find the vendor netdev for the 8832CU
BASE=""; d=$(sysdir_for "$SNIFF_ID")
for i in "$d":*; do [ -d "$i/net" ]&&BASE="$(ls "$i/net" 2>/dev/null|head -1)"; done
[ -n "$BASE" ] || { echo "FAIL: no netdev for 8832CU (vendor module didn't bind)"; dmesg|tail -5; exit 2; }
echo ">> sniffer netdev: $BASE"

# Convert the BASE iface itself to monitor type: down -> set type monitor -> up
# -> set channel. This runs the driver's full MAC/PHY + firmware init via the
# base iface open() AND makes that iface the channel controller. A separate mon0
# vif over a downed base leaves rtw_cfg80211_monitor_if_open() (a no-op) as the
# only "open", so the hardware stays uninitialised and the channel-set hardware
# poll (rtw_set_chbw_cmd) spins forever, wedging the kernel. Requires the driver
# monitor fixes (reference/rtl8852cu: cfg80211_register_netdevice deadlock + the
# monitor-RX NULL-deref + CONFIG_WIFI_MONITOR=y). MON is the base iface itself.
echo ">> setting $BASE to monitor type + channel $CH"
timeout 8 ip link set "$BASE" down 2>/dev/null
timeout 12 iw dev "$BASE" set type monitor 2>/dev/null || { echo "FAIL: set type monitor (rebuild driver with the monitor fixes)"; exit 2; }
timeout 15 ip link set "$BASE" up 2>/dev/null
sleep 3   # let the 8852c firmware download + hw init settle
MON="$BASE"
timeout 12 iw dev "$MON" set channel "$CH" 2>/dev/null || timeout 12 iw dev "$MON" set channel "$CH" HT20 2>/dev/null
echo ">> monitor $MON up on ch$CH (base $BASE kept up for hw init)"

# capture: SA, radiotap PHY type + data rate, for our canonical SA
echo ">> tshark capturing on $MON for ${DUR}s"
timeout $((DUR+4)) tshark -i "$MON" -l -n \
  -Y "wlan.sa == $SA" \
  -T fields -e wlan.sa -e wlan_radio.phy -e wlan_radio.data_rate -e wlan_radio.11ax.mcs \
  >"$CAP" 2>/dev/null &
TSHARK_PID=$!
sleep 2

echo ">> devourer txdemo HE on the 8852BU (rate=$RATE ch$CH ${DUR}s)"
unbind_kernel "$TX_ID"; sleep 1
env DEVOURER_VID=0x35bc DEVOURER_PID=0x0108 DEVOURER_CHANNEL=$CH \
  DEVOURER_TX_GAP_US=2000 DEVOURER_LOG_LEVEL=info DEVOURER_TX_RATE="$RATE" \
  build/txdemo >"$TXLOG" 2>&1 &
TXPID=$!
sleep "$DUR"
kill "$TXPID" 2>/dev/null; TXPID=""
sleep 3
kill "$TSHARK_PID" 2>/dev/null; TSHARK_PID=""

# --- verdict ---
TX_UP=$(grep -c "TX ready" "$TXLOG" 2>/dev/null); TX_UP=${TX_UP:-0}
FRAMES=$(grep -c "$SA" "$CAP" 2>/dev/null); FRAMES=${FRAMES:-0}
HE_FRAMES=$(awk -F'\t' '$2==11{n++} END{print n+0}' "$CAP" 2>/dev/null)
echo "=================================================================="
echo "M5 HE on-air (ch$CH rate=$RATE ${DUR}s):  txdemo TX-ready=$TX_UP"
echo "  sniffer frames from $SA: $FRAMES   of which 11ax(phy=11): $HE_FRAMES"
echo "  --- sample decoded rows (sa / phy / rate / he-mcs) ---"; head -4 "$CAP" 2>/dev/null
if [ "$TX_UP" -lt 1 ]; then
  echo "RESULT: TX bring-up FAILED"; tail -5 "$TXLOG"
elif [ "${HE_FRAMES:-0}" -gt 0 ]; then
  echo "RESULT: PASS — the 8852BU aired real HE (11ax) PPDUs, decoded by the AX sniffer."
elif [ "${FRAMES:-0}" -gt 0 ]; then
  echo "RESULT: frames received from SA but NOT decoded as 11ax — HE PHY not confirmed."
else
  echo "RESULT: no frames witnessed — check monitor channel / TX radiation."
fi
