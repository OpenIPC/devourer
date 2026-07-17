#!/usr/bin/env bash
# ul_ofdma_e2e.sh — tier-C end-to-end for 802.11ax scheduled UL:
#   AP  = devourer HE access point on a Kestrel adapter (RTL8832CU, 35bc:0101),
#         tests/ul_trigger_ap.cpp — beacons HT+VHT+HE IEs, associates a station,
#         registers it as a peer, and drives the fw UL-OFDMA scheduler + Basic
#         Triggers granting it UL opportunities.
#   STA = a real kernel HE supplicant — the RTL8852BU (35bc:0108) under
#         rtw89_8852bu — connects to devourerAP with wpa_supplicant, then
#         generates UL traffic (DHCP + ping) so the AP fw has a BSR to grant.
#
# PASS  = the STA reaches "Connected to <BSSID>" (HE association to the devourer
#         AP works) AND the AP's ap.summary shows ul_rx > 0 (the STA transmits
#         uplink); ul_tb > 0 means HE TB PPDUs (the scheduled-UL response) were
#         observed.
# PARTIAL = associates but no UL / no TB — the fw accepts the grants but the STA
#           doesn't answer triggers (or the RX doesn't classify TB); still proves
#           the HE-AP + peer path.
# FAIL  = no association — iterate the HE IEs in ul_trigger_ap.cpp (dmesg shows
#         the supplicant's rejection reason).
#
#   sudo tests/ul_ofdma_e2e.sh [seconds] [channel]
set -u
cd "$(dirname "$0")/.."
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }

SEC=${1:-40}
CH=${2:-36}
AP_ID="35bc:0101"   # RTL8832CU — devourer HE AP
STA_ID="35bc:0108"  # RTL8852BU — rtw89 HE station
WIT_ID="0bda:8812"  # RTL8812AU — monitor witness (decodes aired Triggers)
SSID="devourerAP"
BIN="/tmp/ul_trigger_ap"
WPA="/tmp/ul_e2e_wpa.conf"
APLOG="/tmp/ul_ofdma_e2e_ap.log"
STALOG="/tmp/ul_ofdma_e2e_sta.log"
WITLOG="/tmp/ul_ofdma_e2e_wit.jsonl"

lsusb -d "$AP_ID" >/dev/null 2>&1 || { echo "SKIP: AP $AP_ID (8832CU) not plugged"; exit 0; }
lsusb -d "$STA_ID" >/dev/null 2>&1 || { echo "SKIP: STA $STA_ID (8852BU) not plugged"; exit 0; }

sysdir_for() {
  local vid=${1%%:*} pid=${1##*:} d
  for d in /sys/bus/usb/devices/*; do
    [ -f "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor")" = "$vid" ] && [ "$(cat "$d/idProduct")" = "$pid" ] \
      && { echo "$d"; return; }
  done
}
netdev_for() { local d; d=$(sysdir_for "$1"); [ -n "$d" ] && ls "$d"/*:*/net/ 2>/dev/null | head -1; }
unbind_kernel() {
  local d iface drv; d=$(sysdir_for "$1"); [ -n "$d" ] || return 0
  for iface in "$d":*; do
    [ -e "$iface/driver" ] || continue
    drv=$(basename "$(readlink "$iface/driver")")
    echo "$(basename "$iface")" > "/sys/bus/usb/drivers/$drv/unbind" 2>/dev/null || true
  done
}

STA_IF=""
cleanup() {
  pkill -9 -x ul_trigger_ap 2>/dev/null || true
  pkill -9 -x wpa_supplicant 2>/dev/null || true
  pkill -9 -x rxdemo 2>/dev/null || true
  [ -n "$STA_IF" ] && { ip addr flush dev "$STA_IF" 2>/dev/null; iw dev "$STA_IF" disconnect 2>/dev/null; }
  rm -f "$WPA"
}
trap cleanup EXIT INT TERM

echo "== ul_ofdma_e2e: AP 8832CU($AP_ID) <- STA 8852BU($STA_ID) ch$CH, ${SEC}s =="

# --- build the AP harness ---
[ -f build/libdevourer.a ] || cmake --build build -j --target devourer >/dev/null 2>&1
g++ -std=c++20 -O2 -Isrc -Iexamples/common tests/ul_trigger_ap.cpp \
    examples/common/env_config.cpp build/libdevourer.a \
    $(pkg-config --cflags --libs libusb-1.0) -lpthread -o "$BIN" || { echo "FAIL: build"; exit 1; }

# --- STA side: 8852BU under rtw89_8852bu ---
modprobe rtw89_8852bu 2>/dev/null || true
unbind_kernel "$AP_ID"       # free the AP dongle for devourer
sleep 2
STA_IF=$(netdev_for "$STA_ID")
[ -n "$STA_IF" ] || { echo "FAIL: 8852BU has no rtw89 netdev (STA)"; exit 1; }
echo "STA netdev: $STA_IF"
printf 'network={\n\tssid="%s"\n\tkey_mgmt=NONE\n\tscan_ssid=1\n}\n' "$SSID" > "$WPA"
pkill -9 -x wpa_supplicant 2>/dev/null; iw dev "$STA_IF" disconnect 2>/dev/null; sleep 1

# --- AP up (full-duplex, HE beacon + trigger loop) ---
# TRIG_GAP_MS=0: rely on the UL_FIXINFO tf_periodic scheduler (the production
# mechanism the fw airs autonomously) rather than the explicit F2P SendTrigger
# loop — F2P is the dead test path and its per-50ms H2C was timing out (rc=-7)
# and wedging the AP's CH12 queue.
env DEVOURER_VID=0x35bc DEVOURER_PID=0x0101 DEVOURER_CHANNEL="$CH" \
    DEVOURER_BCN_TU=25 DEVOURER_TX_WITH_RX=thread DEVOURER_TRIG_GAP_MS=0 \
    DEVOURER_LOG_LEVEL=info "$BIN" "$SEC" >/dev/null 2>"$APLOG" &
sleep 6
grep -q "HE AP up" "$APLOG" || { echo "FAIL: AP did not come up"; tail -5 "$APLOG"; exit 1; }

# --- optional monitor witness (8812AU): decodes any aired Trigger as rx.trigger,
# so we can tell "fw airs a trigger, STA didn't answer" from "fw aired nothing". ---
if lsusb -d "$WIT_ID" >/dev/null 2>&1; then
  unbind_kernel "$WIT_ID"
  env DEVOURER_PID=0x8812 DEVOURER_VID=0x0bda DEVOURER_CHANNEL="$CH" \
      DEVOURER_EVENTS=stdout DEVOURER_LOG_LEVEL=warn build/rxdemo >"$WITLOG" 2>/dev/null &
  echo "witness 8812AU on ch$CH (rx.trigger decode)"
  sleep 4
fi

# --- STA connects ---
dmesg -C 2>/dev/null || true
wpa_supplicant -i "$STA_IF" -c "$WPA" -B >/dev/null 2>&1
echo "-- waiting for HE association --"
LINK=""
for i in $(seq 1 20); do
  LINK=$(iw dev "$STA_IF" link 2>&1 | grep -oE 'Connected to [0-9a-f:]+') && { echo "   $LINK (after ${i}s)"; break; }
  sleep 1
done

# --- generate UL traffic so the fw has a BSR to grant ---
if [ -n "$LINK" ]; then
  ip addr flush dev "$STA_IF" 2>/dev/null
  ip addr add 192.168.99.2/24 dev "$STA_IF" 2>/dev/null
  ( for k in $(seq 1 20); do ping -c 1 -W 1 -I "$STA_IF" 192.168.99.1 >/dev/null 2>&1; sleep 0.3; done ) &
fi

# --- B210 ground truth: while the AP grants the associated STA, is any trigger
# on air? Strong bursts (>+20 dB) that appear only with the grant (vs the
# AP-beacon-only baseline) would be the aired trigger / the STA's TB PPDU. Runs
# on a QUIET channel so the co-located AP+STA stand out; needs the B210. ---
B210LOG="/tmp/ul_ofdma_e2e_b210.txt"; : > "$B210LOG"
if [ -n "$LINK" ] && uhd_find_devices 2>/dev/null | grep -qi B210; then
  echo "-- B210 burst scan on ch$CH during the grant window (8 s) --"
  python3 "$HOME/git/sdr2wifi/iq_burst_scan.py" --freq "$(( (5000 + 5*CH) ))e6" \
      --secs 8 --gain 60 2>/dev/null | grep -E "iq-scan|bursts/sec" | tee "$B210LOG" || true
fi

# --- let the trigger loop run, then collect ---
sleep $((SEC - 22 > 6 ? SEC - 22 : 6))
iw dev "$STA_IF" link > "$STALOG" 2>&1 || true
sleep 4
pkill -9 -x ul_trigger_ap 2>/dev/null || true
sleep 1

echo "--- AP summary ---"
grep -F '"ev":"ap.assoc"' "$APLOG" | tail -1
grep -F '"ev":"ap.ul_ofdma"' "$APLOG" | tail -1
grep -F '"ev":"ul.rx"' "$APLOG" | head -3
SUMM=$(grep -F '"ev":"ap.summary"' "$APLOG" | tail -1); echo "$SUMM"
WITTRIG=$(grep -cF '"ev":"rx.trigger"' "$WITLOG" 2>/dev/null); WITTRIG=${WITTRIG:-0}
echo "--- witness: aired Triggers decoded = $WITTRIG ---"
[ "$WITTRIG" -gt 0 ] && grep -F '"ev":"rx.trigger"' "$WITLOG" | head -1
echo "--- STA link ---"; grep -iE "Connected|freq|signal|tx bitrate|rx bitrate" "$STALOG" | head
echo "--- STA assoc dmesg ---"; dmesg 2>/dev/null | grep -iE "rtw89|assoc|he |802.11ax" | tail -6

# --- verdict ---
ULRX=$(echo "$SUMM" | grep -oE '"ul_rx":[0-9]+' | grep -oE '[0-9]+'); ULRX=${ULRX:-0}
ULTB=$(echo "$SUMM" | grep -oE '"ul_tb":[0-9]+' | grep -oE '[0-9]+'); ULTB=${ULTB:-0}
if [ -n "$LINK" ] && [ "$ULTB" -gt 0 ]; then
  echo "RESULT: PASS — HE association + $ULTB HE TB PPDU(s): scheduled UL closed."
elif [ -n "$LINK" ] && [ "$WITTRIG" -gt 0 ]; then
  echo "RESULT: PARTIAL(trigger-airs) — associated + the fw AIRS Triggers ($WITTRIG"
  echo "        decoded by the witness), but the STA answered with $ULRX contention"
  echo "        UL frames and no HE TB PPDU. Gap is the STA-side TB response."
elif [ -n "$LINK" ] && [ "$ULRX" -gt 0 ]; then
  echo "RESULT: PARTIAL — associated + $ULRX uplink frames, but no aired Trigger"
  echo "        (witness=$WITTRIG) and no TB PPDU: fw isn't scheduling triggers."
elif [ -n "$LINK" ]; then
  echo "RESULT: PARTIAL — HE association works, but no uplink observed after grants."
else
  echo "RESULT: FAIL — no association (iterate HE IEs; see STA dmesg above)."
fi
