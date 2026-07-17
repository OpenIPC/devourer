#!/usr/bin/env bash
# ul_sounding_e2e.sh — the production 802.11ax trigger-airing path, end-to-end.
#
# Unlike ul_ofdma_e2e.sh (which drives the F2P / UL_FIXINFO command surface the
# shipped client fw silently drops), this drives HE SOUNDING: the AP fw builds
# and airs an NDPA -> NDP -> BFRP sequence and arms RX for the beamformee's
# report HE TB PPDU. A BFRP is an 802.11ax Trigger-frame variant, so this is a
# genuine hardware-scheduled, contention-free UL transmission.
#
#   AP  = devourer HE access point on a Kestrel adapter (RTL8832CU, 35bc:0101),
#         tests/ul_trigger_ap.cpp with DEVOURER_SND=1 — advertises itself as an
#         HE beamformer, registers the associated STA as a beamformee, and airs
#         a periodic sounding (StartSounding).
#   STA = a real kernel HE supplicant (RTL8852BU, 35bc:0108, rtw89_8852bu) that
#         associates and, as an SU beamformee, answers the BFRP with an HE TB
#         PPDU carrying its compressed-beamforming report.
#   B210 = ground-truth witness: a strong OFDM burst (the AP's NDPA/BFRP) plus a
#         co-located HE burst one SIFS later (the STA's TB PPDU report).
#
# PASS  = association AND (ap.summary ul_tb > 0  ||  B210 co-located-HE > baseline):
#         the AP airs the sounding and the beamformee answers with a TB PPDU.
# PARTIAL(airs) = associated + soundings sent + B210 strong bursts appear (the
#         NDPA/BFRP airs) but no TB report — STA-side beamformee gap.
# FAIL  = no association (iterate the HE IEs; STA dmesg shows the reason).
#
#   sudo tests/ul_sounding_e2e.sh [seconds] [channel]
set -u
cd "$(dirname "$0")/.."
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }

SEC=${1:-40}
CH=${2:-149}         # a quiet UNII-3 5 GHz channel (rtw89 needs `iw reg set US`)
AP_ID="35bc:0101"    # RTL8832CU — devourer HE AP (beamformer)
STA_ID="35bc:0108"   # RTL8852BU — rtw89 HE station (beamformee)
WIT_ID="${WIT_ID:-2357:012d}"   # RTL8812BU (Archer T3U) on a bus OTHER than the
                                 # AP's — a decode witness that isn't USB-starved
                                 # by the AP (the 8812AU on the AP's bus hears 0).
WIT_VID="0x${WIT_ID%%:*}"; WIT_PID="0x${WIT_ID##*:}"
SSID="devourerAP"
BIN="/tmp/ul_snd_ap"
WPA="/tmp/ul_snd_wpa.conf"
APLOG="/tmp/ul_snd_e2e_ap.log"
STALOG="/tmp/ul_snd_e2e_sta.log"
WITLOG="/tmp/ul_snd_e2e_wit.jsonl"
B210LOG="/tmp/ul_snd_e2e_b210.txt"
FREQ=$(( 5000 + 5*CH ))
# sdr2wifi lives in the INVOKING user's home — NOT $HOME, which is /root under
# sudo (so "$HOME/git/sdr2wifi" silently fails to exist and the B210 scan never
# runs). Resolve via SUDO_USER. Override with SDR2WIFI=... if it lives elsewhere.
SDR2WIFI="${SDR2WIFI:-/home/${SUDO_USER:-$USER}/git/sdr2wifi}"
B210GAIN="${B210GAIN:-62}"

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

echo "== ul_sounding_e2e: AP 8832CU($AP_ID) sounds STA 8852BU($STA_ID) ch$CH ${FREQ}MHz, ${SEC}s =="

# --- VBUS cold-cycle the AP dongle (the 8852C retains a wedged CH12 state
# across soft re-init; only a real power-cycle gives a clean chip). Port 1 of
# hub 3-2.3 is the AP devourer opens; the STA (35bc:0108) is elsewhere. ---
AP_HUB="${AP_HUB:-3-2.3}"; AP_PORT="${AP_PORT:-1}"
if [ "${VBUS_CYCLE:-1}" = 1 ] && command -v uhubctl >/dev/null; then
  echo "-- VBUS cold-cycle AP hub $AP_HUB port $AP_PORT --"
  uhubctl -l "$AP_HUB" -p "$AP_PORT" -a cycle -d 3 >/dev/null 2>&1 || echo "WARN: uhubctl cycle failed"
  sleep 5
fi

# --- build the AP harness ---
[ -f build/libdevourer.a ] || cmake --build build -j --target devourer >/dev/null 2>&1
g++ -std=c++20 -O2 -Isrc -Iexamples/common tests/ul_trigger_ap.cpp \
    examples/common/env_config.cpp build/libdevourer.a \
    $(pkg-config --cflags --libs libusb-1.0) -lpthread -o "$BIN" || { echo "FAIL: build"; exit 1; }

# --- STA side: 8852BU under rtw89_8852bu (world regdomain blocks UNII-3 active
# scan — set US so ch149-165 are usable for association) ---
modprobe rtw89_8852bu 2>/dev/null || true
iw reg set US 2>/dev/null || true
unbind_kernel "$AP_ID"       # free the AP dongle for devourer
sleep 2
STA_IF=$(netdev_for "$STA_ID")
[ -n "$STA_IF" ] || { echo "FAIL: 8852BU has no rtw89 netdev (STA)"; exit 1; }
echo "STA netdev: $STA_IF"
printf 'network={\n\tssid="%s"\n\tkey_mgmt=NONE\n\tscan_ssid=1\n}\n' "$SSID" > "$WPA"
pkill -9 -x wpa_supplicant 2>/dev/null; iw dev "$STA_IF" disconnect 2>/dev/null; sleep 1

# --- AP up in SOUNDING mode (DEVOURER_SND=1): beamformer IEs + StartSounding ---
# Periodic sounding (SND_PERIOD>0): ONE H2C arms fw-autonomous repeated sounding
# (SNDF2P_ADD). This is the correct sustained path — a per-sounding one-shot loop
# wedges the CH12 queue after ~7 (the fw holds an H2C page per one-shot sounding
# until its frame-exchange completes; with no report they never release).
SND_PERIOD=${SND_PERIOD:-10}; SND_GAP=${SND_GAP:-0}
# INJECT=1: run the AP in trigger-INJECTION mode instead of sounding — SendTrigger
# host-builds a raw 802.11ax Basic Trigger (build_basic_trigger) and pushes it
# through the mgmt TX queue (like a beacon/RTS), bypassing the fw sounding/F2P
# scheduler that the client fw doesn't run. The witness decodes an aired trigger
# as rx.trigger (FC=0x24). This is the path that CAN put a trigger on the air.
if [ "${INJECT:-0}" = 1 ]; then
  env DEVOURER_VID=0x35bc DEVOURER_PID=0x0101 DEVOURER_CHANNEL="$CH" \
      DEVOURER_BCN_TU=25 DEVOURER_TX_WITH_RX=thread \
      DEVOURER_TRIG_GAP_MS="${TRIG_GAP_MS:-50}" \
      DEVOURER_LOG_LEVEL=info "$BIN" "$SEC" >/dev/null 2>"$APLOG" &
else
  env DEVOURER_VID=0x35bc DEVOURER_PID=0x0101 DEVOURER_CHANNEL="$CH" \
      DEVOURER_BCN_TU=25 DEVOURER_TX_WITH_RX=thread DEVOURER_SND=1 \
      DEVOURER_SND_PERIOD="$SND_PERIOD" DEVOURER_SND_GAP_MS="$SND_GAP" \
      DEVOURER_LOG_LEVEL=info "$BIN" "$SEC" >/dev/null 2>"$APLOG" &
fi
# Wait up to 25 s for bring-up (fresh-rig FWDL + init_sounding is ~8 s; a fixed
# short sleep raced it and made the whole script bail with "AP did not come up").
for i in $(seq 1 25); do grep -q "HE AP up" "$APLOG" 2>/dev/null && break; sleep 1; done
grep -q "HE AP up" "$APLOG" || { echo "FAIL: AP did not come up"; tail -8 "$APLOG"; exit 1; }
grep -q "init_sounding" "$APLOG" && echo "AP: sounding engine armed"

# --- decode witness: an RTL8812BU (Archer T3U) that decodes any aired BFRP
# (FC=0x24) as rx.trigger — the unambiguous "the BFRP aired" signal. NOTE: on
# this bench the 8812AU sits on the AP's own USB bus and reads 0 frames; whether
# that is USB starvation vs another cause was not conclusively isolated, so this
# uses a separate-bus adapter. WIT=0 disables it (e.g. to leave the B210's bus
# uncontended). ---
WIT_UP=0
if [ "${WIT:-1}" = 1 ] && lsusb -d "$WIT_ID" >/dev/null 2>&1; then
  unbind_kernel "$WIT_ID"
  env DEVOURER_PID="${WIT_PID:-0x012d}" DEVOURER_VID="${WIT_VID:-0x2357}" \
      DEVOURER_CHANNEL="$CH" DEVOURER_RX_DUMP_ALL=1 \
      DEVOURER_EVENTS=stdout DEVOURER_LOG_LEVEL=warn build/rxdemo >"$WITLOG" 2>/dev/null &
  WIT_UP=1
  echo "decode witness ${WIT_VID:-0x2357}:${WIT_PID:-0x012d} on ch$CH (rx.trigger + all-frame count)"
  sleep 4
fi

# --- B210 is the energy/preamble witness; the 2nd Kestrel monitor is disabled by
# default (two same-ID 8852C devourer instances on one hub conflict on bring-up). ---
WIT2LOG="/tmp/ul_snd_e2e_wit2.jsonl"; : > "$WIT2LOG"; WIT2_UP=0
WIT2_PORT="${WIT2_PORT:-2.3.2}"
if [ "${WIT2:-0}" = 1 ]; then
  env DEVOURER_VID=0x35bc DEVOURER_PID=0x0101 DEVOURER_USB_BUS=3 \
      DEVOURER_USB_PORT="$WIT2_PORT" DEVOURER_CHANNEL="$CH" \
      DEVOURER_EVENTS=stdout DEVOURER_LOG_LEVEL=warn build/rxdemo >"$WIT2LOG" 2>/dev/null &
  WIT2_UP=1
  echo "co-located witness 8832CU (hub $AP_HUB port $WIT2_PORT) on ch$CH"
  sleep 4
fi

# --- B210 BASELINE (AP beacon only, no association/sounding yet) ---
B210BASE="/tmp/ul_snd_e2e_b210_base.txt"; : > "$B210BASE"; : > "$B210LOG"
HAS_B210=0; uhd_find_devices 2>/dev/null | grep -qi B210 && HAS_B210=1
if [ "$HAS_B210" = 1 ]; then
  echo "-- B210 baseline (beacon-only) 6 s --"
  python3 "$SDR2WIFI/iq_burst_scan.py" --freq "${FREQ}e6" --secs 6 --gain "$B210GAIN" \
      2>/dev/null | grep -E "bursts>=2us" | tee "$B210BASE" || true
fi

# --- STA connects (as a beamformee) ---
dmesg -C 2>/dev/null || true
wpa_supplicant -i "$STA_IF" -c "$WPA" -B >/dev/null 2>&1
echo "-- waiting for HE association --"
LINK=""
for i in $(seq 1 20); do
  LINK=$(iw dev "$STA_IF" link 2>&1 | grep -oE 'Connected to [0-9a-f:]+') && { echo "   $LINK (after ${i}s)"; break; }
  sleep 1
done
# Keepalive so the STA stays associated across the sounding window (the AP has an
# ARP/ICMP responder; without traffic the supplicant re-associates every ~7s).
if [ -n "$LINK" ]; then
  ip addr flush dev "$STA_IF" 2>/dev/null
  ip addr add 192.168.99.2/24 dev "$STA_IF" 2>/dev/null
  ( for k in $(seq 1 60); do ping -c 1 -W 1 -I "$STA_IF" 192.168.99.1 >/dev/null 2>&1; sleep 0.4; done ) &
fi
# Wait for the AP to start sounding (association latched), then B210-scan the
# sounding window. Not gated on the momentary LINK (the STA re-associates ~7s;
# the sounding fires whenever it is associated).
for i in $(seq 1 10); do grep -q '"ev":"ap.sounding"\|set_snd_para' "$APLOG" && break; sleep 1; done

if [ "$HAS_B210" = 1 ]; then
  echo "-- B210 sounding-window scan 10 s (compare strong/HE vs baseline) --"
  python3 "$SDR2WIFI/iq_burst_scan.py" --freq "${FREQ}e6" --secs 10 --gain "$B210GAIN" \
      2>/dev/null | grep -E "bursts>=2us" | tee "$B210LOG" || true
fi

sleep $((SEC - 30 > 4 ? SEC - 30 : 4))
iw dev "$STA_IF" link > "$STALOG" 2>&1 || true
sleep 4
pkill -9 -x ul_trigger_ap 2>/dev/null || true
sleep 1

echo "--- AP summary ---"
grep -F '"ev":"ap.assoc"' "$APLOG" | tail -1
grep -F '"ev":"ap.bfee"' "$APLOG" | tail -1
SUMM=$(grep -F '"ev":"ap.summary"' "$APLOG" | tail -1); echo "$SUMM"
grep -F 'set_snd_para' "$APLOG" | tail -1
grep -iE 'SER|halt|fw.err' "$APLOG" | tail -2
WITALL=$(grep -cE '"ev":"rx' "$WITLOG" 2>/dev/null); WITALL=${WITALL:-0}
WITTRIG=$(grep -cF '"ev":"rx.trigger"' "$WITLOG" 2>/dev/null); WITTRIG=${WITTRIG:-0}
echo "--- decode witness $WIT_ID: total frames heard=$WITALL, aired BFRP/Triggers=$WITTRIG ---"
[ "$WITALL" -eq 0 ] && echo "    (0 frames — witness deaf: wrong bus/channel or USB-starved by the AP)"
[ "$WITTRIG" -gt 0 ] && grep -F '"ev":"rx.trigger"' "$WITLOG" | head -1
# co-located Kestrel witness (the decisive airing detector)
WIT2ALL=$(grep -cE '"ev":"rx' "$WIT2LOG" 2>/dev/null); WIT2ALL=${WIT2ALL:-0}
WIT2TRIG=$(grep -cF '"ev":"rx.trigger"' "$WIT2LOG" 2>/dev/null); WIT2TRIG=${WIT2TRIG:-0}
WIT2TB=$(grep -cE '"ppdu_type":10|ppdu_type=10' "$WIT2LOG" 2>/dev/null); WIT2TB=${WIT2TB:-0}
echo "--- co-located 8832CU witness: total rx=$WIT2ALL, aired BFRP/Triggers=$WIT2TRIG, HE_TB(ppdu10)=$WIT2TB ---"
[ "$WIT2TRIG" -gt 0 ] && grep -F '"ev":"rx.trigger"' "$WIT2LOG" | head -1
echo "--- STA link ---"; grep -iE "Connected|freq|signal|tx bitrate|rx bitrate" "$STALOG" | head
echo "--- STA assoc dmesg ---"; dmesg 2>/dev/null | grep -iE "rtw89|assoc|he |beamform|802.11ax" | tail -6

# --- B210 baseline-vs-sounding delta. The AP arrives at ~-27 dBm at 1 m, which
# sits BELOW the scanner's "strong>+20dB" threshold, so use the PREAMBLE
# CLASSIFIER counts instead: OFDM L-STF (legacy — beacon + the BFRP trigger) and
# VHT/HE RL-SIG (the HE NDP + the TB PPDU report). A rise in RL-SIG during
# sounding is the NDP/TB signature; a rise in OFDM is the aired BFRP. ---
parse_b210() { grep -oE "$2: [0-9]+" "$1" 2>/dev/null | grep -oE '[0-9]+$' | tail -1; }
BASE_OFDM=$(parse_b210 "$B210BASE" 'OFDM L-STF'); BASE_OFDM=${BASE_OFDM:-0}
SND_OFDM=$(parse_b210 "$B210LOG" 'OFDM L-STF'); SND_OFDM=${SND_OFDM:-0}
BASE_HE=$(parse_b210 "$B210BASE" 'VHT/HE RL-SIG'); BASE_HE=${BASE_HE:-0}
SND_HE=$(parse_b210 "$B210LOG" 'VHT/HE RL-SIG'); SND_HE=${SND_HE:-0}
echo "--- B210 preamble classifier: OFDM L-STF base=$BASE_OFDM snd=$SND_OFDM | HE RL-SIG base=$BASE_HE snd=$SND_HE ---"

# --- verdict ---
ULTB=$(echo "$SUMM" | grep -oE '"ul_tb":[0-9]+' | grep -oE '[0-9]+'); ULTB=${ULTB:-0}
SND=$(echo "$SUMM" | grep -oE '"soundings":[0-9]+' | grep -oE '[0-9]+'); SND=${SND:-0}
TRIG=$(echo "$SUMM" | grep -oE '"triggers":[0-9]+' | grep -oE '[0-9]+'); TRIG=${TRIG:-0}
ASSOCOK=0; grep -q '"associated":true' <<<"$SUMM" && ASSOCOK=1
if [ "${INJECT:-0}" = 1 ]; then
  # Injection mode: SendTrigger host-injects a Basic Trigger via the mgmt queue.
  # The witness decoding rx.trigger with the commanded params proves it airs.
  if [ "$WITTRIG" -gt 0 ] && [ "$ULTB" -gt 0 ]; then
    echo "RESULT: PASS — host-injected Trigger AIRS ($TRIG sent, $WITTRIG decoded by"
    echo "        the witness) AND the STA answered with $ULTB HE TB PPDU(s)."
  elif [ "$WITTRIG" -gt 0 ]; then
    echo "RESULT: TRIGGER-AIRS — the host-injected HE Basic Trigger AIRS: $TRIG sent,"
    echo "        $WITTRIG decoded on-air by the witness with the commanded params"
    echo "        (overturns 'no trigger airs'). But ul_tb=$ULTB — the STA did not"
    echo "        answer with a TB PPDU (host-injected triggers lack fw SIFS-response"
    echo "        arming; that half needs a fw-scheduled trigger the client fw won't air)."
  elif [ "$TRIG" -gt 0 ]; then
    echo "RESULT: PARTIAL — $TRIG triggers injected but witness decoded 0 (witness"
    echo "        deaf/misplaced: frames=$WITALL) — can't confirm airing."
  else
    echo "RESULT: PARTIAL — associated but no triggers injected."
  fi
elif [ "$ULTB" -gt 0 ] || [ "$SND_HE" -gt "$((BASE_HE + 3))" ]; then
  echo "RESULT: PASS — HE sounding closed: ul_tb=$ULTB (AP-decoded report), B210"
  echo "        HE RL-SIG bursts $BASE_HE->$SND_HE (NDP + on-air TB PPDU). $SND soundings."
elif [ "$SND" -gt 0 ] && { [ "$WITTRIG" -gt 0 ] || [ "$SND_OFDM" -gt "$((BASE_OFDM + 3))" ]; }; then
  echo "RESULT: PARTIAL(airs) — $SND soundings; the NDPA/BFRP AIRS (witness BFRP="
  echo "        $WITTRIG, B210 OFDM $BASE_OFDM->$SND_OFDM) but no TB report"
  echo "        (ul_tb=$ULTB, HE RL-SIG $BASE_HE->$SND_HE): STA-side gap."
elif [ "$SND" -gt 0 ]; then
  echo "RESULT: PARTIAL(accepted) — $SND soundings fw-ACCEPTED (no SER) but no aired"
  echo "        frames seen (witness_frames=$WITALL trig=$WITTRIG, B210 OFDM"
  echo "        $BASE_OFDM->$SND_OFDM HE $BASE_HE->$SND_HE): fw accepts SET_SND_PARA"
  echo "        but its client-fw scheduler doesn't air it (or witnesses still deaf)."
elif [ "$ASSOCOK" = 1 ]; then
  echo "RESULT: PARTIAL — HE association works, but no sounding issued."
else
  echo "RESULT: FAIL — no association (iterate HE IEs; see STA dmesg above)."
fi
