#!/usr/bin/env bash
# ul_trigger_airs.sh — tier-A validation for the 802.11ax scheduled-UL path:
# does the RTL8852 firmware, driven by devourer's F2P command (SendTrigger),
# actually AIR an HE Basic Trigger frame?
#
#   Injector : RTL8852BU (35bc:0108) under devourer `kestrelprobe trig` — brings
#              up TX then fires a burst of Basic Triggers via the F2P H2C.
#   Witness  : RTL8812AU (0bda:8812) under devourer rxdemo. A Trigger is an
#              802.11 CONTROL frame aired in a legacy PPDU, so even this 11ac
#              witness captures the bytes; rxdemo decodes them as `rx.trigger`
#              (src/TriggerParse.h) and reports the trigger type / RU / MCS.
#
# PASS  = kestrelprobe reports sent>0 (H2C accepted) AND the witness logs >=1
#         rx.trigger whose fields match the fired config (Basic, BW20, RU 61).
# PARTIAL = H2C accepted but no rx.trigger — the fw likely ignores F2P_TEST in
#           normal (non-MP) mode: fall to tests/ul_ofdma_e2e.sh (UL_FIXINFO) and
#           sweep DEVOURER_TRIG_MODE. This is the documented runtime discovery.
#
# Optional: an SDR duty read (tests/sdr_duty.py) confirms on-air energy while the
# burst runs (set SDR=1; needs the B210).
#
#   sudo tests/ul_trigger_airs.sh [channel] [trigger_count]
set -u
cd "$(dirname "$0")/.."
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root (USB claim + driver unbind)"; exit 2; }

CH=${1:-36}
COUNT=${2:-200}
DUT_ID="35bc:0108"   # RTL8852BU — the F2P trigger source (devourer)
RX_ID="0bda:8812"    # RTL8812AU — the monitor witness (devourer rxdemo)
KP="build/kestrelprobe"
RXDEMO="build/rxdemo"
RXLOG="/tmp/ul_trigger_airs_rx.jsonl"
TRIGLOG="/tmp/ul_trigger_airs_trig.jsonl"
for b in "$KP" "$RXDEMO"; do
  [ -x "$b" ] || { echo "FAIL: $b not built (cmake --build build)"; exit 2; }
done

plugged() { lsusb -d "$1" >/dev/null 2>&1; }
for d in "$DUT_ID injector-8852BU" "$RX_ID witness-8812AU"; do
  read -r id name <<<"$d"
  plugged "$id" || { echo "SKIP: $name ($id) not plugged"; exit 0; }
done

# Resolve the USB interface sysfs node so we can unbind the auto-probing kernel
# driver (rtw89 for the 8852BU, rtw88 for the 8812AU) — modprobe -r does not
# survive a re-enumeration, an explicit unbind of THIS device does.
sysdir_for() {
  local vid=${1%%:*} pid=${1##*:} d
  for d in /sys/bus/usb/devices/*; do
    [ -f "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor")" = "$vid" ] && [ "$(cat "$d/idProduct")" = "$pid" ] \
      && { echo "$d"; return; }
  done
}
unbind_kernel() {  # $1 = VID:PID — unbind any driver from its interface(s)
  local d iface drv; d=$(sysdir_for "$1"); [ -n "$d" ] || return 0
  for iface in "$d":*; do
    [ -e "$iface/driver" ] || continue
    drv=$(basename "$(readlink "$iface/driver")")
    echo "$(basename "$iface")" > "/sys/bus/usb/drivers/$drv/unbind" 2>/dev/null || true
  done
}

RXPID=""
cleanup() {
  [ -n "$RXPID" ] && kill "$RXPID" 2>/dev/null || true
  pkill -x kestrelprobe 2>/dev/null || true
  pkill -x rxdemo 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "== ul_trigger_airs: ch$CH, $COUNT triggers, 8852BU -> 8812AU witness =="
unbind_kernel "$DUT_ID"
unbind_kernel "$RX_ID"
sleep 1

# --- witness first: rxdemo on the 8812AU, capturing rx.trigger ---
DEVOURER_PID=0x8812 DEVOURER_VID=0x0bda DEVOURER_CHANNEL="$CH" \
  DEVOURER_EVENTS=stdout DEVOURER_LOG_LEVEL=warn \
  "$RXDEMO" >"$RXLOG" 2>/dev/null &
RXPID=$!
sleep 6   # let the witness bring up + settle on-channel

if ! kill -0 "$RXPID" 2>/dev/null; then
  echo "FAIL: witness rxdemo exited during bring-up (see $RXLOG)"; exit 1
fi

# --- optional SDR duty read while the burst airs ---
SDR_BG=""
if [ "${SDR:-0}" = "1" ]; then
  PY="tests/.venv/bin/python"; [ -x "$PY" ] || PY=python3
  ( "$PY" tests/sdr_duty.py --freq "$(( 5000 + 5*CH ))e6" --secs 6 \
      >/tmp/ul_trigger_airs_sdr.txt 2>&1 || true ) &
  SDR_BG=$!
fi

# --- fire the triggers on the 8852BU ---
DEVOURER_CHANNEL="$CH" DEVOURER_TRIGGER_COUNT="$COUNT" \
  DEVOURER_EVENTS=stdout DEVOURER_LOG_LEVEL=info \
  "$KP" trig --pid 0x0108 >"$TRIGLOG" 2>>"$TRIGLOG"
TRIG_RC=$?
[ -n "$SDR_BG" ] && wait "$SDR_BG" 2>/dev/null || true
sleep 2   # let the last aired triggers reach the witness

# --- verdict ---
SENT=$(grep -F '"ev":"kestrel.trig"' "$TRIGLOG" | grep -oE '"sent":[0-9]+' | grep -oE '[0-9]+' | tail -1)
SENT=${SENT:-0}
# grep -c prints "0" AND exits 1 on no match, so `|| echo 0` would double it —
# count with wc -l instead so exactly one integer lands in RXTRIG.
RXTRIG=$(grep -cF '"ev":"rx.trigger"' "$RXLOG" 2>/dev/null); RXTRIG=${RXTRIG:-0}

echo "---"
echo "kestrelprobe trig: rc=$TRIG_RC sent=$SENT (H2C accepted count)"
echo "witness rx.trigger events: $RXTRIG"
if [ "$RXTRIG" -gt 0 ]; then
  echo "sample decoded trigger:"; grep -F '"ev":"rx.trigger"' "$RXLOG" | head -1
fi
[ "${SDR:-0}" = "1" ] && { echo "SDR duty:"; cat /tmp/ul_trigger_airs_sdr.txt 2>/dev/null | tail -3; }

if [ "$SENT" -gt 0 ] && [ "$RXTRIG" -gt 0 ]; then
  echo "RESULT: PASS — the fw airs Basic Triggers on the F2P command; witness decoded $RXTRIG."
  exit 0
elif [ "$SENT" -gt 0 ]; then
  echo "RESULT: PARTIAL — H2C accepted but no aired trigger seen. The fw likely"
  echo "        ignores F2P_TEST in normal mode. Next: sweep DEVOURER_TRIG_MODE=0..3,"
  echo "        then tests/ul_ofdma_e2e.sh (UL_FIXINFO tf_periodic) as the production path."
  exit 1
else
  echo "RESULT: FAIL — the F2P H2C was not accepted (bring-up or descriptor issue; see $TRIGLOG)."
  exit 1
fi
