#!/usr/bin/env bash
# adapter_doctor_cold.sh — definitive-verdict wrapper around examples/doctor.
#
# A single warm doctor run can miss a cold-start-only pathology (the #205
# dying unit read valid EFUSE on some warm passes). This wrapper gives the
# doctor what a hard verdict needs, per rep:
#   - TRUE cold: uhubctl VBUS off/on on the DUT's smart-hub port
#   - first-touch: the in-tree rtw88 module temp-blacklisted (udev reloads it
#     on every re-enumeration otherwise)
#   - a vouched traffic source: canonical-SA beacon flood, radiation-verified
#     through a third adapter, so --expect-traffic verdicts are trustworthy
#
# Usage:
#   sudo bash tests/adapter_doctor_cold.sh <hub> <hubport> <sysfs> <bus> <portpath> [reps]
#     e.g. old bench 8812AU:  ... 3-2.3 3 3-2.3.3 3 2.3.3 3
#          new bench 8812AU:  ... 3-2   2 3-2.2   3 2.2   3
#
# Rig knobs (env, defaults = this bench):
#   DOCTOR_RTW88_MOD   in-tree module to keep away   (default rtw88_8812au)
#   DOCTOR_FLOOD_ARGS  txdemo env for the flood      (default 8814AU @ 4/2.3.2)
#   DOCTOR_VERIFY_ARGS rxdemo env for flood verify   (default 8821CU @ 9/1.3)
#   DOCTOR_CHANNEL     bench channel                 (default 6)
#   DOCTOR_DUT_VID     DUT vendor id                 (default 0x0bda)
#   DOCTOR_SKIP_VBUS=1 no per-rep VBUS cycle — for a DUT on a ROOT port
#                      (NEVER uhubctl root ports on this rig: a root-port
#                      cycle once wedged a device past everything but a
#                      machine power-off). Pass '-' for <hub>/<hubport>.
#                      Verdicts are then warm/plug-cold, not per-rep cold.
# The verify runs sequentially before the rep loop, so the DUT itself can
# double as the verify adapter — but prefer pointing DOCTOR_VERIFY_ARGS at a
# third adapter so a stone-deaf DUT can't fail the flood check.
#
# Exit: worst doctor verdict across reps (0 healthy / 1 suspect / 2 failing).

set -u

HUB="${1:?hub (uhubctl -l)}"; HPORT="${2:?hub port}"; SYSFS="${3:?sysfs id}"
BUS="${4:?usb bus}"; PP="${5:?dotted port path}"; REPS="${6:-3}"

MOD="${DOCTOR_RTW88_MOD:-rtw88_8812au}"
CHANNEL="${DOCTOR_CHANNEL:-6}"
DUT_VID="${DOCTOR_DUT_VID:-0x0bda}"   # e.g. 0x2357 for TP-Link-branded DUTs
FLOOD_ARGS="${DOCTOR_FLOOD_ARGS:-DEVOURER_PID=0x8813 DEVOURER_USB_BUS=4 DEVOURER_USB_PORT=2.3.2}"
VERIFY_ARGS="${DOCTOR_VERIFY_ARGS:-DEVOURER_PID=0xc811 DEVOURER_USB_BUS=9 DEVOURER_USB_PORT=1.3}"

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOCTOR="$REPO/build/doctor"
RXDEMO="$REPO/build/rxdemo"
TXDEMO="$REPO/build/txdemo"
BLACKLIST="/etc/modprobe.d/zz-temp-blacklist-doctor.conf"

[ "$(id -u)" = 0 ] || { echo "must run as root"; exit 3; }
[ -x "$DOCTOR" ] || { echo "build/doctor missing — build first"; exit 3; }
command -v uhubctl >/dev/null || { echo "uhubctl not installed"; exit 3; }

TS="$(date +%Y%m%d-%H%M%S)"
LOG="/tmp/devourer-doctor/$TS-$SYSFS"
mkdir -p "$LOG"

cleanup() {
  trap - EXIT INT TERM
  pkill -x txdemo 2>/dev/null
  pkill -x rxdemo 2>/dev/null
  pkill -x doctor 2>/dev/null
  [ "${DOCTOR_SKIP_VBUS:-0}" = 1 ] || uhubctl -l "$HUB" -p "$HPORT" -a on >/dev/null 2>&1
  rm -f "$BLACKLIST"
  modprobe "$MOD" 2>/dev/null
  wait 2>/dev/null
  echo "[cleanup] done — logs in $LOG"
}
trap cleanup EXIT INT TERM

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG/session.log"; }

# flood + radiation check (a deaf verdict is only meaningful while this is up)
env $FLOOD_ARGS DEVOURER_CHANNEL="$CHANNEL" "$TXDEMO" > "$LOG/flood.log" 2>&1 &
sleep 6
kill -0 $! 2>/dev/null || { log "FATAL: flood txdemo died"; tail -5 "$LOG/flood.log"; exit 3; }
timeout -k 5 -s INT 8 env $VERIFY_ARGS DEVOURER_CHANNEL="$CHANNEL" \
  "$RXDEMO" > "$LOG/verify.log" 2>&1
hits=$(grep -o '"hits":[0-9]*' "$LOG/verify.log" | tail -1 | cut -d: -f2)
[ "${hits:-0}" -ge 5 ] || { log "FATAL: flood not radiating (hits=${hits:-0})"; exit 3; }
log "flood radiating (verify hits=$hits) — deaf verdicts are trustworthy"

echo "blacklist $MOD" > "$BLACKLIST"
modprobe -r "$MOD" 2>/dev/null
log "$MOD removed + temp-blacklisted"

worst=0
for i in $(seq 1 "$REPS"); do
  if [ "${DOCTOR_SKIP_VBUS:-0}" = 1 ]; then
    log "--- rep $i/$REPS: no VBUS cycle (root-port DUT) ---"
  else
    log "--- rep $i/$REPS: VBUS cycle $HUB port $HPORT ---"
    uhubctl -l "$HUB" -p "$HPORT" -a off > /dev/null || { log "uhubctl off failed"; exit 3; }
    sleep 5
    uhubctl -l "$HUB" -p "$HPORT" -a on > /dev/null
    t0=$SECONDS
    while [ $((SECONDS - t0)) -lt 20 ]; do
      [ -e "/sys/bus/usb/devices/$SYSFS/idProduct" ] && break
      sleep 0.5
    done
    sleep 1.5
  fi

  "$DOCTOR" --vid "$DUT_VID" --bus "$BUS" --port "$PP" --channel "$CHANNEL" \
    --expect-traffic > "$LOG/rep$i.log" 2>&1
  rc=$?
  [ "$rc" -gt "$worst" ] && [ "$rc" -le 2 ] && worst=$rc
  log "rep $i: $(grep -F '"ev":"doctor.verdict"' "$LOG/rep$i.log" | head -1) (rc=$rc)"
done

log "=== worst verdict across $REPS cold reps: rc=$worst (0=healthy 1=suspect 2=failing) ==="
exit "$worst"
