#!/usr/bin/env bash
# cold_first_init_8812au.sh — issue #205 discriminator.
#
# Per-rep TRUE-COLD (smart-hub VBUS cycle) first-init test of an RTL8812AU
# under devourer, with the in-tree rtw88_8812au kernel module as the
# controlled variable. Background: every "true cold" data point in issue
# #205 was contaminated — rtw88_8812au auto-probes the chip at enumeration
# (boot included) and, on the suspect unit, fails its FW download, leaving
# the chip half-wedged BEFORE devourer/vendor ever ran.
#
# Arms:
#   new-clean  — NEW unit  3-2.2   rtw88_8812au REMOVED  (uncontaminated control)
#   old-clean  — OLD unit  3-2.3.3 rtw88_8812au REMOVED  (THE question: unit bad or contaminated?)
#   old-rtw88  — OLD unit  3-2.3.3 rtw88_8812au LOADED   (contamination repro: expect InitLLT fail)
#   new-rtw88  — NEW unit  3-2.2   rtw88_8812au LOADED   (successful-probe contamination control)
#   old-retry  — OLD unit, clean, TWO back-to-back inits per cold cycle
#                (does a warm 2nd init recover EFUSE/FW/hearing?)
#   old-delay  — OLD unit, clean, 60 s powered-idle before the first init
#                (is the cold defect settling-TIME or init-history dependent?)
#
# Usage: sudo bash tests/cold_first_init_8812au.sh <arm> [reps]
# Logs:  /tmp/devourer-cold-first-init/<ts>-<arm>/
#
# A beacon flood (8814AU, canonical SA) runs for the whole session and is
# radiation-verified via a third adapter (8821CU) before the first rep and
# after the last — a DEAF verdict is only trustworthy while the flood is up.

set -u

ARM="${1:-}"
REPS="${2:-3}"
CHANNEL="${DEVOURER_COLD_CHANNEL:-6}"
OFF_SECS=5          # VBUS off-time per cold cycle
ENUM_TIMEOUT=20     # max wait for re-enumeration
RTW88_SETTLE=8      # wait for rtw88 probe to finish/fail (rtw88 arms)
RX_SECS=25          # DUT rxdemo listen window
VERIFY_SECS=8       # flood-verification rxdemo window
MIN_VERIFY_HITS=5   # flood must produce at least this many hits to count as up

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RXDEMO="$REPO/build/rxdemo"
TXDEMO="$REPO/build/txdemo"

# Rig topology (this bench, 2026-07-07):
#   OLD suspect unit : sysfs 3-2.3.3  = uhubctl hub 3-2.3 port 3
#   NEW unit         : sysfs 3-2.2    = uhubctl hub 3-2   port 2
#   Flood TX         : 8814AU  0bda:8813  bus 4 port-path 2.3.2
#   Flood verify RX  : 8821CU  0bda:c811  bus 9 port-path 1.3
PRE_DELAY=0   # seconds powered-idle between enumeration and first init
RUNS=1        # rxdemo runs per cold cycle (2 = test warm re-init recovery)
case "$ARM" in
  new-clean|new-rtw88)           HUB="3-2";   HPORT=2; SYSFS="3-2.2";   DUT_BUS=3; DUT_PP="2.2" ;;
  old-clean|old-rtw88|old-retry|old-delay)
                                 HUB="3-2.3"; HPORT=3; SYSFS="3-2.3.3"; DUT_BUS=3; DUT_PP="2.3.3" ;;
  *) echo "usage: sudo $0 {new-clean|old-clean|old-rtw88|new-rtw88|old-retry|old-delay} [reps]"; exit 2 ;;
esac
case "$ARM" in
  *-rtw88) WANT_RTW88=loaded ;;
  *)       WANT_RTW88=absent ;;
esac
[ "$ARM" = old-retry ] && RUNS=2
[ "$ARM" = old-delay ] && PRE_DELAY=60
RUNS="${DEVOURER_COLD_RUNS:-$RUNS}"          # override runs-per-cycle
PRE_DELAY="${DEVOURER_COLD_PRE_DELAY:-$PRE_DELAY}"

[ "$(id -u)" = 0 ] || { echo "must run as root (uhubctl + modprobe)"; exit 2; }
[ -x "$RXDEMO" ] && [ -x "$TXDEMO" ] || { echo "build/rxdemo|txdemo missing — build first"; exit 2; }
command -v uhubctl >/dev/null || { echo "uhubctl not installed"; exit 2; }

TS="$(date +%Y%m%d-%H%M%S)"
LOG="/tmp/devourer-cold-first-init/$TS-$ARM"
mkdir -p "$LOG"

FLOOD_PID=""
# udev auto-reloads rtw88_8812au by modalias on every re-enumeration, so a
# bare `modprobe -r` does not survive a VBUS cycle. A temp blacklist does
# (udev invokes `modprobe -b`, which honours it) — same pattern as the rig's
# zz-temp-blacklist-8852au.conf.
BLACKLIST=/etc/modprobe.d/zz-temp-blacklist-rtw88-8812au.conf
cleanup() {
  trap - EXIT INT TERM
  # exact-comm kills of our long-running demos only
  pkill -x txdemo 2>/dev/null
  pkill -x rxdemo 2>/dev/null
  # leave the DUT port powered
  uhubctl -l "$HUB" -p "$HPORT" -a on >/dev/null 2>&1
  # restore the kernel module to its stock (auto-loadable) state
  rm -f "$BLACKLIST"
  modprobe rtw88_8812au 2>/dev/null
  wait 2>/dev/null
  echo "[cleanup] done — logs in $LOG"
}
trap cleanup EXIT INT TERM

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG/session.log"; }

# ---------------------------------------------------------------- session header
{
  echo "arm=$ARM reps=$REPS channel=$CHANNEL"
  echo "git=$(git -C "$REPO" rev-parse --short HEAD 2>/dev/null)  kernel=$(uname -r)"
  date
  lsusb | grep -i realtek
} > "$LOG/rig.txt" 2>&1

# ---------------------------------------------------------------- flood bring-up
start_flood() {
  DEVOURER_CHANNEL="$CHANNEL" DEVOURER_PID=0x8813 \
  DEVOURER_USB_BUS=4 DEVOURER_USB_PORT=2.3.2 \
    "$TXDEMO" > "$LOG/flood.log" 2>&1 &
  FLOOD_PID=$!
  sleep 6   # 8814 bring-up incl. fwdl
  kill -0 "$FLOOD_PID" 2>/dev/null || { log "FATAL: flood txdemo died at start:"; tail -5 "$LOG/flood.log"; exit 1; }
}

verify_flood() { # $1 = tag ; returns 0 if radiating
  local vlog="$LOG/verify-$1.log"
  timeout -k 5 -s INT "$VERIFY_SECS" env \
    DEVOURER_CHANNEL="$CHANNEL" DEVOURER_PID=0xc811 \
    DEVOURER_USB_BUS=9 DEVOURER_USB_PORT=1.3 \
    "$RXDEMO" > "$vlog" 2>&1
  local hits
  hits=$(grep -o 'hits=[0-9]*' "$vlog" | tail -1 | cut -d= -f2)
  hits="${hits:-0}"
  log "flood verify ($1): hits=$hits (need >=$MIN_VERIFY_HITS)"
  [ "$hits" -ge "$MIN_VERIFY_HITS" ]
}

log "starting canonical-SA beacon flood on 8814AU ch$CHANNEL"
start_flood
if ! verify_flood pre; then
  log "flood not radiating — one retry"
  pkill -x txdemo 2>/dev/null; sleep 2; start_flood
  verify_flood pre2 || { log "FATAL: flood dead twice — aborting"; exit 1; }
fi

# ---------------------------------------------------------------- module state
if [ "$WANT_RTW88" = absent ]; then
  echo "blacklist rtw88_8812au" > "$BLACKLIST"
  modprobe -r rtw88_8812au 2>/dev/null
  lsmod | grep -q '^rtw88_8812au' && { log "FATAL: rtw88_8812au still loaded"; exit 1; }
  log "rtw88_8812au removed + temp-blacklisted — nothing touches the DUT before devourer"
else
  rm -f "$BLACKLIST"
  modprobe rtw88_8812au
  log "rtw88_8812au loaded — kernel probe will hit the DUT first (contamination arm)"
fi

# ---------------------------------------------------------------- rep loop
declare -a VERDICTS
for i in $(seq 1 "$REPS"); do
  RLOG="$LOG/rep$i.log"
  JLOG="$LOG/rep$i.journal"
  SINCE="$(date '+%Y-%m-%d %H:%M:%S')"

  log "--- rep $i/$REPS: VBUS off ${OFF_SECS}s on hub $HUB port $HPORT ---"
  uhubctl -l "$HUB" -p "$HPORT" -a off > /dev/null || { log "FATAL: uhubctl off failed"; exit 1; }
  sleep 1
  uhubctl -l "$HUB" -p "$HPORT" >> "$LOG/rep$i.power" 2>&1   # mid-off power-state evidence
  sleep "$((OFF_SECS - 1))"
  uhubctl -l "$HUB" -p "$HPORT" -a on > /dev/null

  # wait for re-enumeration
  t0=$SECONDS; enum=""
  while [ $((SECONDS - t0)) -lt "$ENUM_TIMEOUT" ]; do
    if [ "$(cat /sys/bus/usb/devices/$SYSFS/idProduct 2>/dev/null)" = "8812" ]; then enum=ok; break; fi
    sleep 0.5
  done
  [ -n "$enum" ] || { log "rep $i: DUT never re-enumerated"; VERDICTS+=("rep$i: NO_ENUM"); continue; }
  log "rep $i: re-enumerated after $((SECONDS - t0))s"

  if [ "$WANT_RTW88" = loaded ]; then
    sleep "$RTW88_SETTLE"
    probe=$(journalctl -k --since "$SINCE" | grep -E "rtw88_8812au $SYSFS" | tail -3)
    log "rep $i: rtw88 probe tail: $(echo "$probe" | tail -1 | sed 's/.*rtw88_8812au/rtw88_8812au/')"
  else
    sleep 1.5   # let udev finish; then assert nothing bound to the interface
    if [ -e "/sys/bus/usb/devices/$SYSFS:1.0/driver" ]; then
      log "rep $i: INVALID — a kernel driver bound despite blacklist: $(basename "$(readlink "/sys/bus/usb/devices/$SYSFS:1.0/driver")")"
      VERDICTS+=("rep$i: INVALID(kernel-driver-raced)"); continue
    fi
  fi

  if [ "$PRE_DELAY" -gt 0 ]; then
    log "rep $i: powered-idle ${PRE_DELAY}s before first init"
    sleep "$PRE_DELAY"
  fi

  for r in $(seq 1 "$RUNS"); do
    RLOG="$LOG/rep$i.run$r.log"
    timeout -k 5 -s INT "$RX_SECS" env \
      DEVOURER_CHANNEL="$CHANNEL" DEVOURER_PID=0x8812 \
      DEVOURER_USB_BUS="$DUT_BUS" DEVOURER_USB_PORT="$DUT_PP" \
      "$RXDEMO" > "$RLOG" 2>&1
    rc=$?

    mac_state=$(grep -o "MAC has already power on\|MAC has not been powered on yet" "$RLOG" | tail -1)
    eeprom=$(grep -o "EEPROM ID=0x[0-9A-Fa-f]*" "$RLOG" | tail -1)
    fwrdy=$(grep -o "Polling FW ready OK\|Polling FW ready Fail" "$RLOG" | tail -1)
    pon=$(grep "init-timing: hal_init.power_on" "$RLOG" | tail -1 | grep -o '= [0-9]* ms' | tr -dc 0-9)
    tot=$(grep "init-timing: hal_init.total" "$RLOG" | tail -1 | grep -o '= [0-9]* ms' | tr -dc 0-9)
    hits=$(grep -o 'hits=[0-9]*' "$RLOG" | tail -1 | cut -d= -f2); hits="${hits:-0}"
    fail=$(grep -c "InitLLTTable8812A failed\|InitPowerOn: run power on flow fail\|rtw_hal_init: fail" "$RLOG")

    if [ "$fail" -gt 0 ]; then v="INIT_FAIL(power_on=${pon:-?}ms)"
    elif [ -z "$tot" ] && [ "$rc" != 124 ]; then v="NO_INIT(rc=$rc — see $(basename "$RLOG"))"
    elif [ "$hits" -eq 0 ]; then v="DEAF(init_ok power_on=${pon:-?}ms hal=${tot:-?}ms)"
    else v="HEARS(hits=$hits power_on=${pon:-?}ms hal=${tot:-?}ms)"; fi
    v="$v [$eeprom ${fwrdy:-no-fwdl} mac='${mac_state:-n/a}']"
    log "rep $i run $r: rc=$rc → $v"
    VERDICTS+=("rep$i.run$r: $v")
  done

  journalctl -k --since "$SINCE" | grep -E "usb $SYSFS|$SYSFS:1\.0" > "$JLOG" 2>&1
done

# ---------------------------------------------------------------- flood re-check
kill -0 "$FLOOD_PID" 2>/dev/null || log "WARNING: flood txdemo died mid-session — DEAF verdicts unreliable"
verify_flood post || log "WARNING: flood not radiating at end — DEAF verdicts unreliable"

log "=== $ARM summary ==="
for v in "${VERDICTS[@]}"; do log "  $v"; done
