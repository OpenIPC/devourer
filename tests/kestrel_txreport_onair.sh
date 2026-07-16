#!/usr/bin/env bash
# kestrel_txreport_onair.sh — USR_TX_RPT (#236) on-air validation.
# Runs devourer txdemo on the RTL8852BU (35bc:0108) with TX_WITH_RX=thread so
# the RX loop is up to receive C2H, TXing the canonical beacon. The firmware's
# per-user TX report (FWCMD_H2C_FUNC_USR_TX_RPT, PERIOD mode) is enabled in
# InitWrite; in PERIOD mode the fw emits a C2H USR_TX_RPT_INFO every
# rpt_period_us carrying the freerun TX-egress timestamps — the host-visible
# scheduled-TX air-departure time 11ac/Jaguar could not expose (issue #236).
# Passing = at least one "tx.report" line appears in the devourer log.
#
#   sudo tests/kestrel_txreport_onair.sh [channel] [seconds]
set -u
cd "$(dirname "$0")/.."
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }
CH=${1:-6}; DUR=${2:-10}
TX_ID="35bc:0108"                 # RTL8852BU (devourer txdemo)
TX_HUB="3-2.3"; TX_PORT="3"       # uhubctl VBUS map for the 8852BU
TXLOG="/tmp/kestrel_txreport.log"
[ -x build/txdemo ] || { echo "FAIL: build txdemo"; exit 2; }

sysdir_for() {
  local vid=${1%%:*} pid=${1##*:} d
  for d in /sys/bus/usb/devices/*; do
    [ -f "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor")" = "$vid" ] && [ "$(cat "$d/idProduct")" = "$pid" ] \
      && { echo "$d"; return; }
  done
}
unbind_kernel() {
  local d i; d=$(sysdir_for "$1")
  [ -n "$d" ] && for i in "$d":*; do
    [ -L "$i/driver" ] && echo "$(basename "$i")" > "$(readlink -f "$i/driver")/unbind" 2>/dev/null || true
  done
}
TXPID=""
cleanup() {
  [ -n "$TXPID" ] && kill "$TXPID" 2>/dev/null
  pkill -9 -f "build/txdemo" 2>/dev/null || true
}
trap cleanup EXIT

echo ">> VBUS-cycling the 8852BU ($TX_ID) for a clean devourer bring-up"
uhubctl -l "$TX_HUB" -p "$TX_PORT" -a cycle -d 2 >/dev/null 2>&1
sleep 6

unbind_kernel "$TX_ID"; sleep 1
echo ">> devourer txdemo on the 8852BU for ${DUR}s (TX_WITH_RX=thread, ch$CH)"
env DEVOURER_VID=0x35bc DEVOURER_PID=0x0108 DEVOURER_CHANNEL=$CH \
  DEVOURER_TX_GAP_US=2000 DEVOURER_LOG_LEVEL=debug DEVOURER_TX_RATE=6M \
  DEVOURER_TX_WITH_RX=thread \
  build/txdemo >"$TXLOG" 2>&1 &
TXPID=$!
sleep "$DUR"
kill "$TXPID" 2>/dev/null; TXPID=""
sleep 1

# --- verdict ---
# grep -c already prints 0 on no-match; guard only the missing-file case.
TX_UP=$(grep -c "TX ready" "$TXLOG" 2>/dev/null); TX_UP=${TX_UP:-0}
ENA=$(grep -c "USR_TX_RPT enable" "$TXLOG" 2>/dev/null); ENA=${ENA:-0}
ENA_OK=$(grep -c "USR_TX_RPT enable.*sent" "$TXLOG" 2>/dev/null); ENA_OK=${ENA_OK:-0}
RPTS=$(grep -c "tx.report" "$TXLOG" 2>/dev/null); RPTS=${RPTS:-0}
# Diagnostic: count ALL packet-C2H (rpkt_type=10) that reached the host, and
# show their cat/cls/func — the decisive evidence of whether the fw emits any
# async packet-C2H at all (independent of the USR_TX_RPT-specific match).
C2H_ALL=$(grep -c "c2h.rx" "$TXLOG" 2>/dev/null); C2H_ALL=${C2H_ALL:-0}
echo "=================================================================="
echo "USR_TX_RPT on-air (ch$CH, ${DUR}s):  txdemo TX-ready=$TX_UP  enable=$ENA (sent=$ENA_OK)"
echo "  packet-C2H (any) received: $C2H_ALL"
if [ "$C2H_ALL" -gt 0 ]; then
  echo "  --- distinct C2H cat/cls/func seen ---"
  grep -oE "c2h.rx: cat=[0-9]+ cls=0x[0-9a-f]+ func=0x[0-9a-f]+" "$TXLOG" \
    | sort | uniq -c | head
fi
echo "  tx.report C2H reports received: $RPTS"
if [ "$TX_UP" -lt 1 ]; then
  echo "RESULT: TX bring-up FAILED — see $TXLOG"; tail -6 "$TXLOG"
elif [ "$ENA_OK" -lt 1 ]; then
  echo "RESULT: USR_TX_RPT enable H2C did not send — see $TXLOG"; tail -6 "$TXLOG"
elif [ "${RPTS:-0}" -gt 0 ]; then
  echo "RESULT: PASS — the fw emits per-user TX reports (freerun TX-egress ts):"
  grep "tx.report" "$TXLOG" | head -3
else
  echo "RESULT: enable sent OK but ZERO reports fired."
  echo "  Likely: PERIOD report accounts only registered-role MACIDs; monitor"
  echo "  injection (macid 0, no addr-cam/role) isn't tracked; needs a full BSS STA role."
  echo "  --- txdemo tail ---"; tail -8 "$TXLOG"
fi
