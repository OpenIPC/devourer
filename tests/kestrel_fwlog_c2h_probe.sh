#!/usr/bin/env bash
# kestrel_fwlog_c2h_probe.sh — decisive probe of async packet-C2H delivery on
# the RTL8852BU. Runs txdemo with DEVOURER_KESTREL_FWLOG=1, which routes the
# firmware log to C2H packets (mac_fw_log_cfg output=C2H). If the host receives
# rpkt_type=10 C2H (cls=FW_INFO 0x0, func=C2H_LOG 0x2), the packet-C2H surface
# the #236 capstone depends on (TWT_NOTIFY_EVT, F2P reports) is reachable — the
# USR_TX_RPT-specific non-firing is then a fw gating quirk, not a delivery gap.
#
#   sudo tests/kestrel_fwlog_c2h_probe.sh [seconds]
set -u
cd "$(dirname "$0")/.."
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }
DUR=${1:-10}
TX_HUB="3-2.3"; TX_PORT="3"; LOG=/tmp/kestrel_fwlog.log
[ -x build/txdemo ] || { echo "FAIL: build txdemo"; exit 2; }

sysdir() { for d in /sys/bus/usb/devices/*; do
  [ -f "$d/idVendor" ] && [ "$(cat "$d/idVendor")" = 35bc ] &&
  [ "$(cat "$d/idProduct")" = 0108 ] && { echo "$d"; return; }; done; }
unbind() { local d; d=$(sysdir); [ -n "$d" ] && for i in "$d":*; do
  [ -L "$i/driver" ] && echo "$(basename "$i")" \
    > "$(readlink -f "$i/driver")/unbind" 2>/dev/null || true; done; }
P=""; cleanup() { [ -n "$P" ] && kill "$P" 2>/dev/null;
  pkill -9 -f build/txdemo 2>/dev/null || true; }
trap cleanup EXIT

echo ">> VBUS-cycle + devourer txdemo with fw-log->C2H"
uhubctl -l "$TX_HUB" -p "$TX_PORT" -a cycle -d 2 >/dev/null 2>&1; sleep 6
unbind; sleep 1
env DEVOURER_VID=0x35bc DEVOURER_PID=0x0108 DEVOURER_CHANNEL=6 \
    DEVOURER_TX_GAP_US=2000 DEVOURER_LOG_LEVEL=debug \
    DEVOURER_TX_WITH_RX=thread DEVOURER_KESTREL_FWLOG=1 \
    build/txdemo >"$LOG" 2>&1 & P=$!
sleep "$DUR"; kill "$P" 2>/dev/null; P=""; sleep 1

ENA=$(grep -c "FW-log->C2H enable.*sent" "$LOG" 2>/dev/null); ENA=${ENA:-0}
C2H=$(grep -c "c2h.rx" "$LOG" 2>/dev/null); C2H=${C2H:-0}
echo "=================================================================="
echo "fw-log enable sent: $ENA ; packet-C2H (rpkt_type=10) received: $C2H"
grep -oE "c2h.rx: cat=[0-9]+ cls=0x[0-9a-f]+ func=0x[0-9a-f]+" "$LOG" 2>/dev/null \
  | sort | uniq -c | head
if [ "$C2H" -gt 0 ]; then
  echo "RESULT: PASS — async packet-C2H delivery WORKS; the #236 C2H surface is"
  echo "        reachable. USR_TX_RPT non-firing is a fw gating quirk, not a gap."
else
  echo "RESULT: no packet-C2H — delivery gap (re-check RX + fw-log enable)."
fi
