#!/usr/bin/env bash
# kestrel_inject_sanity.sh — de-risk experiment #2 (plan task #6):
# Does the RTL8852BU's in-kernel driver (rtw89_8852bu) actually RADIATE
# monitor-mode injected frames?
#
# Injector : RTL8852BU (35bc:0108) under rtw89_8852bu, monitor mode, running
#            the canonical devourer beacon (SA 57:42:75:05:d6:00) via
#            tests/inject_beacon.py.
# Receiver : RTL8812AU (0bda:8812) under devourer rxdemo — a PROVEN monitor
#            receiver that emits an `rx.txhit` JSONL event on the canonical SA.
#            (in-tree rtw88 monitor RX is unreliable and no AR9271 is on this
#            rig, so devourer is the trustworthy air witness.)
#
# Decides whether tests/regress.py's kernel-TX cell is AUTHORITATIVE for the
# 8852BU — contrast the RTL8814AU, whose kernel-TX cells read 0 because its
# out-of-tree driver never emits host-pushed beacon injection.
#
#   sudo tests/kestrel_inject_sanity.sh [channel] [seconds]
set -u
cd "$(dirname "$0")/.."
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }

CH=${1:-6}
DUR=${2:-8}
SA="57:42:75:05:d6:00"
DUT_ID="35bc:0108"     # RTL8852BU  (injector, kernel rtw89_8852bu)
RX_ID="0bda:8812"      # RTL8812AU  (receiver, devourer rxdemo)
DUT_HUB="3-2.3"; DUT_PORT="3"   # uhubctl VBUS map for the 8852BU
RXDEMO="build/rxdemo"
RXLOG="/tmp/kestrel_inject_rx.jsonl"
[ -x "$RXDEMO" ] || { echo "FAIL: $RXDEMO not built (cmake --build build)"; exit 2; }

# --- resolve helpers (by VID:PID over sysfs) ---
sysdir_for() {
  local vid=${1%%:*} pid=${1##*:} d
  for d in /sys/bus/usb/devices/*; do
    [ -f "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor")" = "$vid" ] && [ "$(cat "$d/idProduct")" = "$pid" ] \
      && { echo "$d"; return; }
  done
}
netdev_for() { local d; d=$(sysdir_for "$1"); [ -n "$d" ] && ls "$d"/*:*/net/ 2>/dev/null | head -1; }

DUT=""; RXPID=""
cleanup() {
  [ -n "$RXPID" ] && kill "$RXPID" 2>/dev/null
  pkill -9 -f "inject_beacon.py" 2>/dev/null || true
  pkill -9 -f "rxdemo" 2>/dev/null || true
  [ -n "$DUT" ] && { ip link set "$DUT" down 2>/dev/null; iw dev "$DUT" set type managed 2>/dev/null; }
  # rebind rtw88 to the 8812AU receiver so the rig is left as found
  local rxdir iface
  rxdir=$(sysdir_for "$RX_ID")
  [ -n "$rxdir" ] && for iface in "$rxdir":*; do
    [ -d "$iface" ] && echo "$(basename "$iface")" > /sys/bus/usb/drivers/rtw88_8812au/bind 2>/dev/null || true
  done
}
trap cleanup EXIT

# ============ 1. injector: 8852BU under rtw89_8852bu, monitor mode ============
echo ">> VBUS-cycling the 8852BU ($DUT_ID) so rtw89_8852bu re-binds"
uhubctl -l "$DUT_HUB" -p "$DUT_PORT" -a cycle -d 2 >/dev/null 2>&1
sleep 7
DUT=$(netdev_for "$DUT_ID")
[ -n "$DUT" ] || { echo "FAIL: 8852BU has no netdev — rtw89_8852bu did not bind"; exit 2; }
echo "   injector netdev = $DUT"
ip link set "$DUT" down
if ! iw dev "$DUT" set type monitor 2>/dev/null; then
  echo "RESULT: rtw89_8852bu does NOT support monitor mode on this kernel — kernel-TX cell cannot be authoritative."
  exit 0
fi
ip link set "$DUT" up
iw dev "$DUT" set channel "$CH" 2>/dev/null || iw dev "$DUT" set channel "$CH" HT20 2>/dev/null
echo "   injector in monitor mode on ch$CH"

# ============ 2. receiver: 8812AU under devourer rxdemo ============
RXDIR=$(sysdir_for "$RX_ID")
[ -n "$RXDIR" ] || { echo "FAIL: 8812AU receiver not present"; exit 2; }
for iface in "$RXDIR":*; do
  [ -L "$iface/driver" ] && echo "$(basename "$iface")" > "$(readlink -f "$iface/driver")/unbind" 2>/dev/null || true
done
sleep 1
echo ">> starting devourer rxdemo on the 8812AU receiver (ch$CH)"
DEVOURER_VID=0x0bda DEVOURER_PID=0x8812 DEVOURER_CHANNEL=$CH \
  DEVOURER_EVENTS=stdout DEVOURER_LOG_LEVEL=warn \
  "$RXDEMO" >"$RXLOG" 2>/dev/null &
RXPID=$!
sleep 5   # rxdemo bring-up
if ! kill -0 "$RXPID" 2>/dev/null; then
  echo "FAIL: rxdemo receiver died during bring-up (see $RXLOG)"; exit 2
fi

# ============ 3. inject the canonical beacon from the 8852BU ============
echo ">> injecting the canonical beacon from the 8852BU for ${DUR}s"
( cd tests && uv run python inject_beacon.py --iface "$DUT" --duration "$DUR" \
    --interval 0.002 ) 2>&1 | tail -3 || echo "(injector returned nonzero)"
sleep 1
kill "$RXPID" 2>/dev/null; RXPID=""

# ============ 4. verdict ============
HITS=$(grep -c '"ev":"rx.txhit"' "$RXLOG" 2>/dev/null || echo 0)
TOTAL_RX=$(grep -c '"ev":"rx' "$RXLOG" 2>/dev/null || echo 0)
echo "=================================================================="
echo "8852BU kernel monitor-injection sanity (ch$CH, ${DUR}s):"
echo "  devourer receiver rx.txhit (canonical SA $SA): $HITS"
echo "  receiver total rx events                     : $TOTAL_RX"
if [ "${HITS:-0}" -gt 0 ]; then
  echo "RESULT: PASS — rtw89_8852bu radiates monitor-injected frames."
  echo "        => regress.py kernel-TX cell IS authoritative for the 8852BU."
else
  echo "RESULT: ZERO — no injected beacon reached the air witness."
  echo "        => kernel-TX cell NOT authoritative for the 8852BU (like the"
  echo "           8814AU); judge 8852BU TX by devourer-TX / SDR instead."
fi
