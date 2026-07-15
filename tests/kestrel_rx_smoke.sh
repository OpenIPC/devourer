#!/usr/bin/env bash
# kestrel_rx_smoke.sh — RX first-light test for the RTL8852BU. Detaches the
# in-kernel rtw89 driver, runs rxdemo through the full Kestrel monitor
# bring-up (power -> fw -> MAC TRX -> BB/RF -> channel) on ch6, and reports how
# many RX frames / beacons the 11ax parser decodes in a short window.
#
# Usage: sudo tests/kestrel_rx_smoke.sh [channel] [seconds]
set -u
cd "$(dirname "$0")/.."
VID=35bc PID=0108
CH="${1:-6}"
SECS="${2:-6}"
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }
[ -x build/rxdemo ] || { echo "FAIL: build/rxdemo missing"; exit 2; }

# VBUS cold-cycle first, like every sibling harness: a warm chip left by a
# previous run's clean Stop() brings up green but delivers no bulk-IN (the
# chip retains state across soft re-init).
TX_HUB="3-2.3"; TX_PORT="3"
uhubctl -l "$TX_HUB" -p "$TX_PORT" -a cycle -d 2 >/dev/null 2>&1
sleep 8

DEVDIR=""
for d in /sys/bus/usb/devices/*; do
  [ -f "$d/idVendor" ] || continue
  [ "$(cat "$d/idVendor")" = "$VID" ] && [ "$(cat "$d/idProduct")" = "$PID" ] \
    && { DEVDIR="$d"; break; }
done
[ -n "$DEVDIR" ] || { echo "FAIL: $VID:$PID not on the bus"; exit 2; }

REBIND=()
cleanup() {
  pkill -9 -f "build/rxdemo" 2>/dev/null
  local pair drv iface
  for pair in "${REBIND[@]:-}"; do
    [ -n "$pair" ] || continue
    drv="${pair% *}"; iface="${pair#* }"
    echo "$iface" > "$drv/bind" 2>/dev/null && echo "rebound $iface"
  done
}
trap cleanup EXIT

for ifdir in "$DEVDIR":*; do
  [ -L "$ifdir/driver" ] || continue
  drv="$(readlink -f "$ifdir/driver")"; iface="$(basename "$ifdir")"
  echo "unbinding $iface from $(basename "$drv")"
  echo "$iface" > "$drv/unbind"; REBIND+=("$drv $iface")
done
sleep 1

OUT=/tmp/kestrel_rx.jsonl
echo "running rxdemo on ch$CH for ${SECS}s..."
DEVOURER_VID=0x$VID DEVOURER_PID=0x$PID DEVOURER_CHANNEL=$CH \
  timeout "$SECS" build/rxdemo > "$OUT" 2> /tmp/kestrel_rx.log || true

# grep -c prints the count even on zero matches (exit 1) — no fallback echo,
# which would append a second "0" and break the -gt test below.
RXHIT=$(grep -cF '"ev":"rx.txhit"' "$OUT" 2>/dev/null) || true
FRAMES=$(grep -cE '"ev":"rx\.' "$OUT" 2>/dev/null) || true
echo "--- rxdemo stderr tail ---"; tail -6 /tmp/kestrel_rx.log
echo "--- events ($OUT) ---"; grep -E '"ev":"rx\.' "$OUT" 2>/dev/null | head -6
echo "rx events: $FRAMES ; rx.txhit: $RXHIT"
[ "$FRAMES" -gt 0 ] && { echo "PASS: RX first light — frames decoded"; exit 0; }
echo "FAIL: no RX frames decoded"; exit 1
