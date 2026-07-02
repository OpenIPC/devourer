#!/usr/bin/env bash
# STBC on-air sanity gate: does the chip actually EMIT decodable STBC?
#
# Sends canonical-SA beacons at MCS1/STBC from an RTL8814AU (which encodes
# Alamouti across two TX antennas internally, sidestepping the SDR 2-channel
# problem), and receives on an RTL8812AU — the 8812/8821 RX descriptor exposes
# the received HT-SIG STBC bit, which the 8814 RX does NOT (it reports defaults).
# A pass = frames arrive with stbc=1 in the <devourer-stream> line.
#
# Hardware (all on ONE host, adapters inches apart for a STRONG link — a clean
# yes/no needs the frame to decode easily; range confounds a zero result):
#   TX = RTL8814AU (0bda:8813), selected by USB_BUS
#   RX = RTL8812AU (0bda:8812)   ← must be an 8812/8821 to read the STBC bit
#
# Env:
#   TX_BUS=N [TX_PORT=a.b.c]   which 8814 transmits (topology select)
#   CHANNEL=6                  test channel
#   MCS=MCS1                   HT rate to carry STBC (STBC needs HT/VHT, not legacy)
#   DURATION=20                RX capture seconds
#
# Usage:  sudo TX_BUS=4 ./tests/stbc_sanity.sh
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
CHANNEL="${CHANNEL:-6}"; MCS="${MCS:-MCS1}"; DURATION="${DURATION:-20}"
RXLOG="$(mktemp -t devourer-stbc-rx.XXXXXX.log)"
TXLOG="$(mktemp -t devourer-stbc-tx.XXXXXX.log)"

cleanup() {
    for comm in WiFiDriverDemo WiFiDriverTxDem; do pkill -x "$comm" 2>/dev/null || true; done
    rm -f "$RXLOG" "$TXLOG" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "== building =="; cmake --build "$ROOT/build" -j >/dev/null
DEMO="$ROOT/build/WiFiDriverDemo"; TXDEMO="$ROOT/build/WiFiDriverTxDemo"

# Preflight: both chips present.
lsusb | grep -qi "0bda:8812" || { echo "no RTL8812AU (RX) present — plug it into this host" >&2; exit 1; }
lsusb | grep -qi "0bda:8813" || { echo "no RTL8814AU (TX) present" >&2; exit 1; }

echo "== TX: 8814 (bus ${TX_BUS:-first}) beacon at ${MCS}/STBC, ch$CHANNEL =="
TX_ENV=(DEVOURER_PID=0x8813 DEVOURER_CHANNEL="$CHANNEL" DEVOURER_TX_RATE="${MCS}/STBC")
[ -n "${TX_BUS:-}" ] && TX_ENV+=(DEVOURER_USB_BUS="$TX_BUS")
[ -n "${TX_PORT:-}" ] && TX_ENV+=(DEVOURER_USB_PORT="$TX_PORT")
stdbuf -oL -eL env "${TX_ENV[@]}" "$TXDEMO" >"$TXLOG" 2>&1 &
for _ in $(seq 1 30); do grep -q 'TX #.* rc=1' "$TXLOG" 2>/dev/null && break; sleep 1; done
grep -q 'TX #.* rc=1' "$TXLOG" || { echo "TX beacon never injected — check $TXLOG" >&2; exit 3; }
echo "   TX injecting ($(grep -m1 'fixed rate' "$TXLOG" 2>/dev/null || echo '?'))"

echo "== RX: 8812 for ${DURATION}s, reading the STBC bit =="
timeout "$((DURATION+2))" env DEVOURER_PID=0x8812 DEVOURER_STREAM_OUT=1 \
    DEVOURER_CHANNEL="$CHANNEL" "$DEMO" >"$RXLOG" 2>/dev/null &
sleep "$DURATION"; pkill -x WiFiDriverDemo 2>/dev/null || true; wait 2>/dev/null || true

TOTAL=$(grep -c '<devourer-stream>' "$RXLOG" || true)
STBC1=$(grep -c '<devourer-stream>.*stbc=1' "$RXLOG" || true)
echo "== result: canonical frames=$TOTAL  with stbc=1: $STBC1 =="
grep -m3 '<devourer-stream>' "$RXLOG" | sed -E 's/body=.*//'
if [ "${STBC1:-0}" -gt 0 ]; then
    echo "PASS — the chip emits decodable STBC on-air (stbc=1 seen). Proceed to the"
    echo "       TX-diversity mobility measurement (STBC vs single-stream, moving TX)."
elif [ "${TOTAL:-0}" -gt 0 ]; then
    echo "FAIL — frames decode but stbc=0: the 8814 is NOT emitting STBC for a"
    echo "       descriptor STBC bit. TX diversity would need CSD instead (#128)."
else
    echo "INCONCLUSIVE — no canonical frames. Move TX/RX closer (MCS1 needs a strong"
    echo "       link) or confirm both adapters are on ch$CHANNEL."
fi
