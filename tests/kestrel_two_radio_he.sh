#!/usr/bin/env bash
# kestrel_two_radio_he.sh — two-AX-radio HE TX->RX validation. One Kestrel
# (8852BU) airs HE-rate beacons (canonical injection SA); the other (8832CU)
# monitors the same channel and must decode them. This answers whether a second
# Kestrel can serve as the HE-capable monitor/peer that an 11ac witness (the
# 8812AU) cannot — i.e. whether the rig can validate aired HE frames (the
# prerequisite for on-air #236 trigger-frame decode) without extra hardware.
#
# PASS = the 8832CU monitor emits rx.txhit on the 8852BU's canonical SA AND at
# least one decoded frame carries an HE rate code (AX datarate >= 0x180).
#
#   sudo tests/kestrel_two_radio_he.sh [channel]
set -u
cd "$(dirname "$0")/.."
[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root"; exit 2; }
CH=${1:-36}
# The two dongles sit ~cm apart on the same hub; at 20 dBm the monitor's RX
# front-end saturates and decodes nothing. Drop TX power to land in a decodable
# near-field regime (override: 2nd arg = TX dBm).
TXPWR=${2:-0}
TXRATE=${3:-HE1SS_MCS0/20}   # 3rd arg: TX rate (control: pass 6M for legacy)
TX_ID="35bc:0108"; TX_HUB="3-2.3"; TX_PORT="3"   # 8852BU  (transmitter)
RX_ID="35bc:0101"; RX_HUB="3-2.3"; RX_PORT="1"   # 8832CU  (HE monitor)
RXOUT=/tmp/kestrel_2radio_rx.jsonl; RXLOG=/tmp/kestrel_2radio_rx.log
TXLOG=/tmp/kestrel_2radio_tx.log
[ -x build/txdemo ] && [ -x build/rxdemo ] || { echo "FAIL: build txdemo+rxdemo"; exit 2; }

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
TXPID=""; RXPID=""
cleanup() {
  [ -n "$TXPID" ] && kill "$TXPID" 2>/dev/null
  [ -n "$RXPID" ] && kill "$RXPID" 2>/dev/null
  pkill -9 -x -f "build/txdemo" 2>/dev/null || true
  pkill -9 -x -f "build/rxdemo" 2>/dev/null || true
}
trap cleanup EXIT

echo ">> cold-cycle both adapters"
uhubctl -l "$TX_HUB" -p "$TX_PORT" -a cycle -d 2 >/dev/null 2>&1
uhubctl -l "$RX_HUB" -p "$RX_PORT" -a cycle -d 2 >/dev/null 2>&1
sleep 8
sysdir_for "$TX_ID" >/dev/null || { echo "FAIL: 8852BU not on bus"; exit 2; }
sysdir_for "$RX_ID" >/dev/null || { echo "FAIL: 8832CU not on bus"; exit 2; }
unbind_kernel "$TX_ID"; unbind_kernel "$RX_ID"; sleep 1

echo ">> start 8832CU HE monitor on ch$CH"
env DEVOURER_VID=0x35bc DEVOURER_PID=0x0101 DEVOURER_CHANNEL=$CH \
    DEVOURER_LOG_LEVEL=warn DEVOURER_RX_KEEP_CORRUPTED=1 build/rxdemo >"$RXOUT" 2>"$RXLOG" &
RXPID=$!
sleep 5
kill -0 "$RXPID" 2>/dev/null || { echo "FAIL: rxdemo died"; tail -6 "$RXLOG"; exit 2; }

echo ">> start 8852BU HE-rate beacon TX on ch$CH ($TXRATE, ${TXPWR} dBm)"
env DEVOURER_VID=0x35bc DEVOURER_PID=0x0108 DEVOURER_CHANNEL=$CH \
    DEVOURER_TX_GAP_US=2000 DEVOURER_LOG_LEVEL=warn DEVOURER_TX_RATE="$TXRATE" \
    DEVOURER_TX_PWR="$TXPWR" \
    build/txdemo >"$TXLOG" 2>&1 &
TXPID=$!
sleep 8
kill "$TXPID" 2>/dev/null; TXPID=""
kill "$RXPID" 2>/dev/null; RXPID=""
sleep 1

hits=$(grep -cE '"ev":"rx\.txhit"' "$RXOUT" 2>/dev/null); hits=${hits:-0}
pkts=$(grep -cE '"ev":"rx\.pkt"' "$RXOUT" 2>/dev/null); pkts=${pkts:-0}
# The rate rides on rx.txhit (canonical-SA decode). HE codes are >= 0x180 (384).
he=$(grep -E '"ev":"rx\.txhit"' "$RXOUT" 2>/dev/null | grep -oE '"rate":[0-9]+' \
       | grep -oE '[0-9]+' | awk '$1>=384' | wc -l)
maxrate=$(grep -E '"ev":"rx\.txhit"' "$RXOUT" 2>/dev/null | grep -oE '"rate":[0-9]+' \
            | grep -oE '[0-9]+' | sort -n | tail -1)
echo "=================================================================="
echo "8832CU monitor: rx.pkt=$pkts  rx.txhit=$hits  HE-rate hits=$he  max-txhit-rate=${maxrate:-none}"
{ [ "$hits" -gt 0 ] && [ "$he" -gt 0 ]; } \
  && echo "RESULT: PASS — a 2nd Kestrel decodes the other's aired HE frames (rate>=0x180)" \
  || { echo "RESULT: FAIL — no HE-decoded canonical hits (near-field? try lower TX dBm)"; exit 1; }
