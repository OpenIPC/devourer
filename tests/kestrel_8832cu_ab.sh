#!/usr/bin/env bash
# kestrel_8832cu_ab.sh — parametric 2-adapter TX->RX link test (both 35bc:0101).
# Env: BAND(''|6) CH BW(20|40|80|160) RATE(default 6M).
#
# SINGLE RX launch: the 8832CU comes up deaf on a *second* soft-re-init, so a
# two-phase (baseline rxdemo then TX-phase rxdemo) test reads 0 on the second
# launch even when the link is fine. Here we VBUS-cold both DUTs once, launch ONE
# rxdemo for the whole window, and start the TX flood partway through. At 5/6 GHz
# there is ~no ambient rate-4 (6M) traffic, so rate-4 frames in the capture come
# from our TX. RX=hub 3-2.3 p1 (bus3), TX=hub 9-1 p3 (bus9).
#   sudo BAND= CH=36 BW=20  tests/kestrel_8832cu_ab.sh
#   sudo BAND=6 CH=1 BW=160 tests/kestrel_8832cu_ab.sh
set -u
ROOT=/home/josephnef/git/devourer
BAND=${BAND:-}; CH=${CH:-36}; BW=${BW:-20}
# Rate must match the width: a 6M legacy frame is inherently 20 MHz and never
# exercises 40/80/160. Default a bandwidth-appropriate VHT MCS for wide channels;
# link is proven by rx.txhit (the canonical injected SA 57:42:75:05:d6:00), which
# is rate/BW-agnostic — not by a specific "rate" code.
if [ -z "${RATE:-}" ]; then
  case "$BW" in
    160) RATE="VHT2SS_MCS7/160" ;;
    80)  RATE="VHT2SS_MCS7/80" ;;
    40)  RATE="MCS7/40" ;;
    *)   RATE="6M" ;;
  esac
fi
RXBUS=3; TXBUS=9
uhubctl -l 3-2.3 -p 1 -a cycle >/dev/null 2>&1
uhubctl -l 9-1   -p 3 -a cycle >/dev/null 2>&1
sleep 4
for d in /sys/bus/usb/devices/*; do
  [ -f "$d/idVendor" ] || continue
  [ "$(cat $d/idVendor)" = "35bc" ] && [ "$(cat $d/idProduct)" = "0101" ] || continue
  for i in "$d":*; do [ -L "$i/driver" ] && echo "$(basename "$i")" > "$(readlink -f "$i/driver")/unbind" 2>/dev/null||true; done
done
sleep 1
O=/tmp/kab; mkdir -p "$O"; rm -f "$O"/*
LBL="band${BAND:-def}-ch$CH-bw$BW"
echo ">> $LBL : single RX bus$RXBUS (20 s); TX bus$TXBUS floods t=6..16 s"
timeout -s KILL 21 env DEVOURER_VID=0x35bc DEVOURER_PID=0x0101 DEVOURER_USB_BUS=$RXBUS \
  ${BAND:+DEVOURER_BAND=$BAND} DEVOURER_CHANNEL=$CH DEVOURER_BW=$BW DEVOURER_LOG_LEVEL=warn \
  "$ROOT/build/rxdemo" >"$O/rx.jsonl" 2>/dev/null &
RXP=$!
sleep 6
timeout -s KILL 10 env DEVOURER_VID=0x35bc DEVOURER_PID=0x0101 DEVOURER_USB_BUS=$TXBUS \
  ${BAND:+DEVOURER_BAND=$BAND} DEVOURER_CHANNEL=$CH DEVOURER_HOP_BW=$BW DEVOURER_TX_RATE=$RATE \
  DEVOURER_TX_PWR=16 DEVOURER_TX_GAP_US=300 DEVOURER_LOG_LEVEL=warn \
  "$ROOT/build/txdemo" >"$O/tx.out" 2>"$O/tx.err" &
TXP=$!
wait $RXP 2>/dev/null
kill -9 $TXP 2>/dev/null; pkill -9 -x -f build/txdemo 2>/dev/null
HIT=$(grep -c '"ev":"rx.txhit"' "$O/rx.jsonl")
ANY=$(grep -c '"ev":"rx.pkt"' "$O/rx.jsonl")
TXSENT=$(grep -c '"ev":"tx' "$O/tx.out" 2>/dev/null)
echo "   result[$RATE]: rx.txhit=$HIT  any-rx.pkt=$ANY  tx-events=$TXSENT"
if [ "$HIT" -ge 10 ]; then
  echo "   PASS ($LBL @ $RATE) — TX->RX link: $HIT canonical-SA hits"
else
  echo "   FAIL/INCONCLUSIVE ($LBL @ $RATE) — txhit=$HIT any=$ANY tx=$TXSENT"
fi
