#!/usr/bin/env bash
# kestrel_8832cu_6g_txrx.sh — on-air 6 GHz TX+RX validation across two 8832CU
# adapters (both 35bc:0101, selected by USB bus via DEVOURER_USB_BUS). A/B: RX
# monitors 6 GHz with the TX off (baseline), then the other adapter floods 6 GHz
# 6M beacons while the first monitors. A jump in decoded rate-4 (6M) frames when
# the TX turns on proves the 6 GHz TX->RX link (no 6E AP needed). 6 GHz needs a
# tri-band 8852C adapter + DEVOURER_BAND=6. Force-kills so it cannot hang.
#   sudo tests/kestrel_8832cu_6g_txrx.sh   (RX=bus3, TX=bus9, ch37 by default)

ROOT=/home/josephnef/git/devourer
CH=37; RXBUS=3; TXBUS=9
O=/tmp/6gab; mkdir -p "$O"
for d in /sys/bus/usb/devices/*; do
  [ -f "$d/idVendor" ] || continue
  [ "$(cat $d/idVendor)" = "35bc" ] && [ "$(cat $d/idProduct)" = "0101" ] || continue
  for i in "$d":*; do [ -L "$i/driver" ] && echo "$(basename "$i")" > "$(readlink -f "$i/driver")/unbind" 2>/dev/null||true; done
done
sleep 1

echo ">> Phase 1: monitor bus$RXBUS @ 6G ch$CH, NO TX (12 s baseline)"
timeout -s KILL 13 env DEVOURER_VID=0x35bc DEVOURER_PID=0x0101 DEVOURER_USB_BUS=$RXBUS \
  DEVOURER_BAND=6 DEVOURER_CHANNEL=$CH DEVOURER_LOG_LEVEL=warn \
  "$ROOT/build/rxdemo" >"$O/p1.jsonl" 2>/dev/null
P1=$(grep -oE '"rate":4[,}]' "$O/p1.jsonl" | wc -l)  # rate-4 (6M) = the TX rate
echo "   Phase 1 (no TX): 6M(rate4) frames=$P1"

echo ">> Phase 2: TX bus$TXBUS floods 6G ch$CH while bus$RXBUS monitors (15 s)"
timeout -s KILL 18 env DEVOURER_VID=0x35bc DEVOURER_PID=0x0101 DEVOURER_USB_BUS=$RXBUS \
  DEVOURER_BAND=6 DEVOURER_CHANNEL=$CH DEVOURER_DUMP_BODY=1 DEVOURER_LOG_LEVEL=warn \
  "$ROOT/build/rxdemo" >"$O/p2.jsonl" 2>/dev/null &
RXP=$!
sleep 5
timeout -s KILL 10 env DEVOURER_VID=0x35bc DEVOURER_PID=0x0101 DEVOURER_USB_BUS=$TXBUS \
  DEVOURER_BAND=6 DEVOURER_CHANNEL=$CH DEVOURER_TX_RATE=6M DEVOURER_TX_PWR=16 \
  DEVOURER_TX_GAP_US=300 DEVOURER_LOG_LEVEL=warn \
  "$ROOT/build/txdemo" >/dev/null 2>"$O/tx.err" &
TXP=$!
wait $RXP 2>/dev/null
kill -9 $TXP 2>/dev/null
pkill -9 -x -f build/txdemo 2>/dev/null
P2=$(grep -oE '"rate":4[,}]' "$O/p2.jsonl" | wc -l)
echo "   Phase 2 (TX on): 6M(rate4) frames=$P2"
echo ">> VERDICT:"
# The TX floods 6M (rate 4). rate-4 frames appearing only under TX = the 6G
# TX->RX link (spurious rate 0/16/258 noise is present in both phases).
if [ "$P2" -ge 10 ] && [ "$P2" -gt $((P1 + 5)) ]; then
  echo "   PASS — 6 GHz TX(bus$TXBUS) -> RX(bus$RXBUS) link confirmed"
  echo "   (6M frames ${P1} with TX off -> ${P2} with TX on)"
else
  echo "   INCONCLUSIVE — 6M frames off=$P1 on=$P2"
fi
