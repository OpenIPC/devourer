#!/usr/bin/env bash
# kestrel_8832cu_6g160_txrx.sh — on-air 6 GHz *160 MHz* TX+RX across two 8832CU
# adapters (both 35bc:0101, by USB bus). Same A/B method as the 20 MHz test, but
# both ends tune 6 GHz ch1 at 160 MHz (RF synth center ch15 = 6025 MHz). The RF
# synth locks here (RF 0xb7[8]=0, verified by kestrel_6g160_lck_probe.sh); this
# confirms the tune reaches the air. TX width = DEVOURER_HOP_BW; RX = DEVOURER_BW.
# 6M legacy rides the primary 20; the 160 MHz occupied bandwidth itself is
# SDR-proven at 5 GHz (B210 tops out below the 6 GHz 160 block center).
#   sudo tests/kestrel_8832cu_6g160_txrx.sh   (RX=bus3, TX=bus9, ch1/160)

ROOT=/home/josephnef/git/devourer
CH=1; RXBUS=3; TXBUS=9
# TRUE cold per run — the 8832CU retains a deaf state across soft re-init, so a
# kernel-driver unbind alone leaves it decoding nothing. VBUS-cycle both DUTs:
# RX = hub 3-2.3 port 1, TX = hub 9-1 port 3 (smart-hub downstream ports, ppps).
uhubctl -l 3-2.3 -p 1 -a cycle >/dev/null 2>&1
uhubctl -l 9-1   -p 3 -a cycle >/dev/null 2>&1
sleep 4
O=/tmp/6g160ab; mkdir -p "$O"
for d in /sys/bus/usb/devices/*; do
  [ -f "$d/idVendor" ] || continue
  [ "$(cat $d/idVendor)" = "35bc" ] && [ "$(cat $d/idProduct)" = "0101" ] || continue
  for i in "$d":*; do [ -L "$i/driver" ] && echo "$(basename "$i")" > "$(readlink -f "$i/driver")/unbind" 2>/dev/null||true; done
done
sleep 1

echo ">> Phase 1: monitor bus$RXBUS @ 6G ch$CH/160, NO TX (12 s baseline)"
timeout -s KILL 13 env DEVOURER_VID=0x35bc DEVOURER_PID=0x0101 DEVOURER_USB_BUS=$RXBUS \
  DEVOURER_BAND=6 DEVOURER_CHANNEL=$CH DEVOURER_BW=160 DEVOURER_LOG_LEVEL=warn \
  "$ROOT/build/rxdemo" >"$O/p1.jsonl" 2>/dev/null
P1=$(grep -oE '"rate":4[,}]' "$O/p1.jsonl" | wc -l)
echo "   Phase 1 (no TX): 6M(rate4) frames=$P1"

echo ">> Phase 2: TX bus$TXBUS floods 6G ch$CH/160 while bus$RXBUS monitors (15 s)"
timeout -s KILL 18 env DEVOURER_VID=0x35bc DEVOURER_PID=0x0101 DEVOURER_USB_BUS=$RXBUS \
  DEVOURER_BAND=6 DEVOURER_CHANNEL=$CH DEVOURER_BW=160 DEVOURER_LOG_LEVEL=warn \
  "$ROOT/build/rxdemo" >"$O/p2.jsonl" 2>/dev/null &
RXP=$!
sleep 5
timeout -s KILL 10 env DEVOURER_VID=0x35bc DEVOURER_PID=0x0101 DEVOURER_USB_BUS=$TXBUS \
  DEVOURER_BAND=6 DEVOURER_CHANNEL=$CH DEVOURER_HOP_BW=160 DEVOURER_TX_RATE=6M \
  DEVOURER_TX_PWR=16 DEVOURER_TX_GAP_US=300 DEVOURER_LOG_LEVEL=warn \
  "$ROOT/build/txdemo" >/dev/null 2>"$O/tx.err" &
TXP=$!
wait $RXP 2>/dev/null
kill -9 $TXP 2>/dev/null
pkill -9 -x -f build/txdemo 2>/dev/null
P2=$(grep -oE '"rate":4[,}]' "$O/p2.jsonl" | wc -l)
echo "   Phase 2 (TX on): 6M(rate4) frames=$P2"
echo ">> VERDICT:"
if [ "$P2" -ge 10 ] && [ "$P2" -gt $((P1 + 5)) ]; then
  echo "   PASS — 6 GHz 160 MHz TX(bus$TXBUS) -> RX(bus$RXBUS) link confirmed"
  echo "   (6M frames ${P1} with TX off -> ${P2} with TX on)"
else
  echo "   INCONCLUSIVE — 6M frames off=$P1 on=$P2"
fi
