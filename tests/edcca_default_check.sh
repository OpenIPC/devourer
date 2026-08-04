#!/usr/bin/env bash
# Carrier-sense/EDCCA default validation on Jaguar1: register proof (0x8a4
# thresholds programmed vs parked, 0x520[15:14] clear vs set) + on-air
# deferral A/B — the DUT TX counted by a witness while a co-channel flooder
# holds the channel at max duty. Default (carrier-sense + EDCCA on) must
# defer hard; DIS_CCA=1 must punch through (measured 48 vs 376 frames /
# 10 s on the 8821AU at ch6).
#
#   sudo bash tests/edcca_default_check.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/build"
OUT=/tmp/edcca-val; mkdir -p "$OUT"
CH=6
J1_VID=0x2357 J1_PID=0x0120     # 8821AU DUT TX
FLOOD_PID=0xc812                 # 8812CU co-channel flooder
WIT_PID=0xb812                   # 8822BU witness
TX_SA=02:aa:bb:cc:dd:01
RA=02:de:ad:be:ef:01
ESC=$(printf '%s' "$BUILD" | sed 's/[][\.*^$/]/\\&/g')
KILL(){ sudo pkill -9 -f "^$ESC/rxdemo" 2>/dev/null; sudo pkill -9 -f "^$ESC/txdemo" 2>/dev/null; return 0; }
trap KILL EXIT

echo "=== 1) register proof: default (EDCCA on) ==="
KILL; sleep 1
sudo env DEVOURER_VID=$J1_VID DEVOURER_PID=$J1_PID DEVOURER_CHANNEL=$CH \
     DEVOURER_LOG_LEVEL=info "$BUILD/rxdemo" >/dev/null 2>"$OUT/dflt.err" &
w=0; until grep -q "MAC carrier-sense" "$OUT/dflt.err"; do
  sleep 1; w=$((w+1)); [ $w -ge 25 ] && { echo ABORT-no-cca-log; exit 1; }
done
grep -E "EDCCA thresholds|carrier-sense" "$OUT/dflt.err" | head -2
sudo "$BUILD/chipstate" --vid $J1_VID --pid $J1_PID --no-claim \
     --peek 0x8a4-0x8a7 --peek 0x520-0x523 2>/dev/null
KILL; sleep 1

echo "=== 2) register proof: DIS_CCA=1 ==="
sudo env DEVOURER_VID=$J1_VID DEVOURER_PID=$J1_PID DEVOURER_CHANNEL=$CH \
     DEVOURER_DIS_CCA=1 DEVOURER_LOG_LEVEL=info \
     "$BUILD/rxdemo" >/dev/null 2>"$OUT/dis.err" &
w=0; until grep -q "MAC carrier-sense" "$OUT/dis.err"; do
  sleep 1; w=$((w+1)); [ $w -ge 25 ] && { echo ABORT-no-cca-log; exit 1; }
done
grep -E "carrier-sense" "$OUT/dis.err" | head -1
sudo "$BUILD/chipstate" --vid $J1_VID --pid $J1_PID --no-claim \
     --peek 0x8a4-0x8a7 --peek 0x520-0x523 2>/dev/null
KILL; sleep 1

echo "=== 3) on-air deferral A/B under a co-channel flood ==="
# witness hears the DUT SA; flooder screams broadcast at max duty
sudo env DEVOURER_PID=$WIT_PID DEVOURER_CHANNEL=$CH \
     DEVOURER_RX_PCTR=1 DEVOURER_RX_AGG_SA=$TX_SA DEVOURER_LOG_LEVEL=info \
     "$BUILD/rxdemo" >"$OUT/wit.jsonl" 2>"$OUT/wit.err" &
w=0; until grep -qE "async ring of .* URBs submitted|Listening air" "$OUT/wit.err"; do
  sleep 1; w=$((w+1)); [ $w -ge 25 ] && { echo ABORT-wit; exit 1; }
done
sudo env DEVOURER_PID=$FLOOD_PID DEVOURER_CHANNEL=$CH DEVOURER_TX_RATE=MCS1 \
     DEVOURER_TX_GAP_US=0 DEVOURER_DIS_CCA=1 DEVOURER_LOG_LEVEL=warn \
     "$BUILD/txdemo" >/dev/null 2>"$OUT/flood.err" &
sleep 8   # flooder bring-up + duty ramp
for ARM in default discca; do
  EXTRA=""; [ "$ARM" = discca ] && EXTRA="DEVOURER_DIS_CCA=1"
  T0=$(date +%s%N)
  sudo env DEVOURER_VID=$J1_VID DEVOURER_PID=$J1_PID DEVOURER_CHANNEL=$CH \
       DEVOURER_TX_RATE=MCS3 DEVOURER_TX_QOS_DATA=1 DEVOURER_TX_RA=$RA \
       DEVOURER_TX_SA=$TX_SA DEVOURER_TX_PAYLOAD_BYTES=200 \
       DEVOURER_TX_GAP_US=5000 $EXTRA DEVOURER_LOG_LEVEL=warn \
       timeout -s INT 10 "$BUILD/txdemo" >"$OUT/tx_$ARM.jsonl" 2>"$OUT/tx_$ARM.err" || true
  T1=$(date +%s%N)
  N=$(grep -c '"ev":"rx.seq"' "$OUT/wit.jsonl" || true)
  echo "$ARM: witness total so far $N (wall $(( (T1-T0)/1000000 )) ms)"
  echo "$N" > "$OUT/cum_$ARM"
  sleep 1
done
KILL
A=$(cat "$OUT/cum_default"); B=$(cat "$OUT/cum_discca")
echo "witnessed DUT frames: default(EDCCA+CCA on)=$A  dis_cca=$((B-A))  (same 10 s window each)"
echo "raw: $OUT"
