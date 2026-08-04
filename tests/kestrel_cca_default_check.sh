#!/usr/bin/env bash
# Kestrel carrier-sense TX validation — the measurement behind the per-die
# CCA default (RtlKestrelDevice bring-up). Three phases on one witness:
#
#  1) gates-on-from-bring-up TX (DEVOURER_KESTREL_CCA_ON=1): full-rate flow
#     is the pass (the 8852C measured 2106 frames/12 s; a CSMA freeze reads
#     as near-zero + a bulk-OUT stall in tx.err).
#  2) live gate bisect: R_AX_CCA_CFG_0 low-byte poked per arm (none / CCA /
#     CCA+SEC / EDCCA / ALL) via chipstate --no-claim — per-arm frame rates
#     localize a misbehaving detector.
#  3) deferral A/B under a co-channel flood: gates-on must defer, DIS_CCA=1
#     must punch through (8852C: 35 vs 83 frames/10 s).
#
# Run with the die under test as TX (TX_VID/TX_PID); the 8852B arm is the
# open one — a pass there flips its bring-up default too.
#
#   sudo TX_VID=0x35bc TX_PID=0x0101 bash tests/kestrel_cca_default_check.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/build"
OUT=${OUT:-/tmp/kestrel-cca}
CH=${CH:-6}
TX_VID=${TX_VID:-0x35bc}; TX_PID=${TX_PID:-0x0101}
FLOOD_PID=${FLOOD_PID:-0xc812}
WIT_PID=${WIT_PID:-0xb812}
TX_SA=02:aa:bb:cc:dd:01
RA=02:de:ad:be:ef:01
ESC=$(printf '%s' "$BUILD" | sed 's/[][\.*^$/]/\\&/g')
KILL(){ sudo pkill -9 -f "^$ESC/rxdemo" 2>/dev/null; sudo pkill -9 -f "^$ESC/txdemo" 2>/dev/null; return 0; }
trap KILL EXIT
mkdir -p "$OUT"

wit_up(){
  sudo env DEVOURER_PID=$WIT_PID DEVOURER_CHANNEL=$CH \
       DEVOURER_RX_PCTR=1 DEVOURER_RX_AGG_SA=$TX_SA DEVOURER_LOG_LEVEL=info \
       "$BUILD/rxdemo" >"$OUT/wit.jsonl" 2>"$OUT/wit.err" &
  local w=0
  until grep -qE "async ring of .* URBs submitted|Listening air" "$OUT/wit.err"; do
    sleep 1; w=$((w+1))
    [ $w -ge 25 ] && { echo "ABORT: witness never started" >&2; exit 1; }
  done; sleep 1
}
# grep -c prints the 0 itself (exiting 1) — || true keeps set -u-safe without
# emitting a second zero line into $(count).
count(){ grep -c '"ev":"rx.seq"' "$OUT/wit.jsonl" 2>/dev/null || true; }

echo "=== 1) gates-on from bring-up ==="
KILL; sleep 1; : >"$OUT/wit.jsonl"; wit_up
sudo env DEVOURER_VID=$TX_VID DEVOURER_PID=$TX_PID DEVOURER_CHANNEL=$CH \
     DEVOURER_KESTREL_CCA_ON=1 DEVOURER_TX_RATE=MCS3 DEVOURER_TX_QOS_DATA=1 \
     DEVOURER_TX_RA=$RA DEVOURER_TX_SA=$TX_SA DEVOURER_TX_PAYLOAD_BYTES=200 \
     DEVOURER_TX_GAP_US=5000 DEVOURER_LOG_LEVEL=info \
     timeout -s INT 18 "$BUILD/txdemo" >/dev/null 2>"$OUT/tx1.err" || true
echo "gates-on-from-bring-up: $(count) frames witnessed (~10 s steady TX)"
KILL; sleep 1

echo "=== 2) live gate bisect ==="
: >"$OUT/wit.jsonl"; wit_up
sudo env DEVOURER_VID=$TX_VID DEVOURER_PID=$TX_PID DEVOURER_CHANNEL=$CH \
     DEVOURER_TX_RATE=MCS3 DEVOURER_TX_QOS_DATA=1 DEVOURER_TX_RA=$RA \
     DEVOURER_TX_SA=$TX_SA DEVOURER_TX_PAYLOAD_BYTES=200 \
     DEVOURER_TX_GAP_US=5000 DEVOURER_DIS_CCA=1 DEVOURER_LOG_LEVEL=warn \
     "$BUILD/txdemo" >/dev/null 2>"$OUT/tx2.err" &
sleep 8
BASE_HI=$(sudo "$BUILD/chipstate" --vid $TX_VID --pid $TX_PID --no-claim \
          --peek 0xc340-0xc343 2>/dev/null | tail -1 | awk '{print $3$4$5}')
for A in c0 c1 cf d0 df c0; do
  sudo "$BUILD/chipstate" --vid $TX_VID --pid $TX_PID --no-claim \
       --poke "0xc341=0x${BASE_HI:0:2}" >/dev/null 2>&1 || true
  sudo "$BUILD/chipstate" --vid $TX_VID --pid $TX_PID --no-claim \
       --poke "0xc340=0x$A" >/dev/null 2>&1
  C0=$(count); sleep 5; C1=$(count)
  echo "gate byte 0x$A: $((C1-C0)) frames / 5 s"
done
KILL; sleep 1

echo "=== 3) deferral A/B under co-channel flood ==="
: >"$OUT/wit.jsonl"; wit_up
sudo env DEVOURER_PID=$FLOOD_PID DEVOURER_CHANNEL=$CH DEVOURER_TX_RATE=MCS1 \
     DEVOURER_TX_GAP_US=0 DEVOURER_DIS_CCA=1 DEVOURER_LOG_LEVEL=warn \
     "$BUILD/txdemo" >/dev/null 2>"$OUT/flood.err" &
sleep 8
for ARM in ccaon discca; do
  E="DEVOURER_KESTREL_CCA_ON=1"; [ "$ARM" = discca ] && E="DEVOURER_DIS_CCA=1"
  sudo env DEVOURER_VID=$TX_VID DEVOURER_PID=$TX_PID DEVOURER_CHANNEL=$CH $E \
       DEVOURER_TX_RATE=MCS3 DEVOURER_TX_QOS_DATA=1 DEVOURER_TX_RA=$RA \
       DEVOURER_TX_SA=$TX_SA DEVOURER_TX_PAYLOAD_BYTES=200 \
       DEVOURER_TX_GAP_US=5000 DEVOURER_LOG_LEVEL=warn \
       timeout -s INT 10 "$BUILD/txdemo" >/dev/null 2>"$OUT/tx3_$ARM.err" || true
  echo "$(count)" >"$OUT/c_$ARM"; sleep 1
done
KILL
A=$(cat "$OUT/c_ccaon"); B=$(cat "$OUT/c_discca")
echo "under flood: gates-ON=$A  dis_cca=$((B-A))  (10 s each; deferral = ON < dis_cca)"
echo "raw: $OUT"
