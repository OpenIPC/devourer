#!/usr/bin/env bash
# ack_txreport_matrix.sh — M0 contract 3: the unicast-ACK + TxReport capability
# matrix. For each TX generation, three phases against a fixed hardware-ACK
# responder (SetAckResponder on a second adapter):
#   on       — responder armed with MAC1, TX injects unicast QoS-Data to MAC1:
#              expect tx.report ok~1, retries~0 (hardware ACK closes the loop).
#   retarget — responder re-armed with a DIFFERENT unicast MAC2, TX targets
#              MAC2: proves the injected descriptor's RA and the responder MAC
#              are both arbitrary, not baked-in.
#   off      — no responder, TX to MAC1: expect ok~0, retries pinned at the
#              descriptor limit — the no-ACK outcome is VISIBLE per frame.
# Every phase also measures report_coverage (reports / frames sent) and, on
# HalMAC (J2/J3), SW_DEFINE tag-echo gaps + the firmware missed counter.
#
# TX sessions run with DEVOURER_TX_WITH_RX=thread: the CCX reports arrive on
# the C2H RX path, so a TX-only session without an RX loop never sees them on
# J1/J2 (J3 alone drains C2H off its coex runtime) — this matrix measures the
# capability with the RX loop up, the shape a scheduled MAC runs in anyway.
#
#   bash tests/ack_txreport_matrix.sh
#   CELLS="j3-8822cu:0x0bda:0xc812" SECS=10 bash tests/ack_txreport_matrix.sh
set -uo pipefail
cd "$(dirname "$0")/.."

# 8814AU responder. NOT the 8821AU: bench-measured, an armed 8821AU never
# closed the loop (TX retries stayed pinned) while the 8814AU ACKs ~100%.
RESP_VID=${RESP_VID:-0x0bda}; RESP_PID=${RESP_PID:-0x8813}
CH=${CH:-36}; SECS=${SECS:-8}; GAP_US=${GAP_US:-5000}
MAC1=${MAC1:-02:12:34:56:78:9a}
MAC2=${MAC2:-02:12:34:56:78:9b}
TX_SA=${TX_SA:-02:aa:bb:cc:dd:01}   # unicast TA (the ACK RA I/G footgun)
OUT=${OUT:-/tmp/ack_txreport}
CELLS=${CELLS:-"j1-8812au:0x0bda:0x8812 j2-8812bu:0x2357:0x012d j3-8822cu:0x0bda:0xc812"}

cleanup(){ sudo pkill -9 -x rxdemo 2>/dev/null; sudo pkill -9 -x txdemo 2>/dev/null; return 0; }
trap cleanup EXIT
mkdir -p "$OUT"
VERDICTS="$OUT/verdicts.jsonl"; : >"$VERDICTS"

run_phase() { # $1 cell $2 phase $3 tx vid $4 tx pid $5 RA mac $6 responder mac (""=off) $7 expect
  local cell="$1" phase="$2" vid="$3" pid="$4" ra="$5" resp="$6" expect="$7"
  local tag="${cell}_${phase}"
  cleanup; sleep 1
  if [ -n "$resp" ]; then
    sudo env DEVOURER_VID=$RESP_VID DEVOURER_PID=$RESP_PID DEVOURER_CHANNEL=$CH \
        DEVOURER_ACK_RESPONDER=$resp DEVOURER_LOG_LEVEL=info \
        ./build/rxdemo >"$OUT/resp_$tag.jsonl" 2>"$OUT/resp_$tag.err" &
    sleep 6   # responder bring-up
  fi
  sudo env DEVOURER_VID=$vid DEVOURER_PID=$pid DEVOURER_CHANNEL=$CH \
      DEVOURER_TX_QOS_DATA=1 DEVOURER_TX_RA=$ra DEVOURER_TX_SA=$TX_SA \
      DEVOURER_TX_RATE=MCS3 DEVOURER_TX_PAYLOAD_BYTES=200 \
      DEVOURER_TX_GAP_US=$GAP_US DEVOURER_TX_REPORT=1 \
      DEVOURER_TX_WITH_RX=thread DEVOURER_LOG_LEVEL=warn \
      timeout -s INT $SECS ./build/txdemo \
      >"$OUT/tx_$tag.jsonl" 2>"$OUT/tx_$tag.err" || true
  sleep 1
  cleanup; sleep 1
  # Frames sent = the last tx.stats 'submitted' counter (GetTxStats, emitted
  # every 500 frames) — the per-send stderr lines differ per generation.
  local sent
  sent=$(grep '"ev":"tx.stats"' "$OUT/tx_$tag.jsonl" | tail -1 |
         sed -n 's/.*"submitted":\([0-9]*\).*/\1/p')
  sent=${sent:-0}
  echo "-- $tag: sent=$sent reports=$(grep -c '"ev":"tx.report"' "$OUT/tx_$tag.jsonl" || true)"
  python3 tests/ack_txreport_analyze.py "$OUT/tx_$tag.jsonl" \
      --sent "$sent" --cell "$tag" --expect "$expect" | tee -a "$VERDICTS" || true
}

for cell in $CELLS; do
  name=${cell%%:*}; rest=${cell#*:}; vid=${rest%%:*}; pid=${rest#*:}
  echo
  echo "==== cell $name (TX $vid:$pid, ch$CH) ===="
  run_phase "$name" on       "$vid" "$pid" "$MAC1" "$MAC1" on
  run_phase "$name" retarget "$vid" "$pid" "$MAC2" "$MAC2" on
  run_phase "$name" off      "$vid" "$pid" "$MAC1" ""      off
done

echo
echo "==== MATRIX VERDICTS ($VERDICTS) ===="
cat "$VERDICTS"
