#!/usr/bin/env bash
# A-MPDU on-air efficiency A/B — SDR ground truth (per bench discipline:
# judge TX by SDR duty x PHY rate, never receiver frame counts; the RX-capture
# comparisons in tests/ampdu_pacing_sweep.sh are relative-only).
#
# Modes (same adapter, same clean 5 GHz channel, MCS7/20, 1000-byte PSDUs):
#   singles — the classic per-frame flood (the baseline air behaviour)
#   ampdu   — data queue + AGG_EN + retry0 + batched feed + the pacing pokes
#             the sweep found (0x4bc burst-ctrl cleared, 0x455 = 0x20)
#
# Per mode: TX floods for $((SECS+10)) s; one sdr_duty read (retried once if
# the B210 fails to reacquire and reports ~0 — the known back-to-back trap)
# taken mid-flood; TX-side accepted fps from tx.stats. The verdict metric is
# accepted-fps per duty point: more frames per unit airtime = real air-time
# efficiency.
#
#   sudo bash tests/ampdu_onair_ab.sh
#   CH=157 SECS=6 sudo bash tests/ampdu_onair_ab.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/build"

TX_PID=${TX_PID:-0x012d}; TX_VID=${TX_VID:-0x2357}
CH=${CH:-149}; SECS=${SECS:-4}; MCS=${MCS:-7}; BW=${BW:-20}
PAYLOAD=${PAYLOAD:-1000}
OUT=${OUT:-/tmp/ampdu_onair}

KILL(){ pkill -9 -x txdemo 2>/dev/null; return 0; }
trap KILL EXIT
mkdir -p "$OUT"
FREQ=$(( (5000 + 5 * CH) ))e6

echo "455 1 20" >  "$OUT/pacing.wseq"
echo "4bc 1 0"  >> "$OUT/pacing.wseq"

duty_read() { # one sdr_duty read; retry once on the ~0 reacquire failure
  local out
  out=$(python3 "$ROOT/tests/sdr_duty.py" --freq "$FREQ" --rate 25e6 \
        --secs "$SECS" --mcs "$MCS" --bw "$BW" 2>&1 | tail -3)
  echo "$out"
  if echo "$out" | grep -qE "duty=0\.0%"; then
    echo "(reacquire retry)"
    sleep 3
    python3 "$ROOT/tests/sdr_duty.py" --freq "$FREQ" --rate 25e6 \
        --secs "$SECS" --mcs "$MCS" --bw "$BW" 2>&1 | tail -3
  fi
}

run_mode() { # $1 = name, rest = extra TX env
  local name="$1"; shift
  echo "=== mode $name ($*)"
  KILL; sleep 1
  sudo env DEVOURER_VID=$TX_VID DEVOURER_PID=$TX_PID DEVOURER_CHANNEL=$CH \
       DEVOURER_TX_RATE=MCS$MCS/$BW DEVOURER_TX_QOS_DATA=1 \
       DEVOURER_TX_QOS_NOACK=1 DEVOURER_TX_PAYLOAD_BYTES=$PAYLOAD \
       DEVOURER_TX_GAP_US=0 DEVOURER_LOG_LEVEL=warn "$@" \
       timeout -s INT $((SECS + 14)) "$BUILD/txdemo" \
       >"$OUT/tx_$name.jsonl" 2>"$OUT/tx_$name.err" &
  sleep 7 # bring-up + steady state
  duty_read | tee "$OUT/duty_$name.txt"
  wait || true
  KILL; sleep 2
  grep '"ev":"tx.stats"' "$OUT/tx_$name.jsonl" | tail -1
}

run_mode singles
run_mode ampdu DEVOURER_TX_QSEL=0 DEVOURER_TX_AMPDU=16/7/0 \
               DEVOURER_TX_BATCH=3 DEVOURER_TX_USB_AGG=3 \
               DEVOURER_REPLAY_WSEQ="$OUT/pacing.wseq"

echo
echo "=== SUMMARY (ch$CH MCS$MCS/$BW payload=$PAYLOAD) ==="
for m in singles ampdu; do
  duty=$(grep -oE "duty[ =:]+[0-9.]+" "$OUT/duty_$m.txt" | tail -1 | grep -oE "[0-9.]+$")
  stats=$(grep '"ev":"tx.stats"' "$OUT/tx_$m.jsonl" | tail -1)
  sub=$(echo "$stats" | grep -oE '"submitted":[0-9]+' | grep -oE '[0-9]+')
  fail=$(echo "$stats" | grep -oE '"failed":[0-9]+' | grep -oE '[0-9]+')
  frames=$(grep '"ev":"tx.frame"' "$OUT/tx_$m.jsonl" | tail -1 | grep -oE '"n":[0-9]+' | grep -oE '[0-9]+')
  echo "$m: duty=${duty:-?} tx_frames=${frames:-?} urb_submitted=${sub:-?} urb_failed=${fail:-?}"
done
echo "raw logs: $OUT"
