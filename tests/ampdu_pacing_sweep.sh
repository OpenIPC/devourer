#!/usr/bin/env bash
# A-MPDU aggregate-launch pacing sweep. The spike (tests/ampdu_spike.sh,
# docs/aggregation.md) established that the MAC hardware-aggregates
# host-pushed QoS-Data on a data queue, but flow-controls each aggregate
# launch at ~3 ms under sustained feed — tracking REG_AMPDU_MAX_TIME_V1
# (0x455 = 0x70, the aggregate-fill timer) — which caps net A-MPDU goodput
# BELOW plain singles at MCS7/20.
#
# This sweep pokes the candidate pacing registers post-bring-up on the TX
# adapter via the DEVOURER_REPLAY_WSEQ hook (now applied at the end of
# InitWrite too) and measures unique-frame goodput + burst structure on a
# second devourer receiver:
#   - 0x455  AMPDU max time (unit ~32 us): 0x70 baseline, then shrinking
#   - 0x4bc  SW_AMPDU_BURST_MODE_CTRL variants
#
#   sudo bash tests/ampdu_pacing_sweep.sh
#   TX_PID=0x012d RX_PID=0xc812 SECS=8 sudo bash tests/ampdu_pacing_sweep.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/build"

TX_PID=${TX_PID:-0x012d}; TX_VID=${TX_VID:-0x2357}
RX_PID=${RX_PID:-0xc812}; RX_VID=${RX_VID:-0x0bda}
CH=${CH:-6}; SECS=${SECS:-8}; RATE=${RATE:-MCS7}
PAYLOAD=${PAYLOAD:-1000}
OUT=${OUT:-/tmp/ampdu_pacing}

KILL(){ pkill -9 -x rxdemo 2>/dev/null; pkill -9 -x txdemo 2>/dev/null; return 0; }
trap KILL EXIT
mkdir -p "$OUT"

run_cell() { # $1 = cell name, $2 = wseq file ("" = none), rest = extra TX env
  local name="$1" wseq="$2"; shift 2
  echo "=== cell $name (wseq='$wseq' $*)"
  KILL; sleep 1
  sudo env DEVOURER_VID=$RX_VID DEVOURER_PID=$RX_PID DEVOURER_CHANNEL=$CH \
       DEVOURER_STREAM_OUT=1 DEVOURER_EVENT_FLUSH=0 DEVOURER_LOG_LEVEL=warn \
       "$BUILD/rxdemo" >"$OUT/rx_$name.jsonl" 2>"$OUT/rx_$name.err" &
  sleep 6
  local extra=()
  [ -n "$wseq" ] && extra=(DEVOURER_REPLAY_WSEQ="$wseq")
  sudo env DEVOURER_VID=$TX_VID DEVOURER_PID=$TX_PID DEVOURER_CHANNEL=$CH \
       DEVOURER_TX_RATE=$RATE DEVOURER_TX_QOS_DATA=1 DEVOURER_TX_QOS_NOACK=1 \
       DEVOURER_TX_PAYLOAD_BYTES=$PAYLOAD DEVOURER_TX_GAP_US=0 \
       DEVOURER_TX_QSEL=0 DEVOURER_TX_AMPDU=16/7/0 \
       DEVOURER_TX_BATCH=3 DEVOURER_TX_USB_AGG=3 \
       DEVOURER_LOG_LEVEL=warn "${extra[@]}" "$@" \
       timeout -s INT $SECS "$BUILD/txdemo" \
       >"$OUT/tx_$name.jsonl" 2>"$OUT/tx_$name.err" || true
  sleep 1
  KILL; sleep 1
}

# wseq files: "<addr_hex> <width> <val_hex>" per line.
mk_wseq() { echo "$2" > "$OUT/$1.wseq"; echo "$OUT/$1.wseq"; }

W_T20=$(mk_wseq t20  "455 1 20")   # max time 0x20 (~1 ms)
W_T08=$(mk_wseq t08  "455 1 8")    # ~256 us
W_T02=$(mk_wseq t02  "455 1 2")    # ~64 us
W_T00=$(mk_wseq t00  "455 1 0")    # timer off (behaviour unknown)
W_B00=$(mk_wseq b00  "4bc 1 0")    # burst-mode ctrl cleared
W_BC0=$(mk_wseq bc0  "4bc 1 c0")   # burst-mode bits 7:6
W_T08B=$(mk_wseq t08b "455 1 8
4bc 1 c0")                          # combined

run_cell base   ""        # 0x455=0x70 baseline (bring-up value)
run_cell t20    "$W_T20"
run_cell t08    "$W_T08"
run_cell t02    "$W_T02"
run_cell t00    "$W_T00"
run_cell b00    "$W_B00"
run_cell bc0    "$W_BC0"
run_cell t08bc0 "$W_T08B"
# singles control for the session (same air conditions)
KILL; sleep 1
sudo env DEVOURER_VID=$RX_VID DEVOURER_PID=$RX_PID DEVOURER_CHANNEL=$CH \
     DEVOURER_STREAM_OUT=1 DEVOURER_EVENT_FLUSH=0 DEVOURER_LOG_LEVEL=warn \
     "$BUILD/rxdemo" >"$OUT/rx_singles.jsonl" 2>"$OUT/rx_singles.err" &
sleep 6
sudo env DEVOURER_VID=$TX_VID DEVOURER_PID=$TX_PID DEVOURER_CHANNEL=$CH \
     DEVOURER_TX_RATE=$RATE DEVOURER_TX_QOS_DATA=1 DEVOURER_TX_QOS_NOACK=1 \
     DEVOURER_TX_PAYLOAD_BYTES=$PAYLOAD DEVOURER_TX_GAP_US=0 \
     DEVOURER_LOG_LEVEL=warn \
     timeout -s INT $SECS "$BUILD/txdemo" \
     >"$OUT/tx_singles.jsonl" 2>"$OUT/tx_singles.err" || true
KILL

echo
echo "=== RESULTS (payload=$PAYLOAD rate=$RATE secs=$SECS; cells = 0x455/0x4bc pokes) ==="
python3 - "$OUT" "$PAYLOAD" "$SECS" <<'PYEOF'
import glob, json, os, sys

out, payload, secs = sys.argv[1], int(sys.argv[2]), float(sys.argv[3])
want_len = {payload, payload + 4}

def jload(line):
    try:
        return json.loads(line)
    except json.JSONDecodeError:
        return {}

order = ["singles", "base", "t20", "t08", "t02", "t00", "b00", "bc0", "t08bc0"]
for cell in order:
    path = os.path.join(out, f"rx_{cell}.jsonl")
    if not os.path.exists(path):
        continue
    frames, uniq = [], set()
    for line in open(path, errors="replace"):
        if not line.startswith('{"ev":"rx.frame"'):
            continue
        ev = jload(line)
        if ev.get("len") in want_len and not ev.get("crc"):
            frames.append(ev)
            body = ev.get("body", "")
            if len(body) >= 12:
                uniq.add(body[4:12])
    n = len(frames)
    if n == 0:
        print(f"{cell:>8}: NO FRAMES (cell failed — check tx/rx err logs)")
        continue
    paggr = sum(1 for f in frames if f.get("paggr"))
    bursts, cur = [], 1
    gaps = []
    for a, b in zip(frames, frames[1:]):
        dt = (b.get("tsfl", 0) - a.get("tsfl", 0)) & 0xFFFFFFFF
        if dt == 0:
            cur += 1
        else:
            bursts.append(cur); cur = 1
            gaps.append(dt)
    bursts.append(cur)
    gaps.sort()
    gap50 = gaps[len(gaps)//2] if gaps else 0
    print(f"{cell:>8}: uniq_fps={len(uniq)/secs:6.0f}  rx={n} unique={len(uniq)} "
          f"paggr={100*paggr//n}% mean_burst={sum(bursts)/len(bursts):.1f} "
          f"max_burst={max(bursts)} inter_gap_p50={gap50}us")
PYEOF
echo "raw logs: $OUT"
