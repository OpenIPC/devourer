#!/usr/bin/env bash
# A-MPDU feasibility spike: can the MAC hardware aggregate HOST-PUSHED
# (monitor-inject, USE_RATE=1) frames into A-MPDUs, and under which conditions?
#
# Matrix axes (one txdemo run per cell, QoS-Data TID0 frames at $RATE):
#   - QSEL: the monitor-inject MGMT queue (0x12) vs a data queue (TID 0)
#   - descriptor AGG_EN/MAX_AGG_NUM/AMPDU_DENSITY on vs off (DEVOURER_TX_AMPDU)
#   - delivery: gapless single-frame URBs vs one multi-frame URB
#     (DEVOURER_TX_BATCH + DEVOURER_TX_USB_AGG) — is one-URB co-queueing
#     required for the MAC to see aggregatable frames?
#   - QoS ack policy: No-Ack broadcast (the wfb-style flavor) is the default;
#     FULL=1 adds a unicast/normal-ack cell (expect hardware retry storms —
#     the monitor-mode receiver never ACKs).
#
# Detection = RX-side ground truth on a second devourer adapter in monitor
# mode (DEVOURER_STREAM_OUT=1 -> per-frame rx.frame events):
#   - paggr: rx-desc "inside an aggregate" marker
#   - ppdu: the halmac 2-bit received-PPDU counter — runs of the same value
#     are subframes of ONE PPDU (burst-length histogram = A-MPDU size)
#   - tsfl: RX timestamps of subframes within one PPDU cluster within ~its
#     airtime; singles are spaced by preamble+contention
# (Airtime/throughput claims come later from the SDR duty bench — this spike
# only establishes aggregation STRUCTURE.)
#
#   sudo bash tests/ampdu_spike.sh                       # default matrix
#   TX_PID=0xB82C RX_PID=0x8812 sudo bash tests/ampdu_spike.sh
#   FULL=1 sudo bash tests/ampdu_spike.sh                # + unicast-ack cell
#
# Results: per-cell verdict lines + raw logs under $OUT (/tmp/ampdu_spike).
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/build"

TX_PID=${TX_PID:-0x8812}; TX_VID=${TX_VID:-0x0bda}
RX_PID=${RX_PID:-0x0120}; RX_VID=${RX_VID:-0x2357}
CH=${CH:-6}; SECS=${SECS:-8}; RATE=${RATE:-MCS7}
PAYLOAD=${PAYLOAD:-1000}
AGG=${AGG:-16}           # frames per batch/URB + MAX_AGG_NUM (desc units of 2)
DENSITY=${DENSITY:-7}
FULL=${FULL:-}
OUT=${OUT:-/tmp/ampdu_spike}

KILL(){ pkill -9 -x rxdemo 2>/dev/null; pkill -9 -x txdemo 2>/dev/null; return 0; }
trap KILL EXIT
mkdir -p "$OUT"

run_cell() { # $1 = cell name, rest = extra TX-side K=V env
  local name="$1"; shift
  echo "=== cell $name ($*)"
  KILL; sleep 1
  sudo env DEVOURER_VID=$RX_VID DEVOURER_PID=$RX_PID DEVOURER_CHANNEL=$CH \
       DEVOURER_STREAM_OUT=1 DEVOURER_EVENT_FLUSH=0 DEVOURER_LOG_LEVEL=warn \
       "$BUILD/rxdemo" >"$OUT/rx_$name.jsonl" 2>"$OUT/rx_$name.err" &
  sleep 6 # rxdemo bring-up
  sudo env DEVOURER_VID=$TX_VID DEVOURER_PID=$TX_PID DEVOURER_CHANNEL=$CH \
       DEVOURER_TX_RATE=$RATE DEVOURER_TX_QOS_DATA=1 \
       DEVOURER_TX_PAYLOAD_BYTES=$PAYLOAD DEVOURER_TX_GAP_US=0 \
       DEVOURER_LOG_LEVEL=warn "$@" \
       timeout -s INT $SECS "$BUILD/txdemo" \
       >"$OUT/tx_$name.jsonl" 2>"$OUT/tx_$name.err" || true
  sleep 1
  KILL; sleep 1
}

# --- the matrix -------------------------------------------------------------
# control: no agg anywhere — the singles baseline every cell compares against.
run_cell control      DEVOURER_TX_QOS_NOACK=1
# data queue alone (no AGG_EN): does QSEL routing itself change anything?
run_cell qsel0        DEVOURER_TX_QOS_NOACK=1 DEVOURER_TX_QSEL=0
# AGG_EN on the MGMT queue, one-URB delivery: does 0x12 aggregate at all?
run_cell ampdu_mgmt   DEVOURER_TX_QOS_NOACK=1 DEVOURER_TX_AMPDU=$AGG/$DENSITY \
                      DEVOURER_TX_BATCH=$AGG DEVOURER_TX_USB_AGG=$AGG
# AGG_EN + data queue, gapless SINGLE URBs: is USB-agg co-queueing required?
run_cell ampdu_q0     DEVOURER_TX_QOS_NOACK=1 DEVOURER_TX_QSEL=0 \
                      DEVOURER_TX_AMPDU=$AGG/$DENSITY
# AGG_EN + data queue + one-URB delivery: the full recipe.
run_cell ampdu_q0_urb DEVOURER_TX_QOS_NOACK=1 DEVOURER_TX_QSEL=0 \
                      DEVOURER_TX_AMPDU=$AGG/$DENSITY \
                      DEVOURER_TX_BATCH=$AGG DEVOURER_TX_USB_AGG=$AGG
# optional: unicast RA, normal ack policy (nobody ACKs -> hw retry behaviour).
if [ -n "$FULL" ]; then
  run_cell ampdu_ucast DEVOURER_TX_QSEL=0 DEVOURER_TX_AMPDU=$AGG/$DENSITY \
                       DEVOURER_TX_BATCH=$AGG DEVOURER_TX_USB_AGG=$AGG \
                       DEVOURER_TX_RA=02:11:22:33:44:55
fi

echo
echo "=== RESULTS (payload=$PAYLOAD rate=$RATE agg=$AGG) ==="
python3 - "$OUT" "$PAYLOAD" <<'PYEOF'
import glob, json, os, sys

out, payload = sys.argv[1], int(sys.argv[2])
# RX pkt_len may include the 4-byte FCS depending on APPFCS handling.
want_len = {payload, payload + 4}

for path in sorted(glob.glob(os.path.join(out, "rx_*.jsonl"))):
    cell = os.path.basename(path)[3:-6]
    frames = []
    for line in open(path, errors="replace"):
        if not line.startswith('{"ev":"rx.frame"'):
            continue
        try:
            ev = json.loads(line)
        except json.JSONDecodeError:
            continue
        if ev.get("len") in want_len and not ev.get("crc"):
            frames.append(ev)
    n = len(frames)
    if n == 0:
        print(f"{cell:>14}: NO FRAMES (delivery broken in this cell?)")
        continue
    paggr = sum(1 for f in frames if f.get("paggr"))
    # Burst structure from the 2-bit PPDU counter: consecutive frames sharing
    # a value shared one PPDU. Also cross-check with tsfl adjacency (<200 us).
    bursts, cur = [], 1
    for a, b in zip(frames, frames[1:]):
        same_ppdu = a.get("ppdu") == b.get("ppdu")
        dt = (b.get("tsfl", 0) - a.get("tsfl", 0)) & 0xFFFFFFFF
        if same_ppdu and dt < 200:
            cur += 1
        else:
            bursts.append(cur); cur = 1
    bursts.append(cur)
    mean_burst = sum(bursts) / len(bursts)
    max_burst = max(bursts)
    multi = sum(b for b in bursts if b > 1)
    verdict = "AGGREGATED" if (max_burst >= 4 and multi > n * 0.5) else \
              "partial" if max_burst >= 2 and paggr else "singles"
    print(f"{cell:>14}: {verdict:>10}  frames={n} paggr={paggr} "
          f"mean_burst={mean_burst:.1f} max_burst={max_burst}")
PYEOF
echo "raw logs: $OUT"
