#!/usr/bin/env bash
# USB TX aggregation A/B bench: the same frame flood sent per-frame (classic
# send_packet loop) vs batched through send_packets with USB TX aggregation
# (DEVOURER_TX_BATCH + DEVOURER_TX_USB_AGG) — one bulk-OUT URB per batch.
# Measures the host/USB-side frame-rate ceiling and cross-checks delivery on
# a second devourer adapter in monitor mode (rx.txhit counts — a delivery
# sanity check, NOT an airtime/throughput claim; that is the SDR bench's job).
#
#   sudo bash tests/txagg_bench.sh
#   TX_PID=0x8812 TX_VID=0x0bda BATCH=32 PAYLOAD=200 sudo bash tests/txagg_bench.sh
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/build"

TX_PID=${TX_PID:-0x012d}; TX_VID=${TX_VID:-0x2357}
RX_PID=${RX_PID:-0xc812}; RX_VID=${RX_VID:-0x0bda}
CH=${CH:-6}; SECS=${SECS:-10}; RATE=${RATE:-MCS7}
PAYLOAD=${PAYLOAD:-1000}
BATCH=${BATCH:-16}
OUT=${OUT:-/tmp/txagg_bench}

KILL(){ pkill -9 -x rxdemo 2>/dev/null; pkill -9 -x txdemo 2>/dev/null; return 0; }
trap KILL EXIT
mkdir -p "$OUT"

run_mode() { # $1 = mode name, rest = extra TX env
  local name="$1"; shift
  echo "=== mode $name ($*)"
  KILL; sleep 1
  sudo env DEVOURER_VID=$RX_VID DEVOURER_PID=$RX_PID DEVOURER_CHANNEL=$CH \
       DEVOURER_EVENT_FLUSH=0 DEVOURER_LOG_LEVEL=warn \
       "$BUILD/rxdemo" >"$OUT/rx_$name.jsonl" 2>"$OUT/rx_$name.err" &
  sleep 6
  sudo env DEVOURER_VID=$TX_VID DEVOURER_PID=$TX_PID DEVOURER_CHANNEL=$CH \
       DEVOURER_TX_RATE=$RATE DEVOURER_TX_PAYLOAD_BYTES=$PAYLOAD \
       DEVOURER_TX_GAP_US=0 DEVOURER_LOG_LEVEL=warn "$@" \
       timeout -s INT $SECS "$BUILD/txdemo" \
       >"$OUT/tx_$name.jsonl" 2>"$OUT/tx_$name.err" || true
  sleep 1
  KILL; sleep 1
}

run_mode single
run_mode batch DEVOURER_TX_BATCH=$BATCH DEVOURER_TX_USB_AGG=$BATCH

echo
echo "=== RESULTS (payload=$PAYLOAD rate=$RATE batch=$BATCH secs=$SECS) ==="
python3 - "$OUT" "$SECS" <<'PYEOF'
import glob, json, os, sys

out, secs = sys.argv[1], float(sys.argv[2])
def jload(line):
    try:
        return json.loads(line)
    except json.JSONDecodeError:  # unflushed tail line at kill time
        return {}

for mode in ("single", "batch"):
    tx_n = 0
    agg_urbs = agg_frames = 0
    for line in open(os.path.join(out, f"tx_{mode}.jsonl"), errors="replace"):
        if line.startswith('{"ev":"tx.frame"'):
            tx_n = max(tx_n, jload(line).get("n", 0))
        elif line.startswith('{"ev":"tx.agg"'):
            ev = jload(line)
            agg_urbs += 1
            agg_frames += ev.get("frames", 0)
    hits = 0
    for line in open(os.path.join(out, f"rx_{mode}.jsonl"), errors="replace"):
        if line.startswith('{"ev":"rx.txhit"'):
            hits = max(hits, jload(line).get("hits", 0))
    per_urb = (agg_frames / agg_urbs) if agg_urbs else 1.0
    print(f"{mode:>7}: tx={tx_n} ({tx_n/secs:.0f} fps)  rx_hits={hits}  "
          f"frames/URB={per_urb:.1f} ({agg_urbs} agg URBs)")
PYEOF
echo "raw logs: $OUT"
