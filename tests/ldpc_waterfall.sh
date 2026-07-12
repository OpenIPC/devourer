#!/usr/bin/env bash
# LDPC coding-gain waterfall — measure the dB gap between BCC and LDPC at the
# same MCS by tracing delivery-vs-TX-power curves and reading the horizontal
# shift. The emitter steps a flat TXAGC index (DEVOURER_TX_PWR →
# SetTxPowerIndexOverride, 0.5 dB/step on Jaguar1/2, 0.25 dB on Jaguar3); the
# ground adapter counts canonical-SA frames per rx.energy window
# (DEVOURER_RX_AGG_SA=canon, FCS-clean only — corrupt frames never reach the
# aggregate). Per point: delivered = Σ rx.energy frames across the txdemo
# lifetime, sent = the final tx.stats submitted (exact, final:1).
#
# Bench notes (docs/bench-testing-near-field.md): sweep the noise-limited
# low-index regime — near-field at high index both curves sit at PER≈0 and
# the waterfall is invisible. If even index 0 delivers ~100%, add physical
# attenuation/distance or raise the MCS.
#
#   sudo tests/ldpc_waterfall.sh --emit-pid 0x8812 --ground-pid 0x012d \
#        --channel 6 --rate MCS7 --pwr-start 0 --pwr-stop 24 --pwr-step 2
#
# Output: $OUT/points.jsonl (one JSON per point) + per-point raw logs.
# Analysis: python3 tests/ldpc_waterfall.py $OUT/points.jsonl --step-qdb 2
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
TXDEMO="$ROOT/build/txdemo"
RXDEMO="$ROOT/build/rxdemo"

EMIT_VID=0x0bda EMIT_PID="" GROUND_VID=0x0bda GROUND_PID=""
CHANNEL=6 RATE=MCS7 EXTRA_ENCS=""
PWR_START=0 PWR_STOP=24 PWR_STEP=2 SECS=6
OUT=/tmp/devourer-ldpc-waterfall

while [ $# -gt 0 ]; do
  case "$1" in
    --emit-vid) EMIT_VID="$2"; shift 2 ;;
    --emit-pid) EMIT_PID="$2"; shift 2 ;;
    --ground-vid) GROUND_VID="$2"; shift 2 ;;
    --ground-pid) GROUND_PID="$2"; shift 2 ;;
    --channel) CHANNEL="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;          # base spec; run pairs it with /LDPC
    --encs) EXTRA_ENCS="$2"; shift 2 ;;    # comma list overriding the BCC/LDPC pair
    --pwr-start) PWR_START="$2"; shift 2 ;;
    --pwr-stop) PWR_STOP="$2"; shift 2 ;;
    --pwr-step) PWR_STEP="$2"; shift 2 ;;
    --secs) SECS="$2"; shift 2 ;;
    --outdir) OUT="$2"; shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done
[ -n "$EMIT_PID" ] && [ -n "$GROUND_PID" ] || {
  echo "need --emit-pid and --ground-pid" >&2; exit 2; }
[ -x "$TXDEMO" ] && [ -x "$RXDEMO" ] || { echo "build first" >&2; exit 2; }

mkdir -p "$OUT"
RXLOG="$OUT/ground.rx.jsonl"
POINTS="$OUT/points.jsonl"
: > "$POINTS"

RXPID=""
cleanup() {
  [ -n "$RXPID" ] && kill "$RXPID" 2>/dev/null
  pkill -x txdemo 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

# Ground RX up once for the whole sweep (bring-up is ~6 s; per-point windows
# are carved out of its log by byte offset).
DEVOURER_VID="$GROUND_VID" DEVOURER_PID="$GROUND_PID" \
DEVOURER_CHANNEL="$CHANNEL" \
DEVOURER_RX_AGG_SA=canon DEVOURER_RX_ENERGY_MS=500 \
DEVOURER_LOG_LEVEL=warn \
"$RXDEMO" > "$RXLOG" 2>"$OUT/ground.rx.stderr" &
RXPID=$!
sleep 8
kill -0 "$RXPID" 2>/dev/null || { echo "ground RX died — see $OUT/ground.rx.stderr" >&2; exit 1; }

ENCS="$RATE,$RATE/LDPC"
[ -n "$EXTRA_ENCS" ] && ENCS="$EXTRA_ENCS"

IFS=',' read -ra ENC_ARR <<<"$ENCS"
for ENC in "${ENC_ARR[@]}"; do
  IDX="$PWR_START"
  while [ "$IDX" -le "$PWR_STOP" ]; do
    TAG="$(echo "$ENC" | tr '/' '-')_idx$IDX"
    TXLOG="$OUT/tx_$TAG.jsonl"
    OFF0=$(stat -c%s "$RXLOG")
    echo "[$(date +%H:%M:%S)] point enc=$ENC idx=$IDX ..."
    DEVOURER_VID="$EMIT_VID" DEVOURER_PID="$EMIT_PID" \
    DEVOURER_CHANNEL="$CHANNEL" \
    DEVOURER_TX_RATE="$ENC" DEVOURER_TX_PWR="$IDX" \
    DEVOURER_TX_GAP_US=2000 DEVOURER_LOG_LEVEL=warn \
    timeout --signal=TERM $((SECS + 12)) "$TXDEMO" \
        > "$TXLOG" 2>"$OUT/tx_$TAG.stderr" &
    TXPID=$!
    # Let bring-up finish, then measure for SECS, then graceful stop (the
    # final tx.stats needs a clean SIGTERM exit, not SIGKILL).
    sleep $((SECS + 6))
    kill -TERM "$TXPID" 2>/dev/null
    wait "$TXPID" 2>/dev/null
    sleep 1
    OFF1=$(stat -c%s "$RXLOG")
    python3 - "$TXLOG" "$RXLOG" "$OFF0" "$OFF1" "$ENC" "$IDX" >> "$POINTS" <<'EOF'
import json, sys
txlog, rxlog, off0, off1, enc, idx = sys.argv[1:7]
sent = failed = -1
for line in open(txlog, errors="replace"):
    if not line.startswith('{"ev":"tx.stats"'):
        continue
    try:
        ev = json.loads(line)
    except ValueError:
        continue
    sent, failed = int(ev.get("submitted", -1)), int(ev.get("failed", 0))
delivered = 0
with open(rxlog, "rb") as f:
    f.seek(int(off0))
    blob = f.read(int(off1) - int(off0)).decode(errors="replace")
for line in blob.splitlines():
    if not line.startswith('{"ev":"rx.energy"'):
        continue
    try:
        ev = json.loads(line)
    except ValueError:
        continue
    delivered += int(ev.get("frames") or 0)
print(json.dumps({"enc": enc, "idx": int(idx), "sent": sent,
                  "failed": failed, "delivered": delivered}))
EOF
    tail -1 "$POINTS"
    IDX=$((IDX + PWR_STEP))
  done
done

echo "done — analyze: python3 tests/ldpc_waterfall.py $POINTS"
