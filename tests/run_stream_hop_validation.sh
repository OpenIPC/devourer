#!/usr/bin/env bash
# Integrated on-air check: StreamTxDemo (the precoder stream vehicle) hopping
# per-packet across a channel set via FastRetune, confirmed by the B210 seeing
# frames on every channel. Proves the production TX path — not just the
# WiFiDriverTxDemo beacon loop — hops correctly.
#
#   ./tests/run_stream_hop_validation.sh
#   HOP_CHANNELS=1,6,11 RECORDS=4000 ./tests/run_stream_hop_validation.sh
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

HOP_CHANNELS="${HOP_CHANNELS:-1,6,11}"
HOP_FAST="${HOP_FAST:-1}"
TX_PID="${TX_PID:-0x8812}"
INIT_CHANNEL="${INIT_CHANNEL:-1}"
INTERVAL_MS="${INTERVAL_MS:-2}"
RECORDS="${RECORDS:-4000}"        # ~RECORDS*INTERVAL_MS ms of TX
PSDU_BYTES="${PSDU_BYTES:-200}"
SDR_CENTER="${SDR_CENTER:-2437e6}"
SDR_RATE="${SDR_RATE:-61.44e6}"
SDR_GAIN="${SDR_GAIN:-45}"
SDR_DURATION="${SDR_DURATION:-7}"

OUT="${OUT:-/tmp/devourer-stream-hop}"
mkdir -p "$OUT"
STREAM="$OUT/stream.bin"; TX_LOG="$OUT/tx.log"; RX_LOG="$OUT/rx.log"

cleanup() {
    pkill -x StreamTxDemo 2>/dev/null || true
    pkill -f "tests/hop_rx_probe.py" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "== building StreamTxDemo =="
cmake --build "$ROOT/build" -j --target StreamTxDemo >/dev/null

echo "== preparing uv venv =="
cd "$HERE"
[ -d .venv ] || uv venv --system-site-packages .venv >/dev/null
uv pip install --python .venv/bin/python -q -e . >/dev/null 2>&1 || true
PY=.venv/bin/python
"$PY" -c "import uhd,numpy" 2>/dev/null || PY="$(command -v python3)"

echo "== generating $RECORDS length-prefixed PSDUs ($PSDU_BYTES B each) =="
python3 - "$STREAM" "$RECORDS" "$PSDU_BYTES" <<'PY'
import os, struct, sys
path, n, sz = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
with open(path, "wb") as f:
    for i in range(n):
        body = bytes([i & 0xFF]) + os.urandom(sz - 1)
        f.write(struct.pack("<I", len(body)) + body)
PY

echo "== starting hopping StreamTxDemo (RTL8812AU, init ch$INIT_CHANNEL) =="
: >"$TX_LOG"
sudo --preserve-env \
    env DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="$INIT_CHANNEL" \
        DEVOURER_HOP_CHANNELS="$HOP_CHANNELS" DEVOURER_HOP_FAST="$HOP_FAST" \
    "$ROOT/build/StreamTxDemo" --interval-ms "$INTERVAL_MS" <"$STREAM" \
    >"$TX_LOG" 2>&1 &
TX_PID_PROC=$!

echo "== waiting for first TX =="
for _ in $(seq 1 300); do
    grep -q "<stream-tx>" "$TX_LOG" 2>/dev/null && break
    kill -0 "$TX_PID_PROC" 2>/dev/null || { echo "TX exited early"; break; }
    sleep 0.1
done

echo "== starting SDR probe (RX) =="
sudo --preserve-env=UHD_IMAGES_DIR "$PY" "$HERE/hop_rx_probe.py" \
    --channels "$HOP_CHANNELS" --center "$SDR_CENTER" --rate "$SDR_RATE" \
    --gain "$SDR_GAIN" --duration "$SDR_DURATION" >"$RX_LOG" 2>&1 &
RX_PID=$!

wait "$RX_PID" 2>/dev/null || true
cleanup

echo; echo "== TX summary =="; grep -E "stream-tx|HOP_CHANNELS" "$TX_LOG" | tail -5 || true
echo; echo "== SDR verdict =="; sed -n '/=== hop_rx_probe verdict ===/,$p' "$RX_LOG" || true
echo; echo "Logs: $TX_LOG  $RX_LOG"
