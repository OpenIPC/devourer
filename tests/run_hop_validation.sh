#!/usr/bin/env bash
# Single-adapter channel-hopping validation.
#
# Drives an RTL8812AU (TX) through DEVOURER_HOP_CHANNELS while a B210/LibreSDR
# (RX) watches all hop channels at once (tests/hop_rx_probe.py). A single
# wideband receiver can tell "the radio retuned" from "the radio dropped
# frames", which is the whole point: it proves one adapter actually transmits
# on several channels in turn without losing frames across the retune.
#
# Both the TX demo (USB claim) and the SDR probe (USB) need root, so each is
# launched under sudo. All extra args after `--` are forwarded to
# hop_rx_probe.py.
#
#   ./tests/run_hop_validation.sh
#   ./tests/run_hop_validation.sh --channels 1,6,11 --dwell 50 --rounds 8
#
# Cleanup: on any exit (incl. Ctrl-C) the TX demo and SDR probe are killed by
# exact comm / exact arg — never a broad pattern.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

# --- knobs (env-overridable) ---
HOP_CHANNELS="${HOP_CHANNELS:-1,6,11}"
DWELL="${DWELL:-50}"
ROUNDS="${ROUNDS:-8}"
GAP_US="${GAP_US:-2000}"
TX_RATE="${TX_RATE:-6M}"          # 6M OFDM: wide, easy for the SDR to see
TX_PID="${TX_PID:-0x8812}"        # RTL8812AU
HOP_FAST="${HOP_FAST:-0}"         # 0=full SetMonitorChannel, 1=FastRetune(cached), 2=FastRetune(no cache)
SDR_CENTER="${SDR_CENTER:-2437e6}"
SDR_RATE="${SDR_RATE:-61.44e6}"
SDR_GAIN="${SDR_GAIN:-45}"
# Capture window: the hop run is ~rounds*nch*(dwell*gap + ~0.24s retune). The
# measured intra-band retune dominates, so size generously off ROUNDS.
SDR_DURATION="${SDR_DURATION:-0}"   # 0 = auto from ROUNDS below
# Start TX already on a hop channel (2.4GHz) so InitWrite doesn't pay a 5GHz
# band switch + IQK on the first hop.
TX_INIT_CHANNEL="${TX_INIT_CHANNEL:-1}"

OUT="${OUT:-/tmp/devourer-hop-validation}"
mkdir -p "$OUT"
TX_LOG="$OUT/tx.log"
RX_LOG="$OUT/rx.log"

cleanup() {
    pkill -x WiFiDriverTxDemo 2>/dev/null || true
    pkill -f "tests/hop_rx_probe.py" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "== building WiFiDriverTxDemo =="
cmake --build "$ROOT/build" -j --target WiFiDriverTxDemo >/dev/null

echo "== preparing uv venv (system site-packages for uhd) =="
cd "$HERE"
if [ ! -d "$HERE/.venv" ]; then
    uv venv --system-site-packages "$HERE/.venv"
fi
uv pip install --python "$HERE/.venv/bin/python" -q -e "$HERE" >/dev/null 2>&1 || true
PY="$HERE/.venv/bin/python"
if ! "$PY" -c "import uhd, numpy" 2>/dev/null; then
    echo "WARNING: 'import uhd' failed in venv; falling back to system python3." >&2
    PY="$(command -v python3)"
fi

# Auto-size the SDR capture from the run length if not overridden.
nch=$(awk -F, '{print NF}' <<<"$HOP_CHANNELS")
if [ "$SDR_DURATION" = "0" ]; then
    # per-dwell ≈ dwell*gap + ~0.30s retune; +3s margin.
    SDR_DURATION=$(awk -v r="$ROUNDS" -v n="$nch" -v d="$DWELL" -v g="$GAP_US" \
        'BEGIN{printf "%.0f", r*n*(d*g/1e6 + 0.30) + 3}')
fi
echo "== auto SDR_DURATION=${SDR_DURATION}s (rounds=$ROUNDS nch=$nch dwell=$DWELL) =="

echo "== starting hopping TX (RTL8812AU, init ch$TX_INIT_CHANNEL, hop $HOP_CHANNELS) =="
: >"$TX_LOG"
sudo --preserve-env \
    env DEVOURER_PID="$TX_PID" \
        DEVOURER_CHANNEL="$TX_INIT_CHANNEL" \
        DEVOURER_HOP_CHANNELS="$HOP_CHANNELS" \
        DEVOURER_HOP_DWELL_FRAMES="$DWELL" \
        DEVOURER_HOP_ROUNDS="$ROUNDS" \
        DEVOURER_HOP_FAST="$HOP_FAST" \
        DEVOURER_HOP_RADIOTAP="${HOP_RADIOTAP:-}" \
        DEVOURER_TX_GAP_US="$GAP_US" \
        DEVOURER_TX_RATE="$TX_RATE" \
    "$ROOT/build/WiFiDriverTxDemo" >"$TX_LOG" 2>&1 &
TX_PID_PROC=$!

# Wait until the TX is actually hopping (first <devourer-hop> marker) before we
# start the SDR — chip bring-up takes a few seconds and we don't want to burn
# the capture window on it.
echo "== waiting for TX to begin hopping =="
for _ in $(seq 1 300); do
    grep -q "<devourer-hop>" "$TX_LOG" 2>/dev/null && break
    kill -0 "$TX_PID_PROC" 2>/dev/null || { echo "TX exited early"; break; }
    sleep 0.1
done

echo "== starting SDR probe (RX), capturing the hop run =="
sudo --preserve-env=UHD_IMAGES_DIR "$PY" "$HERE/hop_rx_probe.py" \
    --channels "$HOP_CHANNELS" --center "$SDR_CENTER" --rate "$SDR_RATE" \
    --gain "$SDR_GAIN" --duration "$SDR_DURATION" --expect-rounds "$ROUNDS" \
    "$@" >"$RX_LOG" 2>&1 &
RX_PID=$!

# Wait for the bounded TX run to finish (it std::exit(0)s after ROUNDS).
wait "$TX_PID_PROC" 2>/dev/null || true
echo "== TX finished; waiting for SDR capture to end =="
wait "$RX_PID" 2>/dev/null || true

echo
echo "== TX hop markers (from $TX_LOG) =="
grep -E "devourer-hop|devourer-hop-done" "$TX_LOG" | head -40 || true
echo
echo "== SDR verdict (from $RX_LOG) =="
sed -n '/=== hop_rx_probe verdict ===/,$p' "$RX_LOG" || true
echo
echo "Full logs: $TX_LOG  $RX_LOG"
