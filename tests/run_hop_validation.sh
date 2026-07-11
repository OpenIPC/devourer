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
# Keyed FHSS (optional): set SEED to a hex key to drive the SipHash schedule,
# and SLOT_MS to hop on a wall-clock slot instead of frame dwell (required by
# keyed mode — DWELL and slot mode are mutually exclusive). When SEED is set the
# SDR probe is told the same key so it aligns against the keyed order.
SEED="${SEED:-}"
SLOT_MS="${SLOT_MS:-}"
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
    pkill -x txdemo 2>/dev/null || true
    pkill -f "tests/hop_rx_probe.py" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "== building txdemo =="
cmake --build "$ROOT/build" -j --target txdemo >/dev/null

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
    if [ -n "$SLOT_MS" ]; then
        # slot mode: each dwell is a fixed wall-clock slot; +3s margin.
        SDR_DURATION=$(awk -v r="$ROUNDS" -v n="$nch" -v s="$SLOT_MS" \
            'BEGIN{printf "%.0f", r*n*s/1e3 + 3}')
    else
        # per-dwell ≈ dwell*gap + ~0.30s retune; +3s margin.
        SDR_DURATION=$(awk -v r="$ROUNDS" -v n="$nch" -v d="$DWELL" -v g="$GAP_US" \
            'BEGIN{printf "%.0f", r*n*(d*g/1e6 + 0.30) + 3}')
    fi
fi
echo "== auto SDR_DURATION=${SDR_DURATION}s (rounds=$ROUNDS nch=$nch dwell=$DWELL) =="

echo "== starting hopping TX (RTL8812AU, init ch$TX_INIT_CHANNEL, hop $HOP_CHANNELS) =="
: >"$TX_LOG"
# Build the TX hop env: keyed slot mode (SEED+SLOT_MS) drops the frame-dwell var
# because the two are mutually exclusive; otherwise the historic frame dwell.
# TX hops CONTINUOUSLY (no DEVOURER_HOP_ROUNDS) so it spans the whole SDR
# capture. A bounded TX run under-runs the capture: the B210 needs a second or
# two to bring up before its IQ stream starts, and a short bounded TX can finish
# during that gap — the capture then sees only a sliver of the hopping. The
# cleanup trap stops the TX once the capture (RX_PID) closes. ROUNDS still sizes
# the capture window and the probe's expected-round threshold.
tx_env=(DEVOURER_PID="$TX_PID"
        DEVOURER_CHANNEL="$TX_INIT_CHANNEL"
        DEVOURER_HOP_CHANNELS="$HOP_CHANNELS"
        DEVOURER_HOP_FAST="$HOP_FAST"
        DEVOURER_HOP_RADIOTAP="${HOP_RADIOTAP:-}"
        DEVOURER_TX_GAP_US="$GAP_US"
        DEVOURER_TX_RATE="$TX_RATE")
if [ -n "$SLOT_MS" ]; then
    tx_env+=(DEVOURER_HOP_SLOT_MS="$SLOT_MS")
else
    tx_env+=(DEVOURER_HOP_DWELL_FRAMES="$DWELL")
fi
[ -n "$SEED" ] && tx_env+=(DEVOURER_HOP_SEED="$SEED")
sudo --preserve-env \
    env "${tx_env[@]}" \
    "$ROOT/build/txdemo" >"$TX_LOG" 2>&1 &
TX_PID_PROC=$!

# Wait until the TX is actually hopping (first hop.dwell event) before we
# start the SDR — chip bring-up takes a few seconds and we don't want to burn
# the capture window on it.
echo "== waiting for TX to begin hopping =="
for _ in $(seq 1 300); do
    grep -qF '"ev":"hop.dwell"' "$TX_LOG" 2>/dev/null && break
    kill -0 "$TX_PID_PROC" 2>/dev/null || { echo "TX exited early"; break; }
    sleep 0.1
done

echo "== starting SDR probe (RX), capturing the hop run =="
# In keyed mode the probe must know the same key to align against the SipHash
# order instead of a fixed round-robin.
probe_seed=(); [ -n "$SEED" ] && probe_seed=(--seed "$SEED")
sudo --preserve-env=UHD_IMAGES_DIR "$PY" "$HERE/hop_rx_probe.py" \
    --channels "$HOP_CHANNELS" --center "$SDR_CENTER" --rate "$SDR_RATE" \
    --gain "$SDR_GAIN" --duration "$SDR_DURATION" --expect-rounds "$ROUNDS" \
    "${probe_seed[@]}" "$@" >"$RX_LOG" 2>&1 &
RX_PID=$!

# The SDR capture is the bounded operation now; the TX hops until we stop it, so
# it fully covers the capture regardless of B210 bring-up latency. Wait for the
# probe (capture + analysis), then the cleanup trap stops the continuous TX.
echo "== capturing; TX hops until the SDR window closes =="
if ! kill -0 "$TX_PID_PROC" 2>/dev/null; then
    echo "WARNING: TX exited before capture — check $TX_LOG" >&2
fi
wait "$RX_PID" 2>/dev/null || true

echo
echo "== TX hop events (from $TX_LOG) =="
grep -F -e '"ev":"hop.dwell"' -e '"ev":"hop.done"' "$TX_LOG" | head -40 || true
echo
echo "== SDR verdict (from $RX_LOG) =="
sed -n '/=== hop_rx_probe verdict ===/,$p' "$RX_LOG" || true
echo
echo "Full logs: $TX_LOG  $RX_LOG"
