#!/usr/bin/env bash
# Antenna-decorrelation measurement runner (spatial-diversity axis).
#
# Measures the inter-chain envelope correlation and realised diversity gain of a
# multi-chain Realtek adapter, against a *controlled* transmitter: devourer's RX
# demo only surfaces per-chain metrics for the canonical beacon SA
# (57:42:75:05:d6:00), so a second adapter must be injecting that beacon
# (WiFiDriverTxDemo) on the same channel — the standard two-adapter bench setup.
#
# It builds devourer, runs WiFiDriverDemo (RX) with DEVOURER_RX_ALLPATHS=1 for a
# fixed dwell capturing the <devourer-rxpath> lines, then feeds them to
# antenna_decorrelation.py. Optionally starts the TX beacon too (set TX_PID).
#
# Env knobs (all optional):
#   CHANNEL=6         monitor channel for both RX and (if started) TX
#   RX_PID=0xNNNN     restrict the RX demo to one adapter PID (e.g. 0x8813 8814AU)
#   TX_PID=0xNNNN     if set, also start WiFiDriverTxDemo on this PID as the source
#   DURATION=30       capture seconds
#   METRIC=rssi       rssi | snr | evm  (forwarded to the analyser)
#
# Usage:
#   sudo CHANNEL=6 RX_PID=0x8813 TX_PID=0x8812 DURATION=30 \
#        ./tests/run_antenna_decorrelation.sh
#   # or, with the beacon already flying from another host/adapter:
#   sudo RX_PID=0x8813 ./tests/run_antenna_decorrelation.sh
#
# Cleanup: on any exit (incl. Ctrl-C) the demo processes are killed by exact comm.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

CHANNEL="${CHANNEL:-6}"
DURATION="${DURATION:-30}"
METRIC="${METRIC:-rssi}"
CAP="$(mktemp -t devourer-decorr.XXXXXX.log)"

cleanup() {
    # Exact-comm kills only — never a broad pkill pattern.
    for comm in WiFiDriverDemo WiFiDriverTxDemo; do
        pkill -x "$comm" 2>/dev/null || true
    done
    rm -f "$CAP" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "== building devourer =="
if [ ! -d "$ROOT/build" ]; then
    cmake -S "$ROOT" -B "$ROOT/build" >/dev/null
fi
cmake --build "$ROOT/build" -j >/dev/null

DEMO="$ROOT/build/WiFiDriverDemo"
TXDEMO="$ROOT/build/WiFiDriverTxDemo"
[ -x "$DEMO" ] || { echo "WiFiDriverDemo not built at $DEMO" >&2; exit 1; }

# uv venv for the analyser (numpy only; no system packages needed).
if ! command -v uv >/dev/null 2>&1; then
    echo "uv not found — install it (https://docs.astral.sh/uv/)." >&2
    exit 1
fi
[ -d "$HERE/.venv" ] || uv venv "$HERE/.venv" >/dev/null
uv pip install --python "$HERE/.venv/bin/python" -q -e "$HERE" >/dev/null
PY="$HERE/.venv/bin/python"

if [ -n "${TX_PID:-}" ]; then
    [ -x "$TXDEMO" ] || { echo "WiFiDriverTxDemo not built" >&2; exit 1; }
    echo "== starting TX beacon on PID $TX_PID ch$CHANNEL =="
    DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="$CHANNEL" "$TXDEMO" >/dev/null 2>&1 &
    sleep 3   # let it claim + start injecting before RX starts counting
fi

echo "== capturing RX per-chain metrics for ${DURATION}s (ch$CHANNEL) =="
RX_ENV=(DEVOURER_RX_ALLPATHS=1 DEVOURER_CHANNEL="$CHANNEL")
[ -n "${RX_PID:-}" ] && RX_ENV+=(DEVOURER_PID="$RX_PID")
env "${RX_ENV[@]}" "$DEMO" >"$CAP" 2>/dev/null &
RX_PID_RUNNING=$!
sleep "$DURATION"
kill "$RX_PID_RUNNING" 2>/dev/null || true
wait "$RX_PID_RUNNING" 2>/dev/null || true

RXLINES="$(grep -c '<devourer-rxpath>' "$CAP" || true)"
echo "== captured $RXLINES <devourer-rxpath> frames =="
if [ "${RXLINES:-0}" -eq 0 ]; then
    echo "No canonical-SA frames captured. Is the beacon TX flying on ch$CHANNEL," \
         "and is RX_PID the right adapter?" >&2
    exit 2
fi

echo "== analysis =="
"$PY" "$HERE/antenna_decorrelation.py" --metric "$METRIC" "$CAP"
