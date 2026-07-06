#!/usr/bin/env bash
# Compare inter-chain antenna decorrelation across every plugged RTL8814AU.
#
# Two RTL8814AU dongles that enumerate identically (same VID:PID:serial) can only
# be told apart by USB topology, so this drives rxdemo's
# DEVOURER_USB_BUS/PORT selector once per device. Each is measured as RX against
# one controlled beacon TX (the canonical SA), so the only variable between runs
# is the adapter's antenna front-end — e.g. 2 external dipoles (CF-938AC) vs 4
# on-PCB internal antennas (CF-960AC).
#
# Env knobs:
#   TX_PID=0x8812     beacon TX source PID (a *different* adapter; default 8812AU)
#   CHANNEL=6         channel for TX and both RX runs
#   DURATION=30       capture seconds per device
#   METRIC=rssi       rssi | snr | evm  (forwarded to the analyser)
#
# Usage:  sudo ./tests/compare_8814_decorrelation.sh
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

TX_PID="${TX_PID:-0x8812}"
CHANNEL="${CHANNEL:-6}"
DURATION="${DURATION:-30}"
METRIC="${METRIC:-rssi}"
OUTDIR="$(mktemp -d -t devourer-8814cmp.XXXXXX)"

cleanup() {
    for comm in rxdemo txdemo; do
        pkill -x "$comm" 2>/dev/null || true
    done
    rm -f "${TXLOG:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "== building devourer =="
[ -d "$ROOT/build" ] || cmake -S "$ROOT" -B "$ROOT/build" >/dev/null
cmake --build "$ROOT/build" -j >/dev/null
DEMO="$ROOT/build/rxdemo"
TXDEMO="$ROOT/build/txdemo"

# uv venv for the analyser (numpy only).
command -v uv >/dev/null 2>&1 || { echo "uv not found" >&2; exit 1; }
[ -d "$HERE/.venv" ] || uv venv "$HERE/.venv" >/dev/null
uv pip install --python "$HERE/.venv/bin/python" -q numpy >/dev/null
PY="$HERE/.venv/bin/python"

# Discover every 0bda:8813 device as "bus port" pairs from sysfs.
mapfile -t DEVS < <(
    for d in /sys/bus/usb/devices/*/; do
        [ -f "$d/idProduct" ] || continue
        [ "$(cat "$d/idProduct")" = "8813" ] || continue
        echo "$(cat "$d/busnum") $(cat "$d/devpath")"
    done
)
[ "${#DEVS[@]}" -gt 0 ] || { echo "no RTL8814AU (0bda:8813) found" >&2; exit 1; }
echo "== found ${#DEVS[@]} RTL8814AU device(s): ${DEVS[*]/#/[}"

echo "== starting beacon TX on PID $TX_PID ch$CHANNEL =="
TXLOG="$(mktemp -t devourer-txbeacon.XXXXXX.log)"
stdbuf -oL -eL env DEVOURER_PID="$TX_PID" DEVOURER_CHANNEL="$CHANNEL" "$TXDEMO" >"$TXLOG" 2>&1 &
# Wait for a confirmed inject rather than a fixed sleep — the 8812 TX needs
# ~10s to init and occasionally fails to claim on the first try, which would
# otherwise leave every device's capture empty.
for _ in $(seq 1 30); do
    grep -q 'TX #.* rc=1' "$TXLOG" 2>/dev/null && break
    sleep 1
done
grep -q 'TX #.* rc=1' "$TXLOG" 2>/dev/null || {
    echo "TX beacon did not confirm an inject within 30s — check $TXLOG" >&2; exit 3; }
echo "== TX beacon confirmed injecting =="

for dev in "${DEVS[@]}"; do
    read -r bus port <<<"$dev"
    tag="bus${bus}_port${port}"
    cap="$OUTDIR/$tag.log"
    echo
    echo "==================================================================="
    echo "== RX device bus=$bus port=$port  (${DURATION}s, ch$CHANNEL) =="
    echo "==================================================================="
    env DEVOURER_USB_BUS="$bus" DEVOURER_USB_PORT="$port" DEVOURER_PID=0x8813 \
        DEVOURER_RX_ALLPATHS=1 DEVOURER_CHANNEL="$CHANNEL" \
        "$DEMO" >"$cap" 2>/dev/null &
    rxpid=$!
    sleep "$DURATION"
    kill "$rxpid" 2>/dev/null || true
    wait "$rxpid" 2>/dev/null || true

    frames="$(grep -c '<devourer-rxpath>' "$cap" || true)"
    echo "captured ${frames:-0} canonical-SA frames"
    if [ "${frames:-0}" -gt 0 ]; then
        "$PY" "$HERE/antenna_decorrelation.py" --metric "$METRIC" "$cap"
    else
        echo "  (no beacon frames — check TX is flying and this adapter hears ch$CHANNEL)"
    fi
done

echo
echo "== per-device capture logs kept in $OUTDIR =="
trap - EXIT
cleanup