#!/usr/bin/env bash
# M5 parked-jammer resilience: build the demos, then drive the A/B/C/D delivery
# matrix under a B210 narrowband jammer (tests/jammer_resilience.py).
#
# Three radios: TX=RTL8812AU, RX=RTL8812CU, jammer=USRP B210. All three need
# root (USB claim + UHD), so the orchestrator runs under sudo. It manages its
# own child pipes; this wrapper only builds and adds a belt-and-braces cleanup.
#
#   sudo is used non-interactively; extra args pass through to the python driver:
#   ./tests/run_jammer_resilience.sh --jammer-gain 65 --secs 25
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
OUT="${OUT:-/tmp/devourer-jammer-resilience}"
mkdir -p "$OUT"

cleanup() {
    # Exact-name backstop in case the orchestrator died mid-cell.
    sudo -n pkill -x rxdemo 2>/dev/null || true
    sudo -n pkill -x streamtx 2>/dev/null || true
    sudo -n pkill -f "tests/sdr_interferer.py" 2>/dev/null || true
    sudo -n pkill -f "tests/sdr_follower_jammer.py" 2>/dev/null || true
    sudo -n pkill -f "fused_fec_rx.py" 2>/dev/null || true
    sudo -n pkill -f "fused_fec_tx.py" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "== building rxdemo + streamtx =="
cmake --build "$ROOT/build" -j --target rxdemo streamtx >/dev/null

# System python3: UHD (sdr_interferer) is a system package; numpy + the RS FEC
# backend import there too (verified in tests/pyproject.toml notes).
echo "== running jammer-resilience matrix (sudo) =="
sudo -n --preserve-env=UHD_IMAGES_DIR python3 "$HERE/jammer_resilience.py" \
    --build "$ROOT/build" "$@" | tee "$OUT/result.log"
echo "logs: $OUT/result.log"
