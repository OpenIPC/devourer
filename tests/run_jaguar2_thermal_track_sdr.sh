#!/usr/bin/env bash
# SDR on-air validation runner for Jaguar2 thermal tracking.
# Builds devourer, sets up a uv venv (system site-packages for UHD's `uhd`),
# and runs the soak orchestrator under sudo (USB claim needs root). Extra args
# forward to jaguar2_thermal_track_sdr.py, e.g.:
#     ./tests/run_jaguar2_thermal_track_sdr.sh --channel 6 --secs 180
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

cleanup() {
    for comm in txdemo sdr_power_probe.py; do pkill -x "$comm" 2>/dev/null || true; done
    pkill -f "tests/sdr_power_probe.py" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "== building devourer =="
cmake --build "$ROOT/build" -j >/dev/null

echo "== preparing uv venv (system site-packages for uhd) =="
cd "$HERE"
if ! command -v uv >/dev/null 2>&1; then
    echo "uv not found — install it or run the orchestrator with system python3." >&2
    exit 1
fi
[ -d "$HERE/.venv" ] || uv venv --system-site-packages "$HERE/.venv"
uv pip install --python "$HERE/.venv/bin/python" -q -e "$HERE" >/dev/null 2>&1 || true

PY="$HERE/.venv/bin/python"
if ! "$PY" -c "import uhd, numpy" 2>/dev/null; then
    echo "WARNING: 'import uhd' failed in venv; falling back to system python3." >&2
    PY="$(command -v python3)"
fi

echo "== running SDR soak (sudo) =="
exec sudo --preserve-env=UHD_IMAGES_DIR "$PY" "$HERE/jaguar2_thermal_track_sdr.py" "$@"
