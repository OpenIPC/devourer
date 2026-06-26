#!/usr/bin/env bash
# Thermal-vs-TX-gain experiment runner.
#
# Builds devourer, sets up a uv venv (with system site-packages so UHD's
# `uhd` module is importable), then runs the orchestrator under sudo (USB claim
# needs root). All extra args are forwarded to thermal_gain_sweep.py, e.g.:
#
#     ./tests/run_thermal_gain_sweep.sh --channel 6 --start 8 --stop 40 \
#         --step 4 --step-ms 30000
#
# Cleanup: on any exit (including Ctrl-C) leftover demo / SDR-probe processes
# are killed by exact comm. The orchestrator already installs PDEATHSIG +
# killpg handlers; this is belt-and-braces.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

cleanup() {
    # Exact-comm kills only — never a broad pkill pattern.
    for comm in WiFiDriverTxDemo sdr_power_probe.py; do
        pkill -x "$comm" 2>/dev/null || true
    done
    # sdr_power_probe runs as `python3 .../sdr_power_probe.py`; comm is python3,
    # so match on the full arg instead.
    pkill -f "tests/sdr_power_probe.py" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "== building devourer =="
cmake --build "$ROOT/build" -j >/dev/null

echo "== preparing uv venv (system site-packages for uhd) =="
cd "$HERE"
if ! command -v uv >/dev/null 2>&1; then
    echo "uv not found — install it (https://docs.astral.sh/uv/) or run the" \
         "orchestrator with system python3 directly." >&2
    exit 1
fi
if [ ! -d "$HERE/.venv" ]; then
    uv venv --system-site-packages "$HERE/.venv"
fi
# shellcheck disable=SC1091
uv pip install --python "$HERE/.venv/bin/python" -q -e "$HERE" >/dev/null

PY="$HERE/.venv/bin/python"
# Verify uhd is reachable through the venv before we bother claiming USB.
if ! "$PY" -c "import uhd, numpy" 2>/dev/null; then
    echo "WARNING: 'import uhd' failed in the venv; falling back to system python3." >&2
    PY="$(command -v python3)"
fi

echo "== running experiment (sudo) =="
exec sudo --preserve-env=UHD_IMAGES_DIR "$PY" "$HERE/thermal_gain_sweep.py" "$@"
