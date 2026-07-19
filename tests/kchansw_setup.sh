#!/usr/bin/env bash
# kchansw_setup.sh — preflight + venv prep + launch for the kernel
# channel-switch bench. Thin wrapper so a session is one command:
#
#   sudo tests/kchansw_setup.sh inventory
#   sudo tests/kchansw_setup.sh smoke
#   sudo tests/kchansw_setup.sh phase-a [bench args...]
#
# Uses the tests/ uv venv (system-site-packages, per tests/pyproject.toml);
# creates it if missing. The bench itself is tests/kchansw_bench.py.

set -euo pipefail
cd "$(dirname "$0")/.."

[ "$(id -u)" -eq 0 ] || { echo "FAIL: needs root (re-run with sudo)"; exit 2; }
[ $# -ge 1 ] || { echo "usage: $0 {inventory|smoke|phase-a|cold|phase-b|phase-c} [args]"; exit 2; }

VENV=tests/.venv
if [ ! -x "$VENV/bin/python" ]; then
  echo "== creating tests venv (uv, system-site-packages for uhd) =="
  SUDO_UV="$(command -v uv || true)"
  [ -n "$SUDO_UV" ] || { echo "FAIL: uv not on root PATH"; exit 2; }
  "$SUDO_UV" venv "$VENV" --system-site-packages
  "$SUDO_UV" pip install -p "$VENV/bin/python" -e tests >/dev/null
fi

for tool in iw ip modprobe journalctl wpa_supplicant wpa_cli; do
  command -v "$tool" >/dev/null || { echo "FAIL: missing tool: $tool"; exit 2; }
done
[ -x build/rxdemo ] || { echo "FAIL: build/rxdemo missing — cmake --build build -j"; exit 2; }
[ -d /sys/kernel/tracing ] || { echo "FAIL: tracefs not mounted"; exit 2; }

exec "$VENV/bin/python" tests/kchansw_bench.py "$@"
