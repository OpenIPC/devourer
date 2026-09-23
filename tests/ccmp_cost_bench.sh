#!/usr/bin/env bash
# Build + run tests/ccmp_cost_bench.cpp: per-frame cost of the software CCMP
# the AP harnesses use, on this host. openssl (-lcrypto) required. No device.
set -eu
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="${TMPDIR:-/tmp}/ccmp_cost_bench.$$"
trap 'rm -f "$BIN"' EXIT
g++ -std=c++20 -O2 "$REPO/tests/ccmp_cost_bench.cpp" -lcrypto -o "$BIN"
echo "# $(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2 | sed 's/^ //') — single core, $(nproc) online" >&2
"$BIN" "${1:-20000}"
