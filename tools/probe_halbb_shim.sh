#!/usr/bin/env bash
# Probe-compile the vendored halbb-G6 8852C .c against the shim, one TU at a
# time, to surface the current error frontier while the shim is being stood up.
# Not part of the CMake build — a development driver for Stage A/B/C.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
V="$REPO/hal/halbb/rtl8852c/vendor"
INCS=(
  -I"$REPO/hal/halbb"              # shim hal_headers_le.h (../../ from vendor/)
  -I"$REPO/hal/halbb/rtl8852c"     # shim/ bridge
  -I"$V"                           # vendor bb-level headers
  -I"$V/halbb_8852c"               # vendor 8852c headers
)
DEFS=(
  -DCONFIG_RTL8852C=1 -DBB_8852C_SUPPORT=1
  -DHALBB_DBCC_SUPPORT=0 -DHALBB_FW_OFLD_SUPPORT=0
  -DHALBB_DBG_TRACE_SUPPORT=0 -DPLATFOM_IS_LITTLE_ENDIAN=1
)
TU="${1:-halbb_interface.c}"
echo "== probe: $TU =="
gcc -std=gnu99 -fsyntax-only -w "${DEFS[@]}" "${INCS[@]}" "$V/$TU" 2>&1 | head -40
echo "== (top 40 diagnostics) =="
