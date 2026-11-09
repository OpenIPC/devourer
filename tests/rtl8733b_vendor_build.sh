#!/usr/bin/env bash
# rtl8733b_vendor_build.sh — build the pinned vendor RTL8733BU kernel module
# (reference/rtl8733bu-20230626, the tree devourer's firmware, tables and
# register constants were extracted from) against a kernel, out of the
# submodule so the checkout stays clean. The .ko is the kernel cell for
# tests/regress.py on this family and the A/B control docs/rtl8733b.md was
# missing (issue #393: the earlier attempt used the 2024 tree and failed on
# MODULE_IMPORT_NS / platform_ops.h; the pinned tree carries the >= 6.13
# MODULE_IMPORT_NS guard and ships platform/platform_ops.*).
#
# Default: the running host kernel (KSRC=/lib/modules/$(uname -r)/build).
# Inside the pinned-kernel VM (tests/setup_vm.sh) the same script builds
# against 5.15. Prints the .ko path on success; on failure the first errors
# from the make log, and exits 1 — "did not build on kernel X" is itself the
# documented outcome.
#
#   tests/rtl8733b_vendor_build.sh                    # host kernel
#   KVER=5.15.0-xx-generic tests/rtl8733b_vendor_build.sh
#   BUILD_DIR=/tmp/v8733 tests/rtl8733b_vendor_build.sh
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="$ROOT/reference/rtl8733bu-20230626"
KVER="${KVER:-$(uname -r)}"
KSRC="${KSRC:-/lib/modules/$KVER/build}"
BUILD_DIR="${BUILD_DIR:-$ROOT/build/vendor-8733bu}"
JOBS="${JOBS:-$(nproc)}"

[ -f "$SRC/Makefile" ] || { echo "FAIL: $SRC not checked out (git submodule update --init reference/rtl8733bu-20230626)"; exit 2; }
[ -d "$KSRC" ] || { echo "FAIL: no kernel build tree at $KSRC"; exit 2; }

mkdir -p "$BUILD_DIR"
LOG="$BUILD_DIR/make.log"
# A fresh copy every run: kbuild writes objects beside the sources, and a stale
# copy built against another kernel would pass a vermagic check it should fail.
rm -rf "${BUILD_DIR:?}/src"
cp -r "$SRC" "$BUILD_DIR/src"
echo "== building rtl8733bu-20230626 ($(git -C "$SRC" rev-parse --short HEAD)) against $KVER =="
echo "   KSRC=$KSRC  log=$LOG"
if ! make -C "$BUILD_DIR/src" -j"$JOBS" KVER="$KVER" KSRC="$KSRC" >"$LOG" 2>&1; then
  echo "FAIL: vendor module did not build against $KVER"
  grep -nE "error:|Error |No such file" "$LOG" | head -12
  exit 1
fi
KO=$(find "$BUILD_DIR/src" -maxdepth 1 -name '*.ko' | head -1)
[ -n "$KO" ] || { echo "FAIL: make succeeded but no .ko produced (see $LOG)"; exit 1; }
echo "OK: $KO"
modinfo "$KO" 2>/dev/null | grep -E "^(filename|vermagic|version):" | sed 's/^/   /'
grep -c "warning:" "$LOG" | sed 's/^/   warnings: /'
