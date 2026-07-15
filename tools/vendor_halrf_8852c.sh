#!/usr/bin/env bash
# Vendor the RTL8852C halrf-G6 RF source (channel tune + calibrations:
# DACK/IQK/DPK/TSSI/txgapk/kfree) into devourer's hal/ tree so it can be
# compiled VERBATIM as C behind the shared halbb shim, replacing the
# hand-rolled RF C++ in HalKestrel. Sibling of tools/vendor_halbb_8852c.sh;
# reuses hal/halbb/hal_headers_le.h via a thin hal/halrf/hal_headers_le.h that
# adds the RF-register plane (halrf_wrf -> 3-wire) to the bridge.
#
# The copies are UNMODIFIED vendor source. Re-run after `git submodule update`.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$REPO/reference/rtl8852cu/phl/hal_g6/phy/rf"
DST="$REPO/hal/halrf/rtl8852c/vendor"

if [[ ! -d "$SRC" ]]; then
  echo "missing $SRC -> git submodule update --init reference/rtl8852cu" >&2
  exit 1
fi

# The .c translation units for the 8852C RF tune + calibration. Extend as the
# compile/link frontier reveals transitive callees; keep this list authoritative.
C_FILES=(
  halrf_interface.c        # halrf_wreg (BB) / halrf_wrf (RF 3-wire)
  halrf.c                  # core
  halrf_api.c
  halrf_init.c
  halrf_hw_cfg.c
  halrf_iqk.c
  halrf_8852c/halrf_8852c.c
  halrf_8852c/halrf_8852c_api.c
  halrf_8852c/halrf_reg_cfg_8852c.c
  halrf_8852c/halrf_hwimg_8852c.c
  halrf_8852c/halrf_dack_8852c.c
  halrf_8852c/halrf_iqk_8852c.c
  halrf_8852c/halrf_dpk_8852c.c
  halrf_8852c/halrf_tssi_8852c.c
  halrf_8852c/halrf_txgapk_8852c.c
  halrf_8852c/halrf_kfree_8852c.c
  halrf_8852c/halrf_efuse_8852c.c
  halrf_8852c/halrf_ops_rtl8852c.c   # rf_set_ops_8852c (rf_info ops/backup tables)
)
# NOT vendored (yet): halrf_pwr_table.c + halrf_set_pwr_table_8852c.c — the
# TX-power-limit table machinery is a distinct subsystem (devourer has its own
# txpwr path) that drags heavy PHL TPE/DAG/txpwr_lmt structs; scope halrf to the
# RF channel-tune + calibrations (DACK/IQK/DPK/TSSI/txgapk/kfree) first.

mkdir -p "$DST" "$DST/halrf_8852c"

echo "copying halrf .h type headers (rf/ + halrf_8852c/) ..."
cp -f "$SRC"/*.h "$DST"/
cp -f "$SRC"/halrf_8852c/*.h "$DST"/halrf_8852c/

echo "copying halrf .c translation units ..."
for f in "${C_FILES[@]}"; do
  cp -f "$SRC/$f" "$DST/$f"
  echo "  $f"
done

echo "vendored $(ls "$DST"/*.c "$DST"/halrf_8852c/*.c | wc -l) .c + $(ls "$DST"/*.h "$DST"/halrf_8852c/*.h | wc -l) .h into $DST"
echo "done. review 'git status hal/halrf/rtl8852c/vendor' + the shim hal/halrf/hal_headers_le.h"
