#!/usr/bin/env bash
# Vendor the RTL8852B halrf-G6 RF-calibration BACKEND into devourer's hal/
# tree, joining the 8852C backend under hal/halrf/g6/vendor/. The chip-
# agnostic halrf core .c files are already vendored (from reference/
# rtl8852cu) and carry RF_8852B_SUPPORT dispatch — only the per-chip backend
# comes from the rtl8852bu tree. Copies are UNMODIFIED vendor source; all
# interception is via the shared shim, same as the 8852C.
#
# Re-run after `git submodule update` to re-sync. Review `git diff` on the
# copies before committing.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$REPO/reference/rtl8852bu/phl/hal_g6/phy/rf"
DST="$REPO/hal/halrf/g6/vendor"

if [[ ! -d "$SRC" ]]; then
  echo "missing $SRC -> git submodule update --init reference/rtl8852bu" >&2
  exit 1
fi

# The 8852B backend translation units behind the calibrations devourer runs
# (DACK / RX-DCK / IQK / LCK-RCK prereqs / efuse trim + TSSI-DE load).
C_FILES=(
  halrf_8852b/halrf_8852b.c          # rf tune (halrf_ctrl_ch/bw_8852b), rx_dck, lck
  halrf_8852b/halrf_8852b_api.c
  halrf_8852b/halrf_reg_cfg_8852b.c
  halrf_8852b/halrf_hwimg_8852b.c    # radio A/B + NCTL raw data + loaders
  halrf_8852b/halrf_dack_8852b.c
  halrf_8852b/halrf_iqk_8852b.c
  halrf_8852b/halrf_tssi_8852b.c     # halrf_tssi_get_efuse_8852b (DE loader) only
  halrf_8852b/halrf_kfree_8852b.c    # efuse trim (thermal / PA-bias)
  halrf_8852b/halrf_efuse_8852b.c
  halrf_8852b/halrf_ops_rtl8852b.c   # rf_set_ops_8852b (rf_info ops/backup tables)
  # dpk/txgapk compile in but never run: the rfk_ops_8852b function-pointer
  # table references them as data (gc-sections cannot drop data refs), and
  # the glue's support_ability mask keeps the trackers permanently off (they
  # servo TX power against the fixed-dBm model and measurably degrade TX).
  halrf_8852b/halrf_dpk_8852b.c
  halrf_8852b/halrf_txgapk_8852b.c
)
# NOT vendored: halrf_psd_8852b.c — a lab PSD probe outside the link closure.
# halrf_set_pwr_table_8852b.c — the regulatory TX-power-LIMIT subsystem
# devourer deliberately omits (fixed-dBm txpwr; the caller owns compliance).

mkdir -p "$DST/halrf_8852b"

echo "copying halrf_8852b .h headers ..."
cp -f "$SRC"/halrf_8852b/*.h "$DST"/halrf_8852b/

echo "copying halrf_8852b .c translation units ..."
for f in "${C_FILES[@]}"; do
  cp -f "$SRC/$f" "$DST/$f"
  echo "  $f"
done

echo "vendored $(ls "$DST"/halrf_8852b/*.c | wc -l) .c + $(ls "$DST"/halrf_8852b/*.h | wc -l) .h into $DST/halrf_8852b"
echo "done. review 'git status hal/halrf/g6/vendor'"
