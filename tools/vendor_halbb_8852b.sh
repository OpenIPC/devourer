#!/usr/bin/env bash
# Vendor the RTL8852B halbb-G6 baseband BACKEND into devourer's hal/ tree,
# joining the 8852C backend under hal/halbb/g6/vendor/. The chip-agnostic
# halbb core .c files are already vendored (from reference/rtl8852cu, the
# newer drop) and carry BB_8852B_SUPPORT dispatch — only the per-chip backend
# comes from the rtl8852bu tree. Copies are UNMODIFIED vendor source; all
# interception is via the shim (hal/halbb/g6/shim/), same as the 8852C.
#
# Re-run after `git submodule update` to re-sync. Review `git diff` on the
# copies before committing.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$REPO/reference/rtl8852bu/phl/hal_g6/phy/bb"
DST="$REPO/hal/halbb/g6/vendor"

if [[ ! -d "$SRC" ]]; then
  echo "missing $SRC -> git submodule update --init reference/rtl8852bu" >&2
  exit 1
fi

# The 8852B backend translation units the generic dispatchers call into
# (gain cache / gpio routing / rx path / ctrl_bw_ch / table apply).
C_FILES=(
  halbb_8852b/halbb_8852b.c
  halbb_8852b/halbb_8852b_api.c      # ctrl_bw_ch / set_gain_error / gpio_* / rx_path
  halbb_8852b/halbb_reg_cfg_8852b.c  # halbb_cfg_bb_gain_8852b
  halbb_8852b/halbb_hwimg_8852b.c    # phy_reg / gain raw data + loaders
)
# NOT vendored: halbb_8852b_fwofld_api.c — FW-OFLD is compiled off
# (HALBB_FW_OFLD_SUPPORT=0); the stubs live in kestrel_halbb_helpers.c.

mkdir -p "$DST/halbb_8852b"

echo "copying halbb_8852b .h headers ..."
cp -f "$SRC"/halbb_8852b/*.h "$DST"/halbb_8852b/

echo "copying halbb_8852b .c translation units ..."
for f in "${C_FILES[@]}"; do
  cp -f "$SRC/$f" "$DST/$f"
  echo "  $f"
done

echo "vendored $(ls "$DST"/halbb_8852b/*.c | wc -l) .c + $(ls "$DST"/halbb_8852b/*.h | wc -l) .h into $DST/halbb_8852b"
echo "done. review 'git status hal/halbb/g6/vendor'"
