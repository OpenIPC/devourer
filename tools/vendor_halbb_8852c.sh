#!/usr/bin/env bash
# Vendor the RTL8852C halbb-G6 baseband source into devourer's hal/ tree so the
# RX bring-up (halbb_dm_init_per_phy / gpio / gain-error / ctrl_rx_path) can be
# compiled VERBATIM as C behind a thin shim (src: hal/halbb/g6/shim/),
# rather than hand-transcribed into C++. Mirrors how hal/ already carries the
# extract-generated tables — but here we copy whole .c/.h so the committed build
# does not need the reference/ submodule checked out.
#
# The copies are UNMODIFIED vendor source. The only interception is via the
# shim: hal/halbb/g6/shim/ pre-defines the _HAL_HEADERS_LE_H_ include
# guard and supplies a minimal PHL replacement, so every vendor
# `#include "../../hal_headers_le.h"` becomes a no-op and the giant PHL/MAC/BTC
# type graph is never pulled in.
#
# Re-run after `git submodule update` to re-sync. Review `git diff` on the
# copies before committing.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$REPO/reference/rtl8852cu/phl/hal_g6/phy/bb"
DST="$REPO/hal/halbb/g6/vendor"

if [[ ! -d "$SRC" ]]; then
  echo "missing $SRC -> git submodule update --init reference/rtl8852cu" >&2
  exit 1
fi

# The .c translation units needed for the 8852C monitor-RX bring-up. Extend as
# the compile frontier reveals transitive callees; keep this list authoritative.
C_FILES=(
  halbb_interface.c        # register I/O wrappers (halbb_set_reg -> hal_write32)
  halbb_init.c             # halbb_dm_init_per_phy
  halbb_api.c              # init dispatchers, halbb_gpio_setting_init
  halbb_8852c/halbb_8852c.c
  halbb_8852c/halbb_8852c_api.c      # ctrl_rx_path / set_gain_error / gpio_*
  halbb_8852c/halbb_reg_cfg_8852c.c  # halbb_cfg_bb_gain_8852c
  halbb_8852c/halbb_hwimg_8852c.c
  halbb_dig.c
  halbb_edcca.c
  halbb_cfo_trk.c
  halbb_env_mntr.c
  halbb_physts.c
  halbb_ch_info.c
  halbb_hw_cfg.c           # halbb_sel_headline, halbb_cfg_bb_rpl_ofst
  halbb_math_lib.c         # bit helpers (halbb_cal_bit_shift drives every reg write)
  halbb_pmac_setting.c     # halbb_backup/restore_info + set_tmac_tx (halrf cal hooks)
)
# NOT vendored: halbb_fwofld.c — with FW-OFLD off its 3 referenced functions
# (check_fw_ofld/fw_delay/fw_set_reg) are disabled-path stubs, provided by
# hal/halbb/g6/kestrel_halbb_helpers.c instead of dragging the whole
# cmd-offload surface.

mkdir -p "$DST" "$DST/halbb_8852c"

echo "copying halbb .h type headers (bb/ + halbb_8852c/) ..."
cp -f "$SRC"/*.h "$DST"/
cp -f "$SRC"/halbb_8852c/*.h "$DST"/halbb_8852c/

echo "copying halbb .c translation units ..."
for f in "${C_FILES[@]}"; do
  cp -f "$SRC/$f" "$DST/$f"
  echo "  $f"
done

echo "vendored $(ls "$DST"/*.c "$DST"/halbb_8852c/*.c | wc -l) .c + $(ls "$DST"/*.h "$DST"/halbb_8852c/*.h | wc -l) .h into $DST"
echo "done. review 'git status hal/halbb/g6/vendor' and the shim in hal/halbb/g6/shim/"
