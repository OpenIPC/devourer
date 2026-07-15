/* Disabled-path stubs for the vendored halbb-G6 8852C RX bring-up.
 *
 * With FW offload OFF (HALBB_FW_OFLD_SUPPORT=0, dev_cap.io_ofld=0) the register
 * writers reference three fwofld entry points that would otherwise drag the
 * whole cmd-offload/MAC surface in. check_fw_ofld returns false (so the direct
 * register path is always taken); fw_delay/fw_set_reg are never reached at
 * runtime. Providing them here avoids vendoring halbb_fwofld.c. Compiled as C
 * behind the shim, same as the vendored .c. */
#include "halbb_precomp.h"

bool halbb_check_fw_ofld(struct bb_info *bb) { (void)bb; return false; }

bool halbb_fw_delay(struct bb_info *bb, u32 val) { (void)bb; (void)val; return false; }

bool halbb_fw_set_reg(struct bb_info *bb, u32 addr, u32 mask, u32 val, u8 lc) {
  (void)bb; (void)addr; (void)mask; (void)val; (void)lc; return false;
}

bool halbb_fw_set_reg_cmn(struct bb_info *bb, u32 addr, u32 mask, u32 val,
                          enum phl_phy_idx phy_idx, u8 lst_cmd) {
  (void)bb; (void)addr; (void)mask; (void)val; (void)phy_idx; (void)lst_cmd;
  return false;
}

/* halbb_pmac_setting.c brackets set_tmac_tx/restore_info with this fwofld
 * batching toggle — with io_ofld off it only flips bb_cmn_hooker state the
 * direct register path never reads. */
void halbb_fwofld_bitmap_en(struct bb_info *bb, bool en, enum fw_ofld_type app) {
  (void)bb; (void)en; (void)app;
}

/* Spur suppression (halbb_spur_suppress.c, not vendored): channel-specific NBI
 * /CSI notch, not RX-hearing-critical — no-op (no spur notch). */
void halbb_csi_tone_idx(struct bb_info *bb, u8 central_ch, enum channel_width bw,
                        enum band_type band, enum phl_phy_idx phy_idx) {
  (void)bb; (void)central_ch; (void)bw; (void)band; (void)phy_idx;
}
void halbb_nbi_tone_idx(struct bb_info *bb, u8 central_ch, u8 pri_ch,
                        enum channel_width bw, enum band_type band,
                        enum rf_path path) {
  (void)bb; (void)central_ch; (void)pri_ch; (void)bw; (void)band; (void)path;
}
