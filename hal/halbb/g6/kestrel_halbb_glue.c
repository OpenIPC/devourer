/* C glue: owns a bb_info and drives the vendored halbb-G6 BB functions for
 * both Kestrel chips. Compiled as C behind the shim (hal/halbb/
 * hal_headers_le.h). See the header. */
#include "halbb_precomp.h"      /* full bb_info + halbb types + BB functions */
#include "kestrel_halbb_glue.h"

struct kestrel_halbb_ctx {
  struct bb_info bb;
  struct rtw_hal_com_t hal;
  struct rtw_phl_com_t phl;
  struct bb_cmn_info cmn;
};

struct kestrel_halbb_ctx *kestrel_halbb_create(struct kestrel_halbb_bridge *br,
                                               enum kestrel_chip chip,
                                               unsigned char cut,
                                               unsigned char rfe_type) {
  struct kestrel_halbb_ctx *c =
      (struct kestrel_halbb_ctx *)malloc(sizeof(struct kestrel_halbb_ctx));
  if (!c)
    return 0;
  _os_mem_set(0, c, 0, sizeof(*c));

  c->hal.drv_priv = br;          /* -> kestrel_halbb_bridge (register plane) */
  c->hal.cv = cut;
  c->phl.dev_cap.rfe_type = rfe_type;

  c->bb.hal_com = &c->hal;
  c->bb.phl_com = &c->phl;
  c->bb.bb_cmn_hooker = &c->cmn;
  c->bb.bb_phy_idx = HW_PHY_0;
  if (chip == KESTREL_CHIP_8852B) {
    c->bb.ic_type = BB_RTL8852B;
    c->bb.ic_sub_type = BB_IC_SUB_TYPE_8852B_8852B;
    c->hal.chip_id = CHIP_WIFI6_8852B;
  } else {
    c->bb.ic_type = BB_RTL8852C;
    c->bb.ic_sub_type = BB_IC_SUB_TYPE_8852C_8852C;
    c->hal.chip_id = CHIP_WIFI6_8852C;
  }
  c->bb.rx_path = RF_PATH_AB;
  c->bb.bb0_cr_offset = 0;       /* bridge adds the BB window (wIndex=1) */
  br->bb_info = &c->bb; /* halrf's cross-plane rtw_hal_bb_* calls (shim) */
  return c;
}

void kestrel_halbb_rx_bringup(struct kestrel_halbb_ctx *ctx) {
  if (!ctx)
    return;
  struct bb_info *bb = &ctx->bb;
  /* Populate bb_gain_i from the built-in gain raw data (per-chip loader —
   * the halbb_cfg_bb_gain dispatcher's 8852C case adds a set_gain_cr_init
   * the validated bring-up doesn't run, so call the loaders directly). */
  if (bb->ic_type == BB_RTL8852B)
    halbb_cfg_bb_gain_ax_8852b(bb, false, 0, 0);
  else
    halbb_cfg_bb_gain_ax_8852c(bb, false, 0, 0);
  /* LNAON / TRSW / PAPE RF-front-end routing (the both-bands deafness fix on
   * the 8852C). Vendor core dispatcher — per-chip via ic_type. */
  halbb_gpio_setting_init(bb);
  /* Enable both RX chains (rx_path_en). Vendor core dispatcher; its 8852C
   * case passes hal_com->dbcc_en (0 here) — same as the direct call did. */
  halbb_ctrl_rx_path(bb, RF_PATH_AB, HW_PHY_0);
}

void kestrel_halbb_set_gain(struct kestrel_halbb_ctx *ctx, unsigned char central_ch,
                            unsigned char band_type) {
  if (!ctx)
    return;
  struct bb_info *bb = &ctx->bb;
  if (bb->ic_type == BB_RTL8852B) {
    /* The 8852B loader applies both paths in one call (no efem arg). */
    halbb_set_gain_error_8852b(bb, central_ch, (enum band_type)band_type);
  } else {
    /* is_efem=false (compact USB dongle, no external LNA). Vendor skips 2.4G
     * gain-error internally. */
    halbb_set_gain_error_8852c(bb, central_ch, (enum band_type)band_type,
                               false, RF_PATH_A);
    halbb_set_gain_error_8852c(bb, central_ch, (enum band_type)band_type,
                               false, RF_PATH_B);
  }
}

void kestrel_halbb_ctrl_bw_ch(struct kestrel_halbb_ctx *ctx, unsigned char pri_ch,
                              unsigned char central_ch, unsigned char bw,
                              unsigned char band_type) {
  if (!ctx)
    return;
  /* Vendor core dispatcher (halbb_api.c): its per-chip case forwards to
   * halbb_ctrl_bw_ch_<chip> with these exact arguments (seg1 is 80+80-only,
   * unused on both chips). */
  halbb_ctrl_bw_ch(&ctx->bb, pri_ch, central_ch, /*central_ch_seg1=*/0,
                   (enum band_type)band_type, (enum channel_width)bw,
                   HW_PHY_0);
}

void kestrel_halbb_ctrl_tx_path(struct kestrel_halbb_ctx *ctx) {
  /* 8852C-only: the 8852B routes TX antennas per-STA via the CMAC antenna
   * model (TXWD/cctl), there is no path-com block to program. */
  if (ctx && ctx->bb.ic_type == BB_RTL8852C)
    halbb_ctrl_tx_path_tmac_8852c(&ctx->bb, RF_PATH_A, false);
}

void kestrel_halbb_destroy(struct kestrel_halbb_ctx *ctx) {
  if (ctx)
    free(ctx);
}
