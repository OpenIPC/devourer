/* C glue: owns a bb_info and drives the vendored halbb-G6 8852C RX functions.
 * Compiled as C behind the shim (hal/halbb/hal_headers_le.h). See the header. */
#include "halbb_precomp.h"      /* full bb_info + halbb types + RX functions */
#include "kestrel_halbb_glue.h"

struct kestrel_halbb_ctx {
  struct bb_info bb;
  struct rtw_hal_com_t hal;
  struct rtw_phl_com_t phl;
  struct bb_cmn_info cmn;
};

struct kestrel_halbb_ctx *kestrel_halbb_create(struct kestrel_halbb_bridge *br,
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
  br->bb_info = &c->bb; /* halrf's cross-plane rtw_hal_bb_* calls (shim) */
  c->bb.bb_phy_idx = HW_PHY_0;
  c->bb.ic_type = BB_RTL8852C;
  c->bb.ic_sub_type = BB_IC_SUB_TYPE_8852C_8852C;
  c->bb.rx_path = RF_PATH_AB;
  c->bb.bb0_cr_offset = 0;       /* bridge adds the BB window (wIndex=1) */
  return c;
}

void kestrel_halbb_rx_bringup(struct kestrel_halbb_ctx *ctx) {
  if (!ctx)
    return;
  struct bb_info *bb = &ctx->bb;
  /* Populate bb_gain_i from the built-in array_mp_8852c_phy_reg_gain. */
  halbb_cfg_bb_gain_ax_8852c(bb, false, 0, 0);
  /* LNAON / TRSW / PAPE RF-front-end routing (the both-bands deafness fix). */
  halbb_gpio_setting_init_8852c(bb);
  /* Enable both RX chains (0x4978 rx_path_en). */
  halbb_ctrl_rx_path_8852c(bb, RF_PATH_AB, false);
}

void kestrel_halbb_set_gain(struct kestrel_halbb_ctx *ctx, unsigned char central_ch,
                            unsigned char band_type) {
  if (!ctx)
    return;
  /* is_efem=false (compact USB dongle, no external LNA). Vendor skips 2.4G
   * gain-error internally. */
  halbb_set_gain_error_8852c(&ctx->bb, central_ch, (enum band_type)band_type,
                             false, RF_PATH_A);
  halbb_set_gain_error_8852c(&ctx->bb, central_ch, (enum band_type)band_type,
                             false, RF_PATH_B);
}

void kestrel_halbb_ctrl_bw_ch(struct kestrel_halbb_ctx *ctx, unsigned char pri_ch,
                              unsigned char central_ch, unsigned char bw,
                              unsigned char band_type) {
  if (!ctx)
    return;
  halbb_ctrl_bw_ch_8852c(&ctx->bb, pri_ch, central_ch, (enum channel_width)bw,
                         (enum band_type)band_type, HW_PHY_0);
}

void kestrel_halbb_ctrl_tx_path(struct kestrel_halbb_ctx *ctx) {
  if (ctx)
    halbb_ctrl_tx_path_tmac_8852c(&ctx->bb, RF_PATH_A, false);
}

void kestrel_halbb_destroy(struct kestrel_halbb_ctx *ctx) {
  if (ctx)
    free(ctx);
}
