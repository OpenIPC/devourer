/* C glue: owns an rf_info and drives the vendored halrf-G6 8852C RF cals.
 * Compiled as C behind the shared shim. See the header. */
#include "halrf_precomp.h"      /* full rf_info + halrf types + cal functions */
#include "kestrel_halrf_glue.h"

struct kestrel_halrf_ctx {
  struct rf_info rf;
  struct rtw_hal_com_t hal;
  struct rtw_phl_com_t phl;
};

struct kestrel_halrf_ctx *kestrel_halrf_create(struct kestrel_halbb_bridge *br,
                                               unsigned char cut,
                                               unsigned char rfe_type) {
  struct kestrel_halrf_ctx *c =
      (struct kestrel_halrf_ctx *)malloc(sizeof(struct kestrel_halrf_ctx));
  if (!c)
    return 0;
  _os_mem_set(0, c, 0, sizeof(*c));

  c->hal.drv_priv = br;          /* -> kestrel_halbb_bridge (BB + RF planes) */
  c->hal.cv = cut;
  c->hal.chip_id = CHIP_WIFI6_8852C; /* halrf_cmn_info_self_init keys on this */
  c->phl.dev_cap.rfe_type = rfe_type;

  c->rf.hal_com = &c->hal;
  c->rf.phl_com = &c->phl;
  c->rf.ic_type = RF_RTL8852C;
  /* Register the 8852C RFK ops + backup-register tables (rf->rfk_iqk_info,
   * rf_iqk_ops, backup_*_reg) that the per-channel cals (IQK) dereference.
   * halrf_cmn_info_self_init derives ic_type from hal_com->chip_id (set above),
   * then wires rfk_iqk_info = &rf_iqk_hwspec_8852c. Verified: IQK no longer
   * null-derefs. DACK/RX-DCK don't need it (direct _8852c entries). */
  halrf_cmn_info_self_init(&c->rf);
  return c;
}

/* Defined in the vendored halrf_init.c but not declared in any halrf header. */
void halrf_rfk_self_init(struct rf_info *rf);

void kestrel_halrf_rfk_init(struct kestrel_halrf_ctx *ctx) {
  if (!ctx)
    return;
  /* Loads array_mp_8852c_nctl_reg into the one-shot NCTL micro-engine (the
   * sequencer that signals 0x55 @ 0xbff8 when a cal one-shot completes) and
   * resets the per-cal RFK sub-state. Mirrors the halrf_dm_init prologue. */
  halrf_config_nctl_reg(&ctx->rf);
  halrf_rfk_self_init(&ctx->rf);
}

void kestrel_halrf_dac_cal(struct kestrel_halrf_ctx *ctx, unsigned char force) {
  if (ctx)
    halrf_dac_cal_8852c(&ctx->rf, force ? true : false);
}

void kestrel_halrf_rx_dck(struct kestrel_halrf_ctx *ctx) {
  if (ctx)
    halrf_rx_dck_8852c(&ctx->rf, HW_PHY_0, false);
}

void kestrel_halrf_set_ch(struct kestrel_halrf_ctx *ctx, unsigned char center_ch,
                          unsigned char band, unsigned char bw) {
  if (!ctx)
    return;
  struct rtw_chan_def *cd = &ctx->hal.band[HW_PHY_0].cur_chandef;
  cd->center_ch = center_ch;
  cd->chan = center_ch;
  cd->band = (enum band_type)band;
  cd->bw = (enum channel_width)bw;
}

void kestrel_halrf_iqk(struct kestrel_halrf_ctx *ctx) {
  if (ctx)
    halrf_iqk(&ctx->rf, HW_PHY_0, true);
}

void kestrel_halrf_destroy(struct kestrel_halrf_ctx *ctx) {
  if (ctx)
    free(ctx);
}
