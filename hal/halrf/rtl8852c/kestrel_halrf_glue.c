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
  c->phl.dev_cap.rfe_type = rfe_type;

  c->rf.hal_com = &c->hal;
  c->rf.phl_com = &c->phl;
  c->rf.ic_type = RF_RTL8852C;
  return c;
}

void kestrel_halrf_dac_cal(struct kestrel_halrf_ctx *ctx, unsigned char force) {
  if (ctx)
    halrf_dac_cal_8852c(&ctx->rf, force ? true : false);
}

void kestrel_halrf_rx_dck(struct kestrel_halrf_ctx *ctx) {
  if (ctx)
    halrf_rx_dck_8852c(&ctx->rf, HW_PHY_0, false);
}

void kestrel_halrf_destroy(struct kestrel_halrf_ctx *ctx) {
  if (ctx)
    free(ctx);
}
