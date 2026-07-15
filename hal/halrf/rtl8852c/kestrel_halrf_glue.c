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
  /* Enable the one-shot cals (LCK/RCK/DACK/RXDCK/IQK/DPK/TSSI setup) but CLEAR
   * the ongoing TX-power-TRACKING servos: devourer runs a fixed BB dBm
   * (halbb_set_txpwr_dbm), so halrf_tssi_enable's closed-loop servo (gated on
   * HAL_RF_TX_PWR_TRACK) would fight it and pull TX power down (measured:
   * duty 74%->47%). Keep TSSI's static BB power-ctrl setup, drop the trackers. */
  c->rf.support_ability = 0xffffffffu &
      ~(HAL_RF_TX_PWR_TRACK | HAL_RF_TSSI_TRK | HAL_RF_DPK_TRACK |
        HAL_RF_RXDCK_TRACK);
  return c;
}

/* Defined in the vendored halrf_init.c but not declared in any halrf header. */
void halrf_rfk_self_init(struct rf_info *rf);

void kestrel_halrf_rfk_init(struct kestrel_halrf_ctx *ctx) {
  if (!ctx)
    return;
  struct rf_info *rf = &ctx->rf;
  /* Mirror the halrf_dm_init cal prologue that must run before the per-channel
   * IQK/TSSI/DPK. Loads array_mp_8852c_nctl_reg into the one-shot NCTL micro-
   * engine (signals 0x55 @ 0xbff8 on cal completion), resets the per-cal RFK
   * sub-state, then the synth/filter cals (LCK/RCK) the per-channel cals depend
   * on and the efuse trim/TSSI data TSSI consumes. */
  halrf_config_nctl_reg(rf);
  halrf_rfk_self_init(rf);
  halrf_lck_trigger(rf);
  halrf_rck_trigger(rf, HW_PHY_0);
  /* Thermal / PA-bias / TSSI trim from efuse (kfree subsystem) — this is what
   * TSSI's accuracy depends on. NOT halrf_tssi_get_efuse_ex: that loads the
   * regulatory TX-power-LIMIT switch (mac_ax pwr-limit ops), a subsystem
   * devourer deliberately omits (fixed-dBm txpwr, no regulatory enforcement)
   * and whose absence does not affect the TSSI calibration's operation. */
  halrf_get_efuse_trim(rf, HW_PHY_0);
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

void kestrel_halrf_tssi(struct kestrel_halrf_ctx *ctx) {
  if (ctx)
    /* hwtx_en=false: skip the TSSI alignment-K (_halrf_tssi_alimentk), which
     * TXes and re-trims TXAGC toward a TSSI target — unwanted under fixed dBm.
     * Runs the static TSSI BB power-ctrl setup only. */
    halrf_do_tssi_8852c(&ctx->rf, HW_PHY_0, false);
}

void kestrel_halrf_dpk(struct kestrel_halrf_ctx *ctx) {
  if (ctx)
    /* Call the raw cal directly, not halrf_dpk_trigger — the wrapper
     * early-returns on !(rf->support_ability & HAL_RF_DPK), which the glue
     * doesn't populate (mirrors how kestrel_halrf_iqk calls halrf_iqk). */
    halrf_dpk_8852c(&ctx->rf, HW_PHY_0, true);
}

void kestrel_halrf_destroy(struct kestrel_halrf_ctx *ctx) {
  if (ctx)
    free(ctx);
}
