/* C glue: owns an rf_info and drives the vendored halrf-G6 RF cals for both
 * Kestrel chips. Compiled as C behind the shared shim. See the header. */
#include "halrf_precomp.h"      /* full rf_info + halrf types + cal functions */
#include "kestrel_halrf_glue.h"

struct kestrel_halrf_ctx {
  struct rf_info rf;
  struct rtw_hal_com_t hal;
  struct rtw_phl_com_t phl;
};

struct kestrel_halrf_ctx *kestrel_halrf_create(struct kestrel_halbb_bridge *br,
                                               enum kestrel_chip chip,
                                               unsigned char cut,
                                               unsigned char rfe_type) {
  struct kestrel_halrf_ctx *c =
      (struct kestrel_halrf_ctx *)malloc(sizeof(struct kestrel_halrf_ctx));
  if (!c)
    return 0;
  _os_mem_set(0, c, 0, sizeof(*c));

  c->hal.drv_priv = br;          /* -> kestrel_halbb_bridge (BB + RF planes) */
  c->hal.cv = cut;
  /* halrf_cmn_info_self_init keys on chip_id: it derives rf->ic_type and
   * wires the per-chip rfk ops + backup-register tables (rf_iqk_ops,
   * rfk_iqk_info, backup_*_reg) the per-channel cals dereference. */
  c->hal.chip_id =
      chip == KESTREL_CHIP_8852B ? CHIP_WIFI6_8852B : CHIP_WIFI6_8852C;
  c->phl.dev_cap.rfe_type = rfe_type;

  c->rf.hal_com = &c->hal;
  c->rf.phl_com = &c->phl;
  c->rf.ic_type = chip == KESTREL_CHIP_8852B ? RF_RTL8852B : RF_RTL8852C;
  halrf_cmn_info_self_init(&c->rf);
  /* Enable the one-shot cals (LCK/RCK/DACK/RXDCK/IQK) but CLEAR the ongoing
   * TX-power-TRACKING servos and the TSSI/DPK family: devourer runs a fixed
   * BB dBm (halbb_set_txpwr_dbm), and the servos fight it. EVM-settled on the
   * 8832CU (MCS7 data via streamtx, 8822CU monitor, matched RSSI ~78):
   * TSSI+DPK enabled pulled on-air duty 74% -> 43% and medEVM -70 -> -59 —
   * they degrade TX under the fixed-power model, so the gate is permanent
   * until a TSSI-referenced power model exists. */
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
  /* Mirror the halrf_dm_init cal prologue that must run before the per-
   * channel IQK. Loads the per-chip NCTL microcode into the one-shot cal
   * engine (signals 0x55 @ 0xbff8 on cal completion), resets the per-cal RFK
   * sub-state, runs the HW/SW SI reset (real a-die work on the 8852B via the
   * bridge XSI plane; the dispatcher has no 8852C case), then the synth/
   * filter cals (LCK/RCK) the per-channel cals depend on and the efuse
   * trim/TSSI data. */
  halrf_config_nctl_reg(rf);
  halrf_rfk_self_init(rf);
  halrf_si_reset(rf);
  halrf_lck_trigger(rf);
  halrf_rck_trigger(rf, HW_PHY_0);
  /* Thermal / PA-bias / TSSI trim from efuse (kfree subsystem) — this is what
   * TSSI's accuracy depends on. NOT halrf_tssi_get_efuse_ex: that loads the
   * regulatory TX-power-LIMIT switch (mac_ax pwr-limit ops), a subsystem
   * devourer deliberately omits (fixed-dBm txpwr, no regulatory enforcement)
   * and whose absence does not affect the TSSI calibration's operation. */
  halrf_get_efuse_trim(rf, HW_PHY_0);
  /* Load the TSSI DE (differential-error) power targets from efuse — with the
   * efuse_get_info bridge wired this populates tssi_info->tssi_efuse[] so a
   * TSSI-referenced consumer reads real corrections. The mac_ax pwr-limit
   * tail of halrf_tssi_get_efuse_EX is skipped by calling the per-chip loader
   * directly. */
  if (rf->ic_type == RF_RTL8852B) {
#ifdef RF_8852B_SUPPORT
    halrf_tssi_get_efuse_8852b(rf, HW_PHY_0);
#endif
  } else {
#ifdef RF_8852C_SUPPORT
    halrf_tssi_get_efuse_8852c(rf, HW_PHY_0);
#endif
  }
}

/* Apply the built-in radio A/B register tables via the vendor's own loader
 * (halrf_config_radio: per-chip walk with the check-positive engine; each
 * a-die write rides the bridge RF plane, d-die the BB window) and send the
 * accumulated radio pages to the fw (classes 8/9 through the bridge
 * send_h2c). Returns nonzero on success. */
int kestrel_halrf_config_radio(struct kestrel_halrf_ctx *ctx) {
  if (!ctx)
    return 0;
  return halrf_config_radio(&ctx->rf, HW_PHY_0) ? 1 : 0;
}

void kestrel_halrf_dac_cal(struct kestrel_halrf_ctx *ctx, unsigned char force) {
  if (!ctx)
    return;
  /* Direct per-chip calls (the halrf_dack_trigger wrapper adds chlk_map/
   * rfk-ops gates that can silently skip the cal at bring-up). */
  if (ctx->rf.ic_type == RF_RTL8852B) {
#ifdef RF_8852B_SUPPORT
    halrf_dac_cal_8852b(&ctx->rf, force ? true : false);
#endif
  } else {
#ifdef RF_8852C_SUPPORT
    halrf_dac_cal_8852c(&ctx->rf, force ? true : false);
#endif
  }
}

void kestrel_halrf_rx_dck(struct kestrel_halrf_ctx *ctx) {
  if (!ctx)
    return;
  if (ctx->rf.ic_type == RF_RTL8852B) {
#ifdef RF_8852B_SUPPORT
    halrf_rx_dck_8852b(&ctx->rf, HW_PHY_0, false);
#endif
  } else {
#ifdef RF_8852C_SUPPORT
    halrf_rx_dck_8852c(&ctx->rf, HW_PHY_0, false);
#endif
  }
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

void kestrel_halrf_ctl_band_ch_bw(struct kestrel_halrf_ctx *ctx,
                                  unsigned char band, unsigned char central_ch,
                                  unsigned char bw) {
  if (!ctx)
    return;
  if (ctx->rf.ic_type == RF_RTL8852B) {
    /* The core generics dispatch to halrf_ctrl_ch_8852b / halrf_ctrl_bw_8852b
     * (halrf_ctl_band_ch_bw has no 8852B case); the 8852B's synth lock rides
     * inside its ctrl_ch (s0_arfc18 relock priming). */
    halrf_ctl_ch(&ctx->rf, HW_PHY_0, central_ch, (enum band_type)band);
    halrf_ctl_bw(&ctx->rf, HW_PHY_0, (enum channel_width)bw);
  } else {
#ifdef RF_8852C_SUPPORT
    halrf_ctl_band_ch_bw_8852c(&ctx->rf, HW_PHY_0, (enum band_type)band,
                               central_ch, (enum channel_width)bw);
#endif
  }
}

void kestrel_halrf_lck(struct kestrel_halrf_ctx *ctx) {
  /* 8852C-only synth relock; the 8852B locks inside its ctrl_ch. */
#ifdef RF_8852C_SUPPORT
  if (ctx && ctx->rf.ic_type == RF_RTL8852C)
    halrf_lck_8852c(&ctx->rf);
#else
  (void)ctx;
#endif
}

unsigned int kestrel_halrf_read_rf(struct kestrel_halrf_ctx *ctx,
                                   unsigned char path, unsigned int addr) {
  if (!ctx)
    return 0;
  return halrf_rrf(&ctx->rf, path, addr, MASKRF);
}

void kestrel_halrf_iqk(struct kestrel_halrf_ctx *ctx) {
  if (ctx)
    halrf_iqk(&ctx->rf, HW_PHY_0, true);
}

int kestrel_halrf_efuse_get_info(struct kestrel_halrf_ctx *ctx,
                                 unsigned char *efuse_map, unsigned int id,
                                 void *value, unsigned int size,
                                 unsigned char autoload) {
  if (!ctx || !efuse_map)
    return 0;
  bool ok = false;
  if (ctx->rf.ic_type == RF_RTL8852B) {
#ifdef RF_8852B_SUPPORT
    ok = halrf_get_efuse_info_8852b(&ctx->rf, efuse_map,
                                    (enum rtw_efuse_info)id, value, size,
                                    autoload);
#endif
  } else {
#ifdef RF_8852C_SUPPORT
    ok = halrf_get_efuse_info_8852c(&ctx->rf, efuse_map,
                                    (enum rtw_efuse_info)id, value, size,
                                    autoload);
#endif
  }
  return ok ? 1 : 0;
}

void kestrel_halrf_destroy(struct kestrel_halrf_ctx *ctx) {
  if (ctx)
    free(ctx);
}
