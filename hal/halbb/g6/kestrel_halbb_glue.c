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
  int env_mntr_ready; /* lazy halbb_env_mntr init done */
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
  c->bb.cr_type = BB_CLIENT;     /* USB client CR bank; without it the env-monitor
                                  * CR-init switch falls through and the NHM
                                  * ready-bit address stays 0 (report never rdy) */
  c->bb.bb0_cr_offset = 0;       /* bridge adds the BB window (wIndex=1) */
  /* halbb_init_cr_default guards on this flag; it asserts exactly the wiring
   * done above (hal_com/phl_com/bb_cmn_hooker present). */
  c->bb.bb_cmn_info_init_ready = true;
  br->bb_info = &c->bb; /* halrf's cross-plane rtw_hal_bb_* calls (shim) */
  return c;
}

/* Apply the built-in BB register table + load the gain table + the efuse RX
 * gain offsets — the vendor's own loader flow (halbb_init_reg ->
 * halbb_init_cr_default + halbb_init_gain_table + halbb_get_efuse_ofst_init),
 * walking the vendored hwimg arrays with the vendor's check-positive engine.
 * Returns nonzero on success. */
int kestrel_halbb_init_reg(struct kestrel_halbb_ctx *ctx) {
  if (!ctx)
    return 0;
  return halbb_init_reg(&ctx->bb) ? 1 : 0;
}

void kestrel_halbb_rx_bringup(struct kestrel_halbb_ctx *ctx) {
  if (!ctx)
    return;
  struct bb_info *bb = &ctx->bb;
  /* The gain cache (bb_gain_i) is populated by kestrel_halbb_init_reg's
   * halbb_init_gain_table; this entry owns the front-end routing + chains. */
  /* LNAON / TRSW / PAPE RF-front-end routing (the both-bands deafness fix on
   * the 8852C). Vendor core dispatcher — per-chip via ic_type. */
  halbb_gpio_setting_init(bb);
  /* Enable both RX chains (rx_path_en). Vendor core dispatcher; its 8852C
   * case passes hal_com->dbcc_en (0 here) — same as the direct call did. */
  halbb_ctrl_rx_path(bb, RF_PATH_AB, HW_PHY_0);

  /* 8852C physts MEASUREMENT bring-up. Devourer's minimal halbb bring-up runs
   * only halbb_init_reg (BB table) + this, skipping the vendor's
   * halbb_dm_init_per_phy measurement inits. On the 8852B the physts engine
   * fills reports anyway (its table covers it); on the 8852C the reports arrive
   * EMPTY (is_valid=0) without these — halbb_ic_hw_setting_init_8852c sets the
   * BB evm/measurement report-enable (0xa10[0]), and the physts CR map + IE
   * bitmap must be programmed for the engine to snapshot RSSI/SNR/EVM. */
#ifdef BB_8852C_SUPPORT
  if (bb->ic_type == BB_RTL8852C) {
    halbb_ic_hw_setting_init_8852c(bb); /* 0xa10[0] evm/measurement rpt-en */
    halbb_cr_cfg_physts_init(bb);
    halbb_physts_parsing_init(bb);
  }
#endif
}

void kestrel_halbb_set_gain(struct kestrel_halbb_ctx *ctx, unsigned char central_ch,
                            unsigned char band_type) {
  if (!ctx)
    return;
  struct bb_info *bb = &ctx->bb;
  if (bb->ic_type == BB_RTL8852B) {
#ifdef BB_8852B_SUPPORT
    /* The 8852B loader applies both paths in one call (no efem arg). */
    halbb_set_gain_error_8852b(bb, central_ch, (enum band_type)band_type);
#endif
  } else {
#ifdef BB_8852C_SUPPORT
    /* is_efem=false (compact USB dongle, no external LNA). Vendor skips 2.4G
     * gain-error internally. */
    halbb_set_gain_error_8852c(bb, central_ch, (enum band_type)band_type,
                               false, RF_PATH_A);
    halbb_set_gain_error_8852c(bb, central_ch, (enum band_type)band_type,
                               false, RF_PATH_B);
#endif
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
#ifdef BB_8852C_SUPPORT
  if (ctx && ctx->bb.ic_type == BB_RTL8852C)
    halbb_ctrl_tx_path_tmac_8852c(&ctx->bb, RF_PATH_A, false);
#else
  (void)ctx;
#endif
}

/* One-shot active/frame-free NHM absolute noise floor. See the header. */
int kestrel_halbb_env_mntr_nhm(struct kestrel_halbb_ctx *ctx,
                               unsigned short mntr_time_ms,
                               signed char *nhm_pwr_dbm) {
  struct bb_info *bb;
  struct ccx_para_info para;
  struct env_trig_rpt trig;
  struct env_mntr_rpt rpt;
  void *drv;
  u16 mtime;
  int i;

  if (!ctx || !nhm_pwr_dbm)
    return 0;
  bb = &ctx->bb;
  drv = bb->hal_com->drv_priv;
  mtime = mntr_time_ms ? mntr_time_ms : 100;

  /* Lazy env-monitor init: the RX bring-up flow (kestrel_halbb_init_reg) only
   * runs halbb_init_reg, so the NHM CR addresses (incl. nhm_rdy) and the ccx/nhm
   * state are still zero. Run the two vendor inits once before the first
   * trigger. */
  if (!ctx->env_mntr_ready) {
    halbb_cr_cfg_env_mntr_init(bb);
    halbb_env_mntr_init(bb);
    ctx->env_mntr_ready = 1;
  }

  /* One-shot NHM in the vendor's IEEE-11k idle-noise mode. A zeroed para gives
   * nhm_incld_cca=NHM_EXCLUDE_CCA + nav/rssi gating DISABLED (all enum 0). */
  _os_mem_set(0, &para, 0, sizeof(para));
  para.rac_lv = RAC_LV_4;      /* debug priority: always wins the racing arbiter */
  para.mntr_time = mtime;      /* ms; drives ccx_period (must be nonzero) */
  para.nhm_app = NHM_DBG_11K;  /* 802.11k noise-floor thresholds */

  if (!(halbb_env_mntr_trigger(bb, &para, &trig, HW_PHY_0) & NHM_SUCCESS))
    return 0;

  /* The measurement runs mntr_time in HW; wait the window, then poll the ready
   * bit (halbb_env_mntr_result returns NHM_SUCCESS once nhm_rdy is set). */
  _os_delay_ms(drv, mtime);
  for (i = 0; i < 8; ++i) {
    _os_mem_set(0, &rpt, 0, sizeof(rpt));
    if ((halbb_env_mntr_result(bb, &rpt, HW_PHY_0) & NHM_SUCCESS) &&
        rpt.nhm_pwr != ENV_MNTR_FAIL_BYTE) {
      *nhm_pwr_dbm = (signed char)((int)rpt.nhm_pwr - 110); /* 110+x dBm */
      return 1;
    }
    _os_delay_ms(drv, 3);
  }
  return 0;
}

void kestrel_halbb_destroy(struct kestrel_halbb_ctx *ctx) {
  if (ctx)
    free(ctx);
}
