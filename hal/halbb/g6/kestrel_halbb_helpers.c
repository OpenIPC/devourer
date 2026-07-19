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

/* The halbb_ctrl_bw_ch dispatcher's 8852B case branches on
 * halbb_check_fw_ofld (false above), so this fwofld variant is never called —
 * the symbol just has to link (halbb_8852b_fwofld_api.c is not vendored). */
bool halbb_fwofld_ctrl_bw_ch_8852b(struct bb_info *bb, u8 pri_ch, u8 central_ch,
                                   enum channel_width bw, enum band_type band,
                                   enum phl_phy_idx phy_idx) {
  (void)bb; (void)pri_ch; (void)central_ch; (void)bw; (void)band; (void)phy_idx;
  return false;
}

/* DV/PXP (design-verification platform) debug print toggle (halbb_dv_dbg.c,
 * not vendored) — a log-gate only. */
void halbb_dv_pxp_print_en(struct bb_info *bb, bool en) {
  (void)bb; (void)en;
}

/* TPU / TSSI-ctrl MAC CR init (halbb_pwr_ctrl.c, not vendored) — the vendor's
 * TX-power-unit plane. Devourer's hand-ported fixed-dBm TXAGC owns the MAC
 * power CRs (set_txpwr_dbm, applied at every set_channel), so the vendor
 * init stays a no-op here; the pwr_ctrl TU would come in with a
 * TSSI-referenced power model. */
void halbb_tpu_mac_cr_init(struct bb_info *bb, enum phl_phy_idx phy_idx) {
  (void)bb; (void)phy_idx;
}
void halbb_tssi_ctrl_mac_cr_init(struct bb_info *bb, enum phl_phy_idx phy_idx) {
  (void)bb; (void)phy_idx;
}

/* fwofld variants behind halbb_check_fw_ofld (false above) — link-only. */
bool halbb_fwcfg_bb_phy_8852b(struct bb_info *bb, u32 addr, u32 data,
                              enum phl_phy_idx phy_idx) {
  (void)bb; (void)addr; (void)data; (void)phy_idx;
  return false;
}
void halbb_fwofld_set_gain_cr_init_8852c(struct bb_info *bb) { (void)bb; }

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

/* ---- Link-only stubs for the un-vendored halbb subsystems ----------------
 *
 * The verbatim vendor TUs reference entry points that live in halbb .c files
 * we deliberately do NOT vendor. No runtime path reaches them (the callers
 * are un-driven vendor entry points kept verbatim), and GNU --gc-sections
 * dead-strips the references on Linux — but MSVC and MinGW ld resolve
 * archive members before dead-stripping, so the symbols must exist at link.
 * Signatures match the vendored header prototypes exactly (this TU includes
 * them via halbb_precomp.h). */

/* BB debug port (halbb_dbg.c, not vendored): register-window arbitration for
 * the vendor debug console + the fw dbg-print C2H decoder. */
u32 halbb_get_bb_dbg_port_idx(struct bb_info *bb) { (void)bb; return 0; }
void halbb_set_bb_dbg_port(struct bb_info *bb, u32 dbg_port) {
  (void)bb; (void)dbg_port;
}
void halbb_set_bb_dbg_port_ip(struct bb_info *bb, enum bb_dbg_port_ip_t ip) {
  (void)bb; (void)ip;
}
void halbb_release_bb_dbg_port(struct bb_info *bb) { (void)bb; }
bool halbb_bb_dbg_port_racing(struct bb_info *bb, u8 curr_dbg_priority) {
  (void)bb; (void)curr_dbg_priority;
  return false; /* port not granted */
}
u32 halbb_get_bb_dbg_port_val(struct bb_info *bb) { (void)bb; return 0; }
void halbb_cr_hook_fake_init(struct bb_info *bb, u32 *str_table, u32 len) {
  (void)bb; (void)str_table; (void)len;
}
void halbb_cr_hook_init_dump(struct bb_info *bb, u32 *str_table, u32 len) {
  (void)bb; (void)str_table; (void)len;
}
u32 halbb_c2h_fw_dbg(struct bb_info *bb, u16 len, u8 *c2h) {
  (void)bb; (void)len; (void)c2h;
  return 0;
}

/* MP / mass-production counters (halbb_mp_ex.c, not vendored). */
u32 halbb_mp_get_rx_crc_ok(struct bb_info *bb, enum phl_phy_idx phy_idx) {
  (void)bb; (void)phy_idx;
  return 0;
}
u32 halbb_mp_get_rx_crc_err(struct bb_info *bb, enum phl_phy_idx phy_idx) {
  (void)bb; (void)phy_idx;
  return 0;
}
void halbb_dbg_port_sel(struct bb_info *bb, u16 dbg_port_sel, u8 dbg_port_ip_sel,
                        bool dbg_port_ref_clk_en, bool dbg_port_en) {
  (void)bb; (void)dbg_port_sel; (void)dbg_port_ip_sel;
  (void)dbg_port_ref_clk_en; (void)dbg_port_en;
}

/* RA rate-adaptive reports (halbb_ra.c, not vendored): rate parsing helpers
 * + the fw RA-report / TX-history / TX-status C2H decoders. */
bool halbb_is_cck_rate(struct bb_info *bb, u16 rate) {
  (void)bb; (void)rate;
  return false;
}
enum bb_mode_type halbb_get_rate_mode(struct bb_info *bb, u16 rate_idx) {
  (void)bb; (void)rate_idx;
  return BB_LEGACY_MODE;
}
void halbb_rate_idx_parsor(struct bb_info *bb, u16 rate_idx,
                           enum rtw_gi_ltf gi_ltf, struct bb_rate_info *ra_i) {
  (void)bb; (void)rate_idx; (void)gi_ltf; (void)ra_i;
}
void halbb_print_rate_2_buff(struct bb_info *bb, u16 rate_idx,
                             enum rtw_gi_ltf gi_ltf, char *buf, u16 buf_size) {
  (void)bb; (void)rate_idx; (void)gi_ltf;
  if (buf && buf_size)
    buf[0] = '\0';
}
u32 halbb_get_fw_ra_rpt(struct bb_info *bb, u16 len, u8 *c2h) {
  (void)bb; (void)len; (void)c2h;
  return 0;
}
void halbb_get_fw_c2h_tx_hist(struct bb_info *bb, u16 len, u8 *c2h) {
  (void)bb; (void)len; (void)c2h;
}
u32 halbb_get_fw_ra_dbgrpt_wifi7(struct bb_info *bb, u16 len, u8 *c2h) {
  (void)bb; (void)len; (void)c2h;
  return 0;
}
u32 halbb_get_txsts_rpt(struct bb_info *bb, u16 len, u8 *c2h) {
  (void)bb; (void)len; (void)c2h;
  return 0;
}

/* FW statistics C2H (halbb_statistics.c fw half, not vendored). */
u32 halbb_get_fw_c2h_statistics(struct bb_info *bb_0, u16 len, u8 *c2h) {
  (void)bb_0; (void)len; (void)c2h;
  return 0;
}

/* Common per-frame report plane (halbb_cmn_rpt.c, not vendored). */
void halbb_cmn_rpt(struct bb_info *bb, struct physts_rxd *desc,
                   u32 physts_bitmap) {
  (void)bb; (void)desc; (void)physts_bitmap;
}
u8 halbb_get_rssi_min(struct bb_info *bb) { (void)bb; return 0; }

/* Antenna / TX-path diversity (halbb_ant_div.c / halbb_path_div.c /
 * halbb_rua_tbl.c, not vendored). */
void halbb_antdiv_phy_sts(struct bb_info *bb, u32 physts_bitmap,
                          struct physts_rxd *desc) {
  (void)bb; (void)physts_bitmap; (void)desc;
}
void halbb_pathdiv_phy_sts(struct bb_info *bb, struct physts_rxd *desc) {
  (void)bb; (void)desc;
}
void halbb_set_tx_path(struct bb_info *bb, u16 macid,
                       enum bb_path tx_path_sel_1ss) {
  (void)bb; (void)macid; (void)tx_path_sel_1ss;
}
u32 halbb_trxpath_notif(struct bb_info *bb, enum rf_path tx_path,
                        enum rf_path rx_path) {
  (void)bb; (void)tx_path; (void)rx_path;
  return 0;
}

/* Dynamic 1R-CCA / DTR watchdog mechanisms (halbb_dyn_1r_cca.c /
 * halbb_dyn_dtr.c, not vendored). */
void halbb_dyn_1r_cca(struct bb_info *bb) { (void)bb; }
void halbb_dyn_1r_cca_rst(struct bb_info *bb) { (void)bb; }
void halbb_dyn_dtr_watchdog(struct bb_info *bb) { (void)bb; }

/* Watchdog pause arbitration (halbb_init.c pause half, not vendored). */
u8 halbb_pause_func(struct bb_info *bb, enum habb_fun_t pause_func,
                    enum halbb_pause_type pause_type,
                    enum halbb_pause_lv_type lv,
                    u8 val_lehgth,
                    u32 *val_buf, enum phl_phy_idx phy_idx) {
  (void)bb; (void)pause_func; (void)pause_type; (void)lv; (void)val_lehgth;
  (void)val_buf; (void)phy_idx;
  return 0;
}

/* TSSI band-edge config (halbb_pwr_ctrl.c, not vendored — same subsystem as
 * the tpu/tssi cr-init no-ops above). */
void halbb_tssi_ctrl_set_bandedge_cfg(struct bb_info *bb,
                                      enum phl_phy_idx phy_idx,
                                      enum tssi_bandedge_cfg bandedge_cfg) {
  (void)bb; (void)phy_idx; (void)bandedge_cfg;
}

/* BE (11BE / WiFi7) primary-sub-band helper referenced by halbb_env_mntr.c's
 * EDCCA-opt path only in its 11BE branch - dead code on the AX Kestrel ICs
 * (halbb_api_be.c is not vendored). A link-only stub, never called at runtime. */
u8 halbb_get_prim_sb(struct bb_info *bb, u8 central_ch, u8 pri_ch,
                     enum channel_width bw) {
  (void)bb; (void)central_ch; (void)pri_ch; (void)bw;
  return 0;
}
