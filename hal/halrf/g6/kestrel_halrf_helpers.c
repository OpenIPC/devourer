/* Stubs for the deferred halrf TX-power-limit subsystem (halrf_pwr_table.c /
 * halrf_set_pwr_table_8852c.c are not vendored). Registering the 8852C RFK ops
 * (rf_set_ops_8852c via halrf_cmn_info_self_init) makes --gc-sections keep the
 * DPK/TSSI functions, which reference these power-limit lookups — even though
 * only DACK/RX-DCK/IQK are actually driven. Return a benign default so linking
 * succeeds; these paths are not executed. Compiled as C behind the shim. */
#include "halrf_precomp.h"

s8 halrf_get_power_limit(struct rf_info *rf, enum phl_phy_idx phy, u8 path,
                         u16 rate, u8 bw, u8 bf, u8 ph, u8 ch) {
  (void)rf; (void)phy; (void)path; (void)rate; (void)bw; (void)bf; (void)ph;
  (void)ch;
  return 40 * 4; /* ~40 dBm in the s(,2) unit — unused (DPK not run) */
}

/* halrf_init.c's halrf_init/halrf_dm_init reference the debug command parser
 * and dbg-defaults init (halrf_dbg_cmd.c / halrf_dbg.c, not vendored). Neither
 * entry point is called — devourer drives halrf through the glue exports — but
 * linkers that resolve archive members before dead-stripping (MinGW ld, MSVC)
 * still demand the symbols. Link-only no-ops. */
void halrf_cmd_parser_init(struct rf_info *rf) { (void)rf; }
void halrf_dbg_setting_init(struct rf_info *rf) { (void)rf; }

/* ---- Link-only stubs for the un-vendored halrf subsystems ----------------
 *
 * The verbatim vendor TUs reference entry points that live in halrf .c files
 * we deliberately do NOT vendor. No runtime path reaches them (the callers
 * are un-driven vendor entry points kept verbatim), and GNU --gc-sections
 * dead-strips the references on Linux — but MSVC and MinGW ld resolve
 * archive members before dead-stripping, so the symbols must exist at link.
 * Signatures match the vendored header prototypes exactly (this TU includes
 * them via halrf_precomp.h). */

/* Regulatory power tables (halrf_pwr_table.c, not vendored): the vendor's
 * per-regulation power-by-rate / power-limit / power-limit-RU store + apply
 * plane. Devourer's fixed-dBm TXAGC owns TX power instead. */
bool halrf_set_power(struct rf_info *rf, enum phl_phy_idx phy,
                     enum phl_pwr_table pwr_table) {
  (void)rf; (void)phy; (void)pwr_table;
  return false;
}
bool halrf_get_efuse_power_table_switch(struct rf_info *rf,
                                        enum phl_phy_idx phy_idx) {
  (void)rf; (void)phy_idx;
  return false;
}
void halrf_power_by_rate_store_to_array(struct rf_info *rf,
                                        u32 band, u32 tx_num, u32 rate_id,
                                        u32 data) {
  (void)rf; (void)band; (void)tx_num; (void)rate_id; (void)data;
}
void halrf_power_limit_store_to_array(struct rf_info *rf,
                                      u8 regulation, u8 band, u8 bandwidth,
                                      u8 rate, u8 tx_num, u8 beamforming,
                                      u8 chnl, s8 val) {
  (void)rf; (void)regulation; (void)band; (void)bandwidth; (void)rate;
  (void)tx_num; (void)beamforming; (void)chnl; (void)val;
}
void halrf_power_limit_ru_store_to_array(struct rf_info *rf,
                                         u8 band, u8 bandwidth, u8 tx_num,
                                         u8 rate, u8 regulation, u8 chnl,
                                         s8 val) {
  (void)rf; (void)band; (void)bandwidth; (void)tx_num; (void)rate;
  (void)regulation; (void)chnl; (void)val;
}
void halrf_power_limit_set_worldwide(struct rf_info *rf) { (void)rf; }
void halrf_power_limit_ru_set_worldwide(struct rf_info *rf) { (void)rf; }
void halrf_power_limit_set_ext_pwr_limit_table(struct rf_info *rf,
                                               enum phl_phy_idx phy) {
  (void)rf; (void)phy;
}
void halrf_power_limit_set_ext_pwr_limit_ru_table(struct rf_info *rf,
                                                  enum phl_phy_idx phy) {
  (void)rf; (void)phy;
}
void halrf_set_scan_power_table_to_fw_no_6g(struct rf_info *rf) { (void)rf; }
void halrf_set_tpe_control(struct rf_info *rf) { (void)rf; }

/* Per-chip power-table appliers (halrf_set_pwr_table_8852b.c /
 * halrf_set_pwr_table_8852c.c, not vendored) — the MAC-side consumers of the
 * regulatory tables above. */
void halrf_set_fix_power_to_struct_8852b(struct rf_info *rf,
                                         enum phl_phy_idx phy, s8 dbm) {
  (void)rf; (void)phy; (void)dbm;
}
void halrf_set_ref_power_to_struct_8852b(struct rf_info *rf,
                                         enum phl_phy_idx phy) {
  (void)rf; (void)phy;
}
void halrf_set_tx_shape_8852b(struct rf_info *rf, u8 tx_shape_idx) {
  (void)rf; (void)tx_shape_idx;
}
void halrf_pwr_by_rate_info_8852b(struct rf_info *rf, char input[][16],
                                  u32 *_used, char *output, u32 *_out_len) {
  (void)rf; (void)input; (void)_used; (void)output; (void)_out_len;
}
void halrf_pwr_limit_info_8852b(struct rf_info *rf, char input[][16],
                                u32 *_used, char *output, u32 *_out_len) {
  (void)rf; (void)input; (void)_used; (void)output; (void)_out_len;
}
void halrf_pwr_limit_ru_info_8852b(struct rf_info *rf, char input[][16],
                                   u32 *_used, char *output, u32 *_out_len) {
  (void)rf; (void)input; (void)_used; (void)output; (void)_out_len;
}
void halrf_set_fix_power_to_struct_8852c(struct rf_info *rf,
                                         enum phl_phy_idx phy, s8 dbm) {
  (void)rf; (void)phy; (void)dbm;
}
void halrf_set_ref_power_to_struct_8852c(struct rf_info *rf,
                                         enum phl_phy_idx phy) {
  (void)rf; (void)phy;
}
bool halrf_set_power_8852c(struct rf_info *rf, enum phl_phy_idx phy,
                           enum phl_pwr_table pwr_table) {
  (void)rf; (void)phy; (void)pwr_table;
  return false;
}
void halrf_set_tx_shape_8852c(struct rf_info *rf, u8 tx_shape_idx) {
  (void)rf; (void)tx_shape_idx;
}
void halrf_pwr_by_rate_info_8852c(struct rf_info *rf, char input[][16],
                                  u32 *_used, char *output, u32 *_out_len,
                                  enum phl_phy_idx phy) {
  (void)rf; (void)input; (void)_used; (void)output; (void)_out_len; (void)phy;
}
void halrf_pwr_limit_info_8852c(struct rf_info *rf, char input[][16],
                                u32 *_used, char *output, u32 *_out_len,
                                enum phl_phy_idx phy) {
  (void)rf; (void)input; (void)_used; (void)output; (void)_out_len; (void)phy;
}
void halrf_pwr_limit_ru_info_8852c(struct rf_info *rf, char input[][16],
                                   u32 *_used, char *output, u32 *_out_len,
                                   enum phl_phy_idx phy) {
  (void)rf; (void)input; (void)_used; (void)output; (void)_out_len; (void)phy;
}
void halrf_set_bw_power_by_rate_offset_8852c(struct rf_info *rf,
                                             u32 pwr_by_rate_bw_ofst) {
  (void)rf; (void)pwr_by_rate_bw_ofst;
}
s8 halrf_get_pwr_by_rate_bw_control_8852c(struct rf_info *rf,
                                          enum phl_phy_idx phy, u16 rate,
                                          u32 band, s8 pwr_by_rate) {
  (void)rf; (void)phy; (void)rate; (void)band;
  return pwr_by_rate; /* identity — no BW offset applied */
}

/* PSD spectrum measurement (halrf_psd_8852b.c / halrf_psd_8852c.c, not
 * vendored): the vendor MP-tool power-spectral-density capture. */
void halrf_psd_init_8852b(struct rf_info *rf, enum phl_phy_idx phy,
                          u8 path, u8 iq_path, u32 avg, u32 fft) {
  (void)rf; (void)phy; (void)path; (void)iq_path; (void)avg; (void)fft;
}
void halrf_psd_restore_8852b(struct rf_info *rf, enum phl_phy_idx phy) {
  (void)rf; (void)phy;
}
u32 halrf_psd_get_point_data_8852b(struct rf_info *rf, enum phl_phy_idx phy,
                                   s32 point) {
  (void)rf; (void)phy; (void)point;
  return 0;
}
void halrf_psd_query_8852b(struct rf_info *rf, enum phl_phy_idx phy,
                           u32 point, u32 start_point, u32 stop_point,
                           u32 *outbuf) {
  (void)rf; (void)phy; (void)point; (void)start_point; (void)stop_point;
  (void)outbuf;
}
void halrf_psd_init_8852c(struct rf_info *rf, enum phl_phy_idx phy,
                          u8 path, u8 iq_path, u32 avg, u32 fft) {
  (void)rf; (void)phy; (void)path; (void)iq_path; (void)avg; (void)fft;
}
void halrf_psd_restore_8852c(struct rf_info *rf, enum phl_phy_idx phy) {
  (void)rf; (void)phy;
}
u32 halrf_psd_get_point_data_8852c(struct rf_info *rf, enum phl_phy_idx phy,
                                   s32 point) {
  (void)rf; (void)phy; (void)point;
  return 0;
}
void halrf_psd_query_8852c(struct rf_info *rf, enum phl_phy_idx phy,
                           u32 point, u32 start_point, u32 stop_point,
                           u32 *outbuf) {
  (void)rf; (void)phy; (void)point; (void)start_point; (void)stop_point;
  (void)outbuf;
}

/* TAS — time-averaged SAR (halrf_tas.c, not vendored). */
void halrf_tas_pause(struct rf_info *rf, bool is_stop) {
  (void)rf; (void)is_stop;
}
void halrf_tas_set_oft_scan_end(struct rf_info *rf) { (void)rf; }
void halrf_tas_set_oft_to_zero_scan_start(struct rf_info *rf) { (void)rf; }
