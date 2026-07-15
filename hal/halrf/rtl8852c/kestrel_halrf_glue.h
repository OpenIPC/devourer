/* C/C++ boundary for the vendored halrf-G6 8852C RF calibrations.
 *
 * Mirrors hal/halbb/rtl8852c/kestrel_halbb_glue.h: the vendored halrf C owns the
 * full struct rf_info and the RF cal functions; this exposes an opaque handle so
 * devourer's C++ can drive them without including any halrf header. Register/RF
 * access is routed back through the SAME kestrel_halbb_bridge (its read_rf /
 * write_rf callbacks reach devourer's 3-wire RF path).
 */
#ifndef KESTREL_HALRF_GLUE_H
#define KESTREL_HALRF_GLUE_H

#include "../../halbb/rtl8852c/shim/halbb_bridge.h"

#ifdef __cplusplus
extern "C" {
#endif

struct kestrel_halrf_ctx; /* opaque; owns an rf_info + hal_com/phl_com */

struct kestrel_halrf_ctx *kestrel_halrf_create(struct kestrel_halbb_bridge *br,
                                               unsigned char cut,
                                               unsigned char rfe_type);
void kestrel_halrf_destroy(struct kestrel_halrf_ctx *ctx);

/* One-time RFK bring-up: load the 8852C NCTL one-shot micro-engine table
 * (halrf_config_nctl_reg) + reset the RFK sub-state (halrf_rfk_self_init) — the
 * prerequisites halrf_dm_init runs before any per-channel cal. Without the NCTL
 * engine loaded, IQK/DPK's 0xbff8 one-shot-done poll never completes (spins to
 * timeout). Call once after create(), before the per-channel cals. */
void kestrel_halrf_rfk_init(struct kestrel_halrf_ctx *ctx);

/* DAC calibration (halrf_dac_cal_8852c). force!=0 re-runs even if done. */
void kestrel_halrf_dac_cal(struct kestrel_halrf_ctx *ctx, unsigned char force);

/* RX DC calibration (halrf_rx_dck_8852c), HW_PHY_0, non-AFE. */
void kestrel_halrf_rx_dck(struct kestrel_halrf_ctx *ctx);

/* Set the operating channel in rf_info (the per-channel cals — IQK/DPK/TSSI —
 * read hal_com->band[HW_PHY_0].cur_chandef). band 0=2.4G,1=5G,2=6G; bw is
 * enum channel_width. Call before kestrel_halrf_iqk. */
void kestrel_halrf_set_ch(struct kestrel_halrf_ctx *ctx, unsigned char center_ch,
                          unsigned char band, unsigned char bw);

/* IQ imbalance calibration (halrf_iqk), HW_PHY_0, forced. Per-channel. */
void kestrel_halrf_iqk(struct kestrel_halrf_ctx *ctx);

/* TSSI (halrf_do_tssi_8852c), HW_PHY_0, hwtx_en=true. Per-channel TX-power
 * servo setup. Vendor runs it between IQK and DPK. */
void kestrel_halrf_tssi(struct kestrel_halrf_ctx *ctx);

/* Digital pre-distortion (halrf_dpk_8852c), HW_PHY_0, forced. Per-channel;
 * shapes the TX-PA predistortion. Call after IQK (and set_ch). Depends on the
 * NCTL one-shot engine (kestrel_halrf_rfk_init) like IQK. */
void kestrel_halrf_dpk(struct kestrel_halrf_ctx *ctx);

#ifdef __cplusplus
}
#endif
#endif /* KESTREL_HALRF_GLUE_H */
