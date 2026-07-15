/* C/C++ boundary for the vendored halrf-G6 RF calibrations (both Kestrel
 * chips).
 *
 * Mirrors hal/halbb/g6/kestrel_halbb_glue.h: the vendored halrf C owns the
 * full struct rf_info and the RF cal functions; this exposes an opaque handle
 * so devourer's C++ can drive them without including any halrf header. Chip
 * dispatch is the vendor core's own runtime switch (halrf_cmn_info_self_init
 * wires the per-chip rfk ops/backup tables from hal_com->chip_id); the glue
 * forks only the entry points that have no generic dispatcher. Register/RF
 * access is routed back through the SAME kestrel_halbb_bridge (its read_rf /
 * write_rf callbacks reach devourer's 3-wire RF path; read_xsi/write_xsi
 * reach the XTAL-SI plane the 8852B's a-die SI reset needs).
 */
#ifndef KESTREL_HALRF_GLUE_H
#define KESTREL_HALRF_GLUE_H

#include "../../halbb/g6/shim/halbb_bridge.h"

#ifdef __cplusplus
extern "C" {
#endif

struct kestrel_halrf_ctx; /* opaque; owns an rf_info + hal_com/phl_com */

struct kestrel_halrf_ctx *kestrel_halrf_create(struct kestrel_halbb_bridge *br,
                                               enum kestrel_chip chip,
                                               unsigned char cut,
                                               unsigned char rfe_type);
void kestrel_halrf_destroy(struct kestrel_halrf_ctx *ctx);

/* One-time RFK bring-up — the halrf_dm_init cal prologue: load the NCTL
 * one-shot micro-engine table (halrf_config_nctl_reg, per-chip image), reset
 * the RFK sub-state (halrf_rfk_self_init), the HW/SW SI reset (halrf_si_reset
 * — real work on the 8852B via the bridge XSI plane; its dispatch has no
 * 8852C case), then the synth/filter cals (LCK/RCK) the per-channel cals
 * depend on and the efuse trim/TSSI-DE data. Without the NCTL engine loaded,
 * IQK's 0xbff8 one-shot-done poll never completes. Call once after create(),
 * before the per-channel cals. */
void kestrel_halrf_rfk_init(struct kestrel_halrf_ctx *ctx);

/* Parse one logical-efuse field via the vendored per-chip map
 * (halrf_get_efuse_info_<chip>). efuse_map = devourer's parsed logical efuse
 * (>=0x600 bytes); id = enum rtw_efuse_info; autoload!=0 reads efuse (0 =
 * defaults). Writes `size` bytes to value; returns nonzero on success. Wired
 * to the shim's rtw_hal_efuse_get_info via the efuse_get_info bridge callback
 * so the halrf TSSI-DE + thermal-trim efuse reads resolve. */
int kestrel_halrf_efuse_get_info(struct kestrel_halrf_ctx *ctx,
                                 unsigned char *efuse_map, unsigned int id,
                                 void *value, unsigned int size,
                                 unsigned char autoload);

/* DAC calibration (halrf_dac_cal_<chip>). force!=0 re-runs even if done. */
void kestrel_halrf_dac_cal(struct kestrel_halrf_ctx *ctx, unsigned char force);

/* RX DC calibration (halrf_rx_dck_<chip>), HW_PHY_0, non-AFE. */
void kestrel_halrf_rx_dck(struct kestrel_halrf_ctx *ctx);

/* Set the operating channel in rf_info (the per-channel cals — IQK/DPK/TSSI —
 * read hal_com->band[HW_PHY_0].cur_chandef). band 0=2.4G,1=5G,2=6G; bw is
 * enum channel_width. Call before kestrel_halrf_iqk. */
void kestrel_halrf_set_ch(struct kestrel_halrf_ctx *ctx, unsigned char center_ch,
                          unsigned char band, unsigned char bw);

/* Vendored RF channel/band/bw tune. 8852C: halrf_ctl_band_ch_bw_8852c writes
 * RF18 (DAV+DDV, both paths) for band(0=2.4/1=5/2=6G)+central_ch+bw incl. the
 * path-B CAV workaround — follow with kestrel_halrf_lck to relock the synth
 * (6 GHz VCO needs it). 8852B: the core's halrf_ctl_ch + halrf_ctl_bw
 * generics (its synth lock rides inside halrf_ctrl_ch_8852b). */
void kestrel_halrf_ctl_band_ch_bw(struct kestrel_halrf_ctx *ctx,
                                  unsigned char band, unsigned char central_ch,
                                  unsigned char bw);

/* Synth relock (halrf_lck_8852c): sets RF 0x0[mode]=3 + 0x5=0 on both paths,
 * runs the 0xd3-hold set_ch_lck on the current RF18, restores. 8852C-only —
 * the 8852B locks inside its ctrl_ch. */
void kestrel_halrf_lck(struct kestrel_halrf_ctx *ctx);

/* Diagnostic: read a 20-bit RF register (halrf_rrf, MASKRF) on the given path
 * via the vendored RF plane. Used to read the true synth-lock state after a
 * tune — RF 0xb7[8] (LCK-done, 0=locked) and the RF18 readback — which the
 * unreliable BB 0xc5[15] "synthLock" bit does not reflect. */
unsigned int kestrel_halrf_read_rf(struct kestrel_halrf_ctx *ctx,
                                   unsigned char path, unsigned int addr);

/* IQ imbalance calibration (halrf_iqk core dispatcher), HW_PHY_0, forced.
 * Per-channel; depends on the NCTL engine (kestrel_halrf_rfk_init). */
void kestrel_halrf_iqk(struct kestrel_halrf_ctx *ctx);

#ifdef __cplusplus
}
#endif
#endif /* KESTREL_HALRF_GLUE_H */
