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

/* DAC calibration (halrf_dac_cal_8852c). force!=0 re-runs even if done. */
void kestrel_halrf_dac_cal(struct kestrel_halrf_ctx *ctx, unsigned char force);

/* RX DC calibration (halrf_rx_dck_8852c), HW_PHY_0, non-AFE. */
void kestrel_halrf_rx_dck(struct kestrel_halrf_ctx *ctx);

#ifdef __cplusplus
}
#endif
#endif /* KESTREL_HALRF_GLUE_H */
