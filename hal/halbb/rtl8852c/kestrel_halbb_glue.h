/* C/C++ boundary for the vendored halbb-G6 8852C RX bring-up.
 *
 * The vendored halbb C (compiled behind hal/halbb/hal_headers_le.h) owns the
 * full `struct bb_info` and the RX functions (gpio routing, RX-path enable,
 * per-band gain-error). This header exposes an OPAQUE handle + a tiny API so
 * devourer's C++ (RtlKestrelDevice/HalKestrel) can drive them WITHOUT including
 * any halbb header — the C++ side only sees plain types + the register bridge.
 *
 * Register/OS access is routed back to devourer through kestrel_halbb_bridge
 * (shim/halbb_bridge.h): the caller fills its callbacks (BB window read/write,
 * MAC pwr-reg read/write, delay) pointing at its RtlAdapter, then hands it in.
 */
#ifndef KESTREL_HALBB_GLUE_H
#define KESTREL_HALBB_GLUE_H

#include "shim/halbb_bridge.h"

#ifdef __cplusplus
extern "C" {
#endif

struct kestrel_halbb_ctx; /* opaque; owns a bb_info + hal_com/phl_com */

/* Allocate + populate a bb_info for the C8852C, non-DBCC, HW_PHY_0, RX_PATH_AB.
 * `br` must outlive the ctx (its callbacks are stored). `cut` = chip cut
 * version, `rfe_type` = efuse RFE type (drives the gpio/gain RFE branch). */
struct kestrel_halbb_ctx *kestrel_halbb_create(struct kestrel_halbb_bridge *br,
                                               unsigned char cut,
                                               unsigned char rfe_type);
void kestrel_halbb_destroy(struct kestrel_halbb_ctx *ctx);

/* One-shot RX bring-up: load the built-in gain table into the cache, run the
 * LNAON/TRSW/PAPE RF-front-end gpio routing, then enable the RX chains. */
void kestrel_halbb_rx_bringup(struct kestrel_halbb_ctx *ctx);

/* Per-channel: apply the cached per-band LNA/TIA gain-error to both paths.
 * band_type: 0 = 2.4G (vendor skips gain-error), 1 = 5G, 2 = 6G. */
void kestrel_halbb_set_gain(struct kestrel_halbb_ctx *ctx, unsigned char central_ch,
                            unsigned char band_type);

/* Full vendor per-channel BB config (halbb_ctrl_bw_ch_8852c): gain-error,
 * hidden/normal efuse RX gain, band mode-sel, SCO comp, BW/RXBB/ADC, CCK en.
 * Replaces devourer's hand-rolled BB channel-switch. bw is enum channel_width
 * (20=0,40=1,80=2,5=6,10=7); band_type 0=2.4G,1=5G,2=6G. */
void kestrel_halbb_ctrl_bw_ch(struct kestrel_halbb_ctx *ctx, unsigned char pri_ch,
                              unsigned char central_ch, unsigned char bw,
                              unsigned char band_type);

#ifdef __cplusplus
}
#endif
#endif /* KESTREL_HALBB_GLUE_H */
