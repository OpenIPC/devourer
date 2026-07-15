/* C/C++ boundary for the vendored halbb-G6 baseband plane (both Kestrel
 * chips).
 *
 * The vendored halbb C (compiled behind hal/halbb/hal_headers_le.h) owns the
 * full `struct bb_info` and the BB functions (gpio routing, RX-path enable,
 * per-band gain-error, per-channel BB config). This header exposes an OPAQUE
 * handle + a tiny API so devourer's C++ (RtlKestrelDevice/HalKestrel) can
 * drive them WITHOUT including any halbb header — the C++ side only sees
 * plain types + the register bridge. Chip dispatch is the vendor core's own
 * runtime ic_type switch; the glue only selects the ic_type at create and
 * forks the few entry points that have no generic dispatcher.
 *
 * Register/OS access is routed back to devourer through kestrel_halbb_bridge
 * (shim/halbb_bridge.h): the caller fills its callbacks (BB window read/write,
 * MAC pwr-reg read/write, delay, XTAL-SI) pointing at its RtlAdapter, then
 * hands it in.
 */
#ifndef KESTREL_HALBB_GLUE_H
#define KESTREL_HALBB_GLUE_H

#include "shim/halbb_bridge.h"

#ifdef __cplusplus
extern "C" {
#endif

struct kestrel_halbb_ctx; /* opaque; owns a bb_info + hal_com/phl_com */

/* Allocate + populate a bb_info for `chip`, non-DBCC, HW_PHY_0, RX_PATH_AB.
 * `br` must outlive the ctx (its callbacks are stored; br->bb_info is set to
 * the live bb so the halrf plane's cross-plane calls resolve). `cut` = chip
 * cut version, `rfe_type` = efuse RFE type (drives the gpio/gain RFE branch). */
struct kestrel_halbb_ctx *kestrel_halbb_create(struct kestrel_halbb_bridge *br,
                                               enum kestrel_chip chip,
                                               unsigned char cut,
                                               unsigned char rfe_type);
void kestrel_halbb_destroy(struct kestrel_halbb_ctx *ctx);

/* One-shot RX bring-up: load the built-in gain table into the cache
 * (halbb_cfg_bb_gain_ax_<chip>), run the LNAON/TRSW/PAPE RF-front-end gpio
 * routing (halbb_gpio_setting_init), then enable the RX chains
 * (halbb_ctrl_rx_path). */
void kestrel_halbb_rx_bringup(struct kestrel_halbb_ctx *ctx);

/* Per-channel: apply the cached per-band LNA/TIA gain-error
 * (halbb_set_gain_error_<chip>). band_type: 0 = 2.4G (vendor skips
 * gain-error), 1 = 5G, 2 = 6G. */
void kestrel_halbb_set_gain(struct kestrel_halbb_ctx *ctx, unsigned char central_ch,
                            unsigned char band_type);

/* Full vendor per-channel BB config (halbb_ctrl_bw_ch -> per-chip backend):
 * gain-error, hidden/normal efuse RX gain, band mode-sel, SCO comp, BW/RXBB/
 * ADC, CCK en. bw is enum channel_width (20=0,40=1,80=2,160=3,5=6,10=7);
 * band_type 0=2.4G,1=5G,2=6G. */
void kestrel_halbb_ctrl_bw_ch(struct kestrel_halbb_ctx *ctx, unsigned char pri_ch,
                              unsigned char central_ch, unsigned char bw,
                              unsigned char band_type);

/* T-MAC TX path-com routing (halbb_ctrl_tx_path_tmac_8852c): the 0xD800..
 * 0xD820 path-com block (per-cut, via the MAC pwr-reg plane) + the 0x9a4 path
 * enable + both-path TSSI reset. Connects the BB IFFT output to the TX chain.
 * 8852C-only — the 8852B selects its TX antenna per-STA via the CMAC antenna
 * model, so this is a no-op there. Single-PHY, RF_PATH_A. */
void kestrel_halbb_ctrl_tx_path(struct kestrel_halbb_ctx *ctx);

#ifdef __cplusplus
}
#endif
#endif /* KESTREL_HALBB_GLUE_H */
