/* BB-dbgport reader — exploration framework for the Realtek "Jaguar" PHY
 * debug port.
 *
 * STATUS: single-word peek only. The transport (write u32 selector to
 * 0x8FC, read u32 result from 0xFA0 via libusb vendor control) works and
 * is wrapped here with the canonical save/restore pattern from upstream's
 * only in-tree user (hal/rtl8814a/rtl8814a_phycfg.c:460-545,
 * `phy_ADC_CLK_8814A`). The phydm selector catalogue IS vendored now
 * (reference/<chip>/hal/phydm/phydm_debug.c dbg-port routing +
 * phydm_adc_sampling.c presets, e.g. 0x880/0xa80 = per-path ADC buses,
 * 0x392 = EVM) — but a one-word-at-a-time peek of a streaming bus is
 * still not an IQ capture. The productized consumer of the same dbg-port
 * mux is src/LaCapture.h: LA-mode DMAs the selected bus into the TX
 * packet buffer as a contiguous sample stream — that's the per-tone-CSI
 * path (issue #150). This reader remains the lightweight probe for
 * static/slow BB internals.
 *
 * BRICK RISK
 *   Poking 0x8FC while RX is live can leave demod state machines spinning
 *   (cf. the MAC_Active busy-wait at rtl8814a_phycfg.c:475-479). The
 *   canonical save/restore wrapper used here is the only safe pattern.
 *   Recovery ladder if the chip stops responding to RX after a sweep:
 *     1. libusb_reset_device (set DEVOURER_SKIP_RESET=0 on next launch)
 *     2. USB port-level usbreset (tests/regress.py style)
 *     3. power-cycle / replug
 *   Symptoms are typically RX-stalls with control transfers still alive;
 *   no permanent damage observed in-tree, but treat first runs as
 *   destructive until proven otherwise.
 *
 * Gating: every helper is no-op unless an explicit env var is set in
 * examples/rx/main.cpp. There is no automatic invocation — read+restore happens
 * only when a researcher asks for it.
 */

#ifndef BB_DBGPORT_READER_H
#define BB_DBGPORT_READER_H

#include <cstdint>

#include "RtlAdapter.h"
#include "logger.h"

namespace devourer {

class BbDbgportReader {
 public:
  /* Per Hal8814PhyReg.h:120,178 — bDPort_Sel / rDPdt is the upstream
   * naming. We hard-code the addresses both for cross-chip portability
   * (the pair sits in the same BB window on 8812 / 8821 / 8814) and
   * because the Hal8814PhyReg.h aliases aren't in scope here. */
  static constexpr uint16_t kDbgPortSelectorReg = 0x08FC;
  static constexpr uint16_t kDbgPortReadbackReg = 0x0FA0;

  BbDbgportReader(RtlAdapter& device, Logger_t logger);

  /* Save 0x8FC, write `selector`, read 0xFA0, restore 0x8FC. Returns the
   * 32-bit value read from 0xFA0. Pre-condition: caller has already
   * paused TX or accepted that an in-flight TX may glitch (the upstream
   * A-cut user pauses TX, zeros RXIQC, and waits for MAC_Active to clear;
   * this v1 helper does NONE of that — it is the bare transport only).
   *
   * If is_chip_alive() returns false after the restore, the next call
   * will refuse to write to 0x8FC and return 0 instead. The dead state
   * persists for the lifetime of the BbDbgportReader instance to make
   * the failure mode loud rather than silently corrupting samples. */
  uint32_t read_dbgport(uint32_t selector);

  /* Cheap chip-liveness check: read REG_SYS_CFG and verify the bits we
   * already know to be stable across init. Returns false if the chip's
   * stopped servicing reads (all-zeros or all-ones). Called automatically
   * after every read_dbgport(); can also be called externally. */
  bool is_chip_alive();

  /* Has any prior read_dbgport() detected a dead-chip post-write? Once
   * true, stays true for the lifetime of the instance. */
  bool is_wedged() const { return _wedged; }

 private:
  RtlAdapter& _device;
  Logger_t _logger;
  bool _wedged = false;
};

}  // namespace devourer

#endif  // BB_DBGPORT_READER_H
