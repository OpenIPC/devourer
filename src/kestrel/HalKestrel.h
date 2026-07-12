#ifndef KESTREL_HAL_KESTREL_H
#define KESTREL_HAL_KESTREL_H

#include <array>
#include <cstdint>

#include "ChipVariant.h"
#include "RtlAdapter.h"
#include "logger.h"

namespace kestrel {

/* Parsed logical-efuse contents devourer consumes (see MacRegAx.h for the
 * offsets). Defaults are the vendor "not autoloaded" fallbacks. */
struct EfuseInfo {
  std::array<uint8_t, 6> mac{};
  uint8_t xtal_cap = 0x3F;
  uint8_t rfe_type = 0x01;
  uint8_t thermal_a = 0x22;
  uint8_t thermal_b = 0x22;
  bool autoload_ok = false; /* header parsed to a plausible MAC */
};

/* HalKestrel owns the low-level MAC bring-up for the Kestrel (G6) generation:
 * register-op helpers (poll, xtal_si, efuse), the USB power-on sequence, and
 * the physical→logical efuse dump. Ported from reference/rtl8852bu mac_ax
 * (pwr_seq_func_8852b.c, hw.c, efuse.c, _usb_8852b.c). Milestone M1a. The
 * firmware-download layer (M1b) and TRX init (M2) build on top of this. */
class HalKestrel {
public:
  HalKestrel(RtlAdapter device, Logger_t logger, ChipVariant variant);

  /* USB power-on: usb_pre_init + mac_pwr_on_usb_8852b. Leaves the MAC powered
   * (DMAC/CMAC func-enabled) and ready for firmware download. Returns false on
   * a poll timeout (chip wedged — needs a VBUS cold). 8852C shares the 8852B
   * recipe for M1; a variant divergence hook lands if bring-up proves it. */
  bool power_on();

  /* Dump the physical WiFi efuse (register read loop) and parse it to the
   * logical map, filling `out`. Independent of firmware (driver path only).
   * `raw_phys` (optional) receives the 1536-byte physical map for diagnostics.
   * Must run after power_on (the efuse block is gated by the power sequence). */
  bool read_efuse(EfuseInfo &out, std::array<uint8_t, 1536> *raw_phys = nullptr);

  /* Firmware download (M1b): hci_func_en + DLE/HFC pre-init + FWDL of the
   * cut-appropriate NICCE image. Must run after power_on. `cut` is the chip
   * cut version (from ReadChipInfo / R_AX_SYS_CFG1[15:12]). */
  bool download_firmware(uint8_t cut);

  /* Chip cut version, read fresh from R_AX_SYS_CFG1[15:12]. */
  uint8_t read_cut();

  /* Multi-secure-section key index (__mss_index): reads physical efuse
   * 0x5EC/0x5ED and matches the OTP key tables. Selects which appended
   * signature the secure firmware image uses. 0 for a stock (uncustomized)
   * chip. Used by the FWDL security-section handling. */
  uint8_t read_mss_index();

  ChipVariant variant() const { return _variant; }

private:
  /* Read-modify-write helpers on 32/16/8-bit registers. */
  void set32(uint16_t reg, uint32_t bits);
  void clr32(uint16_t reg, uint32_t bits);
  void field32(uint16_t reg, uint32_t val, uint32_t msk, uint8_t sh);
  /* Poll reg until (read & mask) == expect, PWR_POLL cadence. false = timeout. */
  bool poll32(uint16_t reg, uint32_t mask, uint32_t expect);
  /* XTAL_SI indirect write: reg[bitmask] <- val (masked). false = timeout. */
  bool write_xtal_si(uint8_t offset, uint8_t val, uint8_t bitmask);

  bool usb_pre_init();
  /* Physical efuse read loop (read_hw_efuse, DDV bank) into `phys`. */
  bool read_phys_efuse(uint8_t *phys, uint32_t size);
  void enable_efuse_pwr_cut();
  void disable_efuse_pwr_cut();

  RtlAdapter _device;
  Logger_t _logger;
  ChipVariant _variant;
};

} /* namespace kestrel */

#endif /* KESTREL_HAL_KESTREL_H */
