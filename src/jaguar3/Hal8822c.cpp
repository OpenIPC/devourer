#include "Hal8822c.h"

#include <chrono>
#include <thread>
#include <utility>

#include "Jaguar3Common.h"
#include "Hal8822c_PhyTables.h"

namespace jaguar3 {

namespace {
constexpr uint32_t MASKDWORD = 0xFFFFFFFFu;

/* Writer for BB / AGC tables: handles the 0xf9..0xfe pseudo-address delay
 * encoding (see odm_config_bb_phy_8822c) and otherwise does a full-dword BB
 * write. */
void write_bb(RtlUsbAdapter &dev, uint32_t addr, uint32_t data) {
  switch (addr) {
  case 0xfe: std::this_thread::sleep_for(std::chrono::milliseconds(50)); return;
  case 0xfd: std::this_thread::sleep_for(std::chrono::milliseconds(5)); return;
  case 0xfc: std::this_thread::sleep_for(std::chrono::milliseconds(1)); return;
  case 0xfb: std::this_thread::sleep_for(std::chrono::microseconds(50)); return;
  case 0xfa: std::this_thread::sleep_for(std::chrono::microseconds(5)); return;
  case 0xf9: std::this_thread::sleep_for(std::chrono::microseconds(1)); return;
  default: dev.phy_set_bb_reg(static_cast<uint16_t>(addr), MASKDWORD, data);
  }
}
} /* namespace */

Hal8822c::Hal8822c(RtlUsbAdapter device, Logger_t logger)
    : _device{device}, _logger{logger}, _fw{device, logger} {}

void Hal8822c::rtw_hal_init(SelectedChannel /*channel*/) {
  /* Bring-up order (halmac/rtw faithful). power_on() throws until M2, so the
   * steps after it are compiled and wired but not yet reached on hardware. */
  power_on();                        /* M2 — throws (not yet implemented) */
  read_chip_version();               /* M2 — decode cut/vendor/RF -> _phy_ctx */
  init_queue_and_pages();            /* M2 — RQPN/page boundary/auto-LLT */
  _fw.download_default_firmware();   /* M3 — DLFW (bundled NIC image) */
  apply_bb_rf_agc_tables();          /* M4 — BB/AGC via halbb walker */
  /* M4: band + channel (20 MHz) via RadioManagement8822c. */
}

/* Port of rtl8822c_ops.c read_chip_version(). REG_SYS_CFG1_8822C=0x00F0:
 *   [23] RTL_ID (1=test chip), [27] RF_TYPE_ID (1=2T2R), [15:12] CHIP_VER (cut),
 *   [19:16] VENDOR_ID -> >>2 -> 0=TSMC/1=SMIC/2=UMC. */
void Hal8822c::read_chip_version() {
  constexpr uint16_t REG_SYS_CFG1_8822C = 0x00F0;
  uint32_t v = _device.rtw_read32(REG_SYS_CFG1_8822C);

  _ver.test_chip = (v & (1u << 23)) != 0;
  _ver.cut = static_cast<uint8_t>((v >> 12) & 0xf);
  _ver.rf_2t2r = (v & (1u << 27)) ? 1 : 0;
  uint8_t vend = static_cast<uint8_t>(((v >> 16) & 0xf) >> 2);
  _ver.vendor = (vend <= 2) ? vend : 0;

  _phy_ctx.cut_version = _ver.cut; /* feeds the BB/AGC/RF table walker */
  /* _phy_ctx.rfe_type comes from EFUSE — TODO(M2). */

  static const char *vn[] = {"TSMC", "SMIC", "UMC"};
  _logger->info("Jaguar3 chip: 8822C cut={} ({}{}) {} (SYS_CFG1=0x{:08x})",
                _ver.cut, vn[_ver.vendor], _ver.test_chip ? ",test" : "",
                _ver.rf_2t2r ? "2T2R" : "1T1R", v);
}

void Hal8822c::power_on() {
  jaguar3_todo("power-on sequence", Milestone::M2_PowerOnEfuse);
}

void Hal8822c::init_queue_and_pages() {
  jaguar3_todo("queue/page/LLT init", Milestone::M2_PowerOnEfuse);
}

void Hal8822c::apply_bb_rf_agc_tables() {
  /* BB + AGC baseline via the validated halbb walker (src/jaguar3/
   * PhyTableLoader8822c). _phy_ctx must be populated from the chip-version +
   * EFUSE read (TODO M2) before these tables can be trusted on hardware. */
  auto bb = [this](uint32_t addr, uint32_t data) {
    write_bb(_device, addr, data);
  };
  _logger->info("Jaguar3: applying BB phy_reg table ({} words)",
                array_mp_8822c_phy_reg_len);
  PhyTableLoader8822c::Load(array_mp_8822c_phy_reg, array_mp_8822c_phy_reg_len,
                           _phy_ctx, bb);
  _logger->info("Jaguar3: applying AGC table ({} words)",
                array_mp_8822c_agc_tab_len);
  PhyTableLoader8822c::Load(array_mp_8822c_agc_tab, array_mp_8822c_agc_tab_len,
                           _phy_ctx, bb);

  /* RF radio tables (array_mp_8822c_radioa / radiob) need a Jaguar3 RF-register
   * write (odm_set_rf_reg via the BB LSSI path) which is not ported yet. */
  jaguar3_todo("RF radio table application (odm_set_rf_reg)",
               Milestone::M4_RxFirst);
}

} /* namespace jaguar3 */
