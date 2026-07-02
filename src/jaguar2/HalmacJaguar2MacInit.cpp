#include "HalmacJaguar2MacInit.h"

#include <utility>

/* These names are also #define macros in the Realtek hal_com_reg.h pulled in via
 * RtlUsbAdapter.h; #undef them so the scoped constexpr below compile (this .cpp
 * uses no other definition of them). */
#undef REG_RSV_CTRL
#undef REG_RF_CTRL
#undef REG_GPIO_MUXCFG
#undef REG_LED_CFG
#undef REG_PAD_CTRL1
#undef REG_SYS_FUNC_EN
#undef REG_SYS_CFG1
#undef REG_SYS_CFG2
#undef REG_WLRF1
#undef REG_MCUFW_CTRL
#undef REG_ANAPAR_MAC_0
#undef REG_CPU_DMEM_CON
#undef REG_CR_EXT

namespace jaguar2 {

namespace {
/* Common-88xx MAC init registers/constants (same addresses as the Jaguar3
 * set — verified 88xx-common; the halmac init flow is shared). */
constexpr uint16_t REG_RSV_CTRL       = 0x001C;
constexpr uint16_t REG_RF_CTRL        = 0x001F;
constexpr uint16_t REG_GPIO_MUXCFG    = 0x0040;
constexpr uint16_t REG_LED_CFG        = 0x004C;
constexpr uint16_t REG_PAD_CTRL1      = 0x0064;
constexpr uint16_t REG_SYS_FUNC_EN    = 0x0002;
constexpr uint16_t REG_SYS_CFG1       = 0x00F0;
constexpr uint16_t REG_SYS_CFG2       = 0x00FC;
constexpr uint16_t REG_WLRF1          = 0x00EC;
constexpr uint16_t REG_MCUFW_CTRL     = 0x0080;
constexpr uint16_t REG_ANAPAR_MAC_0   = 0x1018;
constexpr uint16_t REG_CPU_DMEM_CON   = 0x1080;
constexpr uint16_t REG_CR_EXT         = 0x1100;

constexpr uint8_t SYS_FUNC_EN_VAL         = 0xD8;
constexpr uint8_t WLAN_PHY_REQ_DELAY      = 0xC;
constexpr uint8_t CHIP_VER_B_CUT          = 1;

bool is_5m(ChannelWidth_t bw) {
  return bw == ChannelWidth_t::CHANNEL_WIDTH_5;
}
bool is_10m(ChannelWidth_t bw) {
  return bw == ChannelWidth_t::CHANNEL_WIDTH_10;
}
} /* namespace */

HalmacJaguar2MacInit::HalmacJaguar2MacInit(RtlUsbAdapter device, Logger_t logger)
    : _device{std::move(device)}, _logger{std::move(logger)} {}

void HalmacJaguar2MacInit::enable_bb_rf(bool enable) {
  if (enable) {
    _device.rtw_write8(REG_SYS_FUNC_EN,
                       _device.rtw_read8(REG_SYS_FUNC_EN) | 0x03);
    _device.rtw_write8(REG_RF_CTRL, _device.rtw_read8(REG_RF_CTRL) | 0x07);
    _device.rtw_write32(REG_WLRF1,
                        _device.rtw_read32(REG_WLRF1) | (0x7u << 24));
  } else {
    _device.rtw_write8(REG_SYS_FUNC_EN,
                       _device.rtw_read8(REG_SYS_FUNC_EN) & ~0x03);
    _device.rtw_write8(REG_RF_CTRL, _device.rtw_read8(REG_RF_CTRL) & ~0x07);
    _device.rtw_write32(REG_WLRF1,
                        _device.rtw_read32(REG_WLRF1) & ~(0x7u << 24));
  }
}

void HalmacJaguar2MacInit::pre_init_system_cfg() {
  _device.rtw_write8(REG_RSV_CTRL, 0);

  /* USB: REG_SYS_CFG2+3 == 0x20 workaround */
  if (_device.rtw_read8(REG_SYS_CFG2 + 3) == 0x20)
    _device.rtw_write8(0xFE5B, _device.rtw_read8(0xFE5B) | (1u << 4));

  /* pinmux */
  uint32_t v = _device.rtw_read32(REG_PAD_CTRL1) & ~((1u << 28) | (1u << 29));
  v |= (1u << 28) | (1u << 29);
  _device.rtw_write32(REG_PAD_CTRL1, v);

  v = _device.rtw_read32(REG_LED_CFG) & ~((1u << 25) | (1u << 26));
  _device.rtw_write32(REG_LED_CFG, v);

  v = _device.rtw_read32(REG_GPIO_MUXCFG) & ~(1u << 2);
  v |= (1u << 2);
  _device.rtw_write32(REG_GPIO_MUXCFG, v);

  enable_bb_rf(false);
  _logger->info("Jaguar2: pre_init_system_cfg done");
}

void HalmacJaguar2MacInit::init_system_cfg(ChannelWidth_t bw, uint8_t cut) {
  uint32_t v = _device.rtw_read32(REG_CPU_DMEM_CON);
  v |= (1u << 16) | (1u << 8); /* BIT_WL_PLATFORM_RST | BIT_DDMA_EN */
  _device.rtw_write32(REG_CPU_DMEM_CON, v);

  _device.rtw_write8(REG_SYS_FUNC_EN + 1,
                     _device.rtw_read8(REG_SYS_FUNC_EN + 1) | SYS_FUNC_EN_VAL);

  /* PHY_REQ_DELAY 0x1100[27:24] by bandwidth (narrowband uses wider delays;
   * 8822B has no narrowband so this is the default path). */
  uint8_t d = _device.rtw_read8(REG_CR_EXT + 3) & 0xF0;
  (void)is_5m;
  (void)is_10m;
  d |= WLAN_PHY_REQ_DELAY;
  _device.rtw_write8(REG_CR_EXT + 3, d);

  /* disable boot-from-flash */
  uint32_t tmp = _device.rtw_read32(REG_MCUFW_CTRL);
  if (tmp & (1u << 20)) { /* BIT_BOOT_FSPI_EN */
    _device.rtw_write32(REG_MCUFW_CTRL, tmp & ~(1u << 20));
    _device.rtw_write32(REG_GPIO_MUXCFG,
                        _device.rtw_read32(REG_GPIO_MUXCFG) & ~(1u << 19));
  }

  if (cut == CHIP_VER_B_CUT)
    _device.rtw_write8(REG_ANAPAR_MAC_0,
                       _device.rtw_read8(REG_ANAPAR_MAC_0) & ~0x07);
  _logger->info("Jaguar2: init_system_cfg done (bw={} cut={})",
                static_cast<int>(bw), cut);
}

} /* namespace jaguar2 */
