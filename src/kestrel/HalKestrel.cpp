#include "HalKestrel.h"

#include <algorithm>
#include <cstdlib>
#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <thread>
#include <utility>
#include <vector>

#include "KestrelFw.h"
#include "MacRegAx.h"
#include "PhyTableLoaderKestrel.h"
#include "hal8852b_phy.h"
#if defined(DEVOURER_HAVE_KESTREL_8852C)
#include "hal8852c_phy.h"
#endif

namespace kestrel {

namespace {
namespace r = kestrel::reg;

void delay_us(uint32_t us) {
  std::this_thread::sleep_for(std::chrono::microseconds(us));
}

} /* namespace */

HalKestrel::HalKestrel(RtlAdapter device, Logger_t logger, ChipVariant variant)
    : _device{std::move(device)}, _logger{std::move(logger)},
      _variant{variant}, _fw{_device, _logger, variant} {}

void HalKestrel::set32(uint16_t reg, uint32_t bits) {
  _device.rtw_write32(reg, _device.rtw_read32(reg) | bits);
}

void HalKestrel::clr32(uint16_t reg, uint32_t bits) {
  _device.rtw_write32(reg, _device.rtw_read32(reg) & ~bits);
}

void HalKestrel::field32(uint16_t reg, uint32_t val, uint32_t msk, uint8_t sh) {
  _device.rtw_write32(reg, r::set_clr_word(_device.rtw_read32(reg), val, msk, sh));
}

bool HalKestrel::poll32(uint16_t reg, uint32_t mask, uint32_t expect) {
  for (uint32_t cnt = r::PWR_POLL_CNT; cnt != 0; --cnt) {
    if ((_device.rtw_read32(reg) & mask) == expect)
      return true;
    delay_us(r::PWR_POLL_DLY_US);
  }
  _logger->error("Kestrel: poll timeout on 0x{:04x} (mask 0x{:08x} expect "
                 "0x{:08x})",
                 reg, mask, expect);
  return false;
}

bool HalKestrel::write_xtal_si(uint8_t offset, uint8_t val, uint8_t bitmask) {
  uint32_t w = 0;
  w = r::set_clr_word(w, offset, r::B_AX_WL_XTAL_SI_ADDR_MSK,
                      r::B_AX_WL_XTAL_SI_ADDR_SH);
  w = r::set_clr_word(w, val, r::B_AX_WL_XTAL_SI_DATA_MSK,
                      r::B_AX_WL_XTAL_SI_DATA_SH);
  w = r::set_clr_word(w, bitmask, r::B_AX_WL_XTAL_SI_BITMASK_MSK,
                      r::B_AX_WL_XTAL_SI_BITMASK_SH);
  w = r::set_clr_word(w, r::XTAL_SI_NORMAL_WRITE, r::B_AX_WL_XTAL_SI_MODE_MSK,
                      r::B_AX_WL_XTAL_SI_MODE_SH);
  w |= r::B_AX_WL_XTAL_SI_CMD_POLL;
  _device.rtw_write32(r::R_AX_WLAN_XTAL_SI_CTRL, w);

  for (uint32_t cnt = r::XTAL_SI_POLLING_CNT; cnt != 0; --cnt) {
    if ((_device.rtw_read32(r::R_AX_WLAN_XTAL_SI_CTRL) &
         r::B_AX_WL_XTAL_SI_CMD_POLL) != r::B_AX_WL_XTAL_SI_CMD_POLL)
      return true;
    delay_us(r::XTAL_SI_POLLING_DLY_US);
  }
  _logger->error("Kestrel: XTAL_SI write timeout (offset 0x{:02x})", offset);
  return false;
}

uint8_t HalKestrel::read_xtal_si(uint8_t offset) {
  /* mac_read_xtal_si: post a NORMAL_READ command, poll, read the result from
   * R_AX_WLAN_XTAL_SI_CTRL byte 1. */
  uint32_t w = 0;
  w = r::set_clr_word(w, offset, r::B_AX_WL_XTAL_SI_ADDR_MSK,
                      r::B_AX_WL_XTAL_SI_ADDR_SH);
  w = r::set_clr_word(w, r::XTAL_SI_NORMAL_READ, r::B_AX_WL_XTAL_SI_MODE_MSK,
                      r::B_AX_WL_XTAL_SI_MODE_SH);
  w |= r::B_AX_WL_XTAL_SI_CMD_POLL;
  _device.rtw_write32(r::R_AX_WLAN_XTAL_SI_CTRL, w);
  for (uint32_t cnt = r::XTAL_SI_POLLING_CNT; cnt != 0; --cnt) {
    if ((_device.rtw_read32(r::R_AX_WLAN_XTAL_SI_CTRL) &
         r::B_AX_WL_XTAL_SI_CMD_POLL) != r::B_AX_WL_XTAL_SI_CMD_POLL)
      return _device.rtw_read8(r::R_AX_WLAN_XTAL_SI_CTRL + 1);
    delay_us(r::XTAL_SI_POLLING_DLY_US);
  }
  _logger->error("Kestrel: XTAL_SI read timeout (offset 0x{:02x})", offset);
  return 0;
}

bool HalKestrel::usb_pre_init() {
  /* usb_pre_init_8852{b,c}: enable USB IO mode, clear RX/TX reset, re-toggle the
   * HCI DMA enables. Bulk-out endpoint pause bookkeeping is skipped — the host
   * owns endpoint selection in userspace. The 8852C (RISC-V) addresses the _V1
   * USB register bank (0x5078/0x5174/0x7880) where the 8852B uses
   * 0x1078/0x1174/0x8380; hitting the wrong bank leaves the 8852C USB TRX path
   * in reset so the RISC-V bootrom never raises H2C_PATH_RDY. */
  const bool c = (_variant == ChipVariant::C8852C);
  const uint16_t host_req =
      c ? r::R_AX_USB_HOST_REQUEST_2_V1 : r::R_AX_USB_HOST_REQUEST_2;
  const uint16_t wlan0_1 = c ? r::R_AX_USB_WLAN0_1_V1 : r::R_AX_USB_WLAN0_1;
  const uint16_t hci_en = c ? r::R_AX_HCI_FUNC_EN_V1 : r::R_AX_HCI_FUNC_EN;
  set32(host_req, r::B_AX_R_USBIO_MODE);
  clr32(wlan0_1, r::B_AX_USBRX_RST | r::B_AX_USBTX_RST);
  clr32(hci_en, r::B_AX_HCI_RXDMA_EN | r::B_AX_HCI_TXDMA_EN);
  set32(hci_en, r::B_AX_HCI_RXDMA_EN | r::B_AX_HCI_TXDMA_EN);
  return true;
}

void HalKestrel::usb_init() {
  /* usb_init_8852b (_usb_8852b.c:203, the mac_hal_init intf_init step — runs
   * AFTER trx_init + feat_init): LFPS filter off, RX bulk size for the link
   * speed, and the per-endpoint NUMP (burst count) for EP5,6,7,9,10,11,12. */
  /* The 8852C runtime USB config is the _V1 register bank (usb_init_8852c);
   * same bit layout, different offsets. */
  const bool c = (_variant == ChipVariant::C8852C);
  const uint16_t npi = c ? r::R_AX_USB3_MAC_NPI_CONFIG_INTF_0_V1
                         : r::R_AX_USB3_MAC_NPI_CONFIG_INTF_0;
  const uint16_t status = c ? r::R_AX_USB_STATUS_V1 : r::R_AX_USB_STATUS;
  const uint16_t ep0 = c ? r::R_AX_USB_ENDPOINT_0_V1 : r::R_AX_USB_ENDPOINT_0;
  const uint16_t ep2 = c ? r::R_AX_USB_ENDPOINT_2_V1 : r::R_AX_USB_ENDPOINT_2;
  clr32(npi, r::B_AX_SSPHY_LFPS_FILTER);

  const uint32_t st = _device.rtw_read32(status);
  uint32_t mode;
  if (st & r::B_AX_R_USB2_SEL)
    mode = r::MAC_AX_USB3;
  else if (st & r::B_AX_MODE_HS)
    mode = r::MAC_AX_USB2;
  else
    mode = r::MAC_AX_USB11;
  const uint8_t bulk = mode == r::MAC_AX_USB3   ? r::USB3_BULKSIZE
                       : mode == r::MAC_AX_USB2 ? r::USB2_BULKSIZE
                                                : r::USB11_BULKSIZE;
  _device.rtw_write8(r::R_AX_RXDMA_SETTING, bulk);

  /* Program NUMP for EP5,6,7,9,10,11,12: select the endpoint index in
   * USB_ENDPOINT_0[3:0], then write NUMP to USB_ENDPOINT_2+1. */
  for (uint8_t ep : r::USB_EP_LIST) {
    uint8_t v = _device.rtw_read8(ep0);
    v = static_cast<uint8_t>(r::set_clr_word(v, ep, r::B_AX_EP_IDX_MSK,
                                             r::B_AX_EP_IDX_SH));
    _device.rtw_write8(ep0, v);
    _device.rtw_write8(ep2 + 1, r::USB_NUMP);
  }
  /* NB: the vendor usb_init_8852b does NOT re-kick HCI RXDMA/TXDMA here —
   * that toggle is usb_pre_init's (pre-FWDL). Golden capture: no 0x8380
   * write after FWDL. */
  _logger->info("Kestrel: USB init (mode=USB{}, NUMP set on 7 eps)",
                mode == r::MAC_AX_USB3 ? 3 : mode == r::MAC_AX_USB2 ? 2 : 1);
}

bool HalKestrel::power_on_8852c() {
  namespace r = kestrel::reg;
  /* mac_set_dut_env_mode: WCPU_FW_CTRL FW_ENV[29:28]=0. */
  field32(r::R_AX_WCPU_FW_CTRL, 0, r::B_AX_FW_ENV_MSK, r::B_AX_FW_ENV_SH);

  /* mac_pwr_switch BOOT_MODE-exit preamble (pwr.c:300): the RISC-V 8852C powers
   * up in a special boot mode. Until it is cleared, the WCPU bootrom auto-boots
   * its ROM fw and never enters the FWDL H2C-wait (H2C_PATH_RDY stays low). The
   * 8852B does not set BOOT_MODE, so the guard makes this a no-op there. */
  if (_device.rtw_read32(r::R_AX_GPIO_MUXCFG) & r::B_AX_BOOT_MODE) {
    clr32(r::R_AX_SYS_PW_CTRL, r::B_AX_APFN_ONMAC);
    clr32(r::R_AX_SYS_STATUS1, r::B_AX_AUTO_WLPON);
    clr32(r::R_AX_GPIO_MUXCFG, r::B_AX_BOOT_MODE);
    clr32(r::R_AX_RSV_CTRL, r::B_AX_R_DIS_PRST);
  }

  /* mac_pwr_on_usb_8852c (pwr_seq_func_8852c.c:296), transcribed verbatim. */
  set32(r::R_AX_LDO_AON_CTRL0, r::B_AX_PD_REGU_L);            /* 0x218[16]=1 */
  clr32(r::R_AX_SYS_PW_CTRL,
        r::B_AX_AFSM_WLSUS_EN | r::B_AX_AFSM_PCIE_SUS_EN);    /* 0x04[12:11]=0 */
  set32(r::R_AX_SYS_PW_CTRL, r::B_AX_DIS_WLBT_PDNSUSEN_SOPC); /* 0x04[18]=1 */
  set32(r::R_AX_WLLPS_CTRL, r::B_AX_DIS_WLBT_LPSEN_LOPC);     /* 0x90[1]=1 */
  clr32(r::R_AX_SYS_PW_CTRL, r::B_AX_APDM_HPDN);             /* 0x04[15]=0 */
  clr32(r::R_AX_SYS_PW_CTRL, r::B_AX_APFM_SWLPS);            /* 0x04[10]=0 */
  field32(r::R_AX_SPS_DIG_ON_CTRL0, 0x7, r::B_AX_OCP_L1_MSK,
          r::B_AX_OCP_L1_SH);                                /* 0x200[15:13]=7 */
  if (!poll32(r::R_AX_SYS_PW_CTRL, r::B_AX_RDY_SYSPWR, r::B_AX_RDY_SYSPWR))
    return false;                                            /* 0x04[17]=1 */
  set32(r::R_AX_SYS_PW_CTRL, r::B_AX_EN_WLON);    /* 0x04[16]=1 */
  set32(r::R_AX_SYS_PW_CTRL, r::B_AX_APFN_ONMAC); /* 0x04[8]=1 */
  if (!poll32(r::R_AX_SYS_PW_CTRL, r::B_AX_APFN_ONMAC, 0))
    return false;                                /* 0x04[8]=0 */

  /* reset platform twice then leave enabled: 0x88[0] = 1->0->1->0->1 */
  auto plat = [&](bool en) {
    uint8_t v = _device.rtw_read8(r::R_AX_PLATFORM_ENABLE);
    _device.rtw_write8(r::R_AX_PLATFORM_ENABLE,
                       en ? (v | r::B_AX_PLATFORM_EN)
                          : static_cast<uint8_t>(v & ~r::B_AX_PLATFORM_EN));
  };
  plat(true);
  plat(false);
  plat(true);
  plat(false);
  plat(true);

  /* CMAC1 power off (the 8852C is a 2-CMAC die; devourer runs CMAC0 only). */
  clr32(r::R_AX_SYS_ISO_CTRL_EXTEND, r::B_AX_CMAC1_FEN);          /* 0x80[30]=0 */
  set32(r::R_AX_SYS_ISO_CTRL_EXTEND, r::B_AX_R_SYM_ISO_CMAC12PP); /* 0x80[5]=1 */
  clr32(r::R_AX_AFE_CTRL1, r::B_AX_WLCMAC1_PC_EN_ALL);            /* 0x24[4:0]=0 */

  /* 0x18[6]=1 then XTAL_SI ANAPAR_WL[6]=1 */
  set32(r::R_AX_SYS_ADIE_PAD_PWR_CTRL, r::B_AX_SYM_PADPDN_WL_PTA_1P3);
  if (!write_xtal_si(r::XTAL_SI_ANAPAR_WL, 0x40, 0x40))
    return false;
  /* 0x18[5]=1 then XTAL_SI ANAPAR_WL[5]=1 */
  set32(r::R_AX_SYS_ADIE_PAD_PWR_CTRL, r::B_AX_SYM_PADPDN_WL_RFC_1P3);
  if (!write_xtal_si(r::XTAL_SI_ANAPAR_WL, 0x20, 0x20))
    return false;
  /* ANAPAR_WL [2]=1,[3]=1,[4]=0,[0]=1,[1]=1,[7]=0; XTAL_XMD_2 [6:4]=001
   * (differs from 8852B's 0); XTAL_XMD_4 [3:0]=0. */
  if (!write_xtal_si(r::XTAL_SI_ANAPAR_WL, 0x04, 0x04) ||
      !write_xtal_si(r::XTAL_SI_ANAPAR_WL, 0x08, 0x08) ||
      !write_xtal_si(r::XTAL_SI_ANAPAR_WL, 0x00, 0x10) ||
      !write_xtal_si(r::XTAL_SI_ANAPAR_WL, 0x01, 0x01) ||
      !write_xtal_si(r::XTAL_SI_ANAPAR_WL, 0x02, 0x02) ||
      !write_xtal_si(r::XTAL_SI_ANAPAR_WL, 0x00, 0x80) ||
      !write_xtal_si(r::XTAL_SI_XTAL_XMD_2, r::XTAL_SI_XTAL_XMD_2_8852C, 0x70) ||
      !write_xtal_si(r::XTAL_SI_XTAL_XMD_4, 0x00, 0x0F))
    return false;

  set32(r::R_AX_PMC_DBG_CTRL2, r::B_AX_SYSON_DIS_PMCR_AX_WRMSK); /* 0xCC[2]=1 */
  set32(r::R_AX_SYS_ISO_CTRL, r::B_AX_ISO_EB2CORE);             /* 0x00[8]=1 */
  clr32(r::R_AX_SYS_ISO_CTRL, 1u << 15);                        /* 0x00[15]=0 */
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  clr32(r::R_AX_SYS_ISO_CTRL, 1u << 14);                        /* 0x00[14]=0 */
  clr32(r::R_AX_PMC_DBG_CTRL2, r::B_AX_SYSON_DIS_PMCR_AX_WRMSK); /* 0xCC[2]=0 */

  /* The 8852C pwr-on tail enables the DMAC (0x8400) + CMAC0 (0xC000) + the
   * LED1 pinmux — where 8852B defers func-en to mac_sys_init (the later
   * OR-writes are idempotent). */
  set32(r::R_AX_DMAC_FUNC_EN, 0xFFFF8000u);
  set32(r::R_AX_CMAC_FUNC_EN, r::CMAC_FUNC_EN_BITS);
  field32(r::R_AX_LED1_FUNC_SEL, 0x1, r::B_AX_PINMUX_EESK_FUNC_SEL_V1_MSK,
          r::B_AX_PINMUX_EESK_FUNC_SEL_V1_SH); /* 0x2DC[27:24]=1 */

  _logger->info("Kestrel: MAC powered on (8852C sequence)");
  return true;
}

bool HalKestrel::power_on() {
  if (!_device.is_usb()) {
    _logger->error("Kestrel: only USB power-on is ported (M1)");
    return false;
  }
  if (_variant == ChipVariant::C8852C)
    return power_on_8852c();
  /* mac_set_dut_env_mode (init.c:26, the first thing mac_hal_init does):
   * WCPU_FW_CTRL FW_ENV[29:28] = 0 (normal). Golden capture line 1. */
  field32(r::R_AX_WCPU_FW_CTRL, 0, r::B_AX_FW_ENV_MSK, r::B_AX_FW_ENV_SH);
  /* NB: usb_pre_init (intf_pre_init) runs AFTER power-on + dmac_pre_init in
   * the vendor mac_hal_init (init.c:421) — see download_firmware. */

  /* mac_pwr_on_usb_8852b, transcribed verbatim (pwr_seq_func_8852b.c:419). */
  clr32(r::R_AX_SYS_PW_CTRL,
        r::B_AX_AFSM_WLSUS_EN | r::B_AX_AFSM_PCIE_SUS_EN); /* 0x04[12:11]=0 */
  set32(r::R_AX_SYS_PW_CTRL, r::B_AX_DIS_WLBT_PDNSUSEN_SOPC); /* 0x04[18]=1 */
  set32(r::R_AX_WLLPS_CTRL, r::B_AX_DIS_WLBT_LPSEN_LOPC);     /* 0x90[1]=1 */
  clr32(r::R_AX_SYS_PW_CTRL, r::B_AX_APDM_HPDN);             /* 0x04[15]=0 */
  clr32(r::R_AX_SYS_PW_CTRL, r::B_AX_APFM_SWLPS);            /* 0x04[10]=0 */
  if (!poll32(r::R_AX_SYS_PW_CTRL, r::B_AX_RDY_SYSPWR, r::B_AX_RDY_SYSPWR))
    return false; /* 0x04[17]=1 */

  set32(r::R_AX_AFE_LDO_CTRL, r::B_AX_AON_OFF_PC_EN); /* 0x20[23]=1 */
  if (!poll32(r::R_AX_AFE_LDO_CTRL, r::B_AX_AON_OFF_PC_EN, r::B_AX_AON_OFF_PC_EN))
    return false;

  /* 0x400[1:0]=01, 0x400[5:4]=11 */
  field32(r::R_AX_SPS_DIG_OFF_CTRL0, 0x1, r::B_AX_C1_L1_MSK, r::B_AX_C1_L1_SH);
  field32(r::R_AX_SPS_DIG_OFF_CTRL0, 0x3, r::B_AX_C3_L1_MSK, r::B_AX_C3_L1_SH);

  set32(r::R_AX_SYS_PW_CTRL, r::B_AX_EN_WLON);    /* 0x04[16]=1 */
  set32(r::R_AX_SYS_PW_CTRL, r::B_AX_APFN_ONMAC); /* 0x04[8]=1 */
  if (!poll32(r::R_AX_SYS_PW_CTRL, r::B_AX_APFN_ONMAC, 0))
    return false; /* 0x04[8]=0 */

  /* reset platform twice then leave enabled: 0x88[0] = 1->0->1->0->1 */
  auto plat = [&](bool en) {
    uint8_t v = _device.rtw_read8(r::R_AX_PLATFORM_ENABLE);
    _device.rtw_write8(r::R_AX_PLATFORM_ENABLE,
                       en ? (v | r::B_AX_PLATFORM_EN)
                          : static_cast<uint8_t>(v & ~r::B_AX_PLATFORM_EN));
  };
  plat(true);
  plat(false);
  plat(true);
  plat(false);
  plat(true);

  /* 0x18[6]=1 then XTAL_SI ANAPAR_WL[6]=1 */
  set32(r::R_AX_SYS_ADIE_PAD_PWR_CTRL, r::B_AX_SYM_PADPDN_WL_PTA_1P3);
  if (!write_xtal_si(r::XTAL_SI_ANAPAR_WL, 0x40, 0x40))
    return false;
  /* 0x18[5]=1 then XTAL_SI ANAPAR_WL[5]=1 */
  set32(r::R_AX_SYS_ADIE_PAD_PWR_CTRL, r::B_AX_SYM_PADPDN_WL_RFC_1P3);
  if (!write_xtal_si(r::XTAL_SI_ANAPAR_WL, 0x20, 0x20))
    return false;

  /* DAV-after-CBV path (the runtime CV read is #if 0 in the vendor source,
   * which hard-codes the CBV branch: ANAPAR_WL[2]=1, [3]=1, [0]=1, [1]=1). */
  if (!write_xtal_si(r::XTAL_SI_ANAPAR_WL, 0x04, 0x04) ||
      !write_xtal_si(r::XTAL_SI_ANAPAR_WL, 0x08, 0x08) ||
      !write_xtal_si(r::XTAL_SI_ANAPAR_WL, 0x00, 0x10) || /* [4]=0 */
      !write_xtal_si(r::XTAL_SI_ANAPAR_WL, 0x01, 0x01) ||
      !write_xtal_si(r::XTAL_SI_ANAPAR_WL, 0x02, 0x02) ||
      !write_xtal_si(r::XTAL_SI_ANAPAR_WL, 0x00, 0x80) ||   /* [7]=0 */
      !write_xtal_si(r::XTAL_SI_SRAM_CTRL, 0x00, 0x02) ||   /* 0xA1[1]=0 */
      !write_xtal_si(r::XTAL_SI_XTAL_XMD_2, 0x00, 0x70) ||  /* 8852B: [6:4]=0 */
      !write_xtal_si(r::XTAL_SI_XTAL_XMD_4, 0x00, 0x0F))    /* 0x26[3:0]=0 */
    return false;

  set32(r::R_AX_PMC_DBG_CTRL2, r::B_AX_SYSON_DIS_PMCR_AX_WRMSK); /* 0xCC[2]=1 */
  set32(r::R_AX_SYS_ISO_CTRL, r::B_AX_ISO_EB2CORE);             /* 0x00[8]=1 */
  clr32(r::R_AX_SYS_ISO_CTRL, 1u << 15);                        /* 0x00[15]=0 */
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  clr32(r::R_AX_SYS_ISO_CTRL, 1u << 14);                    /* 0x00[14]=0 */
  clr32(r::R_AX_PMC_DBG_CTRL2, r::B_AX_SYSON_DIS_PMCR_AX_WRMSK); /* 0xCC[2]=0 */

  /* Power-calibration-done check: physical efuse 0x5E9 == 0xAA. If not
   * calibrated, bump SPS_DIG_ON voltage (0x200[3:0]=9, [25:22]=A). Reads one
   * physical byte via the efuse block (power cut wrapped). */
  uint8_t pwr_k = 0;
  {
    enable_efuse_pwr_cut();
    _device.rtw_write32(r::R_AX_EFUSE_CTRL,
                        (r::PWR_K_CHK_OFFSET & r::B_AX_EF_ADDR_MSK)
                            << r::B_AX_EF_ADDR_SH);
    bool ok = false;
    for (uint32_t cnt = r::EFUSE_WAIT_CNT_PLUS; cnt != 0; --cnt) {
      uint32_t v = _device.rtw_read32(r::R_AX_EFUSE_CTRL);
      if (v & r::B_AX_EF_RDY) {
        pwr_k = static_cast<uint8_t>(v & 0xFF);
        ok = true;
        break;
      }
      delay_us(1);
    }
    disable_efuse_pwr_cut();
    if (!ok)
      _logger->warn("Kestrel: pwr-K efuse read timeout (proceeding)");
  }
  if (pwr_k != r::PWR_K_CHK_VALUE) {
    field32(r::R_AX_SPS_DIG_ON_CTRL0, 0x9, r::B_AX_VOL_L1_MSK, r::B_AX_VOL_L1_SH);
    field32(r::R_AX_SPS_DIG_ON_CTRL0, 0xA, r::B_AX_VREFPFM_L_MSK,
            r::B_AX_VREFPFM_L_SH);
  }

  /* Enable DMAC (0x8400) + CMAC (0xC000) function blocks — the pwr-seq tail
   * (pwr_seq_func_8852b.c:369-405): DMAC = bits 15..30 (no CRPRT yet — that
   * is mac_sys_init's dmac_func_en), CMAC = CMAC_EN|TXEN|RXEN|
   * FORCE_CMACREG_GCKEN|PHYINTF|DMA|PTCLTOP|SCHEDULER|TMAC|RMAC =
   * 0x7000803F. Golden capture: 0x8400=0x7fff8000, 0xC000=0x7000803f. */
  constexpr uint16_t R_AX_DMAC_FUNC_EN = 0x8400;
  constexpr uint32_t kDmacEn = 0x7FFF8000u; /* bits 15..30 */
  set32(R_AX_DMAC_FUNC_EN, kDmacEn);
  constexpr uint32_t kCmacEn = 0x7000803Fu;
  set32(r::R_AX_CMAC_FUNC_EN, kCmacEn);

  /* 0x2D8[7:4] = 1 (EESK pinmux). */
  field32(r::R_AX_EECS_EESK_FUNC_SEL, 0x1, r::B_AX_PINMUX_EESK_FUNC_SEL_MSK,
          r::B_AX_PINMUX_EESK_FUNC_SEL_SH);

  _logger->info("Kestrel: MAC powered on (pwr-K=0x{:02x})", pwr_k);
  return true;
}

void HalKestrel::mac_sys_init() {
  /* mac_sys_init (init.c:309): dmac_func_en + cmac_func_en + chip_func_en,
   * run after FWDL + set_enable_bb_rf, before trx_init. All RMW-OR. */
  /* dmac_func_en_8852b (init_8852b.c:1043): the full runtime DMAC enable —
   * MAC_FUNC|DMAC_FUNC|MPDU_PROC|WD_RLS|DLE_WDE|TXPKT_CTRL|STA_SCH|DLE_PLE|
   * PKT_BUF|DMAC_TBL|PKT_IN|DLE_CPUIO|DISPATCHER|BBRPT|MAC_SEC|DMAC_CRPRT|
   * DMACREG_GCKEN = bits 15..31. Golden capture: 0x8400 -> 0xffff8000. */
  set32(r::R_AX_DMAC_FUNC_EN, 0xFFFF8000u);
  /* cmac_func_en(band0, EN) (init.c:219): clock enables on CK_EN first, then
   * the block enables on CMAC_FUNC_EN. Golden: 0xC004 -> 0xffffffff,
   * 0xC000 -> 0xf000803f. */
  set32(r::R_AX_CK_EN, r::CMAC_CK_EN_BITS);
  set32(r::R_AX_CMAC_FUNC_EN, r::CMAC_FUNC_EN_BITS);
  /* chip_func_en -> chip_func_en_8852b: the OCP patch — SPS_DIG_ON OCP_L1
   * field to max (over-current-protection level). */
  set32(r::R_AX_SPS_DIG_ON_CTRL0,
        (r::B_AX_OCP_L1_MSK & 0x7u) << r::B_AX_OCP_L1_SH);
  _logger->info("Kestrel: mac_sys_init (DMAC+CMAC func/clk en, OCP)");
}

void HalKestrel::enable_efuse_pwr_cut() {
  /* enable_efuse_sw_pwr_cut_8852b (DDV, read path — no unlock code). */
  set32(r::R_AX_PMC_DBG_CTRL2, r::B_AX_SYSON_DIS_PMCR_AX_WRMSK);
  uint16_t v = _device.rtw_read16(r::R_AX_SYS_ISO_CTRL);
  _device.rtw_write16(r::R_AX_SYS_ISO_CTRL,
                      v | (1u << r::B_AX_PWC_EV2EF_SH)); /* [14]=1 */
  delay_us(1000);
  v = _device.rtw_read16(r::R_AX_SYS_ISO_CTRL);
  _device.rtw_write16(r::R_AX_SYS_ISO_CTRL,
                      v | (1u << (r::B_AX_PWC_EV2EF_SH + 1))); /* [15]=1 */
  v = _device.rtw_read16(r::R_AX_SYS_ISO_CTRL);
  _device.rtw_write16(r::R_AX_SYS_ISO_CTRL,
                      static_cast<uint16_t>(v & ~r::B_AX_ISO_EB2CORE)); /* [8]=0 */
}

void HalKestrel::disable_efuse_pwr_cut() {
  /* disable_efuse_sw_pwr_cut_8852b (reverse). */
  uint16_t v = _device.rtw_read16(r::R_AX_SYS_ISO_CTRL);
  _device.rtw_write16(r::R_AX_SYS_ISO_CTRL, v | r::B_AX_ISO_EB2CORE); /* [8]=1 */
  v = _device.rtw_read16(r::R_AX_SYS_ISO_CTRL);
  _device.rtw_write16(r::R_AX_SYS_ISO_CTRL,
                      static_cast<uint16_t>(
                          v & ~(1u << (r::B_AX_PWC_EV2EF_SH + 1)))); /* [15]=0 */
  delay_us(1000);
  v = _device.rtw_read16(r::R_AX_SYS_ISO_CTRL);
  _device.rtw_write16(r::R_AX_SYS_ISO_CTRL,
                      static_cast<uint16_t>(
                          v & ~(1u << r::B_AX_PWC_EV2EF_SH))); /* [14]=0 */
  uint8_t v8 = _device.rtw_read8(r::R_AX_PMC_DBG_CTRL2);
  _device.rtw_write8(r::R_AX_PMC_DBG_CTRL2,
                     static_cast<uint8_t>(v8 & ~r::B_AX_SYSON_DIS_PMCR_AX_WRMSK));
}

bool HalKestrel::read_phys_efuse(uint8_t *phys, uint32_t size) {
  /* read_hw_efuse (DDV bank), 8852B EFUSE_WAIT_CNT_PLUS timeout. */
  enable_efuse_pwr_cut();
  for (uint32_t addr = 0; addr < size; ++addr) {
    _device.rtw_write32(r::R_AX_EFUSE_CTRL,
                        (addr & r::B_AX_EF_ADDR_MSK) << r::B_AX_EF_ADDR_SH);
    uint32_t v = 0;
    bool rdy = false;
    for (uint32_t cnt = r::EFUSE_WAIT_CNT_PLUS; cnt != 0; --cnt) {
      v = _device.rtw_read32(r::R_AX_EFUSE_CTRL);
      if (v & r::B_AX_EF_RDY) {
        rdy = true;
        break;
      }
      delay_us(1);
    }
    if (!rdy) {
      _logger->error("Kestrel: efuse read timeout at 0x{:03x}", addr);
      disable_efuse_pwr_cut();
      return false;
    }
    phys[addr] = static_cast<uint8_t>(v & 0xFF);
  }
  disable_efuse_pwr_cut();
  return true;
}

uint32_t HalKestrel::fw_err_state(const char *where) {
  const uint32_t ctrl = _device.rtw_read32(r::R_AX_HALT_C2H_CTRL);
  if (ctrl == 0)
    return 0;
  const uint32_t err = _device.rtw_read32(r::R_AX_HALT_C2H);
  _logger->error("Kestrel FW-SER at [{}]: HALT_C2H_CTRL=0x{:08x} err=0x{:08x} "
                 "(0x2010=L2_AH_HCI 0x1000=L1_DMAC 0x2000=L2_AH_DMA)",
                 where, ctrl, err);
  return err;
}

void HalKestrel::fw_err_settle(const char *where, uint32_t ms) {
  /* Poll HALT_C2H_CTRL every 1 ms for `ms`, doing no other bus traffic, to
   * detect an ASYNC fw self-crash (vs one triggered by a subsequent write). */
  for (uint32_t i = 0; i < ms; ++i) {
    if (_device.rtw_read32(r::R_AX_HALT_C2H_CTRL) != 0) {
      _logger->error("Kestrel FW-SER (async self-crash) at [{}] after {} ms: "
                     "err=0x{:08x}",
                     where, i, _device.rtw_read32(r::R_AX_HALT_C2H));
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  _logger->info("Kestrel: fw stable through [{}] ({} ms, no SER)", where, ms);
}

uint8_t HalKestrel::read_cut() {
  return static_cast<uint8_t>(_device.rtw_read8(r::R_AX_SYS_CFG1 + 1) >> 4);
}

uint8_t HalKestrel::read_mss_index() {
  /* __mss_index: read physical efuse 0x5EC (externalPN) + 0x5ED
   * (customer[3:0], serialNum[6:4]); match the OTP key tables. */
  enable_efuse_pwr_cut();
  auto rd = [&](uint16_t off) -> uint8_t {
    _device.rtw_write32(r::R_AX_EFUSE_CTRL,
                        (off & r::B_AX_EF_ADDR_MSK) << r::B_AX_EF_ADDR_SH);
    for (uint32_t cnt = r::EFUSE_WAIT_CNT_PLUS; cnt != 0; --cnt) {
      uint32_t v = _device.rtw_read32(r::R_AX_EFUSE_CTRL);
      if (v & r::B_AX_EF_RDY)
        return static_cast<uint8_t>(v & 0xFF);
      delay_us(1);
    }
    return 0xFF;
  };
  uint8_t b1 = rd(0x5EC);
  uint8_t b2 = rd(0x5ED);
  disable_efuse_pwr_cut();

  const uint8_t externalPN = static_cast<uint8_t>(0xFF - (b1 & 0xFF));
  const uint8_t customer = static_cast<uint8_t>(0xF - (b2 & 0xF));
  const uint8_t serialNum = static_cast<uint8_t>(0x7 - ((b2 >> 4) & 0x7));
  /* otpkeysinfo.c: externalPN={0,0}, customer={0,1}, serialNum={0,1}. */
  static const uint8_t kPN[2] = {0x0, 0x0};
  static const uint8_t kCust[2] = {0x0, 0x1};
  static const uint8_t kSerial[2] = {0x0, 0x1};
  for (uint8_t i = 0; i < 2; ++i)
    if (externalPN == kPN[i] && customer == kCust[i] && serialNum == kSerial[i])
      return i;
  return 0;
}

bool HalKestrel::download_firmware(uint8_t cut) {
  /* mac_hal_init order (init.c:406-434): hci_func_en -> dmac_pre_init (DLFW
   * DLE/HFC) -> intf_pre_init (usb_pre_init) -> chk_sec_rec -> fwdl. */
  if (!_fw.fw_pre_init())
    return false;
  if (!usb_pre_init())
    return false;
  /* mac_chk_sec_rec (secure_boot.c:61): is_sec_ic from OTP key cell 0x5ED[7].
   * _security_rec = bit7; sec_rec==0 => secure IC (is_sec_ic=1). */
  const bool is_sec_ic = read_sec_rec() == 0;
  _logger->info("Kestrel: sec-rec {} (is_sec_ic={})",
                is_sec_ic ? "secure" : "non-secure", is_sec_ic);
  return _fw.download_firmware(cut, read_mss_index(), is_sec_ic);
}

uint8_t HalKestrel::read_sec_rec() {
  /* OTP_KEY_INFO_CELL_02 (efuse 0x5ED) bit 7 -> _security_rec. */
  enable_efuse_pwr_cut();
  uint8_t byte_val = 0xFF;
  _device.rtw_write32(r::R_AX_EFUSE_CTRL,
                      (r::OTP_KEY_INFO_CELL_02_ADDR & r::B_AX_EF_ADDR_MSK)
                          << r::B_AX_EF_ADDR_SH);
  for (uint32_t cnt = r::EFUSE_WAIT_CNT_PLUS; cnt != 0; --cnt) {
    uint32_t v = _device.rtw_read32(r::R_AX_EFUSE_CTRL);
    if (v & r::B_AX_EF_RDY) {
      byte_val = static_cast<uint8_t>(v & 0xFF);
      break;
    }
    delay_us(1);
  }
  disable_efuse_pwr_cut();
  return static_cast<uint8_t>((byte_val & 0x80) >> 7);
}

bool HalKestrel::chk_dle_rdy(uint16_t status_reg, uint32_t rdy_bits,
                             const char *what) {
  for (uint32_t cnt = r::DLE_WAIT_CNT; cnt != 0; --cnt) {
    if ((_device.rtw_read32(status_reg) & rdy_bits) == rdy_bits)
      return true;
    delay_us(r::DLE_WAIT_US);
  }
  _logger->error("Kestrel TRX: {} DLE not ready (0x{:04x}=0x{:08x})", what,
                 status_reg, _device.rtw_read32(status_reg));
  return false;
}

bool HalKestrel::dle_init_nic() {
  /* dle_init(MAC_AX_QTA_SCC) for 8852B USB: wde_size25 / ple_size33 /
   * wde_qt25 / ple_qt74(min) / ple_qt75(max). Same register recipe as the
   * DLFW DLE (dle_mix_cfg/quota_cfg/func_en) with the NIC quota. */
  auto field = [](uint32_t v, uint32_t val, uint32_t msk, uint8_t sh) {
    return r::set_clr_word(v, val, msk, sh);
  };
  /* 8852C USB3 SCC (NIC) DLE quota — different sizes + a tx_rpt (Q11) reserve
   * the 8852B set omits; without it the fw can't store a TX completion report
   * and injected frames never release (the ~203-frame TX stall). */
  const bool c = (_variant == ChipVariant::C8852C);
  const uint16_t wde_lnk = c ? r::SCC_WDE_LNK_PAGE_8852C : r::SCC_WDE_LNK_PAGE;
  const uint16_t wde_unlnk =
      c ? r::SCC_WDE_UNLNK_PAGE_8852C : r::SCC_WDE_UNLNK_PAGE;
  const uint16_t ple_lnk = c ? r::SCC_PLE_LNK_PAGE_8852C : r::SCC_PLE_LNK_PAGE;
  const uint16_t wde_hif = c ? r::SCC_WDE_QT_HIF_8852C : r::SCC_WDE_QT_HIF;
  const uint16_t wde_wcpu = c ? r::SCC_WDE_QT_WCPU_8852C : r::SCC_WDE_QT_WCPU;
  const uint16_t wde_cpuio = c ? r::SCC_WDE_QT_CPU_IO_8852C : r::SCC_WDE_QT_CPU_IO;
  /* The 8852C has a DEDICATED USB2 DLE quota (dle.c selects dle_mem_usb2_8852c
   * vs dle_mem_usb3_8852c by get_usb_mode). On a USB2 8832CU the USB3 quota
   * caps the RX-DMA PLE pool and wedges RX (burst-then-idle). Detect the live
   * mode the same way usb_init does (USB_STATUS_V1: R_USB2_SEL=SuperSpeed,
   * else MODE_HS=USB2). */
  bool usb2 = false;
  if (c) {
    const uint32_t st = _device.rtw_read32(r::R_AX_USB_STATUS_V1);
    usb2 = !(st & r::B_AX_R_USB2_SEL) && (st & r::B_AX_MODE_HS);
  }
  const uint16_t *ple_min = !c ? r::SCC_PLE_MIN
                          : usb2 ? r::SCC_PLE_MIN_8852C_USB2
                                 : r::SCC_PLE_MIN_8852C;
  const uint16_t *ple_max = !c ? r::SCC_PLE_MAX
                          : usb2 ? r::SCC_PLE_MAX_8852C_USB2
                                 : r::SCC_PLE_MAX_8852C;
  const int ple_qn = c ? 12 : 11; /* 8852C writes through Q11 (tx_rpt) */
  if (c)
    _logger->info("Kestrel DLE: 8852C SCC quota = {} (RX-DMA Q6 max {})",
                  usb2 ? "USB2" : "USB3", usb2 ? 1646 : 178);
  clr32(r::R_AX_DMAC_FUNC_EN, r::B_AX_DLE_WDE_EN | r::B_AX_DLE_PLE_EN);

  /* WDE: 64B page, bound 0, free = lnk_pge_num. */
  uint32_t v = _device.rtw_read32(r::R_AX_WDE_PKTBUF_CFG);
  v = field(v, r::S_AX_WDE_PAGE_SEL_64, r::B_AX_WDE_PAGE_SEL_MSK,
            r::B_AX_WDE_PAGE_SEL_SH);
  v = field(v, 0, r::B_AX_WDE_START_BOUND_MSK, r::B_AX_WDE_START_BOUND_SH);
  v = field(v, wde_lnk, r::B_AX_WDE_FREE_PAGE_NUM_MSK,
            r::B_AX_WDE_FREE_PAGE_NUM_SH);
  _device.rtw_write32(r::R_AX_WDE_PKTBUF_CFG, v);

  /* PLE: 128B page, bound = (wde_lnk+unlnk)*64/8192, free = ple_lnk. */
  const uint32_t bound = (wde_lnk + wde_unlnk) * 64u / r::DLE_BOUND_UNIT;
  v = _device.rtw_read32(r::R_AX_PLE_PKTBUF_CFG);
  v = field(v, r::S_AX_PLE_PAGE_SEL_128, r::B_AX_PLE_PAGE_SEL_MSK,
            r::B_AX_PLE_PAGE_SEL_SH);
  v = field(v, bound, r::B_AX_PLE_START_BOUND_MSK, r::B_AX_PLE_START_BOUND_SH);
  v = field(v, ple_lnk, r::B_AX_PLE_FREE_PAGE_NUM_MSK,
            r::B_AX_PLE_FREE_PAGE_NUM_SH);
  _device.rtw_write32(r::R_AX_PLE_PKTBUF_CFG, v);

  auto qta = [&](uint16_t reg, uint16_t mn, uint16_t mx) {
    _device.rtw_write32(
        reg, (static_cast<uint32_t>(mn & r::QTA_SIZE_MSK) << r::QTA_MIN_SH) |
                 (static_cast<uint32_t>(mx & r::QTA_SIZE_MSK) << r::QTA_MAX_SH));
  };
  /* WDE quota (min==max). */
  qta(r::R_AX_WDE_QTA0_CFG, wde_hif, wde_hif);
  qta(r::R_AX_WDE_QTA1_CFG, wde_wcpu, wde_wcpu);
  qta(r::R_AX_WDE_QTA3_CFG, 0, 0);
  qta(r::R_AX_WDE_QTA4_CFG, wde_cpuio, wde_cpuio);
  /* PLE quota Q0..Q10 (8852B) or Q0..Q11 (8852C, incl. tx_rpt). */
  for (int i = 0; i < ple_qn; ++i)
    qta(static_cast<uint16_t>(r::R_AX_PLE_QTA0_CFG + i * 4), ple_min[i],
        ple_max[i]);

  set32(r::R_AX_DMAC_FUNC_EN, r::B_AX_DLE_WDE_EN | r::B_AX_DLE_PLE_EN);
  if (!chk_dle_rdy(r::R_AX_WDE_INI_STATUS,
                   r::B_AX_WDE_Q_MGN_INI_RDY | r::B_AX_WDE_BUF_MGN_INI_RDY,
                   "WDE"))
    return false;
  if (!chk_dle_rdy(r::R_AX_PLE_INI_STATUS,
                   r::B_AX_PLE_Q_MGN_INI_RDY | r::B_AX_PLE_BUF_MGN_INI_RDY,
                   "PLE"))
    return false;
  return true;
}

bool HalKestrel::sta_sch_init() {
  /* Enable the station scheduler and wait for its warm-init to complete. */
  set32(r::R_AX_SS_CTRL, r::B_AX_SS_EN);
  bool done = false;
  for (uint32_t cnt = r::TRXCFG_WAIT_CNT; cnt != 0; --cnt) {
    if (_device.rtw_read32(r::R_AX_SS_CTRL) & r::B_AX_SS_INIT_DONE_1) {
      done = true;
      break;
    }
    delay_us(r::TRXCFG_WAIT_US);
  }
  if (!done) {
    _logger->error("Kestrel TRX: sta-sch init timeout (SS_CTRL=0x{:08x})",
                   _device.rtw_read32(r::R_AX_SS_CTRL));
    return false;
  }
  set32(r::R_AX_SS_CTRL, r::B_AX_SS_WARM_INIT_FLG);
  /* HW TRX mode: clear the SW-mode SS2FINFO enable. */
  clr32(r::R_AX_SS_CTRL, r::B_AX_SS_NONEMPTY_SS2FINFO_EN);
  return true;
}

bool HalKestrel::hfc_init_nic() {
  /* hfc_init(rst=1, en=1, h2c_en=1) for the USB SCC 8852B quotas
   * (hci_fc.c / hci_fc_8852b.c). The DLFW HFC left only the h2c precharge, so
   * the runtime channel + public page credits must be programmed before any
   * IO-offload H2C or CH12 stalls after one page. */
  auto w = [&](uint16_t reg, uint32_t v) { _device.rtw_write32(reg, v); };
  auto min_max_grp = [](uint16_t mn, uint16_t mx) {
    return ((static_cast<uint32_t>(mn) & r::B_AX_ACH_MIN_PG_MSK)
            << r::B_AX_ACH_MIN_PG_SH) |
           ((static_cast<uint32_t>(mx) & r::B_AX_ACH_MAX_PG_MSK)
            << r::B_AX_ACH_MAX_PG_SH);
    /* grp = grp_0 -> the B_AX_ACH_GRP bit stays clear. */
  };
  /* The 8852C runtime HFC uses the _V1 register bank + its own USB SCC quotas
   * (larger page budget than the 8852B). Same field layout, so only addresses
   * and page counts diverge. */
  const bool c = (_variant == ChipVariant::C8852C);
  const uint16_t fc_ctrl = c ? r::R_AX_HCI_FC_CTRL_V1_NIC : r::R_AX_HCI_FC_CTRL;
  const uint16_t page_ctrl = c ? r::R_AX_CH_PAGE_CTRL_V1 : r::R_AX_CH_PAGE_CTRL;
  const uint16_t ach0 = c ? r::R_AX_ACH0_PAGE_CTRL_V1 : r::R_AX_ACH0_PAGE_CTRL;
  const uint16_t ach1 = c ? r::R_AX_ACH1_PAGE_CTRL_V1 : r::R_AX_ACH1_PAGE_CTRL;
  const uint16_t ach2 = c ? r::R_AX_ACH2_PAGE_CTRL_V1 : r::R_AX_ACH2_PAGE_CTRL;
  const uint16_t ach3 = c ? r::R_AX_ACH3_PAGE_CTRL_V1 : r::R_AX_ACH3_PAGE_CTRL;
  const uint16_t ch8 = c ? r::R_AX_CH8_PAGE_CTRL_V1 : r::R_AX_CH8_PAGE_CTRL;
  const uint16_t ch9 = c ? r::R_AX_CH9_PAGE_CTRL_V1 : r::R_AX_CH9_PAGE_CTRL;
  const uint16_t pub1 = c ? r::R_AX_PUB_PAGE_CTRL1_V1 : r::R_AX_PUB_PAGE_CTRL1;
  const uint16_t pub2 = c ? r::R_AX_PUB_PAGE_CTRL2_V1 : r::R_AX_PUB_PAGE_CTRL2;
  const uint16_t wp1 = c ? r::R_AX_WP_PAGE_CTRL1_V1 : r::R_AX_WP_PAGE_CTRL1;
  const uint16_t wp2 = c ? r::R_AX_WP_PAGE_CTRL2_V1 : r::R_AX_WP_PAGE_CTRL2;
  const uint16_t ch_min = c ? r::HFC_NIC_CH_MIN_8852C : r::HFC_NIC_CH_MIN;
  const uint16_t ch_max = c ? r::HFC_NIC_CH_MAX_8852C : r::HFC_NIC_CH_MAX;
  const uint16_t pub_g0 = c ? r::HFC_NIC_PUB_G0_8852C : r::HFC_NIC_PUB_G0;
  const uint16_t pub_max = c ? r::HFC_NIC_PUB_MAX_8852C : r::HFC_NIC_PUB_MAX;
  const uint16_t wp07 = c ? r::HFC_NIC_WP_CH07_PREC_8852C : r::HFC_NIC_WP_CH07_PREC;
  const uint16_t wp811 =
      c ? r::HFC_NIC_WP_CH811_PREC_8852C : r::HFC_NIC_WP_CH811_PREC;

  /* set_fc_func_en(0, 0): disable FC + CH12 while reprogramming. */
  clr32(fc_ctrl, r::B_AX_HCI_FC_EN | r::B_AX_HCI_FC_CH12_EN);

  /* Per-channel page ctrl: ACH0-3 + B0MGQ(8) + B0HIQ(9) = {min,max,grp0}. */
  const uint32_t ch = min_max_grp(ch_min, ch_max);
  w(ach0, ch);
  w(ach1, ch);
  w(ach2, ch);
  w(ach3, ch);
  w(ch8, ch);
  w(ch9, ch);

  /* Public page ctrl (set_fc_pubpg): group0/group1 + WP threshold. */
  w(pub1,
    ((static_cast<uint32_t>(pub_g0) & r::B_AX_PUBPG_G0_MSK)
     << r::B_AX_PUBPG_G0_SH) |
        ((static_cast<uint32_t>(r::HFC_NIC_PUB_G1) & r::B_AX_PUBPG_G1_MSK)
         << r::B_AX_PUBPG_G1_SH));
  w(wp2, (static_cast<uint32_t>(r::HFC_NIC_WP_THRD) & r::B_AX_WP_THRD_MSK)
             << r::B_AX_WP_THRD_SH);

  /* Mix cfg (set_fc_mix_cfg): precharges + public max + full conditions. */
  w(page_ctrl,
    ((static_cast<uint32_t>(r::HFC_NIC_CH011_PREC) & r::B_AX_PREC_PAGE_CH011_MSK)
     << r::B_AX_PREC_PAGE_CH011_SH) |
        ((static_cast<uint32_t>(r::HFC_USB_H2C_PREC_8852B) &
          r::B_AX_PREC_PAGE_CH12_MSK)
         << r::B_AX_PREC_PAGE_CH12_SH));
  w(pub2, (static_cast<uint32_t>(pub_max) & r::B_AX_PUBPG_ALL_MSK)
              << r::B_AX_PUBPG_ALL_SH);
  w(wp1,
    ((static_cast<uint32_t>(wp07) & r::B_AX_PREC_PAGE_WP_CH07_MSK)
     << r::B_AX_PREC_PAGE_WP_CH07_SH) |
        ((static_cast<uint32_t>(wp811) & r::B_AX_PREC_PAGE_WP_CH811_MSK)
         << r::B_AX_PREC_PAGE_WP_CH811_SH));
  uint32_t fc = _device.rtw_read32(fc_ctrl);
  /* MODE = STF (1) for USB — matches the vendor golden capture
   * (HCI_FC_CTRL = 0x055b: FC_EN=1, MODE=1). An earlier port set MODE=0 +
   * FC_EN=0 on a wrong assumption; the fw never pulled the bulk cmd_ofld H2C
   * off CH12 until HCI flow control was actually enabled. */
  fc = r::set_clr_word(fc, 1 /* MAC_AX_HCIFC_STF */, r::B_AX_HCI_FC_MODE_MSK,
                       r::B_AX_HCI_FC_MODE_SH);
  fc = r::set_clr_word(fc, r::HFC_FULL_COND_X2, r::B_AX_HCI_FC_WD_FULL_COND_MSK,
                       r::B_AX_HCI_FC_WD_FULL_COND_SH);
  fc = r::set_clr_word(fc, r::HFC_FULL_COND_X2,
                       r::B_AX_HCI_FC_CH12_FULL_COND_MSK,
                       r::B_AX_HCI_FC_CH12_FULL_COND_SH);
  fc = r::set_clr_word(fc, r::HFC_FULL_COND_X2,
                       r::B_AX_HCI_FC_WP_CH07_FULL_COND_MSK,
                       r::B_AX_HCI_FC_WP_CH07_FULL_COND_SH);
  fc = r::set_clr_word(fc, r::HFC_FULL_COND_X2,
                       r::B_AX_HCI_FC_WP_CH811_FULL_COND_MSK,
                       r::B_AX_HCI_FC_WP_CH811_FULL_COND_SH);
  w(fc_ctrl, fc);

  /* set_fc_func_en: enable CH12 credits now, but NOT the master FC_EN. The
   * vendor keeps HCI_FC_CTRL FC_EN=0 through probe-time dmac/trx init (golden
   * capture: 0x08040000) and only flips FC_EN=1 at ifup, immediately before
   * its first bulk cmd_ofld (0x055b). Enabling FC_EN mid-dmac-init — while the
   * DLE/USB reconfig is still settling — faults the fw's HCI/DMA AHB monitor
   * (SER err=0x10002010 L2_AH_HCI|L1_DMAC). enable_hci_fc() flips it later. */
  set32(fc_ctrl, r::B_AX_HCI_FC_CH12_EN);
  delay_us(10);
  _logger->info("Kestrel TRX: NIC HFC quotas applied (CH12 credits live)");
  return true;
}

void HalKestrel::mpdu_proc_init() {
  _device.rtw_write32(r::R_AX_ACTION_FWD0, r::TRXCFG_MPDU_PROC_ACT_FRWD);
  _device.rtw_write32(r::R_AX_TF_FWD, r::TRXCFG_MPDU_PROC_TF_FRWD);
  set32(r::R_AX_MPDU_PROC, r::B_AX_APPEND_FCS | r::B_AX_A_ICV_ERR);
  _device.rtw_write32(r::R_AX_CUT_AMSDU_CTRL, r::TRXCFG_MPDU_PROC_CUT_CTRL);
}

void HalKestrel::sec_eng_init() {
  uint32_t v = _device.rtw_read32(r::R_AX_SEC_ENG_CTRL);
  v |= r::B_AX_CLK_EN_CGCMP | r::B_AX_CLK_EN_WAPI | r::B_AX_CLK_EN_WEP_TKIP;
  v |= r::B_AX_SEC_TX_ENC | r::B_AX_SEC_RX_DEC;
  v |= r::B_AX_MC_DEC | r::B_AX_BC_DEC;
  v |= r::B_AX_BMC_MGNT_DEC | r::B_AX_UC_MGNT_DEC;
  v &= ~r::B_AX_TX_PARTIAL_MODE; /* 8852B */
  _device.rtw_write32(r::R_AX_SEC_ENG_CTRL, v);
  /* MIC/ICV append (security_cam.c:1002). Golden: 0x9D04 -> ...|0x3. */
  set32(r::R_AX_SEC_MPDU_PROC, r::B_AX_APPEND_ICV | r::B_AX_APPEND_MIC);
  if (_variant == ChipVariant::C8852C) {
    /* 8852C-only (security_cam.c is_chip_id(8852C)): default RX drv-info block
     * size + the sec-engine TX-timeout selector. */
    field32(r::R_AX_RCR, r::DRVINFO_PATCH_SIZE, r::B_AX_DRV_INFO_SIZE_MSK,
            r::B_AX_DRV_INFO_SIZE_SH);
    field32(r::R_AX_SEC_DEBUG1, r::B_AX_TX_TO, r::B_AX_TX_TIMEOUT_SEL_MSK,
            r::B_AX_TX_TIMEOUT_SEL_SH);
  }
}

bool HalKestrel::trx_dmac_init() {
  /* dmac_init (trxcfg.c:929): dle_init(NIC quota) -> [preload: no-op on
   * 8852B/USB] -> hfc_init -> sta_sch_init -> mpdu_proc_init -> sec_eng_init.
   * The DMAC/CMAC function enables live in mac_sys_init (vendor order);
   * intf_init (usb_init) runs after trx + feat init, not here. */
  if (!dle_init_nic()) /* NIC-mode DLE quota */
    return false;
  if (!hfc_init_nic()) /* runtime page/credit quotas (CH12 IO-offload flows) */
    return false;
  if (!sta_sch_init())
    return false;
  mpdu_proc_init();
  sec_eng_init();
  _logger->info("Kestrel TRX: DMAC init done (NIC DLE + HFC + sta-sch + mpdu "
                "+ sec)");
  return true;
}

void HalKestrel::rx_fltr_init() {
  /* Frame-type filters: forward all mgmt/ctrl/data subtypes to the host
   * (FWD_TO_HOST for every subtype). */
  _device.rtw_write32(r::R_AX_MGNT_FLTR, r::RX_FLTR_ALL_TO_HOST);
  _device.rtw_write32(r::R_AX_CTRL_FLTR, r::RX_FLTR_ALL_TO_HOST);
  _device.rtw_write32(r::R_AX_DATA_FLTR, r::RX_FLTR_ALL_TO_HOST);
  /* rx_fltr_init_opt (default sniffer profile): sniffer mode + accept A1
   * match / broadcast / multicast + uc/bc CAM match. The PLCP-check bits go
   * to a separate register (R_AX_PLCP_HDR_FLTR) and default enabled — left as
   * hardware reset. rmac_init later ORs in the RX max-MPDU-len field. */
  uint32_t opt = r::B_AX_SNIFFER_MODE | r::B_AX_A_A1_MATCH | r::B_AX_A_BC |
                 r::B_AX_A_MC | r::B_AX_A_UC_CAM_MATCH | r::B_AX_A_BC_CAM_MATCH;
  _device.rtw_write32(r::R_AX_RX_FLTR_OPT, opt);
}

void HalKestrel::rmac_init() {
  /* Response-BA CAM: select SSN source. */
  _device.rtw_write8(
      r::R_AX_RESPBA_CAM_CTRL,
      static_cast<uint8_t>(_device.rtw_read8(r::R_AX_RESPBA_CAM_CTRL) |
                           r::B_AX_SSN_SEL));
  /* Deadlock-protection RX timeouts + reset enable. */
  uint16_t dlk = _device.rtw_read16(r::R_AX_DLK_PROTECT_CTL);
  dlk = static_cast<uint16_t>(r::set_clr_word(dlk, r::TRXCFG_RMAC_DATA_TO,
                                              r::B_AX_RX_DLK_DATA_TIME_MSK,
                                              r::B_AX_RX_DLK_DATA_TIME_SH));
  dlk = static_cast<uint16_t>(r::set_clr_word(dlk, r::TRXCFG_RMAC_CCA_TO,
                                              r::B_AX_RX_DLK_CCA_TIME_MSK,
                                              r::B_AX_RX_DLK_CCA_TIME_SH));
  dlk |= r::B_AX_RX_DLK_RST_EN;
  _device.rtw_write16(r::R_AX_DLK_PROTECT_CTL, dlk);
  /* Receiver channel-enable (the key RX gate). The 8852C uses B_AX_CH_EN_V1
   * ([7:0]) and — as a single-CMAC part in normal RX mode — enables only
   * channel 0 (0x1, matching the working vendor's live RCR); the 8852B uses
   * B_AX_CH_EN ([3:0]) = 0xF. */
  if (_variant == ChipVariant::C8852C)
    _device.rtw_write8(
        r::R_AX_RCR,
        static_cast<uint8_t>(r::set_clr_word(_device.rtw_read8(r::R_AX_RCR), 0x1,
                                             r::B_AX_CH_EN_V1_MSK,
                                             r::B_AX_CH_EN_V1_SH)));
  else
    _device.rtw_write8(
        r::R_AX_RCR,
        static_cast<uint8_t>(r::set_clr_word(_device.rtw_read8(r::R_AX_RCR), 0xF,
                                             r::B_AX_CH_EN_MSK, r::B_AX_CH_EN_SH)));
  /* RX interface timeout threshold. */
  _device.rtw_write32(r::R_AX_RX_TIME_MON,
                      r::set_clr_word(_device.rtw_read32(r::R_AX_RX_TIME_MON),
                                      0x30, r::B_AX_INTF_TIMEOUT_THR_MSK,
                                      r::B_AX_INTF_TIMEOUT_THR_SH));
  /* RX max MPDU length (field of R_AX_RX_FLTR_OPT). */
  _device.rtw_write32(
      r::R_AX_RX_FLTR_OPT,
      r::set_clr_word(_device.rtw_read32(r::R_AX_RX_FLTR_OPT),
                      r::RMAC_RX_MPDU_MAX_LEN, r::B_AX_RX_MPDU_MAX_LEN_MSK,
                      r::B_AX_RX_MPDU_MAX_LEN_SH));
  /* Not all vendors compute VHT SIG-B's CRC — clear the check (rmac_init tail). */
  _device.rtw_write8(
      r::R_AX_PLCP_HDR_FLTR,
      static_cast<uint8_t>(_device.rtw_read8(r::R_AX_PLCP_HDR_FLTR) &
                           ~r::B_AX_VHT_SU_SIGB_CRC_CHK));
}

void HalKestrel::cca_ctrl_init() {
  uint32_t v = _device.rtw_read32(r::R_AX_CCA_CONTROL);
  v |= r::CCA_CTRL_SET;
  v &= ~r::CCA_CTRL_CLR;
  _device.rtw_write32(r::R_AX_CCA_CONTROL, v);
}

void HalKestrel::cmac_com_init() {
  /* TX subcarrier value = 0 (non-loopback) for 20/40/80. */
  uint32_t v = _device.rtw_read32(r::R_AX_TX_SUB_CARRIER_VALUE);
  v = r::set_clr_word(v, 0, r::B_AX_TXSC_MSK, r::B_AX_TXSC_20M_SH);
  v = r::set_clr_word(v, 0, r::B_AX_TXSC_MSK, r::B_AX_TXSC_40M_SH);
  v = r::set_clr_word(v, 0, r::B_AX_TXSC_MSK, r::B_AX_TXSC_80M_SH);
  _device.rtw_write32(r::R_AX_TX_SUB_CARRIER_VALUE, v);
  /* Response-rate-selection: enable OFDM + CCK. */
  _device.rtw_write32(
      r::R_AX_PTCL_RRSR1,
      r::set_clr_word(_device.rtw_read32(r::R_AX_PTCL_RRSR1),
                      r::RRSR_OFDM_CCK_EN, r::B_AX_RRSR_RATE_EN_MSK,
                      r::B_AX_RRSR_RATE_EN_SH));
}

void HalKestrel::cmac_dma_init() {
  /* cmac_dma_init (trxcfg.c:762): clear the RX ptr-full-mode bits so RX DMA
   * streams. This is an is_chip_id(8852A/8852B/8851B/8852BT) branch — 0xC804 is
   * the 8852B RXDMA_CTRL_0 (RU/CSI ptr-full bits [5:0]). On the 8852C the vendor
   * early-returns (touches nothing): 0xC804 is a different _V1 register there,
   * and the RX-data full-mode/reserve-depth lives at R_AX_RX_CTRL0 (0xC808) and
   * must stay at reset. Writing 0xC804 on the 8852C perturbs the RXDMA full/
   * reserve logic -> RX streams an init burst then wedges (C2H, on a separate
   * PLE queue, keeps flowing). So skip it entirely for the 8852C. */
  if (_variant == ChipVariant::C8852C)
    return; /* vendor early-returns on 8852C (0xC804 is a _V1 register there) */
  clr32(r::R_AX_RXDMA_CTRL_0, r::RX_FULL_MODE);
}

/* halbb_gain_band_determine_8852b: map a central channel to the gain-cache band
 * index. 8852B is 2.4/5 GHz only (no 6G), so 4 bands suffice. */
uint8_t HalKestrel::gain_band_determine(uint8_t channel) {
  if (channel <= 14)
    return 0;                    /* 2.4 GHz */
  if (channel >= 36 && channel <= 64)
    return 1;                    /* 5G Low */
  if (channel >= 100 && channel <= 144)
    return 2;                    /* 5G Mid */
  if (channel >= 149 && channel <= 177)
    return 3;                    /* 5G High */
  return 1;                      /* default 5G Low (matches vendor) */
}

/* halbb_init_gain_table -> halbb_cfg_bb_gain_ax_8852b -> halbb_cfg_bb_gain_8852b.
 * Walk array_mp_8852b_phy_reg_gain with the shared conditional-table walker and,
 * for each matched entry, decode (cfg_type,band,path,type) from the pseudo-addr
 * and store the gain bytes into the software cache. This writes ZERO hardware
 * registers (the addr field is NOT a register address) — the cache is applied
 * later per channel by set_gain_error. */
void HalKestrel::init_gain_table(uint32_t rfe_type, uint32_t cut) {
  auto decode = [&](uint32_t addr, uint32_t data) {
    /* 0xf9..0xfe are host-delay pseudo-addresses in the vendor walker — no
     * cache effect. */
    if (addr >= 0xf9 && addr <= 0xfe)
      return;
    const uint8_t cfg_type = (addr >> 24) & 0xff;
    const uint8_t band = (addr >> 16) & 0xff;
    const uint8_t path = (addr >> 8) & 0xff;
    const uint8_t type = addr & 0xff;
    if (band >= kGainBandNum || path >= kGainPathNum)
      return;
    if (cfg_type == 1) {
      /* RPL offset: the low byte encodes bw[7:4] | rxsc_start[3:0]. */
      cfg_rpl_ofst(band, path, (type & 0xf0) >> 4, type & 0xf, data);
      return;
    }
    if (cfg_type != 0) /* only 0 (gain error) + 1 (RPL) are used. */
      return;
    if (type == 0) {
      for (int i = 0; i < 4; i++)
        _lna_gain[band][path][i] = static_cast<int8_t>((data >> (8 * i)) & 0xff);
    } else if (type == 1) {
      for (int i = 0; i < 3; i++)
        _lna_gain[band][path][4 + i] =
            static_cast<int8_t>((data >> (8 * i)) & 0xff);
    } else if (type == 2) {
      for (int i = 0; i < 2; i++)
        _tia_gain[band][path][i] =
            static_cast<int8_t>((data >> (8 * i)) & 0xff);
    }
  };
  const uint32_t *gain = array_mp_8852b_phy_reg_gain;
  uint32_t gain_len = array_mp_8852b_phy_reg_gain_len;
#if defined(DEVOURER_HAVE_KESTREL_8852C)
  if (_variant == ChipVariant::C8852C) {
    gain = array_mp_8852c_phy_reg_gain;
    gain_len = array_mp_8852c_phy_reg_gain_len;
  }
#endif
  apply_phy_table(gain, gain_len, rfe_type, cut, decode);
  _gain_cached = true;
}

/* halbb_set_gain_error_8852b: write the cached per-band LNA/TIA gain into the
 * band-specific BB registers (g-regs for 2.4 GHz, a-regs for 5 GHz). Called on
 * every channel set — this is the RX front-end gain the 5 GHz path needs. */
void HalKestrel::set_gain_error(uint8_t channel) {
  if (!_gain_cached)
    return;
  const uint8_t band = gain_band_determine(channel);
  const bool is_2g = (band == 0);
  /* Register maps (path A/B x 7 LNA / 2 TIA), verbatim from the vendor. */
  static const uint32_t lna_g[2][7] = {
      {0x4678, 0x4678, 0x467C, 0x467C, 0x467C, 0x467C, 0x4680},
      {0x475C, 0x475C, 0x4760, 0x4760, 0x4760, 0x4760, 0x4764}};
  static const uint32_t lna_a[2][7] = {
      {0x45DC, 0x45DC, 0x4660, 0x4660, 0x4660, 0x4660, 0x4664},
      {0x4740, 0x4740, 0x4744, 0x4744, 0x4744, 0x4744, 0x4748}};
  static const uint32_t lna_mask[7] = {0x00ff0000, 0xff000000, 0x000000ff,
                                       0x0000ff00, 0x00ff0000, 0xff000000,
                                       0x000000ff};
  static const uint32_t tia_g[2][2] = {{0x4680, 0x4680}, {0x4764, 0x4764}};
  static const uint32_t tia_a[2][2] = {{0x4664, 0x4664}, {0x4748, 0x4748}};
  static const uint32_t tia_mask[2] = {0x00ff0000, 0xff000000};
  for (int path = 0; path < kGainPathNum; path++) {
    for (int i = 0; i < 7; i++) {
      const uint32_t reg = is_2g ? lna_g[path][i] : lna_a[path][i];
      bb_rmw(reg, lna_mask[i],
             static_cast<uint32_t>(_lna_gain[band][path][i]) & 0xff);
    }
    for (int i = 0; i < 2; i++) {
      const uint32_t reg = is_2g ? tia_g[path][i] : tia_a[path][i];
      bb_rmw(reg, tia_mask[i],
             static_cast<uint32_t>(_tia_gain[band][path][i]) & 0xff);
    }
  }
}

void HalKestrel::cfg_rpl_ofst(uint8_t band, uint8_t path, uint8_t bw,
                              uint8_t rxsc_start, uint32_t data) {
  /* halbb_cfg_bb_rpl_ofst (8852B: 20/40/80 MHz). RXSC start indices: FULL=0,
   * 20=1, 40=9. 40 MHz packs 2 sub-channels, 80 MHz packs 4 (rxsc 20) or 2
   * (rxsc 40). */
  auto s8 = [&](int i) { return static_cast<int8_t>((data >> (8 * i)) & 0xff); };
  if (bw == 0) { /* 20 MHz */
    _rpl_ofst_20[band][path] = s8(0);
  } else if (bw == 1) { /* 40 MHz */
    if (rxsc_start == 0)
      _rpl_ofst_40[band][path][0] = s8(0);
    else if (rxsc_start == 1)
      for (int i = 0; i < 2; i++)
        _rpl_ofst_40[band][path][1 + i] = s8(i);
  } else if (bw == 2) { /* 80 MHz */
    if (rxsc_start == 0)
      _rpl_ofst_80[band][path][0] = s8(0);
    else if (rxsc_start == 1)
      for (int i = 0; i < 4; i++)
        _rpl_ofst_80[band][path][1 + i] = s8(i);
    else if (rxsc_start == 9)
      for (int i = 0; i < 2; i++)
        _rpl_ofst_80[band][path][9 + i] = s8(i);
  }
}

void HalKestrel::set_rxsc_rpl_comp(uint8_t channel) {
  /* halbb_set_rxsc_rpl_comp_8852b: average the two paths' RPL offsets per
   * BW/RXSC, pack into 3 words, write BB 0x49b0/b4/b8 (+ 0x4a00/04/08 mirror).
   * Feeds the BB RX RSSI/RPL computation. */
  if (!_gain_cached)
    return;
  const uint8_t b = gain_band_determine(channel);
  auto avg2 = [](int8_t a, int8_t c) -> uint32_t {
    return static_cast<uint32_t>(static_cast<int8_t>((a + c) >> 1)) & 0xff;
  };
  const uint32_t rpl20 = avg2(_rpl_ofst_20[b][0], _rpl_ofst_20[b][1]);
  auto a40 = [&](int i) { return avg2(_rpl_ofst_40[b][0][i], _rpl_ofst_40[b][1][i]); };
  auto a80 = [&](int i) { return avg2(_rpl_ofst_80[b][0][i], _rpl_ofst_80[b][1][i]); };
  const uint32_t v1 = (rpl20 << 8) | (a40(0) << 16) | (a40(1) << 24);
  const uint32_t v2 =
      a40(2) | (a80(0) << 8) | (a80(1) << 16) | (a80(10) << 24);
  const uint32_t v3 =
      a80(2) | (a80(3) << 8) | (a80(4) << 16) | (a80(9) << 24);
  bb_rmw(0x49b0, 0xffffff00, v1 >> 8);
  bb_rmw(0x4a00, 0xffffff00, v1 >> 8);
  bb_rmw(0x49b4, r::MASKDWORD, v2);
  bb_rmw(0x4a04, r::MASKDWORD, v2);
  bb_rmw(0x49b8, r::MASKDWORD, v3);
  bb_rmw(0x4a08, r::MASKDWORD, v3);
}

void HalKestrel::usb_rx_agg_cfg() {
  /* rx_agg_cfg_usb_8852b (MAC_AX_RX_AGG_MODE_USB): the RXAGG engine is what
   * DMAs RX frames onto the USB bulk-IN, so it MUST be enabled — USB mode is
   * agg_en=1 with DMA_STORE=0 and the default size/timeout thresholds. (The
   * old code cleared bit 0, which is LEN_TH[0], NOT the enable bit (BIT31) —
   * so RXAGG stayed off-by-default and no RX reached the host bulk-IN.) The
   * 11ax rxd parser already walks multi-frame aggregates via next_offset. */
  const bool c8852c = (_variant == ChipVariant::C8852C);
  const uint16_t rxagg0 = c8852c ? r::R_AX_RXAGG_0_V1 : r::R_AX_RXAGG_0;
  /* pkt_num field: the vendor writes the field MSK (0xff) when the driver
   * default is 0 (usb_rx_agg_cfg_8852c) — golden 8852C value 0x80ff2010. The
   * 8852B path keeps 0 (unchanged, on-air-validated). */
  const uint32_t pktnum = c8852c ? r::B_AX_RXAGG_PKTNUM_TH_MSK : 0u;
  const uint32_t len_th =
      c8852c ? static_cast<uint32_t>(r::RXAGGSIZE) * r::COMPAT_RX_AGG_UNIT
             : static_cast<uint32_t>(r::RXAGGSIZE);
  uint32_t v = _device.rtw_read32(rxagg0);
  v = (v & r::B_AX_RXAGG_SW_EN) | r::B_AX_RXAGG_EN |
      (len_th << r::B_AX_RXAGG_LEN_TH_SH) |
      (static_cast<uint32_t>(r::RXAGGTO) << r::B_AX_RXAGG_TIMEOUT_TH_SH) |
      (pktnum << r::B_AX_RXAGG_PKTNUM_TH_SH);
  _device.rtw_write32(rxagg0, v);
  if (c8852c) /* neutralise the small-packet aggregation (hw fn unused). */
    _device.rtw_write32(r::R_AX_RXAGG_1_V1, r::RXAGG_1_V1_DISABLE_SML);
}

void HalKestrel::scheduler_init() {
  /* scheduler_init (trxcfg.c) — band0. SIFS_MACTXEN_T1 is V2 (0x3E) on the
   * 8852C vs V0 (0x47) on the 8852B; the 8852C does NOT write
   * B_AX_PORT_RST_TSF_ADV (8852B/51B/BT-only branch); SIFS_AGGR uses the wider
   * _V1 [31:24] field on the 8852C. */
  const bool c = (_variant == ChipVariant::C8852C);
  field32(r::R_AX_PREBKF_CFG_1, c ? r::SIFS_MACTXEN_T1_V2 : r::SIFS_MACTXEN_T1_V0,
          r::B_AX_SIFS_MACTXEN_T1_MSK, r::B_AX_SIFS_MACTXEN_T1_SH);
  if (!c)
    set32(r::R_AX_SCH_EXT_CTRL, r::B_AX_PORT_RST_TSF_ADV);
  clr32(r::R_AX_CCA_CFG_0, r::B_AX_BTCCA_EN);
  /* ASIC env leaves CCA_CFG_0's CCA/EDCCA bits as-is. AP-path pre-backoff +
   * SIFS-aggregation timing (band0). */
  field32(r::R_AX_PREBKF_CFG_0, r::SCH_PREBKF_16US, r::B_AX_PREBKF_TIME_MSK,
          r::B_AX_PREBKF_TIME_SH);
  field32(r::R_AX_CCA_CFG_0, 0x6a,
          c ? r::B_AX_R_SIFS_AGGR_TIME_V1_MSK : r::B_AX_R_SIFS_AGGR_TIME_MSK,
          c ? r::B_AX_R_SIFS_AGGR_TIME_V1_SH : r::B_AX_R_SIFS_AGGR_TIME_SH);
  /* Beacon-queue EDCA (path BCN): W16 at BCNQ_PARAM+2 = CW|AIFS. */
  const uint8_t cw = static_cast<uint8_t>((3u << 4) | 2u); /* ecw_max/min */
  uint16_t edca = static_cast<uint16_t>(
      ((cw & r::B_AX_BE_0_CW_MSK) << r::B_AX_BE_0_CW_SH) |
      ((r::BCN_IFS_25US & r::B_AX_BE_0_AIFS_MSK) << r::B_AX_BE_0_AIFS_SH));
  _device.rtw_write16(r::R_AX_EDCA_BCNQ_PARAM + 2, edca);
}

bool HalKestrel::addr_cam_init() {
  /* addr_cam_init (addr_cam.c) — set search range + enable, then reset (band0)
   * and poll the CLR bit. */
  uint32_t v = _device.rtw_read32(r::R_AX_ADDR_CAM_CTRL);
  v |= (r::ADDR_CAM_SERCH_RANGE & r::B_AX_ADDR_CAM_RANGE_MSK)
       << r::B_AX_ADDR_CAM_RANGE_SH;
  v |= r::B_AX_ADDR_CAM_EN | r::B_AX_ADDR_CAM_CLR;
  _device.rtw_write32(r::R_AX_ADDR_CAM_CTRL, v);
  for (uint32_t cnt = r::TRXCFG_WAIT_CNT; cnt != 0; --cnt) {
    if (!(_device.rtw_read16(r::R_AX_ADDR_CAM_CTRL) & r::B_AX_ADDR_CAM_CLR))
      return true;
    delay_us(r::TRXCFG_WAIT_US);
  }
  _logger->error("Kestrel CMAC: ADDR_CAM reset timeout");
  return false;
}

void HalKestrel::nav_ctrl_init() {
  /* nav_ctrl_init (mac_two_nav_cfg): plcp/tgr update-nav enable, upper=25ms. */
  uint32_t v = _device.rtw_read32(r::R_AX_WMAC_NAV_CTL);
  v |= r::B_AX_WMAC_PLCP_UP_NAV_EN | r::B_AX_WMAC_TF_UP_NAV_EN;
  v = r::set_clr_word(v, r::NAV_25MS, r::B_AX_WMAC_NAV_UPPER_MSK,
                      r::B_AX_WMAC_NAV_UPPER_SH);
  if (r::NAV_25MS > r::NAV_UPPER_DEFAULT)
    v |= r::B_AX_WMAC_NAV_UPPER_EN;
  _device.rtw_write32(r::R_AX_WMAC_NAV_CTL, v);
  clr32(r::R_AX_SPECIAL_TX_SETTING, r::B_AX_BMC_NAV_PROTECT); /* bmc_nav=0 */
}

void HalKestrel::spatial_reuse_init() {
  /* spatial_reuse_init: SR off, PLCP-source BSSID enable. */
  uint8_t v = static_cast<uint8_t>(_device.rtw_read8(r::R_AX_RX_SR_CTRL) &
                                   ~(r::B_AX_SR_EN | r::B_AX_SR_CTRL_PLCP_EN));
  _device.rtw_write8(r::R_AX_RX_SR_CTRL, v);
  uint8_t v2 = static_cast<uint8_t>(_device.rtw_read8(r::R_AX_BSSID_SRC_CTRL) |
                                    r::B_AX_PLCP_SRC_EN);
  _device.rtw_write8(r::R_AX_BSSID_SRC_CTRL, v2);
}

void HalKestrel::tmac_init() {
  /* tmac_init: non-loopback, TX underflow threshold, TXD FIFO MCS thresholds,
   * SW-preferred AC = BK. */
  clr32(r::R_AX_MAC_LOOPBACK, r::B_AX_MACLBK_EN);
  field32(r::R_AX_TCR0, r::TCR_UDF_THSD, r::B_AX_TCR_UDF_THSD_MSK,
          r::B_AX_TCR_UDF_THSD_SH);
  uint32_t v = _device.rtw_read32(r::R_AX_TXD_FIFO_CTRL);
  v = r::set_clr_word(v, r::TXDFIFO_HIGH_MCS_THRE,
                      r::B_AX_TXDFIFO_HIGH_MCS_THRE_MSK,
                      r::B_AX_TXDFIFO_HIGH_MCS_THRE_SH);
  v = r::set_clr_word(v, r::TXDFIFO_LOW_MCS_THRE,
                      r::B_AX_TXDFIFO_LOW_MCS_THRE_MSK,
                      r::B_AX_TXDFIFO_LOW_MCS_THRE_SH);
  _device.rtw_write32(r::R_AX_TXD_FIFO_CTRL, v);
  field32(r::R_AX_TB_PPDU_CTRL, r::MAC_AX_CMAC_AC_SEL_BK, r::B_AX_SW_PREFER_AC_MSK,
          r::B_AX_SW_PREFER_AC_SH);
}

void HalKestrel::trxptcl_init() {
  /* trxptcl_init: response SIFS (CCK/OFDM), RX-trigger FCS check, and the
   * response-rate-selection (RRSR): 1M CCK, legacy-rate response enabled. */
  uint32_t v = _device.rtw_read32(r::R_AX_TRXPTCL_RESP_0);
  v = r::set_clr_word(v, r::WMAC_SPEC_SIFS_CCK, r::B_AX_WMAC_SPEC_SIFS_CCK_MSK,
                      r::B_AX_WMAC_SPEC_SIFS_CCK_SH);
  v = r::set_clr_word(v,
                      _variant == ChipVariant::C8852C
                          ? r::WMAC_SPEC_SIFS_OFDM_52C
                          : r::WMAC_SPEC_SIFS_OFDM_52B,
                      r::B_AX_WMAC_SPEC_SIFS_OFDM_MSK,
                      r::B_AX_WMAC_SPEC_SIFS_OFDM_SH);
  _device.rtw_write32(r::R_AX_TRXPTCL_RESP_0, v);
  set32(r::R_AX_RXTRIG_TEST_USER_2, r::B_AX_RXTRIG_FCSCHK_EN);

  uint32_t c0 = _device.rtw_read32(r::R_AX_TRXPTCL_RRSR_CTL_0);
  uint32_t rate_en = (((c0 >> r::B_AX_WMAC_RESP_RATE_EN_SH) &
                       r::B_AX_WMAC_RESP_RATE_EN_MSK) |
                      r::WMAC_RRSR_RATE_LEGACY_EN);
  c0 = r::set_clr_word(c0, rate_en, r::B_AX_WMAC_RESP_RATE_EN_MSK,
                       r::B_AX_WMAC_RESP_RATE_EN_SH);
  c0 = r::set_clr_word(c0, r::WMAC_CCK_EN_1M, r::B_AX_WMAC_RRSB_AX_CCK_MSK,
                       r::B_AX_WMAC_RRSB_AX_CCK_SH);
  c0 &= ~r::B_AX_WMAC_RESP_REF_RATE_SEL; /* ref_rate_sel = REF2RXRATEANDCCTBL(0) */
  _device.rtw_write32(r::R_AX_TRXPTCL_RRSR_CTL_0, c0);
}

void HalKestrel::ptcl_init() {
  /* ptcl_init (band0, non-SW/HW TX mode): CMAC TX mode on, trigger-SS off;
   * special-report path -> WLCPU; AMPDU single-BK / agg-retry; agg limits. */
  uint8_t v = static_cast<uint8_t>(
      (_device.rtw_read8(r::R_AX_PTCL_COMMON_SETTING_0) |
       (r::B_AX_CMAC_TX_MODE_0 | r::B_AX_CMAC_TX_MODE_1)) &
      ~(r::B_AX_PTCL_TRIGGER_SS_EN_0 | r::B_AX_PTCL_TRIGGER_SS_EN_1 |
        r::B_AX_PTCL_TRIGGER_SS_EN_UL));
  _device.rtw_write8(r::R_AX_PTCL_COMMON_SETTING_0, v);
  uint8_t h = static_cast<uint8_t>(
      r::set_clr_word(_device.rtw_read8(r::R_AX_PTCLRPT_FULL_HDL),
                      r::FWD_TO_WLCPU, r::B_AX_SPE_RPT_PATH_MSK,
                      r::B_AX_SPE_RPT_PATH_SH));
  _device.rtw_write8(r::R_AX_PTCLRPT_FULL_HDL, h);

  uint8_t bk = static_cast<uint8_t>(
      _device.rtw_read8(r::R_AX_AGG_BK_0) &
      ~(r::B_AX_WDBK_CFG | r::B_AX_EN_RTY_BK | r::B_AX_EN_RTY_BK_COD));
  _device.rtw_write8(r::R_AX_AGG_BK_0, bk);
  uint32_t a = _device.rtw_read32(r::R_AX_AMPDU_AGG_LIMIT);
  a = r::set_clr_word(a, r::PTCL_MAX_AGG_NUM - 1, r::B_AX_MAX_AGG_NUM_MSK,
                      r::B_AX_MAX_AGG_NUM_SH);
  a = r::set_clr_word(a, r::PTCL_AMPDU_MAX_TIME_8852B, r::B_AX_AMPDU_MAX_TIME_MSK,
                      r::B_AX_AMPDU_MAX_TIME_SH);
  _device.rtw_write32(r::R_AX_AMPDU_AGG_LIMIT, a);
}

bool HalKestrel::write_lte(uint32_t offset, uint32_t val) {
  /* mac_write_lte_8852b (non-offload path): wait for the LTE interface ready
   * bit, then post the write via WDATA + CTRL. */
  for (uint32_t cnt = 1000; cnt != 0; --cnt) {
    if (_device.rtw_read8(r::R_AX_LTE_CTRL + 3) & (1u << 5))
      break;
    delay_us(50);
    if (cnt == 1) {
      _logger->error("Kestrel coex: LTE interface not ready");
      return false;
    }
  }
  _device.rtw_write32(r::R_AX_LTE_WDATA, val);
  _device.rtw_write32(r::R_AX_LTE_CTRL, r::LTE_WRITE_CMD | offset);
  return true;
}

void HalKestrel::set_host_rpr() {
  namespace r = kestrel::reg;
  /* WDRLS_MODE = STF (USB is always store-and-forward). */
  field32(r::R_AX_WDRLS_CFG, r::MAC_AX_RPR_MODE_STF, r::B_AX_WDRLS_MODE_MSK,
          r::B_AX_WDRLS_MODE_SH);
  /* RLSRPT0_CFG0: rpr_cfg_stf has all filters DISabled — clear the filter map
   * so every TX result generates a release report. */
  clr32(r::R_AX_RLSRPT0_CFG0, r::B_WDRLS_FLTR_TXOK | r::B_WDRLS_FLTR_RTYLMT |
                                  r::B_WDRLS_FLTR_LIFTIM | r::B_WDRLS_FLTR_MACID);
  /* RLSRPT0_CFG1: AGGNUM=121, TO=255. The timeout is the key — it releases the
   * WD/PLE pages before the (~103-frame) bulk-OUT pool fills. */
  field32(r::R_AX_RLSRPT0_CFG1, r::RPR_STF_AGG, r::B_AX_RLSRPT0_AGGNUM_MSK,
          r::B_AX_RLSRPT0_AGGNUM_SH);
  field32(r::R_AX_RLSRPT0_CFG1, r::RPR_STF_TMR, r::B_AX_RLSRPT0_TO_MSK,
          r::B_AX_RLSRPT0_TO_SH);
  _logger->info("Kestrel TRX: host RPR set (STF, agg=121 tmr=255) — TX page "
                "release enabled");
}

void HalKestrel::port_init() {
  namespace r = kestrel::reg;
  /* mac_port_init band-0 port-0 for injection (net_type = NO_LINK). Our injected
   * mgmt/probe carriers are self-sourced with no BSS, so a context-less NO_LINK
   * port is sufficient — but the port MUST be FUNC-enabled or the CMAC never
   * airs the queued frames (that is the ~103-frame stall). Ported from mport.c
   * mac_port_init: the PCFG_NET_TYPE/TX_SW/TX_RPT/RX_SW/RX_SYNC/RX_RPT writes all
   * fold to 0 for NO_LINK, and the tail `PORT_CFG_P0 |= b_en_l[0]` is the
   * enable. (Beacon-timing PCFGs — TBTT/BCN_AREA/DTIM — are for AP beaconing,
   * not mgmt injection, so they are omitted.) */
  uint32_t v = _device.rtw_read32(r::R_AX_PORT_CFG_P0);
  v = r::set_clr_word(v, r::MAC_AX_NET_TYPE_NO_LINK, r::B_AX_NET_TYPE_P0_MSK,
                      r::B_AX_NET_TYPE_P0_SH);
  v &= ~(r::B_AX_BCNTX_EN_P0 | r::B_AX_TXBCN_RPT_EN_P0 |
         r::B_AX_RX_BSSID_FIT_EN_P0 | r::B_AX_TSF_UDT_EN_P0 |
         r::B_AX_RXBCN_RPT_EN_P0);
  v |= r::B_AX_PORT_FUNC_EN_P0; /* enable the port */
  _device.rtw_write32(r::R_AX_PORT_CFG_P0, v);
  _logger->info("Kestrel TRX: CMAC port0 enabled (NO_LINK) — TX context up "
                "(PORT_CFG_P0=0x{:08x})",
                v);
}

void HalKestrel::sch_tx_en() {
  namespace r = kestrel::reg;
  /* set_hw_sch_tx_en (cmac_tx.c) band-0: RMW-enable the scheduler contention
   * TX queues our TX path uses. A queue whose CTN_TXEN bit is 0 never contends
   * for the medium, so its frames stay pinned in PLE and the mgmt bulk-OUT
   * stalls at ~103 as the page pool fills. The vendor sets this via
   * mac_set_hw_value(SCH_TXEN_CFG) at vif-up; devourer never did. */
  const uint16_t en = r::B_AX_CTN_TXEN_BE_0 | r::B_AX_CTN_TXEN_BK_0 |
                      r::B_AX_CTN_TXEN_VI_0 | r::B_AX_CTN_TXEN_VO_0 |
                      r::B_AX_CTN_TXEN_MGQ | r::B_AX_CTN_TXEN_CPUMGQ |
                      r::B_AX_CTN_TXEN_HGQ;
  uint16_t v = _device.rtw_read16(r::R_AX_CTN_TXEN);
  v |= en;
  _device.rtw_write16(r::R_AX_CTN_TXEN, v);
  /* Driver-side TX contention enable — the 8852C resets it cleared, so injected
   * frames never win a TX grant (no completion, PLE stalls). Vendor sets
   * 0x0003ffff at ifup. Set it for every variant (a no-op where the reset
   * default already has it). */
  _device.rtw_write32(
      r::R_AX_CTN_DRV_TXEN,
      _device.rtw_read32(r::R_AX_CTN_DRV_TXEN) | r::B_AX_CTN_DRV_TXEN_ALL);
  _logger->info("Kestrel TRX: CTN_TXEN=0x{:04x} + CTN_DRV_TXEN enabled", v);

  /* Clear the CMAC CCA medium-busy gates (R_AX_CCA_CFG_0 bits 0-4). The
   * carrier/energy detectors report a perpetual busy that freezes the CSMA
   * backoff, so the scheduler never grants a TX opportunity and injected frames
   * stall in the MBH queue (~103-frame stall). DACK/RX-DCK/ADDCK are applied,
   * but carrier-sense TX needs the deeper RF cal (IQK/DPK) that isn't ported;
   * disabling the gates lets frames air — the intended TX/monitor mode, on-air
   * validated (both the 8852BU and 8832CU radiate). */
  if (_cca_on) {
    /* Experimental (DEVOURER_KESTREL_CCA_ON): leave the CCA gates ENABLED for a
     * carrier-sense TX test. NB: measured NOT sufficient on its own — TX still
     * 103-stalls even with RX-DCK + ADDCK + the EDCCA level threshold seeded to
     * -62 dBm; carrier-sense TX needs deeper RF/BB config (IQK/DPK or a CCA
     * state-machine bring-up). */
    _logger->info("Kestrel TRX: CCA gates LEFT ON (CCA_CFG_0=0x{:08x}) — "
                  "carrier-sense TX test",
                  _device.rtw_read32(r::R_AX_CCA_CFG_0));
    return;
  }
  clr32(r::R_AX_CCA_CFG_0, r::B_AX_CCA_ALL_EN);
  _logger->info("Kestrel TRX: CCA TX gates cleared (CCA_CFG_0=0x{:08x})",
                _device.rtw_read32(r::R_AX_CCA_CFG_0));
}

bool HalKestrel::start_beacon(const uint8_t *body, uint32_t len,
                             uint16_t interval_tu, uint8_t bss_color,
                             uint16_t rate_ax) {
  namespace r = kestrel::reg;
  /* 1) Hand the fw the beacon body (BCN_UPD_V1). Single path A (ntx=1, map=0),
   *    matching the injection cctl. */
  if (!_fw.fw_send_beacon(body, len, /*macid=*/0, rate_ax, /*ntx_path_en=*/1,
                          /*path_map_a=*/0))
    return false;

  /* 2) Reconfigure band-0 port-0 for AP beaconing (mac_port_init AP branch,
   *    single BSS). NET_TYPE=AP + BCN_PRCT + BCNTX_EN(TX_SW) + port enable, all
   *    in PORT_CFG_P0; the DTIM/HIQ power-save PCFGs are omitted. */
  uint32_t p = _device.rtw_read32(r::R_AX_PORT_CFG_P0);
  p = r::set_clr_word(p, r::MAC_AX_NET_TYPE_AP, r::B_AX_NET_TYPE_P0_MSK,
                      r::B_AX_NET_TYPE_P0_SH);
  p |= r::B_AX_TBTT_PROHIB_EN_P0 | r::B_AX_BRK_SETUP_P0; /* BCN_PRCT=1 */
  p |= r::B_AX_BCNTX_EN_P0;                              /* TX_SW=1 */
  p |= r::B_AX_PORT_FUNC_EN_P0;                          /* port enable */
  _device.rtw_write32(r::R_AX_PORT_CFG_P0, p);

  /* 3) Beacon timing (verbatim mac_port_init values). BCN_INTV, BSS_CLR,
   *    TBTT_AGG, BCN_HOLD_TIME, BCN_MASK_AREA, BCN_ERLY, BCN_SETUP, TBTT_ERLY. */
  field32(r::R_AX_BCN_SPACE_CFG_P0, interval_tu, r::B_AX_BCN_SPACE_P0_MSK,
          r::B_AX_BCN_SPACE_P0_SH);
  field32(r::R_AX_PTCL_BSS_COLOR_0, bss_color, r::B_AX_BSS_COLOR_P0_MSK,
          r::B_AX_BSS_COLOR_P0_SH);
  /* 0xC412 (TBTT_AGG) is a 16-bit register — write it at native width so the
   * adjacent aliased registers are not clobbered. */
  {
    uint16_t v = _device.rtw_read16(r::R_AX_TBTT_AGG_P0);
    v = static_cast<uint16_t>(
        (v & ~(r::B_AX_TBTT_AGG_NUM_P0_MSK << r::B_AX_TBTT_AGG_NUM_P0_SH)) |
        ((r::TBTT_AGG_DEF & r::B_AX_TBTT_AGG_NUM_P0_MSK)
         << r::B_AX_TBTT_AGG_NUM_P0_SH));
    _device.rtw_write16(r::R_AX_TBTT_AGG_P0, v);
  }
  /* TBTT_PROHIB_P0 (0xC404) carries BCN_HOLD_TIME[27:16] and BCN_SETUP[7:0]. */
  field32(r::R_AX_TBTT_PROHIB_P0, r::BCN_HOLD_DEF, r::B_AX_TBTT_HOLD_P0_MSK,
          r::B_AX_TBTT_HOLD_P0_SH);
  field32(r::R_AX_TBTT_PROHIB_P0, r::BCN_SETUP_DEF, r::B_AX_TBTT_SETUP_P0_MSK,
          r::B_AX_TBTT_SETUP_P0_SH);
  field32(r::R_AX_BCN_AREA_P0, r::BCN_MASK_DEF, r::B_AX_BCN_MSK_AREA_P0_MSK,
          r::B_AX_BCN_MSK_AREA_P0_SH);
  field32(r::R_AX_BCNERLYINT_CFG_P0, r::BCN_ERLY_DEF, r::B_AX_BCNERLY_P0_MSK,
          r::B_AX_BCNERLY_P0_SH);
  /* 0xC40E (TBTT_ERLY) is the 16-bit alias of BCNERLYINT_CFG_P0's high half —
   * write at native 16-bit width. */
  {
    uint16_t v = _device.rtw_read16(r::R_AX_TBTTERLYINT_CFG_P0);
    v = static_cast<uint16_t>(
        (v & ~(r::B_AX_TBTTERLY_P0_MSK << r::B_AX_TBTTERLY_P0_SH)) |
        ((r::TBTT_ERLY_DEF & r::B_AX_TBTTERLY_P0_MSK) << r::B_AX_TBTTERLY_P0_SH));
    _device.rtw_write16(r::R_AX_TBTTERLYINT_CFG_P0, v);
  }
  _logger->info("Kestrel: HW beacon armed on port0 (intv={}TU color={} "
                "PORT_CFG_P0=0x{:08x})",
                interval_tu, bss_color, p);
  return true;
}

void HalKestrel::mac_enable_imr() {
  /* mac_enable_imr(DMAC)+(CMAC0) + mac_err_imr_ctrl(EN): the System-Error-
   * Recovery interrupt masks (ser_imr_config_8852b auto-gen). Each entry does
   * reg = (reg & ~msk) | set (the vendor's write_mac_reg_auto_ofld masked-write
   * semantics; applied here as direct writes). Values are the auto-gen
   * "clear mask / set value" pairs, verbatim. band0. */
  struct ImrW {
    uint16_t reg;
    uint32_t msk;
    uint32_t set;
  };
  auto apply = [&](const ImrW *t, size_t n) {
    for (size_t i = 0; i < n; ++i)
      _device.rtw_write32(t[i].reg,
                          (_device.rtw_read32(t[i].reg) & ~t[i].msk) | t[i].set);
  };
  /* ser_imr_config(DMAC_SEL). */
  static const ImrW kDmac[] = {
      {0x9430, 0x3337, 0x3327},          /* WDRLS_ERR_IMR */
      {0x9D1C, 0x8, 0x8},                /* SEC_DEBUG */
      {0x9BF4, 0x3e, 0x0},               /* MPDU_TX_ERR_IMR */
      {0x9CF4, 0xb, 0x0},                /* MPDU_RX_ERR_IMR */
      {0x9EF0, 0x7, 0x7},                /* STA_SCHEDULER_ERR_IMR */
      {0x9F1C, 0x30f, 0x101},            /* TXPKTCTL_ERR_IMR_ISR */
      {0x9F2C, 0x30f, 0x303},            /* TXPKTCTL_ERR_IMR_ISR_B1 */
      {0x8C38, 0x7f0ff0ff, 0x3f0ff0ff},  /* WDE_ERR_IMR */
      {0x9038, 0xf0ff0ff, 0xf0ff0df},    /* PLE_ERR_IMR */
      {0x9A20, 0x1, 0x1},                /* PKTIN_ERR_IMR */
      {0x8850, 0xff0fffff, 0xcc000161},  /* HOST_DISPATCHER_ERR_IMR */
      {0x8854, 0x3f07ffff, 0x4000062},   /* CPU_DISPATCHER_ERR_IMR */
      {0x8858, 0x3f031f1f, 0x0},         /* OTHER_DISPATCHER_ERR_IMR */
      {0x9840, 0x1111, 0x1111},          /* CPUIO_ERR_IMR */
      {0x960C, 0x1, 0x1},                /* BBRPT_COM_ERR_IMR_ISR */
      {0x962C, 0xff, 0x0},               /* BBRPT_CHINFO_ERR_IMR_ISR */
  };
  /* ser_imr_config(CMAC_SEL), band0. */
  static const ImrW kCmac[] = {
      {0xC800, 0x80c000, 0xc000},        /* DLE_CTRL (cmac_dma_top) */
      {0xC6C0, 0xff80df01, 0x10800001},  /* PTCL_IMR0 */
      {0xC3E8, 0x3, 0x1},                /* SCHEDULE_ERR_IMR */
      {0xCCFC, 0x3f0000, 0x10000},       /* PHYINFO_ERR_IMR */
      {0xCEF4, 0xff000, 0xe4000},        /* RMAC_ERR_ISR */
      {0xCCEC, 0x780, 0x780},            /* TMAC_ERR_IMR_ISR */
  };
  apply(kDmac, sizeof(kDmac) / sizeof(kDmac[0]));
  apply(kCmac, sizeof(kCmac) / sizeof(kCmac[0]));
  /* ser_imr_config_patch(CMAC): phy-intf timeout threshold = max. */
  field32(r::R_AX_PHYINFO_ERR_IMR, 0x3f, r::B_AX_PHYINTF_TIMEOUT_THR_MSK,
          r::B_AX_PHYINTF_TIMEOUT_THR_SH);
  /* mac_err_imr_ctrl(EN): unmask the top-level DMAC + CMAC0 error IMR. */
  _device.rtw_write32(r::R_AX_DMAC_ERR_IMR, 0xFFFFFFFFu);
  _device.rtw_write32(r::R_AX_CMAC_ERR_IMR, 0xFFFFFFFFu);
}

void HalKestrel::coex_mac_init() {
  /* coex_mac_init_8852b: disable LTE-coex (CTRL + CTRL_2 = 0), then set the
   * SDIO-ctrl coex bit. On the WiFi+BT combo die the coex block otherwise
   * arbitrates the shared front end. */
  write_lte(r::R_AX_LTECOEX_CTRL, 0);
  uint8_t val = _device.rtw_read8(r::R_AX_SYS_SDIO_CTRL + 3);
  write_lte(r::R_AX_LTECOEX_CTRL_2, 0);
  _device.rtw_write8(r::R_AX_SYS_SDIO_CTRL + 3,
                     static_cast<uint8_t>(val | (1u << 2)));
}

void HalKestrel::enable_bb_rf() {
  /* set_enable_bb_rf(1) (hw.c set_enable_bb_rf, enable==1 8852B branch) — the
   * vendor runs this in mac_hal_init immediately after FWDL, before sys_init
   * and trx_init, so the BB/RF is out of reset when the firmware begins its own
   * runtime init. All MAC-space (wIndex=0). */
  set32(r::R_AX_SYS_FUNC_EN, r::B_AX_FEN_BBRSTB | r::B_AX_FEN_BB_GLB_RSTN);
  field32(r::R_AX_SPS_DIG_ON_CTRL0, 0x1, r::B_AX_REG_ZCDC_H_MSK,
          r::B_AX_REG_ZCDC_H_SH);
  /* RDC KS/BB: AFC_AFEDIG write 1 / 0 / 1. */
  set32(r::R_AX_WLRF_CTRL, r::B_AX_AFC_AFEDIG);
  clr32(r::R_AX_WLRF_CTRL, r::B_AX_AFC_AFEDIG);
  set32(r::R_AX_WLRF_CTRL, r::B_AX_AFC_AFEDIG);
  if (_variant == ChipVariant::C8852C) {
    /* 8852C-only (hw.c is_chip_id(8852C)): RX ADC power (AFE LDO) + xtal
     * ANAPAR block the 8852B recipe lacks. "for ADC PWR setting". */
    uint32_t vldo = _device.rtw_read32(r::R_AX_AFE_OFF_CTRL1);
    vldo &= ~r::LDO2PW_LDO_VSEL;
    vldo |= (0x1u << r::B_AX_S0_LDO_VSEL_F_SH) |
            (0x1u << r::B_AX_S1_LDO_VSEL_F_SH);
    _device.rtw_write32(r::R_AX_AFE_OFF_CTRL1, vldo);
    write_xtal_si(r::XTAL_SI_XTAL0, 0x7, 0xFF);
    write_xtal_si(r::XTAL_SI_ANAPAR_WL, 0x6c, 0xFF);
  }
  write_xtal_si(r::XTAL_SI_WL_RFC_S0, 0xC7, 0xFF);
  write_xtal_si(r::XTAL_SI_WL_RFC_S1, 0xC7, 0xFF);
  if (_variant == ChipVariant::C8852C)
    write_xtal_si(r::XTAL_SI_XTAL3, 0xd, 0xFF);
  _device.rtw_write8(r::R_AX_PHYREG_SET, r::PHYREG_SET_XYN_CYCLE);
  _logger->info("Kestrel: set_enable_bb_rf(1) after FWDL (BB/RF out of reset)");
}

bool HalKestrel::phy_bb_rf_init(uint8_t rfe_type, uint8_t cut) {
  /* set_enable_bb_rf toggle (rtl8852b_halinit.c: enable_bb_rf(0) then (1)) —
   * the vendor pulses a disable before the enable to reset the RFC/AFE clock
   * domain. Disable path: clear AFC_AFEDIG, clear BBRSTB|GLB_RSTN, WL_RFC &=
   * 0xF8 (drop the clock-enable nibble). */
  clr32(r::R_AX_WLRF_CTRL, r::B_AX_AFC_AFEDIG);
  {
    uint8_t v8 = _device.rtw_read8(r::R_AX_SYS_FUNC_EN);
    v8 &= ~static_cast<uint8_t>(r::B_AX_FEN_BBRSTB | r::B_AX_FEN_BB_GLB_RSTN);
    _device.rtw_write8(r::R_AX_SYS_FUNC_EN, v8);
  }
  write_xtal_si(r::XTAL_SI_WL_RFC_S0, read_xtal_si(r::XTAL_SI_WL_RFC_S0) & 0xF8,
                0xFF);
  write_xtal_si(r::XTAL_SI_WL_RFC_S1, read_xtal_si(r::XTAL_SI_WL_RFC_S1) & 0xF8,
                0xFF);

  /* set_enable_bb_rf (hw.c) — release the BB from reset + enable the RF/AFE
   * clocks. Without this the BB is held in reset and silently drops every
   * halbb/halrf register write (readback = 0). All MAC-space (wIndex=0). */
  set32(r::R_AX_SYS_FUNC_EN, r::B_AX_FEN_BBRSTB | r::B_AX_FEN_BB_GLB_RSTN);
  field32(r::R_AX_SPS_DIG_ON_CTRL0, 0x1, r::B_AX_REG_ZCDC_H_MSK,
          r::B_AX_REG_ZCDC_H_SH);
  /* RDC KS/BB: AFC_AFEDIG write 1 / 0 / 1. */
  set32(r::R_AX_WLRF_CTRL, r::B_AX_AFC_AFEDIG);
  clr32(r::R_AX_WLRF_CTRL, r::B_AX_AFC_AFEDIG);
  set32(r::R_AX_WLRF_CTRL, r::B_AX_AFC_AFEDIG);
  if (_variant == ChipVariant::C8852C) {
    /* 8852C-only (hw.c is_chip_id(8852C)): RX ADC power (AFE LDO) + xtal
     * ANAPAR block the 8852B recipe lacks. "for ADC PWR setting". */
    uint32_t vldo = _device.rtw_read32(r::R_AX_AFE_OFF_CTRL1);
    vldo &= ~r::LDO2PW_LDO_VSEL;
    vldo |= (0x1u << r::B_AX_S0_LDO_VSEL_F_SH) |
            (0x1u << r::B_AX_S1_LDO_VSEL_F_SH);
    _device.rtw_write32(r::R_AX_AFE_OFF_CTRL1, vldo);
    write_xtal_si(r::XTAL_SI_XTAL0, 0x7, 0xFF);
    write_xtal_si(r::XTAL_SI_ANAPAR_WL, 0x6c, 0xFF);
  }
  write_xtal_si(r::XTAL_SI_WL_RFC_S0, 0xC7, 0xFF);
  write_xtal_si(r::XTAL_SI_WL_RFC_S1, 0xC7, 0xFF);
  if (_variant == ChipVariant::C8852C)
    write_xtal_si(r::XTAL_SI_XTAL3, 0xd, 0xFF);
  _device.rtw_write8(r::R_AX_PHYREG_SET, r::PHYREG_SET_XYN_CYCLE);
  _logger->info("Kestrel PHY: BB/RF enabled — WL_RFC_S0=0x{:02x} S1=0x{:02x} "
                "(want C7) WLRF_CTRL=0x{:08x} SYS_FUNC_EN=0x{:08x}",
                read_xtal_si(r::XTAL_SI_WL_RFC_S0),
                read_xtal_si(r::XTAL_SI_WL_RFC_S1),
                _device.rtw_read32(r::R_AX_WLRF_CTRL),
                _device.rtw_read32(r::R_AX_SYS_FUNC_EN));

  /* --- BB register + gain + radio tables via FIRMWARE OFFLOAD (cmd_ofld H2C),
   * the vendor's io_ofld path for USB (halbb_fw_set_reg / halrf_wrf ->
   * mac_add_cmd_ofld). Each table write is one 16-byte cmd_ofld command; a
   * full 2000-byte buffer auto-flushes with LC forced on its last command
   * (ofld_incompatible_full_cmd), and each SECTION ends with the vendor's LC
   * sentinel flush: halbb = a dummy BB write to 0x1a24, halrf = a 1 us
   * DELAY_OFLD (halbb_fwofld_bitmap_en / halrf_write_fwofld_trigger). The
   * 0xf9..0xfe delay pseudo-addresses are HOST-side sleeps in the vendor
   * walkers (halbb_fwcfg_bb_phy_8852b:42, no flush, no fw delay command);
   * phy1 (0x100xxxxx) entries are skipped for single-PHY. --- */
  /* Flip the master HCI flow-control enable now — the vendor does this at ifup
   * right before its first bulk cmd_ofld (HCI_FC_CTRL 0x08040000 -> 0x055b), so
   * FC_EN is only armed once the DMAC/CMAC/USB init has fully settled. This is
   * what actually credits a CH12 page for the incoming H2C. The 8852C flow
   * control is the _V1 register. */
  set32(_variant == ChipVariant::C8852C ? r::R_AX_HCI_FC_CTRL_V1_NIC
                                        : r::R_AX_HCI_FC_CTRL,
        r::B_AX_HCI_FC_EN);
  delay_us(10);

  _fw.ofld_begin();
  auto bb_emit = [&](uint32_t addr, uint32_t val) {
    switch (addr) {
    case 0xfe: delay_us(50000); return;
    case 0xfd: delay_us(5000); return;
    case 0xfc: delay_us(1000); return;
    case 0xfb: delay_us(50); return;
    case 0xfa: delay_us(5); return;
    case 0xf9: delay_us(1); return;
    default: break;
    }
    if ((addr >> 16) == 0x100) /* phy1-only entry, skip on single-PHY */
      return;
    _fw.ofld_write(r::OFLD_SRC_BB, r::OFLD_TYPE_WRITE, 0,
                   static_cast<uint16_t>(addr & 0xffff), val, r::MASKDWORD);
  };
  /* Chip-specific BB/RF tables — the 8832CU is a different die; applying the
   * 8852B phy_reg/radio tables leaves its RF/BB mis-tuned so the RX front-end is
   * deaf on both bands (BB0x4004 still reads the table value since it is a
   * common core-BB register, masking the mismatch). */
  const uint32_t *phy_reg = array_mp_8852b_phy_reg;
  uint32_t phy_reg_len = array_mp_8852b_phy_reg_len;
  const uint32_t *radioa = array_mp_8852b_radioa;
  uint32_t radioa_len = array_mp_8852b_radioa_len;
  const uint32_t *radiob = array_mp_8852b_radiob;
  uint32_t radiob_len = array_mp_8852b_radiob_len;
#if defined(DEVOURER_HAVE_KESTREL_8852C)
  if (_variant == ChipVariant::C8852C) {
    phy_reg = array_mp_8852c_phy_reg;
    phy_reg_len = array_mp_8852c_phy_reg_len;
    radioa = array_mp_8852c_radioa;
    radioa_len = array_mp_8852c_radioa_len;
    radiob = array_mp_8852c_radiob;
    radiob_len = array_mp_8852c_radiob_len;
  }
#endif
  apply_phy_table(phy_reg, phy_reg_len, rfe_type, cut, bb_emit);
  _fw.ofld_flush(KestrelFw::OfldFlush::BB); /* bitmap_en(false): 0x1a24 LC */
  /* array_mp_8852b_phy_reg_gain is NOT a register table — halbb_init_gain_table
   * -> halbb_cfg_bb_gain_8852b decodes each entry as (cfg_type,band,path,type)
   * and stores `data` into the SOFTWARE gain cache (writing zero hardware
   * registers; emitting it as cmd_ofld faults the fw). Populate the cache here;
   * set_gain_error applies it to the band-specific BB regs per channel — the
   * 5 GHz RX front-end is deaf without it. */
  init_gain_table(rfe_type, cut);

  /* halrf_wrf offload dispatch (halrf_interface.c:202; each RF reg appears
   * twice in the radio table): BIT(16) set = d-die -> a BB write to the
   * 0xe000/0xf000 window; clear = a-die -> RF_CMD_OFLD (fw drives the SI).
   * mask = MASKRF (20-bit full write). Each radio table ends with the halrf
   * trigger (1 us DELAY_OFLD, lc=1) and its radio-page H2C to the fw
   * (halrf_config_8852b_write_radio_a/b_reg_to_fw). */
  std::vector<uint32_t> radio_page;
  auto apply_radio = [&](const uint32_t *arr, size_t len, uint8_t path) {
    radio_page.clear();
    auto emit = [&](uint32_t addr, uint32_t val) {
      switch (addr) {
      case 0xfe: delay_us(50000); return;
      case 0xfd: delay_us(5000); return;
      case 0xfc: delay_us(1000); return;
      case 0xfb: delay_us(50); return;
      case 0xfa: delay_us(5); return;
      case 0xf9: delay_us(1); return;
      default: break;
      }
      /* store_radio_reg (halrf_hwimg_8852b.c:157): page entries are the DRFC
       * (d-die, addr >= 0x100) writes only, packed (addr&0xff)<<20 | data. */
      if (addr >= 0x100)
        radio_page.push_back(((addr & 0xff) << 20) | (val & r::MASKRF));
      if (addr & 0x10000u) {
        const uint16_t off = static_cast<uint16_t>((path ? 0xf000 : 0xe000) +
                                                   ((addr & 0xff) << 2));
        _fw.ofld_write(r::OFLD_SRC_BB, r::OFLD_TYPE_WRITE, 0, off,
                       val & r::MASKRF, r::MASKRF);
      } else {
        _fw.ofld_write(r::OFLD_SRC_RF, r::OFLD_TYPE_WRITE, path,
                       static_cast<uint16_t>(addr & 0xff), val & r::MASKRF,
                       r::MASKRF);
      }
    };
    apply_phy_table(arr, len, rfe_type, cut, emit);
    _fw.ofld_flush(KestrelFw::OfldFlush::RF); /* halrf trigger */
    _fw.radio_page_to_fw(path == 0 ? r::OUTSRC_CL_RADIO_A : r::OUTSRC_CL_RADIO_B,
                         0, radio_page.data(),
                         static_cast<uint16_t>(radio_page.size()));
  };
  apply_radio(radioa, radioa_len, 0);
  apply_radio(radiob, radiob_len, 1);

  /* Decisive readback: BB 0x4004 is host-readable (core BB), and the phy_reg
   * table sets it to 0xCA014000 — but ONLY via fw-offload now (no direct
   * writes). If it reads back the table value, the fw processed the batch. */
  const uint32_t bb4004 = _device.rtw_read32_wide(0x4004u + 0x10000u);
  _logger->info("Kestrel PHY: BB+RF tables applied via fw-offload "
                "(rfe=0x{:02x} cut={}) — BB0x4004=0x{:08x} (want CA014000)",
                rfe_type, cut, bb4004);

  /* 8852C only: connect the BB IFFT output to the TX chain (T-MAC path-com).
   * The 8852B path routes TX per-STA via the CMAC antenna model and so has no
   * analogue; the 8852C needs this explicit routing block or the TX chain is
   * silent even though the RF synth locks. Part of the vendor trx-init. */
  if (_variant == ChipVariant::C8852C) {
    ctrl_tx_path_tmac_8852c();
    ctrl_rx_path_8852c(); /* enable the RX chains (0x4978 rx_path_en) */
    /* bb_dm_init_8852c() STRIPPED (hand-transcribed halbb DM — error source +
     * RX-halt bisect). The extracted BB tables set the DIG/AGC defaults. */
  }

  return true;
}

void HalKestrel::ctrl_tx_path_tmac_8852c() {
  /* halbb_ctrl_tx_path_tmac_8852c, RF_PATH_A / single-PHY (dbcc off).
   * The 0xDC00.. and 0xD800.. registers are CMAC band-0 power-region MAC
   * registers (mac_write_pwr_reg: band-0 -> direct MAC_REG_W32 at the raw
   * offset, region R_AX_PWR_RATE_CTRL 0xD200..0xFFFF), so they take a plain
   * rtw_write32. 0x09a4 / 0x58dc are BB-window registers (bb_rmw). */

  /* powermap clear: 0xDC00..0xDC28 (11 dwords). */
  for (uint16_t off = 0; off <= 4 * 10; off += 4)
    _device.rtw_write32(0xDC00 + off, 0x0);

  /* 0x9a4[4:2] tx path_en / path_map_{a,b} source = 0 (T-MAC TX). */
  bb_rmw(0x09a4, 0x1c, 0x0);

  /* path-com CRs, RF_PATH_A: path_a enable, path_a connect ifft0. */
  static const uint32_t path_com[][2] = {
      {0xD800, 0x08889880}, {0xD804, 0x13111111}, {0xD808, 0x01209313},
      {0xD80C, 0x49249249}, {0xD810, 0x1C9C9C49}, {0xD814, 0x39393939},
      {0xD818, 0x39393939}, {0xD81C, 0x39393939}, {0xD820, 0x00003939},
      {0xD824, 0x00000000}, {0xD828, 0xE0000000}, {0xD82C, 0x00000000}};
  for (const auto &e : path_com)
    _device.rtw_write32(static_cast<uint16_t>(e[0]), e[1]);

  /* TSSI reset (path A): 0x58dc[31:30] = 1 then 3. */
  bb_rmw(0x58dc, 0xC0000000u, 0x1);
  bb_rmw(0x58dc, 0xC0000000u, 0x3);

  _logger->info("Kestrel PHY(8852C): T-MAC TX path-com routed (RF_PATH_A)");
}

void HalKestrel::ctrl_rx_path_8852c() {
  /* halbb_ctrl_rx_path_8852c (halbb_8852c_api.c), non-DBCC, RF_PATH_AB (both RX
   * chains for the monitor). The 0x4978 rx_path_en is the enable devourer never
   * wrote -> the 8852C RX front end stayed disabled (near-deaf). */
  bb_rmw(0x4978, 0xf, 0x0); /* clear first (8852C/D) */
  bb_rmw(0x4978, 0xf, 0x3); /* rx_path_en = AB */
  /* 1RCCA */
  bb_rmw(0x4974, 0x3C000u, 0x3);
  bb_rmw(0x4974, 0x3C0000u, 0x3);
  /* Rx HT nss_limit / mcs_limit */
  bb_rmw(0xd18, (1u << 9) | (1u << 8), 1);
  bb_rmw(0xd18, (1u << 22) | (1u << 21), 1);
  /* Rx HE n_user_max / tb_max_nss (HE_N_USER_MAX_8852C = 8) */
  bb_rmw(0xd80, 0x3fc0u, 8);
  bb_rmw(0xd80, (1u << 16) | (1u << 15) | (1u << 14), 1);
  bb_rmw(0xd80, (1u << 25) | (1u << 24) | (1u << 23), 1);
  /* TSSI reset (both paths) */
  bb_rmw(0x58dc, 0xC0000000u, 0x1);
  bb_rmw(0x58dc, 0xC0000000u, 0x3);
  bb_rmw(0x78dc, 0xC0000000u, 0x1);
  bb_rmw(0x78dc, 0xC0000000u, 0x3);
  /* BB reset (both paths) */
  bb_rmw(0x5818, 1u << 30, 0x1);
  bb_rmw(0x7818, 1u << 30, 0x1);
  bb_rmw(0x5818, 1u << 30, 0x0);
  bb_rmw(0x7818, 1u << 30, 0x0);
  _logger->info("Kestrel PHY(8852C): RX path enabled (RF_PATH_AB, 0x4978=0x3)");
}

/* ======================= 8852C halbb DM init =============================
 * halbb_dm_init_per_phy (halbb_init.c) register writes, ported verbatim for the
 * 8852C: single PHY (HW_PHY_0), non-MP, non-DBCC, cr_type=BB_AP2, so every cr->
 * address resolves from the *_A2 macros in halbb_cr_info_8852c.h. The
 * 8852B-derived bring-up ran none of this, leaving the RX AGC/DIG/EDCCA/CFO/
 * env-monitor unconfigured. BB registers -> bb_rmw (BB window, absolute mask);
 * the 0xDxxx power-region regs -> field32 (MAC power-reg plane, field+shift).
 * A HW_PHY_1 pwr-reg write with dbcc off is a vendor no-op and is omitted. */
void HalKestrel::bb_dm_init_8852c() {
  namespace r = kestrel::reg;
  /* Writes are ordered as halbb_dm_init_per_phy (halbb_init.c) calls them.
   * Functions that write no hardware register on the 8852C (get_efuse, mlo,
   * pause, supportability, cmn_rpt, bb_wrap[BE-only], statistics_reset, la,
   * dfs, ra, ul_tb, pwr_ctrl, rua, antdiv[non-antdiv adapter], pathdiv, mccdm,
   * plcp, sniffer, dv_pxp, auto_debug, cck_rpl_ofst[no-8852C-case]) are omitted. */

  /* 1. halbb_ic_hw_setting_init_8852c (halbb_8852c.c:234). */
  bb_rmw(0x0a10, 1u << 0, 1); /* dbcc_80p80 evm_rpt_en */
  bb_rmw(0x2a10, 1u << 0, 1);
  field32(0xd240, 0x1d8, 0x1ff, 9); /* CBV: min UL txpwr -10dBm [17:9] */
  field32(0xd290, 0x0, 0xff, 0);    /* CBV: UL txpwr comp 0dB [7:0] */

  /* 2. halbb_gpio_setting_init_8852c (halbb_8852c_api.c:2274) — RF-front-end
   * module GPIO: TRSW antenna switch + LNAON external LNA + PAPE routing, both
   * paths. Never run by the 8852B-derived bring-up -> RX antenna/LNA path
   * unrouted (the near-deafness). */
  for (uint32_t base : {0x5800u, 0x7800u}) {
    bb_rmw(base | 0x68, 0x2u, 1);     /* r_tx_ant_sel */
    bb_rmw(base | 0x68, 0x4u, 0);
    bb_rmw(base | 0x68, 0xE0u, 0);    /* -> TRSW look-up-table */
    bb_rmw(base | 0x80, r::MASKDWORD, 0x77777777);
    bb_rmw(base | 0x84, r::MASKDWORD, 0x77777777);
    bb_rmw(base | 0x94, 1u << 24, 1); /* TRSW from rfm_TRSW */
  }
  bb_rmw(0x334, r::MASKDWORD, 0xFFFFFFFF); /* output mode[31:0] */
  bb_rmw(0x338, r::MASKDWORD, 0x0);        /* non dbg_gpio mode */
  bb_rmw(0x33C, r::MASKDWORD, 0x0);        /* path_sel[15:0] */
  bb_rmw(0x340, r::MASKDWORD, 0x0);        /* path_sel[31:16] */
  /* TRSW look-up table (halbb_gpio_trsw_table_8852c), both paths, fields [31:16]. */
  for (uint32_t cr : {0x5868u, 0x7868u}) {
    bb_rmw(cr, 0x00030000u, 1);
    bb_rmw(cr, 0x000C0000u, 2);
    bb_rmw(cr, 0x00300000u, 2);
    bb_rmw(cr, 0x00C00000u, 2);
    bb_rmw(cr, 0x03000000u, 1);
    bb_rmw(cr, 0x0C000000u, 2);
    bb_rmw(cr, 0x30000000u, 2);
    bb_rmw(cr, 0xC0000000u, 2);
  }
  /* RFM output values (halbb_gpio_rfm_8852c): PAPE=[7:0], TRSW=[15:8],
   * LNAON=[23:16]; rfe_type not 63/64 so LNAON=0x8. */
  for (uint32_t cr : {0x5894u, 0x7894u}) {
    bb_rmw(cr, 0x000000FFu, 0x4); /* PAPE_RFM */
    bb_rmw(cr, 0x0000FF00u, 0x6); /* TRSW_RFM */
    bb_rmw(cr, 0x00FF0000u, 0x8); /* LNAON_RFM */
  }

  /* 6. halbb_physts_parsing_init_io_en (halbb_physts.c:3208). CR base
   * PHY_STS_BITMAP_SEARCH_FAIL_A2 = 0x073C; per-IE-page bitmaps RMW the reset
   * default. brk_fail + the modified pages only (unmodified pages RMW to
   * themselves = no-op). page->addr uses the (>=10 -> page--) remap. */
  bb_rmw(0x0738, 0xCu, 0x3);      /* brk_fail_rpt_en */
  bb_rmw(0x0754, 1u << 13, 1);    /* page6 HE_MU  |= IE13_DL_MU_DEF */
  bb_rmw(0x0758, 1u << 13, 1);    /* page7 VHT_MU |= IE13_DL_MU_DEF */
  bb_rmw(0x0764, 1u << 9, 1);     /* page11 CCK_PKT |= IE09 */
  bb_rmw(0x0764, 1u << 1, 1);     /* page11 CCK_PKT |= IE01_CMN_OFDM */
  for (uint32_t a : {0x0768u, 0x076Cu, 0x0770u, 0x0774u, 0x0778u}) {
    bb_rmw(a, 0xF0u, 0x3);        /* pages12-16: ext-path A|B ([7:4]=valid_path) */
    bb_rmw(a, 1u << 9, 1);        /* |= IE09 */
  }

  /* 8. halbb_dbg_setting_init -> halbb_bb_dbg_port_clock_en(false) (dbg.c:168). */
  bb_rmw(0x20F4, 1u << 24, 0);
  bb_rmw(0x20F8, 1u << 31, 0);

  /* 9. halbb_txdiff_tbl_init (halbb_pwr_ctrl.c:730) — TX power-diff table (MAC
   * pwr-reg plane, full dword). 12 MCS rows x 4 dwords @0xD56C, then 48 zeros
   * @0xD62C (CCK/legacy). */
  static const uint32_t kTxDiff[48] = {
      0x00000000, 0x00000000, 0x00000000, 0x00000000, /* MCS0 */
      0x00011111, 0x00011111, 0x00011111, 0x00011111, /* MCS1 */
      0x00022211, 0x00022222, 0x00022222, 0x00022222, /* MCS2 */
      0x00022211, 0x00033333, 0x00033333, 0x00033333, /* MCS3 */
      0x00022211, 0x00044433, 0x00044444, 0x00044444, /* MCS4 */
      0x00021111, 0x00043332, 0x00055554, 0x00055555, /* MCS5 */
      0x00022222, 0x00044433, 0x00066655, 0x00066666, /* MCS6 */
      0x00033321, 0x00054443, 0x00076655, 0x00077777, /* MCS7 */
      0x00032111, 0x00044444, 0x00066655, 0x00088877, /* MCS8 */
      0x00022221, 0x00055443, 0x00076655, 0x00088877, /* MCS9 */
      0x00022111, 0x00044333, 0x00066665, 0x00088777, /* MCS10 */
      0x00022211, 0x00044433, 0x00077654, 0x00088777}; /* MCS11 */
  for (int i = 0; i < 48; i++)
    _device.rtw_write32(static_cast<uint16_t>(0xD56C + i * 4), kTxDiff[i]);
  for (int i = 0; i < 48; i++)
    _device.rtw_write32(static_cast<uint16_t>(0xD62C + i * 4), 0x0);

  /* 10. halbb_spatial_reuse_init -> set_pwr_ref(SR_TXPWR_REF=21) (spatial_reuse.c:49). */
  field32(0xD23C, 0x15, 0x7F, 0);

  /* 11. halbb_macid_ctrl_init (halbb_pwr_ctrl.c:692) — clear 11 per-macid pwr CRs. */
  for (int i = 0; i < 11; i++)
    _device.rtw_write32(static_cast<uint16_t>(0xD36C + i * 4), 0x0);

  /* 13. halbb_statistics_init (halbb_statistics.c:310) — rate/mcs/nss/hdr-type
   * counters, all sub-fields of 0x0900 (RMW). */
  bb_rmw(0x0900, 0x0000000Fu, 0xB);   /* cnt rate = 6M spec */
  bb_rmw(0x0900, 0x000007F0u, 0);     /* HT MCS0 */
  bb_rmw(0x0900, 0x00007800u, 0);     /* VHT MCS0 */
  bb_rmw(0x0900, 0x00180000u, 0);     /* VHT NSS1 */
  bb_rmw(0x0900, 0x00078000u, 0);     /* HE MCS0 */
  bb_rmw(0x0900, 0x00600000u, 0);     /* HE NSS1 */
  bb_rmw(0x0900, 0x1F800000u, 0x08);  /* MAC hdr type = beacon */

  /* 15. halbb_psd_init -> halbb_psd_para_setting (halbb_psd.c:124), all @0x422C. */
  bb_rmw(0x422C, 0x18000u, 1);   /* n_dft (512-pt) */
  bb_rmw(0x422C, 0x1800u, 2);    /* iq_sel */
  bb_rmw(0x422C, 0x6000u, 3);    /* l_avg */
  bb_rmw(0x422C, 0x60000u, 0);   /* in_path_sel */
  bb_rmw(0x422C, 0x180000u, 3);  /* in_source_sel */

  /* 16. halbb_edcca_init (halbb_edcca.c:1113). */
  bb_rmw(0x0C70, 0x03F00000u, 0x29); /* r_collision_t2r_state */

  /* 18. halbb_dig_init -> cfg_bbcr(igi=28): dyn PD-th. OFDM bound>102 ->
   * disable lower bound; CCK bound=103. (8852A-only set_igi_cr not run.) */
  bb_rmw(0x481C, 1u << 29, 0);        /* OFDM PD lower-bound disable */
  bb_rmw(0x4b64, 0xff000000u, 0x99); /* CCK PD bound = -103 (s8) [31:24] */
  bb_rmw(0x4b64, 0x00ff0000u, 0x7f); /* [23:16] = 0x7f */
  bb_rmw(0x4b74, 1u << 30, 1);       /* CCK PD manual-en */

  /* 19. halbb_cfo_trk_init -> halbb_digital_cfo_comp_init (halbb_cfo_trk.c:171). */
  bb_rmw(0x4494, 1u << 29, 1);    /* cfo_comp_seg0_vld */
  bb_rmw(0x4490, 0x0F000000u, 8); /* cfo_wgting */
  field32(0xd248, 0x0, 0x7, 0);   /* 8852C comp-by-DCFO = 0 (pwr-reg plane) */

  /* 22. halbb_env_mntr_init (halbb_env_mntr.c:3895). */
  bb_rmw(0x0C00, 0x1u, 1);         /* ccx_en */
  bb_rmw(0x0C00, 0x2u, 0);         /* ccx_trig_opt */
  bb_rmw(0x0C00, 0x70u, 0);        /* ccx_edcca_opt */
  bb_rmw(0x4448, 0xE0000000u, 4);  /* ccx source sel (dccl, full BW) */
  bb_rmw(0x0C00, 0x1000u, 1);      /* clm_en */
  bb_rmw(0x0C08, 0x40000u, 1);     /* nhm_en */
  bb_rmw(0x0C14, 0x30000u, 1);     /* nhm pwdb method */
  bb_rmw(0x0C28, 0x1000u, 1);      /* ifs collect en */
  bb_rmw(0x0C2C, 0x8000u, 1);      /* ifs t1 en */
  bb_rmw(0x0C30, 0x8000u, 1);      /* ifs t2 en */
  bb_rmw(0x0C34, 0x8000u, 1);      /* ifs t3 en */
  bb_rmw(0x0C38, 0x8000u, 1);      /* ifs t4 en */
  bb_rmw(0x0C18, 0x1u, 1);         /* fahm_en */
  bb_rmw(0x0C18, 0x2u, 1);         /* fahm_en_ofdm */
  bb_rmw(0x0C18, 0x4u, 1);         /* fahm_en_cck */
  bb_rmw(0x0C1C, 0x70u, 1);        /* fahm pwdb sel */
  bb_rmw(0x0C28, 0x100u, 1);       /* fahm dis-count-each-mpdu */
  bb_rmw(0x0C04, 0x40000u, 1);     /* clm edcca en */

  /* 24. halbb_ch_info_init -> halbb_cfg_ch_info_cr (halbb_ch_info.c:490). */
  bb_rmw(0x025C, 0xFCu, 0x3A);        /* ch_info_en (src/cmprs/grp_num) */
  bb_rmw(0x0260, r::MASKDWORD, 0x303); /* ele_bitmap */
  bb_rmw(0x2000, 0x4000u, 1);         /* ch_info_type = MIMO_CH */
  bb_rmw(0x2008, 0x6000000u, 0);      /* seg_len */

  _logger->info("Kestrel PHY(8852C): halbb DM init applied "
                "(gpio/physts/dig/edcca/cfo/env/statistics/psd/chinfo)");
}

namespace {
/* count-trailing-zeros for the RMW field shift. */
inline uint8_t ctz32(uint32_t m) {
  uint8_t s = 0;
  while (m && !(m & 1)) {
    m >>= 1;
    ++s;
  }
  return s;
}
constexpr uint32_t BB_WIN = 0x10000; /* bb0_cr_offset (wIndex=1) */
} /* namespace */

void HalKestrel::bb_rmw(uint32_t addr, uint32_t mask, uint32_t val) {
  const uint32_t a = (addr & 0xffff) + BB_WIN;
  uint32_t v = _device.rtw_read32_wide(a);
  v = (v & ~mask) | ((val << ctz32(mask)) & mask);
  _device.rtw_write32_wide(a, v);
}

uint32_t HalKestrel::rf_read(uint8_t path, uint8_t rf_addr) {
  const uint32_t base = path == 0 ? 0xe000 : 0xf000;
  return _device.rtw_read32_wide(base + (rf_addr << 2) + BB_WIN) & 0xfffffu;
}

void HalKestrel::rf_write(uint8_t path, uint8_t rf_addr, uint32_t val) {
  const uint32_t base = path == 0 ? 0xe000 : 0xf000;
  _device.rtw_write32_wide(base + (rf_addr << 2) + BB_WIN, val & 0xfffffu);
  delay_us(1);
}

void HalKestrel::rf_write_dav(uint8_t path, uint8_t rf_addr, uint32_t val) {
  /* halbb_write_rf_reg_8852b_a: wait for the SI to idle (0x174c w/r busy),
   * then post a full-20-bit write command to 0x370 and confirm the readback. */
  for (int i = 0; i < 500; ++i) {
    uint32_t st = _device.rtw_read32_wide(0x174c + BB_WIN);
    if (!((st >> 24) & 0x1) && !((st >> 25) & 0x1))
      break;
    delay_us(1);
  }
  const uint32_t w = (static_cast<uint32_t>(path & 0x7) << 28) |
                     (static_cast<uint32_t>(rf_addr & 0xff) << 20) |
                     (val & 0xfffffu); /* b_msk_en=0 (full mask) */
  for (int i = 0; i < 500; ++i) {
    _device.rtw_write32_wide(0x370 + BB_WIN, w);
    delay_us(5);
    if (_device.rtw_read32_wide(0x370 + BB_WIN) == w)
      break;
  }
  delay_us(5);
}

uint32_t HalKestrel::rf_read_dav(uint8_t path, uint8_t rf_addr) {
  /* halbb_read_rf_reg_8852b_a: wait SI idle, post the read command to 0x378
   * (path<<8|addr, 11-bit), wait r_done (0x174c[26]), read the value from
   * 0x174c. */
  for (int i = 0; i < 500; ++i) {
    uint32_t st = _device.rtw_read32_wide(0x174c + BB_WIN);
    if (!((st >> 24) & 0x1) && !((st >> 25) & 0x1))
      break;
    delay_us(1);
  }
  const uint32_t r_reg =
      ((static_cast<uint32_t>(path & 0x7) << 8) | rf_addr) & 0x7ffu;
  for (int i = 0; i < 500; ++i) {
    _device.rtw_write32_wide(0x378 + BB_WIN,
                             (_device.rtw_read32_wide(0x378 + BB_WIN) & ~0x7ffu) |
                                 r_reg);
    delay_us(2);
    if ((_device.rtw_read32_wide(0x378 + BB_WIN) & 0x7ffu) == r_reg)
      break;
  }
  for (int j = 0; j < 500; ++j) {
    if ((_device.rtw_read32_wide(0x174c + BB_WIN) >> 26) & 0x1)
      break;
    delay_us(5);
  }
  return _device.rtw_read32_wide(0x174c + BB_WIN) & 0xfffffu;
}

uint32_t HalKestrel::rf_rrf(uint8_t path, uint32_t addr, uint32_t mask) {
  const uint8_t a = static_cast<uint8_t>(addr & 0xff);
  const uint32_t v = (addr & 0x10000u) ? rf_read(path, a) : rf_read_dav(path, a);
  return (v & mask) >> ctz32(mask);
}

void HalKestrel::rf_wrf(uint8_t path, uint32_t addr, uint32_t mask,
                        uint32_t val) {
  const uint8_t a = static_cast<uint8_t>(addr & 0xff);
  const bool ddv = (addr & 0x10000u) != 0; /* DDV=d-die window, else a-die */
  uint32_t w = val & r::MASKRF;
  if (mask != r::MASKRF) {
    /* masked RMW: read the current value from the same die, splice the field. */
    const uint32_t cur = ddv ? rf_read(path, a) : rf_read_dav(path, a);
    w = (cur & ~mask) | ((val << ctz32(mask)) & mask);
  }
  if (ddv)
    rf_write(path, a, w);
  else
    rf_write_dav(path, a, w);
}

void HalKestrel::rf_ctrl_ch(uint8_t channel, bool is_2g) {
  /* halrf_ctrl_ch_8852b: ch_setting for DAV (a-die, reg 0x18) then DDV (d-die,
   * reg 0x10018), each path A + B. DAV path A takes the synth-lock path
   * (halrf_set_s0_arfc18). */
  auto ch_setting = [&](uint8_t path, bool is_dav) {
    const uint32_t reg18 = is_dav ? 0x18u : 0x10018u;
    uint32_t rf18 = rf_rrf(path, reg18, r::MASKRF);
    rf18 &= ~0x3e3ffu;   /* [17:16],[9:8],[7:0] */
    rf18 |= channel;     /* channel */
    if (!is_2g)
      rf18 |= (1u << 16) | (1u << 8); /* 5G */
    rf18 = (rf18 & 0xf0fffu) | (1u << 12);
    if (path == 1 || !is_dav) {
      rf_wrf(path, reg18, r::MASKRF, rf18);
    } else {
      /* halrf_set_s0_arfc18_8852b: 0xd3[8]=1, write RF18 (a-die), poll the
       * synth LCK lock (0xb7[8]==0), 0xd3[8]=0. */
      rf_wrf(0, 0xd3, 1u << 8, 0x1);
      rf_wrf(0, 0x18, r::MASKRF, rf18);
      bool locked = false;
      for (int c = 0; c < 1000; ++c) {
        if (rf_rrf(0, 0xb7, 1u << 8) == 0) {
          locked = true;
          break;
        }
        delay_us(1);
      }
      if (!locked)
        _logger->warn("Kestrel RF: synth LCK lock timeout (path A ch{})",
                      channel);
      rf_wrf(0, 0xd3, 1u << 8, 0x0);
    }
    /* re-latch */
    rf_wrf(path, 0xcf, 1u << 0, 0x0);
    rf_wrf(path, 0xcf, 1u << 0, 0x1);
  };
  ch_setting(0, true); /* DAV path A (locked) */
  ch_setting(1, true); /* DAV path B */
  ch_setting(0, false); /* DDV path A */
  ch_setting(1, false); /* DDV path B */
}

void HalKestrel::rf_ctrl_bw(ChannelWidth_t bw) {
  /* halrf_bw_setting_8852b: RF18 [11:10] bandwidth bits, DAV (0x18) + DDV
   * (0x10018) x path A/B. 40 MHz=BIT11, 80 MHz=BIT10, 20/5/10=both. */
  auto bw_set = [&](uint8_t path, bool is_dav) {
    const uint32_t reg18 = is_dav ? 0x18u : 0x10018u;
    uint32_t v = rf_rrf(path, reg18, r::MASKRF);
    v &= ~((1u << 11) | (1u << 10));
    if (bw == CHANNEL_WIDTH_40)
      v |= (1u << 11);
    else if (bw == CHANNEL_WIDTH_80)
      v |= (1u << 10);
    else
      v |= (1u << 11) | (1u << 10); /* 20/5/10 */
    v = (v & 0xf0fffu) | (1u << 12);
    rf_wrf(path, reg18, r::MASKRF, v);
  };
  bw_set(0, true);  /* DAV path A */
  bw_set(1, true);  /* DAV path B */
  bw_set(0, false); /* DDV path A */
  bw_set(1, false); /* DDV path B */
}

/* Masked BB read (wIndex=1 window), value shifted down to the mask's LSB. */
static inline uint32_t mask_shift(uint32_t v, uint32_t mask) {
  return (v & mask) >> __builtin_ctz(mask);
}
uint32_t HalKestrel::bb_read(uint32_t addr, uint32_t mask) {
  return mask_shift(_device.rtw_read32_wide((addr & 0xffff) + BB_WIN), mask);
}

void HalKestrel::afe_init() {
  /* halrf_afe_init_8852b */
  _device.rtw_write32(0x8040, 0xf); /* halrf_wmac32 (MAC) */
  bb_rmw(0xc0d4, r::MASKDWORD, 0x4486888c);
  bb_rmw(0xc0d8, r::MASKDWORD, 0xc6ba10e0);
  bb_rmw(0xc0dc, r::MASKDWORD, 0x30c52868);
  bb_rmw(0xc0e0, r::MASKDWORD, 0x05008128);
  bb_rmw(0xc0e4, r::MASKDWORD, 0x0000272b);
  bb_rmw(0xc1d4, r::MASKDWORD, 0x4486888c);
  bb_rmw(0xc1d8, r::MASKDWORD, 0xc6ba10e0);
  bb_rmw(0xc1dc, r::MASKDWORD, 0x30c52868);
  bb_rmw(0xc1e0, r::MASKDWORD, 0x05008128);
  bb_rmw(0xc1e4, r::MASKDWORD, 0x0000272b);
}

void HalKestrel::dack_reset() {
  /* halrf_dack_reset_8852b: toggle BIT17 on each ADDCK/DADCK block. */
  for (uint32_t reg : {0xc000u, 0xc020u, 0xc100u, 0xc120u}) {
    bb_rmw(reg, 1u << 17, 0x0);
    bb_rmw(reg, 1u << 17, 0x1);
  }
}

void HalKestrel::drck() {
  /* halrf_drck_8852b: D-die RCK. Trigger 0xc0cc[6], poll 0xc0d0[3], then the
   * manual RCK writeback for LPS. */
  bb_rmw(0xc0cc, 1u << 6, 0x1);
  for (int c = 0; c < 10000 && bb_read(0xc0d0, 1u << 3) == 0; c++)
    delay_us(1);
  bb_rmw(0xc0cc, 1u << 6, 0x0);
  bb_rmw(0xc094, 1u << 9, 0x1);
  delay_us(1);
  bb_rmw(0xc094, 1u << 9, 0x0);
  const uint32_t rck_d = bb_read(0xc0d0, 0xf8000);
  bb_rmw(0xc0cc, 1u << 9, 0x0); /* RCK_SEL=0 */
  bb_rmw(0xc0cc, 0x1f, rck_d);
}

void HalKestrel::addck() {
  /* halrf_addck_8852b: ADC DC-offset calibration, per path. check_addc is
   * debug-only (skipped). */
  struct Path {
    uint32_t b8, d4, f4, fc;
  };
  const Path p[2] = {{0x12b8, 0xc0d4, 0xc0f4, 0xc0fc},
                     {0x32b8, 0xc1d4, 0xc1f4, 0xc1fc}};
  bb_rmw(0xc0f4, 0x30, 0x0); /* manual off (both) */
  bb_rmw(0xc1f4, 0x30, 0x0);
  for (int i = 0; i < 2; i++) {
    bb_rmw(p[i].b8, 1u << 30, 0x1); /* ADC & clk enable */
    bb_rmw(0x032c, 1u << 30, 0x0);
    bb_rmw(0x032c, 1u << 22, 0x0); /* reset calibration */
    bb_rmw(0x032c, 1u << 22, 0x1);
    bb_rmw(0x030c, 0x0f000000, 0xf);
    delay_us(100);
    bb_rmw(0x032c, 1u << 16, 0x0); /* ADC input short */
    bb_rmw(p[i].d4, 1u << 1, 0x1);
    bb_rmw(0x030c, 0x0f000000, 0x3); /* release ADC reset */
    bb_rmw(p[i].f4, 1u << 11, 0x1);  /* trigger DC-offset cal */
    bb_rmw(p[i].f4, 1u << 11, 0x0);
    delay_us(1);
    bb_rmw(p[i].f4, 0x300, 0x1); /* check-done select */
    int c = 0;
    for (; c < 10000 && bb_read(p[i].fc, 1u << 0) == 0; c++)
      delay_us(1);
    if (c >= 10000)
      _logger->warn("Kestrel ADDCK: S{} timeout", i);
    /* restore */
    bb_rmw(p[i].d4, 1u << 1, 0x0);
    bb_rmw(0x032c, 1u << 16, 0x1);
    bb_rmw(0x030c, 0x0f000000, 0xc);
    bb_rmw(0x032c, 1u << 30, 0x1);
    /* backup the measured offsets */
    bb_rmw(p[i].f4, 0x300, 0x0);
    _addck_d[i][0] = static_cast<uint16_t>(bb_read(p[i].fc, 0xffc00));
    _addck_d[i][1] = static_cast<uint16_t>(bb_read(p[i].fc, 0x003ff));
    bb_rmw(p[i].b8, 1u << 30, 0x0);
  }
}

void HalKestrel::addck_reload() {
  /* halrf_addck_reload_8852b: apply the measured ADC DC offsets. */
  const uint32_t f0[2] = {0xc0f0, 0xc1f0}, f4[2] = {0xc0f4, 0xc1f4};
  for (int i = 0; i < 2; i++) {
    bb_rmw(f0[i], 0x3ff0000, _addck_d[i][0]);
    bb_rmw(f4[i], 0xf, static_cast<uint32_t>(_addck_d[i][1]) >> 6);
    bb_rmw(f0[i], 0xfc000000, static_cast<uint32_t>(_addck_d[i][1]) & 0x3f);
    bb_rmw(f4[i], 0x30, 0x3); /* manual */
  }
}

void HalKestrel::dack() {
  /* halrf_dack_8852b_s0/s1: DAC-side MSBK (multi-stage-bit) + DADCK calibration,
   * per path. Auto-mode: the hardware runs the cal and applies the corrections
   * (0xc004[0]/0xc104[0] = auto-on); the optional result backup (for reload-
   * without-remeasure) is skipped. Reduces DAC DC / carrier leakage. */
  struct P {
    uint32_t a0, b8, d8, dc, c04, c24, c0c, c2c, msbk1, msbk2, dadck1, dadck2;
  };
  const P p[2] = {
      {0x12a0, 0x12b8, 0xc0d8, 0xc0dc, 0xc004, 0xc024, 0xc00c, 0xc02c, 0xc040,
       0xc064, 0xc05c, 0xc080},
      {0x32a0, 0x32b8, 0xc1d8, 0xc1dc, 0xc104, 0xc124, 0xc10c, 0xc12c, 0xc140,
       0xc164, 0xc15c, 0xc180}};
  for (int i = 0; i < 2; i++) {
    const P &r = p[i];
    bb_rmw(r.a0, 1u << 15, 1);
    bb_rmw(r.a0, 0x7000, 3);
    bb_rmw(r.b8, 1u << 30, 1);
    bb_rmw(0x030c, 1u << 28, 1);
    bb_rmw(0x032c, 0x80000000, 0);
    delay_us(100);
    bb_rmw(r.d8, 1u << 16, 1);
    bb_rmw(r.dc, (1u << 27) | (1u << 26), 3);
    bb_rmw(r.c04, 1u << 30, 0);
    bb_rmw(r.c24, 1u << 30, 0);
    bb_rmw(r.c04, 0x3ff00000, 0x30);
    bb_rmw(r.c04, (1u << 31) | (1u << 30), 0);
    bb_rmw(r.c04, 1u << 17, 1);
    bb_rmw(r.c24, 1u << 17, 1);
    bb_rmw(r.c0c, 1u << 2, 0);
    bb_rmw(r.c2c, 1u << 2, 0);
    bb_rmw(r.c04, 1u << 0, 1); /* MSBK auto on */
    bb_rmw(r.c24, 1u << 0, 1);
    delay_us(1);
    int c = 0;
    for (; c < 10000 &&
           (bb_read(r.msbk1, 1u << 31) == 0 || bb_read(r.msbk2, 1u << 31) == 0);
         c++)
      delay_us(1);
    if (c >= 10000)
      _logger->warn("Kestrel DACK: S{} MSBK timeout", i);
    bb_rmw(r.c0c, 1u << 2, 1); /* DADCK trigger */
    bb_rmw(r.c2c, 1u << 2, 1);
    c = 0;
    for (; c < 10000 &&
           (bb_read(r.dadck1, 1u << 2) == 0 || bb_read(r.dadck2, 1u << 2) == 0);
         c++)
      delay_us(1);
    if (c >= 10000)
      _logger->warn("Kestrel DACK: S{} DADCK timeout", i);
    bb_rmw(r.c04, 1u << 0, 0); /* auto off */
    bb_rmw(r.c24, 1u << 0, 0);
    bb_rmw(r.d8, 1u << 16, 0);
    bb_rmw(r.a0, 1u << 15, 0);
    bb_rmw(r.a0, 0x7000, 7);
    bb_rmw(r.b8, 1u << 30, 0);
  }
}

void HalKestrel::dac_cal() {
  if (_variant == ChipVariant::C8852C) {
    dac_cal_8852c(); /* the 8852b sequence times out on the 8852C */
    return;
  }
  /* halrf_dac_cal_8852b — ADC/ADDCK subset (DAC-side MSBK/biask not ported).
   * Save the RF registers the cal disturbs so operational radio state survives
   * (the vendor relies on later steps to rewrite 0x0/0x1; devourer's channel
   * set does not). */
  const uint32_t rf0a = rf_rrf(0, 0x0, r::MASKRF), rf0b = rf_rrf(1, 0x0, r::MASKRF);
  const uint32_t rf1a = rf_rrf(0, 0x1, r::MASKRF), rf1b = rf_rrf(1, 0x1, r::MASKRF);
  const uint32_t rf5a = rf_rrf(0, 0x5, r::MASKRF), rf5b = rf_rrf(1, 0x5, r::MASKRF);
  afe_init();
  dack_reset();
  drck();
  rf_wrf(0, 0x5, r::MASKRF, 0x0);
  rf_wrf(1, 0x5, r::MASKRF, 0x0);
  rf_wrf(0, 0x0, r::MASKRF, 0x337e1);
  rf_wrf(1, 0x0, r::MASKRF, 0x337e1);
  addck();
  addck_reload();
  rf_wrf(0, 0x1, r::MASKRF, 0x0);
  rf_wrf(1, 0x1, r::MASKRF, 0x0);
  dack(); /* DAC-side MSBK + DADCK (both paths) */
  /* restore operational RF state */
  rf_wrf(0, 0x0, r::MASKRF, rf0a);
  rf_wrf(1, 0x0, r::MASKRF, rf0b);
  rf_wrf(0, 0x1, r::MASKRF, rf1a);
  rf_wrf(1, 0x1, r::MASKRF, rf1b);
  rf_wrf(0, 0x5, r::MASKRF, rf5a);
  rf_wrf(1, 0x5, r::MASKRF, rf5b);
  _logger->info("Kestrel RF: DACK done (ADDCK S0 ic=0x{:x} qc=0x{:x}, S1 ic=0x{:x}"
                " qc=0x{:x}; MSBK+DADCK both paths)",
                _addck_d[0][0], _addck_d[0][1], _addck_d[1][0], _addck_d[1][1]);
}

/* ======================= 8852C DACK / ADDCK =============================
 * Ported verbatim from reference/rtl8852cu halrf_8852c/halrf_dack_8852c.c +
 * halrf_8852c_api.c. The 8852b-derived cal above uses different ADC-DC-cal
 * registers and times out on the 8852C, desensing its RX. halrf_wreg/rreg map
 * to bb_rmw/bb_read (BB window); halrf_wrf/rrf to rf_wrf/rf_rrf; halrf_w32 to a
 * full-dword BB write. Clock codes: ADC_960M=2, DAC_160M=3, DAC_960M=7. */

void HalKestrel::txck_force_8852c(uint8_t path, bool force, uint8_t ck) {
  const uint32_t reg = 0x12a0u | (static_cast<uint32_t>(path) << 13);
  bb_rmw(reg, 1u << 15, 0x0);
  if (!force)
    return;
  bb_rmw(reg, 0x7000, ck);
  bb_rmw(reg, 1u << 15, 0x1);
}

void HalKestrel::rxck_force_8852c(uint8_t path, bool force, uint8_t ck) {
  const uint32_t reg = 0x12a0u | (static_cast<uint32_t>(path) << 13);
  bb_rmw(reg, 1u << 19, 0x0);
  if (!force)
    return;
  bb_rmw(reg, 0x70000, ck);
  bb_rmw(reg, 1u << 19, 0x1);
  /* halrf_rxck_force_8852c: ck -> ADC bandwidth (ADC_960M=2 -> 80 MHz). */
  ChannelWidth_t bw = CHANNEL_WIDTH_80;
  if (ck == 1) /* ADC_480M */
    bw = CHANNEL_WIDTH_40;
  else if (ck == 3) /* ADC_1920M */
    bw = CHANNEL_WIDTH_160;
  adc_cfg_8852c(bw, path);
}

void HalKestrel::adc_cfg_8852c(ChannelWidth_t bw, uint8_t path) {
  /* halbb_adc_cfg_8852c (halbb_8852c_api.c). is_efem=false (external-FEM only
   * affects the 20/40 adc_rst_cycle branch). */
  const uint32_t idac2 = path ? 0xC1D4 : 0xC0D4;
  const uint32_t adc_op5_bw_sel = path ? 0xC1D8 : 0xC0D8;
  const uint32_t rck_offset = path ? 0xC1C4 : 0xC0C4;
  const uint32_t rck_reset_count = path ? 0xC1E8 : 0xC0E8;
  const uint32_t wbadc_sel = path ? 0xC1E4 : 0xC0E4;
  const uint32_t rx_adc_clk = path ? 0x32A0 : 0x12A0;
  const uint32_t decim_filter = path ? 0xC1EC : 0xC0EC;
  const uint32_t adc_rst_cycle = decim_filter;
  const uint32_t upd_clk_adc = path ? 0x766C : 0x566C;
  const uint32_t agc_restart_th_ib_l = path ? 0x4CAC : 0x4BE8;
  const uint32_t agc_restart_th_ib_bw = path ? 0x4CA8 : 0x4BE4;

  bb_rmw(idac2, 0x780, 0x8);
  bb_rmw(rck_reset_count, 0xFFFF0000, 0x9);
  bb_rmw(wbadc_sel, 0x30, 0x2);
  bb_rmw(rx_adc_clk, 0xFF800000, 0x49);
  bb_rmw(decim_filter, 0x6000, 0x0);

  if (bw == CHANNEL_WIDTH_80) {
    bb_rmw(idac2, 0x7800, 0x2); /* idac2_1 */
    bb_rmw(idac2, 0xC000000, 0x2); /* adc_sample_td */
    bb_rmw(adc_op5_bw_sel, 0x1E0, 0x8);
    bb_rmw(rck_offset, 0x3E0000, 0x0);
    bb_rmw(adc_rst_cycle, 0xFF0000, 0x3);
    bb_rmw(upd_clk_adc, (1u << 17) | (1u << 16), 1);
    bb_rmw(agc_restart_th_ib_l, 0xf, bb_read(agc_restart_th_ib_bw, 0x78000000));
  } else if (bw == CHANNEL_WIDTH_160) {
    bb_rmw(idac2, 0x7800, 0x0);
    bb_rmw(idac2, 0xC000000, 0x2);
    bb_rmw(adc_op5_bw_sel, 0x1E0, 0x4);
    bb_rmw(rck_offset, 0x3E0000, 0x6);
    bb_rmw(adc_rst_cycle, 0xFF0000, 0x3);
    bb_rmw(upd_clk_adc, (1u << 17) | (1u << 16), 2);
  } else { /* 5/10/20/40: ADC clk 80M, WB ADC 160M */
    bb_rmw(idac2, 0x7800, 0x2);
    bb_rmw(idac2, 0xC000000, 0x3);
    bb_rmw(adc_op5_bw_sel, 0x1E0, 0xf);
    bb_rmw(rck_offset, 0x3E0000, 0x0);
    bb_rmw(upd_clk_adc, (1u << 17) | (1u << 16), 0);
    bb_rmw(adc_rst_cycle, 0xFF0000, 0x3); /* is_efem=false */
    if (bw == CHANNEL_WIDTH_40)
      bb_rmw(agc_restart_th_ib_l, 0xf, bb_read(agc_restart_th_ib_bw, 0x7800000));
    else
      bb_rmw(agc_restart_th_ib_l, 0xf, bb_read(agc_restart_th_ib_bw, 0x780000));
  }
}

void HalKestrel::dack_reset_8852c(uint8_t path) {
  const uint32_t reg = path ? 0xc100 : 0xc000;
  bb_rmw(reg, 1u << 17, 0x0);
  bb_rmw(reg, 1u << 17, 0x1);
}

void HalKestrel::drck_8852c() {
  /* halrf_drck_8852c: Ddie RCK. Trigger 0xc0c4[6], poll 0xc0c8[3], pulse
   * 0xc094[9], then the manual RCK writeback for LPS (rck_d from 0xc0c8[14:10]
   * into 0xc0c4[4:0], RCK_SEL 0xc0c4[9]=0). */
  bb_rmw(0xc0c4, 1u << 6, 0x1);
  for (int c = 0; c < 10000 && bb_read(0xc0c8, 1u << 3) == 0; c++)
    delay_us(1);
  bb_rmw(0xc0c4, 1u << 6, 0x0);
  bb_rmw(0xc094, 1u << 9, 0x1);
  delay_us(1);
  bb_rmw(0xc094, 1u << 9, 0x0);
  const uint32_t rck_d = bb_read(0xc0c8, 0x7c00);
  bb_rmw(0xc0c4, 1u << 9, 0x0); /* RCK_SEL=0 */
  bb_rmw(0xc0c4, 0x1f, rck_d);
}

void HalKestrel::dack_manual_off_8852c() {
  /* halrf_dack_manual_off_8852c. */
  bb_rmw(0xc0f8, 0x30000000, 0x0);
  bb_rmw(0xc1f8, 0x30000000, 0x0);
  for (uint32_t reg : {0xc210u, 0xc224u, 0xc238u, 0xc24cu})
    bb_rmw(reg, 1u << 0, 0x0);
}

void HalKestrel::addck_ori_8852c() {
  /* halrf_addck_ori_8852c: ADC DC-offset cal (the ORI variant dac_cal uses).
   * Per path: reset cal (0x032c[22]), short the ADC input (0x032c[16]=0,
   * 0xcXd4[4]=1), trigger (0xcXf4[11]), poll done (0xcXfc[0]), restore, and
   * back up the measured offsets from 0xcXfc into _addck_d. */
  struct P {
    uint32_t b8, d4, f4, fc;
  };
  const P p[2] = {{0x12b8, 0xc0d4, 0xc0f4, 0xc0fc},
                  {0x32b8, 0xc1d4, 0xc1f4, 0xc1fc}};
  bb_rmw(0xc0f4, 0x30, 0x0); /* manual off (both) */
  bb_rmw(0xc1f4, 0x30, 0x0);
  for (int i = 0; i < 2; i++) {
    bb_rmw(p[i].b8, 1u << 30, 0x1); /* ADC & clk enable */
    bb_rmw(0x032c, 1u << 30, 0x0);
    bb_rmw(0x032c, 1u << 22, 0x0); /* reset calibration */
    bb_rmw(0x032c, 1u << 22, 0x1);
    bb_rmw(0x030c, 0x0f000000, 0xf);
    delay_us(100);
    bb_rmw(0x032c, 1u << 16, 0x0); /* ADC input short */
    bb_rmw(p[i].d4, 1u << 4, 0x1);
    bb_rmw(0x030c, 0x0f000000, 0x3); /* release ADC reset */
    bb_rmw(p[i].f4, 1u << 11, 0x1);  /* trigger DC-offset cal */
    bb_rmw(p[i].f4, 1u << 11, 0x0);
    delay_us(1);
    bb_rmw(p[i].f4, 0x300, 0x1); /* check-done select */
    int c = 0;
    for (; c < 10000 && bb_read(p[i].fc, 1u << 0) == 0; c++)
      delay_us(1);
    if (c >= 10000)
      _logger->warn("Kestrel ADDCK(8852C): S{} timeout", i);
    /* restore */
    bb_rmw(p[i].d4, 1u << 4, 0x0);
    bb_rmw(0x032c, 1u << 16, 0x1);
    bb_rmw(0x030c, 0x0f000000, 0xc);
    bb_rmw(0x032c, 1u << 30, 0x1);
    /* backup the measured offsets */
    bb_rmw(p[i].f4, 0x300, 0x0);
    _addck_d[i][0] = static_cast<uint16_t>(bb_read(p[i].fc, 0xffc00));
    _addck_d[i][1] = static_cast<uint16_t>(bb_read(p[i].fc, 0x003ff));
    bb_rmw(p[i].b8, 1u << 30, 0x0);
  }
}

void HalKestrel::addck_reload_8852c() {
  /* halrf_addck_reload_8852c: apply the measured ADC DC offsets (0xcXf8). */
  const uint32_t f8[2] = {0xc0f8, 0xc1f8};
  for (int i = 0; i < 2; i++) {
    bb_rmw(f8[i], 0x0ffc0000, _addck_d[i][0]);
    bb_rmw(f8[i], 0x0003ff00, _addck_d[i][1]);
    bb_rmw(f8[i], 0x30000000, 0x3); /* manual */
  }
}

void HalKestrel::dack_backup_8852c(uint8_t path) {
  /* halrf_dack_backup_s0/s1_8852c: read back MSBK / biasK / DADCK per path. */
  bb_rmw(path ? 0x32b8 : 0x12b8, 1u << 30, 0x1);
  const uint32_t k0 = path ? 0xc100 : 0xc000; /* ic index reg */
  const uint32_t k1 = path ? 0xc120 : 0xc020; /* qc index reg */
  const uint32_t m0 = path ? 0xc15c : 0xc05c; /* ic msbk read */
  const uint32_t m1 = path ? 0xc180 : 0xc080; /* qc msbk read */
  for (uint8_t i = 0; i < 0x10; i++) {
    bb_rmw(k0, 0x1e, i);
    _msbk_d[path][0][i] = static_cast<uint8_t>(bb_read(m0, 0xff000000));
    bb_rmw(k1, 0x1e, i);
    _msbk_d[path][1][i] = static_cast<uint8_t>(bb_read(m1, 0xff000000));
  }
  _biask_d[path][0] = static_cast<uint16_t>(bb_read(path ? 0xc148 : 0xc048, 0xffc));
  _biask_d[path][1] = static_cast<uint16_t>(bb_read(path ? 0xc16c : 0xc06c, 0xffc));
  _dadck_d[path][0] = static_cast<uint8_t>(bb_read(path ? 0xc160 : 0xc060, 0xff000000));
  _dadck_d[path][1] = static_cast<uint8_t>(bb_read(path ? 0xc184 : 0xc084, 0xff000000));
}

void HalKestrel::dack_reload_8852c(uint8_t path) {
  /* halrf_dack_reload_by_path_8852c for index 0 and 1: write the backed-up
   * MSBK/DADCK/biasK into the 0xc200-block reg table + enable "result from
   * reg". temp_offset = (index?0x14:0) + (path?0x28:0). */
  const uint32_t path_off = path ? 0x28 : 0;
  for (uint8_t index = 0; index < 2; index++) {
    const uint32_t off = (index ? 0x14 : 0) + path_off;
    bb_rmw(0xc004, 1u << 17, 0x1);
    bb_rmw(0xc024, 1u << 17, 0x1);
    bb_rmw(0xc104, 1u << 17, 0x1);
    bb_rmw(0xc124, 1u << 17, 0x1);
    auto pack = [&](int base) {
      uint32_t t = 0;
      for (int i = 0; i < 4; i++)
        t |= static_cast<uint32_t>(_msbk_d[path][index][base + i]) << (i * 8);
      return t;
    };
    bb_rmw(0xc200 + off, r::MASKDWORD, pack(12)); /* msbk 15..12 */
    bb_rmw(0xc204 + off, r::MASKDWORD, pack(8));  /* msbk 11..8 */
    bb_rmw(0xc208 + off, r::MASKDWORD, pack(4));  /* msbk 7..4 */
    bb_rmw(0xc20c + off, r::MASKDWORD, pack(0));  /* msbk 3..0 */
    const uint32_t dadck_biask =
        (static_cast<uint32_t>(_biask_d[path][index]) << 22) |
        (static_cast<uint32_t>(_dadck_d[path][index]) << 14);
    bb_rmw(0xc210 + off, r::MASKDWORD, dadck_biask);
    bb_rmw(0xc210 + off, 1u << 0, 0x1); /* enable DACK result from reg */
  }
}

void HalKestrel::dack_8852c() {
  /* halrf_dack_8852c s0/s1: DAC-side MSBK + DADCK. Force 160 MHz DAC clk for
   * the cal, enable DACK (0xcX04[0]), poll the four done bits, restore 960 MHz,
   * back up + reload the result table. */
  struct S {
    uint8_t path;
    uint32_t b8, c04, c24, done_a, done_b, done_c, done_d;
  };
  const S s[2] = {{0, 0x12b8, 0xc004, 0xc024, 0xc040, 0xc064, 0xc05c, 0xc080},
                  {1, 0x32b8, 0xc104, 0xc124, 0xc140, 0xc164, 0xc15c, 0xc180}};
  for (const S &c : s) {
    txck_force_8852c(c.path, true, 3 /* DAC_160M */);
    bb_rmw(c.b8, 1u << 30, 0x1);
    bb_rmw(0x030c, 1u << 28, 0x1);
    bb_rmw(0x032c, 0x80000000, 0x0);
    delay_us(100);
    bb_rmw(c.c04, 0xfff00000, 0x30);
    bb_rmw(c.c24, 0xfff00000, 0x30);
    dack_reset_8852c(c.path);
    bb_rmw(c.c04, 1u << 0, 0x1); /* enable DACK */
    delay_us(1);
    int n = 0;
    for (; n < 10000 &&
           (bb_read(c.done_a, 1u << 31) == 0 || bb_read(c.done_b, 1u << 31) == 0 ||
            bb_read(c.done_c, 1u << 2) == 0 || bb_read(c.done_d, 1u << 2) == 0);
         n++)
      delay_us(1);
    if (n >= 10000)
      _logger->warn("Kestrel DACK(8852C): S{} MSBK/DADCK timeout", c.path);
    bb_rmw(c.c04, 1u << 0, 0x0); /* disable DACK */
    txck_force_8852c(c.path, false, 7 /* DAC_960M */);
    dack_backup_8852c(c.path);
    dack_reload_8852c(c.path);
    bb_rmw(c.b8, 1u << 30, 0x0);
  }
}

void HalKestrel::dac_cal_8852c() {
  /* halrf_dac_cal_8852c orchestration. RF 0x0/0x1/0x5 saved/restored so the
   * operational radio state survives the cal (devourer-side, as for 8852b). */
  const uint32_t rf0a = rf_rrf(0, 0x0, r::MASKRF), rf0b = rf_rrf(1, 0x0, r::MASKRF);
  const uint32_t rf1a = rf_rrf(0, 0x1, r::MASKRF), rf1b = rf_rrf(1, 0x1, r::MASKRF);
  const uint32_t rf5a = rf_rrf(0, 0x5, r::MASKRF), rf5b = rf_rrf(1, 0x5, r::MASKRF);
  /* halrf_afe_init_8852c is a no-op (body #if0'd). */
  drck_8852c();
  dack_manual_off_8852c();
  rf_wrf(0, 0x5, r::MASKRF, 0x0);
  rf_wrf(1, 0x5, r::MASKRF, 0x0);
  rf_wrf(0, 0x0, r::MASKRF, 0x337e1);
  rf_wrf(1, 0x0, r::MASKRF, 0x337e1);
  rxck_force_8852c(0, true, 2 /* ADC_960M */);
  rxck_force_8852c(1, true, 2);
  addck_ori_8852c();
  rxck_force_8852c(0, false, 2);
  rxck_force_8852c(1, false, 2);
  addck_reload_8852c();
  rf_wrf(0, 0x1, r::MASKRF, 0x0);
  rf_wrf(1, 0x1, r::MASKRF, 0x0);
  dack_8852c();
  /* restore operational RF state */
  rf_wrf(0, 0x0, r::MASKRF, rf0a);
  rf_wrf(1, 0x0, r::MASKRF, rf0b);
  rf_wrf(0, 0x1, r::MASKRF, rf1a);
  rf_wrf(1, 0x1, r::MASKRF, rf1b);
  rf_wrf(0, 0x5, r::MASKRF, rf5a);
  rf_wrf(1, 0x5, r::MASKRF, rf5b);
  _logger->info("Kestrel RF(8852C): DACK done (ADDCK S0 ic=0x{:x} qc=0x{:x}, "
                "S1 ic=0x{:x} qc=0x{:x}; MSBK+DADCK+biasK both paths)",
                _addck_d[0][0], _addck_d[0][1], _addck_d[1][0], _addck_d[1][1]);
}

uint8_t HalKestrel::read_thermal(uint8_t path) {
  /* halrf_get_thermal_8852b: pulse RF 0x42[19] (1-0-1), settle, read [6:1]. */
  rf_wrf(path, 0x42, 1u << 19, 0x1);
  rf_wrf(path, 0x42, 1u << 19, 0x0);
  rf_wrf(path, 0x42, 1u << 19, 0x1);
  delay_us(200);
  return static_cast<uint8_t>(rf_rrf(path, 0x42, 0x7e));
}

void HalKestrel::rx_dck_toggle_8852c(uint8_t path) {
  /* halrf_rx_dck_toggle_8852c: edge 0x92[0], poll 0x93[5] done (<=500x2us). */
  rf_wrf(path, 0x92, 1u << 0, 0x0);
  rf_wrf(path, 0x92, 1u << 0, 0x1);
  int cnt = 0;
  for (; cnt < 500 && rf_rrf(path, 0x93, 1u << 5) == 0; cnt++)
    delay_us(2);
  rf_wrf(path, 0x92, 1u << 0, 0x0);
}

void HalKestrel::set_rx_dck_8852c(uint8_t path) {
  /* halrf_set_rx_dck_8852c: DC source from RFC (0x93[3:0]=0), toggle, then the
   * is_auto_res step (default true): read the resolution 0x92[7:5]; if >1, load
   * it into 0x8f[11:9], re-toggle, and restore 0x8f[11:9]=1. */
  rf_wrf(path, 0x93, 0xfu, 0x0);
  rx_dck_toggle_8852c(path);
  const uint32_t res = rf_rrf(path, 0x92, 0xe0u); /* [7:5] */
  if (res > 1) {
    rf_wrf(path, 0x8f, 0xe00u, res); /* [11:9] */
    rx_dck_toggle_8852c(path);
    rf_wrf(path, 0x8f, 0xe00u, 0x1);
  }
}

void HalKestrel::rx_dck_8852c() {
  /* halrf_rx_dck_8852c, RFC (non-AFE) path, non-MP, no TSSI, dbcc off
   * (kpath=RF_AB). Per path: save 0x5 (+ 0x0 when 0x5[0]=0), force RF_RX, run
   * the poll-based DC calibration, restore. The rfe>=50 rek-check retry loop is
   * a reliability refinement (not ported); the core single-pass cal runs. */
  for (uint8_t path = 0; path < 2; path++) {
    const uint32_t rf_5 = rf_rrf(path, 0x5, r::MASKRF);
    uint32_t rf_0 = 0;
    if ((rf_5 & 0x1) == 0)
      rf_0 = rf_rrf(path, 0x0, r::MASKRF);
    rf_wrf(path, 0x5, 1u << 0, 0x0);
    rf_wrf(path, 0x00, 0xf0000u, 0x3); /* MASKRFMODE = RF_RX */
    rf_wrf(path, 0x00, 1u << 1, 0x0);  /* dbcc_en = 0 */
    set_rx_dck_8852c(path);
    rf_wrf(path, 0x5, r::MASKRF, rf_5); /* restore */
    if ((rf_5 & 0x1) == 0)
      rf_wrf(path, 0x0, r::MASKRF, rf_0);
  }
  _logger->info("Kestrel RF(8852C): RX-DCK done (both paths)");
}

void HalKestrel::rx_dck() {
  if (_variant == ChipVariant::C8852C) {
    rx_dck_8852c(); /* the 8852b RX-DCK uses different regs on the 8852C */
    return;
  }
  /* halrf_rx_dck_8852b, RFC (non-AFE) path, no TSSI (devourer isn't in TSSI
   * mode). Per RF path: save 0x5 + the 0x92[1] dck_tune, force RF_RX mode, run
   * the RFC-based DC calibration (0x93[3:0]=0 = source from RFC, then an edge on
   * 0x92[0]) with a 600 us settle, then restore. MASKRFMODE=0xf0000, RF_RX=0x3.
   * All a-die (DAV) RF registers. */
  for (uint8_t path = 0; path < 2; path++) {
    const uint32_t rf_reg5 = rf_rrf(path, 0x5, r::MASKRF);
    const uint32_t dck_tune = rf_rrf(path, 0x92, 1u << 1);
    rf_wrf(path, 0x5, 1u << 0, 0x0);
    rf_wrf(path, 0x92, 1u << 1, 0x0);
    rf_wrf(path, 0x00, 0xf0000u, 0x3); /* MASKRFMODE = RF_RX */
    rf_wrf(path, 0x93, 0xfu, 0x0);     /* DC source: from RFC */
    rf_wrf(path, 0x92, 1u << 0, 0x0);  /* trigger edge */
    rf_wrf(path, 0x92, 1u << 0, 0x1);
    delay_us(600); /* settle (vendor: 30 x 20 us) */
    rf_wrf(path, 0x92, 1u << 1, dck_tune); /* restore */
    rf_wrf(path, 0x5, r::MASKRF, rf_reg5);
  }
  _logger->info("Kestrel RF: RX-DCK done (both paths)");
}

void HalKestrel::set_txpwr_dbm(int16_t dbm_q2) {
  /* halbb_set_txpwr_dbm_8852b (halbb_8852b_api.c): arm fixed-dBm mode, then
   * write the s(9,2) target into the 9-bit field. bb_rmw shifts the value into
   * the masked field. */
  bb_rmw(0x09a4, 1u << 16, 1);
  bb_rmw(0x4594, 0x7fc00000u, static_cast<uint32_t>(dbm_q2) & 0x1ffu);
}

void HalKestrel::fast_retune(uint8_t channel) {
  /* Lean same-band, 20 MHz retune: only the RF synth channel changes. Re-apply
   * the band-specific gain/RPL (the 5 GHz sub-band may shift, e.g. 5G-L->5G-H)
   * and the fixed TX power, then a BB reset — but skip the BB bandwidth config
   * (unchanged) and the per-channel RX-DCK (the slow ~1.2 ms cal; the RX DC term
   * is stable within a band). ~a few ms vs ~90 ms for the full set_channel. */
  const bool is_2g = channel <= 14;
  rf_ctrl_ch(channel, is_2g);
  set_gain_error(channel);
  set_rxsc_rpl_comp(channel);
  bb_reset_all();
  set_txpwr_dbm(static_cast<int16_t>(_txpwr_dbm_q2 + _txpwr_offset_qdb));
  _logger->debug("Kestrel FastRetune: ch{} ({})", channel, is_2g ? "2.4G" : "5G");
}

void HalKestrel::fast_set_bw(ChannelWidth_t bw) {
  /* Lean 20 <-> 5/10 MHz toggle (same channel). Only the BB small-BW field
   * (0x49C4[13:12]) + ACI-detect (0x4738/0x4AA4[16]) differ between 20 MHz and
   * narrowband; RF_BW stays 0 and RF18[11:10] stays "both" for all of 20/5/10,
   * so no RF re-tune or gain re-apply is needed. */
  const uint32_t smallbw = bw == CHANNEL_WIDTH_5    ? 0x1
                           : bw == CHANNEL_WIDTH_10 ? 0x2
                                                    : 0x0;      /* 20 MHz */
  const uint32_t aci = bw == CHANNEL_WIDTH_20 ? 0x1 : 0x0;      /* off for NB */
  bb_rmw(0x49C0, 0xC0000000u, 0x0); /* RF_BW = 0 */
  bb_rmw(0x49C4, 0x3000u, smallbw);
  bb_rmw(0x4738, 0x10000u, aci);
  bb_rmw(0x4AA4, 0x10000u, aci);
  bb_reset_all();
  const int mhz = bw == CHANNEL_WIDTH_5 ? 5 : bw == CHANNEL_WIDTH_10 ? 10 : 20;
  _logger->debug("Kestrel FastSetBandwidth: {} MHz", mhz);
}

void HalKestrel::bb_reset_all() {
  /* halbb_bb_reset_all_8852b (BB regs over wIndex=1; 0xce40 is MAC/CMAC). */
  bb_rmw(0x2344, 1u << 31, 1); /* PD disable */
  bb_rmw(0xc3c, 1u << 9, 1);
  bb_rmw(0x1200, 0x7u << 28, 0x7); /* protect SW-SI */
  bb_rmw(0x3200, 0x7u << 28, 0x7);
  delay_us(1);
  uint8_t v = _device.rtw_read8(0xce40); /* stop phy-sts update (MAC space) */
  _device.rtw_write8(0xce40, static_cast<uint8_t>(v & ~0x1));
  delay_us(2);
  bb_rmw(0x704, 1u << 1, 1); /* BB reset toggle */
  bb_rmw(0x704, 1u << 1, 0);
  bb_rmw(0x1200, 0x7u << 28, 0x0);
  bb_rmw(0x3200, 0x7u << 28, 0x0);
  bb_rmw(0x704, 1u << 1, 1);
  if (_variant == ChipVariant::C8852C) {
    /* cfg_phy_rpt (phy_rpt.c) for the 8852C, matching the working vendor
     * driver's R_AX_PPDU_STAT (0xCE40) = 0x0B00020F: RPT_EN(0) | APP_MAC_INFO(1)
     * | APP_RX_CNT(2) | APP_PLCP_HDR(3) | DMA_MODE(9, the is_chip_id(8852C)
     * bit) | the PPDU filter HAS_A1M/CRC_OK/DMA_OK (24/25/27). Devourer's
     * byte-OR only sets RPT_EN; without the filter bits no PPDU gets a status
     * report, so the 8852C emits zero physts (rate/RSSI read 0). Also forward
     * PPDU-status to the host, not the WLCPU (R_AX_HW_RPT_FWD 0x9C18[1:0]=1). */
    const uint32_t pre = _device.rtw_read32(0xce40);
    _device.rtw_write32(0xce40, 0x0B00020Fu);
    _device.rtw_write32(0x9C18,
                        (_device.rtw_read32(0x9C18) & ~0x3u) | 0x1u);
    _logger->info("Kestrel PPDU_STAT(8852C): 0xCE40 0x{:08x} -> 0x0B00020F", pre);
  } else {
    v = _device.rtw_read8(0xce40); /* start phy-sts update */
    _device.rtw_write8(0xce40, static_cast<uint8_t>(v | 0x1));
  }
  bb_rmw(0x2344, 1u << 31, 0); /* PD enable (2.4G) */
  bb_rmw(0xc3c, 1u << 9, 0);
}

bool HalKestrel::set_channel(uint8_t channel, ChannelWidth_t bw,
                             uint8_t offset) {
  const bool is_2g = channel <= 14;
  /* The RF synth tunes to the bandwidth-block CENTER; `channel` is the primary
   * 20. For 40 MHz: offset UPPER(2) => primary is the upper 20 (center = ch-2,
   * pri_ch=2); otherwise LOWER (center = ch+2, pri_ch=1). `pri_ch` is the BB
   * primary-sub index (0x49C4[11:8]); 20 MHz uses 0. */
  uint8_t center = channel;
  uint8_t pri_ch = 0;
  if (bw == CHANNEL_WIDTH_40) {
    if (offset == 2) {
      center = static_cast<uint8_t>(channel - 2);
      pri_ch = 2;
    } else {
      center = static_cast<uint8_t>(channel + 2);
      pri_ch = 1;
    }
  } else if (bw == CHANNEL_WIDTH_80) {
    /* 80 MHz is 5 GHz-only; the block is fixed by the channel plan, so the
     * primary's position (pri_ch 1..4) and the block center are derived from
     * the channel number alone (offset is not needed). Blocks align on the
     * 36 + 16k grid: {36,40,44,48}->center 42, {52..64}->58, {100..112}->106,
     * {149..161}->155, ... block_start = ch - ((ch-36)/4 % 4)*4. */
    const int bs = channel - (((channel - 36) / 4) % 4) * 4;
    center = static_cast<uint8_t>(bs + 6);
    pri_ch = static_cast<uint8_t>((channel - bs) / 4 + 1);
  }

  /* --- BB ctrl_ch (path A/B mode select) --- */
  bb_rmw(0x4738, 1u << 17, is_2g ? 1 : 0);
  bb_rmw(0x4AA4, 1u << 17, is_2g ? 1 : 0);

  /* --- BB ctrl_bw (halbb_ctrl_bw_8852b) --- */
  if (bw == CHANNEL_WIDTH_20) {
    bb_rmw(0x49C0, 0xC0000000u, 0x0); /* RF_BW [31:30]=0 */
    bb_rmw(0x49C4, 0x3000u, 0x0);     /* small BW [13:12]=0 */
    bb_rmw(0x49C4, 0xf00u, 0x0);      /* pri ch [11:8]=0 */
    bb_rmw(0x12ac, 0xfff000u, 0x333); /* RF mode */
    bb_rmw(0x32ac, 0xfff000u, 0x333);
    bb_rmw(0x4738, 0x10000u, 0x1); /* ACI detect [16]=1 */
    bb_rmw(0x4AA4, 0x10000u, 0x1);
  } else if (bw == CHANNEL_WIDTH_40) {
    bb_rmw(0x49C0, 0xC0000000u, 0x1);            /* RF_BW [31:30]=1 (40) */
    bb_rmw(0x49C4, 0x3000u, 0x0);                /* small BW [13:12]=0 */
    bb_rmw(0x49C4, 0xf00u, pri_ch);              /* pri ch [11:8] */
    bb_rmw(0x12ac, 0xfff000u, 0x333);            /* RF mode */
    bb_rmw(0x32ac, 0xfff000u, 0x333);
    bb_rmw(0x237c, 1u << 0, pri_ch == 1 ? 1 : 0); /* CCK primary sub-channel */
  } else if (bw == CHANNEL_WIDTH_80) {
    bb_rmw(0x49C0, 0xC0000000u, 0x2); /* RF_BW [31:30]=2 (80) */
    bb_rmw(0x49C4, 0x3000u, 0x0);     /* small BW [13:12]=0 */
    bb_rmw(0x49C4, 0xf00u, pri_ch);   /* pri ch [11:8] (1..4) */
    bb_rmw(0x12ac, 0xfff000u, 0xaaa); /* RF mode (base 8852B 80 MHz) */
    bb_rmw(0x32ac, 0xfff000u, 0xaaa);
    /* No CCK primary (0x237c) at 80 MHz — 5 GHz-only, no CCK. */
  } else if (bw == CHANNEL_WIDTH_5 || bw == CHANNEL_WIDTH_10) {
    /* Narrowband (halbb_ctrl_bw_8852b 5/10 MHz): the RF stays in 20 MHz mode
     * (rf_ctrl_bw sets RF18[11:10]=both, below); only the BB "small BW" field
     * re-clocks the ADC/DAC decimation. RF_BW=0, small-BW [13:12]=1(5)/2(10),
     * pri ch=0, RF mode 0x333, ACI-detect OFF. Same center channel. */
    bb_rmw(0x49C0, 0xC0000000u, 0x0);                        /* RF_BW=0 */
    bb_rmw(0x49C4, 0x3000u, bw == CHANNEL_WIDTH_5 ? 0x1 : 0x2); /* small BW */
    bb_rmw(0x49C4, 0xf00u, 0x0);                             /* pri ch=0 */
    bb_rmw(0x12ac, 0xfff000u, 0x333);                        /* RF mode */
    bb_rmw(0x32ac, 0xfff000u, 0x333);
    bb_rmw(0x4738, 0x10000u, 0x0);                           /* ACI detect off */
    bb_rmw(0x4AA4, 0x10000u, 0x0);
  } else {
    _logger->warn("Kestrel set_channel: bw={} not ported (20/40/80/5/10 only)",
                  static_cast<int>(bw));
  }

  /* 8852C ctrl_bw RX tail (halbb_ctrl_bw_8852c): the 8852b bw block above does
   * not configure the 8852C RX ADC for the operating bandwidth, and the DACK
   * leaves the ADC in 80 MHz mode — so a 20/40 MHz channel runs a mismatched
   * ADC clock + stale AGC-restart threshold, desensing RX. Apply the per-path
   * adc_cfg + the r_Rx_BW40_2xFFT_en bits for the active bw. */
  if (_variant == ChipVariant::C8852C) {
    const uint32_t two_fft = (bw == CHANNEL_WIDTH_40) ? 1u : 0u;
    bb_rmw(0x4e30, 1u << 26, two_fft);
    bb_rmw(0x4424, 1u << 2, two_fft);
    adc_cfg_8852c(bw, 0);
    adc_cfg_8852c(bw, 1);
    /* DIG init (halbb_dig_init_io_en -> halbb_dyn_pd_th_*(IGI_NOLINK, set_en=0)):
     * disable the packet-detection lower bound so the no-link/monitor receiver
     * runs at maximum sensitivity. The 8852b-derived bring-up skips DIG, so on
     * the 8852C these sit at the BB-table default (enabled = insensitive),
     * desensing RX. OFDM 0x481C[29]=0 + CCK 0x4b74[30]=0
     * (halbb_set_pd_lower_bound_8852c / _cck_8852c, bound==0 branch). */
    bb_rmw(0x481C, 1u << 29, 0x0);
    bb_rmw(0x4b74, 1u << 30, 0x0);
  }

  /* --- CCK enable (2.4G) / disable (5G) --- */
  bb_rmw(0x700, 1u << 5, is_2g ? 1 : 0);
  bb_rmw(0x2344, 1u << 31, is_2g ? 0 : 1);

  /* --- RF channel: full halrf_ctrl_ch_8852b (DAV+DDV x path A/B, with the
   * path-A synthesizer LCK lock). Tunes to the block CENTER. --- */
  rf_ctrl_ch(center, is_2g);

  /* --- RF bandwidth (halrf_bw_setting_8852b: RF18 [11:10]) --- */
  if (bw != CHANNEL_WIDTH_20)
    rf_ctrl_bw(bw);

  /* Hand-transcribed per-channel cal/PHY refinement (halbb gain-error/RPL +
   * halrf RX-DCK). STRIP for the C8852C (error-source removal + RX-halt
   * bisect): rely on the extracted phy_reg/gain tables instead. Kept for the
   * 8852BU (its RX works). */
  if (_variant != ChipVariant::C8852C) {
    set_gain_error(center);
    set_rxsc_rpl_comp(center);
    rx_dck();
  }

  /* --- BB reset --- */
  bb_reset_all();

  /* Force a fixed BB TX power (halbb_set_txpwr_dbm_8852b). Without it the PHY
   * runs at the phy_reg-table default, which is weak on 5 GHz. Default 20 dBm;
   * overridable via DEVOURER_TX_PWR (set_default_txpwr_dbm). The runtime offset
   * (SetTxPowerOffsetQdb) is folded in here so it survives a channel change. */
  set_txpwr_dbm(static_cast<int16_t>(_txpwr_dbm_q2 + _txpwr_offset_qdb));

  const int bw_mhz = bw == CHANNEL_WIDTH_80   ? 80
                     : bw == CHANNEL_WIDTH_40 ? 40
                     : bw == CHANNEL_WIDTH_5  ? 5
                     : bw == CHANNEL_WIDTH_10 ? 10
                                             : 20;
  _logger->info("Kestrel PHY: tuned to ch{} (center {}) bw{} ({}) — "
                "TXpwr={}dBm (off={}qdB)",
                channel, center, bw_mhz, is_2g ? "2.4G" : "5G",
                (_txpwr_dbm_q2 + _txpwr_offset_qdb) / 4, _txpwr_offset_qdb);

  /* Read-only RX diagnostics. (The earlier version wrote test patterns to BB
   * 0x4004, a-die 0x370 and d-die 0xe000 and never restored them — a debug
   * probe that CORRUPTED live BB/RF state at the end of every set_channel, a
   * prime burst-then-idle suspect. Removed: only read now.) */
  const uint32_t bb_4004 = _device.rtw_read32_wide(0x4004 + BB_WIN);
  const uint32_t bb_4738 = _device.rtw_read32_wide(0x4738 + BB_WIN);
  const uint32_t rf18a = rf_read(0, 0x18);
  const uint32_t rf18b = rf_read(1, 0x18);
  const uint32_t rf00a = rf_read(0, 0x00);
  const uint32_t rf00a_dav = rf_read_dav(0, 0x00);
  const uint32_t rf_c5 = rf_rrf(0, 0xc5, 1u << 15); /* synth lock indicator */
  const uint32_t hci_fen = _device.rtw_read32(0x8380);
  const uint8_t rcr = _device.rtw_read8(0xCE00);
  const uint32_t rxstate = _device.rtw_read32(0xCEF0);
  _logger->info("Kestrel RX-diag: BB0x4004=0x{:08x} (want CA014000) "
                "BB0x4738=0x{:08x} RF18a=0x{:05x} RF18b=0x{:05x} RF00a(dav)="
                "0x{:05x} synthLock={} HCI_FEN=0x{:08x} RCR=0x{:02x} "
                "RXstate=0x{:08x}",
                bb_4004, bb_4738, rf18a, rf18b, rf00a_dav, rf_c5, hci_fen, rcr,
                rxstate);
  return true;
}

bool HalKestrel::trx_cmac_rx_init() {
  /* Full band-0 cmac_init (trxcfg.c cmac_init), in vendor order. rst_port_info
   * and the sec-cam table are host-side software state (no chip registers) and
   * are omitted; the rest is transcribed verbatim. */
  scheduler_init();
  if (!addr_cam_init())
    return false;
  rx_fltr_init();
  cca_ctrl_init();
  nav_ctrl_init();
  spatial_reuse_init();
  tmac_init();
  trxptcl_init();
  rmac_init();
  cmac_com_init();
  ptcl_init();
  cmac_dma_init();
  usb_rx_agg_cfg();
  /* mac_trx_init tail: coex_mac_init. The SER error-IMR enable
   * (mac_enable_imr) is deliberately deferred to AFTER the BB/RF tables — the
   * vendor golden capture writes 0x8520/0xC160=0xffffffff only after the first
   * BB cmd_ofld batches, so the fw error handler isn't armed against a pending
   * HCI/DMA condition while the BB is still unconfigured. */
  coex_mac_init();
  _logger->info("Kestrel TRX: full CMAC init done (sched+addrcam+fltr+cca+nav+"
                "sr+tmac+trxptcl+rmac+com+ptcl+dma+coex)");
  return true;
}

bool HalKestrel::read_efuse(EfuseInfo &out, std::array<uint8_t, 1536> *raw_phys) {
  std::array<uint8_t, r::WL_EFUSE_PHYS_SIZE_8852B> phys{};
  if (!read_phys_efuse(phys.data(), phys.size()))
    return false;
  if (raw_phys)
    *raw_phys = phys;

  /* eeprom_parser (WLAN 2-byte-header form). Logical map defaults to 0xFF. */
  std::array<uint8_t, r::WL_EFUSE_LOG_MAP_SIZE_8852B> log{};
  log.fill(0xFF);
  const uint32_t phy_size = phys.size();
  const uint32_t log_size = log.size();
  uint32_t idx = r::WL_SEC_CTRL_EFUSE_SIZE_8852B; /* skip 4-byte sec-ctrl zone */
  while (idx < phy_size) {
    uint8_t hdr = phys[idx];
    if (hdr == 0xFF)
      break; /* end marker */
    ++idx;
    if (idx >= phy_size)
      break;
    uint8_t hdr2 = phys[idx++];
    uint8_t blk_idx =
        static_cast<uint8_t>(((hdr2 & 0xF0) >> 4) | ((hdr & 0x0F) << 4));
    uint8_t word_en = hdr2 & 0x0F;
    if (idx >= phy_size - 1) {
      _logger->error("Kestrel: efuse parse overrun (hdr 0x{:02x}/0x{:02x})",
                     hdr, hdr2);
      return false;
    }
    for (int i = 0; i < 4; ++i) {
      if (((~(word_en >> i)) & 1) != 1)
        continue;
      uint32_t log_idx = (static_cast<uint32_t>(blk_idx) << 3) + (i << 1);
      if (log_idx + 1 >= log_size || idx + 1 >= phy_size) {
        _logger->error("Kestrel: efuse parse index out of range");
        return false;
      }
      log[log_idx] = phys[idx++];
      log[log_idx + 1] = phys[idx++];
    }
  }

  auto def = [](uint8_t v, uint8_t d) { return v == 0xFF ? d : v; };
  std::memcpy(out.mac.data(), &log[r::EFUSE_USB_MAC_ADDR_8852B], 6);
  out.xtal_cap = def(log[r::EFUSE_RF_XTAL_8852B], 0x3F);
  out.rfe_type = def(log[r::EFUSE_RF_RFE_8852B], 0x01);
  out.thermal_a = def(log[r::EFUSE_RF_THERMAL_A_8852B], 0x22);
  out.thermal_b = def(log[r::EFUSE_RF_THERMAL_B_8852B], 0x22);
  /* Autoload OK ⇔ the MAC isn't all-0xFF (unprogrammed) or all-0. */
  bool all_ff = true, all_00 = true;
  for (uint8_t b : out.mac) {
    all_ff &= (b == 0xFF);
    all_00 &= (b == 0x00);
  }
  out.autoload_ok = !all_ff && !all_00;
  return true;
}

} /* namespace kestrel */
