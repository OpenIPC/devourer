#include "HalKestrel.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>
#include <utility>

#include "KestrelFw.h"
#include "MacRegAx.h"
#include "kestrel_halbb_glue.h" /* vendored halbb-G6 BB bring-up (C) */
#include "kestrel_halrf_glue.h" /* vendored halrf-G6 RF cals (C) */

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
  /* NB: the vendor usb_init_8852b (_usb_8852b.c) does NOT re-kick HCI
   * RXDMA/TXDMA here — that toggle is usb_pre_init's (pre-FWDL,
   * _usb_8852b.c:173); nothing rewrites HCI_FUNC_EN after FWDL. */
  _logger->info("Kestrel: USB init (mode=USB{}, NUMP set on 7 eps)",
                mode == r::MAC_AX_USB3 ? 3 : mode == r::MAC_AX_USB2 ? 2 : 1);
}

bool HalKestrel::power_on_8852c() {
  namespace r = kestrel::reg;
  /* mac_pwr_on_usb_8852c (pwr_seq_func_8852c.c:296), transcribed verbatim.
   * The chip-generic mac_pwr_switch prologue (env mode, BOOT_MODE exit,
   * force-off of a half-on MAC) runs in power_on() before the dispatch. */
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
    _logger->error("Kestrel: only USB power-on is ported");
    return false;
  }
  /* mac_set_dut_env_mode (init.c:26, the first thing mac_hal_init does):
   * WCPU_FW_CTRL FW_ENV[29:28] = 0 (normal DUT mode). */
  field32(r::R_AX_WCPU_FW_CTRL, 0, r::B_AX_FW_ENV_MSK, r::B_AX_FW_ENV_SH);
  /* NB: usb_pre_init (intf_pre_init) runs AFTER power-on + dmac_pre_init in
   * the vendor mac_hal_init (init.c:421) — see download_firmware. */

  /* mac_pwr_switch(on=1) chip-generic prologue (pwr.c:300..405), both dies.
   *
   * BOOT_MODE exit (pwr.c:300): the RISC-V 8852C enumerates in a special boot
   * mode; until GPIO_MUXCFG[19] is cleared the WCPU bootrom auto-boots its ROM
   * fw and never enters the FWDL H2C-wait. The 8852B does not set BOOT_MODE,
   * so the guard makes this a no-op there. */
  if (_device.rtw_read32(r::R_AX_GPIO_MUXCFG) & r::B_AX_BOOT_MODE) {
    clr32(r::R_AX_SYS_PW_CTRL, r::B_AX_APFN_ONMAC);
    clr32(r::R_AX_SYS_STATUS1, r::B_AX_AUTO_WLPON);
    clr32(r::R_AX_GPIO_MUXCFG, r::B_AX_BOOT_MODE);
    clr32(r::R_AX_RSV_CTRL, r::B_AX_R_DIS_PRST);
  }
  /* mac_pwr_sps_ana_setting (pwr.c:461) writes SPS_ANA_ON_CTRL2 only for the
   * 8852B RFE type 0x05 modules; not ported (no such module characterized). */

  /* Half-on force-off (pwr.c:314,349): a USB chip from real cold reads
   * WLMAC_PWR_STE != MAC_OFF (its power FSM auto-runs to enumerate), and the
   * on-sequence must not be applied over that half-on state — the WCPU bootrom
   * would come up FWDL_RDY yet never raise H2C_PATH_RDY (8852B), or fault into
   * SER right after boot. Force the MAC off first via APFM_OFFMAC. (A kernel
   * driver pre-initializing the chip masks this — the FSM then parks in a
   * state the on-sequence tolerates.) */
  const uint32_t pwr_ste = (_device.rtw_read32(r::R_AX_IC_PWR_STATE) >>
                            r::B_AX_WLMAC_PWR_STE_SH) &
                           r::B_AX_WLMAC_PWR_STE_MSK;
  if (pwr_ste != r::MAC_AX_MAC_OFF) {
    _logger->info("Kestrel: MAC half-on from cold (pwr_ste={}) — forcing off",
                  pwr_ste);
    set32(r::R_AX_SYS_PW_CTRL, r::B_AX_EN_WLON);     /* 0x04[16]=1 */
    set32(r::R_AX_SYS_PW_CTRL, r::B_AX_APFM_OFFMAC); /* 0x04[9]=1 */
    if (!poll32(r::R_AX_SYS_PW_CTRL, r::B_AX_APFM_OFFMAC, 0))
      return false; /* poll 0x04[9]=0 */
    clr32(r::R_AX_SYS_PW_CTRL, r::B_AX_EN_WLON);
    clr32(r::R_AX_SYS_PW_CTRL, r::B_AX_APFM_SWLPS);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    /* 8852A/8852B/8851B/8852BT only (pwr.c:380): CMAC clock source back to
     * default before the on-sequence. */
    if (_variant != ChipVariant::C8852C)
      clr32(r::R_AX_AFE_CTRL1, r::B_AX_CMAC_CLK_SEL);
  }

  if (_variant == ChipVariant::C8852C)
    return power_on_8852c();

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
   * 0x7000803F (pwr_seq_func_8852b.c:369-405). */
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
   * DMACREG_GCKEN = bits 15..31 (dmac_func_en_8852b, init_8852b.c). */
  set32(r::R_AX_DMAC_FUNC_EN, 0xFFFF8000u);
  /* cmac_func_en(band0, EN) (init.c:219): clock enables on CK_EN first, then
   * the block enables on CMAC_FUNC_EN (cmac_func_en_8852b, init_8852b.c). */
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
  /* MODE = STF (1) for USB — the hfc_init USB branch selects store-and-
   * forward flow control (hci_fc.c). An earlier port set MODE=0 + FC_EN=0 on
   * a wrong assumption; the fw never pulled the bulk cmd_ofld H2C off CH12
   * until HCI flow control was actually enabled. */
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
   * vendor keeps HCI_FC_CTRL FC_EN=0 through probe-time dmac/trx init
   * (hfc_init clears it on entry, hci_fc.c) and only flips FC_EN=1 at ifup,
   * immediately before its first bulk cmd_ofld. Enabling FC_EN mid-dmac-init
   * — while the DLE/USB reconfig is still settling — faults the fw's HCI/DMA
   * AHB monitor
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
  /* MIC/ICV append (security_cam.c:1002): SEC_MPDU_PROC |= APPEND_ICV|MIC. */
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

void HalKestrel::usb_rx_agg_cfg() {
  /* rx_agg_cfg_usb_8852b (MAC_AX_RX_AGG_MODE_USB): the RXAGG engine is what
   * DMAs RX frames onto the USB bulk-IN, so it MUST be enabled — USB mode is
   * agg_en=1 with DMA_STORE=0 and the default size/timeout thresholds. (The
   * old code cleared bit 0, which is LEN_TH[0], NOT the enable bit (BIT31) —
   * so RXAGG stayed off-by-default and no RX reached the host bulk-IN.) The
   * 11ax rxd parser already walks multi-frame aggregates via next_offset. */
  const bool c8852c = (_variant == ChipVariant::C8852C);
  const uint16_t rxagg0 = c8852c ? r::R_AX_RXAGG_0_V1 : r::R_AX_RXAGG_0;
  /* pkt_num field: the 8852C writes the field MSK (0xff) when the driver
   * default is 0 (usb_rx_agg_cfg_8852c, _usb_8852c.c:382). The 8852B path
   * keeps 0 (unchanged, on-air-validated). */
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
  /* mac_coex_init_8852b prologue (coex_8852b.c:154): B_AX_ENBT arms the BT/
   * LTE-coex block — from a real cold boot the LTE indirect interface never
   * reports ready until it is set, so every write_lte below would time out
   * ("LTE interface not ready") and the GNT arbitration would stay at its
   * cold default, gating the WL RF off both ways (deaf RX + silent TX with a
   * green init). A kernel driver pre-initializing the chip masked this. */
  if (_variant != ChipVariant::C8852C) {
    uint8_t mux = _device.rtw_read8(r::R_AX_GPIO_MUXCFG);
    _device.rtw_write8(r::R_AX_GPIO_MUXCFG,
                       static_cast<uint8_t>(mux | r::B_AX_ENBT));
  }
  /* coex_mac_init_8852b: disable LTE-coex (CTRL + CTRL_2 = 0), then set the
   * SDIO-ctrl coex bit. On the WiFi+BT combo die the coex block otherwise
   * arbitrates the shared front end. */
  write_lte(r::R_AX_LTECOEX_CTRL, 0);
  uint8_t val = _device.rtw_read8(r::R_AX_SYS_SDIO_CTRL + 3);
  write_lte(r::R_AX_LTECOEX_CTRL_2, 0);
  _device.rtw_write8(r::R_AX_SYS_SDIO_CTRL + 3,
                     static_cast<uint8_t>(val | (1u << 2)));
  /* mac_cfg_gnt_8852b with the WiFi-only stance (gnt_wl=1 sw-forced, gnt_bt=0
   * sw-forced, both bands): devourer runs no BT traffic, so the PTA arbiter is
   * pinned to WL — the same standing config the Jaguar3 coex thread re-applies
   * (there the coex fw silences the antenna without it; here the cold-default
   * GNT does). LTE_SW_CFG_1 = WL RFC+BB S0/S1 VAL|CTRL + BT RFC+BB S0/S1 CTRL
   * (val 0); LTE_SW_CFG_2 = WL TX/RX VAL|CTRL + BT TX/RX CTRL (val 0), WL_RX_
   * CTRL preserved. 8852B-only: the 8852C's cold-default GNT is WL-open and
   * its validated register stream stays untouched. */
  if (_variant != ChipVariant::C8852C) {
    write_lte(r::R_AX_LTE_SW_CFG_1,
              r::B_AX_GNT_WL_RFC_S0_SW_VAL | r::B_AX_GNT_WL_RFC_S0_SW_CTRL |
                  r::B_AX_GNT_WL_BB_S0_SW_VAL | r::B_AX_GNT_WL_BB_S0_SW_CTRL |
                  r::B_AX_GNT_BT_RFC_S0_SW_CTRL | r::B_AX_GNT_BT_BB_S0_SW_CTRL |
                  r::B_AX_GNT_WL_RFC_S1_SW_VAL | r::B_AX_GNT_WL_RFC_S1_SW_CTRL |
                  r::B_AX_GNT_WL_BB_S1_SW_VAL | r::B_AX_GNT_WL_BB_S1_SW_CTRL |
                  r::B_AX_GNT_BT_RFC_S1_SW_CTRL | r::B_AX_GNT_BT_BB_S1_SW_CTRL);
    /* (WL_RX_CTRL is preserved-if-set in the vendor; the CTRL_2=0 write above
     * just cleared it, so it folds to 0 here.) */
    write_lte(r::R_AX_LTE_SW_CFG_2,
              r::B_AX_GNT_WL_RX_SW_VAL | r::B_AX_GNT_WL_RX_SW_CTRL |
                  r::B_AX_GNT_WL_TX_SW_VAL | r::B_AX_GNT_WL_TX_SW_CTRL |
                  r::B_AX_GNT_BT_RX_SW_CTRL | r::B_AX_GNT_BT_TX_SW_CTRL);
  }
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

  /* Flip the master HCI flow-control enable now — the vendor does this at ifup
   * right before its first bulk cmd_ofld (HCI_FC_CTRL 0x08040000 -> 0x055b), so
   * FC_EN is only armed once the DMAC/CMAC/USB init has fully settled. This is
   * what actually credits a CH12 page for the incoming H2C. The 8852C flow
   * control is the _V1 register. */
  set32(_variant == ChipVariant::C8852C ? r::R_AX_HCI_FC_CTRL_V1_NIC
                                        : r::R_AX_HCI_FC_CTRL,
        r::B_AX_HCI_FC_EN);
  delay_us(10);

  /* Vendored table ownership (both chips): halbb_init_reg applies the BB
   * phy_reg table + loads the gain table + the efuse RX gain offsets,
   * halrf_config_radio applies radio B then A (vendor order) and sends the
   * radio-page H2Cs through the bridge send_h2c into the mac_ax encoder —
   * the vendor's own loaders walking the vendored hwimg arrays, replacing
   * the C++ walker + cmd_ofld batching. */
  ensure_vnd_ctx(cut, rfe_type);
  if (!kestrel_halbb_init_reg(_halbb_ctx))
    _logger->warn("Kestrel PHY: vendored halbb_init_reg reported failure");
  if (!kestrel_halrf_config_radio(_halrf_ctx))
    _logger->warn("Kestrel PHY: vendored halrf_config_radio reported failure");
  const uint32_t bb4004v = _device.rtw_read32_wide(0x4004u + 0x10000u);
  _logger->info("Kestrel PHY: BB+RF tables applied via vendored loaders "
                "(rfe=0x{:02x} cut={}) — BB0x4004=0x{:08x} (want CA014000)",
                rfe_type, cut, bb4004v);
  /* Vendored halbb bring-up: LNAON/TRSW/PAPE gpio routing + RX chains (the
   * gain cache came from init_reg), then the T-MAC TX path-com routing
   * (halbb_ctrl_tx_path_tmac_8852c) that connects the BB IFFT output to the
   * TX chain on the 8852C — without it its RF synth locks but nothing
   * radiates (glue no-op on the 8852B: per-STA CMAC antenna model). */
  vnd_bb_bringup(cut, rfe_type);
  vnd_bb_ctrl_tx_path();
  /* RFK prologue in the vendor's halrf_dm_init position (rtl8852b_halinit.c
   * runs rtw_hal_rf_dm_init right after the table apply, BEFORE any channel
   * work): NCTL microcode, RFK sub-state reset, the a-die/d-die SI reset,
   * LCK/RCK synth-filter cals, efuse trim + TSSI DE. Position is load-
   * bearing — run lazily after the first tune/BB channel config, the SI
   * reset tears down live a-die state and the radio goes deaf both ways
   * (bisected on the 8852B: rfk_init-after-tune = 0 ambient decodes,
   * dm_init position = decodes normally). */
  if (_halrf_ctx && !_halrf_rfk_inited) {
    kestrel_halrf_rfk_init(_halrf_ctx);
    _halrf_rfk_inited = true;
  }
  return true;
}

/* ---- Vendored halbb-G6 8852C RX bring-up bridge -------------------------- *
 * Realtek's own halbb C (hal/halbb/g6) drives the RX front-end verbatim:
 * gain-table cache load, LNAON/TRSW/PAPE gpio routing, RX-path enable, per-band
 * gain-error. Its register/OS plane is routed back here — BB regs through the
 * wide (wIndex=1) window like bb_rmw; power-region regs through the plain MAC
 * plane like field32. The dev cookie is this HalKestrel. */
unsigned int HalKestrel::halbb_r32(void *dev, unsigned int addr) {
  auto *h = static_cast<HalKestrel *>(dev);
  return h->_device.rtw_read32_wide((addr & 0xffffu) + 0x10000u);
}
void HalKestrel::halbb_w32(void *dev, unsigned int addr, unsigned int val) {
  auto *h = static_cast<HalKestrel *>(dev);
  h->_device.rtw_write32_wide((addr & 0xffffu) + 0x10000u, val);
}
unsigned int HalKestrel::halbb_rpwr(void *dev, unsigned int addr) {
  return static_cast<HalKestrel *>(dev)->_device.rtw_read32(
      static_cast<uint16_t>(addr));
}
void HalKestrel::halbb_wpwr(void *dev, unsigned int addr, unsigned int val) {
  static_cast<HalKestrel *>(dev)->_device.rtw_write32(
      static_cast<uint16_t>(addr), val);
}
void HalKestrel::halbb_delay(void *dev, unsigned int us) {
  (void)dev;
  delay_us(us);
}
unsigned int HalKestrel::halbb_rrf(void *dev, unsigned int path,
                                   unsigned int addr, unsigned int mask) {
  return static_cast<HalKestrel *>(dev)->rf_rrf(static_cast<uint8_t>(path), addr,
                                                mask);
}
unsigned char HalKestrel::halbb_read_xsi(void *dev, unsigned char offset) {
  return static_cast<HalKestrel *>(dev)->read_xtal_si(offset);
}
void HalKestrel::halbb_write_xsi(void *dev, unsigned char offset,
                                 unsigned char val) {
  static_cast<HalKestrel *>(dev)->write_xtal_si(offset, val, 0xff);
}
int HalKestrel::halbb_send_h2c(void *dev, unsigned char h2c_class,
                               unsigned char h2c_func, const unsigned int *data,
                               unsigned short len_bytes) {
  auto *self = static_cast<HalKestrel *>(dev);
  /* Forward ONLY the OUTSRC radio-page classes (halrf_config_*_radio_to_fw:
   * class 8 = radio A, 9 = radio B, func = page#) into the mac_ax H2C
   * encoder. Every other halrf H2C (IQK/DPK offload chatter) stays inert —
   * the validated bring-up runs those cals host-driven. */
  if (h2c_class != r::OUTSRC_CL_RADIO_A && h2c_class != r::OUTSRC_CL_RADIO_B)
    return 0;
  const bool ok = self->_fw.radio_page_to_fw(
      h2c_class, h2c_func, data, static_cast<uint16_t>(len_bytes / 4));
  return ok ? 0 : -1;
}
void HalKestrel::halbb_wrf(void *dev, unsigned int path, unsigned int addr,
                           unsigned int mask, unsigned int val) {
  static_cast<HalKestrel *>(dev)->rf_wrf(static_cast<uint8_t>(path), addr, mask,
                                         val);
}
int HalKestrel::halbb_efuse_get_info(void *dev, unsigned int id, void *value,
                                     unsigned int size) {
  auto *h = static_cast<HalKestrel *>(dev);
  if (!h->_efuse_valid || !h->_halrf_ctx)
    return 0; /* -> shim returns FAILURE -> caller uses table defaults */
  return kestrel_halrf_efuse_get_info(h->_halrf_ctx, h->_efuse_log_map.data(),
                                      id, value, size, /*autoload=*/1);
}

void HalKestrel::ensure_vnd_ctx(uint8_t cut, uint8_t rfe_type) {
  if (_halbb_bridge)
    return;
  auto *br = new kestrel_halbb_bridge{};
  br->dev = this;
  br->read32 = &HalKestrel::halbb_r32;
  br->write32 = &HalKestrel::halbb_w32;
  br->read_pwr = &HalKestrel::halbb_rpwr;
  br->write_pwr = &HalKestrel::halbb_wpwr;
  br->read_rf = &HalKestrel::halbb_rrf;
  br->write_rf = &HalKestrel::halbb_wrf;
  br->delay_us = &HalKestrel::halbb_delay;
  br->efuse_get_info = &HalKestrel::halbb_efuse_get_info;
  br->logline = nullptr;
  /* XTAL-SI plane — the 8852B's a-die SI reset (halrf_si_reset on the RFK
   * prologue) does real get/set pairs through here. */
  br->read_xsi = &HalKestrel::halbb_read_xsi;
  br->write_xsi = &HalKestrel::halbb_write_xsi;
  /* fwcmd plane — the radio-page H2Cs the vendored table loader emits. */
  br->send_h2c = &HalKestrel::halbb_send_h2c;
  _halbb_bridge = br;
  const kestrel_chip chip = _variant == ChipVariant::C8852C
                                ? KESTREL_CHIP_8852C
                                : KESTREL_CHIP_8852B;
  _halbb_ctx = kestrel_halbb_create(br, chip, cut, rfe_type);
  _halrf_ctx = kestrel_halrf_create(br, chip, cut, rfe_type); /* shares the bridge */
  /* Read + cache the efuse shadow now so the halrf cals' efuse reads (TSSI
   * DE targets, thermal trim) resolve through the efuse_get_info callback. */
  if (!_efuse_valid) {
    EfuseInfo ei;
    read_efuse(ei); /* populates _efuse_log_map + _efuse_valid */
  }
}

void HalKestrel::vnd_bb_bringup(uint8_t cut, uint8_t rfe_type) {
  ensure_vnd_ctx(cut, rfe_type);
  kestrel_halbb_rx_bringup(_halbb_ctx);
  _logger->info("Kestrel PHY: halbb-G6 bring-up applied (vendored "
                "gpio/rx-path, rfe={} cut={})", rfe_type, cut);
}

void HalKestrel::vnd_rf_dac_cal() {
  if (_halrf_ctx) {
    kestrel_halrf_dac_cal(_halrf_ctx, 1);
    _logger->info("Kestrel RF: DACK via vendored halrf");
  }
}
void HalKestrel::vnd_rf_rx_dck() {
  if (_halrf_ctx) {
    kestrel_halrf_rx_dck(_halrf_ctx);
    _logger->info("Kestrel RF: RX-DCK via vendored halrf");
  }
}
void HalKestrel::vnd_rf_tune(uint8_t band_type, uint8_t center,
                                           ChannelWidth_t bw, bool diag) {
  if (!_halrf_ctx)
    return;
  uint8_t hbw = bw == CHANNEL_WIDTH_5    ? 6
                : bw == CHANNEL_WIDTH_10 ? 7
                                         : static_cast<uint8_t>(bw);
  kestrel_halrf_ctl_band_ch_bw(_halrf_ctx, band_type, center, hbw);
  kestrel_halrf_lck(_halrf_ctx); /* relock the synth (6 GHz VCO needs this) */
  /* True synth-lock ground truth (RF 0xb7[8]=LCK-done, 0=locked; RF18 readback)
   * — the BB 0xc5[15] "synthLock" is unreliable. Four RF reads + a log line, so
   * skipped on the FastRetune hot path (diag=false): the lock diagnostic is a
   * full-set bring-up aid, not a per-hop need, and the reads are ~4 USB
   * round-trips on this chip's EP0. */
  if (!diag)
    return;
  const uint32_t rf18a = kestrel_halrf_read_rf(_halrf_ctx, 0, 0x18);
  const uint32_t rf18b = kestrel_halrf_read_rf(_halrf_ctx, 1, 0x18);
  const uint32_t lckA = kestrel_halrf_read_rf(_halrf_ctx, 0, 0xb7) & (1u << 8);
  const uint32_t lckB = kestrel_halrf_read_rf(_halrf_ctx, 1, 0xb7) & (1u << 8);
  _logger->info("Kestrel RF: band{} ch{} bw{} RF18a=0x{:05x} RF18b=0x{:05x} "
                "lckA={} lckB={} (lck 0=locked)",
                band_type, center, hbw, rf18a, rf18b, lckA ? 1 : 0, lckB ? 1 : 0);
}

void HalKestrel::vnd_rf_iqk(uint8_t center, uint8_t band, ChannelWidth_t bw) {
  if (!_halrf_ctx)
    return;
  uint8_t hbw = bw == CHANNEL_WIDTH_5    ? 6
                : bw == CHANNEL_WIDTH_10 ? 7
                                         : static_cast<uint8_t>(bw);
  /* The RFK prologue (NCTL microcode the 0xbff8 one-shot-done poll depends on,
   * SI reset, LCK/RCK, efuse trim) ran at phy_bb_rf_init — the vendor's
   * halrf_dm_init position. Running it here instead (after the tune/BB channel
   * config) deafens the radio: see phy_bb_rf_init. */
  kestrel_halrf_set_ch(_halrf_ctx, center, band, hbw);
  /* IQK is 8852C-only, the same evidence-gated stance as TSSI/DPK: on the
   * 8852B the vendored per-channel IQK collapses on-air delivery under the
   * fixed-dBm power model (paired A/B on a 5 GHz MCS0 flood, 8812AU monitor,
   * fixed geometry: IQK-on 10/1 decodes vs IQK-off 100/37 across two cold
   * pairs — repeatable ~10-30x). The 8852C validated on-air WITH IQK, so its
   * path is unchanged. Gate falls with a TSSI-referenced power model. */
  if (_variant == ChipVariant::C8852C)
    kestrel_halrf_iqk(_halrf_ctx);
  _logger->info("Kestrel RF: IQK via vendored halrf (ch{} band{})", center,
                band);
  /* The TSSI/DPK trackers are permanently masked in the glue's
   * support_ability (EVM-settled there: they degrade TX under the fixed-dBm
   * power model). */
}

void HalKestrel::fast_lck_check_8852b() {
  /* halrf_lck_check_8852b: poll RF 0xb7[8]=0 (LCK done), and if the SYN lock
   * bit RF 0xc5[15]=0 run the MMD reset + a RF18 re-set — the physical-lock
   * verify + recovery. Reads 0xb7/0xc5 (irreducible). */
  int c = 0;
  for (; c < 1000; ++c) {
    if (rf_rrf(0, 0xb7, 1u << 8) == 0)
      break;
    delay_us(1);
  }
  if (c == 1000) {
    _logger->warn("Kestrel fast retune: LCK timeout");
    return;
  }
  if (rf_rrf(0, 0xc5, 1u << 15) == 0) { /* MMD reset */
    rf_wrf(0, 0xd5, 1u << 8, 0x1);
    rf_wrf(0, 0xd5, 1u << 6, 0x0);
    rf_wrf(0, 0xd5, 1u << 6, 0x1);
    rf_wrf(0, 0xd5, 1u << 8, 0x0);
  }
  for (int i = 0; i < 10; ++i)
    delay_us(1);
  if (rf_rrf(0, 0xc5, 1u << 15) == 0) { /* re-set RF 0x18 */
    rf_wrf(0, 0xd3, 1u << 8, 0x1);
    const uint32_t t = rf_rrf(0, 0x18, r::MASKRF);
    rf_wrf(0, 0x18, r::MASKRF, t);
    rf_wrf(0, 0xd3, 1u << 8, 0x0);
  }
}

bool HalKestrel::fast_rf_channel_8852b(uint8_t channel, bool relock) {
  if (_variant != ChipVariant::C8852B || !_halbb_ctx)
    return false; /* 8852C uses the vendored ctl_band_ch_bw */
  /* Prime the touched dwords once per epoch (compose cache — one read each).
   * Invalidated by every full set_channel. */
  if (!_kfr_primed) {
    _kfr_rf18_dav = rf_rrf(0, 0x18, r::MASKRF); /* a-die (DAV) */
    _kfr_cf_a = rf_rrf(0, 0xcf, r::MASKRF);
    _kfr_cf_b = rf_rrf(1, 0xcf, r::MASKRF);
    _kfr_primed = true;
  }
  /* halrf_ch_setting_8852b RF18 compose: channel [7:0], 5G bits [16]/[8],
   * BIT(12); the rest rides along from the cached dword (write-only). */
  uint32_t rf18_dav = (_kfr_rf18_dav & ~0x3e3ffu) | channel;
  if (channel > 14)
    rf18_dav |= (1u << 16) | (1u << 8);
  rf18_dav = (rf18_dav & 0xf0fffu) | (1u << 12);
  _kfr_rf18_dav = rf18_dav; /* refresh cache to what we wrote */
  auto cf_toggle = [&](uint8_t path, uint32_t cf) {
    rf_wrf(path, 0xcf, r::MASKRF, cf & ~1u);
    rf_wrf(path, 0xcf, r::MASKRF, cf | 1u);
  };
  /* path-A channel write. The vendored path always runs the synth relock
   * (halrf_set_ch_8852b: RF 0xb1[8:6] hold + set_s0_arfc18's RF 0xd3[8] hold +
   * RF18 write + RF 0xb7[8] LCK poll, then fast_lck_check). For a SAME-sub-band
   * hop the synth moves only a little and settles on its own during the
   * caller's admission window — the ~13 ms LCK poll is pure blocking, and a
   * plain RF18 write holds channel accuracy (soak: 2000 hops, zero
   * wrong-channel, 97 % delivery). The relock is kept for a sub-band crossing
   * (a bigger VCO jump, and the only path with the MMD-reset lock recovery). */
  if (relock) {
    const uint32_t b1 = rf_rrf(0, 0xb1, r::MASKRF);
    rf_wrf(0, 0xb1, 0x1c0, 0x1);
    rf_wrf(0, 0xd3, 1u << 8, 0x1);
    rf_wrf(0, 0x18, r::MASKRF, rf18_dav);
    for (int c = 0; c < 1000; ++c) {
      if (rf_rrf(0, 0xb7, 1u << 8) == 0)
        break;
      delay_us(1);
    }
    rf_wrf(0, 0xd3, 1u << 8, 0x0);
    rf_wrf(0, 0xb1, r::MASKRF, b1);
    fast_lck_check_8852b();
  } else {
    rf_wrf(0, 0x18, r::MASKRF, rf18_dav);
  }
  cf_toggle(0, _kfr_cf_a);
  /* path-B DAV. The DDV (d-die, 0x10018) window is not populated on the
   * single-die 8852B — skipping it costs no channel accuracy (soak-confirmed)
   * and saves two SI writes + two 0xcf toggles per hop. */
  rf_wrf(1, 0x18, r::MASKRF, rf18_dav);
  cf_toggle(1, _kfr_cf_b);
  return true;
}

void HalKestrel::vnd_bb_set_gain(uint8_t channel, uint8_t band_type) {
  if (_halbb_ctx)
    kestrel_halbb_set_gain(_halbb_ctx, channel, band_type);
}

void HalKestrel::vnd_bb_ctrl_tx_path() {
  if (_halbb_ctx && _variant == ChipVariant::C8852C) {
    kestrel_halbb_ctrl_tx_path(_halbb_ctx);
    _logger->info("Kestrel PHY: T-MAC TX path-com routed (vendored, 8852C)");
  }
}

void HalKestrel::vnd_bb_ctrl_bw_ch(uint8_t pri_ch, uint8_t center,
                                       ChannelWidth_t bw, uint8_t band_type) {
  if (!_halbb_ctx)
    return;
  /* devourer ChannelWidth_t -> halbb enum channel_width (differ for 5/10 MHz:
   * devourer 5=5,10=6; halbb 5=6,10=7). */
  uint8_t hbw = bw == CHANNEL_WIDTH_5    ? 6
                : bw == CHANNEL_WIDTH_10 ? 7
                                         : static_cast<uint8_t>(bw);
  kestrel_halbb_ctrl_bw_ch(_halbb_ctx, pri_ch, center, hbw, band_type);
}

bool HalKestrel::nhm_noise_floor(int8_t &dbm, uint16_t mntr_time_ms) {
  if (!_halbb_ctx)
    return false;
  signed char v = 0;
  if (!kestrel_halbb_env_mntr_nhm(_halbb_ctx, mntr_time_ms, &v))
    return false;
  dbm = static_cast<int8_t>(v);
  return true;
}

HalKestrel::~HalKestrel() {
  if (_halrf_ctx)
    kestrel_halrf_destroy(_halrf_ctx);
  if (_halbb_ctx)
    kestrel_halbb_destroy(_halbb_ctx);
  delete static_cast<kestrel_halbb_bridge *>(_halbb_bridge);
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

/* Masked BB read (wIndex=1 window), value shifted down to the mask's LSB. */
static inline uint32_t mask_shift(uint32_t v, uint32_t mask) {
  return (v & mask) >> ctz32(mask);
}
uint32_t HalKestrel::bb_read(uint32_t addr, uint32_t mask) {
  return mask_shift(_device.rtw_read32_wide((addr & 0xffff) + BB_WIN), mask);
}

void HalKestrel::dac_cal() {
  /* Vendored halrf DACK + RX-DCK for both chips (the full vendor cal,
   * MSBK/DADCK/biasK included). Run once at bring-up for the monitor channel.
   *
   * 8852B: the vendor cal drives RF 0x0 (mode) / 0x1 / 0x5 and relies on the
   * vendor driver's later TRX/channel flow to rewrite them; devourer's
   * set_channel does not, so a bare cal leaves the RF parked in cal mode and
   * the radio deaf both ways. Save/restore operational RF state around the
   * cal chain. The 8852C path is on-air-validated bare — leave its register
   * stream untouched. */
  if (_variant != ChipVariant::C8852C) {
    const uint32_t rf0a = rf_rrf(0, 0x0, r::MASKRF),
                   rf0b = rf_rrf(1, 0x0, r::MASKRF);
    const uint32_t rf1a = rf_rrf(0, 0x1, r::MASKRF),
                   rf1b = rf_rrf(1, 0x1, r::MASKRF);
    const uint32_t rf5a = rf_rrf(0, 0x5, r::MASKRF),
                   rf5b = rf_rrf(1, 0x5, r::MASKRF);
    vnd_rf_dac_cal();
    vnd_rf_rx_dck();
    rf_wrf(0, 0x0, r::MASKRF, rf0a);
    rf_wrf(1, 0x0, r::MASKRF, rf0b);
    rf_wrf(0, 0x1, r::MASKRF, rf1a);
    rf_wrf(1, 0x1, r::MASKRF, rf1b);
    rf_wrf(0, 0x5, r::MASKRF, rf5a);
    rf_wrf(1, 0x5, r::MASKRF, rf5b);
    return;
  }
  vnd_rf_dac_cal();
  vnd_rf_rx_dck();
}

uint8_t HalKestrel::read_thermal(uint8_t path) {
  /* halrf_get_thermal_8852b: pulse RF 0x42[19] (1-0-1), settle, read [6:1]. */
  rf_wrf(path, 0x42, 1u << 19, 0x1);
  rf_wrf(path, 0x42, 1u << 19, 0x0);
  rf_wrf(path, 0x42, 1u << 19, 0x1);
  delay_us(200);
  return static_cast<uint8_t>(rf_rrf(path, 0x42, 0x7e));
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
  /* Same-band retune: reuse the band established by the last set_channel (6 GHz
   * can't be re-derived from the channel number). */
  const uint8_t band_type = _cur_band_type;
  /* The halbb gain-error table AND the synth relock are keyed by band + 5 GHz
   * sub-band, not by the exact channel: a hop within one bucket (e.g. 36->44)
   * needs neither a gain rewrite nor a synth relock (small VCO move, settles
   * on its own), while a sub-band crossing needs both. A full set_channel
   * invalidates the bucket. */
  const int gb = gain_bucket(channel, band_type);
  const bool bucket_changed = (gb != _last_gain_bucket);
  /* Compose-cache write-only RF channel set (8852B): eliminates the vendored
   * ctl_ch's per-hop RF18/0xcf reads and, within a sub-band, the ~13 ms synth
   * LCK poll. Falls back to the vendored tune on the 8852C or a cold cache;
   * the synth-lock diagnostic reads are skipped either way (diag=false). */
  if (!fast_rf_channel_8852b(channel, /*relock=*/bucket_changed))
    vnd_rf_tune(band_type, channel, CHANNEL_WIDTH_20, /*diag=*/false);
  if (bucket_changed) {
    vnd_bb_set_gain(channel, band_type);
    _last_gain_bucket = gb;
  }
  bb_reset_all();
  set_txpwr_dbm(static_cast<int16_t>(_txpwr_dbm_q2 + _txpwr_offset_qdb));
  _logger->debug("Kestrel FastRetune: ch{} ({})", channel,
                 band_type == 0 ? "2.4G" : band_type == 2 ? "6G" : "5G");
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
    /* cfg_phy_rpt (phy_rpt.c) for the 8852C R_AX_PPDU_STAT (0xCE40) =
     * 0x0B000201: RPT_EN(0) | DMA_MODE(9, the is_chip_id(8852C) bit) | the PPDU
     * report filter HAS_A1M/CRC_OK/DMA_OK (24/25/27). Devourer's byte-OR only
     * sets RPT_EN; without the filter bits no PPDU gets a status report, so the
     * 8852C emits zero physts.
     *   NB the vendor's value ALSO sets APP_MAC_INFO(1)/APP_RX_CNT(2)/
     * APP_PLCP_HDR(3), which PREPEND those blocks to the report and push the
     * physts_hdr_info out of place (devourer parses the physts, not the appended
     * MAC info) — with them set, the report reads as an invalid/empty physts
     * (is_valid=0). Clearing them makes the report the bare physts, bit-identical
     * to the 8852B (is_valid + rssi_avg_td + IE_01), which the RPKT_TYPE_PPDU
     * handler + the halbb measurement bring-up (kestrel_halbb_rx_bringup 8852C
     * branch) turn into a working per-frame RSSI/SNR passive floor.
     * Also forward PPDU-status to the host, not the WLCPU (R_AX_HW_RPT_FWD
     * 0x9C18[1:0]=1). */
    const uint32_t pre = _device.rtw_read32(0xce40);
    _device.rtw_write32(0xce40, 0x0B000201u);
    _device.rtw_write32(0x9C18,
                        (_device.rtw_read32(0x9C18) & ~0x3u) | 0x1u);
    _logger->info("Kestrel PPDU_STAT(8852C): 0xCE40 0x{:08x} -> 0x0B000201", pre);
  } else {
    v = _device.rtw_read8(0xce40); /* start phy-sts update */
    _device.rtw_write8(0xce40, static_cast<uint8_t>(v | 0x1));
  }
  bb_rmw(0x2344, 1u << 31, 0); /* PD enable (2.4G) */
  bb_rmw(0xc3c, 1u << 9, 0);
}

bool HalKestrel::set_channel(uint8_t channel, ChannelWidth_t bw,
                             uint8_t offset, uint8_t band) {
  /* band: 0 = auto (2.4 GHz if ch<=14 else 5 GHz), 6 = 6 GHz. band_type is the
   * vendor enum band_type (BAND_ON_24G=0 / BAND_ON_5G=1 / BAND_ON_6G=2). */
  const uint8_t band_type = (band == 6) ? 2 : (channel <= 14 ? 0 : 1);
  const bool is_2g = (band_type == 0);
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
    /* 80 MHz block is fixed by the channel plan: the primary's position
     * (pri_ch 1..4) and the block center derive from the channel number. The
     * grid origin differs by band — 5 GHz aligns on the 36 + 16k grid
     * ({36..48}->center 42, ...); 6 GHz aligns on the 1 + 16k grid
     * ({1..13}->center 7, {17..29}->23, ...). block_start = ch - ((ch-o)/4 %
     * 4)*4, o = grid origin. */
    const int o = (band_type == 2) ? 1 : 36;
    const int bs = channel - (((channel - o) / 4) % 4) * 4;
    center = static_cast<uint8_t>(bs + 6);
    pri_ch = static_cast<uint8_t>((channel - bs) / 4 + 1);
  } else if (bw == CHANNEL_WIDTH_160) {
    if (_variant != ChipVariant::C8852C) {
      /* The 8852B die tops out at 80 MHz (rtl8852b_halinit.c bw_sup has no
       * BW_CAP_160M); its GetTxCaps/bw_mask do not advertise 160. */
      _logger->warn("Kestrel set_channel: 160 MHz is 8852C-only (this die "
                    "tops at 80 MHz)");
      return false;
    }
    /* 160 MHz spans 8 20-MHz sub-channels; same grid-origin idea as 80 MHz but
     * modulo 8. block_start = ch - ((ch-o)/4 % 8)*4; center = bs+14 (middle of
     * bs..bs+28); pri_ch 1..8. 5 GHz: {36..64}->center 50; 6 GHz: {1..29}->15. */
    const int o = (band_type == 2) ? 1 : 36;
    const int bs = channel - (((channel - o) / 4) % 8) * 4;
    center = static_cast<uint8_t>(bs + 14);
    pri_ch = static_cast<uint8_t>((channel - bs) / 4 + 1);
    /* 6 GHz 160 MHz TX does not radiate on the C8852C: the RF synth locks
     * (RF18 6G+160, LCK 0xb7[8]=0) and RX-160 works, but the 6G+160 TX-enable
     * path is un-ported — SDR-confirmed 0% duty vs 45% at 6G-80 / 40% at
     * 5G-160. RX/monitor at 6G-160 is fine; only host-pushed TX is silent. */
    if (band_type == 2)
      _logger->warn("Kestrel: 6 GHz 160 MHz TX does not radiate on this chip "
                    "(RF locks, RX ok; 6G+160 TX-enable un-ported). Use <=80 "
                    "MHz for 6 GHz TX.");
  }


  if (_variant == ChipVariant::C8852C) {
    /* DIG PD lower-bound disable (halbb_dig_init_io_en -> dyn_pd_th, set_en=0):
     * the no-link/monitor receiver runs at max sensitivity. NOT covered by
     * halbb_ctrl_bw_ch_8852c (it's a dig_init step), so applied here. OFDM
     * 0x481C[29]=0 + CCK 0x4b74[30]=0. */
    bb_rmw(0x481C, 1u << 29, 0x0);
    bb_rmw(0x4b74, 1u << 30, 0x0);
  }

  /* --- RF channel + bandwidth + synth relock. Tunes to the block CENTER. --- */
  _cur_band_type = band_type; /* for a later same-band FastRetune */
  /* Vendored RF tune, both chips. 8852C: halrf_ctl_band_ch_bw_8852c (RF18
   * band/ch/bw for DAV+DDV both paths + the path-B CAV WA) then
   * halrf_lck_8852c — the relock priming the 6 GHz VCO needs, and it locks
   * 2.4/5 GHz just as well (bus-pinned validated: 5G 72 frames, 6G TX->RX +
   * throughput, 5G narrowband 85 frames, 2.4G CCK beacons decoded). 8852B:
   * the core's halrf_ctl_ch + halrf_ctl_bw generics (synth lock rides inside
   * halrf_ctrl_ch_8852b). */
  vnd_rf_tune(band_type, center, bw);

  /* Full vendor per-channel BB config (halbb_ctrl_bw_ch -> per-chip backend),
   * both chips: per-band gain-error, hidden/normal efuse RX gain, band
   * mode-sel, SCO comp, BW/RXBB/ADC, CCK enable. */
  vnd_bb_ctrl_bw_ch(pri_ch, center, bw, band_type);
  /* Vendored halrf IQK, both chips. The 0xbff8 one-shot-done poll depends on
   * the NCTL micro-engine, which vnd_rf_iqk loads lazily on first call
   * (kestrel_halrf_rfk_init — also LCK/RCK, the SI reset and the efuse trim).
   * Per-channel RX-DCK is not re-run here (the bring-up dac_cal covers it —
   * the shape validated on the 8852C; the RX DC term is stable per band). */
  vnd_rf_iqk(center, band_type, bw);

  /* --- BB reset --- */
  bb_reset_all();

  /* Force a fixed BB TX power (halbb_set_txpwr_dbm_8852b). Without it the PHY
   * runs at the phy_reg-table default, which is weak on 5 GHz. Default 20 dBm;
   * overridable via DEVOURER_TX_PWR (set_default_txpwr_dbm). The runtime offset
   * (SetTxPowerOffsetQdb) is folded in here so it survives a channel change. */
  set_txpwr_dbm(static_cast<int16_t>(_txpwr_dbm_q2 + _txpwr_offset_qdb));

  const int bw_mhz = bw == CHANNEL_WIDTH_160  ? 160
                     : bw == CHANNEL_WIDTH_80  ? 80
                     : bw == CHANNEL_WIDTH_40  ? 40
                     : bw == CHANNEL_WIDTH_5   ? 5
                     : bw == CHANNEL_WIDTH_10  ? 10
                                              : 20;
  _logger->info("Kestrel PHY: tuned to ch{} (center {}) bw{} ({}) — "
                "TXpwr={}dBm (off={}qdB)",
                channel, center, bw_mhz, is_2g ? "2.4G" : "5G",
                (_txpwr_dbm_q2 + _txpwr_offset_qdb) / 4, _txpwr_offset_qdb);

  /* The full BB config just applied this bucket's gain-error, so a same-bucket
   * fast hop can skip the gain rewrite; the full RF tune moved RF18/0xcf, so
   * the fast-channel compose cache must re-prime. */
  _last_gain_bucket = gain_bucket(channel, band_type);
  _kfr_primed = false;
  return true;
}

int HalKestrel::gain_bucket(uint8_t channel, uint8_t band_type) {
  if (band_type == 0)
    return 0; /* 2.4 GHz */
  if (band_type == 2)
    return 4; /* 6 GHz (single bucket here) */
  if (channel <= 64)
    return 1; /* 5G-L */
  if (channel <= 144)
    return 2; /* 5G-M */
  return 3;   /* 5G-H */
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
   * (mac_enable_imr) is deliberately deferred to AFTER the BB/RF tables, so
   * the fw error handler isn't armed against a pending HCI/DMA condition
   * while the BB is still unconfigured. */
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
  /* Cache the parsed logical map as the halrf efuse shadow (the efuse_get_info
   * bridge reads TSSI-DE / thermal fields from it via the vendored 8852c map). */
  static_assert(r::WL_EFUSE_LOG_MAP_SIZE_8852B == 2048, "efuse shadow size");
  std::copy(log.begin(), log.end(), _efuse_log_map.begin());
  _efuse_valid = out.autoload_ok;
  return true;
}

} /* namespace kestrel */
