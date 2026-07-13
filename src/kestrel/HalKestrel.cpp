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
  /* usb_pre_init_8852b: enable USB IO mode, clear RX/TX reset, re-toggle the
   * HCI DMA enables. Bulk-out endpoint pause bookkeeping is skipped — the
   * host owns endpoint selection in userspace. */
  set32(r::R_AX_USB_HOST_REQUEST_2, r::B_AX_R_USBIO_MODE);
  clr32(r::R_AX_USB_WLAN0_1, r::B_AX_USBRX_RST | r::B_AX_USBTX_RST);
  clr32(r::R_AX_HCI_FUNC_EN, r::B_AX_HCI_RXDMA_EN | r::B_AX_HCI_TXDMA_EN);
  set32(r::R_AX_HCI_FUNC_EN, r::B_AX_HCI_RXDMA_EN | r::B_AX_HCI_TXDMA_EN);
  return true;
}

void HalKestrel::usb_init() {
  /* usb_init_8852b (_usb_8852b.c:203, the mac_hal_init intf_init step — runs
   * AFTER trx_init + feat_init): LFPS filter off, RX bulk size for the link
   * speed, and the per-endpoint NUMP (burst count) for EP5,6,7,9,10,11,12. */
  clr32(r::R_AX_USB3_MAC_NPI_CONFIG_INTF_0, r::B_AX_SSPHY_LFPS_FILTER);

  const uint32_t st = _device.rtw_read32(r::R_AX_USB_STATUS);
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
    uint8_t v = _device.rtw_read8(r::R_AX_USB_ENDPOINT_0);
    v = static_cast<uint8_t>(r::set_clr_word(v, ep, r::B_AX_EP_IDX_MSK,
                                             r::B_AX_EP_IDX_SH));
    _device.rtw_write8(r::R_AX_USB_ENDPOINT_0, v);
    _device.rtw_write8(r::R_AX_USB_ENDPOINT_2 + 1, r::USB_NUMP);
  }
  /* NB: the vendor usb_init_8852b does NOT re-kick HCI RXDMA/TXDMA here —
   * that toggle is usb_pre_init's (pre-FWDL). Golden capture: no 0x8380
   * write after FWDL. */
  _logger->info("Kestrel: USB init (mode=USB{}, NUMP set on 7 eps)",
                mode == r::MAC_AX_USB3 ? 3 : mode == r::MAC_AX_USB2 ? 2 : 1);
}

bool HalKestrel::power_on() {
  if (!_device.is_usb()) {
    _logger->error("Kestrel: only USB power-on is ported (M1)");
    return false;
  }
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
  clr32(r::R_AX_DMAC_FUNC_EN, r::B_AX_DLE_WDE_EN | r::B_AX_DLE_PLE_EN);

  /* WDE: 64B page, bound 0, free = lnk_pge_num (166). */
  uint32_t v = _device.rtw_read32(r::R_AX_WDE_PKTBUF_CFG);
  v = field(v, r::S_AX_WDE_PAGE_SEL_64, r::B_AX_WDE_PAGE_SEL_MSK,
            r::B_AX_WDE_PAGE_SEL_SH);
  v = field(v, 0, r::B_AX_WDE_START_BOUND_MSK, r::B_AX_WDE_START_BOUND_SH);
  v = field(v, r::SCC_WDE_LNK_PAGE, r::B_AX_WDE_FREE_PAGE_NUM_MSK,
            r::B_AX_WDE_FREE_PAGE_NUM_SH);
  _device.rtw_write32(r::R_AX_WDE_PKTBUF_CFG, v);

  /* PLE: 128B page, bound = (166+90)*64/8192 = 2, free = 624. */
  const uint32_t bound =
      (r::SCC_WDE_LNK_PAGE + r::SCC_WDE_UNLNK_PAGE) * 64u / r::DLE_BOUND_UNIT;
  v = _device.rtw_read32(r::R_AX_PLE_PKTBUF_CFG);
  v = field(v, r::S_AX_PLE_PAGE_SEL_128, r::B_AX_PLE_PAGE_SEL_MSK,
            r::B_AX_PLE_PAGE_SEL_SH);
  v = field(v, bound, r::B_AX_PLE_START_BOUND_MSK, r::B_AX_PLE_START_BOUND_SH);
  v = field(v, r::SCC_PLE_LNK_PAGE, r::B_AX_PLE_FREE_PAGE_NUM_MSK,
            r::B_AX_PLE_FREE_PAGE_NUM_SH);
  _device.rtw_write32(r::R_AX_PLE_PKTBUF_CFG, v);

  auto qta = [&](uint16_t reg, uint16_t mn, uint16_t mx) {
    _device.rtw_write32(
        reg, (static_cast<uint32_t>(mn & r::QTA_SIZE_MSK) << r::QTA_MIN_SH) |
                 (static_cast<uint32_t>(mx & r::QTA_SIZE_MSK) << r::QTA_MAX_SH));
  };
  /* WDE quota (min==max, wde_qt25). */
  qta(r::R_AX_WDE_QTA0_CFG, r::SCC_WDE_QT_HIF, r::SCC_WDE_QT_HIF);
  qta(r::R_AX_WDE_QTA1_CFG, r::SCC_WDE_QT_WCPU, r::SCC_WDE_QT_WCPU);
  qta(r::R_AX_WDE_QTA3_CFG, 0, 0);
  qta(r::R_AX_WDE_QTA4_CFG, r::SCC_WDE_QT_CPU_IO, r::SCC_WDE_QT_CPU_IO);
  /* PLE quota Q0..Q10 (min = ple_qt74, max = ple_qt75). */
  for (int i = 0; i < 11; ++i)
    qta(static_cast<uint16_t>(r::R_AX_PLE_QTA0_CFG + i * 4), r::SCC_PLE_MIN[i],
        r::SCC_PLE_MAX[i]);

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

  /* set_fc_func_en(0, 0): disable FC + CH12 while reprogramming. */
  clr32(r::R_AX_HCI_FC_CTRL, r::B_AX_HCI_FC_EN | r::B_AX_HCI_FC_CH12_EN);

  /* Per-channel page ctrl: ACH0-3 + B0MGQ(8) + B0HIQ(9) = {18,152,grp0}. */
  const uint32_t ch = min_max_grp(r::HFC_NIC_CH_MIN, r::HFC_NIC_CH_MAX);
  w(r::R_AX_ACH0_PAGE_CTRL, ch);
  w(r::R_AX_ACH1_PAGE_CTRL, ch);
  w(r::R_AX_ACH2_PAGE_CTRL, ch);
  w(r::R_AX_ACH3_PAGE_CTRL, ch);
  w(r::R_AX_CH8_PAGE_CTRL, ch);
  w(r::R_AX_CH9_PAGE_CTRL, ch);

  /* Public page ctrl (set_fc_pubpg): group0/group1 + WP threshold. */
  w(r::R_AX_PUB_PAGE_CTRL1,
    ((static_cast<uint32_t>(r::HFC_NIC_PUB_G0) & r::B_AX_PUBPG_G0_MSK)
     << r::B_AX_PUBPG_G0_SH) |
        ((static_cast<uint32_t>(r::HFC_NIC_PUB_G1) & r::B_AX_PUBPG_G1_MSK)
         << r::B_AX_PUBPG_G1_SH));
  w(r::R_AX_WP_PAGE_CTRL2,
    (static_cast<uint32_t>(r::HFC_NIC_WP_THRD) & r::B_AX_WP_THRD_MSK)
        << r::B_AX_WP_THRD_SH);

  /* Mix cfg (set_fc_mix_cfg): precharges + public max + full conditions. */
  w(r::R_AX_CH_PAGE_CTRL,
    ((static_cast<uint32_t>(r::HFC_NIC_CH011_PREC) & r::B_AX_PREC_PAGE_CH011_MSK)
     << r::B_AX_PREC_PAGE_CH011_SH) |
        ((static_cast<uint32_t>(r::HFC_USB_H2C_PREC_8852B) &
          r::B_AX_PREC_PAGE_CH12_MSK)
         << r::B_AX_PREC_PAGE_CH12_SH));
  w(r::R_AX_PUB_PAGE_CTRL2,
    (static_cast<uint32_t>(r::HFC_NIC_PUB_MAX) & r::B_AX_PUBPG_ALL_MSK)
        << r::B_AX_PUBPG_ALL_SH);
  w(r::R_AX_WP_PAGE_CTRL1,
    ((static_cast<uint32_t>(r::HFC_NIC_WP_CH07_PREC) &
      r::B_AX_PREC_PAGE_WP_CH07_MSK)
     << r::B_AX_PREC_PAGE_WP_CH07_SH) |
        ((static_cast<uint32_t>(r::HFC_NIC_WP_CH811_PREC) &
          r::B_AX_PREC_PAGE_WP_CH811_MSK)
         << r::B_AX_PREC_PAGE_WP_CH811_SH));
  uint32_t fc = _device.rtw_read32(r::R_AX_HCI_FC_CTRL);
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
  w(r::R_AX_HCI_FC_CTRL, fc);

  /* set_fc_func_en: enable CH12 credits now, but NOT the master FC_EN. The
   * vendor keeps HCI_FC_CTRL FC_EN=0 through probe-time dmac/trx init (golden
   * capture: 0x08040000) and only flips FC_EN=1 at ifup, immediately before
   * its first bulk cmd_ofld (0x055b). Enabling FC_EN mid-dmac-init — while the
   * DLE/USB reconfig is still settling — faults the fw's HCI/DMA AHB monitor
   * (SER err=0x10002010 L2_AH_HCI|L1_DMAC). enable_hci_fc() flips it later. */
  set32(r::R_AX_HCI_FC_CTRL, r::B_AX_HCI_FC_CH12_EN);
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
  /* Receiver channel-enable: 0xF = accept on all sub-channels (the key RX
   * gate). */
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
  /* Clear RX full-mode pointer bits so RX DMA streams. */
  clr32(r::R_AX_RXDMA_CTRL_0, r::RX_FULL_MODE);
}

void HalKestrel::usb_rx_agg_cfg() {
  /* Disable USB RX aggregation for monitor (stream each frame; simplest to
   * parse). agg_en=0 leaves the RXAGG_0 enable bit clear. */
  clr32(r::R_AX_RXAGG_0, 1u << 0);
}

void HalKestrel::scheduler_init() {
  /* scheduler_init (trxcfg.c) — band0, 8852B. */
  field32(r::R_AX_PREBKF_CFG_1, r::SIFS_MACTXEN_T1_V0,
          r::B_AX_SIFS_MACTXEN_T1_MSK, r::B_AX_SIFS_MACTXEN_T1_SH);
  set32(r::R_AX_SCH_EXT_CTRL, r::B_AX_PORT_RST_TSF_ADV);
  clr32(r::R_AX_CCA_CFG_0, r::B_AX_BTCCA_EN);
  /* ASIC env leaves CCA_CFG_0's CCA/EDCCA bits as-is. AP-path pre-backoff +
   * SIFS-aggregation timing (8852B band0). */
  field32(r::R_AX_PREBKF_CFG_0, r::SCH_PREBKF_16US, r::B_AX_PREBKF_TIME_MSK,
          r::B_AX_PREBKF_TIME_SH);
  field32(r::R_AX_CCA_CFG_0, 0x6a, r::B_AX_R_SIFS_AGGR_TIME_MSK,
          r::B_AX_R_SIFS_AGGR_TIME_SH);
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
  v = r::set_clr_word(v, r::WMAC_SPEC_SIFS_OFDM_52B,
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
  write_xtal_si(r::XTAL_SI_WL_RFC_S0, 0xC7, 0xFF);
  write_xtal_si(r::XTAL_SI_WL_RFC_S1, 0xC7, 0xFF);
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
  write_xtal_si(r::XTAL_SI_WL_RFC_S0, 0xC7, 0xFF);
  write_xtal_si(r::XTAL_SI_WL_RFC_S1, 0xC7, 0xFF);
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
   * what actually credits a CH12 page for the incoming H2C. */
  set32(r::R_AX_HCI_FC_CTRL, r::B_AX_HCI_FC_EN);
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
  apply_phy_table(array_mp_8852b_phy_reg, array_mp_8852b_phy_reg_len, rfe_type,
                  cut, bb_emit);
  _fw.ofld_flush(KestrelFw::OfldFlush::BB); /* bitmap_en(false): 0x1a24 LC */
  apply_phy_table(array_mp_8852b_phy_reg_gain, array_mp_8852b_phy_reg_gain_len,
                  rfe_type, cut, bb_emit);
  _fw.ofld_flush(KestrelFw::OfldFlush::BB);

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
  apply_radio(array_mp_8852b_radioa, array_mp_8852b_radioa_len, 0);
  apply_radio(array_mp_8852b_radiob, array_mp_8852b_radiob_len, 1);

  /* Decisive readback: BB 0x4004 is host-readable (core BB), and the phy_reg
   * table sets it to 0xCA014000 — but ONLY via fw-offload now (no direct
   * writes). If it reads back the table value, the fw processed the batch. */
  const uint32_t bb4004 = _device.rtw_read32_wide(0x4004u + 0x10000u);
  _logger->info("Kestrel PHY: BB+RF tables applied via fw-offload "
                "(rfe=0x{:02x} cut={}) — BB0x4004=0x{:08x} (want CA014000)",
                rfe_type, cut, bb4004);
  return true;
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
  v = _device.rtw_read8(0xce40); /* start phy-sts update */
  _device.rtw_write8(0xce40, static_cast<uint8_t>(v | 0x1));
  bb_rmw(0x2344, 1u << 31, 0); /* PD enable (2.4G) */
  bb_rmw(0xc3c, 1u << 9, 0);
}

bool HalKestrel::set_channel(uint8_t channel, ChannelWidth_t bw) {
  const bool is_2g = channel <= 14;

  /* --- BB ctrl_ch (path A/B mode select) --- */
  bb_rmw(0x4738, 1u << 17, is_2g ? 1 : 0);
  bb_rmw(0x4AA4, 1u << 17, is_2g ? 1 : 0);

  /* --- BB ctrl_bw: 20 MHz --- */
  if (bw == CHANNEL_WIDTH_20) {
    bb_rmw(0x49C0, 0xC0000000u, 0x0); /* RF_BW [31:30]=0 */
    bb_rmw(0x49C4, 0x3000u, 0x0);     /* small BW [13:12]=0 */
    bb_rmw(0x49C4, 0xf00u, 0x0);      /* pri ch [11:8]=0 */
    bb_rmw(0x12ac, 0xfff000u, 0x333); /* RF mode */
    bb_rmw(0x32ac, 0xfff000u, 0x333);
    bb_rmw(0x4738, 0x10000u, 0x1); /* ACI detect [16]=1 */
    bb_rmw(0x4AA4, 0x10000u, 0x1);
  } else {
    _logger->warn("Kestrel set_channel: only 20 MHz ported (bw={})",
                  static_cast<int>(bw));
  }

  /* --- CCK enable (2.4G) / disable (5G) --- */
  bb_rmw(0x700, 1u << 5, is_2g ? 1 : 0);
  bb_rmw(0x2344, 1u << 31, is_2g ? 0 : 1);

  /* --- RF channel: full halrf_ctrl_ch_8852b (DAV+DDV x path A/B, with the
   * path-A synthesizer LCK lock). --- */
  rf_ctrl_ch(channel, is_2g);

  /* --- BB reset --- */
  bb_reset_all();

  _logger->info("Kestrel PHY: tuned to ch{} bw20 ({})", channel,
                is_2g ? "2.4G" : "5G");

  /* Diagnostic readback: confirm the writes landed and where RX stands.
   * 0x4004 was set to 0xCA014000 by the phy_reg table — a round-trip probe of
   * whether the wIndex=1 BB window actually reaches the baseband. */
  const uint32_t bb_4004 = _device.rtw_read32_wide(0x4004 + BB_WIN);
  _device.rtw_write32_wide(0x4004 + BB_WIN, 0x12345678u);
  const uint32_t bb_4004_rt = _device.rtw_read32_wide(0x4004 + BB_WIN);
  _logger->info("Kestrel RX-diag: BB0x4004(table)=0x{:08x} after-write="
                "0x{:08x} (want CA014000 / 12345678)",
                bb_4004, bb_4004_rt);
  const uint32_t bb_4738 = _device.rtw_read32_wide(0x4738 + BB_WIN);
  const uint32_t rf18a = rf_read(0, 0x18);
  const uint32_t rf18b = rf_read(1, 0x18);
  const uint32_t rf00a = rf_read(0, 0x00); /* RF chip reg (radioa sets it) */
  /* a-die serial reads (the synth): RF18/RF00 via 0x378/0x174c. If these show
   * the channel-encoded RF18 and a nonzero RF00, the a-die is programmed. */
  const uint32_t rf18a_dav = rf_read_dav(0, 0x18);
  const uint32_t rf00a_dav = rf_read_dav(0, 0x00);
  const uint32_t rf_c5 = rf_rrf(0, 0xc5, 1u << 15); /* synth lock indicator */
  _logger->info("Kestrel RX-diag: RF00a(ddv)=0x{:05x} RF18a(dav)=0x{:05x} "
                "RF00a(dav)=0x{:05x} synthLock(0xc5[15])={}",
                rf00a, rf18a_dav, rf00a_dav, rf_c5);
  /* SI/d-die liveness self-test: does the SI command register even hold a
   * write, and does a d-die window write land? */
  const uint32_t si_stat = _device.rtw_read32_wide(0x174c + BB_WIN);
  _device.rtw_write32_wide(0x370 + BB_WIN, 0x0abcde12u); /* a-die cmd reg */
  const uint32_t si370 = _device.rtw_read32_wide(0x370 + BB_WIN);
  _device.rtw_write32_wide(0xe000 + (0x00 << 2) + BB_WIN, 0x155aau); /* d-die A[0] */
  const uint32_t ddv_rt = _device.rtw_read32_wide(0xe000 + (0x00 << 2) + BB_WIN) & 0xfffffu;
  _logger->info("Kestrel SI-probe: 0x174c(status)=0x{:08x} 0x370(cmd rt)=0x{:08x} "
                "(wrote abcde12) d-dieA[0] rt=0x{:05x} (wrote 155aa)",
                si_stat, si370, ddv_rt);
  const uint32_t hci_fen = _device.rtw_read32(0x8380);        /* HCI_FUNC_EN */
  const uint8_t rcr = _device.rtw_read8(0xCE00);              /* RCR CH_EN */
  const uint32_t rxstate = _device.rtw_read32(0xCEF0);       /* RX_STATE_MON */
  _logger->info("Kestrel RX-diag: BB0x4738=0x{:08x} RF18a=0x{:05x} "
                "RF18b=0x{:05x} HCI_FEN=0x{:08x} RCR=0x{:02x} RXstate=0x{:08x}",
                bb_4738, rf18a, rf18b, hci_fen, rcr, rxstate);
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
