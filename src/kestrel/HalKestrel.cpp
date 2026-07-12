#include "HalKestrel.h"

#include <chrono>
#include <cstring>
#include <thread>
#include <utility>

#include "KestrelFw.h"
#include "MacRegAx.h"

namespace kestrel {

namespace {
namespace r = kestrel::reg;

void delay_us(uint32_t us) {
  std::this_thread::sleep_for(std::chrono::microseconds(us));
}

} /* namespace */

HalKestrel::HalKestrel(RtlAdapter device, Logger_t logger, ChipVariant variant)
    : _device{std::move(device)}, _logger{std::move(logger)},
      _variant{variant} {}

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

bool HalKestrel::power_on() {
  if (!_device.is_usb()) {
    _logger->error("Kestrel: only USB power-on is ported (M1)");
    return false;
  }
  if (!usb_pre_init())
    return false;

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

  /* Enable DMAC (0x8400) + CMAC (0xC000) function blocks — the vendor pwr-on
   * leaves these set (mac_sys_init re-asserts them). The full bit masks match
   * pwr_seq_func_8852b.c. */
  constexpr uint16_t R_AX_DMAC_FUNC_EN = 0x8400;
  constexpr uint32_t kDmacEn =
      (1u << 30) | (1u << 29) | (1u << 28) | (1u << 27) | (1u << 26) |
      (1u << 25) | (1u << 24) | (1u << 23) | (1u << 22) | (1u << 21) |
      (1u << 20) | (1u << 19) | (1u << 18) | (1u << 17) | (1u << 16) |
      (1u << 15); /* MAC_FUNC..DMACREG_GCKEN, verbatim mask */
  set32(R_AX_DMAC_FUNC_EN, kDmacEn);
  constexpr uint16_t R_AX_CMAC_FUNC_EN = 0xC000;
  constexpr uint32_t kCmacEn = (1u << 0) | (1u << 5) | (1u << 6) | (1u << 15) |
                               (1u << 16) | (1u << 17) | (1u << 18) |
                               (1u << 19) | (1u << 20) | (1u << 21);
  set32(R_AX_CMAC_FUNC_EN, kCmacEn);

  /* 0x2D8[7:4] = 1 (EESK pinmux). */
  field32(r::R_AX_EECS_EESK_FUNC_SEL, 0x1, r::B_AX_PINMUX_EESK_FUNC_SEL_MSK,
          r::B_AX_PINMUX_EESK_FUNC_SEL_SH);

  _logger->info("Kestrel: MAC powered on (pwr-K=0x{:02x})", pwr_k);
  return true;
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
  KestrelFw fw(_device, _logger, _variant);
  return fw.download_firmware(cut, read_mss_index());
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
}

bool HalKestrel::trx_dmac_init() {
  if (!dle_init_nic()) /* NIC-mode DLE quota */
    return false;
  /* NOTE: the full NIC HFC reprogramming (hfc_init(1,1,1): per-channel +
   * public page quotas) is not yet ported; the DLFW HFC from firmware
   * download is left in place. If the sta-scheduler poll below fails on
   * hardware, the NIC HFC is the missing piece. */
  if (!sta_sch_init())
    return false;
  mpdu_proc_init();
  sec_eng_init();
  _logger->info("Kestrel TRX: DMAC init done (NIC DLE + sta-sch + mpdu + sec)");
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

bool HalKestrel::trx_cmac_rx_init() {
  rx_fltr_init();
  cca_ctrl_init();
  rmac_init();
  cmac_com_init();
  cmac_dma_init();
  usb_rx_agg_cfg();
  _logger->info("Kestrel TRX: CMAC RX init done (fltr + rmac + cca + dma)");
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
