#include "Hal8822c.h"

#include <chrono>
#include <stdexcept>
#include <thread>
#include <utility>

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
    : _device{device}, _logger{logger}, _fw{device, logger},
      _macinit{device, logger}, _iqk{device, logger} {}

/* Run the ported IQK calibration (halrf_iqk_8822c). Called after the channel is
 * tuned. Replaces the captured hardcoded IQK gain registers with a real per-boot
 * calibration. */
void Hal8822c::run_iqk(SelectedChannel channel) {
  _iqk.phy_iq_calibrate(channel.ChannelWidth, channel.Channel);
  /* IQK's macbb() sets REG_TXPAUSE (0x522)=0xff to halt TX during calibration,
   * and the IQK MAC restore only covers {0x520,0x1c,0x70} — TXPAUSE is left
   * paused. The vendor clears it when bringing TX up; for monitor inject we must
   * clear it here, or queued frames never transmit (TX FIFO fills and stalls). */
  _device.rtw_write16(0x0522, 0x0000);
}

/* Full vendor-source bring-up in HalMAC _halmac_init_hal order (hal/hal_halmac.c
 * of rtl88x2cu): power-off (reset from any state) -> pre_init_system_cfg ->
 * power_on (card-enable) -> init_system_cfg -> DLFW -> init_mac_cfg ->
 * init_usb_cfg -> enable_bb_rf -> init_phy (BB/AGC/RF + RFK) -> monitor RX cfg.
 * Every step is ported from vendor source. */
void Hal8822c::rtw_hal_init(SelectedChannel channel) {
  ChannelWidth_t bw = channel.ChannelWidth;

  _macinit.pre_init_system_cfg();
  power_on();                 /* mac_power_switch(on) — card_en_flow */
  read_chip_version();        /* needs MAC alive; supplies cut for system_cfg */
  _phy_ctx.rfe_type = read_efuse_rfe_type(); /* EEPROM_RFE_OPTION_8822C (0xCA) */
  _logger->info("Jaguar3: EFUSE rfe_type=0x{:02x}", _phy_ctx.rfe_type);
  _macinit.init_system_cfg(bw, _ver.cut);

  if (!_fw.download_default_firmware()) {
    _logger->error("Jaguar3: firmware download FAILED (structured)");
    return;
  }
  _logger->info("Jaguar3: firmware booted (structured)");

  if (!_macinit.init_mac_cfg(bw)) {
    _logger->error("Jaguar3: init_mac_cfg FAILED (structured)");
    return;
  }
  _macinit.init_usb_cfg();         /* USB RX-DMA mode — RX delivery to bulk-IN */
  _macinit.enable_bb_rf(true);     /* set_hw_value(EN_BB_RF) */
  apply_bb_rf_agc_tables();        /* init_phy: BB + AGC + RF tables */
  config_phydm_parameter_init();   /* POST_SETTING: 3-wire + OFDM/CCK block + bb-reset */
  init_rfk();                      /* RF cal_init (0x1B00); IQK runs via run_iqk */
  _iqk.dac_calibrate();            /* halrf DACK — DAC cal before IQK (halrf_init) */
  bf_init();                       /* rtl8822c_phy_bf_init (rtl8822c_halinit.c) */
  monitor_rx_cfg();                /* devourer monitor-mode RX enable */
  enable_tx_path();                /* enable OFDM/CCK TX block (gates on-air TX) */
  _logger->info("Jaguar3: bring-up complete");
}

/* Port of config_phydm_parameter_init_8822c(ODM_POST_SETTING): turn on the
 * 3-wire RF interface (0x180c/0x410c), enable the OFDM/CCK block (0x1c3c[1:0]=3),
 * then BB-reset. Run after the BB phy table is applied. The CCK GI-bound/PD-init
 * sub-tuning is dynamic adaptivity, omitted here. */
void Hal8822c::config_phydm_parameter_init() {
  _device.phy_set_bb_reg(0x180c, 0x3, 0x3);
  _device.phy_set_bb_reg(0x180c, 1u << 28, 0x1);
  _device.phy_set_bb_reg(0x410c, 0x3, 0x3);
  _device.phy_set_bb_reg(0x410c, 1u << 28, 0x1);
  _device.phy_set_bb_reg(0x1c3c, 0x3, 0x3); /* enable OFDM and CCK block */
  uint32_t v = _device.rtw_read32(0x0);     /* phydm_bb_reset: 0x0[16] 1->0->1 */
  _device.rtw_write32(0x0, v | (1u << 16));
  _device.rtw_write32(0x0, v & ~(1u << 16));
  _device.rtw_write32(0x0, v | (1u << 16));
}

/* TX-path BB bits the phy-table leaves at pre-setting values, re-asserted after
 * the calibrations (DACK/IQK touch 0x1c3c). Each traced to vendor source; with
 * the 3-wire enable (config_phydm_parameter_init) + DACK this gives full on-air
 * TX (SDR-validated +43 dB, cold).
 *   0x1c3c[1:0]=3   enable OFDM and CCK block (parameter_init POST).
 *   0x1c80[29:24]=0x22  TRX-path setting (phydm trx_mode/rfe path config).
 *   0x1c90[15]=0    enable writing the TX-AGC table (bbrstb TX-AGC report).
 *   0x1cd0[30:28]=7 TX-gain-K path enables (halrf_txgapk_8822c). */
void Hal8822c::enable_tx_path() {
  _device.phy_set_bb_reg(0x1c3c, 0x00000003, 0x3);
  _device.phy_set_bb_reg(0x1c80, 0x3F000000, 0x22);
  _device.phy_set_bb_reg(0x1c90, 0x00008000, 0x0);
  _device.phy_set_bb_reg(0x1cd0, 0x70000000, 0x7);
  _logger->info("Jaguar3: TX path enabled (OFDM/CCK block + AGC/path bits)");
}

/* Port of rtl8822c_phy_bf_init (hal/rtl8822c/rtl8822c_phy.c): beamforming /
 * MU-MIMO defaults + NDPA-sounding setup. devourer never ran this, leaving the
 * NDPA/TXBF path unconfigured; the MAC TX-protocol engine needs it (REG_NDPA_
 * OPT_CTRL / TXBF_CTRL "use NDPA parameter") before it will dequeue injected
 * frames from TXPKTBUF. Single-user monitor inject doesn't sound, so MU-MIMO is
 * left disabled (as the vendor does until sounding completes). */
void Hal8822c::bf_init() {
  constexpr uint16_t REG_MU_TX_CTL = 0x14C0;
  constexpr uint16_t REG_MU_BF_OPTION = 0x167C;
  constexpr uint16_t REG_WMAC_MU_BF_CTL = 0x1680;
  constexpr uint16_t REG_TXBF_CTRL = 0x042C;
  constexpr uint16_t REG_NDPA_OPT_CTRL = 0x045F;

  uint32_t v32 = _device.rtw_read32(REG_MU_TX_CTL);
  v32 |= (1u << 16);                       /* R_MU_P1_WAIT_STATE_EN */
  v32 = (v32 & ~(0xfu << 12)) | (0xAu << 12); /* R_MU_RL = 0xA */
  v32 &= ~(1u << 7);                        /* disable MU-MIMO until sounding */
  v32 &= ~0x3fu;                            /* clear R_MU_TABLE_VALID */
  _device.rtw_write32(REG_MU_TX_CTL, v32);

  _device.rtw_write8(REG_MU_BF_OPTION,
                     static_cast<uint8_t>((3u << 4) | (1u << 6))); /* ACKPOLICY=3, EN */
  _device.rtw_write16(REG_WMAC_MU_BF_CTL, 0);

  /* MU NDPA rate/BW sourced from 0x45F (not the Tx desc): TXBF_CTRL[30] = 1 */
  uint8_t v = _device.rtw_read8(REG_TXBF_CTRL + 3);
  v |= 0x40; /* BIT_USE_NDPA_PARAMETER (BIT30) >> 24 */
  _device.rtw_write8(REG_TXBF_CTRL + 3, v);
  _device.rtw_write8(REG_NDPA_OPT_CTRL, 0x10); /* OFDM 6M, BW20 */

  /* STA2 CSI rate fixed at 6M (temp setting per vendor) */
  uint8_t v6df = _device.rtw_read8(0x06DF);
  _device.rtw_write8(0x06DF, static_cast<uint8_t>((v6df & 0xC0) | 0x04));
  _logger->info("Jaguar3: bf_init (NDPA/MU TX setup) applied");
}

/* Clean de-init — the counterpart to rtw_hal_init, run on shutdown. The kernel
 * rtw88 driver does this on unbind and the adapter always re-enumerates cleanly
 * afterwards; devourer skipping it is what left the chip hung (RX DMA running,
 * RX FIFO overflowing) and unable to re-enumerate (-71). Stop the TRX engine
 * first (clear CR MACRXEN/MACTXEN + close RX filter), then run the card-disable
 * power sequence. Best-effort: any write may fail if the chip is already gone. */
void Hal8822c::rtw_hal_deinit() {
  _logger->info("Jaguar3: clean de-init (stop TRX + card-disable)");
  _device.rtw_write16(0x0100, 0x0000);   /* CR: clear MACRXEN/MACTXEN — halt TRX */
  _device.rtw_write32(0x0608, 0x00000000); /* RCR: drop all RX (stop FIFO fill) */
  power_off();                            /* card-disable PWR_SEQ → re-enumerable */
}

/* Monitor-mode RX configuration (devourer-specific; the vendor driver has no
 * pure-monitor path). Accept all frames incl. CRC/ICV errors, append PHY status
 * drvinfo (so parse_rx_8822c's drvinfo_size is consistent), all RX filter maps
 * open. RCR bits: AAP/APM/AM/AB/ACF/AICV/ACRC32 + APP_PHYSTS. */
void Hal8822c::monitor_rx_cfg() {
  constexpr uint16_t REG_RCR_8822C = 0x0608;
  constexpr uint16_t REG_RXFLTMAP0_8822C = 0x06A0;
  constexpr uint16_t REG_RXFLTMAP1_8822C = 0x06A2;
  constexpr uint16_t REG_RXFLTMAP2_8822C = 0x06A4;
  constexpr uint16_t REG_RX_DRVINFO_SZ_8822C = 0x060F;
  /* Enable the full MAC: CR = TRX-DMA | PROTOCOL_EN | SCHEDULE_EN | MACTXEN |
   * MACRXEN(+ENSWBCN). init_mac_cfg only set CR=0x0F (DMA enable); without
   * MACRXEN (BIT7) the MAC RX engine never runs — the structured-path RX gap. */
  _device.rtw_write16(0x0100, 0x06FF);
  /* accept-all + keep FCS/ICV-error frames + append phy-status (BIT28) */
  _device.rtw_write32(REG_RCR_8822C, 0xF410400E | (1u << 28));
  _device.rtw_write8(REG_RX_DRVINFO_SZ_8822C, 0x04);
  _device.rtw_write16(REG_RXFLTMAP0_8822C, 0xFFFF);
  _device.rtw_write16(REG_RXFLTMAP1_8822C, 0xFFFF);
  _device.rtw_write16(REG_RXFLTMAP2_8822C, 0xFFFF);
  _logger->info("Jaguar3: monitor RX config applied (accept-all + phystatus)");
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

  static const char *vn[] = {"TSMC", "SMIC", "UMC"};
  _logger->info("Jaguar3 chip: 8822C cut={} ({}{}) {} (SYS_CFG1=0x{:08x})",
                _ver.cut, vn[_ver.vendor], _ver.test_chip ? ",test" : "",
                _ver.rf_2t2r ? "2T2R" : "1T1R", v);
}

/* Decode the packed (extended-header) EFUSE into a logical map up to the byte we
 * need and return logical offset 0xCA (EEPROM_RFE_OPTION_8822C). Standard
 * Realtek format: per section a header byte (or header+ext byte) gives a logical
 * block offset + 4-bit word-enable; each enabled 2-byte word follows. */
uint8_t Hal8822c::read_efuse_rfe_type() {
  constexpr uint16_t kRfeLogicalOff = 0x00CA;
  constexpr uint16_t kPhysMax = 1024; /* EFUSE_REAL_CONTENT_LEN_8822C */
  uint8_t map[0x100 + 0x40];          /* enough to cover block holding 0xCA */
  for (auto &b : map) b = 0xFF;

  uint16_t phys = 0;
  uint8_t hdr = 0;
  while (phys < kPhysMax) {
    if (!_device.efuse_OneByteRead(phys++, &hdr) || hdr == 0xFF)
      break;
    uint16_t offset;
    uint8_t word_en;
    if ((hdr & 0x1F) == 0x0F) { /* extended header */
      uint8_t ext = 0;
      if (!_device.efuse_OneByteRead(phys++, &ext))
        break;
      if ((ext & 0x0F) == 0x0F)
        continue; /* invalid / skip */
      offset = static_cast<uint16_t>(((ext & 0xF0) >> 1) | ((hdr & 0xE0) >> 5));
      word_en = ext & 0x0F;
    } else {
      offset = (hdr >> 4) & 0x0F;
      word_en = hdr & 0x0F;
    }
    uint16_t base = static_cast<uint16_t>(offset << 3); /* offset * 8 bytes */
    for (uint8_t i = 0; i < 4; i++) {
      if (word_en & (1u << i))
        continue; /* word skipped */
      for (uint8_t k = 0; k < 2; k++) {
        uint8_t d = 0xFF;
        if (!_device.efuse_OneByteRead(phys++, &d))
          return 0;
        uint16_t idx = static_cast<uint16_t>(base + i * 2 + k);
        if (idx < sizeof(map))
          map[idx] = d;
      }
    }
    if (base > kRfeLogicalOff + 8)
      break; /* past the byte we need */
  }
  uint8_t rfe = map[kRfeLogicalOff];
  return (rfe == 0xFF) ? 0 : rfe;
}

namespace {
/* RTL8822C USB power-on sequence (card-disable -> card-emulation -> active),
 * transcribed from the halmac WLAN_PWR_CFG flow card_en_flow_8822c
 * (TRANS_CARDDIS_TO_CARDEMU_8822C + TRANS_CARDEMU_TO_ACT_8822C), keeping the
 * USB/ALL-interface entries (PCI/SDIO-only steps dropped). Each step is a
 * read-modify-write or a poll on an 8-bit MAC register. */
enum PwrCmd { PC_WRITE, PC_POLL, PC_END };
struct PwrCfg { uint16_t off; uint8_t cmd; uint8_t msk; uint8_t val; };
constexpr uint8_t B(int n) { return static_cast<uint8_t>(1u << n); }

const PwrCfg kPwrOn8822cUsb[] = {
    /* --- card-disable -> card-emulation --- */
    {0x002E, PC_WRITE, B(2), B(2)},
    {0x002D, PC_WRITE, B(0), 0},
    {0x007F, PC_WRITE, B(7), 0},
    {0x004A, PC_WRITE, B(0), 0},
    {0x0005, PC_WRITE, static_cast<uint8_t>(B(3) | B(4)), 0},
    /* --- card-emulation -> active --- */
    {0x0000, PC_WRITE, B(5), 0},
    {0x0005, PC_WRITE, static_cast<uint8_t>(B(4) | B(3) | B(2)), 0},
    {0x0006, PC_POLL, B(1), B(1)},
    {0xFF1A, PC_WRITE, 0xFF, 0},
    {0x002E, PC_WRITE, B(3), 0},
    {0x0006, PC_WRITE, B(0), B(0)},
    {0x0005, PC_WRITE, static_cast<uint8_t>(B(4) | B(3)), 0},
    {0x1018, PC_WRITE, B(2), B(2)},
    {0x0005, PC_WRITE, B(0), B(0)},
    {0x0005, PC_POLL, B(0), 0},
    {0x001F, PC_WRITE, static_cast<uint8_t>(B(7) | B(6)), B(7)},
    {0x00EF, PC_WRITE, static_cast<uint8_t>(B(7) | B(6)), B(7)},
    {0x1045, PC_WRITE, B(4), B(4)},
    {0x0010, PC_WRITE, B(2), B(2)},
    {0x1064, PC_WRITE, B(1), B(1)},
    {0, PC_END, 0, 0},
};
} /* namespace */

namespace {
/* RTL8822C USB power-OFF sequence (card_dis_flow_8822c = TRANS_ACT_TO_CARDEMU +
 * TRANS_CARDEMU_TO_CARDDIS, USB/ALL-interface entries). Run before power-on so
 * devourer resets the MAC from a kernel-left "active" state (where running only
 * the card-enable flow leaves RX dead). Polls are best-effort (non-fatal) so a
 * truly-cold chip — already off — isn't blocked. */
const PwrCfg kPwrOff8822cUsb[] = {
    /* --- active -> card-emulation --- */
    {0x0093, PC_WRITE, B(3), 0},
    {0x001F, PC_WRITE, 0xFF, 0},
    {0x00EF, PC_WRITE, 0xFF, 0},
    {0x1045, PC_WRITE, B(4), 0},
    {0xFF1A, PC_WRITE, 0xFF, 0x30},
    {0x0049, PC_WRITE, B(1), 0},
    {0x0006, PC_WRITE, B(0), B(0)},
    {0x0002, PC_WRITE, B(1), 0},
    {0x0005, PC_WRITE, B(1), B(1)},
    {0x0005, PC_POLL, B(1), 0},
    {0x0000, PC_WRITE, B(5), B(5)},
    /* --- card-emulation -> card-disable --- */
    {0x0007, PC_WRITE, 0xFF, 0x00},
    {0x0067, PC_WRITE, B(5), 0},
    {0x004A, PC_WRITE, B(0), 0},
    {0x0081, PC_WRITE, static_cast<uint8_t>(B(7) | B(6)), 0},
    {0x0090, PC_WRITE, B(1), 0},
    {0x0005, PC_WRITE, static_cast<uint8_t>(B(3) | B(4)), B(3)},
    {0, PC_END, 0, 0},
};
} /* namespace */

void Hal8822c::power_off() {
  for (const PwrCfg *p = kPwrOff8822cUsb; p->cmd != PC_END; ++p) {
    if (p->cmd == PC_WRITE) {
      uint8_t v = _device.rtw_read8(p->off);
      v = static_cast<uint8_t>((v & ~p->msk) | (p->val & p->msk));
      _device.rtw_write8(p->off, v);
    } else { /* PC_POLL — best-effort, don't wedge a cold chip */
      uint32_t cnt = 2000;
      while ((_device.rtw_read8(p->off) & p->msk) != (p->val & p->msk)) {
        if (--cnt == 0)
          break;
        std::this_thread::sleep_for(std::chrono::microseconds(10));
      }
    }
  }
  _logger->info("Jaguar3: power-off (card-disable) sequence applied");
}

void Hal8822c::power_on() {
  /* Reset the MAC from any prior state (e.g. kernel-left active) first. */
  power_off();
  for (const PwrCfg *p = kPwrOn8822cUsb; p->cmd != PC_END; ++p) {
    if (p->cmd == PC_WRITE) {
      uint8_t v = _device.rtw_read8(p->off);
      v = static_cast<uint8_t>((v & ~p->msk) | (p->val & p->msk));
      _device.rtw_write8(p->off, v);
    } else { /* PC_POLL */
      uint32_t cnt = 5000;
      while ((_device.rtw_read8(p->off) & p->msk) != (p->val & p->msk)) {
        if (--cnt == 0)
          throw std::runtime_error(
              "Jaguar3: power-on poll timeout (chip not responding)");
        std::this_thread::sleep_for(std::chrono::microseconds(10));
      }
    }
  }
  _logger->info("Jaguar3: power-on sequence complete (card active)");
}

void Hal8822c::apply_bb_rf_agc_tables() {
  /* BB + AGC baseline via the validated halbb walker (src/jaguar3/
   * PhyTableLoader8822c). _phy_ctx must be populated from the chip-version +
   * EFUSE read (done earlier in rtw_hal_init) before the tables are walked. */
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

  /* RF radio tables. On 8822C an RF register write is a direct BB write to a
   * per-path window: BB[base + (rf_addr&0xff)*4], 20-bit mask, base 0x3c00
   * (path A) / 0x4c00 (path B) — config_phydm_direct_write_rf_reg_8822c. The
   * load is bracketed by phydm_rstb_3wire_8822c(false/true). */
  constexpr uint32_t RFREG_MASK = 0x000fffff;
  auto rf_writer = [this](uint16_t base) {
    return [this, base](uint32_t addr, uint32_t data) {
      switch (addr) {
      case 0xffe: std::this_thread::sleep_for(std::chrono::milliseconds(50)); return;
      case 0xfe:  std::this_thread::sleep_for(std::chrono::microseconds(100)); return;
      case 0xffff: std::this_thread::sleep_for(std::chrono::microseconds(1)); return;
      default:
        _device.phy_set_bb_reg(static_cast<uint16_t>(base + ((addr & 0xff) << 2)),
                               RFREG_MASK, data);
      }
    };
  };
  /* rstb_3wire(false) */
  _device.phy_set_bb_reg(0x1c90, 1u << 8, 0);
  _logger->info("Jaguar3: applying RF radioA ({} words) + radioB ({} words)",
                array_mp_8822c_radioa_len, array_mp_8822c_radiob_len);
  PhyTableLoader8822c::Load(array_mp_8822c_radioa, array_mp_8822c_radioa_len,
                           _phy_ctx, rf_writer(0x3c00));
  PhyTableLoader8822c::Load(array_mp_8822c_radiob, array_mp_8822c_radiob_len,
                           _phy_ctx, rf_writer(0x4c00));
  /* rstb_3wire(true) + force anapar update */
  _device.phy_set_bb_reg(0x1c90, 1u << 8, 1);
  _device.phy_set_bb_reg(0x1830, 1u << 29, 1);
  _device.phy_set_bb_reg(0x4130, 1u << 29, 1);
  _logger->info("Jaguar3: BB/AGC/RF tables applied");
}

/* RF-calibration (RFK) init end-state. The kernel rtw88_8822cu runs
 * halrf_rfk_init_8822c (a ~2500-line IQK/DPK/LCK procedure) which programs the
 * 0x1B00 block + a few RF gain regs; without it the RX path delivers nothing.
 * Here we write the captured end-state directly (same chip) — enough to bring
 * RX up; a full RFK port is future work. Most of the 0x1B00 block is static
 * config; the 0x3d08/0x4d08 gain regs are IQK-computed (chip-specific). */
void Hal8822c::init_rfk() {
  /* odm_read_and_config_mp_8822c_cal_init (halrf_rfk_init_8822c.c): a flat
   * (addr,value) BB-write table (the 0x1B00 IQK/DPK RX-path block) preceded by
   * the iqk/dpk clock/reset/enable bits at 0x1cd0. NOT conditional. */
  _device.phy_set_bb_reg(0x1cd0, 1u << 28, 1); /* iqk_dpk clock src */
  _device.phy_set_bb_reg(0x1cd0, 1u << 29, 1); /* iqk_dpk reset src */
  _device.phy_set_bb_reg(0x1cd0, 1u << 30, 1); /* en IQK_dpk */
  _device.phy_set_bb_reg(0x1cd0, 1u << 31, 0);
  for (uint32_t i = 0; i + 1 < array_mp_8822c_cal_init_len; i += 2)
    write_bb(_device, array_mp_8822c_cal_init[i], array_mp_8822c_cal_init[i + 1]);

  /* The actual IQK gains are now produced by the ported halrf_iqk_8822c
   * (run_iqk), which runs after the channel is tuned — see
   * RtlJaguar3Device. No more hardcoded captured gains. */
  _logger->info("Jaguar3: cal_init ({} pairs) applied",
                array_mp_8822c_cal_init_len / 2);
}

/* Port of rtw_fw_send_h2c_command: 4 HMEBOX mailboxes (msg at 0x1d0+box*4,
 * msg_ext at 0x1f0+box*4), round-robin. Poll REG_HMETFR (0x1cc) until the box's
 * bit clears (box consumed), then write msg_ext first, then msg. */
void Hal8822c::send_h2c_raw(uint32_t msg, uint32_t msg_ext) {
  uint8_t box = _h2c_box & 0x3;
  uint16_t box_reg = static_cast<uint16_t>(0x1d0 + box * 4);
  uint16_t box_ex_reg = static_cast<uint16_t>(0x1f0 + box * 4);
  for (int i = 0; i < 30; ++i) { /* ~3 ms, mirrors the kernel's 100us*30 poll */
    if (((_device.rtw_read<uint8_t>(0x1cc) >> box) & 0x1) == 0)
      break;
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  _device.rtw_write<uint32_t>(box_ex_reg, msg_ext);
  _device.rtw_write<uint32_t>(box_reg, msg);
  _h2c_box = static_cast<uint8_t>((_h2c_box + 1) & 0x3);
}

void Hal8822c::fw_update_wl_phy_info() {
  /* H2C_CMD_WL_PHY_INFO (0x58). word0 = id[7:0] | tx_tp[17:8] | rx_tp[27:18];
   * word1 = tx_rate[7:0] | rx_rate[15:8] | rx_evm[23:16]. A non-zero tx_tp tells
   * the firmware WLAN is actively transmitting so it keeps the RF powered. */
  const uint32_t tx_tp = 100; /* Mbps — signal "WL busy" */
  uint32_t msg = 0x58u | ((tx_tp & 0x3ffu) << 8);
  send_h2c_raw(msg, 0);
}

void Hal8822c::fw_set_pwr_mode_active() {
  /* word0: id[7:0]=0x20, mode[14:8]=0, rlbm[19:16]=0, smart_ps[23:20]=0,
   *        awake_interval[31:24]=1.
   * word1: port_id[7:5]=0, pwr_state[15:8]=RTW_ALL_ON(0xc). */
  uint32_t msg = 0x20u | (1u << 24);
  uint32_t ext = (0xcu << 8);
  send_h2c_raw(msg, ext);
}

} /* namespace jaguar3 */
