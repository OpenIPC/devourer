#include "HalJaguar2.h"

#include <chrono>
#include <cstdlib>
#include <stdexcept>
#include <thread>
#include <utility>

#include "PhyTableLoader.h"
#include "PhyTableLoaderJaguar2.h"

namespace jaguar2 {

namespace {
/* RTL8822B USB power sequence, transcribed from the halmac WLAN_PWR_CFG flow
 * (reference/rtl88x2bu/hal/halmac/halmac_88xx/halmac_8822b/halmac_pwr_seq_8822b.c),
 * keeping the USB- and ALL-interface entries and dropping the SDIO/PCI-only
 * steps. Each step is a byte read-modify-write, a byte poll, or a delay. The
 * 0x10A8..0x10AA writes are cut-C-only in halmac; production RTL8822BU is C-cut
 * (SYS_CFG1 cut field = 2), so they are included. */
enum PwrCmd { PC_WRITE, PC_POLL, PC_DELAY, PC_END };
struct PwrCfg { uint16_t off; uint8_t cmd; uint8_t msk; uint8_t val; };
constexpr uint8_t B(int n) { return static_cast<uint8_t>(1u << n); }

/* card_en_flow_8822b = CARDDIS_TO_CARDEMU + CARDEMU_TO_ACT (USB/ALL). */
const PwrCfg kPwrOn8822bUsb[] = {
    /* --- card-disable -> card-emulation --- */
    {0x004A, PC_WRITE, B(0), 0},
    {0x0005, PC_WRITE, static_cast<uint8_t>(B(3) | B(4) | B(7)), 0},
    /* --- card-emulation -> active --- */
    {0xFF0A, PC_WRITE, 0xFF, 0},
    {0xFF0B, PC_WRITE, 0xFF, 0},
    {0x0012, PC_WRITE, B(1), 0},
    {0x0012, PC_WRITE, B(0), B(0)},
    {0x0020, PC_WRITE, B(0), B(0)},
    {0x0001, PC_DELAY, 0, 1}, /* 1 ms */
    {0x0000, PC_WRITE, B(5), 0},
    {0x0005, PC_WRITE, static_cast<uint8_t>(B(4) | B(3) | B(2)), 0},
    {0x0006, PC_POLL, B(1), B(1)},
    {0xFF1A, PC_WRITE, 0xFF, 0},
    {0x0006, PC_WRITE, B(0), B(0)},
    {0x0005, PC_WRITE, B(7), 0},
    {0x0005, PC_WRITE, static_cast<uint8_t>(B(4) | B(3)), 0},
    {0x10C3, PC_WRITE, B(0), B(0)},
    {0x0005, PC_WRITE, B(0), B(0)},
    {0x0005, PC_POLL, B(0), 0},
    {0x0020, PC_WRITE, B(3), B(3)},
    {0x10A8, PC_WRITE, 0xFF, 0},    /* cut-C */
    {0x10A9, PC_WRITE, 0xFF, 0xef}, /* cut-C */
    {0x10AA, PC_WRITE, 0xFF, 0x0c}, /* cut-C */
    {0x0029, PC_WRITE, 0xFF, 0xF9},
    {0x0024, PC_WRITE, B(2), 0},
    {0x00AF, PC_WRITE, B(5), B(5)},
    {0, PC_END, 0, 0},
};

/* card_dis_flow_8822b = ACT_TO_CARDEMU + CARDEMU_TO_CARDDIS (USB/ALL). */
const PwrCfg kPwrOff8822bUsb[] = {
    /* --- active -> card-emulation --- */
    {0x0093, PC_WRITE, 0xFF, 0xC4},
    {0x001F, PC_WRITE, 0xFF, 0},
    {0x00EF, PC_WRITE, 0xFF, 0},
    {0xFF1A, PC_WRITE, 0xFF, 0x30},
    {0x0049, PC_WRITE, B(1), 0},
    {0x0006, PC_WRITE, B(0), B(0)},
    {0x0002, PC_WRITE, B(1), 0},
    {0x10C3, PC_WRITE, B(0), 0},
    {0x0005, PC_WRITE, B(1), B(1)},
    {0x0005, PC_POLL, B(1), 0},
    {0x0020, PC_WRITE, B(3), 0},
    {0x0000, PC_WRITE, B(5), B(5)},
    /* --- card-emulation -> card-disable --- */
    {0x0007, PC_WRITE, 0xFF, 0x20},
    {0x0067, PC_WRITE, B(5), 0},
    {0x004A, PC_WRITE, B(0), 0},
    {0x0081, PC_WRITE, static_cast<uint8_t>(B(7) | B(6)), 0},
    {0x0005, PC_WRITE, static_cast<uint8_t>(B(3) | B(4)), B(3)},
    {0x0090, PC_WRITE, B(1), 0},
    {0, PC_END, 0, 0},
};

void run_pwr_seq(RtlUsbAdapter &dev, const PwrCfg *seq, uint32_t poll_max,
                 bool poll_fatal) {
  for (const PwrCfg *p = seq; p->cmd != PC_END; ++p) {
    if (p->cmd == PC_WRITE) {
      uint8_t v = dev.rtw_read8(p->off);
      v = static_cast<uint8_t>((v & ~p->msk) | (p->val & p->msk));
      dev.rtw_write8(p->off, v);
    } else if (p->cmd == PC_DELAY) {
      std::this_thread::sleep_for(std::chrono::milliseconds(p->val));
    } else { /* PC_POLL */
      uint32_t cnt = poll_max;
      while ((dev.rtw_read8(p->off) & p->msk) != (p->val & p->msk)) {
        if (--cnt == 0) {
          if (poll_fatal)
            throw std::runtime_error(
                "Jaguar2: power-on poll timeout (chip not responding)");
          break;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));
      }
    }
  }
}
} /* namespace */

HalJaguar2::HalJaguar2(RtlUsbAdapter device, Logger_t logger)
    : _device{std::move(device)}, _logger{std::move(logger)} {}

void HalJaguar2::power_off() {
  run_pwr_seq(_device, kPwrOff8822bUsb, /*poll_max=*/2000, /*poll_fatal=*/false);
  _logger->info("Jaguar2: power-off (card-disable) sequence applied");
}

void HalJaguar2::power_on() {
  power_off(); /* reset from any prior (kernel-left active) state first */
  run_pwr_seq(_device, kPwrOn8822bUsb, /*poll_max=*/5000, /*poll_fatal=*/true);
  _logger->info("Jaguar2: power-on sequence complete (card active)");
}

void HalJaguar2::read_chip_version() {
  constexpr uint16_t REG_SYS_CFG1_8822B = 0x00F0;
  uint32_t v = _device.rtw_read32(REG_SYS_CFG1_8822B);

  _ver.test_chip = (v & (1u << 23)) != 0;
  _ver.cut = static_cast<uint8_t>((v >> 12) & 0xf);
  _ver.rf_2t2r = (v & (1u << 27)) ? 1 : 0;
  /* The kernel rtl88x2bu_ohd tells the FW rf_type=1T1R, tx_ant/rx_ant=A for the
   * Archer T3U (RTL8812BU) even though SYS_CFG bit27 reports 2T2R silicon —
   * the dongle operates single-path. devourer's AB config may route TX to a
   * path the dongle doesn't wire. Force single-path-A to match the kernel. */
  if (getenv("DEVOURER_FORCE_1T1R"))
    _ver.rf_2t2r = 0;
  uint8_t vend = static_cast<uint8_t>(((v >> 16) & 0xf) >> 2);
  _ver.vendor = (vend <= 2) ? vend : 0;

  static const char *vn[] = {"TSMC", "SMIC", "UMC"};
  _logger->info("Jaguar2 chip: 8822B cut={} ({}{}) {} (SYS_CFG1=0x{:08x})",
                _ver.cut, vn[_ver.vendor], _ver.test_chip ? ",test" : "",
                _ver.rf_2t2r ? "2T2R" : "1T1R", v);
}

uint8_t HalJaguar2::read_efuse_rfe() {
  /* Standard 88xx EFUSE logical-map decode (efuse_OneByteRead + 2-byte
   * extended-header format). Ported from the jaguar3 non-EU path; 8822B uses
   * the same physical primitive. Walk only far enough to cover 0xCA. */
  constexpr uint16_t kRfeOff = 0xCA; /* EEPROM_RFE_OPTION_8822B */
  constexpr uint16_t kPhysMax = 1024;
  uint8_t map[0xD0];
  for (auto &b : map) b = 0xFF;

  auto rd = [this](uint16_t a) -> uint8_t {
    uint8_t d = 0xFF;
    return _device.efuse_OneByteRead(a, &d) ? d : 0xFF;
  };

  uint16_t phys = 0;
  while (phys < kPhysMax) {
    uint8_t hdr = rd(phys++);
    if (hdr == 0xFF)
      break;
    uint16_t offset;
    uint8_t word_en;
    if ((hdr & 0x1F) == 0x0F) { /* extended header */
      uint8_t ext = rd(phys++);
      if ((ext & 0x0F) == 0x0F)
        continue;
      offset = static_cast<uint16_t>(((ext & 0xF0) >> 1) | ((hdr & 0xE0) >> 5));
      word_en = ext & 0x0F;
    } else {
      offset = (hdr >> 4) & 0x0F;
      word_en = hdr & 0x0F;
    }
    uint16_t base = static_cast<uint16_t>(offset << 3);
    for (uint8_t i = 0; i < 4; i++) {
      if (word_en & (1u << i))
        continue;
      for (uint8_t k = 0; k < 2; k++) {
        uint8_t d = rd(phys++);
        uint16_t idx = static_cast<uint16_t>(base + i * 2 + k);
        if (idx < sizeof(map))
          map[idx] = d;
      }
    }
    if (base > kRfeOff + 8)
      break;
  }
  uint8_t rfe = map[kRfeOff];
  /* Unprogrammed efuse (0xFF) falls back to rfe_type 0 — matches the vendor
   * Hal_ReadRFEType_8822b error path (rtl8822b_ops.c). Many retail 8812BU/
   * 8822BU dongles (incl. the Archer T3U) ship with a blank RFE byte. */
  if (rfe == 0xFF) {
    _logger->info("Jaguar2: EFUSE rfe_type unprogrammed (0xff) -> default 0");
    rfe = 0;
  } else {
    _logger->info("Jaguar2: EFUSE rfe_type=0x{:02x}", rfe);
  }
  return rfe;
}

void HalJaguar2::phydm_pre_post_setting(bool post) {
  /* config_phydm_parameter_init_8822b: 0x808[29:28] = 0 (pre, disable OFDM/CCK)
   * or 0x3 (post, enable). */
  _device.phy_set_bb_reg(0x808, (1u << 28) | (1u << 29), post ? 0x3 : 0x0);
}

void HalJaguar2::bb_write(uint32_t addr, uint32_t value) {
  switch (addr) {
  case 0xfe: std::this_thread::sleep_for(std::chrono::milliseconds(50)); return;
  case 0xfd: std::this_thread::sleep_for(std::chrono::milliseconds(5)); return;
  case 0xfc: std::this_thread::sleep_for(std::chrono::milliseconds(1)); return;
  case 0xfb: std::this_thread::sleep_for(std::chrono::microseconds(50)); return;
  case 0xfa: std::this_thread::sleep_for(std::chrono::microseconds(5)); return;
  case 0xf9: std::this_thread::sleep_for(std::chrono::microseconds(1)); return;
  default: break;
  }
  _device.phy_set_bb_reg(static_cast<uint16_t>(addr), 0xFFFFFFFFu, value);
  std::this_thread::sleep_for(std::chrono::microseconds(1));
}

void HalJaguar2::rf_write(uint8_t path, uint32_t addr, uint32_t value) {
  if (addr == 0xfe || addr == 0xffe) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return;
  }
  /* Jaguar 3-wire LSSI write: BB reg 0xC90 (path A) / 0xE90 (path B),
   * data = (rf_addr << 20) | (rf_data & 0xfffff), masked to 28 bits. */
  const uint16_t lssi = (path == 0) ? 0x0C90 : 0x0E90;
  uint32_t data_and_addr =
      (((addr & 0xff) << 20) | (value & 0x000fffffu)) & 0x0fffffffu;
  _device.phy_set_bb_reg(lssi, 0xFFFFFFFFu, data_and_addr);
  std::this_thread::sleep_for(std::chrono::microseconds(1));
}

void HalJaguar2::apply_bb_rf_agc_tables(uint8_t rfe_type) {
  JaguarPhyContext ctx{};
  ctx.cut_version = _ver.cut;
  ctx.support_interface = 0x02; /* ODM_ITRF_USB */
  ctx.support_platform = 0x04;  /* ODM_CE */
  ctx.package_type = 0;
  ctx.rfe_type = rfe_type;

  /* rtl8822b_phy.c order: PRE -> init_bb_reg (phy_reg, agc_tab) ->
   * init_rf_reg (radioa, radiob) -> POST. */
  phydm_pre_post_setting(/*post=*/false);

  auto bb = [this](uint32_t a, uint32_t v) { bb_write(a, v); };
  const auto phy_reg = PhyTableLoaderJaguar2::phy_reg();
  const auto agc_tab = PhyTableLoaderJaguar2::agc_tab();
  _logger->info("Jaguar2: applying BB phy_reg ({} words) + agc_tab ({} words)",
                phy_reg.len, agc_tab.len);
  PhyTableLoader::Load(phy_reg.data, phy_reg.len, ctx, bb);
  PhyTableLoader::Load(agc_tab.data, agc_tab.len, ctx, bb);

  const auto radioa = PhyTableLoaderJaguar2::radioa();
  const auto radiob = PhyTableLoaderJaguar2::radiob();
  _logger->info("Jaguar2: applying RF radioa ({} words) + radiob ({} words)",
                radioa.len, radiob.len);
  PhyTableLoader::Load(radioa.data, radioa.len, ctx,
                       [this](uint32_t a, uint32_t v) { rf_write(0, a, v); });
  PhyTableLoader::Load(radiob.data, radiob.len, ctx,
                       [this](uint32_t a, uint32_t v) { rf_write(1, a, v); });

  phydm_pre_post_setting(/*post=*/true);
  _logger->info("Jaguar2: BB/AGC/RF tables applied (rfe_type=0x{:02x})",
                rfe_type);
}

namespace {
uint32_t bit_shift(uint32_t mask) {
  if (mask == 0)
    return 0;
  uint32_t s = 0;
  while (!((mask >> s) & 1u))
    s++;
  return s;
}
} /* namespace */

uint32_t HalJaguar2::rf_read(uint8_t path, uint32_t addr) {
  /* config_phydm_read_rf_reg_8822b: 8822B reads RF registers through a direct
   * BB shadow window (NOT the 3-wire LSSI readback the old driver PHY_QueryRFReg
   * uses) — base 0x2800 (path A) / 0x2c00 (path B) + (rf_addr & 0xff) << 2,
   * masked to the 20-bit RF register width. This is the mechanism all phydm/RF
   * code (channel set, IQK) uses; the 3-wire readback returns stale/garbage and
   * corrupted every masked RF read-modify-write. */
  const uint32_t base = (path == 0) ? 0x2800u : 0x2c00u;
  const uint16_t direct = static_cast<uint16_t>(base + ((addr & 0xff) << 2));
  return _device.rtw_read32(direct) & 0x000FFFFFu;
}

void HalJaguar2::rf_set(uint8_t path, uint32_t addr, uint32_t mask,
                        uint32_t value) {
  if (mask == 0)
    return;
  uint32_t data = value;
  if (mask != 0x000FFFFF) { /* not a full-register (RFREGOFFSETMASK) write */
    uint32_t orig = rf_read(path, addr);
    data = (orig & ~mask) | (value << bit_shift(mask));
  }
  rf_write(path, addr, data);
}

/* phydm_rfe_ifem (rfe_type 0 path) — antenna-switch pins for 2.4G/5G. */
void HalJaguar2::rfe_ifem(uint8_t channel) {
  const bool g2 = channel <= 14;
  if (g2) {
    _device.phy_set_bb_reg(0x0cb0, 0xffffff, 0x745774);
    _device.phy_set_bb_reg(0x0eb0, 0xffffff, 0x745774);
    _device.phy_set_bb_reg(0x0cb4, 0x0000ff00, 0x57);
    _device.phy_set_bb_reg(0x0eb4, 0x0000ff00, 0x57);
  } else {
    _device.phy_set_bb_reg(0x0cb0, 0xffffff, 0x477547);
    _device.phy_set_bb_reg(0x0eb0, 0xffffff, 0x477547);
    _device.phy_set_bb_reg(0x0cb4, 0x0000ff00, 0x75);
    _device.phy_set_bb_reg(0x0eb4, 0x0000ff00, 0x75);
  }
  _device.phy_set_bb_reg(0x0cbc, 0x3f, 0x0);
  _device.phy_set_bb_reg(0x0cbc, (1u << 11) | (1u << 10), 0x0);
  _device.phy_set_bb_reg(0x0ebc, 0x3f, 0x0);
  _device.phy_set_bb_reg(0x0ebc, (1u << 11) | (1u << 10), 0x0);
  /* antenna switch table: 2T2R => 2TX/2RX branch (rx_ant_status == BB_PATH_AB) */
  if (g2) {
    _device.phy_set_bb_reg(0x0ca0, 0x0000ffff, 0xa501);
    _device.phy_set_bb_reg(0x0ea0, 0x0000ffff, 0xa501);
  } else {
    _device.phy_set_bb_reg(0x0ca0, 0x0000ffff, 0xa5a5);
    _device.phy_set_bb_reg(0x0ea0, 0x0000ffff, 0xa5a5);
  }
}

void HalJaguar2::igi_toggle() {
  uint32_t igi = _device.rtw_read32(0x0c50) & 0x7f;
  _device.phy_set_bb_reg(0x0c50, 0x7f, igi - 2);
  _device.phy_set_bb_reg(0x0c50, 0x7f, igi);
  _device.phy_set_bb_reg(0x0e50, 0x7f, igi - 2);
  _device.phy_set_bb_reg(0x0e50, 0x7f, igi);
}

void HalJaguar2::set_channel_bw(uint8_t channel, uint8_t bw, uint8_t rfe_type) {
  const bool g2 = channel <= 14;
  const bool r2t2r = _ver.rf_2t2r != 0;

  /* --- config_phydm_switch_channel_8822b --- */
  rfe_ifem(channel); /* RFE pins for the band (rfe_type 0) */
  (void)rfe_type;

  uint32_t rf18 = rf_read(0, 0x18);
  rf18 &= ~((1u << 18) | (1u << 17) | 0xffu); /* clear band/MASKBYTE0 */
  rf18 |= channel;

  if (g2) {
    _device.phy_set_bb_reg(0x0958, 0x1f, 0x0);       /* AGC table idx 0 */
    _device.phy_set_bb_reg(0x0860, 0x1ffe0000, 0x96a); /* fc for CFO tracking */
    if (channel == 14) {
      _device.phy_set_bb_reg(0x0a24, 0xffffffff, 0x00006577);
      _device.phy_set_bb_reg(0x0a28, 0x0000ffff, 0x0000);
    } else {
      _device.phy_set_bb_reg(0x0a24, 0xffffffff, 0x384f6577);
      _device.phy_set_bb_reg(0x0a28, 0x0000ffff, 0x1525);
    }
  } else {
    /* 5G AGC table + fc (config_phydm_switch_channel_8822b 5G branch) */
    if (channel >= 36 && channel <= 64)
      _device.phy_set_bb_reg(0x0958, 0x1f, 0x1);
    else if (channel >= 100 && channel <= 144)
      _device.phy_set_bb_reg(0x0958, 0x1f, 0x2);
    else if (channel >= 149)
      _device.phy_set_bb_reg(0x0958, 0x1f, 0x3);
    if (channel >= 36 && channel <= 48)
      _device.phy_set_bb_reg(0x0860, 0x1ffe0000, 0x494);
    else if (channel >= 52 && channel <= 64)
      _device.phy_set_bb_reg(0x0860, 0x1ffe0000, 0x453);
    else if (channel >= 100 && channel <= 116)
      _device.phy_set_bb_reg(0x0860, 0x1ffe0000, 0x452);
    else if (channel >= 118 && channel <= 177)
      _device.phy_set_bb_reg(0x0860, 0x1ffe0000, 0x412);
  }

  /* RF 0xBE[17:15] phase-noise: 0 for 2.4G (5G low/mid/high tables omitted —
   * 20MHz 2.4G RX is the current target). */
  rf_set(0, 0xbe, (1u << 17) | (1u << 16) | (1u << 15), g2 ? 0x0 : 0x0);

  if (channel == 144) {
    rf_set(0, 0xdf, (1u << 18), 0x1);
    rf18 |= (1u << 17);
  } else {
    rf_set(0, 0xdf, (1u << 18), 0x0);
    if (channel > 144)
      rf18 |= (1u << 18);
    else if (channel >= 80)
      rf18 |= (1u << 17);
  }

  /* --- config_phydm_switch_bandwidth_8822b (20 MHz) --- */
  uint32_t v8ac = _device.rtw_read32(0x08ac);
  v8ac &= 0xFFCFFC00;
  v8ac |= 0x0; /* CHANNEL_WIDTH_20 = 0 */
  _device.phy_set_bb_reg(0x08ac, 0xffffffff, v8ac);
  _device.phy_set_bb_reg(0x08c4, (1u << 30), 0x1); /* ADC buffer clock */
  rf18 |= (1u << 11) | (1u << 10);                 /* RF BW 20M */

  /* Mask RF18 (RF_CHNLBW) to only its valid fields: channel [7:0], RF-BW-mode
   * [11:10], band-select [18:17]. The RF radioA table leaves spurious bits 8
   * and 16 set; the read-modify-write above preserves them, and those extra
   * bits detune the synth so the OFDM demod never locks (energy detected, zero
   * PSDUs to MAC). The kernel driver's RF18 is a clean channel|BW value. This
   * masking was the missing piece for first RX. */
  rf18 &= 0x00060cffu;

  rf_write(0, 0x18, rf18);
  if (r2t2r)
    rf_write(1, 0x18, rf18);

  /* RF read-error debug toggle (RF_A 0xb8[19] 0->1) */
  rf_set(0, 0xb8, (1u << 19), 0);
  rf_set(0, 0xb8, (1u << 19), 1);

  /* --- phydm_rxdfirpar_by_bw_8822b (RX digital filter, BW20 path) ---
   * The RX DFIR must match the bandwidth or the OFDM demod filter is wrong:
   * energy is detected (CCA/FA fire) but no PSDU ever completes and nothing
   * reaches the MAC RX FIFO. This was the missing piece for first RX. */
  _device.phy_set_bb_reg(0x0948, (1u << 29) | (1u << 28), 0x2);
  _device.phy_set_bb_reg(0x094c, (1u << 29) | (1u << 28), 0x2);
  _device.phy_set_bb_reg(0x0c20, (1u << 31), 0x1);
  _device.phy_set_bb_reg(0x0e20, (1u << 31), 0x1);

  /* --- phydm_ccapar_by_rfe_8822b (CCA thresholds; rfe_type 0 => iFEM C-cut) ---
   * col: 2G/5G x 1R/2R. reg82c/830/838 from cca_ifem_ccut. */
  {
    const int col = g2 ? (r2t2r ? 1 : 0) : (r2t2r ? 3 : 2);
    static const uint32_t cca_ifem[3][4] = {
        {0x75C97010, 0x75C97010, 0x75C97010, 0x75C97010}, /* 0x82c */
        {0x79a0eaaa, 0x79A0EAAC, 0x79a0eaaa, 0x79a0eaaa}, /* 0x830 */
        {0x87765541, 0x87746341, 0x87765541, 0x87746341}, /* 0x838 */
    };
    _device.phy_set_bb_reg(0x082c, 0xffffffff, cca_ifem[0][col]);
    _device.phy_set_bb_reg(0x0830, 0xffffffff, cca_ifem[1][col]);
    _device.phy_set_bb_reg(0x0838, 0xffffffff, cca_ifem[2][col]);
  }

  /* RX-path toggle (0x808 MASKBYTE0) to leave RX dead-zone, then IGI. rx_ant
   * = A+B (0x3) for 2T2R => 0x33 (CCK+OFDM both paths). */
  uint8_t rx_ant = r2t2r ? 0x3 : 0x1;
  _device.phy_set_bb_reg(0x0808, 0xff, 0x0);
  _device.phy_set_bb_reg(0x0808, 0xff, rx_ant | (rx_ant << 4));
  igi_toggle();

  _logger->info("Jaguar2: channel set ch={} bw={} (rf18=0x{:05x})", channel,
                (int)bw, rf18);
}

void HalJaguar2::config_trx_mode() {
  const uint8_t path = _ver.rf_2t2r ? 0x3 : 0x1; /* BB_PATH_AB or A */

  /* RF mode table (3-wire state per path: 0 shutdown/1 standby/2 TX/3 RX). */
  _device.phy_set_bb_reg(0x0c08, 0x0000ffff, 0x3231);
  if (_ver.rf_2t2r)
    _device.phy_set_bb_reg(0x0e08, 0x0000ffff, 0x3231);

  /* --- phydm_config_tx_path_8822b (antenna-path HW-block enable + TX logic
   * map). tx_path = AB (2T2R) or A (1T1R); tx_path_sel_1ss = A. --- */
  _device.phy_set_bb_reg(0x093c, (1u << 19) | (1u << 18), 0x3);
  _device.phy_set_bb_reg(0x080c, (1u << 29) | (1u << 28), 0x1);
  _device.phy_set_bb_reg(0x080c, (1u << 30), 0x1); /* CCK TX path via 0xa07[7] */
  _device.phy_set_bb_reg(0x080c, 0xff, (path << 4) | path);
  /* CCK TX path (phydm_config_cck_tx_path_8822b): AB->0xc, A->0x8. */
  _device.phy_set_bb_reg(0x0a04, 0xf0000000, _ver.rf_2t2r ? 0xc : 0x8);
  /* OFDM TX path (phydm_config_ofdm_tx_path_8822b, 1ss=A). */
  _device.phy_set_bb_reg(0x093c, 0xfff00000, 0x001);
  if (_ver.rf_2t2r) {
    _device.phy_set_bb_reg(0x0940, 0xfff0, 0x043);
  } else {
    _device.phy_set_bb_reg(0x0940, 0xf0, 0x1);
    _device.phy_set_bb_reg(0x0940, 0xff00, 0x0);
  }

  /* --- phydm_config_rx_path_8822b(AB) --- */
  _device.phy_set_bb_reg(0x0a2c, (1u << 22), 0x0); /* disable MRC CCK CCA */
  _device.phy_set_bb_reg(0x0a2c, (1u << 18), 0x0); /* disable MRC CCK barker */
  _device.phy_set_bb_reg(0x0a04, 0x0f000000, 0x0); /* CCK RX path A first */
  _device.phy_set_bb_reg(0x0808, 0xff, (path << 4) | path); /* RX path enable */
  if (_ver.rf_2t2r) {
    _device.phy_set_bb_reg(0x1904, (1u << 16), 0x1); /* antenna weighting */
    _device.phy_set_bb_reg(0x0800, (1u << 28), 0x1); /* htstf ant-wgt */
    _device.phy_set_bb_reg(0x0850, (1u << 23), 0x1); /* MRC modified-ZF */
  } else {
    _device.phy_set_bb_reg(0x1904, (1u << 16), 0x0);
    _device.phy_set_bb_reg(0x0800, (1u << 28), 0x0);
    _device.phy_set_bb_reg(0x0850, (1u << 23), 0x0);
  }

  /* RF mode-table sync: poll RF_A 0x33 until it reads back 0x1. */
  for (int i = 0; i < 100; i++) {
    rf_write(0, 0xef, 0x80000);
    rf_write(0, 0x33, 0x00001);
    std::this_thread::sleep_for(std::chrono::microseconds(2));
    if ((rf_read(0, 0x33) & 0xfffff) == 0x00001)
      break;
  }
  /* Normal mode (no path-B-only TRX): 0xef/0x33/0x3e/0x3f then 0xef=0. */
  rf_write(0, 0xef, 0x80000);
  rf_write(0, 0x33, 0x00001);
  rf_write(0, 0x3e, 0x00034);
  rf_write(0, 0x3f, 0x4080c);
  rf_write(0, 0xef, 0x00000);
  _logger->info("Jaguar2: trx_mode configured (path=0x{:x})", path);
}

void HalJaguar2::do_lck() {
  /* aac_check_8822b (once): fix the AAC code if out of range. */
  if (!_aac_checked) {
    uint32_t temp = (rf_read(0, 0xc9) & 0xf8) >> 3;
    if (temp < 4 || temp > 7) {
      rf_set(0, 0xca, (1u << 19), 0x0);
      rf_set(0, 0xb2, 0x7c000, 0x6);
    }
    _aac_checked = true;
  }

  uint32_t c00 = _device.rtw_read32(0x0c00);
  uint32_t e00 = _device.rtw_read32(0x0e00);
  _device.rtw_write32(0x0c00, 0x4);
  _device.rtw_write32(0x0e00, 0x4);
  rf_write(0, 0x0, 0x10000);
  if (_ver.rf_2t2r)
    rf_write(1, 0x0, 0x10000);

  uint32_t lc_cal = rf_read(0, 0x18); /* backup RF CHNLBW */
  rf_write(0, 0xc4, 0x01402);         /* disable RTK */
  rf_write(0, 0x18, lc_cal | 0x08000); /* start LCK */
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  for (int cnt = 0; cnt < 5; cnt++) {
    if ((rf_read(0, 0x18) & 0x8000) == 0)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  rf_write(0, 0x18, lc_cal);  /* recover channel */
  rf_write(0, 0xc4, 0x81402); /* enable RTK */
  _device.rtw_write32(0x0c00, c00);
  _device.rtw_write32(0x0e00, e00);
  rf_write(0, 0x0, 0x3ffff);
  if (_ver.rf_2t2r)
    rf_write(1, 0x0, 0x3ffff);
  _logger->info("Jaguar2: LCK done");
}

/* ex_hal8822b_wifi_only_hw_config: grant the antenna to WLAN. Without this the
 * combo chip leaves the antenna switch owned by BT and the WL RX is deaf. */
void HalJaguar2::coex_wlan_only() {
  _device.phy_set_bb_reg(0x004c, 0x01800000, 0x2); /* BB control */
  _device.phy_set_bb_reg(0x0cb4, 0xff, 0x77);      /* SW control */
  _device.phy_set_bb_reg(0x0974, 0x300, 0x3);      /* antenna mux switch */
  _device.phy_set_bb_reg(0x1990, 0x300, 0x0);
  _device.phy_set_bb_reg(0x0cbc, 0x80000, 0x0);
  _device.phy_set_bb_reg(0x0070, 0xff000000, 0x0e); /* WL-side controller */
  _device.phy_set_bb_reg(0x1704, 0xffffffff, 0x7700);     /* gnt_wl=1 gnt_bt=0 */
  _device.phy_set_bb_reg(0x1700, 0xffffffff, 0xc00f0038);
  _logger->info("Jaguar2: coex WL-only antenna grant applied");
}

void HalJaguar2::set_tx_power_flat(uint8_t idx) {
  idx &= 0x3f;
  const uint32_t v = static_cast<uint32_t>(idx) * 0x01010101u;
  for (uint16_t off = 0; off <= 0x54; off += 4) {
    _device.rtw_write32(static_cast<uint16_t>(0x1d00 + off), v); /* path A */
    _device.rtw_write32(static_cast<uint16_t>(0x1d80 + off), v); /* path B */
  }
  _logger->info("Jaguar2: TXAGC flat index 0x{:02x} applied", idx);
}

void HalJaguar2::dig_step() {
  /* Per-window false-alarm counts (accumulated since the last reset below). */
  uint32_t ofdm_fa = _device.rtw_read32(0x0f48) & 0xffff;
  uint32_t cck_fa = _device.rtw_read32(0x0a5c) & 0xffff;
  uint32_t fa = ofdm_fa + cck_fa;
  _last_fa = fa;

  /* Reset the hold-type FA/CCA counters so the next window is a fresh delta
   * (11AC phydm_reset_bb_hw_cnt path): OFDM-FA 0x9a4[17] (1->0 = reset->enable),
   * CCK-FA 0xa2c[15] (0->1), CCA 0xb58[0] (1->0). */
  _device.phy_set_bb_reg(0x09a4, 1u << 17, 1);
  _device.phy_set_bb_reg(0x09a4, 1u << 17, 0);
  _device.phy_set_bb_reg(0x0a2c, 1u << 15, 0);
  _device.phy_set_bb_reg(0x0a2c, 1u << 15, 1);
  _device.phy_set_bb_reg(0x0b58, 1u << 0, 1);
  _device.phy_set_bb_reg(0x0b58, 1u << 0, 0);

  uint8_t igi = static_cast<uint8_t>(_device.rtw_read8(0x0c50) & 0x7f);
  uint8_t ni = igi;
  if (fa > 750)
    ni = static_cast<uint8_t>(igi + 2);
  else if (fa > 500)
    ni = static_cast<uint8_t>(igi + 1);
  else if (fa < 250)
    ni = static_cast<uint8_t>(igi >= 2 ? igi - 2 : igi);
  if (ni < 0x1c)
    ni = 0x1c;
  if (ni > 0x3e)
    ni = 0x3e;
  if (ni != igi) {
    _device.phy_set_bb_reg(0x0c50, 0x7f, ni);
    _device.phy_set_bb_reg(0x0e50, 0x7f, ni);
  }
}

void HalJaguar2::enable_rx() {
  /* CR (0x100) full MAC enable: TRX-DMA | PROTOCOL | SCHEDULE | MACTX | MACRX
   * (+ENSWBCN), matching the jaguar3 RX-enable value 0x06FF. init_mac_cfg only
   * set the DMA bits; without MACRXEN (BIT7) the MAC RX engine never runs. */
  _device.rtw_write16(0x0100, 0x06FF);
  /* Promiscuous RX for monitor. The frame-type-accept bits are the critical
   * ones: ADF(BIT11)/ACF(BIT12)/AMF(BIT13) gate data/control/management frames
   * at WMAC — without them the BB decodes frames (CRC-OK) but the MAC RX FIFO
   * stays empty (RXPKT_NUM=0). Value = the vendor monitor RCR (hal_com.c:
   * RCR_AAP|APM|AM|AB|APWRMGT|ADF|AMF|APP_PHYST_RXFF|APP_MIC|APP_ICV) plus ACF
   * so control frames are captured too:
   *   0x7000282F | ACF(0x1000) = 0x7000382F. */
  uint32_t rcr = 0x7000382Fu;
  /* DEVOURER_RX_KEEP_CORRUPTED: also accept CRC32/ICV-error frames (ACRC32 BIT8,
   * AICV BIT9) so the BB's demodulated-but-failed frames still reach the host.
   * Doubles as a bring-up discriminator: if reads>0 with this set but 0 without,
   * MAC->USB delivery works and only clean-decode is marginal. */
  if (getenv("DEVOURER_RX_KEEP_CORRUPTED"))
    rcr |= (1u << 8) | (1u << 9);
  _device.rtw_write32(0x0608, rcr);

  /* Interim fixed IGI (initial gain) until the phydm DIG thread is ported: the
   * BB/AGC table default leaves IGI too low, so the RX drowns in false alarms
   * (~4000/s) and rarely decodes a clean frame. IGI 0x40 drops the FA rate ~7x
   * and yields clean OFDM CRC-OK frames. */
  uint8_t igi = 0x40;
  if (const char *e = getenv("DEVOURER_IGI"))
    igi = static_cast<uint8_t>(strtol(e, nullptr, 0) & 0x7f);
  _device.phy_set_bb_reg(0x0c50, 0x7f, igi);
  _device.phy_set_bb_reg(0x0e50, 0x7f, igi);
  _logger->info("Jaguar2: RX enabled (CR=0x06ff, RCR=0x{:08x}, IGI=0x{:02x})",
                rcr, igi);
}

} /* namespace jaguar2 */
