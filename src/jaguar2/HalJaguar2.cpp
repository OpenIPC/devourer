#include "HalJaguar2.h"

#include <chrono>
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
  /* Jaguar 3-wire LSSI read (phy_RFSerialRead): write the RF offset to
   * rHSSIRead_Jaguar (0x8B0[7:0]), then read back the 20-bit value from the
   * SI/PI readback register per the path's PI-mode bit. */
  constexpr uint16_t rHSSIRead = 0x08B0;
  constexpr uint32_t bHSSIRead_addr = 0xFF;
  constexpr uint32_t rRead_data = 0x000FFFFF;
  const uint16_t pi_sel = (path == 0) ? 0x0C00 : 0x0E00;
  const uint16_t si_read = (path == 0) ? 0x0D08 : 0x0D48;
  const uint16_t pi_read = (path == 0) ? 0x0D04 : 0x0D44;

  if (addr != 0x0)
    _device.phy_set_bb_reg(0x0838, 0x8, 1); /* rCCAonSec: CCA off while reading */

  _device.phy_set_bb_reg(rHSSIRead, bHSSIRead_addr, addr & 0xff);

  /* C-cut needs a settle before the readback latches. */
  if (_ver.cut >= 2)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

  uint32_t pi_mode = (_device.rtw_read32(pi_sel) >> 2) & 0x1;
  uint16_t rb = pi_mode ? pi_read : si_read;
  uint32_t v = _device.rtw_read32(rb);
  return (v & rRead_data) >> bit_shift(rRead_data);
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

  rf_write(0, 0x18, rf18);
  if (r2t2r)
    rf_write(1, 0x18, rf18);

  /* RF read-error debug toggle (RF_A 0xb8[19] 0->1) */
  rf_set(0, 0xb8, (1u << 19), 0);
  rf_set(0, 0xb8, (1u << 19), 1);

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

  /* --- phydm_config_tx_path_8822b (antenna-path HW-block enable; the CCK/OFDM
   * TX-logic-map sub-config is deferred to the TX milestone) --- */
  _device.phy_set_bb_reg(0x093c, (1u << 19) | (1u << 18), 0x3);
  _device.phy_set_bb_reg(0x080c, (1u << 29) | (1u << 28), 0x1);
  _device.phy_set_bb_reg(0x080c, (1u << 30), 0x1);
  _device.phy_set_bb_reg(0x080c, 0xff, (path << 4) | path);

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

void HalJaguar2::enable_rx() {
  /* CR (0x100) full MAC enable: TRX-DMA | PROTOCOL | SCHEDULE | MACTX | MACRX
   * (+ENSWBCN), matching the jaguar3 RX-enable value 0x06FF. init_mac_cfg only
   * set the DMA bits; without MACRXEN (BIT7) the MAC RX engine never runs. */
  _device.rtw_write16(0x0100, 0x06FF);
  /* Promiscuous RX for monitor: accept all frames (RCR AAP|APM|AM|AB|APWRMGT|
   * ADF|AMF|HTC-LOC + APP_PHYST). */
  _device.rtw_write32(0x0608, 0xF400220E | (1u << 28) | (1u << 22));
  _logger->info("Jaguar2: RX enabled (CR=0x06ff, RCR promiscuous)");
}

} /* namespace jaguar2 */
