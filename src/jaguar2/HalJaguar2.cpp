#include "HalJaguar2.h"

#include <chrono>
#include <cstdlib>
#include <stdexcept>
#include <thread>
#include <utility>

#if defined(DEVOURER_HAVE_JAGUAR2_8822B)
#include "Hal8822b_TxpwrLmt.h" /* generated: hal8822b_txpwr_lmt() WW-min limits */
#endif
#if defined(DEVOURER_HAVE_JAGUAR2_8821C)
#include "Hal8821c_TxpwrLmt.h" /* generated: hal8821c_txpwr_lmt() WW-min limits */
#endif
#include "PhyTableLoader.h"

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

/* card_en_flow_8821c = CARDDIS_TO_CARDEMU + CARDEMU_TO_ACT (USB/ALL),
 * transcribed from reference/8821cu/hal/halmac/halmac_88xx/halmac_8821c/
 * halmac_pwr_seq_8821c.c. Every row is PWR_CUT_ALL_MSK (no cut-specific rows,
 * unlike 8822B). PCI/SDIO-only rows dropped. */
const PwrCfg kPwrOn8821cUsb[] = {
    /* --- card-disable -> card-emulation --- */
    {0x004A, PC_WRITE, B(0), 0},
    {0x0005, PC_WRITE, static_cast<uint8_t>(B(3) | B(4) | B(7)), 0},
    /* --- card-emulation -> active --- */
    {0x0020, PC_WRITE, B(0), B(0)},
    {0x0001, PC_DELAY, 0, 1}, /* 1 ms */
    {0x0000, PC_WRITE, B(5), 0},
    {0x0005, PC_WRITE, static_cast<uint8_t>(B(4) | B(3) | B(2)), 0},
    {0x0006, PC_POLL, B(1), B(1)},
    {0x0006, PC_WRITE, B(0), B(0)},
    {0x0005, PC_WRITE, B(7), 0},
    {0x0005, PC_WRITE, static_cast<uint8_t>(B(4) | B(3)), 0},
    {0x10C3, PC_WRITE, B(0), B(0)},
    {0x0005, PC_WRITE, B(0), B(0)},
    {0x0005, PC_POLL, B(0), 0},
    {0x0020, PC_WRITE, B(3), B(3)},
    {0x007C, PC_WRITE, B(1), 0},
    {0, PC_END, 0, 0},
};

/* card_dis_flow_8821c = ACT_TO_CARDEMU + CARDEMU_TO_CARDDIS (USB/ALL). */
const PwrCfg kPwrOff8821cUsb[] = {
    /* --- active -> card-emulation --- */
    {0x0093, PC_WRITE, 0xFF, 0xC4},
    {0x001F, PC_WRITE, 0xFF, 0},
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

HalJaguar2::HalJaguar2(RtlUsbAdapter device, Logger_t logger,
                       ChipVariant variant)
    : _device{std::move(device)}, _logger{std::move(logger)},
      _variant{variant}, _tables{make_jaguar2_phy_tables(variant)} {}

void HalJaguar2::power_off() {
  const PwrCfg *seq =
      _variant == ChipVariant::C8821C ? kPwrOff8821cUsb : kPwrOff8822bUsb;
  run_pwr_seq(_device, seq, /*poll_max=*/2000, /*poll_fatal=*/false);
  _logger->info("Jaguar2: power-off (card-disable) sequence applied");
}

void HalJaguar2::power_on() {
  power_off(); /* reset from any prior (kernel-left active) state first */
  const PwrCfg *seq =
      _variant == ChipVariant::C8821C ? kPwrOn8821cUsb : kPwrOn8822bUsb;
  run_pwr_seq(_device, seq, /*poll_max=*/5000, /*poll_fatal=*/true);
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
  const char *chip = _variant == ChipVariant::C8821C ? "8821C" : "8822B";
  _logger->info("Jaguar2 chip: {} cut={} ({}{}) {} (SYS_CFG1=0x{:08x})", chip,
                _ver.cut, vn[_ver.vendor], _ver.test_chip ? ",test" : "",
                _ver.rf_2t2r ? "2T2R" : "1T1R", v);
}

void HalJaguar2::read_efuse_logical_map(uint8_t *map, uint16_t map_size,
                                        bool dump) {
  /* Standard 88xx EFUSE logical-map decode (efuse_OneByteRead + 2-byte
   * extended-header format). Ported from the jaguar3 non-EU path; 8822B uses
   * the same physical primitive.
   *
   * The physical efuse stores logical words in an arbitrary order (a block for a
   * high logical offset can appear physically before a block for a low offset),
   * so the whole chain must be walked to the natural end (hdr == 0xFF). An
   * earlier version broke out as soon as a block's base exceeded a target offset
   * — but the VID/PID block (logical base 0x100) appears near the *start* of the
   * physical layout, so the walk aborted long before reaching the physically-
   * later blocks (RFE 0xCA always read back 0xFF). */
  constexpr uint16_t kPhysMax = 1024;
  for (uint16_t i = 0; i < map_size; i++)
    map[i] = 0xFF;

  auto rd = [this](uint16_t a) -> uint8_t {
    uint8_t d = 0xFF;
    return _device.efuse_OneByteRead(a, &d) ? d : 0xFF;
  };

  if (dump) {
    /* Raw physical efuse dump (first 0x100 bytes) for hand-decoding the header
     * chain — proves whether the logical-map walk sees the programmed words. */
    char line[3 * 16 + 1];
    for (uint16_t p = 0; p < 0x100; p += 16) {
      int n = 0;
      for (uint16_t i = 0; i < 16; i++)
        n += snprintf(line + n, sizeof(line) - n, "%02x ", rd(p + i));
      _logger->info("Jaguar2: EFUSE phys[{:03x}]: {}", p, line);
    }
  }

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
        if (idx < map_size)
          map[idx] = d;
      }
    }
  }
  if (dump) {
    _logger->info("Jaguar2: EFUSE dump VID={:02x}{:02x} PID={:02x}{:02x} "
                  "chplan(0xB8)={:02x} rfe(0xCA)={:02x} txcal(0xC8)={:02x}",
                  map[0x101], map[0x100], map[0x103], map[0x102], map[0xB8],
                  map[0xCA], map[0xC8]);
  }
}

uint8_t HalJaguar2::read_efuse_rfe() {
  constexpr uint16_t kRfeOff = 0xCA; /* EEPROM_RFE_OPTION_8822B */
  read_efuse_logical_map(_efuse_map, sizeof(_efuse_map),
                         getenv("DEVOURER_EFUSE_DUMP") != nullptr);
  _efuse_valid = true; /* cache for apply_tx_power (avoid a 2nd physical walk) */
  uint8_t rfe = _efuse_map[kRfeOff];
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
  /* The phydm table conditional blocks are gated on the DECODED rfe_type. On
   * 8821C the raw efuse RFE byte is `rfe_type_expand` and the value the tables
   * match against is `rfe_type_expand >> 3` (config_phydm_parameter_init_8821c:
   * `dm->rfe_type = dm->rfe_type_expand >> 3`). Feeding the raw byte (e.g. 0x22)
   * matches no rfe block, so the rfe-specific AGC/RF rows never apply. 8822B
   * uses the raw byte directly. */
  ctx.rfe_type =
      (_variant == ChipVariant::C8821C) ? (rfe_type >> 3) : rfe_type;

  /* rtl8822b_phy.c order: PRE -> init_bb_reg (phy_reg, agc_tab) ->
   * init_rf_reg (radioa, radiob) -> POST. */
  phydm_pre_post_setting(/*post=*/false);

  auto bb = [this](uint32_t a, uint32_t v) { bb_write(a, v); };
  const auto phy_reg = _tables->phy_reg();
  const auto agc_tab = _tables->agc_tab();
  _logger->info("Jaguar2: applying BB phy_reg ({} words) + agc_tab ({} words)",
                phy_reg.len, agc_tab.len);
  PhyTableLoader::Load(phy_reg.data, phy_reg.len, ctx, bb);
  PhyTableLoader::Load(agc_tab.data, agc_tab.len, ctx, bb);

  const auto radioa = _tables->radioa();
  const auto radiob = _tables->radiob();
  _logger->info("Jaguar2: applying RF radioa ({} words) + radiob ({} words)",
                radioa.len, radiob.len);
  PhyTableLoader::Load(radioa.data, radioa.len, ctx,
                       [this](uint32_t a, uint32_t v) { rf_write(0, a, v); });
  /* 1T1R chips (8821C) supply no radiob table (path A only) — skip the path-B
   * RF walk rather than feed the loader a null table. */
  if (radiob.len)
    PhyTableLoader::Load(radiob.data, radiob.len, ctx,
                         [this](uint32_t a, uint32_t v) { rf_write(1, a, v); });

  phydm_pre_post_setting(/*post=*/true);

  /* 8821C: snapshot the CCK TX-filter defaults the BB table just loaded
   * (config_phydm_parameter_init_8821c POST saves rega24/28/aac_8821c). The
   * per-channel 2.4G tune restores these for non-ch14. */
  if (_variant == ChipVariant::C8821C) {
    _cck_a24_8821c = _device.rtw_read32(0x0a24);
    _cck_a28_8821c = _device.rtw_read32(0x0a28);
    _cck_aac_8821c = _device.rtw_read32(0x0aac);
    _cck_saved_8821c = true;
  }
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

void HalJaguar2::set_channel_bw(uint8_t channel, uint8_t bw, uint8_t rfe_type,
                               uint8_t primary_ch_idx) {
  if (_variant == ChipVariant::C8821C) {
    set_channel_bw_8821c(channel, bw, rfe_type, primary_ch_idx);
    return;
  }
  /* The RF/AGC tune to the CENTRAL channel of the wide channel; `channel` is the
   * primary 20 MHz channel and primary_ch_idx its position. 20 MHz: central =
   * primary. 40 MHz: primary_ch_idx 1(lower)/2(upper) -> ±2. 80 MHz: 1..4 ->
   * +6/+2/-2/-6. (The vendor's higher layer passes the central channel to
   * config_phydm_switch_channel_8822b and primary_ch_idx to switch_bandwidth.) */
  uint8_t cch = channel;
  if (bw == 1)
    cch = static_cast<uint8_t>(primary_ch_idx == 2 ? channel - 2 : channel + 2);
  else if (bw == 2) {
    static const int off80[5] = {0, 6, 2, -2, -6};
    cch = static_cast<uint8_t>(channel +
                               off80[primary_ch_idx <= 4 ? primary_ch_idx : 0]);
  }
  const bool g2 = cch <= 14;
  const bool r2t2r = _ver.rf_2t2r != 0;

  /* --- config_phydm_switch_channel_8822b (on the central channel) --- */
  rfe_ifem(cch); /* RFE pins for the band (rfe_type 0) */
  (void)rfe_type;

  uint32_t rf18 = rf_read(0, 0x18);
  rf18 &= ~((1u << 18) | (1u << 17) | 0xffu); /* clear band/MASKBYTE0 */
  rf18 |= cch;

  if (g2) {
    _device.phy_set_bb_reg(0x0958, 0x1f, 0x0);       /* AGC table idx 0 */
    _device.phy_set_bb_reg(0x0860, 0x1ffe0000, 0x96a); /* fc for CFO tracking */
    if (cch == 14) {
      _device.phy_set_bb_reg(0x0a24, 0xffffffff, 0x00006577);
      _device.phy_set_bb_reg(0x0a28, 0x0000ffff, 0x0000);
    } else {
      _device.phy_set_bb_reg(0x0a24, 0xffffffff, 0x384f6577);
      _device.phy_set_bb_reg(0x0a28, 0x0000ffff, 0x1525);
    }
  } else {
    /* 5G AGC table + fc (config_phydm_switch_channel_8822b 5G branch) */
    if (cch >= 36 && cch <= 64)
      _device.phy_set_bb_reg(0x0958, 0x1f, 0x1);
    else if (cch >= 100 && cch <= 144)
      _device.phy_set_bb_reg(0x0958, 0x1f, 0x2);
    else if (cch >= 149)
      _device.phy_set_bb_reg(0x0958, 0x1f, 0x3);
    if (cch >= 36 && cch <= 48)
      _device.phy_set_bb_reg(0x0860, 0x1ffe0000, 0x494);
    else if (cch >= 52 && cch <= 64)
      _device.phy_set_bb_reg(0x0860, 0x1ffe0000, 0x453);
    else if (cch >= 100 && cch <= 116)
      _device.phy_set_bb_reg(0x0860, 0x1ffe0000, 0x452);
    else if (cch >= 118 && cch <= 177)
      _device.phy_set_bb_reg(0x0860, 0x1ffe0000, 0x412);
  }

  /* RF 0xBE[17:15] per-channel phase-noise / VCO-band setting
   * (config_phydm_switch_channel_8822b): 0 for 2.4G, else a per-5G-channel band
   * value from the low/middle/high-band tables. Omitting it (writing 0 on 5G)
   * leaves the 5G VCO mis-tuned so 5G RX/TX fail entirely — this was the sole
   * gap blocking 5G. Path A only (matches the vendor). */
  static const uint8_t low_band[15] = {0x7, 0x6, 0x6, 0x5, 0x0, 0x0, 0x7, 0xff,
                                       0x6, 0x5, 0x0, 0x0, 0x7, 0x6, 0x6};
  static const uint8_t middle_band[23] = {
      0x6, 0x5, 0x0, 0x0, 0x7, 0x6, 0x6, 0xff, 0x0, 0x0, 0x7, 0x6,
      0x6, 0x5, 0x0, 0xff, 0x7, 0x6, 0x6, 0x5, 0x0, 0x0, 0x7};
  static const uint8_t high_band[15] = {0x5, 0x5, 0x0, 0x7, 0x7, 0x6, 0x5, 0xff,
                                        0x0, 0x7, 0x7, 0x6, 0x5, 0x5, 0x0};
  uint8_t rf_be = 0xff;
  if (cch <= 14)
    rf_be = 0x0;
  else if (cch >= 36 && cch <= 64)
    rf_be = low_band[(cch - 36) >> 1];
  else if (cch >= 100 && cch <= 144)
    rf_be = middle_band[(cch - 100) >> 1];
  else if (cch >= 149 && cch <= 177)
    rf_be = high_band[(cch - 149) >> 1];
  if (rf_be != 0xff)
    rf_set(0, 0xbe, (1u << 17) | (1u << 16) | (1u << 15), rf_be);

  if (cch == 144) {
    rf_set(0, 0xdf, (1u << 18), 0x1);
    rf18 |= (1u << 17);
  } else {
    rf_set(0, 0xdf, (1u << 18), 0x0);
    if (cch > 144)
      rf18 |= (1u << 18);
    else if (cch >= 80)
      rf18 |= (1u << 17);
  }

  /* --- config_phydm_switch_bandwidth_8822b (20/40/80 MHz) --- */
  uint32_t v8ac = _device.rtw_read32(0x08ac);
  const uint8_t sub = static_cast<uint8_t>((primary_ch_idx & 0xf) << 2);
  if (bw == 1) { /* 40 MHz */
    _device.phy_set_bb_reg(0x0a00, 1u << 4, primary_ch_idx == 1 ? 1 : 0);
    v8ac &= 0xFF3FF300;
    v8ac |= (sub | 0x1u); /* CHANNEL_WIDTH_40 = 1 */
    _device.phy_set_bb_reg(0x08ac, 0xffffffff, v8ac);
    _device.phy_set_bb_reg(0x08c4, (1u << 30), 0x1);
    rf18 &= ~((1u << 11) | (1u << 10));
    rf18 |= (1u << 11); /* RF BW 40M */
  } else if (bw == 2) { /* 80 MHz */
    v8ac &= 0xFCEFCF00;
    v8ac |= (sub | 0x2u); /* CHANNEL_WIDTH_80 = 2 */
    _device.phy_set_bb_reg(0x08ac, 0xffffffff, v8ac);
    _device.phy_set_bb_reg(0x08c4, (1u << 30), 0x1);
    rf18 &= ~((1u << 11) | (1u << 10));
    rf18 |= (1u << 10); /* RF BW 80M */
    if (rfe_type == 2 || rfe_type == 3 || rfe_type == 17) {
      _device.phy_set_bb_reg(0x0840, 0x0000f000, 0x6);
      _device.phy_set_bb_reg(0x08c8, (1u << 10), 0x1);
    }
  } else { /* 20 MHz */
    v8ac &= 0xFFCFFC00;
    v8ac |= 0x0u; /* CHANNEL_WIDTH_20 = 0 */
    _device.phy_set_bb_reg(0x08ac, 0xffffffff, v8ac);
    _device.phy_set_bb_reg(0x08c4, (1u << 30), 0x1);
    rf18 |= (1u << 11) | (1u << 10); /* RF BW 20M */
  }

  /* RF18 final value. The vendor (config_phydm_switch_channel_8822b) preserves
   * bits 8-16 from the read and never masks — its RF18 comes out clean on 2.4G
   * (0x0c09) and with the 5G band bits (8+16) set on 5G (0x10d24). devourer's
   * direct RF18 read carries SPURIOUS bits 8/16 on 2.4G (an upstream RF-state
   * quirk), so on 2.4G we clear them to reproduce the vendor's clean value; on
   * 5G those bits are the real band indicator and must be kept (masking them was
   * what left the 5G synth in 2.4G mode -> dead 5G RX). Net: devourer's RF18 now
   * matches the vendor's per-band value. */
  if (g2)
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
  /* config_phydm_trx_mode_8821c is a no-op in the vendor (return true): the
   * 8821C RF path/mode comes from the radioa table + switch_rf_set (BTG/WLG) +
   * the 0x808 block-enable, NOT an 8822B-style RF mode-table poke. Running the
   * 8822B sequence below (RF 0xc08/0x3e/0x3f mode table, 0x808/0x940 path map)
   * on the 8821C RF drives it into a wrong mode. Skip for C8821C. */
  if (_variant == ChipVariant::C8821C)
    return;
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
  /* CCK TX path (phydm_config_cck_tx_path_8822b). The vendor calls
   * phydm_config_tx_path_8822b(dm, tx_path, tx_path_sel_1ss, tx_path_sel_1ss) —
   * i.e. tx_path_sel_cck == tx_path_sel_1ss == BB_PATH_A, so CCK is single-path A
   * (0xa04[31:28]=0x8) even on a 2T2R part. (devourer previously used the full
   * AB path 0xc here; the kernel reads 0x8.) */
  _device.phy_set_bb_reg(0x0a04, 0xf0000000, 0x8);
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
  if (_variant == ChipVariant::C8821C) {
    do_lck_8821c();
    return;
  }
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

  /* Backup RF CHNLBW using the FULL read, exactly like the vendor
   * _phy_lc_calibrate_8822b (odm_get_rf_reg RF_CHNLBW, RFREGOFFSETMASK). An
   * earlier version masked lc_cal to 0x00060cff — but on 5G that clears the band
   * bits (8/16), so the LC calibration ran against a 2.4G-band RF18 (mis-tuning
   * the 5G VCO) AND the recover-write left RF18 masked, overriding the correct
   * 5G channel from set_channel_bw. The LCK-done poll may "time out" (the direct
   * RF read returns transient status bits) but the LO does lock. */
  uint32_t lc_cal = rf_read(0, 0x18); /* backup RF CHNLBW (full, no mask) */
  rf_write(0, 0xc4, 0x01402);         /* disable RTK */
  rf_write(0, 0x18, lc_cal | 0x08000); /* start LCK */
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  bool locked = false;
  for (int cnt = 0; cnt < 5; cnt++) {
    if ((rf_read(0, 0x18) & 0x8000) == 0) { locked = true; break; }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  rf_write(0, 0x18, lc_cal);  /* recover channel */
  rf_write(0, 0xc4, 0x81402); /* enable RTK */
  _device.rtw_write32(0x0c00, c00);
  _device.rtw_write32(0x0e00, e00);
  rf_write(0, 0x0, 0x3ffff);
  if (_ver.rf_2t2r)
    rf_write(1, 0x0, 0x3ffff);
  _logger->info("Jaguar2: LCK {} (RF_A 0x18={:05x} 0x00={:05x})",
                locked ? "LOCKED" : "TIMEOUT (LO not locked!)",
                rf_read(0, 0x18), rf_read(0, 0x0));
}

/* _phy_lc_calibrate_8821c: the 8821C LCK is an RF-firmware poke sequence (no
 * host poll) — completely unlike the 8822B RF 0x18[15] toggle+poll (which just
 * times out on 8821C). aac_check first, then RF 0xcc/0xc4/0xcc. */
void HalJaguar2::do_lck_8821c() {
  if (!_aac_checked) {
    uint32_t temp = (rf_read(0, 0xc9) & 0xf8) >> 3;
    if (temp < 4 || temp > 7) {
      rf_set(0, 0xca, (1u << 19), 0x0);
      rf_set(0, 0xb2, 0x7c000, 0x6);
    }
    _aac_checked = true;
  }
  rf_write(0, 0xcc, 0x2018);
  rf_write(0, 0xc4, 0x8f602);
  rf_write(0, 0xcc, 0x201c);
  _logger->info("Jaguar2/8821C: LCK applied (RF-fw poke)");
}

/* config_phydm_switch_rf_set_8821c: select the 2.4G RX front-end path. BTG
 * (BT-shared, path B) vs WLG (WiFi-only, path A) is chosen from rfe_type_expand.
 * The base mux writes (0xcb8/0xa84/0xa80) apply in normal (non-MP) operation;
 * the agc_tab_diff overlay + IGI toggle are MP-mode-only and omitted. */
void HalJaguar2::switch_rf_set_8821c(uint8_t rf_set) {
  /* rf_set: 0=BTG (2.4G BT-shared, path B), 1=WLG (2.4G WiFi, path A),
   * 2=WLA (5G). Transcribed from config_phydm_switch_rf_set_8821c. */
  _device.phy_set_bb_reg(0x1080, (1u << 16), 0x1);
  _device.phy_set_bb_reg(0x0000, (1u << 26), 0x1);
  uint32_t cb8 = _device.rtw_read32(0x0cb8);
  if (rf_set == 0) { /* BTG */
    cb8 |= (1u << 16);
    cb8 &= ~((1u << 18) | (1u << 20) | (1u << 21) | (1u << 22) | (1u << 23));
    _device.phy_set_bb_reg(0x0a84, 0x00ff0000, 0x0e);
    _device.phy_set_bb_reg(0x0a80, 0x0000ffff, 0xfc84);
  } else if (rf_set == 1) { /* WLG */
    cb8 |= (1u << 20) | (1u << 21) | (1u << 22);
    cb8 &= ~((1u << 16) | (1u << 18) | (1u << 23));
    _device.phy_set_bb_reg(0x0a84, 0x00ff0000, 0x12);
    _device.phy_set_bb_reg(0x0a80, 0x0000ffff, 0x7532);
  } else { /* WLA (5G) — cb8 bits only, no 0xa84/0xa80 override */
    cb8 |= (1u << 20) | (1u << 22) | (1u << 23);
    cb8 &= ~((1u << 16) | (1u << 18) | (1u << 21));
  }
  _device.phy_set_bb_reg(0x0cb8, 0xffffffff, cb8);
}

/* config_phydm_switch_band/channel/bandwidth_8821c, transcribed from
 * reference/8821cu phydm_hal_api8821c.c. 1T1R, RF path A; the 2.4G RX path is
 * routed by switch_rf_set (BTG/WLG). Focused on 2.4G/20MHz (the current bring-up
 * target); 5G and 40/80 tune are handled for completeness (M8 validates them). */
void HalJaguar2::set_channel_bw_8821c(uint8_t channel, uint8_t bw,
                                      uint8_t rfe_raw, uint8_t primary_ch_idx) {
  uint8_t cch = channel;
  if (bw == 1)
    cch = static_cast<uint8_t>(primary_ch_idx == 2 ? channel - 2 : channel + 2);
  else if (bw == 2) {
    static const int off80[5] = {0, 6, 2, -2, -6};
    cch = static_cast<uint8_t>(channel +
                               off80[primary_ch_idx <= 4 ? primary_ch_idx : 0]);
  }
  const bool g2 = cch <= 14;
  /* rfe_type_expand -> BTG (else WLG) per config_phydm_parameter_init_8821c. */
  const bool btg = rfe_raw == 2 || rfe_raw == 4 || rfe_raw == 7 ||
                   rfe_raw == 0x22 || rfe_raw == 0x24 || rfe_raw == 0x27 ||
                   rfe_raw == 0x2a || rfe_raw == 0x2c || rfe_raw == 0x2f;

  /* --- config_phydm_switch_band_8821c --- */
  uint32_t rf18 = rf_read(0, 0x18);
  if (g2) {
    _device.phy_set_bb_reg(0x0808, (1u << 28), 0x1); /* enable CCK block */
    _device.phy_set_bb_reg(0x0454, (1u << 7), 0x0);  /* MAC CCK check off */
    _device.phy_set_bb_reg(0x0a80, (1u << 18), 0x0); /* BB CCK check off */
    _device.phy_set_bb_reg(0x0814, 0x0000FC00, 15);  /* CCA mask default */
    rf18 &= ~((1u << 16) | (1u << 9) | (1u << 8) | 0xffu);
    rf18 |= cch;
    switch_rf_set_8821c(btg ? 0 : 1); /* BTG or WLG */
    rf_set(0, 0xdf, (1u << 6), 0x1);   /* RF TXA_TANK LUT mode */
    rf_set(0, 0x64, 0x0000f, 0xf);     /* RF TXA_PA_TANK */
  } else {
    _device.phy_set_bb_reg(0x0a80, (1u << 18), 0x1);
    _device.phy_set_bb_reg(0x0454, (1u << 7), 0x1);
    _device.phy_set_bb_reg(0x0808, (1u << 28), 0x0);
    _device.phy_set_bb_reg(0x0814, 0x0000FC00, 15);
    rf18 &= ~((1u << 16) | (1u << 9) | (1u << 8) | 0xffu);
    rf18 |= (1u << 8) | (1u << 16) | cch;
    switch_rf_set_8821c(2); /* WLA (5G) */
    rf_set(0, 0xdf, (1u << 6), 0x0);
  }
  rf_write(0, 0x18, rf18);

  /* --- config_phydm_switch_channel_8821c --- */
  rf18 = rf_read(0, 0x18);
  rf18 &= ~((1u << 18) | (1u << 17) | 0xffu);
  rf18 |= cch;
  if (g2) {
    _device.phy_set_bb_reg(0x0c1c, 0x00000F00, 0x0);      /* AGC table idx 0 */
    _device.phy_set_bb_reg(0x0860, 0x1ffe0000, 0x96a);    /* fc for CFO track */
    if (cch == 14) {
      _device.phy_set_bb_reg(0x0a24, 0xffffffff, 0x0000b81c);
      _device.phy_set_bb_reg(0x0a28, 0x0000ffff, 0x0000);
      _device.phy_set_bb_reg(0x0aac, 0xffffffff, 0x00003667);
    } else if (_cck_saved_8821c) {
      _device.phy_set_bb_reg(0x0a24, 0xffffffff, _cck_a24_8821c);
      _device.phy_set_bb_reg(0x0a28, 0x0000ffff, _cck_a28_8821c & 0xffff);
      _device.phy_set_bb_reg(0x0aac, 0xffffffff, _cck_aac_8821c);
    }
  } else {
    if (cch >= 36 && cch <= 64)
      _device.phy_set_bb_reg(0x0c1c, 0x00000F00, 0x1);
    else if (cch >= 100 && cch <= 144)
      _device.phy_set_bb_reg(0x0c1c, 0x00000F00, 0x2);
    else if (cch >= 149)
      _device.phy_set_bb_reg(0x0c1c, 0x00000F00, 0x3);
    if (cch >= 36 && cch <= 48)
      _device.phy_set_bb_reg(0x0860, 0x1ffe0000, 0x494);
    else if (cch >= 52 && cch <= 64)
      _device.phy_set_bb_reg(0x0860, 0x1ffe0000, 0x453);
    else if (cch >= 100 && cch <= 116)
      _device.phy_set_bb_reg(0x0860, 0x1ffe0000, 0x452);
    else if (cch >= 118 && cch <= 177)
      _device.phy_set_bb_reg(0x0860, 0x1ffe0000, 0x412);
    if (cch >= 100 && cch <= 140)
      rf18 |= (1u << 17);
    else if (cch > 140)
      rf18 |= (1u << 18);
  }
  rf_write(0, 0x18, rf18);

  /* --- config_phydm_switch_bandwidth_8821c --- */
  rf18 = rf_read(0, 0x18);
  if (bw == 1) { /* 40 MHz */
    _device.phy_set_bb_reg(0x0a00, (1u << 4), primary_ch_idx == 1 ? 1 : 0);
    uint32_t v8ac = _device.rtw_read32(0x08ac);
    v8ac &= 0xff3ff300;
    v8ac |= (static_cast<uint32_t>((primary_ch_idx & 0xf) << 2) | 0x20020000u |
             0x1u);
    _device.phy_set_bb_reg(0x08ac, 0xffffffff, v8ac);
    _device.phy_set_bb_reg(0x08c4, (1u << 30), 0x1);
    rf18 &= ~((1u << 11) | (1u << 10));
    rf18 |= (1u << 11);
  } else if (bw == 2) { /* 80 MHz */
    uint32_t v8ac = _device.rtw_read32(0x08ac);
    v8ac &= 0xfcffcf00;
    v8ac |= (static_cast<uint32_t>((primary_ch_idx & 0xf) << 2) | 0x40040000u |
             0x2u);
    _device.phy_set_bb_reg(0x08ac, 0xffffffff, v8ac);
    _device.phy_set_bb_reg(0x08c4, (1u << 30), 0x1);
    rf18 &= ~((1u << 11) | (1u << 10));
    rf18 |= (1u << 10);
  } else { /* 20 MHz */
    uint32_t v8ac = _device.rtw_read32(0x08ac);
    v8ac &= 0xffcffc00;
    v8ac |= 0x10010000;
    _device.phy_set_bb_reg(0x08ac, 0xffffffff, v8ac);
    _device.phy_set_bb_reg(0x08c4, (1u << 30), 0x1);
    rf18 |= (1u << 11) | (1u << 10);
  }
  rf_write(0, 0x18, rf18);

  /* phydm_rxdfirpar_by_bw_8821c: the RX digital filter must match the bandwidth
   * or the OFDM demod never completes a PSDU (energy detected, nothing reaches
   * the MAC RX FIFO) — the 8821C "first RX" piece. 1T1R: path A regs only
   * (0x948/0x94c/0xc20/0x8f0), distinct from the 8822B set. */
  _device.phy_set_bb_reg(0x0948, (1u << 29) | (1u << 28), 0x2);
  _device.phy_set_bb_reg(0x094c, (1u << 29) | (1u << 28),
                         bw == 2 ? 0x1u : 0x2u);
  _device.phy_set_bb_reg(0x0c20, (1u << 31), bw == 0 ? 0x1u : 0x0u);
  _device.phy_set_bb_reg(0x08f0, (1u << 31), bw == 2 ? 0x1u : 0x0u);

  igi_toggle();
  _logger->info("Jaguar2/8821C: channel set ch={} bw={} cch={} {} (rf18={:05x})",
                channel, (int)bw, cch, btg ? "BTG" : "WLG", rf18);
}

/* ex_hal8822b_wifi_only_hw_config + hal8822b_wifi_only_switch_antenna: grant the
 * antenna to WLAN. Without the hw_config block the combo chip leaves the antenna
 * switch owned by BT and the WL RX is deaf. The switch_antenna write (0xcbc[9:8])
 * selects which physical antenna path the WL side drives, per band — the kernel
 * fires it from rtw_btcoex_wifionly_switchband_notify() during every channel/band
 * set. Omitting it leaves the antenna-path select bits at their reset value, so
 * WL TX/RX runs through an unselected/mismatched path (weak, distorted on air). */
void HalJaguar2::coex_wlan_only(bool is_5g) {
  if (_variant == ChipVariant::C8821C) {
    coex_wlan_only_8821c(is_5g);
    return;
  }
  _device.phy_set_bb_reg(0x004c, 0x01800000, 0x2); /* BB control */
  _device.phy_set_bb_reg(0x0cb4, 0xff, 0x77);      /* SW control */
  _device.phy_set_bb_reg(0x0974, 0x300, 0x3);      /* antenna mux switch */
  _device.phy_set_bb_reg(0x1990, 0x300, 0x0);
  _device.phy_set_bb_reg(0x0cbc, 0x80000, 0x0);
  _device.phy_set_bb_reg(0x0070, 0xff000000, 0x0e); /* WL-side controller */
  _device.phy_set_bb_reg(0x1704, 0xffffffff, 0x7700);     /* gnt_wl=1 gnt_bt=0 */
  _device.phy_set_bb_reg(0x1700, 0xffffffff, 0xc00f0038);
  /* hal8822b_wifi_only_switch_antenna(is_5g): 0xcbc[9:8] = 1 (5G) / 2 (2.4G) */
  _device.phy_set_bb_reg(0x0cbc, 0x300, is_5g ? 0x1 : 0x2);
  _logger->info("Jaguar2: coex WL-only antenna grant applied (is_5g={})",
                is_5g ? 1 : 0);
}

/* ex_hal8821c_wifi_only_hw_config + hal8821c_wifi_only_switch_antenna
 * (reference/8821cu/hal/btc/halbtc8821cwifionly.c). The 8821C wifi-only coex
 * differs from the 8822B: the GNT/owner and the SPDT/DPDT external-antenna
 * switch use different registers (0x70[26], 0x1700/0x1704, 0x6c0/0x6c4, and the
 * antenna switch on 0xcb4[29:28] driven by rfe_module_type — NOT the 8822B
 * 0xcbc[9:8]). */
void HalJaguar2::coex_wlan_only_8821c(bool is_5g) {
  /* hw_config: GNT owner -> WL, gnt_wl=1/gnt_bt=0. */
  _device.phy_set_bb_reg(0x0070, 0x04000000, 0x1);
  _device.phy_set_bb_reg(0x1704, 0xffffffff, 0x7700);
  _device.phy_set_bb_reg(0x1700, 0xffffffff, 0xc00f0038);
  _device.phy_set_bb_reg(0x06c0, 0xffffffff, 0xaaaaaaaa);
  _device.phy_set_bb_reg(0x06c4, 0xffffffff, 0xaaaaaaaa);

  /* switch_antenna: rfe_module_type = efuse RFE & 0x1f decides the ext-ant
   * switch existence + polarity. */
  const uint8_t rfe_mod =
      static_cast<uint8_t>((_efuse_valid ? _efuse_map[0xCA] : 0) & 0x1f);
  const bool exist = !(rfe_mod == 5 || rfe_mod == 6);
  if (!exist) {
    _logger->info("Jaguar2/8821C: coex WL-only (rfe_mod={} no ext-ant switch)",
                  rfe_mod);
    return;
  }
  const bool wlg_at_btg = (rfe_mod == 2 || rfe_mod == 4 || rfe_mod == 7);
  const bool ant_at_main = !(rfe_mod == 3 || rfe_mod == 4);
  bool inv = false; /* ext_ant_switch_ctrl_polarity = 0 */
  if (!ant_at_main)
    inv = !inv;
  if (!is_5g && !wlg_at_btg) /* WLG: swap if WLG not at BTG */
    inv = !inv;

  /* BBSW control (no antenna-diversity): 0x4c[24:23]=2, 0xcb4[7:0]=0x77,
   * 0xcb4[29:28] = 1 (no inverse) / 2 (inverse). */
  _device.phy_set_bb_reg(0x004c, 0x01800000, 0x2);
  _device.phy_set_bb_reg(0x0cb4, 0x000000ff, 0x77);
  _device.phy_set_bb_reg(0x0cb4, 0x30000000, inv ? 0x2 : 0x1);
  _logger->info("Jaguar2/8821C: coex WL-only antenna (rfe_mod={} wlg@btg={} "
                "main={} inv={} is_5g={})",
                rfe_mod, wlg_at_btg, ant_at_main, inv, is_5g ? 1 : 0);
}

/* rtl8821c_phy_bf_init (reference/8821cu/hal/rtl8821c/rtl8821c_phy.c): MU-MIMO /
 * TXBF MAC setup the vendor runs every init. The 8822B bf_init devourer used
 * only wrote 0x1c94 (the grouping bitmap, the LAST write here) and missed the
 * MU/TXBF MAC registers — so the 8821C gets its own faithful bf_init. */
void HalJaguar2::bf_init_8821c() {
  /* REG_MU_TX_CTL (0x14C0): P1 wait-state EN, MU RL=0xA, disable MU-MIMO until
   * sounding done, clear MU-STA table valid. */
  uint32_t v32 = _device.rtw_read32(0x14C0);
  v32 |= (1u << 16);                          /* BIT_R_MU_P1_WAIT_STATE_EN */
  v32 = (v32 & ~(0xfu << 12)) | (0xAu << 12); /* MU RL = 0xA */
  v32 &= ~(1u << 7);                          /* disable MU-MIMO */
  v32 &= ~(0x3fu << 0);                       /* MU table valid = 0 */
  _device.rtw_write32(0x14C0, v32);

  /* REG_MU_BF_OPTION (0x167C): TXMU ACKPOLICY=3 + EN = 0x70. */
  _device.rtw_write8(0x167C, static_cast<uint8_t>((3u << 4) | (1u << 6)));
  /* REG_WMAC_MU_BF_CTL (0x1680) = 0. */
  _device.rtw_write16(0x1680, 0x0);

  /* REG_TXBF_CTRL+3 (0x042F): use NDPA parameter from 0x45F (BIT30>>24=0x40). */
  _device.rtw_write8(0x042F, static_cast<uint8_t>(
                                 _device.rtw_read8(0x042F) | 0x40));
  /* REG_NDPA_OPT_CTRL (0x045F) = 0x10 (NDPA rate OFDM_6M, BW20). */
  _device.rtw_write8(0x045F, 0x10);

  /* STA2 CSI rate fixed at 6M (0x6DF[5:0]=0x4, keep [7:6]). */
  _device.rtw_write8(0x06DF, static_cast<uint8_t>(
                                 (_device.rtw_read8(0x06DF) & 0xC0) | 0x4));
  /* Grouping bitmap (0x1C94) — same value as the 8822B bf_init. */
  _device.rtw_write32(0x1C94, 0xAFFFAFFFu);

  /* rfe_type-2 (1212 module) 5G-RX hardware fix from rtl8821c_hal_init_misc:
   * REG_PAD_CTRL1+3 (0x0067) = 0x36. Only for rfe_type 2 (efuse RFE & 0x1f). */
  if (((_efuse_valid ? _efuse_map[0xCA] : 0) & 0x1f) == 2)
    _device.rtw_write8(0x0067, 0x36);

  _logger->info("Jaguar2/8821C: bf_init (MU/TXBF MAC setup + 0x1c94)");
}

void HalJaguar2::rfe_init() {
  /* phydm_rfe_8822b_init (verbatim): chip-top mux + RFE s0/s1 source select.
   * The kernel runs this from odm_dm_init->phydm_rfe_init after BB/RF config. */
  _device.phy_set_bb_reg(0x0064, (1u << 29) | (1u << 28), 0x3); /* chip top mux */
  _device.phy_set_bb_reg(0x004c, (1u << 26) | (1u << 25), 0x0);
  _device.phy_set_bb_reg(0x0040, (1u << 2), 0x1);
  _device.phy_set_bb_reg(0x1990, 0x3f, 0x30);                   /* from s0/s1 */
  _device.phy_set_bb_reg(0x1990, (1u << 11) | (1u << 10), 0x3);
  _device.phy_set_bb_reg(0x0974, 0x3f, 0x3f);                   /* in / out */
  _device.phy_set_bb_reg(0x0974, (1u << 11) | (1u << 10), 0x3);
  _logger->info("Jaguar2: RFE init (chip-top mux, 0x1990=0xc30)");
}

void HalJaguar2::bf_init() {
  /* rtl8822b_phy_bf_init (verbatim): MU-MIMO / TXBF default setup. Register
   * offsets/bits from halmac_reg_8822b.h + rtl8822b_phy.c. */
  uint32_t v32 = _device.rtw_read32(0x14c0);        /* REG_MU_TX_CTL */
  v32 |= (1u << 16);                                /* MU P1 wait-state en */
  v32 = (v32 & ~(0xFu << 12)) | (0xAu << 12);       /* MU retry limit = 0xA */
  v32 &= ~(1u << 7);                                /* disable MU-MIMO */
  v32 &= ~0x3fu;                                    /* clear MU table valid */
  _device.rtw_write32(0x14c0, v32);

  _device.rtw_write8(0x167c, (3u << 4) | (1u << 6)); /* MU BF ACKPOLICY=3, EN */
  _device.rtw_write16(0x1680, 0);                    /* MU BF ctl default */

  uint8_t v8 = _device.rtw_read8(0x042f);            /* REG_TXBF_CTRL+3 */
  v8 |= (1u << 6);                                   /* USE_NDPA_PARAMETER>>24 */
  _device.rtw_write8(0x042f, v8);
  _device.rtw_write8(0x045f, 0x10);                  /* NDPA rate OFDM_6M/BW20 */

  v8 = _device.rtw_read8(0x06df);
  v8 = (v8 & 0xC0) | 0x4;                            /* STA2 CSI rate = 6M */
  _device.rtw_write8(0x06df, v8);
  _device.rtw_write32(0x1c94, 0xAFFFAFFFu);          /* grouping bitmap */
  _logger->info("Jaguar2: BF init (0x1c94=0xafffafff)");
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

void HalJaguar2::apply_tx_power(uint8_t channel, uint8_t bw, uint8_t rfe_type) {
  /* Port of phy_get_pg_txpwr_idx (hal_com_phycfg.c) + the 4-byte TXAGC write
   * (config_phydm_write_txagc_8822b): TXAGC[rate] = pg-base(rate-section) +
   * per-Nss diff, clamped to the txpwr_lmt regulatory limit, written to 0x1d00
   * (path A) / 0x1d80 (path B). CONFIG_TXPWR_BY_RATE_EN is off upstream, so the
   * rate-section base is the value. Replaces the hot BB-table default that
   * overdrives high-order QAM into PA compression. Handles 2.4G and 5G (20 MHz);
   * BW40/80 is future work (uses BW40/BW80 diffs). */
  if (channel == 0)
    return;
  /* Both variants use the same EFUSE power-by-rate layout at pg_txpwr_saddr=0x10
   * (rtl8821c_halinit.c:61 == the 8822B value). The 8821C is 1T1R, so its path-1
   * EFUSE base reads 0xff and is skipped by the per-path loop below — the shared
   * code reads the real path-A per-channel base, adds the section BW/rate diffs,
   * and clamps to the per-chip txpwr_lmt (8821C table wired into `lmt` above).
   * This replaces the earlier 8821C-only branch that ignored the EFUSE base. */
  const bool g5 = channel > 14;

  /* EFUSE power-by-rate from logical 0x10 (pg_txpwr_saddr). Per path: 2.4G block
   * 18 B then 5G block 24 B (42 B/path). Path p: 2G @ 0x10+p*42, 5G @ 0x22+p*42.
   * Reuse the map read once by read_efuse_rfe earlier in bring_up. */
  if (!_efuse_valid) {
    read_efuse_logical_map(_efuse_map, sizeof(_efuse_map), /*dump=*/false);
    _efuse_valid = true;
  }
  const uint8_t *map = _efuse_map;

  /* channel -> rate group (rtw_get_ch_group). */
  auto group_2g = [](uint8_t ch) -> uint8_t {
    if (ch <= 2) return 0; if (ch <= 5) return 1; if (ch <= 8) return 2;
    if (ch <= 11) return 3; return 4;
  };
  auto group_5g = [](uint8_t ch) -> uint8_t {
    if (ch <= 42) return 0; if (ch <= 48) return 1; if (ch <= 58) return 2;
    if (ch <= 80) return 3; if (ch <= 106) return 4; if (ch <= 114) return 5;
    if (ch <= 122) return 6; if (ch <= 130) return 7; if (ch <= 138) return 8;
    if (ch <= 144) return 9; if (ch <= 155) return 10; if (ch <= 161) return 11;
    if (ch <= 171) return 12; return 13;
  };
  const uint8_t group = g5 ? group_5g(channel) : group_2g(channel);
  const uint8_t cck_group = (channel == 14) ? 5 : group;

  auto s4 = [](uint8_t nib) -> int { /* s4bit -> s8 sign-extend */
    int v = nib & 0x0f; return (v & 0x8) ? (v - 16) : v;
  };
  /* Regulatory limits — exact per (rfe_type, band, bw, section, ntx, channel)
   * from the generated txpwr_lmt table (worldwide-min across FCC/ETSI/MKK; the
   * rfe_type-3 table for the T3U). Keeps high-order QAM out of PA compression.
   * The demo path is 1Tx; the ht2 (2SS) clamp reuses the 1Tx limit (conservative
   * for the rare 2SS injection). sec: 0=CCK 1=OFDM 2=HT. */
  const uint8_t band_i = g5 ? 1 : 0;
  /* Per-chip regulatory limit lookup (worldwide-min across FCC/ETSI/MKK, in
   * TXAGC-index units). 8822B uses the rfe_type-aware table; 8821C the packed
   * txpwr_lmt_t_8821c[] (rfe-independent). sec: 0=CCK 1=OFDM 2=HT. */
  auto lmt = [&](uint8_t sec) -> int {
#if defined(DEVOURER_HAVE_JAGUAR2_8822B)
    if (_variant == ChipVariant::C8822B)
      return hal8822b_txpwr_lmt(rfe_type, band_i, bw, sec, 1, channel);
#endif
#if defined(DEVOURER_HAVE_JAGUAR2_8821C)
    if (_variant == ChipVariant::C8821C)
      return hal8821c_txpwr_lmt(band_i, bw, sec, 1, channel);
#endif
    (void)sec;
    return 63;
  };
  const int lmt_cck = g5 ? 63 : lmt(0);
  const int lmt_ofdm = lmt(1);
  const int lmt_ht = lmt(2);
  auto clamp63 = [](int v) -> uint8_t {
    return static_cast<uint8_t>(v < 0 ? 0 : (v > 63 ? 63 : v));
  };
  auto clamp_lmt = [&](int v, int lmt) -> uint8_t {
    if (v > lmt) v = lmt; return clamp63(v);
  };

  /* 8821C is 1T1R — only path A has a valid EFUSE base + a real RF path (its
   * path-1 block bytes are not tx-power data and 0x1d80 has no path-B PA). */
  const uint8_t npath = _variant == ChipVariant::C8821C ? 1 : 2;
  for (uint8_t path = 0; path < npath; path++) {
    /* base offset of this path's band block, and the BW40-base / first-diff
     * byte offsets within it. 2.4G: 6 CCK-base + 5 BW40-base + diffs (diff@11).
     * 5G: 14 BW40-base + diffs (diff@14), no CCK. */
    const uint16_t blk = static_cast<uint16_t>(0x10 + path * 42 + (g5 ? 18 : 0));
    const uint8_t cck_base = g5 ? 0 : map[blk + cck_group];
    const uint8_t bw40_base = g5 ? map[blk + group] : map[blk + 6 + group];
    if (bw40_base == 0xFF || (!g5 && cck_base == 0xFF)) {
      _logger->info("Jaguar2: apply_tx_power path {} ch{} EFUSE base unprogrammed "
                    "— leaving TXAGC default", path, channel);
      continue;
    }
    const uint8_t d0 = map[blk + (g5 ? 14 : 11)]; /* MSB BW20_Diff0 / LSB OFDM_Diff0 */
    const uint8_t d1 = map[blk + (g5 ? 15 : 12)]; /* LSB BW20_Diff1 (2nd Tx) */
    const int ofdm_diff0 = s4(d0 & 0x0f);
    const int bw20_diff0 = s4((d0 >> 4) & 0x0f);
    const int bw20_diff1 = s4(d1 & 0x0f);

    /* HT bandwidth diff (phy_get_pg_txpwr_idx): BW20 = BW40_base + BW20_Diff;
     * BW40 = BW40_base + BW40_Diff (BW40_Diff[1Tx] is the reference = 0, i.e. the
     * base is the BW40 value); BW80 (5G) = BW40_base + BW80_Diff. Only BW20 adds
     * a diff for the 1Tx path — so BW40/80 use the raw base. (BW80's exact 5G
     * BW80_Diff is a further refinement; the base is a close conservative value.) */
    const int ht_diff0 = (bw == 0) ? bw20_diff0 : 0;
    const int ht_diff1 = (bw == 0) ? bw20_diff1 : 0;
    const uint8_t cck_idx = clamp_lmt(cck_base, lmt_cck);
    const uint8_t ofdm_idx = clamp_lmt(bw40_base + ofdm_diff0, lmt_ofdm);
    const uint8_t ht1_idx = clamp_lmt(bw40_base + ht_diff0, lmt_ht);
    const uint8_t ht2_idx = clamp_lmt(bw40_base + ht_diff0 + ht_diff1, lmt_ht);

    /* Write TXAGC (4 rates / dword). hw_rate: 0x00-03 CCK, 0x04-0b OFDM,
     * 0x0c-13 HT MCS0-7 (1SS), 0x14-1b HT MCS8-15 (2SS). 5G has no CCK. */
    const uint16_t p = static_cast<uint16_t>(0x1d00 + path * 0x80);
    auto wr = [&](uint16_t off, uint8_t idx) {
      _device.rtw_write32(static_cast<uint16_t>(p + off), idx * 0x01010101u);
    };
    if (!g5)
      wr(0x00, cck_idx);
    wr(0x04, ofdm_idx); wr(0x08, ofdm_idx);
    wr(0x0c, ht1_idx);  wr(0x10, ht1_idx);
    wr(0x14, ht2_idx);  wr(0x18, ht2_idx);
    /* VHT1SS — the 8821C's AC mode — uses the same 1SS BW base as HT1SS, and the
     * vendor efuse-calibrates it (config_phydm_write_txagc_8821c writes hw_rate
     * up to VHT; 0x2c-0x35 = VHT1SS MCS0-9 -> regs 0x1d2c/0x1d30/0x1d34). Without
     * this VHT TXAGC stays at the BB-table default — uncalibrated and
     * inconsistent with the CCK/OFDM/HT rates above. 8821C-only so the 8822B's
     * VHT2SS handling stays byte-identical. */
    if (_variant == ChipVariant::C8821C) {
      wr(0x2c, ht1_idx); wr(0x30, ht1_idx); wr(0x34, ht1_idx);
    }

    _logger->info("Jaguar2: TXAGC path {} ch{} {} (g{}): CCK={:#x} OFDM={:#x} "
                  "HT1SS={:#x} HT2SS={:#x} (bw40_base={})", path, channel,
                  g5 ? "5G" : "2G", group, cck_idx, ofdm_idx, ht1_idx, ht2_idx,
                  bw40_base);
  }
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
  /* 8821C: drop APP_PHYST_RXFF (BIT28). The 8821C appends a 32-byte PHY-status
   * block before each RX frame in the RXFF, but its RX descriptor reports
   * drv_info_size=0 (unlike the 8822B, which counts it) — so the shared parser
   * puts the frame pointer 32 bytes early (onto the phy-status) and mis-advances
   * the aggregation, garbling every frame. Monitor RX doesn't need the phy-
   * status, so clearing the append bit makes the frame sit cleanly at
   * descriptor+drvinfo(0)+shift and decode. */
  if (_variant == ChipVariant::C8821C) {
    if (!getenv("DEVOURER_8821C_NO_PHYST")) {
      /* cfg_drv_info_8821c(HALMAC_DRV_INFO_PHY_STATUS): keep RCR APP_PHYSTS
       * (bit28, like the vendor monitor path) and make the chip prepend a
       * 32-byte PHY-status (jgr2 type0/type1) as drvinfo AND — crucially — count
       * it in the RX descriptor's drv_info_size. REG_TRXFF_BNDY+1 low-nibble =
       * 0xF is the vendor's "rxdesc len = 0 issue" fix, without which the
       * descriptor reports drv_info_size=0 despite PHYST=1 and the shared parser
       * mis-locates the body (garbling every frame). With it, drv_info_size
       * reads 32, the body sits cleanly at desc+32+shift, and the phy-status
       * carries per-frame RSSI/SNR/EVM (FrameParserJaguar2 fills RxAtrib).
       * DEVOURER_8821C_NO_PHYST drops the phy-status for the leanest RX. */
      _device.rtw_write8(0x060F, 4); /* REG_RX_DRVINFO_SZ = 4 units = 32 B */
      uint8_t v = _device.rtw_read8(0x0115);
      _device.rtw_write8(0x0115, static_cast<uint8_t>((v & 0xF0) | 0x0F));
    } else {
      rcr &= ~(1u << 28);
    }
  }
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
