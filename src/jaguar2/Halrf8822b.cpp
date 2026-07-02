#include "Halrf8822b.h"

#include <chrono>
#include <thread>
#include <utility>

namespace jaguar2 {

namespace {
constexpr uint32_t MASK20 = 0x000FFFFF; /* MASK20BITS / RFREGOFFSETMASK */
void udelay(uint32_t us) {
  std::this_thread::sleep_for(std::chrono::microseconds(us));
}
uint32_t bshift(uint32_t mask) {
  if (mask == 0)
    return 0;
  uint32_t s = 0;
  while (!((mask >> s) & 1u))
    s++;
  return s;
}
} /* namespace */

Halrf8822b::Halrf8822b(RtlUsbAdapter device, Logger_t logger, uint8_t cut,
                       bool is_2t2r)
    : _device{std::move(device)}, _logger{std::move(logger)}, _cut{cut},
      _2t2r{is_2t2r} {}

/* --- BB access (odm_get/set_bb_reg) --- */
uint32_t Halrf8822b::bb_get(uint16_t addr, uint32_t mask) {
  return (_device.rtw_read32(addr) & mask) >> bshift(mask);
}
void Halrf8822b::bb_set(uint16_t addr, uint32_t mask, uint32_t data) {
  _device.phy_set_bb_reg(addr, mask, data);
}

/* --- RF access: read = direct BB window (config_phydm_read_rf_reg_8822b);
 * write = 3-wire LSSI (config_phydm_write_rf_reg_8822b, 0xC90/0xE90). --- */
uint32_t Halrf8822b::rf_get(uint8_t path, uint32_t addr) {
  const uint32_t base = (path == 0) ? 0x2800u : 0x2c00u;
  const uint16_t direct = static_cast<uint16_t>(base + ((addr & 0xff) << 2));
  return _device.rtw_read32(direct) & MASK20;
}
void Halrf8822b::rf_set(uint8_t path, uint32_t addr, uint32_t mask,
                        uint32_t data) {
  if (mask == 0)
    return;
  uint32_t v = data;
  if (mask != MASK20) {
    uint32_t orig = rf_get(path, addr);
    v = (orig & ~mask) | (data << bshift(mask));
  }
  const uint16_t lssi = (path == 0) ? 0x0C90 : 0x0E90;
  uint32_t data_and_addr = (((addr & 0xff) << 20) | (v & MASK20)) & 0x0fffffffu;
  _device.phy_set_bb_reg(lssi, 0xFFFFFFFFu, data_and_addr);
}
/* _iqk_rf_set_check_8822b: write then verify (RF write can be flaky). */
void Halrf8822b::rf_set_check(uint8_t path, uint16_t addr, uint32_t data) {
  rf_set(path, addr, MASK20, data);
  for (int i = 0; i < 100; i++) {
    if (rf_get(path, addr) == data)
      break;
    udelay(10);
    rf_set(path, addr, MASK20, data);
  }
}

/* --- backup / restore (_iqk_backup_*, _iqk_restore_*) --- */
void Halrf8822b::backup_mac_bb(uint32_t *mac_bk, uint32_t *bb_bk,
                               const uint32_t *mac_reg,
                               const uint32_t *bb_reg) {
  for (int i = 0; i < 2; i++)
    mac_bk[i] = r32(static_cast<uint16_t>(mac_reg[i]));
  for (int i = 0; i < 21; i++)
    bb_bk[i] = r32(static_cast<uint16_t>(bb_reg[i]));
}
void Halrf8822b::backup_rf(uint32_t rf_bk[][2], const uint32_t *reg) {
  for (int i = 0; i < 5; i++) {
    rf_bk[i][0] = rf_get(0, reg[i]);
    rf_bk[i][1] = rf_get(1, reg[i]);
  }
}
void Halrf8822b::restore_mac_bb(const uint32_t *mac_bk, const uint32_t *bb_bk,
                                const uint32_t *mac_reg,
                                const uint32_t *bb_reg) {
  for (int i = 0; i < 2; i++)
    w32(static_cast<uint16_t>(mac_reg[i]), mac_bk[i]);
  for (int i = 0; i < 21; i++)
    w32(static_cast<uint16_t>(bb_reg[i]), bb_bk[i]);
}
void Halrf8822b::restore_rf(const uint32_t *reg, uint32_t rf_bk[][2]) {
  rf_set(0, 0xef, MASK20, 0x0);
  rf_set(1, 0xef, MASK20, 0x0);
  rf_set_check(0, 0xdf, rf_bk[0][0] & ~(1u << 4));
  rf_set_check(1, 0xdf, rf_bk[0][1] & ~(1u << 4));
  for (int i = 1; i < 5; i++) {
    rf_set(0, reg[i], MASK20, rf_bk[i][0]);
    rf_set(1, reg[i], MASK20, rf_bk[i][1]);
  }
}

void Halrf8822b::agc_bnd_int() {
  w32(0x1b00, 0xf8000008);
  w32(0x1b00, 0xf80a7008);
  w32(0x1b00, 0xf8015008);
  w32(0x1b00, 0xf8000008);
}

void Halrf8822b::bb_reset() {
  uint32_t count = 0;
  rf_set(0, 0x0, MASK20, 0x10000);
  rf_set(1, 0x0, MASK20, 0x10000);
  bb_set(0x8f8, 0x0ff00000, 0x0);
  while (true) {
    w32(0x8fc, 0x0);
    bb_set(0x198c, 0x7, 0x7);
    bool cca_ing = bb_get(0xfa0, 1u << 3) != 0;
    if (count > 20000)
      cca_ing = false;
    if (cca_ing) {
      udelay(10);
      count++;
    } else {
      w8(0x808, 0x0);
      bb_set(0xa04, (1u << 27) | (1u << 26) | (1u << 25) | (1u << 24), 0x0);
      bb_set(0x0, 1u << 16, 0x0);
      bb_set(0x0, 1u << 16, 0x1);
      if (bb_get(0x660, 1u << 16))
        w32(0x6b4, 0x89000006);
      break;
    }
  }
}

void Halrf8822b::afe_setting(bool do_iqk) {
  if (do_iqk) {
    w32(0xc60, 0x50000000);
    w32(0xc60, 0x70070040);
    w32(0xe60, 0x50000000);
    w32(0xe60, 0x70070040);
    w32(0xc58, 0xd8000402);
    w32(0xc5c, 0xd1000120);
    w32(0xc6c, 0x00000a15);
    w32(0xe58, 0xd8000402);
    w32(0xe5c, 0xd1000120);
    w32(0xe6c, 0x00000a15);
    bb_reset();
  } else {
    w32(0xc60, 0x50000000);
    w32(0xc60, 0x70038040);
    w32(0xe60, 0x50000000);
    w32(0xe60, 0x70038040);
  }
  bb_set(0x9a4, 1u << 31, 0x0);
}

void Halrf8822b::rfe_setting(bool ext_pa_on) {
  if (ext_pa_on) {
    w32(0xcb0, 0x77777777);
    w32(0xcb4, 0x00007777);
    w32(0xcbc, 0x0000083B);
    w32(0xeb0, 0x77777777);
    w32(0xeb4, 0x00007777);
    w32(0xebc, 0x0000083B);
  } else {
    w32(0xcb0, 0x77777777);
    w32(0xcb4, 0x00007777);
    w32(0xcbc, 0x00000100);
    w32(0xeb0, 0x77777777);
    w32(0xeb4, 0x00007777);
    w32(0xebc, 0x00000100);
  }
}

void Halrf8822b::rf_setting() {
  w32(0x1b00, 0xf8000008);
  w32(0x1bb8, 0x00000000);
  for (uint8_t path = 0; path < 2; path++) {
    uint32_t tmp = rf_get(path, 0xdf);
    tmp = (tmp & ~(1u << 4)) | (1u << 1) | (1u << 11);
    rf_set_check(path, 0xdf, tmp);
    rf_set(path, 0x65, MASK20, 0x09000);
    /* 2.4G branch (5G differs only in RF 0x3f = 0xdefce vs 0x5efce) */
    rf_set(path, 0xef, 1u << 19, 0x1);
    rf_set(path, 0x33, MASK20, 0x00026);
    rf_set(path, 0x3e, MASK20, 0x00037);
    rf_set(path, 0x3f, MASK20, _band2g ? 0x5efce : 0xdefce);
    rf_set(path, 0xef, 1u << 19, 0x0);
  }
}

void Halrf8822b::configure_macbb() {
  w8(0x522, 0x7f);
  bb_set(0x550, (1u << 11) | (1u << 3), 0x0);
  bb_set(0x90c, 1u << 15, 0x1);
  bb_set(0xc94, 1u << 0, 0x1);
  bb_set(0xe94, 1u << 0, 0x1);
  bb_set(0xc94, (1u << 11) | (1u << 10), 0x1);
  bb_set(0xe94, (1u << 11) | (1u << 10), 0x1);
  w32(0xc00, 0x00000004);
  w32(0xe00, 0x00000004);
  bb_set(0xb00, 1u << 8, 0x0);
  bb_set(0x808, 1u << 28, 0x0);
  bb_set(0x838, (1u << 3) | (1u << 2) | (1u << 1), 0x7);
}

void Halrf8822b::lok_setting(uint8_t path) {
  w32(0x1b00, 0xf8000008 | (path << 1));
  w32(0x1bcc, 0x9);
  w8(0x1b23, 0x00);
  if (_band2g) {
    w8(0x1b2b, 0x00);
    rf_set(path, 0x56, MASK20, 0x50df2);
    rf_set(path, 0x8f, MASK20, 0xadc00);
    rf_set(path, 0xef, 1u << 4, 0x1);
    rf_set(path, 0x33, (1u << 1) | (1u << 0), 0x0);
  } else {
    w8(0x1b2b, 0x80);
    rf_set(path, 0x56, MASK20, 0x5086c);
    rf_set(path, 0x8f, MASK20, 0xa9c00);
    rf_set(path, 0xef, 1u << 4, 0x1);
    rf_set(path, 0x33, (1u << 1) | (1u << 0), 0x1);
  }
}

void Halrf8822b::txk_setting(uint8_t path) {
  w32(0x1b00, 0xf8000008 | (path << 1));
  w32(0x1bcc, 0x9);
  w32(0x1b20, 0x01440008);
  if (path == 0x0)
    w32(0x1b00, 0xf800000a);
  else
    w32(0x1b00, 0xf8000008);
  w32(0x1bcc, 0x3f);
  if (_band2g) {
    rf_set(path, 0x56, MASK20, 0x50df2);
    rf_set(path, 0x8f, MASK20, 0xadc00);
    w8(0x1b2b, 0x00);
  } else {
    rf_set(path, 0x56, MASK20, 0x500ef);
    rf_set(path, 0x8f, MASK20, 0xa9c00);
    w8(0x1b2b, 0x80);
  }
}

void Halrf8822b::rxk1_setting(uint8_t path) {
  w32(0x1b00, 0xf8000008 | (path << 1));
  if (_band2g) {
    w8(0x1bcc, 0x9);
    w8(0x1b2b, 0x00);
    w32(0x1b20, 0x01450008);
    w32(0x1b24, 0x01460c88);
    rf_set(path, 0x56, MASK20, 0x510e0);
    rf_set(path, 0x8f, MASK20, 0xacc00);
  } else {
    w8(0x1bcc, 0x09);
    w8(0x1b2b, 0x80);
    w32(0x1b20, 0x00850008);
    w32(0x1b24, 0x00460048);
    rf_set(path, 0x56, MASK20, 0x510e0);
    rf_set(path, 0x8f, MASK20, 0xadc00);
  }
}

void Halrf8822b::rxk2_setting(uint8_t path, bool is_gs) {
  w32(0x1b00, 0xf8000008 | (path << 1));
  if (_band2g) {
    if (is_gs)
      _tmp1bcc = 0x12;
    w8(0x1bcc, _tmp1bcc);
    w8(0x1b2b, 0x00);
    w32(0x1b20, 0x01450008);
    w32(0x1b24, 0x01460848);
    rf_set(path, 0x56, MASK20, 0x510e0);
    rf_set(path, 0x8f, MASK20, 0xa9c00);
  } else {
    if (is_gs)
      _tmp1bcc = (path == 0) ? 0x12 : 0x09;
    w8(0x1bcc, _tmp1bcc);
    w8(0x1b2b, 0x80);
    w32(0x1b20, 0x00850008);
    w32(0x1b24, 0x00460848);
    rf_set(path, 0x56, MASK20, 0x51060);
    rf_set(path, 0x8f, MASK20, 0xa9c00);
  }
}

void Halrf8822b::set_rf0x8(uint8_t path) {
  uint16_t c = 0;
  while (c < 30000) {
    rf_set(path, 0xef, MASK20, 0x0);
    rf_set(path, 0x8, MASK20, 0x0);
    if (rf_get(path, 0x8) == 0x0)
      break;
    c++;
  }
}

void Halrf8822b::iqk_trigger(bool band2g) {
  _band2g = band2g;
  /* Orchestrator (LOK/TXK/RXK tone-measurement loops + backup/restore) is
   * added in the next milestone step; the foundation (settings/backup/restore)
   * is in place above. */
  _logger->info("Jaguar2 IQK: foundation ready (band2g={}) — measurement loops "
                "pending",
                band2g);
}

} /* namespace jaguar2 */
