#include "Halrf8822b.h"

#include <chrono>
#include <thread>
#include <utility>

namespace jaguar2 {

namespace {
constexpr uint32_t MASK20 = 0x000FFFFF; /* MASK20BITS / RFREGOFFSETMASK */
constexpr uint8_t TXIQK = 0, RXIQK1 = 1, RXIQK2 = 2; /* one-shot idx / retry idx */
constexpr uint8_t rxiqk_gs_limit = 6;
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

/* --- nctl LTE-coex gnt access (_iqk_ltec_read/write_8822b) --- */
uint32_t Halrf8822b::ltec_read(uint16_t reg) {
  w32(0x1700, 0x800f0000u | reg);
  uint32_t j = 0;
  while (((_device.rtw_read8(0x1703) & (1u << 5)) == 0) && (j < 30000))
    j++;
  return r32(0x1708);
}
void Halrf8822b::ltec_write(uint16_t reg, uint32_t mask, uint32_t val) {
  if (mask == 0)
    return;
  uint32_t data = val;
  if (mask != 0xffffffffu) {
    uint32_t bitpos = bshift(mask);
    uint32_t cur = ltec_read(reg);
    data = (cur & ~mask) | (val << bitpos);
  }
  w32(0x1704, data);
  uint32_t j = 0;
  while (((_device.rtw_read8(0x1703) & (1u << 5)) == 0) && (j < 30000))
    j++;
  w32(0x1700, 0xc00f0000u | reg);
}

/* _iqk_check_cal_8822b: poll RF0x8==0x12345; LOK ok always, else fail=0x1b08[26] */
bool Halrf8822b::check_cal(uint8_t path, uint8_t cmd) {
  bool notready = true, fail = true;
  uint32_t delay_count = 0;
  while (notready) {
    if (rf_get(path, 0x8) == 0x12345) {
      fail = (cmd == 0x0) ? false : (bb_get(0x1b08, 1u << 26) != 0);
      notready = false;
    } else {
      udelay(10);
      delay_count++;
    }
    if (delay_count >= 20000) {
      fail = true;
      break;
    }
  }
  set_rf0x8(path);
  return fail;
}

/* _iqk_rxk_gsearch_fail_8822b: RXK1/RXK2 gain search (IQMUX/lna_idx machine) */
bool Halrf8822b::rxk_gsearch(uint8_t path, uint8_t step) {
  bool fail = true;
  uint32_t IQK_CMD;
  const uint8_t IQMUX[4] = {0x9, 0x12, 0x1b, 0x24};

  if (step == RXIQK1) {
    IQK_CMD = 0xf8000208u | (1u << (path + 4));
    ltec_write(0x38, 0xffff, 0x7700);
    w32(0x1b00, IQK_CMD);
    w32(0x1b00, IQK_CMD + 0x1);
    udelay(10);
    fail = check_cal(path, 0x1);
    ltec_write(0x38, 0xffffffff, _tmp_gntwl);
  } else { /* RXIQK2 */
    uint8_t idx;
    for (idx = 0; idx < 4; idx++)
      if (_tmp1bcc == IQMUX[idx])
        break;
    if (idx == 4)
      return fail;
    w32(0x1b00, 0xf8000008u | (path << 1));
    w32(0x1bcc, _tmp1bcc);
    IQK_CMD = 0xf8000308u | (1u << (path + 4));
    ltec_write(0x38, 0xffff, 0x7700);
    w32(0x1b00, IQK_CMD);
    w32(0x1b00, IQK_CMD + 0x1);
    udelay(10);
    fail = check_cal(path, 0x1);
    ltec_write(0x38, 0xffffffff, _tmp_gntwl);

    uint32_t rf_reg0 = rf_get(path, 0x0);
    w32(0x1b00, 0xf8000008u | (path << 1));
    uint32_t tmp = (rf_reg0 & 0x1fe0) >> 5;
    _lna_idx = static_cast<uint8_t>(tmp >> 5);
    uint32_t bb_idx = tmp & 0x1f;
    if (bb_idx == 0x1) {
      if (_lna_idx != 0x0)
        _lna_idx--;
      else if (idx != 3)
        idx++;
      else
        _isbnd = true;
      fail = true;
    } else if (bb_idx == 0xa) {
      if (idx != 0)
        idx--;
      else if (_lna_idx != 0x7)
        _lna_idx++;
      else
        _isbnd = true;
      fail = true;
    } else {
      fail = false;
    }
    if (_isbnd)
      fail = false;
    _tmp1bcc = IQMUX[idx];
    if (fail) {
      w32(0x1b00, 0xf8000008u | (path << 1));
      tmp = (r32(0x1b24) & 0xffffe3ffu) | (_lna_idx << 10);
      w32(0x1b24, tmp);
    }
  }
  return fail;
}

/* _lok_one_shot_8822b (backup of lok_idac skipped — no reload) */
bool Halrf8822b::lok_one_shot(uint8_t path) {
  uint32_t IQK_CMD = 0xf8000008u | (1u << (4 + path));
  ltec_write(0x38, 0xffff, 0x7700);
  w32(0x1b00, IQK_CMD);
  w32(0x1b00, IQK_CMD + 1);
  udelay(10);
  bool notready = check_cal(path, 0x0);
  ltec_write(0x38, 0xffffffff, _tmp_gntwl);
  _lok_fail[path] = notready;
  return notready;
}

/* _iqk_one_shot_8822b: TXIQK / RXIQK1 / RXIQK2 (CFIR backup skipped) */
bool Halrf8822b::one_shot(uint8_t path, uint8_t idx) {
  uint32_t IQK_CMD = 0;
  const uint16_t iqk_apply[2] = {0xc94, 0xe94};
  if (idx == TXIQK) {
    IQK_CMD = 0xf8000008u | ((_bw + 4) << 8) | (1u << (path + 4));
  } else if (idx == RXIQK1) {
    IQK_CMD = (_bw == 2) ? (0xf8000808u | (1u << (path + 4)))
                         : (0xf8000708u | (1u << (path + 4)));
  } else { /* RXIQK2 */
    IQK_CMD = 0xf8000008u | ((_bw + 9) << 8) | (1u << (path + 4));
    w32(0x1b00, 0xf8000008u | (path << 1));
    uint32_t tmp = (r32(0x1b24) & 0xffffe3ffu) | ((_lna_idx & 0x7) << 10);
    w32(0x1b24, tmp);
  }
  ltec_write(0x38, 0xffff, 0x7700);
  w32(0x1b00, IQK_CMD);
  w32(0x1b00, IQK_CMD + 0x1);
  udelay(10);
  bool fail = check_cal(path, 0x1);
  ltec_write(0x38, 0xffffffff, _tmp_gntwl);

  w32(0x1b00, 0xf8000008u | (path << 1));
  if (idx == TXIQK) {
    if (fail)
      bb_set(iqk_apply[path], 1u << 0, 0x0); /* disable apply on fail */
    /* success: HW keeps applied result (CFIR backup for reload skipped) */
  }
  if (idx == RXIQK2) {
    w32(0x1b38, 0x20000000);
    if (fail)
      bb_set(iqk_apply[path], (1u << 11) | (1u << 10), 0x0);
  }
  return fail;
}

/* _iqk_rx_iqk_by_path_8822b: RXK step machine (xym debug reads skipped) */
bool Halrf8822b::rx_iqk_by_path(uint8_t path) {
  bool KFAIL = true, gonext;
  switch (_rxiqk_step) {
  case 1: /* gain search RXK1 */
    rxk1_setting(path);
    gonext = false;
    while (true) {
      KFAIL = rxk_gsearch(path, RXIQK1);
      if (KFAIL && _gs_retry[path][0] < 2) {
        _gs_retry[path][0]++;
      } else if (KFAIL) {
        _rxiqk_fail_code[path] = 0;
        _rxiqk_step = 5;
        gonext = true;
      } else {
        _rxiqk_step++;
        gonext = true;
      }
      if (gonext)
        break;
    }
    break;
  case 2: /* gain search RXK2 */
    rxk2_setting(path, true);
    _isbnd = false;
    while (true) {
      KFAIL = rxk_gsearch(path, RXIQK2);
      if (KFAIL && _gs_retry[path][1] < rxiqk_gs_limit)
        _gs_retry[path][1]++;
      else {
        _rxiqk_step++;
        break;
      }
    }
    break;
  case 3: /* RXK1 */
    rxk1_setting(path);
    gonext = false;
    while (true) {
      KFAIL = one_shot(path, RXIQK1);
      if (KFAIL && _retry[path][RXIQK1] < 2) {
        _retry[path][RXIQK1]++;
      } else if (KFAIL) {
        _rxiqk_fail_code[path] = 1;
        _rxiqk_step = 5;
        gonext = true;
      } else {
        _rxiqk_step++;
        gonext = true;
      }
      if (gonext)
        break;
    }
    break;
  case 4: /* RXK2 */
    rxk2_setting(path, false);
    gonext = false;
    while (true) {
      KFAIL = one_shot(path, RXIQK2);
      if (KFAIL && _retry[path][RXIQK2] < 2) {
        _retry[path][RXIQK2]++;
      } else if (KFAIL) {
        _rxiqk_fail_code[path] = 2;
        _rxiqk_step = 5;
        gonext = true;
      } else {
        _rxiqk_step++;
        gonext = true;
      }
      if (gonext)
        break;
    }
    break;
  }
  return KFAIL;
}

void Halrf8822b::iqk_by_path_subfunction(uint8_t path) {
  while (true) {
    rx_iqk_by_path(path);
    if (_rxiqk_step == 5) {
      _iqk_step++;
      _rxiqk_step = 1;
      break;
    }
  }
  _kcount++;
}

/* _iqk_iqk_by_path_8822b (non-segment: run to iqk_step==7) */
void Halrf8822b::iqk_by_path() {
  bool KFAIL;
  while (true) {
    switch (_iqk_step) {
    case 1: /* S0 LOK */
      lok_setting(0);
      lok_one_shot(0);
      _iqk_step++;
      break;
    case 2: /* S1 LOK */
      lok_setting(1);
      lok_one_shot(1);
      _iqk_step++;
      break;
    case 3: /* S0 TXIQK */
      txk_setting(0);
      KFAIL = one_shot(0, TXIQK);
      _kcount++;
      if (KFAIL && _retry[0][TXIQK] < 3)
        _retry[0][TXIQK]++;
      else
        _iqk_step++;
      break;
    case 4: /* S1 TXIQK */
      txk_setting(1);
      KFAIL = one_shot(1, TXIQK);
      _kcount++;
      if (KFAIL && _retry[1][TXIQK] < 3)
        _retry[1][TXIQK]++;
      else
        _iqk_step++;
      break;
    case 5: /* S0 RXIQK */
      iqk_by_path_subfunction(0);
      break;
    case 6: /* S1 RXIQK */
      iqk_by_path_subfunction(1);
      break;
    }
    if (_iqk_step == 7) {
      for (uint8_t i = 0; i < 2; i++) {
        w32(0x1b00, 0xf8000008u | (i << 1));
        w32(0x1b2c, 0x7);
        w32(0x1bcc, 0x0);
        w32(0x1b38, 0x20000000);
      }
      break;
    }
  }
}

void Halrf8822b::start_iqk() {
  /* GNT_WL = 1 (RF 0x1 |= BIT5|BIT0), both paths */
  uint32_t tmp = rf_get(0, 0x1) | (1u << 5) | (1u << 0);
  rf_set(0, 0x1, MASK20, tmp);
  tmp = rf_get(1, 0x1) | (1u << 5) | (1u << 0);
  rf_set(1, 0x1, MASK20, tmp);
  iqk_by_path();
}

void Halrf8822b::iqk_init() {
  /* cu_distance (IQK result variation) */
  w32(0x1b10, 0x88011c00);
}

/* _phy_iq_calibrate_8822b — fresh (non-segment, no reload) IQK. */
void Halrf8822b::iqk_trigger(bool band2g) {
  _band2g = band2g;
  _bw = 0; /* 20 MHz */

  uint32_t mac_bk[2], bb_bk[21], rf_bk[5][2];
  const uint32_t mac_reg[2] = {0x520, 0x550};
  const uint32_t bb_reg[21] = {0x808, 0x90c, 0xc00, 0xcb0, 0xcb4, 0xcbc, 0xe00,
                               0xeb0, 0xeb4, 0xebc, 0x1990, 0x9a4, 0xa04, 0xb00,
                               0x838, 0xc58, 0xc5c, 0xc6c, 0xe58, 0xe5c, 0xe6c};
  const uint32_t rf_reg[5] = {0xdf, 0x8f, 0x65, 0x0, 0x1};

  iqk_init();

  /* reset run state */
  _kcount = 0;
  _iqk_step = 1;
  _rxiqk_step = 1;
  for (int p = 0; p < 2; p++) {
    _lok_fail[p] = true;
    _rxiqk_fail_code[p] = 0;
    for (int k = 0; k < 3; k++)
      _retry[p][k] = 0;
    for (int k = 0; k < 2; k++)
      _gs_retry[p][k] = 0;
  }

  _tmp_gntwl = ltec_read(0x38);
  backup_mac_bb(mac_bk, bb_bk, mac_reg, bb_reg);
  backup_rf(rf_bk, rf_reg);

  while (true) {
    configure_macbb();
    afe_setting(true);
    rfe_setting(false);
    agc_bnd_int();
    rf_setting();
    start_iqk();
    afe_setting(false);
    restore_mac_bb(mac_bk, bb_bk, mac_reg, bb_reg);
    restore_rf(rf_reg, rf_bk);
    if (_iqk_step == 7)
      break;
    _kcount = 0;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  /* rf0xb0 workaround: RF_A 0xb8 phase-noise nudge */
  rf_set(0, 0xb8, MASK20, 0x00a00);
  rf_set(0, 0xb8, MASK20, 0x80a00);

  _logger->info("Jaguar2 IQK done: LOK A/B fail={}/{} | TXK retry A/B={}/{} | "
                "RXK gs_retry A(g1={} g2={}) B(g1={} g2={}) 1shot_retry "
                "A(k1={} k2={}) B(k1={} k2={}) | RXK failcode A/B={}/{} "
                "(0=gs1,1=rxk1,2=rxk2; only meaningful if that step ran)",
                _lok_fail[0], _lok_fail[1], _retry[0][TXIQK], _retry[1][TXIQK],
                _gs_retry[0][0], _gs_retry[0][1], _gs_retry[1][0],
                _gs_retry[1][1], _retry[0][RXIQK1], _retry[0][RXIQK2],
                _retry[1][RXIQK1], _retry[1][RXIQK2], _rxiqk_fail_code[0],
                _rxiqk_fail_code[1]);
}

} /* namespace jaguar2 */
