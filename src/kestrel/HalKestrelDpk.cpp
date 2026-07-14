/* HalKestrelDpk.cpp — verbatim port of halrf_dpk_8852c.c (RTL8852C digital
 * pre-distortion, TX linearity) for the C8852C variant.
 *
 * Thin wrapper driving the on-chip NCTL cal-engine (loaded by
 * config_nctl_reg_8852c). Host-driven: each step writes BB/RF, fires a one-shot
 * at BB 0x8000, polls (halrf_do_one_shot_8852c: pre-enable, then 0xbff8 byte0
 * ==0x55 and 0x80fc[15:0]==0x8000, both up to 2000 counts), reads results.
 *
 * Additive + self-disabling: on any cal fail the AGC/main path force-disables
 * MDPD (dpk_onoff), so DPK never degrades TX below the un-DPK'd path.
 *
 * Register-plane mapping is identical to the IQK/DACK ports:
 *   halrf_wreg==bb_rmw, halrf_rreg==bb_read, halrf_wrf==rf_wrf,
 *   halrf_rrf==rf_rrf, halrf_delay_us==delay_us, get_thermal==read_thermal.
 * io_ofld is FALSE, so write_fwofld_start/end are no-ops (elided) and the
 * one-shot takes the register-poll default branch.
 *
 * Omitted (unreachable on USB single-PHY 5G, matching the IQK wrapper): 6G,
 * DBCC/MCC, MP-mode, HALRF_DZ_LOG, fw-offload, rfe_type==5/21/22/>50, the
 * DPK_RELOAD path (DPK_RELOAD_EN=0), and the RXSRAM/coef/PAS-dump debug reads.
 */
#include "HalKestrel.h"

#include <chrono>
#include <thread>

namespace kestrel {

namespace {
void delay_us(uint32_t us) {
  std::this_thread::sleep_for(std::chrono::microseconds(us));
}

constexpr uint32_t MASKDWORD = 0xffffffffu;
constexpr uint32_t MASKRF = 0xfffffu;
constexpr uint32_t MASKRFMODE = 0xf0000u; /* RF reg 0x10000 mode field */
constexpr uint32_t MASKRFRXBB = 0x003e0u; /* RF reg 0x00 [9:5] rxbb */
constexpr uint32_t NCTL_FINAL_LINE = 0x8000u;
constexpr uint8_t DAC_960M = 7, ADC_1920M = 3;
constexpr uint8_t BAND_ON_24G = 0, BAND_ON_5G = 1, BAND_ON_6G = 2;
constexpr uint8_t CW40 = 1, CW80 = 2;
constexpr uint8_t RF_PATH_A = 0, RF_PATH_B = 1;
constexpr uint8_t RF_DPK = 0x5, RF_RX = 0x3;
constexpr uint8_t CUT_CAV = 0; /* CAV cut id */

/* k-set report/coef register table [kidx][cur_k_set]. */
const uint32_t kDpkKSetReg[2][4] = {{0x8190, 0x8194, 0x8198, 0x81a4},
                                    {0x81a8, 0x81c4, 0x81c8, 0x81e8}};

/* DPK backup register lists (_dpk_cal_select_8852c). */
const uint32_t kDpkKipReg[] = {0x813c, 0x8124, 0x8120, 0xc0d4, 0xc0d8, 0xc0c4,
                               0xc0ec};
const uint32_t kDpkBbReg[] = {0x566c, 0x766c};
const uint32_t kDpkRfReg[] = {0xdf, 0x5f, 0x8f, 0x97, 0xa3, 0x5, 0x10005};
} // namespace

/* ===== NCTL one-shot (halrf_do_one_shot_8852c) ===== */

void HalKestrel::dpk_do_one_shot_8852c(uint8_t /*path*/, uint32_t cmd) {
  /* halrf_before_one_shot_enable_8852c */
  if (bb_read(0xbff8, 0xff) != 0x0) {
    bb_rmw(0x8010, 0x000000ff, 0x00);
    delay_us(1);
  }
  bb_rmw(0x8000, MASKDWORD, cmd);
  /* _halrf_one_shot_nctl_done_check_default_8852c */
  uint16_t count = 1;
  while (count < 2000) {
    if (bb_read(0xbff8, 0xff) == 0x55)
      break;
    delay_us(10);
    count++;
  }
  delay_us(1);
  count = 1;
  while (count < 2000) {
    if (bb_read(0x80fc, 0xffff) == NCTL_FINAL_LINE)
      break;
    delay_us(1);
    count++;
  }
  delay_us(1);
  bb_rmw(0x8010, 0x000000ff, 0x00);
}

void HalKestrel::dpk_one_shot_8852c(uint8_t path, uint8_t id) {
  const uint16_t cmd =
      static_cast<uint16_t>((id << 8) | (0x19 + path * 0x12));
  dpk_do_one_shot_8852c(path, cmd);
}

/* ===== small helpers ===== */

void HalKestrel::dpk_rxagc_onoff_8852c(uint8_t path, bool on) {
  if (path == RF_PATH_A)
    bb_rmw(0x4730, 1u << 31, on);
  else
    bb_rmw(0x4a9c, 1u << 31, on);
}

void HalKestrel::dpk_kip_control_rfc_8852c(uint8_t path, bool by_kip) {
  bb_rmw(0x5670 + (static_cast<uint32_t>(path) << 13), 1u << 1, by_kip);
}

void HalKestrel::dpk_txpwr_bb_force_8852c(uint8_t path, bool force) {
  const uint32_t o = static_cast<uint32_t>(path) << 13;
  bb_rmw(0x56cc + o, 1u << 28, force);
  bb_rmw(0x580c + o, 1u << 15, force);
}

void HalKestrel::dpk_kip_pwr_clk_onoff_8852c(bool on) {
  if (on) {
    bb_rmw(0x8008, MASKDWORD, 0x00000080);
    bb_rmw(0x8088, MASKDWORD, 0x807f030a);
  } else {
    bb_rmw(0x8008, MASKDWORD, 0x00000000);
    bb_rmw(0x8088, MASKDWORD, 0x80000000);
    bb_rmw(0x80f4, 1u << 18, 0x1);
  }
}

void HalKestrel::dpk_bb_afe_setting_8852c(uint8_t path) {
  const uint32_t o13 = static_cast<uint32_t>(path) << 13;
  bb_rmw(0x20fc, 1u << (16 + path), 0x1);
  bb_rmw(0x20fc, 1u << (20 + path), 0x0);
  bb_rmw(0x20fc, 1u << (24 + path), 0x1);
  bb_rmw(0x20fc, 1u << (28 + path), 0x0);
  bb_rmw(0x5670 + o13, MASKDWORD, 0xd801dffd);
  txck_force_8852c(path, true, DAC_960M);
  rxck_force_8852c(path, true, ADC_1920M);
  bb_rmw(0x12b8 + o13, 1u << 30, 0x1);
  bb_rmw(0x030c, 0xff000000, 0x1f);
  bb_rmw(0x030c, 0xff000000, 0x13);
  bb_rmw(0x032c, 0xffff0000, 0x0001);
  bb_rmw(0x032c, 0xffff0000, 0x0041);
  bb_rmw(0x20fc, 1u << (20 + path), 0x1);
  bb_rmw(0x20fc, 1u << (28 + path), 0x1);
  bb_rmw(0x5424 + o13, 1u << 3, 0x1);
  bb_rmw(0x56c4 + o13, 1u << 4, 0x1);
}

void HalKestrel::dpk_bb_afe_restore_8852c(uint8_t path) {
  const uint32_t o13 = static_cast<uint32_t>(path) << 13;
  bb_rmw(0x12b8 + o13, 1u << 30, 0x0);
  bb_rmw(0x20fc, 1u << (16 + path), 0x1);
  bb_rmw(0x20fc, 1u << (20 + path), 0x0);
  bb_rmw(0x20fc, 1u << (24 + path), 0x1);
  bb_rmw(0x20fc, 1u << (28 + path), 0x0);
  bb_rmw(0x5670 + o13, MASKDWORD, 0x00000000);
  bb_rmw(0x12a0 + o13, 0x000FF000, 0x00);
  bb_rmw(0x20fc, 1u << (16 + path), 0x0);
  bb_rmw(0x20fc, 1u << (24 + path), 0x0);
  bb_rmw(0x5424 + o13, 1u << 3, 0x0);
  bb_rmw(0x56c4 + o13, 1u << 4, 0x0);
}

void HalKestrel::dpk_tpg_sel_8852c(uint8_t path, uint8_t kidx) {
  const uint8_t bw = _dpk.bp[path][kidx].bw;
  if (bw == CW80) {
    bb_rmw(0x806c, (1u << 2) | (1u << 1), 0x0);
    bb_rmw(0x8068, MASKDWORD, 0xffe0fa00);
  } else if (bw == CW40) {
    bb_rmw(0x806c, (1u << 2) | (1u << 1), 0x2);
    bb_rmw(0x8068, MASKDWORD, 0xff4009e0);
  } else { /* 20 (and folded 5/10) */
    bb_rmw(0x806c, (1u << 2) | (1u << 1), 0x1);
    bb_rmw(0x8068, MASKDWORD, 0xf9f007d0);
  }
}

void HalKestrel::dpk_kip_preset_8852c(uint8_t path, uint8_t kidx) {
  const uint32_t o8 = static_cast<uint32_t>(path) << 8;
  if (_dpk.bp[path][kidx].band == BAND_ON_24G)
    bb_rmw(0x8078, 0x000FFFFF, 0x50121);
  else
    bb_rmw(0x8078, 0x000FFFFF, 0x50101);
  if (read_cut() == CUT_CAV)
    bb_rmw(0x81bc + o8 + (kidx << 2), 0x00003F00, 0x01);
  else
    bb_rmw(0x81bc + o8 + (kidx << 2), 0x00003F00, 0x0c);
  dpk_kip_control_rfc_8852c(path, true);
  bb_rmw(0x8104 + o8, 1u << 8, kidx);
  dpk_one_shot_8852c(path, D_KIP_PRESET);
}

void HalKestrel::dpk_kip_restore_8852c(uint8_t path) {
  dpk_one_shot_8852c(path, D_KIP_RESTORE);
  dpk_kip_control_rfc_8852c(path, false);
  dpk_txpwr_bb_force_8852c(path, false);
}

void HalKestrel::dpk_rf_setting_8852c(uint8_t path, uint8_t kidx) {
  if (_dpk.bp[path][kidx].band == BAND_ON_24G) {
    rf_wrf(path, 0x00, MASKRF, 0x50121);
    rf_wrf(path, 0x10000, MASKRFMODE, RF_DPK);
    rf_wrf(path, 0x83, 0x00007, 0x2);
    rf_wrf(path, 0x83, 0x000F0, 0x4);
    rf_wrf(path, 0xdf, 1u << 12, 0x1);
    rf_wrf(path, 0x9e, 1u << 8, 0x1);
  } else { /* 5G/6G */
    rf_wrf(path, 0x00, MASKRF, 0x50101);
    rf_wrf(path, 0x10000, MASKRFMODE, RF_DPK);
    if (_dpk.bp[path][kidx].band == BAND_ON_6G && _dpk.bp[path][kidx].ch >= 161)
      rf_wrf(path, 0x97, 0x00F00, 0x8);
    rf_wrf(path, 0xa3, 0xF0000, 0xd);
    rf_wrf(path, 0x5f, 0x0000F, 0x8);
    rf_wrf(path, 0x8c, 0x0000F, 0x0);
    rf_wrf(path, 0x98, 0x0F000, 0x3);
    rf_wrf(path, 0xdf, 1u << 12, 0x1);
    rf_wrf(path, 0x9e, 1u << 8, 0x1);
    /* CHANNEL_WIDTH_160 rxbb-extend omitted (bw folds to <=80). */
  }
}

/* ===== txagc / kset / para ===== */

uint8_t HalKestrel::dpk_kip_set_txagc_8852c(uint8_t path, uint8_t dbm,
                                            bool from_bb) {
  if (from_bb) {
    if (dbm >= _dpk.max_dpk_txagc[path])
      dbm = _dpk.max_dpk_txagc[path];
    if (dbm >= 24)
      dbm = 24;
    else if (dbm <= 7)
      dbm = 7;
    bb_rmw(0x56cc + (static_cast<uint32_t>(path) << 13), 0x0FF80000, dbm << 2);
  }
  dpk_one_shot_8852c(path, D_TXAGC);
  dpk_kset_query_8852c(path);
  return dbm;
}

void HalKestrel::dpk_kset_query_8852c(uint8_t path) {
  const uint32_t o8 = static_cast<uint32_t>(path) << 8;
  bb_rmw(0x81d4 + o8, 0x003F0000, 0x10);
  const uint32_t v = bb_read(0x81fc + o8, 0xE0000000); /* [31:29] */
  /* vendor: cur_k_set = v - 1. Clamp the index to [0,3] defensively so an
   * unexpected 0 readback can't index kDpkKSetReg/BIT() out of range. */
  _dpk.cur_k_set = static_cast<uint8_t>((v ? v - 1 : 0) & 0x3);
}

void HalKestrel::dpk_para_query_8852c(uint8_t path, uint8_t kidx) {
  const uint32_t para =
      bb_read(kDpkKSetReg[kidx][_dpk.cur_k_set] + (static_cast<uint32_t>(path) << 8),
              MASKDWORD);
  _dpk.bp[path][kidx].txagc_dpk = (para >> 10) & 0x3f;
  _dpk.bp[path][kidx].ther_dpk = (para >> 26) & 0x3f;
}

/* ===== sync / dgain / gainloss / pas ===== */

bool HalKestrel::dpk_sync_check_8852c(uint8_t path, uint8_t /*kidx*/) {
  bb_rmw(0x80d4, 0x003F0000, 0x0);
  const uint32_t corr = bb_read(0x80fc, MASKDWORD);
  const uint8_t corr_val = (corr & 0xff00) >> 8;
  bb_rmw(0x80d4, 0x003F0000, 0x9);
  const uint32_t dc = bb_read(0x80fc, MASKDWORD);
  uint16_t dc_i = (dc & 0x0fff0000) >> 16;
  uint16_t dc_q = dc & 0x00000fff;
  if (dc_i >> 11 == 1)
    dc_i = 0x1000 - dc_i;
  if (dc_q >> 11 == 1)
    dc_q = 0x1000 - dc_q;
  bb_rmw(0x80d4, 0x003F0000, 0x8);
  (void)bb_read(0x80fc, 0x0000003F); /* rxbb (debug) */
  bb_rmw(0x80d4, 0x003F0000, 0x31);
  (void)bb_read(0x80fc, 1u << 8); /* rxbb_ov (debug) */
  return (dc_i > 200) || (dc_q > 200) || (corr_val < 170);
}

uint16_t HalKestrel::dpk_dgain_read_8852c() {
  bb_rmw(0x80d4, 0x003F0000, 0x0);
  return static_cast<uint16_t>(bb_read(0x80fc, 0x0FFF0000));
}

uint8_t HalKestrel::dpk_gainloss_read_8852c() {
  bb_rmw(0x80d4, 0x003F0000, 0x6);
  bb_rmw(0x80bc, 1u << 14, 0x1);
  return static_cast<uint8_t>(bb_read(0x80fc, 0x000000F0));
}

uint8_t HalKestrel::dpk_gainloss_8852c(uint8_t path, uint8_t /*kidx*/) {
  dpk_one_shot_8852c(path, D_GAIN_LOSS);
  dpk_kip_set_txagc_8852c(path, 0xff, false);
  const uint32_t o8 = static_cast<uint32_t>(path) << 8;
  bb_rmw(0x81f0 + o8, 0x0003FFFF, 0xf078);
  bb_rmw(0x81f0 + o8, 0xF0000000, 0x0);
  return dpk_gainloss_read_8852c();
}

uint8_t HalKestrel::dpk_pas_read_8852c(uint8_t /*path*/) {
  /* is_check=true path only (the is_check=false 32-entry dump is debug). */
  bb_rmw(0x80d4, 0xff0000, 0x06);
  bb_rmw(0x80bc, 1u << 14, 0x0);
  bb_rmw(0x80c0, 0xff0000, 0x08);
  bb_rmw(0x80c0, 0xff000000, 0x00);
  uint32_t tmp = bb_read(0x80fc, MASKDWORD);
  uint32_t val1_i = (tmp & 0xffff0000) >> 16;
  if (val1_i >= 0x800)
    val1_i = 0x1000 - val1_i;
  uint32_t val1_q = tmp & 0xffff;
  if (val1_q >= 0x800)
    val1_q = 0x1000 - val1_q;
  bb_rmw(0x80c0, 0xff000000, 0x1f);
  tmp = bb_read(0x80fc, MASKDWORD);
  uint32_t val2_i = (tmp & 0xffff0000) >> 16;
  if (val2_i >= 0x800)
    val2_i = 0x1000 - val2_i;
  uint32_t val2_q = tmp & 0xffff;
  if (val2_q >= 0x800)
    val2_q = 0x1000 - val2_q;
  const uint32_t m1 = val1_i * val1_i + val1_q * val1_q;
  const uint32_t m2 = val2_i * val2_i + val2_q * val2_q;
  if (m1 < m2)
    return 2;
  else if (m1 >= (m2 * 8 / 5))
    return 1;
  else if (m1 < (m2 * 8 / 5))
    return 3;
  return 0;
}

/* ===== rxagc / lbk-rxiqk ===== */

void HalKestrel::dpk_kip_set_rxagc_8852c(uint8_t path) {
  dpk_kip_control_rfc_8852c(path, false);
  bb_rmw(0x8078, 0x000FFFFF, rf_rrf(path, 0x00, MASKRF));
  dpk_kip_control_rfc_8852c(path, true);
  dpk_one_shot_8852c(path, D_RXAGC);
}

void HalKestrel::dpk_bypass_rxiqc_8852c(uint8_t path) {
  const uint32_t o8 = static_cast<uint32_t>(path) << 8;
  bb_rmw(0x81a0 + o8, 1u << 7, 0x1);
  bb_rmw(0x813c + o8, MASKDWORD, 0x40000002);
}

void HalKestrel::dpk_lbk_rxiqk_8852c(uint8_t path) {
  const uint32_t o8 = static_cast<uint32_t>(path) << 8;
  bb_rmw(0x81a0 + o8, 1u << 7, 0x1);
  bb_rmw(0x8074, 1u << 31, 0x1);
  dpk_kip_control_rfc_8852c(path, false);
  const uint8_t cur_rxbb = static_cast<uint8_t>(rf_rrf(path, 0x00, MASKRFRXBB));
  const uint32_t rf_11 = rf_rrf(path, 0x11, MASKRF);
  const uint32_t reg_81cc = bb_read(0x81cc + o8, (1u << 13) | (1u << 12));
  rf_wrf(path, 0x11, (1u << 1) | (1u << 0), 0x0);
  rf_wrf(path, 0x11, (1u << 6) | (1u << 5) | (1u << 4), 0x3);
  rf_wrf(path, 0x11, 0x1F000, 0xd);
  rf_wrf(path, 0x00, MASKRFRXBB, 0x1f);
  bb_rmw(0x81cc + o8, 0x0000003F, 0x12);
  bb_rmw(0x81cc + o8, (1u << 13) | (1u << 12), 0x3);
  dpk_kip_control_rfc_8852c(path, true);
  bb_rmw(0x802c, MASKDWORD, 0x00250025);
  dpk_one_shot_8852c(path, D_LBK_RXIQK);
  /* restore */
  dpk_kip_control_rfc_8852c(path, false);
  rf_wrf(path, 0x11, MASKRF, rf_11);
  rf_wrf(path, 0x00, MASKRFRXBB, cur_rxbb);
  bb_rmw(0x81cc + o8, (1u << 13) | (1u << 12), reg_81cc);
  bb_rmw(0x8074, 1u << 31, 0x0);
  bb_rmw(0x80d0, (1u << 21) | (1u << 20), 0x0);
  bb_rmw(0x81dc + o8, 1u << 1, 0x1);
  dpk_kip_control_rfc_8852c(path, true);
}

/* ===== AGC state machine ===== */

bool HalKestrel::dpk_agc_8852c(uint8_t path, uint8_t kidx, uint8_t init_xdbm) {
  uint8_t i = 0, tmp_dbm = init_xdbm, tmp_gl_idx = 0, tmp_rxbb = 0;
  uint8_t goout = 0, agc_cnt = 0, gl_cnt = 0;
  uint16_t dgain = 0;
  bool is_fail = false;
  uint8_t pas_idx;
  _dpk.is_limited_txagc[path] = false;

  do {
    switch (i) {
    case 0: /* SYNC and Dgain */
      dpk_kip_set_rxagc_8852c(path);
      if (agc_cnt == 0) {
        if (_dpk.bp[path][kidx].band == BAND_ON_24G)
          dpk_bypass_rxiqc_8852c(path);
        else
          dpk_lbk_rxiqk_8852c(path);
      }
      is_fail = dpk_sync_check_8852c(path, kidx);
      if (is_fail) {
        goout = 1;
        break;
      }
      dgain = dpk_dgain_read_8852c();
      if (dgain > 0x5fc || dgain < 0x556) {
        dpk_one_shot_8852c(path, D_SYNC);
        dpk_dgain_read_8852c();
      }
      i = 1;
      break;
    case 1: /* GAIN_LOSS and idx */
      tmp_gl_idx = dpk_gainloss_8852c(path, kidx);
      pas_idx = dpk_pas_read_8852c(path);
      if ((pas_idx == 2 || pas_idx == 3) && tmp_gl_idx > 0)
        i = 3;
      else if ((tmp_gl_idx == 0 && pas_idx == 1) || tmp_gl_idx >= 7)
        i = 2;
      else if (tmp_gl_idx == 0)
        i = 3;
      else
        i = 4;
      gl_cnt++;
      break;
    case 2: /* GL > criterion */
      if (tmp_dbm <= 7) {
        dpk_kip_set_txagc_8852c(path, 7, true);
        _dpk.is_limited_txagc[path] = true;
        _dpk.limited_txagc[path] = 0x2e;
        goout = 1;
      } else if (tmp_dbm >= _dpk.max_dpk_txagc[path]) {
        dpk_kip_set_txagc_8852c(path, _dpk.max_dpk_txagc[path], true);
        _dpk.is_limited_txagc[path] = true;
        _dpk.limited_txagc[path] = _dpk.max_dpk_txagc[path] - 7 + 0x2e;
        goout = 1;
      } else {
        tmp_dbm = (tmp_dbm - 3 <= 7) ? 7 : tmp_dbm - 3;
        dpk_kip_set_txagc_8852c(path, tmp_dbm, true);
      }
      i = 0;
      agc_cnt++;
      break;
    case 3: /* GL < criterion */
      if (tmp_dbm >= 24) {
        dpk_kip_set_txagc_8852c(path, 24, true);
        _dpk.is_limited_txagc[path] = true;
        _dpk.limited_txagc[path] = 0x3F;
        goout = 1;
      } else if (tmp_dbm >= _dpk.max_dpk_txagc[path]) {
        dpk_kip_set_txagc_8852c(path, _dpk.max_dpk_txagc[path], true);
        _dpk.is_limited_txagc[path] = true;
        _dpk.limited_txagc[path] = _dpk.max_dpk_txagc[path] - 7 + 0x2e;
        goout = 1;
      } else {
        tmp_dbm = (tmp_dbm + 2 >= 24) ? 24 : tmp_dbm + 2;
        dpk_kip_set_txagc_8852c(path, tmp_dbm, true);
      }
      i = 0;
      agc_cnt++;
      break;
    case 4:
      /* rfe_type==5 branch omitted (default RFE). */
      dpk_kip_control_rfc_8852c(path, false);
      tmp_rxbb = static_cast<uint8_t>(rf_rrf(path, 0x00, MASKRFRXBB));
      tmp_rxbb = (tmp_rxbb + tmp_gl_idx > 0x1f) ? 0x1f : tmp_rxbb + tmp_gl_idx;
      rf_wrf(path, 0x00, MASKRFRXBB, tmp_rxbb);
      dpk_kip_control_rfc_8852c(path, true);
      goout = 1;
      break;
    default:
      goout = 1;
      break;
    }
  } while (!goout && (agc_cnt < 6));
  (void)gl_cnt;
  return is_fail;
}

/* ===== mdpd / idl / normalize / on ===== */

void HalKestrel::dpk_set_mdpd_para_8852c(uint8_t path, uint8_t order) {
  switch (order) {
  case 0: /* (5,3,1) */
    bb_rmw(0x80a0, (1u << 1) | (1u << 0), 0x0);
    bb_rmw(0x809c, (1u << 10) | (1u << 9), 0x2);
    bb_rmw(0x80a0, 0x00001F00, 0x4);
    bb_rmw(0x8070, 0x70000000, 0x1);
    _dpk.dpk_order[path] = 0x6;
    break;
  case 1: /* (5,3,0) */
    bb_rmw(0x80a0, (1u << 1) | (1u << 0), 0x1);
    bb_rmw(0x809c, (1u << 10) | (1u << 9), 0x1);
    bb_rmw(0x80a0, 0x00001F00, 0x0);
    bb_rmw(0x8070, 0x70000000, 0x0);
    _dpk.dpk_order[path] = 0x2;
    break;
  case 2: /* (5,0,0) */
    bb_rmw(0x80a0, (1u << 1) | (1u << 0), 0x2);
    bb_rmw(0x809c, (1u << 10) | (1u << 9), 0x0);
    bb_rmw(0x80a0, 0x00001F00, 0x0);
    bb_rmw(0x8070, 0x70000000, 0x0);
    _dpk.dpk_order[path] = 0x0;
    break;
  case 3: /* (7,3,1) */
    bb_rmw(0x80a0, (1u << 1) | (1u << 0), 0x3);
    bb_rmw(0x809c, (1u << 10) | (1u << 9), 0x3);
    bb_rmw(0x80a0, 0x00001F00, 0x4);
    bb_rmw(0x8070, 0x70000000, 0x1);
    _dpk.dpk_order[path] = 0x1;
    break;
  default:
    break;
  }
}

void HalKestrel::dpk_idl_mpa_8852c(uint8_t path, uint8_t kidx) {
  uint8_t cnt = 0;
  bb_rmw(0x80a0, 1u << 16, 0x1);
  const uint8_t bw = _dpk.bp[path][kidx].bw;
  if (bw <= 0) /* 5/10/20 fold to bw==0 -> (5,0,0) */
    dpk_set_mdpd_para_8852c(path, 0x2);
  else if (bw == CW40 || bw == CW80)
    dpk_set_mdpd_para_8852c(path, 0x1);
  else
    dpk_set_mdpd_para_8852c(path, 0x0);
  bb_rmw(0x809c, 1u << 8, 0x0);
  for (uint16_t c = 0; c < 1000; c++)
    delay_us(1);
  dpk_one_shot_8852c(path, D_MDPK_IDL);
  bb_rmw(0x80d4, 0x003F0000, 0x0);
  bb_rmw(0x80d4, 0x003F0000, 0xf);
  _dpk.ov_flag[path] = static_cast<uint8_t>(bb_read(0x80fc, 1u << 30));
  while (_dpk.ov_flag[path] == 0x1 && cnt < 5) {
    dpk_one_shot_8852c(path, D_MDPK_IDL);
    bb_rmw(0x80d4, 0x003F0000, 0xf);
    _dpk.ov_flag[path] = static_cast<uint8_t>(bb_read(0x80fc, 1u << 30));
    cnt++;
  }
  if (_dpk.ov_flag[path]) {
    dpk_set_mdpd_para_8852c(path, 0x2);
    dpk_one_shot_8852c(path, D_MDPK_IDL);
  }
}

void HalKestrel::dpk_gain_normalize_8852c(uint8_t path, uint8_t kidx) {
  /* is_execute=false path (the only one _dpk_on uses). */
  const uint32_t reg =
      kDpkKSetReg[kidx][_dpk.cur_k_set] + (static_cast<uint32_t>(path) << 8);
  bb_rmw(reg, 0x0000007F, 0x5b);
  _dpk.bp[path][kidx].gs = static_cast<uint8_t>(bb_read(reg, 0x0000007F));
}

void HalKestrel::dpk_on_8852c(uint8_t path, uint8_t kidx) {
  const uint32_t o8 = static_cast<uint32_t>(path) << 8;
  const uint32_t bc = 0x81bc + o8 + (kidx << 2);
  bb_rmw(0x81dc + o8, 1u << 16, 0x1);
  bb_rmw(0x81dc + o8, 1u << 16, 0x0);
  bb_rmw(bc, (1u << 26) | (1u << 25) | (1u << 24), _dpk.dpk_order[path]);
  _dpk.bp[path][kidx].path_ok |= static_cast<uint8_t>(1u << _dpk.cur_k_set);
  bb_rmw(bc, 0xf0000000, _dpk.bp[path][kidx].path_ok);
  dpk_gain_normalize_8852c(path, kidx);
  if (_dpk.is_limited_txagc[path]) {
    _dpk.bp[path][kidx].txagc_dpk = _dpk.limited_txagc[path];
    bb_rmw(kDpkKSetReg[kidx][_dpk.cur_k_set] + o8, 0x0000FC00,
           _dpk.bp[path][kidx].txagc_dpk);
  }
}

void HalKestrel::dpk_onoff_8852c(uint8_t path, bool off) {
  const uint8_t kidx = _dpk.cur_idx[path];
  const bool off_reverse = !off;
  const uint8_t val = static_cast<uint8_t>(_dpk.is_dpk_enable * off_reverse *
                                           _dpk.bp[path][kidx].path_ok);
  bb_rmw(0x81bc + (static_cast<uint32_t>(path) << 8) + (kidx << 2), 0xf0000000,
         val);
}

/* ===== main (per path) + cal_select ===== */

bool HalKestrel::dpk_main_8852c(uint8_t path) {
  const uint8_t kidx = _dpk.cur_idx[path];
  uint8_t init_xdbm = 14;
  bool is_fail = false;
  if (_dpk.bp[path][kidx].band == BAND_ON_5G && _dpk.bp[path][kidx].ch < 100)
    init_xdbm = 18;
  else if (_dpk.bp[path][kidx].band != BAND_ON_24G)
    init_xdbm = 20;

  dpk_kip_control_rfc_8852c(path, false);
  rf_wrf(path, 0x5, 1u << 0, 0);     /* halrf_rf_direct_cntrl(false) */
  rf_wrf(path, 0x10005, MASKRF, 0x03ffd);
  dpk_rf_setting_8852c(path, kidx);
  rx_dck_toggle_8852c(path);
  dpk_kip_pwr_clk_onoff_8852c(true);
  dpk_kip_preset_8852c(path, kidx);
  dpk_txpwr_bb_force_8852c(path, true);
  init_xdbm = dpk_kip_set_txagc_8852c(path, init_xdbm, true);
  dpk_tpg_sel_8852c(path, kidx);
  is_fail = dpk_agc_8852c(path, kidx, init_xdbm);
  if (!is_fail) {
    dpk_idl_mpa_8852c(path, kidx);
    dpk_para_query_8852c(path, kidx);
    dpk_on_8852c(path, kidx);
  }
  /* _error: */
  dpk_kip_control_rfc_8852c(path, false);
  rf_wrf(path, 0x00, MASKRFMODE, RF_RX);
  return is_fail;
}

void HalKestrel::dpk_cal_select_8852c(uint8_t kpath) {
  uint32_t kip_bkup[2][7] = {{0}};
  uint32_t bb_bkup[2] = {0};
  uint32_t rf_bkup[2][7] = {{0}};

  for (uint8_t path = 0; path < 2; path++)
    _dpk.cur_idx[path] = 0; /* DPK_RELOAD_EN=0 */

  for (uint8_t path = 0; path < 2; path++) {
    if (!(kpath & (1u << path)))
      continue;
    const uint32_t o8 = static_cast<uint32_t>(path) << 8;
    for (int i = 0; i < 7; i++)
      kip_bkup[path][i] = bb_read(kDpkKipReg[i] + o8, MASKDWORD);
    for (int i = 0; i < 2; i++)
      bb_bkup[i] = bb_read(kDpkBbReg[i], MASKDWORD);
    for (int i = 0; i < 7; i++)
      rf_bkup[path][i] = rf_rrf(path, kDpkRfReg[i], MASKRF);
    /* information + init */
    _dpk.bp[path][0].path_ok = 0;
    _dpk.ov_flag[path] = 0;
    /* is_tssi_mode false (TSSI not enabled) -> no tssi_pause */

    dpk_rxagc_onoff_8852c(path, false);
    rf_wrf(path, 0x10005, 1u << 0, 0); /* halrf_drf_direct_cntrl(false) */
    dpk_bb_afe_setting_8852c(path);
    const bool is_fail = dpk_main_8852c(path);
    dpk_onoff_8852c(path, is_fail);

    dpk_kip_restore_8852c(path);
    for (int i = 0; i < 7; i++)
      bb_rmw(kDpkKipReg[i] + o8, MASKDWORD, kip_bkup[path][i]);
    for (int i = 0; i < 2; i++)
      bb_rmw(kDpkBbReg[i], MASKDWORD, bb_bkup[i]);
    for (int i = 0; i < 7; i++)
      rf_wrf(path, kDpkRfReg[i], MASKRF, rf_bkup[path][i]);
    dpk_bb_afe_restore_8852c(path);
    dpk_rxagc_onoff_8852c(path, true);
  }
  dpk_kip_pwr_clk_onoff_8852c(false);
}

/* halrf_dpk_8852c entry. band/bw/ch from the current tune. */
void HalKestrel::dpk_8852c(uint8_t band, uint8_t bw, uint8_t center_ch) {
  for (uint8_t path = 0; path < 2; path++) {
    _dpk.bp[path][0].band = band;
    _dpk.bp[path][0].bw = bw;
    _dpk.bp[path][0].ch = center_ch;
    _dpk.max_dpk_txagc[path] = 24; /* pwr_lmt_en==false short-circuit */
  }
  /* _dpk_bypass_check: skip DPK on a CAV die at 5 GHz (C8852C is CBV, so this
   * is false and DPK proceeds). FEM epa_* treated as 0 (internal-PA ref). */
  if (read_cut() == CUT_CAV && band != BAND_ON_24G) {
    _logger->info("Kestrel RFK(8852C): DPK bypassed (CAV cut, 5 GHz)");
    dpk_onoff_8852c(RF_PATH_A, true);
    dpk_onoff_8852c(RF_PATH_B, true);
    return;
  }
  _logger->info("Kestrel RFK(8852C): DPK band={} bw={} ch={} (RF_AB)", band, bw,
                center_ch);
  dpk_cal_select_8852c(0x3 /* RF_AB */);
}

} // namespace kestrel
