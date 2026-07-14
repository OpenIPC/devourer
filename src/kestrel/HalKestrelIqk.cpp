/* HalKestrelIqk.cpp — verbatim port of halrf_iqk_8852c.c (RTL8852C I/Q
 * imbalance calibration: LOK + TXK + RXK) for the C8852C variant.
 *
 * Source: reference/rtl8852cu/phl/hal_g6/phy/rf/halrf_iqk_8852c.c (+ the generic
 * dispatch in halrf_iqk.c). IQK is HOST-DRIVEN via the on-chip NCTL RF micro-
 * sequencer: each step writes BB/RF registers, fires a one-shot by writing an
 * opcode to BB 0x8000, then polls two BB status words (0xbff8 byte0==0x55, then
 * 0x80fc[15:0]==0x8000) and reads the fail bit at 0x8008[26]. All result-
 * readback layers (xym/cfir/sram/fft) are disabled — matching iqk_init_8852c
 * (iqk_*_en=false) — so this is the direct write+poll path with no readback.
 *
 * io_ofld is false in devourer (every bb_rmw/rf_wrf hits hardware immediately),
 * so the vendor halrf_write_fwofld_start/end brackets are no-ops and elided,
 * and _iqk_check_cal takes the register-poll (#else) branch.
 *
 * Register-plane mapping (identical semantics to the vendor helpers):
 *   halrf_wreg(a,m,v) == bb_rmw(a,m,v)   halrf_rreg(a,m) == bb_read(a,m)
 *   halrf_wrf(p,a,m,v) == rf_wrf(p,a,m,v) halrf_rrf(p,a,m) == rf_rrf(p,a,m)
 *   halrf_delay_us == delay_us   halrf_get_thermal_8852c == read_thermal
 *
 * Path coverage: kpath_8852c returns RF_AB (non-DBCC), so IQK runs on RF_PATH_A
 * then RF_PATH_B, both transcribed (the PATH_B WA branches are included).
 */
#include "HalKestrel.h"

#include <chrono>
#include <thread>

#include "hal8852c_phy.h" /* array_mp_8852c_nctl_reg (extract_8852c_phy.py) */

namespace kestrel {

namespace {
void delay_us(uint32_t us) {
  std::this_thread::sleep_for(std::chrono::microseconds(us));
}
} // namespace

/* Constants shared with the vendor source. */
static constexpr uint32_t MASKDWORD = 0xffffffffu;
static constexpr uint32_t MASKRF = 0xfffffu;
/* halrf_api.h DAC/ADC clock enum. */
static constexpr uint8_t ADC_480M = 1;
static constexpr uint8_t ADC_960M = 2;
static constexpr uint8_t ADC_1920M = 3;
static constexpr uint8_t DAC_960M = 7;
/* Band / bandwidth encodings (as stored in _iqk from the current tune). */
static constexpr uint8_t BAND_ON_24G = 0;
static constexpr uint8_t BAND_ON_5G = 1;
static constexpr uint8_t BAND_ON_6G = 2;
static constexpr uint8_t CW20 = 0, CW40 = 1, CW80 = 2;
static constexpr uint8_t RF_PATH_A = 0, RF_PATH_B = 1;
/* iqk_version_8852c (halrf_iqk_8852c.h) — report-register cosmetic value. */
static constexpr uint32_t iqk_version_8852c = 0x25;

/* halrf_iqk.c backup register lists (8852C). MAC backup num is 0. */
static const uint32_t kBackupBbReg8852c[] = {0x2344, 0xc0c4, 0xc0d4, 0xc0d8,
                                             0xc0e8, 0xc0ec, 0xc1c4, 0xc1d4,
                                             0xc1d8, 0xc1e8, 0xc1ec};
static const uint32_t kBackupRfReg8852c[] = {0xdc, 0xef, 0xde,
                                             0x0,  0x1e, 0x5, 0x10005};

/* ================= poll / one-shot (the NCTL engine) ===================== */

/* _iqk_check_cal_8852c: poll 0xbff8 byte0==0x55 (<=8.2ms), then 0x80fc[15:0]
 * ==0x8000 (<=200us), then read fail = 0x8008[26]. */
bool HalKestrel::iqk_check_cal_8852c(uint8_t /*path*/, uint8_t /*ktype*/) {
  bool notready = true, fail = true;
  uint32_t delay_count = 0;
  while (notready) {
    if (bb_read(0xbff8, 0xff) == 0x55) {
      delay_us(10);
      notready = false;
    } else {
      delay_us(10);
      delay_count++;
    }
    if (delay_count > 820) {
      fail = true;
      break;
    }
  }
  notready = true;
  fail = true;
  delay_count = 0;
  while (notready) {
    if (bb_read(0x80fc, 0x0000ffff) == 0x8000) {
      delay_us(10);
      notready = false;
    } else {
      delay_us(10);
      delay_count++;
    }
    if (delay_count > 20) {
      fail = true;
      break;
    }
  }
  delay_us(10);
  if (!notready)
    fail = static_cast<bool>(bb_read(0x8008, 1u << 26));
  bb_rmw(0x8010, 0xff, 0x0);
  return fail;
}

/* _iqk_one_shot_8852c: set the RFC-ctl bit for the ktype, build the opcode,
 * fire (cmd+1) at 0x8000, poll, clear 0x808c. Readback branches are omitted
 * (iqk_*_en all false). */
bool HalKestrel::iqk_one_shot_8852c(uint8_t path, uint8_t ktype) {
  bool fail = false;
  uint32_t iqk_cmd = 0;
  const uint32_t addr_rfc_ctl = 0x5670 + (static_cast<uint32_t>(path) << 13);
  const uint32_t pbit = 1u << (4 + path);
  switch (ktype) {
  case ID_TXAGC:
    iqk_cmd = 0x008 | pbit | (static_cast<uint32_t>(path) << 1);
    break;
  case ID_A_FLoK_coarse:
    bb_rmw(addr_rfc_ctl, 0x00000002, 0x1);
    iqk_cmd = 0x008 | pbit;
    break;
  case ID_G_FLoK_coarse:
    bb_rmw(addr_rfc_ctl, 0x00000002, 0x1);
    iqk_cmd = 0x108 | pbit;
    break;
  case ID_A_FLoK_fine:
    bb_rmw(addr_rfc_ctl, 0x00000002, 0x1);
    iqk_cmd = 0x508 | pbit;
    break;
  case ID_G_FLoK_fine:
    bb_rmw(addr_rfc_ctl, 0x00000002, 0x1);
    iqk_cmd = 0x208 | pbit;
    break;
  case ID_FLOK_vbuffer:
    bb_rmw(addr_rfc_ctl, 0x00000002, 0x1);
    iqk_cmd = 0x308 | pbit;
    break;
  case ID_TXK:
    bb_rmw(addr_rfc_ctl, 0x00000002, 0x0);
    iqk_cmd = 0x008 | pbit | (((0x8 + _iqk.iqk_bw[path]) & 0xf) << 8);
    break;
  case ID_RXAGC:
    bb_rmw(addr_rfc_ctl, 0x00000002, 0x1);
    iqk_cmd = 0x1708 | pbit;
    break;
  case ID_RXK:
    bb_rmw(addr_rfc_ctl, 0x00000002, 0x1);
    iqk_cmd = 0x008 | pbit | (((0xc + _iqk.iqk_bw[path]) & 0xf) << 8);
    break;
  case ID_NBTXK:
    bb_rmw(addr_rfc_ctl, 0x00000002, 0x0);
    iqk_cmd = 0x408 | pbit;
    break;
  case ID_NBRXK:
    bb_rmw(addr_rfc_ctl, 0x00000002, 0x1);
    iqk_cmd = 0x608 | pbit;
    break;
  default:
    return false;
  }
  bb_rmw(0x8000, MASKDWORD, iqk_cmd + 1);
  delay_us(1);
  fail = iqk_check_cal_8852c(path, ktype);
  bb_rmw(0x808c, MASKDWORD, 0x0);
  /* 8. IQK control RFC */
  bb_rmw(addr_rfc_ctl, 0x00000002, 0x0);
  return fail;
}

void HalKestrel::iqk_disable_rxagc_8852c(uint8_t path, uint8_t en) {
  if (path == RF_PATH_A)
    bb_rmw(0x4730, 1u << 31, en);
  else
    bb_rmw(0x4a9c, 1u << 31, en);
}

bool HalKestrel::iqk_check_nbiqc_8852c(uint8_t ktype, uint8_t path) {
  uint32_t x, y;
  const uint32_t base = (ktype == ID_NBTXK ? 0x8138 : 0x813c) +
                        (static_cast<uint32_t>(path) << 8);
  x = bb_read(base, 0xfff00000);
  y = bb_read(base, 0x000fff00);
  /* HALRF_ABS(x, 1024) */
  x = (x > 1024) ? (x - 1024) : (1024 - x);
  if (y > 0x800)
    y = 0x1000 - y;
  return (x > 150) || (y > 150);
}

/* ================= LOK ==================================================== */

void HalKestrel::lok_res_table_8852c(uint8_t path, uint8_t ibias) {
  rf_wrf(path, 0xef, 0x00002, 0x1);
  if (_iqk.iqk_band[path] == BAND_ON_24G)
    rf_wrf(path, 0x30, 0x08000, 0x0);
  else
    rf_wrf(path, 0x30, 0x08000, 0x1);
  rf_wrf(path, 0x30, 0x00038, ibias);
  rf_wrf(path, 0xef, 0x00002, 0x0);
}

bool HalKestrel::lok_2g_finetune_check_8852c(uint8_t path) {
  bool is_fail1;
  const uint32_t temp = rf_rrf(path, 0x58, MASKRF);
  const uint32_t core_i = (temp & 0xf8000) >> 15;
  const uint32_t core_q = (temp & 0x07c00) >> 10;
  is_fail1 = (core_i < 0x3 || core_i > 0x15 || core_q < 0x3 || core_q > 0x15);
  return is_fail1;
}

bool HalKestrel::lok_5g_finetune_check_8852c(uint8_t path) {
  bool is_fail1;
  const uint32_t temp = rf_rrf(path, 0x5c, MASKRF);
  const uint32_t core_i = (temp & 0xfc000) >> 14;
  const uint32_t core_q = (temp & 0x03f00) >> 8;
  is_fail1 = (core_i < 0x3 || core_i > 0x35 || core_q < 0x3 || core_q > 0x35);
  return is_fail1;
}

bool HalKestrel::iqk_lok_2g_8852c(uint8_t path) {
  bool isfail;
  const uint32_t rfc = 0x5670 + (static_cast<uint32_t>(path) << 13);
  const uint32_t p4 = 1u << (4 + path);
  const uint32_t cc = 0x81cc + (static_cast<uint32_t>(path) << 8);
  rf_wrf(path, 0x11, 0x00003, 0x0);
  rf_wrf(path, 0x11, 0x00070, 0x6);
  rf_wrf(path, 0x11, 0x1f000, 0x6);
  bb_rmw(rfc, 0x00000002, 0x1);
  bb_rmw(cc, 0x0000003f, 0x09);
  bb_rmw(0x802c, 0x00000fff, 0x021);
  bb_rmw(0x8000, MASKDWORD, 0x00000109 | p4);
  isfail = iqk_check_cal_8852c(path, 0);
  bb_rmw(0x8010, 0x000000ff, 0x00);
  bb_rmw(rfc, 0x00000002, 0x0);

  rf_wrf(path, 0x11, 0x1f000, 0x12);
  bb_rmw(rfc, 0x00000002, 0x1);
  bb_rmw(cc, 0x0000003f, 0x1b);
  bb_rmw(0x8000, MASKDWORD, 0x00000309 | p4);
  isfail = iqk_check_cal_8852c(path, 0);
  bb_rmw(0x8010, 0x000000ff, 0x00);
  bb_rmw(rfc, 0x00000002, 0x0);

  rf_wrf(path, 0x11, 0x1f000, 0x6);
  bb_rmw(rfc, 0x00000002, 0x1);
  bb_rmw(cc, 0x0000003f, 0x09);
  bb_rmw(0x8000, MASKDWORD, 0x00000209 | p4);
  isfail = iqk_check_cal_8852c(path, 0);
  bb_rmw(0x8010, 0x000000ff, 0x00);
  bb_rmw(rfc, 0x00000002, 0x0);

  rf_wrf(path, 0x11, 0x1f000, 0x12);
  bb_rmw(rfc, 0x00000002, 0x1);
  bb_rmw(cc, 0x0000003f, 0x1b);
  bb_rmw(0x8000, MASKDWORD, 0x00000309 | p4);
  isfail = iqk_check_cal_8852c(path, 0);
  bb_rmw(0x8010, 0x000000ff, 0x00);
  bb_rmw(rfc, 0x00000002, 0x0);
  return isfail;
}

/* USB (or BW20) 5G LOK — the reachable branch on this USB target. */
bool HalKestrel::iqk_lok_5g_8852c(uint8_t path) {
  bool isfail;
  const uint32_t rfc = 0x5670 + (static_cast<uint32_t>(path) << 13);
  const uint32_t p4 = 1u << (4 + path);
  const uint32_t cc = 0x81cc + (static_cast<uint32_t>(path) << 8);
  rf_wrf(path, 0x00, 0xffff0, 0x403e);
  rf_wrf(path, 0x11, 0x00003, 0x0);
  rf_wrf(path, 0x11, 0x00070, 0x6);
  rf_wrf(path, 0x11, 0x1f000, 0x12);
  bb_rmw(rfc, 0x00000002, 0x1);
  bb_rmw(cc, 0x0000003f, 0x1b);
  bb_rmw(0x802c, 0x00000fff, 0x021);
  bb_rmw(0x8000, MASKDWORD, 0x00000709 | p4);
  isfail = iqk_check_cal_8852c(path, 0);
  bb_rmw(0x8010, 0x000000ff, 0x00);

  bb_rmw(rfc, 0x00000002, 0x0);
  rf_wrf(path, 0x11, 0x1f000, 0x06);
  bb_rmw(rfc, 0x00000002, 0x1);
  bb_rmw(cc, 0x0000003f, 0x09);
  bb_rmw(0x8000, MASKDWORD, 0x00000509 | p4);
  isfail = iqk_check_cal_8852c(path, 0);
  bb_rmw(0x8010, 0x000000ff, 0x00);

  bb_rmw(rfc, 0x00000002, 0x0);
  rf_wrf(path, 0x11, 0x1f000, 0x12);
  bb_rmw(rfc, 0x00000002, 0x1);
  bb_rmw(cc, 0x0000003f, 0x1b);
  bb_rmw(0x8000, MASKDWORD, 0x00000309 | p4);
  isfail = iqk_check_cal_8852c(path, 0);
  bb_rmw(0x8010, 0x000000ff, 0x00);

  bb_rmw(rfc, 0x00000002, 0x0);
  (void)isfail;
  return false; /* vendor returns false here */
}

bool HalKestrel::iqk_lok_6g_8852c(uint8_t path) {
  bool isfail;
  const uint32_t rfc = 0x5670 + (static_cast<uint32_t>(path) << 13);
  const uint32_t p4 = 1u << (4 + path);
  const uint32_t cc = 0x81cc + (static_cast<uint32_t>(path) << 8);
  rf_wrf(path, 0x11, 0x00003, 0x0);
  rf_wrf(path, 0x11, 0x00070, 0x6);
  rf_wrf(path, 0x11, 0x1f000, 0x12);
  bb_rmw(rfc, 0x00000002, 0x1);
  bb_rmw(cc, 0x0000003f, 0x1b);
  bb_rmw(0x802c, 0x00000fff, 0x021);
  bb_rmw(0x8000, MASKDWORD, 0x00000709 | p4);
  isfail = iqk_check_cal_8852c(path, 0);
  bb_rmw(0x8010, 0x000000ff, 0x00);

  bb_rmw(rfc, 0x00000002, 0x0);
  rf_wrf(path, 0x11, 0x1f000, 0x06);
  bb_rmw(rfc, 0x00000002, 0x1);
  bb_rmw(cc, 0x0000003f, 0x09);
  bb_rmw(0x8000, MASKDWORD, 0x00000509 | p4);
  isfail = iqk_check_cal_8852c(path, 0);
  bb_rmw(0x8010, 0x000000ff, 0x00);

  bb_rmw(rfc, 0x00000002, 0x0);
  rf_wrf(path, 0x11, 0x1f000, 0x12);
  bb_rmw(rfc, 0x00000002, 0x1);
  bb_rmw(cc, 0x0000003f, 0x1b);
  bb_rmw(0x8000, MASKDWORD, 0x00000309 | p4);
  isfail = iqk_check_cal_8852c(path, 0);
  bb_rmw(0x8010, 0x000000ff, 0x00);

  bb_rmw(rfc, 0x00000002, 0x0);
  return isfail;
}

bool HalKestrel::iqk_lok_8852c(uint8_t path) {
  bool isfail1 = false, isfail2 = false;
  switch (_iqk.iqk_band[path]) {
  case BAND_ON_24G:
    isfail1 = iqk_lok_2g_8852c(path);
    isfail2 = lok_2g_finetune_check_8852c(path);
    break;
  case BAND_ON_5G:
    /* USB is always true on this target, so the 5g (non-bygain) path. */
    isfail1 = iqk_lok_5g_8852c(path);
    isfail2 = lok_5g_finetune_check_8852c(path);
    break;
  case BAND_ON_6G:
    isfail1 = iqk_lok_6g_8852c(path);
    isfail2 = lok_5g_finetune_check_8852c(path);
    break;
  default:
    break;
  }
  rf_wrf(path, 0xef, 0x00004, 0x0);
  return isfail1 | isfail2;
}

void HalKestrel::iqk_txk_setting_8852c(uint8_t path) {
  switch (_iqk.iqk_band[path]) {
  case BAND_ON_24G:
    rf_wrf(path, 0x51, 0x80000, 0x0);
    rf_wrf(path, 0x51, 0x00800, 0x0);
    rf_wrf(path, 0x52, 0x00800, 0x1);
    rf_wrf(path, 0x65, 0xf0000, 0xf);
    rf_wrf(path, 0x55, 0x0001e, 0x0);
    rf_wrf(path, 0xef, 0x00004, 0x1);
    rf_wrf(path, 0x00, 0xfffff, 0x403e0 | _iqk.syn1to2);
    delay_us(10);
    rf_wrf(path, 0x11, 0x00003, 0x0);
    rf_wrf(path, 0x11, 0x00070, 0x6);
    break;
  case BAND_ON_5G:
    rf_wrf(path, 0x64, 0x38000, 0x0);
    rf_wrf(path, 0x7f, 0x00080, 0x1);
    rf_wrf(path, 0x65, 0xf0000, 0xf);
    rf_wrf(path, 0x55, 0x0001e, 0x0);
    rf_wrf(path, 0xef, 0x00004, 0x1);
    rf_wrf(path, 0x00, 0xfffff, 0x403e0 | _iqk.syn1to2);
    delay_us(10);
    rf_wrf(path, 0x11, 0x00003, 0x0);
    rf_wrf(path, 0x11, 0x00070, 0x6);
    break;
  case BAND_ON_6G:
    rf_wrf(path, 0x64, 0x38000, 0x0);
    rf_wrf(path, 0x7f, 0x00080, 0x1);
    rf_wrf(path, 0x65, 0xf0000, 0xf);
    rf_wrf(path, 0x55, 0x0001e, 0x0);
    rf_wrf(path, 0xef, 0x00004, 0x1);
    rf_wrf(path, 0x00, 0xfffff, 0x403e0 | _iqk.syn1to2);
    delay_us(10);
    rf_wrf(path, 0x11, 0x00003, 0x0);
    rf_wrf(path, 0x11, 0x00070, 0x6);
    break;
  default:
    break;
  }
}

/* ================= TXK ==================================================== */

bool HalKestrel::iqk_nbtxk_bygp_8852c(uint8_t path, uint8_t gp) {
  bool fail = false, kfail = false;
  const uint8_t thermal = 0; /* vendor uses a local thermal=0 here */
  const uint32_t cc = 0x81cc + (static_cast<uint32_t>(path) << 8);
  const uint32_t s154 = 0x8154 + (static_cast<uint32_t>(path) << 8);
  const uint32_t a6_power_range[1] = {0x0};
  const uint32_t a6_track_range[1] = {0x3};
  const uint32_t a6_gain_bb[1] = {0x12};
  const uint32_t a6_itqt[1] = {0x12};
  const uint32_t a_power_range[3] = {0x0, 0x0, 0x0};
  const uint32_t a_track_range[3] = {0x3, 0x6, 0x6};
  const uint32_t a_gain_bb[3] = {0x12, 0x0a, 0x0f};
  const uint32_t a_itqt[3] = {0x12, 0x12, 0x12};
  const uint32_t g_power_range[3] = {0x0, 0x0, 0x0};
  const uint32_t g_track_range[3] = {0x5, 0x6, 0x6};
  const uint32_t g_gain_bb[3] = {0x0e, 0x0a, 0x0e};
  const uint32_t g_itqt[3] = {0x12, 0x12, 0x12};

  if (_iqk.iqk_band[path] == BAND_ON_6G) {
    for (gp = 0x0; gp < 0x1; gp++) {
      rf_wrf(path, 0x11, 0x00003, a6_power_range[gp]);
      rf_wrf(path, 0x11, 0x00070, a6_track_range[gp]);
      rf_wrf(path, 0x11, 0x1f000, a6_gain_bb[gp]);
      bb_rmw(cc, MASKDWORD, a6_itqt[gp]);
      bb_rmw(s154, 0x00000100, 0x1);
      bb_rmw(s154, 0x00000010, 0x1);
      bb_rmw(s154, 0x00000004, 0x0);
      bb_rmw(s154, 0x00000003, gp + 1);
      bb_rmw(0x8010, 0x000000ff, 0x00);
      if (thermal < 0x15)
        bb_rmw(cc, MASKDWORD, 0x1b);
      fail = iqk_one_shot_8852c(path, ID_NBTXK);
      bb_rmw(0x9fe0, 1u << (8 + gp + path * 4), fail);
      kfail = kfail | fail;
    }
  } else {
    gp = 0x2;
    switch (_iqk.iqk_band[path]) {
    case BAND_ON_24G:
      rf_wrf(path, 0x11, 0x00003, g_power_range[gp]);
      rf_wrf(path, 0x11, 0x00070, g_track_range[gp]);
      rf_wrf(path, 0x11, 0x1f000, g_gain_bb[gp]);
      bb_rmw(cc, MASKDWORD, g_itqt[gp]);
      break;
    case BAND_ON_5G:
      rf_wrf(path, 0x11, 0x00003, a_power_range[gp]);
      rf_wrf(path, 0x11, 0x00070, a_track_range[gp]);
      rf_wrf(path, 0x11, 0x1f000, a_gain_bb[gp]);
      bb_rmw(cc, MASKDWORD, a_itqt[gp]);
      break;
    default:
      break;
    }
    bb_rmw(s154, 0x00000100, 0x1);
    bb_rmw(s154, 0x00000010, 0x1);
    bb_rmw(s154, 0x00000004, 0x0);
    bb_rmw(s154, 0x00000003, gp + 1);
    bb_rmw(0x8010, 0x000000ff, 0x00);
    if (thermal < 0x15)
      bb_rmw(cc, MASKDWORD, 0x1b);
    kfail = iqk_one_shot_8852c(path, ID_NBTXK);
  }
  _iqk.nb_txcfir[path] =
      bb_read(0x8138 + (static_cast<uint32_t>(path) << 8), MASKDWORD) | 0x2;
  if (kfail)
    _iqk.nb_txcfir[path] = 0x40000002;
  _iqk.is_wb_txiqk[path] = false;
  return kfail;
}

bool HalKestrel::txk_group_sel_8852c(uint8_t path) {
  bool fail = false;
  uint8_t gp = 0x0;
  const uint32_t cc = 0x81cc + (static_cast<uint32_t>(path) << 8);
  const uint32_t s154 = 0x8154 + (static_cast<uint32_t>(path) << 8);
  const uint32_t a6_power_range[1] = {0x0};
  const uint32_t a6_track_range[1] = {0x3};
  const uint32_t a6_gain_bb[1] = {0x12};
  const uint32_t a6_itqt[1] = {0x12};
  const uint32_t a_power_range[3] = {0x0, 0x0, 0x0};
  const uint32_t a_track_range[3] = {0x3, 0x6, 0x6};
  const uint32_t a_gain_bb[3] = {0x12, 0x0a, 0x0f};
  const uint32_t a_itqt[3] = {0x12, 0x12, 0x12};
  const uint32_t g_power_range[3] = {0x0, 0x0, 0x0};
  const uint32_t g_track_range[3] = {0x5, 0x6, 0x6};
  const uint32_t g_gain_bb[3] = {0x0e, 0x0a, 0x0e};
  const uint32_t g_itqt[3] = {0x12, 0x12, 0x12};

  if (_iqk.iqk_band[path] == BAND_ON_6G) {
    for (gp = 0x0; gp < 0x1; gp++) {
      rf_wrf(path, 0x11, 0x00003, a6_power_range[gp]);
      rf_wrf(path, 0x11, 0x00070, a6_track_range[gp]);
      rf_wrf(path, 0x11, 0x1f000, a6_gain_bb[gp]);
      bb_rmw(cc, MASKDWORD, a6_itqt[gp]);
      bb_rmw(s154, 0x00000100, 0x1);
      bb_rmw(s154, 0x00000010, 0x1);
      bb_rmw(s154, 0x00000004, 0x0);
      bb_rmw(s154, 0x00000003, gp + 1);
      bb_rmw(0x808c, MASKDWORD, 0x00010010);
      bb_rmw(0x8010, 0x000000ff, 0x00);
      fail = iqk_one_shot_8852c(path, ID_TXK);
      bb_rmw(0x9fe0, 1u << (8 + gp + path * 4), fail);
    }
  } else {
    for (gp = 0x0; gp < 0x3; gp++) {
      switch (_iqk.iqk_band[path]) {
      case BAND_ON_24G:
        rf_wrf(path, 0x11, 0x00003, g_power_range[gp]);
        rf_wrf(path, 0x11, 0x00070, g_track_range[gp]);
        rf_wrf(path, 0x11, 0x1f000, g_gain_bb[gp]);
        bb_rmw(cc, MASKDWORD, g_itqt[gp]);
        break;
      case BAND_ON_5G:
        rf_wrf(path, 0x11, 0x00003, a_power_range[gp]);
        rf_wrf(path, 0x11, 0x00070, a_track_range[gp]);
        rf_wrf(path, 0x11, 0x1f000, a_gain_bb[gp]);
        bb_rmw(cc, MASKDWORD, a_itqt[gp]);
        break;
      default:
        break;
      }
      bb_rmw(s154, 0x00000100, 0x1);
      bb_rmw(s154, 0x00000010, 0x1);
      bb_rmw(s154, 0x00000004, 0x0);
      bb_rmw(s154, 0x00000003, gp + 1);
      bb_rmw(0x808c, MASKDWORD, 0x00010010);
      bb_rmw(0x8010, 0x000000ff, 0x00);
      fail = iqk_one_shot_8852c(path, ID_TXK);
      bb_rmw(0x9fe0, 1u << (8 + gp + path * 4), fail);
    }
  }
  if (fail) {
    for (gp = 0x0; gp < 0x4; gp++)
      iqk_nbtxk_bygp_8852c(path, gp);
  }
  _iqk.is_wb_txiqk[path] = true;
  return fail;
}

/* ================= RXK ==================================================== */

void HalKestrel::iqk_rxk_setting_8852c(uint8_t path) {
  const uint32_t rfc = 0x5670 + (static_cast<uint32_t>(path) << 13);
  if (path == RF_PATH_A)
    bb_rmw(0x20fc, 0xffff0000, 0x0101);
  else
    bb_rmw(0x20fc, 0xffff0000, 0x0202);
  switch (_iqk.iqk_bw[path]) {
  case CW20:
  case CW40:
    bb_rmw(rfc, 0x00002000, 0x1);
    rxck_force_8852c(path, true, ADC_480M);
    bb_rmw(rfc, 0x60000000, 0x0);
    rf_wrf(path, 0x8f, 0x01000, 0x1);
    bb_rmw(0x12b8 + (static_cast<uint32_t>(path) << 13), 0x40000000, 0x1);
    break;
  case CW80:
    bb_rmw(rfc, 0x00002000, 0x1);
    rxck_force_8852c(path, true, ADC_960M);
    bb_rmw(rfc, 0x60000000, 0x1);
    rf_wrf(path, 0x8f, 0x01000, 0x1);
    bb_rmw(0x12b8 + (static_cast<uint32_t>(path) << 13), 0x40000000, 0x1);
    break;
  default:
    break;
  }
  bb_rmw(0x030c, 0xff000000, 0x0f);
  bb_rmw(0x030c, 0xff000000, 0x03);
  bb_rmw(0x032c, 0xffff0000, 0x0001);
  bb_rmw(0x032c, 0xffff0000, 0x0041);
  if (path == RF_PATH_A)
    bb_rmw(0x20fc, 0xffff0000, 0x1101);
  else
    bb_rmw(0x20fc, 0xffff0000, 0x2202);
}

bool HalKestrel::iqk_nbrxk_bygp_8852c(uint8_t path, uint8_t gp) {
  bool fail = false, kfail = false;
  uint8_t idx;
  uint32_t rf0 = 0, tmp = 0, bkrf0;
  const uint32_t s154 = 0x8154 + (static_cast<uint32_t>(path) << 8);
  const uint32_t rfc = 0x5670 + (static_cast<uint32_t>(path) << 13);
  const uint8_t a_gp[2] = {0x0, 0x2};
  const uint8_t g_gp[4] = {0x0, 0x1, 0x2, 0x3};
  const uint32_t a6_idxrxgain[2] = {0x190, 0x290};
  const uint32_t a6_idxattc2[2] = {0x0, 0x0};
  const uint32_t a6_idxrxagc[2] = {0x4, 0x6};
  const uint32_t a_idxrxgain[2] = {0x190, 0x310};
  const uint32_t a_idxattc2[2] = {0x0, 0x0};
  const uint32_t a_idxrxagc[2] = {0x4, 0x6};
  const uint32_t g_idxrxgain[4] = {0x1d2, 0x1ec, 0x350, 0x360};
  const uint32_t g_idxattc2[4] = {0x0, 0x7, 0x0, 0x3};
  const uint32_t g_idxrxagc[4] = {0x0, 0x1, 0x2, 0x3};

  bkrf0 = rf_rrf(path, 0x00, 0x0c000);
  if (path == RF_PATH_B) {
    rf_wrf(RF_PATH_B, 0xdc, 0x00300, 0x3);
    tmp = rf_rrf(RF_PATH_B, 0x17, 0x00c00);
    rf_wrf(RF_PATH_B, 0x1f, 0x30000, tmp);
    tmp = rf_rrf(RF_PATH_B, 0x17, 0x003ff);
    rf_wrf(RF_PATH_B, 0x1f, 0x003ff, tmp);
  }
  switch (_iqk.iqk_band[path]) {
  case BAND_ON_24G:
    rf_wrf(path, 0x00, 0xf0000, 0xc);
    rf_wrf(path, 0x00, 0x0c000, 0x0);
    rf_wrf(path, 0x84, 0xf0000, 0x9);
    break;
  case BAND_ON_5G:
    rf_wrf(path, 0x00, 0xf0000, 0xc);
    rf_wrf(path, 0x00, 0x0c000, 0x0);
    rf_wrf(path, 0x89, 0x0000f, 0x8);
    break;
  case BAND_ON_6G:
    rf_wrf(path, 0x00, 0xf0000, 0xc);
    rf_wrf(path, 0x00, 0x0c000, 0x0);
    rf_wrf(path, 0x89, 0x0000f, 0x9);
    break;
  default:
    break;
  }
  delay_us(10);
  idx = 0x1;
  switch (_iqk.iqk_band[path]) {
  case BAND_ON_24G:
    gp = g_gp[idx];
    rf_wrf(path, 0x00, 0x03ff0, g_idxrxgain[idx]);
    rf_wrf(path, 0x83, 0x0f000, g_idxattc2[idx]);
    bb_rmw(0x8044, 0x0000e000, g_idxrxagc[idx]);
    break;
  case BAND_ON_5G:
    gp = a_gp[idx];
    rf_wrf(path, 0x00, 0x03ff0, a_idxrxgain[idx]);
    rf_wrf(path, 0x8c, 0x000f0, a_idxattc2[idx]);
    bb_rmw(0x8044, 0x0000e000, a_idxrxagc[idx]);
    break;
  case BAND_ON_6G:
    gp = a_gp[idx];
    rf_wrf(path, 0x00, 0x03ff0, a6_idxrxgain[idx]);
    rf_wrf(path, 0x8c, 0x000f0, a6_idxattc2[idx]);
    bb_rmw(0x8044, 0x0000e000, a6_idxrxagc[idx]);
    break;
  default:
    break;
  }
  bb_rmw(s154, 0x00000100, 0x1);
  bb_rmw(s154, 0x00000010, 0x0);
  bb_rmw(s154, 0x00000007, gp);
  rf_wrf(path, 0x1e, MASKRF, 0x80013);
  delay_us(100);
  delay_us(100);
  tmp = rf_rrf(path, 0x00, MASKRF);
  bb_rmw(0x8024, 0x000fffff, tmp);
  bb_rmw(rfc, 0x00000002, 0x1);
  bb_rmw(0x802c, 0x0fff0000, 0x11);
  fail = iqk_one_shot_8852c(path, ID_RXAGC);
  delay_us(10);
  rf0 = rf_rrf(path, 0x00, MASKDWORD);
  (void)rf0;
  kfail = kfail | fail;

  rf_wrf(path, 0x1e, 0x0003f, 0x13);
  delay_us(100);
  delay_us(100);
  bb_rmw(0x802c, 0x0fff0000, 0x011);
  fail = iqk_one_shot_8852c(path, ID_NBRXK);
  kfail = iqk_check_nbiqc_8852c(ID_NBRXK, path);
  _iqk.nb_rxcfir[path] =
      bb_read(0x813c + (static_cast<uint32_t>(path) << 8), MASKDWORD) | 0x2;
  /* on kfail the vendor dumps the RX SRAM (HALRF_DZ_LOG debug only) — omitted;
   * no calibration effect. */
  if (kfail)
    _iqk.nb_rxcfir[path] = 0x40000002;
  if (path == RF_PATH_B)
    rf_wrf(path, 0xdc, 0x00300, 0x0);
  rf_wrf(path, 0x00, 0x0c000, bkrf0);
  rf_wrf(path, 0x1e, 0x80000, 0x0);
  _iqk.is_wb_rxiqk[path] = false;
  return kfail;
}

bool HalKestrel::rxk_group_sel_8852c(uint8_t path) {
  bool fail = false, kfail = false;
  uint8_t idx, gp = 0;
  uint8_t gp_num = 2;
  uint32_t tmp = 0, bkrf0;
  const uint32_t s154 = 0x8154 + (static_cast<uint32_t>(path) << 8);
  const uint32_t rfc = 0x5670 + (static_cast<uint32_t>(path) << 13);
  const uint8_t a_gp[2] = {0x0, 0x2};
  const uint8_t g_gp[4] = {0x0, 0x1, 0x2, 0x3};
  const uint32_t a6_idxrxgain[2] = {0x190, 0x290};
  const uint32_t a6_idxattc2[2] = {0x0, 0x0};
  const uint32_t a6_idxrxagc[2] = {0x4, 0x6};
  const uint32_t a_idxrxgain[2] = {0x190, 0x310};
  const uint32_t a_idxattc2[2] = {0x0, 0x0};
  const uint32_t a_idxrxagc[2] = {0x4, 0x6};
  const uint32_t g_idxrxgain[4] = {0x1d2, 0x1ec, 0x350, 0x360};
  const uint32_t g_idxattc2[4] = {0x0, 0x7, 0x0, 0x3};
  const uint32_t g_idxrxagc[4] = {0x0, 0x1, 0x2, 0x3};

  bkrf0 = rf_rrf(path, 0x00, 0x0c000);
  if (path == RF_PATH_B) {
    rf_wrf(RF_PATH_B, 0xdc, 0x00300, 0x3);
    tmp = rf_rrf(RF_PATH_B, 0x17, 0x00c00);
    rf_wrf(RF_PATH_B, 0x1f, 0x30000, tmp);
    tmp = rf_rrf(RF_PATH_B, 0x17, 0x003ff);
    rf_wrf(RF_PATH_B, 0x1f, 0x003ff, tmp);
  }
  switch (_iqk.iqk_band[path]) {
  case BAND_ON_24G:
    rf_wrf(path, 0x00, 0xf0000, 0xc);
    rf_wrf(path, 0x00, 0x0c000, 0x0);
    rf_wrf(path, 0x84, 0xf0000, 0x9);
    gp_num = 4;
    break;
  case BAND_ON_5G:
    rf_wrf(path, 0x00, 0xf0000, 0xc);
    rf_wrf(path, 0x00, 0x0c000, 0x0);
    rf_wrf(path, 0x89, 0x0000f, 0x8);
    gp_num = 2;
    break;
  case BAND_ON_6G:
    rf_wrf(path, 0x00, 0xf0000, 0xc);
    rf_wrf(path, 0x00, 0x0c000, 0x0);
    rf_wrf(path, 0x89, 0x0000f, 0x9);
    gp_num = 2;
    break;
  default:
    break;
  }
  delay_us(10);
  for (idx = 0; idx < gp_num; idx++) {
    switch (_iqk.iqk_band[path]) {
    case BAND_ON_24G:
      gp = g_gp[idx];
      rf_wrf(path, 0x00, 0x03ff0, g_idxrxgain[idx]);
      rf_wrf(path, 0x83, 0x0f000, g_idxattc2[idx]);
      bb_rmw(0x8044, 0x0000e000, g_idxrxagc[idx]);
      break;
    case BAND_ON_5G:
      gp = a_gp[idx];
      rf_wrf(path, 0x00, 0x03ff0, a_idxrxgain[idx]);
      rf_wrf(path, 0x8c, 0x000f0, a_idxattc2[idx]);
      bb_rmw(0x8044, 0x0000e000, a_idxrxagc[idx]);
      break;
    case BAND_ON_6G:
      gp = a_gp[idx];
      rf_wrf(path, 0x00, 0x03ff0, a6_idxrxgain[idx]);
      rf_wrf(path, 0x8c, 0x000f0, a6_idxattc2[idx]);
      bb_rmw(0x8044, 0x0000e000, a6_idxrxagc[idx]);
      break;
    default:
      break;
    }
    bb_rmw(s154, 0x00000100, 0x1);
    bb_rmw(s154, 0x00000010, 0x0);
    bb_rmw(s154, 0x00000007, gp);
    rf_wrf(path, 0x1e, MASKRF, 0x80013);
    delay_us(100);
    delay_us(100);
    tmp = rf_rrf(path, 0x00, MASKRF);
    bb_rmw(0x8024, 0x000fffff, tmp);
    bb_rmw(rfc, 0x00000002, 0x1);
    bb_rmw(0x802c, 0x0fff0000, 0x11);
    fail = iqk_one_shot_8852c(path, ID_RXAGC);
    delay_us(10);
    fail = iqk_one_shot_8852c(path, ID_RXK);
    kfail = kfail | fail;
  }
  if (path == RF_PATH_B)
    rf_wrf(path, 0xdc, 0x00300, 0x0);
  rf_wrf(path, 0x00, 0x0c000, bkrf0);
  rf_wrf(path, 0x1e, 0x80000, 0x0);
  if (kfail) {
    for (gp = 0x0; gp < 0x4; gp++)
      iqk_nbrxk_bygp_8852c(path, gp);
    _iqk.is_wb_rxiqk[path] = false;
  } else {
    _iqk.nb_rxcfir[path] = 0x40000000;
    _iqk.is_wb_rxiqk[path] = true;
  }
  return kfail;
}

/* ================= orchestration (per path) ============================== */

void HalKestrel::iqk_by_path_8852c(uint8_t path) {
  bool lok_is_fail = false;
  const uint8_t g_res[2] = {2, 6};
  const uint8_t a_res[2] = {3, 7};
  const bool nbiqk_en = _iqk.is_nbiqk;
  const uint8_t thermal = read_thermal(path);
  if (thermal < 0x14)
    _iqk.is_nbiqk = true;

  /* LOK (retry once with the second ibias table entry). */
  for (uint8_t i = 0; i < 2; i++) {
    const uint8_t ibias =
        (_iqk.iqk_band[path] == BAND_ON_24G) ? g_res[i] : a_res[i];
    lok_res_table_8852c(path, ibias);
    iqk_txk_setting_8852c(path);
    lok_is_fail = iqk_lok_8852c(path);
    if (!lok_is_fail)
      break;
  }
  if (lok_is_fail) {
    rf_wrf(path, 0x08, MASKRF, 0x80200);
    rf_wrf(path, 0x09, MASKRF, 0x80200);
    rf_wrf(path, 0x0a, MASKRF, 0x80200);
  }
  /* TXK */
  if (_iqk.is_nbiqk)
    _iqk.iqk_tx_fail[path] = iqk_nbtxk_bygp_8852c(path, 0x2);
  else
    _iqk.iqk_tx_fail[path] = txk_group_sel_8852c(path);
  /* RXK */
  iqk_rxk_setting_8852c(path);
  if (_iqk.is_nbiqk)
    _iqk.iqk_rx_fail[path] = iqk_nbrxk_bygp_8852c(path, 0x2);
  else
    _iqk.iqk_rx_fail[path] = rxk_group_sel_8852c(path);

  const uint32_t s138 = 0x8138 + (static_cast<uint32_t>(path) << 8);
  const uint32_t s13c = 0x813c + (static_cast<uint32_t>(path) << 8);
  if (_iqk.is_nbiqk) {
    bb_rmw(s138, MASKDWORD, _iqk.nb_txcfir[path]);
    bb_rmw(s13c, MASKDWORD, _iqk.nb_rxcfir[path]);
  } else {
    bb_rmw(s138, MASKDWORD, 0x40000000);
    bb_rmw(s13c, MASKDWORD, 0x40000000);
  }
  _iqk.is_nbiqk = nbiqk_en;
}

void HalKestrel::iqk_get_ch_info_8852c(uint8_t path) {
  (void)rf_rrf(path, 0x18, MASKRF); /* reg_rf18: read-only in vendor */
  /* band/bw/ch already loaded into _iqk by iqk_8852c() from the current tune. */
  _iqk.syn1to2 = 0x1; /* non-DBCC */
  bb_rmw(0x9fe0, 0xff000000, iqk_version_8852c);
  bb_rmw(0x9fe4, 0x000fu << (path * 16), _iqk.iqk_band[path]);
  bb_rmw(0x9fe4, 0x00f0u << (path * 16), _iqk.iqk_bw[path]);
  bb_rmw(0x9fe4, 0xff00u << (path * 16), _iqk.iqk_ch[path]);
  bb_rmw(0x9fe8, 0x000000ff, 0x0); /* nctl_reg_ver: cosmetic fw-log field */
}

void HalKestrel::iqk_macbb_setting_8852c(uint8_t path) {
  const uint32_t rfc = 0x5670u | (static_cast<uint32_t>(path) << 13);
  rf_wrf(path, 0x10005, 0x00001, 0x0);
  bb_rmw(0x20fc, (1u << 16) << path, 0x1);
  bb_rmw(0x20fc, (1u << 20) << path, 0x0);
  bb_rmw(0x20fc, (1u << 24) << path, 0x1);
  bb_rmw(0x20fc, (1u << 28) << path, 0x0);
  iqk_disable_rxagc_8852c(path, 0x0);
  bb_rmw(rfc, MASKDWORD, 0xf801fffd);
  bb_rmw(rfc, 0x00004000, 0x1);
  bb_rmw(rfc, 0x80000000, 0x1);
  txck_force_8852c(path, true, DAC_960M);
  bb_rmw(rfc, 0x00002000, 0x1);
  rxck_force_8852c(path, true, ADC_1920M);
  bb_rmw(rfc, 0x60000000, 0x2);
  bb_rmw(0x12b8u | (static_cast<uint32_t>(path) << 13), 0x40000000, 0x1);
  bb_rmw(0x030c, 0xff000000, 0x1f);
  bb_rmw(0x030c, 0xff000000, 0x13);
  bb_rmw(0x032c, 0xffff0000, 0x0001);
  bb_rmw(0x032c, 0xffff0000, 0x0041);
  bb_rmw(0x20fc, (1u << 20) << path, 0x1);
  bb_rmw(0x20fc, (1u << 28) << path, 0x1);
}

void HalKestrel::iqk_preset_8852c(uint8_t path) {
  const uint8_t idx = _iqk.iqk_table_idx[path];
  const uint32_t s10c = 0x810c + (static_cast<uint32_t>(path) << 8);
  const uint32_t s110 = 0x8110 + (static_cast<uint32_t>(path) << 8);
  rf_wrf(path, 0x55, 0x00001, 0x1);
  rf_wrf(path, 0x18, 0x80000, idx);
  bb_rmw(0x8104 + (static_cast<uint32_t>(path) << 8), 0x00000001, idx);
  bb_rmw(0x8154 + (static_cast<uint32_t>(path) << 8), 0x00000008, idx);
  if (_iqk.iqk_band[path] == BAND_ON_6G) {
    if (idx == 0x0)
      bb_rmw(s10c, MASKDWORD, 0x11111111);
    else
      bb_rmw(s110, MASKDWORD, 0x11111111);
  } else {
    if (idx == 0x0)
      bb_rmw(s10c, MASKDWORD, 0x33112211);
    else
      bb_rmw(s110, MASKDWORD, 0x33112211);
  }
  rf_wrf(path, 0x5, 0x00001, 0x0);
  bb_rmw(0x8008, MASKDWORD, 0x00000080);
  bb_rmw(0x8088, MASKDWORD, 0x81ff010a);
}

void HalKestrel::iqk_restore_8852c(uint8_t path) {
  bb_rmw(0x8000, MASKDWORD, 0x00001219 + (static_cast<uint32_t>(path) << 4));
  delay_us(200);
  (void)iqk_check_cal_8852c(path, 0x12);
  bb_rmw(0x8010, 0x000000ff, 0x00);
  bb_rmw(0x8008, MASKDWORD, 0x00000000);
  bb_rmw(0x8088, MASKDWORD, 0x80000000);
  rf_wrf(path, 0xef, 0x00004, 0x0);
  rf_wrf(path, 0x00, 0xf0000, 0x3);
  rf_wrf(path, 0x05, 0x00001, 0x1);
}

void HalKestrel::iqk_afebb_restore_8852c(uint8_t path) {
  if (path == RF_PATH_A) {
    bb_rmw(0x12b8, 0x40000000, 0x0);
    bb_rmw(0x20fc, 0x00010000, 0x1);
    bb_rmw(0x20fc, 0x00100000, 0x0);
    bb_rmw(0x20fc, 0x01000000, 0x1);
    bb_rmw(0x20fc, 0x10000000, 0x0);
    bb_rmw(0x5670, MASKDWORD, 0x00000000);
    bb_rmw(0x12a0, 0x000ff000, 0x00);
    bb_rmw(0x20fc, 0x00010000, 0x0);
    bb_rmw(0x20fc, 0x01000000, 0x0);
    rf_wrf(RF_PATH_A, 0x10005, 0x00001, 0x1);
  } else {
    bb_rmw(0x32b8, 0x40000000, 0x0);
    bb_rmw(0x20fc, 0x00020000, 0x1);
    bb_rmw(0x20fc, 0x00200000, 0x0);
    bb_rmw(0x20fc, 0x02000000, 0x1);
    bb_rmw(0x20fc, 0x20000000, 0x0);
    bb_rmw(0x7670, MASKDWORD, 0x00000000);
    bb_rmw(0x32a0, 0x000ff000, 0x00);
    bb_rmw(0x20fc, 0x00020000, 0x0);
    bb_rmw(0x20fc, 0x02000000, 0x0);
    rf_wrf(RF_PATH_B, 0x10005, 0x00001, 0x1);
  }
  iqk_disable_rxagc_8852c(path, 0x1); /* re-enable RXAGC */
}

/* halbb_bb_reset_en_8852c (non-DBCC: both path domains). */
void HalKestrel::bb_reset_en_8852c(bool en, uint8_t band) {
  if (en) {
    bb_rmw(0x1200, 0x70000000, 0x0);
    bb_rmw(0x3200, 0x70000000, 0x0);
    bb_rmw(0x704, 1u << 1, 1);
    if (band == BAND_ON_24G)
      bb_rmw(0x2320, 1u << 0, 0x0);
    bb_rmw(0xc3c, 1u << 9, 0x0);
  } else {
    if (band == BAND_ON_24G)
      bb_rmw(0x2320, 1u << 0, 0x1);
    bb_rmw(0xc3c, 1u << 9, 0x1);
    bb_rmw(0x1200, 0x70000000, 0x7);
    bb_rmw(0x3200, 0x70000000, 0x7);
    delay_us(1);
    bb_rmw(0x704, 1u << 1, 0);
  }
}

/* halbb_ctrl_rx_cca_8852c (single-PHY). */
void HalKestrel::bb_ctrl_rx_cca_8852c(bool cca_en, uint8_t band) {
  const bool band_2g = (band == BAND_ON_24G);
  if (cca_en) {
    bb_rmw(0xc3c, 1u << 9, 0);
    /* cck_en (2G) omitted: 8852C CCK enable rides the phy_reg table. */
    (void)band_2g;
  } else {
    bb_rmw(0xc3c, 1u << 9, 1);
    bb_rmw(0x5818, 1u << 30, 1); /* TSSI pause */
    bb_rmw(0x7818, 1u << 30, 1);
    bb_reset_en_8852c(false, band);
    bb_reset_en_8852c(true, band);
    bb_rmw(0x5818, 1u << 30, 0);
    bb_rmw(0x7818, 1u << 30, 0);
    delay_us(1);
  }
}

/* halrf_doiqk (per path): backup -> setup -> LOK/TXK/RXK -> restore. */
void HalKestrel::iqk_doiqk_8852c(uint8_t path) {
  uint32_t backup_bb[sizeof(kBackupBbReg8852c) / sizeof(uint32_t)];
  uint32_t backup_rf[sizeof(kBackupRfReg8852c) / sizeof(uint32_t)];
  const uint8_t band = _iqk.iqk_band[path];

  iqk_get_ch_info_8852c(path);
  for (size_t i = 0; i < sizeof(kBackupBbReg8852c) / sizeof(uint32_t); i++)
    backup_bb[i] = bb_read(kBackupBbReg8852c[i], MASKDWORD);
  for (size_t i = 0; i < sizeof(kBackupRfReg8852c) / sizeof(uint32_t); i++)
    backup_rf[i] = rf_rrf(path, kBackupRfReg8852c[i], MASKRF);
  bb_ctrl_rx_cca_8852c(false, band);
  iqk_macbb_setting_8852c(path);
  iqk_preset_8852c(path);
  iqk_by_path_8852c(path); /* iqk_start_iqk */
  iqk_restore_8852c(path);
  iqk_afebb_restore_8852c(path);
  for (size_t i = 0; i < sizeof(kBackupBbReg8852c) / sizeof(uint32_t); i++)
    bb_rmw(kBackupBbReg8852c[i], MASKDWORD, backup_bb[i]);
  bb_ctrl_rx_cca_8852c(true, band);
  for (size_t i = 0; i < sizeof(kBackupRfReg8852c) / sizeof(uint32_t); i++)
    rf_wrf(path, kBackupRfReg8852c[i], MASKRF, backup_rf[i]);
}

/* halrf_config_8852c_nctl_reg: load the cal-engine microcode table. Applied
 * once after the RF tables, before DACK/IQK/DPK/TSSI. */
void HalKestrel::config_nctl_reg_8852c() {
  bb_rmw(0x0c60, 0x00000003, 0x3);
  bb_rmw(0x0c6c, 0x00000001, 0x1);
  bb_rmw(0x58ac, 0x08000000, 0x1);
  bb_rmw(0x78ac, 0x08000000, 0x1);
  /* NCTL ready handshake: 0x8000=8, then poll 0x8080==4 (<=1000us). */
  bb_rmw(0x8000, MASKDWORD, 0x8);
  for (uint32_t cnt = 0; cnt < 1000; cnt++) {
    bb_rmw(0x8080, MASKDWORD, 0x4);
    delay_us(1);
    if (bb_read(0x8080, MASKDWORD) == 0x4)
      break;
  }
  const uint32_t n = array_mp_8852c_nctl_reg_len;
  for (uint32_t i = 0; i + 1 < n; i += 2) {
    const uint32_t addr = array_mp_8852c_nctl_reg[i];
    const uint32_t data = array_mp_8852c_nctl_reg[i + 1];
    /* halrf_cfg_rf_nctl_8852c: 0xfe..0xf9 are delay opcodes, else a BB write. */
    switch (addr) {
    case 0xfe: delay_us(50000); break;
    case 0xfd: delay_us(5000); break;
    case 0xfc: delay_us(1000); break;
    case 0xfb: delay_us(50); break;
    case 0xfa: delay_us(5); break;
    case 0xf9: delay_us(1); break;
    default: bb_rmw(addr, MASKDWORD, data); break;
    }
  }
  _logger->info("Kestrel RFK(8852C): NCTL cal-engine microcode loaded "
                "({} entries)",
                n / 2);
}

/* halrf_iqk entry: kpath = RF_AB (non-DBCC) -> path A then B. */
void HalKestrel::iqk_8852c(uint8_t band, uint8_t bw, uint8_t center_ch) {
  for (uint8_t path = 0; path < 2; path++) {
    _iqk.iqk_band[path] = band;
    _iqk.iqk_bw[path] = bw;
    _iqk.iqk_ch[path] = center_ch;
    _iqk.iqk_table_idx[path] = 0;
  }
  _logger->info("Kestrel RFK(8852C): IQK band={} bw={} ch={} (RF_AB)", band, bw,
                center_ch);
  iqk_doiqk_8852c(RF_PATH_A);
  iqk_doiqk_8852c(RF_PATH_B);
}

} // namespace kestrel
