#include "PhydmRuntimeJaguar3.h"

namespace jaguar3 {

namespace {
/* phydm_dig.h coverage bounds — the unlinked DIG operating window. */
/* phydm's generic unlinked floor is DIG_MIN_COVERAGE 0x1c, but the kernel
 * carries per-IC "For HW setting" exceptions (e.g. 0x1e on the 8197F) — and
 * the Jaguar3 parts need one: at IGI 0x1c the 8822CU's MCS4+ RX decodes
 * NOTHING (0 of 65k kernel-injected MCS7 frames) while 0x1e is transparent
 * (65.9k of 65k) — hardware-bisected, value-specific. Floor at 0x1e. */
constexpr uint8_t DIG_MIN_COVERAGE = 0x1e;
constexpr uint8_t DIG_MAX_OF_MIN_COVERAGE = 0x22;
constexpr uint8_t DIG_MIN_DFS = 0x20;
/* phydm_adaptivity.h normal-mode EDCCA constants. */
constexpr int TH_L2H_DIFF_IGI = 8;
constexpr int EDCCA_TH_L2H_LB = 48;
constexpr int EDCCA_HL_DIFF_NORMAL = 8;
} // namespace

/* IGI lives in 0x1d70, 7-bit field per RX path (A [6:0], B [14:8]) —
 * phydm_write_dig_reg_jgr3 / phydm_get_igi_reg_val_jgr3. */
uint8_t PhydmRuntimeJaguar3::get_igi() {
  return static_cast<uint8_t>(_device.rtw_read32(0x1d70) & 0x7f);
}
void PhydmRuntimeJaguar3::set_igi(uint8_t igi) {
  _device.phy_set_bb_reg(0x1d70, 0x0000007f, igi);
  _device.phy_set_bb_reg(0x1d70, 0x00007f00, igi);
}

/* phydm_fa_cnt_statistics_jgr3 + phydm_false_alarm_counter_reg_reset: read
 * the per-window false-alarm and CCA counters, then clear and re-arm them so
 * the next tick sees a fresh window. */
PhydmRuntimeJaguar3::FaStats PhydmRuntimeJaguar3::fa_statistics_and_reset() {
  FaStats s;
  uint32_t v;

  v = _device.rtw_read32(0x2d04);
  uint32_t ofdm_fail = (v >> 16) & 0xffff; /* parity fail */
  v = _device.rtw_read32(0x2d08);
  ofdm_fail += (v & 0xffff) + ((v >> 16) & 0xffff); /* rate illegal + crc8 */
  v = _device.rtw_read32(0x2d10);
  ofdm_fail += (v & 0xffff) + ((v >> 16) & 0xffff); /* mcs fail (HT + VHT) */
  v = _device.rtw_read32(0x2d0c);
  ofdm_fail += v & 0xffff; /* crc8 fail (VHT SIG-A) */
  v = _device.rtw_read32(0x2d20);
  ofdm_fail += (v & 0xffff) + ((v >> 16) & 0xffff); /* fast_fsync + sb_search */

  s.cck_fa = _device.rtw_read32(0x1a5c) & 0xffff;

  v = _device.rtw_read32(0x2c08);
  const uint32_t ofdm_cca = (v >> 16) & 0xffff;
  const uint32_t cck_cca = v & 0xffff;

  /* CCK contributes only when its RxIQ weighting is enabled (2.4 GHz —
   * 0x1a14[9:8] == 0), mirroring the channel path's cck_rxiq toggle. */
  s.cck_enabled = ((_device.rtw_read32(0x1a14) >> 8) & 0x3) == 0;
  s.fa_all = ofdm_fail + (s.cck_enabled ? s.cck_fa : 0);
  s.cca_all = ofdm_cca + (s.cck_enabled ? cck_cca : 0);

  /* Counter reset: CCK FA (0x1a2c[15:14] 0->2) and CCK CCA ([13:12] 0->2),
   * then the OFDM/CCA block via the page-2 counter reset (0x1eb4[25] pulse)
   * bracketed by the RX clock-gate bit (0x1d2c[31]). */
  _device.phy_set_bb_reg(0x1a2c, 0x0000c000, 0);
  _device.phy_set_bb_reg(0x1a2c, 0x0000c000, 2);
  _device.phy_set_bb_reg(0x1a2c, 0x00003000, 0);
  _device.phy_set_bb_reg(0x1a2c, 0x00003000, 2);
  _device.phy_set_bb_reg(0x1d2c, 1u << 31, 0);
  _device.phy_set_bb_reg(0x1eb4, 1u << 25, 1);
  _device.phy_set_bb_reg(0x1eb4, 1u << 25, 0);
  _device.phy_set_bb_reg(0x1d2c, 1u << 31, 1);
  return s;
}

/* phydm_dig, unlinked path: step the IGI by the per-window false-alarm level
 * — fa_th {2000, 4000, 5000} steps {+2, +1} up / -2 down (DFS band:
 * {250, 1000, 2000} with the IGI pinned at DIG_MIN_DFS by the boundary
 * rules), clamped to the coverage window. */
void PhydmRuntimeJaguar3::dig(const FaStats &fa, bool is_dfs) {
  int igi = get_igi();

  uint32_t th0 = 2000, th1 = 4000, th2 = 5000;
  uint8_t lo = DIG_MIN_COVERAGE, hi = DIG_MAX_OF_MIN_COVERAGE;
  if (is_dfs) {
    th0 = 250; th1 = 1000; th2 = 2000;
    lo = DIG_MIN_DFS; hi = DIG_MIN_DFS;
  }

  if (fa.fa_all > th2)
    igi += 2;
  else if (fa.fa_all > th1)
    igi += 1;
  else if (fa.fa_all < th0)
    igi -= 2;

  if (igi < lo)
    igi = lo;
  if (igi > hi)
    igi = hi;

  if (igi != get_igi())
    set_igi(static_cast<uint8_t>(igi));
  if (igi != _last_igi_logged) {
    _last_igi_logged = igi;
    _logger->info("Jaguar3 dig: IGI=0x{:02x} (fa={} cca={})", igi, fa.fa_all,
                  fa.cca_all);
  }
}

/* phydm_cckpd_type4, unlinked: 75%-smoothed CCK-FA moving average drives a
 * two-level packet-detection ladder (>1000 -> LV1, <500 -> LV0, else hold).
 * The per-level PD/CS values derive from the BB-table defaults in
 * 0x1ac8/0x1acc/0x1ad0 plus the 8822C/E step ladder
 * (phydm_cck_pd_init_type4); the level is applied per current bw/nrx
 * (phydm_set_cck_pd_lv_type4 / phydm_write_cck_pd_type4). */
void PhydmRuntimeJaguar3::cck_pd(const FaStats &fa) {
  if (!_cck_tbl_valid) {
    const uint32_t r0 = _device.rtw_read32(0x1ac8); /* PD limit, BW20 per nrx */
    const uint32_t r1 = _device.rtw_read32(0x1acc); /* PD limit, BW40 per nrx */
    const uint32_t r2 = _device.rtw_read32(0x1ad0); /* CS ratio fields */
    static const uint8_t pw_step[5] = {0, 9, 12, 14, 17};
    static const uint8_t cs_step[5] = {0, 0, 1, 1, 1};
    for (int lv = 0; lv < 5; ++lv) {
      _cckpd_tbl[0][0][0][lv] = static_cast<uint8_t>((r0 & 0xff) + pw_step[lv]);
      _cckpd_tbl[1][0][0][lv] = static_cast<uint8_t>((r1 & 0xff) + pw_step[lv]);
      _cckpd_tbl[0][0][1][lv] =
          static_cast<uint8_t>((r2 & 0x1f) + cs_step[lv]);
      _cckpd_tbl[1][0][1][lv] =
          static_cast<uint8_t>(((r2 >> 20) & 0x1f) + cs_step[lv]);
      _cckpd_tbl[0][1][0][lv] =
          static_cast<uint8_t>(((r0 >> 8) & 0xff) + pw_step[lv]);
      _cckpd_tbl[1][1][0][lv] =
          static_cast<uint8_t>(((r1 >> 8) & 0xff) + pw_step[lv]);
      _cckpd_tbl[0][1][1][lv] =
          static_cast<uint8_t>(((r2 >> 5) & 0x1f) + cs_step[lv]);
      _cckpd_tbl[1][1][1][lv] =
          static_cast<uint8_t>(((r2 >> 25) & 0x1f) + cs_step[lv]);
    }
    _cck_tbl_valid = true;
  }

  if (_cck_fa_ma == 0xffffffff)
    _cck_fa_ma = fa.cck_fa;
  else
    _cck_fa_ma = (_cck_fa_ma * 3 + fa.cck_fa) >> 2;

  uint8_t lv;
  if (_cck_fa_ma > 1000)
    lv = 1;
  else if (_cck_fa_ma < 500)
    lv = 0;
  else
    return; /* hold */

  /* nrx (0x1a2c[18:17]+1) and CCK bandwidth (0x9b0[3:2]) are re-checked at
   * every apply like the vendor's; only 20/40 MHz have table entries. */
  const uint8_t nrx =
      static_cast<uint8_t>(((_device.rtw_read32(0x1a2c) >> 17) & 0x3) + 1);
  const uint8_t bwc =
      static_cast<uint8_t>((_device.rtw_read32(0x9b0) >> 2) & 0x3);
  if (bwc > 1 || nrx < 1 || nrx > 2)
    return;
  if (_cck_lv_valid && lv == _cck_lv && bwc == _cck_bw && nrx == _cck_nrx)
    return;
  _cck_lv = lv;
  _cck_bw = bwc;
  _cck_nrx = nrx;
  _cck_lv_valid = true;
  _cck_fa_ma = 0xffffffff; /* CCK_FA_MA_RESET on level apply */

  const uint8_t pd = _cckpd_tbl[bwc][nrx - 1][0][lv];
  const uint8_t cs = _cckpd_tbl[bwc][nrx - 1][1][lv];
  if (bwc == 0) { /* BW20 */
    _device.phy_set_bb_reg(0x1ac8, nrx == 1 ? 0x000000ffu : 0x0000ff00u, pd);
    _device.phy_set_bb_reg(0x1ad0, nrx == 1 ? 0x0000001fu : 0x000003e0u, cs);
  } else { /* BW40 */
    _device.phy_set_bb_reg(0x1acc, nrx == 1 ? 0x000000ffu : 0x0000ff00u, pd);
    _device.phy_set_bb_reg(0x1ad0, nrx == 1 ? 0x01f00000u : 0x3e000000u, cs);
  }
  _logger->info("Jaguar3 cckpd: lv={} pd=0x{:02x} cs=0x{:02x} (fa_ma window)",
                lv, pd, cs);
}

/* phydm_edcca_thre_calc_jgr3, normal (non-adaptivity-regulation) mode:
 * th_l2h = max(igi + 8, 48), th_h2l = th_l2h - 8, written to 0x84c as
 * biased bytes (value + 0x80): L2H in [23:16], H2L in [31:24]. */
void PhydmRuntimeJaguar3::edcca(uint8_t igi) {
  const int th_l2h =
      (igi + TH_L2H_DIFF_IGI > EDCCA_TH_L2H_LB) ? igi + TH_L2H_DIFF_IGI
                                                : EDCCA_TH_L2H_LB;
  const int th_h2l = th_l2h - EDCCA_HL_DIFF_NORMAL;
  _device.phy_set_bb_reg(0x84c, 0x00ff0000,
                         static_cast<uint32_t>(th_l2h + 0x80) & 0xff);
  _device.phy_set_bb_reg(0x84c, 0xff000000,
                         static_cast<uint32_t>(th_h2l + 0x80) & 0xff);
  if (th_l2h != _last_l2h_logged) {
    _last_l2h_logged = th_l2h;
    _logger->info("Jaguar3 edcca: L2H={} H2L={}", th_l2h, th_h2l);
  }
}

void PhydmRuntimeJaguar3::tick(uint8_t channel, bool edcca_track) {
  const FaStats fa = fa_statistics_and_reset();
  /* DFS-band decision as the vendor driver's rtw_is_dfs_ch: 5 GHz ch 52-144. */
  const bool is_dfs = channel > 14 && channel >= 52 && channel <= 144;
  dig(fa, is_dfs);
  if (fa.cck_enabled)
    cck_pd(fa);
  if (edcca_track)
    edcca(get_igi());
}

} // namespace jaguar3
