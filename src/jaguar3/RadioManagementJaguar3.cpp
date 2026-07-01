#include "RadioManagementJaguar3.h"

#include <array>
#include <utility>

#include "RateDefinitions.h" /* MGN_* rate enum */
#if defined(DEVOURER_HAVE_JAGUAR3_8822E)
#include "Hal8822e_PhyTables.h"    /* array_mp_8822e_phy_reg_pg */
#endif

extern "C" {
#include "ieee80211_radiotap.h" /* MRateToHwRate */
}

namespace jaguar3 {

RadioManagementJaguar3::RadioManagementJaguar3(RtlUsbAdapter device, Logger_t logger)
    : _device{device}, _logger{std::move(logger)} {}

void RadioManagementJaguar3::set_channel_bwmode(uint8_t channel,
                                              uint8_t /*channel_offset*/,
                                              ChannelWidth_t bwmode) {
  /* 40/80 MHz HT/VHT bandwidth programming is not ported yet (only the 20 MHz
   * and the 5/10 MHz narrowband re-clock paths are). Tune at 20 MHz rather than
   * aborting, so a 40/80 request degrades to a working narrower channel instead
   * of killing the process. */
  if (bwmode == CHANNEL_WIDTH_40 || bwmode == CHANNEL_WIDTH_80)
    _logger->info("Jaguar3: {} MHz BW not yet supported — tuning at 20 MHz",
                  bwmode == CHANNEL_WIDTH_40 ? 40 : 80);
  /* 5/10 MHz: tune the channel at 20 MHz first, then re-clock via
   * set_bandwidth_dividers() at the end of this function. */

  /* RF register write via the 8822C direct-write window:
   * BB[base + (rf_addr<<2)], 20-bit mask. base 0x3c00 (A) / 0x4c00 (B). */
  auto rf_write = [this](uint16_t base, uint8_t rfaddr, uint32_t v) {
    _device.phy_set_bb_reg(static_cast<uint16_t>(base + (rfaddr << 2)),
                           0x000fffff, v);
  };

  /* --- bandwidth 20 MHz (config_phydm_switch_bandwidth_8822c) --- */
  _device.phy_set_bb_reg(0x810, 0x3ff0, 0x19b);        /* RX DFIR 20M */
  _device.phy_set_bb_reg(0x9b0, 0xffc0, 0x0);          /* small-BW = 20M */
  _device.phy_set_bb_reg(0x9b4, 0x00000700, 0x7);      /* DAC 480M */
  _device.phy_set_bb_reg(0x9b4, 0x00700000, 0x6);      /* ADC 160M */
  _device.phy_set_bb_reg(0x9b0, 0xf, 0x0);             /* RF BW */
  _device.phy_set_bb_reg(0xcbc, 1u << 21, 0x0);        /* pilot smoothing */
  _device.phy_set_bb_reg(0x1abc, 1u << 30, 0x0);       /* CCK source */
  _device.phy_set_bb_reg(0x1ae8, 1u << 31, 0x1);
  _device.phy_set_bb_reg(0x1aec, 0xf, 0x6);            /* dynamic CCK PD */
  _device.phy_set_bb_reg(0x88c, 0xf000, 0x1);          /* subtune */

  /* --- channel tune (RF 0x18, config_phydm_switch_channel_8822c): clear
   * [18:17]/[16]/[9:8]/[7:0], set channel; for 5 GHz set BIT16|BIT8 ALWAYS plus
   * the sub-band bit (ch>144 -> BIT18, ch>=80 -> BIT17, else none). The 0x3000
   * band bits are preserved. Matches the kernel ch36 value 0x13124
   * (0x24|0x3000|0x100|0x10000); e.g. ch149 -> 0x53195, ch100 -> 0x33164. --- */
  uint32_t rf18 = (channel & 0xffu) | 0x3000u;
  if (channel > 14) { /* 5 GHz */
    rf18 |= (1u << 16) | (1u << 8);
    if (channel > 144) rf18 |= (1u << 18);
    else if (channel >= 80) rf18 |= (1u << 17);
  }
  _logger->info("Jaguar3: channel {} RF18=0x{:05x}", channel, rf18);
  rf_write(0x3c00, 0x18, rf18);
  rf_write(0x4c00, 0x18, rf18);
  rf_write(0x3c00, 0x3f, 0x18);  /* RXBB 20/40 */
  rf_write(0x4c00, 0x3f, 0x18);
  bool is_2g = channel <= 14;
  _device.phy_set_bb_reg(static_cast<uint16_t>(0x3c00 + (0xdf << 2)), 1u << 18,
                         is_2g ? 1 : 0);

  /* --- 2G/5G band BB --- */
  if (is_2g) {
    _device.rtw_write8(0x454,
                       static_cast<uint8_t>(_device.rtw_read8(0x454) & ~0x80));
    _device.phy_set_bb_reg(0x1a80, 1u << 18, 0x0);
    _device.phy_set_bb_reg(0x1c80, 0x3F000000, 0xF);
  } else {
    _device.rtw_write8(0x454,
                       static_cast<uint8_t>(_device.rtw_read8(0x454) | 0x80));
    _device.phy_set_bb_reg(0x1a80, 1u << 18, 0x1);
    _device.phy_set_bb_reg(0x1c80, 0x3F000000, 0x22);
  }

  /* SCO tracking f_c (phydm_sco_trk_fc_setting_8822c): BB 0xc30[11:0]. */
  uint32_t sco;
  if (channel >= 36 && channel <= 51) sco = 0x494;
  else if (channel >= 52 && channel <= 55) sco = 0x493;
  else if (channel >= 56 && channel <= 111) sco = 0x453;
  else if (channel >= 112 && channel <= 119) sco = 0x452;
  else if (channel >= 120 && channel <= 172) sco = 0x412;
  else if (channel >= 173) sco = 0x411;
  else if (channel >= 1 && channel <= 10) sco = 0x9aa;
  else if (channel == 11 || channel == 12) sco = 0x96a;
  else sco = 0x969; /* 13/14 */
  _device.phy_set_bb_reg(0xc30, 0xfff, sco);

  /* TX DFIR (phydm_tx_dfir_setting_8822c): BB 0x808[22:20] + [6:4]. */
  if (is_2g) {
    _device.phy_set_bb_reg(0x808, 0x700000, channel == 11 ? 0x3 : 0x1);
    _device.phy_set_bb_reg(0x808, 0x70, channel == 13 ? 0x3 : 0x1);
  } else {
    _device.phy_set_bb_reg(0x808, 0x700000, 0x1);
    _device.phy_set_bb_reg(0x808, 0x70, 0x3);
  }

  /* phydm_bb_reset_8822c: toggle the BB reset (MAC reg 0x0 BIT16, 1->0->1) to
   * (re)start the receiver after channel/BW config — the kernel does this after
   * every switch_channel; without it the RX engine never runs. */
  uint32_t r0 = _device.rtw_read32(0x0);
  _device.rtw_write32(0x0, r0 | (1u << 16));
  _device.rtw_write32(0x0, r0 & ~(1u << 16));
  _device.rtw_write32(0x0, r0 | (1u << 16));

  _logger->info("Jaguar3: channel {} / 20MHz set (RF18=0x{:05x}) + BB reset",
                channel, rf18);

  /* For 5/10 MHz: channel is now tuned at 20 MHz; re-clock the baseband. */
  if (bwmode == CHANNEL_WIDTH_5 || bwmode == CHANNEL_WIDTH_10)
    set_bandwidth_dividers(bwmode);
}

void RadioManagementJaguar3::set_tx_power_ref(uint8_t idx, bool zero_diffs) {
  /* Override the 8822C TX-power reference on both RF paths (port of
   * rtw8822c_set_write_tx_power_ref + zeroing the per-rate diffs). idx is a
   * 7-bit power index (0..0x7f); higher = more power. Each txagc write must be
   * preceded by clearing 0x1c90[15] (the txagc-write enable gate).
   *   OFDM ref: 0x18e8 (A) / 0x41e8 (B), field [16:10] (mask 0x1fc00)
   *   CCK  ref: 0x18a0 (A) / 0x41a0 (B), field [22:16] (mask 0x7f0000)
   * Then zero the per-rate diff table at 0x3a00.. so every rate emits at the
   * reference level (uniform max for a strong, simple test/link signal). */
  if (idx > 0x7f)
    idx = 0x7f;
  auto wr = [this](uint16_t off, uint32_t mask, uint32_t v) {
    _device.phy_set_bb_reg(0x1c90, 1u << 15, 0); /* txagc write enable */
    _device.phy_set_bb_reg(off, mask, v);
  };
  wr(0x18e8, 0x1fc00, idx);
  wr(0x41e8, 0x1fc00, idx);
  wr(0x18a0, 0x7f0000, idx);
  wr(0x41a0, 0x7f0000, idx);
  if (zero_diffs)
    for (uint16_t off = 0x3a00; off <= 0x3a7c; off += 4)
      wr(off, 0xffffffff, 0x0);
  _logger->info("Jaguar3: TX power reference set to 0x{:02x} on both paths "
                "(per-rate diffs {})",
                idx, zero_diffs ? "zeroed" : "kept");
}

/* Map a phy_reg_pg pseudo-address to the four MGN_* rates it encodes (byte i of
 * the packed value -> Rate[i]). Ported from _phy_get_rate_values_of_txpwr_by_rate
 * (hal_com_phycfg.c). Returns false for CCK/unknown addresses (handled via the
 * CCK reference, not the OFDM/HT/VHT 0x3a00 diff table). Path-B addresses
 * (0xE20..) carry identical values to path A, so only the 0xC2x/0xC3x/0xC4x set
 * is needed to fill the (path-shared) 0x3a00 table. */
#if defined(DEVOURER_HAVE_JAGUAR3_8822E)
static bool pg_addr_to_rates(uint32_t addr, std::array<uint8_t, 4> &rates) {
  switch (addr) {
  case 0xC24: rates = {MGN_6M, MGN_9M, MGN_12M, MGN_18M}; return true;
  case 0xC28: rates = {MGN_24M, MGN_36M, MGN_48M, MGN_54M}; return true;
  case 0xC2C: rates = {MGN_MCS0, MGN_MCS1, MGN_MCS2, MGN_MCS3}; return true;
  case 0xC30: rates = {MGN_MCS4, MGN_MCS5, MGN_MCS6, MGN_MCS7}; return true;
  case 0xC34: rates = {MGN_MCS8, MGN_MCS9, MGN_MCS10, MGN_MCS11}; return true;
  case 0xC38: rates = {MGN_MCS12, MGN_MCS13, MGN_MCS14, MGN_MCS15}; return true;
  case 0xC3C:
    rates = {MGN_VHT1SS_MCS0, MGN_VHT1SS_MCS1, MGN_VHT1SS_MCS2,
             MGN_VHT1SS_MCS3};
    return true;
  case 0xC40:
    rates = {MGN_VHT1SS_MCS4, MGN_VHT1SS_MCS5, MGN_VHT1SS_MCS6,
             MGN_VHT1SS_MCS7};
    return true;
  case 0xC44:
    rates = {MGN_VHT1SS_MCS8, MGN_VHT1SS_MCS9, MGN_VHT2SS_MCS0,
             MGN_VHT2SS_MCS1};
    return true;
  case 0xC48:
    rates = {MGN_VHT2SS_MCS2, MGN_VHT2SS_MCS3, MGN_VHT2SS_MCS4,
             MGN_VHT2SS_MCS5};
    return true;
  case 0xC4C:
    rates = {MGN_VHT2SS_MCS6, MGN_VHT2SS_MCS7, MGN_VHT2SS_MCS8,
             MGN_VHT2SS_MCS9};
    return true;
  default:
    return false; /* 0xC20 = CCK, or a path-B / unknown address */
  }
}
#endif /* DEVOURER_HAVE_JAGUAR3_8822E */

void RadioManagementJaguar3::apply_power_by_rate_8822e(uint8_t channel,
                                                       uint8_t ref_a,
                                                       uint8_t ref_b) {
#if defined(DEVOURER_HAVE_JAGUAR3_8822E)
  /* Port of the phy_reg_pg (power-by-rate) apply that devourer's table walk
   * skips. The 8822e TXAGC is ref + per-rate diff: the OFDM/HT/VHT reference
   * lives in 0x18e8 (A)/0x41e8 (B), and the signed per-rate diff table at
   * 0x3a00 + (hw_rate & 0xfc) (4 rates/word, byte = hw_rate & 3) — see
   * phydm_hal_api8822e config_phydm_write_txagc_{ref,diff}. Without it every
   * rate emits at the flat reference, so the robust low rates the kernel boosts
   * (e.g. MCS0 = MCS7 + 6 dB) come out ~2 dB low.
   *
   * phy_reg_pg holds the per-rate absolute indices (PHY_REG_PG_EXACT_VALUE). We
   * program diff[rate] = pg[rate] - pg[MCS7] so the reference (ref_base, tuned to
   * match the kernel's MCS7 level) anchors MCS7 at diff 0 and the by-rate spread
   * matches the kernel (MCS0 -> ref_base + 24 = the kernel's +6 dB). */
  const uint32_t band = channel <= 14 ? 0u : 1u;
  const uint32_t *pg = array_mp_8822e_phy_reg_pg;
  const uint32_t n = array_mp_8822e_phy_reg_pg_len;
  auto get_val = [](uint32_t v, int i) -> uint8_t {
    return static_cast<uint8_t>((v >> (i * 8)) & 0xff);
  };

  /* Pass 1: find the MCS7 anchor value (addr 0xC30 byte 3) for this band. */
  uint8_t anchor = 0;
  for (uint32_t i = 0; i + 6 <= n; i += 6) {
    if (pg[i] == band && pg[i + 1] == 0 && (pg[i + 3] & 0xffff) == 0xC30) {
      anchor = get_val(pg[i + 5], 3);
      break;
    }
  }
  if (anchor == 0) {
    _logger->info("Jaguar3: phy_reg_pg has no MCS7 anchor for band {} — "
                  "skipping power-by-rate", band);
    return;
  }

  /* Reference base on both paths (OFDM ref 0x18e8/0x41e8, field [16:10]). */
  auto wr = [this](uint16_t off, uint32_t mask, uint32_t v) {
    _device.phy_set_bb_reg(0x1c90, 1u << 15, 0); /* txagc write enable */
    _device.phy_set_bb_reg(off, mask, v);
  };
  wr(0x18e8, 0x1fc00, ref_a);     /* path A OFDM/HT/VHT ref */
  wr(0x41e8, 0x1fc00, ref_b);     /* path B (efuse gives a distinct per-path base) */
  wr(0x18a0, 0x7f0000, ref_a);    /* CCK ref (2.4G) */
  wr(0x41a0, 0x7f0000, ref_b);

  /* Pass 2: write the per-rate diff table for this band (path-A entries; the
   * 0x3a00 table is path-shared and path-B pg values are identical). */
  int groups = 0;
  for (uint32_t i = 0; i + 6 <= n; i += 6) {
    if (pg[i] != band || pg[i + 1] != 0)
      continue;
    std::array<uint8_t, 4> rates{};
    if (!pg_addr_to_rates(pg[i + 3] & 0xffff, rates))
      continue;
    const uint32_t value = pg[i + 5];
    uint32_t diff_word = 0;
    for (int j = 0; j < 4; ++j) {
      int diff = static_cast<int>(get_val(value, j)) - static_cast<int>(anchor);
      diff_word |= static_cast<uint32_t>(diff & 0x7f) << (j * 8);
    }
    uint8_t hw0 = MRateToHwRate(rates[0]);
    uint16_t reg = static_cast<uint16_t>(0x3a00 + (hw0 & 0xfc));
    wr(reg, 0xffffffff, diff_word);
    ++groups;
  }
  _logger->info("Jaguar3: applied phy_reg_pg power-by-rate (band {}, {} rate "
                "groups, ref A=0x{:02x} B=0x{:02x}, MCS7 anchor 0x{:02x})",
                band, groups, ref_a, ref_b, anchor);
#else
  (void)channel;
  (void)ref_a;
  (void)ref_b;
#endif
}

void RadioManagementJaguar3::set_bandwidth_dividers(ChannelWidth_t bwmode) {
  /* Narrowband baseband underclock — the Jaguar3-only payoff. Applies ONLY the
   * clock-divider / small-BW / RX-DFIR delta from config_phydm_switch_bandwidth
   * _8822c, on top of an already-tuned channel (this re-clocks the baseband
   * without re-running the RF channel tune). 10 MHz halves the DAC/ADC clock,
   * 5 MHz quarters it; radiotap stays
   * 20 MHz so only an SDR sees the change.
   *
   *   small-BW 0x9b0[7:6] : 20M=0x0  10M=0x2  5M=0x1
   *   DAC clk  0x9b4[10:8] : 20M=0x7  10M=0x6  5M=0x4
   *   ADC clk  0x9b4[22:20]: 20M=0x6  10M=0x5  5M=0x4
   *   RX DFIR  0x810[13:4] : 20M=0x19b  5/10M=0x2ab */
  uint8_t small_bw, dac, adc;
  uint16_t dfir;
  switch (bwmode) {
  case CHANNEL_WIDTH_10:
    small_bw = NB_SMALLBW_10M; dac = 0x6; adc = 0x5; dfir = 0x2ab; break;
  case CHANNEL_WIDTH_5:
    small_bw = NB_SMALLBW_5M;  dac = 0x4; adc = 0x4; dfir = 0x2ab; break;
  default: /* CHANNEL_WIDTH_20 — restore full-rate clocks */
    small_bw = 0x0; dac = 0x7; adc = 0x6; dfir = 0x19b; break;
  }
  _device.phy_set_bb_reg(R_RX_DFIR_8822C, 0x3ff0, dfir);
  _device.phy_set_bb_reg(R_SMALL_BW_8822C, 0xc0, small_bw);   /* 0x9b0[7:6] */
  _device.phy_set_bb_reg(R_CLK_DIV_8822C, 0x00000700, dac);   /* 0x9b4[10:8] */
  _device.phy_set_bb_reg(R_CLK_DIV_8822C, 0x00700000, adc);   /* 0x9b4[22:20] */

  /* phydm_bb_reset_8822c: toggle MAC 0x0 BIT16 (1->0->1) so the receiver/DFE
   * relatches at the new sample rate — without it the re-clock doesn't take. */
  uint32_t r0 = _device.rtw_read32(0x0);
  _device.rtw_write32(0x0, r0 | (1u << 16));
  _device.rtw_write32(0x0, r0 & ~(1u << 16));
  _device.rtw_write32(0x0, r0 | (1u << 16));

  const char *name = bwmode == CHANNEL_WIDTH_10  ? "10 MHz"
                     : bwmode == CHANNEL_WIDTH_5 ? "5 MHz"
                                                 : "20 MHz";
  _logger->info("Jaguar3: baseband re-clocked to {} (small_bw={} DAC={} ADC={} "
                "DFIR=0x{:03x})", name, small_bw, dac, adc, dfir);
}

} /* namespace jaguar3 */
