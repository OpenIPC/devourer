#include "RadioManagement8822c.h"

#include <utility>

namespace jaguar3 {

RadioManagement8822c::RadioManagement8822c(RtlUsbAdapter device, Logger_t logger)
    : _device{device}, _logger{std::move(logger)} {}

void RadioManagement8822c::set_channel_bwmode(uint8_t channel,
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

void RadioManagement8822c::set_tx_power_ref(uint8_t idx) {
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
  for (uint16_t off = 0x3a00; off <= 0x3a7c; off += 4)
    wr(off, 0xffffffff, 0x0);
  _logger->info("Jaguar3: TX power reference set to 0x{:02x} on both paths "
                "(per-rate diffs zeroed)",
                idx);
}

void RadioManagement8822c::set_bandwidth_dividers(ChannelWidth_t bwmode) {
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
