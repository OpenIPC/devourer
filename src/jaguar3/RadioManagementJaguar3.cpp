#include "RadioManagementJaguar3.h"

#include <array>
#include <cstdlib>
#include <utility>

#include "RateDefinitions.h" /* MGN_* rate enum */
#if defined(DEVOURER_HAVE_JAGUAR3_8822E)
#include "Hal8822e_PhyTables.h"    /* array_mp_8822e_phy_reg_pg */
#endif

extern "C" {
#include "ieee80211_radiotap.h" /* MRateToHwRate */
}

namespace jaguar3 {

RadioManagementJaguar3::RadioManagementJaguar3(RtlUsbAdapter device, Logger_t logger,
                                               ChipVariant variant)
    : _device{device}, _logger{std::move(logger)}, _variant{variant} {}

void RadioManagementJaguar3::set_channel_bwmode(uint8_t channel,
                                              uint8_t channel_offset,
                                              ChannelWidth_t bwmode) {
  /* HT40: the RF tunes the CENTRAL channel (primary ± 2) and the BB carries a
   * primary-sub-channel index. channel_offset uses the demo's HAL_PRIME
   * convention: 1 = HT40+ (primary is the LOWER 20, secondary above), 2 = HT40-
   * (primary is the UPPER 20). Vendor pri-ch index: primary-lower -> 2,
   * primary-upper -> 1 (get_pri_ch_id / VHT_DATA_SC_20_{LOWER,UPPER}).
   *
   * 80 MHz configures the wide channel; it also supports a 40-in-80 mode where
   * an 80 MHz-tuned channel carries a 40 MHz frame (send_packet sets the
   * DATA_SC sub-channel) that a standard HT40 receiver decodes — the userspace
   * equivalent of `iw 80MHz` + a 40 MHz radiotap. `channel` is the primary
   * 20 MHz and must be the LOWEST of the 80 MHz block (e.g. 149 for
   * 149/153/157/161); the RF tunes the 80 MHz centre (+6 ch = +30 MHz) and the
   * 40 MHz frame lands on the lower 40 of the block. */
  const bool is40 = (bwmode == CHANNEL_WIDTH_40);
  const bool is80 = (bwmode == CHANNEL_WIDTH_80);
  uint8_t central = channel;
  uint8_t pri = 0; /* 0 = 20 MHz / no sub-channel */
  if (is40) {
    if (channel_offset == 2) { central = channel - 2; pri = 1; } /* HT40- */
    else                     { central = channel + 2; pri = 2; } /* HT40+ */
  } else if (is80) {
    central = static_cast<uint8_t>(channel + 6); /* 80 MHz centre */
    /* pri-ch index for 80 MHz is a VHT_DATA_SC_* code (get_pri_ch_id), NOT a
     * plain 0-3 slot: primary = lowest 20 => VHT_DATA_SC_20_LOWEST_OF_80MHZ = 4.
     * This lands in 0x9b0[15:8]=0x44 and REG_DATA_SC 0x483=0xa4 (kernel-parity;
     * verified against a register dump of `iw 149 80MHz`). */
    pri = 4;
  }

  /* RF register write via the 8822C direct-write window:
   * BB[base + (rf_addr<<2)], 20-bit mask. base 0x3c00 (A) / 0x4c00 (B). */
  auto rf_write = [this](uint16_t base, uint8_t rfaddr, uint32_t v) {
    _device.phy_set_bb_reg(static_cast<uint16_t>(base + (rfaddr << 2)),
                           0x000fffff, v);
  };

  _last_channel = channel;

  /* --- bandwidth block (config_phydm_switch_bandwidth_8822c CH_20 / CH_40).
   * The 0x810/0x9b0[7:6]/0x9b4 clock+DFIR writes are the same for 20 and 40 MHz
   * (the vendor 40M case leaves them at the table/20M values); the RF-BW field
   * 0x9b0[3:0], pilot smoothing, CCK-source, CCK-PD and the primary-sub-channel
   * bits differ. --- */
  _device.phy_set_bb_reg(0x810, 0x3ff0, 0x19b);        /* RX DFIR */
  _device.phy_set_bb_reg(0x9b0, 0xc0, 0x0);            /* small-BW off */
  _device.phy_set_bb_reg(0x9b4, 0x00000700, 0x7);      /* DAC 480M */
  _device.phy_set_bb_reg(0x9b4, 0x00700000, 0x6);      /* ADC 160M */
  if (is80) {
    /* config_phydm_switch_bandwidth CH_80: TX/RX RF BW = 0b10/0b10 = 0xa,
     * pri-ch, pilot smoothing off, subtune 0x6. */
    _device.phy_set_bb_reg(0x9b0, 0xf, 0xa);           /* TX/RX RF BW = 80M */
    _device.phy_set_bb_reg(0x9b0, 0xff00,
                           static_cast<uint32_t>(pri | (pri << 4))); /* pri ch */
    _device.phy_set_bb_reg(0xcbc, 1u << 21, 0x1);      /* pilot smoothing off */
    _device.phy_set_bb_reg(0x88c, 0xf000, 0x6);        /* subtune 80M */
    /* 80 MHz decimation/DFIR (phydm 80M path, kernel-parity — matches a
     * register dump of `iw 149 80MHz`): RX DFIR 0x810[3:0]/[19:16]=7,
     * subtune-extra 0x88c[17:16]=3, decimation 0x1944/0x4044[9:8]=3, CCK
     * source [30]=0. */
    _device.phy_set_bb_reg(0x810, 0xf, 0x7);
    _device.phy_set_bb_reg(0x810, 0xf0000, 0x7);
    _device.phy_set_bb_reg(0x88c, 0x30000, 0x3);
    _device.phy_set_bb_reg(0x1944, 0x300, 0x3);
    _device.phy_set_bb_reg(0x4044, 0x300, 0x3);
    _device.phy_set_bb_reg(0x1abc, 1u << 30, 0x0);
  } else if (is40) {
    /* TX_RF_BW field 0x9b0[1:0] (RX_RF_BW [3:2]). Stock 40M = 0b01/0b01 = 0x5.
     * DEVOURER_TX_RF_BW overrides TX_RF_BW (debug/research knob): 0b10 uses the
     * 80M RF path (drains cleanly but shifts the emission -20 MHz), for studying
     * the kernel's 40 MHz-injection stall (libc0607/rtl88x2eu#7 — a kernel-side
     * issue devourer's own 40 MHz TX does not exhibit). */
    uint32_t rfbw_nib = 0x5;
    if (const char *e = std::getenv("DEVOURER_TX_RF_BW")) {
      uint32_t txbw = static_cast<uint32_t>(std::strtol(e, nullptr, 0)) & 0x3;
      rfbw_nib = txbw | (0x1 << 2); /* keep RX_RF_BW = 0b01 */
      _logger->info("Jaguar3: DEVOURER_TX_RF_BW override — 0x9b0[3:0]=0x{:x}",
                    rfbw_nib);
    }
    _device.phy_set_bb_reg(0x1a00, 1u << 4, pri == 1 ? 1 : 0); /* CCK pri ch */
    _device.phy_set_bb_reg(0x9b0, 0xf, rfbw_nib);      /* TX/RX RF BW = 40M */
    _device.phy_set_bb_reg(0x9b0, 0xff00,
                           static_cast<uint32_t>(pri | (pri << 4))); /* pri ch */
    _device.phy_set_bb_reg(0xcbc, 1u << 21, 0x1);      /* pilot smoothing off */
    _device.phy_set_bb_reg(0x1abc, 1u << 30, 0x1);     /* CCK source 5 */
    _device.phy_set_bb_reg(0x1ae8, 1u << 31, 0x0);     /* dynamic CCK PD */
    _device.phy_set_bb_reg(0x1aec, 0xf, 0x8);
    _device.phy_set_bb_reg(0x88c, 0xf000, 0x1);        /* subtune */
  } else {
    _device.phy_set_bb_reg(0x9b0, 0xff00, 0x0);        /* clear pri-ch (20M) */
    _device.phy_set_bb_reg(0x9b0, 0xf, 0x0);           /* RF BW = 20M */
    _device.phy_set_bb_reg(0xcbc, 1u << 21, 0x0);      /* pilot smoothing */
    _device.phy_set_bb_reg(0x1abc, 1u << 30, 0x0);     /* CCK source */
    _device.phy_set_bb_reg(0x1ae8, 1u << 31, 0x1);
    _device.phy_set_bb_reg(0x1aec, 0xf, 0x6);          /* dynamic CCK PD */
    _device.phy_set_bb_reg(0x88c, 0xf000, 0x1);        /* subtune */
  }

  /* --- channel tune (RF 0x18): tune the CENTRAL channel. clear
   * [18:17]/[16]/[9:8]/[7:0], set channel; for 5 GHz set BIT16|BIT8 ALWAYS plus
   * the sub-band bit (ch>144 -> BIT18, ch>=80 -> BIT17, else none). --- */
  const bool is_c = (_variant == ChipVariant::C8822C);
  const bool is_2g = central <= 14;
  /* RF 0x18 channel/band tune. 8822C is a read-modify-write preserving [15:10]
   * and setting the BW20 bits (config_phydm_switch_channel/bandwidth_8822c);
   * 8822E configures its RF band via its own phydm path (config_channel_8822e),
   * so it takes the plain write below. */
  /* RF bandwidth bits in RF18: BW20 = BIT13|BIT12, BW40 = BIT13, BW80 = BIT12
   * (config_phydm_switch_bandwidth). */
  const uint32_t rf_bw_bits = is80   ? (1u << 12)
                              : is40  ? (1u << 13)
                                      : ((1u << 13) | (1u << 12));
  uint32_t rf18;
  if (is_c) {
    /* Clear the channel/band bits AND the BW bits [13:12]: the vendor does
     * rf_reg18 &= ~(BIT13|BIT12) before setting them, and the RMW mask must
     * too — otherwise a prior 20 MHz tune's BIT12 survives and a 40 MHz tune
     * ends up BIT13|BIT12 = 20 MHz RF while BB/descriptor say 40 MHz, which
     * halts the TX engine (frames fill TXPKTBUF but never drain). */
    rf18 = (_device.rtw_read32(static_cast<uint16_t>(0x3c00 + (0x18 << 2))) &
            0xfffffu & ~0x733ffu);
    rf18 |= (central & 0xffu);
    rf18 |= rf_bw_bits;
  } else {
    rf18 = (central & 0xffu) | rf_bw_bits;
  }
  if (central > 14) { /* 5 GHz */
    rf18 |= (1u << 16) | (1u << 8);
    if (central > 144) rf18 |= (1u << 18);
    else if (central >= 80) rf18 |= (1u << 17);
  }
  _logger->info("Jaguar3: central ch {} (pri idx {}) RF18=0x{:05x}", central,
                pri, rf18);

  if (is_c) {
    /* RF writes are bracketed by phydm_rstb_3wire_8822c (HSSI/3-wire reset):
     * (false) 0x1c90[8]=0 before, (true) 0x1c90[8]=1 + 0x1830[29]/0x4130[29]=1
     * "force update anapar" after — the latter pushes the RF/analog shadow to
     * hardware, which is required for the per-channel RF (RXBB/RF18) to reach the
     * analog front-end. RXBB (RF 0x3f) is a gated write: RF 0xee[2]=1,
     * RF 0x33[4:0]=0x12, RF 0x3f, RF 0xee[2]=0 (config_phydm_switch_bandwidth_8822c). */
    /* RXBB (RF 0x3f): BW20 = BIT4|BIT3 (0x18), BW40 = BIT4 (0x10),
     * BW80 = BIT3 (0x08) (config_phydm_switch_bandwidth_8822c). */
    const uint32_t rxbb = is80 ? 0x08u : is40 ? 0x10u : 0x18u;
    _device.phy_set_bb_reg(0x1c90, 1u << 8, 0x0); /* rstb_3wire(false) */
    for (uint16_t base : {uint16_t(0x3c00), uint16_t(0x4c00)}) {
      _device.phy_set_bb_reg(static_cast<uint16_t>(base + (0xee << 2)), 0x4, 0x1);
      _device.phy_set_bb_reg(static_cast<uint16_t>(base + (0x33 << 2)), 0x1f, 0x12);
      rf_write(base, 0x3f, rxbb);
      _device.phy_set_bb_reg(static_cast<uint16_t>(base + (0xee << 2)), 0x4, 0x0);
    }
    rf_write(0x3c00, 0x18, rf18);
    rf_write(0x4c00, 0x18, rf18);
    _device.phy_set_bb_reg(static_cast<uint16_t>(0x3c00 + (0xdf << 2)), 1u << 18,
                           is_2g ? 1 : 0);
    _device.phy_set_bb_reg(0x1c90, 1u << 8, 0x1);   /* rstb_3wire(true) */
    _device.phy_set_bb_reg(0x1830, 1u << 29, 0x1);  /* force update anapar (A) */
    _device.phy_set_bb_reg(0x4130, 1u << 29, 0x1);  /* force update anapar (B) */
  } else {
    rf_write(0x3c00, 0x18, rf18);
    rf_write(0x4c00, 0x18, rf18);
    if (is40 || is80) {
      /* 8822e switch_bandwidth CH_40/CH_80: RF 0x1a TXBB/RXBB bits [12:10]
       * (20M is BIT11|BIT10) via RMW — 40M = BIT12|BIT11, 80M = BIT13|BIT10;
       * plus the TX_CCK_IND workaround (RF 0x1a BIT0=1, BIT16=0) at 40M only. */
      const uint32_t bb = is80 ? ((1u << 13) | (1u << 10))
                               : ((1u << 12) | (1u << 11));
      for (uint16_t base : {uint16_t(0x3c00), uint16_t(0x4c00)}) {
        uint32_t r1a = _device.rtw_read32(
            static_cast<uint16_t>(base + (0x1a << 2))) & 0xfffffu;
        r1a &= ~0x7c00u;
        r1a |= bb;
        if (is40) { r1a |= (1u << 0); r1a &= ~(1u << 16); }
        rf_write(base, 0x1a, r1a);
      }
    } else {
      rf_write(0x3c00, 0x3f, 0x18);  /* RXBB 20 */
      rf_write(0x4c00, 0x3f, 0x18);
    }
    _device.phy_set_bb_reg(static_cast<uint16_t>(0x3c00 + (0xdf << 2)), 1u << 18,
                           is_2g ? 1 : 0);
  }

  /* Per-band RX AGC-table selection (phydm_cck/ofdm_agc_tab_sel_8822c) — 8822C
   * only; the EU (8822e) selects its AGC table via its own phydm path. 0x18ac/
   * 0x41ac [15:12]=CCK table, [8:4]=OFDM table; 0x828[7:3]=AGC lower bound
   * (L_BND_DEFAULT_8822C 0xd; the measured ofdm_rxagc_l_bnd is not tracked). */
  auto ofdm_agc_sel = [this](uint8_t table) {
    _device.phy_set_bb_reg(0x18ac, 0x1f0, table);
    _device.phy_set_bb_reg(0x41ac, 0x1f0, table);
    _device.phy_set_bb_reg(0x828, 0xf8, 0xd);
  };
  auto cck_agc_sel = [this](uint8_t table) {
    _device.phy_set_bb_reg(0x18ac, 0xf000, table);
    _device.phy_set_bb_reg(0x41ac, 0xf000, table);
  };
  const bool bw20 = (bwmode == CHANNEL_WIDTH_20 || bwmode == CHANNEL_WIDTH_10 ||
                     bwmode == CHANNEL_WIDTH_5);
  if (!is_c) {
    /* 8822E: skip 8822c AGC-table select entirely. */
  } else if (central <= 14) {
    cck_agc_sel(bw20 ? 5 : 4);              /* CCK_BW20=5 / CCK_BW40=4 */
    ofdm_agc_sel(bw20 ? 6 : 0);            /* OFDM_2G_BW20=6 / OFDM_2G_BW40=0 */
  } else if (central < 80) {
    ofdm_agc_sel(1);                        /* OFDM_5G_LOW */
  } else if (central <= 144) {
    ofdm_agc_sel(2);                        /* OFDM_5G_MID */
  } else {
    ofdm_agc_sel(3);                        /* OFDM_5G_HIGH */
  }

  /* --- 2G/5G band BB (phydm switch_channel "Other BB Settings") --- */
  if (is_2g) {
    /* Enable CCK Rx IQ (phydm_cck_rxiq_8822c SET): CCK source 5 + weighting [1,1].
     * 8822C only (EU has its own CCK-RXIQ path). */
    if (is_c) {
      _device.phy_set_bb_reg(0x1a9c, 1u << 20, 0x1);
      _device.phy_set_bb_reg(0x1a14, 0x300, 0x0);
    }
    _device.rtw_write8(0x454,
                       static_cast<uint8_t>(_device.rtw_read8(0x454) & ~0x80));
    _device.phy_set_bb_reg(0x1a80, 1u << 18, 0x0);
    _device.phy_set_bb_reg(0x1c80, 0x3F000000, 0xF);
  } else {
    /* Disable CCK Rx IQ (phydm_cck_rxiq_8822c REVERT). 8822C only. */
    if (is_c) {
      _device.phy_set_bb_reg(0x1a9c, 1u << 20, 0x0);
      _device.phy_set_bb_reg(0x1a14, 0x300, 0x3);
    }
    _device.rtw_write8(0x454,
                       static_cast<uint8_t>(_device.rtw_read8(0x454) | 0x80));
    _device.phy_set_bb_reg(0x1a80, 1u << 18, 0x1);
    _device.phy_set_bb_reg(0x1c80, 0x3F000000, 0x22);
  }

  /* SCO tracking f_c (phydm_sco_trk_fc_setting_8822c): BB 0xc30[11:0]. Keyed
   * on the central channel. */
  uint32_t sco;
  if (central >= 36 && central <= 51) sco = 0x494;
  else if (central >= 52 && central <= 55) sco = 0x493;
  else if (central >= 56 && central <= 111) sco = 0x453;
  else if (central >= 112 && central <= 119) sco = 0x452;
  else if (central >= 120 && central <= 172) sco = 0x412;
  else if (central >= 173) sco = 0x411;
  else if (central >= 1 && central <= 10) sco = 0x9aa;
  else if (central == 11 || central == 12) sco = 0x96a;
  else sco = 0x969; /* 13/14 */
  _device.phy_set_bb_reg(0xc30, 0xfff, sco);

  /* TX DFIR (phydm_tx_dfir_setting_8822c): BB 0x808[22:20] + [6:4]. */
  if (is_2g) {
    _device.phy_set_bb_reg(0x808, 0x700000, central == 11 ? 0x3 : 0x1);
    _device.phy_set_bb_reg(0x808, 0x70, central == 13 ? 0x3 : 0x1);
  } else {
    _device.phy_set_bb_reg(0x808, 0x700000, 0x1);
    _device.phy_set_bb_reg(0x808, 0x70, 0x3);
  }

  /* MAC-side bandwidth + TX sub-channel (halmac cfg_bw / cfg_pri_ch_idx). */
  set_mac_bw_txsc(bwmode, pri);

  /* phydm_bb_reset_8822c: toggle the BB reset (MAC reg 0x0 BIT16, 1->0->1) to
   * (re)start the receiver after channel/BW config — the kernel does this after
   * every switch_channel; without it the RX engine never runs. */
  uint32_t r0 = _device.rtw_read32(0x0);
  _device.rtw_write32(0x0, r0 | (1u << 16));
  _device.rtw_write32(0x0, r0 & ~(1u << 16));
  _device.rtw_write32(0x0, r0 | (1u << 16));

  /* halrf_ex_dac_fifo_rst — the vendor runs this after EVERY switch_bandwidth
   * ("fix dac fifo error after TXCK setting"): 40/80 MHz change the TX clock,
   * so reset the DAC FIFO for vendor parity. 8822e only (its AFE DACK-bank soft
   * reset); the 8822c has its own path. */
  if ((is40 || is80) && _variant == ChipVariant::C8822E)
    dack_soft_rst_8822e();

  _logger->info("Jaguar3: central ch {} / {} MHz set (RF18=0x{:05x}) + BB reset",
                central, is80 ? 80 : is40 ? 40 : 20, rf18);

  /* For 5/10 MHz: channel is now tuned at 20 MHz; re-clock the baseband. */
  if (bwmode == CHANNEL_WIDTH_5 || bwmode == CHANNEL_WIDTH_10)
    set_bandwidth_dividers(bwmode);
}

void RadioManagementJaguar3::set_mac_bw_txsc(ChannelWidth_t bw, uint8_t pri) {
  /* halmac cfg_bw_88xx: REG_WMAC_TRXPTCL_CTL 0x668, clear BIT7|BIT8, set BIT7
   * for 40 MHz, BIT8 for 80 MHz. */
  uint32_t ctl = _device.rtw_read32(0x668) & ~((1u << 7) | (1u << 8));
  if (bw == CHANNEL_WIDTH_40)      ctl |= (1u << 7);
  else if (bw == CHANNEL_WIDTH_80) ctl |= (1u << 8);
  _device.rtw_write32(0x668, ctl);

  /* halmac cfg_pri_ch_idx_88xx: REG_DATA_SC 0x483 = txsc20 | (txsc40 << 4).
   * txsc20 = primary-sub-channel index; txsc40 = 9 if idx is 1 or 3 else 10.
   * pri=0 (primary = lowest 20) gives txsc40=10; 20 MHz clears it. */
  uint8_t sc = 0;
  if (bw == CHANNEL_WIDTH_40 || bw == CHANNEL_WIDTH_80) {
    uint8_t txsc40 = (pri == 1 || pri == 3) ? 9 : 10;
    sc = static_cast<uint8_t>((pri & 0xf) | (txsc40 << 4));
  }
  _device.rtw_write8(0x483, sc);
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

void RadioManagementJaguar3::apply_power_by_rate_8822e(
    uint8_t channel, uint8_t ref_a, uint8_t ref_b, bool skip_path_b_ofdm_ref) {
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
  if (!skip_path_b_ofdm_ref)
    wr(0x41e8, 0x1fc00, ref_b);   /* path B — RX-desense hazard, see header */
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
   * _8822c / _8822e, on top of an already-tuned channel (this re-clocks the
   * baseband without re-running the RF channel tune). 10 MHz halves the DAC/ADC
   * clock, 5 MHz quarters it; radiotap stays 20 MHz so only an SDR sees the
   * change. The DAC-divider codes differ per variant for the SAME clock — see
   * the encoding table in the header. */
  const bool is_e = (_variant == ChipVariant::C8822E);
  /* One DAC-code table for BOTH variants — the 8822c values. The 8822e
   * vendor phydm ships different codes (5M=0x2/10M=0x4/20M=0x6, commented as
   * the same clocks) but 0x2 is DEAD on real 8812EU silicon (TX keys up,
   * frames drain, nothing airs — SDR-bisected via DEVOURER_NB_DAC sweep:
   * only 0x4 and 0x6 emit the 5 MHz lobe). libc0607 hit the same wall and
   * patched the vendor table to these values ("Fix DAC clock setting for
   * 5MHz BW, from 8812cu driver"), which the OpenHD ecosystem has been
   * running at 10 MHz on 8812EU since. */
  uint8_t small_bw, dac, adc;
  uint16_t dfir;
  switch (bwmode) {
  case CHANNEL_WIDTH_10:
    small_bw = NB_SMALLBW_10M; dac = 0x6; adc = 0x5; dfir = 0x2ab; break;
  case CHANNEL_WIDTH_5:
    small_bw = NB_SMALLBW_5M;  dac = 0x4; adc = 0x4; dfir = 0x2ab; break;
  default: /* CHANNEL_WIDTH_20 — restore full-rate clocks */
    small_bw = 0x0;            dac = 0x7; adc = 0x6; dfir = 0x19b; break;
  }
  /* DEVOURER_NB_DAC=0xN — debug knob: force the DAC-divider code. For mirror
   * A/B experiments and for empirically mapping the (undocumented) divider
   * field encoding; not for normal use. */
  const char *dac_env = std::getenv("DEVOURER_NB_DAC");
  if (dac_env != nullptr && *dac_env != '\0') {
    dac = static_cast<uint8_t>(std::strtol(dac_env, nullptr, 0) & 0x7);
    _logger->info("Jaguar3: DEVOURER_NB_DAC override — DAC code {:#x}", dac);
  }
  _device.phy_set_bb_reg(R_RX_DFIR_8822C, 0x3ff0, dfir);
  /* 8822e vendor parity: the small-BW write also zeroes the TX/RX pri-ch
   * fields [15:8] (mask 0xffc0); the SDR-validated 8822c path keeps its
   * original [7:6]-only write. */
  _device.phy_set_bb_reg(R_SMALL_BW_8822C, is_e ? 0xffc0 : 0xc0, small_bw);
  _device.phy_set_bb_reg(R_CLK_DIV_8822C, 0x00000700, dac);   /* 0x9b4[10:8] */
  _device.phy_set_bb_reg(R_CLK_DIV_8822C, 0x00700000, adc);   /* 0x9b4[22:20] */

  if (is_e) {
    /* halmac cfg_mac_clk_88xx — the MAC-side half of the narrowband re-clock:
     * MAC clock select at REG_AFE_CTRL1 0x24[21:20] (0=80M, 2=20M, 3=the
     * dedicated 20M_BW_5 mode) + the TSF/EDCA microsecond-tick clocks
     * (0x55c/0x638, in MHz). The vendor applies this for BW5/BW10 on every
     * Jaguar3; the SDR-validated 8822c NB path works without it, so it is
     * applied on the 8822e only, where 5 MHz TX never reaches the air without
     * the 20M_BW_5 MAC mode. */
    uint8_t clk_sel, ustime;
    if (bwmode == CHANNEL_WIDTH_5)       { clk_sel = 3; ustime = 20; }
    else if (bwmode == CHANNEL_WIDTH_10) { clk_sel = 2; ustime = 20; }
    else                                 { clk_sel = 0; ustime = 80; }
    uint32_t afe = _device.rtw_read32(0x24);
    afe = (afe & ~((1u << 20) | (1u << 21))) |
          (static_cast<uint32_t>(clk_sel) << 20);
    _device.rtw_write32(0x24, afe);
    _device.rtw_write8(0x55c, ustime);
    _device.rtw_write8(0x638, ustime);

    /* config_phydm_switch_bandwidth_8822e narrowband extras the 8822c recipe
     * doesn't have: CFR (crest-factor reduction) params + TX triangular-shape
     * spectrum shaping OFF for 5/10 MHz; the 20 MHz restore re-applies the
     * band-dependent shaping (phydm_tx_triangular_shap_cfg_8822e). */
    if (bwmode == CHANNEL_WIDTH_5 || bwmode == CHANNEL_WIDTH_10) {
      _device.phy_set_bb_reg(0xa74, 1u << 31, 0x0);
      _device.phy_set_bb_reg(0xa74, 0x3ff, 0x15);
      _device.phy_set_bb_reg(0xa74, 0xffc00, 0x13);
      _device.phy_set_bb_reg(0x808, 0x70, 0x1);
      _device.phy_set_bb_reg(0x80c, 0xf, 0x5);
      _device.phy_set_bb_reg(0x81c, 0xff, 0x0);
      _device.phy_set_bb_reg(0x81c, 0xf000000, 0x0);
      _device.phy_set_bb_reg(0x8a0, 0xf0000000, 0x0);
    } else {
      const bool is_2g = (_last_channel != 0 && _last_channel <= 14);
      _device.phy_set_bb_reg(0xa74, 1u << 31, 0x1);
      _device.phy_set_bb_reg(0x808, 0x70, 0x3);
      if (is_2g) {
        _device.phy_set_bb_reg(0xa74, 0x3ff, 0x15);
        _device.phy_set_bb_reg(0xa74, 0xffc00, 0x13);
        _device.phy_set_bb_reg(0x80c, 0xf, 0x5);
        _device.phy_set_bb_reg(0x81c, 0xff, 0xff);
        _device.phy_set_bb_reg(0x81c, 0xf000000, 0x0);
        _device.phy_set_bb_reg(0x8a0, 0xf0000000, 0xb);
      } else {
        _device.phy_set_bb_reg(0xa74, 0x3ff, 0x3f);
        _device.phy_set_bb_reg(0xa74, 0xffc00, 0x3f);
        _device.phy_set_bb_reg(0x80c, 0xf, 0x8);
        _device.phy_set_bb_reg(0x81c, 0xff, 0x55);
        _device.phy_set_bb_reg(0x81c, 0xf000000, 0x7);
        _device.phy_set_bb_reg(0x8a0, 0xf0000000, 0x0);
      }
    }
  }

  /* phydm_bb_reset: toggle MAC 0x0 BIT16 (1->0->1) so the receiver/DFE
   * relatches at the new sample rate — without it the re-clock doesn't take. */
  uint32_t r0 = _device.rtw_read32(0x0);
  _device.rtw_write32(0x0, r0 | (1u << 16));
  _device.rtw_write32(0x0, r0 & ~(1u << 16));
  _device.rtw_write32(0x0, r0 | (1u << 16));

  if (is_e) {
    /* phydm_igi_toggle_8822e: bump IGI down/up so the BB HW issues 3-wire and
     * the RF re-enters RX — the BB does not send 3-wire automatically on a
     * path/channel/BW config change. */
    uint32_t igi = _device.rtw_read32(0x1d70);
    _device.rtw_write32(0x1d70, igi - 0x202);
    _device.rtw_write32(0x1d70, igi);

    /* halrf_ex_dac_fifo_rst — "fix dac fifo error after TXCK setting": the
     * DAC FIFO can come out of a TX-clock change misaligned and emit spectral
     * images (the OpenHD 5 MHz "mirror"); soft-reset the AFE DACK banks. */
    dack_soft_rst_8822e();
  }

  const char *name = bwmode == CHANNEL_WIDTH_10  ? "10 MHz"
                     : bwmode == CHANNEL_WIDTH_5 ? "5 MHz"
                                                 : "20 MHz";
  _logger->info("Jaguar3: baseband re-clocked to {} (small_bw={} DAC={} ADC={} "
                "DFIR=0x{:03x})", name, small_bw, dac, adc, dfir);
}

/* halrf_write_check_afe_8822e: an AFE write must be confirmed (the bank reads
 * back non-zero); retry up to 100x past an IO race. Faithful duplicate of
 * Halrf8822e::write_check_afe (see the header note on the shared-base
 * follow-up). */
void RadioManagementJaguar3::write_check_afe_8822e(uint16_t add, uint32_t data) {
  const uint32_t wd = (add == 0x3800 || add == 0x3900) ? data : 0xee32001fu;
  const uint16_t wa = ((add >> 8) == 0x38) ? 0x3800 : 0x3900;
  for (uint32_t count = 0; count < 100; ++count) {
    _device.rtw_write32(0x2dd4, 0x0);
    _device.rtw_write32(add, data);
    _device.rtw_write32(add, data);
    _device.rtw_write32(0x2dd4, 0x0);
    if (_device.rtw_read32(wa) != 0x0)
      return;
    _device.rtw_write32(wa, wd);
    _device.rtw_write32(wa, wd);
  }
}

/* halrf_dack_soft_rst_8822e — soft-reset all four AFE DACK banks (S0/S1 x I/Q). */
void RadioManagementJaguar3::dack_soft_rst_8822e() {
  write_check_afe_8822e(0x3800, 0xee30001fu);
  write_check_afe_8822e(0x3800, 0xee32001fu);
  write_check_afe_8822e(0x382c, 0xee30001fu);
  write_check_afe_8822e(0x382c, 0xee32001fu);
  write_check_afe_8822e(0x3900, 0xee30001fu);
  write_check_afe_8822e(0x3900, 0xee32001fu);
  write_check_afe_8822e(0x392c, 0xee30001fu);
  write_check_afe_8822e(0x392c, 0xee32001fu);
}

} /* namespace jaguar3 */
