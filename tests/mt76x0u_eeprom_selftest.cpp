/* Headless guard for the MT76x0U calibration decode and init tables
 * (src/mt76x0u/eeprom.cpp, phy.cpp, initvals.h) - everything in phase 3 that
 * is a pure function of bytes:
 *
 *  - mt0_sign_extend: mt76x02's sign-and-MAGNITUDE encoding, sign bit set
 *    meaning positive (an easy thing to "fix" into two's complement);
 *  - mt0_ee_lna_gain: wifit3's test vectors for the per-channel LNA gain
 *    (tests/chips/mt76x0u/test_eeprom_lna_gain.py), the fix without which
 *    wifit3 measured 5 GHz RX at ~36% of a good card;
 *  - mt0_ee_parse: board type, the no_2ghz and MT7630 band masks, the
 *    frequency and temperature offsets with their invalid-byte fallbacks;
 *  - mt0_rf_patch_value: the USB arms of mt76x0_rf_patch_reg_array;
 *  - initvals.h spot values against the kernel source. */
#include <cstdio>
#include <cstring>

#include "internal.h"
#include "phy.h"
#include "initvals.h"

namespace {

int fails;

void expect(const char *what, bool ok) {
  if (!ok) {
    std::fprintf(stderr, "mt76x0u_eeprom: FAIL %s\n", what);
    fails++;
  }
}

void put16(uint8_t *ee, unsigned off, uint16_t v) {
  ee[off] = (uint8_t)v;
  ee[off + 1] = (uint8_t)(v >> 8);
}

/* wifit3's _cache(): the four LNA bytes. */
void lna(uint8_t *ee, uint8_t lna_2g, uint8_t lna_5g0, uint8_t lna_5g1,
         uint8_t lna_5g2) {
  std::memset(ee, 0, MT76X0_EEPROM_SIZE);
  ee[MT_EE_LNA_GAIN] = lna_2g;
  ee[MT_EE_LNA_GAIN + 1] = lna_5g0;
  ee[MT_EE_RSSI_OFFSET_2G_1 + 1] = lna_5g1;
  ee[MT_EE_RSSI_OFFSET_5G_1 + 1] = lna_5g2;
}

} // namespace

int main() {
  uint8_t ee[MT76X0_EEPROM_SIZE];

  /* --- sign extension --------------------------------------------------- */
  expect("sign bit set is positive", mt0_sign_extend(0x85, 8) == 5);
  expect("sign bit clear is negative", mt0_sign_extend(0x05, 8) == -5);
  expect("magnitude, not two's complement", mt0_sign_extend(0xff, 8) == 127);
  expect("negative zero is zero", mt0_sign_extend(0x00, 8) == 0);

  /* --- LNA gain: wifit3's vectors --------------------------------------- */
  lna(ee, 4, 8, 10, 12);
  expect("2.4 GHz uses lna_2g", mt0_ee_lna_gain(ee, 6) == 4);
  expect("ch36 (<=64) uses lna_5g[0]", mt0_ee_lna_gain(ee, 36) == 8);
  expect("ch100 (<=128) uses lna_5g[1]", mt0_ee_lna_gain(ee, 100) == 10);
  expect("ch157 (>128) uses lna_5g[2]", mt0_ee_lna_gain(ee, 157) == 12);
  lna(ee, 0, 9, 0x00, 0xff);
  expect("sub-band 1 of 0 falls back to lna_5g[0]", mt0_ee_lna_gain(ee, 100) == 9);
  expect("sub-band 2 of 0xff falls back to lna_5g[0]", mt0_ee_lna_gain(ee, 157) == 9);
  lna(ee, 0xff, 0xff, 0, 0);
  expect("selected 0xff yields 0 (2.4)", mt0_ee_lna_gain(ee, 6) == 0);
  expect("selected 0xff yields 0 (5)", mt0_ee_lna_gain(ee, 36) == 0);
  lna(ee, 0xfe, 0, 0, 0);
  expect("high-bit byte is s8 -2", mt0_ee_lna_gain(ee, 6) == -2);
  lna(ee, 0x80, 0, 0, 0);
  expect("high-bit byte is s8 -128", mt0_ee_lna_gain(ee, 6) == -128);
  lna(ee, 0, 1, 2, 3);
  expect("sub-band edges: 64 -> [0]", mt0_ee_lna_gain(ee, 64) == 1);
  expect("sub-band edges: 128 -> [1]", mt0_ee_lna_gain(ee, 128) == 2);
  expect("sub-band edges: 132 -> [2]", mt0_ee_lna_gain(ee, 132) == 3);

  /* --- RSSI offset clamp ------------------------------------------------ */
  {
    int8_t off[2];
    std::memset(ee, 0, sizeof ee);
    put16(ee, MT_EE_RSSI_OFFSET_5G_0, (uint16_t)((uint8_t)-3 | (11 << 8)));
    mt0_ee_rssi_offset(ee, 36, off);
    expect("RSSI offset within +-10 kept, beyond zeroed", off[0] == -3 && off[1] == 0);
  }

  /* --- parse ----------------------------------------------------------- */
  {
    struct mt76x0u_dev d{};
    std::memset(d.eeprom, 0, sizeof d.eeprom);
    put16(d.eeprom, MT_EE_VERSION, 0x0c01);
    static const uint8_t mac[6] = {0x00, 0x0e, 0x8d, 0x76, 0x10, 0x01};
    std::memcpy(d.eeprom + MT_EE_MAC_ADDR, mac, 6);
    put16(d.eeprom, MT_EE_NIC_CONF_0, 0x0011); /* board type 0: both bands */
    put16(d.eeprom, MT_EE_FREQ_OFFSET, 0x0040);
    put16(d.eeprom, MT_EE_TSSI_BOUND4, 0x8300); /* +3 */
    put16(d.eeprom, MT_EE_2G_TARGET_POWER, 0x8500); /* +5 */
    mt0_ee_parse(&d);
    expect("version and fae", d.ee_version == 0x0c && d.ee_fae == 0x01);
    expect("MAC from 0x004", std::memcmp(d.macaddr, mac, 6) == 0);
    expect("board type 0 is dual band", d.has_2ghz && d.has_5ghz);
    expect("freq offset 0x40 - (+3)", d.freq_offset == 0x3d);
    expect("temp offset +5", d.temp_offset == 5);

    put16(d.eeprom, MT_EE_FREQ_OFFSET, 0x00ff);
    put16(d.eeprom, MT_EE_TSSI_BOUND4, 0xff00);
    put16(d.eeprom, MT_EE_2G_TARGET_POWER, 0xff00);
    mt0_ee_parse(&d);
    expect("invalid freq bytes give 0", d.freq_offset == 0);
    expect("invalid temp byte gives -10", d.temp_offset == -10);

    put16(d.eeprom, MT_EE_NIC_CONF_0, 2u << 12); /* BOARD_TYPE_5GHZ */
    mt0_ee_parse(&d);
    expect("board type 5 GHz only", !d.has_2ghz && d.has_5ghz);
    put16(d.eeprom, MT_EE_NIC_CONF_0, 0);
    d.no_2ghz = 1;
    mt0_ee_parse(&d);
    expect("USB-id quirk masks 2.4 GHz", !d.has_2ghz && d.has_5ghz);
    d.no_2ghz = 0;
    d.is_mt7630 = 1;
    mt0_ee_parse(&d);
    expect("MT7630 strap masks 5 GHz", d.has_2ghz && !d.has_5ghz);
  }

  /* --- RF patch overrides (USB) ----------------------------------------- */
  expect("RF(0,3) -> 0x73 on USB", mt0_rf_patch_value(MT_RF(0, 3), 0x11, 0) == 0x73);
  expect("RF(0,21) -> 0x12 (is_mt7610e is MMIO-only)",
         mt0_rf_patch_value(MT_RF(0, 21), 0x11, 0) == 0x12);
  expect("RF(5,2) -> 0x0c", mt0_rf_patch_value(MT_RF(5, 2), 0x11, 0) == 0x0c);
  expect("RF(5,2) -> 0x1d on MT7630", mt0_rf_patch_value(MT_RF(5, 2), 0x11, 1) == 0x1d);
  expect("other registers pass through", mt0_rf_patch_value(MT_RF(0, 4), 0x30, 1) == 0x30);

  /* --- generated tables, spot values from the kernel source -------------- */
  expect("MT_BCN_OFFSET(0) = 0x41c", common_mac_reg_table[0].reg == 0x41c &&
                                         common_mac_reg_table[0].value == 0xf8f0e8e0);
  expect("MT_MAX_LEN_CFG = 0xa0fff | 0x1000",
         common_mac_reg_table[11].reg == 0x1018 &&
             common_mac_reg_table[11].value == 0xa1fff);
  expect("MT_BBP(CORE, 1) first in the BBP table",
         mt76x0_bbp_init_tab[0].reg == 0x2004 && mt76x0_bbp_init_tab[0].value == 2);
  expect("bbp switch: AGC4 G band 20/40",
         mt76x0_bbp_switch_tab[0].bw_band == (RF_G_BAND | RF_BW_20 | RF_BW_40) &&
             mt76x0_bbp_switch_tab[0].reg_pair.reg == 0x2310);
  expect("RF central starts at MT_RF(0, 1)", mt76x0_rf_central_tab[0].reg == MT_RF(0, 1));
  expect("frequency plan starts at channel 1, G band",
         mt76x0_frequency_plan[0].channel == 1 &&
             mt76x0_frequency_plan[0].band == RF_G_BAND);
  expect("125-entry frequency plans", sizeof mt76x0_frequency_plan /
                                              sizeof mt76x0_frequency_plan[0] == 125);
  expect("26 SDM channels, first 183",
         sizeof mt76x0_sdm_channel == 26 && mt76x0_sdm_channel[0] == 183);

  if (fails) {
    std::fprintf(stderr, "mt76x0u_eeprom: %d failure(s)\n", fails);
    return 1;
  }
  std::printf("mt76x0u_eeprom: ok\n");
  return 0;
}
