#include <array>
#include <cstdio>

#include "rtl8733b/Halmac8733bMac.h"

namespace {
int failures = 0;

void expect(const char *what, bool condition) {
  if (condition)
    return;
  ++failures;
  std::printf("FAIL: %s\n", what);
}
} // namespace

int main() {
  std::array<uint8_t, 32> physical;
  std::array<uint8_t, rtl8733b::kLogicalEfuseSize> logical;
  physical.fill(0xff);

  /* Normal header: block 0, words 0 and 1 present. */
  physical[0] = 0x0c;
  physical[1] = 0x29;
  physical[2] = 0x81;
  physical[3] = 0x34;
  physical[4] = 0x12;
  /* Extended header: block 0x21 -> logical base 0x108, word 0 present. */
  physical[5] = 0x2f;
  physical[6] = 0x4e;
  physical[7] = 0x00;
  physical[8] = 0xe0;
  physical[9] = 0xff;

  size_t used = 0;
  expect("valid packed map",
         rtl8733b::Halmac8733bMac::parse_physical_efuse(
             physical.data(), physical.size(), logical.data(), logical.size(),
             &used));
  expect("normal word 0", logical[0] == 0x29 && logical[1] == 0x81);
  expect("normal word 1", logical[2] == 0x34 && logical[3] == 0x12);
  expect("disabled normal word remains ff",
         logical[4] == 0xff && logical[5] == 0xff);
  expect("extended block", logical[0x108] == 0x00 && logical[0x109] == 0xe0);
  expect("used byte count", used == 9);

  /* Truncated data and a logical target outside the supplied map must fail. */
  const uint8_t truncated[] = {0x0e, 0x29};
  expect("truncated word rejected",
         !rtl8733b::Halmac8733bMac::parse_physical_efuse(
             truncated, sizeof(truncated), logical.data(), logical.size()));
  std::array<uint8_t, 16> small_logical;
  const uint8_t high_block[] = {0x2f, 0x4e, 0x00, 0xe0, 0xff};
  expect("out-of-range logical block rejected",
         !rtl8733b::Halmac8733bMac::parse_physical_efuse(
             high_block, sizeof(high_block), small_logical.data(),
             small_logical.size()));

  /* A fully-written map has no 0xff terminator left to find. Running off the
   * end of the readable span is address-space exhaustion, which the vendor
   * walkers treat as success-with-whatever-parsed — a heavily reprogrammed but
   * valid EFUSE must not fail bring-up. */
  const uint8_t exhausted[] = {0x0c, 0x29, 0x81, 0x34, 0x12};
  size_t exhausted_used = 0;
  expect("exhausted map accepted",
         rtl8733b::Halmac8733bMac::parse_physical_efuse(
             exhausted, sizeof(exhausted), logical.data(), logical.size(),
             &exhausted_used));
  expect("exhausted map reports full span", exhausted_used == sizeof(exhausted));
  expect("exhausted map decoded its words",
         logical[0] == 0x29 && logical[1] == 0x81 && logical[2] == 0x34 &&
             logical[3] == 0x12);

  logical.fill(0xff);
  for (size_t path = 0; path < 2; ++path) {
    const size_t base = rtl8733b::kTxPowerEfuseOffset8733b + path * 42;
    for (size_t i = 0; i < 6; ++i)
      logical[base + i] = static_cast<uint8_t>(0x20 + path * 8 + i);
    for (size_t i = 0; i < 5; ++i)
      logical[base + 6 + i] = static_cast<uint8_t>(0x30 + path * 8 + i);
    logical[base + 11] = path == 0 ? 0xe3 : 0x7a;
    for (size_t i = 0; i < 14; ++i)
      logical[base + 18 + i] = static_cast<uint8_t>(0x40 + path * 8 + i);
    logical[base + 18 + 14] = path == 0 ? 0x7a : 0xe3;
  }
  expect("direct-index power modes decoded",
         rtl8733b::Halmac8733bMac::tx_power_pg_mode(0x00) ==
                 rtl8733b::TxPowerPgMode8733b::DirectIndex &&
             rtl8733b::Halmac8733bMac::tx_power_pg_mode(0x3f) ==
                 rtl8733b::TxPowerPgMode8733b::DirectIndex);
  expect("TSSI-offset power modes decoded",
         rtl8733b::Halmac8733bMac::tx_power_pg_mode(0x40) ==
                 rtl8733b::TxPowerPgMode8733b::TssiOffset &&
             rtl8733b::Halmac8733bMac::tx_power_pg_mode(0x7f) ==
                 rtl8733b::TxPowerPgMode8733b::TssiOffset);
  expect("reserved power modes rejected",
         rtl8733b::Halmac8733bMac::tx_power_pg_mode(0x80) ==
                 rtl8733b::TxPowerPgMode8733b::Unknown &&
             rtl8733b::Halmac8733bMac::tx_power_pg_mode(0xff) ==
                 rtl8733b::TxPowerPgMode8733b::Unknown);

  rtl8733b::DirectTxPowerInfo8733b tx_power;
  expect("direct TX-power layout parses",
         rtl8733b::Halmac8733bMac::parse_direct_tx_power_efuse(
             logical.data(), logical.size(), tx_power));
  expect("required RTL8733B power paths usable", tx_power.usable());
  expect("2G bases decoded",
         tx_power.path[0].cck_base_2g[2] == 0x22 &&
             tx_power.path[0].bw40_base_2g[4] == 0x34);
  expect("2G signed diffs scaled to TXGI steps",
         tx_power.path[0].bw20_diff_2g == -4 &&
             tx_power.path[0].ofdm_diff_2g == 6);
  expect("5G signed diffs scaled to TXGI steps",
         tx_power.path[0].bw20_diff_5g == 14 &&
             tx_power.path[0].ofdm_diff_5g == -12);
  expect("second-path layout offset",
         tx_power.path[1].cck_base_2g[0] == 0x28 &&
             tx_power.path[1].bw40_base_5g[13] == 0x55);
  logical[rtl8733b::kTxPowerEfuseOffset8733b + 6] = 0xff;
  expect("invalid 2G base is reported",
         rtl8733b::Halmac8733bMac::parse_direct_tx_power_efuse(
             logical.data(), logical.size(), tx_power) &&
             !tx_power.path[0].valid_2g && !tx_power.usable());
  expect("truncated direct TX-power layout rejected",
         !rtl8733b::Halmac8733bMac::parse_direct_tx_power_efuse(
             logical.data(), 32, tx_power));

  std::array<uint8_t, rtl8733b::kPhysicalEfuseSize> physical_full;
  logical.fill(0xff);
  physical_full.fill(0xff);
  logical[0x10] = 3;
  logical[0x18] = static_cast<uint8_t>(-4);
  logical[0x3a] = 5;
  logical[0x42] = static_cast<uint8_t>(-6);
  physical_full[0x1dd] = 7;
  physical_full[0x1dc] = static_cast<uint8_t>(-8);
  physical_full[0x1d4] = 9;
  rtl8733b::TssiPowerInfo8733b tssi;
  expect("TSSI logical and physical layouts parse",
         rtl8733b::Halmac8733bMac::parse_tssi_power_efuse(
             logical.data(), logical.size(), physical_full.data(),
             physical_full.size(), tssi));
  expect("TSSI path programming detected",
         tssi.path_a_programmed && tssi.path_b_programmed &&
             tssi.trim_programmed);
  expect("TSSI DE values retain signed representation",
         tssi.path_a_de[0] == 3 && tssi.path_a_de[8] == -4 &&
             tssi.path_b_de[0] == 5 && tssi.path_b_de[8] == -6);
  expect("TSSI trim groups decoded",
         tssi.trim[0][0] == 7 && tssi.trim[0][1] == -8 &&
             tssi.trim[7][0] == 9 && tssi.trim[7][1] == 0);
  logical.fill(0xff);
  physical_full.fill(0xff);
  expect("absent TSSI programming uses vendor zero fallback",
         rtl8733b::Halmac8733bMac::parse_tssi_power_efuse(
             logical.data(), logical.size(), physical_full.data(),
             physical_full.size(), tssi) &&
             !tssi.path_a_programmed && !tssi.path_b_programmed &&
             !tssi.trim_programmed && tssi.path_a_de[8] == 0 &&
             tssi.path_b_de[8] == 0 && tssi.trim[0][0] == 0);
  expect("truncated TSSI physical layout rejected",
         !rtl8733b::Halmac8733bMac::parse_tssi_power_efuse(
             logical.data(), logical.size(), physical_full.data(), 0x1dd,
             tssi));

  rtl8733b::MacState mac_state;
  mac_state.pq_map = 0xf5a4;
  mac_state.rqpn_hlpq = 0x00cb0808;
  mac_state.rqpn_npq = 0x00000808; // observed read-only mirror in bits 15:8
  mac_state.reserved_boundary = 0xe4;
  mac_state.rx_boundary = 0x3eff;
  mac_state.cr = 0xff;
  mac_state.rxdma_mode = 0x1e;
  mac_state.rx_agg = 0x2003;
  mac_state.rcr = 0xe410220e;
  expect("normal USB 3-out MAC state accepted",
         mac_state.matches_normal_usb3out());
  mac_state.rx_agg = 0x2005;
  expect("20 KiB RX aggregate rejected for 16 KiB URBs",
         !mac_state.matches_normal_usb3out());
  mac_state.rx_agg = 0x2003;
  mac_state.reserved_boundary--;
  expect("wrong queue boundary rejected",
         !mac_state.matches_normal_usb3out());

  return failures == 0 ? 0 : 1;
}
