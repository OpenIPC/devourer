#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "rtl8733b/TxDescriptor8733b.h"

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
  using rtl8733b::TxDescConfig;
  std::array<uint8_t, rtl8733b::kTxDescSize> desc{};

  for (uint8_t rate = 0; rate <= 3; ++rate) {
    expect("2G CCK rate ID",
           rtl8733b::tx_rate_id_8733b(rate, 0, false) == 8);
    expect("5G CCK rejected",
           rtl8733b::tx_rate_id_8733b(rate, 0, true) == 0xff);
    expect("40 MHz CCK rejected",
           rtl8733b::tx_rate_id_8733b(rate, 1, false) == 0xff);
  }
  expect("2G legacy rate ID", rtl8733b::tx_rate_id_8733b(4, 0, false) == 6);
  expect("5G legacy rate ID", rtl8733b::tx_rate_id_8733b(4, 0, true) == 7);
  expect("2G HT20 rate ID", rtl8733b::tx_rate_id_8733b(12, 0, false) == 3);
  expect("2G HT40 rate ID", rtl8733b::tx_rate_id_8733b(12, 1, false) == 1);
  expect("5G HT rate ID", rtl8733b::tx_rate_id_8733b(12, 1, true) == 5);
  expect("unsupported rate has no rate ID",
         rtl8733b::tx_rate_id_8733b(20, 0, false) == 0xff);

  std::array<uint8_t, 4> field{};
  rtl8733b::txdesc_set_bits(field.data(), 0, 32, 0x89abcdefu);
  expect("full-width bitfield is defined",
         rtl8733b::txdesc_le32(field.data()) == 0x89abcdefu);
  const auto field_before_invalid = field;
  rtl8733b::txdesc_set_bits(field.data(), 0, 0, 0);
  rtl8733b::txdesc_set_bits(field.data(), 1, 32, 0);
  rtl8733b::txdesc_set_bits(field.data(), 31, 2, 0);
  expect("empty and out-of-range bitfields are no-ops",
         field == field_before_invalid);

  TxDescConfig ofdm{};
  ofdm.packet_size = 48;
  ofdm.sequence = 0x123;
  ofdm.rate_hw = 4;
  ofdm.rate_id = 7;
  ofdm.bmc = true;
  expect("6M descriptor builds",
         rtl8733b::fill_tx_desc_8733b(desc.data(), desc.size(), ofdm));
  const std::array<uint8_t, rtl8733b::kTxDescSize> expected_ofdm = {
      0x30, 0x00, 0x28, 0x01, 0x01, 0x12, 0x07, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00,
      0x04, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0xe7, 0xeb, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x12, 0x00};
  expect("6M descriptor golden bytes", desc == expected_ofdm);
  expect("6M checksum folds to all ones",
         rtl8733b::txdesc_checksum_valid_8733b(desc.data(), desc.size()));

  TxDescConfig cck = ofdm;
  cck.rate_hw = 0; // 1 Mbps long-preamble CCK
  cck.rate_id = 8;
  expect("1M CCK descriptor builds",
         rtl8733b::fill_tx_desc_8733b(desc.data(), desc.size(), cck));
  expect("1M CCK descriptor fields",
         (desc[0x10] & 0x7f) == 0 && (desc[0x06] & 0x1f) == 8 &&
             ((desc[0x14] >> 4) & 1) == 0 &&
             rtl8733b::txdesc_checksum_valid_8733b(desc.data(), desc.size()));

  TxDescConfig ht{};
  ht.packet_size = 1500;
  ht.sequence = 0xabc;
  ht.rate_hw = 19; // MCS7
  ht.rate_id = 3;  // BGN 1SS
  ht.bandwidth = 1;
  ht.short_gi = true;
  expect("MCS7 descriptor builds",
         rtl8733b::fill_tx_desc_8733b(desc.data(), desc.size(), ht));
  const std::array<uint8_t, rtl8733b::kTxDescSize> expected_ht = {
      0xdc, 0x05, 0x28, 0x00, 0x01, 0x12, 0x03, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00,
      0x13, 0x00, 0x02, 0x00, 0x30, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x28, 0xef, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xab, 0x00};
  expect("MCS7 descriptor golden bytes", desc == expected_ht);

  const uint16_t original_checksum =
      static_cast<uint16_t>(desc[28] | (desc[29] << 8));
  desc[39] ^= 0xff;
  expect("checksum excludes bytes 32..39",
         rtl8733b::txdesc_checksum_8733b(desc.data()) == original_checksum);
  desc[7] ^= 1;
  expect("checksum detects mutation inside first 32 bytes",
         !rtl8733b::txdesc_checksum_valid_8733b(desc.data(), desc.size()));

  std::array<uint8_t, rtl8733b::kTxDescSize - 1> short_desc{};
  expect("short output rejected",
         !rtl8733b::fill_tx_desc_8733b(short_desc.data(), short_desc.size(),
                                       ofdm));
  TxDescConfig invalid = ofdm;
  invalid.packet_size = 0;
  expect("zero payload rejected",
         !rtl8733b::fill_tx_desc_8733b(desc.data(), desc.size(), invalid));
  invalid = cck;
  invalid.short_gi = true;
  expect("CCK short preamble rejected until validated",
         !rtl8733b::fill_tx_desc_8733b(desc.data(), desc.size(), invalid));
  invalid = cck;
  invalid.bandwidth = 1;
  expect("CCK at 40 MHz rejected",
         !rtl8733b::fill_tx_desc_8733b(desc.data(), desc.size(), invalid));
  invalid = ofdm;
  invalid.rate_hw = 20; // MCS8 would require a second spatial stream
  expect("second-stream HT rejected",
         !rtl8733b::fill_tx_desc_8733b(desc.data(), desc.size(), invalid));
  invalid = ofdm;
  invalid.bandwidth = 2;
  expect("80 MHz rejected",
         !rtl8733b::fill_tx_desc_8733b(desc.data(), desc.size(), invalid));
  invalid = ofdm;
  invalid.short_gi = true;
  expect("legacy SGI rejected",
         !rtl8733b::fill_tx_desc_8733b(desc.data(), desc.size(), invalid));
  invalid = ht;
  invalid.ldpc = true;
  expect("RTL8733B LDPC rejected",
         !rtl8733b::fill_tx_desc_8733b(desc.data(), desc.size(), invalid));
  invalid = ofdm;
  invalid.packet_offset = 2;
  expect("oversized USB boundary shim rejected",
         !rtl8733b::fill_tx_desc_8733b(desc.data(), desc.size(), invalid));

  return failures == 0 ? 0 : 1;
}
