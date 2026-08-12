#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "RxParseAbort.h"
#include "rtl8733b/FrameParser8733b.h"

namespace {
int failures = 0;

void expect(const char *what, bool condition) {
  if (condition)
    return;
  ++failures;
  std::printf("FAIL: %s\n", what);
}

void set_bits(uint8_t *p, unsigned bit, unsigned width, uint32_t value) {
  uint32_t word = rtl8733b::rx_le32(p);
  const uint32_t mask = width == 32 ? 0xffffffffu
                                    : ((1u << width) - 1u) << bit;
  word = (word & ~mask) | ((value << bit) & mask);
  p[0] = static_cast<uint8_t>(word);
  p[1] = static_cast<uint8_t>(word >> 8);
  p[2] = static_cast<uint8_t>(word >> 16);
  p[3] = static_cast<uint8_t>(word >> 24);
}

std::vector<uint8_t> fixture(uint16_t len, uint8_t drvinfo_units,
                             uint8_t shift) {
  const size_t used = rtl8733b::kRxDescSize + drvinfo_units * 8u + shift + len;
  std::vector<uint8_t> bytes((used + 7u) & ~7u, 0);
  set_bits(bytes.data(), 0, 14, len);
  set_bits(bytes.data(), 16, 4, drvinfo_units);
  set_bits(bytes.data(), 24, 2, shift);
  return bytes;
}
} // namespace

int main() {
  rtl8733b::RxFrame8733b parsed;
  std::array<uint8_t, rtl8733b::kRxDescSize - 1> short_desc{};
  expect("short descriptor rejected",
         !rtl8733b::parse_rx_8733b(short_desc.data(), short_desc.size(), parsed));

  auto normal = fixture(19, 4, 2);
  set_bits(normal.data(), 14, 1, 1);
  set_bits(normal.data(), 15, 1, 1);
  set_bits(normal.data(), 20, 3, 5);
  set_bits(normal.data(), 23, 1, 1);
  set_bits(normal.data(), 26, 1, 1);
  set_bits(normal.data() + 4, 8, 4, 7);
  set_bits(normal.data() + 4, 13, 1, 1);
  set_bits(normal.data() + 4, 15, 1, 1);
  set_bits(normal.data() + 4, 26, 1, 1);
  set_bits(normal.data() + 4, 27, 1, 1);
  set_bits(normal.data() + 8, 0, 12, 0xabc);
  set_bits(normal.data() + 8, 12, 4, 3);
  set_bits(normal.data() + 8, 29, 2, 2);
  set_bits(normal.data() + 12, 0, 7, 19);
  set_bits(normal.data() + 12, 16, 8, 2);
  set_bits(normal.data() + 16, 8, 1, 1);
  set_bits(normal.data() + 20, 0, 32, 0x12345678);
  expect("normal descriptor parses",
         rtl8733b::parse_rx_8733b(normal.data(), normal.size(), parsed));
  expect("body offset includes drvinfo and shift",
         parsed.frame == normal.data() + 24 + 32 + 2);
  expect("descriptor scalar fields",
         parsed.frame_len == 19 && parsed.drvinfo_size == 32 &&
             parsed.shift == 2 && parsed.security == 5 && parsed.tid == 7 &&
             parsed.sequence == 0xabc && parsed.fragment == 3 &&
             parsed.rx_rate == 19 && parsed.dma_aggregate_count == 2 &&
             parsed.ppdu_count == 2 && parsed.tsfl == 0x12345678);
  expect("descriptor flags",
         parsed.crc_err && parsed.icv_err && parsed.qos && parsed.physt &&
             parsed.amsdu && parsed.more_data && parsed.more_fragment &&
             parsed.paggr && parsed.ampdu_eof && parsed.sw_decrypted);
  expect("8-byte next alignment", parsed.next_offset == normal.size());

  auto c2h = fixture(8, 0, 0);
  set_bits(c2h.data() + 8, 28, 1, 1);
  /* Non-normal metadata is undefined for C2H and must not move its body. */
  set_bits(c2h.data(), 16, 4, 7);
  set_bits(c2h.data(), 24, 2, 3);
  expect("C2H descriptor parses",
         rtl8733b::parse_rx_8733b(c2h.data(), c2h.size(), parsed) &&
             parsed.c2h && parsed.drvinfo_size == 0 && parsed.shift == 0 &&
             parsed.frame == c2h.data() + 24);

  auto zero = fixture(1, 0, 0);
  set_bits(zero.data(), 0, 14, 0);
  expect("zero packet length rejected",
         !rtl8733b::parse_rx_8733b(zero.data(), zero.size(), parsed));
  auto truncated = fixture(32, 4, 3);
  truncated.resize(truncated.size() - 9);
  expect("truncated body rejected",
         !rtl8733b::parse_rx_8733b(truncated.data(), truncated.size(), parsed));
  std::array<uint8_t, rtl8733b::kRxDescSize> zero_padding{};
  expect("all-zero aggregate remainder is benign padding",
         devourer::rx_parse_remainder_is_zero_padding(zero_padding.data(),
                                                       zero_padding.size()));
  zero_padding.back() = 1;
  expect("non-zero aggregate remainder is malformed",
         !devourer::rx_parse_remainder_is_zero_padding(zero_padding.data(),
                                                        zero_padding.size()));

  auto first = fixture(7, 0, 0);
  auto second = fixture(11, 0, 0);
  set_bits(first.data() + 12, 16, 8, 2);
  std::vector<uint8_t> aggregate;
  aggregate.insert(aggregate.end(), first.begin(), first.end());
  aggregate.insert(aggregate.end(), second.begin(), second.end());
  rtl8733b::RxFrame8733b one, two;
  const bool first_ok = rtl8733b::parse_rx_8733b(
      aggregate.data(), aggregate.size(), one);
  const bool second_ok = first_ok && rtl8733b::parse_rx_8733b(
      aggregate.data() + one.next_offset,
      aggregate.size() - one.next_offset, two);
  expect("two-packet aggregate walk",
         first_ok && second_ok && one.dma_aggregate_count == 2 &&
             one.next_offset + two.next_offset == aggregate.size());

  /* Aggregate tail: the last frame ends mid-8-byte-group, so its rounded-up
   * next_offset points past the end of the completion buffer. Parsing the frame
   * itself must still succeed (its body is entirely present), and advancing the
   * walk by next_offset must leave no room for another descriptor so the RX
   * loop's `offset + kRxDescSize <= length` guard terminates instead of reading
   * past the buffer. */
  auto tail = fixture(19, 0, 0);
  const size_t tail_end = rtl8733b::kRxDescSize + 19u;
  expect("tail fixture is padded to an 8-byte boundary",
         tail.size() == ((tail_end + 7u) & ~size_t{7}) && tail.size() > tail_end);
  tail.resize(tail_end);
  rtl8733b::RxFrame8733b tail_frame;
  const bool tail_ok =
      rtl8733b::parse_rx_8733b(tail.data(), tail.size(), tail_frame);
  expect("unpadded tail frame parses", tail_ok);
  expect("tail next_offset rounds past the buffer end",
         tail_ok && tail_frame.next_offset > tail.size());
  expect("tail advance terminates the aggregate walk",
         tail_ok && static_cast<size_t>(tail_frame.next_offset) +
                            rtl8733b::kRxDescSize >
                        tail.size());

  rx_pkt_attrib attrib{};
  std::array<uint8_t, 32> ofdm{};
  ofdm[0] = 1;
  ofdm[1] = 72;
  ofdm[5] = 0x90;
  ofdm[7] = 0x60;
  ofdm[16] = static_cast<uint8_t>(-22);
  ofdm[20] = static_cast<uint8_t>(-3);
  ofdm[24] = 34;
  expect("OFDM PHY status parses",
         rtl8733b::parse_phy_status_8733b(ofdm.data(), ofdm.size(), 12,
                                          0,
                                          attrib));
  expect("OFDM PHY metrics",
         attrib.rssi[0] == 72 && attrib.bw == 1 && attrib.ldpc == 1 &&
             attrib.stbc == 1 && attrib.evm[0] == -22 &&
             attrib.cfo_tail == -3 && attrib.snr[0] == 34);

  attrib = {};
  ofdm[5] = 0x00;
  expect("full-width HT PHY status parses",
         rtl8733b::parse_phy_status_8733b(ofdm.data(), ofdm.size(), 19, 1,
                                          attrib));
  expect("RXSC zero uses configured HT bandwidth", attrib.bw == 1);
  attrib = {};
  expect("RXSC zero legacy PHY status parses",
         rtl8733b::parse_phy_status_8733b(ofdm.data(), ofdm.size(), 4, 1,
                                          attrib));
  expect("legacy OFDM stays 20 MHz", attrib.bw == 0);

  attrib = {};
  std::array<uint8_t, 32> cck_status{};
  /* -40 dBm in the vendor S(11,3) field: signed byte 0xd8. */
  cck_status[3] = static_cast<uint8_t>(6u << 3);
  cck_status[11] = static_cast<uint8_t>(0xc0);
  expect("CCK type-6 PHY status parses",
         rtl8733b::parse_phy_status_8733b(cck_status.data(),
                                          cck_status.size(), 0, 0, attrib));
  expect("CCK power converts to raw convention", attrib.rssi[0] == 70);
  expect("truncated PHY status rejected",
         !rtl8733b::parse_phy_status_8733b(ofdm.data(), 20, 12, 0, attrib));

  return failures == 0 ? 0 : 1;
}
