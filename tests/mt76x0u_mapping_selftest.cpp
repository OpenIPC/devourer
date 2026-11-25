/* Headless guard for the MT76x0U <-> devourer RX translations
 * (src/mt76x0u/Mt76x0uMapping.h): DESC_RATE numbering with its out-of-range
 * guards, the RSSI bias and clamp, one chain only (no phantom chains), no
 * fabricated SNR, FCS reported absent, and the QoS TID at the right offset.
 * Header-only: runs whether or not DEVOURER_MT76X0U is on. */
#include <cstdio>
#include <cstring>

#include "mt76x0u/Mt76x0uMapping.h"

namespace {
int fails;
void expect(const char *what, bool ok) {
  if (!ok) {
    std::fprintf(stderr, "mt76x0u_mapping: FAIL %s\n", what);
    fails++;
  }
}
mt76x0u_rx_info rate(mt76x0u_phy phy, uint8_t mcs, uint8_t nss = 1) {
  mt76x0u_rx_info i{};
  i.phy = phy;
  i.mcs = mcs;
  i.nss = nss;
  return i;
}
} // namespace

int main() {
  using namespace mt76x0u;

  expect("CCK 11M", desc_rate(rate(MT76X0U_PHY_CCK, 3)) == 3);
  expect("OFDM 54M", desc_rate(rate(MT76X0U_PHY_OFDM, 7)) == DESC_RATE6M + 7);
  expect("HT MCS7", desc_rate(rate(MT76X0U_PHY_HT, 7)) == DESC_RATEMCS0 + 7);
  expect("HT greenfield as HT", desc_rate(rate(MT76X0U_PHY_HT_GF, 5)) == DESC_RATEMCS0 + 5);
  expect("HT index > 31 is not reported as VHT", desc_rate(rate(MT76X0U_PHY_HT, 40)) == 0);
  expect("VHT 1SS MCS9", desc_rate(rate(MT76X0U_PHY_VHT, 9, 1)) == DESC_RATEVHTSS1MCS0 + 9);
  expect("VHT MCS 10 does not spill into 2SS", desc_rate(rate(MT76X0U_PHY_VHT, 10, 1)) == 0);

  expect("RSSI bias -60 -> 50", rssi_to_raw(-60) == 50);
  expect("RSSI clamps low", rssi_to_raw(-128) == 0);

  expect("BW 40", bw_to_desc(MT76X0U_BW_40) == 1);

  /* A 4-address QoS data frame: FC 0x0388 (data, QoS, ToDS|FromDS). The TID
   * is at 30, not 24 - byte 24 is Address 4. */
  uint8_t f[40];
  std::memset(f, 0, sizeof f);
  f[0] = 0x88;
  f[1] = 0x03;
  f[24] = 0x07; /* address 4, not the TID */
  f[30] = 0x06;
  uint8_t tid = 0;
  expect("4-address QoS TID read at 30", qos_tid(f, sizeof f, tid) && tid == 6);

  mt76x0u_rx_info info = rate(MT76X0U_PHY_HT, 7);
  info.rssi_dbm = -60;
  info.seq = 1234;
  info.bw = MT76X0U_BW_40;
  info.ampdu = 1;
  rx_pkt_attrib a{};
  std::memset(&a, 0x5a, sizeof a); /* stale bytes must be overwritten */
  fill_attrib(info, f, sizeof f, a);
  expect("attrib: length, seq, rate, bw",
         a.pkt_len == sizeof f && a.seq_num == 1234 &&
             a.data_rate == DESC_RATEMCS0 + 7 && a.bw == 1);
  expect("attrib: FCS reported absent (the MAC strips it)", !a.fcs_present);
  expect("attrib: chain 0 only", a.rssi[0] == 50 && a.rssi[1] == 0 &&
                                     a.rssi[2] == 0 && a.rssi[3] == 0);
  expect("attrib: no SNR invented", a.snr[0] == 0 && a.snr[1] == 0);
  expect("attrib: QoS TID", a.qos && a.priority == 6);
  expect("attrib: A-MPDU flag", a.paggr);
  expect("attrib: normal RX", a.pkt_rpt_type == RX_PACKET_TYPE::NORMAL_RX);

  if (fails) {
    std::fprintf(stderr, "mt76x0u_mapping: %d failure(s)\n", fails);
    return 1;
  }
  std::printf("mt76x0u_mapping: ok\n");
  return 0;
}
