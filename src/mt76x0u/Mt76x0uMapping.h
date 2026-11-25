#ifndef MT76X0U_MAPPING_H
#define MT76X0U_MAPPING_H

#include <cstddef>
#include <cstdint>

#include "RxPacket.h"
#include "ieee80211_radiotap.h" /* DESC_RATE* */
#include "mt76x0u/mt76x0u.h"

/* Pure translations from the MT76x0U's RX descriptor vocabulary to
 * devourer's, the counterpart of mt7612u/Mt7612uMapping.h (whose rules and
 * their reasons apply unchanged: the same RXWI rate word, the same DESC_RATE
 * numbering, the same RSSI bias). Pinned by tests/mt76x0u_mapping_selftest.cpp. */
namespace mt76x0u {

/* devourer carries RSSI as an unsigned byte biased by 110 (rx_pkt_attrib). */
inline constexpr int kRssiBiasDb = 110;

inline uint8_t rssi_to_raw(int8_t dbm) {
  const int raw = static_cast<int>(dbm) + kRssiBiasDb;
  return static_cast<uint8_t>(raw < 0 ? 0 : raw > 255 ? 255 : raw);
}

/* The rate word -> DESC_RATE. Out-of-range indices report 0 (DESC_RATE1M)
 * rather than spilling into the next block, as in Mt7612uMapping.h. */
inline uint16_t desc_rate(const struct mt76x0u_rx_info &info) {
  switch (info.phy) {
  case MT76X0U_PHY_CCK:
    return static_cast<uint16_t>(info.mcs & 0x3);
  case MT76X0U_PHY_OFDM:
    return static_cast<uint16_t>(DESC_RATE6M + (info.mcs & 0x7));
  case MT76X0U_PHY_HT:
  case MT76X0U_PHY_HT_GF:
    if (info.mcs > 31)
      return 0;
    return static_cast<uint16_t>(DESC_RATEMCS0 + info.mcs);
  case MT76X0U_PHY_VHT: {
    const uint8_t nss = info.nss ? info.nss : 1;
    if (info.mcs > 9 || nss > 4)
      return 0;
    return static_cast<uint16_t>(DESC_RATEVHTSS1MCS0 + (nss - 1) * 10 + info.mcs);
  }
  }
  return 0;
}

inline uint8_t bw_to_desc(enum mt76x0u_bw bw) {
  switch (bw) {
  case MT76X0U_BW_20:
    return 0;
  case MT76X0U_BW_40:
    return 1;
  case MT76X0U_BW_80:
    return 2;
  }
  return 0;
}

/* The QoS TID, or nothing: QoS Control is at 30, not 24, on a 4-address
 * data frame (Mt7612uMapping.h qos_tid). */
inline bool qos_tid(const uint8_t *f, size_t len, uint8_t &tid) {
  if (len < 2)
    return false;
  const unsigned fc = (unsigned)f[0] | ((unsigned)f[1] << 8);
  if (((fc >> 2) & 3u) != 2u || !(fc & 0x0080u))
    return false;
  const size_t hdr = ((fc & 0x0300u) == 0x0300u) ? 30u : 24u;
  if (len < hdr + 2u)
    return false;
  tid = f[hdr] & 0x0f;
  return true;
}

/* One parsed frame -> rx_pkt_attrib. One chain: rssi[0] only, the rest 0
 * (Realtek's contract for absent paths); no SNR (no validated noise floor
 * on this part). fcs_present is FALSE: this MAC strips the FCS. */
inline void fill_attrib(const struct mt76x0u_rx_info &info, const uint8_t *frame,
                        size_t len, struct rx_pkt_attrib &a) {
  a.pkt_len = static_cast<uint16_t>(len);
  a.crc_err = info.crc_err != 0;
  a.seq_num = info.seq;
  a.data_rate = desc_rate(info);
  a.bw = bw_to_desc(info.bw);
  a.stbc = info.stbc;
  a.ldpc = info.ldpc;
  a.sgi = info.sgi;
  a.paggr = info.ampdu != 0;
  a.pkt_rpt_type = RX_PACKET_TYPE::NORMAL_RX;
  a.fcs_present = false;
  a.rssi[0] = rssi_to_raw(info.rssi_dbm);
  for (int i = 1; i < 4; ++i)
    a.rssi[i] = 0;
  for (int i = 0; i < 4; ++i)
    a.snr[i] = 0;
  uint8_t tid = 0;
  a.qos = qos_tid(frame, len, tid);
  a.priority = tid;
}

} // namespace mt76x0u

#endif /* MT76X0U_MAPPING_H */
