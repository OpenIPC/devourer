#ifndef MT7612U_MAPPING_H
#define MT7612U_MAPPING_H

#include <cstdint>

#include "RxPacket.h"
#include "SelectedChannel.h"
#include "ieee80211_radiotap.h" /* DESC_RATE* */
#include "mt7612u/mt7612u.h"

/*
 * The pure translations between MT7612U's own descriptor vocabulary and the
 * one every other backend here reports in. They live in a header, apart from
 * the device class, because each is a lookup that is easy to get subtly wrong
 * and impossible to notice on a radio: a wrong RSSI base reads as a weak
 * link, a wrong rate code reads as a slow one. tests/mt7612u_mapping_selftest
 * pins all of them.
 */
namespace mt7612u {

/* devourer carries RSSI as an unsigned byte biased by 110 —
 * LinkHealth.cpp:8 is the authority: `rssi_dbm = rssi_raw - 110`. The HAL
 * reports true dBm, so the bias has to be added, not cast around. Casting a
 * signed -63 dBm straight into the byte yields 193, i.e. +83 dBm. */
inline constexpr int kRssiBiasDb = 110;

inline uint8_t rssi_to_raw(int8_t dbm) {
  const int raw = static_cast<int>(dbm) + kRssiBiasDb;
  if (raw < 0)
    return 0;
  if (raw > 255)
    return 255;
  return static_cast<uint8_t>(raw);
}

/* Copy the per-chain signal into an rx_pkt_attrib, and ONLY the per-chain
 * signal.
 *
 * This exists because `mt7612u_rx_info::rssi[4]` is not four chains. The MAC
 * is 2T2R, so [0] and [1] are chains A and B — but rx.cpp assigns
 * `info->noise = info->rssi[2]`, because RXWI byte 14 is a noise floor (the
 * slot mt76 declares and never reads), and byte 15 is unidentified. A loop
 * over all four therefore hands consumers a phantom chain C pinned near the
 * noise floor and a chain D of garbage, which is exactly what an earlier cut
 * of this integration did. Realtek's own contract is that [2..3] are ZERO on
 * a 2-path part (RxPacket.h), so that is what a 2T2R MediaTek must report
 * too, and n_chains is the authority rather than the array's extent.
 *
 * snr[] is filled from the same report while it is in hand, and it is filled in
 * HALF-dB. That is the unit every other producer and consumer of this field
 * uses - Realtek parsers write s(8,1) (FrameParserJaguar2.h), LinkHealth.cpp
 * reads `snr_raw / 2.0`, and RxQuality derives its noise floor as
 * `(rssi_raw - 110) - snr_raw / 2.0`. The library reports whole dB (`snr_db` is
 * rssi[0] - noise), so writing it through unscaled would report every MT7612U
 * link at half its true SNR and put the derived noise floor snr/2 dB high - the
 * same class of fault as the phantom chain above, one field below it.
 *
 * Only meaningful when noise_valid, which is why the unvalidated case leaves
 * the slots at zero rather than writing a plausible number. See the -116 dBm
 * caveat on `noise` in the public header. */
inline void copy_signal(const struct mt7612u_rx_info &info,
                        struct rx_pkt_attrib &out) {
  const unsigned chains = info.n_chains > 2u ? 2u : info.n_chains;

  for (unsigned i = 0; i < chains; ++i)
    out.rssi[i] = rssi_to_raw(info.rssi[i]);
  for (unsigned i = chains; i < 4u; ++i)
    out.rssi[i] = 0;

  for (unsigned i = 0; i < 4u; ++i)
    out.snr[i] = 0;
  if (info.noise_valid) {
    /* PER CHAIN, from that chain's own RSSI. info.snr_db is defined as
     * rssi[0] - noise (rx.cpp), so writing it into every slot would report
     * chain A's SNR on chain B - two identical numbers, which is exactly what
     * hides a dead chain-B antenna: its RSSI drops while its SNR still tracks
     * chain A's. The noise floor is common to both chains, so the per-chain
     * value is simply rssi[i] - noise. */
    for (unsigned i = 0; i < chains; ++i) {
      int half_db = (static_cast<int>(info.rssi[i]) -
                     static_cast<int>(info.noise)) * 2;
      if (half_db > 127)
        half_db = 127;
      if (half_db < -128)
        half_db = -128;
      out.snr[i] = static_cast<int8_t>(half_db);
    }
  }
}

/* The QoS TID, or nothing when this frame carries no QoS Control field.
 *
 * The offset is NOT a constant 24. A 4-address data frame (ToDS and FromDS
 * both set) has a 30-byte header, so its QoS Control sits at 30 - reading
 * byte 24 there returns the low nibble of Address 4, which is arbitrary and
 * WORSE than leaving the TID zero, because it looks like a plausible priority.
 * The subtree's own mt_hdrlen_from_fc() gets this right for the TX path and
 * rx.cpp uses it for the L2-pad fold; this is the same rule, kept here as a
 * pure function so the selftest can pin it. */
inline bool qos_tid(const uint8_t *f, size_t len, uint8_t &tid) {
  if (len < 2)
    return false;
  const unsigned fc = (unsigned)f[0] | ((unsigned)f[1] << 8);
  if (((fc >> 2) & 3u) != 2u)   /* not a data frame */
    return false;
  if (!(fc & 0x0080u))          /* not a QoS subtype */
    return false;
  const size_t hdr = ((fc & 0x0300u) == 0x0300u) ? 30u : 24u;
  if (len < hdr + 2u)
    return false;
  tid = f[hdr] & 0x0f;
  return true;
}

/* mt7612u_rx_info -> the DESC_RATE numbering consumers read, so a caller does
 * not need to know which chip a frame came from. */
inline uint16_t desc_rate(const struct mt7612u_rx_info &info) {
  switch (info.phy) {
  case MT7612U_PHY_CCK:
    /* DESC_RATE1M..11M are 0..3, in the same order as the CCK indices. */
    return static_cast<uint16_t>(info.mcs & 0x3);
  case MT7612U_PHY_OFDM:
    return static_cast<uint16_t>(DESC_RATE6M + (info.mcs & 0x7));
  case MT7612U_PHY_HT:
  case MT7612U_PHY_HT_GF:
    /* HT folds NSS into the MCS number on both sides, so this is a straight
     * offset for MCS 0-31. Bounded because MT_RATE_INDEX is SIX bits: an index
     * of 32..63 would run past DESC_RATEMCS31 into the VHT numbering and
     * report garbage as a real VHT rate rather than as unknown. rx.cpp filters
     * the PHY field, not the index. */
    /* 0 is DESC_RATE1M, not an "unknown" sentinel - the enum has none - so a
     * consumer sees a plausible 1 Mbps CCK frame rather than a rejected one.
     * Still better than the alternative, which was reporting garbage as a real
     * VHT rate; rx.cpp already drops frames whose PHY field names nothing. */
    if (info.mcs > 31)
      return 0;
    return static_cast<uint16_t>(DESC_RATEMCS0 + info.mcs);
  case MT7612U_PHY_VHT: {
    const uint8_t nss = info.nss ? info.nss : 1;
    /* VHT MCS is 0-9 and the DESC numbering strides by 10 per stream, so an
     * index above 9 spills into the NEXT stream's block - SS1 MCS12 would
     * report as SS2 MCS2. Four streams is likewise the end of the numbering. */
    if (info.mcs > 9 || nss > 4)
      return 0;
    return static_cast<uint16_t>(DESC_RATEVHTSS1MCS0 + (nss - 1) * 10 +
                                 info.mcs);
  }
  }
  return 0;
}

/* MT7612U_BW_* -> the RX-descriptor bandwidth code consumers read.
 *
 * The two happen to share 0/1/2 for 20/40/80, so this is an identity — but
 * written out rather than cast, because "the two enums agree today" is not
 * something a reader of a bare static_cast can check, and the earlier cut of
 * this integration had exactly that cast with no note saying anyone had. */
inline uint8_t bw_to_desc(enum mt7612u_bw bw) {
  switch (bw) {
  case MT7612U_BW_20:
    return 0;
  case MT7612U_BW_40:
    return 1;
  case MT7612U_BW_80:
    return 2;
  }
  return 0;
}

/* SelectedChannel width -> the three widths this port implements. Anything
 * wider or narrower is refused rather than silently narrowed: 5/10 MHz has no
 * encoding in the rate word at all, and 160 MHz is beyond the silicon. Note
 * that a width being accepted here does not mean every channel can carry it —
 * mt7612u_set_channel() refuses a control channel that is off the grid for
 * the requested width. */
inline bool width_to_bw(ChannelWidth_t w, enum mt7612u_bw &out,
                        const char *&why) {
  switch (w) {
  case CHANNEL_WIDTH_20:
    out = MT7612U_BW_20;
    return true;
  case CHANNEL_WIDTH_40:
    out = MT7612U_BW_40;
    return true;
  case CHANNEL_WIDTH_80:
    out = MT7612U_BW_80;
    return true;
  case CHANNEL_WIDTH_5:
  case CHANNEL_WIDTH_10:
    why = "5/10 MHz narrowband: MT_RATE_BW has no encoding for it";
    return false;
  default:
    why = "unsupported channel width";
    return false;
  }
}

} // namespace mt7612u

#endif /* MT7612U_MAPPING_H */
