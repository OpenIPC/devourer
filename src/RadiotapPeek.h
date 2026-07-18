#pragma once

/* RadiotapPeek — cheap pre-parse of a send_packet-contract buffer (radiotap
 * header + 802.11 MPDU) used by the send_packets batch packers: validate the
 * radiotap framing and peek the per-packet CHANNEL hop target WITHOUT building
 * a descriptor, so the packer can decide URB grouping (a channel change
 * flushes the pending URB) before handing the frame to the generation's
 * builder. Generation-neutral; the full parse stays in each HAL. */

#include <climits> /* INT_MIN — "field absent" sentinel */
#include <cstddef>
#include <cstdint>

#include "ChannelFreq.h"
/* Codebase convention: the radiotap iterator is C (Radiotap.c), so every
 * include site wraps the header in extern "C" — this header must too, or an
 * earlier bare include would poison the later wrapped ones with C++ linkage. */
extern "C" {
#include "ieee80211_radiotap.h"
}

namespace devourer {

/* Validate the buffer's radiotap framing (the same checks every generation's
 * send_packet front-end does) and return the radiotap header length; 0 on a
 * malformed buffer (too short / zero / no room for an MPDU). */
inline uint16_t radiotap_hdr_len(const uint8_t *packet, size_t length) {
  if (packet == nullptr || length < sizeof(struct ieee80211_radiotap_header))
    return 0;
  const uint16_t rlen = get_unaligned_le16(packet + 2);
  if (rlen == 0 || static_cast<size_t>(rlen) >= length)
    return 0;
  return rlen;
}

/* Peek the radiotap CHANNEL field's hop target: the mapped channel number, or
 * 0 when the field is absent (or the frequency unmappable). Mirrors the
 * CHANNEL handling in each generation's send_packet (frequency authoritative,
 * flags ignored). */
inline int radiotap_peek_channel(const uint8_t *packet, size_t length) {
  const uint16_t rlen = radiotap_hdr_len(packet, length);
  if (rlen == 0)
    return 0;
  auto *hdr = reinterpret_cast<struct ieee80211_radiotap_header *>(
      const_cast<uint8_t *>(packet));
  struct ieee80211_radiotap_iterator it;
  if (ieee80211_radiotap_iterator_init(&it, hdr, rlen, nullptr) != 0)
    return 0;
  while (ieee80211_radiotap_iterator_next(&it) == 0) {
    if (it.this_arg_index == IEEE80211_RADIOTAP_CHANNEL) {
      const int chan = freq_to_chan(get_unaligned_le16(it.this_arg));
      return chan > 0 ? chan : 0;
    }
  }
  return 0;
}

/* Peek the radiotap DBM_TX_POWER field (signed per-packet power delta, dB).
 * Returns INT_MIN when absent — the same "no per-packet power" sentinel the
 * generations' send_packet parsers use. Lets a batch packer group frames by
 * requested power, so a power-bank reprogram (Jaguar3) lands between URBs,
 * never mid-URB. */
inline int radiotap_peek_dbm_tx_power(const uint8_t *packet, size_t length) {
  const uint16_t rlen = radiotap_hdr_len(packet, length);
  if (rlen == 0)
    return INT_MIN;
  auto *hdr = reinterpret_cast<struct ieee80211_radiotap_header *>(
      const_cast<uint8_t *>(packet));
  struct ieee80211_radiotap_iterator it;
  if (ieee80211_radiotap_iterator_init(&it, hdr, rlen, nullptr) != 0)
    return INT_MIN;
  while (ieee80211_radiotap_iterator_next(&it) == 0) {
    if (it.this_arg_index == IEEE80211_RADIOTAP_DBM_TX_POWER)
      return *reinterpret_cast<const int8_t *>(it.this_arg);
  }
  return INT_MIN;
}

} /* namespace devourer */
