#pragma once

/* Decoded view of the 3-byte radiotap MCS field (known, flags, index) on the
 * TX/inject path — one shared reading for the per-generation send_packet
 * parsers, so the HT LDPC/STBC known/flag bits can't silently diverge again
 * (J2/J3 historically ignored them; only the VHT case honoured them; J1
 * carried its own correct copy). Bit positions per the radiotap MCS spec and
 * RadiotapBuilder::build_ht. */

#include <cstdint>

extern "C" {
#include "ieee80211_radiotap.h"
}

namespace devourer {

struct RadiotapMcsField {
  bool have_mcs = false;
  uint8_t mcs = 0;   /* HT MCS index 0..31, valid when have_mcs */
  bool bw40 = false; /* MCS BW subfield == 40 MHz */
  uint8_t sgi = 0;
  uint8_t ldpc = 0;
  uint8_t stbc = 0;  /* STBC stream count 0..3 */
};

inline RadiotapMcsField decode_radiotap_mcs_field(const uint8_t *arg) {
  RadiotapMcsField f;
  const uint8_t known = arg[0];
  const uint8_t flags = arg[1];
  if ((flags & IEEE80211_RADIOTAP_MCS_BW_MASK) == IEEE80211_RADIOTAP_MCS_BW_40)
    f.bw40 = true;
  f.sgi = (flags & 0x04) ? 1 : 0;
  if ((known & 0x10) && (flags & 0x10)) /* FEC-type known + LDPC flag */
    f.ldpc = 1;
  if (known & 0x20) /* STBC known: flags bits [6:5] = stream count */
    f.stbc = (flags >> 5) & 0x3;
  if ((known & IEEE80211_RADIOTAP_MCS_HAVE_MCS) && arg[2] <= 31) {
    f.have_mcs = true;
    f.mcs = arg[2];
  }
  return f;
}

}  // namespace devourer
