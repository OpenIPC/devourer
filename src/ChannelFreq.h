#ifndef CHANNEL_FREQ_H
#define CHANNEL_FREQ_H

#include <cstdint>

/* Wi-Fi channel <-> center-frequency mapping shared by the per-packet radiotap
 * CHANNEL hop path (library side, all generations) and the demos' hop/sweep
 * spec parsing. Generation-neutral, header-only. */

namespace devourer {

/* Map a radiotap CHANNEL frequency (MHz) to a Wi-Fi channel number. Returns 0
 * for a frequency outside the bands devourer drives (caller ignores it). */
static inline int freq_to_chan(uint16_t freq_mhz) {
  if (freq_mhz == 2484)
    return 14;
  if (freq_mhz >= 2412 && freq_mhz <= 2472)
    return (freq_mhz - 2407) / 5;        /* 2.4 GHz ch1..13 */
  if (freq_mhz >= 5000 && freq_mhz <= 5895)
    return (freq_mhz - 5000) / 5;        /* 5 GHz UNII ch */
  return 0;
}

/* Wi-Fi channel number -> center frequency (MHz), for the radiotap CHANNEL
 * field that drives the library's per-packet retune (DEVOURER_HOP_RADIOTAP). */
static inline uint16_t chan_to_freq(int ch) {
  if (ch == 14) return 2484;
  if (ch <= 14) return static_cast<uint16_t>(2407 + 5 * ch);
  return static_cast<uint16_t>(5000 + 5 * ch);
}

} /* namespace devourer */

#endif /* CHANNEL_FREQ_H */
