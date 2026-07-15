#ifndef SELECTEDCHANNEL_H
#define SELECTEDCHANNEL_H

#include <cstdint>

enum ChannelWidth_t {
  CHANNEL_WIDTH_20 = 0,
  CHANNEL_WIDTH_40 = 1,
  CHANNEL_WIDTH_80 = 2,
  CHANNEL_WIDTH_160 = 3,
  CHANNEL_WIDTH_80_80 = 4,
  CHANNEL_WIDTH_5 = 5,
  CHANNEL_WIDTH_10 = 6,
  CHANNEL_WIDTH_MAX = 7,
};

struct SelectedChannel {
  uint8_t Channel;
  uint8_t ChannelOffset;
  ChannelWidth_t ChannelWidth;
  /* Band selector. 0 = auto: 2.4 GHz when Channel<=14, else 5 GHz (the
   * historical behaviour). 6 = 6 GHz (WiFi 6E) — required because a 6 GHz
   * channel number collides with a 5 GHz one (e.g. ch 37 exists in both), so
   * the band cannot be inferred from Channel alone. Only the RTL8852C-family
   * (tri-band) honours 6; other chips ignore it. */
  uint8_t Band = 0;
};

#endif /* SELECTEDCHANNEL_H */
