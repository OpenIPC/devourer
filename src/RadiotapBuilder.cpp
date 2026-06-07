#include "RadiotapBuilder.h"

#include <cctype>
#include <cstdlib>
#include <cstring>
#include <string>

namespace devourer {

std::array<uint8_t, 13> build_legacy_radiotap(uint8_t rate_500kbps) {
  /* Layout (bytes):
   *   0-1: version/pad
   *   2-3: length = 0x000d
   *   4-7: it_present = 0x00008004 (bit 2 RATE | bit 15 TX_FLAGS)
   *   8:   RATE (500 kbps units)
   *   9:   pad (TX_FLAGS' 2-byte alignment)
   *  10-11: TX_FLAGS = 0x0008 (no-ack)
   *  12:   pad
   *
   * Bit-identical to the historic kRadiotapLegacy6M[13] constant except
   * byte 8, which the caller now controls. */
  return std::array<uint8_t, 13>{{
      0x00, 0x00, 0x0d, 0x00, 0x04, 0x80, 0x00,
      0x00, rate_500kbps, 0x00, 0x08, 0x00, 0x00,
  }};
}

uint8_t parse_stream_rate_env() {
  const char* raw = std::getenv("DEVOURER_STREAM_RATE");
  if (raw == nullptr || *raw == '\0') {
    return kStreamRateDefault500kbps;
  }
  /* Normalise: strip leading/trailing whitespace, upper-case. */
  std::string s;
  for (const char* p = raw; *p; ++p) {
    if (*p == ' ' || *p == '\t' || *p == '\n') continue;
    s.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(*p))));
  }
  /* Mnemonic forms first; falls through to int parse. */
  if (s == "6M")  return 12;
  if (s == "9M")  return 18;
  if (s == "12M") return 24;
  if (s == "18M") return 36;
  if (s == "24M") return 48;
  if (s == "36M") return 72;
  if (s == "48M") return 96;
  if (s == "54M") return 108;
  /* Bare integer interpreted as 500 kbps units. Reject anything that's
   * not a representable legacy-OFDM rate so a typo doesn't quietly send
   * an unknown rate (the chip would either silently drop the frame or
   * pick its fallback default). */
  char* end = nullptr;
  long v = std::strtol(s.c_str(), &end, 0);
  if (end && *end == '\0' && v >= 0 && v <= 255) {
    switch (v) {
      case 12: case 18: case 24: case 36: case 48: case 72: case 96: case 108:
        return static_cast<uint8_t>(v);
      default:
        break;
    }
  }
  /* Unrecognised — fall back to default to keep stream demos forward-
   * compatible when a future env var includes "MCS0" etc. */
  return kStreamRateDefault500kbps;
}

}  // namespace devourer
