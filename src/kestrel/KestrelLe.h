#ifndef KESTREL_LE_H
#define KESTREL_LE_H

#include <cstdint>

/* Little-endian 32-bit pack/unpack helpers for the Kestrel mac_ax plane. The
 * H2C content buffers and RX descriptors are all LE dword-packed; these are the
 * one-liners every encoder/parser uses. Shared by KestrelFw.cpp and
 * KestrelFwSched.cpp (the trigger/TWT encoders) so the definition lives in
 * exactly one place. */

namespace kestrel {

inline uint32_t le32(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

inline void put_le32(uint8_t *p, uint32_t v) {
  p[0] = v & 0xFF;
  p[1] = (v >> 8) & 0xFF;
  p[2] = (v >> 16) & 0xFF;
  p[3] = (v >> 24) & 0xFF;
}

} /* namespace kestrel */

#endif /* KESTREL_LE_H */
