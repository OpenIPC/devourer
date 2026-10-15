#pragma once

/* The station address-CAM entry (rtw89 cam.h layout, SHORT length) as eight
 * dwords. One encoder for both ways an entry reaches the hardware: the
 * addr_cam H2C body (KestrelFw::fw_upd_addr_cam_sta, dwords 2..9 of its
 * payload), which the 8852B firmware commits, and the host's own write into
 * the physical CAM (HalKestrel::write_addr_cam_sta) on the 8852C, whose
 * firmware ignores that H2C. Pure, so tests/kestrel_addr_cam_selftest.cpp
 * pins it headless against an entry read back from a live 8852C. */

#include <array>
#include <cstdint>

#include "MacRegAx.h"

namespace kestrel {

/* VALID, `net_type`, no address mask, the SMA/TMA hashes (XOR of the six
 * address bytes, the vendor's unmasked form), `bssid_cam_idx`, SMA = `sma`,
 * TMA = `tma`, and `macid` plus the vendor's bit 14. */
inline std::array<uint32_t, 8> addr_cam_sta_entry(const uint8_t sma[6],
                                                  const uint8_t tma[6],
                                                  uint8_t net_type,
                                                  uint8_t bssid_cam_idx,
                                                  uint8_t macid) {
  uint32_t sh = 0, th = 0;
  for (int i = 0; i < 6; ++i) {
    sh ^= sma[i];
    th ^= tma[i];
  }
  const auto b = [](const uint8_t *p, int i, int sh) {
    return uint32_t{p[i]} << sh;
  };
  return {
      0x1u | (uint32_t{net_type} & 0x3u) << 1 |
          (uint32_t{reg::MAC_AX_NO_MSK} & 0x3u) << 14 | sh << 16 | th << 24,
      uint32_t{bssid_cam_idx} & 0x3fu,
      b(sma, 0, 0) | b(sma, 1, 8) | b(sma, 2, 16) | b(sma, 3, 24),
      b(sma, 4, 0) | b(sma, 5, 8) | b(tma, 0, 16) | b(tma, 1, 24),
      b(tma, 2, 0) | b(tma, 3, 8) | b(tma, 4, 16) | b(tma, 5, 24),
      0u,
      uint32_t{macid} | (1u << 14),
      0u};
}

} // namespace kestrel
