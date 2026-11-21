/* Register values for DeviceConfig::Rx::drop_ctrl / own_addr_only, shared by
 * the Jaguar2, Jaguar3 and RTL8733B monitor RX setups (RCR bits as
 * hal/hal_com_reg.h RCR_AAP..RCR_AB). */
#pragma once
#include <cstdint>

#include "DeviceConfig.h"

namespace devourer {

constexpr uint32_t kRcrAAP = 1u << 0; /* accept all unicast (promiscuous) */
constexpr uint32_t kRcrAPM = 1u << 1; /* accept unicast to MACID */
constexpr uint32_t kRcrAM = 1u << 2;  /* accept multicast (MAR is all-ones) */
constexpr uint32_t kRcrAB = 1u << 3;  /* accept broadcast */

/* The monitor RCR with rx.own_addr_only applied: AAP off, the three
 * address-match accepts on. Unchanged when the knob is off. */
inline uint32_t monitor_rcr(uint32_t rcr, const DeviceConfig::Rx &rx) {
  return rx.own_addr_only ? (rcr & ~kRcrAAP) | kRcrAPM | kRcrAM | kRcrAB : rcr;
}

/* RXFLTMAP1 (0x06A2, one bit per control subtype) for the monitor setup. */
inline uint16_t monitor_ctrl_map(const DeviceConfig::Rx &rx) {
  return rx.drop_ctrl ? 0x0000 : 0xFFFF;
}

/* rx.own_addr_only leaves MACID as the only unicast accept path, and MACID
 * holds one address. A beamformee arm and an ACK responder each program it,
 * so the two can only share a session when they name the same address. */
inline bool own_addr_identity_conflict(const DeviceConfig &cfg,
                                       const uint8_t beamformee_mac[6]) {
  if (!cfg.rx.own_addr_only || !cfg.bf.beamformee_of || !cfg.rx.ack_responder)
    return false;
  for (int i = 0; i < 6; ++i)
    if (cfg.rx.ack_responder->bytes[i] != beamformee_mac[i])
      return true;
  return false;
}

} // namespace devourer
