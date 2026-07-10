#pragma once

/* AckResponder — the hardware ACK engine as a first-class monitor-mode knob.
 *
 * The Realtek MAC auto-ACKs (SIFS-timed, zero host involvement) any unicast
 * frame whose RA matches the port-0 MACID, PROVIDED the port has a network
 * type. devourer's monitor bring-up leaves net_type = 0 (No Link), which is
 * why a monitor radio never ACKs; the AP-mode work proved the gate — with
 * MACID + net_type programmed, a real station's auth/assoc arrive at retry=0
 * (docs/ap-mode.md). This header is that recipe minus the beacon machinery:
 * program the port identity, flip the net type, and the chip becomes an ACK
 * responder for one MAC address while everything else about monitor mode
 * (promiscuous RX, injection) is unchanged.
 *
 * The registers are generation-neutral (same map on Jaguar1/2/3):
 *   0x0610..0x0615  REG_MACID   — the RA the ACK engine matches
 *   0x0618..0x061d  REG_BSSID   — port identity companion (the proven AP
 *                                 recipe programs both)
 *   0x0102[1:0]     MSR/net_type (REG_CR+2) — 0 NoLink / 1 Ad-hoc / 2 Infra /
 *                                 3 AP; any nonzero arms the responder. We
 *                                 use AP (3), the bench-proven value.
 *
 * Turning a passive monitor into an ACTIVE transmitter is a behavioral
 * change — hence opt-in only (DeviceConfig rx.ack_responder / the
 * SetAckResponder runtime call), never a default. The MAC must be UNICAST
 * (I/G bit clear) — a station cannot ACK-target a group address, and the
 * same footgun broke AP association (docs/ap-mode.md). */

#include <cstdint>

#include "RtlAdapter.h"

namespace devourer {
namespace ack {

inline void enable(RtlAdapter &dev, const uint8_t mac[6]) {
  dev.rtw_write<uint32_t>(0x0610, (uint32_t)mac[0] | ((uint32_t)mac[1] << 8) |
                                      ((uint32_t)mac[2] << 16) |
                                      ((uint32_t)mac[3] << 24));
  dev.rtw_write16(0x0614, (uint16_t)(mac[4] | (mac[5] << 8)));
  dev.rtw_write<uint32_t>(0x0618, (uint32_t)mac[0] | ((uint32_t)mac[1] << 8) |
                                      ((uint32_t)mac[2] << 16) |
                                      ((uint32_t)mac[3] << 24));
  dev.rtw_write16(0x061c, (uint16_t)(mac[4] | (mac[5] << 8)));
  const uint8_t nt = dev.rtw_read8(0x0102);
  dev.rtw_write8(0x0102, static_cast<uint8_t>((nt & ~0x03u) | 0x03u));
}

/* Disarm: net_type back to No Link — the gate, so the MACID may stay. */
inline void disable(RtlAdapter &dev) {
  const uint8_t nt = dev.rtw_read8(0x0102);
  dev.rtw_write8(0x0102, static_cast<uint8_t>(nt & ~0x03u));
}

} /* namespace ack */
} /* namespace devourer */
