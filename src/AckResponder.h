#pragma once

/* AckResponder — the hardware ACK engine as a first-class monitor-mode knob.
 *
 * The Realtek MAC's immediate-response engine is SIFS-timed with zero host
 * involvement. The shared recipe programs the port-0 MACID/BSSID and requests
 * AP net_type. On the Jaguar generations covered by the AP-mode measurements,
 * net_type participates in gating the engine. RTL8733B is different: it
 * matches MACID while net_type reads NoLink, so MAC bring-up already permits
 * responses to the adapter's own address and SetAckResponder retargets that
 * match to the requested address. See the measured truth table in
 * AdapterCaps.h and the RTL8733B/Jaguar1 clear implementations.
 *
 * This header carries that register recipe minus the beacon machinery. Which
 * half controls a live disarm is a measured per-die property: use the shared
 * gate-only clear only where net_type is sufficient; otherwise the backend
 * must also move the identity off the responder address with retarget() or an
 * exact captured-identity restore.
 *
 * On the adapter combinations exercised by tests/ampdu_ba_check.sh, the SAME
 * gate also enables the hardware BlockAck responder. RTL8733B has its own
 * air-side proof in tests/rtl8733b_blockack_onair.sh: a Jaguar2 TX forms real
 * A-MPDUs and an independent Jaguar1 witness observes retry copies collapse
 * only while the RTL8733B responder is armed.
 *
 * The registers are generation-neutral (same map on Jaguar1/2/3 and
 * RTL8733B):
 *   0x0610..0x0615  REG_MACID   — the RA the ACK engine matches
 *   0x0618..0x061d  REG_BSSID   — port identity companion (the proven AP
 *                                 recipe programs both)
 *   0x0102[1:0]     MSR/net_type (REG_CR+2) — 0 NoLink / 1 Ad-hoc / 2 Infra /
 *                                 3 AP. We request AP (3); whether the field
 *                                 gates responses is die-specific.
 *
 * Turning a passive monitor into an ACTIVE transmitter is a behavioral
 * change — hence opt-in only (DeviceConfig rx.ack_responder / the
 * SetAckResponder runtime call), never a default. The MAC must be UNICAST
 * (I/G bit clear) — a station cannot ACK-target a group address, and the
 * same footgun broke AP association (docs/ap-mode.md). */

#include <cstdint>
#include <cstring>

#include "RtlAdapter.h"

namespace devourer {
namespace ack {

/* The MACID pair, composed in ONE place. enable(), verify() and retarget()
 * previously each carried their own copy of this packing, in a header whose
 * verify() comment promises to keep the register map in one file: an
 * endianness or width fix applied to one copy would leave the others reading
 * the old layout while still reporting success. */
inline uint32_t macid_lo(const uint8_t mac[6]) {
  return (uint32_t)mac[0] | ((uint32_t)mac[1] << 8) | ((uint32_t)mac[2] << 16) |
         ((uint32_t)mac[3] << 24);
}
inline uint16_t macid_hi(const uint8_t mac[6]) {
  return (uint16_t)(mac[4] | (mac[5] << 8));
}

/* Is the ACK-match identity currently `mac`? The engine matches MACID, so this
 * is the question "would this port answer for `mac`" — on the RTL8733B that is
 * the WHOLE question (see retarget()). Throws nothing; a transport failure
 * reads as "not this address". */
inline bool macid_is(RtlAdapter &dev, const uint8_t mac[6]) noexcept {
  try {
    return dev.rtw_read<uint32_t>(0x0610) == macid_lo(mac) &&
           dev.rtw_read16(0x0614) == macid_hi(mac);
  } catch (...) {
    return false;
  }
}

inline bool enable(RtlAdapter &dev, const uint8_t mac[6]) noexcept {
  try {
    const uint8_t nt = dev.rtw_read8(0x0102);
    /* Close the gate before changing identity. Besides avoiding a transient
     * responder for a half-written MAC during retargeting, this makes every
     * failed identity write leave the radio passive. */
    if (!dev.rtw_write8(0x0102, static_cast<uint8_t>(nt & ~0x03u))) {
      /* The transfer status is not state readback. Retry the safety clear using
       * the value read before the failed transfer; callers verify it before
       * reporting a failed arm as passive. */
      (void)dev.rtw_write8(0x0102, static_cast<uint8_t>(nt & ~0x03u));
      return false;
    }
    if (!dev.rtw_write<uint32_t>(0x0610, macid_lo(mac)) ||
        !dev.rtw_write16(0x0614, macid_hi(mac)) ||
        !dev.rtw_write<uint32_t>(0x0618, macid_lo(mac)) ||
        !dev.rtw_write16(0x061c, macid_hi(mac)))
      return false;
    if (dev.rtw_write8(0x0102,
                       static_cast<uint8_t>((nt & ~0x03u) | 0x03u)))
      return true;
    /* A failed status does not prove the gate write had no side effect. Make a
     * best-effort close before reporting failure; callers additionally verify
     * the passive rollback. */
    (void)dev.rtw_write8(0x0102, static_cast<uint8_t>(nt & ~0x03u));
    return false;
  } catch (...) {
    /* SetAckResponder is a bool contract. Its callers perform a verified
     * rollback and diagnose whichever register readbacks remain unavailable. */
    return false;
  }
}

/* Shared gate-only clear for dies where net_type controls the responder.
 * RTL8733B and Jaguar1/CHIP_8812 callers must additionally retarget MACID. */
inline bool disable(RtlAdapter &dev) {
  const uint8_t nt = dev.rtw_read8(0x0102);
  return dev.rtw_write8(0x0102, static_cast<uint8_t>(nt & ~0x03u));
}

/* Point the ACK-match identity at another address WITHOUT touching the gate.
 *
 * On RTL8733B this is the ONLY thing that changes whether the port answers.
 * net_type is inert there — measured, three cells, single-shot ACK rate at
 * MCS3 with a Jaguar1 soliciting:
 *
 *   never-armed port, peer solicits the adapter's own EFUSE MAC   85.2 %, 82.5 %
 *   never-armed port, peer solicits an address nobody holds        0.0 %  (control)
 *   port armed to X, peer solicits X                              83.3 %
 *
 * A never-armed monitor answers on its own MAC at the same rate as a
 * deliberately armed responder. So on this die the engine matches MACID and
 * nothing else: MAC bring-up's program_mac is what makes it answer, arming
 * merely repoints it, and 0x0102[1:0] does not gate it in either direction.
 * (The gate IS real on the generations the AP-mode work covered; this is a
 * per-die exception, not a correction to the recipe.)
 *
 * Consequences worth stating plainly, because they are easy to get backwards:
 * every never-armed RTL8733B monitor session already auto-ACKs unicast frames
 * addressed to its own EFUSE MAC — a pre-existing property of that die under
 * this library, not something an arm creates. And a disarm cannot make the port
 * silent; it can only take it off the responder address and put it back on the
 * one it shipped with.
 *
 * `mac` is that restore address, normally the adapter's own. Deliberately NOT
 * zero: many Realtek MAC TX paths refuse to schedule a frame when the MAC ID is
 * zero — the T1 canary bug that programming REG_MACID was introduced to fix
 * (src/jaguar1/HalModule.cpp, EepromManager.h) — and a radio being disarmed may
 * still be injecting. Zero would not remove the match either, only move it:
 * 00:00:00:00:00:00 has the I/G bit clear, so is_unicast() accepts it.
 * Jaguar1/CHIP_8812 has a distinct measured failure with the same remedy:
 * NoLink verifies after a gate-only clear, but the old MACID continues
 * answering until the backend restores its captured pre-arm port identity.
 * AdapterCaps.h owns that before/after evidence; the result is not generalized
 * to other Jaguar1 dies.
 *
 * Gate untouched on purpose: this is the identity half, so a caller composes it
 * with disable() in whichever order its die needs, and no generation gets a
 * behaviour change it was not measured for. */
inline bool retarget(RtlAdapter &dev, const uint8_t mac[6]) noexcept {
  try {
    /* Sequenced into locals rather than `&&`: both halves must be ATTEMPTED.
     * Under short-circuit a failed low write would skip the high one — and by
     * this file's own doctrine (see enable(): "The transfer status is not
     * state readback") that low write may still have landed, leaving a
     * half-updated address with the high half never even tried. */
    const bool lo = dev.rtw_write<uint32_t>(0x0610, macid_lo(mac));
    const bool hi = dev.rtw_write16(0x0614, macid_hi(mac));
    return lo && hi;
  } catch (...) {
    return false;
  }
}

inline bool is_disabled(RtlAdapter &dev) {
  return (dev.rtw_read8(0x0102) & 0x03u) == 0;
}

inline bool disable_verified(RtlAdapter &dev) noexcept {
  try {
    (void)disable(dev);
    /* Readback is the safety result: a control transfer may report failure even
     * though the write landed, while a successful transfer alone proves no
     * state. The caller only needs to know whether the active gate is closed. */
    return is_disabled(dev);
  } catch (...) {
    return false;
  }
}

/* Exact port-0 identity snapshot/restore for a backend that must return more
 * than MACID to its pre-arm state. Keep the packed register representation so
 * no byte-order conversion can diverge between capture and restore. */
struct PortIdentity {
  uint32_t macid_lo = 0;
  uint16_t macid_hi = 0;
  uint32_t bssid_lo = 0;
  uint16_t bssid_hi = 0;
};

inline bool snapshot_port_identity(RtlAdapter &dev, PortIdentity &out) noexcept {
  try {
    out = {.macid_lo = dev.rtw_read<uint32_t>(0x0610),
           .macid_hi = dev.rtw_read16(0x0614),
           .bssid_lo = dev.rtw_read<uint32_t>(0x0618),
           .bssid_hi = dev.rtw_read16(0x061c)};
    return true;
  } catch (...) {
    return false;
  }
}

inline bool port_mac_is(const PortIdentity &identity,
                        const uint8_t mac[6]) noexcept {
  return identity.macid_lo == macid_lo(mac) &&
         identity.macid_hi == macid_hi(mac);
}

inline bool restore_port_identity(RtlAdapter &dev,
                                  const PortIdentity &identity) noexcept {
  /* Keep each operation exception-contained so all four halves are attempted
   * even if a transport throws instead of returning false. */
  auto write32 = [&dev](uint16_t reg, uint32_t value) noexcept {
    try {
      return dev.rtw_write<uint32_t>(reg, value);
    } catch (...) {
      return false;
    }
  };
  auto write16 = [&dev](uint16_t reg, uint16_t value) noexcept {
    try {
      return dev.rtw_write16(reg, value);
    } catch (...) {
      return false;
    }
  };
  const bool ml = write32(0x0610, identity.macid_lo);
  const bool mh = write16(0x0614, identity.macid_hi);
  const bool bl = write32(0x0618, identity.bssid_lo);
  const bool bh = write16(0x061c, identity.bssid_hi);
  return ml && mh && bl && bh;
}

inline bool port_identity_is(RtlAdapter &dev,
                             const PortIdentity &identity) noexcept {
  try {
    return dev.rtw_read<uint32_t>(0x0610) == identity.macid_lo &&
           dev.rtw_read16(0x0614) == identity.macid_hi &&
           dev.rtw_read<uint32_t>(0x0618) == identity.bssid_lo &&
           dev.rtw_read16(0x061c) == identity.bssid_hi;
  } catch (...) {
    return false;
  }
}

/* The MAC must be UNICAST: a station cannot ACK-target a group address, so an
 * arm on one can never fire. Lives here rather than in each backend because
 * the precondition is a property of the recipe, not of any one die. */
inline bool is_unicast(const uint8_t mac[6]) { return (mac[0] & 0x01u) == 0; }

/* A rollback target must be a usable station identity, not merely readable.
 * An all-zero MAC has the I/G bit clear (so is_unicast accepts it), remains a
 * matchable address, and also stops TX scheduling on the Jaguar1 8812 path. */
inline bool has_safe_restore_mac(const PortIdentity &identity) noexcept {
  return (identity.macid_lo != 0 || identity.macid_hi != 0) &&
         (identity.macid_lo & 0x01u) == 0;
}

/* Retargeting can only move a responder off its armed address when the restore
 * identity differs. The caller owns both values and must reject equality on a
 * die where the gate clear alone was measured insufficient. */
inline bool disarmable_by_retarget(const uint8_t responder[6],
                                   const PortIdentity &restore) noexcept {
  return !port_mac_is(restore, responder);
}

/* Did the requested arm recipe land? Reads back net_type and the RA the ACK
 * engine matches (MACID), composed exactly as enable() writes them — keeping
 * the register map in ONE file, so a change to enable() cannot silently
 * diverge from a copy of it somewhere else.
 *
 * NB the BSSID companion at 0x0618 is written but not checked: the ACK engine
 * matches on MACID, and 0x0618 is programmed only because the proven AP recipe
 * programs both. Verifying the two behavior-relevant programmed fields keeps
 * this honest without asserting that both are gates on every die. */
inline bool verify(RtlAdapter &dev, const uint8_t mac[6]) noexcept {
  try {
    return (dev.rtw_read8(0x0102) & 0x03u) == 0x03u && macid_is(dev, mac);
  } catch (...) {
    return false;
  }
}

} /* namespace ack */
} /* namespace devourer */
