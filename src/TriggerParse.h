#pragma once

/* TriggerParse — decode a received 802.11ax Trigger frame.
 *
 * A Trigger is an 802.11 CONTROL frame (FC type 1, subtype 2) aired in a
 * legacy/non-HT PPDU, so ANY monitor — including an 11ac witness that cannot
 * decode HE data — captures the bytes and this parser reads them. That is the
 * validation lever for the F2P/UL_FIXINFO scheduled-UL path: point rxdemo at
 * the AP and confirm the aired trigger's fields match what was commanded, which
 * also empirically resolves the fw's RU/mode encoding.
 *
 * Symmetric with build_basic_trigger() (TriggerTwt.h); the round-trip is a
 * headless selftest (tests/trigger_parse_selftest.cpp). Header-only, no deps,
 * neutral namespace — same shape as RxPacket.h / RadiotapPeek.h. */

#include <cstddef>
#include <cstdint>

namespace devourer {

struct TriggerUserInfo {
  uint16_t aid12 = 0;
  uint8_t ru_alloc = 0;
  bool ldpc = false;
  uint8_t mcs = 0;
  bool dcm = false;
  uint8_t ss = 0;
  int8_t tgt_rssi_dbm = 0;
};

struct TriggerInfo {
  uint8_t trigger_type = 0; /* 0 = Basic, 1 = Beamforming Report Poll, ... */
  uint16_t ul_length = 0;
  uint8_t ul_bw = 0;        /* 0 = 20, 1 = 40, 2 = 80, 3 = 160 */
  uint8_t gi_ltf = 0;
  uint8_t num_he_ltf = 0;
  uint8_t ap_tx_power = 0;
  uint8_t ta[6] = {0};      /* transmitter (AP) address */
  uint8_t ra[6] = {0};      /* receiver address */
  uint8_t n_users = 0;      /* User Info fields decoded (capped at cap below) */
  TriggerUserInfo users[8];
};

/* True iff `frame` (an 802.11 frame body, no radiotap, no FCS) is a Trigger
 * control frame. Cheap FC check for an RX-loop fast path. */
inline bool is_trigger_frame(const uint8_t *frame, size_t len) {
  /* FC byte0: version[1:0]=0, type[3:2]=01(control), subtype[7:4]=0010 -> 0x24. */
  return frame != nullptr && len >= 2 && frame[0] == 0x24;
}

/* Parse a Basic-Trigger-style frame into `out`. Returns false if the frame is
 * not a trigger or is too short for the fixed header + one User Info. The FCS
 * (last 4 bytes on a monitor capture that includes it) is ignored; pass either
 * with or without it. Up to 8 User Info fields are decoded. */
inline bool parse_trigger(const uint8_t *frame, size_t len, TriggerInfo &out) {
  /* FC(2) + Duration(2) + RA(6) + TA(6) + Common(8) = 24, then >=1 user (5). */
  if (!is_trigger_frame(frame, len) || len < 24 + 5)
    return false;
  out = TriggerInfo{};
  for (int i = 0; i < 6; ++i) {
    out.ra[i] = frame[4 + i];
    out.ta[i] = frame[10 + i];
  }
  /* Common Info: little-endian 8-byte bitfield (see build_basic_trigger). */
  uint64_t common = 0;
  for (int i = 0; i < 8; ++i)
    common |= static_cast<uint64_t>(frame[16 + i]) << (8 * i);
  out.trigger_type = static_cast<uint8_t>(common & 0xf);
  out.ul_length = static_cast<uint16_t>((common >> 4) & 0xfff);
  out.ul_bw = static_cast<uint8_t>((common >> 18) & 0x3);
  out.gi_ltf = static_cast<uint8_t>((common >> 20) & 0x3);
  out.num_he_ltf = static_cast<uint8_t>((common >> 23) & 0x7);
  out.ap_tx_power = static_cast<uint8_t>((common >> 28) & 0x3f);
  /* User Info fields until the frame ends. Each is 5 common bytes + a
   * Trigger-Dependent User Info whose size is set by the trigger type (Basic=1
   * byte, per 802.11ax). A Padding field (AID12 == 4095) or a sub-6-byte tail
   * (a capture's trailing FCS) ends the list. */
  const size_t tdep = out.trigger_type == 0 ? 1 : 0; /* Basic Trigger: 1 byte */
  const size_t ustride = 5 + tdep;
  size_t o = 24;
  out.n_users = 0;
  while (o + ustride <= len && out.n_users < 8) {
    uint64_t ui = 0;
    for (int i = 0; i < 5; ++i)
      ui |= static_cast<uint64_t>(frame[o + i]) << (8 * i);
    const uint16_t aid = static_cast<uint16_t>(ui & 0xfff);
    /* AID12 == 0xFFF marks a Padding User Info — end of the user list. */
    if (aid == 0xfff)
      break;
    TriggerUserInfo &u = out.users[out.n_users];
    u.aid12 = aid;
    u.ru_alloc = static_cast<uint8_t>((ui >> 12) & 0xff);
    u.ldpc = ((ui >> 20) & 0x1) != 0;
    u.mcs = static_cast<uint8_t>((ui >> 21) & 0xf);
    u.dcm = ((ui >> 25) & 0x1) != 0;
    u.ss = static_cast<uint8_t>((ui >> 26) & 0x3f);
    /* Target RSSI subfield [38:32]: 0..90 = -110..-20 dBm; 127 = max power (0). */
    uint8_t rssi7 = static_cast<uint8_t>((ui >> 32) & 0x7f);
    u.tgt_rssi_dbm =
        (rssi7 >= 91) ? 0 : static_cast<int8_t>(static_cast<int>(rssi7) - 110);
    out.n_users++;
    o += ustride;
  }
  return out.n_users > 0;
}

} /* namespace devourer */
