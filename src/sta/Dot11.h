/* Dot11 — 802.11 management-frame construction and parsing, written to serve
 * either role. In this tree its user is the station (StationSm, BssTable).
 *
 * Pure: no device access, no environment, no clock, no threads, no sockets —
 * the contract `src/hopset/` and `src/chanmig/` keep, and the reason those are
 * testable without hardware. Everything here is a function of its arguments.
 *
 * WHY ONE ROLE-NEUTRAL FILE. A station's auth-request and an AP's
 * auth-response are the same frame with two fields swapped; a beacon an AP
 * builds and a beacon a station parses are the same bytes read in opposite
 * directions. `tests/ap_responder.cpp` and `tests/ap_wpa2.cpp` carry their own
 * inline, untested copies of the builder half and do not use this module. Its
 * golden-byte cells pin the exact rate sets and RSN element those two
 * harnesses air, so the AP-side builders here produce what the validated AP
 * already sends.
 *
 * Byte order: 802.11 is little-endian on the wire. Every 16-bit field here is
 * written and read as such explicitly, never by casting a struct over a
 * buffer.
 */
#ifndef DEVOURER_STA_DOT11_H
#define DEVOURER_STA_DOT11_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace devourer {
namespace sta {

/* ---- frame control ---------------------------------------------------- */

/* fc[0] values: type and subtype together, which is how the harnesses match.
 * Keeping them as the composed byte rather than separate type/subtype fields
 * is deliberate — every dispatch site compares the whole octet. */
enum : uint8_t {
  kFcAssocReq = 0x00,
  kFcAssocResp = 0x10,
  kFcReassocReq = 0x20,
  kFcReassocResp = 0x30,
  kFcProbeReq = 0x40,
  kFcProbeResp = 0x50,
  kFcBeacon = 0x80,
  kFcDisassoc = 0xa0,
  kFcAuth = 0xb0,
  kFcDeauth = 0xc0,
  kFcData = 0x08,
  kFcQosData = 0x88,
};

/* fc[1] flags */
enum : uint8_t {
  kFcToDs = 0x01,
  kFcFromDs = 0x02,
  kFcMoreFrag = 0x04,
  kFcRetry = 0x08,
  kFcPwrMgmt = 0x10,
  kFcMoreData = 0x20,
  kFcProtected = 0x40,
};

/* Element IDs used by an infrastructure BSS association. */
enum : uint8_t {
  kEidSsid = 0,
  kEidSupportedRates = 1,
  kEidDsParams = 3,
  kEidTim = 5,
  kEidErp = 42,
  kEidHtCaps = 45,
  kEidRsn = 48,
  kEidExtSupportedRates = 50,
  kEidHtOperation = 61,
  kEidVhtCaps = 191,
  kEidVhtOperation = 192,
};

inline void put_le16(std::vector<uint8_t>& v, uint16_t x) {
  v.push_back((uint8_t)(x & 0xff));
  v.push_back((uint8_t)(x >> 8));
}
inline uint16_t get_le16(const uint8_t* p) {
  return (uint16_t)(p[0] | (p[1] << 8));
}

/* ---- headers ---------------------------------------------------------- */

/* A 24-byte 3-address management header.
 *
 * `da` is address 1, `sa` address 2, `bssid` address 3. That ordering is the
 * whole difference between the two roles: an AP answering a station passes
 * (station, bssid, bssid); a station addressing its AP passes
 * (bssid, own, bssid). Sequence control is left zero — see assign_seq below,
 * which is NOT optional for a station.
 */
inline std::vector<uint8_t> mgmt_hdr(uint8_t subtype_fc, const uint8_t da[6],
                                     const uint8_t sa[6],
                                     const uint8_t bssid[6]) {
  std::vector<uint8_t> m;
  m.reserve(24);
  m.push_back(subtype_fc);
  m.push_back(0x00);
  put_le16(m, 0); /* duration */
  m.insert(m.end(), da, da + 6);
  m.insert(m.end(), sa, sa + 6);
  m.insert(m.end(), bssid, bssid + 6);
  put_le16(m, 0); /* sequence control */
  return m;
}

/* Write a sequence number into a built frame's Sequence Control field.
 *
 * THIS MATTERS AND IS EASY TO MISS. On this project's MediaTek backend the MAC
 * assigns sequence numbers only for beacons — `MT_TXWI_ACK_CTL_NSEQ` is set
 * for `MT_TXOPT_BEACON` and nothing else (`src/mt7612u/tx.cpp`). Both AP
 * harnesses therefore air every management frame with sequence 0, which
 * survives only because they air so few. A station's data plane feeds the AP's
 * duplicate detector, where a pinned sequence number is precisely what gets
 * dropped. `seq` is a 12-bit counter; the low 4 bits are the fragment number
 * and stay zero for an unfragmented frame.
 */
inline void assign_seq(std::vector<uint8_t>& frame, uint16_t seq) {
  if (frame.size() < 24) return;
  uint16_t sc = (uint16_t)((seq & 0x0fff) << 4);
  frame[22] = (uint8_t)(sc & 0xff);
  frame[23] = (uint8_t)(sc >> 8);
}

/* DUPLICATE DETECTION, 802.11-2016 10.3.2.14. A transmitter that retries -
 * and both ends of this link do, since STA_ACK and AP_RETRY - resends the
 * same frame with the Retry bit set whenever an ACK is lost, so the receiver
 * sees it twice. The standard answer is a per-transmitter cache of the last
 * Sequence Control accepted (per TID for QoS data): a frame with Retry set
 * that matches it is a duplicate and is discarded before anything else looks
 * at it.
 *
 * Without this the second copy reaches the CCMP replay check, which rejects
 * it (same PN) - the right outcome, counted as the wrong thing: a replay
 * counter that goes up on every lost ACK can no longer tell a retransmission
 * from an attack. Pure, per peer; keep one per transmitter.
 *
 * It runs before decryption, as it does in real stacks, so a forged frame can
 * move the cache, and that cuts BOTH ways. A forgery that moves it off a
 * sequence number costs one legitimate retransmission being processed
 * instead of dropped - and the replay window still refuses it. A forgery
 * that moves it ONTO one (Retry=0 at the peer's next sequence number) makes
 * the peer's genuine Retry=1 copy of that frame match and be DROPPED as a
 * duplicate, if the original was lost on air - one lost frame per forgery,
 * counted in the dup-drop counter rather than as a replay. mac80211 has the
 * same exposure; the cache is not an integrity mechanism. */
class DupDetector {
public:
  static constexpr int kNonQosTid = 16;
  static constexpr int kSlots = 17;

  /* True when this frame is a retransmission of the last one accepted on the
   * same TID (Retry bit set, identical Sequence Control). A frame that is not
   * a duplicate becomes the new "last". */
  bool is_duplicate(bool retry, uint16_t seq_ctl, int tid = kNonQosTid) {
    if (tid < 0 || tid >= kSlots) tid = kNonQosTid;
    const bool dup = retry && seen_[tid] && last_[tid] == seq_ctl;
    if (!dup) {
      seen_[tid] = true;
      last_[tid] = seq_ctl;
    }
    return dup;
  }
  void reset() {
    for (int i = 0; i < kSlots; ++i) {
      seen_[i] = false;
      last_[i] = 0;
    }
  }

private:
  bool seen_[kSlots] = {};
  uint16_t last_[kSlots] = {};
};

/* A monotonic 12-bit sequence counter. One per transmitter address; a station
 * needs exactly one for everything it sends.
 *
 * Atomic, because "one for everything it sends" is an invitation to share it
 * between a TX thread and the RX thread that answers management frames - which
 * is exactly the shape both AP harnesses have. The relaxed ordering is right:
 * the only requirement is that no two frames get the same number, not that the
 * numbers order against anything else. */
class SeqCounter {
public:
  uint16_t next() {
    return (uint16_t)(n_.fetch_add(1, std::memory_order_relaxed) & 0x0fff);
  }
  void reset() { n_.store(0, std::memory_order_relaxed); }

private:
  std::atomic<uint16_t> n_{0};
};

/* ---- information elements --------------------------------------------- */

/* Returns false and emits NOTHING when the body will not fit an 8-bit length.
 * The truncating form silently wrote a length shorter than the bytes that
 * followed it, which corrupts every element after it in the frame - a
 * whole-frame corruption with no local symptom. */
inline bool append_ie(std::vector<uint8_t>& m, uint8_t eid, const uint8_t* body,
                      size_t len) {
  if (len > 255) return false;
  m.push_back(eid);
  m.push_back((uint8_t)len);
  m.insert(m.end(), body, body + len);
  return true;
}

/* An SSID is at most 32 octets (802.11-2016 9.4.2.2). Longer is a caller bug,
 * refused here rather than aired as a corrupt element. */
inline bool append_ssid(std::vector<uint8_t>& m, const std::string& ssid) {
  if (ssid.size() > 32) return false;
  return append_ie(m, kEidSsid, (const uint8_t*)ssid.data(), ssid.size());
}

/* The 2.4 GHz rate set both AP harnesses air, byte for byte: 1/2/5.5/11 CCK
 * marked BASIC, then 18/24/36/54 OFDM non-basic. Kept as one function so the
 * AP and a station advertise the same thing and a mismatch cannot appear
 * between them.
 *
 * Note what is NOT here: 6, 9, 12 and 48 Mbps. For an AP's advertised basic
 * set that is a deliberate, on-air-validated choice, and these bytes match
 * what the AP harnesses air. For a STATION's probe and association request it
 * is wrong - a station that omits 6/12/24 is claiming it cannot do the
 * mandatory OFDM rates - which is why a station uses its own builder,
 * append_supported_rates_sta, rather than widening this one underneath a
 * validated AP. */
inline void append_supported_rates(std::vector<uint8_t>& m) {
  static const uint8_t r[] = {0x82, 0x84, 0x8b, 0x96, 0x24, 0x30, 0x48, 0x6c};
  append_ie(m, kEidSupportedRates, r, sizeof r);
}

/* THE STATION'S OWN RATE SET, which is what the note above says a station
 * needs rather than widening the AP's validated one.
 *
 * 802.11-2007 onwards makes 6, 12 and 24 Mbps the mandatory OFDM rates, and a
 * conforming AP whose basic-rate set includes one the station did not
 * advertise refuses the association with status 18 ("does not support all
 * data rates in the BSSBasicRateSet"). The AP's set omits 6, 9, 12 and 48 for
 * on-air-validated reasons of its own; a station claiming the same thing is
 * claiming it cannot do the mandatory rates.
 *
 * A Supported Rates element holds at most eight; the rest go in Extended
 * Supported Rates, which is what append_ext_supported_rates_sta is for. None
 * are marked BASIC - that is the AP's statement to make, not a station's. */
inline void append_supported_rates_sta(std::vector<uint8_t>& m,
                                       bool five_ghz) {
  static const uint8_t g[] = {0x02, 0x04, 0x0b, 0x16,
                              0x0c, 0x12, 0x18, 0x24};
  static const uint8_t a[] = {0x0c, 0x12, 0x18, 0x24,
                              0x30, 0x48, 0x60, 0x6c};

  if (five_ghz) append_ie(m, kEidSupportedRates, a, sizeof a);
  else append_ie(m, kEidSupportedRates, g, sizeof g);
}

/* 2.4 GHz only: the four OFDM rates that did not fit above. A 5 GHz station
 * has all eight of its rates in the Supported Rates element already, and an
 * empty Extended Supported Rates element is malformed. */
inline void append_ext_supported_rates_sta(std::vector<uint8_t>& m,
                                           bool five_ghz) {
  static const uint8_t g[] = {0x30, 0x48, 0x60, 0x6c};

  if (!five_ghz) append_ie(m, kEidExtSupportedRates, g, sizeof g);
}

/* 5 GHz has no CCK, so the basic set is OFDM-only. Airing CCK rates as BASIC
 * on a 5 GHz BSS is a spec violation a strict station may refuse outright. */
inline void append_supported_rates_5g(std::vector<uint8_t>& m) {
  static const uint8_t r[] = {0x8c, 0x12, 0x98, 0x24, 0xb0, 0x48, 0x60, 0x6c};
  append_ie(m, kEidSupportedRates, r, sizeof r);
}

inline void append_ds_params(std::vector<uint8_t>& m, uint8_t chan) {
  append_ie(m, kEidDsParams, &chan, 1);
}

/* Traffic Indication Map (802.11-2016 9.4.2.6).
 *
 * EVERY beacon must carry one. A beacon without a TIM is not a conforming AP
 * beacon (802.11-2016 9.4.2.6), and this tree's AP harnesses, which build
 * their beacons inline, air none.
 *
 * WHAT A MISSING TIM COSTS, precisely. A station in power save loses most of
 * what these harnesses send it - measured, 0/60 pings with power save on
 * against 60/60 with it off - but adding this element would not repair that.
 * The loss is caused by nothing being buffered: replies air the instant the
 * request is parsed, so a dozing station misses them whatever schedule the
 * beacon advertises. What a missing TIM costs is CONFORMANCE, and a station
 * having no DTIM schedule to synchronise to at all. Those are worth fixing on
 * their own; they are not the 0/60.
 *
 * This is the MINIMUM conforming element and nothing more. It advertises
 * "nothing is buffered for anyone":
 *
 *   DTIM Count   0   - this beacon IS a DTIM beacon
 *   DTIM Period  1   - every beacon is, so a station never waits
 *   Bitmap Ctrl  0   - offset 0, and bit 0 clear means no group-addressed
 *                      traffic is buffered either
 *   Partial VBM  0   - one octet, no AID's bit set
 *
 * That is the truth for these harnesses: they buffer nothing and send every
 * reply immediately. It is NOT power-save support, and a station that dozes
 * will still miss frames. Actually serving a dozing peer needs a per-AID
 * bitmap and a buffer, which is out of scope (docs/ap-mode.md), and
 * tests/mt7612u_ap_onair.sh still requires power save OFF.
 *
 * `aid` is accepted so a future implementation that DOES buffer can set the
 * right bit without changing every caller; 0 means "nobody", which is the
 * only thing these harnesses can honestly advertise.
 *
 * AN AID OUTSIDE 1..7 SETS NOTHING, silently, and a caller that begins
 * buffering must notice that before relying on it: this minimum body carries
 * one bitmap octet with offset 0, so it can only page AIDs 1..7. Paging
 * anything higher needs a longer partial virtual bitmap and a non-zero
 * bitmap-control offset, i.e. a real implementation - at which point this
 * function's signature should grow a way to report refusal. It is left
 * silent rather than half-encoding, because a wrapped shift would set some
 * OTHER station's bit and tell the wrong peer to stay awake. */
inline void append_tim(std::vector<uint8_t>& m, uint8_t dtim_count = 0,
                       uint8_t dtim_period = 1, uint16_t aid = 0) {
  uint8_t tim[4];

  tim[0] = dtim_count;
  tim[1] = dtim_period;
  tim[2] = 0;   /* bitmap control: offset 0, no buffered group traffic */
  tim[3] = 0;   /* partial virtual bitmap: one octet, AIDs 1..7 */
  /* AIDs 1..7 live in bits 1..7 of this first octet. Anything larger needs a
   * longer bitmap and an offset, which this minimum form does not carry - so
   * refuse to half-encode it rather than set a bit for the wrong station. */
  if (aid >= 1 && aid <= 7)
    tim[3] = (uint8_t)(1u << aid);
  append_ie(m, kEidTim, tim, sizeof tim);
}

/* The WPA2-PSK RSN element: CCMP group, CCMP pairwise, PSK AKM.
 *
 * Both roles need byte-identical bytes here — an AP advertises it in its
 * beacon and probe response, and a station echoes the AP's choice back in its
 * association request. A station that sends something the AP did not offer is
 * refused, so there is one builder and no per-cipher path that could emit a
 * different or truncated element.
 */
inline void append_rsn_ccmp_psk(std::vector<uint8_t>& m) {
  static const uint8_t rsn[] = {
      0x01, 0x00,                          /* version 1 */
      0x00, 0x0f, 0xac, 0x04,              /* group cipher: CCMP */
      0x01, 0x00, 0x00, 0x0f, 0xac, 0x04,  /* 1 pairwise: CCMP */
      0x01, 0x00, 0x00, 0x0f, 0xac, 0x02,  /* 1 AKM: PSK */
      0x00, 0x00,                          /* RSN capabilities */
  };
  append_ie(m, kEidRsn, rsn, sizeof rsn);
}

/* Walk the IEs in `body` and return a pointer to the first with `eid`, with
 * its length in `len_out`. Returns nullptr when absent.
 *
 * Bounds-checked against a truncated or hostile frame: an element whose length
 * runs past the end of the buffer terminates the walk rather than reading off
 * it. Every caller here is parsing frames from the air. */
inline const uint8_t* find_ie(const uint8_t* body, size_t body_len, uint8_t eid,
                              size_t* len_out) {
  size_t i = 0;

  while (i + 2 <= body_len) {
    uint8_t id = body[i];
    size_t len = body[i + 1];

    if (i + 2 + len > body_len) return nullptr; /* truncated: stop, do not read */
    if (id == eid) {
      if (len_out) *len_out = len;
      return body + i + 2;
    }
    i += 2 + len;
  }
  return nullptr;
}

/* ---- parsing ---------------------------------------------------------- */

/* A parsed RSN element (802.11-2016 9.4.2.25).
 *
 * The element is VARIABLE: counts precede the suite lists, and a real
 * deployment almost never has exactly one of each. A mixed-mode WPA/WPA2 AP
 * offers TKIP and CCMP as pairwise; a WPA3-transition AP offers PSK and
 * PSK-SHA256 as AKM. Byte-comparing against a canonical one-of-each layout
 * would report both as unusable, and a station would skip BSSes it could join
 * perfectly well - so the element is parsed, never byte-compared.
 */
struct RsnInfo {
  /* Version 1, and no suite count that overruns the element. NOT "complete":
   * every field after the version is individually optional, so a version-only
   * or group-cipher-only element is `valid` with its later flags false. */
  bool valid = false;
  uint16_t version = 0;
  bool group_ccmp = false;
  bool pairwise_ccmp = false;   /* CCMP is AMONG the offered pairwise suites */
  bool akm_psk = false;         /* PSK is AMONG the offered AKMs */
  uint16_t capabilities = 0;
  bool mfp_required = false;    /* RSN capabilities bit 6 */
  bool mfp_capable = false;     /* bit 7 */
  uint16_t pairwise_count = 0;
  uint16_t akm_count = 0;
  /* THE WHOLE ELEMENT, as sets, for comparing two of them (the 12.7.6.4
   * downgrade check in Supplicant.h). A selector is its four bytes read
   * big-endian - OUI then type - and 0 means the field was absent. Bit n of a
   * mask is suite 00-0F-AC:n; a suite from any other OUI, or with a type of
   * 32 or more, sets the matching *_other flag instead. */
  uint32_t group_suite = 0;
  uint32_t pairwise_mask = 0;
  bool pairwise_other = false;
  uint32_t akm_mask = 0;
  bool akm_other = false;
  /* The group management cipher, which follows the PMKID list (802.11w).
   * Nothing here implements it; it is parsed so a difference is visible. */
  uint32_t group_mgmt_suite = 0;
};

/* Suite selectors are 4 bytes: a 3-byte OUI then a type. 00-0F-AC is the
 * 802.11 OUI; a vendor OUI is a suite we do not implement and must not
 * mistake for one we do. */
inline bool rsn_suite_is(const uint8_t* s, uint8_t type) {
  return s[0] == 0x00 && s[1] == 0x0f && s[2] == 0xac && s[3] == type;
}

inline uint32_t rsn_suite_u32(const uint8_t* s) {
  return ((uint32_t)s[0] << 24) | ((uint32_t)s[1] << 16) |
         ((uint32_t)s[2] << 8) | (uint32_t)s[3];
}

/* Add one suite to a set: a bit for an 00-0F-AC suite, the flag otherwise. */
inline void rsn_suite_note(const uint8_t* s, uint32_t* mask, bool* other) {
  if (s[0] == 0x00 && s[1] == 0x0f && s[2] == 0xac && s[3] < 32)
    *mask |= 1u << s[3];
  else
    *other = true;
}

/* Parse an RSN element body (the bytes AFTER the EID and length octets).
 *
 * Every step is bounds-checked against `len` and stops rather than reading
 * on: this is attacker-controlled input from the air. Absent trailing fields
 * are legal - an element may stop after the group cipher - and leave their
 * flags false rather than failing the parse. */
inline bool parse_rsn(const uint8_t* p, size_t len, RsnInfo* out) {
  size_t i = 0;

  if (!p || !out) return false;
  *out = RsnInfo{};
  if (len < 2) return false;
  out->version = get_le16(p);
  i = 2;
  if (out->version != 1) { *out = RsnInfo{}; return false; } /* nothing else defined */

  if (i + 4 <= len) {
    out->group_ccmp = rsn_suite_is(p + i, 4);
    out->group_suite = rsn_suite_u32(p + i);
    i += 4;
  }
  if (i + 2 <= len) {
    out->pairwise_count = get_le16(p + i);
    i += 2;
    /* A count that overruns the element is malformed, not merely unsupported.
     * Refuse rather than walk off the end. */
    if (i + (size_t)out->pairwise_count * 4 > len) { *out = RsnInfo{}; return false; }
    for (uint16_t n = 0; n < out->pairwise_count; n++, i += 4) {
      if (rsn_suite_is(p + i, 4)) out->pairwise_ccmp = true;
      rsn_suite_note(p + i, &out->pairwise_mask, &out->pairwise_other);
    }
  }
  if (i + 2 <= len) {
    out->akm_count = get_le16(p + i);
    i += 2;
    if (i + (size_t)out->akm_count * 4 > len) { *out = RsnInfo{}; return false; }
    for (uint16_t n = 0; n < out->akm_count; n++, i += 4) {
      if (rsn_suite_is(p + i, 2)) out->akm_psk = true;
      rsn_suite_note(p + i, &out->akm_mask, &out->akm_other);
    }
  }
  /* An element may legitimately stop before RSN Capabilities, and an absent
   * field means MFPR=0 - which is what hostap assumes too. Such an element is
   * accepted, deliberately, and test_parse_rsn_sets pins that direction so a
   * flip either way is visible rather than silent. An AP that really requires
   * MFP refuses the association regardless. */
  if (i + 2 <= len) {
    out->capabilities = get_le16(p + i);
    out->mfp_required = (out->capabilities & 0x0040) != 0;
    out->mfp_capable = (out->capabilities & 0x0080) != 0;
    i += 2;
  }
  /* PMKID count and list, then the group management cipher. Both optional;
   * a PMKID count that overruns is malformed, like the suite counts. */
  if (i + 2 <= len) {
    const size_t pmkids = get_le16(p + i);
    i += 2;
    if (i + pmkids * 16 > len) { *out = RsnInfo{}; return false; }
    i += pmkids * 16;
  }
  if (i + 4 <= len) {
    out->group_mgmt_suite = rsn_suite_u32(p + i);
    i += 4;
  }
  out->valid = true;
  return true;
}

/* What a station learns about a BSS from one beacon or probe response. */
struct BssInfo {
  uint8_t bssid[6] = {0};
  std::string ssid;
  uint16_t capability = 0;
  uint16_t beacon_interval_tu = 0;
  uint8_t channel = 0;     /* from the DS Parameter Set; 0 when absent or
                            * not channel_valid() */
  bool privacy = false;    /* capability bit 4 */
  bool has_rsn = false;
  bool rsn_ccmp_psk = false; /* the only suite this project speaks */
  /* RSN Capabilities bit 6 (MFPR). A BSS that REQUIRES management-frame
   * protection will refuse an association from a station that does not
   * implement 802.11w - which this project does not. Surfaced so a scan can
   * skip it, rather than associating and failing the handshake with no
   * diagnostic. */
  bool rsn_mfp_required = false;
  uint16_t rsn_capabilities = 0;
  RsnInfo rsn;  /* the whole element, for a caller that wants more than the
                 * one verdict above */
};

/* A channel number this station can tune and pick a band for: 1..14 is
 * 2.4 GHz, 32..253 is 5 GHz (freq = 5000 + 5 * chan, the extended range the
 * devourer 5 GHz synthesizer covers). Anything else - 0, 15..31, 254, 255 -
 * is not a channel, whether it came from a DS element or the receiver. No
 * regulatory check: the caller owns compliance. 6 GHz numbering overlaps
 * both ranges and is not modelled. */
inline bool channel_valid(uint8_t chan) {
  return (chan >= 1 && chan <= 14) || (chan >= 32 && chan <= 253);
}

/* An infrastructure BSS: capability ESS set and IBSS clear (802.11-2016
 * 9.4.1.4). A station's authenticate/associate exchange is meaningless
 * against an IBSS, which has no AP to answer it. */
inline bool bss_is_infrastructure(const BssInfo& b) {
  return (b.capability & 0x0001) != 0 && (b.capability & 0x0002) == 0;
}

/* A BSS this station can join with WPA2-PSK: the Privacy capability bit set
 * AND a usable RSN element (rsn_ccmp_psk). An RSN element under a clear
 * Privacy bit is self-contradictory - a conforming AP sets Privacy whenever it
 * protects data - and joining it would mean guessing which half to believe. */
inline bool bss_is_wpa2_psk(const BssInfo& b) {
  return b.privacy && b.rsn_ccmp_psk;
}

/* A beacon/probe-response body is a 12-byte fixed part (timestamp, beacon
 * interval, capability) followed by IEs. `frame` starts at the 802.11 header.
 * Returns false on anything too short to trust. */
inline bool parse_beacon(const uint8_t* frame, size_t len, BssInfo* out) {
  const size_t fixed = 24 + 12;
  const uint8_t* body;
  size_t body_len, ie_len;

  if (!frame || !out || len < fixed) return false;
  /* A scan loop reuses one BssInfo across beacons. Without this, a frame that
   * omits the SSID, DS Parameter Set or RSN element leaves the PREVIOUS BSS's
   * values in place and the caller reads them as this BSS's - including when a
   * hostile frame truncates an element so find_ie declines it. */
  *out = BssInfo{};
  std::memcpy(out->bssid, frame + 16, 6); /* addr3 */
  out->beacon_interval_tu = get_le16(frame + 24 + 8);
  out->capability = get_le16(frame + 24 + 10);
  out->privacy = (out->capability & 0x0010) != 0;

  body = frame + fixed;
  body_len = len - fixed;

  /* An SSID is at most 32 octets (802.11-2016 9.4.2.2), the same limit
   * append_ssid builds to. A longer SSID element makes the whole frame
   * malformed: it is refused, not recorded under a name no conforming
   * station could send back in an association request. */
  if (const uint8_t* p = find_ie(body, body_len, kEidSsid, &ie_len)) {
    if (ie_len > 32) {
      *out = BssInfo{};
      return false;
    }
    out->ssid.assign((const char*)p, ie_len);
  }
  /* A DS value that is not a channel (15, 255, ...) is dropped here, so the
   * caller's receive channel can stand in for it (BssTable::observe). */
  if (const uint8_t* p = find_ie(body, body_len, kEidDsParams, &ie_len))
    if (ie_len >= 1 && channel_valid(p[0])) out->channel = p[0];
  if (const uint8_t* p = find_ie(body, body_len, kEidRsn, &ie_len)) {
    out->has_rsn = true;
    parse_rsn(p, ie_len, &out->rsn);
    /* Joinable when CCMP is among the offered pairwise suites and PSK among
     * the AKMs - NOT when they are the only ones. And not when the BSS
     * requires management-frame protection, which this project does not
     * implement: better to skip it in the scan than to associate and fail the
     * handshake with no diagnostic. */
    out->rsn_ccmp_psk = out->rsn.valid && out->rsn.group_ccmp &&
                        out->rsn.pairwise_ccmp && out->rsn.akm_psk &&
                        !out->rsn.mfp_required;
    out->rsn_capabilities = out->rsn.capabilities;
    out->rsn_mfp_required = out->rsn.mfp_required;
  }
  return true;
}

/* Authentication frame body: algorithm, sequence, status. */
struct AuthFields {
  uint16_t algorithm = 0;
  uint16_t seq = 0;
  uint16_t status = 0;
};
inline bool parse_auth(const uint8_t* frame, size_t len, AuthFields* out) {
  if (!frame || !out || len < 24 + 6) return false;
  out->algorithm = get_le16(frame + 24);
  out->seq = get_le16(frame + 26);
  out->status = get_le16(frame + 28);
  return true;
}

/* Association-response body: capability, status, AID. */
struct AssocRespFields {
  uint16_t capability = 0;
  uint16_t status = 0;
  uint16_t aid = 0; /* the two top bits are always set on the wire */
};
inline bool parse_assoc_resp(const uint8_t* frame, size_t len,
                             AssocRespFields* out) {
  if (!frame || !out || len < 24 + 6) return false;
  out->capability = get_le16(frame + 24);
  out->status = get_le16(frame + 26);
  out->aid = (uint16_t)(get_le16(frame + 28) & 0x3fff);
  return true;
}

/* Deauth/disassoc reason code. */
inline bool parse_reason(const uint8_t* frame, size_t len, uint16_t* reason) {
  if (!frame || !reason || len < 24 + 2) return false;
  *reason = get_le16(frame + 24);
  return true;
}

/* ---- station-side builders -------------------------------------------- */

/* Probe request. A broadcast-SSID probe with an empty SSID element is a
 * wildcard scan; a named SSID is a directed probe, which is what finds a
 * hidden BSS. */
inline std::vector<uint8_t> build_probe_req(const uint8_t own[6],
                                            const std::string& ssid,
                                            uint8_t chan, bool five_ghz) {
  static const uint8_t bcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  std::vector<uint8_t> m = mgmt_hdr(kFcProbeReq, bcast, own, bcast);

  /* An over-length SSID must abort the build, not produce a management frame
   * that is silently missing its SSID element - which is invalid, and which a
   * peer drops without comment. */
  if (!append_ssid(m, ssid)) return {};
  /* The STATION set here too: an AP may answer a probe request based on the
   * rates it advertises, and the two requests should not claim different
   * capabilities. */
  append_supported_rates_sta(m, five_ghz);
  append_ext_supported_rates_sta(m, five_ghz);
  /* The DS Parameter Set is a 2.4 GHz element (802.11-2016 9.4.2.4); a 5 GHz
   * probe carries no channel element. */
  if (chan && !five_ghz) append_ds_params(m, chan);
  return m;
}

/* Open-system authentication, sequence 1 — the station's half. */
inline std::vector<uint8_t> build_auth_req(const uint8_t own[6],
                                           const uint8_t bssid[6]) {
  std::vector<uint8_t> m = mgmt_hdr(kFcAuth, bssid, own, bssid);
  put_le16(m, 0); /* open system */
  put_le16(m, 1); /* sequence 1 */
  put_le16(m, 0); /* status 0 */
  return m;
}

/* Association request. `capability` must claim ESS, and Privacy when the BSS
 * advertises RSN — an association request whose Privacy bit disagrees with the
 * RSN element it carries is refused by a conforming AP. */
inline std::vector<uint8_t> build_assoc_req(const uint8_t own[6],
                                            const uint8_t bssid[6],
                                            const std::string& ssid,
                                            bool rsn, bool five_ghz,
                                            uint16_t listen_interval = 10) {
  std::vector<uint8_t> m = mgmt_hdr(kFcAssocReq, bssid, own, bssid);
  put_le16(m, (uint16_t)(0x0001 | (rsn ? 0x0010 : 0))); /* ESS | Privacy */
  put_le16(m, listen_interval);
  if (!append_ssid(m, ssid)) return {};
  /* The STATION set, not the AP's: an association request that omits 6 and 12
   * Mbps can be refused with status 18 by any AP whose basic set includes
   * them. See append_supported_rates_sta. */
  append_supported_rates_sta(m, five_ghz);
  append_ext_supported_rates_sta(m, five_ghz);
  /* The RSN element goes AFTER the rates, and anything that follows it must
   * still be emitted on every cipher path: an association request that drops
   * its HT/VHT/ExtCap tail is refused or downgraded by the AP. There is no
   * tail here yet; one added later belongs below this line, not above it. */
  if (rsn) append_rsn_ccmp_psk(m);
  return m;
}

/* Deauthentication, so a station leaves cleanly instead of making the AP time
 * it out. Reason 3 = "station is leaving". */
inline std::vector<uint8_t> build_deauth(const uint8_t own[6],
                                         const uint8_t bssid[6],
                                         uint16_t reason = 3) {
  std::vector<uint8_t> m = mgmt_hdr(kFcDeauth, bssid, own, bssid);
  put_le16(m, reason);
  return m;
}

/* ---- data frames ------------------------------------------------------ */

/* LLC/SNAP header for an ethertype, the 8 bytes that precede every IP payload
 * inside an 802.11 data frame. */
inline void append_llc_snap(std::vector<uint8_t>& m, uint16_t ethertype) {
  m.insert(m.end(), {0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00});
  m.push_back((uint8_t)(ethertype >> 8));
  m.push_back((uint8_t)(ethertype & 0xff));
}

/* A station's uplink data header: to-DS, addr1 = BSSID, addr2 = own,
 * addr3 = destination. The AP's downlink is the mirror (from-DS, addr1 = sta,
 * addr2 = bssid, addr3 = source), which is why this takes a direction rather
 * than being two near-identical functions. */
inline std::vector<uint8_t> data_hdr_to_ds(const uint8_t bssid[6],
                                           const uint8_t own[6],
                                           const uint8_t dest[6],
                                           bool protect, uint16_t seq = 0) {
  std::vector<uint8_t> m;
  m.reserve(24);
  m.push_back(kFcData);
  m.push_back((uint8_t)(kFcToDs | (protect ? kFcProtected : 0)));
  put_le16(m, 0);
  m.insert(m.end(), bssid, bssid + 6);
  m.insert(m.end(), own, own + 6);
  m.insert(m.end(), dest, dest + 6);
  put_le16(m, (uint16_t)((seq & 0x0fff) << 4));
  return m;
}

inline std::vector<uint8_t> data_hdr_from_ds(const uint8_t sta[6],
                                             const uint8_t bssid[6],
                                             const uint8_t src[6],
                                             bool protect, uint16_t seq = 0) {
  std::vector<uint8_t> m;
  m.reserve(24);
  m.push_back(kFcData);
  m.push_back((uint8_t)(kFcFromDs | (protect ? kFcProtected : 0)));
  put_le16(m, 0);
  m.insert(m.end(), sta, sta + 6);
  m.insert(m.end(), bssid, bssid + 6);
  m.insert(m.end(), src, src + 6);
  put_le16(m, (uint16_t)((seq & 0x0fff) << 4));
  return m;
}

/* True for EVERY QoS data subtype, not just QoS Data itself.
 *
 * Data frames are type 2 (fc0 bits 3:2 == 10) and the QoS subtypes are those
 * with bit 7 set - QoS Data, QoS Null, QoS Data+CF-Ack and the rest. An exact
 * `fc0 == 0x88` test misses QoS Null (0xc8), which a real station sends, and
 * then reads its body two bytes early. */
inline bool is_qos_data(uint8_t fc0) { return (fc0 & 0x8c) == 0x88; }

/* Bytes before the frame body: 24 base, +2 for the QoS Control field, +6 for a
 * 4-address frame, +4 for HT Control when the Order bit is set
 * (802.11-2016 9.2.4.1.10). Getting this wrong reads the LLC header at the
 * wrong offset and silently drops the frame. */
/* ------------------------------------------------------- 802.11 <-> 802.3
 *
 * The translation anything that bridges a station's data plane to an
 * Ethernet-shaped interface needs (a TAP device, or an in-process stack with
 * no netdev at all). NOTHING IN THIS TREE CALLS IT YET; it is here, tested,
 * so that such callers share one conversion instead of each writing their
 * own. `append_llc_snap` above encodes the 8-byte LLC/SNAP header and
 * tests/ap_responder.cpp decodes it inline, but neither builds or parses the
 * 14-byte Ethernet II header, which is the actual work.
 *
 * An 802.11 MSDU is LLC/SNAP(8) + payload, and its addresses live in the
 * 802.11 header. An Ethernet II frame is DA(6) + SA(6) + ethertype(2) +
 * payload. So the conversion moves addresses in from outside, and the
 * ethertype up out of the SNAP header.
 *
 * Both directions return 0 rather than truncating or guessing. A silent
 * truncation here would produce a frame that looks well-formed and decodes to
 * nonsense at the far end. */
inline constexpr size_t kEthHdrLen = 14;
inline constexpr size_t kLlcSnapLen = 8;

/* True for the exact SNAP header 802.11 uses to carry an ethertype:
 * AA AA 03 with a zero OUI. Anything else is a payload this translation has
 * no business rewriting - other LLC encodings exist and carry no ethertype at
 * bytes 6..7. */
inline bool is_ethertype_snap(const uint8_t* msdu, size_t len) {
  return len >= kLlcSnapLen && msdu[0] == 0xaa && msdu[1] == 0xaa &&
         msdu[2] == 0x03 && msdu[3] == 0x00 && msdu[4] == 0x00 &&
         msdu[5] == 0x00;
}

/* MSDU -> Ethernet II. Returns bytes written into `out`, or 0. */
inline size_t msdu_to_eth(const uint8_t da[6], const uint8_t sa[6],
                          const uint8_t* msdu, size_t msdu_len,
                          uint8_t* out, size_t out_cap) {
  if (!da || !sa || !msdu || !out) return 0;
  if (!is_ethertype_snap(msdu, msdu_len)) return 0;
  const size_t payload = msdu_len - kLlcSnapLen;
  if (out_cap < kEthHdrLen + payload) return 0;
  std::memcpy(out, da, 6);
  std::memcpy(out + 6, sa, 6);
  out[12] = msdu[6];                 /* ethertype, straight out of the SNAP */
  out[13] = msdu[7];
  std::memcpy(out + kEthHdrLen, msdu + kLlcSnapLen, payload);
  return kEthHdrLen + payload;
}

/* Ethernet II -> MSDU. The addresses come OUT through `out_da`/`out_sa`,
 * because the caller needs them for the 802.11 header, not for the MSDU.
 * Returns bytes written into `out`, or 0. */
inline size_t eth_to_msdu(const uint8_t* eth, size_t eth_len,
                          uint8_t* out, size_t out_cap,
                          uint8_t out_da[6], uint8_t out_sa[6]) {
  if (!eth || !out) return 0;
  if (eth_len < kEthHdrLen) return 0;
  const size_t payload = eth_len - kEthHdrLen;
  if (out_cap < kLlcSnapLen + payload) return 0;
  if (out_da) std::memcpy(out_da, eth, 6);
  if (out_sa) std::memcpy(out_sa, eth + 6, 6);
  out[0] = 0xaa; out[1] = 0xaa; out[2] = 0x03;
  out[3] = 0x00; out[4] = 0x00; out[5] = 0x00;
  out[6] = eth[12];
  out[7] = eth[13];
  std::memcpy(out + kLlcSnapLen, eth + kEthHdrLen, payload);
  return kLlcSnapLen + payload;
}

/* Direction-aware addressing — 802.11-2016 Table 9-26.
 *
 * An AP that relays needs the DESTINATION of a frame, and the destination is
 * not in a fixed place: it is addr1 when the frame comes from the DS and addr3
 * when it goes to the DS. A receive path that reads only addr1 and addr2
 * assumes the frame is for the AP itself, which is true right up until the AP
 * has a second station to forward to.
 *
 *   ToDS FromDS | addr1   addr2   addr3   addr4
 *     0    0    | DA      SA      BSSID   -        (IBSS)
 *     0    1    | DA      BSSID   SA      -        (from the DS)
 *     1    0    | BSSID   SA      DA      -        (to the DS)
 *     1    1    | RA      TA      DA      SA       (4-address / WDS)
 *
 * `hdr` must be at least data_hdr_len() bytes; for the 4-address case that is
 * 30, and data_sa() reads addr4 at offset 24. */
inline const uint8_t* data_da(const uint8_t* hdr, uint8_t fc1) {
  const bool to_ds = (fc1 & kFcToDs) != 0;
  /* DA is addr1 unless the frame is going TO the DS, where addr1 is the
   * BSSID and the real destination sits in addr3. */
  return to_ds ? hdr + 16 : hdr + 4;
}

inline const uint8_t* data_sa(const uint8_t* hdr, uint8_t fc1) {
  const bool to_ds = (fc1 & kFcToDs) != 0;
  const bool from_ds = (fc1 & kFcFromDs) != 0;
  if (to_ds && from_ds) return hdr + 24;   /* addr4 */
  if (from_ds) return hdr + 16;            /* addr3 */
  return hdr + 10;                         /* addr2 */
}

/* True when this frame's final destination is a group address. Note this is
 * NOT the same question as "is this frame group-addressed", which is about
 * addr1/RA and decides which key protects it: a station's broadcast ARP goes
 * out as an individually addressed frame to the AP with a group DA in addr3. */
inline bool data_da_is_group(const uint8_t* hdr, uint8_t fc1) {
  return (data_da(hdr, fc1)[0] & 0x01) != 0;
}

inline size_t data_hdr_len(uint8_t fc0, uint8_t fc1) {
  size_t n = 24;
  if (is_qos_data(fc0)) n += 2;
  if ((fc1 & (kFcToDs | kFcFromDs)) == (kFcToDs | kFcFromDs)) n += 6; /* 4-addr */
  /* HT Control rides only QoS data frames; the Order bit means something else
   * on a non-QoS frame and must not add four bytes there. */
  if (is_qos_data(fc0) && (fc1 & 0x80)) n += 4;
  return n;
}

}  // namespace sta
}  // namespace devourer

#endif /* DEVOURER_STA_DOT11_H */
