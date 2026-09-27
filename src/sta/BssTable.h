/* BssTable — what a scan found, and which of it is worth joining.
 *
 * A station hears the same BSS dozens of times a second and several BSSes at
 * once. This collapses that stream into one record per BSSID, keeps the
 * freshest view of each, and answers the only question the association state
 * machine actually asks: given an SSID, which BSS should I join?
 *
 * Fixed capacity, and it takes frames straight off the air — every field
 * comes from `parse_beacon`, which bounds-checks everything and resets its
 * output so a frame that omits an element cannot leave the previous BSS's
 * value in place.
 *
 * NOT allocation-free: `BssInfo::ssid` is a std::string, so an SSID longer
 * than the short-string buffer heap-allocates on every beacon that carries
 * it. It is a handful of allocations a second on a busy channel, which is
 * nothing next to the CCMP work a station does per frame.
 *
 * WHAT IT DELIBERATELY DOES NOT DO. It does not scan: it has no notion of
 * channels, dwell times or probe requests, because those need a radio and this
 * file is meant to be testable without one. A scanner drives the radio and
 * feeds the frames here.
 */
#ifndef DEVOURER_STA_BSS_TABLE_H
#define DEVOURER_STA_BSS_TABLE_H

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

#include "sta/Dot11.h"

namespace devourer {
namespace sta {

struct BssEntry {
  BssInfo info;
  /* The most recent RSSI in dBm. Signed and initialised to the floor rather
   * than to zero: zero dBm is an enormous signal, and an entry that has never
   * been given an RSSI must not win a comparison against one that has. */
  int8_t rssi = -128;
  uint32_t last_seen_ms = 0;
  uint32_t frames = 0;      /* beacons and probe responses that fed this entry */
};

class BssTable {
 public:
  /* Sixteen. A crowded band shows more than that, but this exists to find one
   * named network, not to render a survey, and the eviction rule below keeps
   * the useful entries. */
  static constexpr int kMaxBss = 16;

  static constexpr int capacity() { return kMaxBss; }
  int count() const { return count_; }

  /* The network this station is looking for.
   *
   * WITHOUT IT THE EVICTION RULE IS AN ATTACK. The victim is the entry heard
   * from longest ago, and a real AP beacons roughly ten times a second - so a
   * neighbour (or an attacker) emitting beacons for sixteen fabricated BSSIDs
   * as fast as the radio allows keeps every fabricated entry at age zero and
   * makes the GENUINE AP the oldest entry, every time. The table thrashes and
   * select() returns nothing for most of the window in which the station is
   * trying to associate.
   *
   * An entry whose SSID matches this is never evicted. Empty by default, so a
   * caller that does not set it gets the old behaviour and the old exposure. */
  void set_wanted(const std::string& ssid) { wanted_ = ssid; }
  const std::string& wanted() const { return wanted_; }

  void clear() {
    for (int i = 0; i < kMaxBss; i++) used_[i] = false;
    count_ = 0;
  }

  const BssEntry* at(int i) const {
    if (i < 0 || i >= kMaxBss || !used_[i]) return nullptr;
    return &slots_[i];
  }

  const BssEntry* find(const uint8_t bssid[6]) const {
    const int i = index_of(bssid);
    return i < 0 ? nullptr : &slots_[i];
  }

  /* Fold one beacon or probe response into the table.
   *
   * Returns the entry, or null when the frame is not one of those two
   * subtypes or does not parse. THE SUBTYPE CHECK IS NOT COSMETIC: every
   * management frame has the same 24-byte header, so handing a deauth or an
   * association response to parse_beacon reads its fixed fields as a
   * timestamp and a capability and invents a BSS out of them.
   *
   * `rx_channel` IS THE CHANNEL THE RADIO WAS ON when it received the frame
   * (0 = unknown). It is required because the beacon cannot always say: the
   * DS Parameter Set is a 2.4 GHz element and many 5 GHz beacons omit it, and
   * without a channel the association request would be built with the
   * 2.4 GHz rate set, which a 5 GHz AP refuses. When the frame DOES carry a DS
   * element
   * and it disagrees, the DS element wins: on 2.4 GHz a receiver hears the
   * adjacent overlapping channels, and the AP's own statement of where it is
   * beats where we happened to hear it. Both sources must pass
   * channel_valid() (Dot11.h); an invalid DS value falls back to a valid
   * receive channel. An entry with no valid channel from either source is
   * kept (it still shows up in find()) but select() never offers it.
   */
  const BssEntry* observe(const uint8_t* frame, size_t len, int8_t rssi,
                          uint8_t rx_channel, uint32_t now_ms) {
    BssInfo info;

    if (!frame || len < 24) return nullptr;
    if (frame[0] != kFcBeacon && frame[0] != kFcProbeResp) return nullptr;
    if (!parse_beacon(frame, len, &info)) return nullptr;
    /* parse_beacon has already dropped a DS value that is not a channel; the
     * receive channel stands in only if it is one itself (channel_valid). */
    if (info.channel == 0 && channel_valid(rx_channel)) info.channel = rx_channel;

    int i = index_of(info.bssid);
    /* allocate() evicts rather than refusing - even when every slot is
     * protected - so it always yields a slot. */
    if (i < 0) i = allocate(now_ms);

    const uint32_t seen = used_[i] ? slots_[i].frames : 0;
    const bool fresh = !used_[i];

    /* A HIDDEN BSS beacons an empty (or all-zero) SSID, and only a directed
     * probe response names it. Replacing the whole entry on every beacon
     * would wipe that name within a beacon interval or two - long before
     * select() is next asked - so the hidden-BSS join a directed probe exists
     * for would essentially never happen. Keep the name this BSSID has
     * already been heard with. */
    if (!fresh && !slots_[i].info.ssid.empty()) {
      bool hidden = true;
      for (char c : info.ssid)
        if (c != 0) { hidden = false; break; }
      if (hidden) info.ssid = slots_[i].info.ssid;
    }

    if (fresh) {
      used_[i] = true;
      count_++;
      slots_[i] = BssEntry{};
    }
    slots_[i].info = info;
    slots_[i].rssi = rssi;
    slots_[i].last_seen_ms = now_ms;
    slots_[i].frames = seen + 1;
    return &slots_[i];
  }

  /* Drop entries not heard from in `max_age_ms`.
   *
   * A station that keeps a BSS forever will happily try to associate with one
   * that went off the air ten minutes ago, and then report "no response" as
   * though the AP were broken. Returns how many were dropped.
   */
  int expire(uint32_t now_ms, uint32_t max_age_ms) {
    int dropped = 0;

    for (int i = 0; i < kMaxBss; i++) {
      if (!used_[i]) continue;
      /* Unsigned subtraction, so a now_ms that has wrapped past an entry's
       * timestamp yields a huge age and expires it, rather than a negative
       * one that never does. */
      if ((uint32_t)(now_ms - slots_[i].last_seen_ms) >= max_age_ms) {
        used_[i] = false;
        count_--;
        dropped++;
      }
    }
    return dropped;
  }

  /* The BSS to join for this SSID: the strongest one this station can
   * actually speak to.
   *
   * "Can speak to" is bss_is_wpa2_psk (Dot11.h): the Privacy bit set and
   * `rsn_ccmp_psk`, which parse_beacon computes as CCMP
   * among the pairwise suites, PSK among the AKMs, and management-frame
   * protection NOT required — 802.11w is unimplemented here, and a BSS that
   * requires it will refuse the association after a successful authentication,
   * which is a confusing place to fail. Skipping it in selection turns that
   * into "no candidate".
   *
   * Ties break on the most recently heard, so a stale entry never beats a live
   * one at equal signal.
   */
  const BssEntry* select(const std::string& ssid) const {
    return best_matching(ssid, /*want_rsn=*/true);
  }

  /* The same question for a station configured for an OPEN network.
   *
   * Two named entry points rather than one `require_rsn` argument, so a
   * caller's intent is in the name it calls. The loop is shared, so the two
   * cannot drift in the tie-break or the eviction-safe iteration.
   *
   * "Open" is `!privacy`, not `!has_rsn`. A WEP BSS carries no RSN element
   * and is not joinable by a station with no keys, and WPA1 lives in a vendor
   * element this parser does not read at all - the Privacy capability bit is
   * the one test that covers every protected BSS. StationSm::join refuses the
   * same way, so a hand-picked entry gets the same answer as a selected one.
   */
  const BssEntry* select_open(const std::string& ssid) const {
    return best_matching(ssid, /*want_rsn=*/false);
  }

 private:
  /* The body both selectors share. `want_rsn` picks the joinability test:
   * WPA2-PSK-CCMP without MFP required, or no encryption at all. */
  const BssEntry* best_matching(const std::string& ssid, bool want_rsn) const {
    const BssEntry* best = nullptr;

    for (int i = 0; i < kMaxBss; i++) {
      if (!used_[i]) continue;
      const BssEntry& e = slots_[i];
      if (e.info.ssid != ssid) continue;
      if (want_rsn ? !bss_is_wpa2_psk(e.info) : e.info.privacy) continue;
      /* No channel from the beacon or the receiver: the association request
       * would be built for the wrong band. Not a candidate. */
      if (!channel_valid(e.info.channel)) continue;
      /* An IBSS (or a frame claiming neither ESS nor IBSS) has no AP to
       * authenticate or associate with. */
      if (!bss_is_infrastructure(e.info)) continue;
      /* "More recent" is a SIGNED difference, not `>` on the raw stamps:
       * the millisecond clock wraps every ~49.7 days, and a raw comparison
       * across the wrap picks the stale entry. Correct while the two are
       * within ~24.8 days of each other, which any practical expire() age
       * keeps true. */
      if (!best || e.rssi > best->rssi ||
          (e.rssi == best->rssi &&
           (int32_t)(e.last_seen_ms - best->last_seen_ms) > 0))
        best = &e;
    }
    return best;
  }

  int index_of(const uint8_t bssid[6]) const {
    for (int i = 0; i < kMaxBss; i++)
      if (used_[i] && std::memcmp(slots_[i].info.bssid, bssid, 6) == 0)
        return i;
    return -1;
  }

  /* A free slot, or the one worth losing.
   *
   * REFUSING WHEN FULL IS THE WRONG ANSWER. Sixteen neighbours heard before
   * the target network would make the table permanently useless, and a scan
   * in a block of flats hits that in a second. The victim is the entry heard
   * from longest ago, with the weakest signal breaking a tie — the two things
   * that make a BSS least likely to be the one being looked for.
   */
  bool protected_slot(int i) const {
    return !wanted_.empty() && slots_[i].info.ssid == wanted_;
  }

  int allocate(uint32_t now_ms) {
    for (int i = 0; i < kMaxBss; i++)
      if (!used_[i]) return i;

    int victim = -1;
    for (int i = 0; i < kMaxBss; i++) {
      if (protected_slot(i)) continue;
      if (victim < 0) { victim = i; continue; }
      const uint32_t age_v = (uint32_t)(now_ms - slots_[victim].last_seen_ms);
      const uint32_t age_i = (uint32_t)(now_ms - slots_[i].last_seen_ms);

      if (age_i > age_v ||
          (age_i == age_v && slots_[i].rssi < slots_[victim].rssi))
        victim = i;
    }
    /* EVERY SLOT IS PROTECTED. Refusing here would turn the anti-flood rule
     * into the attack it exists to prevent: sixteen
     * beacons for sixteen fabricated BSSIDs, all carrying the WANTED SSID,
     * fill the table with unevictable entries and the genuine AP can then
     * never be inserted at all. select() returns only the attacker's BSSes,
     * for as long as they keep beaconing.
     *
     * So fall back to the ordinary victim rule over every slot. The
     * protection still does its job in the case it exists for - a flood
     * of OTHER SSIDs cannot evict the network being looked for - and in the
     * degenerate case where every entry claims to be the wanted network,
     * this table cannot tell which one is real and staying fresh beats
     * staying stuck. */
    if (victim < 0) {
      for (int i = 0; i < kMaxBss; i++) {
        if (victim < 0) { victim = i; continue; }
        const uint32_t age_v = (uint32_t)(now_ms - slots_[victim].last_seen_ms);
        const uint32_t age_i = (uint32_t)(now_ms - slots_[i].last_seen_ms);

        if (age_i > age_v ||
            (age_i == age_v && slots_[i].rssi < slots_[victim].rssi))
          victim = i;
      }
    }
    /* victim >= 0 here: the table is full and the fallback considers every
     * slot. */
    used_[victim] = false;
    count_--;
    return victim;
  }

  BssEntry slots_[kMaxBss];
  bool used_[kMaxBss] = {false};
  int count_ = 0;
  std::string wanted_;
};

}  // namespace sta
}  // namespace devourer

#endif /* DEVOURER_STA_BSS_TABLE_H */
