/* Headless guard for src/sta/BssTable.h — what a scan found, and which of it
 * is worth joining.
 *
 * Pure and header-only like the rest of src/sta/, so this needs neither
 * OpenSSL nor libusb and runs in every CI job.
 *
 * The load-bearing cells are the two that decide whether a station can
 * connect at all: `select()` must pick a BSS this station can actually finish
 * a handshake with, and `observe()` must not invent one out of a frame that
 * is not a beacon. Both are assertions about behaviour a caller depends on,
 * not about the shape of the struct.
 */
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "sta/BssTable.h"

namespace {

using devourer::sta::BssEntry;
using devourer::sta::BssTable;

int g_fail = 0;

void check(bool ok, const char* what) {
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    g_fail++;
  }
}

/* A beacon as a real AP airs one: the 24-byte header, the 12-byte fixed body,
 * then the elements in the standard's order. Built with Dot11.h's own
 * builders, so a change to those shows up here. */
std::vector<uint8_t> beacon(const uint8_t bssid[6], const std::string& ssid,
                            uint8_t chan, bool rsn, bool privacy = true,
                            bool ds = true) {
  static const uint8_t bcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  std::vector<uint8_t> m = devourer::sta::mgmt_hdr(devourer::sta::kFcBeacon,
                                                   bcast, bssid, bssid);

  m.insert(m.end(), 8, 0);                       /* timestamp */
  devourer::sta::put_le16(m, 100);               /* beacon interval, TU */
  devourer::sta::put_le16(m,
                          (uint16_t)(0x0001 | (privacy && rsn ? 0x0010 : 0)));
  devourer::sta::append_ssid(m, ssid);
  devourer::sta::append_supported_rates(m);
  if (ds) devourer::sta::append_ds_params(m, chan);
  if (rsn) devourer::sta::append_rsn_ccmp_psk(m);
  return m;
}

void test_observe_and_dedupe() {
  BssTable t;
  const uint8_t a[6] = {0x02, 0, 0, 0, 0, 0x01};
  const uint8_t b[6] = {0x02, 0, 0, 0, 0, 0x02};

  check(t.count() == 0, "a fresh table is empty");

  std::vector<uint8_t> f = beacon(a, "one", 6, true);
  const BssEntry* e = t.observe(f.data(), f.size(), -40, 6, 1000);
  check(e != nullptr, "a beacon is observed");
  check(t.count() == 1, "...as one entry");
  check(e->info.ssid == "one" && e->info.channel == 6, "...parsed");
  check(e->info.rsn_ccmp_psk, "...and recognised as WPA2-PSK/CCMP");
  check(e->rssi == -40 && e->last_seen_ms == 1000 && e->frames == 1,
        "...with its RSSI, timestamp and frame count");

  /* THE SAME BSS AGAIN IS NOT A SECOND ENTRY. Without this a station hears a
   * beacon ten times a second and fills a sixteen-slot table in under two
   * seconds, then evicts the network it was looking for. */
  e = t.observe(f.data(), f.size(), -55, 6, 1100);
  check(t.count() == 1, "the same BSSID does not create a second entry");
  check(e->rssi == -55 && e->last_seen_ms == 1100 && e->frames == 2,
        "...it refreshes the existing one");

  std::vector<uint8_t> g = beacon(b, "two", 11, true);
  t.observe(g.data(), g.size(), -70, 6, 1200);
  check(t.count() == 2, "a different BSSID is a second entry");
  check(t.find(a) != nullptr && t.find(b) != nullptr, "both are findable");
  const uint8_t absent[6] = {0x02, 0, 0, 0, 0, 0x09};
  check(t.find(absent) == nullptr, "an unheard BSSID is not");
}

/* A frame that is not a beacon or probe response must be refused. Every
 * management frame shares the same 24-byte header, so handing a deauth to
 * parse_beacon reads its reason code as part of a timestamp and invents a
 * BSS. */
void test_only_beacons_are_observed() {
  BssTable t;
  const uint8_t a[6] = {0x02, 0, 0, 0, 0, 0x01};

  /* A 26-byte deauth is refused on LENGTH, not on subtype - parse_beacon
   * needs 36 - so this arm alone would survive deleting the subtype check.
   * Kept because a short frame must also be refused, and followed by a
   * FULL-LENGTH frame whose only fault is its subtype. */
  std::vector<uint8_t> d =
      devourer::sta::build_deauth(a, a, 7);
  check(t.observe(d.data(), d.size(), -40, 6, 1) == nullptr,
        "a short deauth is not observed as a BSS");

  std::vector<uint8_t> longd = beacon(a, "one", 6, true);
  longd[0] = devourer::sta::kFcDeauth;
  check(t.observe(longd.data(), longd.size(), -40, 6, 1) == nullptr,
        "a FULL-LENGTH deauth is not observed either - this is the arm that "
        "tests the subtype rather than the length");

  std::vector<uint8_t> f = beacon(a, "one", 6, true);
  f[0] = devourer::sta::kFcAssocResp;
  check(t.observe(f.data(), f.size(), -40, 6, 1) == nullptr,
        "an association response is not observed as a BSS");

  f[0] = devourer::sta::kFcProbeResp;
  check(t.observe(f.data(), f.size(), -40, 6, 1) != nullptr,
        "a probe response IS observed - same body layout as a beacon");
  check(t.count() == 1, "...into one entry");

  /* A runt. parse_beacon refuses anything shorter than its fixed part, and
   * observe must not reach it with a 24-byte buffer either. */
  check(t.observe(f.data(), 24, -40, 6, 1) == nullptr, "a runt is refused");
  check(t.observe(f.data(), 10, -40, 6, 1) == nullptr, "a 10-byte frame is refused");
  check(t.observe(nullptr, 100, -40, 6, 1) == nullptr, "null is refused");
}

void test_expire() {
  BssTable t;
  const uint8_t a[6] = {0x02, 0, 0, 0, 0, 0x01};
  const uint8_t b[6] = {0x02, 0, 0, 0, 0, 0x02};

  std::vector<uint8_t> f = beacon(a, "one", 6, true);
  std::vector<uint8_t> g = beacon(b, "two", 6, true);
  t.observe(f.data(), f.size(), -40, 6, 1000);
  t.observe(g.data(), g.size(), -40, 6, 5000);

  check(t.expire(5100, 10000) == 0, "nothing expires before its age");
  check(t.count() == 2, "...and the table is unchanged");
  check(t.expire(6000, 2000) == 1, "the older entry expires");
  check(t.count() == 1 && t.find(a) == nullptr && t.find(b) != nullptr,
        "...and it is the right one");

  /* A station that keeps a BSS forever will try to associate with one that
   * went off the air ten minutes ago and then report the AP as broken. */
  check(t.expire(100000, 2000) == 1, "everything stale expires");
  check(t.count() == 0, "...leaving nothing");

  /* THE CLOCK WRAPS. `now_ms` is a uint32_t, so about every fifty days it
   * passes zero, and the subtraction has to be unsigned.
   *
   * A SMALL AGE ACROSS THE WRAP DOES NOT DISTINGUISH THE TWO: 0x00001000 -
   * 0xfffff000 is 0x2000 either way, positive in both readings, so a cell
   * that only crosses the wrap cannot tell an int32_t age from a uint32_t
   * one. The difference only appears once the age passes 2^31 ms - about
   * twenty-five days - where the signed reading goes NEGATIVE and the entry
   * never expires again; the second check below is that case. */
  t.observe(f.data(), f.size(), -40, 6, 0xfffff000u);
  check(t.count() == 1, "an entry seen just before the wrap");
  check(t.expire(0x00001000u, 2000) == 1, "expires just after the wrap");

  t.observe(f.data(), f.size(), -40, 6, 0);
  check(t.expire(0x90000000u, 2000) == 1,
        "an entry twenty-eight days old expires - the age must be UNSIGNED, "
        "or it reads as negative and nothing is ever dropped again");
  check(t.count() == 0, "...leaving the table empty");
}

void test_select() {
  BssTable t;
  const uint8_t weak[6] = {0x02, 0, 0, 0, 0, 0x01};
  const uint8_t strong[6] = {0x02, 0, 0, 0, 0, 0x02};
  const uint8_t open_bss[6] = {0x02, 0, 0, 0, 0, 0x03};
  const uint8_t other[6] = {0x02, 0, 0, 0, 0, 0x04};

  std::vector<uint8_t> f1 = beacon(weak, "net", 6, true);
  std::vector<uint8_t> f2 = beacon(strong, "net", 36, true);
  std::vector<uint8_t> f3 = beacon(open_bss, "net", 1, false, false);
  std::vector<uint8_t> f4 = beacon(other, "elsewhere", 6, true);
  t.observe(f1.data(), f1.size(), -80, 6, 1000);
  t.observe(f2.data(), f2.size(), -35, 6, 1000);
  t.observe(f3.data(), f3.size(), -20, 6, 1000);   /* strongest, and unusable */
  t.observe(f4.data(), f4.size(), -10, 6, 1000);   /* strongest, wrong SSID */

  const BssEntry* s = t.select("net");
  check(s != nullptr, "a candidate is found");
  check(s && std::memcmp(s->info.bssid, strong, 6) == 0,
        "the STRONGEST joinable BSS wins");
  check(s && s->info.channel == 36, "...and carries its channel");

  /* THE THREE NEGATIVE ARMS, each of which the positive one would hide. */
  check(t.select("absent") == nullptr, "an SSID nobody airs has no candidate");
  const BssEntry* o = t.find(open_bss);
  check(o && !o->info.rsn_ccmp_psk,
        "the open BSS is in the table but not joinable");
  check(s && std::memcmp(s->info.bssid, open_bss, 6) != 0,
        "...so it is not selected despite the strongest signal");
  check(s && std::memcmp(s->info.bssid, other, 6) != 0,
        "a stronger BSS with another SSID is not selected");

  /* A tie breaks on the most recently heard, so a stale entry never beats a
   * live one at equal signal. */
  BssTable u;
  const uint8_t p[6] = {0x02, 0, 0, 0, 0, 0x0a};
  const uint8_t q[6] = {0x02, 0, 0, 0, 0, 0x0b};
  std::vector<uint8_t> g1 = beacon(p, "tie", 6, true);
  std::vector<uint8_t> g2 = beacon(q, "tie", 6, true);
  u.observe(g1.data(), g1.size(), -50, 6, 1000);
  u.observe(g2.data(), g2.size(), -50, 6, 2000);
  const BssEntry* w = u.select("tie");
  check(w && std::memcmp(w->info.bssid, q, 6) == 0,
        "at equal signal the fresher entry wins");
}

/* 802.11w is not implemented here. A BSS that REQUIRES management-frame
 * protection will accept the authentication and then refuse the association,
 * which is a confusing place to fail; skipping it in selection turns that
 * into an honest "no candidate". */
void test_mfp_required_is_skipped() {
  BssTable t;
  const uint8_t a[6] = {0x02, 0, 0, 0, 0, 0x01};
  std::vector<uint8_t> f = beacon(a, "net", 6, true);

  /* Set RSN capabilities bit 6 (MFPR) in the element the builder emitted.
   * The capabilities field is the last two octets of that element. */
  size_t ie_len = 0;
  const uint8_t* rsn = devourer::sta::find_ie(f.data() + 36, f.size() - 36,
                                              devourer::sta::kEidRsn, &ie_len);
  check(rsn != nullptr && ie_len >= 2, "the beacon carries an RSN element");
  if (!rsn) return;
  uint8_t* caps = const_cast<uint8_t*>(rsn) + ie_len - 2;
  caps[0] = (uint8_t)(caps[0] | 0x40);

  const BssEntry* e = t.observe(f.data(), f.size(), -30, 6, 1000);
  check(e && e->info.rsn_mfp_required, "MFPR is parsed out of the beacon");
  check(e && !e->info.rsn_ccmp_psk, "...and makes the BSS unjoinable");
  check(t.select("net") == nullptr, "...so selection finds no candidate");
}

/* Full is not the same as broken. Sixteen neighbours heard before the target
 * network would make the table permanently useless, and a scan in a block of
 * flats hits that in a second. */
void test_eviction_when_full() {
  BssTable t;
  const int cap = BssTable::capacity();

  for (int i = 0; i < cap; i++) {
    const uint8_t id[6] = {0x02, 0, 0, 0, 0, (uint8_t)(0x10 + i)};
    std::vector<uint8_t> f = beacon(id, "filler", 6, true);
    /* Ascending timestamps, so slot 0 is the oldest. */
    t.observe(f.data(), f.size(), -50, 6, (uint32_t)(1000 + i));
  }
  check(t.count() == cap, "the table fills to capacity");

  const uint8_t oldest[6] = {0x02, 0, 0, 0, 0, 0x10};
  const uint8_t wanted[6] = {0x02, 0, 0, 0, 0, 0x77};
  std::vector<uint8_t> f = beacon(wanted, "target", 6, true);
  const BssEntry* e = t.observe(f.data(), f.size(), -30, 6, 2000);

  check(e != nullptr, "a new BSS is still admitted when the table is full");
  check(t.find(wanted) != nullptr, "...and is findable");
  check(t.find(oldest) == nullptr, "...having evicted the least recently heard");
  check(t.count() == cap, "...with the count unchanged");
  check(t.select("target") != nullptr,
        "the network being looked for survives a crowded band");

  t.clear();
  check(t.count() == 0 && t.find(wanted) == nullptr, "clear() empties it");
}

/* THE EVICTION RULE IS AN ATTACK WITHOUT set_wanted().
 *
 * The victim is the entry heard from longest ago. A real AP beacons about ten
 * times a second, so its age is nearly always zero-ish - but an attacker
 * emitting beacons for sixteen fabricated BSSIDs as fast as the radio allows
 * keeps every fabricated entry at age zero and makes the GENUINE AP the
 * oldest, every time. The table thrashes and select() returns nothing for
 * most of the window in which the station is trying to associate. */
void test_wanted_ssid_survives_a_flood() {
  BssTable t;
  const uint8_t target[6] = {0x02, 0, 0, 0, 0, 0x77};
  std::vector<uint8_t> want = beacon(target, "target", 6, true);

  /* THE NEGATIVE ARM FIRST, so the positive one is not a coincidence: with no
   * wanted SSID set, the flood evicts the network being looked for. */
  t.observe(want.data(), want.size(), -30, 6, 1000);
  for (int i = 0; i < BssTable::capacity() * 2; i++) {
    const uint8_t id[6] = {0x02, 0, 0, 0, 1, (uint8_t)i};
    std::vector<uint8_t> f = beacon(id, "flood", 6, true);
    t.observe(f.data(), f.size(), -30, 6, (uint32_t)(2000 + i));
  }
  check(t.select("target") == nullptr,
        "unprotected, a flood evicts the wanted network");

  /* And with it set, the same flood cannot touch it. */
  BssTable u;
  u.set_wanted("target");
  u.observe(want.data(), want.size(), -30, 6, 1000);
  for (int i = 0; i < BssTable::capacity() * 4; i++) {
    const uint8_t id[6] = {0x02, 0, 0, 0, 1, (uint8_t)i};
    std::vector<uint8_t> f = beacon(id, "flood", 6, true);
    u.observe(f.data(), f.size(), -30, 6, (uint32_t)(2000 + i));
  }
  check(u.select("target") != nullptr,
        "with set_wanted(), the flood cannot evict it");
  check(u.find(target) != nullptr, "...and it is still findable by BSSID");
  check(u.count() == BssTable::capacity(),
        "...and the table is still full of the flood, as it should be");

  /* The protection is not a leak: a second BSS airing the wanted SSID is
   * admitted, because that is a real roaming candidate. */
  const uint8_t second[6] = {0x02, 0, 0, 0, 0, 0x78};
  std::vector<uint8_t> also = beacon(second, "target", 36, true);
  u.observe(also.data(), also.size(), -20, 6, 9000);
  check(u.find(second) != nullptr, "a second BSS for the wanted SSID is admitted");
  check(u.find(target) != nullptr, "...without evicting the first");
}

/* select_open() - the same question for a station with no keys.
 *
 * The two selectors must disagree about every BSS: one that offers WPA2-PSK
 * is useless to an open station and vice versa. A single function answering
 * both would make the `open` cell of the on-air harness pass against a
 * protected AP, associate, and then carry nothing.
 */
void test_select_open() {
  BssTable t;
  const uint8_t protect[6] = {0x02, 0, 0, 0, 0, 0x11};
  const uint8_t plain[6] = {0x02, 0, 0, 0, 0, 0x12};
  const uint8_t wep[6] = {0x02, 0, 0, 0, 0, 0x13};

  /* Same SSID on all three, so the ONLY thing that can separate them is the
   * joinability test - not the name. */
  std::vector<uint8_t> a = beacon(protect, "net", 6, /*rsn=*/true);
  std::vector<uint8_t> b = beacon(plain, "net", 6, /*rsn=*/false);
  t.observe(a.data(), a.size(), -30, 6, 1000);       /* the STRONGER one */
  t.observe(b.data(), b.size(), -70, 6, 1000);

  const BssEntry* o = t.select_open("net");
  check(o != nullptr, "an open BSS is selectable by select_open");
  if (o) check(std::memcmp(o->info.bssid, plain, 6) == 0,
               "...and it is the OPEN one, not the stronger protected one");

  const BssEntry* w = t.select("net");
  check(w != nullptr, "the protected BSS is still selectable by select");
  if (w) check(std::memcmp(w->info.bssid, protect, 6) == 0,
               "...and it is the protected one");

  /* A WEP BSS: Privacy set, no RSN element at all. Neither selector may
   * offer it - select() because there is no RSN, select_open() because the
   * link is encrypted with a key this station does not have. Testing
   * !has_rsn instead of !privacy would hand it to the open station. */
  std::vector<uint8_t> c = beacon(wep, "wepnet", 6, /*rsn=*/false);
  c[24 + 10] |= 0x10;                             /* capability: Privacy */
  t.observe(c.data(), c.size(), -20, 6, 1000);
  check(t.find(wep) != nullptr && t.find(wep)->info.privacy,
        "the WEP BSS is observed, with Privacy set");
  check(t.select_open("wepnet") == nullptr,
        "...and select_open refuses it although it carries no RSN");
  check(t.select("wepnet") == nullptr, "...and select refuses it too");
}

/* SIXTEEN BEACONS FOR THE WANTED SSID, FROM SIXTEEN BSSIDs.
 *
 * The rule that makes an entry with the wanted SSID unevictable exists to
 * stop a flood of OTHER SSIDs pushing the target out. Applied without a
 * fallback it becomes the attack it was written against: fill every slot
 * with fabricated BSSes all claiming the wanted name, and the GENUINE AP can
 * never be inserted at all - select() then returns only the attacker's, for
 * as long as they keep beaconing.
 */
void test_a_flood_of_the_wanted_ssid_cannot_lock_the_table() {
  BssTable t;
  const uint8_t real_ap[6] = {0x02, 0xff, 0, 0, 0, 0x01};

  t.set_wanted("target");
  for (int i = 0; i < BssTable::kMaxBss; i++) {
    const uint8_t bssid[6] = {0x02, 0, 0, 0, 0, (uint8_t)(0x40 + i)};
    std::vector<uint8_t> f = beacon(bssid, "target", 6, true);
    /* All at the same instant, and all refreshed constantly - which is what
     * an attacker with a radio does. */
    t.observe(f.data(), f.size(), -30, 6, 1000);
  }
  check(t.count() == BssTable::kMaxBss, "the table is full of the wanted SSID");

  std::vector<uint8_t> real = beacon(real_ap, "target", 6, true);
  const BssEntry* e = t.observe(real.data(), real.size(), -20, 6, 2000);
  check(e != nullptr, "the genuine AP is still admitted");
  check(t.find(real_ap) != nullptr, "...and is in the table");
  const BssEntry* sel = t.select("target");
  check(sel != nullptr, "...and something is selectable");
  if (sel)
    check(std::memcmp(sel->info.bssid, real_ap, 6) == 0,
          "...and it is the genuine AP, which has the strongest signal");
}

/* A HIDDEN BSS: its beacons carry an empty (or all-zero) SSID and only a
 * directed probe response names it. The beacons that follow must not wipe
 * the name, or select() never finds the network the probe was sent for. */
void test_hidden_ssid_survives_its_beacons() {
  BssTable t;
  const uint8_t a[6] = {0x02, 0, 0, 0, 0, 0x0a};

  std::vector<uint8_t> pr = beacon(a, "net", 6, true);
  pr[0] = devourer::sta::kFcProbeResp;
  t.observe(pr.data(), pr.size(), -40, 6, 1000);
  check(t.select("net") != nullptr, "the probe response names the BSS");

  std::vector<uint8_t> empty = beacon(a, "", 6, true);
  t.observe(empty.data(), empty.size(), -41, 6, 1100);
  check(t.select("net") != nullptr,
        "AN EMPTY-SSID BEACON DOES NOT WIPE THE NAME");
  std::vector<uint8_t> zeros = beacon(a, std::string(3, '\0'), 6, true);
  t.observe(zeros.data(), zeros.size(), -42, 6, 1200);
  const BssEntry* e = t.select("net");
  check(e != nullptr, "...nor does an all-zero SSID");
  check(e && e->rssi == -42 && e->frames == 3,
        "...while the rest of the entry still refreshes");

  /* A BSS that genuinely renames itself is believed. */
  std::vector<uint8_t> other = beacon(a, "renamed", 6, true);
  t.observe(other.data(), other.size(), -40, 6, 1300);
  check(t.select("net") == nullptr && t.select("renamed") != nullptr,
        "a non-empty new SSID replaces the old one");
}

}  // namespace

/* THE EQUAL-RSSI TIE-BREAK ACROSS THE CLOCK WRAP. The millisecond clock
 * wraps every ~49.7 days; an entry heard just after the wrap has a SMALLER
 * raw timestamp than one heard just before it, and a raw `>` picks the stale
 * one. Both orders of insertion, so the result cannot depend on slot order. */
void test_tie_break_across_the_wrap() {
  const uint8_t before[6] = {0x02, 0, 0, 0, 0x0a, 0x01};
  const uint8_t after[6] = {0x02, 0, 0, 0, 0x0a, 0x02};
  const uint32_t t_before = 0xffffff00u;          /* 256 ms before the wrap */
  const uint32_t t_after = 0x00000100u;           /* 256 ms after it */

  for (int order = 0; order < 2; order++) {
    BssTable t;
    const std::vector<uint8_t> fb = beacon(before, "wrap", 6, true);
    const std::vector<uint8_t> fa = beacon(after, "wrap", 6, true);
    if (order == 0) {
      t.observe(fb.data(), fb.size(), -50, 6, t_before);
      t.observe(fa.data(), fa.size(), -50, 6, t_after);
    } else {
      t.observe(fa.data(), fa.size(), -50, 6, t_after);
      t.observe(fb.data(), fb.size(), -50, 6, t_before);
    }
    const BssEntry* e = t.select("wrap");
    check(e && std::memcmp(e->info.bssid, after, 6) == 0,
          order == 0 ? "tie-break: the entry heard after the wrap wins"
                     : "tie-break: ...whichever was inserted first");
  }

  /* And without a wrap the ordinary case still holds. */
  BssTable t;
  const std::vector<uint8_t> fb = beacon(before, "plain", 6, true);
  const std::vector<uint8_t> fa = beacon(after, "plain", 6, true);
  t.observe(fb.data(), fb.size(), -50, 6, 1000);
  t.observe(fa.data(), fa.size(), -50, 6, 2000);
  const BssEntry* e = t.select("plain");
  check(e && std::memcmp(e->info.bssid, after, 6) == 0,
        "tie-break: the more recent entry wins without a wrap too");
}

/* THE RECEIVE CHANNEL. A beacon without a DS Parameter Set (common on 5 GHz)
 * takes the channel it was received on; one that carries it keeps its own,
 * even when it was heard on an adjacent channel; with neither, the entry is
 * kept but never offered, by either selector. */
void test_rx_channel() {
  const uint8_t a[6] = {0x02, 0, 0, 0, 0x0c, 0x01};

  {
    BssTable t;
    const std::vector<uint8_t> f = beacon(a, "five", 36, true, true, false);
    const BssEntry* e = t.observe(f.data(), f.size(), -40, 36, 1000);
    check(e && e->info.channel == 36,
          "rx channel: a beacon without DS takes the RX channel");
    check(t.select("five") == e, "rx channel: ...and is selectable");
  }
  {
    BssTable t;
    const std::vector<uint8_t> f = beacon(a, "six", 6, true);
    const BssEntry* e = t.observe(f.data(), f.size(), -40, 7, 1000);
    check(e && e->info.channel == 6,
          "rx channel: a DS element that disagrees with the RX channel wins");
  }
  {
    BssTable t;
    const std::vector<uint8_t> f = beacon(a, "none", 36, true, true, false);
    const std::vector<uint8_t> o = beacon(a, "none-open", 36, false, false,
                                          false);
    t.observe(f.data(), f.size(), -40, 0, 1000);
    check(t.find(a) && t.find(a)->info.channel == 0,
          "rx channel: with no channel from either source the entry is kept");
    check(t.select("none") == nullptr,
          "rx channel: ...but select() never offers it");
    BssTable u;
    u.observe(o.data(), o.size(), -40, 0, 1000);
    check(u.select_open("none-open") == nullptr,
          "rx channel: ...nor does select_open()");
  }
}

/* ONLY AN INFRASTRUCTURE BSS IS OFFERED. An IBSS carrying the wanted SSID
 * must not be selectable: joining it would run an AP's
 * authenticate/associate exchange against peers that have no AP. ESS set and
 * IBSS clear, both selectors. */
void test_only_infrastructure_is_offered() {
  const uint8_t a[6] = {0x02, 0, 0, 0, 0x0d, 0x01};
  const auto with_cap = [](std::vector<uint8_t> f, uint16_t cap) {
    f[34] = (uint8_t)(cap & 0xff);                /* capability, LE */
    f[35] = (uint8_t)(cap >> 8);
    return f;
  };

  BssTable t;
  const std::vector<uint8_t> ibss = with_cap(beacon(a, "adhoc", 6, true),
                                             0x0002 | 0x0010);
  const BssEntry* e = t.observe(ibss.data(), ibss.size(), -40, 6, 1000);
  check(e != nullptr, "ibss: the beacon is observed");
  check(t.select("adhoc") == nullptr, "ibss: an IBSS is not selectable");

  BssTable u;
  const std::vector<uint8_t> open_ibss =
      with_cap(beacon(a, "adhoc-open", 6, false, false), 0x0002);
  u.observe(open_ibss.data(), open_ibss.size(), -40, 6, 1000);
  check(u.select_open("adhoc-open") == nullptr,
        "ibss: ...nor by select_open()");

  BssTable v;
  const std::vector<uint8_t> both = with_cap(beacon(a, "both", 6, true),
                                             0x0001 | 0x0002 | 0x0010);
  v.observe(both.data(), both.size(), -40, 6, 1000);
  check(v.select("both") == nullptr,
        "ibss: a frame claiming ESS AND IBSS is not offered either");

  BssTable w;
  const std::vector<uint8_t> ess = beacon(a, "infra", 6, true);
  w.observe(ess.data(), ess.size(), -40, 6, 1000);
  check(w.select("infra") != nullptr, "ibss: ...while an ESS still is");
}

/* A CHANNEL MUST BE A CHANNEL. 1..14 or 32..253, from the DS element or the
 * receiver alike. An invalid DS value must not override a usable receive
 * channel (15 over 6), and 255 must not be selectable: the association
 * request's band is picked by channel > 14, so either would build it for the
 * wrong band. */
void test_channel_validity() {
  const uint8_t a[6] = {0x02, 0, 0, 0, 0x0e, 0x01};
  const auto with_ds = [](std::vector<uint8_t> f, uint8_t ds) {
    /* The DS element is the one after the SSID and rates: find it. */
    for (size_t i = 36; i + 2 < f.size(); i += 2 + f[i + 1])
      if (f[i] == devourer::sta::kEidDsParams) { f[i + 2] = ds; break; }
    return f;
  };

  check(devourer::sta::channel_valid(1) && devourer::sta::channel_valid(14) &&
            devourer::sta::channel_valid(32) &&
            devourer::sta::channel_valid(253),
        "channel: the edges of both ranges are valid");
  check(!devourer::sta::channel_valid(0) && !devourer::sta::channel_valid(15) &&
            !devourer::sta::channel_valid(31) &&
            !devourer::sta::channel_valid(254) &&
            !devourer::sta::channel_valid(255),
        "channel: 0, 15, 31, 254 and 255 are not");

  {
    BssTable t;
    const std::vector<uint8_t> f = with_ds(beacon(a, "ds15", 6, true), 15);
    const BssEntry* e = t.observe(f.data(), f.size(), -40, 6, 1000);
    check(e && e->info.channel == 6,
          "channel: DS 15 is not a channel; the RX channel 6 stands in");
    check(t.select("ds15") != nullptr, "channel: ...and it is selectable");
  }
  {
    BssTable t;
    const std::vector<uint8_t> f = with_ds(beacon(a, "ds255", 6, true), 255);
    t.observe(f.data(), f.size(), -40, 0, 1000);
    check(t.find(a) && t.find(a)->info.channel == 0 &&
              t.select("ds255") == nullptr,
          "channel: DS 255 with no RX channel is not selectable");
  }
  {
    BssTable t;
    const std::vector<uint8_t> f = with_ds(beacon(a, "ds36", 6, true), 36);
    const BssEntry* e = t.observe(f.data(), f.size(), -40, 36, 1000);
    check(e && e->info.channel == 36 && t.select("ds36") == e,
          "channel: a 5 GHz channel in the DS element is kept");
  }
  {
    BssTable t;
    const std::vector<uint8_t> f = beacon(a, "rx15", 6, true, true, false);
    t.observe(f.data(), f.size(), -40, 15, 1000);
    check(t.select("rx15") == nullptr,
          "channel: an invalid RX channel is not used either");
  }
}

/* WPA2 SELECTION NEEDS THE PRIVACY BIT TOO. A CCMP/PSK RSN element under a
 * clear Privacy capability bit is self-contradictory, so the RSN element
 * alone is not enough for select() to offer the BSS. */
void test_rsn_without_privacy_is_not_selected() {
  const uint8_t a[6] = {0x02, 0, 0, 0, 0x0f, 0x01};
  BssTable t;
  const std::vector<uint8_t> f = beacon(a, "noprivacy", 6, true,
                                        /*privacy=*/false);
  const BssEntry* e = t.observe(f.data(), f.size(), -40, 6, 1000);
  check(e && e->info.rsn_ccmp_psk && !e->info.privacy,
        "privacy: setup - a CCMP/PSK RSN element with Privacy clear");
  check(t.select("noprivacy") == nullptr,
        "privacy: it is not selected for WPA2");
}

/* AN SSID ELEMENT LONGER THAN 32 OCTETS MAKES THE BEACON MALFORMED. It is
 * refused rather than recorded under a name no station could send back;
 * append_ssid refuses to build one for the same reason. */
void test_overlong_ssid_is_refused() {
  static const uint8_t bcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  const uint8_t a[6] = {0x02, 0, 0, 0, 0x10, 0x01};
  std::vector<uint8_t> m = devourer::sta::mgmt_hdr(devourer::sta::kFcBeacon,
                                                   bcast, a, a);
  m.insert(m.end(), 8, 0);                       /* timestamp */
  devourer::sta::put_le16(m, 100);
  devourer::sta::put_le16(m, 0x0011);
  m.push_back(devourer::sta::kEidSsid);
  m.push_back(33);
  m.insert(m.end(), 33, 'x');
  devourer::sta::append_ds_params(m, 6);
  devourer::sta::append_rsn_ccmp_psk(m);

  BssTable t;
  check(t.observe(m.data(), m.size(), -40, 6, 1000) == nullptr &&
            t.count() == 0,
        "ssid: a 33-octet SSID element makes the beacon malformed");
  std::vector<uint8_t> ok32 = m;
  ok32[37] = 32;                                  /* the SSID length octet */
  ok32.erase(ok32.begin() + 38);                  /* one 'x' fewer */
  check(t.observe(ok32.data(), ok32.size(), -40, 6, 1000) != nullptr,
        "ssid: ...while 32 octets is accepted");
}

int main() {
  test_observe_and_dedupe();
  test_overlong_ssid_is_refused();
  test_rsn_without_privacy_is_not_selected();
  test_only_infrastructure_is_offered();
  test_channel_validity();
  test_tie_break_across_the_wrap();
  test_rx_channel();
  test_only_beacons_are_observed();
  test_expire();
  test_select();
  test_select_open();
  test_mfp_required_is_skipped();
  test_eviction_when_full();
  test_wanted_ssid_survives_a_flood();
  test_a_flood_of_the_wanted_ssid_cannot_lock_the_table();
  test_hidden_ssid_survives_its_beacons();

  if (g_fail) {
    std::printf("bss_table_selftest: %d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("bss_table_selftest: OK\n");
  return 0;
}
