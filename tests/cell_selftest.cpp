/* Headless guard for the M1 cell core (src/cell/SlotGrid.h, GrantMap.h):
 * legacy-OFDM airtime against the 802.11 formula, slot-budget accounting,
 * fixed-grid layout and TSF phase math, and the grant-map IE round trip,
 * its rejects (CRC, version, length, OUI) and the receiver's epoch /
 * effective-superframe rules. No hardware. */
#include <cstdint>
#include <cstdio>
#include <vector>

#include "cell/GrantMap.h"
#include "cell/SlotGrid.h"

using namespace devourer::cell;

static int failures = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      failures++;                                                              \
    }                                                                          \
  } while (0)

int main() {
  /* Airtime: 14-byte ACK at 6M = 20 + 4*ceil(134/24) = 44 us; a 1500-byte
   * MPDU at 54M = 20 + 4*ceil(12022/216) = 244 us; unsupported rates -> 0. */
  CHECK(ofdm_airtime_us(14, 6) == 44);
  CHECK(ofdm_airtime_us(1500, 54) == 244);
  CHECK(ofdm_airtime_us(100, 6) == 20 + 4 * ((16 + 800 + 6 + 23) / 24));
  CHECK(ofdm_airtime_us(100, 11) == 0);
  CHECK(ofdm_airtime_us(0, 6) == 24); /* SERVICE+tail alone: one symbol */

  /* Slot budget: attempts x (psdu + SIFS + ACK) + retries x gap + margins. */
  {
    SlotBudget b;
    b.mpdu_len = 100;
    b.rate_mbps = 6;
    CHECK(slot_len_us(b) == ofdm_airtime_us(100, 6));
    b.acked = true;
    const int64_t one = ofdm_airtime_us(100, 6) + kSifsUs + 44;
    CHECK(slot_len_us(b) == one);
    b.attempts = 3;
    b.retry_gap_us = 100;
    b.turnaround_us = 10;
    b.guard_us = 500;
    CHECK(slot_len_us(b) == 3 * one + 2 * 100 + 10 + 500);
    b.attempts = 0;
    CHECK(slot_len_us(b) == 0);
    b.attempts = kMaxAttempts + 1;
    CHECK(slot_len_us(b) == 0);
    b.attempts = kMaxAttempts; /* the bounds keep this exact and positive */
    b.retry_gap_us = b.turnaround_us = b.guard_us = kMaxMarginUs;
    b.mpdu_len = kMaxMpduLen;
    CHECK(slot_len_us(b) > 0);
    b.mpdu_len = kMaxMpduLen + 1;
    CHECK(slot_len_us(b) == 0);
    b.mpdu_len = 100;
    b.attempts = 1;
    b.retry_gap_us = b.turnaround_us = b.guard_us = 0;
    b.guard_us = -1; /* a negative margin would under-budget the slot */
    CHECK(slot_len_us(b) == 0);
    b.guard_us = 0;
    b.turnaround_us = -10;
    CHECK(slot_len_us(b) == 0);
    b.turnaround_us = 0;
    b.retry_gap_us = kMaxMarginUs + 1;
    CHECK(slot_len_us(b) == 0);
    b.retry_gap_us = 0;
    CHECK(slot_len_us(b) == one);
    b.rate_mbps = 5;
    CHECK(slot_len_us(b) == 0);
  }

  /* Fixed grid: beacon, DL per UE, turnaround, UL per UE, in order. */
  {
    auto g = build_fixed_grid(100, 0, 2000, {1, 2}, 10000, 1000, 10000);
    CHECK(g.has_value());
    if (g) {
      CHECK(g->period_us() == 102400);
      CHECK(g->slots().size() == 6);
      CHECK(g->slots()[0].kind == SlotKind::Beacon && g->slots()[0].start_us == 0);
      CHECK(g->slots()[1].kind == SlotKind::Downlink && g->slots()[1].ue == 1 &&
            g->slots()[1].start_us == 2000);
      CHECK(g->slots()[3].kind == SlotKind::Turnaround && g->slots()[3].start_us == 22000);
      CHECK(g->slots()[5].kind == SlotKind::Uplink && g->slots()[5].ue == 2 &&
            g->slots()[5].start_us == 33000);
      CHECK(g->used_us() == 43000);
      CHECK(g->find(SlotKind::Uplink, 1) == 4u);
      CHECK(!g->find(SlotKind::Uplink, 3).has_value());
    }
    /* Does not fit: 2 x 60 ms DL in a 100 TU superframe. */
    CHECK(!build_fixed_grid(100, 0, 2000, {1, 2}, 60000, 1000, 1000).has_value());
    /* The grant map's capacity bounds the UE count: 24 fits, 25 does not. */
    std::vector<uint16_t> many(kMaxUesPerCell);
    for (size_t i = 0; i < many.size(); ++i)
      many[i] = static_cast<uint16_t>(i + 1);
    CHECK(build_fixed_grid(1000, 0, 2000, many, 1000, 1000, 1000).has_value());
    many.push_back(99);
    CHECK(!build_fixed_grid(1000, 0, 2000, many, 1000, 1000, 1000).has_value());
    CHECK(kGrantMax == kMaxUesPerCell);
    SlotGrid h(100, 0);
    CHECK(!h.append(SlotKind::Idle, 0, 0));
    CHECK(h.append(SlotKind::Idle, 0, 102400));
    CHECK(!h.append(SlotKind::Idle, 0, 1));
  }

  /* Phase math, with a TBTT offset and across the superframe boundary. */
  {
    auto g = build_fixed_grid(100, 10000, 2000, {7}, 5000, 1000, 5000);
    CHECK(g.has_value());
    if (g) {
      CHECK(g->offset_us() == 10000);
      auto a = g->at(10000);               /* a TBTT */
      CHECK(a.into_us == 0 && a.slot == 0u);
      a = g->at(10000 + 2000);             /* DL slot of UE 7 */
      CHECK(a.slot == 1u && a.superframe == 0);
      a = g->at(10000 + 3 * 102400 + 8500); /* UL slot, superframe 3 */
      CHECK(a.superframe == 3 && a.into_us == 8500 && a.slot == 3u);
      a = g->at(10000 + 50000);            /* idle tail */
      CHECK(!a.slot.has_value());
      a = g->at(5000);                     /* before the first TBTT */
      CHECK(a.superframe == -1 && a.into_us == 102400 - 5000);
      CHECK(g->slot_start_tsf(3, 3) == 10000 + 3 * 102400 + 8000);
    }
    /* Offsets normalise into [0, period). */
    CHECK(SlotGrid(100, -1000).offset_us() == 102400 - 1000);
    CHECK(SlotGrid(100, 102400 + 5).offset_us() == 5);
  }

  /* Grant map: round trip, inside a beacon, and every reject path. */
  {
    GrantMap m;
    m.flags = 1;
    m.cell_id = 0x0102;
    m.epoch = 77;
    m.effective_sf = 0x0102030405060708ull;
    m.period_tu = 100;
    m.grants = {{1, 1, 4, 6, -8, -12}, {2, 2, 5, 24, 4, 300}};
    /* A keyed stand-in for HMAC-SHA1 (NOT cryptographic). */
    auto key = [](uint64_t k) -> Mac {
      return [k](const uint8_t *dd, size_t n, uint8_t out[20]) {
        uint64_t h = 1469598103934665603ull ^ k;
        for (size_t i = 0; i < n; ++i)
          h = (h ^ dd[i]) * 1099511628211ull;
        for (int i = 0; i < 20; ++i) {
          h = (h ^ static_cast<uint64_t>(i)) * 1099511628211ull;
          out[i] = static_cast<uint8_t>(h >> 32);
        }
        return true;
      };
    };
    const Mac k = key(1), other = key(2), none;
    const auto ie = encode_grant_ie(m, k);
    CHECK(ie.size() == 2 + 3 + 21 + 2 * 8 + 2 + kGrantTagLen);
    auto d = decode_grant_ie(ie.data(), ie.size(), k);
    CHECK(d.has_value() && *d == m);
    if (d) {
      CHECK(d->find(2) && d->find(2)->ta_us == 300);
      CHECK(d->find(1)->tx_pwr_qdb == -8);
      CHECK(d->find(9) == nullptr);
    }

    std::vector<uint8_t> bcn(36, 0);
    bcn[0] = 0x80;
    const std::vector<uint8_t> ssid = {0, 3, 'G', 'N', 'T'};
    bcn.insert(bcn.end(), ssid.begin(), ssid.end());
    bcn.insert(bcn.end(), ie.begin(), ie.end());
    auto f = find_grant_map(bcn.data(), bcn.size(), k);
    CHECK(f.has_value() && *f == m);
    CHECK(!find_grant_map(bcn.data(), 36 + ssid.size(), k).has_value()); /* no IE */

    /* Authentication (v2): a forged map is inert; a keyless receiver checks
     * the structure only; a keyed one refuses an untagged (keyless) map. */
    CHECK(!decode_grant_ie(ie.data(), ie.size(), other).has_value()); /* another key */
    CHECK(!find_grant_map(bcn.data(), bcn.size(), other).has_value());
    auto bad = ie;
    bad.back() ^= 0x01; /* tag bit flip */
    CHECK(!decode_grant_ie(bad.data(), bad.size(), k).has_value());
    CHECK(decode_grant_ie(ie.data(), ie.size(), none).has_value()); /* passive: structure */
    const auto open_ie = encode_grant_ie(m, none);
    CHECK(open_ie.size() == ie.size() && decode_grant_ie(open_ie.data(), open_ie.size(), none));
    CHECK(!decode_grant_ie(open_ie.data(), open_ie.size(), k).has_value()); /* zero tag */
    GrantMap moved = m; /* a forger's edit, CRC recomputed: the tag still fails */
    moved.grants[0].ul_slot = 9;
    auto forged = encode_grant_ie(moved, other);
    CHECK(!forged.empty() && !decode_grant_ie(forged.data(), forged.size(), k).has_value());

    bad = ie;
    bad[10] ^= 0x01; /* body bit flip -> CRC */
    CHECK(!decode_grant_ie(bad.data(), bad.size(), k).has_value());
    bad = ie;
    bad[7] = 1; /* version 1 (untagged): refused */
    CHECK(!decode_grant_ie(bad.data(), bad.size(), k).has_value());
    bad = ie;
    bad[2] = 0x00; /* OUI */
    CHECK(!decode_grant_ie(bad.data(), bad.size(), k).has_value());
    bad = ie;
    bad[1] = static_cast<uint8_t>(bad[1] - 1); /* length disagrees with n */
    CHECK(!decode_grant_ie(bad.data(), bad.size(), k).has_value());
    CHECK(!decode_grant_ie(ie.data(), ie.size() - 1, k).has_value()); /* truncated */

    GrantMap big;
    big.grants.resize(kGrantMax + 1);
    CHECK(encode_grant_ie(big, k).empty());
    big.grants.resize(kGrantMax);
    const auto full = encode_grant_ie(big, k);
    CHECK(!full.empty() && full[1] <= 255 && decode_grant_ie(full.data(), full.size(), k));
    const Mac failing = [](const uint8_t *, size_t, uint8_t *) { return false; };
    CHECK(encode_grant_ie(m, failing).empty()); /* a failing Mac: nothing aired */
  }

  /* Receiver: newer epochs only; a map takes effect at its superframe. */
  {
    GrantMapReceiver r(0);
    GrantMap a;
    a.epoch = 5;
    a.effective_sf = 10;
    a.grants = {{1, 1, 2, 6, 0, 0}};
    CHECK(r.offer(a));
    CHECK(r.current(9) == nullptr);   /* not yet in force */
    CHECK(r.current(10) && r.current(10)->epoch == 5);
    GrantMap old = a;
    old.epoch = 4;
    CHECK(!r.offer(old));             /* reordered */
    CHECK(!r.offer(a));               /* replayed */
    GrantMap b = a;
    b.epoch = 6;
    b.effective_sf = 12;
    CHECK(r.offer(b));
    CHECK(r.current(11)->epoch == 5); /* old map until sf 12 */
    CHECK(r.current(12)->epoch == 6);
  }

  /* Two future maps offered back to back: each takes effect at its own sf. */
  {
    GrantMapReceiver r(3);
    GrantMap m6, m7;
    m6.cell_id = m7.cell_id = 3;
    m6.epoch = 6; m6.effective_sf = 12;
    m7.epoch = 7; m7.effective_sf = 20;
    CHECK(r.offer(m6) && r.offer(m7));
    CHECK(r.current(11) == nullptr);
    CHECK(r.current(12) && r.current(12)->epoch == 6);
    CHECK(r.current(19)->epoch == 6);
    CHECK(r.current(20)->epoch == 7);
    /* A UE that wakes late jumps straight to the newest due map. */
    GrantMapReceiver late(3);
    CHECK(late.offer(m6) && late.offer(m7));
    CHECK(late.current(25)->epoch == 7);
    CHECK(late.current(30)->epoch == 7);
    /* A newer map due earlier replaces a queued later one. */
    GrantMapReceiver re(3);
    GrantMap m8 = m6;
    m8.epoch = 8; m8.effective_sf = 15;
    CHECK(re.offer(m6) && re.offer(m7) && re.offer(m8));
    CHECK(re.current(12)->epoch == 6);
    CHECK(re.current(15)->epoch == 8);
    CHECK(re.current(25)->epoch == 8); /* epoch 7 (sf 20) was superseded */
  }

  /* Maps are per cell: another cell's map is refused whatever its epoch, and
   * rebind() starts over for the new cell. */
  {
    GrantMapReceiver r(1);
    GrantMap mine, other;
    mine.cell_id = 1; mine.epoch = 10; mine.effective_sf = 1;
    other.cell_id = 2; other.epoch = 99; other.effective_sf = 1;
    CHECK(!r.offer(other));
    CHECK(r.offer(mine));
    other.epoch = 3;
    CHECK(!r.offer(other));
    CHECK(r.current(1)->cell_id == 1);
    r.rebind(2);
    CHECK(r.cell_id() == 2 && r.current(1) == nullptr);
    CHECK(r.offer(other)); /* a lower epoch is fine in the new cell */
    CHECK(r.current(1)->cell_id == 2 && r.current(1)->epoch == 3);
  }

  if (failures == 0)
    std::printf("cell_selftest: all checks passed\n");
  return failures == 0 ? 0 : 1;
}
