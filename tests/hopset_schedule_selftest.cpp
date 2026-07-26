/* Adaptive hopset schedule math: key derivation, gen-0 legacy equivalence,
 * exact-once per-round coverage over masked hopsets, generation/mask
 * sensitivity, stateless direct join, and pinned known-answer tuples so the
 * derivation can never silently drift. */
#include "hopset/HopsetState.h"
#include <cstdio>
#include <set>

static int fails;
#define CHECK(x, msg)                                                          \
  do {                                                                         \
    if (!(x)) {                                                                \
      std::fprintf(stderr, "FAIL: %s\n", msg);                                 \
      ++fails;                                                                 \
    }                                                                          \
  } while (0)

int main() {
  using devourer::HopSchedule;
  using namespace devourer::hopset;

  const auto master =
      HopSchedule::parse_seed("000102030405060708090a0b0c0d0e0f");
  const auto keys = HopsetKeys::derive(master);

  /* --- key derivation: three distinct keys, master preserved --- */
  CHECK(keys.master == master, "master key preserved");
  CHECK(keys.sched != master && keys.ctrl != master && keys.sched != keys.ctrl,
        "derived keys distinct");
  /* pinned: edge bytes of the derived subkeys (drift tripwire) */
  if (!(keys.sched[0] == 0xf8 && keys.sched[15] == 0x5b &&
        keys.ctrl[0] == 0xf2 && keys.ctrl[15] == 0x18)) {
    std::fprintf(stderr, "KEYKAT: sched %02x..%02x ctrl %02x..%02x\n",
                 keys.sched[0], keys.sched[15], keys.ctrl[0], keys.ctrl[15]);
    CHECK(false, "derived key KAT");
  }

  /* --- mask helpers --- */
  CHECK(full_mask(8) == 0xff && full_mask(64) == ~uint64_t(0), "full_mask");
  CHECK(popcount64(0) == 0 && popcount64(0xff) == 8 &&
            popcount64(~uint64_t(0)) == 64,
        "popcount64");
  CHECK(nth_set_bit(0b101001, 0) == 0 && nth_set_bit(0b101001, 1) == 3 &&
            nth_set_bit(0b101001, 2) == 5,
        "nth_set_bit");
  CHECK(mask_valid(0b0011, 4, 2) && !mask_valid(0b0001, 4, 2) &&
            !mask_valid(0b10011, 4, 2) && !mask_valid(0, 4, 1) &&
            !mask_valid(1, 0, 1) && !mask_valid(1, 65, 1),
        "mask_valid rules");

  /* --- gen 0 == legacy fixed schedule, byte-for-byte --- */
  HopSchedule base(master);
  AdaptiveScheduleView v(base, keys, 12);
  for (uint64_t s = 0; s < 1000; ++s)
    CHECK(v.channel_index(s) == base.channel_index(s, 12),
          "gen-0 keyed equivalence");
  HopSchedule seq = HopSchedule::sequential();
  AdaptiveScheduleView vs(seq, keys, 5);
  for (uint64_t s = 0; s < 100; ++s)
    CHECK(vs.channel_index(s) == s % 5, "gen-0 sequential equivalence");

  /* --- exact-once coverage: every adaptive round visits every active
   * channel exactly once, for mask popcounts 2..8 x generations 1..4 --- */
  for (unsigned pc = 2; pc <= 8; ++pc) {
    for (uint32_t gen = 1; gen <= 4; ++gen) {
      AdaptiveScheduleView av(base, keys, 8);
      av.set_state({7, gen, full_mask(pc), 40, 40 * 8});
      const size_t k = av.active_count();
      CHECK(k == pc, "active_count = popcount");
      for (uint64_t round = 0; round < 20; ++round) {
        std::set<size_t> seen;
        for (uint64_t i = 0; i < k; ++i)
          seen.insert(av.channel_index(av.state().activate_slot + round * k + i));
        std::set<size_t> want;
        for (size_t j = 0; j < k; ++j)
          want.insert(nth_set_bit(full_mask(pc), j));
        CHECK(seen == want, "round covers active set exactly once");
      }
    }
  }

  /* --- non-contiguous mask maps through stable base positions --- */
  {
    const uint64_t mask = 0b10110010; /* base indices 1,4,5,7 */
    AdaptiveScheduleView av(base, keys, 8);
    av.set_state({7, 3, mask, 0, 0});
    std::set<size_t> seen;
    for (uint64_t i = 0; i < 4; ++i)
      seen.insert(av.channel_index(i));
    CHECK((seen == std::set<size_t>{1, 4, 5, 7}),
          "non-contiguous mask -> stable base indices");
  }

  /* --- generation sensitivity: same mask, fresh keyed order per gen --- */
  {
    AdaptiveScheduleView a1(base, keys, 8), a2(base, keys, 8);
    a1.set_state({7, 1, full_mask(8), 0, 0});
    a2.set_state({7, 2, full_mask(8), 0, 0});
    bool differ = false;
    for (uint64_t r = 0; r < 8 && !differ; ++r)
      differ = a1.permutation(r, 8) != a2.permutation(r, 8);
    CHECK(differ, "generation changes the permutation");
    /* ...and gen>=1 full mask differs from the legacy gen-0 order (subkey) */
    bool vs_legacy = false;
    for (uint64_t r = 0; r < 8 && !vs_legacy; ++r)
      vs_legacy = a1.permutation(r, 8) != base.permutation(r, 8);
    CHECK(vs_legacy, "adaptive subkey differs from legacy order");
  }

  /* --- mask sensitivity: equal popcount, different mask, different words --- */
  {
    AdaptiveScheduleView a1(base, keys, 8), a2(base, keys, 8);
    a1.set_state({7, 1, 0b00001111, 0, 0});
    a2.set_state({7, 1, 0b11110000, 0, 0});
    bool differ = false;
    for (uint64_t r = 0; r < 16 && !differ; ++r)
      differ = a1.permutation(r, 4) != a2.permutation(r, 4);
    CHECK(differ, "mask changes the permutation");
  }

  /* --- direct join: an independently-constructed view with the same
   * committed state agrees at a nonzero generation for arbitrary slots --- */
  {
    const HopsetState st{0x1234, 9, 0b01101101, 500, 4000};
    AdaptiveScheduleView a(base, keys, 8);
    a.set_state(st);
    HopSchedule base2(master);
    AdaptiveScheduleView b(base2, HopsetKeys::derive(master), 8);
    b.set_state(st);
    for (uint64_t s = st.activate_slot; s < st.activate_slot + 500; ++s)
      CHECK(a.channel_index(s) == b.channel_index(s), "direct join agrees");
  }

  /* --- pinned channel_index known answers (key, gen, mask, slot) --- */
  {
    AdaptiveScheduleView av(base, keys, 8);
    av.set_state({1, 2, 0b10111011, 100, 600});
    size_t got[12];
    for (int i = 0; i < 12; ++i)
      got[i] = av.channel_index(600 + i);
    const size_t kat[12] = {0, 7, 3, 4, 1, 5, 4, 0, 3, 1, 5, 7};
    bool print = false;
    for (int i = 0; i < 12; ++i)
      if (got[i] != kat[i])
        print = true;
    if (print) {
      std::fprintf(stderr, "KAT: ");
      for (int i = 0; i < 12; ++i)
        std::fprintf(stderr, "%zu,", got[i]);
      std::fprintf(stderr, "\n");
      CHECK(false, "channel_index KAT");
    }
  }

  /* --- fingerprints: deterministic, key/gen/mask/list sensitive --- */
  {
    const uint32_t ch[4] = {36, 40, 44, 48};
    const uint32_t ch2[4] = {36, 40, 44, 52};
    CHECK(hopset_fp(keys, ch, 4) == hopset_fp(keys, ch, 4) &&
              hopset_fp(keys, ch, 4) != hopset_fp(keys, ch2, 4) &&
              hopset_fp(keys, ch, 4) != hopset_fp(keys, ch, 3),
          "hopset_fp binds channel list");
    CHECK(mask_fp(keys, 1, 0xff) != mask_fp(keys, 2, 0xff) &&
              mask_fp(keys, 1, 0xff) != mask_fp(keys, 1, 0xfe),
          "mask_fp binds gen+mask");
  }

  return fails ? 1 : 0;
}
