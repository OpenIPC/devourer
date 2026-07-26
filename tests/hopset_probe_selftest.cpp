/* Keyed recovery-probe placement: pinned known answers for which rounds,
 * positions, and excluded channels the probe draws select; the structural
 * invariants (exactly one probe opportunity per window, probe channel always
 * excluded, data slots unchanged everywhere else); and the inertness
 * guarantees (period 0 / full mask / generation 0 leave slot_info identical
 * to channel_index over long spans). */
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
  HopSchedule base(master);

  /* committed state: 8-channel base, mask drops indices 2 and 5 */
  const uint64_t mask = 0b11011011;
  const HopsetState st{7, 3, mask, 40, 40 * 8};
  const size_t k = popcount64(mask); /* 6 */
  const unsigned P = 4;

  /* --- inertness: period 0, gen 0, and full mask --- */
  {
    AdaptiveScheduleView v(base, keys, 8);
    v.set_state(st); /* period defaults to 0 */
    for (uint64_t s = st.activate_slot; s < st.activate_slot + 10000; ++s) {
      const auto si = v.slot_info(s);
      CHECK(!si.is_probe && si.base_index == v.channel_index(s),
            "period-0 slot_info == channel_index");
    }
    AdaptiveScheduleView g0(base, keys, 8);
    g0.set_probe_period(P); /* gen 0 state */
    for (uint64_t s = 0; s < 10000; ++s) {
      const auto si = g0.slot_info(s);
      CHECK(!si.is_probe && si.base_index == g0.channel_index(s),
            "gen-0 slot_info == channel_index");
    }
    AdaptiveScheduleView fullv(base, keys, 8);
    fullv.set_state({7, 2, full_mask(8), 0, 0});
    fullv.set_probe_period(P);
    for (uint64_t s = 0; s < 10000; ++s) {
      const auto si = fullv.slot_info(s);
      CHECK(!si.is_probe && si.base_index == fullv.channel_index(s),
            "full-mask slot_info == channel_index");
    }
  }

  AdaptiveScheduleView v(base, keys, 8);
  v.set_state(st);
  v.set_probe_period(P);

  /* --- structure over 40 windows: exactly one probe slot per window, probe
   * channel always in the excluded set, every non-probe slot identical to
   * the data schedule --- */
  {
    const uint64_t excluded = full_mask(8) & ~mask;
    for (uint64_t w = 0; w < 40; ++w) {
      unsigned probes = 0;
      for (uint64_t r = w * P; r < (w + 1) * P; ++r)
        for (uint64_t pos = 0; pos < k; ++pos) {
          const uint64_t slot = st.activate_slot + r * k + pos;
          const auto si = v.slot_info(slot);
          if (si.is_probe) {
            ++probes;
            CHECK(excluded & (uint64_t(1) << si.base_index),
                  "probe channel is excluded");
          } else {
            CHECK(si.base_index == v.channel_index(slot),
                  "non-probe slot rides the data schedule");
          }
        }
      CHECK(probes == 1, "exactly one probe per window");
    }
  }

  /* --- determinism: an independently constructed view agrees --- */
  {
    HopSchedule base2(master);
    AdaptiveScheduleView v2(base2, HopsetKeys::derive(master), 8);
    v2.set_state(st);
    v2.set_probe_period(P);
    for (uint64_t s = st.activate_slot; s < st.activate_slot + 5000; ++s) {
      const auto a = v.slot_info(s), b = v2.slot_info(s);
      CHECK(a.base_index == b.base_index && a.is_probe == b.is_probe,
            "independent views agree on probes");
    }
  }

  /* --- keyed placement varies across windows (no fixed phase/position) --- */
  {
    std::set<uint64_t> phases, picks;
    for (uint64_t w = 0; w < 32; ++w)
      for (uint64_t r = w * P; r < (w + 1) * P; ++r)
        for (uint64_t pos = 0; pos < k; ++pos) {
          const uint64_t slot = st.activate_slot + r * k + pos;
          if (v.slot_info(slot).is_probe) {
            phases.insert(r % P);
            picks.insert(v.slot_info(slot).base_index);
          }
        }
    CHECK(phases.size() > 1, "probe phase varies across windows");
    CHECK(picks.size() == 2, "both excluded channels get probed");
  }

  /* --- pinned known answers: (window -> probe round-in-window, pos, chan)
   * for the reference key/state — the drift tripwire --- */
  {
    struct Kat {
      uint64_t rphase, pos;
      size_t chan;
    };
    const Kat kat[6] = {{2, 4, 5}, {0, 3, 5}, {2, 2, 2},
                        {1, 4, 5}, {2, 2, 5}, {1, 1, 2}};
    bool print = false;
    Kat got[6]{};
    for (uint64_t w = 0; w < 6; ++w)
      for (uint64_t r = w * P; r < (w + 1) * P; ++r)
        for (uint64_t pos = 0; pos < k; ++pos) {
          const uint64_t slot = st.activate_slot + r * k + pos;
          const auto si = v.slot_info(slot);
          if (si.is_probe)
            got[w] = {r % P, pos, si.base_index};
        }
    for (int w = 0; w < 6; ++w)
      if (got[w].rphase != kat[w].rphase || got[w].pos != kat[w].pos ||
          got[w].chan != kat[w].chan)
        print = true;
    if (print) {
      std::fprintf(stderr, "PROBEKAT: ");
      for (int w = 0; w < 6; ++w)
        std::fprintf(stderr, "{%llu,%llu,%zu},",
                     (unsigned long long)got[w].rphase,
                     (unsigned long long)got[w].pos, got[w].chan);
      std::fprintf(stderr, "\n");
      CHECK(false, "probe placement KAT");
    }
  }

  return fails ? 1 : 0;
}
