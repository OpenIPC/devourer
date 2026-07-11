#include "HopSchedule.h"
#include <algorithm>
#include <cstdio>
#include <set>
#include <stdexcept>

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
  auto k = HopSchedule::parse_seed("000102030405060708090a0b0c0d0e0f");
  const uint64_t vectors[] = {0x726fdb47dd0e0e31ULL, 0x74f839c593dc67fdULL,
                              0x0d6c8009d9a94f5aULL, 0x85676696d7fb7e2dULL};
  uint8_t msg[4]{};
  for (size_t n = 0; n < 4; ++n) {
    if (n)
      msg[n - 1] = uint8_t(n - 1);
    CHECK(HopSchedule::siphash24(k, msg, n) == vectors[n],
          "SipHash reference vector");
  }
  HopSchedule a(k),
      b(HopSchedule::parse_seed("0x000102030405060708090a0b0c0d0e0f"));
  CHECK(a.fingerprint() == b.fingerprint(), "normalized seed fingerprint");
  for (uint64_t r = 0; r < 100; ++r) {
    auto p = a.permutation(r, 12);
    std::set<size_t> s(p.begin(), p.end());
    CHECK(s.size() == 12 && *s.begin() == 0 && *s.rbegin() == 11,
          "round covers hopset");
    for (size_t i = 0; i < 12; ++i)
      CHECK(a.channel_index(r * 12 + i, 12) == p[i], "stateless lookup");
  }
  CHECK(a.channel_index(UINT64_MAX, 1) == 0, "one-channel hopset");
  CHECK(a.permutation(4, 8) != a.permutation(5, 8), "round sensitivity");
  HopSchedule c(HopSchedule::parse_seed("1"));
  CHECK(a.permutation(4, 8) != c.permutation(4, 8), "key sensitivity");
  bool bad = false;
  try {
    (void)HopSchedule::parse_seed("xyz");
  } catch (const std::invalid_argument &) {
    bad = true;
  }
  CHECK(bad, "bad seed rejected");
  // Sequential (keyless) schedule: plain round-robin, a fixed public
  // fingerprint distinct from any keyed one, and stateless like the keyed path.
  HopSchedule seq = HopSchedule::sequential();
  CHECK(seq.is_sequential() && !a.is_sequential(), "sequential flag");
  CHECK(seq.fingerprint() == 0x53455131u && a.fingerprint() != 0x53455131u,
        "sequential fingerprint sentinel");
  for (uint64_t s = 0; s < 40; ++s)
    CHECK(seq.channel_index(s, 4) == s % 4, "sequential round-robin");
  devourer::HopSyncMarker m{a.fingerprint(), 123, 456,
                            UINT64_C(0x1020304050607080)},
      out;
  auto wire = devourer::HopSyncMarker::encode(m);
  CHECK(devourer::HopSyncMarker::decode(wire.data(), wire.size(), out),
        "marker decodes");
  CHECK(out.fingerprint == m.fingerprint && out.epoch == m.epoch &&
            out.slot == m.slot && out.phase_us == m.phase_us,
        "marker roundtrip");
  return fails ? 1 : 0;
}
