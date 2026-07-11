#ifndef DEVOURER_HOP_SCHEDULE_H
#define DEVOURER_HOP_SCHEDULE_H

#include <array>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace devourer {

class HopSchedule {
public:
  using Key = std::array<uint8_t, 16>;

  explicit HopSchedule(Key key) : key_(key) {}

  static Key parse_seed(const char *text) {
    if (!text || !*text)
      throw std::invalid_argument("empty hop seed");
    std::string s(text);
    if (s.size() >= 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X'))
      s.erase(0, 2);
    if (s.empty() || s.size() > 32)
      throw std::invalid_argument(
          "DEVOURER_HOP_SEED must contain 1..32 hex digits");
    for (char c : s)
      if (!std::isxdigit(static_cast<unsigned char>(c)))
        throw std::invalid_argument(
            "DEVOURER_HOP_SEED contains a non-hex digit");
    if (s.size() & 1)
      s.insert(s.begin(), '0');
    Key key{};
    const size_t first = key.size() - s.size() / 2;
    for (size_t i = 0; i < s.size() / 2; ++i)
      key[first + i] =
          static_cast<uint8_t>((hex(s[2 * i]) << 4) | hex(s[2 * i + 1]));
    return key;
  }

  static HopSchedule from_env(const char *name = "DEVOURER_HOP_SEED") {
    return HopSchedule(parse_seed(std::getenv(name)));
  }

  uint32_t fingerprint() const {
    static const uint8_t tag[] = {'d', 'e', 'v', 'o', 'u', 'r',
                                  'e', 'r', '-', 'h', 'o', 'p'};
    return static_cast<uint32_t>(siphash24(key_, tag, sizeof(tag)));
  }

  std::vector<size_t> permutation(uint64_t round, size_t n) const {
    std::vector<size_t> p(n);
    for (size_t i = 0; i < n; ++i)
      p[i] = i;
    uint64_t counter = 0;
    for (size_t i = n; i > 1; --i) {
      const uint64_t bound = static_cast<uint64_t>(i);
      const uint64_t limit = UINT64_MAX - (UINT64_MAX % bound);
      uint64_t r;
      do {
        r = word(round, counter++);
      } while (r >= limit);
      const size_t j = static_cast<size_t>(r % bound);
      const size_t t = p[i - 1];
      p[i - 1] = p[j];
      p[j] = t;
    }
    return p;
  }

  size_t channel_index(uint64_t slot, size_t n) const {
    if (!n)
      throw std::invalid_argument("empty hopset");
    const auto p = permutation(slot / n, n);
    return p[static_cast<size_t>(slot % n)];
  }

  template <class T>
  const T &channel(uint64_t slot, const std::vector<T> &h) const {
    return h[channel_index(slot, h.size())];
  }

  static uint64_t siphash24(const Key &key, const uint8_t *in, size_t len) {
    const uint64_t k0 = load64(key.data()), k1 = load64(key.data() + 8);
    uint64_t v0 = 0x736f6d6570736575ULL ^ k0, v1 = 0x646f72616e646f6dULL ^ k1;
    uint64_t v2 = 0x6c7967656e657261ULL ^ k0, v3 = 0x7465646279746573ULL ^ k1;
    const uint8_t *end = in + (len & ~size_t(7));
    for (; in != end; in += 8) {
      uint64_t m = load64(in);
      v3 ^= m;
      rounds(v0, v1, v2, v3, 2);
      v0 ^= m;
    }
    uint64_t b = static_cast<uint64_t>(len) << 56;
    for (size_t i = 0; i < (len & 7); ++i)
      b |= static_cast<uint64_t>(in[i]) << (8 * i);
    v3 ^= b;
    rounds(v0, v1, v2, v3, 2);
    v0 ^= b;
    v2 ^= 0xff;
    rounds(v0, v1, v2, v3, 4);
    return v0 ^ v1 ^ v2 ^ v3;
  }

private:
  Key key_;
  static unsigned hex(char c) {
    return c <= '9' ? c - '0' : (c <= 'F' ? c - 'A' + 10 : c - 'a' + 10);
  }
  static uint64_t load64(const uint8_t *p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i)
      v |= uint64_t(p[i]) << (8 * i);
    return v;
  }
  static uint64_t rotl(uint64_t x, int b) { return (x << b) | (x >> (64 - b)); }
  static void round(uint64_t &a, uint64_t &b, uint64_t &c, uint64_t &d) {
    a += b;
    b = rotl(b, 13);
    b ^= a;
    a = rotl(a, 32);
    c += d;
    d = rotl(d, 16);
    d ^= c;
    a += d;
    d = rotl(d, 21);
    d ^= a;
    c += b;
    b = rotl(b, 17);
    b ^= c;
    c = rotl(c, 32);
  }
  static void rounds(uint64_t &a, uint64_t &b, uint64_t &c, uint64_t &d,
                     int n) {
    while (n--)
      round(a, b, c, d);
  }
  uint64_t word(uint64_t round_no, uint64_t counter) const {
    uint8_t msg[17] = {'H'};
    for (int i = 0; i < 8; ++i) {
      msg[1 + i] = uint8_t(round_no >> (8 * i));
      msg[9 + i] = uint8_t(counter >> (8 * i));
    }
    return siphash24(key_, msg, sizeof(msg));
  }
};

struct HopSyncMarker {
  uint32_t fingerprint = 0, epoch = 0, phase_us = 0;
  uint64_t slot = 0;
  static constexpr size_t kSize = 29;
  static std::array<uint8_t, kSize> encode(const HopSyncMarker &m) {
    std::array<uint8_t, kSize> b{{221, 27, 0x57, 0x42, 0x75, 0x48, 1}};
    put32(b.data() + 7, m.fingerprint);
    put32(b.data() + 11, m.epoch);
    put64(b.data() + 15, m.slot);
    put32(b.data() + 23, m.phase_us);
    b[27] = 0xd7;
    b[28] = 0x3a;
    return b;
  }
  static bool decode(const uint8_t *p, size_t n, HopSyncMarker &m) {
    for (size_t i = 0; i + kSize <= n; ++i)
      if (p[i] == 221 && p[i + 1] == 27 && p[i + 2] == 0x57 &&
          p[i + 3] == 0x42 && p[i + 4] == 0x75 && p[i + 5] == 0x48 &&
          p[i + 6] == 1 && p[i + 27] == 0xd7 && p[i + 28] == 0x3a) {
        m.fingerprint = get32(p + i + 7);
        m.epoch = get32(p + i + 11);
        m.slot = get64(p + i + 15);
        m.phase_us = get32(p + i + 23);
        return true;
      }
    return false;
  }

private:
  static void put32(uint8_t *p, uint32_t v) {
    for (int i = 0; i < 4; ++i)
      p[i] = uint8_t(v >> (8 * i));
  }
  static void put64(uint8_t *p, uint64_t v) {
    for (int i = 0; i < 8; ++i)
      p[i] = uint8_t(v >> (8 * i));
  }
  static uint32_t get32(const uint8_t *p) {
    uint32_t v = 0;
    for (int i = 0; i < 4; ++i)
      v |= uint32_t(p[i]) << (8 * i);
    return v;
  }
  static uint64_t get64(const uint8_t *p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i)
      v |= uint64_t(p[i]) << (8 * i);
    return v;
  }
};

} // namespace devourer
#endif
