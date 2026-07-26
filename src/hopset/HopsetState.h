/* HopsetState — the deterministic state model for adaptive FHSS hopset
 * changes. The configured base hopset is immutable; every adaptive state
 * identifies its active channels by stable base-hopset bit positions in a
 * 64-bit mask, so a channel keeps its identity across any sequence of
 * exclusions and restorations.
 *
 * Generation 0 is defined as "legacy": full mask, the raw master key, and the
 * exact fixed-hop permutation of HopSchedule — so with adaptive mode disabled
 * (or before the first commit) the schedule is byte-identical to the shipped
 * fixed-hop behavior. Generations >= 1 derive each round's permutation from
 * (schedule subkey, generation, round, active_mask): including the generation
 * makes a mask change produce a fresh keyed order rather than a filtered form
 * of the previous permutation, and the whole lookup stays a pure function of
 * (keys, committed state, absolute slot) — a receiver joins mid-generation
 * with no RNG state, exactly like the fixed schedule.
 *
 * The schedule and control keys are domain-separated from the master seed
 * (a schedule fingerprint is not authentication — the MAC key is its own
 * derivation). Pure, header-only, selftested; the library reads no env. */
#ifndef DEVOURER_HOPSET_HOPSET_STATE_H
#define DEVOURER_HOPSET_HOPSET_STATE_H

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "HopSchedule.h"

namespace devourer {
namespace hopset {

/* Generations are monotonic within one authority epoch; an authority refuses
 * to commit at the ceiling — recovery is a restart (fresh random epoch,
 * generation back to 0). */
inline constexpr uint32_t kGenCeiling = 0xFFFFFF00u;

/* Base hopsets are capped by the mask width. */
inline constexpr size_t kMaxBaseChannels = 64;

inline unsigned popcount64(uint64_t v) {
  unsigned c = 0;
  for (; v; v &= v - 1)
    ++c;
  return c;
}

inline uint64_t full_mask(size_t n) {
  return n >= 64 ? ~uint64_t(0) : ((uint64_t(1) << n) - 1);
}

/* Index of the j-th set bit (j counted from 0, LSB-first). Precondition:
 * j < popcount64(mask). */
inline size_t nth_set_bit(uint64_t mask, size_t j) {
  for (size_t i = 0; i < 64; ++i)
    if (mask & (uint64_t(1) << i)) {
      if (!j)
        return i;
      --j;
    }
  throw std::invalid_argument("nth_set_bit past popcount");
}

/* A mask is valid against an n-channel base hopset iff it selects only real
 * positions and keeps at least min_active channels in play. */
inline bool mask_valid(uint64_t mask, size_t n, unsigned min_active) {
  if (n == 0 || n > kMaxBaseChannels)
    return false;
  if (mask & ~full_mask(n))
    return false;
  return popcount64(mask) >= min_active;
}

/* Schedule + control keys, domain-separated from the master seed
 * (DEVOURER_HOP_SEED). The master key itself stays the generation-0 schedule
 * key so the legacy fixed schedule is untouched. */
struct HopsetKeys {
  HopSchedule::Key master{}; /* gen-0 (legacy) schedule key */
  HopSchedule::Key sched{};  /* gen>=1 adaptive schedule subkey */
  HopSchedule::Key ctrl{};   /* control-message MAC key */

  static HopsetKeys derive(const HopSchedule::Key &master) {
    static const uint8_t s0[] = {'d', 'e', 'v', 'o', 'u', 'r', 'e', 'r', '-',
                                 'h', 'o', 'p', 's', 'e', 't', '-', 's', '0'};
    static const uint8_t s1[] = {'d', 'e', 'v', 'o', 'u', 'r', 'e', 'r', '-',
                                 'h', 'o', 'p', 's', 'e', 't', '-', 's', '1'};
    static const uint8_t c0[] = {'d', 'e', 'v', 'o', 'u', 'r', 'e', 'r', '-',
                                 'h', 'o', 'p', 's', 'e', 't', '-', 'c', '0'};
    static const uint8_t c1[] = {'d', 'e', 'v', 'o', 'u', 'r', 'e', 'r', '-',
                                 'h', 'o', 'p', 's', 'e', 't', '-', 'c', '1'};
    HopsetKeys out;
    out.master = master;
    pack(out.sched, HopSchedule::siphash24(master, s0, sizeof(s0)),
         HopSchedule::siphash24(master, s1, sizeof(s1)));
    pack(out.ctrl, HopSchedule::siphash24(master, c0, sizeof(c0)),
         HopSchedule::siphash24(master, c1, sizeof(c1)));
    return out;
  }

  uint64_t mac(const uint8_t *p, size_t n) const {
    return HopSchedule::siphash24(ctrl, p, n);
  }

private:
  static void pack(HopSchedule::Key &k, uint64_t lo, uint64_t hi) {
    for (int i = 0; i < 8; ++i) {
      k[i] = static_cast<uint8_t>(lo >> (8 * i));
      k[8 + i] = static_cast<uint8_t>(hi >> (8 * i));
    }
  }
};

/* Fingerprint binding a message to the schedule subkey AND the exact base
 * channel list — a config/routing check (the MAC under the control key is the
 * authentication). Channels are the demo's channel numbers in base order. */
inline uint32_t hopset_fp(const HopsetKeys &keys, const uint32_t *ch,
                          size_t n) {
  std::vector<uint8_t> m = {'d', 'e', 'v', 'o', 'u', 'r', 'e', 'r', '-',
                            'h', 'o', 'p', 's', 'e', 't', '-', 'f', 'p'};
  m.push_back(static_cast<uint8_t>(n));
  for (size_t i = 0; i < n; ++i)
    for (int b = 0; b < 4; ++b)
      m.push_back(static_cast<uint8_t>(ch[i] >> (8 * b)));
  return static_cast<uint32_t>(
      HopSchedule::siphash24(keys.sched, m.data(), m.size()));
}

/* Compact (generation, mask) advertisement for the hot-path sync marker —
 * enough for a follower to detect it missed a transition; the full mask is
 * recovered from the repeated authenticated Commit/Status frames. */
inline uint32_t mask_fp(const HopsetKeys &keys, uint32_t generation,
                        uint64_t mask) {
  uint8_t m[31] = {'d', 'e', 'v', 'o', 'u', 'r', 'e', 'r', '-', 'h',
                   'o', 'p', 's', 'e', 't', '-', 'm', 'f', 'p'};
  for (int i = 0; i < 4; ++i)
    m[19 + i] = static_cast<uint8_t>(generation >> (8 * i));
  for (int i = 0; i < 8; ++i)
    m[23 + i] = static_cast<uint8_t>(mask >> (8 * i));
  return static_cast<uint32_t>(HopSchedule::siphash24(keys.sched, m, sizeof(m)));
}

/* The committed adaptive state both endpoints hold. activate_slot is the
 * absolute-slot activation boundary (the anchor both sides key on — rounds
 * change length when the mask popcount changes, absolute slots never do);
 * activate_round is the same instant counted in the *previous* generation's
 * rounds, carried for window validation. */
struct HopsetState {
  uint32_t epoch = 0;      /* the authority's per-boot random epoch */
  uint32_t generation = 0; /* 0 = legacy fixed schedule, full mask */
  uint64_t active_mask = 0;
  uint64_t activate_round = 0;
  uint64_t activate_slot = 0;
};

/* Generation/mask-aware stateless slot->channel lookup over an immutable
 * base hopset. Generation 0 forwards verbatim to the wrapped HopSchedule
 * (keyed or sequential); generations >= 1 permute the active positions with
 * the schedule subkey. Caller contract: channel_index is asked only for
 * slots at/after the committed state's activation boundary (a follower holds
 * the previous state object until the boundary passes). */
class AdaptiveScheduleView {
public:
  AdaptiveScheduleView(const HopSchedule &base, const HopsetKeys &keys,
                       size_t n_base)
      : base_(&base), keys_(keys), n_(n_base) {
    if (!n_base || n_base > kMaxBaseChannels)
      throw std::invalid_argument("base hopset size out of range");
  }

  void set_state(const HopsetState &st) { st_ = st; }
  const HopsetState &state() const { return st_; }
  size_t base_size() const { return n_; }

  /* Active-round length: base size at gen 0, popcount of the mask after. */
  size_t active_count() const {
    return st_.generation == 0 ? n_
                               : popcount64(st_.active_mask);
  }

  size_t channel_index(uint64_t slot) const {
    if (st_.generation == 0)
      return base_->channel_index(slot, n_);
    const size_t k = popcount64(st_.active_mask);
    if (!k)
      throw std::invalid_argument("empty active mask");
    const uint64_t rel = slot - st_.activate_slot;
    if (k == 1)
      return nth_set_bit(st_.active_mask, 0);
    const auto p = permutation(rel / k, k);
    return nth_set_bit(st_.active_mask, p[rel % k]);
  }

  template <class T>
  const T &channel(uint64_t slot, const std::vector<T> &base_channels) const {
    return base_channels[channel_index(slot)];
  }

  /* --- keyed recovery probes ---------------------------------------------
   * An excluded channel cannot prove recovery unless deliberately revisited.
   * With a probe period P (shared config, like the seed), every P rounds ONE
   * data opportunity is replaced by a probe dwell on an excluded channel.
   * Which round of the P-window, which position within that round, and which
   * excluded channel are all keyed draws ('P'-tagged words per window), so
   * recovery never creates a public periodic target. Inert whenever the
   * period is 0, the mask is full, or the state is generation 0 — slot_info
   * then degenerates to {channel_index(slot), false} and every fixed-hop /
   * no-exclusion behavior is untouched. */
  struct SlotInfo {
    size_t base_index;
    bool is_probe;
  };

  void set_probe_period(unsigned rounds) { probe_period_ = rounds; }
  unsigned probe_period() const { return probe_period_; }

  SlotInfo slot_info(uint64_t slot) const {
    if (probe_period_ == 0 || st_.generation == 0)
      return {channel_index(slot), false};
    const uint64_t excluded = full_mask(n_) & ~st_.active_mask;
    const size_t x = popcount64(excluded);
    if (!x)
      return {channel_index(slot), false};
    const size_t k = popcount64(st_.active_mask);
    const uint64_t rel = slot - st_.activate_slot;
    const uint64_t round = rel / k, pos = rel % k;
    const uint64_t window = round / probe_period_;
    uint64_t counter = 0;
    const uint64_t probe_round = probe_draw(window, counter, probe_period_);
    if (round % probe_period_ != probe_round)
      return {channel_index(slot), false};
    const uint64_t probe_pos = probe_draw(window, counter, k);
    if (pos != probe_pos)
      return {channel_index(slot), false};
    const uint64_t pick = probe_draw(window, counter, x);
    return {nth_set_bit(excluded, static_cast<size_t>(pick)), true};
  }

  /* The gen>=1 permutation: the same rejection-sampled Fisher-Yates as
   * HopSchedule::permutation, driven by words bound to (generation, mask,
   * round) under the schedule subkey. Per-generation rounds start at 0 at
   * the activation boundary. */
  std::vector<size_t> permutation(uint64_t round, size_t k) const {
    std::vector<size_t> p(k);
    for (size_t i = 0; i < k; ++i)
      p[i] = i;
    uint64_t counter = 0;
    for (size_t i = k; i > 1; --i) {
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

private:
  /* 'A' domain tag vs the legacy word's 'H' — the two can never collide even
   * under the same key, and the subkey separates them anyway. */
  uint64_t word(uint64_t round, uint64_t counter) const {
    uint8_t msg[29] = {'A'};
    for (int i = 0; i < 4; ++i)
      msg[1 + i] = static_cast<uint8_t>(st_.generation >> (8 * i));
    for (int i = 0; i < 8; ++i) {
      msg[5 + i] = static_cast<uint8_t>(st_.active_mask >> (8 * i));
      msg[13 + i] = static_cast<uint8_t>(round >> (8 * i));
      msg[21 + i] = static_cast<uint8_t>(counter >> (8 * i));
    }
    return HopSchedule::siphash24(keys_.sched, msg, sizeof(msg));
  }

  /* 'P'-tagged word for probe placement — same 29-byte shape, keyed by the
   * probe window so all draws of one window share a counter stream. */
  uint64_t probe_word(uint64_t window, uint64_t counter) const {
    uint8_t msg[29] = {'P'};
    for (int i = 0; i < 4; ++i)
      msg[1 + i] = static_cast<uint8_t>(st_.generation >> (8 * i));
    for (int i = 0; i < 8; ++i) {
      msg[5 + i] = static_cast<uint8_t>(st_.active_mask >> (8 * i));
      msg[13 + i] = static_cast<uint8_t>(window >> (8 * i));
      msg[21 + i] = static_cast<uint8_t>(counter >> (8 * i));
    }
    return HopSchedule::siphash24(keys_.sched, msg, sizeof(msg));
  }

  /* Unbiased uniform draw in [0, bound) — the permutation()'s rejection-
   * sampling idiom, advancing the caller's counter. */
  uint64_t probe_draw(uint64_t window, uint64_t &counter,
                      uint64_t bound) const {
    const uint64_t limit = UINT64_MAX - (UINT64_MAX % bound);
    uint64_t r;
    do {
      r = probe_word(window, counter++);
    } while (r >= limit);
    return r % bound;
  }

  const HopSchedule *base_;
  HopsetKeys keys_;
  HopsetState st_{};
  size_t n_;
  unsigned probe_period_ = 0;
};

} /* namespace hopset */
} /* namespace devourer */

#endif /* DEVOURER_HOPSET_HOPSET_STATE_H */
