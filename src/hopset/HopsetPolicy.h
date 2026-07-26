/* HopsetPolicy — the receiver-side exclusion/restoration decision engine for
 * adaptive FHSS. The receiver decides because its delivery measurement
 * describes the endpoint that must actually decode; the transmitter validates
 * the proposal structurally and owns the commit.
 *
 * Delivery is authoritative. RSSI/SNR/EVM, energy counters and link verdicts
 * only *classify* an impairment (interference vs weak signal vs lost
 * synchronization) — they set explanation bits and never independently
 * trigger an exclusion, because a quiet channel with a dead link and a noisy
 * channel that still delivers are opposite decisions.
 *
 * Everything here is conservative by construction, because an adaptive
 * exclusion loop is an attack surface: an adversary who can make a channel
 * look bad can otherwise herd the link onto a single channel it then jams.
 * Hence the hard active-channel floor, one exclusion per update, the update
 * cooldown, the broad-degradation stop, the excluded-fraction cap, and
 * mandatory keyed probes before anything is restored.
 *
 * Pure: no I/O, no clock, no env, no randomness — the host feeds closed-slot
 * observations plus the current round and executes the decision. */
#ifndef DEVOURER_HOPSET_HOPSET_POLICY_H
#define DEVOURER_HOPSET_HOPSET_POLICY_H

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "hopset/HopsetState.h"

namespace devourer {
namespace hopset {

/* Explanation bits carried in the proposal's reason_bitmap. Bits 0..2 are the
 * decision's own evidence; 3..6 classify the impairment (never triggers);
 * 7..8 are the probe-driven restoration evidence. */
enum HopsetReasonBit : uint32_t {
  RB_DELIVERY_FLOOR = 1u << 0,      /* target delivery below the exclusion floor */
  RB_PERSISTENT = 1u << 1,          /* sustained for impaired_rounds rounds */
  RB_HEALTHY_ALT = 1u << 2,         /* another active channel is healthy */
  RB_CRC_DOMINANT = 1u << 3,        /* frames arrive but fail FCS */
  RB_ENERGY_INTERFERENCE = 1u << 4, /* link verdict says interference */
  RB_WEAK_SIGNAL = 1u << 5,         /* link verdict says weak signal */
  RB_SYNC_LOSS = 1u << 6,           /* nothing decoded at all on the visit */
  RB_PROBE_RECOVERY = 1u << 7,      /* probes cleared the restoration bar */
  RB_PROBE_EVIDENCE = 1u << 8,      /* decision used keyed-probe observations */

  /* Bits 9..23 belong to the transmitter-side sensing and fusion layers
   * (hopset/HopsetSense.h, hopset/HopsetFusion.h). They ride the same
   * reason_bitmap the proposal wire already carries, so adding them costs no
   * wire change and no marker version — and because the origin field encodes
   * "receiver" as zero, every proposal produced before those layers existed
   * still decodes with the right origin. */
  RB_ORIGIN_SHIFT = 9,
  RB_ORIGIN_MASK = 3u << 9, /* DecisionOrigin: 0=rx 1=tx 2=fused 3=failsafe */

  RB_TX_CCA_HIGH = 1u << 11,
  RB_TX_NHM_BUSY = 1u << 12,
  RB_TX_IGI_ELEVATED = 1u << 13,
  RB_TX_POST_BURST = 1u << 14, /* post-burst evidence — a reactive emitter */
  RB_TX_PRE_BURST = 1u << 15,

  RB_TX_CLEANER_ALT = 1u << 16,
  RB_TX_PERSISTENT = 1u << 17,
  RB_TX_SENSOR_ABSENT = 1u << 18,
  RB_TX_IGI_RAILED = 1u << 19,

  RB_FUSE_TX_DISAGREES = 1u << 20, /* endpoints differ; NOT itself a veto */
  RB_FUSE_VETO_BROAD = 1u << 21,
  RB_FUSE_VETO_REMAIN = 1u << 22,
  RB_TX_REEXAMINED = 1u << 23, /* restoring a TX-originated exclusion */
};

/* Why a decide() returned Hold — every path names exactly one. */
enum class HopsetHold : uint8_t {
  None = 0,
  InsufficientRounds,
  Cooldown,
  ProposalOutstanding,
  NoImpairment,
  NotPersistent,
  BroadDegradation,
  NoHealthyAlternative,
  MinActiveFloor,
  MaxExcludedFrac,
  ProbeHysteresisPending,
};

inline const char *hopset_hold_name(HopsetHold h) {
  switch (h) {
  case HopsetHold::None: return "none";
  case HopsetHold::InsufficientRounds: return "insufficient_rounds";
  case HopsetHold::Cooldown: return "cooldown";
  case HopsetHold::ProposalOutstanding: return "proposal_outstanding";
  case HopsetHold::NoImpairment: return "no_impairment";
  case HopsetHold::NotPersistent: return "not_persistent";
  case HopsetHold::BroadDegradation: return "broad_degradation";
  case HopsetHold::NoHealthyAlternative: return "no_healthy_alternative";
  case HopsetHold::MinActiveFloor: return "min_active_floor";
  case HopsetHold::MaxExcludedFrac: return "max_excluded_frac";
  case HopsetHold::ProbeHysteresisPending: return "probe_hysteresis_pending";
  }
  return "?";
}

/* Conservative shipping defaults. Every field is caller-tunable; the hash
 * stamps the exact configuration into each decision so a log replay is
 * reproducible. */
struct HopsetPolicyConfig {
  uint32_t min_obs_rounds = 8;       /* scored rounds before any decision */
  uint32_t impaired_rounds = 5;      /* consecutive impaired visits required */
  double exclude_delivery = 0.30;    /* per-visit delivery below this = impaired */
  double healthy_delivery = 0.70;    /* windowed delivery at/above this = healthy */
  uint32_t min_active = 3;           /* hard floor on active channels */
  uint64_t update_cooldown_rounds = 10;
  double restore_delivery = 0.70;    /* probe delivery above this counts */
  uint32_t restore_probes = 3;       /* consecutive good probes to restore */
  double broad_degrade_frac = 0.50;  /* impaired share that stops exclusion */
  double max_excluded_frac = 0.50;   /* cap on excluded/base */
  uint32_t ring_rounds = 16;         /* per-channel sliding window depth */

  uint32_t policy_hash() const {
    uint32_t h = 2166136261u;
    auto fold = [&h](uint64_t v) {
      for (int i = 0; i < 8; ++i) {
        h ^= static_cast<uint32_t>((v >> (8 * i)) & 0xff);
        h *= 16777619u;
      }
    };
    /* doubles fold as fixed-point so the hash is bit-identical everywhere */
    auto foldd = [&fold](double d) {
      fold(static_cast<uint64_t>(static_cast<int64_t>(d * 10000.0 + 0.5)));
    };
    fold(min_obs_rounds);
    fold(impaired_rounds);
    foldd(exclude_delivery);
    foldd(healthy_delivery);
    fold(min_active);
    fold(update_cooldown_rounds);
    foldd(restore_delivery);
    fold(restore_probes);
    foldd(broad_degrade_frac);
    foldd(max_excluded_frac);
    fold(ring_rounds);
    return h;
  }
};

/* One closed slot, snapshotted by the host at the dwell boundary. `decoded`
 * is the schedule-level fact (did the transmitter's marker arrive at all);
 * frames/crc_errs refine it into a ratio when the receiver surfaces corrupt
 * frames. verdict is an optional LinkVerdict value — classification only. */
struct SlotObservation {
  uint64_t slot = 0;
  uint64_t round = 0;
  uint32_t base_index = 0;
  bool is_probe = false;
  bool decoded = false;
  uint32_t frames = 0;    /* own-transmitter frames delivered in the dwell */
  uint32_t crc_errs = 0;  /* frames that arrived corrupt (when surfaced) */
  int rssi = -1;          /* raw PWDB mean, <0 = no reading */
  int snr = 0;
  int evm = 0;
  uint32_t dead_us = 0;   /* retune -> first decode; 0 when nothing decoded */
  uint8_t verdict = 255;  /* LinkVerdict, 255 = none */
};

/* A per-channel evidence summary, carried on the decision so the event log
 * explains it without re-deriving anything. */
struct HopsetChanScore {
  uint32_t visits = 0;
  uint32_t decoded = 0;
  double delivery = 0.0;       /* windowed, data visits */
  double probe_delivery = 0.0; /* windowed, probe visits */
  uint32_t probes = 0;
  uint32_t impaired_run = 0;
  bool active = false;
  bool scored = false;
};

struct HopsetDecision {
  enum class Kind : uint8_t { Hold, ProposeExclude, ProposeRestore } kind =
      Kind::Hold;
  uint64_t proposed_mask = 0;
  uint32_t reason_bitmap = 0;
  uint32_t observation_count = 0;
  HopsetHold hold = HopsetHold::None;
  size_t target_index = 0;
  uint32_t policy_hash = 0;
  std::vector<HopsetChanScore> chans;
};

class HopsetPolicy {
public:
  HopsetPolicy(const HopsetPolicyConfig &cfg, size_t n_base)
      : cfg_(cfg), n_(n_base), ch_(n_base) {
    if (!n_base || n_base > kMaxBaseChannels)
      throw std::invalid_argument("base hopset size out of range");
    mask_ = full_mask(n_base);
  }

  const HopsetPolicyConfig &config() const { return cfg_; }
  uint64_t active_mask() const { return mask_; }

  /* Fold one closed slot. Each active channel is visited exactly once per
   * round, so a data visit *is* that channel's round score — no separate
   * round-closing step, and a probe-displaced visit simply doesn't advance
   * that channel's run. */
  void ingest(const SlotObservation &o) {
    if (o.base_index >= n_)
      return;
    if (!any_seen_) {
      any_seen_ = true;
      rounds_seen_ = 1;
      last_round_ = o.round;
    } else if (o.round != last_round_) {
      ++rounds_seen_;
      last_round_ = o.round;
    }
    Chan &c = ch_[o.base_index];
    const double d = delivery_of(o);
    if (o.is_probe) {
      push(c.probe_ring, c.probe_n, c.probe_head, d);
      c.probe_good_run = d > cfg_.restore_delivery ? c.probe_good_run + 1 : 0;
      c.probes++;
      c.saw_probe = true;
    } else {
      push(c.ring, c.ring_n, c.ring_head, d);
      c.visits++;
      if (d > 0.0)
        c.decoded++;
      c.impaired_run = d < cfg_.exclude_delivery ? c.impaired_run + 1 : 0;
      /* classification: never a trigger, only an explanation */
      if (o.frames > 0 && o.crc_errs * 2 >= o.frames)
        c.crc_dominant = true;
      if (!o.decoded && o.frames == 0)
        c.sync_loss = true;
      if (o.verdict == kVerdictInterference)
        c.energy_interference = true;
      if (o.verdict == kVerdictWeak)
        c.weak_signal = true;
    }
  }

  /* A committed state took effect: re-anchor the cooldown, adopt the mask,
   * and clear the evidence of whatever just changed so a restored channel
   * starts from a clean slate instead of its pre-exclusion history. */
  void on_activation(uint64_t mask, uint64_t now_round) {
    const uint64_t changed = mask ^ mask_;
    for (size_t i = 0; i < n_; ++i)
      if (changed & (uint64_t(1) << i))
        ch_[i] = Chan{};
    mask_ = mask;
    last_update_round_ = now_round;
    have_update_ = true;
    outstanding_ = false;
  }

  /* The proposal was answered (rejected, or timed out) without activating.
   * A refused attempt still consumes the update budget: otherwise a receiver
   * whose proposals keep bouncing — exactly what happens against a jammer
   * that moves after every exclusion — re-proposes on every round boundary
   * and floods the control channel. */
  void clear_outstanding(uint64_t now_round) {
    outstanding_ = false;
    last_update_round_ = now_round;
    have_update_ = true;
  }
  bool proposal_outstanding() const { return outstanding_; }

  HopsetDecision decide(uint64_t now_round) {
    HopsetDecision d;
    d.policy_hash = cfg_.policy_hash();
    d.observation_count = rounds_seen_;
    d.chans = summarize();

    if (rounds_seen_ < cfg_.min_obs_rounds)
      return hold(d, HopsetHold::InsufficientRounds);
    if (outstanding_)
      return hold(d, HopsetHold::ProposalOutstanding);
    if (have_update_ && now_round - last_update_round_ <
                            cfg_.update_cooldown_rounds)
      return hold(d, HopsetHold::Cooldown);

    /* --- restoration first: giving a channel back is the safer move, and
     * it keeps a herding adversary from ratcheting the set down --- */
    const uint64_t excluded = full_mask(n_) & ~mask_;
    bool probe_pending = false;
    for (size_t i = 0; i < n_; ++i) {
      if (!(excluded & (uint64_t(1) << i)))
        continue;
      const Chan &c = ch_[i];
      if (c.probe_good_run >= cfg_.restore_probes) {
        d.kind = HopsetDecision::Kind::ProposeRestore;
        d.target_index = i;
        d.proposed_mask = mask_ | (uint64_t(1) << i);
        d.reason_bitmap = RB_PROBE_RECOVERY | RB_PROBE_EVIDENCE;
        outstanding_ = true;
        return d;
      }
      if (c.saw_probe)
        probe_pending = true;
    }

    /* --- exclusion: the worst sustained-impaired active channel --- */
    size_t worst = 0;
    bool have_worst = false;
    double worst_delivery = 1.0;
    unsigned impaired_active = 0, active_scored = 0;
    bool healthy_alt = false, any_impaired = false;
    for (size_t i = 0; i < n_; ++i) {
      if (!(mask_ & (uint64_t(1) << i)))
        continue;
      const Chan &c = ch_[i];
      if (!c.ring_n)
        continue;
      ++active_scored;
      const double w = window_delivery(c);
      if (w < cfg_.healthy_delivery)
        ++impaired_active;
      if (c.impaired_run > 0)
        any_impaired = true;
      /* the trigger is the sustained run of impaired visits, exactly as the
       * policy is stated; the windowed mean only ranks the candidates */
      if (c.impaired_run >= cfg_.impaired_rounds &&
          (!have_worst || w < worst_delivery)) {
        have_worst = true;
        worst = i;
        worst_delivery = w;
      }
    }
    for (size_t i = 0; i < n_; ++i) {
      if (!(mask_ & (uint64_t(1) << i)) || (have_worst && i == worst))
        continue;
      const Chan &c = ch_[i];
      if (c.ring_n && window_delivery(c) >= cfg_.healthy_delivery)
        healthy_alt = true;
    }

    if (!have_worst) {
      if (probe_pending)
        return hold(d, HopsetHold::ProbeHysteresisPending);
      return hold(d, any_impaired ? HopsetHold::NotPersistent
                                  : HopsetHold::NoImpairment);
    }
    /* When most of the band degrades together the problem is not the
     * channel — excluding would only shrink diversity for nothing. */
    if (active_scored &&
        static_cast<double>(impaired_active) /
                static_cast<double>(active_scored) >=
            cfg_.broad_degrade_frac)
      return hold(d, HopsetHold::BroadDegradation);
    if (!healthy_alt)
      return hold(d, HopsetHold::NoHealthyAlternative);

    const uint64_t after = mask_ & ~(uint64_t(1) << worst);
    const uint32_t floor_active =
        cfg_.min_active < 3u ? 3u : cfg_.min_active;
    if (popcount64(after) < floor_active)
      return hold(d, HopsetHold::MinActiveFloor);
    if (static_cast<double>(popcount64(full_mask(n_) & ~after)) /
            static_cast<double>(n_) >
        cfg_.max_excluded_frac)
      return hold(d, HopsetHold::MaxExcludedFrac);

    d.kind = HopsetDecision::Kind::ProposeExclude;
    d.target_index = worst;
    d.proposed_mask = after;
    d.reason_bitmap = RB_DELIVERY_FLOOR | RB_PERSISTENT | RB_HEALTHY_ALT;
    const Chan &c = ch_[worst];
    if (c.crc_dominant)
      d.reason_bitmap |= RB_CRC_DOMINANT;
    if (c.sync_loss)
      d.reason_bitmap |= RB_SYNC_LOSS;
    if (c.energy_interference)
      d.reason_bitmap |= RB_ENERGY_INTERFERENCE;
    if (c.weak_signal)
      d.reason_bitmap |= RB_WEAK_SIGNAL;
    outstanding_ = true;
    return d;
  }

private:
  /* LinkVerdict values used for classification, kept as plain constants so
   * this header stays free of the RX-sensor headers. */
  static constexpr uint8_t kVerdictInterference = 2;
  static constexpr uint8_t kVerdictWeak = 3;
  static constexpr size_t kRing = 64;

  struct Chan {
    /* rings hold the per-visit delivery RATIO, not a pass/fail bit: the
     * windowed mean is what "another channel remains healthy" and "most
     * channels degraded together" have to read, and a bit would score a
     * half-delivering channel as healthy. */
    double ring[kRing]{};
    uint32_t ring_n = 0, ring_head = 0;
    double probe_ring[kRing]{};
    uint32_t probe_n = 0, probe_head = 0;
    uint32_t visits = 0, decoded = 0, probes = 0;
    uint32_t impaired_run = 0, probe_good_run = 0;
    bool crc_dominant = false, sync_loss = false;
    bool energy_interference = false, weak_signal = false;
    bool saw_probe = false;
  };

  void push(double *ring, uint32_t &n, uint32_t &head, double v) {
    const uint32_t depth =
        cfg_.ring_rounds < kRing ? cfg_.ring_rounds : uint32_t(kRing);
    ring[head] = v;
    head = (head + 1) % (depth ? depth : 1);
    if (n < depth)
      ++n;
  }

  static double ratio(const double *ring, uint32_t n) {
    if (!n)
      return 0.0;
    double sum = 0.0;
    for (uint32_t i = 0; i < n; ++i)
      sum += ring[i];
    return sum / static_cast<double>(n);
  }

  static double window_delivery(const Chan &c) { return ratio(c.ring, c.ring_n); }

  double delivery_of(const SlotObservation &o) const {
    if (!o.decoded)
      return 0.0;
    if (o.frames == 0)
      return 1.0; /* the marker decoded; no per-frame detail surfaced */
    const uint32_t good = o.frames > o.crc_errs ? o.frames - o.crc_errs : 0;
    return static_cast<double>(good) / static_cast<double>(o.frames);
  }

  std::vector<HopsetChanScore> summarize() const {
    std::vector<HopsetChanScore> out(n_);
    for (size_t i = 0; i < n_; ++i) {
      const Chan &c = ch_[i];
      HopsetChanScore &s = out[i];
      s.visits = c.visits;
      s.decoded = c.decoded;
      s.delivery = window_delivery(c);
      s.probe_delivery = ratio(c.probe_ring, c.probe_n);
      s.probes = c.probes;
      s.impaired_run = c.impaired_run;
      s.active = (mask_ & (uint64_t(1) << i)) != 0;
      s.scored = c.ring_n != 0 || c.probe_n != 0;
    }
    return out;
  }

  static HopsetDecision &hold(HopsetDecision &d, HopsetHold h) {
    d.kind = HopsetDecision::Kind::Hold;
    d.hold = h;
    return d;
  }

  HopsetPolicyConfig cfg_;
  size_t n_;
  std::vector<Chan> ch_;
  uint64_t mask_ = 0;
  uint64_t last_round_ = 0, last_update_round_ = 0;
  uint32_t rounds_seen_ = 0;
  bool any_seen_ = false, have_update_ = false, outstanding_ = false;
};

} /* namespace hopset */
} /* namespace devourer */

#endif /* DEVOURER_HOPSET_HOPSET_POLICY_H */
