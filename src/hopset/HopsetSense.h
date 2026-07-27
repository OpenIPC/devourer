/* HopsetSense — the transmitter's own view of the band, and the conservative
 * classifier that turns it into an exclusion candidate.
 *
 * The transmitter measures interference where the transmitter sits, which is
 * not where the receiver sits. That single fact governs every threshold here:
 * TX energy is a *proxy* for the thing we care about (does the video arrive),
 * whereas the receiver's delivery is a direct measurement of it. A proxy needs
 * more evidence before it may act, so everything in this module is stricter
 * than its receiver-side counterpart — more samples, longer runs, longer
 * cooldown, and two demonstrably cleaner alternatives instead of one.
 *
 * Two structural rules that are easy to get wrong:
 *
 *  - "No sensor" is not "clean". A generation with no false-alarm/CCA/IGI
 *    facility (Kestrel) must report absence, never a quiet reading, or the
 *    fusion layer will happily conclude the band is empty.
 *  - Pre-burst and post-burst observations answer different questions and are
 *    never pooled. A reactive jammer keys up only after it hears us, so it is
 *    structurally invisible before our burst and plainly visible after it;
 *    averaging the two halves hides exactly the adversary they exist to find.
 *
 * Pure: no I/O, no clock, no randomness. The host samples the chip and feeds
 * closed windows in. */
#ifndef DEVOURER_HOPSET_HOPSET_SENSE_H
#define DEVOURER_HOPSET_HOPSET_SENSE_H

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "hopset/HopsetPolicy.h"
#include "hopset/HopsetState.h"

namespace devourer {
namespace hopset {

/* Which question a sample answers. */
enum class SensePhase : uint8_t { PreBurst = 0, PostBurst = 1 };
inline constexpr size_t kSensePhases = 2;

/* Why a sample may be inadmissible. Recorded, never silently dropped. */
enum TxSenseFlag : uint16_t {
  kTsRetuneFailed = 1u << 0,
  kTsReadFailed = 1u << 1,
  kTsCounterSuspect = 1u << 2,
  kTsBarrierMissed = 1u << 3, /* taken without the discard barrier */
  kTsTruncated = 1u << 4,
  kTsNhmMissing = 1u << 5,
  kTsInjected = 1u << 6, /* synthetic evidence, no hardware behind it */
};

/* One quiet-window observation. The counters are deltas spanning exactly
 * window_us (measured, not nominal — the hardware keeps counting during the
 * read's own bus round-trips), so rates are comparable across windows of
 * different length. Every optional source carries its own valid flag. */
struct TxSenseSample {
  uint64_t slot = 0;
  uint64_t round = 0;
  uint32_t base_index = 0;
  SensePhase phase = SensePhase::PreBurst;
  uint32_t window_us = 0;
  bool probe = false; /* taken on a keyed recovery-probe dwell */

  bool valid_fa = false;
  uint32_t fa_ofdm = 0, fa_cck = 0, cca_ofdm = 0, cca_cck = 0;

  bool valid_igi = false;
  uint8_t igi = 0;

  bool valid_nhm = false;
  uint8_t nhm_busy_pct = 0; /* 100*(total-bucket0)/total, host-derived */
  uint16_t nhm_duration = 0;

  /* Synthetic evidence. A caller that sets this hands the scorer an occupancy
   * directly instead of counters, and every stage downstream — ring depth,
   * dirty-run hysteresis, the cleaner-alternative rule, the floors — treats it
   * exactly like a measured one. It exists so the two endpoint-disagreement
   * scenarios can be built on a bench where both adapters hear the same
   * interferer: the only way to give the transmitter a view that genuinely
   * differs from the receiver's is to construct it. */
  bool injected = false;
  double injected_occupancy = 0.0;

  uint16_t flags = 0;
};

/* Which sources contributed to a score — so a consumer can tell a confident
 * three-source reading from a lone noisy one. */
enum TxSenseSource : uint8_t {
  kSrcCca = 1u << 0,
  kSrcFa = 1u << 1,
  kSrcIgi = 1u << 2,
  kSrcNhm = 1u << 3,
  kSrcInjected = 1u << 4,
};

struct TxSenseConfig {
  /* --- admission --- */
  uint32_t min_window_us = 800;
  uint32_t max_counts_per_ms = 1000; /* beyond this the counter wrapped */
  uint8_t igi_floor = 0x1c;
  uint8_t igi_ceiling = 0x7f;

  /* --- occupancy scoring ---
   * The saturation rates are MEASURED, not guessed: an 8812CU sensing its own
   * dwell against a narrowband interferer that destroys delivery reads about
   * 2.8 CCA and 3.0 false alarms per millisecond, against roughly 0.6 on a
   * clean channel. Earlier estimates of 40 and 20 per millisecond were about
   * thirty times too high, which scored a fully jammed channel at 0.03 of full
   * scale — it could never have crossed any sensible threshold.
   *
   * False alarms carry more weight than CCA because they separate better: the
   * same measurement puts false alarms at roughly 5.6:1 jammed-to-clean and
   * CCA at 4.8:1, and CCA also counts legitimate traffic. */
  double cca_rate_sat = 3.0; /* events/ms mapping to 1.0 */
  double fa_rate_sat = 3.0;
  double igi_span = 32.0; /* IGI counts above the floor mapping to 1.0 */
  double w_cca = 0.30, w_fa = 0.35, w_igi = 0.15, w_nhm = 0.20;

  /* --- evidence sufficiency (deliberately stricter than the RX policy) --- */
  uint32_t min_samples_per_chan = 12;
  uint32_t min_channels_scored = 3;
  uint32_t ring_samples = 24;
  uint32_t min_obs_rounds = 24;

  /* --- exclusion trigger --- */
  double occupancy_dirty = 0.60;
  uint32_t dirty_run = 10;
  double cleaner_margin = 0.30;
  uint32_t min_cleaner_chans = 2;
  double broad_occupancy_frac = 0.50;
  double broad_spread_min = 0.25;
  bool require_post_burst = true;

  /* --- restoration through keyed probes (the TX's own recovery evidence) --- */
  double restore_occupancy = 0.25;
  uint32_t restore_probes = 3;

  /* --- pacing + the anti-herding floors, re-checked here --- */
  uint64_t cooldown_rounds = 40;
  uint32_t min_active = 3;
  double max_excluded_frac = 0.50;

  uint32_t policy_hash() const {
    uint32_t h = 2166136261u;
    auto fold = [&h](uint64_t v) {
      for (int i = 0; i < 8; ++i) {
        h ^= static_cast<uint32_t>((v >> (8 * i)) & 0xff);
        h *= 16777619u;
      }
    };
    auto foldd = [&fold](double d) {
      fold(static_cast<uint64_t>(static_cast<int64_t>(d * 10000.0 + 0.5)));
    };
    fold(min_window_us);
    fold(max_counts_per_ms);
    fold(igi_floor);
    fold(igi_ceiling);
    foldd(cca_rate_sat);
    foldd(fa_rate_sat);
    foldd(igi_span);
    foldd(w_cca);
    foldd(w_fa);
    foldd(w_igi);
    foldd(w_nhm);
    fold(min_samples_per_chan);
    fold(min_channels_scored);
    fold(ring_samples);
    fold(min_obs_rounds);
    foldd(occupancy_dirty);
    fold(dirty_run);
    foldd(cleaner_margin);
    fold(min_cleaner_chans);
    foldd(broad_occupancy_frac);
    foldd(broad_spread_min);
    fold(require_post_burst ? 1 : 0);
    foldd(restore_occupancy);
    fold(restore_probes);
    fold(cooldown_rounds);
    fold(min_active);
    foldd(max_excluded_frac);
    return h;
  }
};

struct SenseScore {
  double occupancy = 0.0;
  uint8_t sources = 0;
};

/* Normalize one sample to [0,1]. Returns false when NO source was valid — the
 * caller must not treat that as 0.0 occupancy, which is the whole point. */
inline bool score_sample(const TxSenseSample &s, const TxSenseConfig &cfg,
                         SenseScore &out) {
  const double win_ms = static_cast<double>(s.window_us) / 1000.0;
  if (win_ms <= 0.0)
    return false;
  /* Synthetic evidence is returned verbatim: the four-source weighting exists
   * to turn counters into an occupancy, and here the occupancy is what the
   * caller handed us. Back-solving it through the weights would make the
   * lever approximate, and an approximate lever proves nothing. */
  if (s.injected) {
    const double v = s.injected_occupancy;
    out.occupancy = v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v);
    out.sources = kSrcInjected;
    return true;
  }
  double acc = 0.0, wsum = 0.0;
  uint8_t src = 0;
  auto clamp01 = [](double v) { return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); };
  if (s.valid_fa) {
    const double cca = static_cast<double>(s.cca_ofdm + s.cca_cck) / win_ms;
    acc += cfg.w_cca * clamp01(cca / cfg.cca_rate_sat);
    wsum += cfg.w_cca;
    src |= kSrcCca;
    const double fa = static_cast<double>(s.fa_ofdm + s.fa_cck) / win_ms;
    acc += cfg.w_fa * clamp01(fa / cfg.fa_rate_sat);
    wsum += cfg.w_fa;
    src |= kSrcFa;
  }
  /* A railed IGI is not a reading: at the floor there is nothing to see, at
   * the ceiling the value is off-scale. Neither is linear, so neither
   * contributes — and the ceiling must never be mistaken for "maximally
   * busy". */
  if (s.valid_igi && s.igi > cfg.igi_floor && s.igi < cfg.igi_ceiling) {
    acc += cfg.w_igi *
           clamp01((static_cast<double>(s.igi) - cfg.igi_floor) / cfg.igi_span);
    wsum += cfg.w_igi;
    src |= kSrcIgi;
  }
  /* The histogram's buckets are placed relative to the gain index, so when
   * that index sits at its floor the buckets sit below the real noise floor
   * and every channel reads fully busy. A railed reading is not a measurement
   * — same rule as the gain index itself. */
  if (s.valid_nhm && s.nhm_duration > 0 && s.nhm_busy_pct > 0 &&
      s.nhm_busy_pct < 100) {
    acc += cfg.w_nhm * clamp01(static_cast<double>(s.nhm_busy_pct) / 100.0);
    wsum += cfg.w_nhm;
    src |= kSrcNhm;
  }
  if (!src || wsum <= 0.0)
    return false;
  out.occupancy = acc / wsum;
  out.sources = src;
  return true;
}

struct TxPhaseEvidence {
  uint32_t samples = 0;
  uint32_t dropped = 0;
  double occupancy = 0.0;
  double occupancy_max = 0.0;
  uint32_t dirty_run = 0;
  uint32_t clean_run = 0;
  uint8_t sources_seen = 0;
  bool scored = false;
  bool igi_railed = false;
};

struct TxChanEvidence {
  TxPhaseEvidence pre, post;
  bool active = false;
  bool sensor_absent = false;
  uint32_t probe_good_run = 0;
  uint32_t probes = 0;

  bool have_evidence() const { return post.scored || pre.scored; }
  /* Post-burst is the sharper instrument when we have it. Only ever read
   * this after have_evidence(). */
  double occupancy() const {
    return post.scored ? post.occupancy : pre.occupancy;
  }
};

enum class TxSenseHold : uint8_t {
  None = 0,
  SensorAbsent,
  InsufficientRounds,
  InsufficientSamples,
  InsufficientChannels,
  Cooldown,
  ProposalOutstanding,
  NoOccupancy,
  NotPersistent,
  PostBurstMissing,
  BroadOccupancy,
  FlatBand,
  NoCleanerAlternative,
  MinActiveFloor,
  MaxExcludedFrac,
};

inline const char *tx_sense_hold_name(TxSenseHold h) {
  switch (h) {
  case TxSenseHold::None: return "none";
  case TxSenseHold::SensorAbsent: return "sensor_absent";
  case TxSenseHold::InsufficientRounds: return "insufficient_rounds";
  case TxSenseHold::InsufficientSamples: return "insufficient_samples";
  case TxSenseHold::InsufficientChannels: return "insufficient_channels";
  case TxSenseHold::Cooldown: return "cooldown";
  case TxSenseHold::ProposalOutstanding: return "proposal_outstanding";
  case TxSenseHold::NoOccupancy: return "no_occupancy";
  case TxSenseHold::NotPersistent: return "not_persistent";
  case TxSenseHold::PostBurstMissing: return "post_burst_missing";
  case TxSenseHold::BroadOccupancy: return "broad_occupancy";
  case TxSenseHold::FlatBand: return "flat_band";
  case TxSenseHold::NoCleanerAlternative: return "no_cleaner_alternative";
  case TxSenseHold::MinActiveFloor: return "min_active_floor";
  case TxSenseHold::MaxExcludedFrac: return "max_excluded_frac";
  }
  return "?";
}

struct TxSenseCandidate {
  enum class Kind : uint8_t { None, ProposeExclude, ProposeRestore } kind =
      Kind::None;
  size_t target_index = 0;
  uint64_t proposed_mask = 0;
  uint32_t reason_bitmap = 0;
  TxSenseHold hold = TxSenseHold::None;
  uint32_t sample_count = 0;
  uint32_t policy_hash = 0;
  /* Band-level facts the fusion layer needs and must not re-derive. Reported
   * even when kind == None: the veto path needs the broad-degradation fact
   * regardless of whether the sensor itself wanted to act. */
  bool broad_occupancy = false;
  bool flat_band = false;
  double band_min_occ = 0.0, band_max_occ = 0.0;
  uint32_t chans_with_evidence = 0;
  std::vector<TxChanEvidence> chans;
};

class HopsetSensor {
public:
  HopsetSensor(const TxSenseConfig &cfg, size_t n_base)
      : cfg_(cfg), n_(n_base), ch_(n_base) {
    if (!n_base || n_base > kMaxBaseChannels)
      throw std::invalid_argument("base hopset size out of range");
    mask_ = full_mask(n_base);
  }

  const TxSenseConfig &config() const { return cfg_; }
  uint64_t active_mask() const { return mask_; }
  bool proposal_outstanding() const { return outstanding_; }

  void ingest(const TxSenseSample &s) {
    if (s.base_index >= n_)
      return;
    if (!any_seen_) {
      any_seen_ = true;
      rounds_seen_ = 1;
      last_round_ = s.round;
    } else if (s.round != last_round_) {
      ++rounds_seen_;
      last_round_ = s.round;
    }
    Chan &c = ch_[s.base_index];
    Phase &p = s.phase == SensePhase::PostBurst ? c.post : c.pre;

    /* A sample taken without the discard barrier carries the previous
     * channel's counters — worse than no sample, because it manufactures
     * cross-channel correlation. Hard drop, like a failed read. */
    if (s.flags & (kTsRetuneFailed | kTsReadFailed | kTsTruncated |
                   kTsBarrierMissed | kTsCounterSuspect)) {
      ++p.dropped;
      return;
    }
    if (s.window_us < cfg_.min_window_us) {
      ++p.dropped;
      return;
    }
    if (implausible(s)) {
      ++p.dropped;
      return;
    }
    if (s.valid_igi &&
        (s.igi <= cfg_.igi_floor || s.igi >= cfg_.igi_ceiling))
      p.igi_railed = true;

    SenseScore sc;
    if (!score_sample(s, cfg_, sc)) {
      ++p.dropped;
      if (!c.pre.n && !c.post.n)
        c.sensor_absent = true;
      return;
    }
    c.sensor_absent = false;
    p.sources_seen |= sc.sources;
    push(p, sc.occupancy);
    p.dirty_run = sc.occupancy >= cfg_.occupancy_dirty ? p.dirty_run + 1 : 0;
    p.clean_run = sc.occupancy < cfg_.occupancy_dirty ? p.clean_run + 1 : 0;
    ++total_samples_;

    /* Probe dwells are the TX's own recovery evidence: they are the only
     * visits an excluded channel gets, and the payload is already suppressed
     * on them, so they cost nothing extra. */
    if (s.probe) {
      ++c.probes;
      c.probe_good_run =
          sc.occupancy <= cfg_.restore_occupancy ? c.probe_good_run + 1 : 0;
    }
  }

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

  /* The proposal was answered without activating. Like the RX policy, a
   * refused attempt still spends the update budget. */
  void clear_outstanding(uint64_t now_round) {
    outstanding_ = false;
    last_update_round_ = now_round;
    have_update_ = true;
  }

  TxSenseCandidate evaluate(uint64_t now_round) {
    TxSenseCandidate d;
    d.policy_hash = cfg_.policy_hash();
    d.sample_count = total_samples_;
    d.chans = summarize();
    band_stats(d);

    if (rounds_seen_ < cfg_.min_obs_rounds)
      return hold(d, TxSenseHold::InsufficientRounds);
    if (outstanding_)
      return hold(d, TxSenseHold::ProposalOutstanding);
    if (have_update_ && now_round - last_update_round_ < cfg_.cooldown_rounds)
      return hold(d, TxSenseHold::Cooldown);
    if (d.chans_with_evidence == 0) {
      for (size_t i = 0; i < n_; ++i)
        if (ch_[i].sensor_absent)
          return hold(d, TxSenseHold::SensorAbsent);
      return hold(d, TxSenseHold::InsufficientSamples);
    }

    /* Restoration first, as on the receiver side: handing a channel back
     * enlarges the option set, and it is the path that keeps an autonomous
     * exclusion from being permanent. */
    const uint64_t excluded = full_mask(n_) & ~mask_;
    for (size_t i = 0; i < n_; ++i) {
      if (!(excluded & (uint64_t(1) << i)))
        continue;
      if (ch_[i].probe_good_run >= cfg_.restore_probes) {
        d.kind = TxSenseCandidate::Kind::ProposeRestore;
        d.target_index = i;
        d.proposed_mask = mask_ | (uint64_t(1) << i);
        d.reason_bitmap = 0;
        outstanding_ = true;
        return d;
      }
    }

    if (d.chans_with_evidence < cfg_.min_channels_scored)
      return hold(d, TxSenseHold::InsufficientChannels);

    /* The worst sustained-dirty active channel. When post-burst evidence is
     * required, the run that counts is the post-burst one — that is the
     * population a reactive jammer shows up in. */
    bool have_worst = false, any_dirty = false, post_missing = false;
    size_t worst = 0;
    double worst_occ = -1.0;
    for (size_t i = 0; i < n_; ++i) {
      if (!(mask_ & (uint64_t(1) << i)))
        continue;
      const Chan &c = ch_[i];
      if (!c.have_evidence())
        continue;
      const Phase &gate = cfg_.require_post_burst ? c.post : worse_phase(c);
      if (gate.dirty_run > 0)
        any_dirty = true;
      if (gate.dirty_run < cfg_.dirty_run)
        continue;
      if (cfg_.require_post_burst && !ph_scored(c.post)) {
        post_missing = true;
        continue;
      }
      const double occ = c.occupancy();
      if (!have_worst || occ > worst_occ) {
        have_worst = true;
        worst = i;
        worst_occ = occ;
      }
    }
    /* Pre-burst dirt with no post-burst corroboration is reported as its own
     * hold, not folded into "nothing here": it is the signature of a parked
     * interferer the receiver is better placed to judge. */
    if (!have_worst && cfg_.require_post_burst && !post_missing)
      for (size_t i = 0; i < n_; ++i)
        if ((mask_ & (uint64_t(1) << i)) && ph_scored(ch_[i].pre) &&
            ch_[i].pre.dirty_run >= cfg_.dirty_run &&
            !ph_scored(ch_[i].post))
          post_missing = true;

    if (!have_worst) {
      if (post_missing)
        return hold(d, TxSenseHold::PostBurstMissing);
      return hold(d, any_dirty ? TxSenseHold::NotPersistent
                               : TxSenseHold::NoOccupancy);
    }
    if (samples_of(ch_[worst]) < cfg_.min_samples_per_chan)
      return hold(d, TxSenseHold::InsufficientSamples);
    /* A band with no spread cannot be attributed to a channel, and a band
     * where most of it is dirty is not a channel fault at all. */
    if (d.flat_band)
      return hold(d, TxSenseHold::FlatBand);
    if (d.broad_occupancy)
      return hold(d, TxSenseHold::BroadOccupancy);

    /* Two demonstrably cleaner alternatives, not one: a single clean channel
     * is exactly what a herding jammer manufactures by parking on everything
     * else. */
    uint32_t cleaner = 0;
    for (size_t i = 0; i < n_; ++i) {
      if (i == worst || !(mask_ & (uint64_t(1) << i)))
        continue;
      const Chan &c = ch_[i];
      if (c.have_evidence() && c.occupancy() <= worst_occ - cfg_.cleaner_margin)
        ++cleaner;
    }
    if (cleaner < cfg_.min_cleaner_chans)
      return hold(d, TxSenseHold::NoCleanerAlternative);

    const uint64_t after = mask_ & ~(uint64_t(1) << worst);
    const uint32_t floor_active = cfg_.min_active < 3u ? 3u : cfg_.min_active;
    if (popcount64(after) < floor_active)
      return hold(d, TxSenseHold::MinActiveFloor);
    if (static_cast<double>(popcount64(full_mask(n_) & ~after)) /
            static_cast<double>(n_) >
        cfg_.max_excluded_frac)
      return hold(d, TxSenseHold::MaxExcludedFrac);

    d.kind = TxSenseCandidate::Kind::ProposeExclude;
    d.target_index = worst;
    d.proposed_mask = after;
    d.reason_bitmap = build_reasons(ch_[worst]);
    outstanding_ = true;
    return d;
  }

private:
  struct Phase {
    double ring[64]{};
    uint32_t n = 0, head = 0;
    uint32_t samples = 0, dropped = 0;
    uint32_t dirty_run = 0, clean_run = 0;
    uint8_t sources_seen = 0;
    bool igi_railed = false;
  };
  struct Chan {
    Phase pre, post;
    bool sensor_absent = false;
    uint32_t probe_good_run = 0, probes = 0;
    bool have_evidence() const { return pre.n || post.n; }
    double occupancy() const {
      return post.n ? mean(post) : mean(pre);
    }
    static double mean(const Phase &p) {
      if (!p.n)
        return 0.0;
      double s = 0.0;
      for (uint32_t i = 0; i < p.n; ++i)
        s += p.ring[i];
      return s / static_cast<double>(p.n);
    }
  };

  bool ph_scored(const Phase &p) const {
    return p.samples >= cfg_.min_samples_per_chan;
  }
  static const Phase &worse_phase(const Chan &c) {
    return c.post.n >= c.pre.n ? c.post : c.pre;
  }
  uint32_t samples_of(const Chan &c) const {
    return cfg_.require_post_burst ? c.post.samples
                                   : c.pre.samples + c.post.samples;
  }
  /* "We have a usable read of this channel" — either phase will do. Whether
   * post-burst corroboration is REQUIRED to act on it is the exclusion
   * trigger's rule, not a property of the evidence. */
  bool scored(const Chan &c) const {
    return ph_scored(c.pre) || ph_scored(c.post);
  }

  void push(Phase &p, double v) {
    const uint32_t depth =
        cfg_.ring_samples < 64 ? cfg_.ring_samples : 64u;
    p.ring[p.head] = v;
    p.head = (p.head + 1) % (depth ? depth : 1);
    if (p.n < depth)
      ++p.n;
    ++p.samples;
  }

  bool implausible(const TxSenseSample &s) const {
    if (!s.valid_fa)
      return false;
    const double win_ms = static_cast<double>(s.window_us) / 1000.0;
    const double cap = cfg_.max_counts_per_ms * win_ms;
    return s.cca_ofdm > cap || s.cca_cck > cap || s.fa_ofdm > cap ||
           s.fa_cck > cap;
  }

  uint32_t build_reasons(const Chan &c) const {
    uint32_t r = 0;
    const Phase &p = c.post.n ? c.post : c.pre;
    if (p.sources_seen & kSrcCca)
      r |= RB_TX_CCA_HIGH;
    if (p.sources_seen & kSrcNhm)
      r |= RB_TX_NHM_BUSY;
    if (p.sources_seen & kSrcIgi)
      r |= RB_TX_IGI_ELEVATED;
    if (c.post.n)
      r |= RB_TX_POST_BURST;
    if (c.pre.n)
      r |= RB_TX_PRE_BURST;
    if (p.igi_railed)
      r |= RB_TX_IGI_RAILED;
    r |= RB_TX_CLEANER_ALT | RB_TX_PERSISTENT;
    return r;
  }

  std::vector<TxChanEvidence> summarize() const {
    std::vector<TxChanEvidence> out(n_);
    for (size_t i = 0; i < n_; ++i) {
      const Chan &c = ch_[i];
      TxChanEvidence &e = out[i];
      auto fill = [&](const Phase &p, TxPhaseEvidence &o, bool is_post) {
        o.samples = p.samples;
        o.dropped = p.dropped;
        o.occupancy = Chan::mean(p);
        o.dirty_run = p.dirty_run;
        o.clean_run = p.clean_run;
        o.sources_seen = p.sources_seen;
        o.igi_railed = p.igi_railed;
        double mx = 0.0;
        for (uint32_t k = 0; k < p.n; ++k)
          mx = p.ring[k] > mx ? p.ring[k] : mx;
        o.occupancy_max = mx;
        o.scored = p.samples >= cfg_.min_samples_per_chan;
        (void)is_post;
      };
      fill(c.pre, e.pre, false);
      fill(c.post, e.post, true);
      e.active = (mask_ & (uint64_t(1) << i)) != 0;
      e.sensor_absent = c.sensor_absent;
      e.probe_good_run = c.probe_good_run;
      e.probes = c.probes;
    }
    return out;
  }

  void band_stats(TxSenseCandidate &d) const {
    double mn = 2.0, mx = -1.0;
    uint32_t scored_n = 0, dirty_n = 0;
    for (size_t i = 0; i < n_; ++i) {
      if (!(mask_ & (uint64_t(1) << i)))
        continue;
      const Chan &c = ch_[i];
      if (!c.have_evidence() || !scored(c))
        continue;
      ++scored_n;
      const double o = c.occupancy();
      if (o < mn)
        mn = o;
      if (o > mx)
        mx = o;
      if (o >= cfg_.occupancy_dirty)
        ++dirty_n;
    }
    d.chans_with_evidence = scored_n;
    d.band_min_occ = scored_n ? mn : 0.0;
    d.band_max_occ = scored_n ? mx : 0.0;
    d.broad_occupancy =
        scored_n && static_cast<double>(dirty_n) /
                            static_cast<double>(scored_n) >=
                        cfg_.broad_occupancy_frac;
    /* Raw geometry only: "the channels look alike". Whether that means doubt
     * (alike and dirty) or confidence (alike and clean) is the fusion layer's
     * call, which is where the dirty floor lives. The sensor's own use below
     * is unambiguous because it is only reached once something was dirty. */
    d.flat_band = scored_n >= cfg_.min_channels_scored &&
                  (d.band_max_occ - d.band_min_occ) < cfg_.broad_spread_min;
  }

  static TxSenseCandidate &hold(TxSenseCandidate &d, TxSenseHold h) {
    d.kind = TxSenseCandidate::Kind::None;
    d.hold = h;
    return d;
  }

  TxSenseConfig cfg_;
  size_t n_;
  std::vector<Chan> ch_;
  uint64_t mask_ = 0;
  uint64_t last_round_ = 0, last_update_round_ = 0;
  uint32_t rounds_seen_ = 0, total_samples_ = 0;
  bool any_seen_ = false, have_update_ = false, outstanding_ = false;
};

} /* namespace hopset */
} /* namespace devourer */

#endif /* DEVOURER_HOPSET_HOPSET_SENSE_H */
