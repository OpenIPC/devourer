/* Active-link evidence — the primary receiver's view of the LIVE video
 * channel, the authoritative half of a migration decision.
 *
 * The asymmetry the whole stack is built around: on the active channel the
 * scout's energy reading is confounded by the wanted video's own airtime, so
 * it is NOT the impairment signal — the primary receiver's DELIVERY is. This
 * models that delivery as a rolling window of `ActiveLinkWindow` records
 * (fed from the primary's rx.txhit / rx.quality / rx.energy JSONL) and
 * classifies each window as impaired-or-not with a fault domain, so a policy
 * can require PERSISTENT channel-attributable impairment before it even looks
 * at candidates.
 *
 * Fault domains matter because migration cannot fix them: a weak signal is a
 * range problem (add power / close distance, not change channel), near-field
 * saturation is an overload problem (back power off), and a broad multi-band
 * degradation is not this channel's fault. Only "decodes are being lost to
 * something local to this channel" is a migration trigger.
 *
 * Pure: no clocks (caller passes now), no I/O. Selftested via ChannelScore. */
#ifndef DEVOURER_CHANMIG_ACTIVE_LINK_H
#define DEVOURER_CHANMIG_ACTIVE_LINK_H

#include <cstdint>
#include <vector>

#include "LinkHealth.h"

namespace devourer {
namespace chanmig {

/* 12-bit 802.11 sequence-number loss estimator (wrap-aware). Fed the seq of
 * each delivered canonical-SA frame; a gap up to half the space is counted as
 * that many expected-but-missing frames, a larger jump is a TX restart (one
 * expected). delivered/expected over a window give the loss ratio. */
class SeqLossEstimator {
public:
  void observe(uint16_t seq) {
    ++delivered_;
    if (have_prev_) {
      const uint16_t gap = static_cast<uint16_t>((seq - prev_) & 0xFFF);
      expected_ += (gap >= 1 && gap < 2048) ? gap : 1;
    } else {
      expected_ += 1;
      have_prev_ = true;
    }
    prev_ = seq;
  }
  uint32_t delivered() const { return delivered_; }
  uint32_t expected() const { return expected_ < delivered_ ? delivered_
                                                            : expected_; }
  double loss_ratio() const {
    const uint32_t e = expected();
    return e ? 1.0 - static_cast<double>(delivered_) / e : 0.0;
  }
  void reset() {
    delivered_ = 0;
    expected_ = 0;
    have_prev_ = false;
  }

private:
  uint32_t delivered_ = 0, expected_ = 0;
  uint16_t prev_ = 0;
  bool have_prev_ = false;
};

/* One primary-telemetry window on the active channel. Optional fields carry a
 * validity flag; the impairment test uses whatever delivery evidence is
 * present, preferring the strongest (FEC margin > seq loss > crc/icv rate >
 * frame collapse). */
struct ActiveLinkWindow {
  int64_t t_ms = 0;

  bool have_delivery = false;
  uint32_t delivered = 0; /* frames this window */
  uint32_t expected = 0;  /* seq-gap estimate; 0 = unknown */

  bool have_fec = false;
  double fec_margin = 0.0; /* external feeder (wfb-ng); <=0 = at/over the edge */

  bool have_crc = false;
  uint32_t crc_err = 0, icv_err = 0; /* DEVOURER_RX_KEEP_CORRUPTED */

  bool have_quality = false;
  LinkVerdict verdict = LinkVerdict::NoSignal;
  int rssi_max_dbm = 0;
  double snr_mean_db = 0.0, evm_mean_db = 0.0;

  bool telemetry_ok = true; /* false when the primary feed went stale/missing */
};

/* Impairment verdict for one window: whether it counts as impaired, and if
 * not-channel-attributable, why (so a policy can say "don't migrate, it's
 * weak-signal"). */
enum class Impair {
  Healthy,       /* delivering fine */
  Channel,       /* losing decodes to something on THIS channel — migratable */
  WeakSignal,    /* range/sensitivity limit — migration won't help */
  Saturation,    /* near-field overload — back power off */
  TelemetryDown, /* no trustworthy primary evidence */
};

inline const char *verdict_label(LinkVerdict v) {
  switch (v) {
  case LinkVerdict::NoSignal: return "NO_SIGNAL";
  case LinkVerdict::Saturated: return "SATURATED";
  case LinkVerdict::Interference: return "INTERFERENCE";
  case LinkVerdict::Weak: return "WEAK";
  case LinkVerdict::Marginal: return "MARGINAL";
  case LinkVerdict::Healthy: return "HEALTHY";
  }
  return "?";
}

struct ActivePolicy {
  double loss_impair = 0.05;      /* >5% seq loss = impaired */
  double crc_impair = 0.10;       /* >10% crc/icv of delivered = impaired */
  double fec_margin_impair = 0.0; /* fec_margin <= this = impaired */
  uint32_t min_frames = 20;       /* fewer = window not judgeable on delivery */
};

/* Classify one window. LinkHealth's verdict is reused as the fault-domain
 * discriminator (its EVM/RSSI split already separates weak from saturated
 * from interfered — we do not re-derive it). */
inline Impair classify_window(const ActiveLinkWindow &w,
                              const ActivePolicy &p) {
  if (!w.telemetry_ok)
    return Impair::TelemetryDown;

  /* Fault domain first: a weak or saturated link is not a channel problem no
   * matter what delivery does. */
  if (w.have_quality) {
    if (w.verdict == LinkVerdict::Weak)
      return Impair::WeakSignal;
    if (w.verdict == LinkVerdict::Saturated)
      return Impair::Saturation;
    if (w.verdict == LinkVerdict::NoSignal && w.delivered == 0)
      return Impair::WeakSignal; /* nothing at all: treat as range, not channel */
  }

  bool impaired = false;
  if (w.have_fec)
    impaired = impaired || w.fec_margin <= p.fec_margin_impair;
  if (w.have_delivery && w.expected >= p.min_frames) {
    const double loss =
        w.expected ? 1.0 - static_cast<double>(w.delivered) / w.expected : 0.0;
    impaired = impaired || loss > p.loss_impair;
  }
  if (w.have_crc && w.delivered >= p.min_frames) {
    const double bad = static_cast<double>(w.crc_err + w.icv_err) /
                       (w.delivered + w.crc_err + w.icv_err);
    impaired = impaired || bad > p.crc_impair;
  }
  /* A window with quality evidence but INTERFERENCE verdict and no delivery
   * signal still counts as channel-impaired (raised floor, high FA). */
  if (!impaired && w.have_quality && w.verdict == LinkVerdict::Interference &&
      !w.have_delivery && !w.have_fec && !w.have_crc)
    impaired = true;

  return impaired ? Impair::Channel : Impair::Healthy;
}

/* Rolling window of active-link records + the derived persistence signals a
 * decision needs: consecutive channel-impaired windows, and whether the
 * newest window's fault domain is a non-channel one (which forces a hold). */
class ActiveLinkTrack {
public:
  explicit ActiveLinkTrack(ActivePolicy p = {}, size_t cap = 64)
      : policy_(p), cap_(cap) {}

  void add(const ActiveLinkWindow &w) {
    const Impair im = classify_window(w, policy_);
    if (im == Impair::Channel)
      ++consec_channel_;
    else
      consec_channel_ = 0;
    last_ = im;
    last_t_ms_ = w.t_ms;
    have_last_ = true;
    if (win_.size() >= cap_)
      win_.erase(win_.begin());
    win_.push_back(w);
  }

  int consecutive_channel_impaired() const { return consec_channel_; }
  Impair last_domain() const { return last_; }
  bool have() const { return have_last_; }
  int64_t last_t_ms() const { return last_t_ms_; }

  /* Newest window's verdict, for the decision's active_verdict field. */
  LinkVerdict last_verdict() const {
    return win_.empty() ? LinkVerdict::NoSignal : win_.back().verdict;
  }

private:
  ActivePolicy policy_;
  size_t cap_;
  std::vector<ActiveLinkWindow> win_;
  int consec_channel_ = 0;
  Impair last_ = Impair::TelemetryDown;
  int64_t last_t_ms_ = 0;
  bool have_last_ = false;
};

} /* namespace chanmig */
} /* namespace devourer */

#endif /* DEVOURER_CHANMIG_ACTIVE_LINK_H */
