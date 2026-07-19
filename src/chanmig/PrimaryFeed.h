/* PrimaryFeed — turn the primary receiver's JSONL stream into ActiveLinkWindow
 * records the RecommendEngine consumes.
 *
 * The primary receiver (rxdemo parked on the video channel) is a SEPARATE
 * process; its stdout is redirected to a file the scout tail-follows. Its
 * relevant events:
 *   rx.txhit    per canonical-SA frame — seq feeds the loss estimator
 *   rx.quality  windowed verdict + rssi/snr/evm (DEVOURER_RXQUALITY)
 *   link.health windowed verdict (DEVOURER_LINKHEALTH)
 *   rx.energy   window boundary + crc_err/icv_err (DEVOURER_RX_KEEP_CORRUPTED)
 * A windowed event (rx.quality preferred, else link.health) closes an
 * ActiveLinkWindow: the frames seen since the last close become its delivery,
 * the verdict its fault domain. Pure line-in / windows-out — no file I/O
 * (the demo does the tailing) and no clock (the caller stamps arrival time).
 *
 * Never a FIFO on the producer side: a wedged scout must not be able to block
 * the primary video receiver, so the contract is an append-only file the
 * scout polls. */
#ifndef DEVOURER_CHANMIG_PRIMARY_FEED_H
#define DEVOURER_CHANMIG_PRIMARY_FEED_H

#include <cstring>
#include <string>
#include <string_view>
#include <vector>

#include "LinkHealth.h"
#include "chanmig/ActiveLink.h"
#include "chanmig/JsonlLite.h"

namespace devourer {
namespace chanmig {

inline LinkVerdict verdict_from_label(std::string_view s) {
  if (s == "HEALTHY")
    return LinkVerdict::Healthy;
  if (s == "SATURATED")
    return LinkVerdict::Saturated;
  if (s == "INTERFERENCE")
    return LinkVerdict::Interference;
  if (s == "WEAK")
    return LinkVerdict::Weak;
  if (s == "MARGINAL")
    return LinkVerdict::Marginal;
  return LinkVerdict::NoSignal;
}

class PrimaryFeedReader {
public:
  /* Feed one line; returns true and fills `out` when this line closed a
   * window. arrival_ms is the scout's own monotonic clock at read time (the
   * two processes share no clock; window granularity dwarfs the skew). */
  bool line(std::string_view l, int64_t arrival_ms, ActiveLinkWindow &out) {
    if (jsonl_ev_is(l, "rx.txhit")) {
      long long seq;
      if (jsonl_int(l, "seq", &seq))
        seq_.observe(static_cast<uint16_t>(seq));
      ++frames_since_;
      saw_frames_ = true;
      return false;
    }
    if (jsonl_ev_is(l, "rx.energy")) {
      long long v;
      if (jsonl_int(l, "crc_err", &v)) {
        crc_ += static_cast<uint32_t>(v);
        have_crc_ = true;
      }
      if (jsonl_int(l, "icv_err", &v))
        icv_ += static_cast<uint32_t>(v);
      return false;
    }
    const bool q = jsonl_ev_is(l, "rx.quality");
    const bool h = !q && jsonl_ev_is(l, "link.health");
    if (!q && !h)
      return false;
    /* link.health is skipped when rx.quality is also on (avoid double-closing
     * one window); prefer rx.quality's richer fields. */
    if (h && seen_quality_)
      return false;
    if (q)
      seen_quality_ = true;

    out = ActiveLinkWindow{};
    out.t_ms = arrival_ms;
    out.telemetry_ok = true;
    std::string label;
    if (jsonl_str(l, "verdict", &label)) {
      out.have_quality = true;
      out.verdict = verdict_from_label(label);
    }
    double d;
    if (jsonl_num(l, "rssi_max_dbm", &d))
      out.rssi_max_dbm = static_cast<int>(d);
    else if (jsonl_num(l, "rssi_dbm", &d))
      out.rssi_max_dbm = static_cast<int>(d);
    if (jsonl_num(l, "snr_mean_db", &d))
      out.snr_mean_db = d;
    else if (jsonl_num(l, "snr_db", &d))
      out.snr_mean_db = d;
    if (jsonl_num(l, "evm_db", &d))
      out.evm_mean_db = d;

    if (saw_frames_) {
      out.have_delivery = true;
      out.delivered = seq_.delivered();
      out.expected = seq_.expected();
    }
    if (have_crc_) {
      out.have_crc = true;
      out.crc_err = crc_;
      out.icv_err = icv_;
    }
    /* reset per-window accumulators */
    seq_.reset();
    frames_since_ = 0;
    saw_frames_ = false;
    crc_ = icv_ = 0;
    have_crc_ = false;
    return true;
  }

private:
  SeqLossEstimator seq_;
  uint32_t frames_since_ = 0, crc_ = 0, icv_ = 0;
  bool saw_frames_ = false, have_crc_ = false, seen_quality_ = false;
};

} /* namespace chanmig */
} /* namespace devourer */

#endif /* DEVOURER_CHANMIG_PRIMARY_FEED_H */
