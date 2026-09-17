/* SenseWindow — the settle -> DISCARD BARRIER -> observe -> read discipline
 * that every frame-free measurement in this codebase follows, in one place.
 *
 * The discipline matters more than it looks. The chip's FA/CCA counters are
 * delta-on-read, so a reading is only about the channel you are on if a
 * throwaway read resets them AFTER the retune has settled and BEFORE the
 * window opens. Miss that and the record silently carries the previous
 * channel's energy; nothing downstream can tell.
 *
 *   caller sleeps the settle        <- caller owns this: it owns stop-awareness
 *   w.barrier(opt);                 <- resets the chip's delta counters
 *   caller sleeps the window
 *   SenseResult r = w.read(opt);    <- everything between the two calls
 *
 * The two sleeps are deliberately the CALLER's. chanscout wants a 50 ms
 * stop-aware chunked nap so SIGINT is responsive; the hopset TX side wants a
 * settle that outlasts its own queued frames draining. Those are policy, and a
 * library that slept would have to pick one. The library owns no thread, takes
 * no clock of record, and performs no sleep.
 *
 * `window_us` is MEASURED, not nominal: the hardware keeps counting during the
 * read's own bus round-trips, and excluding that time would inflate every rate
 * derived from it.
 *
 * Single control thread, like every other control-plane entry point. */
#ifndef DEVOURER_SENSING_SENSE_WINDOW_H
#define DEVOURER_SENSING_SENSE_WINDOW_H

#include <chrono>
#include <cstdint>
#include <functional>

#include "IRtlRadio.h"
#include "RxSense.h"

namespace devourer {
namespace sensing {

/* Monotonic microseconds, used only to MEASURE durations inside one call (the
 * retune, the read's own round-trips). Record timestamps still come from the
 * caller's `now`, chanmig-style — this is the second hand, not the clock of
 * record. Injected so a selftest can make every duration an exact integer. */
using MonotonicUs = std::function<int64_t()>;
int64_t steady_us();

struct SenseOptions {
  bool with_nhm = true;
  /* Skip every hardware touch but keep the phase structure and the caller's
   * time budget intact — the synthetic-evidence lever (examples/tx's injected
   * occupancy), and the honest shape on a part whose front end is blind on
   * this band. */
  bool skip_hardware = false;
  /* A delta beyond this many events per millisecond of MEASURED window is a
   * wrapped or reset counter, not a busy channel. */
  uint32_t counter_ceiling_per_ms = 1000;
};

/* The NHM histogram reduced to the three numbers consumers actually carry.
 * `valid` is present-AND-non-empty; RxEnergy::valid_nhm remains available on
 * SenseResult::energy for a caller that keys on the raw flag instead. */
struct NhmReduction {
  bool valid = false;
  uint32_t total = 0;
  uint8_t busy_pct = 0; /* mass above bucket 0 */
  uint8_t env_pct = 0;  /* mass above the receiver's own floor */
  uint8_t peak_bucket = 0;
  uint16_t duration = 0;
};

inline NhmReduction reduce_nhm(const RxEnergy &e) {
  NhmReduction r;
  if (!e.valid_nhm)
    return r;
  uint32_t total = 0, peak = 0;
  int peak_k = 0;
  for (int k = 0; k < 12; k++) {
    total += e.nhm[k];
    if (e.nhm[k] > peak) {
      peak = e.nhm[k];
      peak_k = k;
    }
  }
  r.total = total;
  r.valid = total > 0;
  if (!r.valid)
    return r;
  r.busy_pct = static_cast<uint8_t>(100u * (total - e.nhm[0]) / total);
  r.env_pct = e.nhm_env_ratio_pct;
  r.peak_bucket = static_cast<uint8_t>(peak_k);
  r.duration = e.nhm_duration;
  return r;
}

/* True when any FA/CCA delta exceeds `ceiling_per_ms` events per millisecond
 * of `window_us`. Always false when the counters are absent or the window is
 * unknown: an absent counter is not a suspect counter. */
inline bool counters_implausible(const RxEnergy &e, int64_t window_us,
                                 uint32_t ceiling_per_ms) {
  if (!e.valid_fa || window_us <= 0)
    return false;
  const uint64_t ceiling =
      static_cast<uint64_t>(ceiling_per_ms) * static_cast<uint64_t>(window_us) /
      1000ull;
  return e.cca_ofdm > ceiling || e.fa_ofdm > ceiling || e.cca_cck > ceiling ||
         e.fa_cck > ceiling;
}

struct SenseResult {
  RxEnergy energy{};
  int64_t window_us = 0; /* MEASURED: barrier-close .. read-complete */
  NhmReduction nhm{};
  bool counters_suspect = false;
  bool nhm_missing = false;
  /* LATCHED: a generation whose counters were valid once and are not now had a
   * read failure. A generation that never had them is simply unported — that
   * is not a failure and must not be flagged as one. */
  bool read_failed = false;
};

class SenseWindow {
public:
  /* `rtl` may be null: a non-Realtek radio has no phydm counters, which is a
   * first-class case and not an error. The barrier becomes a no-op and the
   * read returns an all-invalid snapshot with nhm_missing set. */
  explicit SenseWindow(IRtlRadio *rtl, MonotonicUs clock = nullptr)
      : rtl_(rtl), clock_(clock ? std::move(clock) : MonotonicUs(&steady_us)) {}

  void barrier(const SenseOptions &opt) {
    if (rtl_ && !opt.skip_hardware)
      (void)rtl_->GetRxEnergy(/*with_nhm=*/false);
    barrier_us_ = clock_();
  }

  SenseResult read(const SenseOptions &opt) {
    SenseResult r;
    if (rtl_ && !opt.skip_hardware)
      r.energy = rtl_->GetRxEnergy(opt.with_nhm);
    r.window_us = clock_() - barrier_us_;

    r.nhm = reduce_nhm(r.energy);
    r.nhm_missing = !r.nhm.valid;
    r.counters_suspect =
        counters_implausible(r.energy, r.window_us, opt.counter_ceiling_per_ms);

    if (r.energy.valid_fa)
      ever_valid_ = true;
    invalid_streak_ = r.energy.valid_fa ? 0 : invalid_streak_ + 1;
    r.read_failed = ever_valid_ && !r.energy.valid_fa;
    return r;
  }

  bool energy_ever_valid() const { return ever_valid_; }
  int invalid_streak() const { return invalid_streak_; }

private:
  IRtlRadio *rtl_;
  MonotonicUs clock_;
  int64_t barrier_us_ = 0;
  bool ever_valid_ = false;
  int invalid_streak_ = 0;
};

} /* namespace sensing */
} /* namespace devourer */

#endif /* DEVOURER_SENSING_SENSE_WINDOW_H */
