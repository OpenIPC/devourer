/* LinkAdapt.h — M2 (#256) per-UE link adaptation: the DL rate the grant map
 * advertises (Grant::rate_mbps), stepped from the UE's reported DL signal
 * quality and its HARQ outcome.
 *
 * Inputs, once per window (the DU's TA cadence): the mean DL SNR the UE
 * reported (the per-UE attribution of M0 contract 4, measured at the UE — NOT
 * the device-wide GetRxQuality), and the window's HARQ counts (first
 * transmissions and retransmissions + drops).
 *
 * Rules (legacy OFDM rates, 6..54 Mb/s):
 *   down — the retransmission ratio (retx + dropped) / first transmissions
 *          reaches `down_loss`: one rate down, then `hold_windows` before any
 *          step up. Also: the SNR no longer supports the current rate (below
 *          its threshold): straight down to the best rate it does support.
 *   up   — `up_windows` consecutive clean windows (loss ratio <= `up_loss`)
 *          AND an SNR at least the next rate's threshold + `hysteresis_db`:
 *          one rate up.
 * The SNR thresholds are the 802.11a minimum-SNR order of magnitude for a ~10%
 * PER at 1000 bytes; they are starting points, tunable per deployment (the
 * loss rule corrects them on air). A window with no first transmissions only
 * applies the SNR rule, and breaks a clean streak.
 * Pure header; cell_m2 ctest covers it. */
#ifndef DEVOURER_CELL_LINKADAPT_H
#define DEVOURER_CELL_LINKADAPT_H

#include <array>
#include <cstddef>
#include <cstdint>

namespace devourer::cell {

struct LinkAdaptConfig {
  std::array<int, 8> rates{6, 9, 12, 18, 24, 36, 48, 54};
  std::array<double, 8> snr_db{5, 6, 8, 11, 14, 18, 22, 24};
  double hysteresis_db = 2.0;
  double down_loss = 0.2;
  double up_loss = 0.02;
  int up_windows = 3;
  int hold_windows = 5;
};

class LinkAdapt {
public:
  explicit LinkAdapt(LinkAdaptConfig c = {}, int start_rate_mbps = 24) : cfg_(c) {
    idx_ = 0;
    for (size_t i = 0; i < cfg_.rates.size(); ++i)
      if (cfg_.rates[i] <= start_rate_mbps)
        idx_ = i;
  }
  int rate_mbps() const { return cfg_.rates[idx_]; }

  /* One adaptation window. Returns true when the rate changed. `snr_valid`
   * false: no SNR report this window (only the loss rule applies). */
  bool window(double snr_db, bool snr_valid, uint64_t first_tx, uint64_t lost) {
    const size_t before = idx_;
    const bool have_tx = first_tx > 0;
    const double loss = have_tx ? static_cast<double>(lost) / static_cast<double>(first_tx) : 0.0;
    if (hold_ > 0)
      --hold_;
    if (snr_valid && snr_db < cfg_.snr_db[idx_]) {
      /* the SNR cannot carry this rate: best supported (at least the floor) */
      size_t best = 0;
      for (size_t i = 0; i < cfg_.rates.size(); ++i)
        if (snr_db >= cfg_.snr_db[i])
          best = i;
      idx_ = best < idx_ ? best : idx_;
      clean_ = 0;
      hold_ = cfg_.hold_windows;
    } else if (have_tx && loss >= cfg_.down_loss) {
      if (idx_ > 0)
        --idx_;
      clean_ = 0;
      hold_ = cfg_.hold_windows;
    } else if (have_tx && loss <= cfg_.up_loss) {
      ++clean_;
      if (clean_ >= cfg_.up_windows && hold_ == 0 && idx_ + 1 < cfg_.rates.size() &&
          snr_valid && snr_db >= cfg_.snr_db[idx_ + 1] + cfg_.hysteresis_db) {
        ++idx_;
        clean_ = 0;
      }
    } else {
      /* lossy but not bad enough to step down, or idle (no transmissions):
       * either way no clean credit, so an upgrade needs consecutive clean
       * TRANSMITTING windows */
      clean_ = 0;
    }
    return idx_ != before;
  }

private:
  LinkAdaptConfig cfg_;
  size_t idx_ = 0;
  int clean_ = 0;
  int hold_ = 0;
};

} /* namespace devourer::cell */

#endif /* DEVOURER_CELL_LINKADAPT_H */
