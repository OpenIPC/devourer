/* SurveyFrameAggregator — the frame-driven half of a channel survey dwell.
 *
 * Push side: the RX callback folds every decoded frame in as it arrives.
 * Pull side: the dwell loop zeroes it at the discard barrier and drains it at
 * the end of the observation window. Those are different threads, so the fold
 * is locked; the lock is held for the fold alone and never across the airtime
 * estimate, which is pure arithmetic.
 *
 * This was a file-static in examples/chanscout with a hardcoded source
 * address. Two things changed on the way in: it is a value type the caller
 * owns, and the attribution key is CONFIGURATION. The canonical devourer SA is
 * the demos' choice, not the library's — an integrator flying their own SA
 * needs the same ours-vs-foreign split, and a library that hardcodes one
 * silently scores their own video as interference. */
#ifndef DEVOURER_SENSING_SURVEY_FRAME_AGG_H
#define DEVOURER_SENSING_SURVEY_FRAME_AGG_H

#include <array>
#include <cstdint>
#include <cstring>
#include <mutex>

#include "chanmig/SurveyRecord.h" /* frame_airtime_us */

namespace devourer {
namespace sensing {

struct SurveyAggConfig {
  /* Frames whose 802.11 SA (addr2, bytes [10..16) of the MPDU) equals this are
   * OURS. Leave own_sa_valid false and every frame counts as foreign, which is
   * the right default for a scout that is not also the transmitter. */
  std::array<uint8_t, 6> own_sa{};
  bool own_sa_valid = false;
};

/* One dwell's frame-driven terms, in raw devourer units — exactly the
 * SurveyDwell fields that come from decoded frames rather than the energy
 * counters. */
struct SurveyFrameWindow {
  uint32_t frames = 0;
  int rssi_mean_raw = 0, rssi_max_raw = 0;
  int snr_mean_raw = 0, snr_min_raw = 0;
  int evm_mean_raw = 0;
  bool evm_valid = false;
  uint32_t own_frames = 0;
  uint64_t own_air_us = 0, other_air_us = 0;
};

class SurveyFrameAggregator {
public:
  explicit SurveyFrameAggregator(const SurveyAggConfig &cfg = {}) : cfg_(cfg) {}

  /* Fold one decoded frame. Called on the RX thread.
   *
   * `fcs_present` is not cosmetic: airtime is what occupied the channel, and
   * the FCS was transmitted even where the MAC strips it before DMA — so pass
   * false and the 4 bytes are added back, or occupancy reads light on every
   * frame.
   *
   * A frame with rssi_raw <= 0 carries no usable quality sample (the parser
   * had nothing), but its AIRTIME still counts: it occupied the channel
   * regardless of whether we could measure it. */
  void add(const uint8_t *mpdu, size_t len, uint16_t desc_rate, uint8_t bw,
           bool sgi, bool fcs_present, int rssi_raw, int snr_raw, int evm_raw) {
    const uint32_t on_air_len =
        static_cast<uint32_t>(len) + (fcs_present ? 0u : 4u);
    const uint32_t air =
        chanmig::frame_airtime_us(desc_rate, on_air_len, bw, sgi);
    const bool ours = cfg_.own_sa_valid && mpdu != nullptr && len >= 16 &&
                      std::memcmp(mpdu + 10, cfg_.own_sa.data(), 6) == 0;

    std::lock_guard<std::mutex> lk(mu_);
    if (rssi_raw > 0) {
      ++n_;
      rssi_sum_ += rssi_raw;
      if (rssi_raw > rssi_max_)
        rssi_max_ = rssi_raw;
      snr_sum_ += snr_raw;
      if (snr_raw < snr_min_)
        snr_min_ = snr_raw;
      if (evm_raw != 0) {
        evm_sum_ += evm_raw;
        ++evm_n_;
      }
    }
    if (ours) {
      ++own_frames_;
      own_air_us_ += air;
    } else {
      other_air_us_ += air;
    }
  }

  /* Discard everything folded so far — the barrier, where frames that raced in
   * from the previous channel are thrown away. */
  void reset() {
    std::lock_guard<std::mutex> lk(mu_);
    clear();
  }

  /* Take the window and start the next one. Delta semantics, like the energy
   * counters this sits beside. */
  SurveyFrameWindow drain() {
    std::lock_guard<std::mutex> lk(mu_);
    SurveyFrameWindow w;
    w.frames = n_;
    if (n_) {
      w.rssi_mean_raw = rssi_sum_ / static_cast<int>(n_);
      w.rssi_max_raw = rssi_max_;
      w.snr_mean_raw = snr_sum_ / static_cast<int>(n_);
      w.snr_min_raw = snr_min_;
    }
    if (evm_n_) {
      w.evm_mean_raw = evm_sum_ / static_cast<int>(evm_n_);
      w.evm_valid = true;
    }
    w.own_frames = own_frames_;
    w.own_air_us = own_air_us_;
    w.other_air_us = other_air_us_;
    clear();
    return w;
  }

private:
  void clear() {
    n_ = 0;
    rssi_sum_ = 0;
    rssi_max_ = -128;
    snr_sum_ = 0;
    snr_min_ = 127;
    evm_sum_ = 0;
    evm_n_ = 0;
    own_frames_ = 0;
    own_air_us_ = 0;
    other_air_us_ = 0;
  }

  SurveyAggConfig cfg_;
  mutable std::mutex mu_;
  uint32_t n_ = 0;
  int32_t rssi_sum_ = 0, rssi_max_ = -128, snr_sum_ = 0, snr_min_ = 127;
  int32_t evm_sum_ = 0;
  uint32_t evm_n_ = 0;
  uint32_t own_frames_ = 0;
  uint64_t own_air_us_ = 0, other_air_us_ = 0;
};

} /* namespace sensing */
} /* namespace devourer */

#endif /* DEVOURER_SENSING_SURVEY_FRAME_AGG_H */
