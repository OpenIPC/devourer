/* TimingAdvance.h — M1 (#255) the DU's per-UE timing-advance loop.
 *
 * A UE's uplink is its hardware beacon, steered through its TSF to the slot
 * its grant gives it (examples/cell); the DU measures where each uplink
 * actually arrives against that slot and moves the UE's TA, which the grant
 * map carries back. The TA corrects clock and actuation phase, not
 * propagation (indoors that is ns, below this design's µs).
 *
 * Moved from examples/cell/cell.h: per-UE state belongs to the DU library
 * (src/cell/UeRegistry.h UeContext holds one per UE). Pure header; the
 * cell_node ctest covers it. */
#ifndef DEVOURER_CELL_TIMINGADVANCE_H
#define DEVOURER_CELL_TIMINGADVANCE_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace devourer::cell {

// DU timing-advance controller for one UE: the measured arrival error (DU
// clock, arrival phase minus target) is averaged over a window; at each
// update() the TA moves by -gain * mean, rounded to whole us, clamped to
// +-max_ta. Positive TA = the UE fires earlier.
class TaLoop {
public:
  TaLoop(double gain, int64_t max_ta_us) : gain_(gain), max_(max_ta_us) {}
  /* The window keeps the newest kMaxWindow samples (a loop that is never
   * updated, e.g. CELL_TA=0, must not grow without bound). */
  static constexpr size_t kMaxWindow = 256;
  void sample(int64_t err_us) {
    if (win_.size() == kMaxWindow)
      win_.erase(win_.begin());
    win_.push_back(err_us);
  }
  size_t pending() const { return win_.size(); }
  // Drop the window without moving the TA (samples taken under an old map).
  void discard() { win_.clear(); }
  // Returns true when the TA changed. min_samples guards a noisy window.
  // The window's MEDIAN is the estimate: one wild arrival (a UE's beacon
  // before its first TSF step, a TBTT on the old schedule after one) must
  // not steer the TA — a mean let a single 39 ms sample push a joining UE
  // out of its slot on the rig.
  bool update(size_t min_samples) {
    if (win_.size() < min_samples || win_.empty())
      return false;
    std::vector<int64_t> v;
    v.swap(win_);
    std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2), v.end());
    const double mean = static_cast<double>(v[v.size() / 2]); /* the median */
    last_mean_ = mean;
    const int64_t step = static_cast<int64_t>(std::llround(gain_ * mean));
    if (step == 0)
      return false;
    int64_t t = ta_ + step;  // arrival late (mean > 0) -> fire earlier -> TA up
    if (t > max_)
      t = max_;
    if (t < -max_)
      t = -max_;
    const bool changed = t != ta_;
    ta_ = t;
    return changed;
  }
  int64_t ta() const { return ta_; }
  // Restore a TA (a map that carried a moved TA failed to publish).
  void set_ta(int64_t t) { ta_ = t; }
  double last_mean() const { return last_mean_; }

private:
  double gain_;
  int64_t max_;
  int64_t ta_ = 0;
  std::vector<int64_t> win_;
  double last_mean_ = 0;
};

} /* namespace devourer::cell */

#endif /* DEVOURER_CELL_TIMINGADVANCE_H */
