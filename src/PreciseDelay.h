#pragma once

/* Microsecond delays that mean what they say on every platform.
 *
 * std::this_thread::sleep_for is fine for short waits on Linux, but on
 * Windows no sleep ends before the next scheduler tick: 15.6 ms by default,
 * 1 ms at best with timeBeginPeriod(1). The Kestrel bring-up and the vendored
 * halbb/halrf calibrations wait tens of microseconds thousands of times, so a
 * 50 us wait turning into 15.6 ms stretches a seconds-long bring-up into many
 * minutes.
 *
 * On Windows the wait spins on steady_clock (QPC-backed) and only sleeps
 * while more time is left than a sleep can take. The tick is process-global
 * state that any library may change, so that bound is learned rather than
 * assumed: it starts at one default tick, follows shorter sleeps down slowly
 * (e.g. once timeBeginPeriod(1) is active) and jumps back up at once when a
 * sleep runs long. A wait shorter than the current tick (the 5 ms EDCCA settle
 * under the default 15.6 ms tick) therefore never sleeps. Other platforms keep
 * sleep_for unchanged. */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>

namespace devourer {

#if defined(_WIN32)
namespace detail {
/* Longest a 1 ms sleep_for is expected to take right now, in microseconds.
 * Starts at (and is capped to) the default 15.625 ms tick plus slack. */
inline constexpr uint32_t kMaxSleepUs = 16000;
inline std::atomic<uint32_t> &sleep_bound_us() {
  static std::atomic<uint32_t> bound{kMaxSleepUs};
  return bound;
}
} // namespace detail
#endif

inline void precise_delay_us(uint32_t us) {
#if defined(_WIN32)
  using clock = std::chrono::steady_clock;
  using std::chrono::microseconds;
  /* Kept free after the last sleep for wakeup latency on a busy system. */
  constexpr int64_t kMarginUs = 500;
  const auto deadline = clock::now() + microseconds(us);
  auto &bound = detail::sleep_bound_us();
  for (;;) {
    const auto start = clock::now();
    const int64_t left =
        std::chrono::duration_cast<microseconds>(deadline - start).count();
    const uint32_t b = bound.load(std::memory_order_relaxed);
    if (left <= int64_t(b) + kMarginUs)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const auto took = static_cast<uint32_t>(std::min<int64_t>(
        std::chrono::duration_cast<microseconds>(clock::now() - start).count(),
        detail::kMaxSleepUs));
    bound.store(took >= b ? took : b - (b - took) / 8,
                std::memory_order_relaxed);
  }
  while (clock::now() < deadline)
    std::this_thread::yield();
#else
  std::this_thread::sleep_for(std::chrono::microseconds(us));
#endif
}

} // namespace devourer
