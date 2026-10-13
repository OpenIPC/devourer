#pragma once

/* Microsecond delays that mean what they say on every platform.
 *
 * std::this_thread::sleep_for is fine for short waits on Linux, but on
 * Windows no sleep ends before the next scheduler tick: 15.6 ms by default,
 * 1 ms at best with timeBeginPeriod(1). The Kestrel bring-up and the vendored
 * halbb/halrf calibrations wait tens of microseconds thousands of times, so a
 * 50 us wait turning into 15.6 ms stretches a seconds-long bring-up into many
 * minutes. On Windows, waits under kSpinUs spin on steady_clock (QPC-backed);
 * longer ones sleep for all but the last tick and spin the remainder. Other
 * platforms keep sleep_for unchanged. */

#include <chrono>
#include <cstdint>
#include <thread>

namespace devourer {

inline void precise_delay_us(uint32_t us) {
#if defined(_WIN32)
  constexpr uint32_t kSpinUs = 2000;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::microseconds(us);
  if (us > kSpinUs)
    std::this_thread::sleep_for(std::chrono::microseconds(us - kSpinUs));
  while (std::chrono::steady_clock::now() < deadline)
    std::this_thread::yield();
#else
  std::this_thread::sleep_for(std::chrono::microseconds(us));
#endif
}

} // namespace devourer
