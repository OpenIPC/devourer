/* A demo telemetry thread's answer to a failed register read.
 *
 * A register read throws on a failed control transfer, and the pollers
 * that call the device on their own cadence (busy window, thermal meter, RX
 * energy) run on threads of their own or inside a demo's main loop, where an
 * uncaught throw is std::terminate. Under co-channel load these reads do fail while the chip
 * works on (src/jaguar2/CLAUDE.md has the measured rate), so a poller catches,
 * counts and skips the sample - the same shape as the library's DIG loop -
 * and the next tick reads afresh. The warning is rate-limited (failures 1,
 * 101, 201, ...) and always carries the running count, so a
 * long session neither floods stderr nor hides how often it happened. */
#ifndef DEVOURER_POLL_SKIP_H
#define DEVOURER_POLL_SKIP_H

#include <cstdint>
#include <exception>

#include "logger.h"

namespace devourer {

class PollSkips {
public:
  explicit PollSkips(const char *what) : _what(what) {}

  void note(Logger &log, const std::exception &e) {
    if (_n++ % 100 == 0)
      log.warn("{}: sample skipped after a failed register access ({}), {} so "
               "far", _what, e.what(), _n);
  }

private:
  const char *_what;
  uint64_t _n = 0;
};

} // namespace devourer

#endif /* DEVOURER_POLL_SKIP_H */
