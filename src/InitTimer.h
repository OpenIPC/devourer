#ifndef INIT_TIMER_H
#define INIT_TIMER_H

#include <chrono>
#include <utility>

#include "logger.h"

/* Stage timer for init-path profiling. Emits one info line per checkpoint:
 *
 *   init-timing: <scope>.<stage> = <N> ms
 *
 * `stage()` reports time since the previous checkpoint (or construction);
 * `total()` reports time since construction. Always-on: a handful of log
 * lines per init, negligible next to the USB transfers being measured.
 * tests/bench_init.py parses these lines. */
class InitTimer {
  using clock = std::chrono::steady_clock;

public:
  InitTimer(Logger_t logger, const char *scope)
      : _logger{std::move(logger)}, _scope{scope}, _start{clock::now()},
        _last{_start} {}

  void stage(const char *name) {
    const auto now = clock::now();
    _logger->info("init-timing: {}.{} = {} ms", _scope, name, ms(_last, now));
    _last = now;
  }

  void total() {
    _logger->info("init-timing: {}.total = {} ms", _scope,
                  ms(_start, clock::now()));
  }

private:
  static long long ms(clock::time_point from, clock::time_point to) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(to - from)
        .count();
  }

  Logger_t _logger;
  const char *_scope;
  clock::time_point _start;
  clock::time_point _last;
};

#endif /* INIT_TIMER_H */
