#ifndef INIT_TIMER_H
#define INIT_TIMER_H

#include <chrono>
#include <cstdio>
#include <utility>

#include "logger.h"
#include "UsbXferCount.h"

/* Stage timer for init-path profiling. Emits one event per checkpoint:
 *
 *   {"ev":"init.timing","stage":"<scope>.<stage>","ms":N,"xfers":K}
 *
 * `xfers` is the number of USB control transfers (register reads/writes)
 * the stage spent — see UsbXferCount.h; 0 on the PCIe transport.
 *
 * `stage()` reports time since the previous checkpoint (or construction);
 * `total()` reports time since construction. Always-on: a handful of events
 * per init, negligible next to the USB transfers being measured.
 * tests/bench_init.py parses these events (docs/logging.md). */
class InitTimer {
  using clock = std::chrono::steady_clock;

public:
  InitTimer(Logger_t logger, const char *scope)
      : _logger{std::move(logger)}, _scope{scope}, _start{clock::now()},
        _last{_start}, _x_start{xfers()}, _x_last{_x_start} {}

  void stage(const char *name) {
    const auto now = clock::now();
    const auto x = xfers();
    emit(name, ms(_last, now), static_cast<long long>(x - _x_last));
    _last = now;
    _x_last = x;
  }

  void total() {
    emit("total", ms(_start, clock::now()),
         static_cast<long long>(xfers() - _x_start));
  }

private:
  static uint64_t xfers() {
    return devourer::usb_ctrl_xfers().load(std::memory_order_relaxed);
  }

  void emit(const char *name, long long millis, long long nx) {
    char stage[96];
    std::snprintf(stage, sizeof(stage), "%s.%s", _scope, name);
    devourer::Ev(_logger->events(), "init.timing")
        .f("stage", stage)
        .f("ms", millis)
        .f("xfers", nx);
  }

  static long long ms(clock::time_point from, clock::time_point to) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(to - from)
        .count();
  }

  Logger_t _logger;
  const char *_scope;
  clock::time_point _start;
  clock::time_point _last;
  uint64_t _x_start;
  uint64_t _x_last;
};

#endif /* INIT_TIMER_H */
