#ifndef INIT_TIMER_H
#define INIT_TIMER_H

#include <chrono>
#include <cstdio>
#include <utility>

#include <functional>

#include "logger.h"

/* Stage timer for init-path profiling. Emits one event per checkpoint:
 *
 *   {"ev":"init.timing","stage":"<scope>.<stage>","ms":N[,"xfers":K]}
 *
 * `xfers` is emitted only when the timer was given a transfer counter: the
 * number of register transfers (reads + writes) that stage spent on that one
 * adapter's transport (ITransport::ctrl_xfers, via RtlAdapter::ctrl_xfers),
 * which is the unit a USB bring-up is actually paid in. Per adapter, so two
 * devices brought up in one process do not cross-attribute; 0 on PCIe.
 *
 * `stage()` reports time since the previous checkpoint (or construction);
 * `total()` reports time since construction. Always-on: a handful of events
 * per init, negligible next to the USB transfers being measured.
 * tests/bench_init.py parses these events (docs/logging.md). */
class InitTimer {
  using clock = std::chrono::steady_clock;

public:
  using XferCounter = std::function<uint64_t()>;
  using Drain = std::function<void()>;

  /* `drain`, when given, runs at the start of every checkpoint: a transport
   * that queues writes (ITransport::write_batch_begin) completes the stage's
   * own writes before the stage is measured, so their tail latency is billed
   * to the stage that issued them, not to the next one. */
  InitTimer(Logger_t logger, const char *scope, XferCounter xfers = {},
            Drain drain = {})
      : _logger{std::move(logger)}, _scope{scope}, _xfers{std::move(xfers)},
        _drain{std::move(drain)}, _start{clock::now()}, _last{_start},
        _x_start{count()}, _x_last{_x_start} {}

  void stage(const char *name) {
    if (_drain)
      _drain();
    const auto now = clock::now();
    const auto x = count();
    emit(name, ms(_last, now), static_cast<long long>(x - _x_last));
    _last = now;
    _x_last = x;
  }

  /* Reports the total once. An early return or a throw out of the timed
   * scope still gets its total from the destructor, so a failed bring-up
   * carries its cost too; a scope that called total() emits exactly one. */
  void total() {
    if (_finalized)
      return;
    _finalized = true;
    if (_drain)
      _drain();
    emit("total", ms(_start, clock::now()),
         static_cast<long long>(count() - _x_start));
  }
  ~InitTimer() { total(); }

private:
  uint64_t count() const { return _xfers ? _xfers() : 0; }

  void emit(const char *name, long long millis, long long nx) {
    char stage[96];
    std::snprintf(stage, sizeof(stage), "%s.%s", _scope, name);
    devourer::Ev ev(_logger->events(), "init.timing");
    ev.f("stage", stage).f("ms", millis);
    if (_xfers)
      ev.f("xfers", nx);
  }

  static long long ms(clock::time_point from, clock::time_point to) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(to - from)
        .count();
  }

  Logger_t _logger;
  const char *_scope;
  XferCounter _xfers;
  Drain _drain;
  clock::time_point _start;
  clock::time_point _last;
  uint64_t _x_start;
  uint64_t _x_last;
  bool _finalized = false;
};

#endif /* INIT_TIMER_H */
