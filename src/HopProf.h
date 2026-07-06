#ifndef HOP_PROF_H
#define HOP_PROF_H

#include <chrono>
#include <cstdio>

/* Per-stage timing inside the fast_retune paths (DeviceConfig
 * debug.hop_prof), for driving the hop-latency work (FHSS wants every hop
 * microsecond accounted for). Each fast hop emits one machine-parseable line:
 *
 *   <devourer-hop-prof>gen=<tag> ch=<n> <stage>_us=<n> ... total_us=<n>
 *
 * Stages are generation-specific labels passed at mark() time. Zero overhead
 * when disabled beyond a branch. */

namespace devourer {

class HopProf {
public:
  HopProf(bool enabled, const char *gen, unsigned channel)
      : _enabled(enabled), _gen(gen), _ch(channel) {
    if (!_enabled)
      return;
    _t0 = _last = std::chrono::steady_clock::now();
    _len = std::snprintf(_buf, sizeof(_buf), "<devourer-hop-prof>gen=%s ch=%u",
                         _gen, _ch);
  }

  /* Record the time since the previous mark under `stage`. */
  void mark(const char *stage) {
    if (!_enabled)
      return;
    const auto now = std::chrono::steady_clock::now();
    const long long us =
        std::chrono::duration_cast<std::chrono::microseconds>(now - _last)
            .count();
    _last = now;
    if (_len > 0 && _len < static_cast<int>(sizeof(_buf)))
      _len += std::snprintf(_buf + _len, sizeof(_buf) - _len, " %s_us=%lld",
                            stage, us);
  }

  ~HopProf() {
    if (!_enabled || _len <= 0)
      return;
    const long long total =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - _t0)
            .count();
    std::fprintf(stderr, "%s total_us=%lld\n", _buf, total);
  }

private:
  bool _enabled = false;
  const char *_gen;
  unsigned _ch;
  std::chrono::steady_clock::time_point _t0{}, _last{};
  char _buf[512];
  int _len = 0;
};

} /* namespace devourer */

#endif /* HOP_PROF_H */
