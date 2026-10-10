// Shared by timesync, tdma-style schedulers and the stream-timing TX fit: a
// 32->64-bit TSF reconstruction and an offset-normalised incremental
// least-squares line. Header-only, no device dependency. examples/timesync
// re-exports both under its own namespace.
#pragma once

#include <cstdint>

namespace tsffit {

// 32→64-bit TSF reconstruction (the MAC latches only the low 32 bits per frame;
// it wraps every ~71 min). One per clock source.
struct Recon {
  int64_t hi = 0;
  uint32_t plo = 0;
  bool init = false;
  int64_t operator()(uint32_t lo) {
    if (init && lo < plo) hi += (1LL << 32);
    plo = lo;
    init = true;
    return hi + lo;
  }
};

// Incremental ordinary-least-squares fit y = a·x + b, offset-normalized to the
// first sample so the sums stay well within double precision over a long run.
struct LinFit {
  bool init = false;
  double x0 = 0, y0 = 0;
  long long n = 0;
  double sx = 0, sy = 0, sxx = 0, sxy = 0;

  void add(double x, double y) {
    if (!init) { x0 = x; y0 = y; init = true; }
    double xi = x - x0, yi = y - y0;
    ++n; sx += xi; sy += yi; sxx += xi * xi; sxy += xi * yi;
  }
  bool ready() const { return n >= 16; }
  double slope() const {
    double den = (double)n * sxx - sx * sx;
    return den == 0 ? 1.0 : ((double)n * sxy - sx * sy) / den;
  }
  double intercept() const { return (sy - slope() * sx) / (double)n; }
  // Predicted y at x.
  double at(double x) const { return y0 + slope() * (x - x0) + intercept(); }
  // Inverse: the x that yields a given y (for scheduling in the fitted domain).
  double inverse(double y) const {
    double a = slope();
    return x0 + (a == 0 ? 0 : (y - y0 - intercept()) / a);
  }
  // Fitted rate offset in ppm (slope = dy/dx, for same-unit clocks).
  double ppm() const { return (slope() - 1.0) * 1e6; }
};

}  // namespace tsffit
