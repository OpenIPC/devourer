/* TsfDiscipline.h — M4 (#258) the DU's beacon-grid discipline loop, headless:
 * hold a TSF-locked-TBTT radio's beacon grid (Kestrel/AX, Jaguar1 — the TBTT
 * moves with the TSF in hardware) at a fixed phase of a PTP hardware clock.
 * Extracted from tests/pcie_ptp_tsf_discipline.cpp (the bench tool now runs
 * on it); the radio and PHC I/O stay with the caller.
 *
 * Per iteration the caller takes one BRACKETED sample — PHC read r1, the
 * radio's (tsf, freerun) pair, PHC read r2 — and passes it to sample(). The
 * loop:
 *   - rejects a sample whose bracket r2 - r1 exceeds the running median of
 *     the last 32 by `bracket_slack_us` (host preemption between the reads),
 *     or an explicit `bracket_max_us` cap;
 *   - fits ref = a*x + b over the last `window` accepted samples, x being the
 *     free-running counter (TSF steps never move it, so every step's true
 *     effect is measured, nothing booked) — or, `ledger`, tsf - S with S the
 *     sum of commanded steps, for parts without a free-running counter;
 *   - maps the next TBTT (next TSF multiple of the interval) to ref time and
 *     takes its phase error against `phase_us` (wrapped to +-interval/2);
 *   - runs a PI (gain, ki; integrator clamped +-1500 us, reset outside the
 *     `capture_us` range) and returns the TSF step that brings the grid point
 *     earlier by the error.
 * After applying a nonzero step the caller reports the write's outcome with
 * wrote(): a failed write may have half-updated the counter, so the fit is
 * dropped and re-anchored.
 *
 * Pure header; cell_m4 ctest drives it against a simulated radio + PHC. */
#ifndef DEVOURER_CELL_TSFDISCIPLINE_H
#define DEVOURER_CELL_TSFDISCIPLINE_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace devourer::cell {

struct TsfDisciplineConfig {
  int64_t interval_us = 102400; /* beacon interval (TU * 1024) */
  double phase_us = 0;          /* where the TBTT should sit: ref % interval */
  double gain = 0.9, ki = 0.10;
  double capture_us = 300;      /* integrate only within this phase error */
  size_t window = 64;           /* fit window, accepted samples */
  double bracket_max_us = 0;    /* 0 = adaptive: median of the last 32 + slack */
  double bracket_slack_us = 5;
  double step_comp_us = 2;      /* added to each TSF write (read-add-write lag) */
  bool ledger = false;          /* x = tsf - booked steps (no free-running counter) */
  size_t min_fit = 8;           /* samples before the loop acts */
};

/* What one sample produced. */
struct TsfDecision {
  bool accepted = false;   /* the sample passed the bracket filter */
  bool acting = false;     /* the fit had enough samples to steer */
  double phase_err_us = 0; /* TBTT phase error vs the reference (acting only) */
  int64_t step_us = 0;     /* TSF step to apply now (0 = none) */
  int64_t write_us = 0;    /* the value to add to a fresh ReadTsf(): step + comp */
  double ppm = 0;          /* fitted reference rate per radio rate - 1 (negative: radio fast) */
  double bracket_us = 0;   /* this sample's bracket width */
};

class TsfDiscipline {
public:
  explicit TsfDiscipline(TsfDisciplineConfig c = {}) : cfg_(c) {}

  /* r1, r2: PHC reads around the radio read (us); read_ok: the radio read
   * succeeded; tsf / freerun: the pair read (freerun ignored in ledger mode). */
  TsfDecision sample(double r1, double r2, bool read_ok, int64_t tsf, int64_t freerun) {
    TsfDecision d;
    const double br = r2 - r1;
    d.bracket_us = br;
    double limit = cfg_.bracket_max_us;
    if (limit <= 0) {
      if (brackets_.size() >= 8) {
        std::vector<double> sb(brackets_.begin(), brackets_.end());
        std::nth_element(sb.begin(), sb.begin() + static_cast<std::ptrdiff_t>(sb.size() / 2), sb.end());
        limit = sb[sb.size() / 2] + cfg_.bracket_slack_us;
      } else {
        limit = 1e18; /* learning the baseline */
      }
    }
    brackets_.push_back(br);
    if (brackets_.size() > 32)
      brackets_.pop_front();
    if (!read_ok || br > limit) {
      ++rejected_;
      return d;
    }
    d.accepted = true;
    const double x = cfg_.ledger ? static_cast<double>(tsf) - booked_ : static_cast<double>(freerun);
    const double r = 0.5 * (r1 + r2);
    if (!init_) {
      x0_ = x;
      y0_ = r;
      init_ = true;
    }
    pts_.push_back({x - x0_, r - y0_});
    if (pts_.size() > cfg_.window)
      pts_.pop_front();
    if (pts_.size() < cfg_.min_fit)
      return d;
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    const double m = static_cast<double>(pts_.size());
    for (const auto &p : pts_) {
      sx += p.x;
      sy += p.y;
      sxx += p.x * p.x;
      sxy += p.x * p.y;
    }
    const double den = m * sxx - sx * sx;
    const double a = den != 0 ? (m * sxy - sx * sy) / den : 1.0;
    const double b = (sy - a * sx) / m;
    const int64_t I = cfg_.interval_us;
    const int64_t next_tbtt = (tsf / I + 1) * I;           /* the TSF grid point */
    const double next_tbtt_x = static_cast<double>(next_tbtt) - (static_cast<double>(tsf) - x);
    const double tbtt_ref = y0_ + a * (next_tbtt_x - x0_) + b;
    double e = std::fmod(tbtt_ref - cfg_.phase_us, static_cast<double>(I));
    if (e > I / 2.0)
      e -= static_cast<double>(I);
    else if (e < -I / 2.0)
      e += static_cast<double>(I);
    integ_ = std::fabs(e) < cfg_.capture_us ? std::clamp(integ_ + e, -1500.0, 1500.0) : 0.0;
    const double u = cfg_.gain * e + cfg_.ki * integ_;
    d.acting = true;
    d.phase_err_us = e;
    d.step_us = static_cast<int64_t>(std::llround(u));
    d.write_us = d.step_us != 0 ? d.step_us + static_cast<int64_t>(std::llround(cfg_.step_comp_us)) : 0;
    d.ppm = (a - 1.0) * 1e6;
    pending_step_ = d.step_us;
    return d;
  }

  /* The outcome of applying the last nonzero step. */
  void wrote(bool ok) {
    if (ok) {
      if (cfg_.ledger)
        booked_ += static_cast<double>(pending_step_);
      write_fails_ = 0;
    } else {
      reset();
      ++write_fails_;
    }
    pending_step_ = 0;
  }
  /* A new target phase (M6: the CU assigns a joining DU's phase before it
   * acquires). The fit stays (it is the clock relation, not the target); the
   * integrator restarts. */
  void set_phase(double phase_us) {
    cfg_.phase_us = phase_us;
    integ_ = 0;
  }
  double phase() const { return cfg_.phase_us; }
  /* Drop the fit (re-anchor from fresh samples). */
  void reset() {
    pts_.clear();
    init_ = false;
    booked_ = 0;
    integ_ = 0;
  }

  uint64_t rejected() const { return rejected_; }
  int write_fail_run() const { return write_fails_; }
  size_t fit_points() const { return pts_.size(); }

private:
  struct Pt {
    double x, y;
  };
  TsfDisciplineConfig cfg_;
  std::deque<double> brackets_;
  std::deque<Pt> pts_;
  bool init_ = false;
  double x0_ = 0, y0_ = 0, booked_ = 0, integ_ = 0;
  int64_t pending_step_ = 0;
  uint64_t rejected_ = 0;
  int write_fails_ = 0;
};

} /* namespace devourer::cell */

#endif /* DEVOURER_CELL_TSFDISCIPLINE_H */
