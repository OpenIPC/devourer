/* Headless guard for the M4 cell core: src/cell/TsfDiscipline.h against a
 * simulated radio + PHC (a 20 ppm crystal, a 2 us read-add-write lag the
 * comp mostly covers, 25 us read brackets with occasional preempted reads),
 * checking the TRUE on-air TBTT phase, not just the loop's estimate (sampled
 * just before each correction: between corrections the grid drifts by
 * ppm * loop period — the inherent stepping sawtooth, checked separately); the
 * bracket filter, a failed write's re-anchor, the ledger axis; and
 * src/cell/SyncHealth.h's lock / degrade / lose / recover rules. */
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "cell/SyncHealth.h"
#include "cell/TsfDiscipline.h"

using namespace devourer::cell;

static int failures = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      failures++;                                                              \
    }                                                                          \
  } while (0)

/* A radio whose free-running counter runs (1 + ppm) fast against the PHC and
 * whose TSF = freerun + off; a TSF write lands `lag` us late. */
struct SimRadio {
  double ppm = 20, f0 = 1.0e9, off = 3.7e6, lag = 2.0;
  bool fail_next_write = false;
  double fr(double t) const { return f0 + (1 + ppm * 1e-6) * t; }
  double tsf(double t) const { return fr(t) + off; }
  bool write(double add_us) {
    if (fail_next_write) {
      fail_next_write = false;
      off += 12345; /* a half-updated counter */
      return false;
    }
    off += add_us - lag;
    return true;
  }
  /* The true TBTT phase (ref time of the next TSF grid point, mod I). */
  double tbtt_phase(double t, int64_t I) const {
    const double ts = tsf(t);
    const double k = std::floor(ts / I) + 1;
    const double t_tbtt = (k * I - off - f0) / (1 + ppm * 1e-6);
    double p = std::fmod(t_tbtt, static_cast<double>(I));
    return p < 0 ? p + I : p;
  }
};

static double wrap(double e, double I) {
  e = std::fmod(e, I);
  if (e > I / 2) e -= I;
  if (e < -I / 2) e += I;
  return e;
}

int main() {
  const int64_t I = 102400;
  /* --- Convergence on the free-running axis, with preempted reads. --- */
  {
    TsfDisciplineConfig c;
    c.interval_us = I;
    c.phase_us = 20000;
    TsfDiscipline loop(c);
    SimRadio r;
    double t = 0;
    std::vector<double> true_err;
    double saw = 0;
    bool after_reject = false;
    uint64_t preempted = 0;
    for (int i = 0; i < 400; ++i) {
      t += 500000;
      const bool preempt = (i % 37) == 5; /* host preemption mid-read */
      const double r1 = t, r2 = t + (preempt ? 400 : 25);
      preempted += preempt;
      const double tr = 0.5 * (r1 + r2);
      /* After a rejected sample the grid went uncorrected for two periods. */
      if (i > 100 && !after_reject)
        true_err.push_back(wrap(r.tbtt_phase(t, I) - 20000, I));
      const auto d = loop.sample(r1, r2, true, static_cast<int64_t>(std::llround(r.tsf(tr))),
                                 static_cast<int64_t>(std::llround(r.fr(tr))));
      after_reject = !d.accepted;
      if (d.step_us != 0)
        loop.wrote(r.write(static_cast<double>(d.write_us)));
      if (i > 100)
        saw = std::fmax(saw, std::fabs(wrap(r.tbtt_phase(t, I) - 20000, I)));
      if (i == 399)
        CHECK(std::fabs(d.ppm + 20) < 0.5); /* ref per radio rate: radio fast -> negative */
    }
    double ss = 0, mx = 0;
    for (double e : true_err) {
      ss += e * e;
      mx = std::fmax(mx, std::fabs(e));
    }
    const double rms = std::sqrt(ss / true_err.size());
    CHECK(rms < 2.0 && mx < 5.0);
    CHECK(saw < 10 + 5); /* post-step: the 20 ppm * 500 ms = 10 us sawtooth, no worse */
    CHECK(loop.rejected() >= preempted - 1 && loop.rejected() <= preempted + 1);
    std::printf("cell_m4: 20 ppm crystal, 2 us write lag: true TBTT phase rms %.2f us, max %.2f us "
                "(sawtooth peak %.2f us); %llu preempted reads rejected\n",
                rms, mx, saw, (unsigned long long)loop.rejected());
  }

  /* --- A failed (half-updated) write drops the fit and it re-converges. --- */
  {
    TsfDisciplineConfig c;
    c.interval_us = I;
    TsfDiscipline loop(c);
    SimRadio r;
    double t = 0;
    double last = 0;
    for (int i = 0; i < 300; ++i) {
      t += 500000;
      last = wrap(r.tbtt_phase(t, I), I);
      if (i == 150)
        r.fail_next_write = true;
      const auto d = loop.sample(t, t + 20, true, static_cast<int64_t>(r.tsf(t + 10)),
                                 static_cast<int64_t>(r.fr(t + 10)));
      if (d.step_us != 0) {
        const bool ok = r.write(static_cast<double>(d.write_us));
        loop.wrote(ok);
        if (!ok)
          CHECK(loop.fit_points() == 0 && loop.write_fail_run() == 1);
      }
    }
    CHECK(std::fabs(last) < 5.0);
  }

  /* --- Ledger axis (no free-running counter): converges too. --- */
  {
    TsfDisciplineConfig c;
    c.interval_us = I;
    c.ledger = true;
    c.step_comp_us = 2; /* matches the lag: the ledger stays honest */
    TsfDiscipline loop(c);
    SimRadio r;
    double t = 0;
    double last = 0;
    for (int i = 0; i < 300; ++i) {
      t += 500000;
      last = wrap(r.tbtt_phase(t, I), I);
      const auto d = loop.sample(t, t, true, static_cast<int64_t>(r.tsf(t)), 0);
      if (d.step_us != 0)
        loop.wrote(r.write(static_cast<double>(d.write_us)));
    }
    CHECK(std::fabs(last) < 5.0);
  }

  /* --- A failed radio read is a rejected sample, not a fit point. --- */
  {
    TsfDiscipline loop;
    const auto d = loop.sample(0, 20, false, 0, 0);
    CHECK(!d.accepted && !d.acting && loop.rejected() == 1 && loop.fit_points() == 0);
  }

  /* --- SyncHealth. --- */
  {
    SyncHealthConfig hc;
    hc.lock_us = 10;
    hc.lost_us = 200;
    hc.lock_iters = 3;
    hc.lost_iters = 4;
    SyncHealth h(hc);
    auto at = [](double e) {
      TsfDecision d;
      d.accepted = d.acting = true;
      d.phase_err_us = e;
      return d;
    };
    CHECK(h.state() == SyncState::Lost && !h.coordinated()); /* suspended until proven */
    h.update(at(2), true, true);
    h.update(at(-3), true, true);
    CHECK(h.state() == SyncState::Lost);
    CHECK(h.update(at(1), true, true) == SyncState::Locked && h.coordinated());
    CHECK(h.update(at(40), true, true) == SyncState::Degraded && !h.coordinated());
    CHECK(h.update(at(2), true, true) == SyncState::Degraded); /* one good is not enough */
    h.update(at(2), true, true);
    CHECK(h.update(at(2), true, true) == SyncState::Locked);
    /* A rejected sample (not acting) carries no phase: a locked cell stays
     * locked through a few; lost_iters of them in a row is Lost. */
    TsfDecision rej;
    for (int i = 0; i < 3; ++i)
      CHECK(h.update(rej, true, true) == SyncState::Locked);
    CHECK(h.update(at(1), true, true) == SyncState::Locked); /* the run is reset */
    for (int i = 0; i < 3; ++i)
      CHECK(h.update(rej, true, true) == SyncState::Locked);
    CHECK(h.update(rej, true, true) == SyncState::Lost);
    /* Out of tolerance lost_iters times in a row: Lost (via Degraded). */
    for (int i = 0; i < 3; ++i) h.update(at(1), true, true);
    for (int i = 0; i < 3; ++i)
      CHECK(h.update(at(50), true, true) == SyncState::Degraded);
    CHECK(h.update(at(50), true, true) == SyncState::Lost);
    /* A failed TSF write (the counter may be half-updated): Lost at once. */
    for (int i = 0; i < 3; ++i) h.update(at(1), true, true);
    CHECK(h.coordinated());
    CHECK(h.update(at(1), true, true, false) == SyncState::Lost && !h.coordinated());
    /* The reference or the beacon engine failing: Lost at once. */
    for (int i = 0; i < 3; ++i) h.update(at(1), true, true);
    CHECK(h.coordinated());
    CHECK(h.update(at(1), false, true) == SyncState::Lost);
    for (int i = 0; i < 3; ++i) h.update(at(1), true, true);
    CHECK(h.update(at(1), true, false) == SyncState::Lost);
    /* A phase error past lost_us: Lost at once. */
    for (int i = 0; i < 3; ++i) h.update(at(1), true, true);
    CHECK(h.update(at(500), true, true) == SyncState::Lost);
    CHECK(h.transitions() >= 8);
  }

  if (failures == 0)
    std::printf("cell_m4_selftest: all checks passed\n");
  return failures == 0 ? 0 : 1;
}
