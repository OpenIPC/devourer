/* SyncHealth.h — M4 (#258) loss-of-sync detection for a DU on the PTP
 * fabric: decides whether the cell may run its COORDINATED schedule (one
 * timebase with its neighbours, same-channel slots) or must fall back to an
 * autonomous one (wide guards / its own channel) so a desynced cell cannot
 * collide network-wide.
 *
 * Inputs, once per discipline iteration:
 *   - the loop's outcome (TsfDecision: accepted / acting / phase error);
 *   - ref_ok: the reference itself is trustworthy — e.g. the node's ptp4l is
 *     in SLAVE (or the hub's in MASTER) — the caller's check;
 *   - beacon_ok: the beacon engine is alive (the caller's check, e.g. the
 *     DU's own beacons or its UEs' uplinks still arrive);
 *   - write_ok: the iteration's TSF write (if it made one) succeeded. A
 *     failed write may have half-updated the counter: the grid is unknown.
 * States:
 *   Locked   — |phase error| <= lock_us for `lock_iters` acting samples in a
 *              row, with ref_ok and beacon_ok: coordinated scheduling allowed.
 *   Degraded — an acting sample out of tolerance, but not yet for long:
 *              coordinated scheduling SUSPENDED.
 *   Lost     — ref_ok, beacon_ok or write_ok false, |phase error| > lost_us,
 *              `lost_iters` out-of-tolerance samples in a row, or no acting
 *              sample for `lost_iters` iterations: suspended. Recovery needs
 *              `lock_iters` good samples from scratch.
 * A sample that is not acting (a rejected bracket, the fit still filling)
 * carries no phase information: it neither degrades nor counts toward lock,
 * only toward the no-sample limit. One preempted read does not suspend a
 * locked cell; the rig showed that it otherwise would, about once a minute.
 * Coordinated scheduling is allowed only in Locked (the conservative reading
 * of the M4 deliverable: suspend on any doubt, resume only on evidence). */
#ifndef DEVOURER_CELL_SYNCHEALTH_H
#define DEVOURER_CELL_SYNCHEALTH_H

#include <cmath>
#include <cstdint>

#include "cell/TsfDiscipline.h"

namespace devourer::cell {

enum class SyncState : uint8_t { Lost = 0, Degraded = 1, Locked = 2 };

inline const char *sync_state_name(SyncState s) {
  switch (s) {
  case SyncState::Locked: return "locked";
  case SyncState::Degraded: return "degraded";
  default: return "lost";
  }
}

struct SyncHealthConfig {
  double lock_us = 10;    /* in tolerance */
  double lost_us = 200;   /* beyond this the grid is someone else's slot */
  int lock_iters = 8;     /* consecutive good acting samples to (re)lock */
  int lost_iters = 20;    /* out-of-tolerance (or non-acting) iterations in a row before Lost */
};

class SyncHealth {
public:
  explicit SyncHealth(SyncHealthConfig c = {}) : cfg_(c) {}

  /* One iteration. Returns the state after it. */
  SyncState update(const TsfDecision &d, bool ref_ok, bool beacon_ok, bool write_ok = true) {
    const bool acting = d.accepted && d.acting;
    const bool in_tol = acting && std::fabs(d.phase_err_us) <= cfg_.lock_us;
    if (!ref_ok || !beacon_ok || !write_ok ||
        (acting && std::fabs(d.phase_err_us) > cfg_.lost_us)) {
      enter(SyncState::Lost);
      good_ = 0;
      bad_ = 0;
      miss_ = 0;
      return state_;
    }
    if (!acting) {
      if (++miss_ >= cfg_.lost_iters) {
        enter(SyncState::Lost);
        good_ = 0;
        miss_ = 0;
      }
      return state_;
    }
    miss_ = 0;
    if (in_tol) {
      bad_ = 0;
      if (++good_ >= cfg_.lock_iters)
        enter(SyncState::Locked);
    } else {
      good_ = 0;
      ++bad_;
      if (bad_ >= cfg_.lost_iters) {
        enter(SyncState::Lost);
        bad_ = 0;
      }
      else if (state_ == SyncState::Locked)
        enter(SyncState::Degraded);
    }
    return state_;
  }

  SyncState state() const { return state_; }
  /* May the cell run its coordinated (same-timebase) schedule now? */
  bool coordinated() const { return state_ == SyncState::Locked; }
  /* Transitions so far (for logs/KPIs). */
  uint32_t transitions() const { return transitions_; }

private:
  void enter(SyncState s) {
    if (s != state_)
      ++transitions_;
    state_ = s;
  }
  SyncHealthConfig cfg_;
  SyncState state_ = SyncState::Lost; /* start suspended until proven locked */
  int good_ = 0, bad_ = 0, miss_ = 0;
  uint32_t transitions_ = 0;
};

} /* namespace devourer::cell */

#endif /* DEVOURER_CELL_SYNCHEALTH_H */
