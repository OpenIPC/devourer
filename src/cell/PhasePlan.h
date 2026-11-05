/* PhasePlan.h — M6 (#260) the CU's phase planning: where a JOINING DU's
 * TBTT goes on the shared PTP timebase, so its UEs and its neighbours' can
 * measure each other in gaps (src/cell/MeasGap.h) and co-channel cells never
 * meet a fixed conflict (src/cell/Coord.h). Pure header; the cell_m6 ctest
 * covers it. */
#ifndef DEVOURER_CELL_PHASEPLAN_H
#define DEVOURER_CELL_PHASEPLAN_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "cell/Coord.h"
#include "cell/MeasGap.h"

namespace devourer::cell {

/* --- Phase planning (the CU): where a JOINING DU's TBTT goes. ---
 *
 * The DUs already running keep their phases (re-phasing a live cell would
 * move every UE's clock); a new one gets the phase that best fits them:
 *   - another channel: its UEs and the other's must be able to MEASURE each
 *     other — for every capacity position of each cell, a gap for the
 *     other's beacon (gap_window) clear of that position's slots;
 *   - the same channel: the two cells' active spans (beacon + all
 *     positions) disjoint, so coordination never meets a fixed conflict
 *     (one point), or at least neither beacon inside the other's span (half).
 * Never across bands (gap_cross_band: the UEs refuse such gaps). Running
 * cells on another superframe period are left out (`other_period`): their
 * beacons do not recur at one offset of the joining grid.
 * Each pair scores up to 2 (both directions / both spans); the plan takes
 * the best total over a `step_us` grid and, of the best, the middle of the
 * widest run of equally good phases (the most margin). */
struct PhaseCell {
  uint16_t du = 0;
  uint8_t channel = 0;
  int64_t phase_us = 0;                   /* ignored for the joining cell */
  int64_t period_us = 0;                  /* its superframe (0: the plan's) */
  int64_t beacon_us = 0;                  /* its beacon slot */
  std::vector<std::pair<GridSpan, GridSpan>> positions; /* (DL, UL) per position */
  int64_t span_us = 0;                    /* beacon + every position: the active span */
};
struct PhaseParams {
  int64_t period_us = 102400;
  int64_t retune_us = 30000, guard_us = 3000, beacon_air_us = 500;
  int64_t step_us = 100;
};
struct PhasePlan {
  int64_t phase_us = 0;
  double score = 0, max_score = 0;
  /* per existing cell: how well the pair works at the chosen phase (0..2) */
  std::vector<std::pair<uint16_t, double>> pairs;
  /* existing cells on another superframe period: their beacons do not recur
   * at one offset of the joining grid, so they are not planned against */
  std::vector<uint16_t> other_period;
};
/* At most this many candidate phases (the step grows for a long period). */
constexpr int64_t kMaxPhaseCandidates = 20000;

/* A cell from its reported layout (Coord.h; the F1 Report's fixed items and
 * positions): the beacon is the fixed item at the TBTT. */
inline PhaseCell phase_cell(uint16_t du, uint8_t channel, int64_t phase_us, int64_t period_us,
                            const std::vector<CoordInterval> &fixed,
                            const std::vector<CoordPosition> &positions) {
  PhaseCell c;
  c.du = du;
  c.channel = channel;
  c.phase_us = phase_us;
  c.period_us = period_us;
  int64_t end = 0;
  for (const CoordInterval &f : fixed) {
    if (f.start_us == 0)
      c.beacon_us = f.len_us;
    end = std::max<int64_t>(end, int64_t{f.start_us} + f.len_us);
  }
  for (const CoordPosition &p : positions) {
    c.positions.push_back({{p.dl.start_us, p.dl.len_us}, {p.ul.start_us, p.ul.len_us}});
    end = std::max<int64_t>(end, std::max(int64_t{p.dl.start_us} + p.dl.len_us,
                                          int64_t{p.ul.start_us} + p.ul.len_us));
  }
  c.span_us = end;
  return c;
}

namespace mg_detail {
/* The share of `x`'s positions that can measure a beacon at `offset` of x's grid. */
inline double measurable(const PhaseCell &x, int32_t offset, const PhaseParams &pp) {
  if (x.positions.empty())
    return 0;
  size_t ok = 0;
  for (const auto &pos : x.positions) {
    const std::vector<GridSpan> busy = {{0, x.beacon_us}, pos.first, pos.second};
    ok += gap_window(offset, pp.beacon_air_us, pp.retune_us, pp.guard_us, pp.period_us, busy) ? 1 : 0;
  }
  return static_cast<double>(ok) / static_cast<double>(x.positions.size());
}
inline bool arcs_overlap(int64_t a, int64_t la, int64_t b, int64_t lb, int64_t P) {
  auto mod = [P](int64_t v) { return ((v % P) + P) % P; };
  return la > 0 && lb > 0 && (mod(b - a) < la || mod(a - b) < lb);
}
inline double pair_score(const PhaseCell &joining, int64_t phase, const PhaseCell &e,
                         const PhaseParams &pp) {
  const int64_t P = pp.period_us;
  if (joining.channel != e.channel) {
    if (gap_cross_band(joining.channel, e.channel))
      return 0; /* the UEs never gap across bands */
    return measurable(e, neighbour_offset_us(phase, e.phase_us, P), pp) +
           measurable(joining, neighbour_offset_us(e.phase_us, phase, P), pp);
  }
  if (!arcs_overlap(phase, joining.span_us, e.phase_us, e.span_us, P))
    return 2;
  const bool b1 = arcs_overlap(phase, joining.beacon_us, e.phase_us, e.span_us, P);
  const bool b2 = arcs_overlap(e.phase_us, e.beacon_us, phase, joining.span_us, P);
  return (!b1 && !b2) ? 1 : 0;
}
} /* namespace mg_detail */

inline PhasePlan plan_phase(const PhaseCell &joining, const std::vector<PhaseCell> &existing,
                            const PhaseParams &pp = {}) {
  using namespace mg_detail;
  PhasePlan best;
  const int64_t P = pp.period_us;
  if (P <= 0)
    return best;
  const int64_t step = std::max<int64_t>({1, pp.step_us, (P + kMaxPhaseCandidates - 1) / kMaxPhaseCandidates});
  std::vector<const PhaseCell *> same;
  for (const auto &e : existing) {
    if (e.period_us != 0 && e.period_us != P)
      best.other_period.push_back(e.du);
    else
      same.push_back(&e);
  }
  best.max_score = 2.0 * static_cast<double>(same.size());
  std::vector<double> score;
  for (int64_t ph = 0; ph < P; ph += step) {
    double sc = 0;
    for (const PhaseCell *e : same)
      sc += pair_score(joining, ph, *e, pp);
    score.push_back(sc);
  }
  if (score.empty())
    return best;
  const double top = *std::max_element(score.begin(), score.end());
  /* The widest run of `top` (circular), and its middle. */
  const size_t n = score.size();
  size_t best_len = 0, best_start = 0;
  for (size_t i = 0; i < n; ++i) {
    if (score[i] != top || score[(i + n - 1) % n] == top)
      continue; /* start of a run only */
    size_t len = 0;
    while (len < n && score[(i + len) % n] == top)
      ++len;
    if (len > best_len) {
      best_len = len;
      best_start = i;
    }
  }
  if (best_len == 0) { /* every phase equally good: the whole circle */
    best_len = n;
    best_start = 0;
  }
  best.phase_us = static_cast<int64_t>((best_start + best_len / 2) % n) * step;
  best.score = top;
  for (const PhaseCell *e : same)
    best.pairs.push_back({e->du, pair_score(joining, best.phase_us, *e, pp)});
  return best;
}

} /* namespace devourer::cell */

#endif /* DEVOURER_CELL_PHASEPLAN_H */
