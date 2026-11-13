/* CellScheduler.h — M1 (#255) 5QI-style QoS classes for a cell's UEs.
 *
 * M1's grid is fixed (one DL and one UL opportunity per UE per superframe,
 * no slot resizing), so a class cannot buy more airtime there. What it
 * decides is HOW each UE is served, and who is served when there are more
 * UEs than positions:
 *   - the packet delay budget (PDB) bounds the HARQ budget: the first
 *     attempt can take up to a superframe (the UE's DL slot), and a
 *     retransmission comes one HARQ round trip after the attempt before it
 *     (`retx_sf` superframes: the UE's feedback rides its uplink beacon, which
 *     takes a superframe or two to update), so the attempts that fit are
 *     1 + floor((PDB - superframe) / (retx_sf * superframe)), at least one;
 *     and a DL PDU (or a retransmission) older than the budget is discarded
 *     rather than sent late (PDCP's discard timer). A PDB shorter than one
 *     superframe cannot be met by one DL opportunity per superframe: such a
 *     class is served with one attempt and flagged (`pdb_unmet`), not
 *     pretended;
 *   - the packet error rate (PER) target sets link adaptation: a stricter
 *     target steps the rate down sooner and up later;
 *   - the priority level orders admission, and decides which held UE a
 *     newcomer may preempt when the pool is full (lower value = higher
 *     priority, as in 5G).
 *
 * The classes are a subset of the standardized 5QI characteristics (3GPP TS
 * 23.501, table 5.7.4-1): resource type, default priority level, PDB, PER.
 *
 * Headless; ctest cell_core. */
#ifndef DEVOURER_CELL_CELLSCHEDULER_H
#define DEVOURER_CELL_CELLSCHEDULER_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "cell/LinkAdapt.h"

namespace devourer::cell {

struct QosClass {
  uint8_t qi = 9;          /* the 5QI */
  bool gbr = false;        /* guaranteed bit rate (resource type) */
  uint8_t priority = 90;   /* default priority level: lower = higher priority */
  uint32_t pdb_ms = 300;   /* packet delay budget */
  double per = 1e-6;       /* packet error rate target */
};

/* The standardized characteristics of the 5QIs this cell knows. */
inline std::optional<QosClass> qos_class(uint8_t qi) {
  static const QosClass table[] = {
      {1, true, 20, 100, 1e-2},  /* conversational voice */
      {2, true, 40, 150, 1e-3},  /* conversational video */
      {3, true, 30, 50, 1e-3},   /* real-time gaming, V2X, process automation monitoring */
      {5, false, 10, 100, 1e-6}, /* IMS signalling */
      {7, false, 70, 100, 1e-3}, /* voice, live video, interactive gaming */
      {9, false, 90, 300, 1e-6}, /* best effort (default bearer) */
  };
  for (const QosClass &c : table)
    if (c.qi == qi)
      return c;
  return std::nullopt;
}

/* How a UE of a class is served on a grid of superframe `period_us`. */
struct UePolicy {
  uint8_t qi = 9;
  int harq_attempts = 1;    /* first transmission included */
  int64_t discard_ms = 0;   /* a DL PDU queued longer than this is dropped */
  LinkAdaptConfig la;
  bool pdb_unmet = false;   /* the PDB is shorter than one superframe */
};

/* `base` is the cell's link adaptation; `max_attempts` its HARQ ceiling;
 * `retx_sf` its HARQ round trip in superframes (HarqConfig::timeout_sf).
 * The discard timer is never below one superframe plus `slot_wait_ms`: a PDU
 * can wait that long for its UE's DL slot even on a perfect link. */
inline UePolicy policy_for(const QosClass &q, int64_t period_us, const LinkAdaptConfig &base = {},
                           int max_attempts = 4, int64_t slot_wait_ms = 5, int retx_sf = 3) {
  UePolicy p;
  p.qi = q.qi;
  const int64_t pdb_us = static_cast<int64_t>(q.pdb_ms) * 1000;
  p.pdb_unmet = pdb_us < period_us;
  const int64_t rt_us = static_cast<int64_t>(std::max(1, retx_sf)) * period_us;
  const int64_t retries = period_us > 0 && pdb_us > period_us ? (pdb_us - period_us) / rt_us : 0;
  p.harq_attempts = static_cast<int>(std::clamp<int64_t>(1 + retries, 1, std::max(1, max_attempts)));
  p.discard_ms = std::max<int64_t>(q.pdb_ms, period_us / 1000 + slot_wait_ms);
  p.la = base;
  if (q.per >= 1e-2) { /* tolerant: hold a rate through more loss, climb sooner */
    p.la.down_loss = std::max(base.down_loss, 0.3);
    p.la.up_loss = std::max(base.up_loss, 0.05);
  } else if (q.per <= 1e-5) { /* strict: step down early, climb only with margin */
    p.la.down_loss = std::min(base.down_loss, 0.1);
    p.la.up_loss = std::min(base.up_loss, 0.01);
    p.la.hysteresis_db = base.hysteresis_db + 1.0;
  }
  return p;
}

/* Admission order: by priority level, GBR before non-GBR at a tie, then the
 * order given (first come). Returns indices into `cls`. */
inline std::vector<size_t> admission_order(const std::vector<QosClass> &cls) {
  std::vector<size_t> o(cls.size());
  for (size_t i = 0; i < o.size(); ++i)
    o[i] = i;
  std::stable_sort(o.begin(), o.end(), [&](size_t a, size_t b) {
    if (cls[a].priority != cls[b].priority)
      return cls[a].priority < cls[b].priority;
    return cls[a].gbr && !cls[b].gbr;
  });
  return o;
}

/* A full pool and a newcomer: which held UE it may preempt — the one of the
 * lowest priority, only if strictly lower than the newcomer's (a tie keeps
 * whoever holds the position); the latest of equals. nullopt: deny. */
inline std::optional<size_t> preempt_for(const QosClass &newcomer, const std::vector<QosClass> &held) {
  std::optional<size_t> worst;
  for (size_t i = 0; i < held.size(); ++i)
    if (!worst || held[i].priority >= held[*worst].priority)
      worst = i;
  if (worst && held[*worst].priority > newcomer.priority)
    return worst;
  return std::nullopt;
}

} /* namespace devourer::cell */

#endif /* DEVOURER_CELL_CELLSCHEDULER_H */
