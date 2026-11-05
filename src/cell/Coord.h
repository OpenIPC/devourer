/* Coord.h — M5 (#259) the CU's coordination planner: which slot pair
 * (capacity position) each UE of each DU uses, so that a UE two cells hear
 * never shares airtime with the other cell.
 *
 * Model. Every DU holds its beacon grid to the PTP fabric (M4), so a DU's
 * slot `start_us` (relative to its TBTT) is network time phase_us + start_us
 * (mod the shared superframe). A DU's grid has fixed items (beacon slot, RACH
 * slot) and capacity positions, each a DL + UL slot pair (src/cell/SlotGrid.h
 * build_fixed_grid). A UE's identity is its id; which position (slot pair)
 * it uses is the schedule's choice.
 *
 * Interference relation. `heard_by` lists the OTHER DUs whose radio hears a
 * UE (the CU builds it from the DUs' F1 reports, above an RSSI threshold).
 * Two co-channel items at DUs s != o conflict when their network-time
 * intervals overlap and a UE of either is heard by the other DU. A fixed item
 * stands for all of its DU's UEs (they all listen to its beacon).
 *
 * Plan:
 *   1. Sync tiers (M4): a DU that is not tight-sync and relates to a
 *      co-channel DU gets frequency separation (a spare channel), never
 *      same-channel time slots.
 *   2. UEs are placed greedily, the most-related first, each in its current
 *      position if that is conflict-free (no churn), else the first
 *      conflict-free free position of its DU.
 *   3. A UE with no conflict-free position moves its whole DU to a spare
 *      channel, if one is left, and the plan restarts; otherwise it keeps a
 *      free position and counts as unresolved.
 * Fixed-vs-fixed overlaps (two related beacons at one instant) are not a
 * position problem: they are counted, and they also move a DU when a spare
 * channel is left.
 *
 * Pure header; cell_m5 ctest covers it (incl. random scenarios: a plan with
 * nothing unresolved has no conflicts). */
#ifndef DEVOURER_CELL_COORD_H
#define DEVOURER_CELL_COORD_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <vector>

#include "AdapterCaps.h"
#include "cell/SlotGrid.h"

namespace devourer::cell {

struct CoordInterval {
  int32_t start_us = 0; /* relative to the DU's TBTT */
  int32_t len_us = 0;
  bool operator==(const CoordInterval &) const = default;
};
struct CoordPosition {
  CoordInterval dl, ul;
  bool operator==(const CoordPosition &) const = default;
};
struct CoordUe {
  uint16_t ue = 0;
  uint8_t pos = 0;                 /* current position, 1-based (0 = none) */
  std::vector<uint16_t> heard_by;  /* other DUs that hear this UE */
};
struct CoordDu {
  uint16_t du = 0;
  uint8_t channel = 0;
  SyncTier tier = SyncTier::Tight;
  int64_t phase_us = 0;
  int64_t period_us = 0;
  std::vector<CoordInterval> fixed;     /* beacon, RACH */
  std::vector<CoordPosition> positions; /* index i = position i + 1 */
  std::vector<CoordUe> ues;
};
struct CoordAssign {
  uint16_t ue = 0;
  uint8_t pos = 0;
  bool operator==(const CoordAssign &) const = default;
};
struct CoordPlan {
  std::map<uint16_t, std::vector<CoordAssign>> assign; /* per DU, in its UE order */
  std::map<uint16_t, uint8_t> channel;                 /* per DU (changed or not) */
  size_t moved = 0;          /* UEs whose position changed */
  size_t unresolved = 0;     /* UEs left without a conflict-free position */
  size_t fixed_conflicts = 0;
  std::vector<uint16_t> retuned; /* DUs given a spare channel */
  /* Non-tight DUs that needed frequency separation and got none: their UEs
   * are placed (so the others avoid them) but count as unresolved — a
   * coarse-sync DU never counts as resolved by a shared-channel plan. */
  std::vector<uint16_t> unseparated;
};

namespace coord_detail {
/* Two arcs on the superframe circle overlap. */
inline bool arcs_overlap(int64_t a, int64_t la, int64_t b, int64_t lb, int64_t P) {
  if (la <= 0 || lb <= 0)
    return false;
  if (P <= 0 || la >= P || lb >= P)
    return true;
  auto mod = [P](int64_t x) { return ((x % P) + P) % P; };
  return mod(b - a) < la || mod(a - b) < lb;
}
struct Item {
  size_t du_idx;
  int64_t start;              /* network time (phase + start), not yet reduced */
  int64_t len;
  std::vector<uint16_t> ues;  /* the UEs it concerns at its DU */
};
} /* namespace coord_detail */

/* A DU's layout from its capacity grid (build_fixed_grid with positions
 * 1..capacity): the beacon slot and the RACH slot (with the idle tail a late
 * msgA may run into) are fixed items; DL/UL slot pairs are the positions. */
inline void coord_layout(const SlotGrid &g, CoordDu &d) {
  d.period_us = g.period_us();
  d.fixed.clear();
  d.positions.clear();
  for (const Slot &s : g.slots()) {
    const auto start = static_cast<int32_t>(s.start_us);
    const auto len = static_cast<int32_t>(s.len_us);
    if (s.kind == SlotKind::Beacon)
      d.fixed.push_back({start, len});
    else if (s.kind == SlotKind::Rach)
      d.fixed.push_back({start, static_cast<int32_t>(len + kRachTailGapUs)});
    else if ((s.kind == SlotKind::Downlink || s.kind == SlotKind::Uplink) && s.ue >= 1) {
      if (d.positions.size() < s.ue)
        d.positions.resize(s.ue);
      (s.kind == SlotKind::Downlink ? d.positions[s.ue - 1].dl : d.positions[s.ue - 1].ul) =
          {start, len};
    }
  }
}

/* Is UE `ue` of DU `d` heard by DU `other`? */
inline bool coord_heard(const CoordDu &d, uint16_t ue, uint16_t other) {
  for (const CoordUe &u : d.ues)
    if (u.ue == ue)
      return std::find(u.heard_by.begin(), u.heard_by.end(), other) != u.heard_by.end();
  return false;
}
/* Do DUs a and b interact at all (any UE of one heard by the other)? */
inline bool coord_related(const CoordDu &a, const CoordDu &b) {
  for (const CoordUe &u : a.ues)
    if (std::find(u.heard_by.begin(), u.heard_by.end(), b.du) != u.heard_by.end())
      return true;
  for (const CoordUe &u : b.ues)
    if (std::find(u.heard_by.begin(), u.heard_by.end(), a.du) != u.heard_by.end())
      return true;
  return false;
}

namespace coord_detail {
inline std::vector<uint16_t> all_ues(const CoordDu &d) {
  std::vector<uint16_t> v;
  for (const CoordUe &u : d.ues)
    v.push_back(u.ue);
  return v;
}
inline bool items_conflict(const std::vector<CoordDu> &dus, const std::map<uint16_t, uint8_t> &ch,
                           const Item &x, const Item &y) {
  const CoordDu &s = dus[x.du_idx], &o = dus[y.du_idx];
  if (x.du_idx == y.du_idx || ch.at(s.du) != ch.at(o.du))
    return false;
  const int64_t P = s.period_us == o.period_us ? s.period_us : 0; /* unknown: overlap */
  if (!arcs_overlap(x.start, x.len, y.start, y.len, P))
    return false;
  for (uint16_t u : x.ues)
    if (coord_heard(s, u, o.du))
      return true;
  for (uint16_t u : y.ues)
    if (coord_heard(o, u, s.du))
      return true;
  return false;
}
inline void pos_items(const std::vector<CoordDu> &dus, size_t di, uint8_t pos, uint16_t ue,
                      std::vector<Item> &out) {
  const CoordDu &d = dus[di];
  if (pos == 0 || pos > d.positions.size())
    return;
  const CoordPosition &p = d.positions[pos - 1];
  out.push_back({di, d.phase_us + p.dl.start_us, p.dl.len_us, {ue}});
  out.push_back({di, d.phase_us + p.ul.start_us, p.ul.len_us, {ue}});
}
inline void fixed_items(const std::vector<CoordDu> &dus, size_t di, std::vector<Item> &out) {
  const CoordDu &d = dus[di];
  for (const CoordInterval &f : d.fixed)
    out.push_back({di, d.phase_us + f.start_us, f.len_us, all_ues(d)});
}
} /* namespace coord_detail */

/* Conflicts of the CURRENT assignment (each UE in its `pos`): pairs of
 * interfering items. What an uncoordinated network suffers. */
inline size_t coord_conflicts(const std::vector<CoordDu> &dus,
                              const std::map<uint16_t, uint8_t> *channels = nullptr) {
  using namespace coord_detail;
  std::map<uint16_t, uint8_t> ch;
  for (const CoordDu &d : dus)
    ch[d.du] = channels && channels->count(d.du) ? channels->at(d.du) : d.channel;
  std::vector<Item> items;
  for (size_t i = 0; i < dus.size(); ++i) {
    fixed_items(dus, i, items);
    for (const CoordUe &u : dus[i].ues)
      pos_items(dus, i, u.pos, u.ue, items);
  }
  size_t n = 0;
  for (size_t i = 0; i < items.size(); ++i)
    for (size_t j = i + 1; j < items.size(); ++j)
      if (items_conflict(dus, ch, items[i], items[j]))
        ++n;
  return n;
}

/* The plan. `spare_channels`: channels the CU may move a DU to (each used
 * once), for frequency separation. */
inline CoordPlan coord_plan(const std::vector<CoordDu> &dus,
                            const std::vector<uint8_t> &spare_channels = {}) {
  using namespace coord_detail;
  std::map<uint16_t, uint8_t> ch;
  for (const CoordDu &d : dus)
    ch[d.du] = d.channel;
  std::vector<uint8_t> spare = spare_channels;
  std::vector<uint16_t> retuned;
  auto take_spare = [&](uint16_t du) {
    for (auto it = spare.begin(); it != spare.end(); ++it) {
      bool used = false;
      for (const auto &kv : ch)
        used = used || kv.second == *it;
      if (!used) {
        ch[du] = *it;
        retuned.push_back(du);
        spare.erase(it);
        return true;
      }
    }
    return false;
  };
  /* 1. Tiers: a non-tight DU never shares a channel with a related DU. */
  std::set<uint16_t> unseparated;
  for (size_t i = 0; i < dus.size(); ++i) {
    if (dus[i].tier == SyncTier::Tight)
      continue;
    for (size_t j = 0; j < dus.size(); ++j)
      if (j != i && ch[dus[i].du] == ch[dus[j].du] && coord_related(dus[i], dus[j])) {
        if (!take_spare(dus[i].du))
          unseparated.insert(dus[i].du);
        break;
      }
  }
  for (size_t round = 0;; ++round) {
    CoordPlan plan;
    plan.channel = ch;
    plan.retuned = retuned;
    plan.unseparated.assign(unseparated.begin(), unseparated.end());
    std::vector<Item> placed;
    for (size_t i = 0; i < dus.size(); ++i)
      fixed_items(dus, i, placed);
    std::optional<uint16_t> retune;
    for (size_t i = 0; i < placed.size(); ++i)
      for (size_t j = i + 1; j < placed.size(); ++j)
        if (items_conflict(dus, ch, placed[i], placed[j])) {
          ++plan.fixed_conflicts;
          if (!retune)
            retune = dus[placed[j].du_idx].du;
        }
    /* The UEs, most-related first (then DU order, then id: deterministic). */
    struct Ref {
      size_t di;
      size_t ui;
      size_t rel;
    };
    std::vector<Ref> order;
    for (size_t i = 0; i < dus.size(); ++i)
      for (size_t k = 0; k < dus[i].ues.size(); ++k) {
        size_t rel = 0;
        for (uint16_t o : dus[i].ues[k].heard_by)
          for (const CoordDu &d : dus)
            rel += d.du == o && ch[d.du] == ch[dus[i].du];
        order.push_back({i, k, rel});
      }
    std::stable_sort(order.begin(), order.end(),
                     [](const Ref &a, const Ref &b) { return a.rel > b.rel; });
    std::map<size_t, std::set<uint8_t>> used; /* per DU index: positions taken */
    std::map<std::pair<size_t, size_t>, uint8_t> chosen;
    for (const Ref &r : order) {
      const CoordDu &d = dus[r.di];
      const CoordUe &u = d.ues[r.ui];
      std::vector<uint8_t> cand;
      if (u.pos >= 1 && u.pos <= d.positions.size())
        cand.push_back(u.pos);
      for (size_t p = 1; p <= d.positions.size(); ++p)
        if (p != u.pos)
          cand.push_back(static_cast<uint8_t>(p));
      std::optional<uint8_t> pick, first_free;
      for (uint8_t p : cand) {
        if (used[r.di].count(p))
          continue;
        if (!first_free)
          first_free = p;
        std::vector<Item> mine;
        pos_items(dus, r.di, p, u.ue, mine);
        bool ok = true;
        for (const Item &x : mine)
          for (const Item &y : placed)
            ok = ok && !items_conflict(dus, ch, x, y);
        if (ok) {
          pick = p;
          break;
        }
      }
      if (!pick) {
        ++plan.unresolved;
        if (!retune && !unseparated.count(d.du))
          retune = d.du;
        pick = first_free ? *first_free : u.pos;
      } else if (unseparated.count(d.du)) {
        ++plan.unresolved; /* a time slot is no answer for a coarse-sync DU */
      }
      used[r.di].insert(*pick);
      chosen[{r.di, r.ui}] = *pick;
      pos_items(dus, r.di, *pick, u.ue, placed);
      if (*pick != u.pos)
        ++plan.moved;
    }
    for (size_t i = 0; i < dus.size(); ++i) {
      auto &v = plan.assign[dus[i].du];
      for (size_t k = 0; k < dus[i].ues.size(); ++k)
        v.push_back({dus[i].ues[k].ue, chosen[{i, k}]});
    }
    if (!retune || round >= dus.size() || !take_spare(*retune))
      return plan;
  }
}

} /* namespace devourer::cell */

#endif /* DEVOURER_CELL_COORD_H */
