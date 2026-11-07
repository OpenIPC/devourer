/* UeRegistry.h — the cell's UE pool (M3 #257): which UE id, and which DL/UL
 * slot pair of the grid, each admitted station holds.
 *
 * The grid is laid out for a fixed capacity (src/cell/SlotGrid.h
 * build_fixed_grid with placeholder positions 1..capacity), so slot positions
 * never move when a UE joins or leaves: admitting a station only binds a free
 * position to it. A station that re-joins (same MAC) keeps its id and slots —
 * idempotent admission, so a lost response and a retried request cannot hand
 * one MAC two ids. release() frees a position (departure; the policy of when
 * is the caller's).
 *
 * Pure header; cell_m3 ctest covers it. */
#ifndef DEVOURER_CELL_UEREGISTRY_H
#define DEVOURER_CELL_UEREGISTRY_H

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <vector>

#include "cell/SlotGrid.h"

namespace devourer::cell {

struct UeSlot {
  uint16_t ue = 0;       /* UE id = grid position (1..capacity) */
  uint8_t dl_slot = 0;   /* SlotGrid indices */
  uint8_t ul_slot = 0;
};

class UeRegistry {
public:
  /* positions[i] = the DL/UL slot indices of grid position i + 1. At most
   * kMaxUesPerCell positions are kept: a grid and one grant map cannot carry
   * more (build_fixed_grid / encode_grant_ie refuse), so the pool never
   * admits a station it could not schedule. */
  explicit UeRegistry(std::vector<std::pair<uint8_t, uint8_t>> positions)
      : pos_(std::move(positions)) {
    if (pos_.size() > kMaxUesPerCell)
      pos_.resize(kMaxUesPerCell);
    owner_.resize(pos_.size());
  }

  size_t capacity() const { return pos_.size(); }
  size_t size() const { return by_mac_.size(); }

  /* Bind `mac` to a free position, or return its existing one. nullopt when
   * the pool is full. */
  std::optional<UeSlot> admit(const std::array<uint8_t, 6> &mac) {
    auto it = by_mac_.find(mac);
    if (it != by_mac_.end())
      return slot_of(it->second);
    for (size_t i = 0; i < owner_.size(); ++i)
      if (!owner_[i]) {
        owner_[i] = mac;
        by_mac_[mac] = i;
        return slot_of(i);
      }
    return std::nullopt;
  }
  /* Pre-bind (a statically configured UE): same as admit. */
  std::optional<UeSlot> bind(const std::array<uint8_t, 6> &mac) { return admit(mac); }

  bool release(const std::array<uint8_t, 6> &mac) {
    auto it = by_mac_.find(mac);
    if (it == by_mac_.end())
      return false;
    owner_[it->second].reset();
    by_mac_.erase(it);
    return true;
  }

  /* Release by UE id (the DU knows a silent UE by its id, not its MAC). */
  bool release_id(uint16_t ue) {
    if (ue == 0 || ue > owner_.size() || !owner_[ue - 1])
      return false;
    by_mac_.erase(*owner_[ue - 1]);
    owner_[ue - 1].reset();
    return true;
  }

  std::optional<UeSlot> find(const std::array<uint8_t, 6> &mac) const {
    auto it = by_mac_.find(mac);
    if (it == by_mac_.end())
      return std::nullopt;
    return slot_of(it->second);
  }
  /* The admitted UEs, in position order. */
  std::vector<UeSlot> admitted() const {
    std::vector<UeSlot> v;
    for (size_t i = 0; i < owner_.size(); ++i)
      if (owner_[i])
        v.push_back(slot_of(i));
    return v;
  }

private:
  UeSlot slot_of(size_t i) const {
    return UeSlot{static_cast<uint16_t>(i + 1), pos_[i].first, pos_[i].second};
  }
  std::vector<std::pair<uint8_t, uint8_t>> pos_;
  std::vector<std::optional<std::array<uint8_t, 6>>> owner_;
  std::map<std::array<uint8_t, 6>, size_t> by_mac_;
};

} /* namespace devourer::cell */

#endif /* DEVOURER_CELL_UEREGISTRY_H */
