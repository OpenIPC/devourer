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
 * Per-UE state (M1): each UE id the cell schedules has a UeContext — its
 * timing-advance loop (TimingAdvance.h) and its latest uplink-quality window,
 * attributed per transmitter from the frames' own RSSI/SNR/EVM
 * (UeRxAttribution.h; not the device-wide GetRxQuality). A context comes with
 * a bind or an admission, is dropped with a release (an id reused by another
 * UE keeps nothing of it), and context() creates one for an id the caller
 * schedules otherwise (a handover arrival before its admission). The first
 * uplink window after a context begins is skipped: it may hold the previous
 * holder's frames, whose transmitter address (the id's) is the same.
 *
 * Pure header; cell_m3 ctest covers it. */
#ifndef DEVOURER_CELL_UEREGISTRY_H
#define DEVOURER_CELL_UEREGISTRY_H

#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <vector>

#include "cell/SlotGrid.h"
#include "cell/TimingAdvance.h"
#include "cell/UeRxAttribution.h"

namespace devourer::cell {

struct UeSlot {
  uint16_t ue = 0;       /* UE id = grid position (1..capacity) */
  uint8_t dl_slot = 0;   /* SlotGrid indices */
  uint8_t ul_slot = 0;
};

/* A scheduled UE's per-UE state. */
struct UeContext {
  TaLoop ta;
  UeRxWindow rx{};      /* the latest closed uplink-quality window */
  bool have_rx = false;
  /* The first window closed after the context began may hold a previous
   * holder's frames (the transmitter address is the id's): skipped. */
  bool rx_fresh = true;
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

  /* The TA loop each new context starts with (gain, +-max us). */
  void set_ta_config(double gain, int64_t max_us) {
    ta_gain_ = gain;
    ta_max_ = max_us;
  }
  /* A UE's context; created (fresh) if it has none. */
  UeContext &context(uint16_t ue) {
    auto it = ctx_.find(ue);
    if (it == ctx_.end())
      it = ctx_.emplace(ue, fresh()).first;
    return it->second;
  }
  const UeContext *find_context(uint16_t ue) const {
    auto it = ctx_.find(ue);
    return it == ctx_.end() ? nullptr : &it->second;
  }
  /* Start a UE's state over (a new holder of the id, or an aborted arrival). */
  void reset_context(uint16_t ue) { ctx_.insert_or_assign(ue, fresh()); }
  /* Uplink frames: attribute each to its transmitter (addr2). */
  bool rx_frame(const uint8_t *mpdu, size_t len, int rssi_raw, int snr_raw, int evm_raw, uint32_t tsfl) {
    return rx_.add_mpdu(mpdu, len, rssi_raw, snr_raw, evm_raw, tsfl);
  }
  /* Close the uplink-quality window: each transmitter `ue_of` maps to a UE id
   * (0: not one of ours) that still has a context gets the window in it.
   * Returns those (ue, window), in id order. */
  std::vector<std::pair<uint16_t, UeRxWindow>> close_rx_window(
      const std::function<uint16_t(const std::array<uint8_t, 6> &)> &ue_of) {
    std::vector<std::pair<uint16_t, UeRxWindow>> out;
    for (const UeRxWindow &w : rx_.snapshot().ues) {
      const uint16_t ue = ue_of(w.ta);
      auto c = ue ? ctx_.find(ue) : ctx_.end();
      if (c == ctx_.end())
        continue; /* not ours, or released meanwhile: never recreated by its old frames */
      if (c->second.rx_fresh) {
        c->second.rx_fresh = false;
        continue;
      }
      c->second.rx = w;
      c->second.have_rx = true;
      out.emplace_back(ue, w);
    }
    std::sort(out.begin(), out.end(), [](const auto &a, const auto &b) { return a.first < b.first; });
    return out;
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
        reset_context(static_cast<uint16_t>(i + 1));
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
    ctx_.erase(static_cast<uint16_t>(it->second + 1));
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
    ctx_.erase(ue);
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
  UeContext fresh() const { return UeContext{TaLoop(ta_gain_, ta_max_), {}, false, true}; }
  double ta_gain_ = 0.5;
  int64_t ta_max_ = 25600;
  std::map<uint16_t, UeContext> ctx_;
  UeRxAttribution rx_;
};

} /* namespace devourer::cell */

#endif /* DEVOURER_CELL_UEREGISTRY_H */
