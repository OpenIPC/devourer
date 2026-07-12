/* Per-UE RX attribution — the M0 seed of the scheduled-MAC UeRegistry (5G-NR
 * RAN epic, src/cell/ = the per-cell DU library).
 *
 * GetRxQuality() is deliberately device-wide: one draining accumulator fed by
 * every decoded frame, regardless of transmitter. A cell scheduler adapting
 * per-UE rate/power needs the same window statistics ATTRIBUTED to each
 * transmitter (the 802.11 addr2 / TA). All the primitives already exist per
 * frame — RSSI/SNR/EVM in rx_pkt_attrib, the raw MPDU (and thus the TA) in
 * Packet.Data — so this stays pure caller-side logic: the demos/probes feed
 * add() from the Packet callback, the device RX loops are untouched.
 *
 * Same conventions as RxQualityAccumulator (src/RxQuality.h): raw path-A
 * units in, drain-and-reset snapshot() out (delta semantics), rssi_raw <= 0
 * is not a sample, SNR/EVM folded only when present, passive noise floor =
 * (rssi_raw - 110) - snr_raw/2 per OFDM frame. */
#ifndef DEVOURER_CELL_UE_RX_ATTRIBUTION_H
#define DEVOURER_CELL_UE_RX_ATTRIBUTION_H

#include <array>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <vector>

namespace devourer {
namespace cell {

/* Extract the transmitter address (addr2) from an 802.11 MPDU. Every frame
 * type carries addr2 at bytes [10..16) EXCEPT the two shortest control
 * subtypes — CTS (fc0 0xc4) and ACK (0xd4) — which end at addr1. Returns
 * false for those, for truncated buffers, and for a zero TA (never a legal
 * transmitter; a parser artifact). */
inline bool extract_ta(const uint8_t *mpdu, size_t len, uint8_t out[6]) {
  if (!mpdu || len < 16)
    return false;
  const uint8_t fc0 = mpdu[0];
  if ((fc0 & 0x0c) == 0x04) { /* control frame */
    const uint8_t subtype = fc0 & 0xf0;
    if (subtype == 0xc0 /* CTS */ || subtype == 0xd0 /* ACK */)
      return false;
  }
  std::memcpy(out, mpdu + 10, 6);
  static const uint8_t zero[6] = {0, 0, 0, 0, 0, 0};
  return std::memcmp(out, zero, 6) != 0;
}

/* One UE's drained window — RxQualitySnapshot's frame-driven half, keyed and
 * converted (dBm / dB, so a scheduler doesn't repeat the raw conversions). */
struct UeRxWindow {
  std::array<uint8_t, 6> ta{};
  uint32_t frames = 0;
  int rssi_mean_dbm = 0; /* window mean of path-A PWDB (raw - 110) */
  int rssi_max_dbm = 0;
  double snr_mean_db = 0.0;
  double snr_min_db = 0.0;
  double evm_mean_db = 0.0; /* 0 when evm_valid is false */
  bool evm_valid = false;
  double noise_floor_dbm = 0.0; /* passive: mean rssi_dbm - snr_db */
  bool nf_valid = false;
  uint32_t last_tsfl = 0; /* hardware RX TSF of the newest frame (staleness) */
};

/* A drained attribution window: per-UE stats plus the frames the table had to
 * drop because it was full (a nonzero count means the cap is too small for
 * the environment, or a foreign-traffic filter belongs upstream). */
struct UeRxSnapshot {
  std::vector<UeRxWindow> ues;
  uint32_t evicted_frames = 0;
};

/* Thread-safe per-TA windowed accumulator. add() from the Packet callback for
 * every decoded frame; snapshot() drains the whole table (delta semantics —
 * UEs reappear with their next frame). Bounded: at most `cap` distinct TAs
 * per window; frames from further TAs are counted in evicted_frames, never
 * silently lost. */
class UeRxAttribution {
public:
  explicit UeRxAttribution(size_t cap = 64) : cap_(cap) {}

  /* Raw path-A values straight off rx_pkt_attrib, plus the frame's TA and
   * hardware RX timestamp. Mirrors RxQualityAccumulator::add. */
  void add(const uint8_t ta[6], int rssi_raw, int snr_raw, int evm_raw,
           uint32_t tsfl) {
    if (rssi_raw <= 0)
      return;
    std::lock_guard<std::mutex> lk(mu_);
    Entry *e = find_or_insert(ta);
    if (!e) {
      ++evicted_;
      return;
    }
    ++e->n;
    e->rssi_sum += rssi_raw;
    if (rssi_raw > e->rssi_max)
      e->rssi_max = rssi_raw;
    e->snr_sum += snr_raw;
    if (snr_raw < e->snr_min)
      e->snr_min = snr_raw;
    if (evm_raw != 0) {
      e->evm_sum += evm_raw;
      ++e->evm_n;
    }
    if (snr_raw != 0) {
      e->nf_sum += (rssi_raw - 110) - snr_raw / 2.0;
      ++e->nf_n;
    }
    e->last_tsfl = tsfl;
  }

  /* Convenience: extract the TA from the MPDU and add. Returns false when the
   * frame carries no TA (CTS/ACK/truncated) — not a quality sample. */
  bool add_mpdu(const uint8_t *mpdu, size_t len, int rssi_raw, int snr_raw,
                int evm_raw, uint32_t tsfl) {
    uint8_t ta[6];
    if (!extract_ta(mpdu, len, ta))
      return false;
    add(ta, rssi_raw, snr_raw, evm_raw, tsfl);
    return true;
  }

  /* Drain the window: one UeRxWindow per TA seen, then reset the table. */
  UeRxSnapshot snapshot() {
    UeRxSnapshot s;
    std::lock_guard<std::mutex> lk(mu_);
    s.ues.reserve(entries_.size());
    for (const Entry &e : entries_) {
      UeRxWindow w;
      w.ta = e.ta;
      w.frames = e.n;
      if (e.n) {
        w.rssi_mean_dbm = static_cast<int>(e.rssi_sum / static_cast<int64_t>(e.n)) - 110;
        w.rssi_max_dbm = e.rssi_max - 110;
        w.snr_mean_db = (e.snr_sum / static_cast<double>(e.n)) / 2.0;
        w.snr_min_db = e.snr_min / 2.0;
      }
      if (e.evm_n) {
        w.evm_mean_db = (e.evm_sum / static_cast<double>(e.evm_n)) / 2.0;
        w.evm_valid = true;
      }
      if (e.nf_n) {
        w.noise_floor_dbm = e.nf_sum / static_cast<double>(e.nf_n);
        w.nf_valid = true;
      }
      w.last_tsfl = e.last_tsfl;
      s.ues.push_back(w);
    }
    s.evicted_frames = evicted_;
    entries_.clear();
    evicted_ = 0;
    return s;
  }

private:
  struct Entry {
    std::array<uint8_t, 6> ta{};
    uint32_t n = 0;
    int64_t rssi_sum = 0;
    int rssi_max = -128;
    int64_t snr_sum = 0;
    int snr_min = 127;
    int64_t evm_sum = 0;
    uint32_t evm_n = 0;
    double nf_sum = 0.0;
    uint32_t nf_n = 0;
    uint32_t last_tsfl = 0;
  };

  /* Linear scan — the table is small (cap defaults to 64) and drains every
   * window, so a map's overhead buys nothing. Caller holds mu_. */
  Entry *find_or_insert(const uint8_t ta[6]) {
    for (Entry &e : entries_)
      if (std::memcmp(e.ta.data(), ta, 6) == 0)
        return &e;
    if (entries_.size() >= cap_)
      return nullptr;
    Entry e;
    std::memcpy(e.ta.data(), ta, 6);
    entries_.push_back(e);
    return &entries_.back();
  }

  std::mutex mu_;
  size_t cap_;
  std::vector<Entry> entries_;
  uint32_t evicted_ = 0;
};

} // namespace cell
} // namespace devourer

#endif /* DEVOURER_CELL_UE_RX_ATTRIBUTION_H */
