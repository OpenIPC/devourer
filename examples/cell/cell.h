// cell.h — the M1 (#255) reference cell node's pure logic: identities, the
// three on-air frame shapes, clock bookkeeping and the two control loops.
// Header-only, no radio dependency (examples/cell/main.cpp drives a radio with
// it; tests/cell_node_selftest.cpp covers it).
//
// On-air (one channel, one superframe = one beacon interval, src/cell/SlotGrid):
//   DU beacon   — hardware beacon at the DU's TBTT (slot 0), SA = du_mac(cell),
//                 carrying the grant map IE (src/cell/GrantMap.h).
//   DL frame    — one unicast QoS-Data per UE in its DL slot, RA = ue_mac(ue),
//                 TA = du_mac, normal ACK policy: the UE arms the hardware ACK
//                 responder for its MAC. Kestrel's TX descriptor has no no-ACK
//                 path, so an unACKed unicast counts as a failure and escalates
//                 the queue's contention window to CWmax — the next frame then
//                 waits a uniform 0..1023 slots (0-9.2 ms; measured on the
//                 8852CE). Body: the HARQ seq, window low edge, process id and
//                 attempt, versioned and tagged (layout at build_dl_frame).
//   UE uplink   — the UE's own hardware beacon, SA = ue_mac(ue), placed in its
//                 UL slot by stepping the UE's TSF; its 'C' 'U' IE carries the
//                 grant epoch it runs, its HARQ feedback (src/cell/Harq.h; the
//                 report's generation superframe extended by the DU around its
//                 own count) and its DL SNR, versioned and tagged (layout at
//                 build_ue_beacon). A hardware beacon
//                 because send_packet has no sub-slot departure control
//                 (docs/scheduled-mac.md); with SetBeaconBackoff(false) it
//                 leaves a fixed 22 us after its TBTT.
//   RACH msgA   — (M3) a joining station's broadcast data frame in the RACH
//                 slot: addr2 = its own (random) MAC, addr3 = the DU (BSSID),
//                 body = the src/cell/Rach.h msgA. Broadcast: nothing ACKs
//                 it, so no contention-window escalation on the joiner.
//   RACH msgB   — (M3) a second vendor IE in the DU beacon: the Rach.h
//                 random-access responses ('R' 'B' ...).
//
// Clocks. Kestrel stamps RX frames (RxAtrib.tsfl) with the MAC's free-running
// counter, which TSF writes never move. So a UE fits the DU's TSF against its
// own free-running counter (devourer::TsfSync over the DU beacons) and that fit
// stays valid while the UE steps its TSF to place its uplink. The DU never
// steps its TSF, so its tsf - freerun offset is a constant it reads once.
#ifndef DEVOURER_EXAMPLES_CELL_H
#define DEVOURER_EXAMPLES_CELL_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

#include "cell/GrantMap.h"
#include "cell/Harq.h"
#include "cell/SlotGrid.h"

namespace cellnode {

using devourer::cell::GrantMap;
using devourer::cell::SlotGrid;
using devourer::cell::SlotKind;

// --- Identities ---------------------------------------------------------------
// Locally administered unicast: 02:ce:11:<cell hi>:<cell lo>:<ue>, ue 0 = DU.
inline void cell_mac(uint8_t out[6], uint16_t cell, uint8_t ue) {
  const uint8_t m[6] = {0x02, 0xce, 0x11, static_cast<uint8_t>(cell >> 8),
                        static_cast<uint8_t>(cell), ue};
  std::memcpy(out, m, 6);
}
// The UE id when `mac` is a station of `cell` (0 = its DU), else -1.
inline int cell_station(const uint8_t *mac, uint16_t cell) {
  uint8_t m[6];
  cell_mac(m, cell, 0);
  return std::memcmp(mac, m, 5) == 0 ? mac[5] : -1;
}

// --- Frames --------------------------------------------------------------------
namespace detail {
inline void put(std::vector<uint8_t> &o, uint64_t v, int n) {
  for (int i = 0; i < n; ++i)
    o.push_back(static_cast<uint8_t>(v >> (8 * i)));
}
inline uint64_t get(const uint8_t *p, int n) {
  uint64_t v = 0;
  for (int i = 0; i < n; ++i)
    v |= static_cast<uint64_t>(p[i]) << (8 * i);
  return v;
}
// 24-byte beacon header + timestamp/interval/capability + SSID "CELL" + rates.
inline std::vector<uint8_t> beacon_head(const uint8_t sa[6], int interval_tu) {
  std::vector<uint8_t> f = {0x80, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  f.insert(f.end(), sa, sa + 6);
  f.insert(f.end(), sa, sa + 6);
  f.insert(f.end(), {0x00, 0x00, 0, 0, 0, 0, 0, 0, 0, 0});
  f.push_back(static_cast<uint8_t>(interval_tu));
  f.push_back(static_cast<uint8_t>(interval_tu >> 8));
  f.insert(f.end(), {0x00, 0x00, 0x00, 0x04, 'C', 'E', 'L', 'L', 0x01, 0x01, 0x8c});
  return f;
}
}  // namespace detail

// The DU beacon: header + the grant map IE (+ the RACH responses IE when
// `rach_rsp` — an encode_rach_responses body — has any entry). Empty if
// either cannot encode.
inline std::vector<uint8_t> build_du_beacon(const GrantMap &m,
                                            const std::vector<uint8_t> &rach_rsp = {}) {
  uint8_t sa[6];
  cell_mac(sa, m.cell_id, 0);
  auto f = detail::beacon_head(sa, m.period_tu);
  const auto ie = devourer::cell::encode_grant_ie(m);
  if (ie.empty())
    return {};
  f.insert(f.end(), ie.begin(), ie.end());
  if (rach_rsp.size() > 3) { /* 'R' 'B' n, n > 0 */
    if (rach_rsp.size() + 3 > 255)
      return {};
    f.insert(f.end(), {221, static_cast<uint8_t>(3 + rach_rsp.size()), devourer::cell::kGrantOui[0],
                       devourer::cell::kGrantOui[1], devourer::cell::kGrantOui[2]});
    f.insert(f.end(), rach_rsp.begin(), rach_rsp.end());
  }
  return f;
}

// The RACH responses body ('R' 'B' ...) in a DU beacon, or {nullptr, 0}.
inline std::pair<const uint8_t *, size_t> find_rach_responses(const uint8_t *mpdu, size_t len) {
  if (mpdu == nullptr || len < 36 || mpdu[0] != 0x80)
    return {nullptr, 0};
  for (size_t off = 36; off + 2 <= len;) {
    const size_t l = mpdu[off + 1];
    if (off + 2 + l > len)
      break;
    const uint8_t *v = mpdu + off + 2;
    if (mpdu[off] == 221 && l >= 5 && v[0] == devourer::cell::kGrantOui[0] &&
        v[1] == devourer::cell::kGrantOui[1] && v[2] == devourer::cell::kGrantOui[2] &&
        v[3] == 'R' && v[4] == 'B')
      return {v + 3, l - 3};
    off += 2 + l;
  }
  return {nullptr, 0};
}

// RACH msgA frame: broadcast data (FC 0x08), addr2 = the joiner's MAC,
// addr3 = the DU of `cell`, body = a Rach.h msgA payload.
inline std::vector<uint8_t> build_rach_frame(uint16_t cell, const uint8_t joiner[6],
                                             const std::vector<uint8_t> &msga) {
  uint8_t du[6];
  cell_mac(du, cell, 0);
  std::vector<uint8_t> f = {0x08, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  f.insert(f.end(), joiner, joiner + 6);
  f.insert(f.end(), du, du + 6);
  f.insert(f.end(), {0x00, 0x00});
  f.insert(f.end(), msga.begin(), msga.end());
  return f;
}
// A RACH frame for `cell`: the joiner MAC and the msgA payload, or nullopt.
// `len` must EXCLUDE a trailing FCS (Packet::Data carries one when
// RxAtrib.fcs_present): the msgA decode is length-exact.
struct RachFrame {
  std::array<uint8_t, 6> joiner{};
  const uint8_t *msga = nullptr;
  size_t msga_len = 0;
};
inline std::optional<RachFrame> parse_rach_frame(const uint8_t *p, size_t len, uint16_t cell) {
  if (p == nullptr || len < 24 + 2 || p[0] != 0x08 || p[24] != 'R' || p[25] != 'A')
    return std::nullopt;
  static const uint8_t bcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  if (std::memcmp(p + 4, bcast, 6) != 0 || cell_station(p + 16, cell) != 0)
    return std::nullopt;
  RachFrame r;
  std::memcpy(r.joiner.data(), p + 10, 6);
  r.msga = p + 24;
  r.msga_len = len - 24;
  return r;
}


/* --- Scheduling-frame integrity (M2 review) ---------------------------------
 * The UE uplink IE (HARQ feedback) and the DL body carry a protocol version
 * and an 8-byte tag: HMAC-SHA1 keyed by the cell PSK over a direction label,
 * the cell id and the body, truncated (the primitive is injected, as for the
 * RACH: `CellMac`; examples fill it from OpenSSL). With no key (an empty
 * CellMac) the tag is all-zero and receivers accept it: an unauthenticated
 * development cell. A receiver WITH a key rejects a bad or missing tag. Replay:
 * the DU accepts a UE's feedback only with a gen_sf no older than the last it
 * accepted (main.cpp); a replayed DL frame is a HARQ duplicate. */
using CellMac = std::function<bool(const uint8_t *data, size_t len, uint8_t out[20])>;
constexpr uint8_t kCellProtoVersion = 2;
constexpr size_t kCellTagLen = 8;

namespace detail {
inline void cell_tag(const CellMac &mac, char label, uint16_t cell, const uint8_t *body, size_t n,
                     uint8_t out[kCellTagLen]) {
  std::memset(out, 0, kCellTagLen);
  if (!mac)
    return;
  std::vector<uint8_t> m = {static_cast<uint8_t>(label), static_cast<uint8_t>(cell),
                            static_cast<uint8_t>(cell >> 8)};
  m.insert(m.end(), body, body + n);
  uint8_t full[20];
  if (mac(m.data(), m.size(), full))
    std::memcpy(out, full, kCellTagLen);
}
inline bool cell_tag_ok(const CellMac &mac, char label, uint16_t cell, const uint8_t *body, size_t n,
                        const uint8_t *tag) {
  if (!mac)
    return true; /* unauthenticated cell */
  uint8_t want[kCellTagLen];
  cell_tag(mac, label, cell, body, n, want);
  uint8_t d = 0;
  for (size_t i = 0; i < kCellTagLen; ++i)
    d |= static_cast<uint8_t>(want[i] ^ tag[i]);
  return d == 0;
}
}  // namespace detail

/* UE IE body after the OUI: 'C' 'U' ver ue(2) epoch(4) base(4) bitmap(4)
 * gen_sf(4, low 32 bits) snr(1) snr_gen(1) | tag(8). */
constexpr size_t kUeBodyLen = 2 + 1 + 2 + 4 + 4 + 4 + 4 + 1 + 1;
constexpr uint8_t kUeIeLen = static_cast<uint8_t>(3 + kUeBodyLen + kCellTagLen);

// The UE uplink beacon: header + the 'C' 'U' vendor IE above. dl_snr_half_db
// is the mean DL SNR (0.5 dB units, 0 = none) and snr_gen counts SNR
// measurements: the DU folds a reported SNR once per new snr_gen, not once per
// beacon that repeats it.
inline std::vector<uint8_t> build_ue_beacon(uint16_t cell, uint8_t ue, int interval_tu,
                                            uint32_t epoch,
                                            devourer::cell::HarqFeedback fb = {},
                                            uint8_t dl_snr_half_db = 0, uint8_t snr_gen = 0,
                                            const CellMac &mac = {}) {
  uint8_t sa[6];
  cell_mac(sa, cell, ue);
  auto f = detail::beacon_head(sa, interval_tu);
  std::vector<uint8_t> b = {'C', 'U', kCellProtoVersion};
  detail::put(b, ue, 2);
  detail::put(b, epoch, 4);
  detail::put(b, fb.base, 4);
  detail::put(b, fb.bitmap, 4);
  detail::put(b, fb.gen_sf & 0xFFFFFFFFull, 4);
  b.push_back(dl_snr_half_db);
  b.push_back(snr_gen);
  uint8_t t[kCellTagLen];
  detail::cell_tag(mac, 'U', cell, b.data(), b.size(), t);
  f.insert(f.end(), {221, kUeIeLen, devourer::cell::kGrantOui[0], devourer::cell::kGrantOui[1],
                     devourer::cell::kGrantOui[2]});
  f.insert(f.end(), b.begin(), b.end());
  f.insert(f.end(), t, t + kCellTagLen);
  return f;
}

struct UeBeacon {
  uint8_t ue = 0;
  uint32_t epoch = 0;
  devourer::cell::HarqFeedback fb;
  uint8_t dl_snr_half_db = 0; /* 0 = none */
  uint8_t snr_gen = 0;
};
// Parse a UE uplink beacon of `cell` (MPDU, radiotap stripped): nullopt for
// another cell, another protocol version, an SA/body mismatch, or (keyed
// cell) a tag that does not verify.
inline std::optional<UeBeacon> parse_ue_beacon(const uint8_t *p, size_t len, uint16_t cell,
                                               const CellMac &mac = {}) {
  if (p == nullptr || len < 36 || p[0] != 0x80)
    return std::nullopt;
  const int st = cell_station(p + 10, cell);
  if (st <= 0)
    return std::nullopt;
  for (size_t off = 36; off + 2 <= len;) {
    const size_t l = p[off + 1];
    if (off + 2 + l > len)
      break;
    const uint8_t *v = p + off + 2;
    if (p[off] == 221 && l == kUeIeLen && v[0] == devourer::cell::kGrantOui[0] &&
        v[1] == devourer::cell::kGrantOui[1] && v[2] == devourer::cell::kGrantOui[2] &&
        v[3] == 'C' && v[4] == 'U') {
      const uint8_t *b = v + 3;
      if (b[2] != kCellProtoVersion)
        return std::nullopt;
      const uint64_t ue = detail::get(b + 3, 2);  // 16 bits on air
      if (ue != static_cast<uint64_t>(st))
        return std::nullopt;  // SA and body disagree
      if (!detail::cell_tag_ok(mac, 'U', cell, b, kUeBodyLen, b + kUeBodyLen))
        return std::nullopt;
      UeBeacon r;
      r.ue = static_cast<uint8_t>(ue);
      r.epoch = static_cast<uint32_t>(detail::get(b + 5, 4));
      r.fb.base = static_cast<uint32_t>(detail::get(b + 9, 4));
      r.fb.bitmap = static_cast<uint32_t>(detail::get(b + 13, 4));
      r.fb.gen_sf = detail::get(b + 17, 4); /* low 32 bits: extend at the DU */
      r.dl_snr_half_db = b[21];
      r.snr_gen = b[22];
      return r;
    }
    off += 2 + l;
  }
  return std::nullopt;
}

// HARQ fields of a DL frame (src/cell/Harq.h HarqTx).
struct DlHarq {
  uint32_t low = 0;    /* the sender's window low edge */
  uint8_t pid = 0;
  uint8_t attempt = 1; /* 1 = first transmission */
};

/* DL body: 'C' 'D' ver ue(2) sf(8) seq(4) low(4) pid(1) attempt(1) | tag(8). */
constexpr size_t kDlTaggedLen = 2 + 1 + 2 + 8 + 4 + 4 + 1 + 1;
constexpr size_t kDlBodyLen = kDlTaggedLen + kCellTagLen;

// The DL frame MPDU (no FCS): QoS-Data, RA = UE, TA = BSSID = DU.
inline std::vector<uint8_t> build_dl_frame(uint16_t cell, uint8_t ue, uint64_t sf, uint32_t seq,
                                           size_t pad, DlHarq h = {}, const CellMac &mac = {}) {
  uint8_t ra[6], ta[6];
  cell_mac(ra, cell, ue);
  cell_mac(ta, cell, 0);
  std::vector<uint8_t> f = {0x88, 0x02, 0x00, 0x00};  // QoS-Data, FromDS
  f.insert(f.end(), ra, ra + 6);
  f.insert(f.end(), ta, ta + 6);
  f.insert(f.end(), ta, ta + 6);
  f.insert(f.end(), {0x00, 0x00, 0x00, 0x00});  // seq ctl, QoS ctl: TID 0, normal ACK
  std::vector<uint8_t> b = {'C', 'D', kCellProtoVersion};
  detail::put(b, ue, 2);
  detail::put(b, sf, 8);
  detail::put(b, seq, 4);
  detail::put(b, h.low, 4);
  b.push_back(h.pid);
  b.push_back(h.attempt);
  uint8_t t[kCellTagLen];
  detail::cell_tag(mac, 'D', cell, b.data(), b.size(), t);
  f.insert(f.end(), b.begin(), b.end());
  f.insert(f.end(), t, t + kCellTagLen);
  f.insert(f.end(), pad, 0x5a);
  return f;
}

struct DlFrame {
  uint8_t ue = 0;
  uint64_t sf = 0;
  uint32_t seq = 0;
  DlHarq harq;
};
inline std::optional<DlFrame> parse_dl_frame(const uint8_t *p, size_t len, uint16_t cell,
                                             const CellMac &mac = {}) {
  if (p == nullptr || len < 26 + kDlBodyLen || p[0] != 0x88)
    return std::nullopt;
  const int ra = cell_station(p + 4, cell), ta = cell_station(p + 10, cell);
  const uint8_t *b = p + 26;
  if (ra <= 0 || ta != 0 || b[0] != 'C' || b[1] != 'D' || b[2] != kCellProtoVersion)
    return std::nullopt;
  const uint64_t ue = detail::get(b + 3, 2);  // 16 bits on air
  if (ue != static_cast<uint64_t>(ra))
    return std::nullopt;
  if (!detail::cell_tag_ok(mac, 'D', cell, b, kDlTaggedLen, b + kDlTaggedLen))
    return std::nullopt;
  DlFrame d;
  d.ue = static_cast<uint8_t>(ue);
  d.sf = detail::get(b + 5, 8);
  d.seq = static_cast<uint32_t>(detail::get(b + 13, 4));
  d.harq.low = static_cast<uint32_t>(detail::get(b + 17, 4));
  d.harq.pid = b[21];
  d.harq.attempt = b[22];
  return d;
}

// Radiotap for one DL frame: the legacy rate (Mb/s), and — only when
// `pwr_db` is set — a DBM_TX_POWER field. On Kestrel that field is an
// ABSOLUTE per-frame level against the calibrated base (it bypasses the
// session offset and the per-rate shape), so an ordinary frame must not carry
// it: present = RATE (bit 2) alone, or RATE | DBM_TX_POWER (bit 10). No
// TX_FLAGS, so the frame keeps normal-ACK semantics.
inline std::vector<uint8_t> build_dl_radiotap(int rate_mbps, std::optional<int8_t> pwr_db = {}) {
  if (!pwr_db)
    return {0x00, 0x00, 0x09, 0x00, 0x04, 0x00, 0x00, 0x00, static_cast<uint8_t>(rate_mbps * 2)};
  return {0x00, 0x00, 0x0a, 0x00, 0x04, 0x04, 0x00, 0x00,
          static_cast<uint8_t>(rate_mbps * 2), static_cast<uint8_t>(*pwr_db)};
}

// --- Clocks --------------------------------------------------------------------
// 32-bit -> 64-bit counter reconstruction, mirroring devourer::TsfSync's: feed
// it the same RX tsfl stream in the same order, and extend() maps a later
// 32-bit reading of the same counter into that 64-bit domain.
class Wrap32 {
public:
  int64_t feed(uint32_t lo) {
    if (init_ && lo < prev_)
      hi_ += int64_t{1} << 32;
    prev_ = lo;
    init_ = true;
    return hi_ + lo;
  }
  // A reading taken after the last feed(): if it is numerically far below the
  // last fed value the counter has wrapped since.
  int64_t extend(uint32_t lo) const {
    int64_t v = hi_ + lo;
    if (init_ && lo + (uint64_t{1} << 31) < prev_)
      v += int64_t{1} << 32;
    return v;
  }
  bool ready() const { return init_; }

private:
  bool init_ = false;
  uint32_t prev_ = 0;
  int64_t hi_ = 0;
};

// The 64-bit value congruent to `lo` mod 2^32 that is nearest to `ref` (a
// recent full 64-bit read of the same counter): extends an RX tsfl stamp.
inline int64_t extend32_near(uint32_t lo, int64_t ref) {
  const int64_t base = ref & ~((int64_t{1} << 32) - 1);
  int64_t v = base | lo;
  if (v - ref > (int64_t{1} << 31))
    v -= int64_t{1} << 32;
  else if (ref - v > (int64_t{1} << 31))
    v += int64_t{1} << 32;
  return v;
}

// Wrap a phase error into [-period/2, period/2).
inline int64_t wrap_phase(int64_t err, int64_t period) {
  int64_t e = err % period;
  if (e < 0)
    e += period;
  if (e >= period / 2)
    e -= period;
  return e;
}

// UE uplink placement. The UE's beacon fires when ITS tsf % period == 0; it
// must fire when the DU's tsf % period == ul_off (UL slot start on the DU
// grid) minus the timing advance. So the UE wants its tsf to read
// du_tsf - ul_off + ta. Returns the TSF step to apply now (0 within
// `deadband_us`), given the UE's current tsf and the DU tsf for the same
// instant.
inline int64_t ue_tsf_step(int64_t ue_tsf, int64_t du_tsf, int64_t ul_off, int64_t ta_us,
                           int64_t period, int64_t deadband_us) {
  const int64_t want = du_tsf - ul_off + ta_us;
  const int64_t err = wrap_phase(ue_tsf - want, period);  // + = UE ahead
  return (err > deadband_us || err < -deadband_us) ? -err : 0;
}

// DU timing-advance controller for one UE: the measured arrival error (DU
// clock, arrival phase minus target) is averaged over a window; at each
// update() the TA moves by -gain * mean, rounded to whole us, clamped to
// +-max_ta. Positive TA = the UE fires earlier.
class TaLoop {
public:
  TaLoop(double gain, int64_t max_ta_us) : gain_(gain), max_(max_ta_us) {}
  /* The window keeps the newest kMaxWindow samples (a loop that is never
   * updated, e.g. CELL_TA=0, must not grow without bound). */
  static constexpr size_t kMaxWindow = 256;
  void sample(int64_t err_us) {
    if (win_.size() == kMaxWindow)
      win_.erase(win_.begin());
    win_.push_back(err_us);
  }
  size_t pending() const { return win_.size(); }
  // Drop the window without moving the TA (samples taken under an old map).
  void discard() { win_.clear(); }
  // Returns true when the TA changed. min_samples guards a noisy window.
  // The window's MEDIAN is the estimate: one wild arrival (a UE's beacon
  // before its first TSF step, a TBTT on the old schedule after one) must
  // not steer the TA — a mean let a single 39 ms sample push a joining UE
  // out of its slot on the rig.
  bool update(size_t min_samples) {
    if (win_.size() < min_samples || win_.empty())
      return false;
    std::vector<int64_t> v;
    v.swap(win_);
    std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2), v.end());
    const double mean = static_cast<double>(v[v.size() / 2]); /* the median */
    last_mean_ = mean;
    const int64_t step = static_cast<int64_t>(std::llround(gain_ * mean));
    if (step == 0)
      return false;
    int64_t t = ta_ + step;  // arrival late (mean > 0) -> fire earlier -> TA up
    if (t > max_)
      t = max_;
    if (t < -max_)
      t = -max_;
    const bool changed = t != ta_;
    ta_ = t;
    return changed;
  }
  int64_t ta() const { return ta_; }
  // Restore a TA (a map that carried a moved TA failed to publish).
  void set_ta(int64_t t) { ta_ = t; }
  double last_mean() const { return last_mean_; }

private:
  double gain_;
  int64_t max_;
  int64_t ta_ = 0;
  std::vector<int64_t> win_;
  double last_mean_ = 0;
};

}  // namespace cellnode

#endif /* DEVOURER_EXAMPLES_CELL_H */
