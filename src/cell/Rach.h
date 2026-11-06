/* Rach.h — M3 (#257) admission: a 2-step random-access join into a running
 * cell, authenticated before anything persistent is allocated.
 *
 *   msgA (UE -> DU, in the grid's RACH slot): the joining UE's MAC, a 64-bit
 *        random nonce (its temporary identity until it holds a UE id), the
 *        attempt number, a freshness stamp (the DU superframe it was sent in)
 *        and an authentication tag.
 *   msgB (DU -> UE, the random-access response in the DU's next beacon(s)):
 *        per answered request, the recipient's MAC and nonce, a status
 *        (granted / denied: pool full / denied: bad request), the UE id and
 *        its DL/UL slot indices, and a tag. A client accepts an answer only
 *        for its own MAC AND nonce.
 *
 * Authentication. Both tags are HMAC-SHA1 over the message (with the cell id
 * and a direction label), keyed by the cell's pre-shared key and truncated
 * to 8 bytes. The primitive is injected (`Mac`): this header links no crypto,
 * like src/sta (examples fill it from OpenSSL through sta::CryptoOps). The
 * DU verifies a msgA's tag and freshness BEFORE allocating a UE id: an
 * unauthenticated or stale (replayed) request gets nothing.
 *
 * Contention (the UE side, RachClient):
 *   - a randomized opportunity: the UE transmits in a superframe drawn from
 *     its backoff window, at a random offset inside the RACH slot;
 *   - collision detection: no msgB for its nonce within `response_sf`
 *     superframes = the attempt was lost (collided or faded);
 *   - resolution: the same nonce is retried (so a DU that did hear it answers
 *     idempotently), with the backoff window doubling up to `max_window_sf`;
 *   - after `max_attempts` the client gives up (a defined failure state).
 * DU side (RachServer): duplicate requests (same nonce) get the same answer;
 * the same MAC re-joining keeps its UE id (UeRegistry); a full pool answers
 * "denied: full", which ends the client's attempt (no retry storm).
 *
 * Pure header; cell_m3 ctest covers formats, auth/freshness rejects, the
 * client state machine and a multi-UE contention simulation. */
#ifndef DEVOURER_CELL_RACH_H
#define DEVOURER_CELL_RACH_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <algorithm>
#include <iterator>
#include <map>
#include <optional>
#include <utility>
#include <vector>

#include "cell/GrantMap.h"
#include "cell/UeRegistry.h"

namespace devourer::cell {

/* Mac: cell/GrantMap.h (HMAC-SHA1 keyed by the cell PSK). */

constexpr size_t kRachTagLen = 8;
/* msgA envelope version: an unknown version is refused before anything else
 * (a joiner built against another envelope revision is never half-parsed). */
constexpr uint8_t kRachProtoVersion = 1;
/* Departure budget for a msgA inside the RACH slot (M0 on the 8852CE: a
 * send_packet's p99.9 departure is ~0.46 ms on a clean channel). The rare
 * multi-ms tail beyond it (max ~4.8 ms measured; one msgA aired 4.7 ms late
 * on the rig) is what SlotGrid.h's kRachTailGapUs keeps idle before the next
 * beacon, so a late msgA never lands on it. */
constexpr int64_t kRachDepartureGuardUs = 500;

enum class RachStatus : uint8_t { Granted = 0, DeniedFull = 1, DeniedBad = 2 };

struct RachRequest {
  uint16_t cell_id = 0;
  std::array<uint8_t, 6> mac{};
  uint64_t nonce = 0;
  uint8_t attempt = 1;
  uint32_t fresh_sf = 0; /* low 32 bits of the DU superframe it was sent in */
};

struct RachResponse {
  std::array<uint8_t, 6> mac{}; /* the recipient (the requesting station) */
  uint64_t nonce = 0;
  RachStatus status = RachStatus::Granted;
  uint16_t ue = 0;
  uint8_t dl_slot = 0, ul_slot = 0;
  bool operator==(const RachResponse &) const = default;
};

namespace rach_detail {
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
/* Constant-time tag compare (the tags come from untrusted input). */
inline bool tag_eq(const uint8_t *a, const uint8_t *b) {
  uint8_t d = 0;
  for (size_t i = 0; i < kRachTagLen; ++i)
    d |= static_cast<uint8_t>(a[i] ^ b[i]);
  return d == 0;
}
/* The authenticated bytes of a request: label 'A' + cell + body. */
inline std::vector<uint8_t> req_body(const RachRequest &r) {
  std::vector<uint8_t> b;
  put(b, r.cell_id, 2);
  b.insert(b.end(), r.mac.begin(), r.mac.end());
  put(b, r.nonce, 8);
  b.push_back(r.attempt);
  put(b, r.fresh_sf, 4);
  return b;
}
inline std::vector<uint8_t> rsp_body(const RachResponse &r) {
  std::vector<uint8_t> b(r.mac.begin(), r.mac.end());
  put(b, r.nonce, 8);
  b.push_back(static_cast<uint8_t>(r.status));
  put(b, r.ue, 2);
  b.push_back(r.dl_slot);
  b.push_back(r.ul_slot);
  return b;
}
inline bool tag(const Mac &mac, char label, uint16_t cell, const std::vector<uint8_t> &body,
                uint8_t out[kRachTagLen]) {
  std::vector<uint8_t> m = {static_cast<uint8_t>(label)};
  put(m, cell, 2);
  m.insert(m.end(), body.begin(), body.end());
  uint8_t full[20];
  if (!mac || !mac(m.data(), m.size(), full))
    return false;
  std::memcpy(out, full, kRachTagLen);
  return true;
}
constexpr size_t kReqLen = 2 + 6 + 8 + 1 + 4;
constexpr size_t kRspLen = 6 + 8 + 1 + 2 + 1 + 1;
}  // namespace rach_detail

/* msgA payload ('R' 'A' ver + body + tag; the tag covers the version).
 * Empty if the MAC fails. */
inline std::vector<uint8_t> encode_rach_request(const RachRequest &r, const Mac &mac) {
  auto body = rach_detail::req_body(r);
  body.insert(body.begin(), kRachProtoVersion);
  uint8_t t[kRachTagLen];
  if (!rach_detail::tag(mac, 'A', r.cell_id, body, t))
    return {};
  std::vector<uint8_t> o = {'R', 'A'};
  o.insert(o.end(), body.begin(), body.end());
  o.insert(o.end(), t, t + kRachTagLen);
  return o;
}

/* Decode + authenticate a msgA payload for `cell`. nullopt for anything
 * malformed, of another envelope version, of another cell, or whose tag does
 * not verify. Freshness is the server's check (it knows the current
 * superframe). */
inline std::optional<RachRequest> decode_rach_request(const uint8_t *p, size_t len, uint16_t cell,
                                                      const Mac &mac) {
  using namespace rach_detail;
  if (p == nullptr || len != 3 + kReqLen + kRachTagLen || p[0] != 'R' || p[1] != 'A' ||
      p[2] != kRachProtoVersion)
    return std::nullopt;
  RachRequest r;
  const uint8_t *b = p + 3;
  r.cell_id = static_cast<uint16_t>(get(b, 2));
  if (r.cell_id != cell)
    return std::nullopt;
  std::memcpy(r.mac.data(), b + 2, 6);
  r.nonce = get(b + 8, 8);
  r.attempt = b[16];
  r.fresh_sf = static_cast<uint32_t>(get(b + 17, 4));
  uint8_t t[kRachTagLen];
  if (!tag(mac, 'A', cell, std::vector<uint8_t>(p + 2, b + kReqLen), t) || !tag_eq(t, b + kReqLen))
    return std::nullopt;
  return r;
}

constexpr size_t kRachMaxPerBeacon = 8; /* 3 + 8 x 27 = 219 bytes: one IE */

/* msgB: the random-access response IE body for the DU beacon:
 * 'R' 'B' n(1) n x {body + tag}, at most kRachMaxPerBeacon entries (the
 * server pages longer queues across beacons). Entries whose tag cannot be
 * computed are left out. */
inline std::vector<uint8_t> encode_rach_responses(uint16_t cell,
                                                  const std::vector<RachResponse> &rs,
                                                  const Mac &mac) {
  std::vector<uint8_t> o = {'R', 'B', 0};
  uint8_t n = 0;
  for (const RachResponse &r : rs) {
    if (n == kRachMaxPerBeacon)
      break;
    const auto body = rach_detail::rsp_body(r);
    uint8_t t[kRachTagLen];
    if (!rach_detail::tag(mac, 'B', cell, body, t))
      continue;
    o.insert(o.end(), body.begin(), body.end());
    o.insert(o.end(), t, t + kRachTagLen);
    ++n;
  }
  o[2] = n;
  return o;
}

/* Decode a msgB body; only entries whose tag verifies are returned. */
inline std::vector<RachResponse> decode_rach_responses(const uint8_t *p, size_t len, uint16_t cell,
                                                       const Mac &mac) {
  using namespace rach_detail;
  std::vector<RachResponse> out;
  if (p == nullptr || len < 3 || p[0] != 'R' || p[1] != 'B')
    return out;
  const size_t n = p[2];
  const size_t each = kRspLen + kRachTagLen;
  if (len != 3 + n * each)
    return out;
  for (size_t i = 0; i < n; ++i) {
    const uint8_t *b = p + 3 + i * each;
    uint8_t t[kRachTagLen];
    if (!tag(mac, 'B', cell, std::vector<uint8_t>(b, b + kRspLen), t) || !tag_eq(t, b + kRspLen))
      continue;
    RachResponse r;
    std::memcpy(r.mac.data(), b, 6);
    r.nonce = get(b + 6, 8);
    if (b[14] > static_cast<uint8_t>(RachStatus::DeniedBad))
      continue;
    r.status = static_cast<RachStatus>(b[14]);
    r.ue = static_cast<uint16_t>(get(b + 15, 2));
    r.dl_slot = b[17];
    r.ul_slot = b[18];
    out.push_back(r);
  }
  return out;
}

/* --- DU side ------------------------------------------------------------ */
struct RachServerConfig {
  uint32_t fresh_window_sf = 20;  /* a request older/newer than this is refused */
  uint32_t answer_beacons = 4;    /* beacons each answer is published in */
  uint32_t answer_max_age_sf = 40; /* an unpublished answer is dropped after this */
  size_t history_max = 256;       /* answered requests remembered for duplicates */
};

class RachServer {
public:
  RachServer(uint16_t cell, UeRegistry &reg, Mac mac, RachServerConfig c = {})
      : cell_(cell), reg_(reg), mac_(std::move(mac)), cfg_(c) {}

  /* A received msgA payload at superframe `sf`. Returns the answer it
   * produced (also queued for the beacon), or nullopt when the request is
   * ignored (malformed, other cell, bad tag, stale): ignored requests
   * allocate nothing and get no answer. */
  std::optional<RachResponse> on_request(const uint8_t *p, size_t len, uint64_t sf) {
    auto r = decode_rach_request(p, len, cell_, mac_);
    if (!r) {
      ++rejected_;
      return std::nullopt;
    }
    const int64_t age = static_cast<int32_t>(static_cast<uint32_t>(sf) - r->fresh_sf);
    if (age < -static_cast<int64_t>(cfg_.fresh_window_sf) ||
        age > static_cast<int64_t>(cfg_.fresh_window_sf)) {
      ++stale_;
      return std::nullopt;
    }
    expire(sf);
    /* Duplicate (a retry of an answered request — same MAC AND nonce): the
     * same answer again, provided a grant still matches the registry (the
     * position may have been released and reused since). */
    const Key k{r->mac, r->nonce};
    auto it = answered_.find(k);
    if (it != answered_.end()) {
      const RachResponse &old = it->second.r;
      const auto now = reg_.find(r->mac);
      const bool still = old.status != RachStatus::Granted ||
                         (now && now->ue == old.ue && now->dl_slot == old.dl_slot &&
                          now->ul_slot == old.ul_slot);
      if (still) {
        ++dups_;
        queue(old, sf);
        return old;
      }
      answered_.erase(it); /* stale grant: admit afresh below */
    }
    RachResponse a;
    a.mac = r->mac;
    a.nonce = r->nonce;
    const auto slot = reg_.admit(r->mac);
    if (!slot) {
      a.status = RachStatus::DeniedFull;
    } else {
      a.status = RachStatus::Granted;
      a.ue = slot->ue;
      a.dl_slot = slot->dl_slot;
      a.ul_slot = slot->ul_slot;
    }
    if (answered_.size() >= cfg_.history_max)
      answered_.erase(oldest_answered());
    answered_[k] = Answered{a, sf};
    queue(a, sf);
    return a;
  }

  /* The answers for superframe `sf`'s beacon: at most kRachMaxPerBeacon,
   * the least-published first, so a queue longer than one beacon is paged
   * across beacons and every answer gets `answer_beacons` publications. An
   * answer still unpublished `answer_max_age_sf` after it was made is
   * dropped (its client has long retried). */
  std::vector<RachResponse> pending(uint64_t sf) {
    auto out = select(sf);
    published(out);
    return out;
  }
  /* The two halves of pending(), for a caller whose beacon update can fail:
   * select() picks the answers for the next beacon WITHOUT counting them;
   * published() counts a set of answers as having aired once — call it per
   * superframe with what is actually on air (after a failed update, the
   * previous set still is), so a failed update never consumes an answer. */
  std::vector<RachResponse> select(uint64_t sf) {
    expire(sf);
    std::vector<const Live *> v;
    for (const auto &kv : live_)
      v.push_back(&kv.second);
    std::stable_sort(v.begin(), v.end(), [](const Live *a, const Live *b) {
      return a->pubs != b->pubs ? a->pubs < b->pubs : a->made_sf < b->made_sf;
    });
    std::vector<RachResponse> out;
    for (const Live *l : v) {
      if (out.size() == kRachMaxPerBeacon)
        break;
      out.push_back(l->r);
    }
    return out;
  }
  void published(const std::vector<RachResponse> &aired) {
    for (const RachResponse &r : aired) {
      auto it = live_.find(Key{r.mac, r.nonce});
      if (it != live_.end() && ++it->second.pubs >= cfg_.answer_beacons)
        live_.erase(it);
    }
  }
  size_t history() const { return answered_.size(); }
  uint64_t rejected() const { return rejected_; }
  uint64_t stale() const { return stale_; }
  uint64_t duplicates() const { return dups_; }

private:
  using Key = std::pair<std::array<uint8_t, 6>, uint64_t>; /* (MAC, nonce) */
  struct Answered {
    RachResponse r;
    uint64_t sf = 0;
  };
  struct Live {
    RachResponse r;
    uint64_t made_sf = 0;
    uint32_t pubs = 0;
  };
  void queue(const RachResponse &a, uint64_t sf) {
    live_[Key{a.mac, a.nonce}] = Live{a, sf, 0};
  }
  /* Forget what no retry can still refer to: requests older than the
   * freshness window plus the answer lifetime. */
  void expire(uint64_t sf) {
    const uint64_t keep = cfg_.fresh_window_sf + cfg_.answer_max_age_sf;
    for (auto it = answered_.begin(); it != answered_.end();)
      it = sf > it->second.sf + keep ? answered_.erase(it) : std::next(it);
    for (auto it = live_.begin(); it != live_.end();)
      it = sf > it->second.made_sf + cfg_.answer_max_age_sf ? live_.erase(it) : std::next(it);
  }
  std::map<Key, Answered>::iterator oldest_answered() {
    auto o = answered_.begin();
    for (auto it = answered_.begin(); it != answered_.end(); ++it)
      if (it->second.sf < o->second.sf)
        o = it;
    return o;
  }

  uint16_t cell_;
  UeRegistry &reg_;
  Mac mac_;
  RachServerConfig cfg_;
  std::map<Key, Answered> answered_;
  std::map<Key, Live> live_;
  uint64_t rejected_ = 0, stale_ = 0, dups_ = 0;
};

/* --- UE side ------------------------------------------------------------ */
struct RachClientConfig {
  uint32_t first_window_sf = 4;   /* initial backoff window (superframes) */
  uint32_t max_window_sf = 64;
  uint32_t response_sf = 4;       /* superframes to wait for msgB */
  uint8_t max_attempts = 10;
};

enum class RachState : uint8_t { Waiting = 0, Sent = 1, Joined = 2, Denied = 3, Failed = 4 };

class RachClient {
public:
  /* `mac` = this station; `rnd` returns uniform 32-bit randomness. */
  RachClient(std::array<uint8_t, 6> mac, uint64_t nonce, std::function<uint32_t()> rnd,
             RachClientConfig c = {})
      : mac_(mac), nonce_(nonce), rnd_(std::move(rnd)), cfg_(c), window_(c.first_window_sf) {
    schedule(0);
  }

  /* Should the UE transmit its msgA in superframe `sf`? If so, returns the
   * start offset inside the RACH slot, drawn so the whole transmission
   * (`tx_us`: msgA airtime + the radio's departure guard) ends inside the
   * slot, and moves to Sent. A slot too short to fit `tx_us` at all is a
   * configuration error: the client fails rather than transmit across the
   * slot edge. */
  std::optional<int64_t> opportunity(uint64_t sf, int64_t slot_len_us, int64_t tx_us) {
    if (state_ == RachState::Waiting && slot_len_us - tx_us <= 0) {
      state_ = RachState::Failed;
      return std::nullopt;
    }
    if (state_ == RachState::Sent && sf > sent_sf_ + cfg_.response_sf) {
      /* no answer: lost (collision or fade) -> back off and retry */
      ++losses_;
      if (attempt_ >= cfg_.max_attempts) {
        state_ = RachState::Failed;
        return std::nullopt;
      }
      window_ = window_ * 2 > cfg_.max_window_sf ? cfg_.max_window_sf : window_ * 2;
      state_ = RachState::Waiting;
      schedule(sf);
    }
    if (state_ != RachState::Waiting || sf < next_sf_)
      return std::nullopt;
    state_ = RachState::Sent;
    sent_sf_ = sf;
    ++attempt_;
    const uint32_t span = static_cast<uint32_t>(slot_len_us - tx_us) + 1;
    return static_cast<int64_t>(rnd_() % span); /* start + tx_us <= slot_len_us */
  }

  /* The msgA opportunity() granted was NOT transmitted (its deadline passed
   * before the radio could take it): no attempt spent; try again from the
   * next superframe. */
  void unsend(uint64_t sf) {
    if (state_ != RachState::Sent || attempt_ == 0)
      return;
    --attempt_;
    state_ = RachState::Waiting;
    next_sf_ = sf + 1;
  }

  /* Answers decoded from a DU beacon. Returns true when one was for us
   * (our MAC and nonce). Only an outstanding request accepts an answer: not
   * before the first transmission, and not after the client has failed or
   * already got one. */
  bool on_responses(const std::vector<RachResponse> &rs) {
    if (attempt_ == 0 || state_ == RachState::Joined || state_ == RachState::Denied ||
        state_ == RachState::Failed)
      return false;
    for (const RachResponse &r : rs) {
      if (r.nonce != nonce_ || r.mac != mac_)
        continue;
      answer_ = r;
      state_ = r.status == RachStatus::Granted ? RachState::Joined : RachState::Denied;
      return true;
    }
    return false;
  }

  RachState state() const { return state_; }
  uint64_t nonce() const { return nonce_; }
  uint8_t attempt() const { return static_cast<uint8_t>(attempt_); }
  const std::optional<RachResponse> &answer() const { return answer_; }
  uint32_t losses() const { return losses_; }

private:
  void schedule(uint64_t from) {
    next_sf_ = from + 1 + (window_ ? rnd_() % window_ : 0);
  }

  std::array<uint8_t, 6> mac_;
  uint64_t nonce_;
  std::function<uint32_t()> rnd_;
  RachClientConfig cfg_;
  uint32_t window_;
  RachState state_ = RachState::Waiting;
  uint64_t next_sf_ = 0, sent_sf_ = 0;
  uint32_t attempt_ = 0, losses_ = 0;
  std::optional<RachResponse> answer_;
};

} /* namespace devourer::cell */

#endif /* DEVOURER_CELL_RACH_H */
