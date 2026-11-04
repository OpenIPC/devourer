/* Rach.h — M3 (#257) admission: a 2-step random-access join into a running
 * cell, authenticated before anything persistent is allocated.
 *
 *   msgA (UE -> DU, in the grid's RACH slot): the joining UE's MAC, a 64-bit
 *        random nonce (its temporary identity until it holds a UE id), the
 *        attempt number, a freshness stamp (the DU superframe it was sent in)
 *        and an authentication tag.
 *   msgB (DU -> UE, the random-access response in the DU's next beacon(s)):
 *        per answered nonce, a status (granted / denied: pool full / denied:
 *        bad request), the UE id and its DL/UL slot indices, and a tag.
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
#include <map>
#include <optional>
#include <vector>

#include "cell/GrantMap.h"
#include "cell/UeRegistry.h"

namespace devourer::cell {

/* HMAC-SHA1 keyed by the cell PSK: data -> 20-byte tag. False on failure. */
using Mac = std::function<bool(const uint8_t *data, size_t len, uint8_t out[20])>;

constexpr size_t kRachTagLen = 8;

enum class RachStatus : uint8_t { Granted = 0, DeniedFull = 1, DeniedBad = 2 };

struct RachRequest {
  uint16_t cell_id = 0;
  std::array<uint8_t, 6> mac{};
  uint64_t nonce = 0;
  uint8_t attempt = 1;
  uint32_t fresh_sf = 0; /* low 32 bits of the DU superframe it was sent in */
};

struct RachResponse {
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
  std::vector<uint8_t> b;
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
constexpr size_t kRspLen = 8 + 1 + 2 + 1 + 1;
}  // namespace rach_detail

/* msgA payload ('R' 'A' + body + tag). Empty if the MAC fails. */
inline std::vector<uint8_t> encode_rach_request(const RachRequest &r, const Mac &mac) {
  const auto body = rach_detail::req_body(r);
  uint8_t t[kRachTagLen];
  if (!rach_detail::tag(mac, 'A', r.cell_id, body, t))
    return {};
  std::vector<uint8_t> o = {'R', 'A'};
  o.insert(o.end(), body.begin(), body.end());
  o.insert(o.end(), t, t + kRachTagLen);
  return o;
}

/* Decode + authenticate a msgA payload for `cell`. nullopt for anything
 * malformed, of another cell, or whose tag does not verify. Freshness is the
 * server's check (it knows the current superframe). */
inline std::optional<RachRequest> decode_rach_request(const uint8_t *p, size_t len, uint16_t cell,
                                                      const Mac &mac) {
  using namespace rach_detail;
  if (p == nullptr || len != 2 + kReqLen + kRachTagLen || p[0] != 'R' || p[1] != 'A')
    return std::nullopt;
  RachRequest r;
  const uint8_t *b = p + 2;
  r.cell_id = static_cast<uint16_t>(get(b, 2));
  if (r.cell_id != cell)
    return std::nullopt;
  std::memcpy(r.mac.data(), b + 2, 6);
  r.nonce = get(b + 8, 8);
  r.attempt = b[16];
  r.fresh_sf = static_cast<uint32_t>(get(b + 17, 4));
  uint8_t t[kRachTagLen];
  if (!tag(mac, 'A', cell, std::vector<uint8_t>(b, b + kReqLen), t) || !tag_eq(t, b + kReqLen))
    return std::nullopt;
  return r;
}

/* msgB: the random-access response IE body for the DU beacon:
 * 'R' 'B' n(1) n x {body + tag}. Entries whose tag cannot be computed are
 * left out. */
inline std::vector<uint8_t> encode_rach_responses(uint16_t cell,
                                                  const std::vector<RachResponse> &rs,
                                                  const Mac &mac) {
  std::vector<uint8_t> o = {'R', 'B', 0};
  uint8_t n = 0;
  for (const RachResponse &r : rs) {
    if (n == 8)
      break; /* keeps the IE well under 255 bytes */
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
    r.nonce = get(b, 8);
    r.status = static_cast<RachStatus>(b[8]);
    r.ue = static_cast<uint16_t>(get(b + 9, 2));
    r.dl_slot = b[11];
    r.ul_slot = b[12];
    if (b[8] > static_cast<uint8_t>(RachStatus::DeniedBad))
      continue;
    out.push_back(r);
  }
  return out;
}

/* --- DU side ------------------------------------------------------------ */
struct RachServerConfig {
  uint32_t fresh_window_sf = 20;  /* a request older/newer than this is refused */
  uint32_t answer_ttl_sf = 4;     /* superframes an answer stays in the beacon */
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
    /* Duplicate (a retry of an already-answered nonce): same answer again. */
    auto it = answered_.find(r->nonce);
    if (it != answered_.end()) {
      ++dups_;
      queue(it->second, sf);
      return it->second;
    }
    RachResponse a;
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
    answered_[r->nonce] = a;
    queue(a, sf);
    return a;
  }

  /* The answers to put in superframe `sf`'s beacon (expired ones dropped). */
  std::vector<RachResponse> pending(uint64_t sf) {
    std::vector<RachResponse> out;
    for (auto it = live_.begin(); it != live_.end();) {
      if (sf > it->second.until) {
        it = live_.erase(it);
        continue;
      }
      out.push_back(it->second.r);
      ++it;
    }
    return out;
  }
  uint64_t rejected() const { return rejected_; }
  uint64_t stale() const { return stale_; }
  uint64_t duplicates() const { return dups_; }

private:
  struct Live {
    RachResponse r;
    uint64_t until = 0;
  };
  void queue(const RachResponse &a, uint64_t sf) { live_[a.nonce] = Live{a, sf + cfg_.answer_ttl_sf}; }

  uint16_t cell_;
  UeRegistry &reg_;
  Mac mac_;
  RachServerConfig cfg_;
  std::map<uint64_t, RachResponse> answered_;
  std::map<uint64_t, Live> live_;
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
  /* `rnd` returns uniform 32-bit randomness (the caller's RNG). */
  RachClient(uint64_t nonce, std::function<uint32_t()> rnd, RachClientConfig c = {})
      : nonce_(nonce), rnd_(std::move(rnd)), cfg_(c), window_(c.first_window_sf) {
    schedule(0);
  }

  /* Should the UE transmit its msgA in superframe `sf`? If so, returns the
   * offset (0..slot_len_us-1) inside the RACH slot and moves to Sent. */
  std::optional<int64_t> opportunity(uint64_t sf, int64_t slot_len_us) {
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
    return slot_len_us > 0 ? static_cast<int64_t>(rnd_() % static_cast<uint32_t>(slot_len_us)) : 0;
  }

  /* Answers decoded from a DU beacon. Returns true when one was for us. */
  bool on_responses(const std::vector<RachResponse> &rs) {
    if (state_ == RachState::Joined || state_ == RachState::Denied)
      return false;
    for (const RachResponse &r : rs) {
      if (r.nonce != nonce_)
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
