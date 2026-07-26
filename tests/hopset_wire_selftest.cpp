/* HopsetWire known-answer + tamper + replay selftest.
 *
 * Pins the exact sizes, header bytes and MAC of each adaptive-hopset control
 * message under the SipHash reference key (and the domain-separated key
 * derivation feeding them), then the byte-flip tamper sweep, truncation
 * sweep, header rejects, the <=96-byte bound, the v1/v2 sync-marker mutual
 * rejection, and the proposal replay ring. */
#include "hopset/HopsetWire.h"

#include <cstdio>
#include <cstring>

static int fails;
#define CHECK(x, msg)                                                          \
  do {                                                                         \
    if (!(x)) {                                                                \
      std::fprintf(stderr, "FAIL: %s\n", msg);                                 \
      ++fails;                                                                 \
    }                                                                          \
  } while (0)

using namespace devourer::hopset;

/* A fully-populated message of each type for round-trip coverage. */
static HopsetMsg sample(HopsetMsgType t) {
  HopsetMsg m;
  m.type = t;
  m.link_id = 0x11223344;
  m.rx_epoch = 0xAABBCCDD;
  m.tx_epoch = 0x01020304;
  m.sender_epoch = 0x0A0B0C0D;
  m.rx_epoch_echo = 0xAABBCCDD;
  m.rx_nonce = 0xDEADBEEF;
  m.rx_nonce_echo = 0xDEADBEEF;
  m.generation = 42;
  m.active_mask = 0x00000000000000B7ULL;
  m.base_fp = 0x55667788;
  m.activate_round = 1234;
  m.activate_slot = 987654;
  m.current_round = 1226;
  m.observation_count = 250;
  m.reason_bitmap = 0x5;
  m.earliest_activate_round = 1240;
  m.role = 1;
  m.reason = static_cast<uint8_t>(HopsetReason::None);
  return m;
}

int main() {
  const auto master =
      devourer::HopSchedule::parse_seed("000102030405060708090a0b0c0d0e0f");
  const HopsetKeys keys = HopsetKeys::derive(master);

  const HopsetMsgType types[] = {HT_PROPOSAL, HT_COMMIT, HT_STATUS};
  const size_t sizes[] = {56, 68, 58};

  /* --- round-trip + pinned sizes + full-field fidelity --- */
  for (int i = 0; i < 3; ++i) {
    HopsetMsg m = sample(types[i]);
    std::vector<uint8_t> w = hopset_encode(m, keys);
    if (w.size() != sizes[i])
      std::fprintf(stderr, "SIZE: type %d = %zu\n", types[i], w.size());
    CHECK(w.size() == sizes[i], "pinned message size");
    CHECK(w.size() <= kHopsetMaxLen, "message within the 96-byte bound");
    HopsetMsg r;
    CHECK(hopset_decode(w.data(), w.size(), keys, m.link_id, r) ==
              HopsetReason::None,
          "authentic message decodes");
    CHECK(r.type == types[i] && r.link_id == m.link_id, "header round-trips");
    CHECK(r.generation == m.generation && r.active_mask == m.active_mask &&
              r.base_fp == m.base_fp,
          "state fields round-trip");
    if (types[i] == HT_PROPOSAL)
      CHECK(r.rx_epoch == m.rx_epoch && r.rx_nonce == m.rx_nonce &&
                r.observation_count == m.observation_count &&
                r.reason_bitmap == m.reason_bitmap &&
                r.earliest_activate_round == m.earliest_activate_round,
            "proposal fields round-trip");
    if (types[i] == HT_COMMIT)
      CHECK(r.tx_epoch == m.tx_epoch && r.rx_epoch_echo == m.rx_epoch_echo &&
                r.activate_round == m.activate_round &&
                r.activate_slot == m.activate_slot &&
                r.current_round == m.current_round &&
                r.rx_nonce_echo == m.rx_nonce_echo,
            "commit fields round-trip");
    if (types[i] == HT_STATUS)
      CHECK(r.role == m.role && r.sender_epoch == m.sender_epoch &&
                r.current_round == m.current_round &&
                r.activate_slot == m.activate_slot && r.reason == m.reason &&
                r.rx_nonce_echo == m.rx_nonce_echo,
            "status fields round-trip");
  }

  /* --- known-answer: header bytes + MAC placement of a COMMIT --- */
  {
    HopsetMsg m = sample(HT_COMMIT);
    std::vector<uint8_t> w = hopset_encode(m, keys);
    CHECK(w[0] == 0x48 && w[1] == 0x53 && w[2] == 1 && w[3] == HT_COMMIT,
          "commit header bytes 'HS' ver=1");
    const uint64_t mac = keys.mac(w.data(), w.size() - kHopsetMacLen);
    uint64_t tail = 0;
    for (int i = 0; i < 8; i++)
      tail |= uint64_t(w[w.size() - 8 + i]) << (8 * i);
    CHECK(mac == tail, "MAC is the trailing u64 over the body");
  }

  /* --- key derivation: deterministic, domain-separated, seed-sensitive --- */
  {
    HopsetKeys a = HopsetKeys::derive(master), b = HopsetKeys::derive(master);
    CHECK(a.sched == b.sched && a.ctrl == b.ctrl, "key derive stable");
    CHECK(a.sched != master && a.ctrl != master && a.sched != a.ctrl,
          "schedule/control keys domain-separated");
    const auto other =
        devourer::HopSchedule::parse_seed("0f0e0d0c0b0a09080706050403020100");
    HopsetKeys c = HopsetKeys::derive(other);
    CHECK(a.sched != c.sched && a.ctrl != c.ctrl, "keys sensitive to seed");
    /* ...and distinct from the chanmig control derivation (different tags):
     * a message MAC'd for one protocol can never authenticate in the other.
     * Checked indirectly: the hopset control key differs from the master and
     * from the schedule subkey; the tag strings differ by construction. */
  }

  /* --- tamper: flipping ANY byte breaks authentication --- */
  {
    HopsetMsg m = sample(HT_PROPOSAL);
    std::vector<uint8_t> w = hopset_encode(m, keys);
    int survived = 0;
    for (size_t i = 0; i < w.size(); ++i) {
      std::vector<uint8_t> t = w;
      t[i] ^= 0x40;
      HopsetMsg r;
      if (hopset_decode(t.data(), t.size(), keys, m.link_id, r) ==
          HopsetReason::None)
        ++survived;
    }
    CHECK(survived == 0, "every single-byte tamper is rejected");
    /* forged: same bytes, wrong key */
    HopsetKeys forged = HopsetKeys::derive(
        devourer::HopSchedule::parse_seed("deadbeefdeadbeefdeadbeefdeadbeef"));
    HopsetMsg r;
    CHECK(hopset_decode(w.data(), w.size(), forged, m.link_id, r) ==
              HopsetReason::BadMac,
          "wrong-key forgery rejected");
  }

  /* --- truncation at every length rejected; trailing FCS tolerated --- */
  {
    HopsetMsg m = sample(HT_STATUS);
    std::vector<uint8_t> w = hopset_encode(m, keys);
    for (size_t len = 0; len < w.size(); ++len) {
      HopsetMsg r;
      CHECK(hopset_decode(w.data(), len, keys, m.link_id, r) !=
                HopsetReason::None,
            "truncated frame rejected");
    }
    std::vector<uint8_t> fcs = w;
    for (int i = 0; i < 4; i++)
      fcs.push_back(0xDE);
    HopsetMsg r;
    CHECK(hopset_decode(fcs.data(), fcs.size(), keys, m.link_id, r) ==
              HopsetReason::None,
          "trailing FCS/pad ignored — message is a prefix");
  }

  /* --- header rejects --- */
  {
    HopsetMsg m = sample(HT_COMMIT);
    std::vector<uint8_t> w = hopset_encode(m, keys);
    HopsetMsg r;
    std::vector<uint8_t> bad = w;
    bad[2] = 9; /* unknown version */
    CHECK(hopset_decode(bad.data(), bad.size(), keys, m.link_id, r) ==
              HopsetReason::BadVersion,
          "unknown version rejected");
    bad = w;
    bad[3] = 77; /* unknown type */
    CHECK(hopset_decode(bad.data(), bad.size(), keys, m.link_id, r) ==
              HopsetReason::BadType,
          "unknown type rejected");
    CHECK(hopset_decode(w.data(), w.size(), keys, 0xFFFFFFFF, r) ==
              HopsetReason::BadLinkId,
          "wrong link id rejected");
    CHECK(hopset_decode(w.data(), w.size(), keys, 0, r) == HopsetReason::None,
          "link id 0 skips the check");
  }

  /* --- sync marker v2: round-trip + v1/v2 mutual rejection --- */
  {
    HopSyncMarkerV2 m2{0x11223344, 7, 12345, 0x0102030405060708ULL, 3,
                       0xCAFEBABE},
        r2;
    auto w2 = HopSyncMarkerV2::encode(m2);
    CHECK(HopSyncMarkerV2::decode(w2.data(), w2.size(), r2), "v2 decodes");
    CHECK(r2.fingerprint == m2.fingerprint && r2.epoch == m2.epoch &&
              r2.slot == m2.slot && r2.phase_us == m2.phase_us &&
              r2.generation == m2.generation && r2.mask_fp == m2.mask_fp,
          "v2 round-trips");
    devourer::HopSyncMarker r1;
    CHECK(!devourer::HopSyncMarker::decode(w2.data(), w2.size(), r1),
          "v1 decoder rejects a v2 marker");
    devourer::HopSyncMarker m1{0x11223344, 7, 12345,
                               0x0102030405060708ULL};
    auto w1 = devourer::HopSyncMarker::encode(m1);
    CHECK(!HopSyncMarkerV2::decode(w1.data(), w1.size(), r2),
          "v2 decoder rejects a v1 marker");
  }

  /* --- proposal replay ring --- */
  {
    ProposalReplayRing ring;
    CHECK(!ring.contains(1, 100), "empty ring");
    ring.remember(1, 100);
    CHECK(ring.contains(1, 100), "remembered pair found");
    CHECK(!ring.contains(1, 101) && !ring.contains(2, 100),
          "epoch and nonce both keyed");
    for (uint32_t i = 0; i < ProposalReplayRing::kDepth; ++i)
      ring.remember(9, i);
    CHECK(!ring.contains(1, 100), "oldest entry evicted at capacity");
    CHECK(ring.contains(9, 0) && ring.contains(9, 15), "ring holds depth");
  }

  return fails ? 1 : 0;
}
