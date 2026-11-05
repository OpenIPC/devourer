/* Headless guard for the M3 cell core: src/cell/UeRegistry.h (fixed-position
 * pool, idempotent admission, release) and src/cell/Rach.h (msgA / msgB
 * formats, authentication and freshness rejects, duplicate handling, pool
 * exhaustion, the client's backoff / collision / give-up rules), plus a
 * contention simulation: several UEs join a running cell through one RACH
 * slot with collisions, every grant is unique and the pre-admitted UE's slot
 * never changes. The MAC here is a keyed stand-in (the protocol logic does
 * not depend on the primitive; examples fill Mac from OpenSSL HMAC-SHA1). */
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <set>
#include <vector>

#include "cell/Rach.h"
#include "cell/SlotGrid.h"
#include "cell/UeRegistry.h"

using namespace devourer::cell;

static int failures = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      failures++;                                                              \
    }                                                                          \
  } while (0)

/* A keyed stand-in for HMAC-SHA1 (NOT cryptographic): different keys give
 * different tags, any byte change changes the tag. */
static Mac fake_mac(uint64_t key) {
  return [key](const uint8_t *d, size_t n, uint8_t out[20]) {
    uint64_t h = 1469598103934665603ull ^ key;
    for (size_t i = 0; i < n; ++i)
      h = (h ^ d[i]) * 1099511628211ull;
    for (int i = 0; i < 20; ++i) {
      h = (h ^ static_cast<uint64_t>(i)) * 1099511628211ull;
      out[i] = static_cast<uint8_t>(h >> 32);
    }
    return true;
  };
}

static std::array<uint8_t, 6> mac_of(uint8_t n) { return {0x02, 0xaa, 0, 0, 0, n}; }

int main() {
  /* --- Grid: the optional RACH slot sits after the UL slots. --- */
  {
    auto g = build_fixed_grid(100, 0, 1500, {1, 2, 3}, 4000, 1000, 2000, 8000);
    CHECK(g.has_value() && g->slots().back().kind == SlotKind::Rach &&
          g->slots().back().len_us == 8000);
    CHECK(g->find(SlotKind::Rach, 0).has_value());
    auto h = build_fixed_grid(100, 0, 1500, {1, 2}, 4000, 1000, 2000);
    CHECK(h && !h->find(SlotKind::Rach, 0).has_value());
    /* A RACH slot that leaves less than kRachTailGapUs before the next beacon
     * is refused: 1500 + 4x4000 + 1000 + 4x2000 = 26500 used, so a RACH slot
     * of 102400 - 26500 - 5000 fits exactly and one microsecond more does not. */
    CHECK(build_fixed_grid(100, 0, 1500, {1, 2, 3, 4}, 4000, 1000, 2000, 102400 - 26500 - kRachTailGapUs));
    CHECK(!build_fixed_grid(100, 0, 1500, {1, 2, 3, 4}, 4000, 1000, 2000, 102400 - 26500 - kRachTailGapUs + 1));
  }

  /* --- Registry: positions never move; admission is idempotent. --- */
  {
    UeRegistry r({{1, 5}, {2, 6}, {3, 7}});
    CHECK(r.capacity() == 3 && r.size() == 0);
    auto a = r.admit(mac_of(1));
    auto b = r.admit(mac_of(2));
    CHECK(a && a->ue == 1 && a->dl_slot == 1 && a->ul_slot == 5);
    CHECK(b && b->ue == 2 && b->ul_slot == 6);
    auto a2 = r.admit(mac_of(1)); /* re-join keeps the id */
    CHECK(a2 && a2->ue == 1 && r.size() == 2);
    CHECK(r.admit(mac_of(3)) && !r.admit(mac_of(4))); /* full */
    CHECK(r.release(mac_of(2)) && !r.release(mac_of(2)));
    auto d = r.admit(mac_of(4)); /* the freed position, same slots as before */
    CHECK(d && d->ue == 2 && d->dl_slot == 2 && d->ul_slot == 6);
    CHECK(r.admitted().size() == 3 && r.find(mac_of(1))->ue == 1);
    CHECK(r.release_id(1) && !r.find(mac_of(1)) && !r.release_id(1) && !r.release_id(9));
    CHECK(r.admit(mac_of(5))->ue == 1); /* the released position, reused */
  }

  const Mac key = fake_mac(0x1234), other = fake_mac(0x9999);

  /* --- msgA: round trip, and every reject. --- */
  {
    RachRequest q;
    q.cell_id = 7;
    q.mac = mac_of(9);
    q.nonce = 0x0102030405060708ull;
    q.attempt = 3;
    q.fresh_sf = 1000;
    const auto p = encode_rach_request(q, key);
    CHECK(p.size() == 3 + 21 + kRachTagLen && p[2] == kRachProtoVersion);
    auto d = decode_rach_request(p.data(), p.size(), 7, key);
    CHECK(d && d->nonce == q.nonce && d->attempt == 3 && d->fresh_sf == 1000 && d->mac == q.mac);
    CHECK(!decode_rach_request(p.data(), p.size(), 8, key));   /* other cell */
    CHECK(!decode_rach_request(p.data(), p.size(), 7, other)); /* wrong key */
    auto bad = p;
    bad[10] ^= 1; /* any body byte: the tag no longer verifies */
    CHECK(!decode_rach_request(bad.data(), bad.size(), 7, key));
    CHECK(!decode_rach_request(p.data(), p.size() - 1, 7, key));
    CHECK(encode_rach_request(q, Mac{}).empty()); /* no primitive: no message */
    auto v = p;
    v[2] = kRachProtoVersion + 1; /* another envelope revision: refused outright */
    CHECK(!decode_rach_request(v.data(), v.size(), 7, key));
  }

  /* --- msgB: round trip; forged/foreign entries are dropped one by one. --- */
  {
    std::vector<RachResponse> rs = {{mac_of(1), 11, RachStatus::Granted, 2, 2, 6},
                                    {mac_of(2), 12, RachStatus::DeniedFull, 0, 0, 0}};
    const auto b = encode_rach_responses(7, rs, key);
    CHECK(decode_rach_responses(b.data(), b.size(), 7, key) == rs);
    CHECK(decode_rach_responses(b.data(), b.size(), 7, other).empty());
    CHECK(decode_rach_responses(b.data(), b.size(), 8, key).empty());
    auto t = b;
    t[3 + 15] ^= 1; /* the first entry's ue id: only that entry is dropped */
    auto d = decode_rach_responses(t.data(), t.size(), 7, key);
    CHECK(d.size() == 1 && d[0].nonce == 12);
    std::vector<RachResponse> many(12, RachResponse{mac_of(1), 1, RachStatus::Granted, 1, 1, 1});
    const auto capped = encode_rach_responses(7, many, key);
    CHECK(capped[2] == kRachMaxPerBeacon && capped.size() + 3 <= 255); /* one IE */
  }

  /* --- Server: authenticate + freshness before allocating; duplicates;
   *     a full pool denies. --- */
  {
    UeRegistry reg({{1, 5}, {2, 6}});
    RachServer srv(7, reg, key);
    auto req = [&](uint8_t m, uint64_t nonce, uint32_t fresh, const Mac &k) {
      RachRequest q;
      q.cell_id = 7;
      q.mac = mac_of(m);
      q.nonce = nonce;
      q.fresh_sf = fresh;
      return encode_rach_request(q, k);
    };
    auto forged = req(1, 100, 500, other);
    CHECK(!srv.on_request(forged.data(), forged.size(), 500) && reg.size() == 0);
    auto stale = req(1, 101, 400, key); /* 100 superframes old: a replay */
    CHECK(!srv.on_request(stale.data(), stale.size(), 500) && reg.size() == 0);
    CHECK(srv.rejected() == 1 && srv.stale() == 1);
    auto ok1 = req(1, 102, 498, key);
    auto a1 = srv.on_request(ok1.data(), ok1.size(), 500);
    CHECK(a1 && a1->status == RachStatus::Granted && a1->ue == 1 && reg.size() == 1);
    auto again = srv.on_request(ok1.data(), ok1.size(), 501); /* a retry */
    CHECK(again && *again == *a1 && srv.duplicates() == 1 && reg.size() == 1);
    auto ok2 = req(2, 103, 500, key);
    auto ok3 = req(3, 104, 500, key);
    CHECK(srv.on_request(ok2.data(), ok2.size(), 500)->ue == 2);
    auto full = srv.on_request(ok3.data(), ok3.size(), 500);
    CHECK(full && full->status == RachStatus::DeniedFull && reg.size() == 2);
    /* Each answer is published in answer_beacons (4) beacons, then dropped. */
    for (int i = 0; i < 4; ++i)
      CHECK(srv.pending(502 + i).size() == 3);
    CHECK(srv.pending(506).empty());
    /* A grant whose position was released and reused is not resurrected by
     * a retry of the old request: the station is admitted afresh. */
    CHECK(reg.release(mac_of(1)));
    auto take = req(5, 105, 506, key);
    CHECK(srv.on_request(take.data(), take.size(), 506)->ue == 1); /* reuses position 1 */
    auto retry = srv.on_request(ok1.data(), ok1.size(), 507);   /* MAC 1's old nonce */
    CHECK(retry && retry->status == RachStatus::DeniedFull);     /* pool full now, not ue 1 */
    /* Same nonce from another MAC is another request, answered to that MAC. */
    auto other_mac = req(9, 102, 507, key);
    auto om = srv.on_request(other_mac.data(), other_mac.size(), 507);
    CHECK(om && om->mac == mac_of(9) && om->status == RachStatus::DeniedFull);
  }

  /* --- Server: a queue longer than one beacon is paged; history is bounded. --- */
  {
    std::vector<std::pair<uint8_t, uint8_t>> pos;
    for (uint8_t i = 0; i < 20; ++i)
      pos.push_back({i, static_cast<uint8_t>(20 + i)});
    UeRegistry reg(pos);
    RachServerConfig sc;
    sc.answer_beacons = 1;
    sc.history_max = 16;
    RachServer srv(7, reg, key, sc);
    for (uint8_t i = 0; i < 12; ++i) {
      RachRequest q;
      q.cell_id = 7;
      q.mac = mac_of(static_cast<uint8_t>(40 + i));
      q.nonce = 500 + i;
      q.fresh_sf = 100;
      const auto p = encode_rach_request(q, key);
      CHECK(srv.on_request(p.data(), p.size(), 100).has_value());
    }
    const auto b1 = srv.pending(101), b2 = srv.pending(102);
    CHECK(b1.size() == kRachMaxPerBeacon && b2.size() == 12 - kRachMaxPerBeacon);
    std::set<uint64_t> seen;
    for (const auto &r : b1) seen.insert(r.nonce);
    for (const auto &r : b2) seen.insert(r.nonce);
    CHECK(seen.size() == 12); /* every answer published once */
    CHECK(srv.pending(103).empty());
    CHECK(srv.history() == 12);
    for (uint8_t i = 0; i < 8; ++i) { /* past history_max: the oldest go */
      RachRequest q;
      q.cell_id = 7;
      q.mac = mac_of(static_cast<uint8_t>(80 + i));
      q.nonce = 900 + i;
      q.fresh_sf = 104;
      const auto p = encode_rach_request(q, key);
      srv.on_request(p.data(), p.size(), 104);
    }
    CHECK(srv.history() <= 16);
    srv.pending(1000); /* far later: everything aged out */
    RachRequest q;
    q.cell_id = 7;
    q.mac = mac_of(99);
    q.nonce = 1;
    q.fresh_sf = 1000;
    const auto p = encode_rach_request(q, key);
    srv.on_request(p.data(), p.size(), 1000);
    CHECK(srv.history() == 1);
  }

  /* --- select() / published(): a beacon that never aired consumes nothing. --- */
  {
    UeRegistry reg({{1, 5}, {2, 6}});
    RachServerConfig sc;
    sc.answer_beacons = 2;
    RachServer srv(7, reg, key, sc);
    RachRequest q;
    q.cell_id = 7;
    q.mac = mac_of(1);
    q.nonce = 77;
    q.fresh_sf = 50;
    const auto p = encode_rach_request(q, key);
    CHECK(srv.on_request(p.data(), p.size(), 50).has_value());
    for (int i = 0; i < 5; ++i) /* five failed beacon updates: selected, never aired */
      CHECK(srv.select(51 + i).size() == 1);
    const auto sel = srv.select(56);
    srv.published(sel); /* aired once */
    CHECK(srv.select(57).size() == 1);
    srv.published(sel); /* aired twice: done */
    CHECK(srv.select(58).empty());
  }

  /* --- Client unsend(): a msgA that never left spends no attempt. --- */
  {
    uint32_t seed = 7;
    auto rnd = [&seed] { seed = seed * 1103515245u + 12345u; return seed >> 8; };
    RachClient cl(mac_of(3), 99, rnd);
    uint64_t sf = 0;
    while (!cl.opportunity(sf, 8000, 700))
      ++sf;
    CHECK(cl.attempt() == 1 && cl.state() == RachState::Sent);
    cl.unsend(sf);
    CHECK(cl.attempt() == 0 && cl.state() == RachState::Waiting);
    CHECK(!cl.opportunity(sf, 8000, 700));          /* not the same superframe again */
    CHECK(cl.opportunity(sf + 1, 8000, 700) && cl.attempt() == 1);
  }

  /* --- Registry is capped at what a grid / grant map can carry. --- */
  {
    std::vector<std::pair<uint8_t, uint8_t>> pos(kMaxUesPerCell + 5, {1, 2});
    UeRegistry big(pos);
    CHECK(big.capacity() == kMaxUesPerCell);
  }

  /* --- Client: backoff, loss detection, same nonce, give-up. --- */
  {
    uint32_t seed = 1;
    auto rnd = [&seed] { seed = seed * 1103515245u + 12345u; return seed >> 8; };
    RachClientConfig cc;
    cc.first_window_sf = 4;
    cc.max_window_sf = 16;
    cc.response_sf = 3;
    cc.max_attempts = 3;
    RachClient c(mac_of(7), 777, rnd, cc);
    uint64_t sf = 0;
    std::optional<int64_t> off;
    for (; sf < 100 && !(off = c.opportunity(sf, 8000, 700)); ++sf) {
    }
    CHECK(off && *off >= 0 && *off + 700 <= 8000 && c.state() == RachState::Sent && c.attempt() == 1);
    const uint64_t first = sf;
    CHECK(first >= 1 && first <= 4); /* inside the first window */
    /* No answer: after response_sf the client backs off and tries again. */
    int sent = 1;
    for (++sf; sf < 400 && c.state() != RachState::Failed; ++sf)
      if (c.opportunity(sf, 8000, 700))
        ++sent;
    CHECK(sent == 3 && c.state() == RachState::Failed && c.losses() == 3);
    /* A late answer cannot revive a failed client. */
    CHECK(!c.on_responses({{mac_of(7), 777, RachStatus::Granted, 1, 1, 5}}) &&
          c.state() == RachState::Failed);
    /* Offsets always leave room for the whole transmission, at the edge too. */
    for (int k = 0; k < 2000; ++k) {
      RachClient e(mac_of(8), 1000 + k, rnd, cc);
      std::optional<int64_t> o;
      for (uint64_t x = 0; x < 50 && !(o = e.opportunity(x, 1000, 999)); ++x) {
      }
      CHECK(o && (*o == 0 || *o == 1));
    }
    RachClient tight(mac_of(8), 5, rnd, cc);
    CHECK(!tight.opportunity(10, 600, 700) && tight.state() == RachState::Failed);
    /* No answer is accepted before the first transmission. */
    RachClient early(mac_of(9), 6, rnd, cc);
    CHECK(!early.on_responses({{mac_of(9), 6, RachStatus::Granted, 1, 1, 5}}));
    /* An answer for another nonce is ignored; ours joins. */
    RachClient j(mac_of(42), 42, rnd, cc);
    for (sf = 0; !j.opportunity(sf, 8000, 700); ++sf) {
    }
    CHECK(!j.on_responses({{mac_of(42), 41, RachStatus::Granted, 1, 1, 5}}) && j.state() == RachState::Sent);
    /* our nonce but another station's MAC: not ours */
    CHECK(!j.on_responses({{mac_of(41), 42, RachStatus::Granted, 1, 1, 5}}) && j.state() == RachState::Sent);
    CHECK(j.on_responses({{mac_of(42), 42, RachStatus::Granted, 2, 2, 6}}) && j.state() == RachState::Joined);
    CHECK(j.answer()->ue == 2);
    RachClient dn(mac_of(43), 43, rnd, cc);
    for (sf = 0; !dn.opportunity(sf, 8000, 700); ++sf) {
    }
    CHECK(dn.on_responses({{mac_of(43), 43, RachStatus::DeniedFull, 0, 0, 0}}) && dn.state() == RachState::Denied);
    CHECK(!dn.opportunity(sf + 50, 8000, 700)); /* denied: no retry storm */
  }

  /* --- Contention: 1 UE already in, 8 join through one 8 ms RACH slot; two
   *     msgA within 2 ms of each other in the same superframe collide (both
   *     lost). Capacity 8: seven get a grant, one is denied. --- */
  {
    UeRegistry reg({{1, 9}, {2, 10}, {3, 11}, {4, 12}, {5, 13}, {6, 14}, {7, 15}, {8, 16}});
    const auto existing = reg.admit(mac_of(1));
    RachServer srv(7, reg, key);
    uint32_t seed = 99;
    auto rnd = [&seed] { seed = seed * 1664525u + 1013904223u; return seed >> 8; };
    std::vector<RachClient> cl;
    for (uint8_t i = 0; i < 8; ++i)
      cl.emplace_back(mac_of(static_cast<uint8_t>(10 + i)), 1000 + i, rnd);
    uint64_t collisions = 0;
    std::vector<RachResponse> beacon; /* the answers in this superframe's beacon */
    uint64_t sf = 0;
    for (; sf < 2000; ++sf) {
      for (auto &c : cl)
        c.on_responses(beacon);
      bool done = true;
      for (auto &c : cl)
        if (c.state() == RachState::Waiting || c.state() == RachState::Sent)
          done = false;
      if (done)
        break;
      std::vector<std::pair<int64_t, size_t>> tx; /* (offset, client) */
      for (size_t i = 0; i < cl.size(); ++i)
        if (auto off = cl[i].opportunity(sf, 8000, 700))
          tx.push_back({*off, i});
      for (size_t a = 0; a < tx.size(); ++a) {
        bool hit = false;
        for (size_t b = 0; b < tx.size(); ++b)
          if (a != b && std::llabs(tx[a].first - tx[b].first) < 2000)
            hit = true;
        if (hit) {
          ++collisions;
          continue;
        }
        RachRequest q;
        q.cell_id = 7;
        q.mac = mac_of(static_cast<uint8_t>(10 + tx[a].second));
        q.nonce = cl[tx[a].second].nonce();
        q.attempt = cl[tx[a].second].attempt();
        q.fresh_sf = static_cast<uint32_t>(sf);
        const auto p = encode_rach_request(q, key);
        srv.on_request(p.data(), p.size(), sf);
      }
      beacon = srv.pending(sf + 1);
    }
    int joined = 0, denied = 0;
    std::set<uint16_t> ids;
    for (auto &c : cl) {
      if (c.state() == RachState::Joined) {
        ++joined;
        ids.insert(c.answer()->ue);
      } else if (c.state() == RachState::Denied) {
        ++denied;
      }
    }
    CHECK(joined == 7 && denied == 1);
    CHECK(ids.size() == 7 && !ids.count(existing->ue)); /* unique, never the existing UE's */
    CHECK(collisions > 0); /* the run really exercised loss -> backoff -> retry */
    CHECK(reg.find(mac_of(1))->ue == existing->ue && reg.find(mac_of(1))->ul_slot == existing->ul_slot);
    CHECK(sf < 2000);
    std::printf("cell_m3: 8 joiners, 7 positions free: %d joined, %d denied in %llu superframes "
                "(%llu collided msgA)\n",
                joined, denied, (unsigned long long)sf, (unsigned long long)collisions);
  }

  if (failures == 0)
    std::printf("cell_m3_selftest: all checks passed\n");
  return failures == 0 ? 0 : 1;
}
