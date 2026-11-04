// examples/cell — the M1 (#255) reference cell: one gNB-DU and N UEs on a
// fixed slot grid, plus an on-air witness. One binary, CELL_ROLE=du|ue|witness.
// Frame shapes, clocks and the control loops are in cell.h; the grid is
// src/cell/SlotGrid.h and the grant map src/cell/GrantMap.h.
//
//   DU      beacons the grant map at its TBTT (CW=0 beacon, fixed departure),
//           sends one unicast DL frame per UE in that UE's DL slot (submitted
//           when its TSF reaches the slot start; hardware-ACKed, retries per
//           DEVOURER_TX_RETRY_LIMIT, which the DL slot budget must cover),
//           measures each UE's uplink
//           beacon arrival on its own TSF and closes a per-UE timing-advance
//           loop by republishing the map (new epoch, effective 3 superframes
//           ahead — the M0 beacon-update latency is 1-2 beacons).
//   UE      fits the DU's TSF from the DU beacons (devourer::TsfSync against
//           its free-running counter), takes the grant map in force, and keeps
//           its own uplink beacon in its UL slot (minus its TA) by stepping its
//           TSF; ACKs its DL frames in hardware (SetAckResponder on its MAC).
//           Logs each DL frame's arrival phase on the DU grid.
//   witness logs every cell frame with its RX stamp (tests/cell_2ue_analyze.py
//           maps them onto the DU clock and checks slots and overlaps).
//
// Kestrel PCIe/USB: needs tsf_write_ok, tbtt_follows_tsf and beacon_backoff_ctl
// (the UE steers its beacon through its TSF). Radio: DEVOURER_PCIE_BDF or
// DEVOURER_VID/PID (examples/common/radio_open.h).
//
// Env (defaults in brackets; DU, UEs and witness must agree on the grid):
//   CELL_ROLE du|ue|witness   CELL_ID [1]   CELL_UE (role ue) 1..254
//   CELL_UES [1,2]            (DU: the UE list, grid order)
//   CELL_TU [100]             superframe = beacon interval, TU
//   CELL_BCN_SLOT_US [1500] CELL_DL_SLOT_US [4000] CELL_TURN_US [1000]
//   CELL_UL_SLOT_US [2000]    slot lengths (src/cell/SlotGrid.h budget)
//   CELL_UL_TARGET_US [500]   where in its UL slot the DU wants an arrival
//   CELL_DL_GUARD_US [500]    DL departure guard in the DL slot budget (M0:
//                             8852CE ch149 p99.9 0.46 ms)
//   DEVOURER_TX_RETRY_LIMIT [0] DL retries; the DL slot must budget them
// The DU refuses a grid whose slots cannot carry their frames: beacon airtime
// + 22 us departure in the beacon slot, the DL frame's full budget (attempts x
// (airtime + SIFS + ACK) + retry gaps + guard) in a DL slot, and the UL target
// plus the uplink beacon's airtime inside a UL slot.
//   CELL_TA [1] CELL_TA_GAIN [0.5] CELL_TA_EVERY_S [2]   DU TA loop
//   CELL_DL_PAD [64]          DL payload padding bytes
//   CELL_HARQ [1]             DL HARQ from the UE feedback (src/cell/Harq.h);
//                             0 = one attempt per frame (the baseline)
//   CELL_HARQ_ATTEMPTS [4] CELL_HARQ_TIMEOUT_SF [3]
//   CELL_LA [1]               DL link adaptation (src/cell/LinkAdapt.h, needs
//                             HARQ's loss counts), rate published in the grant
//   CELL_DL_RATE [24]         the starting DL rate, Mb/s (legacy OFDM)
//   CELL_ATTEN_UE [0] CELL_ATTEN_DB [0]   send that UE's DL at N dB against
//                             the chip's calibrated base (a per-packet radiotap
//                             DBM_TX_POWER level; needs per_packet_txpower): a
//                             path-loss stand-in for the reliability test
//   CELL_PSK                  the cell key: tags (HMAC-SHA1, 8 bytes) the HARQ
//                             feedback IE and the DL body; unset = an
//                             unauthenticated development cell. Needs OpenSSL.
//   CELL_SECS [60]            run time
//   DEVOURER_CHANNEL [149]
// Output: JSONL on stdout ("ev":"cell.*").
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "RadiotapBuilder.h"
#include "SelectedChannel.h"
#include "TsfSync.h"
#include "cell.h"
#include "cell/GrantMap.h"
#include "cell/Harq.h"
#include "cell/LinkAdapt.h"
#include "cell/SlotGrid.h"
#include "logger.h"
#include "radio_open.h"
#if defined(DEVOURER_CELL_OPENSSL)
#include <openssl/evp.h>
#include <openssl/hmac.h>
#endif

using namespace cellnode;
using devourer::cell::build_fixed_grid;
using devourer::cell::Grant;
using devourer::cell::GrantMapReceiver;
using devourer::cell::HarqConfig;
using devourer::cell::HarqReceiver;
using devourer::cell::HarqSender;
using devourer::cell::HarqStats;
using devourer::cell::LinkAdapt;

namespace {

long envl(const char *k, long d, long lo, long hi) {
  const char *v = std::getenv(k);
  if (!v || !*v)
    return d;
  char *end = nullptr;
  const long x = std::strtol(v, &end, 0);
  if (*end || x < lo || x > hi) {
    fprintf(stderr, "invalid %s=\"%s\" (want an integer in %ld..%ld)\n", k, v, lo, hi);
    std::exit(2);
  }
  return x;
}
double envf(const char *k, double d, double lo, double hi) {
  const char *v = std::getenv(k);
  if (!v || !*v)
    return d;
  char *end = nullptr;
  const double x = std::strtod(v, &end);
  if (*end || !(x >= lo && x <= hi)) {
    fprintf(stderr, "invalid %s=\"%s\" (want a number in %g..%g)\n", k, v, lo, hi);
    std::exit(2);
  }
  return x;
}
std::vector<uint16_t> env_ues() {
  std::vector<uint16_t> v;
  const char *s = std::getenv("CELL_UES");
  std::string t = s && *s ? s : "1,2";
  size_t p = 0;
  while (p <= t.size()) {
    const size_t q = t.find(',', p);
    const std::string item = t.substr(p, q == std::string::npos ? std::string::npos : q - p);
    char *end = nullptr;
    const long x = std::strtol(item.c_str(), &end, 10);
    if (item.empty() || *end || x < 1 || x > 254) {
      fprintf(stderr, "invalid CELL_UES=\"%s\" (want ids 1..254, comma-separated)\n", t.c_str());
      std::exit(2);
    }
    for (uint16_t seen : v)
      if (seen == x) {
        fprintf(stderr, "invalid CELL_UES=\"%s\" (UE %ld listed twice)\n", t.c_str(), x);
        std::exit(2);
      }
    v.push_back(static_cast<uint16_t>(x));
    if (q == std::string::npos)
      break;
    p = q + 1;
  }
  return v;
}

struct Cfg {
  uint16_t cell = 1;
  int tu = 100;
  int64_t bcn_us = 1500, dl_us = 4000, turn_us = 1000, ul_us = 2000, ul_target_us = 500;
  uint8_t ch = 149;
  int secs = 60;
  std::string psk;
};
Cfg cfg_from_env() {
  Cfg c;
  c.cell = static_cast<uint16_t>(envl("CELL_ID", 1, 1, 65535));
  c.tu = static_cast<int>(envl("CELL_TU", 100, 10, 1000));
  c.bcn_us = envl("CELL_BCN_SLOT_US", 1500, 100, 100000);
  c.dl_us = envl("CELL_DL_SLOT_US", 4000, 100, 100000);
  c.turn_us = envl("CELL_TURN_US", 1000, 100, 100000);
  c.ul_us = envl("CELL_UL_SLOT_US", 2000, 100, 100000);
  c.ul_target_us = envl("CELL_UL_TARGET_US", 500, 0, 100000);
  c.ch = static_cast<uint8_t>(envl("DEVOURER_CHANNEL", 149, 1, 196));
  c.secs = static_cast<int>(envl("CELL_SECS", 60, 1, 86400));
  if (const char *k = std::getenv("CELL_PSK"))
    c.psk = k;
  return c;
}

/* The scheduling-frame tag primitive (cell.h CellMac): HMAC-SHA1 keyed by
 * CELL_PSK, or empty (an unauthenticated cell) without a key. */
CellMac cell_auth(const Cfg &c) {
  if (c.psk.empty())
    return {};
#if defined(DEVOURER_CELL_OPENSSL)
  const std::string key = c.psk;
  return [key](const uint8_t *d, size_t n, uint8_t out[20]) {
    unsigned int len = 0;
    return HMAC(EVP_sha1(), key.data(), static_cast<int>(key.size()), d, n, out, &len) != nullptr &&
           len == 20;
  };
#else
  return {};
#endif
}
bool auth_ok(const Cfg &c) {
#if !defined(DEVOURER_CELL_OPENSSL)
  if (!c.psk.empty()) {
    fprintf(stderr, "CELL_PSK needs a build with OpenSSL\n");
    return false;
  }
#endif
  if (!c.psk.empty() && c.psk.size() < 8) {
    fprintf(stderr, "CELL_PSK needs 8+ characters\n");
    return false;
  }
  return true;
}
std::optional<SlotGrid> grid_for(const Cfg &c, const std::vector<uint16_t> &ues) {
  return build_fixed_grid(c.tu, 0, c.bcn_us, ues, c.dl_us, c.turn_us, c.ul_us);
}

std::mutex g_out_mu;
void emit(const char *line) {
  std::lock_guard<std::mutex> lk(g_out_mu);
  fputs(line, stdout);
  fputc('\n', stdout);
  fflush(stdout);
}

// Returns the TSF reading that ended the wait (>= target).
int64_t sleep_until_tsf(IRadio *dev, int64_t target) {
  for (;;) {
    const int64_t now = static_cast<int64_t>(dev->ReadTsf());
    const int64_t left = target - now;
    if (left <= 0)
      return now;
    if (left > 1500)
      std::this_thread::sleep_for(std::chrono::microseconds(left - 1000));
  }
}

bool caps_ok(IRadio *dev) {
  const auto c = dev->GetAdapterCaps();
  if (!c.tsf_write_ok || !c.tbtt_follows_tsf || !c.beacon_backoff_ctl || !c.hw_beacon_txtsf) {
    fprintf(stderr, "this radio cannot run a cell node (tsf_write_ok=%d tbtt_follows_tsf=%d "
                    "beacon_backoff_ctl=%d hw_beacon_txtsf=%d)\n",
            c.tsf_write_ok, c.tbtt_follows_tsf, c.beacon_backoff_ctl, c.hw_beacon_txtsf);
    return false;
  }
  return true;
}

// The DU's grid must carry its frames (see the header); checked before the
// radio is opened, so a configuration error never touches the hardware.
bool du_budget_ok(const Cfg &c, const std::vector<uint16_t> &ues) {
  const size_t pad = static_cast<size_t>(envl("CELL_DL_PAD", 64, 0, 1400));
  const int64_t dl_guard = envl("CELL_DL_GUARD_US", 500, 0, 100000);
  const int retry_limit = static_cast<int>(envl("DEVOURER_TX_RETRY_LIMIT", 0, 0, 63));
  if (!grid_for(c, ues)) {
    fprintf(stderr, "the grid does not fit a %d TU superframe\n", c.tu);
    return false;
  }
  {
    using devourer::cell::ofdm_airtime_us;
    using devourer::cell::SlotBudget;
    using devourer::cell::slot_len_us;
    GrantMap probe;
    probe.cell_id = c.cell;
    probe.period_tu = static_cast<uint16_t>(c.tu);
    probe.grants.resize(ues.size());
    const int64_t bcn_need = 22 + ofdm_airtime_us(build_du_beacon(probe).size() + 4, 6);
    SlotBudget dl;
    dl.mpdu_len = build_dl_frame(c.cell, 1, 0, 0, pad).size() + 4;
    dl.acked = true;
    dl.attempts = retry_limit + 1;
    /* per retry: ACK timeout (~SIFS + ACK + slot) + AIFS + CWmin backoff */
    dl.retry_gap_us = devourer::cell::kSifsUs + 44 + devourer::cell::kSlotTimeUs + 34 +
                      15 * devourer::cell::kSlotTimeUs;
    dl.guard_us = dl_guard;
    const int64_t dl_need = slot_len_us(dl);
    const int64_t ul_need =
        c.ul_target_us + ofdm_airtime_us(build_ue_beacon(c.cell, 1, c.tu, 0).size() + 4, 6);
    if (bcn_need > c.bcn_us || dl_need == 0 || dl_need > c.dl_us || ul_need > c.ul_us) {
      fprintf(stderr, "slots too short for their frames: beacon %lld/%lld us, DL %lld/%lld us "
                      "(%d attempts, guard %lld), UL target+airtime %lld/%lld us\n",
              (long long)bcn_need, (long long)c.bcn_us, (long long)dl_need, (long long)c.dl_us,
              retry_limit + 1, (long long)dl_guard, (long long)ul_need, (long long)c.ul_us);
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
int run_du(IRadio *dev, const Cfg &c) {
  const auto ues = env_ues();
  const auto grid = grid_for(c, ues);
  if (!grid) {
    fprintf(stderr, "the grid does not fit a %d TU superframe\n", c.tu);
    return 2;
  }
  const bool ta_on = envl("CELL_TA", 1, 0, 1) != 0;
  const double ta_gain = envf("CELL_TA_GAIN", 0.5, 0.0, 1.0);
  const int ta_every_s = static_cast<int>(envl("CELL_TA_EVERY_S", 2, 1, 60));
  const size_t pad = static_cast<size_t>(envl("CELL_DL_PAD", 64, 0, 1400));
  const bool harq_on = envl("CELL_HARQ", 1, 0, 1) != 0;
  const bool la_on = harq_on && envl("CELL_LA", 1, 0, 1) != 0;
  HarqConfig hc;
  hc.max_attempts = static_cast<int>(envl("CELL_HARQ_ATTEMPTS", 4, 1, 16));
  hc.timeout_sf = static_cast<int>(envl("CELL_HARQ_TIMEOUT_SF", 3, 1, 50));
  const int dl_rate0 = static_cast<int>(envl("CELL_DL_RATE", 24, 6, 54));
  const uint16_t atten_ue = static_cast<uint16_t>(envl("CELL_ATTEN_UE", 0, 0, 254));
  const int8_t atten_db = static_cast<int8_t>(envl("CELL_ATTEN_DB", 0, -40, 0));
  const int64_t P = grid->period_us();
  if (atten_ue != 0 && atten_db != 0 && !dev->GetAdapterCaps().per_packet_txpower) {
    fprintf(stderr, "CELL_ATTEN_DB needs per-packet TX power (per_packet_txpower=0): the "
                    "attenuation would silently not apply\n");
    return 2;
  }
  const CellMac auth = cell_auth(c);

  dev->InitWrite(SelectedChannel{c.ch, 0, CHANNEL_WIDTH_20});
  if (!dev->SetBeaconBackoff(false)) {
    fprintf(stderr, "SetBeaconBackoff(false) refused\n");
    return 1;
  }
  dev->SetCcaMode(true);  // coordinated grid: no carrier-sense deferral

  GrantMap map;
  GrantMap prev_map; /* the map before the last publish (in force until map's effective_sf) */
  map.cell_id = c.cell;
  map.epoch = 1;
  map.effective_sf = 0;
  map.period_tu = static_cast<uint16_t>(c.tu);
  for (uint16_t ue : ues) {
    Grant g;
    g.ue = ue;
    g.dl_slot = static_cast<uint8_t>(*grid->find(SlotKind::Downlink, ue));
    g.ul_slot = static_cast<uint8_t>(*grid->find(SlotKind::Uplink, ue));
    g.rate_mbps = static_cast<uint8_t>(LinkAdapt({}, dl_rate0).rate_mbps());
    map.grants.push_back(g);
  }
  auto bcn = build_du_beacon(map);
  if (bcn.empty() || !dev->StartBeacon(bcn.data(), bcn.size(), c.tu)) {
    fprintf(stderr, "StartBeacon failed\n");
    return 1;
  }

  uint64_t t0 = 0, f0 = 0;
  if (!dev->ReadTsfAndFreeRun(t0, f0)) {
    fprintf(stderr, "ReadTsfAndFreeRun unsupported\n");
    return 1;
  }
  // DU tsf - freerun: constant (the DU never writes its TSF).
  const int64_t tsf_minus_fr = static_cast<int64_t>(t0) - static_cast<int64_t>(f0);
  std::atomic<int64_t> fr_ref{static_cast<int64_t>(f0)};

  std::mutex mu;  // guards ta, ue_epoch
  std::map<uint16_t, TaLoop> ta;
  std::map<uint16_t, uint32_t> ue_epoch;
  for (uint16_t ue : ues)
    ta.emplace(ue, TaLoop(ta_gain, P / 4));
  /* Per-UE DL reliability state (guarded by mu): HARQ sender, link
   * adaptation, the UE-reported DL SNR over the current window, and the
   * HARQ counts at the last adaptation window. */
  struct DlUe {
    HarqSender harq;
    LinkAdapt la;
    double snr_sum = 0;
    uint64_t snr_n = 0;
    HarqStats at_window;
    uint64_t ul_in_window = 0; /* uplinks heard from this UE since the last window */
    bool prev_full = false;    /* the previous window was fully present too */
    uint32_t plain_seq = 0; /* CELL_HARQ=0: one attempt per frame */
    uint64_t last_gen = 0;     /* replay floor: the newest feedback gen_sf accepted */
    bool have_gen = false;
    uint8_t last_snr_gen = 0;  /* the SNR measurement last folded */
    bool have_snr_gen = false;
  };
  std::map<uint16_t, DlUe> dlue;
  for (uint16_t ue : ues)
    dlue.emplace(ue, DlUe{HarqSender(hc), LinkAdapt({}, dl_rate0)});

  char line[384];
  snprintf(line, sizeof line,
           "{\"ev\":\"cell.du.start\",\"cell\":%u,\"period_us\":%lld,\"ues\":%zu,\"ch\":%u,"
           "\"tsf_minus_fr\":%lld}",
           c.cell, (long long)P, ues.size(), c.ch, (long long)tsf_minus_fr);
  emit(line);
  for (size_t i = 0; i < grid->slots().size(); ++i) {
    const auto &s = grid->slots()[i];
    snprintf(line, sizeof line,
             "{\"ev\":\"cell.slot\",\"idx\":%zu,\"kind\":%u,\"ue\":%u,\"start_us\":%lld,"
             "\"len_us\":%lld}",
             i, (unsigned)s.kind, s.ue, (long long)s.start_us, (long long)s.len_us);
    emit(line);
  }

  std::thread rx([&] {
    dev->StartRxLoop([&](const Packet &p) {
      if (p.RxAtrib.crc_err)
        return;
      const auto ub = parse_ue_beacon(p.Data.data(), p.Data.size(), c.cell, auth);
      if (!ub)
        return;
      const auto slot = grid->find(SlotKind::Uplink, ub->ue);
      if (!slot)
        return;
      const int64_t fr = extend32_near(p.RxAtrib.tsfl, fr_ref.load());
      const int64_t at = fr + tsf_minus_fr;  // DU tsf at RX
      const auto pos = grid->at(at);
      const int64_t target = grid->slots()[*slot].start_us + c.ul_target_us;
      const int64_t err = wrap_phase(pos.into_us - target, P);
      int64_t ta_now = 0;
      {
        std::lock_guard<std::mutex> lk(mu);
        auto it = ta.find(ub->ue);
        if (it != ta.end()) {
          it->second.sample(err);
          ta_now = it->second.ta();
        }
        ue_epoch[ub->ue] = ub->epoch;
        auto d = dlue.find(ub->ue);
        if (d != dlue.end()) {
          ++d->second.ul_in_window;
          devourer::cell::HarqFeedback fb = ub->fb; /* gen_sf: low 32 bits on air */
          fb.gen_sf = static_cast<uint64_t>(
              extend32_near(static_cast<uint32_t>(fb.gen_sf), pos.superframe));
          /* Replay floor: a report older than one already applied is a
           * replay (or reordering) and is not applied. The UE repeats its
           * latest report in every beacon, so equal gen_sf is normal. */
          const bool fresh_fb = !d->second.have_gen || fb.gen_sf >= d->second.last_gen;
          if (fresh_fb) {
            d->second.last_gen = fb.gen_sf;
            d->second.have_gen = true;
            if (harq_on)
              d->second.harq.on_feedback(fb);
          }
          /* One sample per SNR measurement, not per beacon repeating it. */
          if (ub->dl_snr_half_db &&
              (!d->second.have_snr_gen || ub->snr_gen != d->second.last_snr_gen)) {
            d->second.snr_sum += ub->dl_snr_half_db / 2.0;
            ++d->second.snr_n;
            d->second.last_snr_gen = ub->snr_gen;
            d->second.have_snr_gen = true;
          }
        }
      }
      char l[256];
      snprintf(l, sizeof l,
               "{\"ev\":\"cell.ul\",\"ue\":%u,\"sf\":%lld,\"phase_us\":%lld,\"err_us\":%lld,"
               "\"in_slot\":%d,\"ue_epoch\":%u,\"ta_us\":%lld}",
               ub->ue, (long long)pos.superframe, (long long)pos.into_us, (long long)err,
               pos.slot && *pos.slot == *slot ? 1 : 0, ub->epoch, (long long)ta_now);
      emit(l);
    });
  });

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(c.secs);
  auto next_ta = std::chrono::steady_clock::now() + std::chrono::seconds(ta_every_s);
  int64_t sf = grid->at(static_cast<int64_t>(dev->ReadTsf())).superframe + 1;
  while (std::chrono::steady_clock::now() < deadline) {
    for (uint16_t ue : ues) {
      const size_t idx = *grid->find(SlotKind::Downlink, ue);
      const int64_t start = grid->slot_start_tsf(sf, idx);
      /* What to send: a due HARQ retransmission, else new data (or, with
       * HARQ off, a new frame every superframe). */
      std::optional<devourer::cell::HarqTx> tx;
      {
        std::lock_guard<std::mutex> lk(mu);
        DlUe &d = dlue.at(ue);
        if (harq_on) {
          tx = d.harq.next(static_cast<uint64_t>(sf), true);
        } else {
          const uint32_t s = d.plain_seq++;
          tx = devourer::cell::HarqTx{s, s % devourer::cell::kHarqProcesses, s, 1, false};
        }
      }
      if (!tx) {
        char l[120];
        snprintf(l, sizeof l, "{\"ev\":\"cell.dl.idle\",\"ue\":%u,\"sf\":%lld}", ue,
                 (long long)sf);
        emit(l);
        continue; /* window full, nothing due */
      }
      /* The rate of the grant IN FORCE: a staged rate applies from its map's
       * effective superframe, when the UE applies that map too. */
      const GrantMap &in_force =
          static_cast<uint64_t>(sf) >= map.effective_sf || prev_map.grants.empty() ? map : prev_map;
      const Grant *gr = in_force.find(ue) ? in_force.find(ue) : map.find(ue);
      const int rate = gr ? gr->rate_mbps : 6;
      /* The frame is built before the wait; the reading that ends the wait
       * is the submit stamp (no extra register reads on the send path). */
      auto f = build_dl_frame(c.cell, static_cast<uint8_t>(ue), static_cast<uint64_t>(sf), tx->seq,
                              pad,
                              DlHarq{tx->low, static_cast<uint8_t>(tx->pid),
                                     static_cast<uint8_t>(tx->attempt)},
                              auth);
      std::vector<uint8_t> pkt = build_dl_radiotap(
          rate, ue == atten_ue && atten_db != 0 ? std::optional<int8_t>(atten_db) : std::nullopt);
      pkt.insert(pkt.end(), f.begin(), f.end());
      const int64_t sub = sleep_until_tsf(dev, start);
      const bool ok = dev->send_packet(pkt.data(), pkt.size());
      if (!ok && harq_on) {
        std::lock_guard<std::mutex> lk(mu);
        dlue.at(ue).harq.unsend(*tx); /* never reached the radio: no attempt spent */
      }
      char l[220];
      snprintf(l, sizeof l,
               "{\"ev\":\"cell.dl\",\"ue\":%u,\"sf\":%lld,\"seq\":%u,\"attempt\":%d,"
               "\"rate\":%d,\"late_us\":%lld,\"ok\":%d}",
               ue, (long long)sf, tx->seq, tx->attempt, rate, (long long)(sub - start), ok ? 1 : 0);
      emit(l);
    }
    // Idle part of the superframe: refresh the freerun reference, run the TA
    // loop, republish the map when a TA moved.
    uint64_t t = 0, f = 0;
    if (dev->ReadTsfAndFreeRun(t, f))
      fr_ref.store(static_cast<int64_t>(f));
    if (ta_on && std::chrono::steady_clock::now() >= next_ta) {
      next_ta += std::chrono::seconds(ta_every_s);
      /* Stage the next map; it replaces `map` only once it is on air. */
      GrantMap next = map;
      bool changed = false;
      {
        std::lock_guard<std::mutex> lk(mu);
        for (Grant &g : next.grants) {
          TaLoop &l = ta.at(g.ue);
          // Only judge a UE running the newest map: its arrivals reflect it.
          const bool current = ue_epoch.count(g.ue) && ue_epoch[g.ue] == map.epoch;
          if (!current)
            l.discard();
          else if (l.update(3)) {
            g.ta_us = static_cast<int16_t>(l.ta());
            changed = true;
          }
          if (la_on) {
            DlUe &d = dlue.at(g.ue);
            const HarqStats &st = d.harq.stats();
            const uint64_t first = st.new_tx - d.at_window.new_tx;
            const uint64_t lost = (st.retx - d.at_window.retx) + (st.dropped - d.at_window.dropped);
            const bool snr_ok = d.snr_n > 0;
            const double snr = snr_ok ? d.snr_sum / static_cast<double>(d.snr_n) : 0.0;
            /* Judge only windows the UE was present for throughout (>= 75% of
             * the superframes carried its uplink), after an equally present
             * one: at the edges of a UE's presence the HARQ timeouts and
             * drops are of frames sent while it was not listening, and say
             * nothing about the channel. */
            const uint64_t sf_per_window =
                static_cast<uint64_t>(ta_every_s) * 1000000ull / static_cast<uint64_t>(P);
            const bool full = d.ul_in_window * 4 >= sf_per_window * 3;
            const bool present = full && d.prev_full;
            d.prev_full = full;
            if (present)
              d.la.window(snr, snr_ok, first, lost);
            d.at_window = st;
            d.snr_sum = 0;
            d.snr_n = 0;
            d.ul_in_window = 0;
            char la_l[220];
            snprintf(la_l, sizeof la_l,
                     "{\"ev\":\"cell.la\",\"ue\":%u,\"rate\":%d,\"snr_db\":%.1f,"
                     "\"first_tx\":%llu,\"lost\":%llu,\"present\":%d}",
                     g.ue, d.la.rate_mbps(), snr, (unsigned long long)first,
                     (unsigned long long)lost, present ? 1 : 0);
            emit(la_l);
            /* Compare with the live grant, not "did it change now": a rate
             * whose map failed to publish is retried on the next pass. */
            if (d.la.rate_mbps() != g.rate_mbps) {
              g.rate_mbps = static_cast<uint8_t>(d.la.rate_mbps());
              changed = true;
            }
          }
        }
      }
      if (changed) {
        ++next.epoch;
        next.effective_sf = static_cast<uint64_t>(sf + 3);
        const auto nb = build_du_beacon(next);
        const bool ok = !nb.empty() && dev->UpdateBeaconPayload(nb.data(), nb.size());
        if (ok) {
          prev_map = map;
          map = next;
        } else {
          /* The old map stays on air: keep its epoch (UEs keep reporting it,
           * so sampling continues) and put the loops back on its TAs; the
           * next TA pass retries. */
          std::lock_guard<std::mutex> lk(mu);
          for (const Grant &g : map.grants)
            ta.at(g.ue).set_ta(g.ta_us);
        }
        const GrantMap &shown = ok ? map : next;
        char l[320];
        int n = snprintf(l, sizeof l, "{\"ev\":\"cell.grant\",\"epoch\":%u,\"effective_sf\":%llu,"
                                      "\"ok\":%d,\"ta\":{",
                         shown.epoch, (unsigned long long)shown.effective_sf, ok ? 1 : 0);
        for (size_t i = 0; i < shown.grants.size() && n > 0 && n < (int)sizeof l; ++i)
          n += snprintf(l + n, sizeof l - n, "%s\"%u\":[%d,%u]", i ? "," : "", shown.grants[i].ue,
                        shown.grants[i].ta_us, shown.grants[i].rate_mbps);
        if (n > 0 && n < (int)sizeof l)
          snprintf(l + n, sizeof l - n, "}}");
        emit(l);
      }
    }
    ++sf;
  }
  dev->StopBeacon();
  dev->StopRxLoop();
  rx.join();
  for (auto &kv : dlue) {
    const HarqStats &st = kv.second.harq.stats();
    char l[240];
    snprintf(l, sizeof l,
             "{\"ev\":\"cell.harq\",\"ue\":%u,\"harq\":%d,\"new_tx\":%llu,\"retx\":%llu,"
             "\"acked\":%llu,\"dropped\":%llu,\"nacks\":%llu,\"timeouts\":%llu,\"rate\":%d}",
             kv.first, harq_on ? 1 : 0, (unsigned long long)st.new_tx, (unsigned long long)st.retx,
             (unsigned long long)st.acked, (unsigned long long)st.dropped,
             (unsigned long long)st.nacks, (unsigned long long)st.timeouts,
             kv.second.la.rate_mbps());
    emit(l);
  }
  emit("{\"ev\":\"cell.du.done\"}");
  return 0;
}

// ---------------------------------------------------------------------------
int run_ue(IRadio *dev, const Cfg &c) {
  const uint8_t me = static_cast<uint8_t>(envl("CELL_UE", 0, 1, 254));
  dev->InitWrite(SelectedChannel{c.ch, 0, CHANNEL_WIDTH_20});
  devourer::MacAddr own;
  cell_mac(own.bytes.data(), c.cell, me);
  if (!dev->SetAckResponder(own)) {
    fprintf(stderr, "SetAckResponder refused (ack_responder_ok=%d)\n",
            dev->GetAdapterCaps().ack_responder_ok ? 1 : 0);
    return 1;
  }
  if (!dev->SetBeaconBackoff(false)) {
    fprintf(stderr, "SetBeaconBackoff(false) refused\n");
    return 1;
  }
  dev->SetCcaMode(true);

  const CellMac auth = cell_auth(c);
  std::mutex mu;  // guards sync, wrap, rxr
  devourer::TsfSync sync;
  Wrap32 wrap;
  GrantMapReceiver rxr(c.cell);
  /* DL HARQ receiver + the DL SNR since the last feedback publish (mu). */
  HarqReceiver hrx;
  double dl_snr_sum = 0;
  uint64_t dl_snr_n = 0;
  uint64_t dl_new = 0, dl_dup = 0;
  const int64_t P = static_cast<int64_t>(c.tu) * devourer::cell::kTuUs;
  std::optional<SlotGrid> grid;  // rebuilt from the map's UE list
  std::vector<uint16_t> grid_ues;
  uint8_t du_mac[6];
  cell_mac(du_mac, c.cell, 0);

  std::thread rx([&] {
    dev->StartRxLoop([&](const Packet &p) {
      if (p.RxAtrib.crc_err || p.Data.size() < 24)
        return;
      if (p.Data[0] == 0x80 && std::memcmp(p.Data.data() + 10, du_mac, 6) == 0) {
        const auto eg = p.TxEgressTsf();
        if (!eg)
          return;
        std::lock_guard<std::mutex> lk(mu);
        sync.Add(*eg, p.RxAtrib.tsfl);
        wrap.feed(p.RxAtrib.tsfl);
        if (auto m = devourer::cell::find_grant_map(p.Data.data(), p.Data.size()))
          rxr.offer(*m);
        return;
      }
      const auto d = parse_dl_frame(p.Data.data(), p.Data.size(), c.cell, auth);
      if (!d || d->ue != me)
        return;
      std::optional<size_t> want;
      SlotGrid::At pos;
      bool fresh = false;
      {
        std::lock_guard<std::mutex> lk(mu);
        /* HARQ first: the feedback must count every frame that arrived. */
        fresh = hrx.on_frame(d->seq, d->harq.low);
        if (fresh)
          ++dl_new;
        else
          ++dl_dup;
        if (p.RxAtrib.snr[0] > 0) { /* raw = 0.5 dB units; 0 = no measurement */
          dl_snr_sum += p.RxAtrib.snr[0] / 2.0;
          ++dl_snr_n;
        }
        if (!sync.Ready() || !grid)
          return;
        const int64_t du_at =
            sync.RemoteForLocal(static_cast<uint64_t>(wrap.extend(p.RxAtrib.tsfl)));
        want = grid->find(SlotKind::Downlink, me);
        pos = grid->at(du_at);
      }
      if (!want)
        return;
      char l[260];
      snprintf(l, sizeof l,
               "{\"ev\":\"cell.dl.rx\",\"ue\":%u,\"sf\":%llu,\"seq\":%u,\"attempt\":%u,"
               "\"new\":%d,\"rx_sf\":%lld,\"phase_us\":%lld,\"in_slot\":%d,\"rssi\":%d,"
               "\"snr_half_db\":%d}",
               me, (unsigned long long)d->sf, d->seq, d->harq.attempt, fresh ? 1 : 0,
               (long long)pos.superframe, (long long)pos.into_us,
               pos.slot && *pos.slot == *want ? 1 : 0, p.RxAtrib.rssi[0] - 110, p.RxAtrib.snr[0]);
      emit(l);
    });
  });

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(c.secs);
  bool beaconing = false;
  uint32_t running_epoch = 0;
  devourer::cell::HarqFeedback last_fb{};
  uint8_t last_snr_half = 0;
  uint8_t last_snr_gen = 0;
  long steps = 0;
  /* TSF-write compensation: a read-add-write lands a few us late (the MMIO
   * round trip between ReadTsf and the write). Each step is checked against
   * the tsf - freerun offset read back after it, and the residual is folded
   * in (half per step, clamped), so the steady state is the crystal drift
   * alone, not a re-step every pass. */
  double comp_us = 0;
  long step_fails = 0;
  /* The DU superframes this UE was running (synced, map in force), for the
   * DL accounting: frames sent outside it were never expected. */
  int64_t listen_from = -1, listen_to = -1;
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    uint64_t t = 0, f = 0;
    if (!dev->ReadTsfAndFreeRun(t, f))
      continue;
    int64_t du_now = 0;
    const GrantMap *m = nullptr;
    GrantMap mcopy;
    {
      std::lock_guard<std::mutex> lk(mu);
      if (!sync.Ready() || !wrap.ready())
        continue;
      du_now = sync.RemoteForLocal(static_cast<uint64_t>(wrap.extend(static_cast<uint32_t>(f))));
      const int64_t sfn = du_now >= 0 ? du_now / P : 0;
      m = rxr.current(static_cast<uint64_t>(sfn));
      if (m)
        mcopy = *m;
      if (m) {
        std::vector<uint16_t> u;
        for (const Grant &g : m->grants)
          u.push_back(g.ue);
        if (u != grid_ues) {
          grid_ues = u;
          grid = grid_for(c, u);
        }
      }
    }
    if (!m || !grid)
      continue;
    const Grant *g = mcopy.find(me);
    if (!g || g->ul_slot >= grid->slots().size())
      continue;
    const int64_t ul_off = grid->slots()[g->ul_slot].start_us;
    {
      const int64_t sfn = du_now / P;
      if (listen_from < 0)
        listen_from = sfn + 1; /* this superframe's DL may already be past */
      listen_to = sfn - 1; /* the last superframe fully listened */
    }
    /* The uplink beacon carries the grant epoch in force, the HARQ
     * feedback and the DL SNR: republish whenever any of them moved (the
     * DU reads them at its next UL slot; a beacon update lands 1-2 beacons
     * later, which Harq's nack_min_age_sf allows for). */
    devourer::cell::HarqFeedback fb{};
    uint8_t snr_half = last_snr_half;
    uint8_t snr_gen = last_snr_gen;
    {
      std::lock_guard<std::mutex> lk(mu);
      fb = hrx.feedback(static_cast<uint64_t>(du_now / P)); /* the UE's view of the DU sf */
      if (dl_snr_n > 0) { /* a NEW measurement: a new snr_gen */
        const double m = dl_snr_sum / static_cast<double>(dl_snr_n);
        snr_half = static_cast<uint8_t>(m * 2 < 1 ? 1 : (m * 2 > 255 ? 255 : std::lround(m * 2)));
        ++snr_gen;
        dl_snr_sum = 0;
        dl_snr_n = 0;
      }
    }
    /* Republish on a CHANGE of content (epoch, feedback base/bitmap, a new
     * SNR measurement) — not on the report's generation time alone, which
     * moves every superframe: each re-download can cost a beacon on some
     * generations. The published gen_sf is when this content was generated. */
    const bool content_moved = !beaconing || running_epoch != mcopy.epoch ||
                               fb.base != last_fb.base || fb.bitmap != last_fb.bitmap ||
                               snr_gen != last_snr_gen;
    if (content_moved) {
      const auto b = build_ue_beacon(c.cell, me, c.tu, mcopy.epoch, fb, snr_half, snr_gen, auth);
      const bool ok = beaconing ? dev->UpdateBeaconPayload(b.data(), b.size())
                                : dev->StartBeacon(b.data(), b.size(), c.tu);
      if (!ok) {
        fprintf(stderr, "UE beacon %s failed\n", beaconing ? "update" : "start");
        continue;
      }
      beaconing = true;
      const bool epoch_moved = running_epoch != mcopy.epoch;
      running_epoch = mcopy.epoch;
      last_fb = fb;
      last_snr_half = snr_half;
      last_snr_gen = snr_gen;
      if (epoch_moved) { /* a feedback-only refresh logs nothing */
        char l[160];
        snprintf(l, sizeof l, "{\"ev\":\"cell.ue.epoch\",\"ue\":%u,\"epoch\":%u,\"ta_us\":%d}",
                 me, mcopy.epoch, g->ta_us);
        emit(l);
      }
    }
    const int64_t step = ue_tsf_step(static_cast<int64_t>(t), du_now, ul_off, g->ta_us, P, 2);
    if (step != 0) {
      /* The offset changes only on a write, so the loop's (t, f) pair gives
       * the offset this step must produce. */
      const int64_t want_off = static_cast<int64_t>(t) - static_cast<int64_t>(f) + step;
      const uint64_t cur = dev->ReadTsf();
      if (!dev->WriteTsf(static_cast<uint64_t>(static_cast<int64_t>(cur) + step +
                                               static_cast<int64_t>(std::llround(comp_us))))) {
        /* Not a step: the next pass re-measures from a fresh read pair. */
        ++step_fails;
        char l[120];
        snprintf(l, sizeof l, "{\"ev\":\"cell.ue.step_fail\",\"ue\":%u,\"step_us\":%lld}",
                 me, (long long)step);
        emit(l);
        continue;
      }
      ++steps;
      int64_t resid = 0;
      uint64_t t2 = 0, f2 = 0;
      if (dev->ReadTsfAndFreeRun(t2, f2)) {
        resid = static_cast<int64_t>(t2) - static_cast<int64_t>(f2) - want_off;
        if (resid > -1000 && resid < 1000) { /* a sane read-back only */
          comp_us -= 0.5 * static_cast<double>(resid);
          if (comp_us > 50) comp_us = 50;
          if (comp_us < -50) comp_us = -50;
        }
      }
      double skew = 0;
      {
        std::lock_guard<std::mutex> lk(mu);
        skew = sync.SkewPpm();
      }
      char l[200];
      snprintf(l, sizeof l,
               "{\"ev\":\"cell.ue.step\",\"ue\":%u,\"step_us\":%lld,\"resid_us\":%lld,"
               "\"comp_us\":%.1f,\"skew_ppm\":%.2f}",
               me, (long long)step, (long long)resid, comp_us, skew);
      emit(l);
    }
  }
  if (beaconing)
    dev->StopBeacon();
  dev->StopRxLoop();
  rx.join();
  dev->ClearAckResponder();
  char l[200];
  snprintf(l, sizeof l,
           "{\"ev\":\"cell.ue.done\",\"ue\":%u,\"steps\":%ld,\"step_fails\":%ld,"
           "\"listen_from_sf\":%lld,\"listen_to_sf\":%lld,\"dl_new\":%llu,\"dl_dup\":%llu}",
           me, steps, step_fails, (long long)listen_from, (long long)listen_to,
           (unsigned long long)dl_new, (unsigned long long)dl_dup);
  emit(l);
  return 0;
}

// ---------------------------------------------------------------------------
int run_witness(IRadio *dev, const Cfg &c) {
  std::atomic<bool> done{false};
  std::thread rx([&] {
    dev->Init([&](const Packet &p) {
      if (p.RxAtrib.crc_err || p.Data.size() < 24)
        return;
      char l[220];
      const uint8_t *d = p.Data.data();
      if (d[0] == 0x80 && cell_station(d + 10, c.cell) >= 0) {
        const int st = cell_station(d + 10, c.cell);
        const auto eg = p.TxEgressTsf();
        uint32_t ep = 0;
        if (st > 0) {
          if (auto ub = parse_ue_beacon(d, p.Data.size(), c.cell))
            ep = ub->epoch;
        } else if (auto m = devourer::cell::find_grant_map(d, p.Data.size())) {
          ep = m->epoch;
        }
        snprintf(l, sizeof l,
                 "{\"ev\":\"cell.w\",\"k\":\"%s\",\"st\":%d,\"tsfl\":%u,\"egress\":%llu,"
                 "\"epoch\":%u,\"len\":%zu}",
                 st == 0 ? "bcn" : "ul", st, p.RxAtrib.tsfl,
                 (unsigned long long)(eg ? *eg : 0), ep, p.Data.size());
        emit(l);
      } else if (auto dl = parse_dl_frame(d, p.Data.size(), c.cell)) {
        snprintf(l, sizeof l,
                 "{\"ev\":\"cell.w\",\"k\":\"dl\",\"st\":%u,\"tsfl\":%u,\"sf\":%llu,\"seq\":%u,"
                 "\"len\":%zu}",
                 dl->ue, p.RxAtrib.tsfl, (unsigned long long)dl->sf, dl->seq, p.Data.size());
        emit(l);
      }
    }, SelectedChannel{c.ch, 0, CHANNEL_WIDTH_20});
    done = true;
  });
  std::this_thread::sleep_for(std::chrono::seconds(c.secs));
  while (!done) {
    dev->StopRxLoop();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  rx.join();
  return 0;
}

}  // namespace

int main() {
  const char *role = std::getenv("CELL_ROLE");
  if (!role || (std::strcmp(role, "du") && std::strcmp(role, "ue") && std::strcmp(role, "witness"))) {
    fprintf(stderr, "set CELL_ROLE=du|ue|witness\n");
    return 2;
  }
  const Cfg c = cfg_from_env();
  if (!auth_ok(c))
    return 2;
  if (std::strcmp(role, "du") == 0 && !du_budget_ok(c, env_ues()))
    return 2;
  auto logger = std::make_shared<Logger>();
  apply_logging_env(*logger);
  OpenedRadio radio = open_radio_from_env(logger, 0x0bda, 0);
  if (!radio.dev)
    return 1;
  IRadio *dev = radio.dev.get();
  if (std::strcmp(role, "witness") == 0)
    return run_witness(dev, c);
  if (!caps_ok(dev))
    return 1;
  return std::strcmp(role, "du") == 0 ? run_du(dev, c) : run_ue(dev, c);
}
