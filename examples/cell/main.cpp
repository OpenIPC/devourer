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
//   CELL_CAPACITY [#CELL_UES] UE positions in the grid (M3): static UEs take
//                             1..n of them, joiners get the rest
//   CELL_RACH_US [0]          RACH (contention) slot length; 0 = no joining.
//                             Needs CELL_PSK and an OpenSSL build.
//   CELL_PSK                  the cell key (HMAC-SHA1, 8-byte tags): the HARQ
//                             feedback IE, the DL body and the RACH; unset = an
//                             unauthenticated development cell (no RACH).
//   CELL_UE=0 (role ue)       a JOINING UE: it gets its id through the RACH
//   CELL_JOINER_MAC           the joiner's MAC (default: derived from the host
//                             name + DEVOURER_PCIE_BDF, so a restart re-joins
//                             into the same position instead of a new one)
//   CELL_RELEASE_SF [50]      DU: release a JOINED UE whose uplink has been
//                             absent this many superframes (static UEs stay)
//   CELL_SECS [60]            run time
// DU service mode (M4, #258) — set CELL_REF_PTP to hold this DU's beacon grid
// to a PTP hardware clock (src/cell/TsfDiscipline.h; needs a Tight sync tier,
// AdapterCaps sync_tier) and gate the coordinated schedule on its health
// (src/cell/SyncHealth.h). The DU starts AUTONOMOUS (carrier sense on, random
// beacon backoff, kGrantAutonomous in the grant map, so its UEs do the same)
// and turns coordinated (no deferral, deterministic beacon) only once Locked.
// It falls back to autonomous on any loss.
//   CELL_REF_PTP              the PHC, e.g. /dev/ptp0 (unset: a standalone
//                             cell, always coordinated, the M1-M3 behaviour)
//   CELL_PHASE_US [0]         where this DU's TBTT sits: ref % period
//   CELL_SYNC_LOOP_MS [500]   discipline period
//   CELL_SYNC_ACQUIRE_S [30]  wait for lock before serving (then autonomous)
//   CELL_SYNC_LOCK_US [10] CELL_SYNC_LOST_US [200]   SyncHealth thresholds
//   CELL_SYNC_LOCK_ITERS [8] CELL_SYNC_LOST_ITERS [20]
//   CELL_REF_CHECK            a shell command; exit 0 = the reference is good
//                             (e.g. ptp4l in SLAVE, checked with pmc); run
//                             every CELL_REF_CHECK_S [2], killed after
//                             CELL_REF_CHECK_TIMEOUT_MS [1500] (a failure).
//                             Required with CELL_REF_PTP: a readable PHC is
//                             not proof of sync.
// F1 (M5, #259) — set CELL_F1_CU to take this DU's slot positions from a CU
// (tools/ran-controller, src/cell/F1.h). Needs the DU service mode: the
// network superframe count comes from the PTP clock. The DU reports its
// grid, its UEs and the other cells' UEs it hears; it stages the CU's
// schedules and applies each on its network-wide instant. It stays
// autonomous until it has a CU, and falls back to autonomous when the CU
// goes quiet.
//   CELL_F1_CU                host:port of the CU (UDP)
//   CELL_F1_KEY               the F1 key (HMAC-SHA1, as CELL_PSK); required
//                             unless CELL_F1_INSECURE=1 (a development link).
//                             Schedules are taken only from the CU's address.
//   CELL_F1_REPORT_MS [1000]  report period
//   CELL_F1_TIMEOUT_MS [3000] no CU message this long = CU outage
//   CELL_F1_MIN_LEAD_SF [8] CELL_F1_PUBLISH_LEAD_SF [5]   the failure model
//   CELL_SYNC_UL_SF [30]      beacon-engine proxy: once UEs in the map have
//                             been heard, all of them silent this many
//                             superframes = the DU's beacon is not reaching
//                             them. Three failed beacon updates in a row also
//                             count.
//   DEVOURER_CHANNEL [149]
// Output: JSONL on stdout ("ev":"cell.*").
#include <algorithm>
#include <array>
#include <atomic>
#include <random>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

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
#include "cell/Coord.h"
#include "cell/F1.h"
#include "cell/Rach.h"
#include "cell/SyncHealth.h"
#include "cell/TsfDiscipline.h"
#include "cell/UeRegistry.h"
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

#if defined(__linux__)
/* A PTP hardware clock (/dev/ptpN) as a dynamic POSIX clock: Linux only. */
#define CELL_FD_TO_CLOCKID(fd) ((~(clockid_t)(fd) << 3) | 3)
#endif

bool phc_us(clockid_t c, double &us) {
  struct timespec t;
  if (clock_gettime(c, &t) != 0)
    return false;
  us = static_cast<double>(t.tv_sec) * 1e6 + static_cast<double>(t.tv_nsec) / 1e3;
  return true;
}

/* Run `sh -c cmd` with a deadline: true only when it exits 0 in time. A
 * check that hangs, or is still running when `stop` is raised, is killed
 * (its whole process group) and reaped, and counts as a failure. */
bool run_check(const char *cmd, int timeout_ms, const std::atomic<bool> &stop) {
  const pid_t pid = fork();
  if (pid < 0)
    return false;
  if (pid == 0) {
    setpgid(0, 0);
    execl("/bin/sh", "sh", "-c", cmd, static_cast<char *>(nullptr));
    _exit(127);
  }
  setpgid(pid, pid); /* both sides: no race with the child's exec */
  const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  int status = 0;
  for (;;) {
    const pid_t r = waitpid(pid, &status, WNOHANG);
    if (r == pid)
      return WIFEXITED(status) && WEXITSTATUS(status) == 0;
    if (r < 0)
      return false;
    if (stop.load() || std::chrono::steady_clock::now() >= until) {
      kill(-pid, SIGKILL);
      waitpid(pid, &status, 0);
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

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
  size_t capacity = 2;
  int64_t rach_us = 0;
  std::string psk;
  uint8_t ch = 149;
  int secs = 60;
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
  c.capacity = static_cast<size_t>(envl("CELL_CAPACITY", static_cast<long>(env_ues().size()), 1,
                                        static_cast<long>(devourer::cell::kMaxUesPerCell)));
  c.rach_us = envl("CELL_RACH_US", 0, 0, 50000);
  if (const char *k = std::getenv("CELL_PSK"))
    c.psk = k;
  c.ch = static_cast<uint8_t>(envl("DEVOURER_CHANNEL", 149, 1, 196));
  c.secs = static_cast<int>(envl("CELL_SECS", 60, 1, 86400));
  return c;
}

/* The cell's HMAC-SHA1 primitive keyed by CELL_PSK: it tags the scheduling
 * frames (cell.h CellMac) and authenticates the M3 RACH (src/cell/Rach.h
 * Mac, the same function type). Empty without a key: an unauthenticated
 * cell, which cannot RACH. */
CellMac hmac_sha1(const std::string &key) {
  if (key.empty())
    return {};
#if defined(DEVOURER_CELL_OPENSSL)
  return [key](const uint8_t *d, size_t n, uint8_t out[20]) {
    unsigned int len = 0;
    return HMAC(EVP_sha1(), key.data(), static_cast<int>(key.size()), d, n, out, &len) != nullptr &&
           len == 20;
  };
#else
  return {};
#endif
}
CellMac cell_auth(const Cfg &c) { return hmac_sha1(c.psk); }
devourer::cell::Mac rach_mac(const Cfg &c) { return cell_auth(c); }
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
bool rach_ok(const Cfg &c) {
  if (c.rach_us == 0)
    return true;
#if !defined(DEVOURER_CELL_OPENSSL)
  fprintf(stderr, "CELL_RACH_US needs a build with OpenSSL (RACH authentication)\n");
  return false;
#else
  if (c.psk.size() < 8) {
    fprintf(stderr, "CELL_RACH_US needs CELL_PSK (8+ characters)\n");
    return false;
  }
  return true;
#endif
}

/* The grid is laid out for CELL_CAPACITY positions (placeholder UE ids
 * 1..capacity), so slot positions never move as UEs join (UeRegistry). */
std::optional<SlotGrid> grid_for(const Cfg &c) {
  std::vector<uint16_t> pos;
  for (size_t i = 1; i <= c.capacity; ++i)
    pos.push_back(static_cast<uint16_t>(i));
  return build_fixed_grid(c.tu, 0, c.bcn_us, pos, c.dl_us, c.turn_us, c.ul_us, c.rach_us);
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
  /* Static UEs take positions 1..n, in order (UeRegistry binds in order). */
  for (size_t i = 0; i < ues.size(); ++i)
    if (ues[i] != i + 1 || ues.size() > c.capacity) {
      fprintf(stderr, "CELL_UES must be 1..n with n <= CELL_CAPACITY (%zu)\n", c.capacity);
      return false;
    }
  if (!rach_ok(c))
    return false;
  if (!grid_for(c)) {
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
    probe.grants.resize(c.capacity);
    /* worst case: every position granted + a full RACH answer IE */
    const std::vector<uint8_t> rar_max(
        c.rach_us > 0 ? 3 + devourer::cell::kRachMaxPerBeacon *
                                (devourer::cell::rach_detail::kRspLen + devourer::cell::kRachTagLen)
                      : 0,
        0);
    std::vector<uint8_t> rar = rar_max;
    if (!rar.empty()) {
      rar[0] = 'R';
      rar[1] = 'B';
      rar[2] = static_cast<uint8_t>(devourer::cell::kRachMaxPerBeacon);
    }
    const int64_t bcn_need = 22 + ofdm_airtime_us(build_du_beacon(probe, rar).size() + 4, 6);
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
  const auto grid = grid_for(c);
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
  const int64_t release_sf = envl("CELL_RELEASE_SF", 50, 2, 100000);

  /* M4 DU service mode (see the header): the PHC, the discipline loop, the
   * health that gates the coordinated schedule. */
  const char *ref_ptp = std::getenv("CELL_REF_PTP");
  const bool svc = ref_ptp && *ref_ptp;
  const devourer::SyncTier tier = devourer::sync_tier(dev->GetAdapterCaps());
  struct Fd {
    int fd = -1;
    ~Fd() {
      if (fd >= 0)
        close(fd);
    }
  } ref_fd;
  clockid_t refclk = CLOCK_REALTIME;
  if (svc) {
    if (tier != devourer::SyncTier::Tight) {
      fprintf(stderr, "CELL_REF_PTP needs a tight-sync DU (this radio: %s)\n",
              devourer::sync_tier_name(tier));
      return 2;
    }
#if defined(__linux__)
    ref_fd.fd = open(ref_ptp, O_RDONLY);
    if (ref_fd.fd < 0) {
      fprintf(stderr, "open %s failed\n", ref_ptp);
      return 1;
    }
    refclk = CELL_FD_TO_CLOCKID(ref_fd.fd);
#else
    fprintf(stderr, "CELL_REF_PTP needs a PTP hardware clock (Linux only)\n");
    return 2;
#endif
  }
  const int sync_loop_ms = static_cast<int>(envl("CELL_SYNC_LOOP_MS", 500, 50, 10000));
  const int acquire_s = static_cast<int>(envl("CELL_SYNC_ACQUIRE_S", 30, 0, 3600));
  const int64_t sync_ul_sf = envl("CELL_SYNC_UL_SF", 30, 2, 100000);
  const int ref_check_s = static_cast<int>(envl("CELL_REF_CHECK_S", 2, 1, 600));
  /* M5 F1: the CU link. */
  const char *f1_cu = std::getenv("CELL_F1_CU");
  const bool f1_on = f1_cu && *f1_cu;
  const std::string f1_key = std::getenv("CELL_F1_KEY") ? std::getenv("CELL_F1_KEY") : "";
  const int f1_report_ms = static_cast<int>(envl("CELL_F1_REPORT_MS", 1000, 50, 60000));
  devourer::cell::f1::StagerConfig f1c;
  f1c.cu_timeout_ms = envl("CELL_F1_TIMEOUT_MS", 3000, 100, 600000);
  f1c.min_lead_sf = static_cast<uint64_t>(envl("CELL_F1_MIN_LEAD_SF", 8, 4, 10000));
  f1c.publish_lead_sf = static_cast<uint64_t>(envl("CELL_F1_PUBLISH_LEAD_SF", 5, 3, 10000));
  if (f1_on && !svc) {
    fprintf(stderr, "CELL_F1_CU needs the DU service mode (CELL_REF_PTP): the network "
                    "superframe count comes from the PTP clock\n");
    return 2;
  }
  if (f1_on && f1c.publish_lead_sf + 3 > f1c.min_lead_sf) {
    fprintf(stderr, "CELL_F1_MIN_LEAD_SF must exceed CELL_F1_PUBLISH_LEAD_SF by 3+\n");
    return 2;
  }
#if !defined(DEVOURER_CELL_OPENSSL)
  if (!f1_key.empty()) {
    fprintf(stderr, "CELL_F1_KEY needs a build with OpenSSL\n");
    return 2;
  }
#endif
  /* The CU moves this DU's UEs: without a key anyone on the LAN could. An
   * unauthenticated link is an explicit development choice. */
  if (f1_on && f1_key.empty() && envl("CELL_F1_INSECURE", 0, 0, 1) == 0) {
    fprintf(stderr, "CELL_F1_CU needs CELL_F1_KEY (or CELL_F1_INSECURE=1 for an "
                    "unauthenticated development link)\n");
    return 2;
  }
  sockaddr_storage f1_addr{};
  socklen_t f1_addr_len = 0;
  struct SockFd {
    int fd = -1;
    ~SockFd() {
      if (fd >= 0)
        close(fd);
    }
  } f1_fd;
  if (f1_on) {
    const std::string hp = f1_cu;
    const size_t colon = hp.rfind(':');
    addrinfo hints{}, *res = nullptr;
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    if (colon == std::string::npos ||
        getaddrinfo(hp.substr(0, colon).c_str(), hp.substr(colon + 1).c_str(), &hints, &res) != 0 ||
        !res) {
      fprintf(stderr, "CELL_F1_CU=\"%s\": want host:port\n", f1_cu);
      return 2;
    }
    std::memcpy(&f1_addr, res->ai_addr, res->ai_addrlen);
    f1_addr_len = static_cast<socklen_t>(res->ai_addrlen);
    f1_fd.fd = socket(res->ai_family, SOCK_DGRAM, 0);
    freeaddrinfo(res);
    if (f1_fd.fd < 0) {
      fprintf(stderr, "F1 socket failed\n");
      return 1;
    }
  }
  const CellMac f1_mac = hmac_sha1(f1_key);
  const char *ref_cmd = std::getenv("CELL_REF_CHECK");
  const int ref_timeout_ms =
      static_cast<int>(envl("CELL_REF_CHECK_TIMEOUT_MS", 1500, 50, 600000));
  if (svc && !(ref_cmd && *ref_cmd)) {
    /* A readable PHC is not proof of sync: it free-runs on after ptp4l
     * stops, and the loop would stay locked to it. */
    fprintf(stderr, "CELL_REF_PTP needs CELL_REF_CHECK (e.g. a pmc check that ptp4l is "
                    "SLAVE)\n");
    return 2;
  }
  devourer::cell::TsfDisciplineConfig dc;
  dc.interval_us = P;
  dc.phase_us = static_cast<double>(envl("CELL_PHASE_US", 0, 0, P - 1));
  devourer::cell::TsfDiscipline disc(dc);
  devourer::cell::SyncHealthConfig shc;
  shc.lock_us = envf("CELL_SYNC_LOCK_US", 10, 0.5, 10000);
  shc.lost_us = envf("CELL_SYNC_LOST_US", 200, shc.lock_us, 100000);
  shc.lock_iters = static_cast<int>(envl("CELL_SYNC_LOCK_ITERS", 8, 1, 1000));
  shc.lost_iters = static_cast<int>(envl("CELL_SYNC_LOST_ITERS", 20, 1, 1000));
  devourer::cell::SyncHealth health(shc);

  dev->InitWrite(SelectedChannel{c.ch, 0, CHANNEL_WIDTH_20});
  /* Coordinated: no carrier-sense deferral, a deterministic beacon departure
   * (the cell owns its grid slots). Autonomous: carrier sense and the random
   * beacon backoff, so a cell off the shared timebase defers to its
   * neighbours instead of colliding with their slots. A standalone cell
   * (no CELL_REF_PTP) owns its channel: always coordinated. */
  bool coordinated = !svc;
  auto apply_mode = [&](bool coord) {
    if (!dev->SetBeaconBackoff(!coord)) {
      fprintf(stderr, "SetBeaconBackoff(%d) refused\n", coord ? 0 : 1);
      return false;
    }
    dev->SetCcaMode(coord);
    return true;
  };
  if (!apply_mode(coordinated))
    return 1;

  /* The UE pool over the capacity grid's fixed positions; static UEs are
   * bound first (positions 1..n == their ids, du_budget_ok checks the list),
   * joiners take the rest through the RACH. */
  std::vector<std::pair<uint8_t, uint8_t>> positions;
  for (size_t i = 1; i <= c.capacity; ++i)
    positions.push_back({static_cast<uint8_t>(*grid->find(SlotKind::Downlink, static_cast<uint16_t>(i))),
                         static_cast<uint8_t>(*grid->find(SlotKind::Uplink, static_cast<uint16_t>(i)))});
  devourer::cell::UeRegistry reg(positions);
  const bool rach_on = c.rach_us > 0;
  devourer::cell::RachServer srv(c.cell, reg, rach_mac(c));
  const auto rach_slot = grid->find(SlotKind::Rach, 0);
  GrantMap map;
  GrantMap prev_map; /* the map before the last publish (in force until map's effective_sf) */
  map.cell_id = c.cell;
  map.flags = coordinated ? 0 : devourer::cell::kGrantAutonomous;
  map.epoch = 1;
  map.effective_sf = 0;
  map.period_tu = static_cast<uint16_t>(c.tu);
  auto grant_for = [&](const devourer::cell::UeSlot &u) {
    Grant g;
    g.ue = u.ue;
    g.dl_slot = u.dl_slot;
    g.ul_slot = u.ul_slot;
    g.rate_mbps = static_cast<uint8_t>(LinkAdapt({}, dl_rate0).rate_mbps());
    return g;
  };
  for (uint16_t ue : ues) {
    std::array<uint8_t, 6> m{};
    cell_mac(m.data(), c.cell, static_cast<uint8_t>(ue));
    map.grants.push_back(grant_for(*reg.bind(m)));
  }
  /* A UE's id is its identity; its slot pair (capacity position) is the
   * grant's, which the CU (F1) may move. Positions are 1..capacity. */
  auto position_of = [&](const Grant &g) -> uint8_t {
    return g.dl_slot < grid->slots().size() ? static_cast<uint8_t>(grid->slots()[g.dl_slot].ue) : 0;
  };
  auto set_position = [&](Grant &g, uint8_t pos) {
    g.dl_slot = static_cast<uint8_t>(*grid->find(SlotKind::Downlink, pos));
    g.ul_slot = static_cast<uint8_t>(*grid->find(SlotKind::Uplink, pos));
  };
  /* A position no grant of `m` uses (0: none left). */
  auto free_position = [&](const GrantMap &m) -> uint8_t {
    for (size_t p = 1; p <= c.capacity; ++p) {
      bool used = false;
      for (const Grant &g : m.grants)
        used = used || position_of(g) == p;
      if (!used)
        return static_cast<uint8_t>(p);
    }
    return 0;
  };
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
  /* The RX timestamp mapping, one snapshot: a free-running reference near
   * now and DU tsf - freerun (constant between TSF writes, which only the
   * DU service mode's discipline makes). Published together from one radio
   * read: every superframe, and after each step. `valid` is false while a
   * step's read-back has failed, and RX then attributes nothing. */
  struct ClockSnap {
    int64_t fr_ref = 0, tsf_minus_fr = 0;
    bool valid = false;
  };
  std::mutex clk_mu;
  ClockSnap clk{static_cast<int64_t>(f0), static_cast<int64_t>(t0) - static_cast<int64_t>(f0), true};
  auto publish_clock = [&](uint64_t t, uint64_t f) {
    std::lock_guard<std::mutex> lk(clk_mu);
    clk = {static_cast<int64_t>(f), static_cast<int64_t>(t) - static_cast<int64_t>(f), true};
  };
  /* An RX tsfl (32-bit free-running stamp) as DU tsf; nullopt while stale. */
  auto rx_tsf = [&](uint32_t tsfl) -> std::optional<int64_t> {
    std::lock_guard<std::mutex> lk(clk_mu);
    if (!clk.valid)
      return std::nullopt;
    return extend32_near(tsfl, clk.fr_ref) + clk.tsf_minus_fr;
  };

  /* The reference check: a command, off the radio path (it may take tens of
   * ms) and bounded by CELL_REF_CHECK_TIMEOUT_MS, its verdict an atomic the
   * discipline reads. */
  std::atomic<bool> ref_ok{true};
  std::atomic<bool> ref_stop{false};
  auto run_ref_check = [&] {
    if (svc)
      ref_ok.store(run_check(ref_cmd, ref_timeout_ms, ref_stop));
  };
  run_ref_check();
  std::thread ref_thr;
  if (svc)
    ref_thr = std::thread([&] {
      while (!ref_stop.load()) {
        for (int i = 0; i < ref_check_s * 10 && !ref_stop.load(); ++i)
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!ref_stop.load())
          run_ref_check();
      }
    });
  struct Joiner {
    std::atomic<bool> &stop;
    std::thread &t;
    ~Joiner() {
      stop.store(true);
      if (t.joinable())
        t.join();
    }
  } ref_join{ref_stop, ref_thr};

  /* One discipline iteration: a PHC-bracketed radio read, the step, the
   * health verdict. Runs on the DU's main thread (the only TSF writer). */
  std::atomic<int64_t> net_off{0}; /* network sf - local sf, from the PHC */
  std::atomic<bool> net_ok{false};
  uint64_t sync_samples = 0;
  std::array<uint64_t, 3> sync_in_state{};
  auto sync_once = [&](int64_t sf_now, bool beacon_ok) {
    uint64_t t = 0, f = 0;
    double r1 = 0, r2 = 0;
    const bool p1 = phc_us(refclk, r1);
    const bool rd = dev->ReadTsfAndFreeRun(t, f);
    const bool p2 = phc_us(refclk, r2);
    /* A failed PHC read is a rejected sample and an unhealthy reference. */
    const auto d = disc.sample(r1, r2, rd && p1 && p2, static_cast<int64_t>(t),
                               static_cast<int64_t>(f));
    bool wok = true;
    if (d.step_us != 0) {
      wok = dev->WriteTsf(dev->ReadTsf() + static_cast<uint64_t>(d.write_us));
      disc.wrote(wok);
      /* After any write attempt the RX offset must be re-read before it is
       * trusted again: a failed read-back is a timing failure too (Lost),
       * and RX attributes nothing until the next superframe's refresh. */
      uint64_t t2 = 0, f2 = 0;
      if (dev->ReadTsfAndFreeRun(t2, f2)) {
        publish_clock(t2, f2);
      } else {
        std::lock_guard<std::mutex> lk(clk_mu);
        clk.valid = false;
        wok = false;
      }
    }
    const bool rok = ref_ok.load() && p1 && p2;
    /* The network superframe: floor((PTP - phase) / P) at every DU. While
     * the grid is locked to the phase, (PTP - phase) - TSF is a whole
     * number of superframes; rounding absorbs the us of residual. */
    if (d.accepted && d.acting && rok && std::fabs(d.phase_err_us) < P / 4.0) {
      const double r_mid = 0.5 * (r1 + r2);
      const double k = (r_mid - dc.phase_us - static_cast<double>(t)) / static_cast<double>(P);
      net_off.store(static_cast<int64_t>(std::llround(k)));
      net_ok.store(true);
    }
    const auto st = health.update(d, rok, beacon_ok, wok);
    /* The conversion is trusted only while the grid is held to the fabric:
     * once Lost (reference, write or phase failure) it is re-derived from a
     * fresh good sample before any network instant is used again. */
    if (st == devourer::cell::SyncState::Lost)
      net_ok.store(false);
    ++sync_samples;
    ++sync_in_state[static_cast<size_t>(st)];
    char l[300];
    snprintf(l, sizeof l,
             "{\"ev\":\"cell.sync\",\"sf\":%lld,\"state\":\"%s\",\"accepted\":%d,"
             "\"phase_us\":%.1f,\"step_us\":%lld,\"write_ok\":%d,\"ppm\":%.2f,"
             "\"ref_ok\":%d,\"beacon_ok\":%d}",
             (long long)sf_now, devourer::cell::sync_state_name(st), d.acting ? 1 : 0,
             d.phase_err_us, (long long)d.step_us, wok ? 1 : 0, d.ppm, rok ? 1 : 0,
             beacon_ok ? 1 : 0);
    emit(l);
  };
  auto set_mode = [&](bool coord, int64_t sf_now) {
    if (coord == coordinated || !apply_mode(coord))
      return;
    coordinated = coord;
    char l[160];
    snprintf(l, sizeof l, "{\"ev\":\"cell.mode\",\"sf\":%lld,\"mode\":\"%s\"}",
             (long long)sf_now, coord ? "coordinated" : "autonomous");
    emit(l);
  };
  if (svc) {
    /* Acquire before serving: the first steps can move the grid by up to half
     * a superframe, which no UE should see mid-schedule. */
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(acquire_s);
    while (!health.coordinated() && std::chrono::steady_clock::now() < until) {
      sync_once(-1, true);
      std::this_thread::sleep_for(std::chrono::milliseconds(sync_loop_ms));
    }
    /* The DU stays autonomous until the map clearing the flag is in force
     * (the main loop's mode rule). */
    char l[160];
    snprintf(l, sizeof l, "{\"ev\":\"cell.sync.acquire\",\"locked\":%d,\"samples\":%llu}",
             health.coordinated() ? 1 : 0, (unsigned long long)sync_samples);
    emit(l);
  }

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
  std::vector<uint8_t> rar_on_air;                 /* the RACH answers in the live beacon */
  std::vector<devourer::cell::RachResponse> rar_on_air_set; /* ... as answers, for accounting */
  std::map<uint16_t, int64_t> last_ul_sf;          /* per UE: superframe of its last uplink (mu) */
  std::map<uint16_t, int64_t> heard_ul_sf;         /* ... only ever set by an uplink heard (mu) */
  int beacon_fail_run = 0;                         /* consecutive failed beacon updates */
  /* Per UE, the UL slot of the grant each live map gives it, keyed by the
   * map's epoch (a UE reports the epoch it runs): the TA target (mu). */
  struct UlSlots {
    uint32_t epoch = 0;
    std::map<uint16_t, uint8_t> ul;
  };
  UlSlots ul_now, ul_prev;
  auto ul_slots_of = [&](const GrantMap &m) {
    UlSlots u;
    u.epoch = m.epoch;
    for (const Grant &g : m.grants)
      u.ul[g.ue] = g.ul_slot;
    return u;
  };
  ul_now = ul_slots_of(map);

  /* --- F1 (M5): shared between the main loop and the F1 thread (f1_mu). --- */
  std::mutex f1_mu;
  devourer::cell::f1::Stager stager(f1c);
  struct F1View { /* what the main loop last published for reports/validation */
    int64_t net_sf = -1; /* the network superframe now; -1 = not known yet */
    uint8_t sync_state = 0;
    bool coordinated = false;
    std::vector<devourer::cell::CoordAssign> ues;
  } f1_view;
  struct Heard {
    double rssi = 0;
    std::chrono::steady_clock::time_point seen;
  };
  std::map<std::pair<uint16_t, uint16_t>, Heard> heard; /* (cell, ue) -> (f1_mu) */
  devourer::cell::CoordDu layout;
  devourer::cell::coord_layout(*grid, layout);
  auto now_ms = [] {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  };

  char line[384];
  snprintf(line, sizeof line,
           "{\"ev\":\"cell.du.start\",\"cell\":%u,\"period_us\":%lld,\"ues\":%zu,\"ch\":%u,"
           "\"tsf_minus_fr\":%lld,\"tier\":\"%s\",\"svc\":%d}",
           c.cell, (long long)P, ues.size(), c.ch,
           (long long)(static_cast<int64_t>(t0) - static_cast<int64_t>(f0)),
           devourer::sync_tier_name(tier), svc ? 1 : 0);
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
      if (rach_on) {
        const size_t mlen = p.Data.size() - (p.RxAtrib.fcs_present && p.Data.size() >= 4 ? 4 : 0);
        if (auto rf = parse_rach_frame(p.Data.data(), mlen, c.cell)) {
          const auto at_o = rx_tsf(p.RxAtrib.tsfl);
          if (!at_o)
            return;
          const auto pos = grid->at(*at_o);
          std::optional<devourer::cell::RachResponse> a;
          /* Only a request that arrived inside the RACH slot is considered:
           * one heard in a data slot or idle air allocates nothing. */
          const bool in_rach = rach_slot && pos.slot && *pos.slot == *rach_slot;
          if (in_rach) {
            std::lock_guard<std::mutex> lk(mu);
            a = srv.on_request(rf->msga, rf->msga_len, static_cast<uint64_t>(pos.superframe));
          }
          char l[240];
          snprintf(l, sizeof l,
                   "{\"ev\":\"cell.rach\",\"sf\":%lld,\"phase_us\":%lld,\"in_slot\":%d,"
                   "\"joiner\":\"%02x:%02x:%02x:%02x:%02x:%02x\",\"answered\":%d,\"status\":%d,"
                   "\"ue\":%u}",
                   (long long)pos.superframe, (long long)pos.into_us,
                   in_rach ? 1 : 0, rf->joiner[0],
                   rf->joiner[1], rf->joiner[2], rf->joiner[3], rf->joiner[4], rf->joiner[5],
                   a ? 1 : 0, a ? static_cast<int>(a->status) : -1, a ? a->ue : 0);
          emit(l);
          return;
        }
      }
      /* Another cell's UE (its uplink beacon, SA 02:ce:11:<cell>:<ue>): what
       * F1 reports as heard, for the CU's interference relation. */
      if (f1_on && p.Data.size() >= 16 && p.Data[0] == 0x80 && p.Data[10] == 0x02 &&
          p.Data[11] == 0xce && p.Data[12] == 0x11) {
        const uint16_t oc = static_cast<uint16_t>(p.Data[13] << 8 | p.Data[14]);
        const uint16_t ou = p.Data[15];
        if (oc != c.cell && ou >= 1) {
          const double r = static_cast<double>(p.RxAtrib.rssi[0]) - 110.0;
          std::lock_guard<std::mutex> lk(f1_mu);
          auto &h = heard[{oc, ou}];
          const bool fresh = h.seen.time_since_epoch().count() == 0;
          h.rssi = fresh ? r : 0.8 * h.rssi + 0.2 * r;
          h.seen = std::chrono::steady_clock::now();
          return;
        }
      }
      const auto ub = parse_ue_beacon(p.Data.data(), p.Data.size(), c.cell, auth);
      if (!ub)
        return;
      /* The UL slot of the grant the UE runs (by the epoch it reports). */
      std::optional<size_t> slot;
      {
        std::lock_guard<std::mutex> lk(mu);
        /* Only an epoch the DU still knows: an uplink from an older map (a UE
         * that missed updates) is not timed against a slot it did not use. */
        const UlSlots *u = ub->epoch == ul_now.epoch    ? &ul_now
                           : ub->epoch == ul_prev.epoch ? &ul_prev
                                                        : nullptr;
        if (u) {
          auto it = u->ul.find(ub->ue);
          if (it != u->ul.end())
            slot = it->second;
        }
      }
      if (!slot)
        return;
      const auto at_o = rx_tsf(p.RxAtrib.tsfl);  // DU tsf at RX
      if (!at_o)
        return;
      const auto pos = grid->at(*at_o);
      const int64_t target = grid->slots()[*slot].start_us + c.ul_target_us;
      const int64_t err = wrap_phase(pos.into_us - target, P);
      int64_t ta_now = 0;
      {
        std::lock_guard<std::mutex> lk(mu);
        auto it = ta.find(ub->ue);
        if (it != ta.end()) {
          if (ta_on) /* CELL_TA=0: no loop to feed */
            it->second.sample(err);
          ta_now = it->second.ta();
        }
        ue_epoch[ub->ue] = ub->epoch;
        last_ul_sf[ub->ue] = pos.superframe;
        heard_ul_sf[ub->ue] = pos.superframe;
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
  auto next_sync = std::chrono::steady_clock::now() + std::chrono::milliseconds(sync_loop_ms);
  int64_t sf = grid->at(static_cast<int64_t>(dev->ReadTsf())).superframe + 1;
  bool cu_alive_prev = !f1_on;
  /* The F1 thread: reports out, schedules in (staged under f1_mu), acks. */
  std::atomic<bool> f1_stop{false};
  std::thread f1_thr;
  if (f1_on)
    f1_thr = std::thread([&] {
      namespace f1 = devourer::cell::f1;
      auto send = [&](const std::vector<uint8_t> &m) {
        if (!m.empty())
          sendto(f1_fd.fd, m.data(), m.size(), 0, reinterpret_cast<const sockaddr *>(&f1_addr),
                 f1_addr_len);
      };
      auto next_report = std::chrono::steady_clock::now();
      uint64_t report_seq = 0;
      while (!f1_stop.load()) {
        if (std::chrono::steady_clock::now() >= next_report) {
          next_report += std::chrono::milliseconds(f1_report_ms);
          f1::Report r;
          {
            std::lock_guard<std::mutex> lk(f1_mu);
            r.du = c.cell;
            r.tier = static_cast<uint8_t>(tier);
            r.sync_state = f1_view.sync_state;
            r.coordinated = f1_view.coordinated;
            r.channel = c.ch;
            r.phase_us = static_cast<int64_t>(dc.phase_us);
            r.period_us = P;
            r.net_sf = f1_view.net_sf >= 0 ? static_cast<uint64_t>(f1_view.net_sf) : 0;
            const uint64_t rt = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count());
            report_seq = std::max(report_seq + 1, rt);
            r.seq = report_seq;
            r.applied_version = stager.applied_version();
            r.staged_version = stager.staged_version();
            r.fixed = layout.fixed;
            r.positions = layout.positions;
            r.ues = f1_view.ues;
            const auto now = std::chrono::steady_clock::now();
            for (auto it = heard.begin(); it != heard.end();) {
              if (now - it->second.seen > std::chrono::seconds(10)) { /* gone */
                it = heard.erase(it);
                continue;
              }
              if (r.heard.size() < f1::kMaxHeard)
                r.heard.push_back({it->first.first, it->first.second,
                                   static_cast<int8_t>(std::lround(it->second.rssi))});
              ++it;
            }
            if (f1_view.net_sf < 0) /* no network superframe yet: nothing to plan on */
              r.ues.clear();
          }
          send(f1::encode_report(r, f1_mac));
        }
        pollfd pf{f1_fd.fd, POLLIN, 0};
        if (poll(&pf, 1, 50) <= 0 || !(pf.revents & POLLIN))
          continue;
        uint8_t buf[2048];
        sockaddr_storage from{};
        socklen_t fl = sizeof from;
        const ssize_t n = recvfrom(f1_fd.fd, buf, sizeof buf, 0, reinterpret_cast<sockaddr *>(&from), &fl);
        if (n <= 0)
          continue;
        /* Only the configured CU's address (the tag and freshness come next). */
        auto same_peer = [&] {
          if (from.ss_family != f1_addr.ss_family)
            return false;
          if (from.ss_family == AF_INET) {
            const auto &a = reinterpret_cast<const sockaddr_in &>(from);
            const auto &b = reinterpret_cast<const sockaddr_in &>(f1_addr);
            return a.sin_port == b.sin_port && a.sin_addr.s_addr == b.sin_addr.s_addr;
          }
          if (from.ss_family == AF_INET6) {
            const auto &a = reinterpret_cast<const sockaddr_in6 &>(from);
            const auto &b = reinterpret_cast<const sockaddr_in6 &>(f1_addr);
            return a.sin6_port == b.sin6_port &&
                   std::memcmp(&a.sin6_addr, &b.sin6_addr, sizeof a.sin6_addr) == 0;
          }
          return false;
        };
        if (!same_peer())
          continue;
        auto s = f1::decode_schedule(buf, static_cast<size_t>(n), f1_mac);
        if (!s || s->du != c.cell)
          continue;
        f1::AckStatus st;
        int64_t net_now = -1;
        {
          std::lock_guard<std::mutex> lk(f1_mu);
          net_now = f1_view.net_sf;
          /* Carryable: exactly the UEs of the live map, each in a distinct
           * position of this grid. */
          auto valid = [&](const f1::Schedule &x) {
            if (x.ues.size() != f1_view.ues.size())
              return false;
            std::set<uint8_t> seen_pos;
            std::set<uint16_t> seen_ue;
            for (const auto &a : x.ues) {
              bool known = false;
              for (const auto &u : f1_view.ues)
                known = known || u.ue == a.ue;
              if (!known || !seen_ue.insert(a.ue).second || a.pos < 1 || a.pos > c.capacity ||
                  !seen_pos.insert(a.pos).second)
                return false;
            }
            return seen_ue.size() == f1_view.ues.size(); /* one-to-one, every live UE */
          };
          /* Without a network superframe (not yet locked, or Lost) a NEW
           * schedule cannot be timed: Late. The keepalive of the version the
           * DU already runs or stages is still answered (and is CU contact);
           * its instant is not re-checked against a clock it does not have. */
          const bool known = s->version == stager.applied_version() ||
                             s->version == stager.staged_version();
          if (net_now >= 0)
            st = stager.on_schedule(*s, static_cast<uint64_t>(net_now), now_ms(), valid);
          else if (known && s->version != 0)
            st = stager.on_schedule(*s, 0, now_ms(), valid);
          else
            st = f1::AckStatus::Late;
        }
        send(f1::encode_ack({c.cell, s->version, st, net_now < 0 ? 0 : static_cast<uint64_t>(net_now)},
                            f1_mac));
        if (st != f1::AckStatus::Applied && st != f1::AckStatus::Staged) {
          char l[200];
          snprintf(l, sizeof l,
                   "{\"ev\":\"cell.f1.reject\",\"version\":%u,\"effective_net_sf\":%llu,"
                   "\"net_sf\":%lld,\"status\":\"%s\"}",
                   s->version, (unsigned long long)s->effective_net_sf, (long long)net_now,
                   f1::ack_status_name(st));
          emit(l);
        }
      }
    });
  struct F1Joiner {
    std::atomic<bool> &stop;
    std::thread &t;
    ~F1Joiner() {
      stop.store(true);
      if (t.joinable())
        t.join();
    }
  } f1_join{f1_stop, f1_thr};
  while (std::chrono::steady_clock::now() < deadline) {
    /* Serve the UEs of the map IN FORCE: a newly admitted UE starts
     * receiving DL only from its grant's effective superframe, when it
     * applies that map too. */
    const GrantMap &serving =
        static_cast<uint64_t>(sf) >= map.effective_sf || prev_map.grants.empty() ? map : prev_map;
    std::vector<uint16_t> served;
    for (const Grant &g : serving.grants)
      served.push_back(g.ue);
    for (uint16_t ue : served) {
      const size_t idx = serving.find(ue)->dl_slot;
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
      publish_clock(t, f);
    if (svc && std::chrono::steady_clock::now() >= next_sync) {
      next_sync = std::chrono::steady_clock::now() + std::chrono::milliseconds(sync_loop_ms);
      /* The beacon-engine proxy (no beacon TX counter on this path): the
       * newest uplink of the UEs in the live map that were ever heard. */
      bool beacon_ok = beacon_fail_run < 3;
      {
        std::lock_guard<std::mutex> lk(mu);
        int64_t newest = -1;
        for (const Grant &g : map.grants) {
          auto h = heard_ul_sf.find(g.ue);
          if (h != heard_ul_sf.end())
            newest = std::max(newest, h->second);
        }
        if (newest >= 0 && sf - newest > sync_ul_sf)
          beacon_ok = false;
      }
      sync_once(sf, beacon_ok);
    }
    /* The DU's own access mode. Suspending is immediate: the desynced DU
     * is the side that must defer (its UEs follow ~3 superframes later,
     * when the flagged map is in force). Resuming waits until the map
     * telling the UEs so is on air AND in force, so the DU is never
     * deterministic while its UEs are told to defer, or vice versa. */
    /* F1: the view the reports and the schedule validator read, the CU
     * contact, and a staged schedule that is due. */
    bool cu_alive = true;
    std::optional<devourer::cell::f1::Schedule> f1_due;
    if (f1_on) {
      std::lock_guard<std::mutex> lk(f1_mu);
      f1_view.net_sf = net_ok.load() && health.state() != devourer::cell::SyncState::Lost
                           ? sf + net_off.load()
                           : -1;
      f1_view.sync_state = static_cast<uint8_t>(health.state());
      f1_view.coordinated = coordinated;
      f1_view.ues.clear();
      for (const Grant &g : map.grants)
        f1_view.ues.push_back({g.ue, position_of(g)});
      cu_alive = stager.cu_alive(now_ms());
      if (f1_view.net_sf >= 0)
        f1_due = stager.due(static_cast<uint64_t>(f1_view.net_sf));
      if (auto ab = stager.take_abandoned()) {
        char l[160];
        snprintf(l, sizeof l, "{\"ev\":\"cell.f1.abandoned\",\"version\":%u,\"sf\":%lld}",
                 ab->version, (long long)sf);
        emit(l);
      }
    }
    if (f1_on && cu_alive != cu_alive_prev) {
      cu_alive_prev = cu_alive;
      char l[120];
      snprintf(l, sizeof l, "{\"ev\":\"cell.f1.cu\",\"sf\":%lld,\"alive\":%d}", (long long)sf,
               cu_alive ? 1 : 0);
      emit(l);
    }
    /* Coordinated scheduling needs the fabric (M4) and, with F1, the CU. */
    const bool may_coordinate = health.coordinated() && cu_alive;
    if (svc) {
      /* The map in force (as the UEs see it): the live one from its
       * effective superframe, the previous one until then. */
      const GrantMap &in_force = static_cast<uint64_t>(sf) >= map.effective_sf ? map : prev_map;
      set_mode(may_coordinate && in_force.flags == 0, sf);
    }
    /* Stage the next map; it replaces `map` only once it is on air. */
    GrantMap next = map;
    bool changed = false;
    /* The mode the UEs must follow: the health's verdict (retried while a
     * publish fails). */
    const uint8_t want_flags =
        !svc || may_coordinate ? 0 : devourer::cell::kGrantAutonomous;
    if (next.flags != want_flags) {
      next.flags = want_flags;
      changed = true;
    }
    std::vector<uint8_t> rar_now;
    std::vector<devourer::cell::RachResponse> rar_sel;
    std::vector<uint16_t> to_release;
    {
      std::lock_guard<std::mutex> lk(mu);
      /* Joiners the RACH server admitted: a grant, and their per-UE state. */
      for (const auto &u : reg.admitted())
        if (!next.find(u.ue)) {
          Grant g = grant_for(u);
          /* Its default slot pair may be another UE's (the CU moved that one
           * there): then the first free one; the CU re-plans it. */
          for (const Grant &o : next.grants)
            if (position_of(o) == position_of(g)) {
              if (const uint8_t fp = free_position(next))
                set_position(g, fp);
              break;
            }
          next.grants.push_back(g);
          ta.emplace(u.ue, TaLoop(ta_gain, P / 4));
          dlue.emplace(u.ue, DlUe{HarqSender(hc), LinkAdapt({}, dl_rate0)});
          last_ul_sf[u.ue] = sf; /* its silence clock starts at admission */
          changed = true;
        }
      /* Departure: a JOINED UE (static ones stay) whose uplink has been
       * absent release_sf superframes gives its position back. */
      for (auto it = next.grants.begin(); it != next.grants.end();) {
        const uint16_t ue = it->ue;
        const bool is_static = std::find(ues.begin(), ues.end(), ue) != ues.end();
        auto ls = last_ul_sf.find(ue);
        if (!is_static && ls != last_ul_sf.end() && sf - ls->second > release_sf) {
          /* dropped from the staged map now; its registry position is freed
           * only once that map is on air (below), so a new joiner can never
           * be granted slots the live map still gives the old UE */
          to_release.push_back(ue);
          it = next.grants.erase(it);
          changed = true;
          continue;
        }
        ++it;
      }
      if (rach_on) {
        rar_sel = srv.select(static_cast<uint64_t>(sf + 1));
        rar_now = devourer::cell::encode_rach_responses(c.cell, rar_sel, rach_mac(c));
      }
    }
    if (ta_on && std::chrono::steady_clock::now() >= next_ta) {
      next_ta += std::chrono::seconds(ta_every_s);
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
    }
    /* A due F1 schedule: every UE's position, on its network-wide instant. */
    std::optional<uint64_t> f1_eff;
    if (f1_due) {
      /* Re-validated against `next` as it is now: a UE admitted since the
       * schedule was staged (the schedule does not name it) moves off any
       * pair the schedule takes; if none is free, the schedule is not
       * applied (it is abandoned at its deadline and the CU re-plans). */
      GrantMap trial = next;
      for (const auto &a : f1_due->ues)
        if (Grant *g = const_cast<Grant *>(trial.find(a.ue)))
          set_position(*g, a.pos);
      bool fits = true;
      for (Grant &g : trial.grants) {
        bool named = false;
        for (const auto &a : f1_due->ues)
          named = named || a.ue == g.ue;
        if (named)
          continue;
        bool clash = false;
        for (const Grant &o : trial.grants)
          clash = clash || (o.ue != g.ue && position_of(o) == position_of(g));
        if (clash) {
          Grant self = g;
          g.dl_slot = 0xff; /* not counted by free_position */
          const uint8_t fp = free_position(trial);
          g = self;
          if (fp == 0)
            fits = false;
          else
            set_position(g, fp);
        }
      }
      if (fits) {
        next = trial;
        f1_eff =
            static_cast<uint64_t>(static_cast<int64_t>(f1_due->effective_net_sf) - net_off.load());
        changed = true;
      } else {
        char l[160];
        snprintf(l, sizeof l, "{\"ev\":\"cell.f1.unfit\",\"version\":%u,\"sf\":%lld}",
                 f1_due->version, (long long)sf);
        emit(l);
        f1_due.reset();
      }
    }
    /* One pending map at a time: while the live map is not yet in force
     * (its effective superframe ahead), a new change waits, so no
     * republish can move an instant already announced (an F1 schedule's
     * above all). The change is recomputed next pass; the TA loops go
     * back to the live map's values, as after a failed publish. */
    if (changed && static_cast<uint64_t>(sf) < map.effective_sf) {
      std::lock_guard<std::mutex> lk(mu);
      for (const Grant &g : map.grants)
        ta.at(g.ue).set_ta(g.ta_us);
      next = map;
      changed = false;
      to_release.clear();
      f1_eff.reset();
    }
    /* The deadline, checked again right before the publish: a schedule
     * that can no longer reach the UEs N+2 ahead is not put on air. */
    if (f1_eff && static_cast<uint64_t>(sf) + 2 > *f1_eff) {
      next = map;
      changed = false;
      f1_eff.reset();
    }
    if (changed || rar_now != rar_on_air) {
      if (changed) {
        ++next.epoch;
        next.effective_sf = f1_eff ? *f1_eff : static_cast<uint64_t>(sf + 3);
      }
      const auto nb = build_du_beacon(next, rar_now);
      const bool ok = !nb.empty() && dev->UpdateBeaconPayload(nb.data(), nb.size());
      beacon_fail_run = ok ? 0 : beacon_fail_run + 1;
      if (ok) {
        rar_on_air = rar_now;
        rar_on_air_set = rar_sel;
      }
      if (changed) {
        if (ok) {
          for (const Grant &g : next.grants) {
            const Grant *was = map.find(g.ue);
            if (!was || was->dl_slot != g.dl_slot || was->ul_slot != g.ul_slot) {
              char l[200];
              snprintf(l, sizeof l,
                       "{\"ev\":\"cell.assign\",\"ue\":%u,\"pos\":%u,\"dl_slot\":%u,"
                       "\"ul_slot\":%u,\"epoch\":%u,\"effective_sf\":%llu}",
                       g.ue, position_of(g), g.dl_slot, g.ul_slot, next.epoch,
                       (unsigned long long)next.effective_sf);
              emit(l);
            }
          }
          prev_map = map;
          map = next;
          if (f1_eff) {
            std::lock_guard<std::mutex> lk(f1_mu);
            /* due() released it this pass and the deadline was checked just
             * before the publish, so this holds; a false here (it cannot
             * be) is logged, not reported as applied. */
            if (!stager.published(static_cast<uint64_t>(sf + net_off.load()))) {
              emit("{\"ev\":\"cell.f1.publish_late\"}");
              f1_eff.reset();
            }
          }
          if (f1_eff) {
            char l[200];
            snprintf(l, sizeof l,
                     "{\"ev\":\"cell.f1.applied\",\"version\":%u,\"effective_net_sf\":%llu,"
                     "\"effective_sf\":%llu,\"sf\":%lld}",
                     f1_due->version, (unsigned long long)f1_due->effective_net_sf,
                     (unsigned long long)*f1_eff, (long long)sf);
            emit(l);
          }
          std::lock_guard<std::mutex> lk(mu);
          ul_prev = ul_now;
          ul_now = ul_slots_of(map);
          for (uint16_t ue : to_release) {
            reg.release_id(ue);
            last_ul_sf.erase(ue);
            heard_ul_sf.erase(ue);
            char l[120];
            snprintf(l, sizeof l, "{\"ev\":\"cell.release\",\"ue\":%u,\"sf\":%lld}", ue,
                     (long long)sf);
            emit(l);
          }
        } else {
          /* A release in `next` is retried next pass (its silence clock still
           * runs). The old map stays on air: keep its epoch (UEs keep reporting it,
           * so sampling continues) and put the loops back on its TAs; the
           * next TA pass retries. */
          std::lock_guard<std::mutex> lk(mu);
          for (const Grant &g : map.grants)
            ta.at(g.ue).set_ta(g.ta_us);
        }
        const GrantMap &shown = ok ? map : next;
        char l[320];
        int n = snprintf(l, sizeof l, "{\"ev\":\"cell.grant\",\"epoch\":%u,\"effective_sf\":%llu,"
                                      "\"ok\":%d,\"flags\":%u,\"ta\":{",
                         shown.epoch, (unsigned long long)shown.effective_sf, ok ? 1 : 0,
                         shown.flags);
        for (size_t i = 0; i < shown.grants.size() && n > 0 && n < (int)sizeof l; ++i)
          n += snprintf(l + n, sizeof l - n, "%s\"%u\":[%d,%u]", i ? "," : "", shown.grants[i].ue,
                        shown.grants[i].ta_us, shown.grants[i].rate_mbps);
        if (n > 0 && n < (int)sizeof l)
          snprintf(l + n, sizeof l - n, "}}");
        emit(l);
      }
    }
    /* The answers on air this superframe (after a failed update: the
     * previous set, still airing) count one publication. */
    if (rach_on && !rar_on_air_set.empty()) {
      std::lock_guard<std::mutex> lk(mu);
      srv.published(rar_on_air_set);
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
  if (svc) {
    char l[240];
    snprintf(l, sizeof l,
             "{\"ev\":\"cell.sync.summary\",\"samples\":%llu,\"rejected\":%llu,"
             "\"locked\":%llu,\"degraded\":%llu,\"lost\":%llu,\"transitions\":%u}",
             (unsigned long long)sync_samples, (unsigned long long)disc.rejected(),
             (unsigned long long)sync_in_state[2], (unsigned long long)sync_in_state[1],
             (unsigned long long)sync_in_state[0], health.transitions());
    emit(l);
  }
  emit("{\"ev\":\"cell.du.done\"}");
  return 0;
}

// ---------------------------------------------------------------------------
int run_ue(IRadio *dev, const Cfg &c) {
  /* CELL_UE=0: a joining UE, which gets its id through the RACH. */
  std::atomic<int> me_a{static_cast<int>(envl("CELL_UE", 0, 0, 254))};
  if (me_a == 0 && (c.rach_us == 0 || !rach_ok(c))) {
    fprintf(stderr, "CELL_UE=0 (join) needs CELL_RACH_US and CELL_PSK\n");
    return 2;
  }
  dev->InitWrite(SelectedChannel{c.ch, 0, CHANNEL_WIDTH_20});
  auto arm_ack = [&](int ue) {
    devourer::MacAddr own;
    cell_mac(own.bytes.data(), c.cell, static_cast<uint8_t>(ue));
    if (dev->SetAckResponder(own))
      return true;
    fprintf(stderr, "SetAckResponder refused (ack_responder_ok=%d)\n",
            dev->GetAdapterCaps().ack_responder_ok ? 1 : 0);
    return false;
  };
  if (me_a > 0 && !arm_ack(me_a))
    return 1;
  if (!dev->SetBeaconBackoff(false)) {
    fprintf(stderr, "SetBeaconBackoff(false) refused\n");
    return 1;
  }
  dev->SetCcaMode(true);
  bool ue_autonomous = false; /* the DU's kGrantAutonomous, followed */

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
  /* The capacity grid (same env as the DU): positions never move. */
  const std::optional<SlotGrid> grid = grid_for(c);
  if (!grid) {
    fprintf(stderr, "the grid does not fit a %d TU superframe\n", c.tu);
    return 2;
  }
  uint8_t du_mac[6];
  cell_mac(du_mac, c.cell, 0);
  /* RACH (joining UE): its own random MAC + nonce, the client, and the
   * answers the RX thread finds in DU beacons (mu). */
  std::random_device rd;
  std::mt19937 rng((static_cast<uint64_t>(rd()) << 32) ^ rd());
  /* A STABLE joiner identity: a restarted joiner re-joins into the position
   * it already holds (UeRegistry is idempotent per MAC) instead of taking a
   * second one. CELL_JOINER_MAC, else derived from host name + radio. */
  std::array<uint8_t, 6> joiner_mac{};
  if (const char *jm = std::getenv("CELL_JOINER_MAC")) {
    const auto pm = devourer::parse_mac(jm);
    if (!pm) {
      fprintf(stderr, "invalid CELL_JOINER_MAC=\"%s\"\n", jm);
      return 2;
    }
    joiner_mac = pm->bytes;
  } else {
    char host[256] = {0};
    gethostname(host, sizeof host - 1);
    const char *bdf = std::getenv("DEVOURER_PCIE_BDF");
    uint64_t h = 1469598103934665603ull;
    for (const char *q : {static_cast<const char *>(host), bdf ? bdf : ""})
      for (; *q; ++q)
        h = (h ^ static_cast<uint8_t>(*q)) * 1099511628211ull;
    joiner_mac = {0x02, 0xce, 0xfe, static_cast<uint8_t>(h >> 16), static_cast<uint8_t>(h >> 8),
                  static_cast<uint8_t>(h)};
  }
  const uint64_t nonce = (static_cast<uint64_t>(rng()) << 32) | rng();
  devourer::cell::RachClient rach(joiner_mac, nonce, [&rng] { return static_cast<uint32_t>(rng()); });
  std::vector<devourer::cell::RachResponse> rar_seen;
  const auto mac = rach_mac(c);
  const auto rach_idx = grid->find(SlotKind::Rach, 0);

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
        if (me_a == 0) {
          const auto rb = find_rach_responses(p.Data.data(), p.Data.size());
          if (rb.first) {
            auto rs = devourer::cell::decode_rach_responses(rb.first, rb.second, c.cell, mac);
            rar_seen.insert(rar_seen.end(), rs.begin(), rs.end());
          }
        }
        return;
      }
      const int me = me_a.load();
      const auto d = parse_dl_frame(p.Data.data(), p.Data.size(), c.cell, auth);
      if (!d || me == 0 || d->ue != me)
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
        want = grid->find(SlotKind::Downlink, static_cast<uint16_t>(me));
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
    }
    /* M4: follow the DU's mode — autonomous = carrier sense + a random
     * uplink-beacon backoff while the DU is off the shared timebase. */
    if (m) {
      const bool want_auto = (mcopy.flags & devourer::cell::kGrantAutonomous) != 0;
      if (want_auto != ue_autonomous && dev->SetBeaconBackoff(want_auto)) {
        dev->SetCcaMode(!want_auto);
        ue_autonomous = want_auto;
        char l[140];
        snprintf(l, sizeof l, "{\"ev\":\"cell.ue.mode\",\"ue\":%d,\"mode\":\"%s\"}",
                 me_a.load(), want_auto ? "autonomous" : "coordinated");
        emit(l);
      }
    }
    if (me_a == 0) {
      /* Joining: answers first, then maybe an attempt next superframe. */
      std::vector<devourer::cell::RachResponse> rs;
      {
        std::lock_guard<std::mutex> lk(mu);
        rs.swap(rar_seen);
      }
      const int64_t sfn = du_now / P;
      if (rach.on_responses(rs)) {
        const auto &a = *rach.answer();
        char l[200];
        snprintf(l, sizeof l,
                 "{\"ev\":\"cell.ue.join\",\"sf\":%lld,\"status\":%d,\"ue\":%u,\"attempts\":%u,"
                 "\"losses\":%u}",
                 (long long)sfn, static_cast<int>(a.status), a.ue, rach.attempt(), rach.losses());
        emit(l);
        if (rach.state() != devourer::cell::RachState::Joined)
          break; /* denied: a defined end, no retry storm */
        if (!arm_ack(a.ue))
          break;
        me_a = a.ue;
        continue;
      }
      if (rach.state() == devourer::cell::RachState::Failed) {
        emit("{\"ev\":\"cell.ue.join\",\"status\":-1}");
        break;
      }
      const int64_t rlen = grid->slots()[*rach_idx].len_us;
      const int64_t air = devourer::cell::ofdm_airtime_us(
          24 + 3 + devourer::cell::rach_detail::kReqLen + devourer::cell::kRachTagLen + 4, 6);
      /* the whole msgA, plus the departure guard, must end inside the slot */
      const int64_t tx_us = air + devourer::cell::kRachDepartureGuardUs;
      const auto off = rach.opportunity(static_cast<uint64_t>(sfn + 1), rlen, tx_us);
      if (!off)
        continue;
      devourer::cell::RachRequest q;
      q.cell_id = c.cell;
      q.mac = joiner_mac;
      q.nonce = nonce;
      q.attempt = rach.attempt();
      q.fresh_sf = static_cast<uint32_t>(sfn + 1);
      const auto msga = devourer::cell::encode_rach_request(q, mac);
      auto pkt = build_dl_radiotap(6); /* RATE only: keep the session TX power */
      const auto fr = build_rach_frame(c.cell, joiner_mac.data(), msga);
      pkt.insert(pkt.end(), fr.begin(), fr.end());
      /* The target DU instant -> this UE's free-running counter, then wait. */
      const int64_t target_du = grid->slot_start_tsf(sfn + 1, *rach_idx) + *off;
      int64_t target_local = 0;
      {
        std::lock_guard<std::mutex> lk(mu);
        target_local = sync.LocalForRemote(static_cast<uint64_t>(target_du));
      }
      int64_t late = -1; /* how late the wait ended; -1 = the clock read failed */
      for (;;) {
        uint64_t t2 = 0, f2 = 0;
        if (!dev->ReadTsfAndFreeRun(t2, f2))
          break;
        int64_t now_local = 0;
        {
          std::lock_guard<std::mutex> lk(mu);
          now_local = wrap.extend(static_cast<uint32_t>(f2));
        }
        const int64_t left = target_local - now_local;
        if (left <= 0) {
          late = -left;
          break;
        }
        if (left > 1500)
          std::this_thread::sleep_for(std::chrono::microseconds(left - 1000));
      }
      /* Last check before the radio: the whole msgA must still fit the RACH
       * slot from where we actually are. Missed it (or lost the clock):
       * no transmission, no attempt spent: the client tries again. */
      if (late < 0 || *off + late + tx_us > rlen) {
        rach.unsend(static_cast<uint64_t>(sfn + 1));
        char l[160];
        snprintf(l, sizeof l, "{\"ev\":\"cell.ue.rach_skip\",\"sf\":%lld,\"late_us\":%lld}",
                 (long long)(sfn + 1), (long long)late);
        emit(l);
        continue;
      }
      const bool ok = !msga.empty() && dev->send_packet(pkt.data(), pkt.size());
      if (!ok)
        rach.unsend(static_cast<uint64_t>(sfn + 1));
      char l[200];
      snprintf(l, sizeof l,
               "{\"ev\":\"cell.ue.rach\",\"sf\":%lld,\"offset_us\":%lld,\"attempt\":%u,\"ok\":%d}",
               (long long)(sfn + 1), (long long)*off, rach.attempt(), ok ? 1 : 0);
      emit(l);
      continue;
    }
    if (!m)
      continue;
    const int me = me_a.load();
    const Grant *g = mcopy.find(static_cast<uint16_t>(me));
    if (!g)
      continue;
    /* The DU's slot indices must mean the same slots on this UE's grid (the
     * grid is built from the shared env): a disagreement is a configuration
     * error. Stop rather than transmit in someone else's slot. The pair may
     * be any capacity position (M5: the CU moves UEs), but one position:
     * the DL and UL slot of the same pair. */
    {
      const auto &sl = grid->slots();
      const bool ok = g->ul_slot < sl.size() && g->dl_slot < sl.size() &&
                      sl[g->ul_slot].kind == SlotKind::Uplink &&
                      sl[g->dl_slot].kind == SlotKind::Downlink &&
                      sl[g->ul_slot].ue == sl[g->dl_slot].ue && sl[g->ul_slot].ue >= 1;
      if (!ok) {
        char l[200];
        snprintf(l, sizeof l,
                 "{\"ev\":\"cell.ue.grid_mismatch\",\"ue\":%d,\"dl_slot\":%u,\"ul_slot\":%u,"
                 "\"slots\":%zu}",
                 me, g->dl_slot, g->ul_slot, sl.size());
        emit(l);
        fprintf(stderr, "grant slots do not match this UE's grid: check CELL_CAPACITY / "
                        "CELL_RACH_US / slot lengths against the DU\n");
        break;
      }
    }
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
           me_a.load(), steps, step_fails, (long long)listen_from, (long long)listen_to,
           (unsigned long long)dl_new, (unsigned long long)dl_dup);
  emit(l);
  return 0;
}

// ---------------------------------------------------------------------------
int run_witness(IRadio *dev, const Cfg &c) {
  /* CELL_WITNESS_CELLS [CELL_ID]: the cells to record (M5: two co-channel
   * cells on one receiver clock); each line names its cell. */
  std::vector<uint16_t> cells;
  if (const char *wc = std::getenv("CELL_WITNESS_CELLS")) {
    std::string t = wc;
    for (size_t p0 = 0; p0 <= t.size();) {
      const size_t q = t.find(',', p0);
      const long v = std::strtol(t.substr(p0, q == std::string::npos ? std::string::npos : q - p0).c_str(),
                                 nullptr, 10);
      if (v < 1 || v > 65535) {
        fprintf(stderr, "invalid CELL_WITNESS_CELLS=\"%s\"\n", wc);
        return 2;
      }
      cells.push_back(static_cast<uint16_t>(v));
      if (q == std::string::npos)
        break;
      p0 = q + 1;
    }
  }
  if (cells.empty())
    cells.push_back(c.cell);
  std::atomic<bool> done{false};
  std::thread rx([&] {
    dev->Init([&](const Packet &p) {
      if (p.RxAtrib.crc_err || p.Data.size() < 24)
        return;
      char l[240];
      const uint8_t *d = p.Data.data();
      for (const uint16_t cell : cells) {
      if (d[0] == 0x80 && cell_station(d + 10, cell) >= 0) {
        const int st = cell_station(d + 10, cell);
        const auto eg = p.TxEgressTsf();
        uint32_t ep = 0;
        if (st > 0) {
          if (auto ub = parse_ue_beacon(d, p.Data.size(), cell))
            ep = ub->epoch;
        } else if (auto m = devourer::cell::find_grant_map(d, p.Data.size())) {
          ep = m->epoch;
        }
        snprintf(l, sizeof l,
                 "{\"ev\":\"cell.w\",\"k\":\"%s\",\"st\":%d,\"tsfl\":%u,\"egress\":%llu,"
                 "\"epoch\":%u,\"len\":%zu,\"cell\":%u}",
                 st == 0 ? "bcn" : "ul", st, p.RxAtrib.tsfl,
                 (unsigned long long)(eg ? *eg : 0), ep, p.Data.size(), cell);
        emit(l);
        return;
      } else if (auto rf = parse_rach_frame(
                     d, p.Data.size() - (p.RxAtrib.fcs_present && p.Data.size() >= 4 ? 4 : 0),
                     cell)) {
        snprintf(l, sizeof l,
                 "{\"ev\":\"cell.w\",\"k\":\"rach\",\"st\":0,\"tsfl\":%u,\"len\":%zu,"
                 "\"joiner\":\"%02x%02x%02x%02x%02x%02x\",\"cell\":%u}",
                 p.RxAtrib.tsfl, p.Data.size(), rf->joiner[0], rf->joiner[1], rf->joiner[2],
                 rf->joiner[3], rf->joiner[4], rf->joiner[5], cell);
        emit(l);
        return;
      } else if (auto dl = parse_dl_frame(d, p.Data.size(), cell)) {
        snprintf(l, sizeof l,
                 "{\"ev\":\"cell.w\",\"k\":\"dl\",\"st\":%u,\"tsfl\":%u,\"sf\":%llu,\"seq\":%u,"
                 "\"len\":%zu,\"cell\":%u}",
                 dl->ue, p.RxAtrib.tsfl, (unsigned long long)dl->sf, dl->seq, p.Data.size(), cell);
        emit(l);
        return;
      }
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
