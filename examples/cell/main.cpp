// examples/cell — the M1 (#255) reference cell: one gNB-DU and N UEs on a
// fixed slot grid, plus an on-air witness. One binary, CELL_ROLE=du|ue|witness
// (|replayer: a test-only replay attacker, below).
// Frame shapes, clocks and the control loops are in cell.h; the grid is
// src/cell/SlotGrid.h and the grant map src/cell/GrantMap.h.
//
//   DU      beacons the grant map at its TBTT (CW=0 beacon, fixed departure),
//           sends one unicast DL frame per UE in that UE's DL slot (submitted
//           when its TSF reaches the slot start; hardware-ACKed, retries per
//           DEVOURER_TX_RETRY_LIMIT, which the DL slot budget must cover;
//           it carries up to 8 of the CU's DL PDUs queued for the UE —
//           F1 Downlink, also for a UE only staged for a handover — and the
//           UE delivers each PDU sn once: cell.ue.dl_gap, done dl_pdus),
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
//                             With per-cell keys: THIS cell's key (K_c,
//                             src/cell/Keys.h; a UE: its serving cell's).
//   CELL_AREA_KEY             per-cell keys (Keys.h): the area key K_A, which
//                             tags the neighbour list (what a UE checks of
//                             another cell). A UE beacon's area tag is then
//                             the UE's OWN area key (from CELL_UE_KEY), which
//                             a neighbour DU checks with the key the CU sends
//                             it (F1 AreaKeys). Unset = one shared key
//                             (CELL_PSK) for everything.
//   CELL_NL_SIGN_KEY CELL_NL_CERT   DU: its Ed25519 NL signing seed (64 hex)
//                             and the area CA's signature over its public key
//                             (128 hex): the NL rides with its signature and
//                             the certificate (src/cell/MeasGap.h).
//   CELL_CA_PUB               UE: the area CA's public key (64 hex): a beacon
//                             counts as a cell's (measured, its NL taken) only
//                             through CA -> cell key -> NL.
//   CELL_UE_NBR_FRESH [1]     UE, with CELL_CA_PUB: the replay checks. A
//                             serving beacon's signed window must not go back;
//                             a neighbour's must be within one of the serving
//                             one's, and its beacon must land within
//                             CELL_UE_NBR_PHASE_US [300] of the TBTT offset the
//                             serving list names. Else it is not measured
//                             (cell.ue.nbr_refused, the first 20 per reason;
//                             counts in cell.ue.done).
//   CELL_ROLE=replayer        test only: records cell CELL_REPLAY_CELL [2]'s
//                             DU beacons for CELL_REPLAY_FREEZE_S [30] s, then
//                             re-airs the last one every CELL_REPLAY_EVERY_MS
//                             [100] ms (cell.replay.frozen, cell.replay.done).
//   CELL_UE_KEY               UE, per-cell keys: its own key K_(c,u) (home
//                             cell and id), which unwraps the target cell's
//                             key a handover carries. Without it, or for a
//                             context that is not its own, the UE refuses the
//                             handover (cell.ue.ho_no_context).
//   CELL_UE=0 (role ue)       a JOINING UE: it gets its id through the RACH
//   CELL_JOINER_MAC           the joiner's MAC (default: derived from the host
//                             name + DEVOURER_PCIE_BDF, so a restart re-joins
//                             into the same position instead of a new one)
//   CELL_QOS [""]             DU: "ue:5qi,..." — a UE's QoS class (5QI 1 2 3
//                             5 7 9, CellScheduler.h): its HARQ budget, DL
//                             discard timer and link adaptation. Unlisted:
//                             the cell's HARQ and link adaptation, no discard
//   CELL_CS_RETUNE_US [26000] a channel move's retune budget (DU and UE):
//                             the move needs it, + 2 ms, after every slot and
//                             before the next beacon (ChannelSwitch.h)
//   CELL_UE_MEAS_LEVEL [""]   UE, test benches: "cell:dBm,..." — each listed
//                             cell's beacon measurements shifted so their
//                             median after calibration (8 beacons) is that
//                             level (MeasLevelPin): a suite's geometry
//                             independent of where the radios stand
//   CELL_RELEASE_SF [50]      DU: release a JOINED UE whose uplink has been
//                             absent this many superframes (static UEs stay)
//   CELL_GAP_SUSPEND_SF [0]   DU and UEs alike: suspend gaps (MeasGap.h). A
//                             UE measures a neighbour no idle window reaches
//                             in its suspend superframes (one in this many,
//                             staggered by id; its uplink beacon stopped
//                             around the gap); the DU airs it no DL then or
//                             the superframe after. Decided at the UEs'
//                             floor gap bound CELL_UE_GAP_RETUNE_US [26000] /
//                             CELL_UE_GAP_GUARD_US [2000], read by the DU too.
//   CELL_UE_DAPS_VISIT [1]    UE: between accepting an INTER-FREQUENCY handover
//                             and its switch, visit the target's channel once a
//                             superframe (after its uplink slot, in its idle
//                             time) to air a copy of its uplink there: the
//                             target DU forwards it to the CU (sequential DAPS;
//                             cell.ue.daps_visit)
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
//   CELL_PHASE_AUTO [0]       1 = ask the CU for it (M6 phase planning,
//                             src/cell/PhasePlan.h) before acquiring: a
//                             Report with phase -1, answered by F1 Phase;
//                             CELL_PHASE_WAIT_S [30] for the answer, else
//                             CELL_PHASE_US. Needs CELL_F1_CU.
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
//   CELL_HO_RELEASE_SF [20]   M6 handover, source side: superframes after the
//                             switch before the UE's grant is released (only
//                             if it has stopped uplinking here)
//   CELL_HO_FALLBACK_SF [10]  M6 handover, UE side: no grant at the target
//                             this many superframes after the switch = return
//                             to the source (which still holds the UE)
//   CELL_DAPS [1]             M6 DAPS (same channel). Every own-UE uplink is
//                             forwarded to the CU (F1 Uplink) whenever F1 is
//                             on. DAPS adds the UE's OTHER identity during a
//                             handover (the target hears it under its source
//                             id before the switch, the source under its
//                             target id after); the CU delivers each uplink
//                             once. 0 = the serving path only (no duplicate).
//   CELL_MEAS_L3_K [4] CELL_MEAS_STALE_MS [2000]  the reported uplink quality:
//                             a 3GPP L3 filter per (cell, UE), stale = absent
//   CELL_SYNC_UL_SF [30]      beacon-engine proxy: once UEs in the map have
//                             been heard, all of them silent this many
//                             superframes = the DU's beacon is not reaching
//                             them. Three failed beacon updates in a row also
//                             count.
//   CELL_UE_FIT_STEP_US [200] CELL_UE_FIT_CONFIRM [6]   UE: a serving beacon
//                             this far off the clock fit is held out of it;
//                             that many in a row agreeing on one offset are a
//                             step of the DU's TSF, re-anchored (TsfSync step
//                             detection, cell.ue.fit_step). 0 = off. 6
//                             beacons outlast the DU's discipline pass
//                             (CELL_SYNC_LOOP_MS 500): a jump the DU undoes
//                             itself is never followed, only a lasting one.
//   CELL_UE_REJOIN_SF [30]    UE that joined through the RACH: not granted by
//                             the map in force this many superframes (its DU
//                             restarted or released it) = drop the id and
//                             join again, with a fresh nonce (cell.ue.rejoin).
//                             0 = never. Static UEs never re-join.
//   CELL_UE_REATTACH_SF [200] UE: for this long after a confirmed step of the
//                             DU's clock, a grant map at a LOWER epoch that is
//                             due near its beacon's superframe means the DU
//                             restarted: the grant state starts over
//                             (cell.ue.reattach). 0 = never.
//   CELL_DU_TX_RAMP           DU, test: T:DB,T:DB,... the TX power offset
//                             (dB against the calibrated base; beacons and DL)
//                             piecewise linear in seconds from
//                             CELL_DU_TX_RAMP_T0 (unix s) or the service
//                             start, held at the ends (cell.du.txpwr): an
//                             emulated crossing.
//   CELL_DU_TEST_TSF_STEP     DU, test only: US@SF — add US to the TSF once at
//                             local superframe SF, behind the discipline's back
//                             (cell.test.tsf_step); the discipline undoes it.
//   CELL_DU_TEST_REPHASE_FAIL DU, test only: 1 = announce a phase move as
//                             usual, then do not make it (why "test"): what
//                             its UEs must survive (cell.ue.cs_rephase_undone)
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
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <condition_variable>
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
#include "cell/CellScheduler.h"
#include "cell/ChannelSwitch.h"
#include "cell/Coord.h"
#include "cell/F1.h"
#include "cell/Keys.h"
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
using devourer::cell::DlStreamRx;
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
/* Ed25519 (OpenSSL) for signed neighbour lists (src/cell/MeasGap.h). */
template <size_t N>
std::optional<std::array<uint8_t, N>> hex_bytes(const char *h) {
  if (h == nullptr || std::strlen(h) != 2 * N)
    return std::nullopt;
  std::array<uint8_t, N> out{};
  for (size_t i = 0; i < N; ++i) {
    unsigned v = 0;
    if (std::sscanf(h + 2 * i, "%2x", &v) != 1)
      return std::nullopt;
    out[i] = static_cast<uint8_t>(v);
  }
  return out;
}
#if defined(DEVOURER_CELL_OPENSSL)
/* The signer for a 32-byte seed, and its public key. */
std::pair<devourer::cell::NlSigner, std::optional<devourer::cell::Ed25519Pub>>
ed25519_signer(const std::array<uint8_t, 32> &seed) {
  EVP_PKEY *pk = EVP_PKEY_new_raw_private_key(EVP_PKEY_ED25519, nullptr, seed.data(), seed.size());
  if (!pk)
    return {{}, std::nullopt};
  devourer::cell::Ed25519Pub pub{};
  size_t pl = pub.size();
  const bool have_pub = EVP_PKEY_get_raw_public_key(pk, pub.data(), &pl) == 1 && pl == pub.size();
  std::shared_ptr<EVP_PKEY> key(pk, EVP_PKEY_free);
  devourer::cell::NlSigner sign = [key](const uint8_t *m, size_t n, uint8_t sig[64]) {
    EVP_MD_CTX *c = EVP_MD_CTX_new();
    size_t sl = 64;
    const bool ok = c && EVP_DigestSignInit(c, nullptr, nullptr, nullptr, key.get()) == 1 &&
                    EVP_DigestSign(c, sig, &sl, m, n) == 1 && sl == 64;
    EVP_MD_CTX_free(c);
    return ok;
  };
  return {sign, have_pub ? std::optional<devourer::cell::Ed25519Pub>(pub) : std::nullopt};
}
bool ed25519_verify(const uint8_t pub[32], const uint8_t *m, size_t n, const uint8_t sig[64]) {
  EVP_PKEY *pk = EVP_PKEY_new_raw_public_key(EVP_PKEY_ED25519, nullptr, pub, 32);
  if (!pk)
    return false;
  EVP_MD_CTX *c = EVP_MD_CTX_new();
  const bool ok = c && EVP_DigestVerifyInit(c, nullptr, nullptr, nullptr, pk) == 1 &&
                  EVP_DigestVerify(c, sig, 64, m, n) == 1;
  EVP_MD_CTX_free(c);
  EVP_PKEY_free(pk);
  return ok;
}
#endif

CellMac cell_auth(const Cfg &c) { return hmac_sha1(c.psk); }
/* The area key (src/cell/Keys.h): what another cell must be able to check —
 * the neighbour list and the UE beacon's area tag. CELL_AREA_KEY, else the
 * cell key (one shared key, as before). */
CellMac area_auth(const Cfg &c) {
  const char *a = std::getenv("CELL_AREA_KEY");
  return a && *a ? hmac_sha1(a) : cell_auth(c);
}
devourer::cell::Mac rach_mac(const Cfg &c) { return cell_auth(c); }
bool auth_ok(const Cfg &c) {
#if !defined(DEVOURER_CELL_OPENSSL)
  /* Without OpenSSL a key would become an EMPTY authenticator — silently
   * unauthenticated: refused, for every key. */
  for (const char *k : {"CELL_AREA_KEY", "CELL_UE_KEY", "CELL_NL_SIGN_KEY", "CELL_NL_CERT", "CELL_CA_PUB"})
    if (const char *v = std::getenv(k); v && *v) {
      fprintf(stderr, "%s needs a build with OpenSSL\n", k);
      return false;
    }
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
    /* ... + a full neighbour list, and with signed lists its signature and
     * certificate (MeasGap.h) */
    std::vector<uint8_t> extra = devourer::cell::encode_nl_ie(
        c.cell, 0, std::vector<devourer::cell::Neighbour>(devourer::cell::kMaxNeighbours), {});
    if (const char *sk = std::getenv("CELL_NL_SIGN_KEY"); sk && *sk) {
      const auto sig = devourer::cell::encode_nl_sig_ie(
          c.cell, 0, 0, extra, [](const uint8_t *, size_t, uint8_t s[64]) { std::memset(s, 0, 64); return true; });
      const auto cert = devourer::cell::encode_nl_cert_ie(c.cell, {}, {});
      extra.insert(extra.end(), sig.begin(), sig.end());
      extra.insert(extra.end(), cert.begin(), cert.end());
    }
    const int64_t bcn_need =
        22 + ofdm_airtime_us(build_du_beacon(probe, cell_auth(c), rar, extra).size() + 4, 6);
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
  /* How long before its slot a DL frame takes its PDUs and is built. */
  const int64_t dl_pick_lead_us = envl("CELL_DL_PICK_LEAD_US", 3000, 500, 100000);
  /* A channel move's retune budget (the 8852CE's FastRetune: 23.5-23.7 ms). */
  const int64_t cs_retune_us = envl("CELL_CS_RETUNE_US", 26000, 1000, 100000);
  const int64_t grid_span_us = [&] { /* every slot's end: where a move's window starts */
    int64_t span = 0;
    for (const auto &x : grid->slots())
      span = std::max<int64_t>(span, x.start_us + x.len_us);
    return span;
  }();
  /* A frame that would start later than this into its slot is not sent (the
   * slot budget's guard, as du_budget_ok counts it). */
  const int64_t dl_guard_us = envl("CELL_DL_GUARD_US", 500, 0, 100000);
  const bool harq_on = envl("CELL_HARQ", 1, 0, 1) != 0;
  const bool la_on = harq_on && envl("CELL_LA", 1, 0, 1) != 0;
  HarqConfig hc;
  hc.max_attempts = static_cast<int>(envl("CELL_HARQ_ATTEMPTS", 4, 1, 16));
  hc.timeout_sf = static_cast<int>(envl("CELL_HARQ_TIMEOUT_SF", 3, 1, 50));
  const int dl_rate0 = static_cast<int>(envl("CELL_DL_RATE", 24, 6, 54));
  const uint16_t atten_ue = static_cast<uint16_t>(envl("CELL_ATTEN_UE", 0, 0, 254));
  const int8_t atten_db = static_cast<int8_t>(envl("CELL_ATTEN_DB", 0, -40, 0));
  const int64_t P = grid->period_us();
  /* M1 QoS (CellScheduler.h): CELL_QOS="ue:5qi,..." gives a UE a 5QI class,
   * and with it its HARQ budget, DL discard timer and link adaptation. A UE
   * not listed is served as before (the cell's HARQ and link adaptation, no
   * discard): DAPS's buffered PDUs, for one, are older than any budget. */
  std::map<uint16_t, devourer::cell::UePolicy> qos_pol;
  if (const char *q = std::getenv("CELL_QOS")) {
    std::string spec(q);
    for (size_t i = 0; i < spec.size();) {
      const size_t comma = spec.find(',', i);
      const std::string item = spec.substr(i, comma == std::string::npos ? std::string::npos : comma - i);
      unsigned ue = 0, qi = 0;
      char tail = 0;
      const auto cls = sscanf(item.c_str(), "%u:%u%c", &ue, &qi, &tail) == 2 && ue >= 1 && ue <= 254 && qi <= 255
                           ? devourer::cell::qos_class(static_cast<uint8_t>(qi))
                           : std::nullopt;
      if (!cls) {
        fprintf(stderr, "CELL_QOS: \"%s\" is not ue:5qi with a known 5QI (1 2 3 5 7 9)\n", item.c_str());
        return 2;
      }
      qos_pol[static_cast<uint16_t>(ue)] = devourer::cell::policy_for(*cls, P, {}, hc.max_attempts, 5, hc.timeout_sf);
      if (comma == std::string::npos)
        break;
      i = comma + 1;
    }
  }   if (atten_ue != 0 && atten_db != 0 && !dev->GetAdapterCaps().per_packet_txpower) {
    fprintf(stderr, "CELL_ATTEN_DB needs per-packet TX power (per_packet_txpower=0): the "
                    "attenuation would silently not apply\n");
    return 2;
  }
  const CellMac auth = cell_auth(c);
  const CellMac area = area_auth(c);
  const char *area_env_du = std::getenv("CELL_AREA_KEY");
  const bool per_cell_keys = area_env_du && *area_env_du;
  /* Signed neighbour lists (MeasGap.h): CELL_NL_SIGN_KEY (this DU's Ed25519
   * seed, 64 hex) and CELL_NL_CERT (the CA's signature over its public key,
   * 128 hex). The NL then rides with its signature and the certificate. */
  devourer::cell::NlSigner nl_sign;
  std::vector<uint8_t> nl_cert_ie;
  {
    const char *sk = std::getenv("CELL_NL_SIGN_KEY");
    const char *ce = std::getenv("CELL_NL_CERT");
    if ((sk && *sk) || (ce && *ce)) {
      const auto seed = hex_bytes<32>(sk);
      const auto cert = hex_bytes<64>(ce);
#if defined(DEVOURER_CELL_OPENSSL)
      auto [sg, pub] = seed ? ed25519_signer(*seed)
                            : std::pair<devourer::cell::NlSigner, std::optional<devourer::cell::Ed25519Pub>>{};
      if (!seed || !cert || !sg || !pub) {
        fprintf(stderr, "CELL_NL_SIGN_KEY (64 hex) and CELL_NL_CERT (128 hex) go together\n");
        return 2;
      }
      nl_sign = sg;
      nl_cert_ie = devourer::cell::encode_nl_cert_ie(c.cell, *pub, *cert);
#else
      (void)seed;
      (void)cert;
#endif
    }
  }
  std::vector<uint8_t> nl_signed_for, nl_sig_ie; /* the last NL signed, and its NS IE */
  bool nl_sign_failed = false, nl_unsigned_no_net = false;
  uint32_t nl_signed_win = 0;
  const int64_t release_sf = envl("CELL_RELEASE_SF", 50, 2, 100000);
  /* Suspend gaps (MeasGap.h): a UE some neighbour leaves no idle window
   * goes deaf in its suspend superframes; no DL to it then or the next one.
   * Decided with the UEs' FLOOR gap bound: the same env as theirs. */
  const int64_t gap_suspend_every = envl("CELL_GAP_SUSPEND_SF", 0, 0, 100000);
  const int64_t gap_floor_us = envl("CELL_UE_GAP_RETUNE_US", 26000, 1000, 500000);
  const int64_t gap_guard_du_us = envl("CELL_UE_GAP_GUARD_US", 2000, 0, 50000);

  /* M4 DU service mode (see the header): the PHC, the discipline loop, the
   * health that gates the coordinated schedule. */
  const char *ref_ptp = std::getenv("CELL_REF_PTP");
  const bool svc = ref_ptp && *ref_ptp;
  if (nl_sign && !svc)
    fprintf(stderr, "CELL_NL_SIGN_KEY without CELL_REF_PTP: no network superframe to sign a window "
                    "with, so the list airs unsigned (cell.nl.unsigned_no_net)\n");
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
    /* Non-blocking: sends happen on the radio RX path (DAPS forwarding) and
     * must never wait on the network; a full socket drops (counted). */
    if (f1_fd.fd >= 0)
      fcntl(f1_fd.fd, F_SETFL, fcntl(f1_fd.fd, F_GETFL, 0) | O_NONBLOCK);
    freeaddrinfo(res);
    if (f1_fd.fd < 0) {
      fprintf(stderr, "F1 socket failed\n");
      return 1;
    }
  }
  const CellMac f1_mac = hmac_sha1(f1_key);
  /* Only the configured CU's address (the tag and freshness come next). */
  auto f1_from_cu = [&](const sockaddr_storage &from) {
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
  reg.set_ta_config(ta_gain, P / 4); /* each UE's TA loop (its UeContext) */
  /* Each UE's home identity (src/cell/Keys.h), recorded at its admission —
   * static, a RACH joiner's MAC, or the home a handover command carries —
   * and reported to the CU (F1 Report.homes) for the UE's handover context.
   * Every admission writes it, so a reused id never keeps another UE's
   * (f1_mu, after the threads start). */
  std::map<uint16_t, devourer::cell::HomeId> ue_home;
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
    ue_home[ue] = devourer::cell::HomeId{false, c.cell, ue, {}};
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
  /* M6 phase planning: a joining DU asks the CU where its grid goes (the
   * running cells keep theirs), before it acquires: a Report with phase -1
   * (its grid, channel, tier), every 500 ms, until a Phase answer from the
   * CU's address. Without one in time it keeps CELL_PHASE_US. Before the
   * beacon starts: a waiting DU airs nothing a UE could lock to. */
  if (svc && f1_on && envl("CELL_PHASE_AUTO", 0, 0, 1) != 0) {
    namespace f1 = devourer::cell::f1;
    const int wait_s = static_cast<int>(envl("CELL_PHASE_WAIT_S", 30, 1, 600));
    devourer::cell::CoordDu lay;
    devourer::cell::coord_layout(*grid, lay);
    f1::Report rq;
    rq.du = c.cell;
    rq.tier = static_cast<uint8_t>(tier);
    rq.channel = c.ch;
    rq.phase_us = -1; /* asks for a phase */
    rq.period_us = P;
    rq.fixed = lay.fixed;
    rq.positions = lay.positions;
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(wait_s);
    auto next_req = std::chrono::steady_clock::now();
    std::optional<int64_t> got;
    uint64_t rq_seq = 0, first_seq = 0;
    while (!got && std::chrono::steady_clock::now() < until) {
      if (std::chrono::steady_clock::now() >= next_req) {
        next_req += std::chrono::milliseconds(500);
        rq_seq = std::max(rq_seq + 1, static_cast<uint64_t>(
                                          std::chrono::duration_cast<std::chrono::microseconds>(
                                              std::chrono::system_clock::now().time_since_epoch())
                                              .count()));
        rq.seq = rq_seq;
        if (first_seq == 0)
          first_seq = rq_seq;
        const auto m = f1::encode_report(rq, f1_mac);
        sendto(f1_fd.fd, m.data(), m.size(), 0, reinterpret_cast<const sockaddr *>(&f1_addr),
               f1_addr_len);
      }
      pollfd pf{f1_fd.fd, POLLIN, 0};
      if (poll(&pf, 1, 50) <= 0 || !(pf.revents & POLLIN))
        continue;
      uint8_t buf[f1::kMaxDatagram];
      sockaddr_storage from{};
      socklen_t fl = sizeof from;
      const ssize_t n = recvfrom(f1_fd.fd, buf, sizeof buf, 0, reinterpret_cast<sockaddr *>(&from), &fl);
      if (n <= 0 || !f1_from_cu(from))
        continue;
      /* The CU echoes the request's seq: an answer to an earlier run's
       * request (a replay) is older than this run's first. */
      if (auto ph = f1::decode_phase(buf, static_cast<size_t>(n), f1_mac);
          ph && ph->du == c.cell && ph->phase_us >= 0 && ph->phase_us < P &&
          ph->seq >= first_seq && ph->seq <= rq_seq)
        got = ph->phase_us;
    }
    char l[160];
    if (got) {
      dc.phase_us = static_cast<double>(*got);
      disc.set_phase(dc.phase_us);
      snprintf(l, sizeof l, "{\"ev\":\"cell.phase.assigned\",\"phase_us\":%lld}", (long long)*got);
    } else {
      snprintf(l, sizeof l, "{\"ev\":\"cell.phase.default\",\"phase_us\":%.0f}", dc.phase_us);
    }
    emit(l);
  }
  /* CELL_DU_TX_RAMP=T:DB,T:DB,...: TX power offset (dB, against the
   * calibrated base) piecewise linear in seconds since the DU starts serving;
   * held at the ends. Test only (an emulated crossing). */
  std::vector<std::pair<double, double>> tx_ramp;
  if (const char *rp = std::getenv("CELL_DU_TX_RAMP"); rp && *rp) {
    std::string t = rp;
    for (size_t p0 = 0; p0 < t.size();) {
      const size_t q = t.find(',', p0);
      const std::string item = t.substr(p0, q == std::string::npos ? std::string::npos : q - p0);
      double ts = 0, db = 0;
      if (std::sscanf(item.c_str(), "%lf:%lf", &ts, &db) != 2 || ts < 0 || db < -40 || db > 20 ||
          (!tx_ramp.empty() && ts <= tx_ramp.back().first)) {
        fprintf(stderr, "CELL_DU_TX_RAMP: T:DB,... with T increasing, -40 <= DB <= 20\n");
        return 2;
      }
      tx_ramp.push_back({ts, db});
      if (q == std::string::npos)
        break;
      p0 = q + 1;
    }
  }
  /* The ramp's zero: CELL_DU_TX_RAMP_T0 (unix seconds: the DUs of one test
   * ramp together, on their hosts' synced clocks), else service start. */
  double ramp_t0_unix = 0;
  if (const char *z = std::getenv("CELL_DU_TX_RAMP_T0"); z && *z) {
    char *end = nullptr;
    ramp_t0_unix = std::strtod(z, &end);
    if (end == z || *end != '\0' || !(ramp_t0_unix > 0)) {
      fprintf(stderr, "CELL_DU_TX_RAMP_T0: unix seconds (> 0), got \"%s\"\n", z);
      return 2;
    }
  }
  if (!tx_ramp.empty() && !dev->GetTxPowerCaps().supported) {
    fprintf(stderr, "CELL_DU_TX_RAMP: this radio has no runtime TX power control\n");
    return 2;
  }
  bool ramp_short_warned = false;
  std::chrono::steady_clock::time_point ramp_t0{};
  int ramp_set_qdb = INT_MIN;
  auto bcn = build_du_beacon(map, auth);
  if (bcn.empty() || !dev->StartBeacon(bcn.data(), bcn.size(), c.tu)) {
    fprintf(stderr, "StartBeacon failed\n");
    return 1;
  }
  std::vector<uint8_t> live_bcn = bcn; /* what airs now: restarted after a channel move */
  /* The channel this DU serves on: DEVOURER_CHANNEL at start, then where a
   * channel move (ChannelSwitch.h, from the CU) took it. */
  std::atomic<uint8_t> du_ch{c.ch};

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
  /* The cell's phase now (us): dc.phase_us is the main thread's (the
   * discipline's); the RX and F1 threads read this, which a live phase move
   * (ChannelSwitch.h v2) changes with net_off, in the idle part of a
   * superframe. */
  std::atomic<int64_t> phase_live{static_cast<int64_t>(dc.phase_us)};
  /* phase + net_off * P in ONE value: an uplink's network time must not mix
   * a phase and an offset from either side of a move (main thread writes). */
  std::atomic<int64_t> net_base{static_cast<int64_t>(dc.phase_us)};
  std::atomic<bool> net_ok{false};
  uint64_t sync_samples = 0;
  std::array<uint64_t, 3> sync_in_state{};
  /* CELL_DU_TEST_TSF_STEP=US@SF (test only): one TSF jump the discipline
   * did not ask for, what a UE's clock fit must survive. */
  int64_t test_step_us = 0, test_step_sf = -1;
  const bool test_rephase_fail = envl("CELL_DU_TEST_REPHASE_FAIL", 0, 0, 1) != 0;
  if (const char *ts = std::getenv("CELL_DU_TEST_TSF_STEP"); ts && *ts) {
    long long us = 0, at = 0;
    if (std::sscanf(ts, "%lld@%lld", &us, &at) != 2 || at < 0 || us == 0 || std::llabs(us) > P) {
      fprintf(stderr, "CELL_DU_TEST_TSF_STEP: US@SF, 0 < |US| <= period\n");
      return 2;
    }
    test_step_us = us;
    test_step_sf = at;
  }
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
      net_base.store(phase_live.load() + net_off.load() * P);
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
    /* The test step goes in AFTER this pass's sample, so it stays on air
     * until the next pass (CELL_SYNC_LOOP_MS) undoes it; the RX offset is
     * re-read as after any write. */
    if (test_step_sf >= 0 && sf_now >= test_step_sf) {
      const bool ok = dev->WriteTsf(dev->ReadTsf() + static_cast<uint64_t>(test_step_us));
      test_step_sf = -1;
      uint64_t t3 = 0, f3 = 0;
      if (dev->ReadTsfAndFreeRun(t3, f3)) {
        publish_clock(t3, f3);
      } else { /* as after any write: no stale offset until a fresh read */
        std::lock_guard<std::mutex> lk(clk_mu);
        clk.valid = false;
      }
      char l2[120];
      snprintf(l2, sizeof l2, "{\"ev\":\"cell.test.tsf_step\",\"sf\":%lld,\"step_us\":%lld,\"ok\":%d}",
               (long long)sf_now, (long long)test_step_us, ok ? 1 : 0);
      emit(l2);
    }
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

  std::mutex mu;  // guards reg's per-UE contexts (TA, uplink window), ue_epoch
  std::map<uint16_t, uint32_t> ue_epoch;
  for (uint16_t ue : ues)
    reg.context(ue);
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
    /* The CU DL PDUs each HARQ seq in flight carries, and their stream (a
     * retransmission re-airs the same ones). */
    std::map<uint32_t, std::pair<uint64_t, std::vector<uint32_t>>> pdus;
    std::map<uint32_t, int64_t> first_ms; /* each seq's first attempt (QoS discard) */
    /* A handover arrival not heard yet: the frames aired to it so far may
     * have gone out before the UE switched (see the uplink handler). */
    bool arriving = false;
  };
  std::map<uint16_t, DlUe> dlue;
  /* The CU's DL PDUs (F1 Downlink) waiting per UE (guarded by mu). Kept
   * for a UE this DU has only STAGED too (a handover target under DAPS
   * buffers what the source is still airing): sent from its first DL frame,
   * oldest first, kDlMaxPdus per frame. Bounded: the oldest go first. */
  struct DlQueue {
    std::deque<uint32_t> sns;
    /* The newest sn a DAPS-visit answer has offered (the queue keeps them all:
     * nothing ACKs an answer; the UE dedups after the switch). */
    std::optional<uint32_t> offered;
    uint64_t epoch = 0;   /* the stream queued (F1.h DownlinkPdu) */
    uint32_t last_sn = 0; /* the newest sn taken in it */
    std::chrono::steady_clock::time_point last_push;
    uint64_t dropped = 0, refused = 0;
    std::map<uint32_t, int64_t> at_ms; /* when each sn came (the QoS discard timer) */
    uint64_t discarded = 0;
  };
  constexpr size_t kDlQueueMax = 64;
  /* A contiguous queue's out-of-window PDUs (at most kDlQueueMax - 33) and the
   * fresh one fit one frame; with gaps pick_dl_pdus still keeps the fresh
   * one's place. */
  static_assert(kDlMaxPdus >= kDlQueueMax - devourer::cell::DlStreamRx::kReorder, "a DL frame too small");
  std::map<uint16_t, DlQueue> dlq;
  /* A UE's DL status against its queue here (mu held): what it has goes.
   * Logged when anything went. `via`: its own uplink, or its source-identity
   * beacon heard before the switch. */
  auto status_prune = [&](uint16_t ue, const UeDlStatus &st, const char *via) {
    auto qi = dlq.find(ue);
    if (qi == dlq.end() || static_cast<uint32_t>(qi->second.epoch) != st.epoch32)
      return;
    const size_t n = prune_dl_queue(qi->second.sns, st);
    if (n == 0)
      return;
    char l[200];
    snprintf(l, sizeof l,
             "{\"ev\":\"cell.dl.status_prune\",\"ue\":%u,\"top\":%u,\"missing\":%u,\"pruned\":%zu,"
             "\"left\":%zu,\"via\":\"%s\"}",
             ue, st.top, static_cast<unsigned>(__builtin_popcount(st.missing)), n, qi->second.sns.size(), via);
    emit(l);
  };
  /* A handover arrival's DL frames in flight given up (mu held): aired
   * before the UE switched (it switches up to ~2 superframes after the
   * instant), their retransmissions would go before any new data. Their
   * PDUs go back to the head of the queue; the UE's DL status (its beacons)
   * then prunes what it has (status_prune). Returns (frames given up, PDUs
   * re-queued). */
  auto arrival_abandon = [&](DlUe &d, uint16_t ue) -> std::pair<size_t, size_t> {
    const auto gone = d.harq.abandon_all();
    std::vector<uint32_t> back;
    uint64_t back_epoch = 0;
    for (uint32_t seq : gone)
      if (auto pi = d.pdus.find(seq); pi != d.pdus.end()) {
        back_epoch = pi->second.first;
        back.insert(back.end(), pi->second.second.begin(), pi->second.second.end());
        d.pdus.erase(pi);
      }
    size_t requeued = 0;
    if (auto qi = dlq.find(ue); qi != dlq.end() && !back.empty() && qi->second.epoch == back_epoch) {
      const auto [added, over] = requeue_dl(qi->second.sns, std::move(back), kDlQueueMax);
      requeued = added;
      qi->second.dropped += over;
      if (over) {
        char l[120];
        snprintf(l, sizeof l, "{\"ev\":\"cell.dl.requeue_overflow\",\"ue\":%u,\"dropped\":%zu}", ue, over);
        emit(l);
      }
    }
    return {gone.size(), requeued};
  };
  /* A UE's DL state, from its QoS class if it has one (qos_pol). */
  auto make_dlue = [&](uint16_t ue) {
    auto it = qos_pol.find(ue);
    if (it == qos_pol.end())
      return DlUe{HarqSender(hc), LinkAdapt({}, dl_rate0)};
    HarqConfig h = hc;
    h.max_attempts = it->second.harq_attempts;
    char l[220];
    snprintf(l, sizeof l,
             "{\"ev\":\"cell.qos\",\"ue\":%u,\"qi\":%u,\"harq_attempts\":%d,\"discard_ms\":%lld,"
             "\"pdb_unmet\":%d,\"la_down\":%.2f,\"la_up\":%.2f}",
             ue, it->second.qi, h.max_attempts, (long long)it->second.discard_ms, it->second.pdb_unmet ? 1 : 0,
             it->second.la.down_loss, it->second.la.up_loss);
    emit(l);
    return DlUe{HarqSender(h), LinkAdapt(it->second.la, dl_rate0)};
  };
  for (uint16_t ue : ues)
    dlue.emplace(ue, make_dlue(ue));
  std::vector<uint8_t> rar_on_air;                 /* the RACH answers in the live beacon */
  std::vector<uint8_t> ho_on_air;                  /* the handover IEs in the live beacon */
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
  /* Filtered uplink quality (L3, src/cell/Handover.h) per (cell, UE): this
   * cell's own UEs and the other cells' it hears; the F1 report's
   * measurements (f1_mu). */
  devourer::cell::MeasTable meas_tab(devourer::cell::L3FilterConfig{
      static_cast<int>(envl("CELL_MEAS_L3_K", 4, 0, 19)), envl("CELL_MEAS_STALE_MS", 2000, 100, 60000)});
  devourer::cell::CoordDu layout;
  devourer::cell::coord_layout(*grid, layout);
  /* M6 handover (src/cell/HoCommand.h), staged by the F1 thread (f1_mu).
   * Target side: a UE coming in, by the id it gets here, its position, and
   * the switch instant in this DU's superframe count. Source side: a UE
   * leaving, with the IE this DU airs to it until the switch. */
  struct HoIn {
    uint32_t id = 0;
    uint8_t pos = 0;
    uint64_t eff_sf = 0;
    int16_t ta_us = 0; /* the UE's TA at its source (HoCommand v17) */
    bool cond = false; /* conditional (F1 v18): eff_sf is the expiry, published at once */
  };
  std::map<uint16_t, int16_t> ta_report; /* f1_mu: each own UE's TA in the map on air */
  struct HoOut {
    uint32_t id = 0;
    uint64_t eff_sf = 0;
    uint16_t target = 0, target_ue = 0;
    std::vector<uint8_t> ie;
    bool cond = false; /* conditional: eff_sf is the expiry; the UE may leave any time before it */
  };
  std::map<uint16_t, HoIn> ho_in;   /* by target UE id */
  std::set<uint16_t> ho_cancel;     /* published arrivals the CU cancelled */
  std::map<uint16_t, uint32_t> ho_published; /* published arrivals: UE id -> handover id */
  /* M6 measurement gaps (f1_mu): the CU's neighbour list as offsets on this
   * grid (aired in the NL IE), and what each own UE reported measuring (its
   * MR IE), relayed in Report.ue_meas. */
  std::vector<devourer::cell::Neighbour> nb_list;
  std::vector<devourer::cell::f1::NeighbourEntry> nb_raw; /* f1_mu: the list as the CU sent it */
  uint32_t nl_gen = 1; /* bumped whenever the list changes (UEs refuse older) */
  bool have_nb_seq = false;
  uint64_t last_nb_seq = 0;
  /* Per-cell keys: the other cells' UEs' own area keys (Keys.h A_u, F1
   * AreaKeys from the CU, unwrapped with this cell's key), by (cell, UE);
   * a heard UE beacon counts only with its own (f1_mu). */
  std::map<std::pair<uint16_t, uint16_t>, CellMac> ue_area_tab;
  bool have_ak_seq = false;
  uint64_t last_ak_seq = 0;
  struct UeReport {
    std::vector<devourer::cell::MeasEntryUe> m;
    int64_t at_ms = 0;
    uint32_t gen = 0;
  };
  std::map<uint16_t, UeReport> ue_reports;
  std::map<uint16_t, HoOut> ho_out; /* by (source) UE id */
  bool have_ho_seq = false;
  uint64_t last_ho_seq = 0;
  /* A staged channel move (ChannelSwitch.h; f1_mu): announced in the beacon
   * until its instant, then this DU and its UEs retune together. */
  struct CsStaged {
    uint32_t id = 0;
    uint8_t channel = 0;
    int64_t eff_sf = 0; /* this DU's count */
    int aired = 0;      /* superframes a published beacon carried its IE */
    int64_t phase = -1; /* a phase move (ChannelSwitch.h v2): to this phase, */
    int64_t at_us = 0, step_us = 0; /* the TSF step and its instant (rephase_plan) */
  };
  std::optional<CsStaged> cs_staged;
  uint32_t cs_aired_id = 0; /* the move whose IE the live beacon carries (f1_mu) */
  bool have_cs_seq = false;
  uint64_t last_cs_seq = 0;
  /* DAPS: the other identities to forward, until a local superframe (f1_mu). */
  const bool daps_on = envl("CELL_DAPS", 1, 0, 1) != 0;
  std::map<std::pair<uint16_t, uint16_t>, int64_t> daps_ids;
  /* A staged arrival's source identity -> its id here (f1_mu): the UE's
   * beacons to its source carry its DL status, so this DU's queue for it is
   * pruned before the switch too. Expires with its daps_ids entry. */
  std::map<std::pair<uint16_t, uint16_t>, uint16_t> arrival_of;
  /* Sequential DAPS, DL (f1_mu): an INTER-FREQUENCY staged arrival's SOURCE
   * identity -> its id here and its switch instant (this DU's count): a visit
   * copy heard under that identity before the switch is answered with its
   * queued DL PDUs. */
  struct VisitAns {
    uint16_t target_ue = 0;
    int64_t eff_sf = 0;
  };
  std::map<std::pair<uint16_t, uint16_t>, VisitAns> visit_src;
  /* The answers, aired by their own thread (started with the RX loop). */
  struct AnsFrame {
    std::vector<uint8_t> pkt;
    uint16_t ue = 0;
    size_t pdus = 0;
    uint32_t first = 0;
    std::optional<uint32_t> prev_offered; /* the queue's offered mark before this answer */
    uint32_t offered = 0;                 /* ... and with it */
    std::chrono::steady_clock::time_point deadline; /* the UE leaves the channel after */
  };
  /* An answer not aired (queue full, expired, failed) gives its PDUs back:
   * the offered mark returns to where it was, unless a later answer moved it. */
  auto unoffer = [&](uint16_t ue, const std::optional<uint32_t> &prev, uint32_t was) {
    std::lock_guard<std::mutex> lk(mu);
    if (auto qi = dlq.find(ue); qi != dlq.end() && qi->second.offered && *qi->second.offered == was)
      qi->second.offered = prev;
  };
  std::mutex ans_mu;
  std::condition_variable ans_cv;
  std::deque<AnsFrame> ans_q;
  bool ans_stop = false;
  std::thread ans_thr([&] {
    for (;;) {
      AnsFrame a;
      {
        std::unique_lock<std::mutex> lk(ans_mu);
        ans_cv.wait(lk, [&] { return ans_stop || !ans_q.empty(); });
        if (ans_stop)
          return;
        a = std::move(ans_q.front());
        ans_q.pop_front();
      }
      /* The UE holds the target's channel only a few ms after its visit copy:
       * an answer later than that is wasted — dropped, its PDUs given back. */
      const bool expired = std::chrono::steady_clock::now() > a.deadline;
      const bool sent = !expired && dev->send_packet(a.pkt.data(), a.pkt.size());
      if (!sent)
        unoffer(a.ue, a.prev_offered, a.offered);
      char l[200];
      snprintf(l, sizeof l,
               "{\"ev\":\"cell.daps.visit_dl\",\"ue\":%u,\"pdus\":%zu,\"first\":%u,\"sent\":%d,\"expired\":%d,"
               "\"t_ms\":%lld}",
               a.ue, a.pdus, a.first, sent ? 1 : 0, expired ? 1 : 0,
               (long long)std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
                   .count());
      emit(l);
    }
  });
  struct AnsJoin {
    std::mutex &m;
    std::condition_variable &cv;
    bool &stop;
    std::thread &t;
    ~AnsJoin() {
      {
        std::lock_guard<std::mutex> lk(m);
        stop = true;
      }
      cv.notify_all();
      if (t.joinable())
        t.join();
    }
  } ans_join{ans_mu, ans_cv, ans_stop, ans_thr};
  std::atomic<uint64_t> f1_send_drops{0};
  auto send_f1 = [&](const std::vector<uint8_t> &m) { /* any thread: a non-blocking sendto */
    if (!f1_on || m.empty())
      return;
    if (sendto(f1_fd.fd, m.data(), m.size(), 0, reinterpret_cast<const sockaddr *>(&f1_addr),
               f1_addr_len) < 0)
      ++f1_send_drops;
  };
  auto forward_ul = [&](uint16_t cell, uint16_t ue, const Packet &p) {
    /* The common seq is the reception instant on the shared timebase: no
     * timebase (not locked), no forwarding. */
    if (!net_ok.load())
      return;
    const auto at = rx_tsf(p.RxAtrib.tsfl);
    if (!at)
      return;
    devourer::cell::f1::UplinkFwd u;
    u.du = c.cell;
    u.cell = cell;
    u.ue = ue;
    u.net_us = *at + net_base.load();
    u.seq = devourer::cell::ul_seq_from_network_time(u.net_us, P);
    u.rssi_dbm = static_cast<int8_t>(static_cast<int>(p.RxAtrib.rssi[0]) - 110);
    send_f1(devourer::cell::f1::encode_uplink(u, f1_mac));
  };
  const int64_t ho_release_sf = envl("CELL_HO_RELEASE_SF", 20, 3, 100000);
  /* A frame's EVM (dB, negative), if the radio measured one (raw: dB*2, 0 =
   * none; the same convention as UeRxAttribution). */
  auto frame_evm_db = [](const Packet &pk) -> std::optional<double> {
    if (pk.RxAtrib.evm[0] == 0)
      return std::nullopt;
    return static_cast<double>(pk.RxAtrib.evm[0]) / 2.0;
  };
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
          if (a && a->status == devourer::cell::RachStatus::Granted) {
            /* Its home: the (authenticated) MAC it joined with. */
            std::lock_guard<std::mutex> lk(f1_mu);
            ue_home[a->ue] = devourer::cell::HomeId{true, 0, 0, a->mac};
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
          /* Only an authentic one counts (its AREA tag: this DU does not hold
           * the other cell's key) — as heard, and for DAPS forwarding of a
           * handing-over UE under its other identity. Per-cell keys: with
           * THAT UE's own area key (from the CU), none = not counted. */
          CellMac amac = area;
          if (per_cell_keys) {
            std::lock_guard<std::mutex> lk(f1_mu);
            auto it = ue_area_tab.find({oc, ou});
            amac = it != ue_area_tab.end() ? it->second : CellMac{};
            if (!amac)
              return;
          }
          const auto ob = parse_ue_beacon(p.Data.data(), p.Data.size(), oc, amac, true);
          if (!ob)
            return;
          bool fwd = false;
          std::optional<uint16_t> arrival; /* staged here: its id */
          {
            std::lock_guard<std::mutex> lk(f1_mu);
            if (auto ai = arrival_of.find({oc, ou}); ai != arrival_of.end())
              arrival = ai->second;
            meas_tab.update(oc, ou, static_cast<double>(p.RxAtrib.rssi[0]) - 110.0,
                            static_cast<double>(p.RxAtrib.snr[0]) / 2.0 /* raw dB*2 */, now_ms(),
                            frame_evm_db(p));
            fwd = daps_on && daps_ids.count({oc, ou}) > 0;
          }
          if (arrival && ob->dl) {
            std::lock_guard<std::mutex> lk(mu);
            status_prune(*arrival, *ob->dl, "source_id");
          }
          if (fwd) {
            forward_ul(oc, ou, p);
            /* A visit copy from a staged arrival, before its switch: answer at
             * once (the UE holds this channel a few ms) with the DL PDUs not
             * yet offered to it — broadcast, so nothing waits for an ACK. */
            std::optional<VisitAns> va;
            if (const auto at = rx_tsf(p.RxAtrib.tsfl)) {
              std::lock_guard<std::mutex> lk(f1_mu);
              if (auto it = visit_src.find({oc, ou}); it != visit_src.end() && *at / P < it->second.eff_sf)
                va = it->second;
            }
            if (va) {
              std::vector<uint32_t> offer;
              uint64_t ep = 0;
              std::optional<uint32_t> prev;
              {
                std::lock_guard<std::mutex> lk(mu);
                if (auto qi = dlq.find(va->target_ue); qi != dlq.end()) {
                  DlQueue &q = qi->second;
                  ep = q.epoch;
                  prev = q.offered;
                  for (uint32_t sn : q.sns)
                    if ((!q.offered || devourer::cell::seq_lt(*q.offered, sn)) && offer.size() < kDlMaxPdus)
                      offer.push_back(sn);
                  if (!offer.empty())
                    q.offered = offer.back();
                }
              }
              if (!offer.empty()) {
                /* Aired by the answer thread: never send from this RX callback
                 * (the RX loop recycles the TX pages; a send from inside it
                 * froze the DU on the rig). */
                auto vf = build_visit_dl_frame(c.cell, static_cast<uint8_t>(va->target_ue), 0, offer, ep, auth);
                auto pkt = build_dl_radiotap(6);
                pkt.insert(pkt.end(), vf.begin(), vf.end());
                bool queued = false;
                {
                  std::lock_guard<std::mutex> lk(ans_mu);
                  if (ans_q.size() < 8) {
                    ans_q.push_back({std::move(pkt), va->target_ue, offer.size(), offer.front(), prev, offer.back(),
                                     std::chrono::steady_clock::now() + std::chrono::milliseconds(3)});
                    queued = true;
                  }
                }
                if (queued)
                  ans_cv.notify_one();
                else
                  unoffer(va->target_ue, prev, offer.back());
              }
            }
            char l[140];
            snprintf(l, sizeof l, "{\"ev\":\"cell.daps.fwd\",\"cell\":%u,\"ue\":%u,\"rssi\":%d}", oc, ou,
                     static_cast<int>(p.RxAtrib.rssi[0]) - 110);
            emit(l);
          }
          return;
        }
      }
      const auto ub = parse_ue_beacon(p.Data.data(), p.Data.size(), c.cell, auth);
      if (!ub)
        return;
      if (f1_on) { /* an own UE's (verified) uplink: its serving measurement */
        {
          std::lock_guard<std::mutex> lk(f1_mu);
          meas_tab.update(c.cell, ub->ue, static_cast<double>(p.RxAtrib.rssi[0]) - 110.0,
                          static_cast<double>(p.RxAtrib.snr[0]) / 2.0 /* raw dB*2 */, now_ms(),
                          frame_evm_db(p));
        }
        forward_ul(c.cell, ub->ue, p); /* the serving path: always (F1 on) */
        if (auto mr = devourer::cell::find_mr_ie(p.Data.data(), p.Data.size(), c.cell, ub->ue, auth)) {
          /* Fresh only: a generation newer than the last one taken from this
           * UE, and near this DU's own superframe count (a replayed old
           * report is refused). */
          const auto at = rx_tsf(p.RxAtrib.tsfl);
          const uint32_t sf_now = at ? static_cast<uint32_t>(*at / P) : 0;
          std::lock_guard<std::mutex> lk(f1_mu);
          auto &ur = ue_reports[ub->ue];
          const bool newer = ur.at_ms == 0 || static_cast<int32_t>(mr->gen - ur.gen) > 0;
          const bool near = at && std::abs(static_cast<int32_t>(mr->gen - sf_now)) < 100;
          if (newer && near)
            ur = {mr->cells, now_ms(), mr->gen};
        }
      }
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
        if (const auto *cx = reg.find_context(ub->ue)) {
          if (ta_on) /* CELL_TA=0: no loop to feed */
            reg.context(ub->ue).ta.sample(err);
          ta_now = cx->ta.ta();
        }
        /* its uplink quality, attributed to it (UeRegistry's window) */
        reg.rx_frame(p.Data.data(), p.Data.size(), p.RxAtrib.rssi[0], p.RxAtrib.snr[0], p.RxAtrib.evm[0],
                     p.RxAtrib.tsfl);
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
          /* A handover arrival's first uplink: the UE is here. What its
           * feedback did not ACK was aired before it switched (it switches up
           * to ~2 superframes after the instant); retransmitting those would
           * go before any new data — the first frames after a same-channel
           * switch were attempts 2-3 of PDUs it had. Given up instead, their
           * PDUs back at the head of the queue (the UE dedups by sn). */
          if (d->second.arriving) {
            d->second.arriving = false;
            const auto [gone, requeued] = arrival_abandon(d->second, ub->ue);
            char l[160];
            snprintf(l, sizeof l,
                     "{\"ev\":\"cell.ho.arrival_heard\",\"ue\":%u,\"sf\":%lld,\"abandoned\":%zu,\"requeued\":%zu}",
                     ub->ue, (long long)pos.superframe, gone, requeued);
            emit(l);
          }
          if (ub->dl)
            status_prune(ub->ue, *ub->dl, "own");
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
  ramp_t0 = std::chrono::steady_clock::now();
  if (ramp_t0_unix > 0) {
    const double now_unix = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    ramp_t0 -= std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(now_unix - ramp_t0_unix));
  }
  if (!tx_ramp.empty()) { /* the anchor used: checkable afterwards */
    char l[160];
    snprintf(l, sizeof l, "{\"ev\":\"cell.du.txpwr_t0\",\"anchor\":\"%s\",\"t0_unix\":%.3f}",
             ramp_t0_unix > 0 ? "unix" : "service_start", ramp_t0_unix);
    emit(l);
  }
  auto next_ta = std::chrono::steady_clock::now() + std::chrono::seconds(ta_every_s);
  auto next_rxw = next_ta; /* the per-UE uplink windows' cadence */
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
            r.channel = du_ch.load();
            r.phase_us = phase_live.load();
            r.period_us = P;
            r.gap_suspend_sf = static_cast<uint16_t>(std::min<int64_t>(gap_suspend_every, 65535));
            r.net_sf = f1_view.net_sf >= 0 ? static_cast<uint64_t>(f1_view.net_sf) : 0;
            r.net_off = f1_view.net_sf >= 0 ? net_off.load() : 0;
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
            r.per_cell_keys = per_cell_keys;
            /* The homes of exactly the UEs this report names, read under the
             * same lock (one view). */
            for (const auto &u : r.ues)
              if (auto h = ue_home.find(u.ue); h != ue_home.end() && r.homes.size() < f1::kMaxUes)
                r.homes.push_back({u.ue, h->second});
            f1::fill_report_measurements(r, c.cell, meas_tab.snapshot(now_ms()));
            for (auto &q : r.meas) /* each own UE's TA now (carried on a handover, v17) */
              if (auto t = ta_report.find(q.ue); t != ta_report.end())
                q.ta_us = t->second;
            /* The UEs' own measurements (their MR IEs): each entry's age is its
             * sample's plus the report's since; each UE's serving entry first,
             * whole UEs only against the cap (a truncation is logged). */
            size_t dropped_ues = 0;
            for (auto it = ue_reports.begin(); it != ue_reports.end();) {
              const int64_t rep_age = now_ms() - it->second.at_ms;
              if (it->second.at_ms == 0 || rep_age > 10000) {
                it = ue_reports.erase(it);
                continue;
              }
              std::vector<f1::UeCellMeas> mine;
              for (int serving_pass = 1; serving_pass >= 0; --serving_pass)
                for (const auto &e : it->second.m)
                  if ((e.cell == c.cell) == (serving_pass == 1))
                    mine.push_back({it->first, e.cell, static_cast<int16_t>(e.rssi_dbm * 4),
                                    static_cast<int16_t>(e.snr_db * 4),
                                    static_cast<uint16_t>(std::min<int64_t>(e.age_ms + rep_age, 65535))});
              if (r.ue_meas.size() + mine.size() <= f1::kMaxUeMeas)
                r.ue_meas.insert(r.ue_meas.end(), mine.begin(), mine.end());
              else
                ++dropped_ues;
              ++it;
            }
            if (dropped_ues) {
              char l[120];
              snprintf(l, sizeof l, "{\"ev\":\"cell.f1.ue_meas_truncated\",\"ues\":%zu}", dropped_ues);
              emit(l);
            }
            if (f1_view.net_sf < 0) /* no network superframe yet: nothing to plan on */
              r.ues.clear();
          }
          send(f1::encode_report(r, f1_mac));
        }
        pollfd pf{f1_fd.fd, POLLIN, 0};
        if (poll(&pf, 1, 50) <= 0 || !(pf.revents & POLLIN))
          continue;
        uint8_t buf[f1::kMaxDatagram];
        sockaddr_storage from{};
        socklen_t fl = sizeof from;
        const ssize_t n = recvfrom(f1_fd.fd, buf, sizeof buf, 0, reinterpret_cast<sockaddr *>(&from), &fl);
        if (n <= 0)
          continue;
        if (!f1_from_cu(from))
          continue;
        if (f1::peek_type(buf, static_cast<size_t>(n)) == f1::Type::Downlink) {
          auto dp = f1::decode_downlink(buf, static_cast<size_t>(n), f1_mac);
          if (!dp || dp->du != c.cell || dp->ue == 0 || dp->ue > 254)
            continue;
          std::lock_guard<std::mutex> lk(mu);
          if (dlq.size() >= f1::kMaxUes && !dlq.count(dp->ue))
            continue; /* no room for another UE's queue */
          DlQueue &q = dlq[dp->ue];
          const bool fresh_q = q.sns.empty() && q.epoch == 0;
          if (fresh_q || dp->epoch > q.epoch) { /* a new stream: the old one's PDUs go */
            q.dropped += q.sns.size();
            q.sns.clear();
            q.offered.reset();
            q.epoch = dp->epoch;
          } else if (dp->epoch < q.epoch || !devourer::cell::seq_lt(q.last_sn, dp->sn)) {
            ++q.refused; /* an older stream, or not newer than one taken: a replay */
            continue;
          }
          q.last_sn = dp->sn;
          q.sns.push_back(dp->sn);
          q.last_push = std::chrono::steady_clock::now();
          q.at_ms[dp->sn] = now_ms();
          if (q.at_ms.size() > 4 * kDlQueueMax) {
            /* Forget the old: a PDU in HARQ flight may come back to the
             * queue (an arrival's abandon), so recent ones are kept too. */
            const int64_t nq = now_ms();
            for (auto a = q.at_ms.begin(); a != q.at_ms.end();)
              a = nq - a->second > 10000 && std::find(q.sns.begin(), q.sns.end(), a->first) == q.sns.end()
                      ? q.at_ms.erase(a)
                      : std::next(a);
          }
          const auto du_it = dlue.find(dp->ue);
          if (q.offered || du_it == dlue.end() || du_it->second.arriving) {
            /* a DAPS target's queue (staged, arriving, or answering visits):
             * when each PDU came, against the frames' picks */
            char l[120];
            snprintf(l, sizeof l, "{\"ev\":\"cell.dl.pdu_in\",\"ue\":%u,\"sn\":%u,\"t_ms\":%lld}", dp->ue, dp->sn,
                     (long long)now_ms());
            emit(l);
          }
          if (q.sns.size() > kDlQueueMax) {
            q.sns.pop_front();
            ++q.dropped;
          }
          continue;
        }
        if (f1::peek_type(buf, static_cast<size_t>(n)) == f1::Type::AreaKeys) {
          auto ak = f1::decode_area_keys(buf, static_cast<size_t>(n), f1_mac);
          if (!ak || ak->du != c.cell || !auth)
            continue;
          std::map<std::pair<uint16_t, uint16_t>, CellMac> t;
          for (const auto &e : ak->keys)
            if (auto k = devourer::cell::wrap_area_key(auth, e.cell, e.ue, e.nonce, e.wrapped))
              t[{e.cell, e.ue}] = hmac_sha1(devourer::cell::key_to_hex(*k));
          std::lock_guard<std::mutex> lk(f1_mu);
          if (have_ak_seq && ak->seq <= last_ak_seq)
            continue; /* a replay or reordering */
          have_ak_seq = true;
          last_ak_seq = ak->seq;
          ue_area_tab = std::move(t);
          continue;
        }
        if (f1::peek_type(buf, static_cast<size_t>(n)) == f1::Type::Neighbours) {
          auto nb = f1::decode_neighbours(buf, static_cast<size_t>(n), f1_mac);
          if (!nb || nb->du != c.cell)
            continue;
          /* Under f1_mu, as a phase move recomputes them: offsets built for
           * an old phase must never land after the move's. */
          std::lock_guard<std::mutex> lk(f1_mu);
          std::vector<devourer::cell::Neighbour> v;
          for (const auto &e : nb->cells)
            v.push_back({e.cell, e.channel,
                         devourer::cell::neighbour_offset_us(e.phase_us, phase_live.load(), P)});
          if (have_nb_seq && nb->seq <= last_nb_seq)
            continue; /* a replay or reordering */
          have_nb_seq = true;
          last_nb_seq = nb->seq;
          nb_raw = nb->cells; /* their phases: the offsets again after a phase move */
          if (v != nb_list) {
            nb_list = std::move(v);
            ++nl_gen;
          }
          continue;
        }
        if (f1::peek_type(buf, static_cast<size_t>(n)) == f1::Type::ChannelSwitch) {
          auto cs = f1::decode_channel_switch(buf, static_cast<size_t>(n), f1_mac);
          if (!cs || cs->du != c.cell)
            continue;
          f1::AckStatus st = f1::AckStatus::Staged;
          int64_t net_now = -1;
          const char *why = "";
          {
            std::lock_guard<std::mutex> lk(f1_mu);
            net_now = f1_view.net_sf;
            const bool same = cs_staged && cs_staged->id == cs->id;
            const int64_t eff_new = static_cast<int64_t>(cs->effective_net_sf) - net_off.load();
            const bool reused = same && (cs_staged->channel != cs->channel || cs_staged->eff_sf != eff_new ||
                                         cs_staged->phase != cs->phase_us);
            /* A phase move: on this channel, at an instant idle in both grids. */
            const bool rephase = cs->phase_us >= 0;
            const auto rp = rephase && cs->phase_us < P
                                ? devourer::cell::rephase_plan(phase_live.load(), cs->phase_us, grid_span_us, 1000, P)
                                : std::nullopt;
            if (have_cs_seq && cs->seq <= last_cs_seq) {
              st = f1::AckStatus::Stale; /* a replay or reordering */
            } else {
              have_cs_seq = true;
              last_cs_seq = cs->seq;
              const int64_t eff = static_cast<int64_t>(cs->effective_net_sf) - net_off.load();
              if (reused) {
                st = f1::AckStatus::Invalid, why = "id_reused"; /* another move under a staged id */
              } else if (same) {
                st = f1::AckStatus::Staged; /* a resend of the one staged */
              } else if (net_now < 0) {
                st = f1::AckStatus::Late, why = "no_clock";
              } else if (static_cast<int64_t>(cs->effective_net_sf) < net_now + 4) {
                st = f1::AckStatus::Late, why = "lead"; /* announced too few superframes ahead */
              } else if ((rephase ? cs->channel != du_ch.load() : cs->channel == du_ch.load()) ||
                         !ho_in.empty() || !ho_out.empty() || (cs_staged && cs_staged->id > cs->id)) {
                st = f1::AckStatus::Invalid, why = "busy_or_same"; /* a phase move: on its own channel only */
              } else if (rephase && !rp) {
                st = f1::AckStatus::Invalid, why = "no_window"; /* no instant idle in both grids */
              } else if (!rephase && !devourer::cell::cs_retune_at_us(grid_span_us, cs_retune_us, 1000, P)) {
                st = f1::AckStatus::Invalid, why = "no_window"; /* the retune would not fit: never announced */
              } else {
                cs_staged = CsStaged{cs->id, cs->channel, eff, 0, rephase ? cs->phase_us : -1,
                                     rp ? rp->at_us : 0, rp ? rp->step_us : 0};
              }
            }
          }
          send(f1::encode_ack({c.cell, f1::AckKind::ChannelSwitch, cs->id, st,
                               net_now < 0 ? 0 : static_cast<uint64_t>(net_now)},
                              f1_mac));
          char l[200];
          snprintf(l, sizeof l,
                   "{\"ev\":\"cell.cs.cmd\",\"id\":%u,\"channel\":%u,\"phase_us\":%d,\"effective_net_sf\":%llu,"
                   "\"status\":\"%s\",\"why\":\"%s\"}",
                   cs->id, cs->channel, cs->phase_us, (unsigned long long)cs->effective_net_sf,
                   f1::ack_status_name(st), why);
          emit(l);
          continue;
        }
        if (f1::peek_type(buf, static_cast<size_t>(n)) == f1::Type::Handover) {
          auto h = f1::decode_handover(buf, static_cast<size_t>(n), f1_mac);
          if (!h || h->first != c.cell)
            continue;
          bool moving = false;
          {
            std::lock_guard<std::mutex> lk(f1_mu);
            moving = cs_staged.has_value();
          }
          if (moving) { /* a channel move staged: no handover across it */
            send(f1::encode_ack({c.cell, f1::AckKind::Handover, h->second.id, f1::AckStatus::Invalid, 0}, f1_mac));
            continue;
          }
          const devourer::cell::HoCommand &cmd = h->second;
          f1::AckStatus st = f1::AckStatus::Invalid;
          const char *role = cmd.target == c.cell ? "target" : cmd.source == c.cell ? "source" : "none";
          int64_t net_now = -1;
          bool fresh = true, again = false;
          std::vector<devourer::cell::CoordAssign> live;
          {
            std::lock_guard<std::mutex> lk(f1_mu);
            if (have_ho_seq && cmd.seq <= last_ho_seq) {
              fresh = false; /* a replay or reordering: changes nothing */
              st = f1::AckStatus::Stale;
            } else {
              have_ho_seq = true;
              last_ho_seq = cmd.seq;
            }
            net_now = f1_view.net_sf;
            live = f1_view.ues;
            again = (cmd.target == c.cell && ho_in.count(cmd.target_ue) &&
                     ho_in[cmd.target_ue].id == cmd.id) ||
                    (cmd.source == c.cell && ho_out.count(cmd.ue) && ho_out[cmd.ue].id == cmd.id);
          }
          const auto off = net_off.load();
          const auto pos_free = [&](uint8_t pos) {
            for (const auto &u : live)
              if (u.pos == pos)
                return false;
            return true;
          };
          if (!fresh) {
          } else if (cmd.target == c.cell && cmd.target_pos == 0) {
            /* Cancel (the handover failed at the source): drop the UE this
             * id pre-admitted, staged or already published. */
            bool staged_only = false;
            {
              std::lock_guard<std::mutex> lk(f1_mu);
              /* Only what THIS id staged or published: never another UE that
               * holds the same id here. */
              auto it = ho_in.find(cmd.target_ue);
              auto pub = ho_published.find(cmd.target_ue);
              if (it != ho_in.end() && it->second.id == cmd.id) {
                ho_in.erase(it);
                staged_only = true;
              } else if (pub != ho_published.end() && pub->second == cmd.id) {
                ho_cancel.insert(cmd.target_ue);
              }
            }
            if (staged_only) {
              std::lock_guard<std::mutex> lk(mu);
              reg.release_id(cmd.target_ue);
            }
            role = "cancel";
            st = f1::AckStatus::Staged;
          } else if (again) {
            st = f1::AckStatus::Staged; /* the CU resending: already staged */
          } else if (net_now < 0 ||
                     cmd.effective_net_sf < static_cast<uint64_t>(net_now) + f1c.min_lead_sf) {
            st = f1::AckStatus::Late;
          } else if (cmd.target == c.cell) {
            bool known = false;
            for (const auto &u : live)
              known = known || u.ue == cmd.target_ue;
            if (!known && cmd.target_pos >= 1 && cmd.target_pos <= c.capacity &&
                pos_free(cmd.target_pos) && cmd.target_ue >= 1 && cmd.target_ue <= 254) {
              /* Recorded BEFORE the registry admit, so the main loop can never
               * see the admitted UE without its arrival and grant it as an
               * ordinary joiner; undone if the admit does not give the id. */
              const auto eff = static_cast<uint64_t>(static_cast<int64_t>(cmd.effective_net_sf) - off);
              {
                std::lock_guard<std::mutex> lk(f1_mu);
                ho_in[cmd.target_ue] = {cmd.id, cmd.target_pos, eff, cmd.ta_us, cmd.conditional()};
              }
              /* Pre-admit under exactly the id the CU gave it (the registry
               * hands out the lowest free one; the CU chose that one). */
              std::array<uint8_t, 6> um{};
              cell_mac(um.data(), c.cell, static_cast<uint8_t>(cmd.target_ue));
              bool got = false;
              {
                std::lock_guard<std::mutex> lk(mu);
                auto u = reg.admit(um);
                got = u && u->ue == cmd.target_ue;
                if (u && !got)
                  reg.release(um);
              }
              if (got) {
                st = f1::AckStatus::Staged;
                std::lock_guard<std::mutex> lk(f1_mu);
                /* DAPS: the UE's source identity, heard here until the switch
                 * (+ the release window, for its last old-identity beacons). */
                daps_ids[{cmd.source, cmd.ue}] = static_cast<int64_t>(eff) + ho_release_sf;
                arrival_of[{cmd.source, cmd.ue}] = cmd.target_ue;
                /* Answered only across channels (the source's channel, from
                 * this DU's neighbour list): on one channel the target hears
                 * every uplink of the UE, and an answer there would land in
                 * the source cell's schedule. */
                for (const auto &nb : nb_list)
                  if (nb.cell == cmd.source && nb.channel != 0 && nb.channel != du_ch.load())
                    visit_src[{cmd.source, cmd.ue}] = {cmd.target_ue, static_cast<int64_t>(eff)};
                ue_home[cmd.target_ue] = cmd.home; /* reported on, for its next handover */
              } else {
                std::lock_guard<std::mutex> lk(f1_mu);
                ho_in.erase(cmd.target_ue);
              }
            }
          } else if (cmd.source == c.cell) {
            bool known = false;
            for (const auto &u : live)
              known = known || u.ue == cmd.ue;
            const auto dl = grid->find(SlotKind::Downlink, cmd.target_pos);
            const auto ul = grid->find(SlotKind::Uplink, cmd.target_pos);
            if (known && dl && ul) {
              /* The target's grid is this grid (one env per network): its
               * slot indices for that position are ours. */
              devourer::cell::HoIe ie;
              ie.id = cmd.id;
              ie.cell = c.cell;
              ie.ue = cmd.ue;
              ie.target = cmd.target;
              ie.target_ue = cmd.target_ue;
              ie.dl_slot = static_cast<uint8_t>(*dl);
              ie.ul_slot = static_cast<uint8_t>(*ul);
              ie.effective_sf = static_cast<uint64_t>(static_cast<int64_t>(cmd.effective_net_sf) - off);
              ie.tsf_delta_us = cmd.tsf_delta_us;
              ie.channel = cmd.target_channel != du_ch.load() ? cmd.target_channel : 0;
              ie.ctx = cmd.ctx; /* the target's key for this UE: relayed, opaque */
              ie.cond_lost_sf = cmd.cond_lost_sf; /* conditional: relayed */
              ie.cond_margin_db = cmd.cond_margin_db;
              ie.cond_ttt_ms = cmd.cond_ttt_ms;
              auto bytes = devourer::cell::encode_ho_ie(ie, auth);
              if (!bytes.empty()) {
                std::lock_guard<std::mutex> lk(f1_mu);
                ho_out[cmd.ue] = {cmd.id, ie.effective_sf, cmd.target, cmd.target_ue, bytes, cmd.conditional()};
                /* DAPS: its target identity, heard here after the switch. */
                daps_ids[{cmd.target, cmd.target_ue}] =
                    static_cast<int64_t>(ie.effective_sf) + ho_release_sf;
                st = f1::AckStatus::Staged;
              }
            }
          }
          send(f1::encode_ack({c.cell, f1::AckKind::Handover, cmd.id, st,
                               net_now < 0 ? 0 : static_cast<uint64_t>(net_now)},
                              f1_mac));
          if (!again) {
            char l[240];
            snprintf(l, sizeof l,
                     "{\"ev\":\"cell.ho.cmd\",\"id\":%u,\"role\":\"%s\",\"ue\":%u,"
                     "\"target\":%u,\"target_ue\":%u,\"pos\":%u,\"effective_net_sf\":%llu,"
                     "\"status\":\"%s\"}",
                     cmd.id, role, cmd.ue, cmd.target, cmd.target_ue, cmd.target_pos,
                     (unsigned long long)cmd.effective_net_sf, f1::ack_status_name(st));
            emit(l);
          }
          continue;
        }
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
        send(f1::encode_ack({c.cell, f1::AckKind::Schedule, s->version, st,
                             net_now < 0 ? 0 : static_cast<uint64_t>(net_now)},
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
    /* In DL-slot order (the slots are in time order): in grant order a UE
     * whose slot comes first in the superframe was served after the other's
     * slot, ~4 ms late into it (a handed-over UE at position 1 of a DU whose
     * own UE has position 2). */
    std::sort(served.begin(), served.end(),
              [&](uint16_t a, uint16_t b) { return serving.find(a)->dl_slot < serving.find(b)->dl_slot; });
    /* Pace the pass to its superframe when there is no DL to wait for (a
     * DU with no UE, e.g. after its last one handed over): `sf` must track
     * the TSF, or the network superframe it reports runs away. */
    if (served.empty())
      sleep_until_tsf(dev, grid->slot_start_tsf(sf, grid->slots().size() > 1 ? 1 : 0));
    std::vector<devourer::cell::Neighbour> nl_dl; /* this DU's own NL, for suspend gaps */
    if (gap_suspend_every > 0 && f1_on) {
      std::lock_guard<std::mutex> lk(f1_mu);
      nl_dl = nb_list;
    }
    for (uint16_t ue : served) {
      const size_t idx = serving.find(ue)->dl_slot;
      if (gap_suspend_every > 0 && devourer::cell::suspend_dl_skip(sf, ue, gap_suspend_every)) {
        const Grant *sg = serving.find(ue);
        const auto &sl = grid->slots();
        const std::vector<devourer::cell::GridSpan> busy = {
            {0, c.bcn_us}, {sl[sg->dl_slot].start_us, sl[sg->dl_slot].len_us},
            {sl[sg->ul_slot].start_us, sl[sg->ul_slot].len_us}};
        if (devourer::cell::needs_suspend(nl_dl, du_ch.load(), gap_floor_us, gap_guard_du_us, P, busy)) {
          char l[120];
          snprintf(l, sizeof l, "{\"ev\":\"cell.dl.suspend\",\"ue\":%u,\"sf\":%lld}", ue, (long long)sf);
          emit(l);
          continue; /* the UE is (or may be) on the neighbour's channel */
        }
      }
      const int64_t start = grid->slot_start_tsf(sf, idx);
      /* The frame is picked and built just before its slot, not right after
       * the previous one: a CU PDU that arrives in between rides THIS frame,
       * not the next superframe's (after a DAPS switch that was the first
       * target frame missing the next PDU: ~2 superframes of DL gap). */
      sleep_until_tsf(dev, start - dl_pick_lead_us);
      /* What to send: a due HARQ retransmission, else new data (or, with
       * HARQ off, a new frame every superframe). */
      std::optional<devourer::cell::HarqTx> tx;
      std::vector<uint32_t> pdus; /* the CU DL PDUs this frame carries */
      uint64_t pdu_epoch = 0;
      size_t q_left = 0;
      std::optional<uint32_t> offered_mark; /* the queue's DAPS-visit offered mark (logged) */
      const long long pick_ms = static_cast<long long>(now_ms());
      {
        std::lock_guard<std::mutex> lk(mu);
        DlUe &d = dlue.at(ue);
        if (harq_on) {
          /* Not heard yet: nothing aired to it is known to have arrived, so
           * every frame is a new one with the freshest PDUs, not a
           * retransmission of a frame aired before it switched. */
          if (d.arriving)
            arrival_abandon(d, ue);
          tx = d.harq.next(static_cast<uint64_t>(sf), true);
          /* A classed UE's retransmission past its discard timer (counted
           * from the first attempt) is given up, not aired late. */
          if (auto pol = qos_pol.find(ue); pol != qos_pol.end())
            for (int k = 0; k < 32 && tx && tx->retx; ++k) {
              auto f = d.first_ms.find(tx->seq);
              const int64_t age = f == d.first_ms.end() ? 0 : now_ms() - f->second;
              if (age <= pol->second.discard_ms)
                break;
              size_t n = 0;
              if (auto pi = d.pdus.find(tx->seq); pi != d.pdus.end()) {
                n = pi->second.second.size();
                d.pdus.erase(pi);
              }
              d.first_ms.erase(tx->seq);
              d.harq.abandon(tx->seq);
              char l[160];
              snprintf(l, sizeof l,
                       "{\"ev\":\"cell.dl.discard\",\"ue\":%u,\"sf\":%lld,\"n\":%zu,\"oldest_ms\":%lld,\"retx\":1}",
                       ue, (long long)sf, n, (long long)age);
              emit(l);
              tx = d.harq.next(static_cast<uint64_t>(sf), true);
            }
        } else {
          const uint32_t s = d.plain_seq++;
          tx = devourer::cell::HarqTx{s, s % devourer::cell::kHarqProcesses, s, 1, false};
        }
        if (tx) {
          if (tx->retx) {
            if (auto pi = d.pdus.find(tx->seq); pi != d.pdus.end()) {
              pdu_epoch = pi->second.first;
              pdus = pi->second.second;
            }
          } else if (auto qi = dlq.find(ue); qi != dlq.end()) {
            pdu_epoch = qi->second.epoch;
            offered_mark = qi->second.offered;
            /* PDUs a DAPS-visit answer has not offered go first; the offered
             * ones (the UE most likely took them before the switch) fill the
             * rest, still sent in case an answer was lost — the UE takes a
             * late copy within its reorder window. Without answers this is
             * plain queue order. */
            /* The QoS discard timer: a PDU queued past its UE's budget is
             * dropped, not sent late (counted, logged per frame). */
            if (auto pol = qos_pol.find(ue); pol != qos_pol.end()) {
              const int64_t now_q = now_ms();
              size_t n = 0;
              int64_t oldest = 0;
              auto &dq = qi->second.sns;
              for (auto it = dq.begin(); it != dq.end();) {
                auto a = qi->second.at_ms.find(*it);
                if (a != qi->second.at_ms.end() && now_q - a->second > pol->second.discard_ms) {
                  oldest = std::max(oldest, now_q - a->second);
                  qi->second.at_ms.erase(a);
                  it = dq.erase(it);
                  ++n;
                } else {
                  ++it;
                }
              }
              if (n) {
                qi->second.discarded += n;
                char l[140];
                snprintf(l, sizeof l, "{\"ev\":\"cell.dl.discard\",\"ue\":%u,\"sf\":%lld,\"n\":%zu,\"oldest_ms\":%lld}",
                         ue, (long long)sf, n, (long long)oldest);
                emit(l);
              }
            }
            pdus = pick_dl_pdus(qi->second.sns, qi->second.offered, kDlMaxPdus);
            if (harq_on && !pdus.empty())
              d.pdus[tx->seq] = {pdu_epoch, pdus};
            if (harq_on)
              d.first_ms[tx->seq] = now_ms();
          }
          /* Resolved seqs (below the window's low edge) carry nothing more. */
          for (auto pi = d.pdus.begin(); pi != d.pdus.end();)
            pi = devourer::cell::seq_lt(pi->first, tx->low) ? d.pdus.erase(pi) : std::next(pi);
          for (auto fi = d.first_ms.begin(); fi != d.first_ms.end();)
            fi = devourer::cell::seq_lt(fi->first, tx->low) ? d.first_ms.erase(fi) : std::next(fi);
          if (auto qi = dlq.find(ue); qi != dlq.end())
            q_left = qi->second.sns.size();
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
                              auth, pdus, pdu_epoch);
      std::vector<uint8_t> pkt = build_dl_radiotap(
          rate, ue == atten_ue && atten_db != 0 ? std::optional<int8_t>(atten_db) : std::nullopt);
      pkt.insert(pkt.end(), f.begin(), f.end());
      const int64_t sub = sleep_until_tsf(dev, start);
      /* Ready too late (the pick and build run after the lead's wait; a rare
       * OS stall of a few hundred µs was seen before this, too): not aired
       * past its slot's guard — its PDUs and HARQ attempt go back. */
      const bool late = sub - start > dl_guard_us;
      if (late) {
        char l[160];
        snprintf(l, sizeof l, "{\"ev\":\"cell.dl.late_skip\",\"ue\":%u,\"sf\":%lld,\"late_us\":%lld}", ue,
                 (long long)sf, (long long)(sub - start));
        emit(l);
      }
      const bool ok = !late && dev->send_packet(pkt.data(), pkt.size());
      if (!ok) {
        std::lock_guard<std::mutex> lk(mu);
        if (harq_on)
          dlue.at(ue).harq.unsend(*tx); /* never reached the radio: no attempt spent */
        if (!tx->retx && !pdus.empty()) { /* its PDUs go back, first in line */
          if (harq_on)
            dlue.at(ue).pdus.erase(tx->seq);
          if (auto qi = dlq.find(ue); qi != dlq.end() && qi->second.epoch == pdu_epoch)
            qi->second.sns.insert(qi->second.sns.begin(), pdus.begin(), pdus.end());
        }
      }
      /* After DAPS-visit answers: what the frame carried against the mark. */
      std::string pick;
      if (offered_mark && !pdus.empty()) {
        size_t fresh = 0;
        for (uint32_t sn : pdus)
          fresh += devourer::cell::seq_lt(*offered_mark, sn) ? 1 : 0;
        pick = ",\"sn0\":" + std::to_string(pdus.front()) + ",\"fresh\":" + std::to_string(fresh) +
               ",\"offered\":" + std::to_string(*offered_mark) + ",\"pick_ms\":" + std::to_string(pick_ms);
      }
      char l[320];
      snprintf(l, sizeof l,
               "{\"ev\":\"cell.dl\",\"ue\":%u,\"sf\":%lld,\"seq\":%u,\"attempt\":%d,"
               "\"rate\":%d,\"late_us\":%lld,\"ok\":%d,\"pdus\":%zu,\"q\":%zu%s}",
               ue, (long long)sf, tx->seq, tx->attempt, rate, (long long)(sub - start), ok ? 1 : 0,
               pdus.size(), q_left, pick.c_str());
      emit(l);
    }
    /* DL queues of UEs this DU does not serve and the CU stopped feeding
     * (handed over, or a cancelled handover's staged UE) go after 10 s. */
    {
      std::lock_guard<std::mutex> lk(mu);
      const auto now_q = std::chrono::steady_clock::now();
      for (auto qi = dlq.begin(); qi != dlq.end();) {
        if (!dlue.count(qi->first) && now_q - qi->second.last_push > std::chrono::seconds(10)) {
          if (qi->second.dropped || qi->second.refused || !qi->second.sns.empty()) {
            char l[140];
            snprintf(l, sizeof l,
                     "{\"ev\":\"cell.dl.q_gone\",\"ue\":%u,\"left\":%zu,\"dropped\":%llu,\"refused\":%llu}",
                     qi->first, qi->second.sns.size(), (unsigned long long)qi->second.dropped,
                     (unsigned long long)qi->second.refused);
            emit(l);
          }
          qi = dlq.erase(qi);
        } else {
          ++qi;
        }
      }
    }
    /* CELL_DU_TX_RAMP (test): the DU's TX power offset over time, beacons
     * and DL alike — what a UE crossing from this cell to another sees. */
    if (!tx_ramp.empty()) {
      const double ts = std::chrono::duration<double>(std::chrono::steady_clock::now() - ramp_t0).count();
      double want = tx_ramp.back().second;
      for (size_t i = 0; i < tx_ramp.size(); ++i) {
        if (ts < tx_ramp[i].first) {
          if (i == 0) {
            want = tx_ramp[0].second;
          } else {
            const auto &a = tx_ramp[i - 1], &b = tx_ramp[i];
            want = a.second + (b.second - a.second) * (ts - a.first) / (b.first - a.first);
          }
          break;
        }
      }
      const int want_qdb = static_cast<int>(std::lround(want * 4));
      if (std::abs(want_qdb - ramp_set_qdb) >= 2 || ramp_set_qdb == INT_MIN) { /* 0.5 dB steps */
        const int applied = dev->SetTxPowerOffsetQdb(want_qdb);
        /* The family clamps to its PA window (the applied value says so,
         * logged below); retrying the same target cannot change that. */
        ramp_set_qdb = want_qdb;
        if (applied != want_qdb && !ramp_short_warned) {
          ramp_short_warned = true;
          fprintf(stderr, "CELL_DU_TX_RAMP: wanted %d qdB, applied %d (clamped)\n", want_qdb, applied);
        }
        char l[160];
        snprintf(l, sizeof l,
                 "{\"ev\":\"cell.du.txpwr\",\"sf\":%lld,\"t_s\":%.2f,\"want_qdb\":%d,\"applied_qdb\":%d}",
                 (long long)sf, ts, want_qdb, applied);
        emit(l);
      }
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
    std::vector<devourer::cell::Neighbour> nb_snap;
    uint32_t nl_gen_snap = 0;
    std::map<uint16_t, HoIn> ho_in_snap;
    std::map<uint16_t, HoOut> ho_out_snap;
    std::set<uint16_t> ho_cancel_snap;
    if (f1_on) {
      std::lock_guard<std::mutex> lk(f1_mu);
      f1_view.net_sf = net_ok.load() && health.state() != devourer::cell::SyncState::Lost
                           ? sf + net_off.load()
                           : -1;
      f1_view.sync_state = static_cast<uint8_t>(health.state());
      f1_view.coordinated = coordinated;
      for (auto it = daps_ids.begin(); it != daps_ids.end();)
        it = it->second < sf ? daps_ids.erase(it) : std::next(it);
      for (auto it = arrival_of.begin(); it != arrival_of.end();)
        it = daps_ids.count(it->first) ? std::next(it) : arrival_of.erase(it);
      for (auto it = visit_src.begin(); it != visit_src.end();)
        it = it->second.eff_sf < sf ? visit_src.erase(it) : std::next(it);
      nb_snap = nb_list;
      nl_gen_snap = nl_gen;
      ho_in_snap = ho_in;
      ho_out_snap = ho_out;
      /* Kept until the live map no longer has the UE (a deferred or failed
       * publish retries the removal). */
      for (auto it = ho_cancel.begin(); it != ho_cancel.end();)
        if (!map.find(*it)) {
          ho_published.erase(*it);
          it = ho_cancel.erase(it);
        } else {
          ++it;
        }
      ho_cancel_snap = ho_cancel;
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
    std::vector<uint8_t> ho_ies;     /* handover IEs to air (source side) */
    std::optional<CsStaged> csn;     /* a staged channel move, announced in ho_ies */
    std::vector<uint16_t> ho_done;   /* source-side handovers concluded */
    std::optional<uint64_t> ho_eff;  /* a handover arrival's instant (target side) */
    uint16_t ho_eff_ue = 0;
    std::vector<uint16_t> ho_expired; /* arrivals past their window */
    /* No handover is aired or published off the shared timebase. */
    const bool synced =
        !svc || (health.state() != devourer::cell::SyncState::Lost && net_ok.load());
    {
      std::lock_guard<std::mutex> lk(mu);
      /* Joiners the RACH server admitted: a grant, and their per-UE state. */
      for (const auto &u : reg.admitted())
        if (!next.find(u.ue)) {
          Grant g = grant_for(u);
          /* A handover arrival: published for its switch instant, in the
           * position the CU gave it, no earlier than the publish lead. */
          if (auto hi = ho_in_snap.find(u.ue); hi != ho_in_snap.end()) {
            /* Past its window (it can no longer be announced N+2 ahead): the
             * arrival expires, its registry id comes back; the UE, never
             * granted here, falls back to its source. */
            if (static_cast<uint64_t>(sf) + 2 > hi->second.eff_sf) {
              reg.release_id(u.ue);
              ho_expired.push_back(u.ue);
              continue;
            }
            /* Conditional: in force as soon as it can be announced (the UE
             * may come any time before the expiry, eff_sf). */
            const bool cond = hi->second.cond;
            if ((!cond && static_cast<uint64_t>(sf) + f1c.publish_lead_sf < hi->second.eff_sf) || f1_due ||
                ho_eff || !synced)
              continue; /* not yet (or another instant is being published, or no clock) */
            set_position(g, hi->second.pos);
            /* The UE's TA at its source: its clock and actuation phase do not
             * change with the cell, so the target's loop starts there. */
            g.ta_us = hi->second.ta_us;
            next.grants.push_back(g);
            reg.context(u.ue).ta.set_ta(hi->second.ta_us);
            if (hi->second.ta_us) {
              char l[120];
              snprintf(l, sizeof l, "{\"ev\":\"cell.ho.ta_carried\",\"ue\":%u,\"ta_us\":%d}", u.ue,
                       hi->second.ta_us);
              emit(l);
            }
            /* Fresh DL state (an id reused after another UE left keeps
             * nothing of it); its DL queue is the DAPS buffer: kept. */
            dlue.insert_or_assign(u.ue, make_dlue(u.ue));
            dlue.at(u.ue).arriving = true;
            /* The silence clock from the switch (conditional: the expiry, so
             * a UE that never came is released after it). */
            last_ul_sf[u.ue] = static_cast<int64_t>(hi->second.eff_sf);
            ho_eff = cond ? static_cast<uint64_t>(sf) + f1c.publish_lead_sf : hi->second.eff_sf;
            ho_eff_ue = u.ue;
            changed = true;
            continue;
          }
          /* Its default slot pair may be another UE's (the CU moved that one
           * there): then the first free one; the CU re-plans it. */
          for (const Grant &o : next.grants)
            if (position_of(o) == position_of(g)) {
              if (const uint8_t fp = free_position(next))
                set_position(g, fp);
              break;
            }
          next.grants.push_back(g);
          reg.context(u.ue);
          /* Fresh DL state; a joiner has no DL stream yet (the CU starts
           * one once this DU reports it): a queue left under its id is
           * another UE's. */
          dlue.insert_or_assign(u.ue, make_dlue(u.ue));
          dlq.erase(u.ue);
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
      /* Cancelled arrivals already in the map: released as departures. */
      for (uint16_t ue : ho_cancel_snap)
        for (auto it = next.grants.begin(); it != next.grants.end(); ++it)
          if (it->ue == ue) {
            next.grants.erase(it);
            to_release.push_back(ue);
            changed = true;
            break;
          }
      /* Source side: air each handover IE through its switch instant; after
       * it, release the UE only if it has stopped uplinking HERE (it took
       * the target's identity). If it is still heard as ours, it never
       * switched: it keeps its grant (make-before-break). */
      for (const auto &kv : ho_out_snap) {
        const uint16_t ue = kv.first;
        const HoOut &ho = kv.second;
        auto h = heard_ul_sf.find(ue);
        /* Conditional: the UE leaves whenever its condition holds, so a UE
         * silent here release_sf superframes has gone (or is lost to this
         * cell either way): released now, not held to the expiry — its grant,
         * the IE on air and the CU's view of it go with it. Once out of the
         * live map, the entry concludes. */
        const bool cond_gone = ho.cond && (!map.find(ue) || (h != heard_ul_sf.end() &&
                                                            sf - h->second > static_cast<int64_t>(ho_release_sf)));
        if (!cond_gone && static_cast<uint64_t>(sf) <= ho.eff_sf + 2) {
          if (synced)
            ho_ies.insert(ho_ies.end(), ho.ie.begin(), ho.ie.end());
          continue;
        }
        if (!cond_gone && static_cast<uint64_t>(sf) < ho.eff_sf + static_cast<uint64_t>(ho_release_sf))
          continue;
        /* Its last uplink as ours in the switch superframe itself (+2: the
         * UE's poll and its beacon update) is still the old identity. */
        const bool switched = cond_gone || h == heard_ul_sf.end() ||
                              h->second <= static_cast<int64_t>(ho.eff_sf) + 2;
        if (switched && map.find(ue)) {
          /* Released through the normal publish; this entry concludes once
           * the live map no longer has the UE (a deferred publish retries). */
          for (auto it = next.grants.begin(); it != next.grants.end(); ++it)
            if (it->ue == ue) {
              next.grants.erase(it);
              to_release.push_back(ue);
              changed = true;
              break;
            }
          continue;
        }
        ho_done.push_back(ue);
        char l[200];
        snprintf(l, sizeof l,
                 "{\"ev\":\"cell.ho.%s\",\"id\":%u,\"ue\":%u,\"target\":%u,\"target_ue\":%u,"
                 "\"sf\":%lld,\"last_heard_sf\":%lld}",
                 switched ? "released" : "kept", ho.id, ue, ho.target, ho.target_ue, (long long)sf,
                 (long long)(h == heard_ul_sf.end() ? -1 : h->second));
        emit(l);
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
          TaLoop &l = reg.context(g.ue).ta;
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
    /* Each UE's uplink quality over the window, from its own frames
     * (UeRegistry; the SA is 02:ce:11:<cell>:<ue>): closed on the TA cadence,
     * whether or not the TA loop runs (CELL_TA=0). */
    if (std::chrono::steady_clock::now() >= next_rxw) {
      next_rxw += std::chrono::seconds(ta_every_s);
      std::lock_guard<std::mutex> lk(mu);
      const auto wins = reg.close_rx_window([&](const std::array<uint8_t, 6> &ta) -> uint16_t {
        const bool ours = ta[0] == 0x02 && ta[1] == 0xce && ta[2] == 0x11 &&
                          static_cast<uint16_t>(ta[3] << 8 | ta[4]) == c.cell;
        return ours ? ta[5] : 0;
      });
      for (const auto &[ue, w] : wins) {
        char l[220];
        snprintf(l, sizeof l,
                 "{\"ev\":\"cell.ue_rx\",\"ue\":%u,\"frames\":%u,\"rssi_dbm\":%d,\"snr_db\":%.1f,"
                 "\"snr_min_db\":%.1f,\"snr_ok\":%d,\"evm_db\":%.1f,\"evm_ok\":%d}",
                 ue, w.frames, w.rssi_mean_dbm, w.snr_mean_db, w.snr_min_db, w.snr_valid ? 1 : 0, w.evm_mean_db,
                 w.evm_valid ? 1 : 0);
        emit(l);
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
        reg.context(g.ue).ta.set_ta(g.ta_us);
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
    /* A handover arrival that cannot go out with this map (its instant too
     * close, or an F1 schedule instant in the same map): only ITS grant
     * leaves `next`; every other change of the pass stands. It is retried
     * next pass, or expires past its window. */
    if (ho_eff && (static_cast<uint64_t>(sf) + 2 > *ho_eff || f1_eff)) {
      for (auto it = next.grants.begin(); it != next.grants.end(); ++it)
        if (it->ue == ho_eff_ue) {
          next.grants.erase(it);
          break;
        }
      {
        std::lock_guard<std::mutex> lk(mu);
        reg.reset_context(ho_eff_ue);
        dlue.erase(ho_eff_ue);
        last_ul_sf.erase(ho_eff_ue);
      }
      ho_eff.reset();
    }
    if (!ho_expired.empty()) {
      std::lock_guard<std::mutex> lk(f1_mu);
      for (uint16_t ue : ho_expired) {
        ho_in.erase(ue);
        char l[120];
        snprintf(l, sizeof l, "{\"ev\":\"cell.ho.target_expired\",\"ue\":%u,\"sf\":%lld}", ue,
                 (long long)sf);
        emit(l);
      }
    }
    /* The neighbour list rides every beacon on the shared timebase (its
     * offsets are network-time facts), with zero entries too: it is how UEs
     * learn a list went away, and its tag is what proves a beacon is this
     * cell's to anyone measuring it. */
    if (synced && f1_on) {
      const auto nl = devourer::cell::encode_nl_ie(c.cell, nl_gen_snap, nb_snap, area);
      ho_ies.insert(ho_ies.end(), nl.begin(), nl.end());
      /* A staged channel move, announced until its instant. */
      {
        std::lock_guard<std::mutex> lk(f1_mu);
        csn = cs_staged;
      }
      if (csn && sf < csn->eff_sf) {
        const auto cie = devourer::cell::encode_cs_ie(
            {csn->id, c.cell, csn->channel, static_cast<uint64_t>(csn->eff_sf),
             static_cast<int32_t>(csn->step_us), static_cast<int32_t>(csn->at_us)},
            auth);
        ho_ies.insert(ho_ies.end(), cie.begin(), cie.end());
      }
      /* Signed once per list and network window (its freshness: the
       * network superframe / kNlWindowSf, on the shared timebase). */
      const int64_t net_sf_now = net_ok.load() ? sf + net_off.load() : -1;
      const uint32_t win = net_sf_now >= 0
                               ? static_cast<uint32_t>(net_sf_now / devourer::cell::kNlWindowSf)
                               : 0;
      if (nl_sign && !nl.empty() && net_sf_now < 0 && !nl_unsigned_no_net) {
        /* No network superframe (no PTP service mode, or sync lost): no
         * window to sign, so the list airs unsigned and signed UEs refuse it. */
        nl_unsigned_no_net = true;
        emit("{\"ev\":\"cell.nl.unsigned_no_net\"}");
      }
      if (nl_sign && !nl.empty() && net_sf_now >= 0) { /* aired with the list */
        if (nl_unsigned_no_net) {
          nl_unsigned_no_net = false;
          emit("{\"ev\":\"cell.nl.signed_again\"}");
        }
        if (nl != nl_signed_for || win != nl_signed_win) {
          auto sig = devourer::cell::encode_nl_sig_ie(c.cell, nl_gen_snap, win, nl, nl_sign);
          if (!sig.empty()) {
            nl_sig_ie = std::move(sig);
            nl_signed_for = nl;
            nl_signed_win = win;
          } else if (!nl_sign_failed) { /* retried next beacon; logged once */
            nl_sign_failed = true;
            emit("{\"ev\":\"cell.nl.sign_fail\"}");
          }
        }
        /* Only a signature over THIS list and window: never an older one. */
        if (nl == nl_signed_for && win == nl_signed_win) {
          ho_ies.insert(ho_ies.end(), nl_sig_ie.begin(), nl_sig_ie.end());
          ho_ies.insert(ho_ies.end(), nl_cert_ie.begin(), nl_cert_ie.end());
        }
      }
    }
    if (changed || rar_now != rar_on_air || ho_ies != ho_on_air) {
      if (changed) {
        ++next.epoch;
        next.effective_sf = f1_eff   ? *f1_eff
                            : ho_eff ? *ho_eff
                                     : static_cast<uint64_t>(sf + 3);
      }
      const auto nb = build_du_beacon(next, auth, rar_now, ho_ies);
      const bool ok = !nb.empty() && dev->UpdateBeaconPayload(nb.data(), nb.size());
      beacon_fail_run = ok ? 0 : beacon_fail_run + 1;
      if (ok)
        live_bcn = nb;
      if (ok) { /* what the live beacon now announces (nothing, if the IE went) */
        std::lock_guard<std::mutex> lk(f1_mu);
        cs_aired_id = csn && sf < csn->eff_sf && cs_staged && cs_staged->id == csn->id ? csn->id : 0;
      }
      if (ok) {
        rar_on_air = rar_now;
        rar_on_air_set = rar_sel;
        ho_on_air = ho_ies;
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
          {
            /* The TA each UE runs is its grant's in the map now on air: the
             * report carries exactly that (a carried arrival's at once, a
             * released ID's never, a failed publish's not at all). */
            std::lock_guard<std::mutex> lk(f1_mu);
            ta_report.clear();
            for (const Grant &g : map.grants)
              ta_report[g.ue] = g.ta_us;
          }
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
          if (ho_eff) {
            std::lock_guard<std::mutex> lk(f1_mu);
            const auto it = ho_in.find(ho_eff_ue);
            char l[200];
            snprintf(l, sizeof l,
                     "{\"ev\":\"cell.ho.target_staged\",\"id\":%u,\"ue\":%u,\"pos\":%u,"
                     "\"effective_sf\":%llu,\"sf\":%lld}",
                     it != ho_in.end() ? it->second.id : 0, ho_eff_ue,
                     it != ho_in.end() ? it->second.pos : 0, (unsigned long long)*ho_eff,
                     (long long)sf);
            emit(l);
            if (it != ho_in.end()) {
              ho_published[ho_eff_ue] = it->second.id;
              ho_in.erase(it);
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
            reg.context(g.ue).ta.set_ta(g.ta_us);
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
    if (!ho_done.empty()) {
      std::lock_guard<std::mutex> lk(f1_mu);
      for (uint16_t ue : ho_done)
        ho_out.erase(ue);
    }
    /* The answers on air this superframe (after a failed update: the
     * previous set, still airing) count one publication. */
    if (rach_on && !rar_on_air_set.empty()) {
      std::lock_guard<std::mutex> lk(mu);
      srv.published(rar_on_air_set);
    }
    /* A staged channel move: in the idle part of the superframe before its
     * instant (after every slot), the beacon stops, the radio retunes and the
     * beacon restarts — the TSF does not move, so neither does the grid. Only
     * a move its UEs were told of (its IE aired in >= 3 superframes) is made;
     * a beacon that will not restart on the new channel takes the DU back to
     * the old one. The CU gets the outcome (Applied, or Invalid). */
    {
      std::optional<CsStaged> cs;
      {
        std::lock_guard<std::mutex> lk(f1_mu);
        if (cs_staged && cs_aired_id == cs_staged->id && sf < cs_staged->eff_sf)
          ++cs_staged->aired;
        cs = cs_staged;
      }
      if (cs && cs->phase >= 0 && sf + 1 >= cs->eff_sf) {
        /* A phase move: at at_us into this (the last old) superframe, the
         * TSF steps by step_us; the discipline takes the new phase (its fit
         * is on the free-running counter, which the step does not move), and
         * the network superframe offset follows. The beacon runs on: Kestrel
         * fires it once more at the old phase, idle by the plan. */
        const int64_t old_phase = phase_live.load();
        bool moved = false;
        int64_t late_us = 0;
        const char *why = "";
        if (sf + 1 != cs->eff_sf) {
          why = "missed";
        } else if (cs->aired < 3) {
          why = "unannounced";
        } else if (test_rephase_fail) {
          why = "test"; /* announced, not made: the UEs must find their way back */
        } else {
          const int64_t want = grid->slot_start_tsf(sf, 0) + cs->at_us;
          late_us = sleep_until_tsf(dev, want) - want;
          /* Late: still fine while the landing stays clear of the next TBTT. */
          if (cs->at_us + late_us + std::max<int64_t>(0, cs->step_us) >= P - 1000 ||
              cs->at_us + late_us >= P - 1000) {
            why = "late";
          } else {
            const bool wrote = dev->WriteTsf(static_cast<uint64_t>(static_cast<int64_t>(dev->ReadTsf()) + cs->step_us));
            /* After any write attempt the RX offset is re-read before it is
             * trusted again (as the discipline's own writes). */
            uint64_t t2 = 0, f2 = 0;
            if (dev->ReadTsfAndFreeRun(t2, f2)) {
              publish_clock(t2, f2);
            } else {
              std::lock_guard<std::mutex> lk(clk_mu);
              clk.valid = false;
            }
            if (!wrote) {
              why = "write";
              disc.wrote(false); /* a failed write may have half-updated it: re-anchor */
            } else {
              moved = true;
              dc.phase_us = static_cast<double>(cs->phase);
              disc.set_phase(dc.phase_us);
              disc.stepped(cs->step_us);
              const int64_t no = net_off.load() + (old_phase - cs->phase - cs->step_us) / P;
              net_base.store(cs->phase + no * P);
              net_off.store(no);
              /* The phase and the neighbours' offsets (relative to it) change
               * together under f1_mu. */
              std::lock_guard<std::mutex> lk(f1_mu);
              phase_live.store(cs->phase);
              std::vector<devourer::cell::Neighbour> v;
              for (const auto &e : nb_raw)
                v.push_back({e.cell, e.channel, devourer::cell::neighbour_offset_us(e.phase_us, cs->phase, P)});
              if (v != nb_list) {
                nb_list = std::move(v);
                ++nl_gen;
              }
            }
          }
        }
        char l[260];
        snprintf(l, sizeof l,
                 "{\"ev\":\"cell.cs.rephase\",\"id\":%u,\"from_us\":%lld,\"to_us\":%lld,\"step_us\":%lld,"
                 "\"at_us\":%lld,\"sf\":%lld,\"ok\":%d,\"late_us\":%lld,\"aired\":%d,\"why\":\"%s\"}",
                 cs->id, (long long)old_phase, (long long)cs->phase, (long long)cs->step_us, (long long)cs->at_us,
                 (long long)sf, moved ? 1 : 0, (long long)late_us, cs->aired, why);
        emit(l);
        if (f1_on) {
          const auto ack = devourer::cell::f1::encode_ack({c.cell, devourer::cell::f1::AckKind::ChannelSwitch, cs->id,
                                           moved ? devourer::cell::f1::AckStatus::Applied : devourer::cell::f1::AckStatus::Invalid,
                                           static_cast<uint64_t>(std::max<int64_t>(0, sf + net_off.load()))},
                                          f1_mac);
          sendto(f1_fd.fd, ack.data(), ack.size(), 0, reinterpret_cast<const sockaddr *>(&f1_addr), f1_addr_len);
        }
        std::lock_guard<std::mutex> lk(f1_mu);
        cs_staged.reset();
        cs_aired_id = 0;
      } else if (cs && sf + 1 >= cs->eff_sf) {
        const auto at = devourer::cell::cs_retune_at_us(grid_span_us, cs_retune_us, 1000, P);
        const uint8_t from = du_ch.load();
        bool moved = false;
        int64_t late_us = 0, retune_us = 0;
        const char *why = "";
        if (!at || sf + 1 != cs->eff_sf) {
          why = "missed";
        } else if (cs->aired < 3) {
          why = "unannounced"; /* the UEs were not told: never moved without them */
        } else {
          late_us = sleep_until_tsf(dev, grid->slot_start_tsf(sf, 0) + *at) - (grid->slot_start_tsf(sf, 0) + *at);
          bool stopped = false;
          for (int k = 0; k < 3 && !stopped; ++k)
            stopped = dev->StopBeacon();
          if (!stopped) {
            why = "stop_beacon";
          } else {
            auto tune = [&](uint8_t ch) {
              try {
                dev->FastRetune(ch);
                return true;
              } catch (const std::exception &) {
                return false;
              }
            };
            auto restart = [&] {
              for (int k = 0; k < 3; ++k)
                if (dev->StartBeacon(live_bcn.data(), live_bcn.size(), c.tu))
                  return true;
              return false;
            };
            const auto r0 = std::chrono::steady_clock::now();
            const bool tuned = tune(cs->channel);
            retune_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - r0)
                            .count();
            if (tuned && restart()) {
              moved = true;
              du_ch.store(cs->channel);
            } else {
              why = tuned ? "restart_beacon" : "retune";
              /* back where its UEs can still find it */
              const bool back = !tuned || tune(from);
              if (!back || !restart())
                emit("{\"ev\":\"cell.cs.stranded\"}"); /* no beacon: the CU sees the DU go quiet */
            }
          }
        }
        char l[240];
        snprintf(l, sizeof l,
                 "{\"ev\":\"cell.cs.switch\",\"id\":%u,\"from\":%u,\"to\":%u,\"sf\":%lld,\"ok\":%d,"
                 "\"retune_us\":%lld,\"late_us\":%lld,\"aired\":%d,\"why\":\"%s\"}",
                 cs->id, from, cs->channel, (long long)sf, moved ? 1 : 0, (long long)retune_us, (long long)late_us,
                 cs->aired, why);
        emit(l);
        if (f1_on) { /* the outcome, from this thread (a UDP send is thread-safe) */
          const auto ack = devourer::cell::f1::encode_ack({c.cell, devourer::cell::f1::AckKind::ChannelSwitch, cs->id,
                                           moved ? devourer::cell::f1::AckStatus::Applied : devourer::cell::f1::AckStatus::Invalid,
                                           static_cast<uint64_t>(std::max<int64_t>(0, sf + net_off.load()))},
                                          f1_mac);
          sendto(f1_fd.fd, ack.data(), ack.size(), 0, reinterpret_cast<const sockaddr *>(&f1_addr), f1_addr_len);
        }
        std::lock_guard<std::mutex> lk(f1_mu);
        cs_staged.reset();
        cs_aired_id = 0;
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
  if (f1_on) {
    char l[120];
    snprintf(l, sizeof l, "{\"ev\":\"cell.f1.summary\",\"send_drops\":%llu}",
             (unsigned long long)f1_send_drops.load());
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
  /* The serving cell: c.cell until a handover (M6) moves the UE. */
  std::atomic<uint16_t> cell_a{c.cell};
  dev->InitWrite(SelectedChannel{c.ch, 0, CHANNEL_WIDTH_20});
  auto arm_ack = [&](int ue) {
    devourer::MacAddr own;
    cell_mac(own.bytes.data(), cell_a.load(), static_cast<uint8_t>(ue));
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

  /* The serving cell's key: a handover switches it (src/cell/Keys.h — per-
   * cell keys when CELL_AREA_KEY is set; else one shared key that never
   * changes). Read under a small lock: the RX thread verifies with it. */
  struct CellKey {
    std::mutex m;
    std::string hex;
    CellMac mac;
  };
  auto cur_key = std::make_shared<CellKey>();
  cur_key->hex = c.psk;
  cur_key->mac = hmac_sha1(c.psk);
  const CellMac auth =
      c.psk.empty() ? CellMac{}
                    : CellMac([cur_key](const uint8_t *d, size_t n, uint8_t out[20]) {
                        CellMac m;
                        {
                          std::lock_guard<std::mutex> lk(cur_key->m);
                          m = cur_key->mac;
                        }
                        return m && m(d, n, out);
                      });
  auto cell_key_hex = [cur_key] {
    std::lock_guard<std::mutex> lk(cur_key->m);
    return cur_key->hex;
  };
  auto set_cell_key = [cur_key](const std::string &hex) {
    std::lock_guard<std::mutex> lk(cur_key->m);
    cur_key->hex = hex;
    cur_key->mac = hmac_sha1(hex);
  };
  const CellMac area = area_auth(c);
  const char *area_env = std::getenv("CELL_AREA_KEY");
  const bool per_cell_keys = area_env && *area_env;
  /* Its own key (home cell, home UE): unwraps the target cell's key a
   * handover carries. */
  const char *uk = std::getenv("CELL_UE_KEY");
  const CellMac ue_key_mac = uk && *uk ? hmac_sha1(uk) : CellMac{};
  /* Signed neighbour lists: with the CA's public key (CELL_CA_PUB, 64 hex) an
   * NL counts — and with it the beacon as that cell's — only through the
   * chain CA -> cell key -> NL (MeasGap.h NlVerifier), never by the area tag. */
  std::optional<devourer::cell::NlVerifier> nlv;
  /* Replay checks on signed lists (mu): the serving cell's window and where
   * its beacons land; refusals by reason. CELL_UE_NBR_FRESH=0 turns the
   * checks off (a test's counterfactual). */
  const bool nbr_fresh = envl("CELL_UE_NBR_FRESH", 1, 0, 1) != 0;
  const int64_t nbr_phase_tol_us = envl("CELL_UE_NBR_PHASE_US", 300, 10, 50000);
  bool have_serv_win = false, have_serv_phase = false;
  /* A cell switch: the new serving cell's first window may be one behind
   * (both sit on the one network superframe, but not at the same instant). */
  bool serv_switched = false;
  uint64_t serv_win_ms = 0; /* when the serving window last advanced */
  int serv_off_run = 0;     /* serving copies off the reference phase in a row */
  uint32_t serv_win = 0;
  int64_t serv_phase = 0;
  std::map<std::string, uint64_t> nbr_refused;
  if (const char *ca = std::getenv("CELL_CA_PUB"); ca && *ca) {
    const auto pub = hex_bytes<32>(ca);
    if (!pub) {
      fprintf(stderr, "CELL_CA_PUB: 64 hex\n");
      return 2;
    }
#if defined(DEVOURER_CELL_OPENSSL)
    nlv.emplace(*pub, ed25519_verify);
#endif
  }
  /* Its own area key (Keys.h A_u = HMAC(K_u, 'V')): its beacon's area tag,
   * which only it (and, through the CU, the neighbour DUs) can produce. */
  CellMac ue_area = area;
  if (per_cell_keys && ue_key_mac)
    if (auto a = devourer::cell::ue_area_key(ue_key_mac))
      ue_area = hmac_sha1(*a);
  if (per_cell_keys && !ue_key_mac) {
    /* Its beacon's area tag is its own key (Keys.h A_u): without it no other
     * cell's DU would ever count it as heard (or forward it for DAPS), and it
     * could not hand over. */
    fprintf(stderr, "CELL_AREA_KEY needs CELL_UE_KEY: without it no other cell hears this UE "
                    "(its area tag) and it cannot hand over\n");
    return 2;
  }
  std::mutex mu;  // guards sync, wrap, rxr
  devourer::TsfSync sync;
  /* A step of the DU's TSF re-anchors the fit instead of reading as skew:
   * on the rig a ~1.3 ms jump once ran a UE's skew to thousands of ppm. */
  sync.SetStepDetect(envl("CELL_UE_FIT_STEP_US", 200, 0, 1000000),
                     static_cast<int>(envl("CELL_UE_FIT_CONFIRM", 6, 1, 50)));
  long long fit_steps_seen = 0, fit_rejected_seen = 0;
  /* Re-attachment after a DU restart (mu): after a confirmed step of the
   * DU's clock, for CELL_UE_REATTACH_SF superframes, a grant map at a lower
   * epoch but due near the beacon's own superframe starts the UE's grant
   * state over (GrantMapReceiver::offer_after_restart); the main loop then
   * resets what belonged to the old DU instance. */
  const int64_t reattach_sf = envl("CELL_UE_REATTACH_SF", 200, 0, 100000);
  long long reattach_armed_steps = 0; /* sync.Steps() at the arming */
  int64_t reattach_until_ms = 0;
  bool reattached = false;
  uint32_t reattach_old_epoch = 0, reattach_new_epoch = 0;
  Wrap32 wrap;
  GrantMapReceiver rxr(c.cell);
  /* DL HARQ receiver + the DL SNR since the last feedback publish (mu). */
  HarqReceiver hrx;
  double dl_snr_sum = 0;
  uint64_t dl_snr_n = 0;
  uint64_t dl_new = 0, dl_dup = 0;
  /* The CU's DL stream (F1 Downlink PDUs in the DL frames): one sn space for
   * the UE's whole life, across handovers — never reset on a switch. */
  DlStreamRx dl_stream;
  /* Serving beacons whose grant map is intact but not authentic (another
   * key, or none): refused (mu). */
  uint64_t gm_ok = 0, gm_refused = 0;
  const int64_t P = static_cast<int64_t>(c.tu) * devourer::cell::kTuUs;
  /* The capacity grid (same env as the DU): positions never move. */
  const std::optional<SlotGrid> grid = grid_for(c);
  if (!grid) {
    fprintf(stderr, "the grid does not fit a %d TU superframe\n", c.tu);
    return 2;
  }
  uint8_t du_mac[6]; /* the serving DU (mu) */
  cell_mac(du_mac, c.cell, 0);
  /* M6 handover (src/cell/HoCommand.h), all under mu: the newest IE the
   * serving DU's beacon addressed to us, the one accepted (fresh, instant
   * ahead), and the target's grant maps heard while waiting for it. */
  std::optional<devourer::cell::HoIe> ho_seen, ho_pending;
  /* A conditional handover's switch decision (HoCommand.h), and the last
   * source superframe a serving beacon was heard in (-1 = none yet). */
  std::optional<devourer::cell::ChoCondition> cho;
  /* The serving superframe of the last serving beacon heard, from the
   * beacon's own departure TSF (the RX path): a late loop pass does not
   * delay a loss. -1 = none since the last switch. */
  std::atomic<int64_t> serv_bcn_sf{-1};
  /* M5: the serving cell's channel move (ChannelSwitch.h), as seen in its
   * beacon, accepted (CsReceiver), and after it the way back if the cell is
   * not heard on the new channel. */
  std::optional<devourer::cell::CsIe> cs_seen, cs_pending;
  devourer::cell::CsReceiver cs_rx;
  struct CsBack {
    uint8_t to = 0;           /* the channel it came from */
    int64_t deadline_sf = 0;
    uint64_t bcn_at = 0;      /* serving beacons counted at the move */
  };
  std::optional<CsBack> cs_back;
  std::atomic<uint64_t> serv_bcn_n{0}; /* serving-cell beacons heard */
  const int64_t ue_cs_retune_us = envl("CELL_CS_RETUNE_US", 26000, 1000, 100000);
  std::string ho_target_key;  /* the accepted handover's target key (hex) */
  CellMac target_auth;        /* ... as a Mac: the target's buffered maps (mu) */
  uint32_t ho_no_ctx_id = 0;  /* the last command refused for no context (logged once) */
  devourer::cell::HoReceiver horx;
  std::optional<GrantMapReceiver> rxr_target;
  uint8_t tgt_du_mac[6] = {0};
  uint16_t visit_tgt_cell = 0, visit_tgt_ue = 0; /* (mu) the pending handover's target identity */
  uint64_t visit_dl_rx = 0;                      /* DAPS-visit DL answers taken (mu) */
  bool force_beacon = false;
  /* After a switch, until the target grants us: the way back (mu). */
  /* M6 measurement gaps (src/cell/MeasGap.h), under mu: the neighbour list
   * the serving DU airs, and the UE's own filtered beacon measurements
   * (serving and neighbours), reported in its uplink beacon (MR IE). */
  std::vector<devourer::cell::Neighbour> nbrs;
  int64_t nbrs_ms = 0;    /* when the serving DU's NL was last heard */
  uint32_t nbrs_gen = 0;  /* the newest NL generation accepted (this serving cell) */
  bool have_nbrs_gen = false;
  uint32_t mr_gen = 0;    /* the MR generation (strictly increasing) */
  /* Gap scheduling: the retune bound both retunes are planned with (the
   * configured floor, raised by any slower retune measured), and neighbours
   * parked after skipping (no slot clear for them) until a superframe. */
  /* 26 ms: the 8852CE's FastRetune measured 23.5-23.7 ms (11 retunes on the
   * rig) + ~10 %. The CU plans phases with the same numbers (RAN_GAP_*). */
  const int64_t gap_retune_floor_us = envl("CELL_UE_GAP_RETUNE_US", 26000, 1000, 500000);
  int64_t gap_retune_us = gap_retune_floor_us;
  std::map<uint16_t, int64_t> gap_parked;
  int64_t no_step_until_sf = -1; /* hold TSF steps until this superframe (after a gap) */
  /* CELL_UE_MEAS_LEVEL: a test bench's placement, emulated (MeasLevelPin). */
  MeasLevelPin meas_pin(std::getenv("CELL_UE_MEAS_LEVEL") ? std::getenv("CELL_UE_MEAS_LEVEL") : "");
  if (!meas_pin.bad().empty()) { /* a typo must not leave a cell on its real level unnoticed */
    fprintf(stderr, "CELL_UE_MEAS_LEVEL: cannot parse \"%s\" (want cell:dBm,...)\n", meas_pin.bad().front().c_str());
    return 2;
  }
  if (meas_pin.active()) { /* everything this UE measures of these cells is shifted: say so */
    const std::string d = meas_pin.describe();
    char l[200];
    snprintf(l, sizeof l, "{\"ev\":\"cell.ue.meas_pin_cfg\",\"levels\":\"%.150s\"}", d.c_str());
    emit(l);
  }
  devourer::cell::MeasTable ue_meas(devourer::cell::L3FilterConfig{
      static_cast<int>(envl("CELL_MEAS_L3_K", 4, 0, 19)), envl("CELL_MEAS_STALE_MS", 3000, 100, 60000)});
  const int64_t gap_every_sf = envl("CELL_UE_GAP_SF", 10, 0, 100000); /* 0 = no gaps */
  const int64_t gap_guard_us = envl("CELL_UE_GAP_GUARD_US", 2000, 0, 50000);
  /* Suspend gaps (MeasGap.h): one in this many superframes (0 = off); the DU
   * must run the same value (it skips this UE's DL then). */
  const int64_t gap_suspend_every = envl("CELL_GAP_SUSPEND_SF", 0, 0, 100000);
  /* DAPS across channels (sequential): between accepting an inter-frequency
   * handover and its switch, visit the target's channel once a superframe to
   * air a copy of the uplink there (the target DU forwards it to the CU). */
  const bool daps_visit = envl("CELL_UE_DAPS_VISIT", 1, 0, 1) != 0;
  const int64_t mr_every_ms = envl("CELL_UE_MR_MS", 1000, 100, 60000);
  auto ue_ms = [] {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  };
  /* Inter-frequency: the channel the UE runs on, a retune the switch (or
   * a fallback) asks the control thread for, and the first beacon of the new
   * serving DU after it (the deaf time). */
  uint8_t cur_ch = c.ch;
  std::optional<uint8_t> retune_to;
  std::atomic<int64_t> retune_done_us{0}; /* steady clock, 0 = none pending */
  struct HoBack {
    std::string key;     /* the source cell's key, to return to */
    uint8_t channel = 0; /* the source's channel, to return to */
    uint16_t cell = 0, ue = 0;
    int64_t delta_us = 0;
    int64_t deadline_sf = 0; /* in the target's count */
    uint32_t id = 0;
    uint64_t bcn_at = 0; /* serving beacons heard at the switch: commit only after a newer one */
  };
  std::optional<HoBack> ho_back;
  std::optional<GrantMapReceiver> rxr_source;
  const int64_t ho_fallback_sf = envl("CELL_HO_FALLBACK_SF", 10, 2, 10000);
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
  uint64_t nonce = (static_cast<uint64_t>(rng()) << 32) | rng();
  /* A UE that got its id through the RACH re-joins once the DU's authentic
   * map in force has not granted it for CELL_UE_REJOIN_SF superframes: the
   * DU restarted (a new instance does not know the id) or released it. A
   * static UE (CELL_UE > 0) never does: its DU grants it again. */
  const bool static_ue = me_a > 0;
  const int64_t rejoin_sf = envl("CELL_UE_REJOIN_SF", 30, 0, 100000); /* 0 = never */
  int64_t absent_from_sf = -1; /* first superframe of the current absence */
  int rejoin_stop_fails = 0;   /* consecutive failed beacon stops before a re-join */
  devourer::cell::RachClient rach(joiner_mac, nonce, [&rng] { return static_cast<uint32_t>(rng()); });
  std::vector<devourer::cell::RachResponse> rar_seen;
  const CellMac &mac = auth; /* the RACH: the serving cell's key */
  const auto rach_idx = grid->find(SlotKind::Rach, 0);

  std::thread rx([&] {
    dev->StartRxLoop([&](const Packet &p) {
      if (p.RxAtrib.crc_err || p.Data.size() < 24)
        return;
      if (p.Data[0] == 0x80) {
        std::lock_guard<std::mutex> lk(mu);
        /* Any cell DU's beacon (02:ce:11:<cell>:00): the UE's measurement of
         * that cell — its serving one, a same-channel neighbour, or one heard
         * in a measurement gap. */
        /* Only a beacon that proves it is that cell's (its tagged NL IE) is
         * measured: a frame merely carrying a cell-shaped address is not. */
        if (p.Data[10] == 0x02 && p.Data[11] == 0xce && p.Data[12] == 0x11 && p.Data[15] == 0) {
          const uint16_t bc = static_cast<uint16_t>(p.Data[13] << 8 | p.Data[14]);
          std::optional<devourer::cell::NlIe> nl;
          if (!nlv) {
            nl = devourer::cell::find_nl_ie(p.Data.data(), p.Data.size(), bc, area);
          } else if (auto sn = nlv->check(p.Data.data(), p.Data.size(), bc)) {
            /* Replay (MeasGap.h): a genuine signed beacon re-aired. Fresh:
             * the serving window never goes back; a neighbour's is within
             * one of the serving cell's (heard live), and its beacon lands
             * on the TBTT offset the serving list names (shared timebase). */
            nl = sn->nl;
            const char *why = nullptr;
            const bool serving = bc == cell_a.load();
            int64_t phase = -1, want = -1;
            if (sync.Ready() && wrap.ready()) {
              const int64_t at = sync.RemoteForLocal(static_cast<uint64_t>(wrap.extend(p.RxAtrib.tsfl)));
              phase = ((at % P) + P) % P;
            }
            /* The serving reference phase moves only slowly: a copy off it
             * by more than the tolerance is refused, never taken as the new
             * reference (a replayed serving beacon cannot drag it). */
            auto phase_off = [&](int64_t a, int64_t b) {
              int64_t d = std::llabs(a - b) % P;
              return std::min(d, P - d);
            };
            if (serving) {
              const uint32_t back = serv_switched ? 1u : 0u;
              if (nbr_fresh && have_serv_win && sn->win + back < serv_win) {
                why = "stale";
              } else if (nbr_fresh && have_serv_phase && phase >= 0 &&
                         phase_off(phase, serv_phase) > nbr_phase_tol_us && ++serv_off_run < 20) {
                why = "phase"; /* 20 in a row, none on the reference: the cell moved */
              } else {
                if (phase >= 0 && have_serv_phase && serv_off_run >= 20)
                  have_serv_phase = false; /* re-taken below */
                serv_off_run = 0;
                if (!have_serv_win || sn->win > serv_win || serv_switched)
                  serv_win_ms = ue_ms();
                serv_win = sn->win;
                have_serv_win = true;
                serv_switched = false;
                if (phase >= 0) { /* where serving beacons land: the reference */
                  if (!have_serv_phase) {
                    serv_phase = phase;
                  } else { /* 1/8 of the wrapped error per beacon */
                    int64_t e = (phase - serv_phase) % P;
                    if (e > P / 2) e -= P;
                    if (e < -P / 2) e += P;
                    serv_phase = ((serv_phase + e / 8) % P + P) % P;
                  }
                  have_serv_phase = true;
                }
              }
            } else if (nbr_fresh) {
              const int64_t dw = static_cast<int64_t>(sn->win) - static_cast<int64_t>(serv_win);
              const auto it = std::find_if(nbrs.begin(), nbrs.end(),
                                           [&](const devourer::cell::Neighbour &x) { return x.cell == bc; });
              /* The serving window itself must be current: one that has not
               * advanced for two windows' time is no "now" to compare with. */
              const bool serv_current =
                  have_serv_win && ue_ms() - serv_win_ms <=
                                       static_cast<uint64_t>(2 * devourer::cell::kNlWindowSf * P / 1000);
              if (!serv_current || dw < -1 || dw > 1) {
                why = "stale";
              } else if (it == nbrs.end()) {
                why = "unlisted";
              } else if (phase < 0 || !have_serv_phase) {
                why = "phase";
              } else {
                want = (serv_phase + it->offset_us) % P;
                if (phase_off(phase, want) > nbr_phase_tol_us)
                  why = "phase";
              }
            }
            if (why) {
              nl.reset();
              auto &n = nbr_refused[why];
              if (n++ < 20) {
                char l[240];
                snprintf(l, sizeof l,
                         "{\"ev\":\"cell.ue.nbr_refused\",\"cell\":%u,\"why\":\"%s\",\"win\":%u,\"serving_win\":%u,"
                         "\"phase\":%lld,\"want\":%lld,\"serv_phase\":%lld}",
                         bc, why, sn->win, serv_win, (long long)phase, (long long)want,
                         have_serv_phase ? (long long)serv_phase : -1LL);
                emit(l);
              }
            }
          }
          std::optional<double> lvl = static_cast<double>(p.RxAtrib.rssi[0]) - 110.0;
          if (nl && meas_pin.active()) {
            const bool was = meas_pin.offset(bc).has_value();
            lvl = meas_pin.adjust(bc, *lvl);
            if (!was && meas_pin.offset(bc)) {
              char l[120];
              snprintf(l, sizeof l, "{\"ev\":\"cell.ue.meas_pin\",\"ue\":%d,\"cell\":%u,\"offset_db\":%.1f}",
                       me_a.load(), bc, *meas_pin.offset(bc));
              emit(l);
            }
          }
          if (nl && lvl) {
            ue_meas.update(bc, 0, *lvl, static_cast<double>(p.RxAtrib.snr[0]) / 2.0 /* raw dB*2 */, ue_ms());
            /* The serving cell's list: only a generation not older than the
             * one accepted (a replayed older list is refused). */
            if (bc == cell_a.load() && (!have_nbrs_gen || nl->gen >= nbrs_gen)) {
              nbrs = nl->cells;
              nbrs_gen = nl->gen;
              have_nbrs_gen = true;
              nbrs_ms = ue_ms();
            }
          }
        }
        if (std::memcmp(p.Data.data() + 10, du_mac, 6) != 0) {
          /* The handover target's beacons: its grant maps, buffered for the
           * switch (they do not feed the clock: that is the source's until
           * the switch, then shifted, not re-acquired). */
          if (rxr_target && std::memcmp(p.Data.data() + 10, tgt_du_mac, 6) == 0)
            if (auto m = devourer::cell::find_grant_map(p.Data.data(), p.Data.size(), target_auth))
              rxr_target->offer(*m);
          return;
        }
        const auto eg = p.TxEgressTsf();
        if (!eg)
          return;
        if (const int64_t rd = retune_done_us.exchange(0); rd != 0) {
          const int64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count();
          char l[160];
          snprintf(l, sizeof l,
                   "{\"ev\":\"cell.ue.retune_first_beacon\",\"ue\":%d,\"after_us\":%lld}",
                   me_a.load(), (long long)(now_us - rd));
          emit(l);
        }
        sync.Add(*eg, p.RxAtrib.tsfl);
        if (me_a > 0)
          if (auto h = devourer::cell::find_ho_ie(p.Data.data(), p.Data.size(), cell_a.load(),
                                                  static_cast<uint16_t>(me_a.load()), auth))
            ho_seen = h;
        if (auto cs = devourer::cell::find_cs_ie(p.Data.data(), p.Data.size(), cell_a.load(), auth))
          cs_seen = cs;
        ++serv_bcn_n;
        serv_bcn_sf.store(static_cast<int64_t>(*eg) / P);
        wrap.feed(p.RxAtrib.tsfl);
        if (reattach_sf > 0 && sync.Steps() != reattach_armed_steps) {
          reattach_armed_steps = sync.Steps();
          reattach_until_ms = ue_ms() + reattach_sf * P / 1000;
        }
        auto m = devourer::cell::find_grant_map(p.Data.data(), p.Data.size(), auth);
        if (m) {
          ++gm_ok;
        } else if (auth && devourer::cell::find_grant_map(p.Data.data(), p.Data.size(), {})) {
          if (gm_refused++ == 0) {
            char l[120];
            snprintf(l, sizeof l, "{\"ev\":\"cell.ue.gm_refused\",\"ue\":%d,\"cell\":%u}",
                     me_a.load(), cell_a.load());
            emit(l);
          }
        }
        if (m) {
          const uint32_t old_epoch = rxr.newest_epoch();
          if (!rxr.offer(*m) && ue_ms() < reattach_until_ms &&
              rxr.offer_after_restart(*m, static_cast<uint64_t>(*eg) / static_cast<uint64_t>(P))) {
            reattached = true;
            reattach_until_ms = 0;
            reattach_old_epoch = old_epoch;
            reattach_new_epoch = m->epoch;
          }
        }
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
      /* A DAPS-visit answer from the handover target (broadcast RA, its DU
       * as TA, our target identity): its DL PDUs, before the switch. */
      if (p.Data.size() >= 10 && p.Data[4] == 0xff) {
        std::lock_guard<std::mutex> lk(mu);
        if (rxr_target && visit_tgt_cell != 0 && std::memcmp(p.Data.data() + 10, tgt_du_mac, 6) == 0)
          if (auto vd = parse_visit_dl_frame(p.Data.data(), p.Data.size(), visit_tgt_cell, target_auth);
              vd && vd->ue == visit_tgt_ue) {
            int fresh_n = 0;
            for (uint32_t sn : vd->pdus)
              fresh_n += dl_stream.accept(vd->epoch, sn) ? 1 : 0;
            for (const auto &gp : dl_stream.take_gaps()) {
              char l[160];
              snprintf(l, sizeof l,
                       "{\"ev\":\"cell.ue.dl_gap\",\"ue\":%d,\"cell\":%u,\"after_sn\":%u,\"missing\":%u}",
                       me, cell_a.load(), gp.after_sn, gp.missing);
              emit(l);
            }
            ++visit_dl_rx;
            char l[200];
            snprintf(l, sizeof l,
                     "{\"ev\":\"cell.ue.visit_dl\",\"ue\":%d,\"pdus\":%zu,\"pdu_new\":%d,\"t_ms\":%llu}", me,
                     vd->pdus.size(), fresh_n, (unsigned long long)ue_ms());
            emit(l);
          }
        return;
      }
      const auto d = parse_dl_frame(p.Data.data(), p.Data.size(), cell_a.load(), auth);
      if (!d || me == 0 || d->ue != me)
        return;
      std::optional<size_t> want;
      SlotGrid::At pos;
      bool fresh = false;
      int pdu_new = 0; /* CU DL PDUs this frame delivered for the first time */
      {
        std::lock_guard<std::mutex> lk(mu);
        /* HARQ first: the feedback must count every frame that arrived. */
        fresh = hrx.on_frame(d->seq, d->harq.low);
        if (fresh)
          ++dl_new;
        else
          ++dl_dup;
        for (uint32_t sn : d->pdus)
          pdu_new += dl_stream.accept(d->epoch, sn) ? 1 : 0;
        for (const auto &g : dl_stream.take_gaps()) {
          char l[160];
          snprintf(l, sizeof l,
                   "{\"ev\":\"cell.ue.dl_gap\",\"ue\":%d,\"cell\":%u,\"after_sn\":%u,\"missing\":%u}",
                   me, cell_a.load(), g.after_sn, g.missing);
          emit(l);
        }
        if (p.RxAtrib.snr[0] > 0) { /* raw = 0.5 dB units; 0 = no measurement */
          dl_snr_sum += p.RxAtrib.snr[0] / 2.0;
          ++dl_snr_n;
        }
        if (!sync.Ready() || !grid)
          return;
        const int64_t du_at =
            sync.RemoteForLocal(static_cast<uint64_t>(wrap.extend(p.RxAtrib.tsfl)));
        pos = grid->at(du_at);
        /* The DL slot of our grant in the map in force (the CU may have
         * moved us off the position matching our id). */
        if (const GrantMap *gm = rxr.current(static_cast<uint64_t>(std::max<int64_t>(pos.superframe, 0))))
          if (const Grant *g = gm->find(static_cast<uint16_t>(me)))
            want = g->dl_slot;
      }
      /* No grant in force: still logged, as out of slot (granted 0). */
      char l[360];
      snprintf(l, sizeof l,
               "{\"ev\":\"cell.dl.rx\",\"ue\":%u,\"sf\":%llu,\"seq\":%u,\"attempt\":%u,"
               "\"new\":%d,\"rx_sf\":%lld,\"phase_us\":%lld,\"in_slot\":%d,\"granted\":%d,"
               "\"rssi\":%d,\"snr_half_db\":%d,\"pdus\":%zu,\"pdu_new\":%d,\"t_ms\":%llu}",
               me, (unsigned long long)d->sf, d->seq, d->harq.attempt, fresh ? 1 : 0,
               (long long)pos.superframe, (long long)pos.into_us,
               want && pos.slot && *pos.slot == *want ? 1 : 0, want ? 1 : 0,
               p.RxAtrib.rssi[0] - 110, p.RxAtrib.snr[0], d->pdus.size(), pdu_new,
               (unsigned long long)ue_ms());
      emit(l);
    });
  });

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(c.secs);
  bool beaconing = false;
  uint32_t running_epoch = 0;
  devourer::cell::HarqFeedback last_fb{};
  uint8_t last_snr_half = 0;
  uint8_t last_snr_gen = 0;
  std::optional<UeDlStatus> last_dl_st;
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
  bool rearm_ack = false;
  int64_t next_mr_ms = 0;
  int64_t next_gap_sf = -1; /* the superframe of the next measurement gap */
  size_t gap_rr = 0;        /* round-robin over the other-channel neighbours */
  size_t susp_rr = 0;       /* ... over those only a suspend gap reaches */
  int64_t last_susp_sf = -1;
  bool susp_infeasible = false;
  bool had_susp_prev = false; /* last pass had neighbours only a suspend gap reaches */
  std::optional<std::chrono::steady_clock::time_point> next_pass_at; /* (a switch's) */
  std::optional<uint8_t> place_ul_slot; /* an inter-frequency switch: place the TSF after its retune */
  bool rephase_place = false; /* a phase move: the beacon stopped and the TSF placed (no retune) */
  /* A phase move this UE made: watched until `until_sf` for its clock fit
   * confirming the OPPOSITE step (the DU did not move after all: missed,
   * refused, or its write failed), which the fit absorbs by itself; the
   * uplink beacon is then re-placed on the grid that stands. */
  struct RephaseCheck {
    int64_t step = 0, until_sf = 0;
    long long steps_at = 0;
    uint32_t id = 0;
  };
  std::optional<RephaseCheck> rp_check;
  std::optional<std::chrono::steady_clock::time_point> beacon_hold_until; /* no beacon restart before */
  int place_tries = 0;                   /* ... retried before the beacon restart if it failed */
  int64_t restart_du = -1; /* the switch's expedited pass (DU time): the beacon restarts there */
  /* The switch's TSF placement, while the beacon is stopped: its restart
   * then fires in the target's uplink slot from the first one (a step after
   * the restart came too late for it: the first uplink landed at the
   * source's phase, 40 ms off). False (still pending) if the clock could not
   * be read or written: retried before the restart, a few times. */
  auto switch_place = [&]() -> bool {
    if (!place_ul_slot || *place_ul_slot >= grid->slots().size()) {
      place_ul_slot.reset();
      return true;
    }
    uint64_t t2 = 0, f2 = 0;
    int64_t du2 = -1;
    if (dev->ReadTsfAndFreeRun(t2, f2)) {
      std::lock_guard<std::mutex> lk(mu);
      if (sync.Ready() && wrap.ready())
        du2 = sync.RemoteForLocal(static_cast<uint64_t>(wrap.extend(static_cast<uint32_t>(f2))));
    }
    int64_t st = 0;
    bool ok = du2 >= 0;
    if (ok) {
      st = ue_tsf_step(static_cast<int64_t>(t2), du2, grid->slots()[*place_ul_slot].start_us, 0, P, 2);
      ok = st == 0 || dev->WriteTsf(static_cast<uint64_t>(static_cast<int64_t>(dev->ReadTsf()) + st +
                                                          static_cast<int64_t>(std::llround(comp_us))));
    }
    if (!ok) {
      char l[160];
      snprintf(l, sizeof l, "{\"ev\":\"cell.ue.switch_place_fail\",\"ue\":%d,\"try\":%d,\"what\":\"%s\"}",
               me_a.load(), place_tries + 1, du2 < 0 ? "read" : "write");
      emit(l);
      if (++place_tries >= 3)
        place_ul_slot.reset(); /* the ordinary step corrects it, a pass later */
      return false;
    }
    /* Kestrel's beacon timer takes a TSF write from the TBTT after next: the
     * first beacon after this step fires at the OLD phase, i.e. at
     * (slot + step) on the target's grid. Harmless in its idle time; inside
     * its active span it would collide, so the restart then waits that TBTT
     * out (one uplink, not a collision). The beacon restarts no earlier than
     * the expedited pass (restart_du), so the stale TBTT is the first one at
     * that phase from then on. */
    int64_t span = 0;
    for (const auto &x : grid->slots())
      span = std::max<int64_t>(span, x.start_us + x.len_us);
    const int64_t stale = ((grid->slots()[*place_ul_slot].start_us + st) % P + P) % P;
    const bool collide = st != 0 && (stale < span + 1000 || stale > P - 1000);
    if (collide) {
      const int64_t from = std::max(du2, restart_du);
      int64_t s_at = from / P * P + stale;
      if (s_at < from)
        s_at += P;
      beacon_hold_until = std::chrono::steady_clock::now() + std::chrono::microseconds(s_at - du2 + 2000);
    }
    char l[200];
    snprintf(l, sizeof l,
             "{\"ev\":\"cell.ue.switch_place\",\"ue\":%d,\"step_us\":%lld,\"ok\":1,\"stale_us\":%lld,"
             "\"hold\":%d}",
             me_a.load(), (long long)st, (long long)stale, collide ? 1 : 0);
    emit(l);
    place_ul_slot.reset();
    return true;
  };
  std::vector<uint8_t> last_bcn; /* the uplink beacon frame on air (for DAPS visits) */
  int64_t last_visit_sf = -1;
  uint64_t visits = 0, visits_ok = 0, steps_held = 0;
  uint64_t susp_gaps = 0, susp_heard = 0, susp_fail = 0;
  bool expedited = false;
  int expedite_tries = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    /* A DAPS-visit window: passes 30 ms apart, so the next superframe's
     * visit is in reach after one ends ~75 ms into its superframe. */
    const bool visiting = daps_visit && ho_pending && ho_pending->channel != 0 && ho_pending->channel != cur_ch;
    {
      /* Right after a switch: the next pass at the new cell's first superframe
       * (+1 ms), so the uplink beacon restarts — and the step places it —
       * before that superframe's uplink slot, not a whole pass later. */
      auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(visiting ? 30 : 100);
      if (next_pass_at && *next_pass_at < until)
        until = *next_pass_at;
      expedited = next_pass_at.has_value();
      next_pass_at.reset();
      std::this_thread::sleep_until(until);
    }
    /* The expedited pass is not given up on one failed clock read: retried
     * 2 ms later (a few times), not at the normal pass interval. */
    auto retry_expedited = [&] {
      if (expedited && ++expedite_tries <= 5)
        next_pass_at = std::chrono::steady_clock::now() + std::chrono::milliseconds(2);
    };
    uint64_t t = 0, f = 0;
    if (!dev->ReadTsfAndFreeRun(t, f)) {
      retry_expedited();
      continue;
    }
    int64_t du_now = 0;
    const GrantMap *m = nullptr;
    GrantMap mcopy;
    {
      std::lock_guard<std::mutex> lk(mu);
      if (!sync.Ready() || !wrap.ready()) {
        retry_expedited();
        continue;
      }
      expedite_tries = 0;
      du_now = sync.RemoteForLocal(static_cast<uint64_t>(wrap.extend(static_cast<uint32_t>(f))));
      if (sync.Steps() != fit_steps_seen || sync.Rejected() != fit_rejected_seen) {
        char l[200];
        snprintf(l, sizeof l,
                 "{\"ev\":\"cell.ue.fit_step\",\"ue\":%d,\"steps\":%lld,\"last_step_us\":%lld,"
                 "\"rejected\":%lld,\"skew_ppm\":%.2f}",
                 me_a.load(), sync.Steps(), (long long)sync.LastStepUs(), sync.Rejected(),
                 sync.SkewPpm());
        emit(l);
        fit_steps_seen = sync.Steps();
        fit_rejected_seen = sync.Rejected();
      }
      /* Re-attached to a restarted DU (the RX path started the grant state
       * over): what belonged to the old DU instance goes, as on a switch. */
      if (reattached) {
        reattached = false;
        hrx = HarqReceiver{}; /* its DL sequence space starts again */
        mr_gen = 0;           /* its MR window is its new superframe count */
        nbrs.clear();
        have_serv_phase = false; /* another serving grid */
        serv_switched = true;
        serv_off_run = 0;
        have_nbrs_gen = false;
        gap_parked.clear();
        next_gap_sf = -1;      /* deadlines on the old count: re-set on the new one */
        no_step_until_sf = -1;
        ho_seen.reset();      /* commands aired by the old instance */
        ho_pending.reset();
        cho.reset();
        cs_seen.reset();
        cs_pending.reset();
        /* A target that restarted mid-handover lost the staged UE: the way
         * back stays, its deadline re-based on the new count. */
        if (ho_back)
          ho_back->deadline_sf = du_now / P + ho_fallback_sf;
        force_beacon = true;
        listen_from = -1;
        char l[200];
        snprintf(l, sizeof l,
                 "{\"ev\":\"cell.ue.reattach\",\"ue\":%d,\"cell\":%u,\"old_epoch\":%u,"
                 "\"new_epoch\":%u,\"sf\":%lld}",
                 me_a.load(), cell_a.load(), reattach_old_epoch, reattach_new_epoch,
                 (long long)(du_now / P));
        emit(l);
      }
      int64_t sfn = du_now >= 0 ? du_now / P : 0;
      /* M6: a fresh handover command is accepted once (HoReceiver: newer
       * id, instant ahead); at its instant the UE switches: the clock is
       * SHIFTED to the target's (TsfSync::ShiftRemote, the CU's delta), the
       * identity becomes the target's, and the target's buffered grant
       * maps take over. No scan, no re-acquisition, no association. */
      /* M5: the serving cell moves to another channel (ChannelSwitch.h):
       * accepted once (newer id, instant ahead); in the idle part of the
       * superframe before its instant the UE stops its beacon and retunes
       * with the cell (the retune below), restarting the beacon at once — no
       * TSF change, so its uplink keeps its slot. Not heard there within 10
       * superframes: back to the channel it came from. */
      if (cs_seen) {
        /* Not while a handover is pending: then not taken (nor marked seen),
         * so the cell's next beacons offer it again. */
        if (!ho_pending && cs_rx.accept(*cs_seen, static_cast<uint64_t>(sfn))) {
          cs_pending = cs_seen;
          char l[160];
          snprintf(l, sizeof l,
                   "{\"ev\":\"cell.ue.cs_staged\",\"ue\":%d,\"id\":%u,\"channel\":%u,\"eff_sf\":%llu,"
                   "\"step_us\":%d,\"at_us\":%d}",
                   me_a.load(), cs_seen->id, cs_seen->channel, (unsigned long long)cs_seen->effective_sf,
                   cs_seen->step_us, cs_seen->at_us);
          emit(l);
        }
        cs_seen.reset();
      }
      /* A phase move (ChannelSwitch.h v2): at at_us into the last old
       * superframe the DU steps its TSF by step_us; so does this UE's view of
       * it, at once, and its own uplink beacon is re-placed for its slot on
       * the moved grid (stopped, placed, restarted next pass: as after a
       * switch). The superframe count runs on. */
      if (cs_pending && cs_pending->step_us != 0 && du_now >= 0) {
        const int64_t win = (static_cast<int64_t>(cs_pending->effective_sf) - 1) * P + cs_pending->at_us;
        if (du_now >= win) {
          const int64_t before = du_now;
          sync.ShiftRemote(cs_pending->step_us);
          du_now = sync.RemoteForLocal(static_cast<uint64_t>(wrap.extend(static_cast<uint32_t>(f))));
          sfn = du_now / P;
          have_serv_phase = false; /* the serving grid moved */
          no_step_until_sf = -1;
          /* Its beacon stops on every move (it fired on the old grid); it is
           * placed for its slot when it has one, and restarts next pass. */
          const GrantMap *cm = rxr.current(static_cast<uint64_t>(sfn));
          if (const Grant *gg = cm ? cm->find(static_cast<uint16_t>(me_a.load())) : nullptr) {
            place_ul_slot = gg->ul_slot;
            place_tries = 0;
          }
          rephase_place = true;
          rp_check = RephaseCheck{cs_pending->step_us, sfn + 30, sync.Steps(), cs_pending->id};
          char l[240];
          snprintf(l, sizeof l,
                   "{\"ev\":\"cell.ue.cs_rephase\",\"ue\":%d,\"id\":%u,\"step_us\":%d,\"late_us\":%lld,"
                   "\"du_before\":%lld,\"du_after\":%lld}",
                   me_a.load(), cs_pending->id, cs_pending->step_us, (long long)(before - win),
                   (long long)before, (long long)du_now);
          emit(l);
          cs_pending.reset();
        } else {
          const auto due = std::chrono::steady_clock::now() + std::chrono::microseconds(win - du_now);
          if (!next_pass_at || due < *next_pass_at)
            next_pass_at = due;
        }
      }
      if (rp_check) {
        const int64_t tol = std::max<int64_t>(200, std::llabs(rp_check->step) / 4);
        if (sync.Steps() > rp_check->steps_at && std::llabs(sync.LastStepUs() + rp_check->step) < tol) {
          du_now = sync.RemoteForLocal(static_cast<uint64_t>(wrap.extend(static_cast<uint32_t>(f))));
          sfn = du_now / P;
          have_serv_phase = false;
          const GrantMap *cm = rxr.current(static_cast<uint64_t>(sfn));
          if (const Grant *gg = cm ? cm->find(static_cast<uint16_t>(me_a.load())) : nullptr) {
            place_ul_slot = gg->ul_slot;
            place_tries = 0;
          }
          rephase_place = true;
          char l[200];
          snprintf(l, sizeof l,
                   "{\"ev\":\"cell.ue.cs_rephase_undone\",\"ue\":%d,\"id\":%u,\"fit_step_us\":%lld,\"sf\":%lld}",
                   me_a.load(), rp_check->id, (long long)sync.LastStepUs(), (long long)sfn);
          emit(l);
          rp_check.reset();
        } else if (sfn > rp_check->until_sf) {
          rp_check.reset();
        }
      }
      if (cs_pending && cs_pending->step_us == 0 && du_now >= 0 && !retune_to) {
        int64_t span = 0;
        for (const auto &x : grid->slots())
          span = std::max<int64_t>(span, x.start_us + x.len_us);
        const auto at = devourer::cell::cs_retune_at_us(span, ue_cs_retune_us, 1000, P);
        /* no window of its own (its budget): still right after every slot */
        const int64_t win = (static_cast<int64_t>(cs_pending->effective_sf) - 1) * P + at.value_or(span + 1000);
        if (du_now >= win) {
          if (cs_pending->channel != cur_ch) {
            retune_to = cs_pending->channel;
            cs_back = CsBack{cur_ch, static_cast<int64_t>(cs_pending->effective_sf) + 10, serv_bcn_n.load()};
            /* the beacon restarts on the new channel before its next uplink slot */
            next_pass_at = std::chrono::steady_clock::now() + std::chrono::microseconds(ue_cs_retune_us + 2000);
          }
          char l[200];
          snprintf(l, sizeof l,
                   "{\"ev\":\"cell.ue.cs_switch\",\"ue\":%d,\"id\":%u,\"from\":%u,\"to\":%u,\"late_us\":%lld}",
                   me_a.load(), cs_pending->id, cur_ch, cs_pending->channel, (long long)(du_now - win));
          emit(l);
          cs_pending.reset();
        } else {
          const auto due = std::chrono::steady_clock::now() + std::chrono::microseconds(win - du_now);
          if (!next_pass_at || due < *next_pass_at)
            next_pass_at = due;
        }
      }
      if (cs_back && !retune_to) {
        if (cur_ch == cs_back->to) {
          cs_back.reset(); /* its own retune failed: still on the old channel */
        } else if (serv_bcn_n.load() > cs_back->bcn_at && sfn >= cs_back->deadline_sf - 10) {
          cs_back.reset(); /* the cell is heard on the new channel */
        } else if (sfn > cs_back->deadline_sf) {
          retune_to = cs_back->to;
          char l[160];
          snprintf(l, sizeof l, "{\"ev\":\"cell.ue.cs_fallback\",\"ue\":%d,\"to\":%u,\"sf\":%lld}",
                   me_a.load(), cs_back->to, (long long)sfn);
          emit(l);
          cs_back.reset();
        }
      }
      if (ho_seen) {
        /* Per-cell keys: the target's key, unwrapped from the context the IE
         * carries for this UE alone. Without one (or a context not for this
         * UE) the command is not taken — the target could not be verified. */
        std::optional<std::string> tkey;
        if (per_cell_keys) {
          const bool none = std::all_of(ho_seen->ctx.begin(), ho_seen->ctx.end(),
                                        [](uint8_t b) { return b == 0; });
          if (auto k = none || !ue_key_mac ? std::nullopt
                                           : devourer::cell::unwrap_context(ue_key_mac, ho_seen->id,
                                                                            ho_seen->target, ho_seen->ctx))
            tkey = devourer::cell::key_to_hex(*k);
          if (!tkey && ho_seen->id != ho_no_ctx_id) {
            ho_no_ctx_id = ho_seen->id;
            char l[120];
            snprintf(l, sizeof l, "{\"ev\":\"cell.ue.ho_no_context\",\"id\":%u}", ho_seen->id);
            emit(l);
          }
        }
        if ((!per_cell_keys || tkey) && horx.accept(*ho_seen, static_cast<uint64_t>(sfn))) {
          ho_pending = ho_seen;
          ho_target_key = tkey ? *tkey : cell_key_hex();
          target_auth = hmac_sha1(ho_target_key);
          rxr_target.emplace(ho_pending->target);
          cell_mac(tgt_du_mac, ho_pending->target, 0);
          visit_tgt_cell = ho_pending->target;
          visit_tgt_ue = ho_pending->target_ue;
          if (ho_pending->conditional())
            cho.emplace(*ho_pending);
          else
            cho.reset();
          char l[240];
          snprintf(l, sizeof l,
                   "{\"ev\":\"cell.ue.ho_staged\",\"id\":%u,\"ue\":%u,\"from\":%u,\"to\":%u,"
                   "\"target_ue\":%u,\"effective_sf\":%llu,\"sf\":%lld,\"cond\":%d}",
                   ho_pending->id, ho_pending->ue, ho_pending->cell, ho_pending->target,
                   ho_pending->target_ue, (unsigned long long)ho_pending->effective_sf,
                   (long long)sfn, ho_pending->conditional() ? 1 : 0);
          emit(l);
        }
        ho_seen.reset();
      }
      /* Conditional: the UE's own decision (its source silent or no longer
       * granting it, or the target ahead on its own measurements); at the
       * expiry it has lapsed and the UE stays. */
      bool cho_go = false;
      if (ho_pending && ho_pending->conditional() && cho) {
        /* The target's level only while freshly measured: a target that went
         * quiet must not finish the hold on a cached value (its gap revisit
         * period, twice, and never under a second). */
        const int64_t fresh_ms = std::max<int64_t>(1000, 2 * gap_every_sf * P / 1000);
        std::optional<double> sv, tg;
        for (const auto &e : ue_meas.snapshot(ue_ms())) {
          if (e.cell == ho_pending->cell)
            sv = e.rssi_dbm;
          if (e.cell == ho_pending->target && e.age_ms <= fresh_ms)
            tg = e.rssi_dbm;
        }
        const GrantMap *cm = rxr.current(static_cast<uint64_t>(sfn));
        const bool grants = !cm || cm->find(static_cast<uint16_t>(me_a.load()));
        const auto why = cho->update(sfn, serv_bcn_sf.load(), grants, sv, tg, ue_ms());
        char l[260];
        if (why == devourer::cell::ChoCondition::Why::Expired) {
          snprintf(l, sizeof l, "{\"ev\":\"cell.ue.cho_expired\",\"id\":%u,\"sf\":%lld}",
                   ho_pending->id, (long long)sfn);
          emit(l);
          ho_pending.reset();
          rxr_target.reset();
          visit_tgt_cell = 0;
          cho.reset();
        } else if (why != devourer::cell::ChoCondition::Why::None) {
          cho_go = true;
          snprintf(l, sizeof l,
                   "{\"ev\":\"cell.ue.cho_exec\",\"id\":%u,\"why\":\"%s\",\"sf\":%lld,"
                   "\"expiry_sf\":%llu,\"serving_heard_sf\":%lld,\"serving_dbm\":%.1f,\"target_dbm\":%.1f}",
                   ho_pending->id, devourer::cell::cho_why_name(why), (long long)sfn,
                   (unsigned long long)ho_pending->effective_sf, (long long)serv_bcn_sf.load(),
                   sv ? *sv : 0.0, tg ? *tg : 0.0);
          emit(l);
        }
      }
      if (ho_pending && (cho_go || (!ho_pending->conditional() &&
                                    static_cast<uint64_t>(sfn) >= ho_pending->effective_sf))) {
        const devourer::cell::HoIe h = *ho_pending;
        cho.reset();
        const int64_t before = du_now;
        sync.ShiftRemote(h.tsf_delta_us);
        du_now = sync.RemoteForLocal(static_cast<uint64_t>(wrap.extend(static_cast<uint32_t>(f))));
        const std::string source_key = cell_key_hex();
        set_cell_key(ho_target_key); /* the target's key from now (the same one if shared) */
        cell_a.store(h.target);
        me_a.store(h.target_ue);
        cell_mac(du_mac, h.target, 0);
        rxr_source = rxr;
        rxr = *rxr_target;
        rxr_target.reset();
        visit_tgt_cell = 0;
        nbrs.clear(); /* the new serving cell airs its own list */
        have_serv_phase = false; /* another serving grid */
        serv_switched = true;
        /* A hold set before the switch (after a gap or a DAPS visit) is on
         * the source's superframe count: never carried into the target's —
         * the switch must place the uplink at once (a held step left it
         * 40 ms off for 9 superframes, and the target's TA loop took that
         * for delay). */
        no_step_until_sf = -1;
        serv_off_run = 0;
        have_nbrs_gen = false;
        gap_parked.clear();
        ho_pending.reset();
        hrx = HarqReceiver{}; /* the target's DL sequence space */
        force_beacon = true;
        listen_from = -1;     /* DL accounting restarts on the target's count */
        sfn = du_now >= 0 ? du_now / P : 0;
        serv_bcn_sf.store(-1); /* another serving count */
        char l[300];
        snprintf(l, sizeof l,
                 "{\"ev\":\"cell.ue.ho\",\"id\":%u,\"from\":%u,\"ue\":%u,\"to\":%u,"
                 "\"target_ue\":%u,\"source_sf\":%lld,\"target_sf\":%lld,\"delta_us\":%lld,"
                 "\"source_tsf\":%lld,\"target_tsf\":%lld}",
                 h.id, h.cell, h.ue, h.target, h.target_ue, (long long)(before / P), (long long)sfn,
                 (long long)h.tsf_delta_us, (long long)before, (long long)du_now);
        emit(l);
        rearm_ack = true;
        ho_back = HoBack{source_key, cur_ch, h.cell, h.ue, h.tsf_delta_us, sfn + ho_fallback_sf, h.id,
                         serv_bcn_n.load()};
        if (du_now >= 0) {
          restart_du = (sfn + 1) * P + 1000;
          next_pass_at = std::chrono::steady_clock::now() + std::chrono::microseconds(restart_du - du_now);
        }
        if (h.channel != 0 && h.channel != cur_ch) {
          retune_to = h.channel; /* inter-frequency: done right after this lock */
          place_ul_slot = h.ul_slot; /* and the TSF placed for the target's slot */
          place_tries = 0;
        }
      }
      /* Committed once the target's map in force grants us; no grant by the
       * deadline = back to the source (conditional: the UE never ends up
       * with neither cell). */
      if (ho_back) {
        const GrantMap *tm = rxr.current(static_cast<uint64_t>(sfn));
        /* ... and a beacon of the new serving cell heard since the switch: a
         * map cached from before it proves nothing about the target now. */
        if (tm && tm->find(static_cast<uint16_t>(me_a.load())) && serv_bcn_n.load() > ho_back->bcn_at) {
          ho_back.reset();
          rxr_source.reset();
        } else if (sfn >= ho_back->deadline_sf) {
          const HoBack b = *ho_back;
          if (b.channel != cur_ch)
            retune_to = b.channel; /* back to the source's channel */
          sync.ShiftRemote(-b.delta_us);
          du_now = sync.RemoteForLocal(static_cast<uint64_t>(wrap.extend(static_cast<uint32_t>(f))));
          set_cell_key(b.key); /* back to the source's key */
          cell_a.store(b.cell);
          me_a.store(b.ue);
          cell_mac(du_mac, b.cell, 0);
          if (rxr_source)
            rxr = *rxr_source;
          rxr_source.reset();
          nbrs.clear();
          have_serv_phase = false; /* another serving grid */
          serv_switched = true;
          serv_off_run = 0;
          have_nbrs_gen = false;
          gap_parked.clear();
          ho_back.reset();
          hrx = HarqReceiver{};
          force_beacon = true;
          listen_from = -1;
          rearm_ack = true;
          sfn = du_now >= 0 ? du_now / P : 0;
          serv_bcn_sf.store(-1); /* another serving count */
          char l[200];
          snprintf(l, sizeof l,
                   "{\"ev\":\"cell.ue.ho_fallback\",\"id\":%u,\"to\":%u,\"ue\":%u,\"sf\":%lld}",
                   b.id, b.cell, b.ue, (long long)sfn);
          emit(l);
        }
      }
      m = rxr.current(static_cast<uint64_t>(sfn));
      if (m)
        mcopy = *m;
    }
    if (retune_to) {
      /* Inter-frequency switch (or fallback): the uplink beacon stops, the
       * radio retunes (FastRetune, Kestrel's lean intra-band path), and the
       * beacon restarts on the new channel next pass. The clock needs
       * nothing: the TSF delta already put it on the new cell's. */
      const uint8_t from = cur_ch, to = *retune_to;
      /* The beacon must be stopped first (never carried across the channel
       * change): a failed stop is retried; the retune waits for it (the
       * fallback deadline bounds how long). */
      bool stopped = !beaconing;
      for (int k = 0; k < 3 && !stopped; ++k)
        stopped = dev->StopBeacon();
      if (!stopped) {
        char l[120];
        snprintf(l, sizeof l, "{\"ev\":\"cell.ue.stop_beacon_fail\",\"ue\":%d,\"to_ch\":%u}",
                 me_a.load(), to);
        emit(l);
      } else {
        beaconing = false;
        retune_to.reset();
        const auto r0 = std::chrono::steady_clock::now();
        bool tuned = true;
        try {
          dev->FastRetune(to);
        } catch (const std::exception &ex) {
          tuned = false; /* stays on its channel; no target grant -> the fallback */
          char l[200];
          snprintf(l, sizeof l, "{\"ev\":\"cell.ue.retune_fail\",\"ue\":%d,\"to_ch\":%u,\"what\":\"%.80s\"}",
                   me_a.load(), to, ex.what());
          emit(l);
        }
        const auto r1 = std::chrono::steady_clock::now();
        if (!tuned)
          place_ul_slot.reset(); /* stays on the source: nothing to place */
        else
          switch_place();
        if (tuned) {
          cur_ch = to;
          retune_done_us.store(std::chrono::duration_cast<std::chrono::microseconds>(
                                   r1.time_since_epoch())
                                   .count());
          char l[200];
          snprintf(l, sizeof l,
                   "{\"ev\":\"cell.ue.retune\",\"ue\":%d,\"from_ch\":%u,\"to_ch\":%u,"
                   "\"retune_us\":%lld}",
                   me_a.load(), from, to,
                   (long long)std::chrono::duration_cast<std::chrono::microseconds>(r1 - r0).count());
          emit(l);
        }
      }
    }
    if (rephase_place) {
      /* A phase move: the uplink beacon stops, the TSF is placed for its slot
       * on the moved grid, and the beacon restarts next pass (as after a
       * switch, minus the retune). A failed stop is retried next pass. */
      bool stopped = !beaconing;
      for (int k = 0; k < 3 && !stopped; ++k)
        stopped = dev->StopBeacon();
      if (stopped) {
        beaconing = false;
        rephase_place = false;
        switch_place();
      }
    }
    if (rearm_ack) {
      /* The ACK responder for the identity now in use; retried every pass
       * until the radio takes it (DL is not ACKed for it before). */
      rearm_ack = !arm_ack(me_a.load());
      if (rearm_ack) {
        char l[120];
        snprintf(l, sizeof l, "{\"ev\":\"cell.ue.ack_rearm_fail\",\"ue\":%d}", me_a.load());
        emit(l);
      }
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
    if (!g) {
      /* Not granted by the map in force. A joined UE (not mid-handover)
       * absent this long re-joins: a fresh nonce (the DU answers a
       * repeated one from its cache: the old id), its uplink beacon off,
       * the id's DL sequence space gone. */
      const int64_t sf_now = du_now / P;
      if (absent_from_sf < 0)
        absent_from_sf = sf_now;
      if (!static_ue && rejoin_sf > 0 && !ho_back && !ho_pending &&
          sf_now - absent_from_sf >= rejoin_sf) {
        const int old_ue = me;
        /* The old id's beacon must be off first: a failed stop defers the
         * re-join to the next pass (the absence stands), never a RACH
         * beside a live old beacon. */
        if (beaconing && !dev->StopBeacon()) {
          if (rejoin_stop_fails++ == 0) {
            char l[120];
            snprintf(l, sizeof l, "{\"ev\":\"cell.ue.rejoin_stop_fail\",\"ue\":%d}", old_ue);
            emit(l);
          }
          continue;
        }
        beaconing = false;
        rejoin_stop_fails = 0;
        dev->ClearAckResponder();
        nonce = (static_cast<uint64_t>(rng()) << 32) | rng();
        rach = devourer::cell::RachClient(joiner_mac, nonce, [&rng] { return static_cast<uint32_t>(rng()); });
        {
          std::lock_guard<std::mutex> lk(mu);
          rar_seen.clear();
          hrx = HarqReceiver{};
        }
        me_a = 0;
        char l[200];
        snprintf(l, sizeof l,
                 "{\"ev\":\"cell.ue.rejoin\",\"old_ue\":%d,\"cell\":%u,\"absent_sf\":%lld,\"sf\":%lld}",
                 old_ue, cell_a.load(), (long long)(sf_now - absent_from_sf), (long long)sf_now);
        emit(l);
        absent_from_sf = -1;
      }
      continue;
    }
    absent_from_sf = -1;
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
    /* ... and the DL stream status: a handover target hears it (directly, or
     * forwarded by DAPS before the switch) and sends only what is missing. */
    std::optional<UeDlStatus> dl_st;
    uint8_t snr_half = last_snr_half;
    uint8_t snr_gen = last_snr_gen;
    {
      std::lock_guard<std::mutex> lk(mu);
      fb = hrx.feedback(static_cast<uint64_t>(du_now / P)); /* the UE's view of the DU sf */
      if (const auto st = dl_stream.status())
        dl_st = UeDlStatus{static_cast<uint32_t>(st->epoch), st->top, st->missing, st->low, st->floor};
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
    /* The measurement report rides the uplink beacon, refreshed this often. */
    if (ue_ms() >= next_mr_ms) {
      next_mr_ms = ue_ms() + mr_every_ms;
      force_beacon = true;
    }
    const bool content_moved = force_beacon || !beaconing || running_epoch != mcopy.epoch ||
                               fb.base != last_fb.base || fb.bitmap != last_fb.bitmap ||
                               snr_gen != last_snr_gen || dl_st != last_dl_st;
    if (content_moved && !beaconing && place_ul_slot && !switch_place()) {
      /* the switch's placement failed: no restart at the old phase; again soon */
      next_pass_at = std::chrono::steady_clock::now() + std::chrono::milliseconds(2);
      continue;
    }
    if (content_moved && !beaconing && beacon_hold_until) {
      if (std::chrono::steady_clock::now() < *beacon_hold_until)
        continue; /* a switch's stale TBTT is still ahead (see switch_place) */
      beacon_hold_until.reset();
    }
    if (content_moved) {
      auto b = build_ue_beacon(cell_a.load(), me, c.tu, mcopy.epoch, fb, snr_half, snr_gen, auth, ue_area, dl_st);
      {
        std::vector<devourer::cell::MeasEntryUe> mr;
        {
          std::lock_guard<std::mutex> lk(mu);
          for (const auto &e : ue_meas.snapshot(ue_ms()))
            if (mr.size() < devourer::cell::kMaxNeighbours + 1)
              mr.push_back({e.cell, static_cast<int8_t>(std::lround(std::clamp(e.rssi_dbm, -128.0, 127.0))),
                            static_cast<int8_t>(std::lround(std::clamp(e.snr_db, -128.0, 127.0))),
                            static_cast<uint16_t>(std::clamp<int64_t>(e.age_ms, 0, 65535))});
        }
        if (!mr.empty()) {
          /* Its generation: the serving superframe, strictly increasing (the
           * DU refuses anything not newer, and far from its own count). */
          mr_gen = std::max(mr_gen + 1, static_cast<uint32_t>(du_now / P));
          {
            std::string cells;
            for (const auto &e : mr) {
              char b2[48];
              snprintf(b2, sizeof b2, "%s\"%u\":%d", cells.empty() ? "" : ",", e.cell, e.rssi_dbm);
              cells += b2;
            }
            char l[300];
            snprintf(l, sizeof l, "{\"ev\":\"cell.ue.mr\",\"ue\":%d,\"cell\":%u,\"sf\":%lld,\"rssi\":{%s}}",
                     me, cell_a.load(), (long long)(du_now / P), cells.c_str());
            emit(l);
          }
          const auto ie =
              devourer::cell::encode_mr_ie(cell_a.load(), static_cast<uint16_t>(me), mr_gen, mr, auth);
          b.insert(b.end(), ie.begin(), ie.end());
        }
      }
      const bool ok = beaconing ? dev->UpdateBeaconPayload(b.data(), b.size())
                                : dev->StartBeacon(b.data(), b.size(), c.tu);
      if (ok)
        last_bcn = b;
      if (!ok) {
        fprintf(stderr, "UE beacon %s failed\n", beaconing ? "update" : "start");
        continue;
      }
      beaconing = true;
      force_beacon = false;
      const bool epoch_moved = running_epoch != mcopy.epoch;
      running_epoch = mcopy.epoch;
      last_fb = fb;
      last_snr_half = snr_half;
      last_snr_gen = snr_gen;
      last_dl_st = dl_st;
      if (epoch_moved) { /* a feedback-only refresh logs nothing */
        char l[160];
        snprintf(l, sizeof l, "{\"ev\":\"cell.ue.epoch\",\"ue\":%u,\"epoch\":%u,\"ta_us\":%d}",
                 me, mcopy.epoch, g->ta_us);
        emit(l);
      }
    }
    const int64_t step = ue_tsf_step(static_cast<int64_t>(t), du_now, ul_off, g->ta_us, P, 2);
    /* No TSF step right after a measurement gap: on the rig, both uplink
     * losses seen with gaps (two beacons each) followed a gap with a step
     * in the next pass; 20 gaps without one lost nothing. The mechanism is
     * not established; the step simply waits a few superframes. */
    if (step != 0 && du_now / P < no_step_until_sf) {
      ++steps_held;
    } else if (step != 0) {
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
    /* A measurement gap (MeasGap.h): for a neighbour on ANOTHER channel,
     * each revisited every gap_every_sf superframes, if a window around its
     * beacon fits clear of the serving beacon and this UE's own DL/UL slots
     * — planned with a CONSERVATIVE retune bound for both retunes. The UE
     * retunes there, hears the beacon (the RX path measures it), retunes
     * back. Never during a handover; never across bands (the full channel
     * path); skipped if it wakes late; listening cut short if the outbound
     * retune overran into the return budget. */
    /* One gap (idle or suspend): deaf from `start` (DU clock) for `len`.
     * A suspend gap stops the uplink beacon first (it must never fire on
     * the neighbour's channel) and leaves it stopped: the loop restarts it. */
    auto serving_now = [&]() -> std::optional<int64_t> {
      uint64_t t3 = 0, f3 = 0;
      if (!dev->ReadTsfAndFreeRun(t3, f3))
        return std::nullopt;
      std::lock_guard<std::mutex> lk(mu);
      return sync.RemoteForLocal(static_cast<uint64_t>(wrap.extend(static_cast<uint32_t>(f3))));
    };
    auto run_gap = [&](const devourer::cell::Neighbour &n, int64_t start, int64_t len, bool suspend) {
      char l[360];
      const auto now_du = serving_now();
      const int64_t wait = now_du ? start - *now_du : -1;
      if (wait <= 0 || wait >= 2 * P)
        return false;
      std::this_thread::sleep_for(std::chrono::microseconds(wait));
      /* Woke late? The window was checked for its planned start only. */
      const auto woke = serving_now();
      const int64_t late = woke ? *woke - start : gap_guard_us;
      if (late > gap_guard_us / 2) {
        snprintf(l, sizeof l,
                 "{\"ev\":\"cell.ue.gap_late\",\"ue\":%d,\"cell\":%u,\"late_us\":%lld,\"suspend\":%d}", me,
                 n.cell, (long long)late, suspend ? 1 : 0);
        emit(l);
        return false;
      }
      if (suspend && beaconing) {
        if (!dev->StopBeacon()) {
          snprintf(l, sizeof l, "{\"ev\":\"cell.ue.gap_stop_fail\",\"ue\":%d,\"cell\":%u}", me, n.cell);
          emit(l);
          return false; /* never retune with the beacon armed */
        }
        beaconing = false;
      }
      const auto g0 = std::chrono::steady_clock::now();
      /* The window's end on this clock: the radio must be back by it. */
      const auto win_end = g0 + std::chrono::microseconds(len - std::max<int64_t>(0, late));
      bool out_ok = true;
      try {
        dev->FastRetune(n.channel);
      } catch (const std::exception &) {
        out_ok = false;
      }
      const auto g1 = std::chrono::steady_clock::now();
      /* Listen until the beacon has passed, but never into the return
       * budget (an outbound overrun shortens the listening). */
      const auto listen_until = win_end - std::chrono::microseconds(gap_retune_us);
      if (out_ok && g1 < listen_until)
        std::this_thread::sleep_until(listen_until);
      const auto g1b = std::chrono::steady_clock::now();
      /* Back: retried; a radio that will not return is handed to the
       * normal retune path (beacon stopped, never on the wrong channel). */
      bool back_ok = false;
      for (int k = 0; k < 3 && !back_ok; ++k) {
        try {
          dev->FastRetune(cur_ch);
          back_ok = true;
        } catch (const std::exception &) {
        }
      }
      const auto g2 = std::chrono::steady_clock::now();
      if (!back_ok) {
        if (beaconing && dev->StopBeacon())
          beaconing = false;
        retune_to = cur_ch;
        snprintf(l, sizeof l, "{\"ev\":\"cell.ue.gap_return_fail\",\"ue\":%d,\"ch\":%u}", me, cur_ch);
        emit(l);
      }
      const int64_t out_us = std::chrono::duration_cast<std::chrono::microseconds>(g1 - g0).count();
      const int64_t back_us = std::chrono::duration_cast<std::chrono::microseconds>(g2 - g1b).count();
      no_step_until_sf = du_now / P + 4; /* see the TSF step below */
      /* The bound only grows with what was measured (never below the floor). */
      gap_retune_us = std::max({gap_retune_floor_us, gap_retune_us, out_us, back_us});
      std::optional<devourer::cell::MeasEntry> got;
      {
        std::lock_guard<std::mutex> lk(mu);
        for (const auto &e : ue_meas.snapshot(ue_ms()))
          if (e.cell == n.cell && e.age_ms <= (g2 - g0) / std::chrono::milliseconds(1) + 1)
            got = e;
      }
      snprintf(l, sizeof l,
               "{\"ev\":\"cell.ue.gap\",\"ue\":%d,\"cell\":%u,\"channel\":%u,"
               "\"heard\":%d,\"rssi\":%.1f,\"deaf_us\":%lld,\"out_us\":%lld,"
               "\"back_us\":%lld,\"over_us\":%lld,\"ok\":%d,\"suspend\":%d,\"sf\":%lld,\"start_us\":%lld}",
               me, n.cell, n.channel, got ? 1 : 0, got ? got->rssi_dbm : 0.0,
               (long long)std::chrono::duration_cast<std::chrono::microseconds>(g2 - g0).count(),
               (long long)out_us, (long long)back_us,
               (long long)std::chrono::duration_cast<std::chrono::microseconds>(g2 - win_end).count(),
               out_ok && back_ok ? 1 : 0, suspend ? 1 : 0, (long long)(start / P), (long long)(start % P));
      emit(l);
      if (suspend) {
        ++susp_gaps;
        susp_heard += got ? 1 : 0;
        susp_fail += out_ok && back_ok ? 0 : 1;
      }
      return true;
    };
    /* A sequential-DAPS visit (inter-frequency handover staged, before its
     * switch): in superframe S, after this UE's uplink slot, retune to the
     * target's channel, air a copy of the uplink beacon at an instant clear of
     * the target cell's active span, and retune back before the serving
     * beacon. Its own beacon fires at its uplink slot, outside the window. */
    if (daps_visit && ho_pending && ho_pending->channel != 0 && ho_pending->channel != cur_ch &&
        beaconing && !last_bcn.empty() && grid && g) {
      const int64_t sfn = du_now / P;
      const auto &sl = grid->slots();
      const int64_t ul_end = sl[g->ul_slot].start_us + sl[g->ul_slot].len_us;
      int64_t span = 0; /* the grid's active span (the target runs the same grid) */
      for (const auto &x : sl)
        span = std::max<int64_t>(span, x.start_us + x.len_us);
      const int64_t o_t = ((-ho_pending->tsf_delta_us) % P + P) % P; /* the target's TBTT here */
      /* send_packet only queues: the copy is given kVisitHoldUs on the target's
       * channel to leave (no per-frame TX report on Kestrel) before the
       * retune back — else it is flushed, or aired late on the wrong one. */
      constexpr int64_t kVisitHoldUs = 4000;
      const int64_t lo = ul_end + gap_guard_us + gap_retune_us,
                    hi = P - gap_guard_us - gap_retune_us - kVisitHoldUs - 1000;
      /* The whole send — queued at v, airing any time during the hold — must
       * stay clear of the target cell's active span (1 ms guards). */
      auto clear_of_target = [&](int64_t a, int64_t b) {
        for (int64_t x = a; x <= b; x += 250) {
          const int64_t d = ((x - o_t) % P + P) % P;
          if (d < span + 1000 || d > P - 1000)
            return false;
        }
        return true;
      };
      std::optional<int64_t> v;
      for (int64_t x = lo; x <= hi && !v; x += 500)
        if (clear_of_target(x - 500, x + kVisitHoldUs + 1000))
          v = x;
      const auto now_du = serving_now();
      for (const int64_t S : {sfn, sfn + 1}) {
        if (!v || !now_du || S <= last_visit_sf ||
            static_cast<uint64_t>(S) >= ho_pending->effective_sf) /* through the one before the switch */
          continue;
        const int64_t go = S * P + *v - gap_retune_us - gap_guard_us; /* leave the serving channel */
        if (go - *now_du <= 1000 || go - *now_du >= 2 * P)
          continue;
        last_visit_sf = S;
        std::this_thread::sleep_for(std::chrono::microseconds(go - *now_du));
        auto skip = [&](const char *why, int64_t late_us) {
          char l[200];
          snprintf(l, sizeof l,
                   "{\"ev\":\"cell.ue.daps_visit_skip\",\"ue\":%d,\"sf\":%lld,\"why\":\"%s\",\"late_us\":%lld}",
                   me, (long long)S, why, (long long)late_us);
          emit(l);
        };
        /* Woke late: the window was planned for `go` only (as run_gap). */
        if (const auto woke = serving_now(); !woke || *woke - go > gap_guard_us / 2) {
          skip("late", woke ? *woke - go : -1);
          break;
        }
        const auto g0 = std::chrono::steady_clock::now();
        bool out_ok = true, sent = false;
        try {
          dev->FastRetune(ho_pending->channel);
        } catch (const std::exception &) {
          out_ok = false;
        }
        const auto g1 = std::chrono::steady_clock::now();
        if (out_ok) {
          /* The outbound retune may overrun: send only if the instant is
           * still (about) ahead — the copy must not slide into the target's
           * span, nor the return past the serving beacon. */
          const auto t_now = serving_now();
          const int64_t at = S * P + *v;
          if (!t_now || *t_now > at + 1000) {
            skip("retune_overrun", t_now ? *t_now - at : -1);
          } else {
            if (*t_now < at)
              std::this_thread::sleep_for(std::chrono::microseconds(at - *t_now));
            auto pkt = build_dl_radiotap(6);
            pkt.insert(pkt.end(), last_bcn.begin(), last_bcn.end());
            sent = dev->send_packet(pkt.data(), pkt.size());
            std::this_thread::sleep_for(std::chrono::microseconds(kVisitHoldUs));
          }
        }
        const auto g1b = std::chrono::steady_clock::now();
        bool back_ok = false;
        for (int k = 0; k < 3 && !back_ok; ++k) {
          try {
            dev->FastRetune(cur_ch);
            back_ok = true;
          } catch (const std::exception &) {
          }
        }
        const auto g2 = std::chrono::steady_clock::now();
        if (!back_ok) {
          if (beaconing && dev->StopBeacon())
            beaconing = false;
          retune_to = cur_ch;
        }
        /* The bound only grows with what was measured (as for gaps). */
        gap_retune_us = std::max<int64_t>(
            {gap_retune_us, std::chrono::duration_cast<std::chrono::microseconds>(g1 - g0).count(),
             std::chrono::duration_cast<std::chrono::microseconds>(g2 - g1b).count()});
        /* Steps wait only out this superframe (visits come every one: a
         * longer hold would freeze them for the whole window). */
        no_step_until_sf = S + 1;
        ++visits;
        visits_ok += out_ok && back_ok && sent ? 1 : 0;
        char l[260];
        snprintf(l, sizeof l,
                 "{\"ev\":\"cell.ue.daps_visit\",\"ue\":%d,\"sf\":%lld,\"at_us\":%lld,\"ch\":%u,"
                 "\"sent\":%d,\"out_us\":%lld,\"deaf_us\":%lld,\"ok\":%d}",
                 me, (long long)S, (long long)*v, ho_pending->channel, sent ? 1 : 0,
                 (long long)std::chrono::duration_cast<std::chrono::microseconds>(g1 - g0).count(),
                 (long long)std::chrono::duration_cast<std::chrono::microseconds>(g2 - g0).count(),
                 out_ok && back_ok ? 1 : 0);
        emit(l);
        break;
      }
    }
    /* Gaps go on through a conditional handover's preparation: the UE's own
     * measurement of the target is its switch condition. */
    if ((gap_every_sf > 0 || gap_suspend_every > 0) && (!ho_pending || ho_pending->conditional()) &&
        !ho_back) {
      std::vector<devourer::cell::Neighbour> other, susp;
      const int64_t sfn = du_now / P;
      const auto &sl = grid->slots();
      const std::vector<devourer::cell::GridSpan> busy = {
          {0, c.bcn_us},
          {sl[g->dl_slot].start_us, sl[g->dl_slot].len_us},
          {sl[g->ul_slot].start_us, sl[g->ul_slot].len_us}};
      {
        std::lock_guard<std::mutex> lk(mu);
        if (ue_ms() - nbrs_ms > 3000) { /* no list heard lately: none */
          nbrs.clear();
          have_serv_phase = false; /* no serving grid heard either */
        }
        for (const auto &n : nbrs) {
          /* Only a suspend gap reaches it (decided at the FLOOR bound, as the
           * DU decides): measured in this UE's suspend superframes instead. */
          if (gap_suspend_every > 0 &&
              devourer::cell::suspend_only(n, cur_ch, gap_retune_floor_us, gap_guard_us, P, busy)) {
            susp.push_back(n);
            continue;
          }
          auto pk = gap_parked.find(n.cell);
          if (n.channel != 0 && n.channel != cur_ch && (pk == gap_parked.end() || sfn >= pk->second))
            other.push_back(n);
        }
      }
      /* A suspend gap STARTS in one of this UE's suspend superframes S, at
       * ((offset - retune - guard) mod P) into it, and so ends before S + 2
       * (its length is under P): exactly the superframes the DU skips this
       * UE's DL in. Taken for S = sfn + 1, or for S = sfn itself if this
       * pass came late and its start is still ahead. */
      /* The next suspend gap's start on the DU clock (for an idle gap not
       * to run into it), from a fresh reading: the pass may have slept. */
      const int64_t susp_len = 2 * (gap_retune_us + gap_guard_us) + 500;
      auto next_susp = [&](int64_t now) -> std::optional<std::pair<int64_t, int64_t>> {
        if (susp.empty() || susp_len >= P)
          return std::nullopt;
        const auto &n = susp[susp_rr % susp.size()];
        const int64_t w = ((n.offset_us - gap_retune_us - gap_guard_us) % P + P) % P;
        for (int64_t S = now / P; S <= now / P + gap_suspend_every + 1; ++S)
          if (S > last_susp_sf && devourer::cell::suspend_sf(S, static_cast<uint16_t>(me), gap_suspend_every) &&
              S * P + w - now > gap_guard_us)
            return std::make_pair(S, S * P + w);
        return std::nullopt;
      };
      bool susp_ran = false; /* this pass slept through a suspend gap: its sfn is stale */
      const bool had_susp = had_susp_prev;
      had_susp_prev = !susp.empty();
      if (!susp.empty()) {
        if (susp_len >= P) {
          if (!susp_infeasible) { /* the measured retunes outgrew the superframe */
            susp_infeasible = true;
            char l[160];
            snprintf(l, sizeof l, "{\"ev\":\"cell.ue.suspend_infeasible\",\"ue\":%d,\"retune_us\":%lld}", me,
                     (long long)gap_retune_us);
            emit(l);
          }
        } else if (const auto now_du = serving_now()) {
          /* Needed from now on (a list just heard): nothing before is missed. */
          if (!had_susp)
            last_susp_sf = std::max(last_susp_sf, *now_du / P - 1);
          /* Suspend superframes gone by without their gap since the last one
           * accounted for: said, never silent (each, or one aggregate after
           * a long pause). */
          {
            const int64_t w0 =
                ((susp[susp_rr % susp.size()].offset_us - gap_retune_us - gap_guard_us) % P + P) % P;
            int64_t n_missed = 0, first_missed = -1, last_missed = -1;
            for (int64_t S = last_susp_sf + 1; S <= *now_du / P; ++S)
              if (devourer::cell::suspend_sf(S, static_cast<uint16_t>(me), gap_suspend_every) &&
                  S * P + w0 <= *now_du) {
                if (n_missed++ == 0)
                  first_missed = S;
                last_missed = S;
              }
            if (n_missed > 0) {
              last_susp_sf = last_missed;
              char l[200];
              if (n_missed <= 3)
                for (int64_t S = first_missed; S <= last_missed; ++S) {
                  if (!devourer::cell::suspend_sf(S, static_cast<uint16_t>(me), gap_suspend_every))
                    continue;
                  snprintf(l, sizeof l, "{\"ev\":\"cell.ue.suspend_missed\",\"ue\":%d,\"sf\":%lld}", me,
                           (long long)S);
                  emit(l);
                }
              else {
                snprintf(l, sizeof l,
                         "{\"ev\":\"cell.ue.suspend_missed\",\"ue\":%d,\"sf\":%lld,\"to_sf\":%lld,\"n\":%lld}",
                         me, (long long)first_missed, (long long)last_missed, (long long)n_missed);
                emit(l);
              }
            }
          }
          /* Taken if it starts within this pass's reach (< 2 P ahead). */
          if (const auto ns = next_susp(*now_du); ns && ns->second - *now_du < 2 * P) {
            const auto n = susp[susp_rr % susp.size()];
            /* Only a gap that ran consumes its superframe; a failed one (late,
             * the beacon would not stop) is left to the missed-scan above. */
            if (run_gap(n, ns->second, susp_len, true)) {
              last_susp_sf = ns->first;
              ++susp_rr;
              susp_ran = true;
            }
          }
        }
      }
      /* Each measurable neighbour every gap_every_sf: one gap per
       * gap_every_sf / n superframes (at least 2 apart). */
      const int64_t every =
          std::max<int64_t>(2, gap_every_sf / static_cast<int64_t>(std::max<size_t>(1, other.size())));
      if (next_gap_sf < 0)
        next_gap_sf = sfn + every;
      if (gap_every_sf > 0 && !susp_ran && !other.empty() && sfn >= next_gap_sf) {
        next_gap_sf = sfn + every;
        const auto n = other[gap_rr++ % other.size()];
        const bool cross_band = devourer::cell::gap_cross_band(cur_ch, n.channel);
        const auto w = cross_band ? std::nullopt
                                  : devourer::cell::gap_window(n.offset_us, 500, gap_retune_us,
                                                               gap_guard_us, P, busy);
        if (!w) {
          gap_parked[n.cell] = sfn + 50 * every; /* rarely re-checked */
          char l[300];
          snprintf(l, sizeof l,
                   "{\"ev\":\"cell.ue.gap_skip\",\"ue\":%d,\"cell\":%u,\"offset_us\":%d,"
                   "\"why\":\"%s\"}",
                   me, n.cell, n.offset_us, cross_band ? "band" : "slots");
          emit(l);
        } else {
          gap_parked.erase(n.cell);
          /* Never into the next suspend gap: an idle gap and the 100 ms pass
           * after it would carry the loop past that gap's start (and the
           * same alignment would repeat every time). Deferred a superframe. */
          const int64_t start = (sfn + 1) * P + w->start;
          const auto now_du = serving_now();
          const auto ns = now_du ? next_susp(*now_du) : std::nullopt;
          if (ns && start + w->len + 150000 > ns->second && start < ns->second + susp_len) {
            next_gap_sf = sfn + 1;
            --gap_rr; /* the same neighbour, next superframe */
          } else if (!run_gap(n, start, w->len, false)) {
            next_gap_sf = sfn + 1; /* its start went by (a slept pass): the next superframe */
            --gap_rr;
          }
        }
      }
    }
  }
  if (beaconing)
    dev->StopBeacon();
  dev->StopRxLoop();
  rx.join();
  dev->ClearAckResponder();
  char l[700];
  snprintf(l, sizeof l,
           "{\"ev\":\"cell.ue.done\",\"ue\":%u,\"steps\":%ld,\"step_fails\":%ld,"
           "\"listen_from_sf\":%lld,\"listen_to_sf\":%lld,\"dl_new\":%llu,\"dl_dup\":%llu,"
           "\"keyed\":%d,\"gm_ok\":%llu,\"gm_refused\":%llu,\"nbr_refused\":{\"stale\":%llu,"
           "\"unlisted\":%llu,\"phase\":%llu},\"dl_pdus\":{\"delivered\":%llu,\"duplicates\":%llu,"
           "\"missing\":%llu,\"stale\":%llu,\"top\":%lld},\"suspend_gaps\":{\"n\":%llu,\"heard\":%llu,"
           "\"fail\":%llu},\"daps_visits\":{\"n\":%llu,\"ok\":%llu,\"dl_answers\":%llu},\"steps_held\":%llu}",
           me_a.load(), steps, step_fails, (long long)listen_from, (long long)listen_to,
           (unsigned long long)dl_new, (unsigned long long)dl_dup, auth ? 1 : 0,
           (unsigned long long)gm_ok, (unsigned long long)gm_refused,
           (unsigned long long)nbr_refused["stale"], (unsigned long long)nbr_refused["unlisted"],
           (unsigned long long)nbr_refused["phase"], (unsigned long long)dl_stream.delivered(),
           (unsigned long long)dl_stream.duplicates(), (unsigned long long)dl_stream.missing_total(),
           (unsigned long long)dl_stream.stale(),
           dl_stream.top() ? (long long)*dl_stream.top() : -1LL, (unsigned long long)susp_gaps,
           (unsigned long long)susp_heard, (unsigned long long)susp_fail, (unsigned long long)visits,
           (unsigned long long)visits_ok, (unsigned long long)visit_dl_rx, (unsigned long long)steps_held);
  emit(l);
  return 0;
}

// ---------------------------------------------------------------------------
int run_witness(IRadio *dev, const Cfg &c) {
  /* With CELL_PSK, a beacon's grant map counts only if authentic; without
   * one the witness is a passive recorder (structure only). */
  const CellMac wauth = cell_auth(c);
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
        } else if (auto m = devourer::cell::find_grant_map(d, p.Data.size(), wauth)) {
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

// ---------------------------------------------------------------------------
/* CELL_ROLE=replayer (test only): the replay attacker of MeasGap.h. It records
 * cell CELL_REPLAY_CELL's genuine DU beacons until CELL_REPLAY_FREEZE_S, then
 * keeps re-airing the last one every CELL_REPLAY_EVERY_MS [100] until the end
 * — signature, certificate and all, never on the cell's TBTT phase. */
int run_replayer(IRadio *dev, const Cfg &c) {
  const auto cell = static_cast<uint16_t>(envl("CELL_REPLAY_CELL", 2, 1, 65535));
  const int freeze_s = static_cast<int>(envl("CELL_REPLAY_FREEZE_S", 30, 1, 86400));
  const int every_ms = static_cast<int>(envl("CELL_REPLAY_EVERY_MS", 100, 10, 60000));
  uint8_t du[6];
  cell_mac(du, cell, 0);
  std::mutex m;
  std::vector<uint8_t> last;
  uint64_t captured = 0;
  std::atomic<bool> frozen{false};
  dev->InitWrite(SelectedChannel{c.ch, 0, CHANNEL_WIDTH_20});
  std::thread rx([&] {
    dev->StartRxLoop([&](const Packet &p) {
      if (frozen || p.RxAtrib.crc_err || p.Data.size() < 36 || p.Data[0] != 0x80 ||
          std::memcmp(p.Data.data() + 10, du, 6) != 0)
        return;
      std::lock_guard<std::mutex> lk(m);
      last.assign(p.Data.begin(), p.Data.end());
      ++captured;
    });
  });
  const auto t0 = std::chrono::steady_clock::now();
  std::this_thread::sleep_until(t0 + std::chrono::seconds(freeze_s));
  frozen = true;
  std::vector<uint8_t> frame;
  {
    std::lock_guard<std::mutex> lk(m);
    frame = last;
    char l[120];
    snprintf(l, sizeof l, "{\"ev\":\"cell.replay.frozen\",\"cell\":%u,\"captured\":%llu,\"len\":%zu}",
             cell, (unsigned long long)captured, frame.size());
    emit(l);
  }
  uint64_t sent = 0;
  const auto end = t0 + std::chrono::seconds(c.secs);
  while (!frame.empty() && std::chrono::steady_clock::now() < end) {
    auto pkt = build_dl_radiotap(6);
    pkt.insert(pkt.end(), frame.begin(), frame.end());
    if (dev->send_packet(pkt.data(), pkt.size()))
      ++sent;
    std::this_thread::sleep_for(std::chrono::milliseconds(every_ms));
  }
  char l[120];
  snprintf(l, sizeof l, "{\"ev\":\"cell.replay.done\",\"cell\":%u,\"sent\":%llu}", cell,
           (unsigned long long)sent);
  emit(l);
  dev->StopRxLoop();
  rx.join();
  return 0;
}

}  // namespace

int main() {
  const char *role = std::getenv("CELL_ROLE");
  if (!role || (std::strcmp(role, "du") && std::strcmp(role, "ue") && std::strcmp(role, "witness") &&
                std::strcmp(role, "replayer"))) {
    fprintf(stderr, "set CELL_ROLE=du|ue|witness|replayer\n");
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
  if (std::strcmp(role, "replayer") == 0)
    return run_replayer(dev, c);
  if (!caps_ok(dev))
    return 1;
  return std::strcmp(role, "du") == 0 ? run_du(dev, c) : run_ue(dev, c);
}
