/* sta_client.cpp — devourer as an 802.11 infrastructure STATION.
 *
 * The mirror of tests/ap_responder.cpp and tests/ap_wpa2.cpp: where those
 * serve a BSS, this one JOINS one. Scan, authenticate, associate, run the
 * WPA2-PSK four-way as the supplicant, and carry CCMP-protected data to and
 * from the host through a TAP device. The protocol is src/sta/; this file is
 * the integration around it, over IRadio.
 *
 * NO BACKEND BRANCH. The two places where the silicon genuinely differs are
 * handled through the library:
 *
 *   - the trailing FCS, via RxAtrib.fcs_present (see mpdu_len()).
 *   - the station identity, via IRadio::SetStationIdentity, called only when
 *     AdapterCaps::station_mode_ok is true (MT7612U, and the Realtek 8822C /
 *     8822B arm - docs/realtek-station-arm.md). A backend that reports false
 *     is refused here (exit 2) rather than run as a station whose ACKs nobody
 *     armed; DEVOURER_STA_ARM=0 runs it unarmed, as a control.
 *
 * WHAT THIS OWNS THAT src/sta/ DOES NOT:
 *
 *   - THE SCANNER. BssTable parses beacons and ranks them; scan_step() and
 *     supervise() wire it to StationSm with channels and dwell times.
 *   - THE RECONNECT POLICY. StationSm notices a silent AP and fails; when to
 *     re-join is an integrator's decision, and supervise() is this file's.
 *   - THE DATA PLANE. CCMP framing is library code; which key, which PN space
 *     and which replay / duplicate window a frame belongs to is decided here.
 *     There is one transmitter on this data plane - the joined AP - so one
 *     DupDetector covers it (src/sta/Dot11.h: one per transmitter).
 *
 * TRANSMISSION IS THIS FILE'S, NOT THE ARM'S (IRadio::SetStationIdentity).
 * Unicast airs with an ACK-requesting radiotap (DEVOURER_STA_ACK=0 turns that
 * off), and the hardware retry limit defaults to kStationRetryLimit unless
 * DEVOURER_TX_RETRY_LIMIT is set - so DEVOURER_TX_RETRY_LIMIT=0 is how a run
 * asks for the single-shot uplink the library warns about at arm time.
 *
 * THE STATION'S OWN ADDRESS IS THE ADAPTER'S. On MT7612U the auto-response
 * engine matches address 1 against MT_MAC_ADDR, so a station transmitting
 * from any other address is not acknowledged (docs/mt7612u-station-identity.md).
 * `own` comes from GetPermanentMacAddress and is never invented.
 *
 * THE RECEIVE PATH IS PROMISCUOUS on MT7612U until the arm
 * (Mt7612uRadio::StartRxLoop installs the monitor filter); the arm installs
 * the managed filter, which drops unicast not addressed to `own` but still
 * passes every BSS's beacons and group traffic, and DEVOURER_STA_ARM=0 stays
 * promiscuous. So StationSm::on_rx is the address filter either way, and its
 * refusal counters are printed at every exit: they distinguish "the AP never
 * answered" from "we never heard the AP". Its `not-for-us` count (our BSS,
 * someone else's unicast) is also the witness that the managed filter is on:
 * near zero while armed, whatever such traffic is on the air.
 *
 * Exit status: 0 the run completed (the ledger says how it went); 1 setup
 * failed; 2 refused - the adapter's station_mode_ok is false, or the
 * duration, DEVOURER_CHANNEL, DEVOURER_STA_SCAN_DWELL_MS or
 * DEVOURER_STA_BACKOFF_MS is not a valid number in range; 3 a FAULT - an
 * exception was caught or the TAP failed mid-run. The run still left the BSS,
 * cleared the identity and printed its ledger (whose first line then carries
 * `fault=1`), but it did not end the way it was asked to.
 *
 * Build: CMake target StaClientSelftest, binary build/sta_client.
 * Run:
 *   sudo DEVOURER_VID=0x0e8d DEVOURER_PID=0x7612 DEVOURER_CHANNEL=6 \
 *        DEVOURER_STA_SSID=devourerSTA DEVOURER_STA_PSK=devourer123 \
 *        DEVOURER_STA_TAP=dvsta0 build/sta_client 60
 * Headless (no device, no root, no airtime):
 *   build/sta_client --self-test
 * On air: tests/sta_client_onair.sh.
 */
#include <atomic>
#include <chrono>
#include <cstdint>
#include <poll.h>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include <time.h>

#include <csignal>
#include <fcntl.h>
#include <linux/if.h>
#include <linux/if_tun.h>
#include <net/if_arp.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <libusb.h>
#include <openssl/rand.h>

#include "DeviceSession.h"
#include "RadiotapBuilder.h"
#include "RxPacket.h"
#include "SelectedChannel.h"
#include "TxMode.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"
#include "openssl_crypto_ops.h"
#include "sta/BssTable.h"
#include "sta/Ccmp.h"
#include "sta/Dot11.h"
#include "sta/StationSm.h"
#include "usb_select.h"

namespace {

using devourer::sta::BssEntry;
using devourer::sta::BssTable;
using devourer::sta::StationSm;

/* ---- configuration ----------------------------------------------------- */

/* The hardware retry limit a station runs with unless DEVOURER_TX_RETRY_LIMIT
 * says otherwise. Nonzero because a station's unicast - authentication,
 * association, the four-way, data - relies on MAC retransmission, and the
 * library default of 0 sends each frame exactly once. */
constexpr int kStationRetryLimit = 7;

std::string g_ssid = "devourerAP";
std::string g_psk;                 /* empty means an OPEN network */
uint8_t g_chan = 6;
/* The channels scan_step() sweeps while unassociated; defaults to the one
 * configured channel. Never swept while associated - retuning under a live
 * association loses it. */
std::vector<uint8_t> g_scan_chans;
uint32_t g_scan_dwell_ms = 250;
bool g_reconnect = true;
uint32_t g_rejoin_backoff_ms = 1000;
/* DEVOURER_STA_ARM=0: never call SetStationIdentity. A control: everything
 * else about the run is unchanged. */
bool g_arm = true;

/* ---- the radio side, which the headless cells never touch --------------- */

IRadio* g_dev = nullptr;
std::vector<uint8_t> g_rt;      /* NOACK radiotap: group-addressed frames */
std::vector<uint8_t> g_rt_ack;  /* ACK-requested: unicast, unless empty */
std::mutex g_q_mu;
std::vector<std::vector<uint8_t>> g_q;
std::atomic<uint64_t> g_sent{0}, g_send_fail{0}, g_q_drop{0};
/* The channel the radio is tuned to: BssTable::observe takes it as the
 * channel of a beacon that does not state its own. Written by the main loop,
 * read by the RX thread. No backend reports a frame's own RX channel
 * (RxAtrib has none), so a frame received just before a retune can be
 * delivered after it: 0 while a retune is in progress, and for kRetuneGuardMs
 * after one rx_frame() passes "unknown" (0) - a beacon without a DS Parameter
 * Set is then not folded in at all, rather than tagged with the new channel. */
std::atomic<uint8_t> g_tuned{6};
std::atomic<uint32_t> g_retune_ms{0};
constexpr uint32_t kRetuneGuardMs = 50;

/* ---- the station core, under one mutex --------------------------------- */

std::mutex g_mu;
devourer::test::OpenSslCryptoOps g_crypto;
BssTable g_bss;
StationSm g_sm;
uint8_t g_own[6] = {0};
devourer::sta::SeqCounter g_data_seq;

/* THE TRANSMIT PN SPACE BELONGS TO THE PAIRWISE KEY. It starts at 1 (PN 0 is
 * never valid) and restarts at every PTK install - see note_keys(): reusing a
 * PN under a new key is keystream reuse. */
uint64_t g_tx_pn = 1;
devourer::sta::CcmpReplay g_rx_replay;       /* pairwise, per TID */
/* 802.11 duplicate detection for the AP's unicast: a retransmission a lost
 * ACK caused is dropped before decrypt and counted as a duplicate, not a
 * replay. Group frames are never retried and would clobber the cache. Reset
 * per association (on_association), never per key. */
devourer::sta::DupDetector g_rx_dup;
std::atomic<uint64_t> g_dup_drop{0};
devourer::sta::CcmpReplay g_group_replay;    /* the GTK's own PN space */
/* WHICH KEYS THE PN STATE BELONGS TO, as the supplicant's install
 * generations rather than copies of the keys. */
uint32_t g_ptk_gen_seen = 0;
uint32_t g_gtk_gen_seen = 0;

/* the scan/join policy's own state */
size_t g_scan_idx = 0;
uint32_t g_scan_switch_ms = 0;
uint32_t g_next_join_ms = 0;
int g_join_attempts = 0;
bool g_gave_up = false;
/* Entering Failed is an EVENT, and supervise() runs on every loop pass: the
 * latch makes one lost link one reconnect, not one per pass. */
bool g_failed_noted = false;
/* Cleared by every join attempt is the latch above; this one says whether
 * the link was ever up since the last loss, so retries while the AP is away
 * are not counted as more lost links. */
bool g_was_associated = false;

/* ---- the ledger --------------------------------------------------------- */

std::atomic<uint64_t> g_beacons{0}, g_probe_tx{0};
std::atomic<uint64_t> g_joins{0}, g_associations{0}, g_reconnects{0};
std::atomic<uint64_t> g_enc_rx{0}, g_mic_fail{0}, g_replays{0};
std::atomic<uint64_t> g_group_rx{0}, g_plain_rx{0}, g_rx_short{0};
/* Plaintext data (not EAPOL) from our BSS on a protected link: refused, and
 * counted - it is also the on-air harness's positive witness that unicast
 * addressed to us gets through the receive filter. */
std::atomic<uint64_t> g_plain_refused{0};
/* One counter per direction, so each direction's books close on their own:
 *   from host == encrypted + plaintext + dropped down
 *   queued    == aired + queue dropped + send failed */
std::atomic<uint64_t> g_tap_tx{0}, g_tap_rx{0}, g_tap_drop{0},
    g_tap_down_drop{0};
std::atomic<uint64_t> g_q_in{0};   /* every frame handed to enqueue() */
std::atomic<bool> g_tap_stop{false}; /* the TAP reader's exit */
std::atomic<uint64_t> g_tap_read_err{0}; /* a fatal TAP poll/read: a fault */
std::atomic<uint64_t> g_tx_enc{0}, g_tx_enc_fail{0}, g_tx_plain{0};
std::atomic<uint64_t> g_crc_err{0}, g_amsdu_drop{0}, g_frag_drop{0};
/* The group / pairwise rekey rides INSIDE the cipher, so it is counted apart
 * from the four-way's cleartext EAPOL. */
std::atomic<uint64_t> g_eapol_enc_rx{0}, g_eapol_enc_tx{0};
/* How many times each key has actually been installed. A rekey the data
 * plane failed to notice is otherwise invisible: the link stays Connected
 * and frames stop arriving. */
std::atomic<uint64_t> g_ptk_installs{0}, g_gtk_installs{0};
/* Protected frames we hold NO KEY for - a group frame at a key id the
 * supplicant has not installed. Counted apart from MIC
 * failures: a key we were never given is not tampering. */
std::atomic<uint64_t> g_no_key{0};

int g_tap_fd = -1;

/* A CLEAN STOP: the on-air harness ends a run by signalling this process,
 * and the ledger printed on the way out is the run's diagnostic. Written by
 * the signal handler AND by the RX and TAP threads, read by the main loop:
 * a std::atomic, because a volatile sig_atomic_t is only safe against a
 * signal handler, not across threads - and a lock-free one, so the
 * handler's store stays async-signal-safe. */
std::atomic<int> g_stop{0};
static_assert(std::atomic<int>::is_always_lock_free,
              "the signal handler's store must be lock-free");
extern "C" void on_signal(int) { g_stop.store(1); }
/* Set by every caught exception and by a failed TAP: the run is stopped
 * through the normal teardown, and the exit status says it was a fault
 * (exit_status()) rather than a run that completed. */
std::atomic<int> g_fault{0};
/* A fault stops the run; the teardown and the exit status do the rest. */
void fault(const char* what, const char* detail) {
  if (detail)
    std::fprintf(stderr, "sta_client: FAULT: %s threw: %s - stopping\n", what,
                 detail);
  else
    std::fprintf(stderr, "sta_client: FAULT: %s - stopping\n", what);
  g_fault = 1;
  g_stop = 1;
}
int exit_status() { return g_fault.load() ? 3 : 0; }

/* ---- small helpers ------------------------------------------------------ */

uint32_t now_ms() {
  static const auto t0 = std::chrono::steady_clock::now();
  return (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now() - t0)
      .count();
}

/* True when the frame was queued; false when the full queue dropped it. */
bool enqueue(std::vector<uint8_t> mpdu) {
  /* addr1's I/G bit: a group address is never ACKed. */
  const bool unicast = mpdu.size() >= 10 && (mpdu[4] & 0x01) == 0;
  const std::vector<uint8_t>& rt = (unicast && !g_rt_ack.empty()) ? g_rt_ack : g_rt;
  std::vector<uint8_t> f;
  f.reserve(rt.size() + mpdu.size());
  f.insert(f.end(), rt.begin(), rt.end());
  f.insert(f.end(), mpdu.begin(), mpdu.end());
  std::lock_guard<std::mutex> lk(g_q_mu);
  g_q_in.fetch_add(1);
  /* Bounded: everything queued here answers a received frame or a timer, so
   * an unbounded queue is an allocation the air controls. */
  if (g_q.size() < 128) {
    g_q.push_back(std::move(f));
    return true;
  }
  g_q_drop.fetch_add(1);
  return false;
}

/* The dBm convention this tree uses (src/LinkHealth.cpp, src/RxQuality.h):
 * dBm ~= raw - 110; MT7612U's mapping layer converts into the same raw
 * scale. Only the ORDERING matters - BssTable ranks BSSes heard by one radio
 * in one scan - and a raw 0 (the PHY reported nothing) must not outrank a
 * real reading, hence the floor rather than -110. */
int8_t rssi_dbm(uint8_t raw) {
  if (raw == 0) return -128;
  const int dbm = (int)raw - 110;
  return (int8_t)(dbm < -128 ? -128 : (dbm > 127 ? 127 : dbm));
}

/* ---- keys and per-association state ------------------------------------- */

/* AN OPEN ASSOCIATION IS CONFIRMED BY THE AP'S FIRST UNICAST REPLY. The
 * station cannot see the AP's side: an Association Response it received but
 * whose acknowledgement the AP never saw leaves the AP without the station,
 * and on an open BSS nothing says so - the AP drops the station's uplink and
 * may never deauthenticate it. (On WPA2 the four-way is the confirmation:
 * the AP starts it only for a station it holds, and HandshakeTimeout covers
 * the rest.) So an open association counts as unconfirmed until a unicast
 * data frame from the AP arrives for this station; if the host has asked
 * kConfirmUplink questions and kConfirmMs has passed since the first of them
 * without one, the link is lost (StationSm::link_lost) and the ordinary
 * re-join policy takes over. The window opens at the host's first question,
 * not at the association: a host idle for longer than kConfirmMs that then
 * sends a burst must still get its kConfirmMs for the reply.
 *
 * A QUESTION IS A FRAME WHOSE ANSWER, IF ONE EXISTS, THE AP MUST FORWARD BACK
 * (solicits_reply): an ARP request, an ICMP / ICMPv6 echo request, a unicast
 * IPv6 neighbour solicitation, a TCP SYN, a DNS query. So one-way traffic -
 * a UDP video or telemetry uplink, the FPV case - is never judged, and
 * neither is a host's multicast chatter (IPv6 RS/MLD, mDNS), a gratuitous or
 * probe ARP, or an idle host. But a question can go unanswered on a healthy
 * link: the host pings or ARPs a peer that is switched off, or its DNS has
 * no upstream.
 *
 * BACKOFF PER BSS. So consecutive verdicts on one BSS back off: after n of
 * them (g_strikes), the next association on that BSS is judged only once
 * kConfirmMs * 2^n has passed since it was made (strike_backoff_ms), capped
 * at kStrikeCapMs. Questions asked inside the backoff are not counted. A
 * host asking a dead peer therefore costs at most one re-join per backoff
 * period - 10 s, 20 s, 40 s ... then one every 2 minutes - instead of one
 * every kConfirmMs, while an association the AP really dropped is still
 * found, at worst a backoff period late. A unicast reply (the confirmation)
 * or an association on a different BSS resets the count. An unheld
 * association under one-way traffic alone is found only when the host's
 * stack next asks something (its neighbour re-verification is a unicast ARP
 * request). */
constexpr uint32_t kConfirmMs = 5000;
constexpr uint32_t kConfirmUplink = 3;
constexpr uint32_t kStrikeCapMs = 120000;
/* docs/station-client.md and the on-air harness state these numbers. */
static_assert(kConfirmMs == 5000 && kConfirmUplink == 3 &&
                  kStrikeCapMs == 120000,
              "docs/station-client.md documents 3 questions / 5 s / a 2 min "
              "backoff cap: change them together");
/* An open association with no unicast reply yet: the verdict may fire for
 * it, once g_judge_from_ms has passed. */
bool g_judge = false;
uint32_t g_judge_from_ms = 0;
uint32_t g_strikes = 0;             /* consecutive verdicts on... */
uint8_t g_strike_bss[6] = {0};      /* ...this BSS */

/* How long an association on a BSS with `strikes` consecutive verdicts waits
 * before it is judged: 0, then kConfirmMs * 2^strikes, capped. */
uint32_t strike_backoff_ms(uint32_t strikes) {
  if (strikes == 0) return 0;
  uint32_t d = kConfirmMs;
  for (uint32_t i = 0; i < strikes && d < kStrikeCapMs; i++) d *= 2;
  return d < kStrikeCapMs ? d : kStrikeCapMs;
}

/* Wall-clock seconds.microseconds, the form hostapd -t stamps its lines
 * with, so the on-air harness can order the station's events against the
 * AP's. Into `buf`, which it returns. */
const char* wall_stamp(char* buf, size_t n) {
  struct timespec ts {};
  clock_gettime(CLOCK_REALTIME, &ts);
  std::snprintf(buf, n, "%lld.%06ld", (long long)ts.tv_sec,
                (long)(ts.tv_nsec / 1000));
  return buf;
}
bool g_uplink_seen = false;         /* supervise() saw the first question */
uint32_t g_uplink_first_ms = 0;     /* ...at this time: the window opens */
uint32_t g_uplink_unconfirmed = 0;
std::atomic<uint64_t> g_unconfirmed_lost{0};

/* Whether the host's MSDU (LLC/SNAP + payload) asks for an answer that, if
 * one exists, the AP must carry back to this station (kConfirmMs above).
 * Conservative: anything not recognised is not a question. */
bool solicits_reply(const uint8_t* msdu, size_t len, const uint8_t da[6]) {
  if (len < devourer::sta::kLlcSnapLen) return false;
  const uint8_t* p = msdu + devourer::sta::kLlcSnapLen;
  const size_t n = len - devourer::sta::kLlcSnapLen;
  const unsigned et = ((unsigned)msdu[6] << 8) | msdu[7];
  if (et == 0x0806) {
    /* An ARP request (op 1) for someone else's address; not a gratuitous
     * one (sender == target) or a duplicate-address probe (sender 0), which
     * no one answers. Sender IP at 14, target IP at 24. */
    static const uint8_t zero4[4] = {0, 0, 0, 0};
    return n >= 28 && p[6] == 0 && p[7] == 1 &&
           std::memcmp(p + 14, zero4, 4) != 0 &&
           std::memcmp(p + 14, p + 24, 4) != 0;
  }
  if (da[0] & 0x01) return false;   /* group-addressed IP: no unicast owed */
  uint8_t proto = 0;
  const uint8_t* l4 = nullptr;
  size_t l4n = 0;
  if (et == 0x0800) {
    if (n < 20 || (p[0] >> 4) != 4) return false;
    const size_t ihl = (size_t)(p[0] & 0x0f) * 4;
    /* A non-first fragment carries no transport header. */
    if (ihl < 20 || n < ihl || ((p[6] & 0x1f) | p[7]) != 0) return false;
    proto = p[9];
    l4 = p + ihl;
    l4n = n - ihl;
    if (proto == 1) return l4n >= 1 && l4[0] == 8;          /* echo request */
  } else if (et == 0x86dd) {
    if (n < 40 || (p[0] >> 4) != 6) return false;
    proto = p[6];
    l4 = p + 40;
    l4n = n - 40;
    if (proto == 58)                  /* echo request, unicast NS (NUD) */
      return l4n >= 1 && (l4[0] == 128 || l4[0] == 135);
  } else {
    return false;
  }
  /* TCP: a SYN only (SYN set, ACK clear). Any other segment can go
   * unanswered on a healthy link: a RST, a keepalive or a retransmission to a
   * peer that has gone, or a bare ACK left over from before a re-join. */
  if (proto == 6) return l4n >= 14 && (l4[13] & 0x12) == 0x02;
  /* UDP: a DNS query only - any other UDP may be one-way. */
  return proto == 17 && l4n >= 4 && (((unsigned)l4[2] << 8) | l4[3]) == 53;
}

/* Called under g_mu when the station reaches Connected on a new
 * association. The duplicate cache is reset here and NOT at a rekey: it is
 * per transmitter and TID over Sequence Control (DupDetector, Dot11.h), which
 * a rekey does not restart - a Retry copy of the rekey's own message 3 must
 * still be a duplicate. The per-key state is not reset here: a PTK or GTK
 * rekey happens with the machine already Connected, so note_keys() owns it,
 * keyed on the supplicant's install generations. */
bool probe(uint8_t chan);   /* below, with the scan */
std::atomic<uint64_t> g_nudges{0};

/* THE NUDGE. An AP may hold a transmitted frame's TX status until its next
 * transmission - the MT7612U on mt76x2u does (docs/station-client.md) - and
 * hostapd acts on an association only once the Association Response's
 * status (ACK) is in: it counts the station associated, and on WPA2 starts
 * the four-way, only from that status. Nothing else need make the AP
 * transmit to us soon, so the association - and the four-way - can stall for
 * seconds while the station's traffic is dropped. One probe request, which
 * every AP answers, makes it transmit and releases the held status. Sent the
 * moment an Association Response is accepted, open or WPA2; on WPA2 a
 * second one follows if no EAPOL has arrived kNudgeAgainMs later (supervise),
 * and the four-way timeout re-joins if even that is not enough. Caller holds
 * g_mu. */
constexpr uint32_t kNudgeAgainMs = 1000;
uint32_t g_nudge_ms = 0;
bool g_nudge_again = false;   /* a second nudge is still owed (WPA2) */
uint32_t g_nudge_eapol_rx = 0;
void nudge(uint32_t now) {
  if (probe(g_sm.channel() ? g_sm.channel() : g_chan)) g_nudges.fetch_add(1);
  g_nudge_ms = now;
}

void on_association(uint32_t now) {
  /* The backoff is per BSS: a different BSS is judged afresh. */
  if (g_strikes && std::memcmp(g_strike_bss, g_sm.bssid(), 6) != 0)
    g_strikes = 0;
  g_judge = g_sm.security() == StationSm::Security::Open;
  g_judge_from_ms = now + strike_backoff_ms(g_strikes);
  g_uplink_seen = false;
  g_uplink_unconfirmed = 0;
  g_rx_dup.reset();
  g_failed_noted = false;
  g_was_associated = true;
  const uint64_t n = g_associations.fetch_add(1) + 1;
  /* One line per association, so a re-join is visible while the run lasts
   * and not only in the exit ledger. */
  char at[32];
  std::fprintf(stderr, "  station connected (association %llu) at=%s\n",
               (unsigned long long)n, wall_stamp(at, sizeof at));
}

/* A REKEY RESTARTS A PN SPACE, and the windows restart with it - both
 * directions, both keys. The AP's new key starts at PN 1, so a window left at
 * the old key's head would reject every frame; and our transmit PN under a
 * new key must restart, because continuing it is keystream reuse.
 * Caller holds g_mu. */
void note_keys() {
  const devourer::sta::Supplicant& sup = g_sm.supplicant();

  if (sup.ptk_valid() && sup.ptk_generation() != g_ptk_gen_seen) {
    g_ptk_gen_seen = sup.ptk_generation();
    g_tx_pn = 1;
    g_rx_replay.reset();
    g_ptk_installs.fetch_add(1);
  }
  if (sup.gtk_valid() && sup.gtk_generation() != g_gtk_gen_seen) {
    g_gtk_gen_seen = sup.gtk_generation();
    /* Seeded from the AUTHENTICATED Key RSC, not reset: a window opened at
     * whichever group frame arrives first would accept a replayed capture
     * from earlier in this GTK's life. */
    g_group_replay.seed(sup.gtk_rsc());
    g_gtk_installs.fetch_add(1);
  }
}

/* Defined with the transmit path below; the receive path needs it for a
 * rekey's answer, which is encrypted exactly as a data frame is.
 * `from_host` false for an MSDU this file originates itself (a rekey's EAPOL
 * answer): it is then counted in g_eapol_enc_tx, not in the host's books. */
bool air_msdu(const uint8_t* msdu, size_t len, const uint8_t da[6],
              const uint8_t* tk = nullptr, bool from_host = true);

/* ---- UP: one received MPDU --------------------------------------------- */

/* Hand a decrypted (or never-encrypted) MSDU to the host. Returns false when
 * it is not something the host can be given - a non-ethertype LLC encoding,
 * or too big for the buffer. Caller holds g_mu. */
bool tap_up(const uint8_t* da, const uint8_t* sa, const uint8_t* msdu,
            size_t len) {
  uint8_t eth[2048];
  const size_t n = devourer::sta::msdu_to_eth(da, sa, msdu, len, eth,
                                              sizeof eth);
  if (n == 0) { g_tap_drop.fetch_add(1); return false; }
  if (g_tap_fd < 0) return true;          /* no TAP: counted, not an error */
  if (::write(g_tap_fd, eth, n) == (ssize_t)n) { g_tap_tx.fetch_add(1); return true; }
  /* The fd is non-blocking (tap_open): a host that is not reading costs a
   * counted drop (EAGAIN), never a stalled RX thread holding g_mu. */
  g_tap_drop.fetch_add(1);
  return false;
}

/* THE RECEIVE DECISION, with no Packet and no radio in it, so every branch is
 * reachable from `sta_client --self-test`. `mpdu`/`len` is the MPDU without
 * its FCS (mpdu_len()). */
void rx_frame(const uint8_t* mpdu, size_t len, int8_t rssi, uint32_t now) {
  std::lock_guard<std::mutex> l(g_mu);

  if (len < 24) { g_rx_short.fetch_add(1); return; }

  /* The scan folds in EVERY beacon and probe response on the channel; on_rx
   * below ignores everything that is not our BSS, so the two need not agree
   * about which AP matters. */
  if (mpdu[0] == devourer::sta::kFcBeacon ||
      mpdu[0] == devourer::sta::kFcProbeResp) {
    uint8_t rx_chan = g_tuned.load();
    if ((uint32_t)(now - g_retune_ms.load()) < kRetuneGuardMs) rx_chan = 0;
    devourer::sta::BssInfo ds;
    /* Channel unknown and the frame does not state one: not folded in (see
     * g_tuned). */
    const bool usable = rx_chan != 0 ||
                        (devourer::sta::parse_beacon(mpdu, len, &ds) &&
                         ds.channel != 0);
    if (usable && g_bss.observe(mpdu, len, rssi, rx_chan, now))
      g_beacons.fetch_add(1);
  }

  const StationSm::State before = g_sm.state();
  g_sm.on_rx(mpdu, len, now);
  /* An Association Response accepted: Associating -> Connected (open) or
   * FourWay (WPA2). */
  if (before == StationSm::State::Associating &&
      g_sm.state() != StationSm::State::Associating &&
      g_sm.state() != StationSm::State::Failed) {
    nudge(now);
    g_nudge_again = g_sm.state() == StationSm::State::FourWay;
    g_nudge_eapol_rx = g_sm.eapol_rx;
  }
  if (before != StationSm::State::Connected && g_sm.connected())
    on_association(now);
  if (g_sm.connected()) note_keys();

  /* The data plane runs only on a live association: a protected frame that
   * arrives before one cannot be decrypted with a key we do not have. */
  if (!g_sm.connected()) return;

  const uint8_t fc0 = mpdu[0], fc1 = mpdu[1];
  if (fc0 != devourer::sta::kFcData && !devourer::sta::is_qos_data(fc0)) return;
  /* A station receives from the DS. ToDS set is another station's uplink. */
  if (!(fc1 & devourer::sta::kFcFromDs) || (fc1 & devourer::sta::kFcToDs))
    return;
  if (std::memcmp(mpdu + 10, g_sm.bssid(), 6) != 0) return;
  const bool to_us = std::memcmp(mpdu + 4, g_own, 6) == 0;
  const bool group = (mpdu[4] & 0x01) != 0;
  if (!to_us && !group) return;

  /* FRAGMENTS AND A-MSDUs ARE REFUSED, NOT MISREAD: neither is reassembled
   * here, and half an MSDU handed up as a whole one decodes to nonsense.
   * More Fragments is clear on a LAST fragment, so the fragment number is
   * checked too. */
  if ((fc1 & devourer::sta::kFcMoreFrag) || (mpdu[22] & 0x0f)) {
    g_frag_drop.fetch_add(1);
    return;
  }
  const size_t hlen = devourer::sta::data_hdr_len(fc0, fc1);
  if (len < hlen) { g_rx_short.fetch_add(1); return; }
  if (devourer::sta::is_qos_data(fc0) && (mpdu[24] & 0x80)) {
    g_amsdu_drop.fetch_add(1);
    return;
  }

  const uint8_t* da = devourer::sta::data_da(mpdu, fc1);
  const uint8_t* sa = devourer::sta::data_sa(mpdu, fc1);
  /* THE QoS CONTROL FIELD IS AT A FIXED OFFSET (24), NOT AT hlen - 2: HT
   * Control follows it on a frame with the Order bit set. A 4-address frame
   * cannot reach here - the FromDS/ToDS test above admits from-the-DS
   * frames only. */
  const int tid = devourer::sta::is_qos_data(fc0)
                      ? (mpdu[24] & 0x0f)
                      : devourer::sta::CcmpReplay::kNonQosTid;

  if (!group &&
      g_rx_dup.is_duplicate((fc1 & devourer::sta::kFcRetry) != 0,
                            (uint16_t)(mpdu[22] | (mpdu[23] << 8)), tid)) {
    g_dup_drop.fetch_add(1);
    return;
  }

  if (!(fc1 & devourer::sta::kFcProtected)) {
    /* Plaintext on a WPA2 link is not forwarded: accepting it would let
     * anyone on the channel inject into the host's stack. Cleartext EAPOL is
     * StationSm::on_rx's, and it has already had it. */
    if (g_sm.security() != StationSm::Security::Open) {
      /* Not counted: the four-way's own cleartext EAPOL (expected here),
       * and the no-data subtypes (Null / QoS Null - subtype bit 2), which
       * carry nothing to refuse. */
      const uint8_t* msdu = mpdu + hlen;
      const bool eapol =
          devourer::sta::is_ethertype_snap(msdu, len - hlen) &&
          msdu[6] == 0x88 && msdu[7] == 0x8e;
      const bool no_data = (fc0 & 0x40) != 0;
      if (!eapol && !no_data) g_plain_refused.fetch_add(1);
      return;
    }
    if (to_us) {                        /* the AP holds this association */
      g_judge = false;
      g_strikes = 0;                    /* ...so its BSS backs off no more */
    }
    g_plain_rx.fetch_add(1);
    if (len > hlen) tap_up(da, sa, mpdu + hlen, len - hlen);
    return;
  }
  if (g_sm.security() == StationSm::Security::Open) return;

  /* THE CCMP HEADER AND THE MIC MUST BE THERE BEFORE ANYTHING READS THEM -
   * the key-id read below touches mpdu[hlen + 3]. A frame this short is
   * malformed, not forged, so it is not counted as a MIC failure. */
  if (devourer::sta::ccmp_decrypted_len(len, hlen) == 0) {
    g_rx_short.fetch_add(1);
    return;
  }
  g_enc_rx.fetch_add(1);
  /* Key id 0 is the pairwise key, by the convention every AP follows
   * (hostapd's group key index toggles 1 <-> 2); the frame's own key id
   * chooses, not its address - an AP may unicast under the group key
   * during a rekey. */
  const uint8_t key_id = devourer::sta::ccmp_key_id(mpdu + hlen);
  const devourer::sta::Supplicant& sup = g_sm.supplicant();
  const bool pairwise = key_id == 0;
  const uint8_t* tk = pairwise ? sup.tk() : sup.gtk();
  /* The supplicant installs only a CCMP-length GTK (Supplicant::kGtkLenCcmp). */
  if (!pairwise && (!sup.gtk_valid() || key_id != sup.gtk_key_id())) {
    g_no_key.fetch_add(1);
    return;
  }

  std::vector<uint8_t> plain(devourer::sta::ccmp_decrypted_len(len, hlen));
  size_t plain_len = 0;
  uint64_t pn = 0;
  /* A PAIRWISE REKEY COSTS AT MOST ONE FRAME HERE, by protocol
   * (802.11-2016 12.7.6.5): the supplicant installs the new PTK at message 3,
   * the authenticator only once it has accepted message 4, so for one round
   * trip the AP still transmits under the old key. Not defended against - a
   * grace-period key would be a second key and a second replay window to
   * save one frame. The on-air cell asserts MIC failures <= PTK installs. */
  if (!devourer::sta::ccmp_decrypt(g_crypto, tk, mpdu, len, hlen, mpdu + 10,
                                   plain.data(), plain.size(), &plain_len,
                                   &pn)) {
    g_mic_fail.fetch_add(1);
    return;
  }
  /* THE PN IS ADMITTED ONLY AFTER THE MIC VERIFIED; admitting it first lets
   * anyone on the channel advance the window with garbage. */
  devourer::sta::CcmpReplay& win = pairwise ? g_rx_replay : g_group_replay;
  if (!win.accept(pn, pairwise ? tid : devourer::sta::CcmpReplay::kNonQosTid)) {
    g_replays.fetch_add(1);
    return;
  }
  if (!pairwise) g_group_rx.fetch_add(1);

  /* AN EAPOL-KEY FRAME INSIDE THE CIPHER IS A REKEY, and it is the state
   * machine's, not the host's. One not addressed to us, or not under the
   * PAIRWISE key, is not part of any handshake this station is in: anything
   * on the BSS could have forged it under the group key. It is dropped
   * either way - an EAPOL-Key frame is never the host's. Any OTHER EAPOL
   * packet (EAP, Start, Logoff) is the host's: on_decrypted_msdu returns
   * false for it and it is delivered below. */
  const bool is_eapol_key =
      devourer::sta::is_ethertype_snap(plain.data(), plain_len) &&
      plain[6] == 0x88 && plain[7] == 0x8e &&
      devourer::sta::eapol_is_key(plain.data() + devourer::sta::kLlcSnapLen,
                                  plain_len - devourer::sta::kLlcSnapLen);
  if (is_eapol_key && (!to_us || !pairwise)) return;

  /* THE ANSWER GOES OUT UNDER THE KEY THE REQUEST CAME IN UNDER - for a PTK
   * rekey the OLD pairwise key, since the authenticator switches only once
   * it has accepted message 4. Copied BEFORE on_decrypted_msdu(): `tk` points
   * into the supplicant, and message 3 installs the new key through it. */
  uint8_t tk_in[16];
  if (pairwise) std::memcpy(tk_in, tk, 16);

  std::vector<uint8_t> reply;
  if (to_us && pairwise &&
      g_sm.on_decrypted_msdu(plain.data(), plain_len, now, &reply)) {
    g_eapol_enc_rx.fetch_add(1);
    if (!reply.empty()) {
      std::vector<uint8_t> out;
      devourer::sta::append_llc_snap(out, 0x888e);
      out.insert(out.end(), reply.begin(), reply.end());
      /* Addressed to the BSSID: the AP is both the receiver and the
       * destination of an EAPOL-Key frame. */
      if (air_msdu(out.data(), out.size(), g_sm.bssid(), tk_in,
                   /*from_host=*/false))
        g_eapol_enc_tx.fetch_add(1);
    }
    /* Only now - after the reply was encrypted under the old key - do the PN
     * spaces restart for a newly installed one. */
    note_keys();
    devourer::sta::secure_wipe(tk_in, sizeof tk_in);
    return;
  }
  if (pairwise) devourer::sta::secure_wipe(tk_in, sizeof tk_in);
  tap_up(da, sa, plain.data(), plain_len);
}

/* ---- DOWN: one MSDU onto the air --------------------------------------- */

/* Frame and (on a protected link) encrypt one MSDU for `da`, and queue it.
 * Returns false when the cipher refused. Caller holds g_mu. Shared by the
 * host's traffic and a rekey's answer, so the PN space has one owner. */
bool air_msdu(const uint8_t* msdu, size_t len, const uint8_t da[6],
              const uint8_t* tk, bool from_host) {
  const bool protect = g_sm.security() != StationSm::Security::Open;
  /* Null means "whatever is installed now"; a rekey's answer passes the key
   * its request arrived under. */
  if (!tk) tk = g_sm.supplicant().tk();
  std::vector<uint8_t> hdr = devourer::sta::data_hdr_to_ds(
      g_sm.bssid(), g_own, da, protect, g_data_seq.next());

  if (!protect) {
    hdr.insert(hdr.end(), msdu, msdu + len);
    if (from_host) g_tx_plain.fetch_add(1);
    /* Only a question that was queued: one the full queue dropped never
     * reached the AP, so its missing answer says nothing. */
    const bool question = from_host && g_judge && solicits_reply(msdu, len, da);
    if (enqueue(std::move(hdr)) && question) g_uplink_unconfirmed++;
    return true;
  }
  /* 0 means the length would overflow: refused like any cipher failure. */
  const size_t cap = devourer::sta::ccmp_encrypted_len(hdr.size(), len);
  if (cap == 0) { g_tx_enc_fail.fetch_add(1); return false; }
  std::vector<uint8_t> f(cap);
  const size_t n = devourer::sta::ccmp_encrypt(
      g_crypto, tk, hdr.data(), hdr.size(), g_own,
      g_tx_pn, /*key_id=*/0, msdu, len, f.data(), f.size());
  if (n == 0) { g_tx_enc_fail.fetch_add(1); return false; }
  /* Only after the cipher succeeded: a PN burned on a frame never aired is
   * harmless; a PN reused because a failure skipped the increment is not. */
  g_tx_pn++;
  f.resize(n);
  if (from_host) g_tx_enc.fetch_add(1);
  enqueue(std::move(f));
  return true;
}

/* ---- DOWN: one Ethernet frame from the host ---------------------------- */

/* Outside the reader thread so the headless cells can drive it. */
void tap_down_one(const uint8_t* eth, size_t len) {
  uint8_t msdu[2048], da[6], sa[6];
  /* COUNTED FIRST, so "from host" is every frame the host handed us and the
   * books close. */
  g_tap_rx.fetch_add(1);
  const size_t m = devourer::sta::eth_to_msdu(eth, len, msdu, sizeof msdu, da,
                                              sa);
  if (m == 0) { g_tap_down_drop.fetch_add(1); return; }

  std::lock_guard<std::mutex> l(g_mu);
  if (!g_sm.connected()) { g_tap_down_drop.fetch_add(1); return; }

  /* THE SOURCE ADDRESS MUST BE OURS: the AP matches addr2 against the
   * association it holds. A TAP with the wrong MAC is the usual cause. */
  if (std::memcmp(sa, g_own, 6) != 0) { g_tap_down_drop.fetch_add(1); return; }

  if (!air_msdu(msdu, m, da)) g_tap_down_drop.fetch_add(1);
}

/* ---- the scan and the reconnect policy ---------------------------------- */

/* Which channel the radio should be on while looking for a BSS. */
uint8_t scan_step(uint32_t now) {
  if (g_scan_chans.empty()) return g_chan;
  if (g_scan_chans.size() == 1) return g_scan_chans[0];
  if ((uint32_t)(now - g_scan_switch_ms) >= g_scan_dwell_ms) {
    g_scan_switch_ms = now;
    g_scan_idx = (g_scan_idx + 1) % g_scan_chans.size();
  }
  return g_scan_chans[g_scan_idx];
}

/* A directed probe request for the SSID we want, on the channel we are on:
 * it finds a hidden BSS and shortens the wait on a swept channel. False when
 * none could be built or the full queue dropped it; only a queued one is
 * counted. Caller holds g_mu. */
bool probe(uint8_t chan) {
  std::vector<uint8_t> m =
      devourer::sta::build_probe_req(g_own, g_ssid, chan, chan > 14);
  if (m.empty()) return false;
  devourer::sta::assign_seq(m, g_data_seq.next());
  if (!enqueue(std::move(m))) return false;   /* the full queue dropped it */
  g_probe_tx.fetch_add(1);
  return true;
}

const char* fail_name(StationSm::Failure f);

/* The join and re-join policy. Returns the channel the radio should be tuned
 * to. Caller must NOT hold g_mu. With DEVOURER_STA_RECONNECT=0 the first
 * failure - a lost link or a failed first join - ends the attempts: the run
 * then idles, unassociated, until its time is up. */
uint8_t supervise(uint32_t now) {
  std::lock_guard<std::mutex> l(g_mu);

  /* WPA2: still no EAPOL kNudgeAgainMs after the first nudge - nudge once
   * more (see nudge()). */
  if (g_nudge_again) {
    if (g_sm.state() != StationSm::State::FourWay ||
        g_sm.eapol_rx != g_nudge_eapol_rx) {
      g_nudge_again = false;
    } else if ((uint32_t)(now - g_nudge_ms) >= kNudgeAgainMs) {
      g_nudge_again = false;
      nudge(now);
    }
  }

  /* An unconfirmed open association the host has been talking through
   * (see kConfirmMs) - lost, through the ordinary failure path below. The
   * window opens the first time this pass sees a question. */
  /* Inside a struck BSS's backoff nothing is judged, and its questions are
   * dropped: the window must open at a question asked after it. */
  if (g_judge && (int32_t)(now - g_judge_from_ms) < 0) g_uplink_unconfirmed = 0;
  if (g_judge && g_uplink_unconfirmed > 0 && !g_uplink_seen) {
    g_uplink_seen = true;
    g_uplink_first_ms = now;
  }
  if (g_sm.state() == StationSm::State::Connected && g_judge &&
      g_uplink_seen && g_uplink_unconfirmed >= kConfirmUplink &&
      (uint32_t)(now - g_uplink_first_ms) >= kConfirmMs) {
    char at[32];
    std::fprintf(stderr,
                 "  station association unconfirmed: %u frames sent, no "
                 "unicast reply from the AP in %u ms at=%s\n",
                 g_uplink_unconfirmed, (unsigned)(now - g_uplink_first_ms),
                 wall_stamp(at, sizeof at));
    g_judge = false;
    /* One more consecutive verdict on this BSS: the next association on it
     * waits longer before it is judged. */
    if (g_strikes && std::memcmp(g_strike_bss, g_sm.bssid(), 6) != 0)
      g_strikes = 0;
    g_strikes++;
    std::memcpy(g_strike_bss, g_sm.bssid(), 6);
    g_unconfirmed_lost.fetch_add(1);
    g_sm.link_lost();
  }

  const StationSm::State st = g_sm.state();
  if (st != StationSm::State::Idle && st != StationSm::State::Failed)
    return g_sm.channel() ? g_sm.channel() : g_chan;

  if (g_gave_up) return g_chan;

  /* The transition INTO Failed, handled once. */
  if (st == StationSm::State::Failed && !g_failed_noted) {
    g_failed_noted = true;
    std::fprintf(stderr, "  station %s: %s%s\n",
                 g_was_associated ? "link lost" : "join failed",
                 fail_name(g_sm.fail_reason()),
                 g_reconnect ? "" : " - DEVOURER_STA_RECONNECT=0, not re-joining");
    /* Counted apart from a first join, once per lost link. */
    if (g_was_associated) {
      g_was_associated = false;
      g_reconnects.fetch_add(1);
    }
    if (!g_reconnect) { g_gave_up = true; return g_chan; }
    /* The backoff is for a re-join only; the first attempt does not wait. */
    g_next_join_ms = now + g_rejoin_backoff_ms;
  }
  if (g_next_join_ms && (int32_t)(now - g_next_join_ms) < 0)
    return scan_step(now);

  /* Aged first, so a BSS that went off the air is not joined and reported as
   * "no response". Ten seconds is ~100 beacon intervals. */
  g_bss.expire(now, 10000);

  const bool open = g_sm.security() == StationSm::Security::Open;
  const BssEntry* e = open ? g_bss.select_open(g_ssid) : g_bss.select(g_ssid);
  const uint8_t chan = scan_step(now);
  if (!e) {
    probe(chan);
    g_next_join_ms = now + 200;      /* probe again shortly, do not spin */
    return chan;
  }

  uint8_t snonce[32];
  /* A FRESH SNonce for every attempt: reusing one makes the PTK a function
   * of the ANonce alone. */
  if (!open && RAND_bytes(snonce, sizeof snonce) != 1) {
    /* A fault, not an outcome: the run stops and exits 3. */
    fault("RAND_bytes", "refusing to join with a predictable SNonce");
    g_gave_up = true;
    return g_chan;
  }
  g_join_attempts++;
  g_joins.fetch_add(1);
  g_next_join_ms = 0;
  g_failed_noted = false;
  if (!g_sm.join(*e, open ? nullptr : snonce, now)) {
    /* join() refused the BSS itself (cipher, MFP, no channel, not an ESS...)
     * and will refuse the same entry again: back off rather than spin. */
    g_next_join_ms = now + g_rejoin_backoff_ms;
  }
  return e->info.channel ? e->info.channel : chan;
}

/* ---- TAP ---------------------------------------------------------------- */

int tap_open(const char* name, const uint8_t mac[6]) {
  /* NON-BLOCKING: tap_up() writes from the RX callback under g_mu, and the
   * teardown waits for the RX loop - a blocked write would stall both. */
  int fd = ::open("/dev/net/tun", O_RDWR | O_NONBLOCK);
  if (fd < 0) { perror("  TAP: open /dev/net/tun"); return -1; }
  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof ifr);
  ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
  std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", name);
  if (::ioctl(fd, TUNSETIFF, &ifr) < 0) {
    perror("  TAP: TUNSETIFF (CAP_NET_ADMIN?)");
    ::close(fd);
    return -1;
  }
  /* THE TAP CARRIES THE RADIO'S MAC: every frame the host sends leaves with
   * addr2 = our 802.11 address, and tap_down_one refuses any other source. */
  struct ifreq set;
  std::memset(&set, 0, sizeof set);
  std::snprintf(set.ifr_name, IFNAMSIZ, "%s", ifr.ifr_name);
  set.ifr_hwaddr.sa_family = ARPHRD_ETHER;
  std::memcpy(set.ifr_hwaddr.sa_data, mac, 6);
  int s = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (s >= 0) {
    if (::ioctl(s, SIOCSIFHWADDR, &set) < 0)
      perror("  TAP: SIOCSIFHWADDR");
    ::close(s);
  }
  std::fprintf(stderr,
               "  TAP: %s open with %02x:%02x:%02x:%02x:%02x:%02x - the host "
               "stack owns ARP/ICMP/DHCP\n",
               ifr.ifr_name, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return fd;
}

/* ---- the radio adapter --------------------------------------------------- */

/* THE MPDU'S LENGTH WITHOUT ITS FCS. Realtek delivers the four trailing FCS
 * bytes in Packet::Data (RxAtrib.fcs_present, src/RxPacket.h); MT7612U strips
 * them. Counted in, they move the expected CCMP MIC four bytes late and every
 * protected frame reads as a MIC failure - an attack where there is a length
 * bug. 0 rather than an unsigned underflow for a runt shorter than its FCS. */
size_t mpdu_len(size_t raw, bool fcs_present) {
  if (!fcs_present) return raw;
  return raw >= 4 ? raw - 4 : 0;
}

void on_rx(const Packet& p) {
  const size_t mlen = mpdu_len(p.Data.size(), p.RxAtrib.fcs_present);
  if (p.RxAtrib.crc_err) { g_crc_err.fetch_add(1); return; }
  if (mlen < 24) { g_rx_short.fetch_add(1); return; }
  /* An exception must not unwind into the backend's RX thread (that is
   * std::terminate: no leave, no clear, no ledger). Stop the run instead. */
  try {
    rx_frame(p.Data.data(), mlen, rssi_dbm(p.RxAtrib.rssi[0]), now_ms());
  } catch (const std::exception& e) {
    fault("receive path", e.what());
  } catch (...) {
    fault("receive path", "unknown exception");
  }
}

const char* state_name(StationSm::State s) {
  switch (s) {
    case StationSm::State::Idle: return "Idle";
    case StationSm::State::Authenticating: return "Authenticating";
    case StationSm::State::Associating: return "Associating";
    case StationSm::State::FourWay: return "FourWay";
    case StationSm::State::Connected: return "Connected";
    case StationSm::State::Failed: return "Failed";
  }
  return "?";
}

const char* fail_name(StationSm::Failure f) {
  switch (f) {
    case StationSm::Failure::None: return "none";
    case StationSm::Failure::AuthTimeout: return "auth-timeout";
    case StationSm::Failure::AuthRefused: return "auth-refused";
    case StationSm::Failure::AssocTimeout: return "assoc-timeout";
    case StationSm::Failure::AssocRefused: return "assoc-refused";
    case StationSm::Failure::Deauthenticated: return "deauthenticated";
    case StationSm::Failure::HandshakeTimeout: return "handshake-timeout";
    case StationSm::Failure::BeaconLost: return "beacon-lost";
    case StationSm::Failure::NoPmk: return "no-pmk";
    case StationSm::Failure::NotConfigured: return "not-configured";
    case StationSm::Failure::NoChannel: return "no-channel";
    case StationSm::Failure::NotInfrastructure: return "not-infrastructure";
    case StationSm::Failure::SsidMismatch: return "ssid-mismatch";
    case StationSm::Failure::Unconfirmed: return "unconfirmed";
  }
  return "?";
}

/* THE LEDGER, printed at every exit once `sta_client up:` has printed,
 * whether the run worked or not: "we heard
 * nothing", "we heard the wrong AP" and "we heard our AP and it said no" are
 * different lines here. */
/* THE STATE THE RUN ENDED IN, taken before the teardown's leave() - which
 * always returns the machine to Idle, and would otherwise make every ledger
 * read Idle: a run that gave up (DEVOURER_STA_RECONNECT=0) must say Failed
 * and why. Caller holds g_mu. */
struct RunEnd {
  bool taken = false;
  StationSm::State state = StationSm::State::Idle;
  StationSm::Failure reason = StationSm::Failure::None;
  unsigned status = 0, aid = 0;
  bool keyed = false;
};
RunEnd g_end;
void take_run_end() {
  g_end.taken = true;
  g_end.state = g_sm.state();
  g_end.reason = g_sm.fail_reason();
  g_end.status = g_sm.status();
  g_end.aid = g_sm.aid();
  g_end.keyed = g_sm.keyed();
}

void report() {
  std::lock_guard<std::mutex> l(g_mu);
  if (!g_end.taken) take_run_end();
  std::fprintf(stderr, "fault=%d state=%s", g_fault.load(),
               state_name(g_end.state));
  if (g_end.state == StationSm::State::Failed)
    std::fprintf(stderr, " reason=%s status=%u", fail_name(g_end.reason),
                 g_end.status);
  std::fprintf(stderr, " aid=%u keyed=%d bss_known=%d\n", g_end.aid,
               (int)g_end.keyed, g_bss.count());
  std::fprintf(stderr,
               "  join: beacons observed=%llu, probes sent=%llu (nudges %llu),"
               " joins=%llu, associations=%llu, reconnects=%llu,"
               " unconfirmed=%llu\n",
               (unsigned long long)g_beacons.load(),
               (unsigned long long)g_probe_tx.load(),
               (unsigned long long)g_nudges.load(),
               (unsigned long long)g_joins.load(),
               (unsigned long long)g_associations.load(),
               (unsigned long long)g_reconnects.load(),
               (unsigned long long)g_unconfirmed_lost.load());
  std::fprintf(stderr,
               "  station rx: auth_tx=%u assoc_tx=%u eapol_tx=%u eapol_rx=%u"
               " beacons=%u assoc_repeat=%u\n",
               g_sm.auth_tx, g_sm.assoc_tx, g_sm.eapol_tx, g_sm.eapol_rx,
               g_sm.beacons_rx, g_sm.rx_assoc_repeat);
  std::fprintf(stderr,
               "  refused by the address filter: not-our-bss=%u,"
               " not-for-us=%u, ignored=%u, malformed=%u, tx-dropped=%u"
               " (protected data, handled here: %u)\n",
               g_sm.rx_not_our_bss, g_sm.rx_not_for_us, g_sm.rx_ignored,
               g_sm.rx_malformed, g_sm.tx_dropped, g_sm.rx_protected);
  const devourer::sta::Supplicant& sup = g_sm.supplicant();
  std::fprintf(stderr,
               "  rekeys (EAPOL inside the cipher): received=%llu,"
               " answered=%llu; keys installed: PTK=%llu GTK=%llu\n",
               (unsigned long long)g_eapol_enc_rx.load(),
               (unsigned long long)g_eapol_enc_tx.load(),
               (unsigned long long)g_ptk_installs.load(),
               (unsigned long long)g_gtk_installs.load());
  std::fprintf(stderr,
               "  four-way: mic_failures=%u replays=%u retransmits=%u"
               " malformed=%u out_of_state=%u ignored=%u crypto_errors=%u"
               " rsn_mismatches=%u\n",
               sup.mic_failures, sup.replays, sup.retransmits, sup.malformed,
               sup.out_of_state, sup.ignored, sup.crypto_errors,
               sup.rsn_mismatches);
  std::fprintf(stderr,
               "  data plane: encrypted rx=%llu (group=%llu), plaintext rx="
               "%llu, plaintext refused=%llu, MIC failures=%llu, replays"
               " rejected=%llu, duplicates dropped=%llu, no key for it=%llu\n",
               (unsigned long long)g_enc_rx.load(),
               (unsigned long long)g_group_rx.load(),
               (unsigned long long)g_plain_rx.load(),
               (unsigned long long)g_plain_refused.load(),
               (unsigned long long)g_mic_fail.load(),
               (unsigned long long)g_replays.load(),
               (unsigned long long)g_dup_drop.load(),
               (unsigned long long)g_no_key.load());
  std::fprintf(stderr,
               "  refused before the host: fragmented=%llu, A-MSDU=%llu,"
               " short=%llu, crc_err=%llu\n",
               (unsigned long long)g_frag_drop.load(),
               (unsigned long long)g_amsdu_drop.load(),
               (unsigned long long)g_rx_short.load(),
               (unsigned long long)g_crc_err.load());
  /* The two identities (see the ledger declarations) hold exactly here,
   * because the TAP reader is stopped and the queue drained before this
   * runs. A rekey's encrypted EAPOL answer is in `queued` and `answered`,
   * not in `encrypted`. */
  std::fprintf(stderr,
               "  TAP: to host=%llu, from host=%llu, dropped up=%llu,"
               " dropped down=%llu, read errors=%llu\n",
               (unsigned long long)g_tap_tx.load(),
               (unsigned long long)g_tap_rx.load(),
               (unsigned long long)g_tap_drop.load(),
               (unsigned long long)g_tap_down_drop.load(),
               (unsigned long long)g_tap_read_err.load());
  std::fprintf(stderr,
               "  tx: encrypted=%llu (cipher refused %llu), plaintext=%llu,"
               " queued=%llu, aired=%llu, send failed=%llu, queue dropped=%llu\n",
               (unsigned long long)g_tx_enc.load(),
               (unsigned long long)g_tx_enc_fail.load(),
               (unsigned long long)g_tx_plain.load(),
               (unsigned long long)g_q_in.load(),
               (unsigned long long)g_sent.load(),
               (unsigned long long)g_send_fail.load(),
               (unsigned long long)g_q_drop.load());
}

/* A whole-string decimal integer (surrounding whitespace allowed), or
 * false: std::atoi would read garbage as 0 - a zero-length run, channel 0. */
bool parse_long_strict(const char* s, long* out) {
  if (!s) return false;
  char* end = nullptr;
  errno = 0;
  const long v = std::strtol(s, &end, 10);
  if (end == s || errno == ERANGE) return false;
  while (*end == ' ' || *end == '\t' || *end == '\n') ++end;
  if (*end != '\0') return false;
  *out = v;
  return true;
}

/* The run length (argv[1]): seconds, > 0. */
bool parse_secs(const char* s, int* out) {
  long v = 0;
  if (!parse_long_strict(s, &v) || v <= 0 || v > 86400) return false;
  *out = (int)v;
  return true;
}

/* DEVOURER_CHANNEL: a channel this station can tune (channel_valid). */
bool parse_channel(const char* s, uint8_t* out) {
  long v = 0;
  if (!parse_long_strict(s, &v) || v <= 0 || v > 255 ||
      !devourer::sta::channel_valid((uint8_t)v))
    return false;
  *out = (uint8_t)v;
  return true;
}

/* A millisecond knob from the environment, parsed strictly
 * (devourer_env_long_strict) and bounded to [lo, hi]. Unset keeps *out (the
 * default); set but empty, non-numeric or out of range returns false. */
constexpr long kDwellMinMs = 10, kDwellMaxMs = 10000;
constexpr long kBackoffMinMs = 0, kBackoffMaxMs = 60000;
bool parse_env_ms(const char* name, long lo, long hi, uint32_t* out) {
  if (!std::getenv(name)) return true;
  long v = 0;
  if (!devourer_env_long_strict(name, &v) || v < lo || v > hi) return false;
  *out = (uint32_t)v;
  return true;
}

/* The station's retry limit: DEVOURER_TX_RETRY_LIMIT when the library
 * actually took it - the same strict parse devourer_config_from_env applies,
 * so an empty or non-numeric value is not mistaken for one - else
 * kStationRetryLimit. Returns whether the environment supplied it. */
bool apply_station_retry_limit(devourer::DeviceConfig& cfg) {
  long v = 0;
  if (devourer_env_long_strict("DEVOURER_TX_RETRY_LIMIT", &v)) return true;
  cfg.tx.retry_limit = kStationRetryLimit;
  return false;
}

std::vector<uint8_t> parse_chan_list(const char* s) {
  std::vector<uint8_t> v;
  while (*s) {
    char* end = nullptr;
    const long n = std::strtol(s, &end, 10);
    if (end == s) break;
    if (devourer::sta::channel_valid((uint8_t)(n > 0 && n < 256 ? n : 0)))
      v.push_back((uint8_t)n);
    s = (*end == ',') ? end + 1 : end;
  }
  return v;
}

void send_batch() {
  std::vector<std::vector<uint8_t>> batch;
  { std::lock_guard<std::mutex> l(g_q_mu); batch.swap(g_q); }
  for (auto& f : batch) {
    if (g_dev->send_packet(f.data(), f.size())) g_sent.fetch_add(1);
    else g_send_fail.fetch_add(1);
  }
}

int self_test();

}  // namespace

int main(int argc, char** argv) {
  /* HEADLESS FIRST, before libusb is touched. */
  if (argc > 1 && std::strcmp(argv[1], "--self-test") == 0) return self_test();

  /* FIRST, before the open and the bring-up: a harness may start this
   * process with SIGINT ignored (a background job of a non-interactive
   * shell), and an explicit handler both undoes that and lets a stop during
   * bring-up end the run at the loop instead of being lost. Safe this early:
   * the handler is one atomic store, and nothing reads g_stop before the
   * loop. The device open (up to 15 s waiting for re-enumeration) and the
   * bring-up do not poll it, so a stop there takes effect once they return. */
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  int sec = 60;
  if (argc > 1 && !parse_secs(argv[1], &sec)) {
    std::fprintf(stderr, "usage: sta_client [seconds > 0] | --self-test "
                         "(got '%s')\n", argv[1]);
    return 2;
  }
  if (const char* s = std::getenv("DEVOURER_STA_SSID")) g_ssid = s;
  if (const char* k = std::getenv("DEVOURER_STA_PSK")) g_psk = k;
  if (const char* c = std::getenv("DEVOURER_CHANNEL"); c && !parse_channel(c, &g_chan)) {
    std::fprintf(stderr, "sta_client: DEVOURER_CHANNEL='%s' is not a valid "
                         "channel\n", c);
    return 2;
  }
  if (const char* c = std::getenv("DEVOURER_STA_SCAN_CHANNELS"))
    g_scan_chans = parse_chan_list(c);
  if (g_scan_chans.empty()) g_scan_chans.push_back(g_chan);
  if (!parse_env_ms("DEVOURER_STA_SCAN_DWELL_MS", kDwellMinMs, kDwellMaxMs,
                    &g_scan_dwell_ms)) {
    std::fprintf(stderr, "sta_client: DEVOURER_STA_SCAN_DWELL_MS must be %ld..%ld\n",
                 kDwellMinMs, kDwellMaxMs);
    return 2;
  }
  if (const char* r = std::getenv("DEVOURER_STA_RECONNECT"))
    g_reconnect = std::strcmp(r, "0") != 0;
  if (!parse_env_ms("DEVOURER_STA_BACKOFF_MS", kBackoffMinMs, kBackoffMaxMs,
                    &g_rejoin_backoff_ms)) {
    std::fprintf(stderr, "sta_client: DEVOURER_STA_BACKOFF_MS must be %ld..%ld\n",
                 kBackoffMinMs, kBackoffMaxMs);
    return 2;
  }
  if (const char* a = std::getenv("DEVOURER_STA_ARM"))
    g_arm = std::strcmp(a, "0") != 0;

  devourer::DeviceConfig cfg = devourer_config_from_env();
  const bool limit_from_env = apply_station_retry_limit(cfg);
  /* A station always receives. A backend whose TX bring-up closes the RX
   * path unless asked (Jaguar3's InitWrite) must be asked up front:
   * StartRxLoop after a TX-only bring-up is not a reliable way to get RX. */
  cfg.rx.enable_with_tx = true;

  auto logger = std::make_shared<Logger>();
  apply_logging_env(*logger);
  /* The teardown order (radio, interface, handle, libusb_exit) is held by
   * the demos' RAII session; declared before the RX thread, so it outlives
   * that thread's join. */
  devourer::DeviceSession session{logger};
  libusb_context* ctx = nullptr;
  libusb_init(&ctx);
  session.adopt_context(ctx);
  libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
  /* The MT7612U by default (with DEVOURER_VID=0x0e8d); DEVOURER_VID /
   * DEVOURER_PID name any other adapter, which must then report
   * station_mode_ok (or run with DEVOURER_STA_ARM=0). */
  static const uint16_t pids[] = {0x7612};
  auto* h = open_selected_usb(ctx, logger, pids, 1);
  if (!h) return 1;
  session.adopt_handle(h);
  std::shared_ptr<devourer::UsbDeviceLock> lk;
  if (devourer::claim_interface_then_reset(
          h, devourer::find_wifi_interface(h), logger, true, lk) != 0)
    return 1;
  session.adopt_lock(lk);
  WiFiDriver wifi(logger);
  session.adopt_device(wifi.CreateRadio(h, ctx, lk, cfg));
  g_dev = session.device();
  if (!g_dev) return 1;

  /* Refused before any bring-up: a station on an adapter that cannot arm its
   * identity is not acknowledged by the AP it joins. Capabilities are
   * resolved at construction. */
  const devourer::AdapterCaps caps = g_dev->GetAdapterCaps();
  if (g_arm && !caps.station_mode_ok) {
    std::fprintf(stderr,
                 "sta_client: REFUSED - this adapter's station_mode_ok is "
                 "false (IRadio::SetStationIdentity is not ported or not "
                 "measured on it). DEVOURER_STA_ARM=0 runs it unarmed.\n");
    return 2;
  }

  /* No rate control: the harness's job is to be able to ASK for a rate.
   * Unicast requests an ACK, so the hardware retries it; group-addressed
   * frames stay NOACK - nobody acknowledges them. */
  const char* rate_s = std::getenv("DEVOURER_TX_RATE");
  if (!rate_s || !*rate_s) rate_s = "6M";
  g_rt = devourer::build_stream_radiotap(devourer::parse_tx_mode_str(rate_s));
  if (const char* a = std::getenv("DEVOURER_STA_ACK"); !a || !*a || std::strcmp(a, "0") != 0)
    g_rt_ack = devourer::build_stream_radiotap(devourer::parse_tx_mode_str(rate_s),
                                               /*no_ack=*/false);
  std::fprintf(stderr, "  TX rate: %s, unicast %s, tx.retry_limit %d (%s)\n",
               rate_s,
               g_rt_ack.empty() ? "NOACK (no retries)"
                                : "ACK-requested (hardware retries)",
               cfg.tx.retry_limit,
               limit_from_env ? "DEVOURER_TX_RETRY_LIMIT" : "station default");
  try {
    g_dev->InitWrite(SelectedChannel{g_chan, 0, CHANNEL_WIDTH_20});
  } catch (const std::exception& e) {
    std::fprintf(stderr, "sta_client: bring-up failed: %s\n", e.what());
    return 1;
  } catch (...) {
    std::fprintf(stderr, "sta_client: bring-up failed\n");
    return 1;
  }
  g_tuned.store(g_chan);
  g_retune_ms.store(now_ms());

  /* AFTER InitWrite: there is no device behind the radio until bring-up. A
   * station cannot invent its address (see the top of this file). */
  if (!g_dev->GetPermanentMacAddress(g_own)) {
    std::fprintf(stderr,
                 "sta_client: the radio does not report its MAC address - a "
                 "station cannot invent one\n");
    return 1;
  }

  {
    std::lock_guard<std::mutex> l(g_mu);
    /* set_wanted BEFORE any beacon is folded in, so a flood of fabricated
     * BSSIDs cannot age the genuine AP out of the table. */
    g_bss.set_wanted(g_ssid);
    const bool ok = g_psk.empty()
                        ? g_sm.configure_open(g_ssid, g_own)
                        : g_sm.configure(g_crypto, g_ssid, g_psk.c_str(), g_own);
    if (!ok) {
      std::fprintf(stderr, "sta_client: configure failed\n");
      return 1;
    }
  }

  /* A TAP that was asked for and could not be opened is a refusal, not a
   * TAP-less run. */
  if (const char* t = std::getenv("DEVOURER_STA_TAP")) {
    g_tap_fd = tap_open(t, g_own);
    if (g_tap_fd < 0) {
      std::fprintf(stderr, "sta_client: DEVOURER_STA_TAP=%s could not be "
                           "opened - refusing to run without it\n", t);
      return 1;
    }
  }

  /* A thread that cannot be started is refused before anything is armed
   * or joined: nothing to undo yet. */
  std::thread rx;
  try {
    rx = std::thread([&] {
      try {
        g_dev->StartRxLoop(on_rx);
      } catch (const std::exception& e) {
        fault("RX loop", e.what());
      } catch (...) {
        fault("RX loop", "unknown exception");
      }
    });
  } catch (const std::system_error& e) {
    std::fprintf(stderr, "sta_client: RX thread did not start: %s\n",
                 e.what());
    return 1;
  }

  /* From here on `rx` is running: a TAP reader that cannot be started stops
   * the run through the normal teardown (leave, clear, ledger), never by
   * unwinding past a joinable `rx`. */
  std::thread tap_rd;
  if (g_tap_fd >= 0) try {
    tap_rd = std::thread([&] {
      uint8_t eth[2048];
      const int fd = g_tap_fd;
      for (;;) {
        /* A poll with a timeout and a stop flag: a close() from another
         * thread does not wake a read() already blocked on the fd. */
        pollfd pf{fd, POLLIN, 0};
        const int r = ::poll(&pf, 1, 200);
        if (g_tap_stop.load()) return;
        if (r < 0 && errno == EINTR) continue;
        if (r == 0) continue;
        /* A TAP that errors (or is deleted under us) is a fault, not a
         * quiet end of the reader: the station would look healthy and
         * carry nothing from the host. */
        if (r < 0 || (pf.revents & (POLLERR | POLLHUP | POLLNVAL))) {
          g_tap_read_err.fetch_add(1);
          fault("TAP poll", std::strerror(r < 0 ? errno : EIO));
          return;
        }
        const ssize_t got = ::read(fd, eth, sizeof eth);
        /* The fd is non-blocking: poll() said readable, but a frame can
         * still be gone - nothing to read, wait again. */
        if (got < 0 && (errno == EAGAIN || errno == EWOULDBLOCK ||
                        errno == EINTR))
          continue;
        if (got <= 0) {
          g_tap_read_err.fetch_add(1);
          fault("TAP read", got < 0 ? std::strerror(errno) : "end of file");
          return;
        }
        /* Guarded like on_rx: a throw here is std::terminate otherwise. */
        try {
          tap_down_one(eth, (size_t)got);
        } catch (const std::exception& e) {
          fault("TAP path", e.what());
          return;
        } catch (...) {
          fault("TAP path", "unknown exception");
          return;
        }
      }
    });
  } catch (const std::system_error& e) {
    fault("TAP thread start", e.what());
  }

  std::fprintf(stderr,
               "sta_client up: own %02x:%02x:%02x:%02x:%02x:%02x ssid '%s' "
               "%s ch%u station_mode_ok=%d arm=%d\n",
               g_own[0], g_own[1], g_own[2], g_own[3], g_own[4], g_own[5],
               g_ssid.c_str(), g_psk.empty() ? "OPEN" : "WPA2-PSK", g_chan,
               (int)caps.station_mode_ok, (int)g_arm);

  uint8_t tuned = g_chan;
  /* THE IDENTITY IS ARMED FOR THE BSS ACTUALLY JOINED, once per BSSID, after
   * StartRxLoop (IRadio's ordering rule) - the BSSID is not known before a
   * BSS has been selected. A refused arm is retried a few times, a second
   * apart. */
  uint8_t bssid_armed[6] = {0};
  uint8_t bssid_joined[6] = {0};
  constexpr int kArmTries = 5;
  int arm_tries = 0;
  bool arm_ok = false;
  bool arm_attempted = false;   /* any SetStationIdentity call this run */
  uint32_t next_arm_ms = 0;
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(sec);
  while (!g_stop && std::chrono::steady_clock::now() < end) try {
    const uint32_t now = now_ms();
    const uint8_t want = supervise(now);
    if (want && want != tuned) {
      /* Only ever reached while unassociated: supervise() returns the joined
       * channel once the machine has left Idle/Failed. */
      g_tuned.store(0);              /* unknown until the retune returns */
      g_dev->SetMonitorChannel(SelectedChannel{want, 0, CHANNEL_WIDTH_20});
      tuned = want;
      g_retune_ms.store(now_ms());
      g_tuned.store(want);
    }
    bool arm_now = false;
    bool join_now = false;
    {
      std::lock_guard<std::mutex> l(g_mu);
      g_sm.tick(now);
      if (g_sm.state() != StationSm::State::Idle &&
          std::memcmp(bssid_joined, g_sm.bssid(), 6) != 0) {
        std::memcpy(bssid_joined, g_sm.bssid(), 6);
        join_now = true;
      }
      /* DECIDED under g_mu, MADE below without it. */
      if (g_arm && caps.station_mode_ok &&
          g_sm.state() != StationSm::State::Idle) {
        if (std::memcmp(bssid_armed, g_sm.bssid(), 6) != 0) {
          std::memcpy(bssid_armed, g_sm.bssid(), 6);
          arm_tries = 0;
          arm_ok = false;
          next_arm_ms = now;
        }
        if (!arm_ok && arm_tries < kArmTries &&
            (int32_t)(now - next_arm_ms) >= 0) {
          ++arm_tries;
          next_arm_ms = now + 1000;
          arm_now = true;
        }
      }
      std::vector<uint8_t> f;
      while (g_sm.pop_tx(&f)) {
        devourer::sta::assign_seq(f, g_data_seq.next());
        enqueue(std::move(f));
      }
    }
    if (join_now)
      std::fprintf(stderr,
                   "  station joining BSSID %02x:%02x:%02x:%02x:%02x:%02x\n",
                   bssid_joined[0], bssid_joined[1], bssid_joined[2],
                   bssid_joined[3], bssid_joined[4], bssid_joined[5]);
    /* THE ARM IS MADE OUTSIDE g_mu: it is synchronous USB control I/O, and a
     * libusb synchronous transfer can wait on the thread handling events -
     * the RX thread, inside on_rx, which takes g_mu (IRadio::StartRxLoop's
     * lock rule). Before the send below, so the auth just queued airs armed. */
    if (arm_now) {
      const devourer::MacAddr own{{g_own[0], g_own[1], g_own[2], g_own[3],
                                   g_own[4], g_own[5]}};
      const devourer::MacAddr bss{{bssid_armed[0], bssid_armed[1],
                                   bssid_armed[2], bssid_armed[3],
                                   bssid_armed[4], bssid_armed[5]}};
      arm_attempted = true;
      arm_ok = g_dev->SetStationIdentity(own, bss);
      std::fprintf(stderr,
                   "  station identity %s for BSSID "
                   "%02x:%02x:%02x:%02x:%02x:%02x (attempt %d/%d)\n",
                   arm_ok ? "armed" : "REFUSED", bssid_armed[0],
                   bssid_armed[1], bssid_armed[2], bssid_armed[3],
                   bssid_armed[4], bssid_armed[5], arm_tries, kArmTries);
    }
    send_batch();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  } catch (const std::exception& e) {
    /* Out through the normal exit path below - leave, clear, ledger - and
     * not through std::terminate. */
    fault("main loop", e.what());
  } catch (...) {
    fault("main loop", "unknown exception");
  }

  /* LEAVE CLEANLY: an abandoned association stays alive at the AP until it
   * times the station out, holding an AID. */
  /* Guarded like every teardown step: `tap_rd` and `rx` are still joinable
   * here, so an exception must not leave main. */
  try {
    std::lock_guard<std::mutex> l(g_mu);
    take_run_end();
    g_sm.leave();
    std::vector<uint8_t> f;
    while (g_sm.pop_tx(&f)) {
      devourer::sta::assign_seq(f, g_data_seq.next());
      enqueue(std::move(f));
    }
  } catch (const std::exception& e) {
    fault("leave", e.what());
  } catch (...) {
    fault("leave", "unknown exception");
  }
  /* THE TEARDOWN ORDER. Both producers stop before the last drain - the TAP
   * reader, then the RX thread (a rekey's answer is queued from it) - so
   * nothing is encrypted or queued between the drain and the ledger and its
   * two identities hold exactly. The TAP fd is closed under g_mu, after the
   * RX thread is gone, because tap_up() writes through it from that thread.
   * The final send needs no RX loop: it is the leave's deauth. */
  g_tap_stop.store(true);
  if (tap_rd.joinable()) tap_rd.join();
  /* Guarded: a throw here would leave `rx` joinable, and its destructor
   * would terminate before the clear and the ledger. */
  try {
    g_dev->StopRxLoop();
  } catch (const std::exception& e) {
    fault("StopRxLoop", e.what());
  } catch (...) {
    fault("StopRxLoop", "unknown exception");
  }
  if (rx.joinable()) rx.join();
  {
    std::lock_guard<std::mutex> l(g_mu);
    if (g_tap_fd >= 0) { ::close(g_tap_fd); g_tap_fd = -1; }
  }
  try {
    send_batch();
  } catch (const std::exception& e) {
    fault("final send", e.what());
  } catch (...) {
    fault("final send", "unknown exception");
  }
  /* Cleared on the way out whenever an arm was attempted - every path that
   * can reach SetStationIdentity ends here. The result is the only way to
   * learn a rollback did not land (IRadio: the port may keep answering for
   * `own`; on MT7612U, the managed receive filter may still be in force), so
   * it is printed; on a backend whose arm wrote nothing it is trivially
   * true. */
  if (arm_attempted) {
    const char* r = "NOT VERIFIED";
    try {
      if (g_dev->ClearStationIdentity()) r = "restored (verified)";
      /* The port may still answer for `own`: a fault, not a quiet line. */
      else fault("ClearStationIdentity", "rollback NOT VERIFIED");
    } catch (const std::exception& e) {
      fault("ClearStationIdentity", e.what());
    } catch (...) {
      fault("ClearStationIdentity", "unknown exception");
    }
    std::fprintf(stderr, "  station identity clear: %s\n", r);
  }

  report();
  return exit_status();
}

/* The headless cells. Included rather than linked because everything they
 * drive is in this file's anonymous namespace. */
#include "sta_client_selftest.inc"
