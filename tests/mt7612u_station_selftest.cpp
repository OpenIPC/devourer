/* Headless guard for the MT7612U station-identity decision and the ownership
 * hand-off (src/mt7612u/StationIdentity.h).
 *
 * In the style of tests/ack_responder_selftest.cpp. The hardware counterpart,
 * `mt7612uprobe staid`, needs a device and root; the logic that can be wrong
 * without the silicon being involved is a policy decision rather than a
 * register write, so all of it is reachable from here. The two cells that
 * matter most:
 *
 *   - BOTH register reads fail CLOSED: a failed MT_AUTO_RSP_CFG read must not
 *     arm a station whose ability to acknowledge is unknown.
 *   - a failed port-identity read is reported as a failed read, not laundered
 *     into a mismatch against 00:00:00:00:00:00.
 *
 * What this does NOT cover: anything about the silicon. Whether the MAC
 * actually receives or acknowledges with a given identity is measured on
 * hardware - docs/mt7612u-station-identity.md, whose retraction section is
 * worth reading before quoting any number from it. */
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "StationIdentity.h"
#include "regs.h"

static int failures = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      failures++;                                                              \
    }                                                                          \
  } while (0)

namespace {

constexpr uint32_t kAutoRspEn = 1u << 0;   /* MT_AUTO_RSP_EN */
/* The monitor filter Mt7612uRadio's RX loop asks for (keep_corrupted off). */
constexpr uint32_t kMonitor = MT_RX_FILTR_CFG_CRC_ERR | MT_RX_FILTR_CFG_PHY_ERR;
constexpr uint32_t kManaged = MT_RX_FILTR_CFG_MANAGED;

const uint8_t kOwn[6]   = {0x40, 0xa5, 0xef, 0x5a, 0x32, 0xf8};
const uint8_t kBssid[6] = {0x02, 0x42, 0x75, 0x05, 0xd6, 0xaa};
const uint8_t kOther[6] = {0x02, 0x00, 0x00, 0xac, 0x1d, 0x01};
const uint8_t kMcast[6] = {0x01, 0x00, 0x5e, 0x00, 0x00, 0x01};
const uint8_t kZero[6]  = {0, 0, 0, 0, 0, 0};

mt7612u_sta_verdict decide(const uint8_t *own, const uint8_t *bssid,
                           const uint8_t *port, int port_ok,
                           uint32_t cfg, int rsp_ok) {
  return mt7612u_sta_decide(own, bssid, port, port_ok, cfg, rsp_ok, kAutoRspEn);
}

void test_the_ordinary_case() {
  CHECK(decide(kOwn, kBssid, kOwn, 1, kAutoRspEn, 1) == MT7612U_STA_OK);
}

void test_malformed_arguments() {
  CHECK(decide(nullptr, kBssid, kOwn, 1, kAutoRspEn, 1) == MT7612U_STA_BAD_ARGS);
  CHECK(decide(kOwn, nullptr, kOwn, 1, kAutoRspEn, 1) == MT7612U_STA_BAD_ARGS);
  CHECK(decide(kMcast, kBssid, kMcast, 1, kAutoRspEn, 1) == MT7612U_STA_MULTICAST);
  CHECK(decide(kOwn, kMcast, kOwn, 1, kAutoRspEn, 1) == MT7612U_STA_MULTICAST);
  CHECK(decide(kOwn, kOwn, kOwn, 1, kAutoRspEn, 1) == MT7612U_STA_SAME_ADDR);

  /* Argument validation must come BEFORE anything that depends on a register
   * read, so a caller mistake is reported as a caller mistake even when the
   * device is unreachable. */
  CHECK(decide(kMcast, kBssid, kOwn, 0, 0, 0) == MT7612U_STA_MULTICAST);
  CHECK(decide(kOwn, kOwn, kOwn, 0, 0, 0) == MT7612U_STA_SAME_ADDR);
}

/* BOTH reads must fail CLOSED. If they disagreed, a failed read on one side
 * would arm. */
void test_failed_reads_refuse() {
  CHECK(decide(kOwn, kBssid, kOwn, 0, kAutoRspEn, 1) == MT7612U_STA_READ_FAILED);
  CHECK(decide(kOwn, kBssid, kOwn, 1, kAutoRspEn, 0) == MT7612U_STA_READ_FAILED);
  CHECK(decide(kOwn, kBssid, kOwn, 0, 0, 0) == MT7612U_STA_READ_FAILED);

  /* And specifically: a failed AUTO_RSP read must NOT be waved through just
   * because the value that came back happens to look armed - a check of the
   * form `read_ok && !(rsp & EN)` skips exactly this case. */
  CHECK(decide(kOwn, kBssid, kOwn, 1, kAutoRspEn, 0) != MT7612U_STA_OK);

  /* A failed port read must not be laundered into a mismatch against the
   * all-zero address that zero-initialised locals would hold. The verdict
   * has to say the read failed. */
  CHECK(decide(kZero, kBssid, kZero, 0, kAutoRspEn, 1) == MT7612U_STA_READ_FAILED);
}

void test_port_identity_must_be_ours() {
  CHECK(decide(kOwn, kBssid, kOther, 1, kAutoRspEn, 1) == MT7612U_STA_PORT_MISMATCH);
  /* The case the hardware gate covers as case 5: a responder holds the port
   * identity, so arming a station on our own address must be refused. */
  CHECK(decide(kOwn, kBssid, kOther, 1, kAutoRspEn, 1) != MT7612U_STA_OK);
  /* And the reverse - asking to be the address the responder moved it to -
   * is refused too, because that address is not this adapter. */
  CHECK(decide(kOther, kBssid, kOwn, 1, kAutoRspEn, 1) == MT7612U_STA_PORT_MISMATCH);
}

void test_auto_response_engine_must_be_on() {
  CHECK(decide(kOwn, kBssid, kOwn, 1, 0, 1) == MT7612U_STA_AUTO_RSP_OFF);
  /* Other bits set but not the enable is still off. */
  CHECK(decide(kOwn, kBssid, kOwn, 1, 0xfffffffeu, 1) == MT7612U_STA_AUTO_RSP_OFF);
  CHECK(decide(kOwn, kBssid, kOwn, 1, 0xffffffffu, 1) == MT7612U_STA_OK);
}

/* --- the ownership hand-off ---------------------------------------------- */
mt7612u_sta_event observe(mt7612u_sta_state *s, const uint8_t *port,
                          int read_ok, int allow_restore) {
  return mt7612u_sta_port_observed(
      s, mt7612u_port_compare(port, read_ok, s->own), allow_restore);
}

void test_port_compare_is_tri_state() {
  CHECK(mt7612u_port_compare(kOwn, 1, kOwn) == MT7612U_PORT_SAME);
  CHECK(mt7612u_port_compare(kOther, 1, kOwn) == MT7612U_PORT_DIFFERENT);
  /* A failed read is UNKNOWN - never DIFFERENT, whatever the buffer holds. */
  CHECK(mt7612u_port_compare(kOther, 0, kOwn) == MT7612U_PORT_UNKNOWN);
  CHECK(mt7612u_port_compare(kZero, 0, kOwn) == MT7612U_PORT_UNKNOWN);
  CHECK(mt7612u_port_compare(nullptr, 1, kOwn) == MT7612U_PORT_UNKNOWN);
}

void test_ownership_handoff() {
  mt7612u_sta_state s{};
  CHECK(s.armed == 0);

  /* Nothing armed: a responder taking the identity is not a loss, and must
   * not produce a diagnostic. */
  CHECK(observe(&s, kOther, 1, 0) == MT7612U_STA_EV_NONE);

  mt7612u_sta_arm(&s, kOwn, kBssid, kMonitor);
  CHECK(s.armed == 1);
  CHECK(std::memcmp(s.bssid, kBssid, 6) == 0);
  CHECK(std::memcmp(s.own, kOwn, 6) == 0);

  /* A write that left the identity where it was - re-armed on the same
   * address, or failed before it moved anything - drops nothing. */
  CHECK(observe(&s, kOwn, 1, 0) == MT7612U_STA_EV_NONE);
  CHECK(s.armed == 1);

  /* A failed read-back is not a move: the arm stands, and the caller is told
   * it is unverified. */
  CHECK(observe(&s, kOther, 0, 0) == MT7612U_STA_EV_UNVERIFIED);
  CHECK(s.armed == 1);

  /* A verified move AFTERWARDS invalidates the station - the ordering the
   * arm-time check cannot help with, and the one a real caller is likelier
   * to hit. */
  CHECK(observe(&s, kOther, 1, 0) == MT7612U_STA_EV_DROPPED);
  CHECK(s.armed == 0);
  CHECK(s.lost == 1);

  /* Idempotent: observing the move twice is one loss, not two. */
  CHECK(observe(&s, kOther, 1, 0) == MT7612U_STA_EV_NONE);

  /* The identity coming back does NOT re-arm by itself (a stopped responder
   * or beacon: the caller re-arms). */
  CHECK(observe(&s, kOwn, 1, 0) == MT7612U_STA_EV_NONE);
  CHECK(s.armed == 0);

  /* Re-arming after the responder gives it back works, and the recorded
   * BSSID is the new one rather than a survivor of the previous arm. */
  mt7612u_sta_arm(&s, kOwn, kOther, kMonitor);
  CHECK(s.armed == 1);
  CHECK(s.lost == 0);
  CHECK(std::memcmp(s.bssid, kOther, 6) == 0);
  mt7612u_sta_clear(&s);
  CHECK(s.armed == 0);
  CHECK(observe(&s, kOther, 1, 0) == MT7612U_STA_EV_NONE);
}

/* A beacon start that moves the identity, then fails and unwinds it back:
 * the arm it dropped is restored - but only when the identity really is
 * back, and a cleared station is never resurrected. */
void test_failed_start_restores_the_arm() {
  mt7612u_sta_state s{};
  mt7612u_sta_arm(&s, kOwn, kBssid, kMonitor);
  CHECK(observe(&s, kOther, 1, 0) == MT7612U_STA_EV_DROPPED);

  /* The unwind did not land (still elsewhere) or cannot be read: no. */
  CHECK(observe(&s, kOther, 1, 1) == MT7612U_STA_EV_NONE);
  CHECK(observe(&s, kOwn, 0, 1) == MT7612U_STA_EV_NONE);
  CHECK(s.armed == 0);

  /* The identity is back: restored, with its own address and BSSID. */
  CHECK(observe(&s, kOwn, 1, 1) == MT7612U_STA_EV_RESTORED);
  CHECK(s.armed == 1);
  CHECK(s.lost == 0);
  CHECK(std::memcmp(s.bssid, kBssid, 6) == 0);

  /* Cleared, then an unwind: nothing to restore. */
  mt7612u_sta_clear(&s);
  CHECK(observe(&s, kOwn, 1, 1) == MT7612U_STA_EV_NONE);
  CHECK(s.armed == 0);
}

/* A clear must not leave the previous BSSID readable: the C API returns it to
 * callers, and a stale value would name a BSS this station is not on. */
void test_clear_wipes_the_bssid() {
  mt7612u_sta_state s{};
  mt7612u_sta_arm(&s, kOwn, kBssid, kMonitor);
  mt7612u_sta_clear(&s);
  CHECK(std::memcmp(s.bssid, kZero, 6) == 0);
  CHECK(std::memcmp(s.own, kZero, 6) == 0);
}

/* Argument validation runs before any register I/O. */
void test_check_args_needs_no_device() {
  CHECK(mt7612u_sta_check_args(kOwn, kBssid) == MT7612U_STA_OK);
  CHECK(mt7612u_sta_check_args(nullptr, kBssid) == MT7612U_STA_BAD_ARGS);
  CHECK(mt7612u_sta_check_args(kMcast, kBssid) == MT7612U_STA_MULTICAST);
  CHECK(mt7612u_sta_check_args(kOwn, kOwn) == MT7612U_STA_SAME_ADDR);
}

/* The managed filter, bit by bit (these are DROP bits). What a station needs
 * to keep hearing must stay clear; what the cells measured must stay set. */
void test_managed_filter_bits() {
  CHECK(kManaged == 0x00015f97u);
  /* kept: every BSS's beacons and group traffic (a re-scan, a re-join),
   * broadcast and multicast, PS-Poll and BAR */
  CHECK(!(kManaged & MT_RX_FILTR_CFG_OTHER_BSS));
  CHECK(!(kManaged & MT_RX_FILTR_CFG_BCAST));
  CHECK(!(kManaged & MT_RX_FILTR_CFG_MCAST));
  CHECK(!(kManaged & MT_RX_FILTR_CFG_PSPOLL));
  CHECK(!(kManaged & MT_RX_FILTR_CFG_BAR));
  /* dropped: unicast not addressed to MT_MAC_ADDR - the bit that makes a
   * moved port identity a deaf station - and the rest */
  CHECK(kManaged & MT_RX_FILTR_CFG_PROMISC);
  CHECK(kManaged & MT_RX_FILTR_CFG_CRC_ERR);
  CHECK(kManaged & MT_RX_FILTR_CFG_DUP);
  CHECK(kManaged & MT_RX_FILTR_CFG_ACK);
  /* and the monitor filter is the other extreme: no address or BSS drop */
  CHECK(!(kMonitor & (MT_RX_FILTR_CFG_PROMISC | MT_RX_FILTR_CFG_OTHER_BSS |
                      MT_RX_FILTR_CFG_DUP)));
}

/* Who owns MT_RX_FILTR_CFG, and what goes back when the station lets go. */
void test_rx_filter_ownership() {
  mt7612u_sta_state s{};

  /* No station: a request is installed as asked, and nothing is recorded. */
  CHECK(mt7612u_sta_rx_filter_request(&s, kMonitor, kManaged) == kMonitor);
  CHECK(s.rx_filtr_restore == 0);

  /* The arm records what the register held. */
  mt7612u_sta_arm(&s, kOwn, kBssid, kMonitor);
  CHECK(s.rx_filtr_restore == kMonitor);

  /* A RE-arm reads the managed filter the first arm installed; recording
   * THAT would leave the receiver managed after the clear. */
  mt7612u_sta_arm(&s, kOwn, kOther, kManaged);
  CHECK(s.rx_filtr_restore == kMonitor);

  /* A receiver restarted under the arm asks for the monitor filter again
   * (keep_corrupted on this time): the managed filter stays, and the new
   * request is what the clear will put back. */
  const uint32_t keep = MT_RX_FILTR_CFG_PHY_ERR;
  CHECK(mt7612u_sta_rx_filter_request(&s, keep, kManaged) == kManaged);
  CHECK(s.rx_filtr_restore == keep);

  /* Dropped by a port move: requests are installed again, and still
   * recorded, because a restore re-arms and its clear must put back the
   * latest. */
  CHECK(mt7612u_sta_port_observed(&s, MT7612U_PORT_DIFFERENT, 0) ==
        MT7612U_STA_EV_DROPPED);
  CHECK(mt7612u_sta_rx_filter_request(&s, kMonitor, kManaged) == kMonitor);
  CHECK(s.rx_filtr_restore == kMonitor);
  CHECK(mt7612u_sta_port_observed(&s, MT7612U_PORT_SAME, 1) ==
        MT7612U_STA_EV_RESTORED);
  CHECK(mt7612u_sta_rx_filter_request(&s, kMonitor, kManaged) == kManaged);

  /* Cleared: back to installing requests as asked. */
  mt7612u_sta_clear(&s);
  CHECK(mt7612u_sta_rx_filter_request(&s, kMonitor, kManaged) == kMonitor);
  CHECK(s.rx_filtr_restore == 0);

  /* An arm after a clear records afresh. */
  mt7612u_sta_arm(&s, kOwn, kBssid, keep);
  CHECK(s.rx_filtr_restore == keep);
}

} // namespace

int main() {
  test_the_ordinary_case();
  test_malformed_arguments();
  test_failed_reads_refuse();
  test_port_identity_must_be_ours();
  test_auto_response_engine_must_be_on();
  test_port_compare_is_tri_state();
  test_ownership_handoff();
  test_failed_start_restores_the_arm();
  test_clear_wipes_the_bssid();
  test_check_args_needs_no_device();
  test_managed_filter_bits();
  test_rx_filter_ownership();

  if (failures) {
    std::fprintf(stderr, "mt7612u_station_selftest: %d failure(s)\n", failures);
    return 1;
  }
  std::printf("mt7612u_station_selftest: all checks passed\n");
  return 0;
}
