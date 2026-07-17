/* Headless regression guard for the 802.11ax Trigger frame build/parse path
 * (src/TriggerTwt.h build_basic_trigger + src/TriggerParse.h parse_trigger) and
 * the RU-allocation helper. Pure byte-level — no hardware. The on-air half
 * (witness rxdemo decode of an fw-aired trigger vs the commanded config) is
 * tier-A validation (tests/ul_trigger_airs.sh). */
#include <cstdint>
#include <cstdio>
#include <vector>

#include "TriggerParse.h"
#include "TriggerTwt.h"

using namespace devourer;

static int failures = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      failures++;                                                              \
    }                                                                          \
  } while (0)

int main() {
  /* RU-allocation helper golden vectors (802.11ax Trigger User Info subfield). */
  CHECK(he_ru_alloc(242, 0) == 61); /* full 20 MHz */
  CHECK(he_ru_alloc(106, 0) == 53);
  CHECK(he_ru_alloc(52, 0) == 37);
  CHECK(he_ru_alloc(26, 5) == 5);
  CHECK(he_ru_alloc(484, 0) == 65); /* full 40 MHz */
  CHECK(he_ru_alloc(996, 0) == 67); /* full 80 MHz */
  CHECK(he_ru_alloc(999, 0) == 61); /* unknown -> full-20 fallback */

  /* Round-trip a 2-user Basic Trigger through build -> parse. */
  const uint8_t ta[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
  const uint8_t ra[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  TriggerConfig cfg;
  cfg.ul_bw = 0;      /* 20 MHz */
  cfg.gi_ltf = 1;
  cfg.num_he_ltf = 2;
  cfg.ap_tx_power = 21;
  cfg.n_users = 2;
  cfg.users[0].aid12 = 1;
  cfg.users[0].ru_alloc = he_ru_alloc(106, 0); /* 53 */
  cfg.users[0].ul_mcs = 4;
  cfg.users[0].ss = 1;
  cfg.users[0].ldpc = true;
  cfg.users[0].dcm = false;
  cfg.users[0].tgt_rssi_dbm = -60;
  cfg.users[1].aid12 = 2;
  cfg.users[1].ru_alloc = he_ru_alloc(106, 1); /* 54 */
  cfg.users[1].ul_mcs = 7;
  cfg.users[1].ss = 2;
  cfg.users[1].ldpc = false;
  cfg.users[1].dcm = true;
  cfg.users[1].tgt_rssi_dbm = -75;

  std::vector<uint8_t> buf(64, 0);
  size_t n = build_basic_trigger(cfg, ra, ta, buf.data(), buf.size());
  CHECK(n == 24 + 2 * 5); /* fixed header + 2 user infos */
  CHECK(buf[0] == 0x24);  /* FC: control / trigger */

  TriggerInfo info;
  CHECK(parse_trigger(buf.data(), n, info));
  CHECK(info.trigger_type == 0); /* Basic */
  CHECK(info.ul_bw == 0);
  CHECK(info.gi_ltf == 1);
  CHECK(info.num_he_ltf == 2);
  CHECK(info.ap_tx_power == 21);
  for (int i = 0; i < 6; ++i) {
    CHECK(info.ta[i] == ta[i]);
    CHECK(info.ra[i] == ra[i]);
  }
  CHECK(info.n_users == 2);
  CHECK(info.users[0].aid12 == 1);
  CHECK(info.users[0].ru_alloc == 53);
  CHECK(info.users[0].mcs == 4);
  CHECK(info.users[0].ss == 1);
  CHECK(info.users[0].ldpc == true);
  CHECK(info.users[0].dcm == false);
  CHECK(info.users[0].tgt_rssi_dbm == -60);
  CHECK(info.users[1].aid12 == 2);
  CHECK(info.users[1].ru_alloc == 54);
  CHECK(info.users[1].mcs == 7);
  CHECK(info.users[1].ss == 2);
  CHECK(info.users[1].ldpc == false);
  CHECK(info.users[1].dcm == true);
  CHECK(info.users[1].tgt_rssi_dbm == -75);

  /* A capture that still carries the 4-byte FCS tail parses the same (the <5B
   * remainder is dropped). */
  std::vector<uint8_t> withfcs(buf.begin(), buf.begin() + n);
  withfcs.insert(withfcs.end(), {0xde, 0xad, 0xbe, 0xef});
  TriggerInfo info2;
  CHECK(parse_trigger(withfcs.data(), withfcs.size(), info2));
  CHECK(info2.n_users == 2);

  /* Non-trigger / too-short frames are rejected (no partial parse). */
  TriggerInfo info3;
  const uint8_t beacon_fc[30] = {0x80, 0x00}; /* mgmt beacon */
  CHECK(!parse_trigger(beacon_fc, sizeof(beacon_fc), info3));
  CHECK(!parse_trigger(buf.data(), 20, info3));
  CHECK(!parse_trigger(nullptr, n, info3));
  CHECK(!is_trigger_frame(beacon_fc, 2));
  CHECK(is_trigger_frame(buf.data(), n));

  if (failures) {
    std::fprintf(stderr, "trigger_parse: %d FAILURES\n", failures);
    return 1;
  }
  std::printf("trigger_parse: all checks passed\n");
  return 0;
}
