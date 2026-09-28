/* Headless guard for TxMode::no_agg's wire form (RadiotapTxFlags.h): the
 * devourer-private TX_FLAGS bit a backend with AdapterCaps::tx_no_agg_ok reads
 * to keep a frame out of an A-MPDU.
 *
 * For every layout the builder emits (legacy, HT, VHT, HE), with NOACK both
 * ways, walk the header with the shared radiotap iterator and check
 *   - a default TxMode carries no NOAGG bit (existing streams byte-identical),
 *   - "/NOAGG" parses to no_agg, and no_agg sets exactly kRadiotapTxFlagNoAgg,
 *     leaving the length (the Jaguar3 HT/VHT length contract) and every other
 *     bit alone,
 *   - radiotap_tx_no_agg() reads the bit back, and NOACK stays independent.
 */
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "RadiotapBuilder.h"
#include "RadiotapTxFlags.h"
#include "ieee80211_radiotap.h"

static int g_fail = 0;
static void expect(const char *what, bool ok) {
  std::printf("%s %s\n", ok ? "ok  " : "FAIL", what);
  if (!ok)
    ++g_fail;
}

/* The TX_FLAGS value, or -1 if the field is absent. */
static int tx_flags(const std::vector<uint8_t> &rt) {
  auto *hdr = reinterpret_cast<struct ieee80211_radiotap_header *>(
      const_cast<uint8_t *>(rt.data()));
  struct ieee80211_radiotap_iterator it;
  if (ieee80211_radiotap_iterator_init(&it, hdr, (int)rt.size(), nullptr) != 0)
    return -1;
  while (ieee80211_radiotap_iterator_next(&it) == 0)
    if (it.this_arg_index == IEEE80211_RADIOTAP_TX_FLAGS)
      return it.this_arg[0] | (it.this_arg[1] << 8);
  return -1;
}

/* Headers differ only in the NOAGG bit (one byte, that bit). */
static bool only_noagg_differs(const std::vector<uint8_t> &a,
                               const std::vector<uint8_t> &b) {
  if (a.size() != b.size())
    return false;
  int diff = 0;
  for (size_t i = 0; i < a.size(); ++i)
    if (a[i] != b[i]) {
      ++diff;
      if ((a[i] ^ b[i]) != (devourer::kRadiotapTxFlagNoAgg >> 8))
        return false;
    }
  return diff == 1;
}

int main() {
  using devourer::build_stream_radiotap;
  using devourer::parse_tx_mode_str;
  const char *specs[] = {"6M", "MCS7", "MCS0/40/SGI", "VHT1SS_MCS3/80/LDPC",
                         "HE1SS_MCS7/80"};
  char what[160];
  for (const char *spec : specs) {
    const devourer::TxMode plain = parse_tx_mode_str(spec);
    const devourer::TxMode flagged =
        parse_tx_mode_str(std::string(spec) + "/NOAGG");
    std::snprintf(what, sizeof what, "%s: default TxMode has no_agg off", spec);
    expect(what, !plain.no_agg);
    std::snprintf(what, sizeof what, "%s/NOAGG parses to no_agg", spec);
    expect(what, flagged.no_agg);
    for (const bool no_ack : {true, false}) {
      const auto p = build_stream_radiotap(plain, no_ack);
      const auto f = build_stream_radiotap(flagged, no_ack);
      const int fp = tx_flags(p), ff = tx_flags(f);
      std::snprintf(what, sizeof what, "%s no_ack=%d: default header has no NOAGG bit",
                    spec, no_ack);
      expect(what, fp >= 0 && !devourer::radiotap_tx_no_agg((uint16_t)fp));
      std::snprintf(what, sizeof what, "%s no_ack=%d: no_agg sets the NOAGG bit",
                    spec, no_ack);
      expect(what, ff >= 0 && devourer::radiotap_tx_no_agg((uint16_t)ff));
      std::snprintf(what, sizeof what, "%s no_ack=%d: NOACK unchanged by no_agg",
                    spec, no_ack);
      expect(what, ff >= 0 && fp >= 0 &&
                       (ff & IEEE80211_RADIOTAP_F_TX_NOACK) ==
                           (fp & IEEE80211_RADIOTAP_F_TX_NOACK));
      std::snprintf(what, sizeof what,
                    "%s no_ack=%d: no_agg changes only that bit (same length)",
                    spec, no_ack);
      expect(what, only_noagg_differs(p, f));
    }
  }
  if (g_fail) {
    std::printf("%d check(s) failed\n", g_fail);
    return 1;
  }
  std::printf("all NOAGG checks passed\n");
  return 0;
}
