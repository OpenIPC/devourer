/* Headless guard for the radiotap TX_FLAGS NOACK choice
 * (build_stream_radiotap(mode, no_ack)).
 *
 * The flag is the difference between a link that retransmits and one that
 * does not: on the MT7612U, NOACK clears the TXWI ACK request and the
 * hardware retry (MT_TX_RETRY_CFG) never runs (docs/mt7612u-tx-retry.md
 * measures what that costs a unicast link). So: for every
 * layout the builder emits (legacy, HT, VHT, HE), walk the header with the
 * shared radiotap iterator and check
 *   - the one-argument form sets NOACK (the stream default is unchanged),
 *   - no_ack=true is byte-identical to the one-argument form,
 *   - no_ack=false clears NOACK and changes nothing else in the header.
 */
#include <cstdint>
#include <cstdio>
#include <vector>

#include "RadiotapBuilder.h"
#include "RadiotapPeek.h"
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

int main() {
  using devourer::build_stream_radiotap;
  using devourer::parse_tx_mode_str;
  const char *specs[] = {"6M", "MCS7", "MCS0/40/SGI", "VHT1SS_MCS3/80/LDPC",
                         "HE1SS_MCS7/80"};
  for (const char *spec : specs) {
    const devourer::TxMode m = parse_tx_mode_str(spec);
    const auto dflt = build_stream_radiotap(m);
    const auto noack = build_stream_radiotap(m, /*no_ack=*/true);
    const auto ack = build_stream_radiotap(m, /*no_ack=*/false);
    char what[160];
    const int fd = tx_flags(dflt), fa = tx_flags(ack);
    std::snprintf(what, sizeof what, "%s: default header carries TX_FLAGS with NOACK", spec);
    expect(what, fd >= 0 && (fd & IEEE80211_RADIOTAP_F_TX_NOACK));
    std::snprintf(what, sizeof what, "%s: no_ack=true is byte-identical to the default", spec);
    expect(what, noack == dflt);
    std::snprintf(what, sizeof what, "%s: no_ack=false clears NOACK", spec);
    expect(what, fa >= 0 && !(fa & IEEE80211_RADIOTAP_F_TX_NOACK));
    /* Nothing else moves: same length, and only the NOACK bit differs. */
    bool only_flag = ack.size() == dflt.size();
    int diff_bytes = 0;
    for (size_t i = 0; only_flag && i < ack.size(); ++i)
      if (ack[i] != dflt[i]) {
        ++diff_bytes;
        only_flag = (ack[i] ^ dflt[i]) == IEEE80211_RADIOTAP_F_TX_NOACK;
      }
    std::snprintf(what, sizeof what, "%s: no_ack=false changes only the NOACK bit", spec);
    expect(what, only_flag && diff_bytes == 1);
  }
  if (g_fail) {
    std::printf("%d check(s) failed\n", g_fail);
    return 1;
  }
  std::printf("all NOACK checks passed\n");
  return 0;
}
