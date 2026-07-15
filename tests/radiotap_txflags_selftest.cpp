/* Headless wire-contract guard for the radiotap TX-flag path: what
 * build_stream_radiotap() emits must decode back to the TxMode that went in.
 * Guards the J2/J3 regression where HT LDPC/STBC known/flag bits were
 * silently ignored (only the VHT case honoured them). No device is opened. */
#include <cstdio>
#include <cstring>
#include <vector>

#include "RadiotapBuilder.h"
#include "RadiotapTxFlags.h"
#include "ieee80211_radiotap.h"

static int failures = 0;

#define CHECK_EQ(expr, want)                                                   \
  do {                                                                         \
    long got_ = long(expr);                                                    \
    if (got_ != long(want)) {                                                  \
      std::fprintf(stderr, "FAIL %s:%d: %s = %ld, want %ld\n", __FILE__,       \
                   __LINE__, #expr, got_, long(want));                         \
      failures++;                                                              \
    }                                                                          \
  } while (0)

/* Walk a built radiotap header and return the decoded MCS field. */
static devourer::RadiotapMcsField decode_ht(const std::vector<uint8_t> &rt) {
  struct ieee80211_radiotap_iterator it;
  auto *hdr = reinterpret_cast<struct ieee80211_radiotap_header *>(
      const_cast<uint8_t *>(rt.data()));
  devourer::RadiotapMcsField f;
  if (ieee80211_radiotap_iterator_init(&it, hdr, rt.size(), nullptr) != 0) {
    std::fprintf(stderr, "FAIL: iterator init\n");
    failures++;
    return f;
  }
  while (ieee80211_radiotap_iterator_next(&it) == 0) {
    if (it.this_arg_index == IEEE80211_RADIOTAP_MCS)
      f = devourer::decode_radiotap_mcs_field(it.this_arg);
  }
  return f;
}

int main() {
  /* HT MCS0/20, LDPC+STBC (mabur's MAX_RANGE control mode). */
  devourer::TxMode m;
  m.mode = devourer::TxMode::Mode::HT;
  m.ht_mcs = 0;
  m.bw_mhz = 20;
  m.ldpc = true;
  m.stbc = true;
  auto f = decode_ht(devourer::build_stream_radiotap(m));
  CHECK_EQ(f.have_mcs, 1);
  CHECK_EQ(f.mcs, 0);
  CHECK_EQ(f.bw40, 0);
  CHECK_EQ(f.sgi, 0);
  CHECK_EQ(f.ldpc, 1);
  CHECK_EQ(f.stbc, 1);

  /* HT MCS7/40/SGI, no LDPC/STBC. */
  m = devourer::TxMode{};
  m.mode = devourer::TxMode::Mode::HT;
  m.ht_mcs = 7;
  m.bw_mhz = 40;
  m.sgi = true;
  f = decode_ht(devourer::build_stream_radiotap(m));
  CHECK_EQ(f.have_mcs, 1);
  CHECK_EQ(f.mcs, 7);
  CHECK_EQ(f.bw40, 1);
  CHECK_EQ(f.sgi, 1);
  CHECK_EQ(f.ldpc, 0);
  CHECK_EQ(f.stbc, 0);

  /* Raw 3-byte MCS field (bypasses the builder): BW code 20U (not 40 MHz),
   * STBC stream count 2, FEC known + LDPC, GI not short, MCS 5. Pins the
   * decoder's bit positions independently of what build_stream_radiotap can
   * emit. */
  {
    const uint8_t raw[3] = {
        /* known */ IEEE80211_RADIOTAP_MCS_HAVE_BW |
            IEEE80211_RADIOTAP_MCS_HAVE_MCS | IEEE80211_RADIOTAP_MCS_HAVE_FEC |
            IEEE80211_RADIOTAP_MCS_HAVE_STBC,
        /* flags */ IEEE80211_RADIOTAP_MCS_BW_20U |
            IEEE80211_RADIOTAP_MCS_FEC_LDPC |
            (2 << IEEE80211_RADIOTAP_MCS_STBC_SHIFT),
        /* index */ 5};
    const auto g = devourer::decode_radiotap_mcs_field(raw);
    CHECK_EQ(g.have_mcs, 1);
    CHECK_EQ(g.mcs, 5);
    CHECK_EQ(g.bw40, 0);
    CHECK_EQ(g.sgi, 0);
    CHECK_EQ(g.ldpc, 1);
    CHECK_EQ(g.stbc, 2);
  }

  /* VHT wire contract: pin the byte positions the device parsers read
   * (known bit0 = STBC, arg[2] bit0 = STBC flag, arg[8] bit0 = LDPC). */
  m = devourer::TxMode{};
  m.mode = devourer::TxMode::Mode::VHT;
  m.vht_mcs = 3;
  m.vht_nss = 1;
  m.bw_mhz = 80;
  m.ldpc = true;
  m.stbc = true;
  auto rt = devourer::build_stream_radiotap(m);
  /* 22-byte VHT radiotap: header(8) + TX_FLAGS(2) + VHT info(12) at off 10. */
  CHECK_EQ(rt.size(), 22);
  CHECK_EQ(rt[10] & 0x01, 0x01); /* known: STBC        */
  CHECK_EQ(rt[12] & 0x01, 0x01); /* flags: STBC        */
  CHECK_EQ(rt[18] & 0x01, 0x01); /* coding[u0]: LDPC   */
  CHECK_EQ(rt[13], 4);           /* bw code: 80 MHz    */

  if (failures) {
    std::fprintf(stderr, "%d failure(s)\n", failures);
    return 1;
  }
  std::printf("radiotap_txflags: OK\n");
  return 0;
}
