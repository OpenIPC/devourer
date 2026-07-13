/* Headless round-trip guard for the 802.11ax HE TX radiotap path: parse a
 * DEVOURER_TX_RATE HE spec -> build the radiotap (RadiotapBuilder build_he) ->
 * walk it with the shared radiotap iterator (Radiotap.c) -> decode the HE field
 * and confirm MCS / NSS / BW / coding / GI-LTF and the derived AX descriptor
 * rate (0x180 | (nss-1)<<4 | mcs). Locks the encoding + the Radiotap.c size
 * table entry for field 23 so a regression fails ctest, not only on an AX
 * sniffer. Mirrors the parse in src/kestrel/RtlKestrelDevice.cpp send_packet. */
#include <cstdint>
#include <cstdio>
#include <vector>

#include "RadiotapBuilder.h"
#include "RadiotapPeek.h"      /* wraps ieee80211_radiotap.h in extern "C" */
#include "ieee80211_radiotap.h"

static int g_fail = 0;
static void expect(const char *what, bool ok) {
  if (!ok) { ++g_fail; std::printf("FAIL: %s\n", what); }
}

struct HeDecoded {
  bool found = false;
  unsigned mcs = 0, nss = 0, bw = 0;
  bool ldpc = false, stbc = false;
  unsigned gi = 0, ltf = 0;
  uint16_t ax_rate = 0;
};

static HeDecoded decode_he(const std::vector<uint8_t> &rt) {
  HeDecoded d;
  auto *hdr = reinterpret_cast<struct ieee80211_radiotap_header *>(
      const_cast<uint8_t *>(rt.data()));
  struct ieee80211_radiotap_iterator it;
  if (ieee80211_radiotap_iterator_init(&it, hdr, (int)rt.size(), nullptr) != 0)
    return d;
  while (ieee80211_radiotap_iterator_next(&it) == 0) {
    if (it.this_arg_index != IEEE80211_RADIOTAP_HE)
      continue;
    auto rd16 = [&](int w) -> uint16_t {
      return (uint16_t)(it.this_arg[w * 2] | (it.this_arg[w * 2 + 1] << 8));
    };
    const uint16_t d3 = rd16(2), d5 = rd16(4), d6 = rd16(5);
    d.mcs = (d3 & IEEE80211_RADIOTAP_HE_DATA3_DATA_MCS) >> 8;
    d.nss = d6 & IEEE80211_RADIOTAP_HE_DATA6_NSTS;
    d.ldpc = (d3 & IEEE80211_RADIOTAP_HE_DATA3_CODING) != 0;
    d.stbc = (d3 & IEEE80211_RADIOTAP_HE_DATA3_STBC) != 0;
    d.bw = d5 & IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC;
    d.gi = (d5 & IEEE80211_RADIOTAP_HE_DATA5_GI) >>
           IEEE80211_RADIOTAP_HE_DATA5_GI_SHIFT;
    d.ltf = (d5 & IEEE80211_RADIOTAP_HE_DATA5_LTF_SIZE) >>
            IEEE80211_RADIOTAP_HE_DATA5_LTF_SIZE_SHIFT;
    unsigned nss = d.nss < 1 ? 1 : (d.nss > 4 ? 4 : d.nss);
    d.ax_rate = (uint16_t)(0x180 + (nss - 1) * 16 + d.mcs);
    d.found = true;
  }
  return d;
}

int main() {
  using namespace devourer;

  /* HE1SS_MCS7/80 -> SU, mcs 7, nss 1, 80 MHz, default GI 2xLTF+0.8us. */
  {
    TxMode m = parse_tx_mode_str("HE1SS_MCS7/80");
    expect("HE1SS parsed as HE mode", m.mode == TxMode::Mode::HE);
    expect("HE1SS mcs 7", m.he_mcs == 7);
    expect("HE1SS nss 1", m.he_nss == 1);
    expect("HE1SS bw 80", m.bw_mhz == 80);
    HeDecoded d = decode_he(build_stream_radiotap(m));
    expect("HE1SS field present in radiotap", d.found);
    expect("HE1SS decode mcs 7", d.mcs == 7);
    expect("HE1SS decode nss 1", d.nss == 1);
    expect("HE1SS decode bw=2 (80MHz)", d.bw == 2);
    expect("HE1SS default GI 0.8us", d.gi == 0);
    expect("HE1SS default 2xLTF", d.ltf == 2);
    expect("HE1SS AX rate 0x187", d.ax_rate == 0x187);
  }

  /* HE2SS_MCS11/20/LDPC -> mcs 11, nss 2, 20 MHz, LDPC. */
  {
    TxMode m = parse_tx_mode_str("HE2SS_MCS11/20/LDPC");
    expect("HE2SS parsed as HE mode", m.mode == TxMode::Mode::HE);
    expect("HE2SS mcs 11", m.he_mcs == 11);
    expect("HE2SS nss 2", m.he_nss == 2);
    expect("HE2SS ldpc", m.ldpc);
    HeDecoded d = decode_he(build_stream_radiotap(m));
    expect("HE2SS field present", d.found);
    expect("HE2SS decode mcs 11", d.mcs == 11);
    expect("HE2SS decode nss 2", d.nss == 2);
    expect("HE2SS decode bw=0 (20MHz)", d.bw == 0);
    expect("HE2SS decode ldpc", d.ldpc);
    /* 0x180 + (2-1)*16 + 11 = 0x180 + 16 + 11 = 0x19B */
    expect("HE2SS AX rate 0x19B", d.ax_rate == 0x19B);
  }

  /* An out-of-range MCS (>11) must NOT parse as HE. */
  {
    TxMode m = parse_tx_mode_str("HE1SS_MCS12");
    expect("HE MCS12 rejected -> legacy fallback", m.mode == TxMode::Mode::Legacy);
  }

  if (g_fail) {
    std::printf("he_radiotap_selftest: %d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("he_radiotap_selftest: all checks passed\n");
  return 0;
}
