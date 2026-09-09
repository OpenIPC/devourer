/* Headless guard for the MT7612U <-> devourer descriptor translations
 * (src/mt7612u/Mt7612uMapping.h).
 *
 * Every one of these is a lookup that is easy to get subtly wrong and
 * impossible to notice on a radio: a wrong RSSI base reads as a weak link, a
 * wrong rate code reads as a slow one, and a per-chain copy that runs off the
 * end of the real chains reads as a third antenna that is always near the
 * noise floor. Each of those has actually happened here, which is why the
 * translations are pure functions in a header with this test under them
 * rather than inline in the device class. */
#include "mt7612u/Mt7612uMapping.h"

#include <cstdio>
#include <cstring>

using namespace mt7612u;

namespace {

int fails;

void expect(const char *what, bool ok) {
  if (!ok) {
    std::fprintf(stderr, "mt7612u_mapping: FAIL %s\n", what);
    fails++;
  }
}

struct mt7612u_rx_info rx(enum mt7612u_phy phy, uint8_t mcs, uint8_t nss = 1) {
  struct mt7612u_rx_info i {};
  i.phy = phy;
  i.mcs = mcs;
  i.nss = nss;
  return i;
}

} // namespace

int main() {
  /* --- RSSI: an unsigned byte biased by 110, not a cast --- */
  expect("rssi -63 dBm -> 47", rssi_to_raw(-63) == 47);
  expect("rssi -110 dBm -> 0", rssi_to_raw(-110) == 0);
  expect("rssi 0 dBm -> 110", rssi_to_raw(0) == 110);
  expect("rssi -128 dBm clamps to 0", rssi_to_raw(-128) == 0);
  expect("rssi +127 dBm clamps to 237", rssi_to_raw(127) == 237);
  /* The bug this constant exists for: a straight cast of -63 gives 193, which
   * LinkHealth reads back as +83 dBm. */
  expect("rssi is biased, not cast",
         rssi_to_raw(-63) != static_cast<uint8_t>(static_cast<int8_t>(-63)));
  expect("rssi round-trips to dBm",
         static_cast<int>(rssi_to_raw(-63)) - 110 == -63);

  /* --- per-chain signal: 2T2R, and rssi[2] is the NOISE FLOOR --- */
  {
    struct mt7612u_rx_info i {};
    struct rx_pkt_attrib a {};

    i.n_chains = 2;
    i.rssi[0] = -55;
    i.rssi[1] = -58;
    i.rssi[2] = -92; /* rx.cpp: info->noise = info->rssi[2] */
    i.rssi[3] = 0x7f;
    i.noise = -92;
    i.snr_db = 37;
    i.noise_valid = 1;
    copy_signal(i, a);

    expect("chain A copied", a.rssi[0] == rssi_to_raw(-55));
    expect("chain B copied", a.rssi[1] == rssi_to_raw(-58));
    /* THE regression this function exists to prevent. Copying all four slots
     * publishes the noise floor as a third antenna — an earlier cut of this
     * integration did exactly that, and every per-chain consumer saw a phantom
     * chain C sitting near -92 dBm. */
    expect("the noise floor is NOT published as chain C", a.rssi[2] == 0);
    expect("the unidentified slot is NOT published as chain D", a.rssi[3] == 0);
    expect("snr filled on the real chains", a.snr[0] == 37 && a.snr[1] == 37);
    expect("snr not invented past the real chains",
           a.snr[2] == 0 && a.snr[3] == 0);
  }
  {
    /* Without a valid noise estimate there is no SNR to report. Zero, not a
     * plausible-looking number — the noise field reads a physically impossible
     * -116 dBm on a quiet channel (see the caveat in the public header). */
    struct mt7612u_rx_info i {};
    struct rx_pkt_attrib a {};

    i.n_chains = 2;
    i.rssi[0] = -55;
    i.snr_db = 37;
    i.noise_valid = 0;
    copy_signal(i, a);
    expect("no snr without a valid noise estimate",
           a.snr[0] == 0 && a.snr[1] == 0);
    expect("rssi still reported without noise", a.rssi[0] == rssi_to_raw(-55));
  }
  {
    /* A 1-chain report must not publish chain B, and a report claiming more
     * chains than the silicon has must not run off the end. */
    struct mt7612u_rx_info i {};
    struct rx_pkt_attrib a {};

    i.n_chains = 1;
    i.rssi[0] = -40;
    i.rssi[1] = -91;
    copy_signal(i, a);
    expect("1 chain publishes only chain A",
           a.rssi[0] == rssi_to_raw(-40) && a.rssi[1] == 0);

    struct rx_pkt_attrib b {};
    i.n_chains = 9;
    copy_signal(i, b);
    expect("an over-large chain count is clamped to the 2T2R truth",
           b.rssi[1] == rssi_to_raw(-91) && b.rssi[2] == 0 && b.rssi[3] == 0);
  }

  /* --- rate: the DESC_RATE numbering every backend reports in --- */
  expect("CCK 1M -> 0", desc_rate(rx(MT7612U_PHY_CCK, 0)) == 0);
  expect("CCK 11M -> 3", desc_rate(rx(MT7612U_PHY_CCK, 3)) == 3);
  expect("OFDM 6M -> DESC_RATE6M", desc_rate(rx(MT7612U_PHY_OFDM, 0)) == 0x04);
  expect("OFDM 54M -> 11", desc_rate(rx(MT7612U_PHY_OFDM, 7)) == 11);
  expect("HT MCS0 -> DESC_RATEMCS0", desc_rate(rx(MT7612U_PHY_HT, 0)) == 0x0c);
  /* MCS7 is 19 — the code the witness logged for our own injected frames, so
   * this one is pinned against a measurement rather than against the header. */
  expect("HT MCS7 -> 19", desc_rate(rx(MT7612U_PHY_HT, 7)) == 19);
  expect("HT MCS15 -> 27", desc_rate(rx(MT7612U_PHY_HT, 15)) == 27);
  expect("HT-GF uses the HT numbering",
         desc_rate(rx(MT7612U_PHY_HT_GF, 7)) == desc_rate(rx(MT7612U_PHY_HT, 7)));
  expect("VHT 1SS MCS0 -> DESC_RATEVHTSS1MCS0",
         desc_rate(rx(MT7612U_PHY_VHT, 0, 1)) == 0x2c);
  expect("VHT 2SS MCS0 -> +10",
         desc_rate(rx(MT7612U_PHY_VHT, 0, 2)) == 0x2c + 10);
  expect("VHT 2SS MCS9 -> +19",
         desc_rate(rx(MT7612U_PHY_VHT, 9, 2)) == 0x2c + 19);
  expect("VHT nss 0 is treated as 1",
         desc_rate(rx(MT7612U_PHY_VHT, 3, 0)) ==
             desc_rate(rx(MT7612U_PHY_VHT, 3, 1)));

  /* --- bandwidth code --- */
  expect("BW_20 -> 0", bw_to_desc(MT7612U_BW_20) == 0);
  expect("BW_40 -> 1", bw_to_desc(MT7612U_BW_40) == 1);
  expect("BW_80 -> 2", bw_to_desc(MT7612U_BW_80) == 2);

  /* --- channel width --- */
  {
    enum mt7612u_bw bw = MT7612U_BW_80;
    const char *why = "";

    expect("20 MHz accepted", width_to_bw(CHANNEL_WIDTH_20, bw, why));
    expect("20 MHz -> BW_20", bw == MT7612U_BW_20);
    expect("40 MHz accepted", width_to_bw(CHANNEL_WIDTH_40, bw, why));
    expect("40 MHz -> BW_40", bw == MT7612U_BW_40);
    expect("80 MHz accepted", width_to_bw(CHANNEL_WIDTH_80, bw, why));
    expect("80 MHz -> BW_80", bw == MT7612U_BW_80);

    /* Refused, not narrowed. A silently narrowed 5 MHz request would air at
     * 20 MHz — four times the occupied bandwidth the caller asked for. */
    why = "";
    expect("5 MHz refused", !width_to_bw(CHANNEL_WIDTH_5, bw, why));
    expect("5 MHz refusal says why", why[0] != '\0');
    why = "";
    expect("10 MHz refused", !width_to_bw(CHANNEL_WIDTH_10, bw, why));
    expect("10 MHz refusal says why", why[0] != '\0');
    why = "";
    expect("160 MHz refused",
           !width_to_bw(static_cast<ChannelWidth_t>(CHANNEL_WIDTH_160), bw, why));
    expect("160 MHz refusal says why", why[0] != '\0');
  }

  if (fails) {
    std::fprintf(stderr, "mt7612u_mapping: %d failure(s)\n", fails);
    return 1;
  }
  std::printf("mt7612u_mapping: all checks passed\n");
  return 0;
}
