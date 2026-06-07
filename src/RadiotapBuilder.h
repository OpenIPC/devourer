/* Stream-carrier radiotap helper shared between the stream TX binaries.
 *
 * The stream demos historically each shipped a private kRadiotapLegacy6M[13]
 * constant. This helper switches between three carrier modes — Legacy OFDM,
 * HT-MCS, and VHT — under one env var (DEVOURER_STREAM_RATE), so the
 * carrier rate becomes a robustness-vs-throughput knob for the precoder
 * stream link's FEC layers (PR #86 / #87).
 *
 * Three wire-format facts the chip-side send_packet relies on:
 *
 *   - Length 13 (0x0d) → legacy path. radiotap_length != 0x0d sets vht=true
 *     in RtlJaguarDevice::send_packet, switching rate_id to 9.
 *   - The radiotap iterator parses *all* fields regardless of vht — so a
 *     13-byte HT-MCS radiotap (it_present=MCS|TX_FLAGS, no RATE) is
 *     correctly parsed via the IEEE80211_RADIOTAP_MCS case and stays on
 *     rate_id=8. That's what we want for HT-MCS carriers.
 *   - VHT needs a 22-byte radiotap (it_present=VHT|TX_FLAGS). Length>13
 *     puts us on rate_id=9 — that's what the VHT branch requires.
 *
 * For HT-MCS to actually set fixed_rate (vs falling back to MGN_1M CCK at
 * 1 Mbps), the F1 gate DEVOURER_TX_HT_MCS=1 must also be set on the same
 * process. parse_stream_rate_env() logs a warning to stderr if an HT
 * rate is parsed without that gate; VHT has no such gate (send_packet's
 * VHT branch always sets fixed_rate from the VHT info field).
 */

#ifndef RADIOTAP_BUILDER_H
#define RADIOTAP_BUILDER_H

#include <cstdint>
#include <vector>

namespace devourer {

struct StreamRateCfg {
  enum class Mode { Legacy, HT, VHT };
  Mode mode = Mode::Legacy;

  /* Legacy: in 500 kbps units. 12=6M, 18=9M, 24=12M, 36=18M, 48=24M,
   * 72=36M, 96=48M, 108=54M. Default 6M. */
  uint8_t legacy_rate_500kbps = 12;

  /* HT: 0..31 (NSS is implicit — MCS0-7 = 1ss, 8-15 = 2ss, ...). */
  uint8_t ht_mcs = 0;

  /* VHT: per IEEE 802.11ac. mcs 0..9, nss 1..4. */
  uint8_t vht_mcs = 0;
  uint8_t vht_nss = 1;

  /* Bandwidth in MHz. Legacy mode ignores this (always 20 MHz). HT
   * recognises 20 or 40; VHT recognises 20/40/80/160. */
  uint8_t bw_mhz = 20;

  bool sgi = false;
  bool ldpc = false;
  bool stbc = false;
};

/* Build a radiotap header according to cfg.mode. Output is always a
 * complete, well-formed radiotap header — no 802.11 frame body. */
std::vector<uint8_t> build_stream_radiotap(const StreamRateCfg& cfg);

/* Parse DEVOURER_STREAM_RATE / DEVOURER_STREAM_BW / DEVOURER_STREAM_SGI /
 * DEVOURER_STREAM_LDPC / DEVOURER_STREAM_STBC. Unrecognised values fall
 * back to default 6M legacy.
 *
 * DEVOURER_STREAM_RATE grammar (case-insensitive):
 *   - Legacy: 6M / 9M / 12M / 18M / 24M / 36M / 48M / 54M
 *   - HT:     MCS0 .. MCS31
 *   - VHT:    VHT1SS_MCS0 .. VHT4SS_MCS9
 */
StreamRateCfg parse_stream_rate_env();

}  // namespace devourer

#endif  // RADIOTAP_BUILDER_H
