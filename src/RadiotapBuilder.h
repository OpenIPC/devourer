/* Stream-carrier radiotap helper shared between the stream TX binaries.
 *
 * The two stream demos (StreamDuplexDemo, StreamTxDemo) historically each
 * shipped a private `kRadiotapLegacy6M[13]` constant. With the precoder
 * stream link's FEC layers now in master, the carrier MCS/BW is a useful
 * robustness-vs-throughput knob — but the underlying chip's send_packet
 * uses `radiotap_length != 0x0d` (= 13) as the legacy-vs-VHT path
 * selector, so anything that swaps the header has to keep the 13-byte
 * length to stay on the legacy path.
 *
 * This v1 helper switches between the eight legacy OFDM rates (6 / 9 /
 * 12 / 18 / 24 / 36 / 48 / 54 Mbps) by mutating byte 8 (the RATE field)
 * of an otherwise byte-identical 13-byte header. Higher-rate HT-MCS / VHT
 * paths are deliberately out of scope of this first PR — they need PR #88
 * (DEVOURER_TX_HT_MCS) to land first and they ship a non-13-byte radiotap
 * which switches send_packet into its VHT branch. Follow-up PR.
 */

#ifndef RADIOTAP_BUILDER_H
#define RADIOTAP_BUILDER_H

#include <array>
#include <cstdint>

namespace devourer {

/* RATE field convention: u8 in 500 kbps units. 12 = 6 Mbps, 18 = 9 Mbps,
 * 24 = 12 Mbps, 36 = 18 Mbps, 48 = 24 Mbps, 72 = 36 Mbps, 96 = 48 Mbps,
 * 108 = 54 Mbps. Validated against IEEE 802.11 §17.3.4 and the
 * MGN_*M enum positions in src/RadioManagementModule.h. */
constexpr uint8_t kStreamRateDefault500kbps = 12;  // 6 Mbps

/* Build a 13-byte legacy-OFDM radiotap header with the given RATE field
 * (in 500 kbps units). All other bytes are bit-identical to the historic
 * kRadiotapLegacy6M constant: presence = RATE | TX_FLAGS, TX_FLAGS = 8
 * (no-ack). Length stays at 0x0d so send_packet's vht-detection
 * heuristic keeps this on the legacy path. */
std::array<uint8_t, 13> build_legacy_radiotap(uint8_t rate_500kbps);

/* Look up DEVOURER_STREAM_RATE and return the corresponding 500 kbps
 * units value, or kStreamRateDefault500kbps if the env var is unset or
 * unrecognised. Accepted forms: "6M", "9M", "12M", "18M", "24M", "36M",
 * "48M", "54M" (case-insensitive). A bare integer (e.g. "12") is
 * interpreted as already in 500 kbps units. */
uint8_t parse_stream_rate_env();

}  // namespace devourer

#endif  // RADIOTAP_BUILDER_H
