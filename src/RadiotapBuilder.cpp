#include "RadiotapBuilder.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace devourer {

namespace {

constexpr uint8_t kRadiotapVersion = 0;

/* it_present bit positions per radiotap spec. */
constexpr uint32_t kPresentRate     = 1u << 2;
constexpr uint32_t kPresentTxFlags  = 1u << 15;
constexpr uint32_t kPresentMcs      = 1u << 19;
constexpr uint32_t kPresentVht      = 1u << 21;

constexpr uint16_t kTxFlagsNoAck = 0x0008;

void emit_u8(std::vector<uint8_t>& v, uint8_t x) { v.push_back(x); }
void emit_u16_le(std::vector<uint8_t>& v, uint16_t x) {
  v.push_back(static_cast<uint8_t>(x & 0xFF));
  v.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
}
void emit_u32_le(std::vector<uint8_t>& v, uint32_t x) {
  v.push_back(static_cast<uint8_t>(x & 0xFF));
  v.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
  v.push_back(static_cast<uint8_t>((x >> 16) & 0xFF));
  v.push_back(static_cast<uint8_t>((x >> 24) & 0xFF));
}

std::vector<uint8_t> build_legacy(const StreamRateCfg& cfg) {
  /* 13-byte legacy-OFDM radiotap. Bit-identical to the historic
   * kRadiotapLegacy6M[13] constant except byte 8 (RATE), which the caller
   * controls. Length stays 13 so send_packet's vht-detection heuristic
   * keeps this on the legacy path. */
  std::vector<uint8_t> r;
  r.reserve(13);
  emit_u8(r, kRadiotapVersion);
  emit_u8(r, 0);                                          /* pad */
  emit_u16_le(r, 13);                                     /* it_len */
  emit_u32_le(r, kPresentRate | kPresentTxFlags);         /* it_present */
  emit_u8(r, cfg.legacy_rate_500kbps);                    /* RATE */
  emit_u8(r, 0);                                          /* pad (TX_FLAGS u16 align) */
  emit_u16_le(r, kTxFlagsNoAck);                          /* TX_FLAGS */
  emit_u8(r, 0);                                          /* trailing pad to 13 */
  return r;
}

std::vector<uint8_t> build_ht(const StreamRateCfg& cfg) {
  /* 13-byte HT radiotap: presence = TX_FLAGS | MCS, no RATE field. Total
   * header length must remain 13 so send_packet stays on rate_id=8 (HT,
   * not VHT). Verified against txdemo/main.cpp's beacon_frame[] HT layout
   * at line 300 — same it_present, same MCS field positions.
   *
   * MCS field layout (3 bytes after TX_FLAGS):
   *   byte 0: known mask
   *   byte 1: flags (BW / SGI / FEC / STBC)
   *   byte 2: MCS index 0..31
   */
  constexpr uint8_t kKnownBw   = 1u << 0;
  constexpr uint8_t kKnownMcs  = 1u << 1;
  constexpr uint8_t kKnownGi   = 1u << 2;
  constexpr uint8_t kKnownFec  = 1u << 4;
  constexpr uint8_t kKnownStbc = 1u << 5;

  uint8_t known = kKnownMcs | kKnownGi | kKnownFec | kKnownBw | kKnownStbc;
  uint8_t flags = 0;
  /* BW: 0=20, 1=40. (20L/20U upper/lower not exposed.) */
  if (cfg.bw_mhz >= 40) flags |= 0x01;
  if (cfg.sgi)          flags |= 0x04;
  if (cfg.ldpc)         flags |= 0x10;
  if (cfg.stbc)         flags |= 0x20;  /* one STBC stream */

  std::vector<uint8_t> r;
  r.reserve(13);
  emit_u8(r, kRadiotapVersion);
  emit_u8(r, 0);
  emit_u16_le(r, 13);
  emit_u32_le(r, kPresentTxFlags | kPresentMcs);
  emit_u16_le(r, kTxFlagsNoAck);
  emit_u8(r, known);
  emit_u8(r, flags);
  emit_u8(r, cfg.ht_mcs <= 31 ? cfg.ht_mcs : 0);
  return r;
}

std::vector<uint8_t> build_vht(const StreamRateCfg& cfg) {
  /* 22-byte VHT radiotap. Identical layout to txdemo/main.cpp's
   * DEVOURER_TX_VHT=1 path (lines 374-409) — header(8) + TX_FLAGS(2) +
   * VHT info(12). Length > 13 triggers send_packet's vht=true branch
   * (rate_id=9), which is required for the VHT info field to be honoured. */
  uint8_t bw_code;
  switch (cfg.bw_mhz) {
    case 40:  bw_code = 1;  break;
    case 80:  bw_code = 4;  break;
    case 160: bw_code = 11; break;
    default:  bw_code = 0;  break;
  }
  /* known mask: STBC(0) | GI(2) | BW(6) — matches send_packet's VHT
   * decoder at RtlJaguarDevice.cpp:110-138. */
  const uint16_t known = (1u << 0) | (1u << 2) | (1u << 6);
  uint8_t vht_flags = 0;
  if (cfg.stbc) vht_flags |= 0x01;
  if (cfg.sgi)  vht_flags |= 0x04;
  const uint8_t mcs_nss_user0 = static_cast<uint8_t>(
      ((cfg.vht_mcs & 0x0F) << 4) | (cfg.vht_nss & 0x0F));
  const uint8_t coding_user0  = cfg.ldpc ? 0x01 : 0x00;

  std::vector<uint8_t> r;
  r.reserve(22);
  emit_u8(r, kRadiotapVersion);
  emit_u8(r, 0);
  emit_u16_le(r, 22);
  emit_u32_le(r, kPresentTxFlags | kPresentVht);
  emit_u16_le(r, kTxFlagsNoAck);
  emit_u16_le(r, known);
  emit_u8(r, vht_flags);
  emit_u8(r, bw_code);
  emit_u8(r, mcs_nss_user0);
  emit_u8(r, 0);              /* user 1 mcs_nss */
  emit_u8(r, 0);              /* user 2 mcs_nss */
  emit_u8(r, 0);              /* user 3 mcs_nss */
  emit_u8(r, coding_user0);
  emit_u8(r, 0);              /* group_id */
  emit_u16_le(r, 0);          /* partial_aid */
  return r;
}

std::string to_upper_stripped(const char* raw) {
  std::string s;
  for (const char* p = raw; *p; ++p) {
    if (*p == ' ' || *p == '\t' || *p == '\n') continue;
    s.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(*p))));
  }
  return s;
}

bool parse_uint(const std::string& s, size_t pos, unsigned* out) {
  if (pos >= s.size()) return false;
  unsigned v = 0;
  size_t i = pos;
  while (i < s.size() && s[i] >= '0' && s[i] <= '9') {
    v = v * 10 + static_cast<unsigned>(s[i] - '0');
    ++i;
    if (v > 1000) return false;
  }
  if (i == pos) return false;
  *out = v;
  return true;
}

}  // namespace

std::vector<uint8_t> build_stream_radiotap(const StreamRateCfg& cfg) {
  switch (cfg.mode) {
    case StreamRateCfg::Mode::HT:  return build_ht(cfg);
    case StreamRateCfg::Mode::VHT: return build_vht(cfg);
    case StreamRateCfg::Mode::Legacy:
    default:                       return build_legacy(cfg);
  }
}

StreamRateCfg parse_stream_rate_env() {
  StreamRateCfg cfg;

  /* Bandwidth (cross-cuts modes). */
  if (const char* bw = std::getenv("DEVOURER_STREAM_BW")) {
    int v = std::atoi(bw);
    if (v == 20 || v == 40 || v == 80 || v == 160) {
      cfg.bw_mhz = static_cast<uint8_t>(v);
    }
  }
  cfg.sgi  = std::getenv("DEVOURER_STREAM_SGI")  != nullptr;
  cfg.ldpc = std::getenv("DEVOURER_STREAM_LDPC") != nullptr;
  cfg.stbc = std::getenv("DEVOURER_STREAM_STBC") != nullptr;

  const char* raw = std::getenv("DEVOURER_STREAM_RATE");
  if (raw == nullptr || *raw == '\0') {
    return cfg;  /* defaults: legacy 6M */
  }
  const std::string s = to_upper_stripped(raw);

  /* Legacy mnemonics. */
  if (s == "6M")  { cfg.legacy_rate_500kbps = 12;  return cfg; }
  if (s == "9M")  { cfg.legacy_rate_500kbps = 18;  return cfg; }
  if (s == "12M") { cfg.legacy_rate_500kbps = 24;  return cfg; }
  if (s == "18M") { cfg.legacy_rate_500kbps = 36;  return cfg; }
  if (s == "24M") { cfg.legacy_rate_500kbps = 48;  return cfg; }
  if (s == "36M") { cfg.legacy_rate_500kbps = 72;  return cfg; }
  if (s == "48M") { cfg.legacy_rate_500kbps = 96;  return cfg; }
  if (s == "54M") { cfg.legacy_rate_500kbps = 108; return cfg; }

  /* HT: MCS<N>, 0..31. */
  if (s.rfind("MCS", 0) == 0) {
    unsigned mcs;
    if (parse_uint(s, 3, &mcs) && mcs <= 31) {
      cfg.mode = StreamRateCfg::Mode::HT;
      cfg.ht_mcs = static_cast<uint8_t>(mcs);
      if (std::getenv("DEVOURER_TX_HT_MCS") == nullptr) {
        std::fprintf(
            stderr,
            "<stream-radiotap>warning: DEVOURER_STREAM_RATE=MCS%u requires "
            "DEVOURER_TX_HT_MCS=1 to actually fly at the requested rate "
            "(otherwise send_packet falls back to 1M CCK)\n",
            mcs);
      }
      return cfg;
    }
  }

  /* VHT: VHT<NSS>SS_MCS<MCS>, NSS 1..4, MCS 0..9. */
  if (s.rfind("VHT", 0) == 0) {
    unsigned nss;
    if (parse_uint(s, 3, &nss) && nss >= 1 && nss <= 4) {
      size_t after_nss = 3;
      while (after_nss < s.size() && s[after_nss] >= '0' && s[after_nss] <= '9')
        ++after_nss;
      const std::string tail = "SS_MCS";
      if (s.compare(after_nss, tail.size(), tail) == 0) {
        unsigned mcs;
        if (parse_uint(s, after_nss + tail.size(), &mcs) && mcs <= 9) {
          cfg.mode = StreamRateCfg::Mode::VHT;
          cfg.vht_nss = static_cast<uint8_t>(nss);
          cfg.vht_mcs = static_cast<uint8_t>(mcs);
          return cfg;
        }
      }
    }
  }

  /* Unrecognised — fall back to default 6M legacy. */
  std::fprintf(stderr,
               "<stream-radiotap>warning: unrecognised DEVOURER_STREAM_RATE=%s,"
               " falling back to 6M legacy\n", raw);
  return cfg;
}

}  // namespace devourer
