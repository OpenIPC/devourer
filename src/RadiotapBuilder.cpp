#include "RadiotapBuilder.h"

#include "ieee80211_radiotap.h" /* HE field masks */

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace devourer {

namespace {

constexpr uint8_t kRadiotapVersion = 0;

/* it_present bit positions per radiotap spec. */
constexpr uint32_t kPresentRate     = 1u << 2;
constexpr uint32_t kPresentTxFlags  = 1u << 15;
constexpr uint32_t kPresentMcs      = 1u << 19;
constexpr uint32_t kPresentVht      = 1u << 21;
constexpr uint32_t kPresentHe       = 1u << 23;

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

std::vector<uint8_t> build_legacy(const TxMode& cfg) {
  /* 13-byte legacy-OFDM radiotap. Byte 8 (RATE) carried from cfg; length stays
   * 13 so send_packet's vht-detection heuristic keeps this on the legacy path. */
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

std::vector<uint8_t> build_ht(const TxMode& cfg) {
  /* 13-byte HT radiotap: presence = TX_FLAGS | MCS, no RATE field. Length stays
   * 13 so send_packet keeps rate_id=8 (HT, not VHT).
   *
   * MCS field layout (3 bytes after TX_FLAGS): known mask, flags, MCS index. */
  constexpr uint8_t kKnownBw   = 1u << 0;
  constexpr uint8_t kKnownMcs  = 1u << 1;
  constexpr uint8_t kKnownGi   = 1u << 2;
  constexpr uint8_t kKnownFec  = 1u << 4;
  constexpr uint8_t kKnownStbc = 1u << 5;

  uint8_t known = kKnownMcs | kKnownGi | kKnownFec | kKnownBw | kKnownStbc;
  uint8_t flags = 0;
  /* BW: 0=20, 1=40. */
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

std::vector<uint8_t> build_vht(const TxMode& cfg) {
  /* 22-byte VHT radiotap: header(8) + TX_FLAGS(2) + VHT info(12). Length > 13
   * triggers send_packet's vht=true branch (rate_id=9). */
  uint8_t bw_code;
  switch (cfg.bw_mhz) {
    case 40:  bw_code = 1;  break;
    case 80:  bw_code = 4;  break;
    case 160: bw_code = 11; break;
    default:  bw_code = 0;  break;
  }
  /* known mask: STBC(0) | GI(2) | BW(6) — matches send_packet's VHT decoder. */
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

/* Map the chip's 3-bit GI/LTF code (enum rtw_gi_ltf) to the radiotap HE
 * (GI, LTF-size) pair for emission. GI: 0=0.8us 1=1.6us 2=3.2us; LTF: 1=1x
 * 2=2x 3=4x. Inverse of the Kestrel send_packet HE parser. */
void he_giltf_to_radiotap(uint8_t gi_ltf, uint8_t* gi, uint8_t* ltf) {
  switch (gi_ltf) {
    case 0: *gi = 2; *ltf = 3; break; /* 4xLTF + 3.2us */
    case 1: *gi = 0; *ltf = 3; break; /* 4xLTF + 0.8us */
    case 2: *gi = 1; *ltf = 2; break; /* 2xLTF + 1.6us */
    case 4: *gi = 1; *ltf = 1; break; /* 1xLTF + 1.6us */
    case 5: *gi = 0; *ltf = 1; break; /* 1xLTF + 0.8us */
    case 3:
    default: *gi = 0; *ltf = 2; break; /* 2xLTF + 0.8us (HE-SU default) */
  }
}

std::vector<uint8_t> build_he(const TxMode& cfg) {
  /* 22-byte HE radiotap: header(8) + TX_FLAGS(2) + HE info(12, data1..data6).
   * The Kestrel send_packet HE parser reads MCS/coding/STBC (data3), BW+GI+LTF
   * (data5) and NSTS (data6) and maps them to the AX descriptor rate. */
  uint8_t bw_code; /* HE DATA5 BW_RU_ALLOC: 0=20 1=40 2=80 3=160; RU 6=106 7=242 */
  switch (cfg.bw_mhz) {
    case 40:  bw_code = 1; break;
    case 80:  bw_code = 2; break;
    case 160: bw_code = 3; break;
    default:  bw_code = 0; break;
  }
  /* ER SU is a 20 MHz-only format: the BW_RU_ALLOC value names the RU size
   * (242-tone = full-width ER, 106-tone = the upper-bandwidth ~3dB variant)
   * and overrides any configured bw_mhz. */
  if (cfg.he_er == 1) bw_code = IEEE80211_RADIOTAP_HE_DATA5_RU_242;
  else if (cfg.he_er == 2) bw_code = IEEE80211_RADIOTAP_HE_DATA5_RU_106;
  uint8_t gi = 0, ltf = 2;
  he_giltf_to_radiotap(cfg.he_gi_ltf, &gi, &ltf);

  uint16_t data1 =
      IEEE80211_RADIOTAP_HE_DATA1_DATA_MCS_KNOWN |
      IEEE80211_RADIOTAP_HE_DATA1_CODING_KNOWN |
      IEEE80211_RADIOTAP_HE_DATA1_STBC_KNOWN |
      IEEE80211_RADIOTAP_HE_DATA1_BW_RU_ALLOC_KNOWN; /* FORMAT=SU(0) */
  if (cfg.he_er) data1 |= IEEE80211_RADIOTAP_HE_DATA1_FORMAT_EXT_SU;
  if (cfg.he_dcm) data1 |= IEEE80211_RADIOTAP_HE_DATA1_DATA_DCM_KNOWN;
  const uint16_t data2 = IEEE80211_RADIOTAP_HE_DATA2_GI_KNOWN |
                         IEEE80211_RADIOTAP_HE_DATA2_NUM_LTF_SYMS_KNOWN;
  uint16_t data3 =
      static_cast<uint16_t>((cfg.he_mcs & 0x0f) << 8) & /* DATA_MCS bits 8..11 */
      IEEE80211_RADIOTAP_HE_DATA3_DATA_MCS;
  if (cfg.ldpc) data3 |= IEEE80211_RADIOTAP_HE_DATA3_CODING;
  if (cfg.stbc) data3 |= IEEE80211_RADIOTAP_HE_DATA3_STBC;
  if (cfg.he_dcm) data3 |= IEEE80211_RADIOTAP_HE_DATA3_DATA_DCM;
  const uint16_t data5 = static_cast<uint16_t>(
      (bw_code & 0x0f) |
      ((gi & 0x3) << IEEE80211_RADIOTAP_HE_DATA5_GI_SHIFT) |
      ((ltf & 0x3) << IEEE80211_RADIOTAP_HE_DATA5_LTF_SIZE_SHIFT));
  const uint16_t data6 = static_cast<uint16_t>(cfg.he_nss & 0x0f); /* NSTS */

  std::vector<uint8_t> r;
  r.reserve(22);
  emit_u8(r, kRadiotapVersion);
  emit_u8(r, 0);
  emit_u16_le(r, 22);
  emit_u32_le(r, kPresentTxFlags | kPresentHe);
  emit_u16_le(r, kTxFlagsNoAck);
  emit_u16_le(r, data1);
  emit_u16_le(r, data2);
  emit_u16_le(r, data3);
  emit_u16_le(r, 0);      /* data4 */
  emit_u16_le(r, data5);
  emit_u16_le(r, data6);
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

/* Parse the rate mnemonic (first '/'-token) into cfg. Returns false if the
 * token is not a recognised rate. */
bool parse_rate_token(const std::string& s, TxMode* cfg) {
  /* CCK (2.4GHz only — the chip drops CCK at 5GHz, see RtlJaguarDevice). The
   * 500kbps count doubles as the MGN_* byte here too: 1M=2=MGN_1M, 2M=4=MGN_2M,
   * 5.5M=11=MGN_5_5M, 11M=22=MGN_11M. 1M CCK is the most robust rate (~9dB more
   * link budget than 6M OFDM) for a long-range beacon. */
  if (s == "1M")   { cfg->legacy_rate_500kbps = 2;  return true; }
  if (s == "2M")   { cfg->legacy_rate_500kbps = 4;  return true; }
  if (s == "5.5M" || s == "5_5M") { cfg->legacy_rate_500kbps = 11; return true; }
  if (s == "11M")  { cfg->legacy_rate_500kbps = 22; return true; }

  if (s == "6M")  { cfg->legacy_rate_500kbps = 12;  return true; }
  if (s == "9M")  { cfg->legacy_rate_500kbps = 18;  return true; }
  if (s == "12M") { cfg->legacy_rate_500kbps = 24;  return true; }
  if (s == "18M") { cfg->legacy_rate_500kbps = 36;  return true; }
  if (s == "24M") { cfg->legacy_rate_500kbps = 48;  return true; }
  if (s == "36M") { cfg->legacy_rate_500kbps = 72;  return true; }
  if (s == "48M") { cfg->legacy_rate_500kbps = 96;  return true; }
  if (s == "54M") { cfg->legacy_rate_500kbps = 108; return true; }

  /* HT: MCS<N>, 0..31. */
  if (s.rfind("MCS", 0) == 0) {
    unsigned mcs;
    if (parse_uint(s, 3, &mcs) && mcs <= 31) {
      cfg->mode = TxMode::Mode::HT;
      cfg->ht_mcs = static_cast<uint8_t>(mcs);
      return true;
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
          cfg->mode = TxMode::Mode::VHT;
          cfg->vht_nss = static_cast<uint8_t>(nss);
          cfg->vht_mcs = static_cast<uint8_t>(mcs);
          return true;
        }
      }
    }
  }

  /* HE (802.11ax, Kestrel): HE<NSS>SS_MCS<MCS>, NSS 1..4, MCS 0..11. */
  if (s.rfind("HE", 0) == 0) {
    unsigned nss;
    if (parse_uint(s, 2, &nss) && nss >= 1 && nss <= 4) {
      size_t after_nss = 2;
      while (after_nss < s.size() && s[after_nss] >= '0' && s[after_nss] <= '9')
        ++after_nss;
      const std::string tail = "SS_MCS";
      if (s.compare(after_nss, tail.size(), tail) == 0) {
        unsigned mcs;
        if (parse_uint(s, after_nss + tail.size(), &mcs) && mcs <= 11) {
          cfg->mode = TxMode::Mode::HE;
          cfg->he_nss = static_cast<uint8_t>(nss);
          cfg->he_mcs = static_cast<uint8_t>(mcs);
          return true;
        }
      }
    }
  }
  return false;
}

}  // namespace

std::vector<uint8_t> build_stream_radiotap(const TxMode& cfg) {
  switch (cfg.mode) {
    case TxMode::Mode::HT:  return build_ht(cfg);
    case TxMode::Mode::VHT: return build_vht(cfg);
    case TxMode::Mode::HE:  return build_he(cfg);
    case TxMode::Mode::Legacy:
    default:                return build_legacy(cfg);
  }
}

TxMode parse_tx_mode_str(const std::string& spec) {
  TxMode cfg;  /* defaults: legacy 6M, 20 MHz, no SGI/LDPC/STBC */

  if (spec.empty()) {
    return cfg;
  }
  const std::string s = to_upper_stripped(spec.c_str());

  /* Split on '/': first token = rate, rest = bandwidth (numeric) or modifier
   * flags (SGI / LDPC / STBC). */
  std::vector<std::string> tokens;
  size_t start = 0;
  while (start <= s.size()) {
    size_t slash = s.find('/', start);
    if (slash == std::string::npos) {
      tokens.push_back(s.substr(start));
      break;
    }
    tokens.push_back(s.substr(start, slash - start));
    start = slash + 1;
  }
  if (tokens.empty() || tokens[0].empty() || !parse_rate_token(tokens[0], &cfg)) {
    /* free function, no Logger handle — hand-rolled diagnostic-plane line */
    std::fprintf(stderr,
                 "devourer [W] unrecognised TX rate '%s', "
                 "falling back to 6M legacy\n", spec.c_str());
    return TxMode{};
  }

  for (size_t i = 1; i < tokens.size(); ++i) {
    const std::string& t = tokens[i];
    if (t == "SGI")       cfg.sgi = true;
    else if (t == "LDPC") cfg.ldpc = true;
    else if (t == "STBC") cfg.stbc = true;
    else if (t == "ER" || t == "ER106" || t == "DCM") {
      /* HE ER SU / DCM are 802.11ax-only modifiers (Kestrel). */
      if (cfg.mode != TxMode::Mode::HE) {
        std::fprintf(stderr,
                     "devourer [W] ignoring '%s': HE rates only\n", t.c_str());
        continue;
      }
      if (t == "DCM") cfg.he_dcm = true;
      else            cfg.he_er = (t == "ER106") ? 2 : 1;
    }
    else if (t == "20" || t == "40" || t == "80" || t == "160")
      cfg.bw_mhz = static_cast<uint8_t>(std::atoi(t.c_str()));
    else if (!t.empty())
      std::fprintf(stderr,
                   "devourer [W] ignoring unrecognised TX-mode token "
                   "'%s'\n", t.c_str());
  }
  /* ER SU spec limits, applied at parse time so the configured default is
   * honest (the Kestrel send path clamps again defensively): 242-tone = MCS
   * 0-2, 106-tone = MCS 0, both NSS 1, 20 MHz. DCM pairs with MCS 0/1/3/4
   * (0/1 under ER) and excludes STBC. */
  if (cfg.mode == TxMode::Mode::HE && cfg.he_er) {
    const uint8_t mcs_max = (cfg.he_er == 2) ? 0 : 2;
    if (cfg.he_mcs > mcs_max || cfg.he_nss > 1) {
      std::fprintf(stderr,
                   "devourer [W] HE ER SU clamp: nss %u->1, mcs %u->%u\n",
                   cfg.he_nss, cfg.he_mcs,
                   cfg.he_mcs > mcs_max ? mcs_max : cfg.he_mcs);
      cfg.he_nss = 1;
      if (cfg.he_mcs > mcs_max) cfg.he_mcs = mcs_max;
    }
    cfg.bw_mhz = 20;
  }
  if (cfg.mode == TxMode::Mode::HE && cfg.he_dcm) {
    const bool mcs_ok = cfg.he_er ? (cfg.he_mcs <= 1)
                                  : (cfg.he_mcs <= 1 || cfg.he_mcs == 3 ||
                                     cfg.he_mcs == 4);
    if (!mcs_ok) {
      std::fprintf(stderr,
                   "devourer [W] HE DCM dropped: MCS%u not DCM-capable\n",
                   cfg.he_mcs);
      cfg.he_dcm = false;
    } else if (cfg.stbc) {
      std::fprintf(stderr, "devourer [W] HE DCM excludes STBC: dropping STBC\n");
      cfg.stbc = false;
    }
  }
  return cfg;
}

}  // namespace devourer
