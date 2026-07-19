/* ChannelDef — the channel identity the adaptive-channel-migration stack
 * (scout survey → scoring → migration protocol) agrees on.
 *
 * Identity is the exact tuple SetMonitorChannel consumes — (band, primary,
 * width, offset) — because that is what both endpoints must ultimately tune;
 * the center frequency is derived, never stored. Validation here is GRID
 * legality only (a 40 MHz pair must be a real pair, an 80 MHz primary must
 * sit inside a real quad): there is no regulatory database anywhere in
 * devourer — the caller owns compliance, and the no_ir/dfs flags are
 * caller-provided attributes carried through so policy can honor them.
 *
 * Offset semantics match SelectedChannel::ChannelOffset:
 *   40 MHz: 1 = primary is the lower half (secondary above), 2 = upper.
 *   80 MHz: 1..4 = primary position lowest..highest within the quad
 *           (the HALs derive the block center as primary +6/+2/-2/-6).
 * On the 5 GHz grid both are fully determined by the primary, so the parser
 * derives them and an explicit contradicting suffix is an error; a 2.4 GHz
 * 40 MHz pair is genuinely ambiguous (ch 6 pairs either way) and needs the
 * suffix. */
#ifndef DEVOURER_CHANMIG_CHANNEL_DEF_H
#define DEVOURER_CHANMIG_CHANNEL_DEF_H

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "ChannelFreq.h"
#include "SelectedChannel.h"

namespace devourer {
namespace chanmig {

struct ChannelDef {
  uint8_t band = 0;    /* 2 = 2.4 GHz, 5 = 5 GHz */
  uint8_t primary = 0; /* primary 20 MHz channel number */
  ChannelWidth_t width = CHANNEL_WIDTH_20;
  uint8_t offset = 0;  /* SelectedChannel::ChannelOffset semantics (above) */
  /* Caller-provided attributes (not identity): */
  bool no_ir = false;  /* receive-only — never a migration target */
  bool dfs = false;    /* informational */
  bool backup = false; /* preferred backup — higher scout revisit cadence */

  /* Identity key over the RF tuple only (flags are attributes). */
  uint32_t key() const {
    return (uint32_t(band) << 24) | (uint32_t(primary) << 16) |
           (uint32_t(width) << 8) | offset;
  }
  bool same_rf(const ChannelDef &o) const { return key() == o.key(); }

  /* Occupied width in half-MHz units (5 MHz narrowband needs the half). */
  int span_halfmhz() const {
    switch (width) {
    case CHANNEL_WIDTH_5: return 10;
    case CHANNEL_WIDTH_10: return 20;
    case CHANNEL_WIDTH_40: return 80;
    case CHANNEL_WIDTH_80: return 160;
    default: return 40; /* 20 MHz */
    }
  }

  /* Center of the occupied block, half-MHz units (MHz * 2). */
  int center_halfmhz() const {
    const int p = 2 * static_cast<int>(chan_to_freq(primary));
    if (width == CHANNEL_WIDTH_40)
      return p + (offset == 1 ? 20 : -20);
    if (width == CHANNEL_WIDTH_80) {
      /* offset = primary position 1..4 → center at +6/+2/-2/-6 channels. */
      static const int off_ch[4] = {6, 2, -2, -6};
      const int i = (offset >= 1 && offset <= 4) ? offset - 1 : 0;
      return p + 2 * 5 * off_ch[i];
    }
    return p; /* 20 / 5 / 10 MHz: primary is the center */
  }
  uint16_t center_mhz() const {
    return static_cast<uint16_t>(center_halfmhz() / 2);
  }

  SelectedChannel to_selected() const {
    return SelectedChannel{primary, offset, width, 0};
  }

  /* Canonical text: "<band>:<primary>/<width>[u|l]" — "5:104/40l", "5:36/80",
   * "2:6/40u", "5:149/20". The 40 MHz suffix is always emitted (u = secondary
   * above / primary lower, l = secondary below); 80 MHz needs none (position
   * derives from the primary). Buffer must hold >= 16 bytes. */
  void format(char *out, size_t n) const {
    static const char *w[] = {"20", "40", "80", "160", "80+80", "5", "10"};
    const char *ws = (width < CHANNEL_WIDTH_MAX) ? w[width] : "?";
    const char *suf = "";
    if (width == CHANNEL_WIDTH_40)
      suf = (offset == 1) ? "u" : "l";
    std::snprintf(out, n, "%u:%u/%s%s", band, primary, ws, suf);
  }
  std::string str() const {
    char b[20];
    format(b, sizeof(b));
    return b;
  }
};

enum class DefError {
  Ok,
  BadBand,         /* band not 2 or 5 */
  BadPrimary,      /* primary outside the band's channel range */
  BadWidth,        /* width unsupported as a migration candidate (160/80+80) */
  BadOffset,       /* offset contradicts the grid-derived one, or out of range */
  OffGrid40,       /* no legal 40 MHz pair contains this primary */
  OffGrid80,       /* no legal 80 MHz quad contains this primary */
  AmbiguousOffset, /* 2.4 GHz 40 MHz with no explicit u/l */
};

inline const char *def_error_name(DefError e) {
  switch (e) {
  case DefError::Ok: return "ok";
  case DefError::BadBand: return "bad_band";
  case DefError::BadPrimary: return "bad_primary";
  case DefError::BadWidth: return "bad_width";
  case DefError::BadOffset: return "bad_offset";
  case DefError::OffGrid40: return "off_grid_40";
  case DefError::OffGrid80: return "off_grid_80";
  case DefError::AmbiguousOffset: return "ambiguous_offset";
  }
  return "?";
}

/* Validate grid legality and derive a missing offset where the grid fully
 * determines it (5 GHz 40/80). Mutates only the offset field. */
inline DefError normalize(ChannelDef &d) {
  if (d.band != 2 && d.band != 5)
    return DefError::BadBand;
  if (d.band == 2 && (d.primary < 1 || d.primary > 14))
    return DefError::BadPrimary;
  /* 5 GHz: the synth grid runs past the UNII channels (chan_to_freq covers
   * 16..253); grid math below constrains 40/80 further. */
  if (d.band == 5 && (d.primary < 16 || d.primary > 253))
    return DefError::BadPrimary;

  switch (d.width) {
  case CHANNEL_WIDTH_20:
  case CHANNEL_WIDTH_5:
  case CHANNEL_WIDTH_10:
    if (d.offset != 0)
      return DefError::BadOffset;
    return DefError::Ok;

  case CHANNEL_WIDTH_40:
    if (d.band == 2) {
      /* Any ±4 pair inside ch1..13. Ch14 sits 12 MHz above ch13 (not on the
       * 5 MHz grid), so it can be neither primary nor secondary of a pair. */
      if (d.primary == 14)
        return DefError::OffGrid40;
      const bool up_ok = d.primary + 4 <= 13;
      const bool dn_ok = d.primary >= 5;
      if (!up_ok && !dn_ok)
        return DefError::OffGrid40;
      if (d.offset == 0) {
        if (up_ok && dn_ok)
          return DefError::AmbiguousOffset;
        d.offset = up_ok ? 1 : 2;
        return DefError::Ok;
      }
      if (d.offset == 1)
        return up_ok ? DefError::Ok : DefError::OffGrid40;
      if (d.offset == 2)
        return dn_ok ? DefError::Ok : DefError::OffGrid40;
      return DefError::BadOffset;
    }
    {
      /* 5 GHz pairs are grid-fixed. The raster restarts at 149 (5745) — the
       * UNII-3 block is not on the 36-anchored raster. */
      const int anchor = d.primary >= 149 ? 149 : 36;
      const int rel = d.primary - anchor;
      if (rel < 0 || rel % 4 != 0)
        return DefError::OffGrid40;
      const uint8_t derived = (rel % 8 == 0) ? 1 : 2;
      if (d.offset == 0) {
        d.offset = derived;
        return DefError::Ok;
      }
      return d.offset == derived ? DefError::Ok : DefError::BadOffset;
    }

  case CHANNEL_WIDTH_80: {
    if (d.band == 2)
      return DefError::BadWidth;
    const int anchor = d.primary >= 149 ? 149 : 36;
    const int rel = d.primary - anchor;
    if (rel < 0 || rel % 4 != 0)
      return DefError::OffGrid80;
    const uint8_t derived = static_cast<uint8_t>((rel % 16) / 4 + 1);
    if (d.offset == 0) {
      d.offset = derived;
      return DefError::Ok;
    }
    return d.offset == derived ? DefError::Ok : DefError::BadOffset;
  }

  default:
    return DefError::BadWidth;
  }
}

inline DefError validate(const ChannelDef &d) {
  ChannelDef copy = d;
  const DefError e = normalize(copy);
  if (e != DefError::Ok)
    return e;
  /* An unset offset that normalize() had to fill means the caller's def was
   * incomplete but derivable — treat as valid (identical key after fill). */
  return DefError::Ok;
}

/* The 20 MHz constituent bins of the occupied block, ascending. Returns the
 * bin count (1/2/4); out must hold 4. Narrowband occupies within one bin. */
inline int constituent_bins(const ChannelDef &d, uint8_t out[4]) {
  switch (d.width) {
  case CHANNEL_WIDTH_40: {
    const uint8_t lo = (d.offset == 1) ? d.primary
                                       : static_cast<uint8_t>(d.primary - 4);
    out[0] = lo;
    out[1] = static_cast<uint8_t>(lo + 4);
    return 2;
  }
  case CHANNEL_WIDTH_80: {
    const uint8_t pos = (d.offset >= 1 && d.offset <= 4) ? d.offset : 1;
    const uint8_t base = static_cast<uint8_t>(d.primary - 4 * (pos - 1));
    for (int i = 0; i < 4; i++)
      out[i] = static_cast<uint8_t>(base + 4 * i);
    return 4;
  }
  default:
    out[0] = d.primary;
    return 1;
  }
}

/* Span intersection in MHz (0 when disjoint or cross-band). */
inline int overlap_mhz(const ChannelDef &a, const ChannelDef &b) {
  if (a.band != b.band)
    return 0;
  const int alo = a.center_halfmhz() - a.span_halfmhz() / 2;
  const int ahi = a.center_halfmhz() + a.span_halfmhz() / 2;
  const int blo = b.center_halfmhz() - b.span_halfmhz() / 2;
  const int bhi = b.center_halfmhz() + b.span_halfmhz() / 2;
  const int lo = alo > blo ? alo : blo;
  const int hi = ahi < bhi ? ahi : bhi;
  return hi > lo ? (hi - lo) / 2 : 0;
}

/* Disjoint but with an edge gap under 20 MHz — the adjacent-channel-leakage
 * regime a scoring policy penalizes. */
inline bool adjacent(const ChannelDef &a, const ChannelDef &b) {
  if (a.band != b.band || overlap_mhz(a, b) > 0)
    return false;
  const int ahi = a.center_halfmhz() + a.span_halfmhz() / 2;
  const int alo = a.center_halfmhz() - a.span_halfmhz() / 2;
  const int bhi = b.center_halfmhz() + b.span_halfmhz() / 2;
  const int blo = b.center_halfmhz() - b.span_halfmhz() / 2;
  const int gap = (alo >= bhi) ? alo - bhi : blo - ahi;
  return gap >= 0 && gap < 40; /* half-MHz units */
}

/* --- scan-plan grammar ---
 *
 *   plan  = tok[,tok...]
 *   tok   = <primary>[/<width>[u|l]][:<flags>]
 *   width = 5 | 10 | 20 (default) | 40 | 80
 *   u/l   = 40 MHz secondary above / below (required on 2.4 GHz, derived and
 *           checked on 5 GHz)
 *   flags = any of  b (preferred backup)  p (no-IR / receive-only)
 *                   d (DFS, informational)
 *
 *   e.g. "104/40l:b,36/80,132/20:p"
 *
 * Band derives from the primary (<=14 → 2.4 GHz); mixed-band plans are
 * rejected because the scout's cheap retune path is intra-band. Malformed and
 * off-grid tokens are REPORTED, never silently dropped — a silently missing
 * migration candidate is a field hazard. */
struct PlanParseError {
  std::string token;
  std::string reason;
};

inline bool parse_chan_token(const std::string &tok, ChannelDef &out,
                             std::string &err) {
  out = ChannelDef{};
  std::string t = tok;
  /* Optional canonical band prefix ("5:104/40l" — what format() emits), so
   * formatted defs round-trip through the parser. Checked against the band
   * the primary derives. */
  int want_band = 0;
  if (t.size() >= 2 && (t[0] == '2' || t[0] == '5') && t[1] == ':') {
    want_band = t[0] - '0';
    t = t.substr(2);
  }
  /* flags suffix */
  const size_t colon = t.find(':');
  if (colon != std::string::npos) {
    for (size_t i = colon + 1; i < t.size(); ++i) {
      switch (t[i]) {
      case 'b': out.backup = true; break;
      case 'p': out.no_ir = true; break;
      case 'd': out.dfs = true; break;
      default:
        err = "unknown flag";
        return false;
      }
    }
    t = t.substr(0, colon);
  }
  /* width (+ u/l) */
  ChannelWidth_t width = CHANNEL_WIDTH_20;
  uint8_t offset = 0;
  const size_t slash = t.find('/');
  if (slash != std::string::npos) {
    std::string w = t.substr(slash + 1);
    t = t.substr(0, slash);
    if (!w.empty() && (w.back() == 'u' || w.back() == 'l')) {
      offset = (w.back() == 'u') ? 1 : 2;
      w.pop_back();
    }
    if (w == "5") width = CHANNEL_WIDTH_5;
    else if (w == "10") width = CHANNEL_WIDTH_10;
    else if (w == "20") width = CHANNEL_WIDTH_20;
    else if (w == "40") width = CHANNEL_WIDTH_40;
    else if (w == "80") width = CHANNEL_WIDTH_80;
    else {
      err = "bad width";
      return false;
    }
    if (offset != 0 && width != CHANNEL_WIDTH_40) {
      err = "u/l suffix is 40 MHz-only";
      return false;
    }
  }
  char *end = nullptr;
  const long ch = std::strtol(t.c_str(), &end, 10);
  if (t.empty() || end == nullptr || *end != '\0' || ch <= 0 || ch > 253) {
    err = "bad primary channel";
    return false;
  }
  out.primary = static_cast<uint8_t>(ch);
  out.band = (ch <= 14) ? 2 : 5;
  if (want_band != 0 && want_band != out.band) {
    err = "band prefix contradicts the primary";
    return false;
  }
  out.width = width;
  out.offset = offset;
  const DefError e = normalize(out);
  if (e != DefError::Ok) {
    err = def_error_name(e);
    return false;
  }
  return true;
}

inline bool parse_scan_plan(const char *text, std::vector<ChannelDef> &out,
                            std::vector<PlanParseError> &errors) {
  out.clear();
  errors.clear();
  if (text == nullptr || *text == '\0') {
    errors.push_back({"", "empty plan"});
    return false;
  }
  const std::string s = text;
  size_t pos = 0;
  while (pos <= s.size()) {
    const size_t comma = s.find(',', pos);
    const std::string tok = s.substr(
        pos, comma == std::string::npos ? std::string::npos : comma - pos);
    pos = (comma == std::string::npos) ? s.size() + 1 : comma + 1;
    if (tok.empty())
      continue;
    ChannelDef d;
    std::string err;
    if (!parse_chan_token(tok, d, err)) {
      errors.push_back({tok, err});
      continue;
    }
    if (!out.empty() && d.band != out.front().band) {
      errors.push_back({tok, "mixed band (scout plans are single-band)"});
      continue;
    }
    bool dup = false;
    for (const ChannelDef &o : out)
      if (o.same_rf(d)) {
        errors.push_back({tok, "duplicate candidate"});
        dup = true;
        break;
      }
    if (!dup)
      out.push_back(d);
  }
  if (out.empty() && errors.empty())
    errors.push_back({"", "empty plan"});
  return errors.empty() && !out.empty();
}

} /* namespace chanmig */
} /* namespace devourer */

#endif /* DEVOURER_CHANMIG_CHANNEL_DEF_H */
