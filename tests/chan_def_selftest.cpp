/* ChannelDef grid-legality + geometry + scan-plan grammar selftest. */
#include "chanmig/ChannelDef.h"

#include <cstdio>
#include <cstring>

static int fails;
#define CHECK(x, msg)                                                          \
  do {                                                                         \
    if (!(x)) {                                                                \
      std::fprintf(stderr, "FAIL: %s\n", msg);                                 \
      ++fails;                                                                 \
    }                                                                          \
  } while (0)

using devourer::chanmig::adjacent;
using devourer::chanmig::ChannelDef;
using devourer::chanmig::constituent_bins;
using devourer::chanmig::DefError;
using devourer::chanmig::normalize;
using devourer::chanmig::overlap_mhz;
using devourer::chanmig::parse_chan_token;
using devourer::chanmig::parse_scan_plan;
using devourer::chanmig::PlanParseError;

static ChannelDef mk(uint8_t band, uint8_t primary, ChannelWidth_t w,
                     uint8_t off = 0) {
  ChannelDef d;
  d.band = band;
  d.primary = primary;
  d.width = w;
  d.offset = off;
  return d;
}

static DefError norm(ChannelDef &d) { return normalize(d); }

int main() {
  /* --- 5 GHz 40 MHz pair grid: offset derives from the primary --- */
  {
    struct Row {
      uint8_t primary;
      DefError want;
      uint8_t off;
    } rows[] = {
        {36, DefError::Ok, 1},  {40, DefError::Ok, 2},
        {44, DefError::Ok, 1},  {48, DefError::Ok, 2},
        {100, DefError::Ok, 1}, {104, DefError::Ok, 2},
        {149, DefError::Ok, 1}, {153, DefError::Ok, 2},
        {157, DefError::Ok, 1}, {161, DefError::Ok, 2},
        {165, DefError::Ok, 1}, {38, DefError::OffGrid40, 0},
        {34, DefError::OffGrid40, 0},
    };
    for (const Row &r : rows) {
      ChannelDef d = mk(5, r.primary, CHANNEL_WIDTH_40);
      const DefError e = norm(d);
      CHECK(e == r.want, "5 GHz 40 MHz grid verdict");
      if (r.want == DefError::Ok)
        CHECK(d.offset == r.off, "derived 40 MHz offset");
    }
    ChannelDef bad = mk(5, 36, CHANNEL_WIDTH_40, 2); /* 36 is a lower half */
    CHECK(norm(bad) == DefError::BadOffset, "contradicting 40 suffix");
    ChannelDef ok = mk(5, 36, CHANNEL_WIDTH_40, 1);
    CHECK(norm(ok) == DefError::Ok, "consistent explicit 40 suffix");
  }

  /* --- 5 GHz 80 MHz quads: position 1..4 derives from the primary --- */
  {
    struct Row {
      uint8_t primary;
      uint8_t pos;
    } rows[] = {{36, 1},  {40, 2},  {44, 3},  {48, 4},  {52, 1},  {64, 4},
                {100, 1}, {112, 4}, {149, 1}, {153, 2}, {161, 4}};
    for (const Row &r : rows) {
      ChannelDef d = mk(5, r.primary, CHANNEL_WIDTH_80);
      CHECK(norm(d) == DefError::Ok, "80 MHz quad verdict");
      CHECK(d.offset == r.pos, "derived 80 MHz position");
    }
    ChannelDef offgrid = mk(5, 38, CHANNEL_WIDTH_80);
    CHECK(norm(offgrid) == DefError::OffGrid80, "80 MHz off-grid");
    ChannelDef b2 = mk(2, 6, CHANNEL_WIDTH_80);
    CHECK(norm(b2) == DefError::BadWidth, "no 80 MHz on 2.4");
    ChannelDef wrongpos = mk(5, 40, CHANNEL_WIDTH_80, 1);
    CHECK(norm(wrongpos) == DefError::BadOffset, "80 MHz position mismatch");
  }

  /* --- 2.4 GHz 40 MHz: genuinely ambiguous without a suffix --- */
  {
    ChannelDef amb = mk(2, 6, CHANNEL_WIDTH_40);
    CHECK(norm(amb) == DefError::AmbiguousOffset, "ch6/40 ambiguous");
    ChannelDef lo = mk(2, 1, CHANNEL_WIDTH_40);
    CHECK(norm(lo) == DefError::Ok && lo.offset == 1, "ch1/40 derives up");
    ChannelDef hi = mk(2, 13, CHANNEL_WIDTH_40);
    CHECK(norm(hi) == DefError::Ok && hi.offset == 2, "ch13/40 derives down");
    ChannelDef up = mk(2, 6, CHANNEL_WIDTH_40, 1);
    CHECK(norm(up) == DefError::Ok, "ch6/40u explicit");
    ChannelDef bad = mk(2, 10, CHANNEL_WIDTH_40, 1); /* 10+4 = the CCK island */
    CHECK(norm(bad) == DefError::OffGrid40, "secondary on ch14 rejected");
    ChannelDef c14 = mk(2, 14, CHANNEL_WIDTH_40);
    CHECK(norm(c14) == DefError::OffGrid40, "ch14/40 rejected");
  }

  /* --- 20 / narrowband widths --- */
  {
    ChannelDef bad = mk(5, 36, CHANNEL_WIDTH_20, 1);
    CHECK(norm(bad) == DefError::BadOffset, "20 MHz takes no offset");
    ChannelDef nb = mk(5, 149, CHANNEL_WIDTH_5);
    CHECK(norm(nb) == DefError::Ok, "5 MHz narrowband candidate");
    ChannelDef badband = mk(3, 36, CHANNEL_WIDTH_20);
    CHECK(norm(badband) == DefError::BadBand, "band must be 2 or 5");
    ChannelDef badprim = mk(2, 15, CHANNEL_WIDTH_20);
    CHECK(norm(badprim) == DefError::BadPrimary, "2.4 primary range");
    ChannelDef w160 = mk(5, 36, CHANNEL_WIDTH_160);
    CHECK(norm(w160) == DefError::BadWidth, "160 not a candidate width");
  }

  /* --- identity key: RF tuple only, flags excluded --- */
  {
    ChannelDef a = mk(5, 104, CHANNEL_WIDTH_40);
    norm(a);
    ChannelDef b = a;
    b.backup = true;
    b.no_ir = true;
    CHECK(a.key() == b.key() && a.same_rf(b), "flags are not identity");
    ChannelDef c = mk(5, 104, CHANNEL_WIDTH_20);
    CHECK(a.key() != c.key(), "width changes identity");
  }

  /* --- constituent bins --- */
  {
    uint8_t bins[4];
    ChannelDef d = mk(5, 104, CHANNEL_WIDTH_40);
    norm(d);
    CHECK(constituent_bins(d, bins) == 2 && bins[0] == 100 && bins[1] == 104,
          "104/40l bins");
    ChannelDef q = mk(5, 36, CHANNEL_WIDTH_80);
    norm(q);
    CHECK(constituent_bins(q, bins) == 4 && bins[0] == 36 && bins[1] == 40 &&
              bins[2] == 44 && bins[3] == 48,
          "36/80 bins");
    ChannelDef q2 = mk(5, 161, CHANNEL_WIDTH_80);
    norm(q2);
    CHECK(constituent_bins(q2, bins) == 4 && bins[0] == 149 && bins[3] == 161,
          "161/80 bins anchor at 149");
    ChannelDef s = mk(5, 149, CHANNEL_WIDTH_20);
    CHECK(constituent_bins(s, bins) == 1 && bins[0] == 149, "20 MHz one bin");
  }

  /* --- center derivation --- */
  {
    ChannelDef d = mk(5, 36, CHANNEL_WIDTH_80);
    norm(d);
    CHECK(d.center_mhz() == 5210, "36/80 center 5210");
    ChannelDef u = mk(5, 104, CHANNEL_WIDTH_40);
    norm(u);
    CHECK(u.center_mhz() == 5510, "104/40l center 5510");
    ChannelDef b = mk(2, 6, CHANNEL_WIDTH_40, 1);
    norm(b);
    CHECK(b.center_mhz() == 2447, "6/40u center 2447");
    ChannelDef s = mk(5, 149, CHANNEL_WIDTH_20);
    CHECK(s.center_mhz() == 5745, "149/20 center 5745");
  }

  /* --- overlap / adjacency geometry --- */
  {
    ChannelDef w = mk(5, 36, CHANNEL_WIDTH_80);
    norm(w);
    ChannelDef in = mk(5, 40, CHANNEL_WIDTH_20);
    CHECK(overlap_mhz(w, in) == 20, "20-in-80 overlap");
    ChannelDef pair = mk(5, 44, CHANNEL_WIDTH_40);
    norm(pair);
    CHECK(overlap_mhz(w, pair) == 40, "40-in-80 overlap");
    ChannelDef out = mk(5, 149, CHANNEL_WIDTH_20);
    CHECK(overlap_mhz(w, out) == 0, "disjoint blocks");
    ChannelDef a = mk(5, 36, CHANNEL_WIDTH_20), b = mk(5, 40, CHANNEL_WIDTH_20);
    CHECK(adjacent(a, b) && overlap_mhz(a, b) == 0, "touching bins adjacent");
    ChannelDef c = mk(5, 44, CHANNEL_WIDTH_20);
    CHECK(!adjacent(a, c), "20 MHz gap is not adjacent");
    ChannelDef g1 = mk(2, 1, CHANNEL_WIDTH_20), g6 = mk(2, 6, CHANNEL_WIDTH_20);
    CHECK(adjacent(g1, g6), "2.4 GHz 5 MHz gap adjacent");
    ChannelDef xb = mk(2, 6, CHANNEL_WIDTH_20);
    CHECK(overlap_mhz(w, xb) == 0 && !adjacent(w, xb), "cross-band disjoint");
  }

  /* --- canonical format round-trip --- */
  {
    const char *tokens[] = {"5:104/40l", "5:36/80", "2:6/40u", "5:149/20",
                            "5:157/40u", "2:11/20"};
    for (const char *t : tokens) {
      ChannelDef d;
      std::string err;
      CHECK(parse_chan_token(t, d, err), "canonical token parses");
      CHECK(d.str() == t, "format round-trip");
    }
    ChannelDef d;
    std::string err;
    CHECK(!parse_chan_token("2:36/20", d, err), "band prefix mismatch");
  }

  /* --- scan-plan grammar --- */
  {
    std::vector<ChannelDef> out;
    std::vector<PlanParseError> errs;
    CHECK(parse_scan_plan("104/40l:b,36/80,132/20:p", out, errs),
          "well-formed plan parses");
    CHECK(out.size() == 3 && errs.empty(), "3 candidates, no errors");
    CHECK(out[0].backup && !out[0].no_ir, "backup flag");
    CHECK(out[0].width == CHANNEL_WIDTH_40 && out[0].offset == 2,
          "40l offset");
    CHECK(out[2].no_ir, "no-IR flag");

    CHECK(!parse_scan_plan("104/40l,6", out, errs), "mixed band rejected");
    CHECK(errs.size() == 1 && errs[0].token == "6", "mixed-band token named");
    CHECK(out.size() == 1, "surviving candidates kept");

    CHECK(!parse_scan_plan("36,36", out, errs), "duplicate reported");
    CHECK(!parse_scan_plan("36,38/40,44:x,52/33", out, errs),
          "malformed tokens reported");
    CHECK(errs.size() == 3, "every bad token reported");
    CHECK(out.size() == 1 && out[0].primary == 36, "good token survives");
    CHECK(!parse_scan_plan("", out, errs), "empty plan rejected");
    CHECK(!parse_scan_plan("6/40", out, errs), "ambiguous 2.4 pair reported");
  }

  /* --- to_selected adapter --- */
  {
    ChannelDef d = mk(5, 104, CHANNEL_WIDTH_40);
    norm(d);
    const SelectedChannel s = d.to_selected();
    CHECK(s.Channel == 104 && s.ChannelOffset == 2 &&
              s.ChannelWidth == CHANNEL_WIDTH_40,
          "SelectedChannel mapping");
  }

  return fails ? 1 : 0;
}
