/* EvidenceStore fold-path + SurveyJsonl round-trip + JsonlLite selftest.
 *
 * The fold path is the aggregator's trust boundary: synthetic counter wrap,
 * missing constituent bins, stale and delayed records, width overlap, mixed
 * calibration domains, and failure-flagged dwells must all be handled
 * exactly — a record that should not rank must never rank. */
#include "chanmig/EvidenceStore.h"
#include "chanmig/JsonlLite.h"
#include "chanmig/SurveyJsonl.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

static int fails;
#define CHECK(x, msg)                                                          \
  do {                                                                         \
    if (!(x)) {                                                                \
      std::fprintf(stderr, "FAIL: %s\n", msg);                                 \
      ++fails;                                                                 \
    }                                                                          \
  } while (0)

using namespace devourer::chanmig;
using Fold = EvidenceStore::Fold;

static bool near(double a, double b) { return a > b - 1e-9 && a < b + 1e-9; }

static std::vector<ChannelDef> plan() {
  std::vector<ChannelDef> out;
  std::vector<PlanParseError> errs;
  /* 36/20, the 44+48 pair, and a 20 MHz candidate INSIDE that pair (width
   * overlap: bin 44 serves both). */
  parse_scan_plan("36,44/40,44", out, errs);
  return out;
}
static constexpr uint32_t kPlanHash = 0xabcd1234;
static constexpr int64_t kMaxAge = 60000;

static SurveyDwell dwell(uint8_t bin, int64_t t_end, uint64_t round = 0,
                         uint32_t scout = 1) {
  SurveyDwell d;
  d.def.band = 5;
  d.def.primary = bin;
  d.def.width = CHANNEL_WIDTH_20;
  d.plan_hash = kPlanHash;
  d.t_start_ms = t_end - 130;
  d.t_end_ms = t_end;
  d.observe_ms = 100;
  d.settle_ms = 30;
  d.retune_us = 800;
  d.valid_fa = true;
  d.cca_ofdm = 40;
  d.fa_ofdm = 12;
  d.valid_igi = true;
  d.igi = 0x30;
  d.valid_nhm = true;
  d.nhm_busy_pct = 10;
  d.frames = 5;
  d.dvr_air_us = 2000;
  d.oth_air_us = 8000;
  d.round = round;
  d.scout_id = scout;
  return d;
}

int main() {
  /* --- accept path + generation + rate reduction --- */
  {
    EvidenceStore s(plan(), kPlanHash, kMaxAge);
    CHECK(s.candidate_count() == 3, "3 candidates");
    CHECK(s.generation() == 0, "generation starts at 0");
    CHECK(s.ingest(dwell(36, 1000), 1100) == Fold::Accepted, "accept");
    CHECK(s.generation() == 1, "generation increments");
    std::vector<const BinCell *> cells;
    s.fresh_cells(36, 1100, cells);
    CHECK(cells.size() == 1, "cell stored");
    CHECK(near(cells[0]->cca_rate, 400.0) && near(cells[0]->fa_rate, 120.0),
          "per-second rate reduction");
    CHECK(near(cells[0]->dvr_air_frac, 0.02) &&
              near(cells[0]->oth_air_frac, 0.08),
          "airtime fractions");
    CHECK(s.scout_id() == 1, "scout identity latched");
  }

  /* --- counter wrap / implausible deltas --- */
  {
    EvidenceStore s(plan(), kPlanHash, kMaxAge);
    SurveyDwell d = dwell(36, 1000);
    d.cca_ofdm = 0xFFFFFF00u; /* a wrapped counter reads as an absurd rate */
    CHECK(s.ingest(d, 1100) == Fold::RejectedSuspect, "wrap rejected");
    SurveyDwell f = dwell(36, 1000);
    f.flags |= kFlagCounterSuspect;
    CHECK(s.ingest(f, 1100) == Fold::RejectedSuspect,
          "producer-flagged suspect rejected");
    CHECK(s.generation() == 0, "rejects never bump the generation");
    CHECK(s.fold_count(Fold::RejectedSuspect) == 2, "reject reason counted");
    /* boundary: exactly at the ceiling is plausible */
    SurveyDwell b = dwell(36, 1000);
    b.cca_ofdm = 100 * 1000; /* kMaxCountsPerMs * observe_ms */
    CHECK(s.ingest(b, 1100) == Fold::Accepted, "ceiling itself accepted");
  }

  /* --- missing bins => width-incomplete --- */
  {
    EvidenceStore s(plan(), kPlanHash, kMaxAge);
    (void)s.ingest(dwell(44, 1000), 1100);
    /* candidate 1 is 44/40 (bins 44+48): only one bin covered */
    CHECK(s.bins_covered(1, 1100) == 1 && s.bin_count(1) == 2,
          "wide candidate width-incomplete");
    CHECK(s.evidence_age_ms(1, 1100) == -1,
          "incomplete width has no evidence age");
    (void)s.ingest(dwell(48, 2000), 2100);
    CHECK(s.bins_covered(1, 2100) == 2, "full width covered");
    CHECK(s.evidence_age_ms(1, 2100) == 1100,
          "age = oldest freshest constituent bin");
  }

  /* --- width overlap: a shared bin serves both candidates --- */
  {
    EvidenceStore s(plan(), kPlanHash, kMaxAge);
    (void)s.ingest(dwell(44, 1000), 1100);
    /* candidate 2 is 44/20 — the same bin covers it fully */
    CHECK(s.bins_covered(2, 1100) == 1 && s.bin_count(2) == 1,
          "shared bin covers the 20 MHz candidate");
    CHECK(s.evidence_age_ms(2, 1100) == 100, "age from the shared cell");
  }

  /* --- stale and delayed records --- */
  {
    EvidenceStore s(plan(), kPlanHash, kMaxAge);
    CHECK(s.ingest(dwell(36, 1000), 1000 + kMaxAge) == Fold::Accepted,
          "exactly max-age accepted");
    CHECK(s.ingest(dwell(36, 1000), 1000 + kMaxAge + 1) == Fold::RejectedStale,
          "past max-age rejected");
    /* Delayed out-of-order arrival: an older-but-fresh record folds, and the
     * newest cell (by record time, not arrival) still wins. */
    CHECK(s.ingest(dwell(36, 5000), 5100) == Fold::Accepted, "newer record");
    CHECK(s.ingest(dwell(36, 3000), 5200) == Fold::Accepted,
          "delayed record folds by record time");
    std::vector<const BinCell *> cells;
    s.fresh_cells(36, 5200, cells);
    CHECK(cells.size() == 3, "all fresh cells kept");
    /* Freshness is judged against record time as the store ages. */
    s.fresh_cells(36, 3000 + kMaxAge + 1, cells);
    bool has3000 = false;
    for (auto *c : cells)
      has3000 = has3000 || c->t_ms == 3000;
    CHECK(!has3000, "delayed record ages out by its record time");
  }

  /* --- failure-flagged dwells never fold --- */
  {
    EvidenceStore s(plan(), kPlanHash, kMaxAge);
    for (uint16_t flag : {kFlagRetuneFailed, kFlagReadFailed, kFlagTruncated}) {
      SurveyDwell d = dwell(36, 1000);
      d.flags |= flag;
      CHECK(s.ingest(d, 1100) == Fold::RejectedFlags, "failure flag rejected");
    }
    SurveyDwell z = dwell(36, 1000);
    z.observe_ms = 0;
    CHECK(s.ingest(z, 1100) == Fold::RejectedFlags, "zero observation");
  }

  /* --- unknown bin / plan mismatch --- */
  {
    EvidenceStore s(plan(), kPlanHash, kMaxAge);
    CHECK(s.ingest(dwell(149, 1000), 1100) == Fold::RejectedUnknownBin,
          "bin outside the plan");
    SurveyDwell d = dwell(36, 1000);
    d.plan_hash = 0x1111;
    CHECK(s.ingest(d, 1100) == Fold::RejectedPlanMismatch, "plan mismatch");
  }

  /* --- mixed calibration domains reset the store --- */
  {
    EvidenceStore s(plan(), kPlanHash, kMaxAge);
    (void)s.ingest(dwell(36, 1000, 0, 1), 1100);
    (void)s.ingest(dwell(44, 1000, 0, 1), 1100);
    CHECK(s.ingest(dwell(48, 2000, 0, 2), 2100) == Fold::RejectedMixedScout,
          "different scout rejected");
    std::vector<const BinCell *> cells;
    s.fresh_cells(36, 2100, cells);
    CHECK(cells.empty(), "rings reset on domain change");
    CHECK(s.scout_id() == 2, "new domain adopted");
    CHECK(s.ingest(dwell(48, 3000, 0, 2), 3100) == Fold::Accepted,
          "new domain folds after reset");
  }

  /* --- rounds covered + ring bound --- */
  {
    EvidenceStore s(plan(), kPlanHash, kMaxAge);
    (void)s.ingest(dwell(36, 1000, 3), 1100);
    (void)s.ingest(dwell(36, 2000, 5), 2100);
    CHECK(s.rounds_covered(0, 2100) == 3, "rounds span (hi-lo+1)");
    for (int i = 0; i < 40; i++)
      (void)s.ingest(dwell(36, 3000 + i, 6), 3100 + i);
    std::vector<const BinCell *> cells;
    s.fresh_cells(36, 3200, cells);
    CHECK(cells.size() == 32, "ring bounded");
  }

  /* --- full-width dwells land in the candidate's wide ring --- */
  {
    EvidenceStore s(plan(), kPlanHash, kMaxAge);
    SurveyDwell w = dwell(44, 1000);
    w.def.width = CHANNEL_WIDTH_40;
    w.def.offset = 1;
    w.flags |= kFlagFullWidth;
    CHECK(s.ingest(w, 1100) == Fold::Accepted, "wide dwell accepted");
    std::vector<const BinCell *> cells;
    s.wide_cells(1, 1100, cells);
    CHECK(cells.size() == 1, "wide ring holds it");
    s.fresh_cells(44, 1100, cells);
    CHECK(cells.empty(), "wide dwell is not bin evidence");
    SurveyDwell u = w;
    u.def.primary = 157; /* a wide def not in the plan */
    CHECK(s.ingest(u, 1100) == Fold::RejectedUnknownBin,
          "unknown wide candidate rejected");
  }

  /* --- SurveyJsonl round-trip pins the schema --- */
  {
    std::FILE *f = std::tmpfile();
    devourer::EventSink sink;
    sink.configure(f);
    SurveyDwell d = dwell(44, 12345, 7);
    d.def.width = CHANNEL_WIDTH_40;
    d.def.offset = 1;
    d.flags = kFlagFullWidth;
    d.seq = 99;
    d.evm_valid = true;
    d.evm_mean_raw = -52;
    d.nhm[0] = 200;
    d.nhm[11] = 3;
    d.nhm_dur = 2;
    d.nhm_peak = 0;
    d.adapter_gen = 2;
    d.scout_id = 0x9f3a2c11;
    emit_survey_dwell(sink, d);
    std::fflush(f);
    long n = std::ftell(f);
    std::rewind(f);
    std::string line(static_cast<size_t>(n), '\0');
    CHECK(std::fread(line.data(), 1, line.size(), f) == line.size(),
          "read back emission");
    std::fclose(f);

    SurveyDwell r;
    CHECK(survey_dwell_from_jsonl(line, r), "round-trip parses");
    CHECK(r.def.same_rf(d.def), "channel def survives");
    CHECK(r.seq == d.seq && r.round == d.round && r.flags == d.flags,
          "counters survive");
    CHECK(r.t_end_ms == d.t_end_ms && r.observe_ms == d.observe_ms,
          "timing survives");
    CHECK(r.valid_fa && r.cca_ofdm == d.cca_ofdm && r.fa_ofdm == d.fa_ofdm,
          "energy survives");
    CHECK(r.valid_igi && r.igi == d.igi, "igi survives");
    CHECK(r.valid_nhm && r.nhm_busy_pct == d.nhm_busy_pct &&
              r.nhm[0] == 200 && r.nhm[11] == 3,
          "nhm survives");
    CHECK(r.evm_valid && r.evm_mean_raw == -52, "evm survives");
    CHECK(r.dvr_air_us == d.dvr_air_us && r.oth_air_us == d.oth_air_us,
          "airtime survives");
    CHECK(r.scout_id == d.scout_id && r.plan_hash == d.plan_hash,
          "identity survives");

    /* Nullable fields: a generation with no counters emits null, and the
     * parse must come back invalid — never a fake zero. */
    std::FILE *f2 = std::tmpfile();
    devourer::EventSink sink2;
    sink2.configure(f2);
    SurveyDwell d2 = dwell(36, 1000);
    d2.valid_fa = false;
    d2.valid_igi = false;
    d2.valid_nhm = false;
    emit_survey_dwell(sink2, d2);
    std::fflush(f2);
    long n2 = std::ftell(f2);
    std::rewind(f2);
    std::string line2(static_cast<size_t>(n2), '\0');
    CHECK(std::fread(line2.data(), 1, line2.size(), f2) == line2.size(),
          "read back null emission");
    std::fclose(f2);
    SurveyDwell r2;
    CHECK(survey_dwell_from_jsonl(line2, r2), "null form parses");
    CHECK(!r2.valid_fa && !r2.valid_igi && !r2.valid_nhm,
          "null fields stay invalid");
  }

  /* --- JsonlLite scanner corner cases --- */
  {
    const std::string line =
        "{\"ev\":\"rx.quality\",\"verdict\":\"HEALTHY\",\"frames\":120,"
        "\"snr_mean_db\":18.5,\"evm_db\":null,"
        "\"text\":\"fake \\\"frames\\\":999 inside a string\","
        "\"rssi_dbm\":[-40,-42],\"igi\":48}";
    CHECK(jsonl_ev_is(line, "rx.quality"), "event name match");
    CHECK(!jsonl_ev_is(line, "rx.qual"), "prefix does not match");
    long long v = 0;
    CHECK(jsonl_int(line, "frames", &v) && v == 120, "int field");
    double dv = 0;
    CHECK(jsonl_num(line, "snr_mean_db", &dv) && dv == 18.5, "double field");
    CHECK(!jsonl_num(line, "evm_db", &dv), "null reads as absent");
    CHECK(!jsonl_num(line, "missing", &dv), "absent key");
    std::string sv;
    CHECK(jsonl_str(line, "verdict", &sv) && sv == "HEALTHY", "string field");
    CHECK(jsonl_str(line, "text", &sv) &&
              sv == "fake \"frames\":999 inside a string",
          "escapes unescaped");
    /* The poisoned "frames":999 inside the string value must not shadow the
     * real field. */
    CHECK(jsonl_int(line, "frames", &v) && v == 120,
          "key inside a string value never matches");
    int arr[4];
    int n = 0;
    CHECK(jsonl_arr(line, "rssi_dbm", arr, 4, &n) && n == 2 && arr[0] == -40 &&
              arr[1] == -42,
          "array field");
    CHECK(!jsonl_num("not json at all", "frames", &dv), "non-JSON line");
    CHECK(!jsonl_num("{\"ev\":\"x\",\"k\":", "k", &dv), "truncated line");
  }

  return fails ? 1 : 0;
}
