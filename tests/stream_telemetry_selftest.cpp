// Headless guard for src/StreamTelemetry.h: known-answer bytes for the addr3
// field and the marker IE, the canonical-SA-in-addr3 rejection, clipping, the
// 655.36 ms TSF unwrap on both sides of a wrap, marker tamper/truncation/version
// rejection, the latency arithmetic and the window accumulator's p50/max.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "StreamTelemetry.h"

using namespace devourer::stream_timing;

static int fails = 0;
#define CHECK(cond, msg)                                              \
  do {                                                                \
    if (!(cond)) {                                                    \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, msg); \
      ++fails;                                                        \
    }                                                                 \
  } while (0)

int main() {
  // ---- FrameTiming KAT ------------------------------------------------
  {
    FrameTiming t;
    t.has_capture = true; t.has_tsf = true; t.has_depth = true; t.tx_async = false;
    t.depth = 3; t.tsf10_lo = 0xbeef; t.c2s10 = 0x0102;
    uint8_t b[6];
    t.encode(b);
    const uint8_t want[6] = {0x27, 0x03, 0xef, 0xbe, 0x02, 0x01};
    CHECK(std::memcmp(b, want, 6) == 0, "FrameTiming KAT bytes");
    FrameTiming d;
    CHECK(FrameTiming::decode(b, d), "FrameTiming decodes");
    CHECK(d.has_capture && d.has_tsf && d.has_depth && !d.tx_async, "flags round-trip");
    CHECK(d.depth == 3 && d.tsf10_lo == 0xbeef && d.c2s10 == 0x0102, "fields round-trip");
  }
  // The canonical SA left in addr3 by a demo that writes no telemetry.
  {
    const uint8_t sa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
    FrameTiming d;
    CHECK(!FrameTiming::decode(sa, d), "canonical SA in addr3 is not telemetry");
    uint8_t v0[6] = {0x00, 0, 0, 0, 0, 0};
    CHECK(!FrameTiming::decode(v0, d), "version 0 rejected");
    uint8_t v2[6] = {0x40, 0, 0, 0, 0, 0};
    CHECK(!FrameTiming::decode(v2, d), "version 2 rejected");
  }
  // ---- FrameTimingExt (addr1) KAT ---------------------------------------
  {
    FrameTimingExt x;
    x.t_queue10 = 0x1234; x.t_write_prev10 = 0xabcd; x.ctr = 0x5e;
    uint8_t b[6];
    x.encode(b);
    const uint8_t want[6] = {0x07, 0x34, 0x12, 0xcd, 0xab, 0x5e};
    CHECK(std::memcmp(b, want, 6) == 0, "FrameTimingExt KAT bytes");
    CHECK((b[0] & 0x01) && (b[0] & 0x02), "addr1 stays a locally administered group address");
    FrameTimingExt d;
    CHECK(FrameTimingExt::decode(b, d) && d.t_queue10 == 0x1234 &&
              d.t_write_prev10 == 0xabcd && d.ctr == 0x5e,
          "FrameTimingExt round-trip");
    const uint8_t bcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    CHECK(!FrameTimingExt::decode(bcast, d), "broadcast DA is not an extension");
    uint8_t uni[6] = {0x06, 0, 0, 0, 0, 0};  // version 1 but the group bit clear
    CHECK(!FrameTimingExt::decode(uni, d), "a unicast-looking DA is rejected");
    uint8_t v2[6] = {0x0b, 0, 0, 0, 0, 0};
    CHECK(!FrameTimingExt::decode(v2, d), "other extension version rejected");
  }
  // ---- units ----------------------------------------------------------
  CHECK(clip10(0) == 0 && clip10(9) == 0 && clip10(10) == 1, "clip10 floor");
  CHECK(clip10(655350) == 0xffff && clip10(10000000) == 0xffff, "clip10 clips");
  {
    // Submit at 1,000,000 us -> lo16 of 100000 = 0x86a0. Arrival 200 us later.
    const uint16_t lo = static_cast<uint16_t>((1000000 / 10) & 0xffff);
    CHECK(unwrap_tsf10(lo, 1000200) == 1000000, "unwrap, no wrap");
    // Across the wrap: submit just below a 655360 boundary, arrival just above.
    const int64_t sub = 5 * kTsf10WrapUs - 100;
    const uint16_t lo2 = static_cast<uint16_t>((sub / 10) & 0xffff);
    CHECK(unwrap_tsf10(lo2, 5 * kTsf10WrapUs + 300) == sub, "unwrap across wrap up");
    // Arrival slightly BEFORE submit (fit error): still the nearest congruent.
    const int64_t sub3 = 7 * kTsf10WrapUs + 50;
    const uint16_t lo3 = static_cast<uint16_t>((sub3 / 10) & 0xffff);
    CHECK(unwrap_tsf10(lo3, 7 * kTsf10WrapUs - 20) == sub3, "unwrap across wrap down");
  }
  // ---- latency ----------------------------------------------------------
  {
    FrameTiming t;
    t.has_tsf = true; t.has_capture = true;
    t.tsf10_lo = static_cast<uint16_t>((123456780 / 10) & 0xffff);
    t.c2s10 = 250;  // 2.5 ms
    FrameLatency l;
    CHECK(latency(t, 123456780 + 1500, l), "latency computes");
    CHECK(l.submit_to_air_us == 1500, "submit->air");
    CHECK(l.has_capture && l.capture_to_air_us == 1500 + 2500, "capture->air");
    FrameTiming notsf;
    CHECK(!latency(notsf, 0, l), "no tsf -> no latency");
  }
  // ---- marker KAT / tamper / truncation / version ------------------------
  {
    TimingMarker m;
    m.flags = kMkPrespStamped | kMkFitReady;
    m.tsf_pred_us = 0x0102030405060708ull;
    m.host_ns = 0x1112131415161718ull;
    m.fit_ppm_x100 = -1234;
    m.fit_n = 42;
    m.frames = 1000;
    m.t_queue_p50_10 = 1; m.t_queue_max_10 = 2;
    m.t_write_p50_10 = 3; m.t_write_max_10 = 4;
    m.c2s_p50_10 = 5; m.c2s_max_10 = 6;
    m.depth_max = 7; m.captured = 8;
    auto b = TimingMarker::encode(m);
    CHECK(b.size() == 51, "marker size");
    const uint8_t head[7] = {221, 49, 0x57, 0x42, 0x75, 0x49, 1};
    CHECK(std::memcmp(b.data(), head, 7) == 0, "marker header KAT");
    CHECK(b[49] == 0xd7 && b[50] == 0x3b, "marker tail KAT");
    TimingMarker d;
    CHECK(TimingMarker::decode(b.data(), b.size(), d), "marker decodes");
    CHECK(d.flags == m.flags && d.tsf_pred_us == m.tsf_pred_us && d.host_ns == m.host_ns &&
              d.fit_ppm_x100 == -1234 && d.fit_n == 42 && d.frames == 1000 &&
              d.t_queue_p50_10 == 1 && d.t_queue_max_10 == 2 && d.t_write_p50_10 == 3 &&
              d.t_write_max_10 == 4 && d.c2s_p50_10 == 5 && d.c2s_max_10 == 6 &&
              d.depth_max == 7 && d.captured == 8,
          "marker fields round-trip");
    // Embedded in a whole MPDU (header + fixed fields + IEs) it is still found.
    std::vector<uint8_t> mpdu;
    const uint8_t sa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
    append_marker_mpdu(mpdu, sa, 36, m);
    CHECK(mpdu[0] == 0x50 && mpdu.size() == 24 + 8 + 2 + 2 + 2 + 15 + 3 + 51, "marker mpdu shape");
    CHECK(TimingMarker::decode(mpdu.data(), mpdu.size(), d) && d.frames == 1000, "marker found in mpdu");
    // Truncated: not found.
    CHECK(!TimingMarker::decode(b.data(), b.size() - 1, d), "truncated marker rejected");
    // Tail tamper: not found.
    auto t = b; t[50] ^= 1;
    CHECK(!TimingMarker::decode(t.data(), t.size(), d), "tail-tampered marker rejected");
    // Other version: not found.
    auto v = b; v[6] = 2;
    CHECK(!TimingMarker::decode(v.data(), v.size(), d), "other marker version rejected");
    // The hop sync marker's type byte (0x48) is not ours.
    auto h = b; h[5] = 0x48;
    CHECK(!TimingMarker::decode(h.data(), h.size(), d), "hop marker type rejected");
  }
  // ---- window ------------------------------------------------------------
  {
    TimingWindow w;
    for (unsigned i = 1; i <= 101; ++i) w.add(i * 10, 1000, 20000, i % 2 == 0, i % 5);
    CHECK(w.frames() == 101, "window frame count");
    TimingMarker m;
    w.drain_into(m);
    CHECK(m.frames == 101 && m.captured == 50 && m.depth_max == 4, "window counts");
    CHECK(m.t_queue_p50_10 == 51 && m.t_queue_max_10 == 101, "window queue p50/max");
    CHECK(m.t_write_p50_10 == 100 && m.c2s_max_10 == 2000, "window write/c2s");
    w.drain_into(m);
    CHECK(m.frames == 0 && m.t_queue_max_10 == 0, "window reset after drain");
    // A window longer than the reservoir: a monotonically rising series of
    // 40000 frames has its true median at ~20000 us; a reservoir that stopped
    // taking samples at 8192 would report ~4096 us.
    TimingWindow big;
    for (unsigned i = 1; i <= 40000; ++i) big.add(i, 0, 0, false, 0);
    big.drain_into(m);
    CHECK(m.frames == 40000, "long window frame count");
    CHECK(m.t_queue_p50_10 > 1700 && m.t_queue_p50_10 < 2300,
          "long window p50 is the whole window's, not the first 8192 frames'");
    CHECK(m.t_queue_max_10 == 4000, "long window max exact");
  }

  // ---- CCX join ring --------------------------------------------------------
  {
    TxReportJoin j;
    std::optional<TxFrameRec> r;
    CHECK(!j.match(5) && j.unmatched() == 1, "report before any send is unmatched");
    TxFrameRec a; a.frame = 7; a.t_queue_us = 30;
    j.sent(5, a);
    r = j.match(5);
    CHECK(r && r->frame == 7 && r->t_queue_us == 30 && j.joined() == 1, "report joins its frame");
    CHECK(!j.match(5) && j.unmatched() == 2, "a second report for the same tag is unmatched");
    for (unsigned i = 0; i < 300; ++i) { TxFrameRec x; x.frame = 100 + i; j.sent(uint8_t(i), x); }
    r = j.match(10);  // tag 10 was written at i=10 and again at i=266
    CHECK(r && r->frame == 100 + 266, "a wrapped tag yields the latest frame with it");
    CHECK(j.sent_count() == 301, "sent count");
  }

  if (fails == 0) std::printf("stream_telemetry selftest OK\n");
  return fails == 0 ? 0 : 1;
}
