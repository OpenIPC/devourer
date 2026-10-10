/* StreamTelemetry — per-frame TX-side timing for the stream link, in bytes the
 * receiver never otherwise reads.
 *
 * Two carriers, both pure wire codecs here (no device dependency):
 *
 *  1. FrameTiming — six bytes in the 802.11 header's addr3 (BSSID) of every
 *     stream frame, and FrameTimingExt — five more in addr1 (the DA), which
 *     stays a group address so nothing ACKs it (byte 0 = 0x03 | version<<2;
 *     broadcast decodes as "none"). Which receivers deliver such a DA, and
 *     the numbers, are in docs/stream-timing.md. The stream demos fill addr3 with the canonical SA today and
 *     no receiver reads it (every consumer keys on addr2 and slices the body at
 *     +24), so the FEC bodies stay byte-for-byte untouched and the MTU is
 *     unchanged. It carries what kestrel-air puts in its slice header: the
 *     backlog depth, the time from the producer's capture stamp (or the stdin
 *     read) to send_packet, and the low 16 bits of the transmitter's PREDICTED
 *     TSF at the send call in 10 µs units, so a receiver that knows the
 *     transmitter's clock (TsfSync fed from egress-stamped frames) gets the
 *     one-way submit→arrival latency per frame, host jitter on the TX side
 *     only. The TX never reads a register per frame: the TSF is predicted from
 *     a slow host-clock↔TSF fit (examples/common/host_tsf_fit.h).
 *
 *  2. TimingMarker — a vendor IE (221 / OUI 57:42:75 / type 0x49; 0x48 is the
 *     hop sync marker) on its own periodic frame, a probe response with the
 *     canonical SA. It carries the absolute (predicted TSF, host ns) pair, the
 *     fit state, and windowed stage statistics since the previous marker:
 *     stdin-read→send (t_queue), the previous frame's send_packet wall time
 *     (t_write, kestrel-air's T_WRITE), capture→send, depth. On the dies whose
 *     MAC stamps an injected probe response with its egress TSF (measured:
 *     Jaguar2, Jaguar3, Kestrel — tests/probe_resp_egress_tsf_check.sh) the
 *     marker frame's own timestamp field is the hardware egress pair the RX
 *     fit needs; where it is not (Jaguar1: the 8821AU rewrites the field with
 *     a counter that is not either TSF port), the flags say so and the clock
 *     pair rides the hardware TBTT beacon instead. A software-stamped pair must
 *     never feed the fit: it folds the mean latency into the offset.
 *
 * Versioned with backward rejection (the HopSyncMarker convention). The
 * FrameTiming version lives in addr3 byte 0 bits 7..5 and is chosen so the
 * canonical SA's first byte (0x57 → version 2) decodes as "no telemetry": a
 * frame from a demo that never wrote the field is rejected, not misread. */
#ifndef DEVOURER_STREAM_TELEMETRY_H
#define DEVOURER_STREAM_TELEMETRY_H

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdlib>
#include <cstdint>
#include <optional>
#include <vector>

namespace devourer {
namespace stream_timing {

/* ---- units ----------------------------------------------------------- */

/* Stage times travel as u16 in 10 µs units (655.35 ms range), clipped. */
inline uint16_t clip10(uint64_t us) {
  const uint64_t t = us / 10;
  return t > 0xffff ? 0xffff : static_cast<uint16_t>(t);
}

/* The TSF travels as the low 16 bits of (tsf_us / 10): a 655.36 ms wrap.
 * Unwrap against a nearby absolute (the receiver's mapped arrival): the fit
 * error is tens of µs, so the nearest congruent value is the right one as long
 * as |arrival − submit| < 327 ms, which any live stream satisfies. */
inline constexpr int64_t kTsf10WrapUs = 655360;
inline int64_t unwrap_tsf10(uint16_t lo16, int64_t near_us) {
  const int64_t base = near_us - (((near_us % kTsf10WrapUs) + kTsf10WrapUs) % kTsf10WrapUs) +
                       static_cast<int64_t>(lo16) * 10;
  int64_t best = base;
  for (int64_t cand : {base - kTsf10WrapUs, base + kTsf10WrapUs})
    if (std::llabs(cand - near_us) < std::llabs(best - near_us)) best = cand;
  return best;
}

/* ---- per-frame field: addr3 ------------------------------------------ */

inline constexpr uint8_t kFrameVersion = 1; /* byte 0 bits 7..5 */
inline constexpr uint8_t kFrameHasCapture = 0x01; /* c2s measured from a producer capture stamp */
inline constexpr uint8_t kFrameHasTsf = 0x02;     /* tsf10_lo is from a ready fit */
inline constexpr uint8_t kFrameHasDepth = 0x04;   /* depth is a real in-flight count */
inline constexpr uint8_t kFrameTxAsync = 0x08;    /* async TX: t_write describes the submit, not the air */

struct FrameTiming {
  bool has_capture = false, has_tsf = false, has_depth = false, tx_async = false;
  uint8_t depth = 0;      /* frames in flight behind this one, clipped 255 */
  uint16_t tsf10_lo = 0;  /* predicted TX TSF at send_packet, (us/10) & 0xffff */
  uint16_t c2s10 = 0;     /* capture (or stdin read) → send_packet, 10 µs, clipped */

  static constexpr size_t kSize = 6;

  void encode(uint8_t out[kSize]) const {
    uint8_t f = 0;
    if (has_capture) f |= kFrameHasCapture;
    if (has_tsf) f |= kFrameHasTsf;
    if (has_depth) f |= kFrameHasDepth;
    if (tx_async) f |= kFrameTxAsync;
    out[0] = static_cast<uint8_t>((kFrameVersion << 5) | (f & 0x1f));
    out[1] = depth;
    out[2] = static_cast<uint8_t>(tsf10_lo);
    out[3] = static_cast<uint8_t>(tsf10_lo >> 8);
    out[4] = static_cast<uint8_t>(c2s10);
    out[5] = static_cast<uint8_t>(c2s10 >> 8);
  }
  /* False for any other version — including the canonical SA left in addr3
   * by a demo that does not write telemetry (0x57 >> 5 == 2). */
  static bool decode(const uint8_t in[kSize], FrameTiming &t) {
    if ((in[0] >> 5) != kFrameVersion) return false;
    t.has_capture = in[0] & kFrameHasCapture;
    t.has_tsf = in[0] & kFrameHasTsf;
    t.has_depth = in[0] & kFrameHasDepth;
    t.tx_async = in[0] & kFrameTxAsync;
    t.depth = in[1];
    t.tsf10_lo = static_cast<uint16_t>(in[2] | (in[3] << 8));
    t.c2s10 = static_cast<uint16_t>(in[4] | (in[5] << 8));
    return true;
  }
};

/* ---- per-frame extension: addr1 ------------------------------------------ */

/* addr1 byte 0: bit0 group (must stay set — a unicast DA would solicit an
 * ACK), bit1 locally administered, bits 7..2 the version. Broadcast ff reads
 * as version 63 and is rejected. */
inline constexpr uint8_t kExtVersion = 1;

struct FrameTimingExt {
  uint16_t t_queue10 = 0;      /* stdin read → send_packet, 10 µs, clipped */
  uint16_t t_write_prev10 = 0; /* the PREVIOUS frame's send_packet wall time, 10 µs
                                * (kestrel-air's T_WRITE); 0 on the first frame */
  uint8_t ctr = 0;             /* the transmitter's frame counter, low byte */

  static constexpr size_t kSize = 6;

  void encode(uint8_t out[kSize]) const {
    out[0] = static_cast<uint8_t>(0x03 | (kExtVersion << 2));
    out[1] = static_cast<uint8_t>(t_queue10);
    out[2] = static_cast<uint8_t>(t_queue10 >> 8);
    out[3] = static_cast<uint8_t>(t_write_prev10);
    out[4] = static_cast<uint8_t>(t_write_prev10 >> 8);
    out[5] = ctr;
  }
  static bool decode(const uint8_t in[kSize], FrameTimingExt &e) {
    if ((in[0] & 0x03) != 0x03 || (in[0] >> 2) != kExtVersion) return false;
    e.t_queue10 = static_cast<uint16_t>(in[1] | (in[2] << 8));
    e.t_write_prev10 = static_cast<uint16_t>(in[3] | (in[4] << 8));
    e.ctr = in[5];
    return true;
  }
};

/* Per-frame latency on the receiver. `remote_arrival_us` is this frame's
 * hardware arrival (tsfl) mapped onto the transmitter's TSF by the receiver's
 * fit. Returns false when the frame carries no TSF. On success `submit_to_air`
 * is arrival − predicted submit (µs; negative only by the fit error) and, when
 * the frame has a capture stamp, `capture_to_air` adds the TX-side c2s. */
struct FrameLatency {
  int64_t submit_to_air_us = 0;
  int64_t capture_to_air_us = 0;
  bool has_capture = false;
};
inline bool latency(const FrameTiming &t, int64_t remote_arrival_us, FrameLatency &out) {
  if (!t.has_tsf) return false;
  const int64_t submit = unwrap_tsf10(t.tsf10_lo, remote_arrival_us);
  out.submit_to_air_us = remote_arrival_us - submit;
  out.has_capture = t.has_capture;
  out.capture_to_air_us = out.submit_to_air_us + static_cast<int64_t>(t.c2s10) * 10;
  return true;
}

/* ---- periodic marker IE ----------------------------------------------- */

inline constexpr uint8_t kMarkerIeType = 0x49;
inline constexpr uint8_t kMarkerVersion = 1;
inline constexpr uint8_t kMkPrespStamped = 0x01;  /* this frame's timestamp field is a hardware egress TSF */
inline constexpr uint8_t kMkBeaconRunning = 0x02; /* a hardware TBTT beacon with the same SA carries the clock */
inline constexpr uint8_t kMkFitReady = 0x04;      /* tsf_pred_us comes from a ready host↔TSF fit */
inline constexpr uint8_t kMkTxAsync = 0x08;
inline constexpr uint8_t kMkCaptureSource = 0x10; /* the producer supplies capture stamps */

struct TimingMarker {
  uint8_t flags = 0;
  uint64_t tsf_pred_us = 0; /* predicted TX TSF when the marker was built */
  uint64_t host_ns = 0;     /* TX steady_clock at the same instant */
  int32_t fit_ppm_x100 = 0; /* host-clock vs TSF rate, 0.01 ppm */
  uint16_t fit_n = 0;       /* ReadTsf samples in the fit */
  uint32_t frames = 0;      /* data frames in the window since the previous marker */
  uint16_t t_queue_p50_10 = 0, t_queue_max_10 = 0; /* stdin read → send_packet */
  uint16_t t_write_p50_10 = 0, t_write_max_10 = 0; /* send_packet wall time */
  uint16_t c2s_p50_10 = 0, c2s_max_10 = 0;         /* capture → send_packet */
  uint8_t depth_max = 0;
  uint16_t captured = 0;    /* frames in the window that had a producer stamp */

  static constexpr size_t kSize = 51; /* IE hdr 2 + OUI 3 + type 1 + ver 1 + body 42 + tail 2 */

  static std::array<uint8_t, kSize> encode(const TimingMarker &m) {
    std::array<uint8_t, kSize> b{{221, kSize - 2, 0x57, 0x42, 0x75, kMarkerIeType, kMarkerVersion}};
    size_t o = 7;
    b[o++] = m.flags;
    put64(b.data() + o, m.tsf_pred_us); o += 8;
    put64(b.data() + o, m.host_ns); o += 8;
    put32(b.data() + o, static_cast<uint32_t>(m.fit_ppm_x100)); o += 4;
    put16(b.data() + o, m.fit_n); o += 2;
    put32(b.data() + o, m.frames); o += 4;
    put16(b.data() + o, m.t_queue_p50_10); o += 2;
    put16(b.data() + o, m.t_queue_max_10); o += 2;
    put16(b.data() + o, m.t_write_p50_10); o += 2;
    put16(b.data() + o, m.t_write_max_10); o += 2;
    put16(b.data() + o, m.c2s_p50_10); o += 2;
    put16(b.data() + o, m.c2s_max_10); o += 2;
    b[o++] = m.depth_max;
    put16(b.data() + o, m.captured); o += 2;
    b[o++] = 0xd7;
    b[o++] = 0x3b;
    return b;
  }
  /* Scans the buffer (a whole MPDU is fine — the signature cannot start inside
   * the 802.11 header: the byte after a 0xdd in addr1/addr2/addr3 is never the
   * length 49 followed by the OUI). Rejects every other version. */
  static bool decode(const uint8_t *p, size_t n, TimingMarker &m) {
    for (size_t i = 0; i + kSize <= n; ++i) {
      if (p[i] != 221 || p[i + 1] != kSize - 2 || p[i + 2] != 0x57 || p[i + 3] != 0x42 ||
          p[i + 4] != 0x75 || p[i + 5] != kMarkerIeType || p[i + 6] != kMarkerVersion ||
          p[i + kSize - 2] != 0xd7 || p[i + kSize - 1] != 0x3b)
        continue;
      size_t o = i + 7;
      m.flags = p[o++];
      m.tsf_pred_us = get64(p + o); o += 8;
      m.host_ns = get64(p + o); o += 8;
      m.fit_ppm_x100 = static_cast<int32_t>(get32(p + o)); o += 4;
      m.fit_n = get16(p + o); o += 2;
      m.frames = get32(p + o); o += 4;
      m.t_queue_p50_10 = get16(p + o); o += 2;
      m.t_queue_max_10 = get16(p + o); o += 2;
      m.t_write_p50_10 = get16(p + o); o += 2;
      m.t_write_max_10 = get16(p + o); o += 2;
      m.c2s_p50_10 = get16(p + o); o += 2;
      m.c2s_max_10 = get16(p + o); o += 2;
      m.depth_max = p[o++];
      m.captured = get16(p + o);
      return true;
    }
    return false;
  }

 private:
  static void put16(uint8_t *p, uint16_t v) { p[0] = uint8_t(v); p[1] = uint8_t(v >> 8); }
  static void put32(uint8_t *p, uint32_t v) {
    for (int i = 0; i < 4; ++i) p[i] = uint8_t(v >> (8 * i));
  }
  static void put64(uint8_t *p, uint64_t v) {
    for (int i = 0; i < 8; ++i) p[i] = uint8_t(v >> (8 * i));
  }
  static uint16_t get16(const uint8_t *p) { return uint16_t(p[0] | (p[1] << 8)); }
  static uint32_t get32(const uint8_t *p) {
    uint32_t v = 0;
    for (int i = 0; i < 4; ++i) v |= uint32_t(p[i]) << (8 * i);
    return v;
  }
  static uint64_t get64(const uint8_t *p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v |= uint64_t(p[i]) << (8 * i);
    return v;
  }
};

/* The marker's own frame: a probe response (FC 0x50) to the broadcast DA with
 * the given SA as addr2 and addr3, the 8-byte timestamp field zero (the MAC
 * overwrites it on the dies that stamp — the flag says which), beacon interval
 * and capability as a plain open BSS, an SSID IE naming the stream, the DS
 * parameter set for the channel, then the marker IE. Appended after the
 * caller's radiotap header. */
inline void append_mgmt_mpdu(std::vector<uint8_t> &out, uint8_t fc0, const uint8_t sa[6],
                             uint8_t channel, const TimingMarker *m) {
  const uint8_t hdr[24] = {fc0, 0x00, 0x00, 0x00,
                           0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                           sa[0], sa[1], sa[2], sa[3], sa[4], sa[5],
                           sa[0], sa[1], sa[2], sa[3], sa[4], sa[5],
                           0x00, 0x00};
  out.insert(out.end(), hdr, hdr + 24);
  out.insert(out.end(), 8, 0);            /* timestamp */
  out.push_back(0x64); out.push_back(0x00); /* interval 100 TU */
  out.push_back(0x21); out.push_back(0x04); /* ESS + short preamble */
  static const char ssid[] = "devourer-stream";
  out.push_back(0x00); out.push_back(sizeof(ssid) - 1);
  out.insert(out.end(), ssid, ssid + sizeof(ssid) - 1);
  out.push_back(0x03); out.push_back(0x01); out.push_back(channel);
  if (m) {
    const auto ie = TimingMarker::encode(*m);
    out.insert(out.end(), ie.begin(), ie.end());
  }
}
inline void append_marker_mpdu(std::vector<uint8_t> &out, const uint8_t sa[6],
                               uint8_t channel, const TimingMarker &m) {
  append_mgmt_mpdu(out, 0x50, sa, channel, &m);
}
/* The Jaguar1 fallback: the same shape as a beacon (FC 0x80) for StartBeacon,
 * whose MAC-stamped timestamp is the egress pair on that family. */
inline void append_beacon_mpdu(std::vector<uint8_t> &out, const uint8_t sa[6], uint8_t channel) {
  append_mgmt_mpdu(out, 0x80, sa, channel, nullptr);
}

/* ---- TX-side window accumulator -------------------------------------- */

/* What a TX demo feeds per frame and drains into a marker. Counts and maxima
 * are exact for the whole window; the p50 comes from a bounded uniform
 * reservoir (Algorithm R over the window's samples), so a window longer than
 * the reservoir is still represented end to end. */
class TimingWindow {
 public:
  static constexpr size_t kMaxSamples = 8192;

  void add(uint64_t t_queue_us, uint64_t t_write_us, uint64_t c2s_us, bool captured,
           unsigned depth) {
    ++_frames;
    if (captured) ++_captured;
    if (depth > _depth_max) _depth_max = depth;
    /* One slot decision per frame for all three series: the reservoir index
     * is uniform over [0, frames) (xorshift64*, seeded per window). */
    size_t slot = kMaxSamples;  /* "append" while the reservoir fills */
    if (_frames > kMaxSamples) {
      _rng ^= _rng >> 12; _rng ^= _rng << 25; _rng ^= _rng >> 27;
      slot = static_cast<size_t>((_rng * 0x2545F4914F6CDD1DULL) % _frames);
    }
    push(_q, t_queue_us, _q_max, slot);
    push(_w, t_write_us, _w_max, slot);
    push(_c, c2s_us, _c_max, slot);
  }
  uint32_t frames() const { return _frames; }

  /* Fill the window fields of `m` and reset for the next window. */
  void drain_into(TimingMarker &m) {
    m.frames = _frames;
    m.captured = static_cast<uint16_t>(std::min<uint32_t>(_captured, 0xffff));
    m.depth_max = static_cast<uint8_t>(std::min<unsigned>(_depth_max, 255));
    m.t_queue_p50_10 = clip10(p50(_q)); m.t_queue_max_10 = clip10(_q_max);
    m.t_write_p50_10 = clip10(p50(_w)); m.t_write_max_10 = clip10(_w_max);
    m.c2s_p50_10 = clip10(p50(_c)); m.c2s_max_10 = clip10(_c_max);
    _frames = _captured = 0; _depth_max = 0;
    _q.clear(); _w.clear(); _c.clear();
    _q_max = _w_max = _c_max = 0;
  }

 private:
  static void push(std::vector<uint64_t> &v, uint64_t x, uint64_t &mx, size_t slot) {
    if (x > mx) mx = x;
    if (v.size() < kMaxSamples) v.push_back(x);
    else if (slot < kMaxSamples) v[slot] = x;
  }
  static uint64_t p50(std::vector<uint64_t> &v) {
    if (v.empty()) return 0;
    auto mid = v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2);
    std::nth_element(v.begin(), mid, v.end());
    return *mid;
  }
  uint32_t _frames = 0, _captured = 0;
  unsigned _depth_max = 0;
  uint64_t _rng = 0x9E3779B97F4A7C15ULL;
  std::vector<uint64_t> _q, _w, _c;
  uint64_t _q_max = 0, _w_max = 0, _c_max = 0;
};

/* ---- CCX report join ----------------------------------------------------- */

/* Joins a HalMAC CCX TX report to the frame it describes by the 8-bit
 * SW_DEFINE tag the descriptor carried (IRtlRadio::NextTxReportTag read right
 * before the send). A 256-slot ring keyed by tag: a slot is written per
 * send, consumed by the report that echoes its tag, and a report whose slot
 * is empty or already consumed (a tag that wrapped past 256 unreported
 * frames, or a report for a frame sent before the join was armed) counts as
 * unmatched. Pure; the TX thread owns it. */
struct TxFrameRec {
  uint64_t frame = 0;     /* the transmitter's frame counter */
  uint64_t send_ns = 0;   /* host clock at send_packet */
  uint32_t t_queue_us = 0, c2s_us = 0;
  bool marker = false;    /* a marker frame, not a data frame */
};
class TxReportJoin {
 public:
  void sent(uint8_t tag, const TxFrameRec &rec) {
    _slot[tag] = rec;
    _live[tag] = true;
    ++_sent;
  }
  /* The record for a report's tag, or nullopt (counted as unmatched). */
  std::optional<TxFrameRec> match(uint8_t tag) {
    if (!_live[tag]) { ++_unmatched; return std::nullopt; }
    _live[tag] = false;
    ++_joined;
    return _slot[tag];
  }
  uint64_t sent_count() const { return _sent; }
  uint64_t joined() const { return _joined; }
  uint64_t unmatched() const { return _unmatched; }

 private:
  std::array<TxFrameRec, 256> _slot{};
  std::array<bool, 256> _live{};
  uint64_t _sent = 0, _joined = 0, _unmatched = 0;
};

}  // namespace stream_timing
}  // namespace devourer

#endif /* DEVOURER_STREAM_TELEMETRY_H */
