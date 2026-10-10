// Shared stdin framing for the stdin-driven stream demos (streamtx,
// duplex) and their headless regression self-test
// (StreamStdinSelftest).
//
// Centralises the two things that have to stay correct on every Windows
// toolchain, so there is a single source of truth instead of one copy per
// demo:
//
//   1. set_stdin_binary() — put stdin in binary mode so a 0x1A (Ctrl-Z, which
//      text-mode stdin treats as EOF) or a CRLF byte in the binary
//      <u32_le len><PSDU> stream isn't translated away. Gated on _WIN32, NOT
//      _MSC_VER: mingw/GCC defines _WIN32 but not _MSC_VER, yet still ships
//      _setmode. A _MSC_VER gate silently leaves mingw stdin in TEXT mode and
//      truncates the first PSDU ("short read on stdin (76/269)") before a
//      single frame is transmitted.
//
//   2. read_exact() — the byte reader, returning a tri-state so each caller
//      keeps its own short-read policy (the TX demo aborts on a truncated
//      record; the duplex demo just stops its TX thread and lets RX run on).
//
//   3. read_record() — one whole <u32_le len><body> record, which is what
//      every caller actually wanted. The little-endian assembly and the
//      zero/oversize check were re-typed in four demos before this existed,
//      and they had already drifted. Callers that need the length word before
//      the body — the duplex demo escapes to a control TLV on its top bit —
//      compose read_length() and read_body() instead.
//
// The result states are deliberately finer than "it failed": a producer that
// closed cleanly between records is an ordinary end of run, one that closed
// with a length word already written has lost a record, and a length out of
// range is a producer bug rather than a stream end. Each demo maps them to its
// own policy; none of them has to re-derive them.
//
// StreamStdinSelftest + tests/stream_stdin_test.cmake exercise this header
// headlessly (no libusb, no hardware), so a regression in the _WIN32 gate
// fails CI on the mingw job instead of only surfacing on a real radio.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

#if defined(_WIN32)
  #include <io.h>
  #include <fcntl.h>
#endif

namespace stream_stdin {

// Put stdin into binary mode. No-op off Windows (POSIX has no text mode).
inline void set_stdin_binary() {
#if defined(_WIN32)
  _setmode(_fileno(stdin), _O_BINARY);
#endif
}

// Put stdout into binary mode. Only the self-test's --gen path needs this, but
// it lives here so all the toolchain-gated _setmode logic stays in one place.
inline void set_stdout_binary() {
#if defined(_WIN32)
  _setmode(_fileno(stdout), _O_BINARY);
#endif
}

enum class ReadResult {
  Ok,     // got all n bytes
  Eof,    // clean stream close: 0 bytes read with EOF before any byte
  Short,  // stream ended mid-record (truncation)
};

// Read exactly n bytes from f into buf.
inline ReadResult read_exact(std::FILE *f, void *buf, std::size_t n) {
  std::size_t got = 0;
  auto *p = static_cast<std::uint8_t *>(buf);
  while (got < n) {
    std::size_t r = std::fread(p + got, 1, n - got, f);
    if (r == 0) {
      if (got == 0 && std::feof(f)) return ReadResult::Eof;
      return ReadResult::Short;
    }
    got += r;
  }
  return ReadResult::Ok;
}

// Read the 4-byte little-endian length prefix. `len` is untouched unless Ok.
inline ReadResult read_length(std::FILE *f, std::uint32_t &len) {
  std::uint8_t b[4];
  const ReadResult r = read_exact(f, b, sizeof(b));
  if (r == ReadResult::Ok)
    len = static_cast<std::uint32_t>(b[0]) |
          (static_cast<std::uint32_t>(b[1]) << 8) |
          (static_cast<std::uint32_t>(b[2]) << 16) |
          (static_cast<std::uint32_t>(b[3]) << 24);
  return r;
}

// Read `n` body bytes into `out`, sizing it to match. A zero-length body is Ok
// and leaves `out` empty — callers that forbid one check the length first.
inline ReadResult read_body(std::FILE *f, std::vector<std::uint8_t> &out,
                            std::size_t n) {
  out.resize(n);
  return n ? read_exact(f, out.data(), n) : ReadResult::Ok;
}

/* ---- the control escape -------------------------------------------------
 *
 * A length word with its top bit set is not a PSDU: the body is a control
 * TLV, <op:u8><args...>, bounded at kCtlMax rather than at the PSDU maximum.
 * The duplex demo introduced it for the adaptive link's live knobs; streamtx
 * honours it too. Opcodes are one namespace across the demos; a demo that
 * does not implement one ignores it (and says so in its stream.ctl event). */
constexpr uint32_t kCtlFlag = 0x80000000u;
constexpr size_t kCtlMax = 256;
enum CtlOp : uint8_t {
  kCtlSetPwr = 1,     /* <idx:u8>                    flat TXAGC override */
  kCtlSetRate = 2,    /* <spec:ascii>                live TX rate */
  kCtlSetChan = 3,    /* <ch:u8><offset:u8><width:u8> SetMonitorChannel */
  kCtlCaptureTs = 4,  /* <ns:u64 LE>  the producer's capture time of the NEXT
                       * data record, CLOCK_MONOTONIC ns on the same host. The
                       * TX measures capture->send_packet from it and carries
                       * that in the frame (src/StreamTelemetry.h). A stamp
                       * followed by another stamp, or by EOF, is dropped. */
};

/* Decode a kCtlCaptureTs body. False unless it is exactly op 4 + 8 bytes. */
inline bool parse_capture_ts(const std::vector<std::uint8_t> &ctl,
                             std::uint64_t &ns) {
  if (ctl.size() != 9 || ctl[0] != kCtlCaptureTs) return false;
  ns = 0;
  for (int i = 0; i < 8; ++i)
    ns |= static_cast<std::uint64_t>(ctl[1 + i]) << (8 * i);
  return true;
}

enum class RecordResult {
  Ok,          // a complete record is in `out`
  Eof,         // clean close: nothing at all where a record would have started
  Short,       // the stream ended part-way through a record
  EofMidBody,  // the length word arrived, then the stream closed before a byte
               // of its body — one record lost, cleanly
  BadLength,   // zero, or larger than the caller's bound
  Control,     // read_item only: a control TLV is in `ctl`, not a record
};

// Read one whole <u32_le len><body> record. `max` bounds the body. When
// `raw_len` is given it receives the length word verbatim even on BadLength,
// so a caller with its own convention in the high bits can inspect it.
inline RecordResult read_record(std::FILE *f, std::vector<std::uint8_t> &out,
                                std::size_t max,
                                std::uint32_t *raw_len = nullptr) {
  std::uint32_t len = 0;
  switch (read_length(f, len)) {
  case ReadResult::Eof:
    return RecordResult::Eof;
  case ReadResult::Short:
    return RecordResult::Short;
  case ReadResult::Ok:
    break;
  }
  if (raw_len)
    *raw_len = len;
  if (len == 0 || len > max)
    return RecordResult::BadLength;
  switch (read_body(f, out, len)) {
  case ReadResult::Ok:
    return RecordResult::Ok;
  case ReadResult::Eof:
    return RecordResult::EofMidBody;
  case ReadResult::Short:
    return RecordResult::Short;
  }
  return RecordResult::Short;  // unreachable; keeps every toolchain quiet
}

/* Read one stdin item: a data record (RecordResult::Ok, body in `out`) or a
 * control TLV (RecordResult::Control, <op><args> in `ctl`). Every other state
 * is read_record's. A control body outside [1, kCtlMax] is BadLength with the
 * raw length word in `raw_len`, so the caller can say which it was. */
inline RecordResult read_item(std::FILE *f, std::vector<std::uint8_t> &out,
                              std::vector<std::uint8_t> &ctl, std::size_t max,
                              std::uint32_t *raw_len = nullptr) {
  std::uint32_t len = 0;
  switch (read_length(f, len)) {
  case ReadResult::Eof:
    return RecordResult::Eof;
  case ReadResult::Short:
    return RecordResult::Short;
  case ReadResult::Ok:
    break;
  }
  if (raw_len)
    *raw_len = len;
  if (len & kCtlFlag) {
    const std::uint32_t clen = len & ~kCtlFlag;
    if (clen == 0 || clen > kCtlMax)
      return RecordResult::BadLength;
    switch (read_body(f, ctl, clen)) {
    case ReadResult::Ok:
      return RecordResult::Control;
    case ReadResult::Eof:
      return RecordResult::EofMidBody;
    case ReadResult::Short:
      return RecordResult::Short;
    }
  }
  if (len == 0 || len > max)
    return RecordResult::BadLength;
  switch (read_body(f, out, len)) {
  case ReadResult::Ok:
    return RecordResult::Ok;
  case ReadResult::Eof:
    return RecordResult::EofMidBody;
  case ReadResult::Short:
    return RecordResult::Short;
  }
  return RecordResult::Short;
}

}  // namespace stream_stdin
