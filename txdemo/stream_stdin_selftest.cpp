// StreamStdinSelftest — headless regression test for txdemo/stream_stdin.h.
//
// The stdin-driven stream demos (StreamTxDemo, StreamDuplexDemo) read a binary
// <u32_le len><PSDU> stream from stdin. On Windows that only works if stdin is
// put into binary mode; the gate has to be `_WIN32` (not `_MSC_VER`) so it also
// fires under mingw/GCC. A regression there is invisible to a build-only CI job
// — it compiles fine and only corrupts bytes at runtime — so this binary
// exercises the exact set_stdin_binary() + read_exact() path the demos use, with
// no libusb and no radio.
//
//   StreamStdinSelftest --gen   writes the canonical stream to stdout (binary)
//   StreamStdinSelftest         reads that stream from stdin and round-trips it
//
// tests/stream_stdin_test.cmake pipes the first into the second. The canonical
// records deliberately contain 0x1A (Ctrl-Z = EOF to a text-mode read), 0x0D
// and 0x0A, so any text-mode translation truncates or mangles the stream and the
// reader reports FAIL with a non-zero exit.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "stream_stdin.h"

// Canonical self-test stream. --gen and the reader share this table, so a
// byte-for-byte mismatch on read means binary mode is broken. Total = 5 records,
// 20 body bytes (5+3+4+1+7) — kept in lockstep with the expected string in
// tests/stream_stdin_test.cmake.
static const std::vector<std::vector<uint8_t>> &canonical_records() {
  static const std::vector<std::vector<uint8_t>> recs = {
      {0x1A, 0x0D, 0x0A, 0x00, 0xFF},
      {0x41, 0x1A, 0x42},
      {0x0D, 0x0A, 0x0D, 0x0A},
      {0x1A},
      {0x00, 0x1A, 0x0D, 0x0A, 0x1A, 0x7F, 0x80},
  };
  return recs;
}

static void put_u32_le(uint8_t *p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

static int do_gen() {
  stream_stdin::set_stdout_binary();
  for (const auto &rec : canonical_records()) {
    uint8_t len_bytes[4];
    put_u32_le(len_bytes, static_cast<uint32_t>(rec.size()));
    std::fwrite(len_bytes, 1, sizeof(len_bytes), stdout);
    if (!rec.empty()) std::fwrite(rec.data(), 1, rec.size(), stdout);
  }
  std::fflush(stdout);
  return 0;
}

static int do_check() {
  stream_stdin::set_stdin_binary();
  const auto &expected = canonical_records();
  size_t got_records = 0, got_bytes = 0;
  for (const auto &exp : expected) {
    uint8_t len_bytes[4];
    auto r = stream_stdin::read_exact(stdin, len_bytes, sizeof(len_bytes));
    if (r != stream_stdin::ReadResult::Ok) {
      std::fprintf(stderr,
                   "stream_stdin_selftest: FAIL — len-prefix read for record %zu "
                   "returned %d (binary stdin likely broken: a 0x1A in a prior "
                   "record was read as EOF)\n",
                   got_records, static_cast<int>(r));
      return 2;
    }
    uint32_t len = static_cast<uint32_t>(len_bytes[0])
                 | (static_cast<uint32_t>(len_bytes[1]) << 8)
                 | (static_cast<uint32_t>(len_bytes[2]) << 16)
                 | (static_cast<uint32_t>(len_bytes[3]) << 24);
    if (len != exp.size()) {
      std::fprintf(stderr,
                   "stream_stdin_selftest: FAIL — record %zu length %u != "
                   "expected %zu (stream desynced; likely CRLF/Ctrl-Z "
                   "translation)\n",
                   got_records, len, exp.size());
      return 3;
    }
    std::vector<uint8_t> body(len);
    if (len) {
      r = stream_stdin::read_exact(stdin, body.data(), len);
      if (r != stream_stdin::ReadResult::Ok) {
        std::fprintf(stderr,
                     "stream_stdin_selftest: FAIL — body read for record %zu "
                     "returned %d\n",
                     got_records, static_cast<int>(r));
        return 4;
      }
    }
    if (body != exp) {
      std::fprintf(stderr,
                   "stream_stdin_selftest: FAIL — record %zu body differs from "
                   "source (text-mode translation corrupted the stream)\n",
                   got_records);
      return 5;
    }
    ++got_records;
    got_bytes += len;
  }
  // Confirm clean EOF right after the last record — no trailing corruption.
  uint8_t extra;
  if (stream_stdin::read_exact(stdin, &extra, 1) != stream_stdin::ReadResult::Eof) {
    std::fprintf(stderr,
                 "stream_stdin_selftest: FAIL — expected EOF after %zu records "
                 "but more bytes followed\n",
                 got_records);
    return 6;
  }
  std::fprintf(stdout, "stream_stdin_selftest: records=%zu bytes=%zu OK\n",
               got_records, got_bytes);
  std::fflush(stdout);
  return 0;
}

int main(int argc, char **argv) {
  if (argc > 1 && std::strcmp(argv[1], "--gen") == 0) return do_gen();
  return do_check();
}
