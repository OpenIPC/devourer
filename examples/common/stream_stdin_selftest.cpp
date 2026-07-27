// StreamStdinSelftest — headless regression test for examples/common/stream_stdin.h.
//
// The stdin-driven stream demos (streamtx, duplex) read a binary
// <u32_le len><PSDU> stream from stdin. On Windows that only works if stdin is
// put into binary mode; the gate has to be `_WIN32` (not `_MSC_VER`) so it also
// fires under mingw/GCC. A regression there is invisible to a build-only CI job
// — it compiles fine and only corrupts bytes at runtime — so this binary
// exercises the exact set_stdin_binary() + read_record() path the demos use,
// with no libusb and no radio. It also pins what each record state means,
// since the four demos map those states onto four different policies.
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

// Bound for read_record. Comfortably above the canonical records, so a
// BadLength here means the stream desynced, never that the cap was too tight.
static const std::size_t kMaxRecord = 4096;

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
    // read_record is the composed path every demo now uses: the length
    // assembly and the range check live in the header rather than four times
    // over, so this exercises what the demos actually run.
    std::vector<uint8_t> body;
    uint32_t len = 0;
    const auto r = stream_stdin::read_record(stdin, body, kMaxRecord, &len);
    if (r == stream_stdin::RecordResult::BadLength) {
      std::fprintf(stderr,
                   "stream_stdin_selftest: FAIL — record %zu length %u out of "
                   "range (stream desynced; likely CRLF/Ctrl-Z translation)\n",
                   got_records, len);
      return 3;
    }
    if (r != stream_stdin::RecordResult::Ok) {
      std::fprintf(stderr,
                   "stream_stdin_selftest: FAIL — record %zu returned %d "
                   "(binary stdin likely broken: a 0x1A in a prior record was "
                   "read as EOF)\n",
                   got_records, static_cast<int>(r));
      return 2;
    }
    if (len != exp.size()) {
      std::fprintf(stderr,
                   "stream_stdin_selftest: FAIL — record %zu length %u != "
                   "expected %zu (stream desynced; likely CRLF/Ctrl-Z "
                   "translation)\n",
                   got_records, len, exp.size());
      return 4;
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

// Every demo maps read_record's states onto its own policy — streamtx warns on
// one and exits 2 on another, txdemo counts a lost shard, duplex stops its TX
// thread and leaves RX running. So the states have to mean exactly what they
// say. Driven through tmpfile() rather than the pipe, since these are the cases
// a well-formed stream never produces.
static int check_state(const char *what, const std::vector<uint8_t> &bytes,
                       std::size_t max, stream_stdin::RecordResult want,
                       uint32_t want_len) {
  std::FILE *f = std::tmpfile();
  if (!f) {
    std::fprintf(stderr, "stream_stdin_selftest: tmpfile() failed\n");
    return 7;
  }
  if (!bytes.empty()) std::fwrite(bytes.data(), 1, bytes.size(), f);
  std::rewind(f);
  std::vector<uint8_t> body;
  uint32_t len = 0;
  const auto got = stream_stdin::read_record(f, body, max, &len);
  std::fclose(f);
  if (got != want) {
    std::fprintf(stderr,
                 "stream_stdin_selftest: FAIL — %s returned %d, expected %d\n",
                 what, static_cast<int>(got), static_cast<int>(want));
    return 7;
  }
  if (want_len && len != want_len) {
    std::fprintf(stderr,
                 "stream_stdin_selftest: FAIL — %s reported length %u, "
                 "expected %u\n", what, len, want_len);
    return 7;
  }
  return 0;
}

static int do_states() {
  using R = stream_stdin::RecordResult;
  auto rec = [](uint32_t len, std::size_t body_bytes) {
    std::vector<uint8_t> v(4);
    put_u32_le(v.data(), len);
    v.insert(v.end(), body_bytes, 0x1A);  // 0x1A: EOF to a text-mode read
    return v;
  };
  struct Case {
    const char *what;
    std::vector<uint8_t> bytes;
    std::size_t max;
    R want;
    uint32_t want_len;
  };
  const std::vector<Case> cases = {
      {"a complete record", rec(3, 3), 16, R::Ok, 3},
      {"nothing at all", {}, 16, R::Eof, 0},
      {"a truncated length prefix", {0x05, 0x00}, 16, R::Short, 0},
      {"a length with no body at all", rec(4, 0), 16, R::EofMidBody, 4},
      {"a body cut short", rec(8, 3), 16, R::Short, 8},
      {"a zero length", rec(0, 0), 16, R::BadLength, 0},
      {"a length past the bound", rec(99, 0), 16, R::BadLength, 99},
  };
  for (const auto &c : cases) {
    const int rc = check_state(c.what, c.bytes, c.max, c.want, c.want_len);
    if (rc) return rc;
  }

  /* The duplex demo escapes to a control TLV on the length's top bit, and
   * bounds that body at 256 rather than at its PSDU maximum — so it reads the
   * length, masks it, and only then reads the body. That composition is the
   * reason read_length and read_body are public at all, and nothing else
   * covers it: a record whose length has the top bit set must arrive
   * verbatim, and must not be mistaken for an oversize PSDU. */
  {
    std::vector<uint8_t> bytes(4);
    const uint32_t escaped = 0x80000000u | 3u;
    put_u32_le(bytes.data(), escaped);
    const std::vector<uint8_t> want = {0x01, 0x1A, 0x0D};
    bytes.insert(bytes.end(), want.begin(), want.end());
    std::FILE *f = std::tmpfile();
    if (!f) {
      std::fprintf(stderr, "stream_stdin_selftest: tmpfile() failed\n");
      return 7;
    }
    std::fwrite(bytes.data(), 1, bytes.size(), f);
    std::rewind(f);
    uint32_t len = 0;
    std::vector<uint8_t> body;
    const bool ok =
        stream_stdin::read_length(f, len) == stream_stdin::ReadResult::Ok &&
        (len & 0x80000000u) != 0 &&
        stream_stdin::read_body(f, body, len & 0x7fffffffu) ==
            stream_stdin::ReadResult::Ok &&
        body == want;
    std::fclose(f);
    if (!ok) {
      std::fprintf(stderr, "stream_stdin_selftest: FAIL — the control-opcode "
                           "escape (top length bit) did not round-trip\n");
      return 7;
    }
  }
  std::fprintf(stdout, "stream_stdin_selftest: %zu record states OK\n",
               cases.size());
  return 0;
}

int main(int argc, char **argv) {
  if (argc > 1 && std::strcmp(argv[1], "--gen") == 0) return do_gen();
  if (const int rc = do_states()) return rc;
  return do_check();
}
