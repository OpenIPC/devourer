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
//   2. read_exact() — the length-prefixed record reader, returning a tri-state
//      so each caller keeps its own short-read policy (the TX demo aborts on a
//      truncated record; the duplex demo just stops its TX thread and lets RX
//      run on).
//
// StreamStdinSelftest + tests/stream_stdin_test.cmake exercise this header
// headlessly (no libusb, no hardware), so a regression in the _WIN32 gate
// fails CI on the mingw job instead of only surfacing on a real radio.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>

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

}  // namespace stream_stdin
