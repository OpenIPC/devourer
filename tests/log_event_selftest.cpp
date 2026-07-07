/* Headless guard for the JSONL event emitter (src/Event.h) — the machine
 * event stream every test script parses (docs/logging.md). Checks the
 * '{"ev":"name",' first-field serialization guarantee, JSON string escaping,
 * number/bool/null/hex/array field forms, the inline->scratch spill path,
 * and the 64 KiB truncation cap. A regression here silently corrupts every
 * consumer, so it fails `ctest` instead. */
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>

#include "Event.h"

using devourer::Ev;
using devourer::EventSink;

static int g_fail = 0;

/* Capture one emission through a tmpfile-backed sink. */
struct Capture {
  std::FILE *f;
  EventSink sink;
  Capture() : f(std::tmpfile()) { sink.configure(f); }
  ~Capture() { std::fclose(f); }
  std::string text() {
    std::fflush(f);
    long n = std::ftell(f);
    std::rewind(f);
    std::string s((size_t)n, '\0');
    if (std::fread(s.data(), 1, s.size(), f) != s.size())
      s.clear();
    return s;
  }
};

static void expect_eq(const char *what, const std::string &got,
                      const std::string &want) {
  if (got == want)
    return;
  ++g_fail;
  std::printf("FAIL: %s\n  got:  %s\n  want: %s\n", what, got.c_str(),
              want.c_str());
}

static void expect_true(const char *what, bool ok) {
  if (ok)
    return;
  ++g_fail;
  std::printf("FAIL: %s\n", what);
}

int main() {
  /* First-field guarantee: exact '{"ev":"name"' prefix, one line, LF end. */
  {
    Capture c;
    Ev(c.sink, "rx.txhit").f("hits", 3).f("total_rx", 10);
    expect_eq("basic event", c.text(),
              "{\"ev\":\"rx.txhit\",\"hits\":3,\"total_rx\":10}\n");
  }

  /* No fields. */
  {
    Capture c;
    Ev(c.sink, "hop.done").emit();
    expect_eq("empty event", c.text(), "{\"ev\":\"hop.done\"}\n");
  }

  /* Numbers: negatives, unsigned 64-bit, double, NaN -> null, bool, null. */
  {
    Capture c;
    Ev(c.sink, "n")
        .f("i", -42)
        .f("u", 18446744073709551615ull)
        .f("d", 2.5)
        .f("nan", std::numeric_limits<double>::quiet_NaN())
        .f("b", true)
        .f("z", nullptr);
    expect_eq("numbers", c.text(),
              "{\"ev\":\"n\",\"i\":-42,\"u\":18446744073709551615,\"d\":2.5,"
              "\"nan\":null,\"b\":true,\"z\":null}\n");
  }

  /* String escaping. */
  {
    Capture c;
    /* "\x01" split from "f" — otherwise C lexes the hex escape as \x1f */
    Ev(c.sink, "s").f("v", std::string_view("a\"b\\c\nd\te\x01" "f"));
    expect_eq("escaping", c.text(),
              "{\"ev\":\"s\",\"v\":\"a\\\"b\\\\c\\nd\\te\\u0001f\"}\n");
  }

  /* Hex register form + byte-blob hex + arrays. */
  {
    Capture c;
    const uint8_t blob[] = {0xde, 0xad, 0x0f};
    const int chains[] = {-52, -60, 0, 0};
    const double evm[] = {-28.5, -30.0};
    Ev(c.sink, "x")
        .hexf("addr", 0x808, 4)
        .hexf("val", 0x2a)
        .hex("body", blob, sizeof(blob))
        .arr("rssi", chains, 4)
        .arr("evm", evm, 2);
    expect_eq("hex+arrays", c.text(),
              "{\"ev\":\"x\",\"addr\":\"0x0808\",\"val\":\"0x2a\","
              "\"body\":\"dead0f\",\"rssi\":[-52,-60,0,0],"
              "\"evm\":[-28.5,-30]}\n");
  }

  /* Spill path: a body larger than the inline buffer must still emit one
   * well-formed line. */
  {
    Capture c;
    uint8_t big[3000];
    for (size_t i = 0; i < sizeof(big); i++)
      big[i] = (uint8_t)i;
    Ev(c.sink, "rx.frame").f("len", 3000).hex("body", big, sizeof(big));
    std::string out = c.text();
    expect_true("spill: single line",
                out.size() > 6000 && out.back() == '\n' &&
                    out.find('\n') == out.size() - 1);
    expect_true("spill: prefix",
                out.rfind("{\"ev\":\"rx.frame\",\"len\":3000,\"body\":\"", 0) ==
                    0);
    expect_true("spill: closed", out.substr(out.size() - 3) == "\"}\n");
    /* Second event on the same thread reuses the scratch cleanly. */
    Ev(c.sink, "rx.frame").f("len", 1);
    expect_true("scratch reuse",
                c.text().find("{\"ev\":\"rx.frame\",\"len\":1}\n") !=
                    std::string::npos);
  }

  /* Truncation cap: an over-64KiB field is dropped (-> null) and the line
   * carries "truncated":true, still valid JSON. */
  {
    Capture c;
    static uint8_t huge[70 * 1024];
    std::memset(huge, 0xab, sizeof(huge));
    Ev(c.sink, "big").f("pre", 1).hex("body", huge, sizeof(huge)).f("post", 2);
    std::string out = c.text();
    expect_true("truncation: capped", out.size() < Ev::kMaxLine + 2);
    expect_true("truncation: marker",
                out.find("\"truncated\":true}") != std::string::npos);
    expect_true("truncation: dropped field -> null",
                out.find("\"body\":null") != std::string::npos);
    expect_true("truncation: prefix survives",
                out.rfind("{\"ev\":\"big\",\"pre\":1,", 0) == 0);
  }

  /* Disabled sink emits nothing. */
  {
    Capture c;
    c.sink.disable();
    Ev(c.sink, "off").f("k", 1);
    expect_eq("disabled sink", c.text(), "");
  }

  if (g_fail) {
    std::printf("log_event_selftest: %d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("log_event_selftest: all checks passed\n");
  return 0;
}
