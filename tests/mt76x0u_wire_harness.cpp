/* Drives the MT76x0U library against a chip model over a pipe, so its USB
 * transfer sequence can be logged and compared without hardware
 * (tests/mt76x0u_wire_model.py serves the model and writes the log).
 *
 *   mt76x0u_wire_harness FW_DIR [force] [once] [CH[+|-] ...]
 *
 * Runs open + bring-up (firmware upload kept if running, unless "force"),
 * then a channel set per argument ("36+" = HT40+, "40-" = HT40-; each set
 * runs twice, as the library defaults to, unless "once"), then close.
 *
 * Protocol, one line each way per transfer (hex payloads, "-" for none):
 *   -> C <type> <req> <wValue> <wIndex> <wLength> <out-data>
 *   <- <rc> <in-data>
 *   -> B <ep> <len> <timeout_ms> <out-data>
 *   <- <rc> <xfered> <in-data>
 *   -> M <label>                      (marker, no reply)
 * The library's own diagnostics go to stderr. Exit 0 when every step
 * succeeded, 1 otherwise (and the failing step is named on stderr). */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "internal.h"

namespace {

void hex_out(const uint8_t *p, int n) {
  if (n <= 0) {
    std::fputs("-", stdout);
    return;
  }
  for (int i = 0; i < n; i++)
    std::fprintf(stdout, "%02x", p[i]);
}

int hex_in(const char *s, uint8_t *out, int max) {
  if (!s || !std::strcmp(s, "-"))
    return 0;
  int n = 0;
  while (s[0] && s[1] && n < max) {
    unsigned v;
    if (std::sscanf(s, "%2x", &v) != 1)
      break;
    out[n++] = (uint8_t)v;
    s += 2;
  }
  return n;
}

char line[1 << 17];

bool read_reply() {
  if (!std::fgets(line, sizeof line, stdin)) {
    std::fprintf(stderr, "harness: model closed the pipe\n");
    std::exit(2);
  }
  line[std::strcspn(line, "\r\n")] = 0;
  return true;
}

int ctrl(void *, uint8_t type, uint8_t req, uint16_t val, uint16_t idx, uint8_t *data,
         uint16_t len) {
  const bool in = type & 0x80;
  std::fprintf(stdout, "C %02x %02x %04x %04x %u ", type, req, val, idx, len);
  hex_out(data, in ? 0 : len);
  std::fputc('\n', stdout);
  std::fflush(stdout);
  read_reply();
  int rc = 0;
  char hex[sizeof line];
  hex[0] = 0;
  std::sscanf(line, "%d %s", &rc, hex);
  if (in && rc > 0)
    hex_in(hex, data, len);
  return rc;
}

int bulk(void *, uint8_t ep, uint8_t *buf, int len, int *xfered, unsigned timeout_ms) {
  const bool in = ep & 0x80;
  std::fprintf(stdout, "B %02x %d %u ", ep, len, timeout_ms);
  hex_out(buf, in ? 0 : len);
  std::fputc('\n', stdout);
  std::fflush(stdout);
  read_reply();
  int rc = 0, n = 0;
  char hex[sizeof line];
  hex[0] = 0;
  std::sscanf(line, "%d %d %s", &rc, &n, hex);
  if (in && n > 0)
    n = hex_in(hex, buf, len);
  *xfered = n;
  return rc;
}

void marker(const char *label) {
  std::fprintf(stdout, "M %s\n", label);
  std::fflush(stdout);
}

} // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s FW_DIR [force] [CH[+|-] ...]\n", argv[0]);
    return 2;
  }
  const char *fw_dir = argv[1];
  int arg = 2, force = 0, once = 0;
  for (; arg < argc; arg++) {
    if (!std::strcmp(argv[arg], "force"))
      force = 1;
    else if (!std::strcmp(argv[arg], "once"))
      once = 1;
    else
      break;
  }
  static const mt0_io_hook hook = {ctrl, bulk, nullptr};
  const char *err = nullptr;

  marker("open");
  struct mt76x0u_dev *d = mt0_open_hooked(&hook, 0, &err);
  if (!d) {
    std::fprintf(stderr, "harness: open failed: %s\n", err ? err : "?");
    return 1;
  }
  mt76x0u_set_tune_twice(d, once ? 0 : 1);
  marker("bring-up");
  if (mt76x0u_bring_up(d, fw_dir, force)) {
    std::fprintf(stderr, "harness: bring-up failed\n");
    mt76x0u_close(d);
    return 1;
  }
  int rc = 0;
  for (; arg < argc; arg++) {
    std::string a = argv[arg];
    int ht40 = 0;
    if (!a.empty() && (a.back() == '+' || a.back() == '-')) {
      ht40 = a.back() == '+' ? 1 : -1;
      a.pop_back();
    }
    marker(("set_channel " + std::string(argv[arg])).c_str());
    if (mt76x0u_set_channel(d, (uint8_t)std::atoi(a.c_str()), ht40)) {
      std::fprintf(stderr, "harness: channel %s failed\n", argv[arg]);
      rc = 1;
    }
  }
  marker("close");
  mt76x0u_close(d);
  return rc;
}
