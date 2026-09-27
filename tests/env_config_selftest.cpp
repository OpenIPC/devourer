/* Headless guard for examples/common/env_config.cpp - the demos' env-var to
 * DeviceConfig mapping.
 *
 * One knob today, DEVOURER_TX_RETRY_LIMIT, the one env_config parses
 * strictly: the whole value must be one number (base auto-detect,
 * surrounding whitespace allowed), clamped to the documented 0..63. Anything
 * else warns and reads as unset, i.e. the library default of 0 - never as
 * the number a prefix parse would find in it.
 */
#include <cstdio>
#include <cstdlib>

#include "env_config.h"

namespace {

int g_fail = 0;

void check(bool ok, const char* what) {
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    g_fail++;
  }
}

void set_env(const char* name, const char* value) {
#ifdef _WIN32
  _putenv_s(name, value ? value : "");   /* "" removes it */
#else
  if (value)
    setenv(name, value, 1);
  else
    unsetenv(name);
#endif
}

int retry_limit_for(const char* value) {
  set_env("DEVOURER_TX_RETRY_LIMIT", value);
  return devourer_config_from_env().tx.retry_limit;
}

void test_retry_limit() {
  check(retry_limit_for(nullptr) == 0,
        "DEVOURER_TX_RETRY_LIMIT unset -> the library default, 0");
  check(retry_limit_for("5") == 5, "DEVOURER_TX_RETRY_LIMIT=5 -> 5");
  check(retry_limit_for("0x7") == 7,
        "DEVOURER_TX_RETRY_LIMIT=0x7 keeps env_long's base auto-detect");

  /* The documented grammar is 0..63, clamped. */
  check(retry_limit_for("100") == 63, "DEVOURER_TX_RETRY_LIMIT=100 clamps to 63");
  check(retry_limit_for("-3") == 0, "DEVOURER_TX_RETRY_LIMIT=-3 clamps to 0");

  /* Surrounding whitespace is isspace()'s set, \r included - the same
   * strings mt7612uprobe txs accepts. */
  check(retry_limit_for("5\r") == 5,
        "DEVOURER_TX_RETRY_LIMIT=\"5\\r\" (trailing CR) reads as 5");
  check(retry_limit_for(" 5 ") == 5,
        "DEVOURER_TX_RETRY_LIMIT=\" 5 \" (surrounding spaces) reads as 5");

  /* Not a number: rejected, so the default stands. "5x" is the case a prefix
   * parse gets wrong (it would read 5); whitespace only is the case where
   * skipping trailing whitespace before checking for digits would accept an
   * empty number. */
  check(retry_limit_for("5x") == 0,
        "DEVOURER_TX_RETRY_LIMIT=5x (trailing garbage) is rejected, not read as 5");
  check(retry_limit_for("x") == 0, "DEVOURER_TX_RETRY_LIMIT=x is rejected");
  check(retry_limit_for(" ") == 0,
        "DEVOURER_TX_RETRY_LIMIT=\" \" (whitespace only) is rejected");
  check(retry_limit_for("\t") == 0,
        "DEVOURER_TX_RETRY_LIMIT=\"\\t\" (whitespace only) is rejected");

  set_env("DEVOURER_TX_RETRY_LIMIT", nullptr);
}

}  // namespace

int main() {
  test_retry_limit();
  if (g_fail) {
    std::printf("env_config_selftest: %d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("env_config_selftest: OK\n");
  return 0;
}
