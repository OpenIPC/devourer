/* Headless guard for src/RxFilter.h: the monitor RCR and RXFLTMAP1 values the
 * two opt-in RX filter knobs (DeviceConfig::Rx::drop_ctrl, own_addr_only)
 * produce from each family's monitor base RCR. Defaults must leave both
 * registers exactly as the monitor setup has always written them. */
#include <cstdio>

#include "RxFilter.h"

namespace {

int g_fail = 0;

void check(bool ok, const char *what) {
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    g_fail++;
  }
}

} // namespace

int main() {
  using devourer::DeviceConfig;
  /* Monitor base RCRs: Jaguar3 (HalJaguar3::monitor_rx_cfg), Jaguar2
   * (HalJaguar2::enable_rx), RTL8733B (configure_monitor_rx). */
  const unsigned base[] = {0xF410400Fu, 0xF000002Fu, 0x90000001u};

  DeviceConfig::Rx off;
  for (unsigned b : base)
    check(devourer::monitor_rcr(b, off) == b, "knobs off: RCR unchanged");
  check(devourer::monitor_ctrl_map(off) == 0xFFFF, "knobs off: RXFLTMAP1 open");

  DeviceConfig::Rx own;
  own.own_addr_only = true;
  check(devourer::monitor_rcr(0xF410400Fu, own) == 0xF410400Eu,
        "own_addr_only on Jaguar3: AAP cleared, nothing else");
  check(devourer::monitor_rcr(0xF000002Fu, own) == 0xF000002Eu,
        "own_addr_only on Jaguar2: AAP cleared, nothing else");
  check(devourer::monitor_rcr(0x90000001u, own) == 0x9000000Eu,
        "own_addr_only on the RTL8733B: AAP cleared, APM|AM|AB set");
  check(devourer::monitor_ctrl_map(own) == 0xFFFF,
        "own_addr_only leaves RXFLTMAP1 open");

  DeviceConfig::Rx ctrl;
  ctrl.drop_ctrl = true;
  check(devourer::monitor_ctrl_map(ctrl) == 0x0000, "drop_ctrl closes RXFLTMAP1");
  for (unsigned b : base)
    check(devourer::monitor_rcr(b, ctrl) == b, "drop_ctrl leaves RCR unchanged");

  if (g_fail) {
    std::printf("rx_filter_selftest: %d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("rx_filter_selftest: OK\n");
  return 0;
}
