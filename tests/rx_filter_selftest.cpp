/* Headless guard for src/RxFilter.h. */
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

  /* One MACID: a beamformee and an ACK responder conflict under
   * own_addr_only only when their addresses differ. */
  const uint8_t bfee[6] = {0x00, 0xe0, 0x4c, 0x88, 0x22, 0xce};
  const devourer::MacAddr other{{0x02, 0x12, 0x34, 0x56, 0x78, 0x9a}};
  const devourer::MacAddr same{{0x00, 0xe0, 0x4c, 0x88, 0x22, 0xce}};
  DeviceConfig both;
  both.rx.own_addr_only = true;
  both.bf.beamformee_of = other;
  both.rx.ack_responder = other;
  check(devourer::own_addr_identity_conflict(both, bfee),
        "own_addr_only: beamformee + responder on another address conflict");
  both.rx.ack_responder = same;
  check(!devourer::own_addr_identity_conflict(both, bfee),
        "own_addr_only: responder on the beamformee address is fine");
  both.rx.ack_responder = other;
  both.rx.own_addr_only = false;
  check(!devourer::own_addr_identity_conflict(both, bfee),
        "promiscuous: no conflict");
  both.rx.own_addr_only = true;
  both.bf.beamformee_of.reset();
  check(!devourer::own_addr_identity_conflict(both, bfee),
        "no beamformee: no conflict");

  if (g_fail) {
    std::printf("rx_filter_selftest: %d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("rx_filter_selftest: OK\n");
  return 0;
}
