// Headless guard for kestrel::addr_cam_sta_entry, the station address-CAM
// encoder behind both the 8852B's addr_cam H2C and the 8852C's host write.
// The expected words are read-backs from live hardware, not hand-derived:
// what the 8852B firmware committed for its own H2C, and what an 8852C held
// after the host write. A layout change that breaks either fails here.
#include <cstdint>
#include <cstdio>

#include "kestrel/KestrelAddrCam.h"

namespace {

int failures = 0;

void expect(const char *what, uint32_t got, uint32_t want) {
  if (got != want) {
    std::printf("FAIL %s: got %08x want %08x\n", what, got, want);
    ++failures;
  }
}

} // namespace

int main() {
  const uint8_t bssid[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
  const uint8_t infra = kestrel::reg::MAC_AX_NET_TYPE_INFRA;

  // 8852B, firmware-committed entry for register_infra_sta (macid 1, BSSID
  // CAM 1), physical words 0..3 read through the indirect window.
  {
    const uint8_t own[6] = {0x02, 0x11, 0x22, 0x33, 0x44, 0x55};
    const auto w = kestrel::addr_cam_sta_entry(own, bssid, infra, 1, 1);
    expect("8852B w0", w[0], 0x03130005u);
    expect("8852B w1", w[1], 0x00000001u);
    expect("8852B w2", w[2], 0x33221102u);
    expect("8852B w3", w[3], 0x00025544u);
  }

  // 8852C, host-written entry at macid 63 (BSSID CAM 1), words 0..4 read back.
  {
    const uint8_t own[6] = {0x02, 0x12, 0x34, 0x56, 0x78, 0x9a};
    const auto w = kestrel::addr_cam_sta_entry(own, bssid, infra, 1, 63);
    expect("8852C w0", w[0], 0x03900005u);
    expect("8852C w1", w[1], 0x00000001u);
    expect("8852C w2", w[2], 0x56341202u);
    expect("8852C w3", w[3], 0x00029a78u);
    expect("8852C w4", w[4], 0x01000000u);
    expect("8852C w6", w[6], 63u | (1u << 14));
  }

  if (failures == 0)
    std::printf("kestrel_addr_cam: ok\n");
  return failures == 0 ? 0 : 1;
}
