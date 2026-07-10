// reglat.cpp — register round-trip latency of the transport: USB vendor-control
// transfer vs PCIe BAR2 MMIO. Times RtlAdapter::rtw_read32 in a tight loop — the
// SAME call on both buses, so it's a fair USB-vs-PCIe comparison of the per-op
// latency and jitter that bound AdjustBeaconTimingFine (read-modify-write of the
// TSF) and hence the closed-loop uplink timing advance. No bring-up: the read
// works right after claim (USB) / vfio-map (PCIe).
//
//   USB:  sudo ./build/reglat                    (DEVOURER_PID/VID pick the adapter)
//   PCIe: sudo DEVOURER_PCIE_BDF=0000:01:00.0 ./build/reglat   (vfio-bind first:
//         sudo tests/pcie_vfio_bind.sh 0000:01:00.0)
//
// Bench numbers (this rig): USB 8822EU ~66 us/op (jitter p99-p50 ~48 us); PCIe
// 8821CE MMIO ~2.2 us/op (jitter ~0.09 us) — ~30x faster, ~600x lower jitter.
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>
#include <libusb.h>
#include "RtlAdapter.h"
#include "UsbOpen.h"
#include "logger.h"
#if defined(DEVOURER_HAVE_PCIE)
#include "PcieTransport.h"
#endif

static int N = 3000;
static constexpr uint32_t kReg = 0x00F0;  // SYS_CFG1 — safe read-only, any state

static void report(const char *what, RtlAdapter &a) {
  std::vector<double> us(N);
  for (int i = 0; i < N; i++) {
    auto t0 = std::chrono::steady_clock::now();
    volatile uint32_t v = a.rtw_read32(kReg);
    (void)v;
    us[i] = std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - t0)
                .count();
  }
  std::sort(us.begin(), us.end());
  double sum = 0;
  for (double x : us) sum += x;
  auto q = [&](double p) { return us[(int)(p * N)]; };
  printf("%s rtw_read32(0x%04x) x%d: mean=%.3f p50=%.3f p90=%.3f p99=%.3f "
         "min=%.3f max=%.3f us  jitter(p99-p50)=%.3f\n",
         what, kReg, N, sum / N, q(.50), q(.90), q(.99), us.front(), us.back(),
         q(.99) - q(.50));
}

int main() {
  auto lg = std::make_shared<Logger>();
  if (const char *n = std::getenv("DEVOURER_REGLAT_N")) {
    int nn = atoi(n);  // (not std::max: <windows.h> defines a max() macro on MSVC)
    N = nn > 100 ? nn : 100;
  }

#if defined(DEVOURER_HAVE_PCIE)
  if (const char *bdf = std::getenv("DEVOURER_PCIE_BDF")) {
    auto t = devourer::PcieTransport::Open(bdf, lg);
    if (!t) {
      fprintf(stderr, "pcie open failed for %s (vfio-bound?)\n", bdf);
      return 1;
    }
    RtlAdapter a(t, lg, {});
    report("PCIe MMIO", a);
    return 0;
  }
#endif

  libusb_context *ctx = nullptr;
  libusb_init(&ctx);
  libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
  uint16_t vid = 0x0bda, pid = 0;
  if (const char *v = std::getenv("DEVOURER_VID")) vid = (uint16_t)strtoul(v, 0, 0);
  if (const char *p = std::getenv("DEVOURER_PID")) pid = (uint16_t)strtoul(p, 0, 0);
  auto *h = libusb_open_device_with_vid_pid(ctx, vid, pid);
  if (!h) {
    fprintf(stderr, "usb open %04x:%04x failed (set DEVOURER_PID)\n", vid, pid);
    return 1;
  }
  std::shared_ptr<devourer::UsbDeviceLock> lk;
  if (devourer::claim_interface_then_reset(h, 0, lg, true, lk) != 0) return 1;
  RtlAdapter a(h, lg, ctx, lk, {});
  report("USB ctrl-xfer", a);
  return 0;
}
