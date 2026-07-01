// In-process hotplug test exercising `libusb_wrap_sys_device(fd)` — the
// libusb construction path Android takes when an app receives a fd from
// `UsbDeviceConnection.getFileDescriptor()`. Designed to catch regressions
// on the Android-style construction path without needing an Android
// device in the loop.
//
// SCOPE NOTE: this test exercises the `libusb_wrap_sys_device(fd)`
// codepath, but it does NOT reproduce the original Android hotplug bug
// fixed by 01cad75 (PR #42). Validated 2026-05-26 by building against
// pre-#42 master (8c87cd5): both sessions RX cleanly (iter1=243,
// iter2=241), same as post-#42. The Android gap evidently involves more
// than just the construction path — likely fd lifetime / USB reset
// semantics / Android-stack-specific teardown behavior we don't
// replicate with sysfs unbind/rebind. Treat this as a smoke check for
// the wrap_sys_device path, not a regression test for the specific
// hotplug bug. Genuine Android validation still requires PixelPilot or
// hx-esp32cam-fpv on a real device.
//
// What it does
//   1. Walk /sys/bus/usb/devices to find a device matching VID:PID
//      (default: 2357:0120, T2U Plus / RTL8821AU). Records its sysfs
//      path so we can sysfs-unbind/rebind it later.
//   2. Read its busnum + devnum from sysfs, open /dev/bus/usb/BBB/DDD
//      directly via plain open(2).
//   3. Construct a libusb handle from that fd via
//      `libusb_wrap_sys_device(ctx, fd, &handle)` — same path Android
//      takes from `UsbDeviceConnection.getFileDescriptor()`.
//   4. Run a regular `CreateRtlDevice` + `Init` for RX_SECONDS to count
//      RX hits. Then close.
//   5. Run an externally-triggered sysfs unbind+rebind so the chip
//      goes through a real USB reset cycle.
//   6. Re-find the device (it has a new devnum), open the new
//      /dev/bus/usb/BBB/DDD, wrap-construct a new libusb handle in the
//      SAME libusb context as session 1, run RX again, count hits.
//
// If session 1 RXs but session 2 is silent, we've reproduced the
// Android-style hotplug failure mode on Linux — which would be the
// first time. Use this binary as a regression check against future
// changes to HalModule's 8821 init.
//
// Build (against an existing master build with libWiFiDriver.a):
//
//   g++ -std=c++20 -O2 tests/in_process_hotplug_wrap.cpp \
//       -o /tmp/in_process_hotplug_wrap \
//       -I src -I hal build/libWiFiDriver.a \
//       $(pkg-config --cflags --libs libusb-1.0) -pthread
//
// Run (needs root for sysfs unbind + /dev/bus/usb open):
//
//   sudo /tmp/in_process_hotplug_wrap [vid] [pid]
//   sudo /tmp/in_process_hotplug_wrap 2357 0120     # explicit T2U Plus
//
// Configuration
//   VID / PID         positional argv if given; otherwise default
//                     2357:0120.
//   CHANNEL           env DEVOURER_CHANNEL (default 6, busy 2.4 GHz).
//   RX_SECONDS        env RX_SECONDS (default 6).

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <string>
#include <thread>
#include <unistd.h>
#include <libusb.h>

#include "logger.h"
#include "RxPacket.h"
#include "RtlUsbAdapter.h"
#include "WiFiDriver.h"

namespace {

struct UsbDeviceInfo {
  std::string sysfs_path;    // e.g. /sys/bus/usb/devices/1-3
  std::string sysfs_port;    // e.g. "1-3" — used for unbind/bind
  int busnum;
  int devnum;
};

// Walks /sys/bus/usb/devices/ for the first entry matching (vid, pid).
// Returns false on miss. Doesn't engage libusb — the whole point of this
// test is to NOT go through libusb_get_device_list / open_device_with_pid.
bool find_usb_device(uint16_t vid, uint16_t pid, UsbDeviceInfo* out) {
  FILE* pipe = popen("ls -1 /sys/bus/usb/devices/ 2>/dev/null", "r");
  if (!pipe) return false;
  char name[256];
  while (fgets(name, sizeof(name), pipe)) {
    size_t n = strlen(name);
    while (n && (name[n - 1] == '\n' || name[n - 1] == '\r')) name[--n] = 0;
    // Skip interface entries (contain a ':' like "1-3:1.0").
    if (strchr(name, ':')) continue;
    std::string sysfs = std::string("/sys/bus/usb/devices/") + name;
    auto read_hex16 = [](const std::string& path) -> int {
      FILE* f = fopen(path.c_str(), "r");
      if (!f) return -1;
      unsigned v = 0;
      int r = fscanf(f, "%x", &v);
      fclose(f);
      return r == 1 ? static_cast<int>(v) : -1;
    };
    auto read_int = [](const std::string& path) -> int {
      FILE* f = fopen(path.c_str(), "r");
      if (!f) return -1;
      int v = 0;
      int r = fscanf(f, "%d", &v);
      fclose(f);
      return r == 1 ? v : -1;
    };
    int v = read_hex16(sysfs + "/idVendor");
    int p = read_hex16(sysfs + "/idProduct");
    if (v != vid || p != pid) continue;
    int b = read_int(sysfs + "/busnum");
    int d = read_int(sysfs + "/devnum");
    if (b < 0 || d < 0) continue;
    out->sysfs_path = sysfs;
    out->sysfs_port = name;
    out->busnum = b;
    out->devnum = d;
    pclose(pipe);
    return true;
  }
  pclose(pipe);
  return false;
}

int open_usbfs(int busnum, int devnum) {
  char path[64];
  snprintf(path, sizeof(path), "/dev/bus/usb/%03d/%03d", busnum, devnum);
  int fd = open(path, O_RDWR | O_CLOEXEC);
  if (fd < 0) {
    fprintf(stderr, "open %s: %s\n", path, strerror(errno));
  }
  return fd;
}

std::atomic<int> g_rx{0};
void on_pkt(const Packet& /*p*/) { g_rx.fetch_add(1); }

int run_session(libusb_context* ctx, uint16_t vid, uint16_t pid,
                int channel, int rx_seconds, const char* label) {
  auto logger = std::make_shared<Logger>();

  UsbDeviceInfo info{};
  if (!find_usb_device(vid, pid, &info)) {
    fprintf(stderr, "[%s] no /sys/bus/usb device with VID:PID %04x:%04x\n",
            label, vid, pid);
    return -1;
  }
  fprintf(stdout, "[%s] sysfs=%s bus=%d dev=%d\n", label,
          info.sysfs_path.c_str(), info.busnum, info.devnum);

  int fd = open_usbfs(info.busnum, info.devnum);
  if (fd < 0) return -2;

  libusb_device_handle* h = nullptr;
  int rc = libusb_wrap_sys_device(ctx, static_cast<intptr_t>(fd), &h);
  if (rc < 0 || h == nullptr) {
    fprintf(stderr, "[%s] libusb_wrap_sys_device rc=%d\n", label, rc);
    close(fd);
    return -3;
  }
  // libusb takes a reference but the fd lifetime is ours — keep it open.
  if (libusb_kernel_driver_active(h, 0) > 0) libusb_detach_kernel_driver(h, 0);

  // Note: we deliberately do NOT call libusb_reset_device here. Android
  // doesn't reset on its wrap_sys_device path either — the kernel xhci
  // re-enumeration on replug is the only reset the chip gets.
  rc = libusb_claim_interface(h, 0);
  if (rc != 0) {
    fprintf(stderr, "[%s] claim_interface rc=%d\n", label, rc);
    libusb_close(h);
    close(fd);
    return -4;
  }

  int before = g_rx.load();
  WiFiDriver drv(logger);
  auto dev = drv.CreateRtlDevice(h);

  std::thread t([&]{
    dev->Init(on_pkt, SelectedChannel{
      .Channel = static_cast<uint8_t>(channel),
      .ChannelOffset = 0,
      .ChannelWidth = CHANNEL_WIDTH_20,
    });
  });

  std::this_thread::sleep_for(std::chrono::seconds(rx_seconds));
  int after = g_rx.load();
  int hits = after - before;
  fprintf(stdout, "[%s] hits=%d (total=%d)\n", label, hits, after);
  fflush(stdout);

  // Stop the Init loop and tear down. Same caveat as the older test —
  // dev->Init may continue spinning briefly on bulk errors after close.
  dev->should_stop = true;
  libusb_release_interface(h, 0);
  libusb_close(h);
  close(fd);
  if (t.joinable()) t.join();
  return hits;
}

bool wait_for_reenum(uint16_t vid, uint16_t pid, int seconds) {
  char cmd[64];
  snprintf(cmd, sizeof(cmd), "lsusb | grep -q %04x:%04x", vid, pid);
  for (int i = 0; i < seconds * 2; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (system(cmd) == 0) return true;
  }
  return false;
}

}  // namespace

int main(int argc, char** argv) {
  uint16_t vid = 0x2357;
  uint16_t pid = 0x0120;
  if (argc >= 3) {
    vid = static_cast<uint16_t>(strtoul(argv[1], nullptr, 16));
    pid = static_cast<uint16_t>(strtoul(argv[2], nullptr, 16));
  }
  int channel = 6;
  if (const char* c = getenv("DEVOURER_CHANNEL")) channel = atoi(c);
  int rx_seconds = 6;
  if (const char* s = getenv("RX_SECONDS")) rx_seconds = atoi(s);

  fprintf(stdout, "=== libusb_wrap_sys_device hotplug test ===\n");
  fprintf(stdout, "target: %04x:%04x  channel %d  RX %d s/session\n",
          vid, pid, channel, rx_seconds);

  libusb_context* ctx = nullptr;
  if (libusb_init(&ctx) != 0) {
    fprintf(stderr, "libusb_init failed\n");
    return 1;
  }
  libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);

  fprintf(stdout, "\n--- session 1 (cold) ---\n");
  int h1 = run_session(ctx, vid, pid, channel, rx_seconds, "iter1");

  // Find the sysfs port to unbind. find_usb_device returns it, but we
  // already dropped that info. Re-query.
  UsbDeviceInfo info{};
  if (!find_usb_device(vid, pid, &info)) {
    fprintf(stderr, "device disappeared between session 1 and replug\n");
    libusb_exit(ctx);
    return 1;
  }
  fprintf(stdout,
          "\n--- triggering sysfs unbind+rebind %s (full USB reset cycle) ---\n",
          info.sysfs_port.c_str());
  char unbind_cmd[256], bind_cmd[256];
  snprintf(unbind_cmd, sizeof(unbind_cmd),
           "echo %s > /sys/bus/usb/drivers/usb/unbind",
           info.sysfs_port.c_str());
  snprintf(bind_cmd, sizeof(bind_cmd),
           "echo %s > /sys/bus/usb/drivers/usb/bind",
           info.sysfs_port.c_str());
  if (system(unbind_cmd) != 0)
    fprintf(stderr, "unbind exit nonzero — continuing\n");
  std::this_thread::sleep_for(std::chrono::seconds(1));
  if (system(bind_cmd) != 0)
    fprintf(stderr, "bind exit nonzero — continuing\n");
  if (!wait_for_reenum(vid, pid, 10)) {
    fprintf(stderr, "device did not re-enumerate after 10s\n");
    libusb_exit(ctx);
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));

  fprintf(stdout, "\n--- session 2 (post-replug, same libusb ctx) ---\n");
  int h2 = run_session(ctx, vid, pid, channel, rx_seconds, "iter2");

  fprintf(stdout, "\n=== summary ===\n");
  fprintf(stdout, "iter1 hits=%d  iter2 hits=%d  delta=%+d\n",
          h1, h2, h2 - h1);
  if (h1 > 0 && h2 == 0) {
    fprintf(stdout,
            "VERDICT: Android-style hotplug regression — iter2 RX silent.\n"
            "         If running against a pre-#42 master, this is the bug.\n"
            "         If running against post-#42 master, this is a regression.\n");
  } else if (h1 > 0 && h2 > 0) {
    fprintf(stdout,
            "VERDICT: hotplug works under wrap_sys_device — both sessions RX.\n");
  } else if (h1 == 0) {
    fprintf(stdout,
            "VERDICT: iter1 RX silent — channel may be quiet, retry on a\n"
            "         busier channel (DEVOURER_CHANNEL=1|6|11).\n");
  }

  libusb_exit(ctx);
  return 0;
}
