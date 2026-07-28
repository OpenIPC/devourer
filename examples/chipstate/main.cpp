/* chipstate — read a chip's registers WITHOUT touching it.
 *
 * Every other entry point into this driver reconfigures the chip on the way in:
 * `claim_interface_then_reset` re-enumerates it, `Init`/`InitWrite` re-run the
 * power sequence, reload the BB/RF/MAC tables and re-calibrate. That is exactly
 * what you must not do when the question is "what state did the previous
 * session leave this chip in?" — the act of looking destroys the evidence.
 *
 * So this tool: open, claim, DO NOT reset, construct the device (chip identity
 * resolves from SYS_CFG2 at construction, no bring-up needed), dump, exit. The
 * output is the DEVOURER_DUMP_CANARY format, so two dumps diff directly:
 *
 *   sudo build/chipstate --pid 0x8812 > degraded.canary
 *   # ... VBUS power-cycle the adapter ...
 *   sudo build/chipstate --pid 0x8812 > healthy.canary
 *   python3 tests/canary_diff.py degraded.canary healthy.canary --strict
 *
 * Reading a powered-down chip returns garbage or fails — that is a real answer
 * about the chip, not a tool error, so it is reported rather than hidden.
 *
 * Note --init: it runs a normal bring-up before dumping, which is what you want
 * for a healthy-reference dump on a chip that has just been power-cycled and is
 * therefore not configured at all.
 */
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <memory>

#if __has_include(<libusb.h>)
#include <libusb.h>
#else
#include <libusb-1.0/libusb.h>
#endif

#include "DeviceSession.h"
#include "IRtlDevice.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "logger.h"
#include "SignalStop.h"

namespace {

/* Same list the other demos' open loop iterates; --pid narrows to one. */
const uint16_t kRealtekPids[] = {0x8812, 0x8813, 0x881a, 0x0811, 0xa811,
                                 0x0820, 0x0821, 0x8822, 0x0120, 0x012d,
                                 0xb82c, 0xc811, 0xc812, 0xa81a};

struct Args {
  uint16_t vid = 0x0bda;
  int pid = -1;
  int channel = 6;
  bool init = false;
};

void usage() {
  std::fprintf(stderr,
               "usage: chipstate [--vid 0xNNNN] [--pid 0xNNNN] [--init] "
               "[--channel N]\n"
               "  default: attach read-only, no USB reset, no bring-up.\n"
               "  --init : run a full bring-up first (for a healthy reference\n"
               "           dump on a freshly power-cycled adapter).\n");
}

} // namespace

int main(int argc, char **argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    const char *v = (i + 1 < argc) ? argv[i + 1] : nullptr;
    if (!std::strcmp(argv[i], "--vid") && v) {
      a.vid = static_cast<uint16_t>(std::strtol(v, nullptr, 0));
      ++i;
    } else if (!std::strcmp(argv[i], "--pid") && v) {
      a.pid = static_cast<int>(std::strtol(v, nullptr, 0));
      ++i;
    } else if (!std::strcmp(argv[i], "--channel") && v) {
      a.channel = static_cast<int>(std::strtol(v, nullptr, 0));
      ++i;
    } else if (!std::strcmp(argv[i], "--init")) {
      a.init = true;
    } else {
      usage();
      return 2;
    }
  }

  auto logger = std::make_shared<Logger>();
  install_devourer_signal_handlers();
  devourer::DeviceSession session{logger};

  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) < 0) {
    logger->error("libusb_init failed");
    return 3;
  }
  session.adopt_context(ctx);

  libusb_device_handle *handle = nullptr;
  if (a.pid >= 0) {
    handle = libusb_open_device_with_vid_pid(ctx, a.vid,
                                             static_cast<uint16_t>(a.pid));
  } else {
    for (uint16_t pid : kRealtekPids) {
      handle = libusb_open_device_with_vid_pid(ctx, a.vid, pid);
      if (handle)
        break;
    }
  }
  if (!handle) {
    logger->error("no adapter found (vid {:04x})", a.vid);
    return 3;
  }

  /* do_reset=false is the whole point: a USB reset re-runs the chip's own boot
   * and would wipe the state we came to read. Only --init opts into disturbing
   * the chip, and even then the reset stays off so the bring-up starts from
   * whatever the chip currently holds. */
  std::shared_ptr<devourer::UsbDeviceLock> lock;
  if (devourer::claim_interface_then_reset(
          handle, devourer::find_wifi_interface(handle), logger,
          /*do_reset=*/false, lock) != 0) {
    session.adopt_handle(handle);
    return 3;
  }
  session.adopt_handle(handle);
  session.adopt_lock(lock);

  devourer::DeviceConfig cfg;
  /* Leave the chip exactly as found. Without this the device destructor runs
   * the card-disable sequence on the way out, so the tool would power down the
   * very state it exists to inspect — one look and the evidence is gone. */
  cfg.tuning.teardown_power_down = false;
  WiFiDriver driver(logger);
  std::unique_ptr<IRtlDevice> owned = driver.CreateRtlDevice(handle, ctx, lock, cfg);
  if (!owned) {
    logger->error("CreateRtlDevice failed (chip support not built?)");
    return 3;
  }
  session.adopt_device(std::move(owned));
  IRtlDevice *const dev = session.device();

  if (a.init) {
    logger->info("chipstate: --init, running a full bring-up before the dump");
    dev->InitWrite(SelectedChannel{.Channel = static_cast<uint8_t>(a.channel),
                                   .ChannelOffset = 0,
                                   .ChannelWidth = CHANNEL_WIDTH_20});
  } else {
    logger->info("chipstate: read-only attach (no USB reset, no bring-up) — "
                 "the chip is being read exactly as the last session left it");
  }

  dev->DumpChipState();
  return 0;
}
