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

#include <vector>

#include "DeviceSession.h"
#include "IRtlDevice.h"
#include "RtlAdapter.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "logger.h"
#include "SignalStop.h"

namespace {

/* Same list the other demos' open loop iterates; --pid narrows to one. */
const uint16_t kRealtekPids[] = {0x8812, 0x8813, 0x881a, 0x0811, 0xa811,
                                 0x0820, 0x0821, 0x8822, 0x0120, 0x012d,
                                 0xb82c, 0xc811, 0xc812, 0xa81a};

/* One --peek/--poke, kept in argv order so a poke-then-peek verifies the
 * write inside a single claim. */
struct RegOp {
  bool write = false;
  uint16_t addr = 0;
  uint16_t end = 0;   /* peek range: inclusive last addr (== addr if single) */
  uint32_t val = 0;   /* poke */
  int width = 1;      /* poke: 1/2/4 */
};

struct Args {
  uint16_t vid = 0x0bda;
  int pid = -1;
  int channel = 6;
  bool init = false;
  bool no_claim = false;
  std::vector<RegOp> ops;
};

void usage() {
  std::fprintf(stderr,
               "usage: chipstate [--vid 0xNNNN] [--pid 0xNNNN] [--init] "
               "[--channel N]\n"
               "                 [--peek 0xA[-0xB]]... [--poke 0xA=0xV[:W]]...\n"
               "  default: attach read-only, no USB reset, no bring-up.\n"
               "  --init : run a full bring-up first (for a healthy reference\n"
               "           dump on a freshly power-cycled adapter).\n"
               "  --peek : dump register byte(s) over the vendor-control path\n"
               "           (range inclusive, 16 bytes/row) instead of the\n"
               "           canary set. Bypasses chip dispatch — any die.\n"
               "  --poke : write a register (width W = 1/2/4, default from the\n"
               "           value magnitude). The bench-bisection intervention\n"
               "           lever; ops run in argv order, so a trailing --peek\n"
               "           verifies the write in the same claim.\n"
               "  --no-claim : (peek/poke only) skip the interface claim —\n"
               "           vendor control rides EP0 with device recipient, so\n"
               "           registers stay reachable while another process\n"
               "           (e.g. an armed rxdemo) owns the interface.\n");
}

/* Range-checked address parse: a silently-wrapped register (0x12345 ->
 * 0x2345) on a poke tool is a corruption hazard, so out-of-range is a parse
 * error, never a truncation. */
bool parse_reg_addr(const char *s, char **end, uint16_t &out) {
  unsigned long v = std::strtoul(s, end, 0);
  if (*end == s || v > 0xffff)
    return false;
  out = static_cast<uint16_t>(v);
  return true;
}

bool parse_peek(const char *s, RegOp &op) {
  char *end = nullptr;
  if (!parse_reg_addr(s, &end, op.addr))
    return false;
  op.end = op.addr;
  if (*end == '-') {
    if (!parse_reg_addr(end + 1, &end, op.end) || op.end < op.addr)
      return false;
  }
  return *end == '\0';
}

bool parse_poke(const char *s, RegOp &op) {
  char *end = nullptr;
  op.write = true;
  if (!parse_reg_addr(s, &end, op.addr))
    return false;
  if (*end != '=')
    return false;
  const char *vs = end + 1;
  unsigned long v = std::strtoul(vs, &end, 0);
  if (end == vs || v > 0xfffffffful)
    return false;
  op.val = static_cast<uint32_t>(v);
  if (*end == ':') {
    op.width = static_cast<int>(std::strtoul(end + 1, &end, 0));
    if (op.width != 1 && op.width != 2 && op.width != 4)
      return false;
    /* An explicit width the value doesn't fit is a mistake, not a mask. */
    if (op.width < 4 && (op.val >> (op.width * 8)) != 0)
      return false;
  } else {
    op.width = op.val <= 0xff ? 1 : op.val <= 0xffff ? 2 : 4;
  }
  return *end == '\0';
}

/* Raw register client over the transport layer — deliberately below
 * CreateRtlDevice so it works on any die, configured or not. */
int run_reg_ops(libusb_device_handle *handle, Logger_t logger,
                libusb_context *ctx,
                std::shared_ptr<devourer::UsbDeviceLock> lock,
                const std::vector<RegOp> &ops) {
  RtlAdapter adapter(handle, logger, ctx, lock);
  /* A failed vendor-control read throws (UsbTransport::ctrl_read) — on a
   * powered-down or wedged chip that is a real answer about the chip, so
   * report which op died and exit nonzero instead of terminating. */
  try {
    for (const RegOp &op : ops) {
      if (op.write) {
        if (op.width == 4)
          adapter.rtw_write32(op.addr, op.val);
        else if (op.width == 2)
          adapter.rtw_write16(op.addr, static_cast<uint16_t>(op.val));
        else
          adapter.rtw_write8(op.addr, static_cast<uint8_t>(op.val));
        std::printf("poke 0x%04x = 0x%0*x\n", op.addr, op.width * 2, op.val);
      } else {
        for (uint32_t row = op.addr & ~0xfu; row <= op.end; row += 16) {
          std::printf("0x%04x:", row);
          for (uint32_t i = row; i < row + 16; ++i) {
            if (i < op.addr || i > op.end)
              std::printf("   ");
            else
              std::printf(" %02x", adapter.rtw_read8(static_cast<uint16_t>(i)));
          }
          std::printf("\n");
        }
      }
    }
  } catch (const std::exception &e) {
    std::fflush(stdout);
    logger->error("register access failed ({}) — chip powered down / wedged? "
                  "That is the finding, not a tool error.", e.what());
    return 4;
  }
  std::fflush(stdout);
  return 0;
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
    } else if (!std::strcmp(argv[i], "--no-claim")) {
      a.no_claim = true;
    } else if (!std::strcmp(argv[i], "--peek") && v) {
      RegOp op;
      if (!parse_peek(v, op)) {
        usage();
        return 2;
      }
      a.ops.push_back(op);
      ++i;
    } else if (!std::strcmp(argv[i], "--poke") && v) {
      RegOp op;
      if (!parse_poke(v, op)) {
        usage();
        return 2;
      }
      a.ops.push_back(op);
      ++i;
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

  /* --no-claim + reg ops: EP0 vendor control with device recipient does not
   * need the interface, so peeks/pokes work while another process (a live
   * armed rxdemo) owns it — the concurrent-intervention mode. */
  if (a.no_claim) {
    if (a.ops.empty()) {
      logger->error("--no-claim is peek/poke-only (the canary dump needs the "
                    "claimed device)");
      session.adopt_handle(handle);
      return 2;
    }
    session.adopt_handle(handle);
    return run_reg_ops(handle, logger, ctx, nullptr, a.ops);
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

  /* --peek/--poke: raw transport-level register access, no device
   * construction at all — the chip is not even identified, let alone
   * configured, so this works mid-experiment on any die. */
  if (!a.ops.empty())
    return run_reg_ops(handle, logger, ctx, lock, a.ops);

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
