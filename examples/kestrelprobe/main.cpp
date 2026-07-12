/* kestrelprobe — staged bring-up driver for the Kestrel (Wi-Fi 6 / 802.11ax,
 * RTL8852BU / RTL8852CU) milestones, the USB sibling of pcieprobe. Validates
 * the layers one at a time, bottom-up:
 *
 *   id     (M0) USB open + PID-table variant + AX-native identity: the die-id
 *               at R_AX_SYS_CHIPINFO (0x00FC) must match the PID-selected
 *               variant (0x51 = 8852B, 0x52 = 8852C); cut version from
 *               R_AX_SYS_CFG1[15:12].
 *   power  (M1) + mac_ax power-on sequence + efuse logical map.  [not ported]
 *   fw     (M1) + firmware download (NICCE image) + fw-ready poll. [not ported]
 *
 * Usage: sudo kestrelprobe [id|power|fw] [--vid 0xNNNN] [--pid 0xNNNN]
 * Default stage: id. Without --vid/--pid the open loop scans the Kestrel PID
 * table (kestrel/KestrelUsbIds.h). Two cold-plug traps: the TX20U Nano first
 * enumerates as a ZeroCD disk (0bda:1a2b) until usb_modeswitch flips it to
 * 35bc:0108, and the in-kernel rtw89_8852bu module auto-probes the NIC at
 * every enumeration — temp-blacklist it (modprobe -r does not survive a
 * re-enumeration; see docs/adapter-doctor.md).
 *
 * Events (stdout JSONL): kestrel.id / kestrel.power / kestrel.fw with
 * ok:true|false — exit code 0 only if the requested stage passed. */
#if defined(__ANDROID__) || defined(_MSC_VER) || defined(__APPLE__)
#include <libusb.h>
#else
#include <libusb-1.0/libusb.h>
#endif

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

#include "Event.h"
#include "RtlAdapter.h"
#include "UsbOpen.h"
#include "logger.h"

#include "kestrel/ChipVariant.h"
#include "kestrel/KestrelUsbIds.h"
#include "kestrel/RtlKestrelDevice.h"

namespace {

const char *variant_name(kestrel::ChipVariant v) {
  return v == kestrel::ChipVariant::C8852B ? "8852B" : "8852C";
}

bool parse_hex16(const char *s, uint16_t &out) {
  char *end = nullptr;
  long v = std::strtol(s, &end, 0);
  if (end == s || *end != '\0' || v < 0 || v > 0xFFFF)
    return false;
  out = static_cast<uint16_t>(v);
  return true;
}

} /* namespace */

int main(int argc, char **argv) {
  std::string stage = "id";
  uint16_t want_vid = 0, want_pid = 0; /* 0 = scan the Kestrel PID table */
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    if (k == "id" || k == "power" || k == "fw")
      stage = k;
    else if (k == "--vid" && i + 1 < argc && parse_hex16(argv[++i], want_vid))
      ;
    else if (k == "--pid" && i + 1 < argc && parse_hex16(argv[++i], want_pid))
      ;
    else {
      std::fprintf(stderr,
                   "usage: %s [id|power|fw] [--vid 0xNNNN] [--pid 0xNNNN]\n",
                   argv[0]);
      return 2;
    }
  }
  const int want = stage == "fw" ? 2 : stage == "power" ? 1 : 0;

  auto logger = std::make_shared<Logger>();

  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) < 0) {
    logger->error("libusb_init failed");
    return 1;
  }

  libusb_device_handle *handle = nullptr;
  kestrel::ChipVariant variant = kestrel::ChipVariant::C8852B;
  uint16_t vid = 0, pid = 0;
  if (want_vid && want_pid) {
    auto v = kestrel::variant_for_usb_id(want_vid, want_pid);
    if (!v) {
      logger->error("{:04x}:{:04x} is not in the Kestrel PID table "
                    "(kestrel/KestrelUsbIds.h)",
                    want_vid, want_pid);
      libusb_exit(ctx);
      return 2;
    }
    handle = libusb_open_device_with_vid_pid(ctx, want_vid, want_pid);
    variant = *v;
    vid = want_vid;
    pid = want_pid;
  } else {
    for (const auto &kid : kestrel::kKestrelUsbIds) {
      handle = libusb_open_device_with_vid_pid(ctx, kid.vid, kid.pid);
      if (handle) {
        variant = kid.variant;
        vid = kid.vid;
        pid = kid.pid;
        break;
      }
    }
  }
  if (!handle) {
    logger->error("no Kestrel adapter found (scanned the KestrelUsbIds "
                  "table). If lsusb shows 0bda:1a2b the dongle is still in "
                  "ZeroCD disk mode — usb_modeswitch it first; if a rtw89_* "
                  "module holds it, temp-blacklist (not modprobe -r).");
    devourer::Ev(logger->events(), "kestrel.id")
        .f("ok", false)
        .f("why", "open");
    libusb_exit(ctx);
    return 1;
  }

  std::shared_ptr<devourer::UsbDeviceLock> lock;
  if (devourer::claim_interface_then_reset(handle, 0, logger, true, lock) !=
      0) {
    libusb_close(handle);
    libusb_exit(ctx);
    return 1;
  }

  /* ---- stage id (M0): register plane only, no power, no DMA ---- */
  RtlKestrelDevice dev(RtlAdapter(handle, logger, ctx, lock, {}), logger,
                       variant);
  const kestrel::ChipInfo info = dev.ReadChipInfo();
  const bool id_ok = info.matches(variant);
  logger->info("M0: {:04x}:{:04x} -> {} | die-id=0x{:02x} (want 0x{:02x}) "
               "cut={}",
               vid, pid, variant_name(variant), info.die_id,
               variant == kestrel::ChipVariant::C8852B ? 0x51 : 0x52,
               info.cut);
  devourer::Ev(logger->events(), "kestrel.id")
      .f("ok", id_ok)
      .f("variant", variant_name(variant))
      .hexf("vid", vid, 4)
      .hexf("pid", pid, 4)
      .hexf("die_id", info.die_id, 2)
      .f("cut", info.cut);
  if (!id_ok || want < 1) {
    libusb_close(handle);
    libusb_exit(ctx);
    return id_ok ? 0 : 1;
  }

  /* ---- stage power (M1a): mac_ax power-on + efuse dump ---- */
  kestrel::EfuseInfo efuse;
  bool power_ok = false;
  try {
    power_ok = dev.PowerOnAndReadEfuse(efuse);
  } catch (const std::exception &e) {
    logger->error("M1a: power/efuse threw: {}", e.what());
  }
  char mac[18] = "??:??:??:??:??:??";
  if (power_ok)
    snprintf(mac, sizeof(mac), "%02x:%02x:%02x:%02x:%02x:%02x", efuse.mac[0],
             efuse.mac[1], efuse.mac[2], efuse.mac[3], efuse.mac[4],
             efuse.mac[5]);
  logger->info("M1a: power_ok={} MAC={} xtal=0x{:02x} rfe=0x{:02x} "
               "thermalA=0x{:02x} thermalB=0x{:02x} autoload={}",
               power_ok, mac, efuse.xtal_cap, efuse.rfe_type, efuse.thermal_a,
               efuse.thermal_b, efuse.autoload_ok);
  devourer::Ev(logger->events(), "kestrel.power")
      .f("ok", power_ok)
      .f("mac", mac)
      .hexf("xtal", efuse.xtal_cap, 2)
      .hexf("rfe", efuse.rfe_type, 2)
      .f("autoload", efuse.autoload_ok);
  if (!power_ok || want < 2) {
    libusb_close(handle);
    libusb_exit(ctx);
    return power_ok ? 0 : 1;
  }

  /* ---- stage fw (M1b): firmware download ---- [not ported yet] */
  logger->error("M1b: fw download not ported yet (Kestrel milestone M1b)");
  devourer::Ev(logger->events(), "kestrel.fw")
      .f("ok", false)
      .f("why", "not ported (M1b)");
  libusb_close(handle);
  libusb_exit(ctx);
  return 1;
}
