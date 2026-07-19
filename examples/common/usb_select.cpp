#include "usb_select.h"

#include <chrono>
#include <cstdlib>
#include <thread>

/* Fill the pick descriptor from an opened handle (best-effort — identity
 * logging must never fail an open that succeeded). */
static void describe(libusb_device_handle *h, uint16_t vid, uint16_t pid,
                     UsbPick *picked) {
  if (picked == nullptr || h == nullptr)
    return;
  picked->vid = vid;
  picked->pid = pid;
  libusb_device *dev = libusb_get_device(h);
  if (dev == nullptr)
    return;
  picked->bus = libusb_get_bus_number(dev);
  uint8_t ports[8];
  const int pc = libusb_get_port_numbers(dev, ports, sizeof(ports));
  picked->port.clear();
  for (int p = 0; p < pc; ++p)
    picked->port += (picked->port.empty() ? "" : ".") + std::to_string(ports[p]);
  picked->speed = libusb_get_device_speed(dev);
}

libusb_device_handle *
open_selected_usb(libusb_context *ctx, const std::shared_ptr<Logger> &logger,
                  const uint16_t *default_pids, size_t n_default_pids,
                  UsbPick *picked) {
  /* DEVOURER_PID env var (hex, e.g. "0x8813") restricts the open loop to a
   * single PID. Useful when multiple Realtek adapters are plugged.
   * DEVOURER_VID overrides the VID (default 0x0bda Realtek) — needed to reach
   * OEM-rebadged Jaguar dongles like the TP-Link Archer T2U Plus (2357:0120). */
  const char *pid_env = std::getenv("DEVOURER_PID");
  uint16_t target_pid = 0;
  if (pid_env != nullptr) {
    target_pid = static_cast<uint16_t>(std::strtoul(pid_env, nullptr, 0));
    logger->info("DEVOURER_PID={:04x} (limiting to this PID)", target_pid);
  }
  uint16_t target_vid = 0x0bda;
  if (const char *vid_env = std::getenv("DEVOURER_VID")) {
    target_vid = static_cast<uint16_t>(std::strtoul(vid_env, nullptr, 0));
    logger->info("DEVOURER_VID={:04x} (overriding default VID)", target_vid);
  }
  libusb_device_handle *dev_handle = nullptr;

  /* DEVOURER_USB_BUS (+ optional DEVOURER_USB_PORT) select a specific device by
   * USB topology when several share one VID:PID and even the serial — e.g. two
   * RTL8814AU dongles (CF-938AC vs CF-960AC) that enumerate identically, so only
   * the bus/port tells them apart. DEVOURER_USB_PORT is the dotted libusb port
   * path (as in sysfs `devpath` / `lsusb -t`, e.g. "2.3.2"). When bus is unset,
   * the VID:PID open loop below runs as before. */
  if (const char *bus_env = std::getenv("DEVOURER_USB_BUS")) {
    const auto want_bus = static_cast<uint8_t>(std::strtoul(bus_env, nullptr, 0));
    const char *port_env = std::getenv("DEVOURER_USB_PORT");
    /* A named socket is EXPECTED to hold the device — but the previous
     * session's close/kill leaves the chip re-enumerating (firmware reload
     * through ROM, sometimes via the ZeroCD id) for several seconds. Poll
     * bounded instead of failing on the first empty scan. */
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(15);
    bool waited = false;
    uint16_t matched_pid = 0;
    do {
      libusb_device **list = nullptr;
      ssize_t n = libusb_get_device_list(ctx, &list);
      for (ssize_t i = 0; i < n && dev_handle == nullptr; ++i) {
        libusb_device_descriptor dd{};
        if (libusb_get_device_descriptor(list[i], &dd) != 0) continue;
        if (dd.idVendor != target_vid) continue;
        if (target_pid != 0 && dd.idProduct != target_pid) continue;
        if (libusb_get_bus_number(list[i]) != want_bus) continue;
        if (port_env != nullptr) {
          uint8_t ports[8];
          int pc = libusb_get_port_numbers(list[i], ports, sizeof(ports));
          std::string path;
          for (int p = 0; p < pc; ++p)
            path += (path.empty() ? "" : ".") + std::to_string(ports[p]);
          if (path != port_env) continue;
        }
        if (libusb_open(list[i], &dev_handle) == 0) {
          matched_pid = dd.idProduct;
          logger->info("Opened device {:04x}:{:04x} on bus {} port {}",
                       dd.idVendor, dd.idProduct, want_bus,
                       port_env ? port_env : "(any)");
        }
      }
      if (list != nullptr) libusb_free_device_list(list, 1);
      if (dev_handle != nullptr) break;
      if (!waited) {
        logger->warn("DEVOURER_USB_BUS={} PORT={} matched no device — waiting "
                     "for it to (re-)enumerate",
                     want_bus, port_env ? port_env : "(any)");
        waited = true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } while (std::chrono::steady_clock::now() < deadline);
    /* Topology selection is strict: falling through to the VID:PID loop here
     * would silently open a DIFFERENT adapter sharing the id (the exact
     * ambiguity DEVOURER_USB_BUS exists to resolve) — fail instead. */
    if (dev_handle == nullptr) {
      logger->error("DEVOURER_USB_BUS={} PORT={} matched no device", want_bus,
                    port_env ? port_env : "(any)");
      return nullptr;
    }
    describe(dev_handle, target_vid, matched_pid, picked);
    return dev_handle;
  }

  for (size_t i = 0; i < n_default_pids; ++i) {
    const uint16_t pid = default_pids[i];
    if (target_pid != 0 && pid != target_pid) continue;
    dev_handle = libusb_open_device_with_vid_pid(ctx, target_vid, pid);
    if (dev_handle != nullptr) {
      logger->info("Opened device {:04x}:{:04x}", target_vid, pid);
      describe(dev_handle, target_vid, pid, picked);
      return dev_handle;
    }
  }
  /* DEVOURER_PID can name a PID not in the default list (e.g. 0x0120 for the
   * T2U Plus). Try that direct combination once before giving up. */
  if (target_pid != 0) {
    dev_handle = libusb_open_device_with_vid_pid(ctx, target_vid, target_pid);
    if (dev_handle != nullptr) {
      logger->info("Opened device {:04x}:{:04x} (via DEVOURER_PID)",
                   target_vid, target_pid);
      describe(dev_handle, target_vid, target_pid, picked);
      return dev_handle;
    }
  }
  logger->error("Cannot find any supported device under VID {:04x}",
                target_vid);
  return nullptr;
}
