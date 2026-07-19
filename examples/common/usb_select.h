#pragma once

/* Shared demo-side USB device selection — the DEVOURER_PID / DEVOURER_VID /
 * DEVOURER_USB_BUS / DEVOURER_USB_PORT open loop, factored out of rxdemo so
 * multi-adapter demos (chanscout as the second radio next to a primary
 * receiver) bind deterministically without a third copy of the logic.
 *
 * Selection rules (unchanged from the historical rxdemo behaviour):
 *   DEVOURER_VID=0xNNNN   override the default Realtek VID (OEM-rebadged
 *                         dongles, e.g. TP-Link 2357:xxxx).
 *   DEVOURER_PID=0xNNNN   restrict to one PID; may name a PID outside the
 *                         caller's default list (tried directly).
 *   DEVOURER_USB_BUS=N    select by USB topology when several adapters share
 *   DEVOURER_USB_PORT=a.b.c  one VID:PID (and even the serial). The port path
 *                         is the dotted libusb port chain (sysfs devpath /
 *                         `lsusb -t`). Topology selection is STRICT: no
 *                         silent fallback to a different same-id adapter,
 *                         and it polls up to 15 s for a device still
 *                         re-enumerating after the previous session.
 *
 * Returns an opened (not claimed) handle, or nullptr after logging why. */

#include <libusb.h>

#include <cstdint>
#include <memory>
#include <string>

#include "logger.h"

/* Physical identity of the opened device, for role/identity logging (the
 * scout.id event): what was matched and where it sits on the bus. */
struct UsbPick {
  uint16_t vid = 0, pid = 0;
  uint8_t bus = 0;
  std::string port;    /* dotted port path, empty when unavailable */
  int speed = 0;       /* libusb_get_device_speed enum value */
};

libusb_device_handle *
open_selected_usb(libusb_context *ctx, const std::shared_ptr<Logger> &logger,
                  const uint16_t *default_pids, size_t n_default_pids,
                  UsbPick *picked = nullptr);
