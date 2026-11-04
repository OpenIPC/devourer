// radio_open.h — open the radio a bench tool names in its environment, over
// USB or PCIe, so one probe binary serves both transports:
//   DEVOURER_PCIE_BDF=0000:05:00.0 -> vfio PCIe (DEVOURER_PCIE=ON builds; the
//                                     device bound by tests/pcie_vfio_bind.sh)
//   otherwise                      -> USB by DEVOURER_VID / DEVOURER_PID, with
//                                     the tool's own defaults.
// The radio gets devourer_config_from_env() either way. The USB context and
// handle are deliberately never released: these are one-shot bench tools that
// end in _exit/return from main, so the OS reclaims them.
#ifndef DEVOURER_RADIO_OPEN_H
#define DEVOURER_RADIO_OPEN_H

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#include <libusb.h>

#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"
#if defined(DEVOURER_HAVE_PCIE)
#include "PcieTransport.h"
#endif

struct OpenedRadio {
  std::unique_ptr<IRadio> dev; /* null on failure (the reason is on stderr) */
  std::string label;           /* "pcie 0000:05:00.0" / "usb 0bda:8812" */
};

inline OpenedRadio open_radio_from_env(const std::shared_ptr<Logger> &logger,
                                       uint16_t def_vid, uint16_t def_pid) {
  OpenedRadio r;
  if (const char *bdf = std::getenv("DEVOURER_PCIE_BDF")) {
#if defined(DEVOURER_HAVE_PCIE)
    r.label = std::string("pcie ") + bdf;
    auto t = devourer::PcieTransport::Open(bdf, logger);
    if (!t) {
      fprintf(stderr, "pcie open failed for %s (vfio-bound?)\n", bdf);
      return r;
    }
    WiFiDriver wifi(logger);
    r.dev = wifi.CreateRadioPcie(std::move(t), devourer_config_from_env());
    if (!r.dev) fprintf(stderr, "no driver for the PCIe chip at %s\n", bdf);
#else
    fprintf(stderr, "DEVOURER_PCIE_BDF=%s but this build has no PCIe "
                    "(configure with -DDEVOURER_PCIE=ON)\n", bdf);
#endif
    return r;
  }
  uint16_t vid = def_vid, pid = def_pid;
  if (const char *v = std::getenv("DEVOURER_VID")) vid = (uint16_t)strtoul(v, 0, 0);
  if (const char *p = std::getenv("DEVOURER_PID")) pid = (uint16_t)strtoul(p, 0, 0);
  char buf[24];
  snprintf(buf, sizeof buf, "usb %04x:%04x", vid, pid);
  r.label = buf;
  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) < 0) { fprintf(stderr, "libusb_init failed\n"); return r; }
  libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
  auto *h = libusb_open_device_with_vid_pid(ctx, vid, pid);
  if (!h) { fprintf(stderr, "open fail %04x:%04x\n", vid, pid); return r; }
  std::shared_ptr<devourer::UsbDeviceLock> lk;
  if (devourer::claim_interface_then_reset(h, devourer::find_wifi_interface(h),
                                           logger, true, lk) != 0)
    return r;
  WiFiDriver wifi(logger);
  r.dev = wifi.CreateRadio(h, ctx, lk, devourer_config_from_env());
  if (!r.dev) fprintf(stderr, "no driver for %04x:%04x\n", vid, pid);
  return r;
}

#endif /* DEVOURER_RADIO_OPEN_H */
