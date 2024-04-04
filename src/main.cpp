#include <cassert>
#include <iostream>
#include <memory>

#include <libusb.h>

#include <spdlog/sinks/android_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include "FrameParser.h"
#include "RtlUsbAdapter.h"
#include "WiFiDriver.h"

#define USB_VENDOR_ID 0x0bda
#define USB_PRODUCT_ID 0x8812

static void packetProcessor(const Packet &packet) {}

int main() {
  libusb_context *ctx;
  libusb_device **devs;
  int rc;
  ssize_t cnt;

#ifdef __ANDROID__
  auto logger = spdlog::android_logger_mt("android", tag);
#else
  auto logger = spdlog::stderr_color_mt("stderr");
#endif

  rc = libusb_init(&ctx);
  if (rc < 0)
    return rc;

// #ifndef NDEBUG
#if 0
  libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_DEBUG);
#endif

  libusb_device_handle *dev_handle =
      libusb_open_device_with_vid_pid(ctx, USB_VENDOR_ID, USB_PRODUCT_ID);
  if (dev_handle == NULL) {
    logger->error("Cannot find device {:04x}:{:04x}", USB_VENDOR_ID,
                  USB_PRODUCT_ID);
    libusb_exit(ctx);
    return 1;
  }

  /*Check if kenel driver attached*/
  if (libusb_kernel_driver_active(dev_handle, 0)) {
    rc = libusb_detach_kernel_driver(dev_handle, 0); // detach driver
    assert(rc == 0);
  }
  rc = libusb_claim_interface(dev_handle, 0);
  assert(rc == 0);

  WiFiDriver wifi_driver{logger};
  auto rtlDevice = wifi_driver.CreateRtlDevice(dev_handle);
  rtlDevice->Init(packetProcessor, SelectedChannel{
                                       .Channel = 36,
                                       .ChannelOffset = 0,
                                       .ChannelWidth = CHANNEL_WIDTH_20,
                                   });

  rc = libusb_release_interface(dev_handle, 0);
  assert(rc == 0);

  libusb_close(dev_handle);

  libusb_exit(ctx);
  return 0;
}
