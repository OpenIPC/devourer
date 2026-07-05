#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <climits>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#if defined(_MSC_VER)
  #include <windows.h>
  #include <process.h>
  typedef int pid_t;
  #define fork() (0)
  #define sleep(seconds) Sleep((seconds)*1000)
#elif defined(__ANDROID__)
  #include <libusb.h>
#elif defined(__APPLE__)
  #include <unistd.h>
  #include <libusb.h>
#else
  #include <unistd.h>
  #include <libusb-1.0/libusb.h>
#endif

#include "BfReportDetect.h"
#include "ChannelFreq.h"
#include "RxPacket.h"
#if defined(DEVOURER_HAVE_JAGUAR1)
#include "jaguar1/RtlJaguarDevice.h"
#endif
#if defined(DEVOURER_HAVE_JAGUAR2)
#include "jaguar2/RtlJaguar2Device.h"
#endif
#if defined(DEVOURER_HAVE_JAGUAR3)
#include "jaguar3/RtlJaguar3Device.h"
#endif
#include "RtlUsbAdapter.h"
#include "SignalStop.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "RadiotapBuilder.h"
#include "logger.h"

#define USB_VENDOR_ID 0x0bda

/* Known USB product IDs for the Realtek Jaguar family — same set as the RX
 * demo (demo/main.cpp). */
static constexpr uint16_t kRealtekProductIds[] = {
    0x8812, 0x0811, 0xa811, 0xb811, 0x8813,
    0xb812, 0xb82c, /* RTL8822BU (Jaguar2); OEM VIDs via DEVOURER_VID/PID */
    0xc82c, 0xc82e, 0xc812, /* RTL8822CU/8812CU (Jaguar3 CU) */
    0x881a, 0x881b, 0x881c, 0xa81a, /* RTL8812EU (Jaguar3 EU; a81a = BL-M8812EU2) */
    0xe822, 0xa82a, /* RTL8822EU (Jaguar3 EU) */
};

/* Process-start reference for the init-timing lines (see src/InitTimer.h).
 * `init-timing: txdemo.first_tx_submit` is the end-to-end "ready to TX" mark:
 * exec → first bulk-OUT submitted. */
static const std::chrono::steady_clock::time_point g_proc_start =
    std::chrono::steady_clock::now();
static long long ms_since_start() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now() - g_proc_start)
      .count();
}

/* Build a radiotap header carrying CHANNEL (freq) + TX_FLAGS, then append the
 * 802.11 body. The library reads the CHANNEL field and FastRetunes per packet. */
static std::vector<uint8_t> build_frame_with_channel(uint16_t freq,
                                                     const uint8_t *body,
                                                     size_t body_len) {
  std::vector<uint8_t> f;
  /* version, pad, len(le16)=14, present(le32)= CHANNEL(bit3)|TX_FLAGS(bit15) */
  const uint8_t hdr[] = {0x00, 0x00, 0x0e, 0x00, 0x08, 0x80, 0x00, 0x00};
  f.insert(f.end(), hdr, hdr + sizeof(hdr));
  f.push_back(freq & 0xff);          /* CHANNEL freq le16 */
  f.push_back((freq >> 8) & 0xff);
  f.push_back(0x00);                  /* CHANNEL flags le16 (unused) */
  f.push_back(0x00);
  f.push_back(0x08);                  /* TX_FLAGS le16 */
  f.push_back(0x00);
  f.insert(f.end(), body, body + body_len);
  return f;
}

static int g_rx_count = 0;
static void packetProcessor(const Packet &packet) {
  /* C2H packets are chip-side status (one per TX during concurrent TX+RX on
   * Jaguar3), not 802.11 frames — skip before counting/parsing. */
  if (packet.RxAtrib.pkt_rpt_type == RX_PACKET_TYPE::C2H_PACKET)
    return;
  ++g_rx_count;
  /* RX liveness marker for the TX+RX=thread mode: first frame + every 500th.
   * Without it a deaf RX loop is indistinguishable from a quiet channel. */
  if (g_rx_count == 1 || g_rx_count % 500 == 0) {
    printf("<devourer-rx>total=%d len=%zu\n", g_rx_count, packet.Data.size());
    fflush(stdout);
  }
  /* BF self-sounding report detector (DEVOURER_BF_DETECT_REPORT modes 1-4,
   * shared with WiFiDriverDemo): with DEVOURER_TX_WITH_RX=thread this is how
   * a single-radio sounder surfaces its own captured beamforming reports. */
  devourer::bf::detect_report(packet);
  /* Surface frames whose source MAC matches the txdemo's injected beacon
   * (57:42:75:05:d6:00). The 802.11 header starts at packet.Data[0]; SA is
   * at bytes [10..15] for a non-FromDS, non-ToDS frame. */
  if (packet.Data.size() >= 16) {
    static const uint8_t kTxSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
    if (std::memcmp(packet.Data.data() + 10, kTxSa, 6) == 0) {
      static int hits = 0;
      ++hits;
      printf("<devourer-tx-hit>RX from txdemo SA: hits=%d total_rx=%d len=%zu\n",
             hits, g_rx_count, packet.Data.size());
      fflush(stdout);
    }
  }
}

void usb_event_loop(Logger_t _logger, libusb_context *ctx) {
  /* Poll with a timeout (not the blocking libusb_handle_events) so the thread
   * notices g_devourer_should_stop and returns — letting main() join it before
   * libusb_exit. A still-running event thread inside libusb at exit time trips
   * libusb's mutex-destroy assertion (SIGABRT/core dump). */
  while (!g_devourer_should_stop) {
    struct timeval tv {0, 100000};
    int r = libusb_handle_events_timeout_completed(ctx, &tv, nullptr);
    /* TIMEOUT is normal poll-expiry; INTERRUPTED (EINTR) is a transient wakeup
     * (a signal hit the underlying poll, more likely under concurrent USB load).
     * Neither is fatal. Do NOT break on them: this thread services async TX
     * completions, so if it exits, the completion callbacks stop firing,
     * submitted URBs are never freed, and within a few frames every
     * libusb_submit_transfer fails — the "TX works briefly then every send
     * fails" wedge (seen intermittently while an RX ran concurrently). Keep
     * polling; only log a genuinely unexpected error and carry on. */
    if (r < 0 && r != LIBUSB_ERROR_TIMEOUT && r != LIBUSB_ERROR_INTERRUPTED) {
      static int logged = 0;
      if (logged++ < 5)
        _logger->error("libusb_handle_events: {} (continuing)", r);
    }
  }
}

int main(int argc, char **argv) {
  libusb_context *context = nullptr;
  libusb_device_handle *handle = nullptr;
  int rc;

  auto logger = std::make_shared<Logger>();

  /* SIGINT/SIGTERM -> break the TX loop and run a clean chip de-init, so the
   * harness's `timeout` doesn't leave the adapter's USB core hung. */
  install_devourer_signal_handlers();

  /* Two modes:
   *  1. Termux/Android: argv[1] = numeric USB fd (wrapped via
   *     libusb_wrap_sys_device).
   *  2. Linux/macOS: no argv (or non-numeric) — open by VID/PID using the
   *     same list as the RX demo. DEVOURER_PID=0xNNNN restricts to a single
   *     PID. */
  long fd = (argc >= 2) ? std::strtol(argv[1], nullptr, 0) : 0;
  const bool termux_mode = (fd > 0);

  if (termux_mode) {
    logger->info("Termux mode: wrapping fd {}", fd);
    libusb_set_option(NULL, LIBUSB_OPTION_NO_DEVICE_DISCOVERY);
    libusb_set_option(NULL, LIBUSB_OPTION_WEAK_AUTHORITY);
    rc = libusb_init(&context);
    rc = libusb_wrap_sys_device(context, (intptr_t)fd, &handle);
  } else {
    rc = libusb_init(&context);
    if (rc < 0) return rc;

    const char *pid_env = std::getenv("DEVOURER_PID");
    uint16_t target_pid = 0;
    if (pid_env != nullptr) {
      target_pid = static_cast<uint16_t>(std::strtoul(pid_env, nullptr, 0));
      logger->info("DEVOURER_PID={:04x} (limiting to this PID)", target_pid);
    }
    /* DEVOURER_VID overrides the VID (default 0x0bda) — needed for OEM-rebadged
     * Jaguar dongles like the TP-Link Archer T2U Plus (2357:0120). */
    uint16_t target_vid = USB_VENDOR_ID;
    if (const char *vid_env = std::getenv("DEVOURER_VID")) {
      target_vid = static_cast<uint16_t>(std::strtoul(vid_env, nullptr, 0));
      logger->info("DEVOURER_VID={:04x} (overriding default VID)", target_vid);
    }
    /* DEVOURER_USB_BUS (+ optional DEVOURER_USB_PORT) select a device by USB
     * topology when several share one VID:PID and even the serial — e.g. two
     * identical RTL8814AU dongles. DEVOURER_USB_PORT is the dotted libusb port
     * path (sysfs `devpath` / `lsusb -t`). Unset = the VID:PID open loop below.
     * Mirrors the RX demo (demo/main.cpp). */
    if (const char *bus_env = std::getenv("DEVOURER_USB_BUS")) {
      const auto want_bus =
          static_cast<uint8_t>(std::strtoul(bus_env, nullptr, 0));
      const char *port_env = std::getenv("DEVOURER_USB_PORT");
      libusb_device **list = nullptr;
      ssize_t n = libusb_get_device_list(context, &list);
      for (ssize_t i = 0; i < n && handle == NULL; ++i) {
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
        if (libusb_open(list[i], &handle) == 0)
          logger->info("Opened device {:04x}:{:04x} on bus {} port {}",
                       dd.idVendor, dd.idProduct, want_bus,
                       port_env ? port_env : "(any)");
      }
      if (list != nullptr) libusb_free_device_list(list, 1);
      if (handle == NULL)
        logger->error("DEVOURER_USB_BUS={} PORT={} matched no device", want_bus,
                      port_env ? port_env : "(any)");
    }

    for (uint16_t pid : kRealtekProductIds) {
      if (handle != NULL) break;
      if (target_pid != 0 && pid != target_pid) continue;
      handle = libusb_open_device_with_vid_pid(context, target_vid, pid);
      if (handle != NULL) {
        logger->info("Opened device {:04x}:{:04x}", target_vid, pid);
        break;
      }
    }
    /* DEVOURER_PID can name a PID not in kRealtekProductIds (e.g. 0x0120 for
     * the T2U Plus). Try that direct combination once before giving up. */
    if (handle == NULL && target_pid != 0) {
      handle = libusb_open_device_with_vid_pid(context, target_vid, target_pid);
      if (handle != NULL) {
        logger->info("Opened device {:04x}:{:04x} (via DEVOURER_PID)",
                     target_vid, target_pid);
      }
    }
    if (handle == NULL) {
      logger->error("No supported device found under VID {:04x}", target_vid);
      libusb_exit(context);
      return 1;
    }
  }

  libusb_device *device = libusb_get_device(handle);
  libusb_device_descriptor desc{};
  libusb_get_device_descriptor(device, &desc);
  logger->info("Vendor/Product ID: {:04x}:{:04x}", desc.idVendor,
               desc.idProduct);

  logger->info("init-timing: txdemo.open_device = {} ms", ms_since_start());
  /* Claim-before-reset (see src/UsbOpen.h): the exclusive interface claim is the
   * primary guard — a second devourer on this adapter gets BUSY and we bail here
   * before the reset, so it can't re-enumerate the adapter out from under the
   * owner. Reset skipped in termux_mode (forked child shares the fd) and for
   * DEVOURER_SKIP_RESET (warm pickup). */
  std::shared_ptr<devourer::UsbDeviceLock> usb_lock;
  rc = devourer::claim_interface_then_reset(
      handle, 0, logger,
      !termux_mode && std::getenv("DEVOURER_SKIP_RESET") == nullptr, usb_lock);
  logger->info("init-timing: txdemo.usb_reset = {} ms", ms_since_start());
  if (rc != 0) {
    libusb_close(handle);
    libusb_exit(context);
    return 1;
  }

  /* USB-wire sentinel writes — gated behind DEVOURER_USB_SENTINEL=1. Used by
   * tools/usbmon_pcap_diff.py --phase-split to delimit the init phase in a
   * devourer-side capture. We write to REG_DUMMY (0x04FC, documented as a
   * no-side-effect scratch register in hal_com_reg.h:322) so the sentinel
   * cannot perturb chip state. The 2-byte payload encodes the marker:
   * 0xDEAD before init, 0xBEEF at init-done. Diff tool matches by
   * wValue (== REG_DUMMY) AND payload (\\xad\\xde or \\xef\\xbe LE). */
  const bool sentinel_enabled = std::getenv("DEVOURER_USB_SENTINEL") != nullptr;
  auto write_sentinel = [&](uint16_t marker, const char *label) {
    if (!sentinel_enabled) return;
    uint16_t payload = marker;
    int srt = libusb_control_transfer(
        handle, /*bmRequestType*/ 0x40, /*bRequest*/ 5,
        /*wValue=REG_DUMMY*/ 0x04FC, /*wIndex*/ 0,
        reinterpret_cast<unsigned char *>(&payload), sizeof(payload),
        /*timeout_ms*/ 500);
    logger->info("USB sentinel {}: REG_DUMMY <= 0x{:04x} rc={}", label, marker, srt);
  };

  write_sentinel(0xDEAD, "pre-init");

  /* Optional interrupt-IN poller on EP 0x85 — gated by
   * DEVOURER_POLL_INTR_IN=1. The 8814AU descriptor exposes an Interrupt IN
   * endpoint at 0x85 (64-byte, bInterval=1) which carries C2H (chip-to-host)
   * messages; the upstream aircrack-ng 8814au driver submits a perpetual URB
   * on it under CONFIG_USB_INTERRUPT_IN_PIPE. Devourer currently never reads
   * it, so the chip's C2H buffer fills and firmware may stall waiting for
   * drainage — a candidate explanation for "bulk-OUT URBs complete OK but
   * nothing reaches the air" (issue #36). This thread polls EP 0x85 until
   * the process is killed; failures other than -ETIMEDOUT are logged once
   * per N. */
  /* Optional bulk-IN drainer on EP 0x81 — gated by DEVOURER_DRAIN_BULK_IN=1.
   * The kernel `88XXau` driver pre-arms 8 bulk-IN URBs of 32 KB each on
   * EP 0x81 at the end of init, *before* the first TX. The RTL8814AU
   * delivers TX-status reports back on the bulk-IN endpoint mixed with
   * RX data; if the host never has IN URBs pending, the chip cannot
   * deliver TX status and queues TX indefinitely — which fits the
   * observed pathology exactly (bulk-OUT URBs complete OK at the libusb
   * level but nothing reaches the air). Spawn a thread that pre-submits
   * a small pool of bulk-IN URBs on EP 0x81 before TX begins and keeps
   * a stream of them in flight. */
  std::atomic<bool> bulk_in_running{false};
  std::thread bulk_in_thread;
  if (std::getenv("DEVOURER_DRAIN_BULK_IN")) {
    bulk_in_running = true;
    bulk_in_thread = std::thread([handle, &bulk_in_running, logger]() {
      static constexpr int BUF_SIZE = 16 * 1024;
      uint8_t buf[BUF_SIZE];
      uint64_t reads = 0;
      while (bulk_in_running) {
        int actual = 0;
        int rc = libusb_bulk_transfer(handle, 0x81, buf, sizeof(buf),
                                       &actual, 200 /* ms */);
        if (rc == 0 && actual > 0) {
          ++reads;
          if (reads <= 5 || (reads % 200) == 0) {
            logger->info("EP 0x81 IN #{}: {} bytes (head=0x{:02x}{:02x})",
                         reads, actual, buf[0], buf[1]);
          }
        } else if (rc != 0 && rc != LIBUSB_ERROR_TIMEOUT) {
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
      }
    });
    logger->info("DEVOURER_DRAIN_BULK_IN — EP 0x81 bulk-IN drainer running");
  }

  std::atomic<bool> intr_running{false};
  std::thread intr_in_thread;
  if (std::getenv("DEVOURER_POLL_INTR_IN")) {
    intr_running = true;
    intr_in_thread = std::thread([handle, &intr_running, logger]() {
      uint8_t buf[64];
      uint64_t reads = 0, errs = 0;
      while (intr_running) {
        int actual = 0;
        int rc = libusb_interrupt_transfer(handle, 0x85, buf, sizeof(buf),
                                            &actual, 100 /* ms */);
        if (rc == 0 && actual > 0) {
          ++reads;
          if (reads <= 20 || (reads % 100) == 0) {
            char hex[64 * 2 + 1] = {0};
            /* Explicit template arg so MSVC's `windows.h` `min` macro doesn't
             * mangle this — same pattern as RtlUsbAdapter.cpp:435. */
            int hex_len = std::min<int>(actual, 32);
            for (int k = 0; k < hex_len; ++k) {
              static const char hd[] = "0123456789abcdef";
              hex[2*k]   = hd[buf[k] >> 4];
              hex[2*k+1] = hd[buf[k] & 0xF];
            }
            logger->info("EP 0x85 IN #{}: {} bytes, head={}",
                         reads, actual, hex);
          }
        } else if (rc != 0 && rc != LIBUSB_ERROR_TIMEOUT) {
          ++errs;
          if ((errs % 50) == 1) {
            logger->error("EP 0x85 IN rc={} (#{})", rc, errs);
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }
    });
    logger->info("DEVOURER_POLL_INTR_IN — EP 0x85 interrupt-IN poller running");
  }

  WiFiDriver wifi_driver{logger};
  auto rtlDevice = wifi_driver.CreateRtlDevice(handle, nullptr, usb_lock);
  if (!rtlDevice) {
    /* Factory returns null when this chip's generation wasn't compiled in
     * (per-chip CMake options); it already logged which. */
    logger->error("No driver for this chip in this build — exiting");
    return 1;
  }
  logger->info("init-timing: txdemo.create_device = {} ms", ms_since_start());

  /* Jaguar1-only research features (TX-mode default, fast-retune hopping,
   * thermal telemetry, TXAGC override, BB-reg probe) are not part of the
   * IRtlDevice contract — reach them by downcasting. jag is null on Jaguar3,
   * where those call sites are skipped, and compiled out entirely when Jaguar1
   * support isn't built. */
#if defined(DEVOURER_HAVE_JAGUAR1)
  RtlJaguarDevice *jag = dynamic_cast<RtlJaguarDevice *>(rtlDevice.get());
#endif
  /* Jaguar2 (8822BU) downcast — used only for the CW single-tone idle-hold. */
#if defined(DEVOURER_HAVE_JAGUAR2)
  RtlJaguar2Device *jag2 = dynamic_cast<RtlJaguar2Device *>(rtlDevice.get());
#endif
  /* Jaguar3 (8822C/E) downcast — used only for the CW single-tone idle-hold. */
#if defined(DEVOURER_HAVE_JAGUAR3)
  RtlJaguar3Device *jag3 = dynamic_cast<RtlJaguar3Device *>(rtlDevice.get());
#endif

  int channel = 161;
  if (const char *ch_env = std::getenv("DEVOURER_CHANNEL")) {
    channel = std::atoi(ch_env);
    logger->info("DEVOURER_CHANNEL set — tuning TX to channel {}", channel);
  }

  /* Bandwidth for init + hopping. DEVOURER_HOP_BW = 20|40|80 (default 20),
   * DEVOURER_HOP_OFFSET = primary-channel offset (0=DONT_CARE, 1=LOWER/HT40+,
   * 2=UPPER/HT40-) for 40/80. FastRetune reuses the device's bandwidth.
   * DEVOURER_NB_BW = 5|10 re-clocks the baseband to narrowband (Jaguar3 only). */
  ChannelWidth_t init_width = CHANNEL_WIDTH_20;
  uint8_t init_offset = 0;
  if (const char *e = std::getenv("DEVOURER_HOP_BW")) {
    int mhz = std::atoi(e);
    init_width = mhz == 80 ? CHANNEL_WIDTH_80
               : mhz == 40 ? CHANNEL_WIDTH_40
                           : CHANNEL_WIDTH_20;
    if (init_width != CHANNEL_WIDTH_20)
      init_offset = 1; /* HT40+ by default */
    if (const char *o = std::getenv("DEVOURER_HOP_OFFSET"))
      init_offset = static_cast<uint8_t>(std::atoi(o));
  }
  if (const char *nb = std::getenv("DEVOURER_NB_BW")) {
    int mhz = std::atoi(nb);
    if (mhz == 5)
      init_width = CHANNEL_WIDTH_5;
    else if (mhz == 10)
      init_width = CHANNEL_WIDTH_10;
    logger->info("DEVOURER_NB_BW={} — TX bandwidth {} MHz", nb, mhz);
  }

  /* TX power: only override when DEVOURER_TX_PWR is set. Without it, the library
   * programs its own efuse-calibrated default per chip (see InitWrite) — the
   * demo must NOT impose a flat reference or it clobbers that default (a
   * hardcoded 40 left the 8822e ~3 dB below the kernel). */
  if (const char *p = std::getenv("DEVOURER_TX_PWR"))
    rtlDevice->SetTxPower(static_cast<uint8_t>(std::strtol(p, nullptr, 0)));

  /* DEVOURER_TX_WITH_RX — concurrent RX alongside the TX loop on the SAME
   * claimed adapter. Two modes:
   *
   * =thread (recommended): one bring-up (InitWrite below), then StartRxLoop on
   * a std::thread next to the TX loop — the single-radio self-sounding ground
   * station (arm the sounder with DEVOURER_BF_ARM_SOUNDER and surface the
   * captured reports with DEVOURER_BF_DETECT_REPORT). The RX thread is spawned
   * after InitWrite, further down.
   *
   * Any other non-empty value: the legacy fork of an RX child on the same
   * handle. That pattern is Termux-specific (libusb_wrap_sys_device keeps the
   * kernel fd shared across fork); on a regular Linux libusb context after
   * fork(), both processes race on the same URB submission queue and the first
   * vendor request after fork tends to fail with "rtw_read: iostream error". */
  const char *tx_with_rx = std::getenv("DEVOURER_TX_WITH_RX");
  const bool rx_thread_mode = tx_with_rx && std::string(tx_with_rx) == "thread";
  if (tx_with_rx && !rx_thread_mode) {
    pid_t fpid = fork();
    if (fpid == 0) {
      rtlDevice->Init(packetProcessor,
                      SelectedChannel{
                          .Channel = static_cast<uint8_t>(channel),
                          .ChannelOffset = 0,
                          .ChannelWidth = CHANNEL_WIDTH_20,
                      });
      return 1;
    }
  }

  rtlDevice->InitWrite(SelectedChannel{
      .Channel = static_cast<uint8_t>(channel),
      .ChannelOffset = init_offset,
      .ChannelWidth = init_width});

  write_sentinel(0xBEEF, "post-init/pre-TX");
  logger->info("init-timing: txdemo.init_write = {} ms", ms_since_start());

  std::thread usb_thread(usb_event_loop, logger, context);

  /* DEVOURER_TX_WITH_RX=thread: run the RX worker loop on its own thread next
   * to the TX loop — one bring-up (the InitWrite above), one claimed handle.
   * StartRxLoop assumes the chip is up and takes over bulk-IN; send_packet's
   * bulk-OUT is safe alongside it. packetProcessor runs on this thread. */
  std::thread rx_thread;
  if (rx_thread_mode) {
    rx_thread = std::thread([&rtlDevice, logger] {
      /* An uncaught exception in a std::thread is std::terminate — a transient
       * USB read failure in the RX loop must not tear down the TX process. */
      try {
        rtlDevice->StartRxLoop(packetProcessor);
      } catch (const std::exception &e) {
        logger->error("RX loop died: {} (TX continues)", e.what());
      }
    });
    logger->info("DEVOURER_TX_WITH_RX=thread: RX loop started alongside TX");
  }

  uint8_t beacon_frame[] = {
      0x00, 0x00, 0x0a, 0x00, 0x00, 0x80, 0x00, 0x00, 0x08,
      0x00, // radiotap: TX_FLAGS only (rate-less) — rate comes from SetTxMode
      /* Mgmt / probe-request frame (FC=0x40 0x00). Was DATA / ToDS=1
       * (FC=0x08 0x01) which requires an AP context the chip doesn't
       * have in monitor mode — the chip silently NAKed every bulk OUT. */
      0x40, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x57,
      0x42, 0x75, 0x05, 0xd6, 0x00, 0x57, 0x42, 0x75, 0x05, 0xd6, 0x00,
      0x80, 0x00, // 80211 header
      0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x24, 0x4f,
      0xa0, 0xc5, 0x4a, 0xbb, 0x6a, 0x55, 0x03, 0x72, 0xf8, 0x4d, 0xc4,
      0x9d, 0x1a, 0x51, 0xb7, 0x3f, 0x98, 0xf1, 0xe7, 0x46, 0x4d, 0x1c,
      0x21, 0x86, 0x15, 0x21, 0x02, 0xf4, 0x88, 0x63, 0xff, 0x51, 0x66,
      0x34, 0xf2, 0x16, 0x71, 0xf5, 0x76, 0x0b, 0x35, 0xc0, 0xe1, 0x44,
      0xcd, 0xce, 0x4e, 0x35, 0xd9, 0x85, 0x9a, 0xcf, 0x4d, 0x48, 0x4c,
      0x8f, 0x28, 0x6f, 0x10, 0xb0, 0xa9, 0x5d, 0xbf, 0xcb, 0x6f};

  /* On-air rate/modulation comes from the device TX-mode default.
   * parse_tx_mode_env() reads DEVOURER_TX_RATE — a single string
   * "<rate>[/<bw>][/SGI][/LDPC][/STBC]" (e.g. "MCS7/40/SGI", "VHT2SS_MCS3/80",
   * "6M") — into a TxMode. The beacon's rate-less radiotap (above) lets that
   * default apply; a frame embedding its own rate radiotap overrides it per
   * packet. Default (no env) = 6 M legacy. Replaces the former per-knob
   * DEVOURER_TX_MCS/_VHT/_LDPC/_STBC/_BW env vars + the DEVOURER_TX_HT_MCS gate. */
  /* TX-mode default (DEVOURER_TX_RATE) — now a first-class IRtlDevice feature so
   * it applies to Jaguar3 (8822CU/EU) too. The demo's beacon is rate-less, so
   * without this its Jaguar3 TX fell back to MGN_1M (1 Mbps) regardless of
   * DEVOURER_TX_RATE. Per-packet radiotap still overrides. */
  const devourer::TxMode tx_mode_base = devourer::parse_tx_mode_env();
  rtlDevice->SetTxMode(tx_mode_base);

  /* DEVOURER_CONT_TX: arm modulated continuous TX (the sibling of the CW tone).
   * All three generations engage a true 100%-duty hardware modulated carrier the
   * demo idle-holds (cont_hw): Jaguar1 via the 0x914 continuous mode, Jaguar2 via
   * the same once rCCAonSec (0x838) is set, Jaguar3 via the JGR3 PMAC packet
   * generator (a fixed 6M OFDM carrier). Rate from DEVOURER_TX_RATE (Jaguar1/2). */
  const bool cont_tx = std::getenv("DEVOURER_CONT_TX") != nullptr;
  bool cont_hw = false; /* true = HW continuous engaged (idle-hold); false = feed */
  if (cont_tx) {
    bool armed = false;
#if defined(DEVOURER_HAVE_JAGUAR1)
    if (jag) { jag->StartContinuousTx(tx_mode_base); armed = true; cont_hw = true; }
#endif
#if defined(DEVOURER_HAVE_JAGUAR2)
    if (jag2) { jag2->StartContinuousTx(tx_mode_base); armed = true; cont_hw = true; }
#endif
#if defined(DEVOURER_HAVE_JAGUAR3)
    if (jag3) { jag3->StartContinuousTx(tx_mode_base); armed = true; cont_hw = true; }
#endif
    if (armed)
      logger->info("DEVOURER_CONT_TX: armed ({})",
                   cont_hw ? "HW 100%-duty carrier, idle-hold"
                           : "rate applied, beacon feed");
    else
      logger->error("DEVOURER_CONT_TX set but device is not a supported chip");
  }

  /* DEVOURER_TX_STBC_TOGGLE=1 alternates the STBC bit every frame (keeping the
   * rate from DEVOURER_TX_RATE, which must be HT/VHT for STBC to apply). The RX
   * reports the received STBC bit per frame, so an alternating TX lets one
   * moving capture compare STBC vs single-stream delivery on the same fading —
   * the transmit-side mobility measurement, tagged for free by the receiver. */
  const bool stbc_toggle = std::getenv("DEVOURER_TX_STBC_TOGGLE") != nullptr;
  if (stbc_toggle)
    logger->info("DEVOURER_TX_STBC_TOGGLE: alternating STBC on/off per frame");

  std::vector<uint8_t> tx_buf(beacon_frame, beacon_frame + sizeof(beacon_frame));

  /* Frame-size knob for throughput benchmarking. DEVOURER_TX_PAYLOAD_BYTES=N
   * pads the 802.11 body so the on-air PSDU is exactly N bytes — send_packet
   * writes real_packet_length (= PSDU) into the 16-bit TX-desc PKT_SIZE, so N
   * up to 65535 is valid (the chip's RX side caps at 16383). Pad-up only: if N
   * is below the existing body we leave it and warn. The on-wire bulk-OUT URB
   * is N + TXDESC_SIZE bytes. Default unset = the small probe-request beacon. */
  if (const char *e = std::getenv("DEVOURER_TX_PAYLOAD_BYTES")) {
    long want = std::strtol(e, nullptr, 0);
    size_t radiotap_len = 10;  /* rate-less TX_FLAGS-only radiotap */
    size_t body_len = tx_buf.size() - radiotap_len;
    if (want > 0 && static_cast<size_t>(want) > body_len) {
      tx_buf.insert(tx_buf.end(), static_cast<size_t>(want) - body_len, 0x00);
      logger->info("DEVOURER_TX_PAYLOAD_BYTES — PSDU padded {} -> {} bytes",
                   body_len, want);
    } else if (want > 0) {
      logger->warn("DEVOURER_TX_PAYLOAD_BYTES={} <= current body {} — ignored "
                   "(pad-up only)", want, body_len);
    }
  }

  /* DEVOURER_TX_NDPA_RA=aa:bb:cc:dd:ee:ff — beamforming-sounding probe:
   * replace the beacon with a 19-byte VHT NDP Announcement control frame
   * (IEEE 802.11-2016 9.3.1.19) addressed to RA, TA = the canonical SA.
   * Pair with DEVOURER_TX_NDPA=1 (library-side TX-descriptor NDPA bit, so
   * the MAC auto-appends the hardware-generated NDP) and
   * DEVOURER_TX_RATE=VHT2SS_MCS0 (sounding must be a VHT PPDU).
   * STA Info = AID 0 (non-AP/unassociated), SU feedback, Nc index 0. */
  if (const char *ra_env = std::getenv("DEVOURER_TX_NDPA_RA")) {
    unsigned ra[6];
    if (std::sscanf(ra_env, "%x:%x:%x:%x:%x:%x", &ra[0], &ra[1], &ra[2],
                    &ra[3], &ra[4], &ra[5]) != 6) {
      logger->error("DEVOURER_TX_NDPA_RA — bad MAC '{}'", ra_env);
      return 1;
    }
    /* STA Info (2 bytes LE): [11:0] AID, [12] feedback type (0=SU, 1=MU),
     * [15:13] Nc index. DEVOURER_TX_NDPA_MU=1 sets the MU bit so the
     * beamformee appends the per-tone delta-SNR MU Exclusive report. */
    const uint8_t sta_info_hi =
        std::getenv("DEVOURER_TX_NDPA_MU") ? 0x10 : 0x00; /* bit 12 */
    std::vector<uint8_t> ndpa(tx_buf.begin(), tx_buf.begin() + 10); // radiotap
    const uint8_t ndpa_body[19] = {
        0x54, 0x00,             /* FC: type=control, subtype=NDPA */
        0x64, 0x00,             /* duration ~100 us */
        static_cast<uint8_t>(ra[0]), static_cast<uint8_t>(ra[1]),
        static_cast<uint8_t>(ra[2]), static_cast<uint8_t>(ra[3]),
        static_cast<uint8_t>(ra[4]), static_cast<uint8_t>(ra[5]),
        0x57, 0x42, 0x75, 0x05, 0xd6, 0x00, /* TA = canonical SA */
        0x04,                   /* sounding dialog token: seq=1, bits[1:0]=0 */
        0x00, sta_info_hi       /* STA Info: AID=0, SU/MU feedback, Nc=0 */
    };
    ndpa.insert(ndpa.end(), ndpa_body, ndpa_body + sizeof(ndpa_body));
    tx_buf = std::move(ndpa);
    logger->info("DEVOURER_TX_NDPA_RA — sending VHT NDPA to {} instead of "
                 "the beacon", ra_env);
  }

  /* Thermal monitoring — read inline on the TX (owning) thread, so no
   * background thread shares the libusb handle (no USB contention). Cadence is
   * derived from DEVOURER_THERMAL_POLL_MS over the ~2 ms/packet loop; 0 =
   * disabled. DEVOURER_THERMAL_WARN_DELTA overrides the warn threshold (thermal
   * units above the EFUSE baseline; default 15). */
  long thermal_every = 0;
  if (const char *e = std::getenv("DEVOURER_THERMAL_POLL_MS")) {
    long ms = std::strtol(e, nullptr, 0);
    if (ms > 0) thermal_every = ms / 2 < 1 ? 1 : ms / 2;
  }
  int thermal_warn_delta = 15;
  if (const char *e = std::getenv("DEVOURER_THERMAL_WARN_DELTA")) {
    thermal_warn_delta = std::atoi(e);
  }
  if (thermal_every > 0) {
    logger->info("DEVOURER_THERMAL_POLL_MS — thermal monitor on, every {} TX "
                 "frames, warn_delta={}", thermal_every, thermal_warn_delta);
  }
  bool thermal_warned = false;

  /* Inter-frame gap. Default 2 ms (~500 fps, gentle on the USB bulk EP). Lower
   * it (e.g. DEVOURER_TX_GAP_US=0) to raise the TX duty cycle for thermal /
   * heating experiments — at the default gap the PA barely warms. */
  long tx_gap_us = 2000;
  bool tx_gap_set = false; /* explicit DEVOURER_TX_GAP_US wins (incl. 0 = no gap) */
  if (const char *e = std::getenv("DEVOURER_TX_GAP_US")) {
    tx_gap_us = std::strtol(e, nullptr, 0);
    if (tx_gap_us < 0) tx_gap_us = 0;
    tx_gap_set = true;
  }

  /* Channel-hopping mode (frequency-diversity validation). When
   * DEVOURER_HOP_CHANNELS="1,6,11" is set the TX loop dwells DWELL_FRAMES
   * frames on each listed channel, then retunes to the next via
   * SetMonitorChannel, cycling for HOP_ROUNDS full passes (0 = forever). Each
   * retune is wall-clock timed and announced as a <devourer-hop> marker so a
   * wideband SDR receiver can correlate which frames landed on which channel
   * and confirm none are dropped across the retune. Intra-band (e.g. all of
   * 1/6/11 in 2.4 GHz) is the cheap case — no band switch, no IQK — so the
   * measured switch_us here is the real per-hop cost of the current
   * (un-optimised) SetMonitorChannel path. */
  std::vector<int> hop_channels;
  long hop_dwell = 50;
  long hop_rounds = 0;
  /* 0 = full SetMonitorChannel; 1 = FastRetune (cached RF writes, fastest);
   * 2 = FastRetune without the RF cache (sw_chnl only, for A/B measurement). */
  const int hop_fast =
      std::getenv("DEVOURER_HOP_FAST") ? std::atoi(std::getenv("DEVOURER_HOP_FAST")) : 0;
  /* When set, drive hopping through the radiotap CHANNEL field (the library
   * FastRetunes per packet) instead of calling FastRetune from the demo —
   * exercises the radiotap-driven path. */
  const char *hop_rt_env = std::getenv("DEVOURER_HOP_RADIOTAP");
  const bool hop_radiotap = hop_rt_env != nullptr && hop_rt_env[0] != '\0';
  if (const char *e = std::getenv("DEVOURER_HOP_CHANNELS")) {
    std::string s(e);
    size_t pos = 0;
    while (pos < s.size()) {
      size_t comma = s.find(',', pos);
      std::string tok = s.substr(pos, comma == std::string::npos
                                          ? std::string::npos
                                          : comma - pos);
      if (!tok.empty()) {
        int ch = std::atoi(tok.c_str());
        if (ch > 0) hop_channels.push_back(ch);
      }
      if (comma == std::string::npos) break;
      pos = comma + 1;
    }
  }
  if (!hop_channels.empty()) {
    if (const char *e = std::getenv("DEVOURER_HOP_DWELL_FRAMES")) {
      hop_dwell = std::strtol(e, nullptr, 0);
      if (hop_dwell < 1) hop_dwell = 1;
    }
    if (const char *e = std::getenv("DEVOURER_HOP_ROUNDS")) {
      hop_rounds = std::strtol(e, nullptr, 0);
      if (hop_rounds < 0) hop_rounds = 0;
    }
    std::string list;
    for (size_t i = 0; i < hop_channels.size(); ++i)
      list += (i ? "," : "") + std::to_string(hop_channels[i]);
    logger->info("DEVOURER_HOP_CHANNELS — hopping [{}] dwell={} frames "
                 "rounds={} ({}){}",
                 list, hop_dwell, hop_rounds,
                 hop_rounds ? "bounded" : "forever",
                 hop_fast ? " [FastRetune]" : "");
  }

  /* TX-gain ramp (thermal-vs-gain experiment). When DEVOURER_TX_PWR_START is
   * set, force the per-rate TXAGC index to an absolute value and step it up by
   * STEP every STEP_MS, in one continuous TX session (chip never stops, so we
   * observe cumulative heating). Each (re-)apply re-runs the channel-set so the
   * new index reaches the TXAGC registers. A <devourer-txpwr> marker is emitted
   * on every change so the harness can correlate gain index with the thermal /
   * SDR streams. Without START, behaviour is unchanged (EFUSE per-rate power). */
  bool pwr_ramp = false;
  int pwr_cur = 0, pwr_stop = 0, pwr_step = 4;
  long pwr_step_ms = 30000;
  long pwr_next_step_ms = 0;
  bool txpwr_readback = std::getenv("DEVOURER_TX_PWR_READBACK") != nullptr;

  /* DEVOURER_TX_MCS_SWEEP="MCS0,MCS2,MCS4,..." — the MCS-headroom axis of the
   * link probe: step the on-air rate through the list every DEVOURER_TX_MCS_STEP_MS
   * (default 2000), emitting a <devourer-contx>mcs=<spec> marker per step. A
   * beacon-feed rate sweep (decoupled from the idle-hold HW carrier, like the
   * power ramp) so the ground reads decodable per-frame SNR/delivery at each MCS
   * and picks the highest rate the link holds. */
  std::vector<std::pair<devourer::TxMode, std::string>> mcs_sweep;
  long mcs_step_ms = 2000, mcs_next_step_ms = 0;
  size_t mcs_idx = 0;
  if (const char *e = std::getenv("DEVOURER_TX_MCS_SWEEP")) {
    std::string s = e, tok;
    std::stringstream ss(s);
    while (std::getline(ss, tok, ',')) {
      if (tok.empty()) continue;
      mcs_sweep.emplace_back(devourer::parse_tx_mode_str(tok), tok);
    }
    if (const char *ms = std::getenv("DEVOURER_TX_MCS_STEP_MS"))
      mcs_step_ms = std::strtol(ms, nullptr, 0);
    if (!mcs_sweep.empty())
      logger->info("DEVOURER_TX_MCS_SWEEP — {} rates, {} ms/step",
                   mcs_sweep.size(), mcs_step_ms);
  }
  auto apply_txpwr = [&](int idx) {
    /* TXAGC override is a Jaguar1 (RtlJaguarDevice) feature; a no-op otherwise. */
#if defined(DEVOURER_HAVE_JAGUAR1)
    if (!jag)
      return;
    jag->SetTxPowerOverride(idx);
    jag->ApplyTxPower();  /* SetMonitorChannel early-returns on same ch */
    printf("<devourer-txpwr>index=%d t_ms=%lld\n", idx,
           static_cast<long long>(ms_since_start()));
    if (txpwr_readback) {
      /* Confirm the per-rate TXAGC writes landed: path-A 1M-CCK is byte0 of
       * 0xc20, 6M-OFDM is byte0 of 0xc24. If these read back == idx but on-air
       * power doesn't follow (CCK), the chip floors CCK elsewhere; if they read
       * back != idx, something clobbered the write. */
      uint32_t cck1m = jag->ReadBBReg(0xc20, 0x000000ff);
      uint32_t ofdm6m = jag->ReadBBReg(0xc24, 0x000000ff);
      printf("<devourer-txpwr-rb>index=%d cck1m=%u ofdm6m=%u\n", idx, cck1m,
             ofdm6m);
    }
    fflush(stdout);
#else
    (void)idx;
#endif
  };
  if (const char *e = std::getenv("DEVOURER_TX_PWR_START")) {
    pwr_ramp = true;
    pwr_cur = std::atoi(e);
    pwr_stop = pwr_cur;
    if (const char *s = std::getenv("DEVOURER_TX_PWR_STOP")) pwr_stop = std::atoi(s);
    if (const char *s = std::getenv("DEVOURER_TX_PWR_STEP")) pwr_step = std::atoi(s);
    if (const char *s = std::getenv("DEVOURER_TX_PWR_STEP_MS"))
      pwr_step_ms = std::strtol(s, nullptr, 0);
    if (pwr_step < 1) pwr_step = 1;
    logger->info("DEVOURER_TX_PWR ramp — index {}..{} step {} every {} ms",
                 pwr_cur, pwr_stop, pwr_step, pwr_step_ms);
  }
  if (!hop_channels.empty() && pwr_ramp) {
    logger->warn("hop mode active — disabling TX-power ramp (both drive "
                 "SetMonitorChannel)");
    pwr_ramp = false;
  }

  /* Hop bookkeeping: dwell_no counts dwells from 0; frames_in_dwell counts
   * frames sent on the current dwell. When hop_rounds>0 we stop after exactly
   * that many full passes over the channel list. */
  long dwell_no = -1;
  long frames_in_dwell = 0;
  const long total_dwells =
      hop_rounds > 0 ? hop_rounds * static_cast<long>(hop_channels.size()) : 0;

  long tx_count = 0;
  long consec_fail = 0;
  /* If the TX path isn't enabled the chip NAKs every bulk-OUT; hammering a
   * non-draining endpoint for the whole run is what wedged its USB core. Bail
   * out after a short burst of consecutive failures so we shut down cleanly
   * (Stop()) instead of flooding the chip into a -71 hang. */
  /* DEVOURER_TX_MAXFAIL raises the consecutive-failure abort threshold (0 =
   * never abort). Lets a bench run ride out transient bulk-OUT NAK bursts
   * instead of bailing on the first one — e.g. when characterising sustained
   * wide-bandwidth TX — the way the kernel driver (which retries forever) does. */
  long kMaxConsecFail = 8;
  if (const char *mf = std::getenv("DEVOURER_TX_MAXFAIL")) {
    long v = std::atol(mf);
    kMaxConsecFail = (v <= 0) ? LONG_MAX : v; /* LONG_MAX, not std::...::max() —
      MSVC <windows.h> defines a max() macro that would mangle the latter */
  }
  /* Inter-frame delay (default 2 ms ~ 500 fps). DEVOURER_TX_INTERVAL_MS overrides
   * it — used to test whether the Jaguar3 sustained-TX ceiling is wall-clock
   * (timer) or frame-count (per-frame resource) bound. */
  int tx_interval_ms = 2;
  if (const char *iv = std::getenv("DEVOURER_TX_INTERVAL_MS"))
    tx_interval_ms = std::atoi(iv);

  /* DEVOURER_CW_TONE: the CW carrier was armed inside InitWrite (Jaguar-1 or
   * Jaguar-2) — the baseband modulators are off, so there is nothing to
   * transmit. Idle-hold until SIGINT keeps the carrier up, then StopCwTone
   * restores the chip; control falls through to the normal de-init below. */
  if (std::getenv("DEVOURER_CW_TONE")) {
    bool cw = false;
#if defined(DEVOURER_HAVE_JAGUAR1)
    if (jag) cw = true;
#endif
#if defined(DEVOURER_HAVE_JAGUAR2)
    if (jag2) cw = true;
#endif
#if defined(DEVOURER_HAVE_JAGUAR3)
    if (jag3) cw = true;
#endif
    if (cw) {
      logger->info("CW tone hold — idling until SIGINT (Ctrl-C to stop)");
      while (!g_devourer_should_stop)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
#if defined(DEVOURER_HAVE_JAGUAR1)
      if (jag) jag->StopCwTone();
#endif
#if defined(DEVOURER_HAVE_JAGUAR2)
      if (jag2) jag2->StopCwTone();
#endif
#if defined(DEVOURER_HAVE_JAGUAR3)
      if (jag3) jag3->StopCwTone();
#endif
    }
  }

  /* DEVOURER_CONT_TX HW-continuous idle-hold (all generations): the continuous
   * mode holds a 100%-duty modulated carrier once armed. Prime a few frames so
   * the Jaguar1/2 engine has a PPDU to repeat (harmless on Jaguar3, whose PMAC
   * self-generates), then idle-hold — the carrier self-radiates. SIGINT falls
   * through to StopContinuousTx in the de-init below. NB: this is the
   * spectral/thermal stimulus; the link probe uses the plain feed + power ramp
   * (no DEVOURER_CONT_TX) for decodable per-frame SNR. */
  if (cont_tx && cont_hw) {
    for (int i = 0; i < 64 && !g_devourer_should_stop; i++)
      rtlDevice->send_packet(tx_buf.data(), tx_buf.size()); /* prime (NAKs ok) */
    logger->info("DEVOURER_CONT_TX hold — 100%%-duty modulated carrier up, "
                 "idling until SIGINT (Ctrl-C to stop)");
    while (!g_devourer_should_stop)
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  while (!g_devourer_should_stop) {
    if (tx_count == 0) {
      logger->info("init-timing: txdemo.first_tx_submit = {} ms",
                   ms_since_start());
      if (pwr_ramp) {
        apply_txpwr(pwr_cur);
        pwr_next_step_ms = ms_since_start() + pwr_step_ms;
      }
    }
    if (pwr_ramp && pwr_cur < pwr_stop && ms_since_start() >= pwr_next_step_ms) {
      pwr_cur += pwr_step;
      if (pwr_cur > pwr_stop) pwr_cur = pwr_stop;
      apply_txpwr(pwr_cur);
      pwr_next_step_ms = ms_since_start() + pwr_step_ms;
    }
    /* MCS-headroom sweep: apply each rate for one dwell, marking the boundary. */
    if (!mcs_sweep.empty() &&
        (tx_count == 0 || ms_since_start() >= mcs_next_step_ms)) {
      if (tx_count != 0) mcs_idx = (mcs_idx + 1) % mcs_sweep.size();
      rtlDevice->SetTxMode(mcs_sweep[mcs_idx].first);
      printf("<devourer-contx>mcs=%s t_ms=%ld\n",
             mcs_sweep[mcs_idx].second.c_str(), ms_since_start());
      fflush(stdout);
      mcs_next_step_ms = ms_since_start() + mcs_step_ms;
    }
    /* Retune at each dwell boundary. The first iteration (frames_in_dwell==0,
     * dwell_no==-1) selects the first hop channel; SetMonitorChannel
     * early-returns if it equals the InitWrite channel. */
    if (!hop_channels.empty() && frames_in_dwell == 0) {
      ++dwell_no;
      if (total_dwells > 0 && dwell_no >= total_dwells) break;
      int ch = hop_channels[dwell_no % hop_channels.size()];
      auto sw0 = std::chrono::steady_clock::now();
      const char *mode = "";
      if (hop_radiotap) {
        /* Library retunes from the CHANNEL field on the next send_packet; the
         * 802.11 body is beacon_frame past its 10-byte radiotap. */
        tx_buf = build_frame_with_channel(devourer::chan_to_freq(ch),
                                          beacon_frame + 10,
                                          sizeof(beacon_frame) - 10);
        mode = " radiotap";
      }
#if defined(DEVOURER_HAVE_JAGUAR1)
      else if (hop_fast && jag) {
        jag->FastRetune(static_cast<uint8_t>(ch), /*cache_rf=*/hop_fast != 2);
        mode = " fast";
      }
#endif
      else {
        rtlDevice->SetMonitorChannel(SelectedChannel{
            .Channel = static_cast<uint8_t>(ch),
            .ChannelOffset = init_offset,
            .ChannelWidth = init_width});
      }
      long long switch_us =
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now() - sw0)
              .count();
      printf("<devourer-hop>dwell=%ld round=%ld channel=%d frame=%ld "
             "switch_us=%lld t_ms=%lld%s\n",
             dwell_no, dwell_no / static_cast<long>(hop_channels.size()), ch,
             tx_count, switch_us, ms_since_start(), mode);
      fflush(stdout);
    }
    if (stbc_toggle) {
      devourer::TxMode m = tx_mode_base;
      m.stbc = static_cast<uint8_t>(tx_count & 1);   /* 0,1,0,1,… per frame */
      rtlDevice->SetTxMode(m);
    }
    rc = rtlDevice->send_packet(tx_buf.data(), tx_buf.size());
    ++tx_count;
    if (!hop_channels.empty() && ++frames_in_dwell >= hop_dwell)
      frames_in_dwell = 0;
    if (tx_count <= 10 || tx_count % 500 == 0) {
      printf("<devourer-tx>TX #%ld rc=%d\n", tx_count, rc);
      fflush(stdout);
    }
    /* Thermal telemetry is a Jaguar1 (RtlJaguarDevice) feature; skip on
     * Jaguar3, where jag is null; compiled out when Jaguar1 isn't built. */
#if defined(DEVOURER_HAVE_JAGUAR1)
    if (jag && thermal_every > 0 && tx_count % thermal_every == 0) {
      auto t = jag->GetThermalStatus();
      if (t.valid) {
        printf("<devourer-thermal>raw=%u baseline=%u delta=%+d status=%s\n",
               t.raw, t.baseline, t.delta, ThermalBucket(t));
        if (t.delta >= thermal_warn_delta && !thermal_warned) {
          logger->warn("thermal: chip running hot — raw={} baseline={} "
                       "delta=+{} (>= {}); TX power tracking backing off, "
                       "sustained TX may degrade the PA",
                       t.raw, t.baseline, t.delta, thermal_warn_delta);
          thermal_warned = true;
        } else if (t.delta < thermal_warn_delta) {
          thermal_warned = false;
        }
      } else {
        printf("<devourer-thermal>raw=%u baseline=none status=%s\n",
               t.raw, ThermalBucket(t));
      }
      fflush(stdout);
    }
#endif /* DEVOURER_HAVE_JAGUAR1 */
    if (rc) {
      consec_fail = 0;
    } else if (++consec_fail >= kMaxConsecFail) {
      logger->error("TX aborting: {} consecutive bulk-OUT failures — TX path "
                    "not enabled (chip NAKing). Stopping to avoid wedging the "
                    "adapter; clean card-disable follows.",
                    consec_fail);
      break;
    }
    if (tx_gap_set) {
      /* explicit gap wins: >0 sleeps, 0 = no inter-frame sleep (max flood) */
      if (tx_gap_us > 0)
        std::this_thread::sleep_for(std::chrono::microseconds(tx_gap_us));
    } else if (tx_interval_ms > 0)
      std::this_thread::sleep_for(std::chrono::milliseconds(tx_interval_ms));
  }

  /* Bounded hop mode (DEVOURER_HOP_ROUNDS>0) reaches here when its rounds
   * complete; the signal and back-off paths also fall through. */
  if (!hop_channels.empty()) {
    printf("<devourer-hop-done>frames=%ld dwells=%ld\n", tx_count, dwell_no + 1);
    fflush(stdout);
  }

  /* Stop and join the libusb event thread BEFORE any teardown: a thread still
   * inside libusb when libusb_exit runs trips an internal mutex-destroy assertion
   * (core dump). Setting the flag also covers the back-off exit path, where no
   * signal was delivered. */
  g_devourer_should_stop = true;

  /* Undo modulated continuous TX before de-init (clears the 0x914/0x1ca4 hold +
   * BB reset), so the chip returns to normal TX/RX and re-enumerates cleanly. */
  if (cont_tx) {
#if defined(DEVOURER_HAVE_JAGUAR1)
    if (jag) jag->StopContinuousTx();
#endif
#if defined(DEVOURER_HAVE_JAGUAR2)
    if (jag2) jag2->StopContinuousTx();
#endif
#if defined(DEVOURER_HAVE_JAGUAR3)
    if (jag3) jag3->StopContinuousTx();
#endif
  }

  /* Join the RX loop BEFORE Stop(): the chip de-init must not race in-flight
   * RX URBs (and StartRxLoop's event pump must exit before libusb_exit). */
  rtlDevice->StopRxLoop();
  if (rx_thread.joinable())
    rx_thread.join();
  if (usb_thread.joinable())
    usb_thread.join();

  /* Clean chip de-init before releasing the interface (card-disable PWR_SEQ), so
   * the adapter re-enumerates instead of hanging its USB core. */
  rtlDevice->Stop();

  /* Tolerant teardown: if the chip already dropped off the bus (e.g. a TX-path
   * wedge), release_interface returns an error — log it, don't assert/abort. */
  rc = libusb_release_interface(handle, 0);
  if (rc != 0)
    logger->info("libusb_release_interface rc={} (device already gone?)", rc);

  libusb_close(handle);
  libusb_exit(context);
  return 0;
}
