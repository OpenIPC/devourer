// StreamDuplexDemo — single-chip full-duplex for the precoder stream link.
//
// Combines WiFiDriverDemo's RX loop (Init → infinite_read → packet callback)
// with StreamTxDemo's stdin-driven TX (read length-prefixed PSDU body →
// send_packet) on ONE claimed interface. RX runs in the main thread; TX in a
// worker thread reads stdin and calls send_packet concurrently. libusb is
// thread-safe; the two bulk endpoints (_bulk_in_ep, _bulk_out_ep) don't share
// transfer state.
//
// Used by tools/precoder/tun_p2p.py in --mode=duplex with a single PID per
// peer. Replaces the StreamTxDemo + WiFiDriverDemo pair (one adapter per
// direction) with a single binary per peer (one adapter per peer, ergo two
// adapters total for a P2P link instead of four).
//
// On-wire wire format on stdin is identical to StreamTxDemo:
//     <u32_le length><length bytes of descrambled PSDU body>
// EOF on stdin closes the TX side cleanly; RX keeps running until the process
// terminates.
//
// RX emission on stdout mirrors demo/main.cpp's DEVOURER_STREAM_OUT path —
// `<devourer-stream>rate=R len=L body=HEX` for every frame matching the
// canonical SA. Other stdout output is suppressed; stderr carries logger and
// counters.

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#if defined(_MSC_VER)
  #include <io.h>
  #include <fcntl.h>
  #include <windows.h>
  typedef int pid_t;
  #define sleep(seconds) Sleep((seconds)*1000)
#elif defined(__ANDROID__)
  #include <libusb.h>
  #include <unistd.h>
#elif defined(__APPLE__)
  #include <unistd.h>
  #include <libusb.h>
#else
  #include <unistd.h>
  #include <libusb-1.0/libusb.h>
#endif

#include "FrameParser.h"
#include "RtlUsbAdapter.h"
#include "WiFiDriver.h"
#include "logger.h"

#define USB_VENDOR_ID 0x0bda

static constexpr uint16_t kRealtekProductIds[] = {
    0x8812, 0x0811, 0xa811, 0xb811, 0x8813,
};

// Same radiotap + probe-request header as StreamTxDemo / PrecoderDemo. The
// canonical SA matcher in the packet processor below is identical to
// demo/main.cpp's, so any tooling that already grep'd <devourer-stream>
// lines keeps working unchanged.
static const uint8_t kRadiotapLegacy6M[13] = {
    0x00, 0x00, 0x0d, 0x00, 0x04, 0x80, 0x00,
    0x00, 0x0c, 0x00, 0x08, 0x00, 0x00};
static const uint8_t kCanonicalSa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};

static std::vector<uint8_t> build_dot11_probe_req() {
  std::vector<uint8_t> h = {
      0x40, 0x00, 0x00, 0x00,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  };
  h.insert(h.end(), kCanonicalSa, kCanonicalSa + 6);
  h.insert(h.end(), kCanonicalSa, kCanonicalSa + 6);
  h.push_back(0x80);
  h.push_back(0x00);
  return h;
}

static bool read_exact(FILE *f, void *buf, size_t n) {
  size_t got = 0;
  auto *p = static_cast<uint8_t *>(buf);
  while (got < n) {
    size_t r = std::fread(p + got, 1, n - got, f);
    if (r == 0) {
      if (got == 0 && std::feof(f)) return false;
      std::fprintf(stderr,
                   "stream_duplex_demo: short stdin read (%zu/%zu)\n", got, n);
      return false;
    }
    got += r;
  }
  return true;
}

// RX callback — emits `<devourer-stream>` on canonical-SA matches. Wrapped in
// a mutex against the TX thread's printf calls so the two log streams don't
// interleave mid-line. (The TX thread only writes to stderr, RX to stdout, so
// in practice they don't collide, but keeping the mutex is cheap.)
static std::mutex g_print_mu;
static std::atomic<long> g_rx_hits{0};

static void packet_processor(const Packet &packet) {
  if (packet.Data.size() < 16) return;
  if (std::memcmp(packet.Data.data() + 10, kCanonicalSa, 6) != 0) return;
  long hits = ++g_rx_hits;
  std::lock_guard<std::mutex> lk(g_print_mu);
  std::printf("<devourer-stream>rate=%u len=%zu body=",
              packet.RxAtrib.data_rate, packet.Data.size());
  for (size_t i = 24; i < packet.Data.size(); ++i)
    std::printf("%02x", packet.Data[i]);
  std::printf("\n");
  std::fflush(stdout);
  if (hits <= 5 || hits % 500 == 0) {
    std::fprintf(stderr, "<stream-duplex>rx hits=%ld\n", hits);
    std::fflush(stderr);
  }
}

struct TxArgs {
  class RtlJaguarDevice *rtl;  // unique_ptr lives in main(); raw ptr OK while
                                // we join() before that unique_ptr goes away
  int interval_ms;
  size_t max_psdu;
  std::atomic<bool> *should_stop;
  std::shared_ptr<Logger> logger;
};

static void tx_thread(TxArgs args) {
  auto dot11 = build_dot11_probe_req();
  std::vector<uint8_t> tx_buf;
  tx_buf.reserve(sizeof(kRadiotapLegacy6M) + dot11.size() + args.max_psdu);
  long tx_count = 0;

  while (!args.should_stop->load()) {
    uint8_t len_bytes[4];
    if (!read_exact(stdin, len_bytes, sizeof(len_bytes))) {
      // Clean EOF or short read — TX side done. RX keeps running.
      std::fprintf(stderr, "<stream-duplex>tx EOF after %ld PSDUs\n", tx_count);
      break;
    }
    uint32_t len = static_cast<uint32_t>(len_bytes[0])
                 | (static_cast<uint32_t>(len_bytes[1]) << 8)
                 | (static_cast<uint32_t>(len_bytes[2]) << 16)
                 | (static_cast<uint32_t>(len_bytes[3]) << 24);
    if (len == 0 || len > args.max_psdu) {
      std::fprintf(stderr,
                   "<stream-duplex>tx PSDU len %u out of range (max %zu)\n",
                   len, args.max_psdu);
      break;
    }
    std::vector<uint8_t> psdu(len);
    if (!read_exact(stdin, psdu.data(), len)) {
      std::fprintf(stderr, "<stream-duplex>tx EOF mid-PSDU (%u bytes)\n", len);
      break;
    }
    tx_buf.clear();
    tx_buf.insert(tx_buf.end(), kRadiotapLegacy6M,
                  kRadiotapLegacy6M + sizeof(kRadiotapLegacy6M));
    tx_buf.insert(tx_buf.end(), dot11.begin(), dot11.end());
    tx_buf.insert(tx_buf.end(), psdu.begin(), psdu.end());
    bool ok = args.rtl->send_packet(tx_buf.data(), tx_buf.size());
    ++tx_count;
    if (tx_count <= 5 || tx_count % 500 == 0) {
      std::fprintf(stderr,
                   "<stream-duplex>tx #%ld ok=%d psdu=%u\n",
                   tx_count, ok ? 1 : 0, len);
      std::fflush(stderr);
    }
    if (args.interval_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(args.interval_ms));
    }
  }
}

int main(int argc, char **argv) {
  auto logger = std::make_shared<Logger>();

  int interval_ms = 2;
  size_t max_psdu = 4096;
  long termux_fd = 0;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--interval-ms" && i + 1 < argc) {
      interval_ms = std::atoi(argv[++i]);
    } else if (a == "--max-psdu" && i + 1 < argc) {
      max_psdu = static_cast<size_t>(std::strtoul(argv[++i], nullptr, 0));
    } else {
      char *end = nullptr;
      long v = std::strtol(a.c_str(), &end, 0);
      if (end && *end == '\0' && v > 0) termux_fd = v;
    }
  }

#if defined(_MSC_VER)
  _setmode(_fileno(stdin), _O_BINARY);
#endif

  libusb_context *context = nullptr;
  libusb_device_handle *handle = nullptr;
  int rc;

  if (termux_fd > 0) {
    libusb_set_option(NULL, LIBUSB_OPTION_NO_DEVICE_DISCOVERY);
    libusb_set_option(NULL, LIBUSB_OPTION_WEAK_AUTHORITY);
    libusb_init(&context);
    rc = libusb_wrap_sys_device(context, (intptr_t)termux_fd, &handle);
    if (rc < 0) {
      logger->error("libusb_wrap_sys_device: {}", rc);
      return 1;
    }
  } else {
    rc = libusb_init(&context);
    if (rc < 0) return rc;
    /* Match WiFiDriverDemo's libusb log level convention — DEVOURER_USB_QUIET
     * drops to WARNING so a long-running duplex doesn't flood the harness. */
    libusb_set_option(context, LIBUSB_OPTION_LOG_LEVEL,
                      std::getenv("DEVOURER_USB_QUIET")
                          ? LIBUSB_LOG_LEVEL_WARNING
                          : LIBUSB_LOG_LEVEL_INFO);
    uint16_t target_pid = 0;
    if (const char *pid_env = std::getenv("DEVOURER_PID")) {
      target_pid = static_cast<uint16_t>(std::strtoul(pid_env, nullptr, 0));
    }
    uint16_t target_vid = USB_VENDOR_ID;
    if (const char *vid_env = std::getenv("DEVOURER_VID")) {
      target_vid = static_cast<uint16_t>(std::strtoul(vid_env, nullptr, 0));
    }
    for (uint16_t pid : kRealtekProductIds) {
      if (target_pid != 0 && pid != target_pid) continue;
      handle = libusb_open_device_with_vid_pid(context, target_vid, pid);
      if (handle != NULL) {
        logger->info("Opened device {:04x}:{:04x}", target_vid, pid);
        break;
      }
    }
    if (handle == NULL && target_pid != 0) {
      handle = libusb_open_device_with_vid_pid(context, target_vid, target_pid);
    }
    if (handle == NULL) {
      logger->error("No supported device found under VID {:04x}", target_vid);
      libusb_exit(context);
      return 1;
    }
  }

  if (libusb_kernel_driver_active(handle, 0)) {
    libusb_detach_kernel_driver(handle, 0);
  }
  if (termux_fd == 0 && !std::getenv("DEVOURER_SKIP_RESET")) {
    libusb_reset_device(handle);
  }
  rc = libusb_claim_interface(handle, 0);
  assert(rc == 0);

  WiFiDriver wifi_driver{logger};
  auto rtlDevice = wifi_driver.CreateRtlDevice(handle);

  int channel = 6;
  if (const char *ch_env = std::getenv("DEVOURER_CHANNEL")) {
    channel = std::atoi(ch_env);
  }
  rtlDevice->SetTxPower(40);

  std::atomic<bool> should_stop{false};

  // Spawn TX thread first; it'll block on stdin until our peer pushes a
  // length-prefixed PSDU. Then drop into Init() (the RX loop) in the main
  // thread.
  TxArgs txa{rtlDevice.get(), interval_ms, max_psdu, &should_stop, logger};
  std::thread tx{tx_thread, std::move(txa)};

  logger->info("StreamDuplexDemo entering RX loop on ch {} — TX thread ready",
               channel);
  // RX loop. Same Init() path as WiFiDriverDemo; SelectedChannel sets up the
  // shared monitor-mode bring-up (StartWithMonitorMode + SetMonitorChannel).
  rtlDevice->Init(packet_processor,
                  SelectedChannel{.Channel = static_cast<uint8_t>(channel),
                                  .ChannelOffset = 0,
                                  .ChannelWidth = CHANNEL_WIDTH_20});

  // Init() returns only on should_stop (set by signal handler in the future
  // — none wired here, so Ctrl-C ends the process abruptly and the OS reaps
  // the TX thread).
  should_stop = true;
  if (tx.joinable()) tx.join();
  libusb_release_interface(handle, 0);
  libusb_close(handle);
  libusb_exit(context);
  return 0;
}
