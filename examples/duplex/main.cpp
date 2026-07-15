// duplex — single-chip full-duplex for the precoder stream link.
//
// Combines rxdemo's RX loop (Init → infinite_read → packet callback)
// with streamtx's stdin-driven TX (read length-prefixed PSDU body →
// send_packet) on ONE claimed interface. RX runs in the main thread; TX in a
// worker thread reads stdin and calls send_packet concurrently. libusb is
// thread-safe; the two bulk endpoints (_bulk_in_ep, _bulk_out_ep) don't share
// transfer state.
//
// Used by tools/precoder/tun_p2p.py in --mode=duplex with a single PID per
// peer. Replaces the streamtx + rxdemo pair (one adapter per
// direction) with a single binary per peer (one adapter per peer, ergo two
// adapters total for a P2P link instead of four).
//
// On-wire wire format on stdin is identical to streamtx:
//     <u32_le length><length bytes of descrambled PSDU body>
// EOF on stdin closes the TX side cleanly; RX keeps running until the process
// terminates.
//
// RX emission on stdout mirrors examples/rx/main.cpp's DEVOURER_STREAM_OUT path —
// one `rx.frame` JSONL event for every frame matching the canonical SA.
// stdout is the JSONL event plane (stream.* control telemetry included);
// stderr carries the human diagnostics (logger).

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
  /* libusb.h explicitly: the pre-seam RtlUsbAdapter.h used to pull it in
   * for every consumer; the bus-neutral RtlAdapter.h no longer does. */
  #include <libusb.h>
  #include <io.h>
  #include <fcntl.h>
  #include <windows.h>
  typedef int pid_t;
  #define sleep(seconds) Sleep((seconds)*1000)
#elif defined(__MINGW32__) || defined(__MINGW64__)
  // mingw builds: POSIX libusb/unistd PLUS io.h/fcntl.h for binary stdin.
  #include <io.h>
  #include <fcntl.h>
  #include <unistd.h>
  #include <libusb-1.0/libusb.h>
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

#include "RxPacket.h"
#include "RadiotapBuilder.h"
#include "RtlAdapter.h"
#if defined(DEVOURER_HAVE_JAGUAR1)
#include "jaguar1/RtlJaguarDevice.h"
#endif
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "env_config.h"
#include "logger.h"
#include "stream_stdin.h"

#define USB_VENDOR_ID 0x0bda

static constexpr uint16_t kRealtekProductIds[] = {
    0x8812, 0x0811, 0xa811, 0xb811, 0x8813,
};

// Same probe-request header as streamtx / precoder; radiotap is now
// built once at startup from DEVOURER_STREAM_RATE — accepts legacy
// (6M..54M), HT (MCS0..MCS31), or VHT (VHT1SS_MCS0..VHT4SS_MCS9) carrier
// modes. Default is 6M legacy OFDM, bit-identical to the historic
// kRadiotapLegacy6M constant. The canonical SA matcher in the packet
// processor below is identical to examples/rx/main.cpp's, so tooling that
// consumes rx.frame events sees the same frames from either demo.
// Radiotap is MUTABLE here (the adaptive link rewrites the on-air rate live via
// the stdin SET_RATE control op). Guarded by g_rt_mu against the TX thread.
static std::mutex g_rt_mu;
static std::vector<uint8_t> g_radiotap =
    devourer::build_stream_radiotap(devourer_tx_mode_from_env());
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

// RX callback — emits an `rx.frame` event on canonical-SA matches. Event
// lines are emitted atomically (one fwrite per line, see src/Event.h), so no
// print mutex is needed against the TX thread's emissions.
static std::atomic<long> g_rx_hits{0};

/* Event sink for the demo's JSONL emissions (packet_processor and tx_thread
 * are free functions) — points at the main() Logger's sink, set before the
 * TX thread spawns / Init() runs. */
static devourer::EventSink *g_ev = nullptr;

/* DEVOURER_TX_STATUS=1: surface chip-side C2H frames (TX-status reports
 * from the same 8812/8821 chip we're TXing on). Best-effort 8814A TX_RPT
 * decode mirrors examples/rx/main.cpp; the C2H sub-type ID isn't enumerated in
 * the vendored headers so the raw hex stays in the line. */
static const bool g_tx_status_enabled =
    std::getenv("DEVOURER_TX_STATUS") != nullptr;

static void packet_processor(const Packet &packet) {
  if (packet.RxAtrib.pkt_rpt_type == RX_PACKET_TYPE::C2H_PACKET) {
    if (!g_tx_status_enabled) return;
    devourer::Ev(*g_ev, "fw.c2h")
        .f("len", packet.Data.size())
        .hex("bytes", packet.Data.data(), packet.Data.size());
    if (packet.Data.size() >= 8) {
      for (size_t hoff : {size_t(1), size_t(2)}) {
        if (packet.Data.size() < hoff + 6) continue;
        const uint8_t *h = packet.Data.data() + hoff;
        uint8_t  queue   = h[0] & 0x1f;
        uint8_t  retry   = h[2] & 0x3f;
        uint16_t qt_raw  = static_cast<uint16_t>(h[3] | (h[4] << 8));
        uint32_t qt_us   = static_cast<uint32_t>(qt_raw) * 256u;
        uint8_t  rate    = h[5];
        devourer::Ev(*g_ev, "tx.status")
            .f("hoff", hoff)
            .f("queue", queue)
            .f("retry", retry)
            .f("airtime_us", qt_us)
            .f("rate", rate);
      }
    }
    return;
  }
  if (packet.Data.size() < 16) return;
  if (std::memcmp(packet.Data.data() + 10, kCanonicalSa, 6) != 0) return;
  long hits = ++g_rx_hits;
  // Full field set (mirrors examples/rx/main.cpp's rx.frame) so the adaptive
  // VRX can score RSSI/SNR and the VTX can read RCF/DISC bodies + ACK_SEQ.
  {
    const int rssi[2] = {packet.RxAtrib.rssi[0], packet.RxAtrib.rssi[1]};
    const int evm[2] = {packet.RxAtrib.evm[0], packet.RxAtrib.evm[1]};
    const int snr[2] = {packet.RxAtrib.snr[0], packet.RxAtrib.snr[1]};
    const size_t body_len =
        packet.Data.size() > 24 ? packet.Data.size() - 24 : 0;
    devourer::Ev(*g_ev, "rx.frame")
        .f("rate", packet.RxAtrib.data_rate)
        .f("len", packet.Data.size())
        .f("crc", packet.RxAtrib.crc_err ? 1 : 0)
        .f("icv", packet.RxAtrib.icv_err ? 1 : 0)
        .arr("rssi", rssi, 2)
        .arr("evm", evm, 2)
        .arr("snr", snr, 2)
        .f("seq", packet.RxAtrib.seq_num)
        .f("tsfl", packet.RxAtrib.tsfl)
        .f("bw", packet.RxAtrib.bw)
        .f("stbc", packet.RxAtrib.stbc)
        .f("ldpc", packet.RxAtrib.ldpc)
        .f("sgi", packet.RxAtrib.sgi)
        .hex("body", packet.Data.data() + 24, body_len);
  }
  if (hits <= 5 || hits % 500 == 0) {
    devourer::Ev(*g_ev, "stream.rx").f("hits", hits);
  }
}

struct TxArgs {
  class IRtlDevice *rtl;  // unique_ptr lives in main(); raw ptr OK while
                          // we join() before that unique_ptr goes away
  int interval_ms;
  size_t max_psdu;
  std::atomic<bool> *should_stop;
  std::shared_ptr<Logger> logger;
};

static void tx_thread(TxArgs args) {
  auto dot11 = build_dot11_probe_req();
  std::vector<uint8_t> tx_buf;
  tx_buf.reserve(g_radiotap.size() + dot11.size() + args.max_psdu);
  long tx_count = 0;

  while (!args.should_stop->load()) {
    uint8_t len_bytes[4];
    if (stream_stdin::read_exact(stdin, len_bytes, sizeof(len_bytes)) !=
        stream_stdin::ReadResult::Ok) {
      // Clean EOF or short read — TX side done. RX keeps running.
      devourer::Ev(*g_ev, "stream.eof").f("tx_count", tx_count);
      break;
    }
    uint32_t len = static_cast<uint32_t>(len_bytes[0])
                 | (static_cast<uint32_t>(len_bytes[1]) << 8)
                 | (static_cast<uint32_t>(len_bytes[2]) << 16)
                 | (static_cast<uint32_t>(len_bytes[3]) << 24);

    // Control-opcode escape: top bit set -> the body is a control TLV (the
    // adaptive link's live knobs), not a PSDU. <op:u8><payload...>.
    if (len & 0x80000000u) {
      uint32_t clen = len & 0x7fffffffu;
      if (clen == 0 || clen > 256) break;
      std::vector<uint8_t> ctl(clen);
      if (stream_stdin::read_exact(stdin, ctl.data(), clen) !=
          stream_stdin::ReadResult::Ok)
        break;
      uint8_t op = ctl[0];
      if (op == 1 && clen >= 2) {                 // SET_PWR <idx>
        /* Flat TXAGC override via the generation-agnostic runtime TX-power
         * API (previously Jaguar1-only): applies live on every family. */
        args.rtl->SetTxPowerIndexOverride(ctl[1]);
      } else if (op == 2 && clen >= 2) {          // SET_RATE <spec ascii>
        std::string spec(ctl.begin() + 1, ctl.end());
        auto rt = devourer::build_stream_radiotap(devourer::parse_tx_mode_str(spec));
        std::lock_guard<std::mutex> lk(g_rt_mu);
        g_radiotap = std::move(rt);
      } else if (op == 3 && clen >= 4) {          // SET_CHAN <ch><offset><width>
        args.rtl->SetMonitorChannel(SelectedChannel{
            .Channel = ctl[1], .ChannelOffset = ctl[2],
            .ChannelWidth = static_cast<ChannelWidth_t>(ctl[3])});
      }
      devourer::Ev(*g_ev, "stream.ctl").f("op", op).f("len", clen);
      continue;
    }

    if (len == 0 || len > args.max_psdu) {
      args.logger->error("tx PSDU len {} out of range (max {})", len,
                         args.max_psdu);
      break;
    }
    std::vector<uint8_t> psdu(len);
    if (stream_stdin::read_exact(stdin, psdu.data(), len) !=
        stream_stdin::ReadResult::Ok) {
      /* EOF mid-PSDU: `bytes` = the expected PSDU length that was cut short. */
      devourer::Ev(*g_ev, "stream.eof").f("tx_count", tx_count).f("bytes", len);
      break;
    }
    tx_buf.clear();
    {
      std::lock_guard<std::mutex> lk(g_rt_mu);   // live rate may be rewritten
      tx_buf.insert(tx_buf.end(), g_radiotap.begin(), g_radiotap.end());
    }
    tx_buf.insert(tx_buf.end(), dot11.begin(), dot11.end());
    tx_buf.insert(tx_buf.end(), psdu.begin(), psdu.end());
    bool ok = args.rtl->send_packet(tx_buf.data(), tx_buf.size());
    ++tx_count;
    if (tx_count <= 5 || tx_count % 500 == 0) {
      devourer::Ev(*g_ev, "stream.tx")
          .f("n", tx_count)
          .f("ok", ok ? 1 : 0)
          .f("psdu", len);
    }
    if (args.interval_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(args.interval_ms));
    }
  }
}

int main(int argc, char **argv) {
  auto logger = std::make_shared<Logger>();
  apply_logging_env(*logger); /* DEVOURER_LOG_LEVEL / DEVOURER_EVENTS / ... */
  g_ev = &logger->events();

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

  // Make stdin binary so a 0x1A/CRLF doesn't corrupt the length-prefixed PSDU
  // stream. Gated on _WIN32 (not _MSC_VER) in the shared helper — see
  // examples/common/stream_stdin.h.
  stream_stdin::set_stdin_binary();

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
    /* Match rxdemo's libusb log level convention — WARNING by
     * default, DEVOURER_USB_DEBUG=1 opts into DEBUG. */
    libusb_set_option(context, LIBUSB_OPTION_LOG_LEVEL,
                      std::getenv("DEVOURER_USB_DEBUG")
                          ? LIBUSB_LOG_LEVEL_DEBUG
                          : LIBUSB_LOG_LEVEL_WARNING);
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

  /* Claim-before-reset (see src/UsbOpen.h): the exclusive claim is the primary
   * guard — a second devourer on this adapter gets BUSY here and bails before
   * the reset, so it can't re-enumerate the adapter out from under the owner. */
  std::shared_ptr<devourer::UsbDeviceLock> usb_lock;
  rc = devourer::claim_interface_then_reset(handle, devourer::find_wifi_interface(handle), logger,
      termux_fd == 0 && std::getenv("DEVOURER_SKIP_RESET") == nullptr, usb_lock);
  if (rc != 0) {
    libusb_close(handle);
    libusb_exit(context);
    return 1;
  }

  WiFiDriver wifi_driver{logger};
  auto rtlDevice = wifi_driver.CreateRtlDevice(handle, nullptr, usb_lock,
                                               devourer_config_from_env());

  int channel = 6;
  if (const char *ch_env = std::getenv("DEVOURER_CHANNEL")) {
    channel = std::atoi(ch_env);
  }
  /* DEVOURER_TX_POWER: flat TXAGC index (see streamtx). Unset = each
   * family's calibrated default — SetTxPower is now a real flat override on
   * EVERY generation, so the old unconditional SetTxPower(40) (a no-op on
   * Jaguar1/2) is gone. */
  if (const char *p = std::getenv("DEVOURER_TX_POWER"))
    rtlDevice->SetTxPower(static_cast<uint8_t>(std::atoi(p)));

  std::atomic<bool> should_stop{false};

  // Spawn TX thread first; it'll block on stdin until our peer pushes a
  // length-prefixed PSDU. Then drop into Init() (the RX loop) in the main
  // thread.
  TxArgs txa{rtlDevice.get(), interval_ms, max_psdu, &should_stop, logger};
  std::thread tx{tx_thread, std::move(txa)};

  logger->info("duplex entering RX loop on ch {} — TX thread ready",
               channel);
  // RX loop. Same Init() path as rxdemo; SelectedChannel sets up the
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
