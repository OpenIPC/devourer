/* busy_window_probe — the on-air half of IRadio::ArmChannelBusy.
 *
 * Drives the armed window against whatever the bench is transmitting and
 * prints one machine-readable line per sample, so tests/busy_window_probe.sh
 * can assert the separations the feature claims. It uses only the public
 * contract (ArmChannelBusy + GetChannelBusy), which is the point: if a
 * consumer cannot reproduce these numbers through that pair, the contract is
 * wrong.
 *
 * Arms (--mode):
 *   sampled  GetChannelBusy() with no window armed — the shipped ~2 ms sample,
 *            for the spread comparison.
 *   window   arm, wait, read. The feature.
 *   interrupt  arm, take an NHM read mid-window, read. Must come back INVALID
 *            with spoil=interrupted: the reading is destroyed on the JGR3 map
 *            and 3-4 points high on the 11AC map, and neither is this window.
 *   retune   arm, retune mid-window, read. Must be INVALID (spoil=retuned):
 *            the counter runs across the channel change.
 *   early    arm, read immediately. Must be INVALID (spoil=not-elapsed)
 *            rather than the previous window's latched value.
 *   txsess   arm, TRANSMIT inside the window, read. Must stay valid and be
 *            flagged own_tx: Realtek reads low (the receiver is deaf while
 *            the PA is up), MediaTek reads high (it counts own airtime).
 *   stale    arm, let it COMPLETE, read it, arm again, read immediately. The
 *            second read must be INVALID. The result register latches the
 *            previous window, so this only holds if the trigger clears the
 *            ready bit - an assumption the headless selftest can only model,
 *            which is why it is checked here against real silicon.
 *   race     a second thread hammers GetRxQuality() for the whole window while
 *            this one arms and reads. Every reading must be either refused as
 *            interrupted or a full-length window — never a short one wearing a
 *            valid flag. This is the on-air check of the locking: without it
 *            the note could land before a concurrent arm while the re-arm
 *            lands after it, and a destroyed window reads back as data.
 *   quality  like `interrupt`, but through GetRxQuality() instead of
 *            GetRxEnergy(). That is the call a consumer actually makes, and
 *            it is how a survey dwell springs the trap without ever touching
 *            the busy API.
 *
 * Exit 0 = ran, 3 = no adapter, 5 = this backend cannot arm a window.
 */
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#if __has_include(<libusb.h>)
#include <libusb.h>
#else
#include <libusb-1.0/libusb.h>
#endif

#include "AdapterCaps.h"
#include "DeviceSession.h"
#include "IRtlRadio.h"
#include "RadiotapBuilder.h"
#include "RxPacket.h"
#include "TxMode.h"
#include "WiFiDriver.h"
#include "logger.h"

namespace {

void nap(uint32_t ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

const char *spoil_name(devourer::BusySpoil s) {
  switch (s) {
  case devourer::BusySpoil::Interrupted:
    return "interrupted";
  case devourer::BusySpoil::Retuned:
    return "retuned";
  case devourer::BusySpoil::NotElapsed:
    return "not-elapsed";
  default:
    return "none";
  }
}

const char *source_name(devourer::BusySource s) {
  switch (s) {
  case devourer::BusySource::Clm:
    return "clm";
  case devourer::BusySource::ChTime:
    return "chtime";
  default:
    return "none";
  }
}

void emit(const char *mode, int i, const devourer::ChannelBusy &b,
          uint32_t armed_us, uint32_t sent) {
  /* No padding inside a key=value token: the harness parses these fields, and
   * "busy= 71%" splits into two. */
  std::printf("BUSY mode=%-9s i=%2d valid=%d busy=%u%% src=%-6s window_us=%u "
              "armed_us=%u spoil=%-11s own_tx=%d tx_frames=%u sent=%u\n",
              mode, i, b.valid ? 1 : 0, b.busy_pct, source_name(b.source),
              b.window_us, armed_us, spoil_name(b.spoil),
              b.own_tx_in_window ? 1 : 0, b.own_tx_frames, sent);
  std::fflush(stdout);
}

/* Every exit from main() after the RX thread is running must go through this.
 * That thread is detached and may still be inside Init/StartRxLoop, so letting
 * DeviceSession's destructor close the handle under it is a use-after-free —
 * which is exactly what the SKIP returns used to do. _exit runs no
 * destructors, so the USB lock is dropped by hand first: one left behind makes
 * the next run of this script refuse the adapter it just used. */
struct Cleanup {
  IRadio *dev = nullptr;
  std::shared_ptr<devourer::UsbDeviceLock> *lock = nullptr;
  devourer::DeviceSession *session = nullptr;
  bool rx_on = false;
};
Cleanup g_cleanup;

int finish(int code) {
  std::fflush(stdout);
  if (!g_cleanup.rx_on)
    return code;
  if (g_cleanup.dev)
    g_cleanup.dev->StopRxLoop();
  if (g_cleanup.lock)
    g_cleanup.lock->reset();
  if (g_cleanup.session)
    g_cleanup.session->release_lock();
  _exit(code);
}

} // namespace

int main(int argc, char **argv) {
  uint16_t vid = 0x0bda, pid = 0x8812;
  int channel = 165, other = 100, reps = 8, window_ms = 240, rx_on = 0;
  std::string mode = "window";
  for (int i = 1; i < argc; i++) {
    if (!std::strcmp(argv[i], "--vid") && i + 1 < argc)
      vid = (uint16_t)std::strtoul(argv[++i], nullptr, 0);
    else if (!std::strcmp(argv[i], "--pid") && i + 1 < argc)
      pid = (uint16_t)std::strtoul(argv[++i], nullptr, 0);
    else if (!std::strcmp(argv[i], "--channel") && i + 1 < argc)
      channel = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--other") && i + 1 < argc)
      other = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--reps") && i + 1 < argc)
      reps = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--window-ms") && i + 1 < argc)
      window_ms = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--rx"))
      rx_on = 1;
    else if (!std::strcmp(argv[i], "--mode") && i + 1 < argc)
      mode = argv[++i];
    else {
      std::fprintf(stderr,
                   "usage: %s [--vid N --pid N] [--channel N] [--other N] "
                   "[--reps N] [--window-ms N] [--rx] "
                   "[--mode sampled|window|interrupt|retune|early|txsess]\n",
                   argv[0]);
      return 2;
    }
  }

  auto logger = std::make_shared<Logger>();
  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) < 0) {
    std::fprintf(stderr, "libusb_init failed\n");
    return 3;
  }
  devourer::DeviceSession session(logger);
  libusb_device_handle *handle = libusb_open_device_with_vid_pid(ctx, vid, pid);
  if (!handle) {
    std::fprintf(stderr, "no adapter %04x:%04x\n", vid, pid);
    return 3;
  }
  std::shared_ptr<devourer::UsbDeviceLock> lock;
  if (devourer::claim_interface_then_reset(
          handle, devourer::find_wifi_interface(handle), logger,
          /*do_reset=*/true, lock) != 0) {
    session.adopt_handle(handle);
    return 3;
  }
  session.adopt_handle(handle);
  session.adopt_lock(lock);

  devourer::DeviceConfig cfg;
  WiFiDriver driver(logger);
  std::unique_ptr<IRadio> owned = driver.CreateRadio(handle, ctx, lock, cfg);
  if (!owned) {
    std::fprintf(stderr, "CreateRadio failed (chip support not built?)\n");
    return 3;
  }
  session.adopt_device(std::move(owned));
  IRadio *const dev = session.device();
  auto *const rtl = dynamic_cast<IRtlRadio *>(dev);

  const devourer::AdapterCaps caps = dev->GetAdapterCaps();
  std::printf("BUSY-GEN %s chip=%s busy_airtime_ok=%d busy_airtime_measured=%d "
              "rx_energy_ok=%d\n",
              devourer::generation_name(caps.generation), caps.chip_name,
              caps.busy_airtime_ok ? 1 : 0, caps.busy_airtime_measured ? 1 : 0,
              caps.rx_energy_ok ? 1 : 0);
  std::fflush(stdout);

  const SelectedChannel chan_def{.Channel = static_cast<uint8_t>(channel),
                                 .ChannelOffset = 0,
                                 .ChannelWidth = CHANNEL_WIDTH_20};
  std::thread rx_thread;
  if (rx_on) {
    /* The MediaTek MAC only accumulates busy time with the receiver running,
     * so its arm is only meaningful under --rx. Init does not return on that
     * backend (it drives the RX path inline), so it gets its own thread and
     * the measurement runs on this one — the radio serialises register access
     * internally. */
    rx_thread = std::thread([dev, chan_def]() {
      dev->Init([](const Packet &) {}, chan_def);
    });
    /* Detached immediately: Init does not return on that backend, so joining
     * it would hang and letting the std::thread destructor see it joinable
     * would abort the process. */
    rx_thread.detach();
    g_cleanup.dev = dev;
    g_cleanup.lock = &lock;
    g_cleanup.session = &session;
    g_cleanup.rx_on = true;
    /* Bring-up finishes asynchronously, and the arm refuses until the
     * receiver is actually running, so wait for it rather than guessing. */
    /* Bring-up finishes asynchronously and the arm refuses until the receiver
     * is actually running, so wait for it rather than guessing. Note the
     * MT7612U does not always get there while the channel is already
     * saturated — bring it up before the interferer, not after. */
    for (int t = 0; t < 60; t++) {
      nap(250);
      if (dev->ArmChannelBusy(1000) != 0) {
        /* Consume it: an armed window left behind would make the first
         * `sampled` read take the ARMED branch and report a 1 ms window as
         * the shipped sampled path. */
        nap(5);
        (void)dev->GetChannelBusy();
        break;
      }
    }
  } else {
    dev->InitWrite(chan_def);
    nap(200);
  }

  const uint32_t window_us = static_cast<uint32_t>(window_ms) * 1000u;

  for (int i = 1; i <= reps; i++) {
    uint32_t sent = 0;

    if (mode == "sampled") {
      const devourer::ChannelBusy b = dev->GetChannelBusy();
      emit("sampled", i, b, 0, 0);
      nap(300); /* a survey dwell cadence */
      continue;
    }

    const uint32_t armed = dev->ArmChannelBusy(window_us);
    if (armed == 0) {
      std::printf("SKIP backend cannot arm a busy window\n");
      return finish(5);
    }
    /* Wait the window the hardware actually granted, not the one requested. */
    const uint32_t wait_ms = armed / 1000u + 15u;

    if (mode == "interrupt") {
      nap(wait_ms / 2);
      if (rtl)
        (void)rtl->GetRxEnergy(/*with_nhm=*/true);
      nap(wait_ms / 2);
    } else if (mode == "retune") {
      nap(wait_ms / 2);
      dev->SetMonitorChannel(
          SelectedChannel{.Channel = static_cast<uint8_t>(other),
                          .ChannelOffset = 0,
                          .ChannelWidth = CHANNEL_WIDTH_20});
      nap(wait_ms / 2);
    } else if (mode == "race") {
      std::atomic<bool> stop{false};
      std::thread hammer([&]() {
        while (!stop.load(std::memory_order_relaxed)) {
          if (rtl)
            (void)rtl->GetRxQuality();
        }
      });
      nap(wait_ms);
      stop.store(true, std::memory_order_relaxed);
      hammer.join();
    } else if (mode == "quality") {
      nap(wait_ms / 2);
      if (rtl)
        (void)rtl->GetRxQuality();
      nap(wait_ms / 2);
    } else if (mode == "early") {
      nap(5);
    } else if (mode == "stale") {
      /* Let this window finish and consume it, so the result register holds a
       * completed measurement; then arm again and read at once. */
      nap(wait_ms);
      const devourer::ChannelBusy done = dev->GetChannelBusy();
      emit("stale-1st", i, done, armed, 0);
      if (dev->ArmChannelBusy(window_us) == 0) {
        std::printf("SKIP second arm refused\n");
        return finish(5);
      }
      nap(5);
    } else if (mode == "txsess") {
      static const uint8_t dot11[36] = {0x08, 0x00, 0x00, 0x00, 0xff, 0xff,
                                        0xff, 0xff, 0xff, 0xff, 0x02, 0x11,
                                        0x22, 0x33, 0x44, 0x55, 0x02, 0x11,
                                        0x22, 0x33, 0x44, 0x55};
      devourer::TxMode tx_mode;
      const std::vector<uint8_t> rt = devourer::build_stream_radiotap(tx_mode);
      std::vector<uint8_t> buf(rt.begin(), rt.end());
      buf.insert(buf.end(), dot11, dot11 + sizeof dot11);
      const auto t0 = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now() - t0 <
             std::chrono::milliseconds(wait_ms)) {
        if (dev->send_packet(buf.data(), buf.size()))
          sent++;
      }
    } else {
      nap(wait_ms);
    }

    const devourer::ChannelBusy b = dev->GetChannelBusy();
    emit(mode.c_str(), i, b, armed, sent);

    if (mode == "retune")
      dev->SetMonitorChannel(chan_def);
  }
  return finish(0);
}
