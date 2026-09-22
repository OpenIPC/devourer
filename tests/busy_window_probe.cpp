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
 *   revive   arm, Stop(), retune, read. The window must not outlive its own
 *            hardware session. A backend that does not forget it here hands
 *            back a spoil reason earned by a session that no longer exists —
 *            measured as spoil=retuned on all four Realtek backends with the
 *            reset removed. The assertion is on the REASON only: whether the
 *            read is valid depends on how deeply that backend's Stop tears
 *            the chip down, and on a Jaguar2 (whose Stop only joins its
 *            runtime threads) the sampled path answers with a live 2 ms
 *            window. Not runnable on MediaTek: Stop() closes the device.
 *   txsess   arm, TRANSMIT inside the window, read. Must stay valid and be
 *            flagged own_tx: Realtek reads low (the receiver is deaf while
 *            the PA is up), MediaTek reads high (it counts own airtime).
 *   stale    arm, let it COMPLETE, read it, arm again, read immediately. The
 *            second read must be INVALID. The result register latches the
 *            previous window, so this only holds if the trigger clears the
 *            ready bit - an assumption the headless selftest can only model,
 *            which is why it is checked here against real silicon.
 *   race     a second thread hammers GetRxQuality() for the whole window while
 *            this one arms and reads. Every reading must come back refused as
 *            interrupted — the hammer re-arms the shared engine continuously,
 *            so no window survives it, and the failure being hunted is a SHORT
 *            window wearing a valid flag. This is the on-air check of the locking: without it
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
#include <vector>

#if __has_include(<libusb.h>)
#include <libusb.h>
#else
#include <libusb-1.0/libusb.h>
#endif

#include "AdapterCaps.h"
#include "Event.h"
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

devourer::EventSink g_ev;

/* JSON Lines through the shared sink, like every other machine-readable
 * output in the tree: one atomic write per line, so a consumer parsing stdout
 * (or a second writer on it) is never handed a half-line or a bespoke
 * format. */
void emit(const char *mode, int i, const devourer::ChannelBusy &b,
          uint32_t armed_us, uint32_t sent) {
  devourer::Ev(g_ev, "busy.window")
      .t()
      .f("mode", mode)
      .f("i", static_cast<long long>(i))
      .f("valid", b.valid)
      .f("busy_pct", static_cast<long long>(b.busy_pct))
      .f("source", source_name(b.source))
      .f("window_us", static_cast<long long>(b.window_us))
      .f("armed_us", static_cast<long long>(armed_us))
      .f("spoil", spoil_name(b.spoil))
      .f("own_tx", b.own_tx_in_window)
      .f("tx_frames", static_cast<long long>(b.own_tx_frames))
      .f("sent", static_cast<long long>(sent));
}

/* Every exit from main() after the RX thread is running must go through this.
 * That thread is detached and may still be inside Init/StartRxLoop, so letting
 * DeviceSession's destructor close the handle under it is a use-after-free —
 * which is exactly what the SKIP returns used to do. _exit runs no
 * destructors, so the USB lock is dropped by hand first: one left behind makes
 * the next run of this script refuse the adapter it just used. The libusb
 * context is NOT closed on that path — the kernel reclaims it at process exit,
 * and closing it under a live RX thread is the very hazard being avoided. The
 * non-RX paths return normally and the session closes everything.
 * std::_Exit rather than POSIX _exit: same semantics, and it builds on MSVC. */
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
  std::_Exit(code);
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
                   "[--mode sampled|window|interrupt|quality|race|retune|early|"
                   "revive|stale|txsess]\n",
                   argv[0]);
      return 2;
    }
  }

  g_ev.configure(stdout);
  auto logger = std::make_shared<Logger>();
  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) < 0) {
    std::fprintf(stderr, "libusb_init failed\n");
    return 3;
  }
  devourer::DeviceSession session(logger);
  /* So every exit path, including the early ones, runs libusb_exit(). */
  session.adopt_context(ctx);
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
  devourer::Ev(g_ev, "busy.caps")
      .t()
      .f("generation", devourer::generation_name(caps.generation))
      .f("chip", caps.chip_name)
      .f("busy_airtime_ok", caps.busy_airtime_ok)
      .f("busy_airtime_measured", caps.busy_airtime_measured)
      .f("rx_energy_ok", caps.rx_energy_ok);

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

  /* `revive` tears the session down and brings it back. With --rx the Realtek
   * Init is running on a DETACHED thread (it ends in a blocking StartRxLoop),
   * so Stop() and SetMonitorChannel would run underneath it — the same
   * use-after-free the cleanup path above exists to avoid, and the sequence
   * RtlJaguar3Device::InitWrite refuses outright. Refuse rather than wedge. */
  if (mode == "revive" && rx_on) {
    devourer::Ev(g_ev, "busy.skip")
        .t()
        .f("why", "revive cannot run with the RX loop live");
    return finish(5);
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
      devourer::Ev(g_ev, "busy.skip").t().f("why", "cannot arm a busy window");
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
        /* Register I/O can throw on a USB glitch. An escaping exception here
         * terminates the process instead of failing the arm under test. */
        while (!stop.load(std::memory_order_relaxed)) {
          try {
            if (rtl)
              (void)rtl->GetRxQuality();
          } catch (const std::exception &) {
          }
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
    } else if (mode == "revive") {
      /* The window must not survive its own hardware session.
       *
       * Stop() ends the session, but that alone does not protect an armed
       * window, and the mechanism differs by backend. On the RTL8733B,
       * SetMonitorChannel re-runs bring_up_to_phy(), which sets the flag
       * with_ccx gates on back to true. On Jaguar1/2/3 the retune does no
       * bring-up at all — there the window stays reachable simply because
       * nothing ever clears `_brought_up`. Either way a backend that does
       * not reset the window in Stop() hands the next read a spoil reason
       * earned by a session that no longer exists, and the retune note below
       * is what stamps it.
       *
       * The harness asserts the REASON only. Whether the read is valid
       * depends on how deeply that backend's Stop tears the chip down:
       * measured false on the RTL8733B, Jaguar1 and Jaguar3, and TRUE with a
       * 2 ms window on a Jaguar2, whose Stop only joins its runtime
       * threads.
       *
       * Read `5/5 spoil=none` as ONE confirmation, not five. This arm sits
       * inside the rep loop, so reps 2..N arm a chip the previous rep's
       * Stop() already tore down — which succeeds only because of the
       * residual this very comment is about (nothing clears the gate
       * with_ccx reads). Only rep 1 exercises the intended live-session
       * sequence. The negative control is what discriminates: with the
       * reset removed every rep reports spoil=retuned. */
      nap(wait_ms / 2);
      dev->Stop();
      nap(50);
      dev->SetMonitorChannel(
          SelectedChannel{.Channel = static_cast<uint8_t>(channel),
                          .ChannelOffset = 0,
                          .ChannelWidth = CHANNEL_WIDTH_20});
      nap(wait_ms);
    } else if (mode == "stale") {
      /* Let this window finish and consume it, so the result register holds a
       * completed measurement; then arm again and read at once. */
      nap(wait_ms);
      const devourer::ChannelBusy done = dev->GetChannelBusy();
      emit("stale-1st", i, done, armed, 0);
      if (dev->ArmChannelBusy(window_us) == 0) {
        devourer::Ev(g_ev, "busy.skip").t().f("why", "second arm refused");
        return finish(5);
      }
      nap(5);
    } else if (mode == "txsess") {
      /* A 24-byte 802.11 data header, written out in full: frame control,
       * duration, addr1/2/3 and the sequence-control field. An earlier cut
       * listed 22 bytes into a 36-byte array and worked only because the
       * implicit zeros happened to land where seq-ctrl belongs. */
      static const uint8_t dot11[24] = {
          0x08, 0x00,                          /* frame control: data       */
          0x00, 0x00,                          /* duration                  */
          0xff, 0xff, 0xff, 0xff, 0xff, 0xff,  /* addr1: broadcast          */
          0x02, 0x11, 0x22, 0x33, 0x44, 0x55,  /* addr2: source             */
          0x02, 0x11, 0x22, 0x33, 0x44, 0x55,  /* addr3: BSSID              */
          0x00, 0x00};                         /* sequence control          */
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
