/* stop_during_init_probe — exercises the StopRxLoop contract on IRadio.h
 * (tests/stop_during_init.sh runs the matrix):
 *
 *   initstop   Init() on a thread; StopRxLoop() after --stop-at-ms; the Init
 *              thread must still be inside Init at the stop and must return
 *              within --timeout-s after it. Then a standalone StartRxLoop()
 *              runs for --rx-ms and must see frames: the chip the stop
 *              interrupted has to be a working receiver, so a bring-up that
 *              failed and returned on its own cannot pass as an honoured
 *              stop. (Before the fix StartRxLoop cleared the flag and the
 *              loop ran forever.)
 *   restart    Init() on a thread; after --stop-at-ms StopRxLoop(); join;
 *              then StartRxLoop() on a new thread, run --rx-ms, StopRxLoop(),
 *              join. Both joins must return and the second loop must see
 *              frames: a stop left by a finished loop must not refuse a
 *              standalone restart.
 *   stale      StopRxLoop() BEFORE Init(); Init() on a thread; after --rx-ms
 *              StopRxLoop(); join. RX must have run (frames > 0 on a channel
 *              that carries traffic): a pre-Init stale stop must not refuse
 *              Init's own loop.
 *
 * --no-reset skips libusb_reset_device before the claim (Kestrel: a reset
 * on running firmware re-enumerates the adapter, src/kestrel/CLAUDE.md).
 * --fw-dir <dir> (or DEVOURER_MT7612U_FW_DIR) names the decompressed
 * mt7662.bin + mt7662_rom_patch.bin for an MT7612U, whose firmware is not
 * embedded. A bring-up that throws is reported as INIT_THREW, not a core.
 * Prints one line: RESULT mode=<m> outcome=<OK|DEADLOCK|...> ...
 * Exit 0 = OK, 1 = the contract failed, 3 = no adapter.
 */
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include <libusb.h>

#include "DeviceSession.h"
#include "IRadio.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "logger.h"

using clk = std::chrono::steady_clock;

/* The RESULT line is one machine record; an exception message is free text
 * that may carry quotes or newlines, so it is flattened before it goes in. */
static std::string result_safe(std::string s) {
  for (char &c : s)
    if (c == '"' || c == '\\')
      c = '\'';
    else if ((unsigned char)c < 0x20)
      c = ' ';
  return s;
}

static bool join_within(std::thread &t, std::atomic<bool> &done, int timeout_s,
                        long *elapsed_ms) {
  const auto t0 = clk::now();
  while (!done.load() &&
         clk::now() - t0 < std::chrono::seconds(timeout_s))
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  *elapsed_ms = (long)std::chrono::duration_cast<std::chrono::milliseconds>(
                    clk::now() - t0)
                    .count();
  if (!done.load())
    return false;
  t.join();
  return true;
}

int main(int argc, char **argv) {
  uint16_t vid = 0x0bda, pid = 0xb812;
  int channel = 6, stop_at_ms = 500, rx_ms = 4000, timeout_s = 20;
  std::string mode = "initstop";
  bool do_reset = true;
  std::string fw_dir;
  if (const char *e = std::getenv("DEVOURER_MT7612U_FW_DIR"))
    fw_dir = e;
  for (int i = 1; i < argc; i++) {
    if (!std::strcmp(argv[i], "--vid") && i + 1 < argc)
      vid = (uint16_t)std::strtoul(argv[++i], nullptr, 0);
    else if (!std::strcmp(argv[i], "--pid") && i + 1 < argc)
      pid = (uint16_t)std::strtoul(argv[++i], nullptr, 0);
    else if (!std::strcmp(argv[i], "--channel") && i + 1 < argc)
      channel = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--stop-at-ms") && i + 1 < argc)
      stop_at_ms = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--rx-ms") && i + 1 < argc)
      rx_ms = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--timeout-s") && i + 1 < argc)
      timeout_s = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--mode") && i + 1 < argc)
      mode = argv[++i];
    else if (!std::strcmp(argv[i], "--no-reset"))
      do_reset = false;
    else if (!std::strcmp(argv[i], "--fw-dir") && i + 1 < argc)
      fw_dir = argv[++i];
    else {
      std::fprintf(stderr, "bad arg %s\n", argv[i]);
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
  session.adopt_context(ctx);
  libusb_device_handle *handle = libusb_open_device_with_vid_pid(ctx, vid, pid);
  if (!handle) {
    std::fprintf(stderr, "no adapter %04x:%04x\n", vid, pid);
    return 3;
  }
  std::shared_ptr<devourer::UsbDeviceLock> lock;
  if (devourer::claim_interface_then_reset(
          handle, devourer::find_wifi_interface(handle), logger, do_reset, lock) !=
      0) {
    session.adopt_handle(handle);
    return 3;
  }
  session.adopt_handle(handle);
  session.adopt_lock(lock);
  devourer::DeviceConfig cfg;
  if (!fw_dir.empty()) {
    /* An explicit directory must hold both files: the backend falls back to
     * its default search paths otherwise, and a run would then pass on
     * firmware the caller did not name. */
    for (const char *f : {"mt7662.bin", "mt7662_rom_patch.bin"}) {
      if (!std::filesystem::exists(std::filesystem::path(fw_dir) / f)) {
        std::printf("RESULT mode=%s outcome=BAD_FW_DIR missing=%s/%s\n",
                    mode.c_str(), fw_dir.c_str(), f);
        return 2;
      }
    }
    cfg.mt7612u.firmware_dir = fw_dir;
  }
  WiFiDriver driver(logger);
  std::unique_ptr<IRadio> owned = driver.CreateRadio(handle, ctx, lock, cfg);
  if (!owned) {
    std::fprintf(stderr, "CreateRadio failed\n");
    return 3;
  }
  session.adopt_device(std::move(owned));
  IRadio *const dev = session.device();
  const SelectedChannel chan{.Channel = (uint8_t)channel,
                             .ChannelOffset = 0,
                             .ChannelWidth = CHANNEL_WIDTH_20};

  std::atomic<long> frames{0};
  auto on_pkt = [&frames](const Packet &) { frames++; };

  auto fail_hung = [&](const char *what, long waited_ms) {
    std::printf("RESULT mode=%s outcome=DEADLOCK stage=%s waited_ms=%ld "
                "frames=%ld\n",
                mode.c_str(), what, waited_ms, frames.load());
    std::fflush(stdout);
    /* The RX thread is stuck inside the library; destructors would run the
     * teardown under it. Drop the USB lock by hand and leave. */
    lock.reset();
    session.release_lock();
    std::_Exit(1);
  };

  if (mode == "stale")
    dev->StopRxLoop(); /* the pre-Init stale request */

  std::atomic<bool> init_done{false};
  std::atomic<bool> init_threw{false};
  std::string init_what;
  std::thread init_thread([&]() {
    try {
      dev->Init(on_pkt, chan);
    } catch (const std::exception &e) {
      init_what = e.what();
      init_threw = true;
    }
    init_done = true;
  });
  const int first_wait = (mode == "initstop" || mode == "restart") ? stop_at_ms
                                                                   : rx_ms;
  std::this_thread::sleep_for(std::chrono::milliseconds(first_wait));
  const long frames_at_stop = frames.load();
  if (init_threw.load()) {
    /* Bring-up failed before the contract was reached (missing MT7612U
     * firmware, a dead adapter): nothing below would measure it. */
    init_thread.join();
    std::printf("RESULT mode=%s outcome=INIT_THREW what=\"%s\"\n", mode.c_str(),
                result_safe(init_what).c_str());
    return 1;
  }
  if (mode == "initstop" && init_done.load()) {
    /* Init came back on its own before the scheduled stop: a failed bring-up
     * (Kestrel returns normally on one) or a stop time past the loop's life.
     * Nothing about the contract was exercised. */
    init_thread.join();
    std::printf("RESULT mode=initstop outcome=INIT_RETURNED_BEFORE_STOP "
                "stop_at_ms=%d frames=%ld\n",
                stop_at_ms, frames_at_stop);
    return 1;
  }
  dev->StopRxLoop();
  long waited = 0;
  if (!join_within(init_thread, init_done, timeout_s, &waited))
    fail_hung("init", waited);
  const long init_return_ms = waited;
  if (init_threw.load()) {
    std::printf("RESULT mode=%s outcome=INIT_THREW what=\"%s\"\n", mode.c_str(),
                result_safe(init_what).c_str());
    return 1;
  }

  if (mode == "restart") {
    frames = 0;
    std::atomic<bool> rx_done{false};
    std::thread rx_thread([&]() {
      dev->StartRxLoop(on_pkt);
      rx_done = true;
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(rx_ms));
    const long f = frames.load();
    const bool loop_still_running = !rx_done.load();
    dev->StopRxLoop();
    if (!join_within(rx_thread, rx_done, timeout_s, &waited))
      fail_hung("restart", waited);
    const bool ok = loop_still_running && f > 0;
    std::printf("RESULT mode=restart outcome=%s init_return_ms=%ld "
                "restart_loop_ran=%d restart_frames=%ld restart_return_ms=%ld\n",
                ok ? "OK" : "RESTART_REFUSED", init_return_ms,
                loop_still_running, f, waited);
    return ok ? 0 : 1;
  }
  if (mode == "stale") {
    const bool ok = frames_at_stop > 0;
    std::printf("RESULT mode=stale outcome=%s frames_during_init=%ld "
                "init_return_ms=%ld\n",
                ok ? "OK" : "INIT_RX_REFUSED", frames_at_stop, init_return_ms);
    return ok ? 0 : 1;
  }
  /* The stop was honoured; now show the chip it interrupted is up. */
  frames = 0;
  std::atomic<bool> rx_done{false};
  std::thread rx_thread([&]() {
    dev->StartRxLoop(on_pkt);
    rx_done = true;
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(rx_ms));
  const long verify_frames = frames.load();
  const bool loop_still_running = !rx_done.load();
  dev->StopRxLoop();
  if (!join_within(rx_thread, rx_done, timeout_s, &waited))
    fail_hung("verify", waited);
  const bool ok = loop_still_running && verify_frames > 0;
  std::printf("RESULT mode=initstop outcome=%s stop_at_ms=%d init_return_ms=%ld "
              "frames_at_stop=%ld verify_frames=%ld\n",
              ok ? "OK" : "NO_RX_AFTER_STOP", stop_at_ms, init_return_ms,
              frames_at_stop, verify_frames);
  return ok ? 0 : 1;
}
