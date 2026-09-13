/* cca_gates_probe — drive IRtlRadio::SetCcaGates / GetCcaGates from the tree.
 *
 * The gate split had no in-tree caller, so nothing in the repo reproduced the
 * tables that motivated it. This is that caller: it walks the four gate
 * states, the two legacy SetCcaMode states, and the pre-bring-up refusal, and
 * prints one machine-readable line per step for tests/cca_gates_regcheck.sh
 * to assert against. Register-level confirmation is the regcheck's job (it
 * peeks 0x520/0x524 with examples/chipstate --no-claim while this holds the
 * interface); this binary reports what the API says, so a disagreement
 * between the two is itself the finding.
 *
 *   sudo build/CcaGatesProbe --pid 0xc812 --channel 36
 *   sudo build/CcaGatesProbe --pid 0x8812 --channel 36 --hold 12
 *
 * --hold N keeps each state applied for N seconds so an external peek can
 * sample it. Exit 0 = every step behaved; 4 = not a Realtek radio; 5 = the
 * backend does not implement the split (the not-ported default, not a
 * failure).
 */
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <thread>

#if __has_include(<libusb.h>)
#include <libusb.h>
#else
#include <libusb-1.0/libusb.h>
#endif

#include "AdapterCaps.h"
#include "DeviceSession.h"
#include "IRtlRadio.h"
#include "WiFiDriver.h"
#include "logger.h"

namespace {

int fails = 0;

void check(bool ok, const char *what) {
  std::printf("%s %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok)
    fails++;
}

void report(const char *tag, bool ret, bool primary, bool edcca) {
  std::printf("GATES %-26s ret=%d primary=%d edcca=%d\n", tag, ret ? 1 : 0,
              primary ? 1 : 0, edcca ? 1 : 0);
  std::fflush(stdout);
}

} // namespace

int main(int argc, char **argv) {
  uint16_t vid = 0x0bda, pid = 0xc812;
  int channel = 36, retune = 0, fast_retune = 0, hold = 0;
  for (int i = 1; i < argc; i++) {
    if (!std::strcmp(argv[i], "--vid") && i + 1 < argc)
      vid = (uint16_t)std::strtoul(argv[++i], nullptr, 0);
    else if (!std::strcmp(argv[i], "--pid") && i + 1 < argc)
      pid = (uint16_t)std::strtoul(argv[++i], nullptr, 0);
    else if (!std::strcmp(argv[i], "--channel") && i + 1 < argc)
      channel = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--retune") && i + 1 < argc)
      retune = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--fast-retune") && i + 1 < argc)
      fast_retune = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--hold") && i + 1 < argc)
      hold = std::atoi(argv[++i]);
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
  if (!rtl) {
    std::printf("SKIP not a Realtek radio (IRtlRadio cast failed)\n");
    return 4;
  }

  /* Name the family for the harness. The BB EDCCA threshold register is
   * per-generation (Jaguar1 0x8a4, Jaguar3 0x84c) and a tracker cell that
   * pokes the wrong one reports "no tracker running" instead of failing —
   * a false negative on exactly the arm the split exists to serve. Caps are
   * resolved at construction, so this is readable before bring-up. */
  std::printf("GATES-GEN %s\n",
              devourer::generation_name(dev->GetAdapterCaps().generation));
  std::fflush(stdout);

  /* Pre-bring-up: both calls must refuse, and the refusal must not write the
   * caller's variables. Poisoned true so an assignment is visible. */
  {
    bool p = true, e = true;
    const bool got = rtl->GetCcaGates(p, e);
    report("pre-bringup-get", got, p, e);
    check(!got, "GetCcaGates refuses before bring-up");
    check(p && e, "GetCcaGates leaves out-params alone when it refuses");
    const bool set = rtl->SetCcaGates(false, true);
    report("pre-bringup-set", set, false, true);
    check(!set, "SetCcaGates refuses before bring-up");
  }

  dev->InitWrite(SelectedChannel{.Channel = static_cast<uint8_t>(channel),
                                 .ChannelOffset = 0,
                                 .ChannelWidth = CHANNEL_WIDTH_20});

  {
    bool p = true, e = true;
    const bool got = rtl->GetCcaGates(p, e);
    report("bringup-default", got, p, e);
    if (!got) {
      std::printf("SKIP backend does not implement the gate split "
                  "(not-ported default)\n");
      /* The not-ported exit still carries the pre-bring-up verdict: those
       * checks ran above and a backend that failed them has not "cleanly
       * refused", whatever it does about the split. */
      return fails ? 1 : 5;
    }
    check(!p && !e, "both gates are ENABLED at bring-up (the default)");
  }

  /* The four states, each read back from the hardware. */
  for (int i = 0; i < 4; i++) {
    const bool want_p = (i & 2) != 0, want_e = (i & 1) != 0;
    char tag[48];
    std::snprintf(tag, sizeof tag, "set-primary%d-edcca%d", want_p ? 1 : 0,
                  want_e ? 1 : 0);
    const bool set = rtl->SetCcaGates(want_p, want_e);
    bool p = false, e = false;
    const bool got = rtl->GetCcaGates(p, e);
    report(tag, set && got, p, e);
    check(set && got && p == want_p && e == want_e,
          "gate state reads back as written");
    /* Hold AFTER reporting, not before: the line above is what an external
     * reader (tests/cca_gates_regcheck.sh peeking the registers with
     * chipstate) waits for, so the state has to still be applied when it
     * arrives. Holding first advertised each state one step too late. */
    if (hold)
      std::this_thread::sleep_for(std::chrono::seconds(hold));
  }

  /* Retune survival. Measured on both families, the state is intact after a
   * retune — within a band and across a band change — but by different
   * mechanisms: Jaguar3 records the pair and re-asserts it in
   * SetMonitorChannel, Jaguar1 records nothing and survives only because its
   * channel path does not rewrite those registers. See src/IRtlRadio.h. */
  if (retune) {
    rtl->SetCcaGates(true, false);
    dev->SetMonitorChannel(SelectedChannel{.Channel =
                                               static_cast<uint8_t>(retune),
                                           .ChannelOffset = 0,
                                           .ChannelWidth = CHANNEL_WIDTH_20});
    bool p = false, e = false;
    const bool got = rtl->GetCcaGates(p, e);
    report("after-retune", got, p, e);
    /* GetCcaGates reads 0x520 only, so the API cannot speak for the rest of
     * the gate state. Hold so the harness can peek 0x524[11] out of band. */
    if (hold)
      std::this_thread::sleep_for(std::chrono::seconds(hold));
  }

  /* FastRetune is the other channel path, and on Jaguar3 its fallback does
   * not carry SetMonitorChannel's re-assert — so whether the state survives
   * it is a separate question from --retune above, not the same one. */
  if (fast_retune) {
    rtl->SetCcaGates(true, false);
    dev->FastRetune(static_cast<uint8_t>(fast_retune));
    bool p = false, e = false;
    const bool got = rtl->GetCcaGates(p, e);
    report("after-fast-retune", got, p, e);
    if (hold)
      std::this_thread::sleep_for(std::chrono::seconds(hold));
  }

  /* Legacy path: SetCcaMode must still be exactly SetCcaGates(d, d). */
  for (int i = 0; i < 2; i++) {
    const bool d = i == 0;
    dev->SetCcaMode(d);
    bool p = false, e = false;
    const bool got = rtl->GetCcaGates(p, e);
    report(d ? "setccamode-true" : "setccamode-false", got, p, e);
    check(got && p == d && e == d, "SetCcaMode moves both gates together");
    if (hold)
      std::this_thread::sleep_for(std::chrono::seconds(hold));
  }

  std::printf("%s\n", fails ? "cca_gates_probe: FAIL" : "cca_gates_probe: PASS");
  return fails ? 1 : 0;
}
