/* txpower — reference consumer of the runtime TX-power API.
 *
 * Opens one adapter, brings it up for TX, prints the family's TxPowerCaps,
 * then walks the requested knob sequence — a quarter-dB offset ramp
 * (SetTxPowerOffsetQdb), a flat index (SetTxPowerIndexOverride), per-rate
 * diffs (SetTxPowerRateDiffs), or both — echoing the applied qdB and a
 * TxPowerState snapshot after every step. This is the shape of an adaptive-link
 * controller's power leg: set, read back, observe saturation, react.
 *
 * Pure CLI configuration (no environment variables — the API is the point):
 *
 *   --vid 0xNNNN --pid 0xNNNN   adapter select (default: first Realtek PID)
 *   --channel N                 monitor channel for bring-up (default 36)
 *   --bw 20|40|80               bandwidth (default 20)
 *   --flat N                    force flat TXAGC index before the ramp (-1 clears)
 *   --offset-start Q            offset ramp start, qdB (default: no ramp)
 *   --offset-stop Q             offset ramp stop, qdB (default = start)
 *   --step-qdb Q                ramp increment, qdB (default 4 = 1 dB)
 *   --step-ms N                 dwell per step, ms (default 500)
 *   --rate-diffs I,I,...,I      10 comma-separated qdB per rate
 *                               (cck,legacy,m0..m7) or 'clear' to nullopt
 *   --switch-channel N          after the ramp: SetMonitorChannel(N) and re-dump
 *                               state — proves the offset is sticky across a
 *                               full channel set (and re-folds against the new
 *                               channel's per-rate table)
 *   --retune N                  after the ramp: FastRetune(N) and re-dump state
 *                               — proves the hop path leaves TXAGC alone
 *   --thermal                   print a thermal snapshot with each state line
 *
 * Machine-readable output (one JSONL event per step, consumed by
 * tests/txpwr_offset_regcheck.sh):
 *
 *   {"ev":"txpwr.caps","supported":1,"max":63,"step_qdb":2,...}
 *   {"ev":"txpwr.state","flat":-1,"offset_qdb":-24,"steps":-12,"satlo":0,
 *    "sathi":0,"cck":28,"ofdm":34,"mcs7":30,"rb":1,"rate_diffs":0}
 */
#ifdef _WIN32
#define NOMINMAX
#endif

#if defined(__ANDROID__) || defined(_MSC_VER) || defined(__APPLE__)
#include <libusb.h>
#else
#include <libusb-1.0/libusb.h>
#endif

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "SignalStop.h"
#include "ThermalStatus.h"
#include "TxPower.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "caps_event.h"
#include "env_config.h"
#include "logger.h"

namespace {

/* Event sink for the demo's JSONL emissions (print_state is a free function)
 * — points at the main() Logger's sink, set right after Logger construction. */
devourer::EventSink *g_ev = nullptr;

/* The Realtek PIDs the demos' open loop iterates; --pid narrows to one. */
const uint16_t kRealtekPids[] = {0x8812, 0x8813, 0x881a, 0x0811, 0xa811,
                                 0x0820, 0x0821, 0x8822, 0x0120, 0x012d,
                                 0xb82c, 0xc811, 0xc812, 0xa81a};

struct Args {
  uint16_t vid = 0x0bda;
  int pid = -1; /* -1 = iterate kRealtekPids */
  int channel = 36;
  int bw = 20;
  int flat = -2; /* -2 = untouched, -1 = clear, >=0 = force */
  int offset_start = 0;
  int offset_stop = 0;
  bool have_ramp = false;
  int step_qdb = 4;
  int step_ms = 500;
  int switch_channel = -1;
  int retune = -1;
  bool thermal = false;
  std::string rate_diffs;
};

bool parse_int(const char *s, int &out) {
  char *end = nullptr;
  long v = std::strtol(s, &end, 0);
  if (end == s || *end != '\0')
    return false;
  out = static_cast<int>(v);
  return true;
}

bool parse_args(int argc, char **argv, Args &a) {
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    auto next = [&](int &out) {
      return i + 1 < argc && parse_int(argv[++i], out);
    };
    int v = 0;
    if (k == "--vid" && next(v))
      a.vid = static_cast<uint16_t>(v);
    else if (k == "--pid" && next(v))
      a.pid = v;
    else if (k == "--channel" && next(a.channel))
      ;
    else if (k == "--bw" && next(a.bw))
      ;
    else if (k == "--flat" && next(a.flat))
      ;
    else if (k == "--offset-start" && next(a.offset_start))
      a.have_ramp = true;
    else if (k == "--offset-stop" && next(a.offset_stop))
      a.have_ramp = true;
    else if (k == "--step-qdb" && next(a.step_qdb))
      ;
    else if (k == "--step-ms" && next(a.step_ms))
      ;
    else if (k == "--switch-channel" && next(a.switch_channel))
      ;
    else if (k == "--retune" && next(a.retune))
      ;
    else if (k == "--thermal")
      a.thermal = true;
    else if (k == "--rate-diffs") {
      if (i + 1 >= argc)
        return false;
      a.rate_diffs = argv[++i];
    } else {
      std::fprintf(stderr, "devourer [W] unknown/incomplete arg: %s\n",
                   k.c_str());
      return false;
    }
  }
  if (a.have_ramp && a.offset_stop == 0 && a.offset_start != 0)
    a.offset_stop = a.offset_start;
  return true;
}

ChannelWidth_t bw_enum(int bw) {
  switch (bw) {
  case 40:
    return CHANNEL_WIDTH_40;
  case 80:
    return CHANNEL_WIDTH_80;
  default:
    return CHANNEL_WIDTH_20;
  }
}

void print_state(IRtlDevice *dev, bool with_thermal) {
  const devourer::TxPowerState s = dev->GetTxPowerState();
  devourer::Ev(*g_ev, "txpwr.state")
      .f("flat", s.flat_index)
      .f("offset_qdb", s.offset_qdb)
      .f("steps", s.offset_steps)
      .f("satlo", s.saturated_low ? 1 : 0)
      .f("sathi", s.saturated_high ? 1 : 0)
      .f("cck", s.cck_index)
      .f("ofdm", s.ofdm_index)
      .f("mcs7", s.mcs7_index)
      .f("rb", s.hw_readback ? 1 : 0)
      .f("rate_diffs", s.rate_diffs_custom ? 1 : 0);
  if (with_thermal) {
    const devourer::ThermalStatus t = dev->GetThermalStatus();
    devourer::Ev ev(*g_ev, "thermal");
    ev.t().f("raw", t.raw);
    if (t.valid)
      ev.f("baseline", t.baseline).f("delta", t.delta);
    else
      ev.f("baseline", nullptr);
    ev.f("status", devourer::ThermalBucket(t));
  }
}

} // namespace

int main(int argc, char **argv) {
  Args a;
  if (!parse_args(argc, argv, a))
    return 2;

  auto logger = std::make_shared<Logger>();
  apply_logging_env(*logger); /* DEVOURER_LOG_LEVEL / DEVOURER_EVENTS / ... */
  g_ev = &logger->events();
  install_devourer_signal_handlers();

  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) < 0) {
    logger->error("libusb_init failed");
    return 1;
  }
  libusb_device_handle *handle = nullptr;
  if (a.pid >= 0) {
    handle = libusb_open_device_with_vid_pid(ctx, a.vid,
                                             static_cast<uint16_t>(a.pid));
  } else {
    for (uint16_t pid : kRealtekPids) {
      handle = libusb_open_device_with_vid_pid(ctx, a.vid, pid);
      if (handle)
        break;
    }
  }
  if (!handle) {
    logger->error("no adapter found ({:04x}:{})", a.vid,
                  a.pid >= 0 ? "requested pid" : "any Realtek pid");
    libusb_exit(ctx);
    return 1;
  }

  std::shared_ptr<devourer::UsbDeviceLock> lock;
  if (devourer::claim_interface_then_reset(handle, devourer::find_wifi_interface(handle), logger, true, lock) !=
      0) {
    libusb_close(handle);
    libusb_exit(ctx);
    return 1;
  }

  WiFiDriver driver(logger);
  std::unique_ptr<IRtlDevice> dev =
      driver.CreateRtlDevice(handle, ctx, lock, devourer_config_from_env());
  if (!dev) {
    logger->error("CreateRtlDevice failed (chip support not built?)");
    libusb_close(handle);
    libusb_exit(ctx);
    return 1;
  }

  devourer::emit_adapter_caps(*g_ev, dev.get());

  const devourer::TxPowerCaps caps = dev->GetTxPowerCaps();
  devourer::Ev(*g_ev, "txpwr.caps")
      .f("supported", caps.supported ? 1 : 0)
      .f("max", caps.index_max)
      .f("step_qdb", caps.step_qdb)
      .f("step_measured", caps.step_measured ? 1 : 0)
      .f("min_qdb", caps.offset_min_qdb)
      .f("max_qdb", caps.offset_max_qdb);
  if (!caps.supported) {
    logger->error("TX-power API not wired for this family yet");
    dev->Stop();
    libusb_close(handle);
    libusb_exit(ctx);
    return 3;
  }

  dev->InitWrite(SelectedChannel{.Channel = static_cast<uint8_t>(a.channel),
                                 .ChannelOffset = 0,
                                 .ChannelWidth = bw_enum(a.bw)});
  logger->info("brought up on ch{} bw{}", a.channel, a.bw);

  /* Baseline state before any knob moves (the offset=0 parity reference). */
  print_state(dev.get(), a.thermal);

  if (a.flat >= -1) {
    dev->SetTxPowerIndexOverride(a.flat);
    logger->info("flat index override -> {}", a.flat);
    print_state(dev.get(), a.thermal);
  }

  if (!a.rate_diffs.empty()) {
    if (a.rate_diffs == "clear") {
      const bool ok = dev->SetTxPowerRateDiffs(std::nullopt);
      logger->info("rate-diffs clear -> {}", ok ? "ok" : "unsupported");
    } else {
      devourer::TxRateDiffsQdb d;
      int v[10] = {0};
      int n = 0;
      const char* p = a.rate_diffs.c_str();
      char* end = nullptr;
      while (n < 10) {
        v[n++] = static_cast<int>(std::strtol(p, &end, 10));
        if (*end != ',') break;
        p = end + 1;
      }
      if (n != 10 || *end != '\0') {
        std::fprintf(stderr, "devourer [E] --rate-diffs wants 10 comma ints "
                             "(cck,legacy,m0..m7) or 'clear'\n");
        return 2;
      }
      d.cck = static_cast<int8_t>(v[0]);
      d.legacy = static_cast<int8_t>(v[1]);
      for (int i2 = 0; i2 < 8; ++i2) d.mcs[i2] = static_cast<int8_t>(v[2 + i2]);
      const bool ok = dev->SetTxPowerRateDiffs(d);
      logger->info("rate-diffs -> {}", ok ? "applied" : "unsupported");
    }
    print_state(dev.get(), a.thermal);
  }

  if (a.have_ramp) {
    const int dir = (a.offset_stop >= a.offset_start) ? 1 : -1;
    const int inc = (a.step_qdb > 0 ? a.step_qdb : 4) * dir;
    for (int q = a.offset_start;
         (dir > 0 ? q <= a.offset_stop : q >= a.offset_stop) &&
         !g_devourer_should_stop;
         q += inc) {
      const int applied = dev->SetTxPowerOffsetQdb(q);
      devourer::Ev(*g_ev, "txpwr.offset").f("requested", q).f("applied", applied);
      print_state(dev.get(), a.thermal);
      std::this_thread::sleep_for(std::chrono::milliseconds(a.step_ms));
    }
  }

  if (a.switch_channel > 0 && !g_devourer_should_stop) {
    dev->SetMonitorChannel(
        SelectedChannel{.Channel = static_cast<uint8_t>(a.switch_channel),
                        .ChannelOffset = 0,
                        .ChannelWidth = bw_enum(a.bw)});
    logger->info("SetMonitorChannel -> ch{} (offset must re-fold)",
                 a.switch_channel);
    print_state(dev.get(), a.thermal);
  }

  if (a.retune > 0 && !g_devourer_should_stop) {
    dev->FastRetune(static_cast<uint8_t>(a.retune));
    logger->info("FastRetune -> ch{} (TXAGC registers must be untouched)",
                 a.retune);
    print_state(dev.get(), a.thermal);
  }

  dev->Stop();
  libusb_close(handle);
  libusb_exit(ctx);
  return 0;
}
