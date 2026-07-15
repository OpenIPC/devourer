/* doctor — adapter health triage: is this dongle dying?
 *
 * Motivated by a field failure mode (OpenIPC/devourer#205) where a degrading
 * RTL8812AU enumerates fine, inits green — and is stone-deaf, because its
 * EFUSE reads return stochastic garbage (wrong RFE/PA/LNA → wrong PHY tables)
 * and its 8051 never boots firmware (non-fatal on Jaguar1). Neither shows up
 * as an init failure.
 *
 * What it runs (see src/AdapterHealth.h for the classifier semantics):
 *   1. bring-up      — InitWrite; an abort is an immediate FAILING.
 *   2. EFUSE probe   — N fresh physical map reads cross-compared. Any
 *                      read-to-read mismatch = dying silicon (a healthy chip
 *                      is byte-identical every read). Also validates the
 *                      0x8129 EEPROM ID. (8822E: skipped by design — its OTP
 *                      is not reliably readable post-bring-up.)
 *   3. FW boot       — checksum + MCU-ready outcome of the bring-up's
 *                      firmware download.
 *   4. RX smoke      — count FCS-clean frames for a window. Ambient traffic
 *                      counts; in an RF-quiet place hearing nothing is only
 *                      SUSPECT unless --expect-traffic vouches for a source
 *                      (bench flood / busy AP on the channel).
 *
 * Verdict: HEALTHY / SUSPECT / FAILING (+ reasons), exit code 0 / 1 / 2
 * (3 = tool/open error). Machine-readable summary as one JSONL event:
 *
 *   {"ev":"doctor.verdict","verdict":"FAILING","reasons":"0x12",
 *    "efuse_reads":4,"efuse_mismatch":3,"efuse_bad_id":4,"efuse_id":"0x1029",
 *    "fw_attempted":1,"fw_ready":0,"rx_ok":0,"rx_crc":7,"init":1}
 *
 * CLI (no environment variables — device selection included, since a rig may
 * hold two same-PID/same-serial adapters where only topology tells them
 * apart):
 *
 *   --vid 0xNNNN --pid 0xNNNN   adapter select (default: first Realtek PID)
 *   --bus N --port a.b.c        topology select (dotted libusb port path)
 *   --channel N                 bring-up + listen channel (default 6)
 *   --reads N                   EFUSE stability passes (default 4)
 *   --listen-secs N             RX smoke window (default 8; 0 = skip)
 *   --expect-traffic            operator vouches for on-channel traffic:
 *                               0 frames heard upgrades to FAILING
 *
 * Bench protocol for a definitive verdict on a suspect unit: put it on a
 * uhubctl-switchable hub port, VBUS-cycle, run doctor with a beacon flood on
 * the channel and --expect-traffic, and repeat from cold a few times
 * (tests/adapter_doctor_cold.sh wraps exactly that).
 */
#ifdef _WIN32
#define NOMINMAX
#endif

#if defined(__ANDROID__) || defined(_MSC_VER) || defined(__APPLE__)
#include <libusb.h>
#else
#include <libusb-1.0/libusb.h>
#endif

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include "AdapterHealth.h"
#include "caps_event.h"
#include "RxPacket.h"
#include "SignalStop.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "logger.h"

namespace {

/* The Realtek PIDs the demos' open loop iterates; --pid narrows to one. */
const uint16_t kRealtekPids[] = {0x8812, 0x8813, 0x881a, 0x0811, 0xa811,
                                 0x0820, 0x0821, 0x8822, 0x0120, 0x012d,
                                 0xb82c, 0xc811, 0xc812, 0xa81a};

struct Args {
  uint16_t vid = 0x0bda;
  int pid = -1; /* -1 = iterate kRealtekPids */
  int bus = -1; /* -1 = no topology filter */
  std::string port;
  int channel = 6;
  int reads = 4;
  int listen_secs = 8;
  bool expect_traffic = false;
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
    else if (k == "--bus" && next(a.bus))
      ;
    else if (k == "--port" && i + 1 < argc)
      a.port = argv[++i];
    else if (k == "--channel" && next(a.channel))
      ;
    else if (k == "--reads" && next(a.reads))
      ;
    else if (k == "--listen-secs" && next(a.listen_secs))
      ;
    else if (k == "--expect-traffic")
      a.expect_traffic = true;
    else {
      std::fprintf(stderr, "devourer [W] unknown/incomplete arg: %s\n",
                   k.c_str());
      return false;
    }
  }
  return true;
}

/* Open by USB topology (bus + dotted port path) when a rig holds several
 * same-PID/same-serial adapters — the matcher rxdemo uses, CLI-fed. */
libusb_device_handle *open_by_topology(libusb_context *ctx, const Args &a,
                                       const std::shared_ptr<Logger> &logger) {
  libusb_device_handle *handle = nullptr;
  libusb_device **list = nullptr;
  ssize_t n = libusb_get_device_list(ctx, &list);
  for (ssize_t i = 0; i < n && handle == nullptr; ++i) {
    libusb_device_descriptor dd{};
    if (libusb_get_device_descriptor(list[i], &dd) != 0)
      continue;
    if (dd.idVendor != a.vid)
      continue;
    if (a.pid >= 0 && dd.idProduct != static_cast<uint16_t>(a.pid))
      continue;
    if (libusb_get_bus_number(list[i]) != a.bus)
      continue;
    if (!a.port.empty()) {
      uint8_t ports[8];
      int pc = libusb_get_port_numbers(list[i], ports, sizeof(ports));
      std::string path;
      for (int p = 0; p < pc; ++p)
        path += (path.empty() ? "" : ".") + std::to_string(ports[p]);
      if (path != a.port)
        continue;
    }
    if (libusb_open(list[i], &handle) == 0)
      logger->info("Opened device {:04x}:{:04x} on bus {} port {}",
                   dd.idVendor, dd.idProduct, a.bus,
                   a.port.empty() ? "(any)" : a.port.c_str());
  }
  if (list != nullptr)
    libusb_free_device_list(list, 1);
  return handle;
}

const char *yn(bool b) { return b ? "yes" : "NO"; }

} // namespace

int main(int argc, char **argv) {
  Args a;
  if (!parse_args(argc, argv, a))
    return 3;

  auto logger = std::make_shared<Logger>();
  install_devourer_signal_handlers();

  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) < 0) {
    logger->error("libusb_init failed");
    return 3;
  }
  libusb_device_handle *handle = nullptr;
  if (a.bus >= 0) {
    handle = open_by_topology(ctx, a, logger);
  } else if (a.pid >= 0) {
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
    logger->error("no adapter found (vid {:04x})", a.vid);
    libusb_exit(ctx);
    return 3;
  }

  std::shared_ptr<devourer::UsbDeviceLock> lock;
  if (devourer::claim_interface_then_reset(handle, devourer::find_wifi_interface(handle), logger, true, lock) !=
      0) {
    libusb_close(handle);
    libusb_exit(ctx);
    return 3;
  }

  /* keep_corrupted so the RX smoke can report the corrupt-frame count too
   * (informational only); enable_with_tx so Jaguar3's InitWrite keeps the RX
   * filters open for the StartRxLoop smoke window. */
  devourer::DeviceConfig cfg;
  cfg.rx.keep_corrupted = true;
  cfg.rx.enable_with_tx = true;

  WiFiDriver driver(logger);
  std::unique_ptr<IRtlDevice> dev =
      driver.CreateRtlDevice(handle, ctx, lock, cfg);
  if (!dev) {
    logger->error("CreateRtlDevice failed (chip support not built?)");
    libusb_close(handle);
    libusb_exit(ctx);
    return 3;
  }

  devourer::emit_adapter_caps(logger->events(), dev.get());

  devourer::AdapterHealthInput in;

  /* 1. bring-up */
  try {
    dev->InitWrite(SelectedChannel{.Channel = static_cast<uint8_t>(a.channel),
                                   .ChannelOffset = 0,
                                   .ChannelWidth = CHANNEL_WIDTH_20});
    in.init_completed = true;
  } catch (const std::exception &e) {
    logger->error("bring-up FAILED: {}", e.what());
    in.init_completed = false;
  }

  /* 3. FW boot outcome — read even after an init abort: on the fatal-DLFW
   * generations (Jaguar2/3) the throw IS the fw failure, and the status
   * still says which stage died (checksum vs MCU boot). */
  in.fw = dev->GetFwBootStatus();

  if (in.init_completed) {
    /* 2. EFUSE stability */
    in.efuse = dev->ProbeEfuseStability(a.reads);

    /* 4. RX smoke */
    if (a.listen_secs > 0 && !g_devourer_should_stop) {
      std::atomic<uint32_t> ok{0}, crc{0};
      std::thread rx([&] {
        dev->StartRxLoop([&](const Packet &p) {
          if (p.RxAtrib.crc_err || p.RxAtrib.icv_err)
            crc++;
          else
            ok++;
        });
      });
      for (int s = 0; s < a.listen_secs && !g_devourer_should_stop; ++s)
        std::this_thread::sleep_for(std::chrono::seconds(1));
      dev->StopRxLoop();
      rx.join();
      in.rx_checked = true;
      in.rx_traffic_expected = a.expect_traffic;
      in.rx_frames_ok = ok.load();
      in.rx_frames_crc = crc.load();
    }
  }

  uint32_t reasons = 0;
  const devourer::AdapterVerdict v =
      devourer::ClassifyAdapterHealth(in, reasons);

  /* ---- report ---- */
  std::printf("\n== adapter doctor ==\n");
  std::printf("bring-up:        %s\n", yn(in.init_completed));
  if (in.efuse.supported) {
    std::printf("efuse stability: %d reads, %d mismatched, %d bad-id "
                "(last id 0x%04x%s)\n",
                in.efuse.reads, in.efuse.mismatched_reads,
                in.efuse.invalid_id_reads, in.efuse.eeprom_id,
                in.efuse.eeprom_id == devourer::kRtlEepromId ? "" : " ≠ 0x8129");
    if (in.efuse.mismatched_reads > 0)
      std::printf("                 first mismatch at map offset 0x%x\n",
                  in.efuse.first_mismatch_off);
  } else {
    std::printf("efuse stability: not probed (unsupported on this chip or "
                "bring-up failed)\n");
  }
  if (in.fw.supported && in.fw.attempted)
    std::printf("fw boot:         checksum %s, MCU ready %s\n",
                yn(in.fw.checksum_ok), yn(in.fw.ready_ok));
  if (in.rx_checked)
    std::printf("rx smoke:        %u ok / %u corrupt in %d s%s\n",
                in.rx_frames_ok, in.rx_frames_crc, a.listen_secs,
                in.rx_frames_ok == 0 && !a.expect_traffic
                    ? " (no traffic guarantee — inconclusive if RF-quiet)"
                    : "");

  std::printf("verdict:         %s\n", devourer::AdapterVerdictName(v));
  if (reasons & devourer::kAdapterInitFailed)
    std::printf("  - bring-up aborted\n");
  if (reasons & devourer::kAdapterEfuseUnstable)
    std::printf("  - EFUSE reads are unstable — dying silicon; retire this "
                "adapter\n");
  if (reasons & devourer::kAdapterEfuseIdInvalid)
    std::printf("  - EFUSE EEPROM ID is corrupt (calibration untrustworthy)\n");
  if (reasons & devourer::kAdapterEfuseBlank)
    std::printf("  - EFUSE is blank (autoload fail) — normal on some dev "
                "boards, odd on retail adapters\n");
  if (reasons & devourer::kAdapterFwBootFailed)
    std::printf("  - MCU never booted firmware\n");
  if (reasons & devourer::kAdapterRxDeafToTraffic)
    std::printf("  - deaf to a vouched traffic source\n");
  if (reasons & devourer::kAdapterRxSilent)
    std::printf("  - heard nothing (supply known traffic + --expect-traffic "
                "for a hard verdict)\n");

  /* Machine-parseable summary (consumed by tests/adapter_doctor_cold.sh). */
  devourer::Ev(logger->events(), "doctor.verdict")
      .f("verdict", devourer::AdapterVerdictName(v))
      .hexf("reasons", reasons)
      .f("efuse_reads", in.efuse.reads)
      .f("efuse_mismatch", in.efuse.mismatched_reads)
      .f("efuse_bad_id", in.efuse.invalid_id_reads)
      .hexf("efuse_id", in.efuse.eeprom_id, 4)
      .f("fw_attempted", in.fw.attempted ? 1 : 0)
      .f("fw_ready", in.fw.ready_ok ? 1 : 0)
      .f("rx_ok", in.rx_frames_ok)
      .f("rx_crc", in.rx_frames_crc)
      .f("init", in.init_completed ? 1 : 0);

  dev->Stop();
  libusb_close(handle);
  libusb_exit(ctx);
  switch (v) {
  case devourer::AdapterVerdict::Healthy:
    return 0;
  case devourer::AdapterVerdict::Suspect:
    return 1;
  case devourer::AdapterVerdict::Failing:
    return 2;
  default:
    return 3;
  }
}
