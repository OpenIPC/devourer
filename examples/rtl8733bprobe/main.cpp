/* Staged RTL8731BU / RTL8733BU USB bring-up probe.
 *
 *   id     USB/register-plane identity only
 *   off    vendor card-disable/RF-off from the current state
 *   power  + vendor card-enable and system-clock setup
 *   efuse  + physical OTP read and logical-map decode
 *   fw     + reserved-page/DDMA firmware download and WCPU ready handshake
 *   mac    + normal-mode HALMAC queues, WMAC, and USB RX-DMA setup
 *   phy    + pinned MAC/BB/AGC/RFK-init/RF parameter images and readback
 *   chan   + legal 20/40 MHz or experimental 5/10 MHz tune and readback
 *   tssi   + program factory TSSI DE offsets with closed-loop tracking off
 *   tssi-bb + digital TSSI common/DCK/slope state, then factory DE, still off
 *   tssi-thermal + OFDM/HT thermal table and baseline, tracking still off
 *   tssi-analog + self-reverting ANAPAR/RF tracking setup audit, still off
 *   tssi-enable + capped, no-packet closed-loop enable/disable audit
 *
 * This remains diagnostic support rather than a second driver path. The normal
 * WiFiDriver factory uses the same production bring-up components; PHY/RF,
 * calibration, and frame-plane work are still staged separately. */
#if __has_include(<libusb.h>)
#include <libusb.h>
#else
#include <libusb-1.0/libusb.h>
#endif

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>

#include "DeviceSession.h"
#include "Event.h"
#include "RtlAdapter.h"
#include "ThermalStatus.h"
#include "UsbOpen.h"
#include "logger.h"
#include "rtl8733b/Halmac8733bMac.h"
#include "rtl8733b/Phy8733b.h"
#include "rtl8733b/Rtl8733bBringup.h"
#include "rtl8733b/Rtl8733bUsbIds.h"

namespace {
class PowerDownGuard {
public:
  explicit PowerDownGuard(rtl8733b::Rtl8733bBringup &dev) : _dev(dev) {}
  ~PowerDownGuard() {
    if (!_armed)
      return;
    try {
      _dev.power_off();
    } catch (...) {
      // Never replace the diagnostic stage's result while unwinding.
    }
  }
  void arm() { _armed = true; }

private:
  rtl8733b::Rtl8733bBringup &_dev;
  bool _armed = false;
};

void log_snapshot(const Logger_t &logger, rtl8733b::Rtl8733bBringup &dev,
                  const char *phase) {
  const rtl8733b::RegisterSnapshot state = dev.read_register_snapshot();
  logger->info(
      "state {}: SYS_FUNC_EN=0x{:04x} RF_CTRL=0x{:02x} CR=0x{:02x} "
      "MCUFW_CTRL=0x{:04x} SYS_STATUS1=0x{:08x} EXT_SYS_FUNC_EN=0x{:08x}",
      phase, state.sys_func_en, state.rf_ctrl, state.cr, state.mcufw_ctrl,
      state.sys_status1, state.ext_sys_func_en);
  devourer::Ev(logger->events(), "rtl8733b.state")
      .f("phase", phase)
      .hexf("sys_func_en", state.sys_func_en, 4)
      .hexf("rf_ctrl", state.rf_ctrl, 2)
      .hexf("cr", state.cr, 2)
      .hexf("mcufw", state.mcufw_ctrl, 4)
      .hexf("sys_status1", state.sys_status1, 8)
      .hexf("ext_sys_func_en", state.ext_sys_func_en, 8);
}

const char *tx_power_mode_name(rtl8733b::TxPowerPgMode8733b mode) {
  switch (mode) {
  case rtl8733b::TxPowerPgMode8733b::DirectIndex:
    return "direct";
  case rtl8733b::TxPowerPgMode8733b::TssiOffset:
    return "tssi-offset";
  case rtl8733b::TxPowerPgMode8733b::Unknown:
    return "unknown";
  }
  return "unknown";
}

bool hex16(const char *text, uint16_t &out) {
  char *end = nullptr;
  const unsigned long value = std::strtoul(text, &end, 0);
  if (end == text || *end != '\0' || value > 0xffff)
    return false;
  out = static_cast<uint16_t>(value);
  return true;
}

libusb_device_handle *open_matching(libusb_context *ctx, uint16_t vid,
                                    uint16_t pid, int bus, int address) {
  libusb_device **list = nullptr;
  const ssize_t count = libusb_get_device_list(ctx, &list);
  libusb_device_handle *handle = nullptr;
  for (ssize_t i = 0; i < count && handle == nullptr; ++i) {
    libusb_device_descriptor desc{};
    if (libusb_get_device_descriptor(list[i], &desc) != 0 ||
        desc.idVendor != vid || desc.idProduct != pid)
      continue;
    if (bus >= 0 && libusb_get_bus_number(list[i]) != bus)
      continue;
    if (address >= 0 && libusb_get_device_address(list[i]) != address)
      continue;
    if (libusb_open(list[i], &handle) != 0)
      handle = nullptr;
  }
  if (list != nullptr)
    libusb_free_device_list(list, 1);
  return handle;
}
} // namespace

int main(int argc, char **argv) {
  std::string stage = "id";
  uint16_t want_vid = 0, want_pid = 0;
  uint16_t parsed_bus = 0, parsed_address = 0;
  int want_bus = -1, want_address = -1;
  uint16_t channel = 6, width = 20, offset = 0;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "id" || arg == "off" || arg == "power" ||
        arg == "efuse" || arg == "fw" || arg == "mac" || arg == "phy" ||
        arg == "chan" || arg == "tssi" || arg == "tssi-bb" ||
        arg == "tssi-thermal" || arg == "tssi-analog" ||
        arg == "tssi-enable")
      stage = arg;
    else if (arg == "--vid" && i + 1 < argc && hex16(argv[++i], want_vid))
      ;
    else if (arg == "--pid" && i + 1 < argc && hex16(argv[++i], want_pid))
      ;
    else if (arg == "--bus" && i + 1 < argc &&
             hex16(argv[++i], parsed_bus) && parsed_bus <= 255)
      want_bus = parsed_bus;
    else if (arg == "--address" && i + 1 < argc &&
             hex16(argv[++i], parsed_address) && parsed_address <= 255)
      want_address = parsed_address;
    else if (arg == "--channel" && i + 1 < argc &&
             hex16(argv[++i], channel) && channel <= 255)
      ;
    else if (arg == "--width" && i + 1 < argc &&
             hex16(argv[++i], width) &&
             (width == 5 || width == 10 || width == 20 || width == 40))
      ;
    else if (arg == "--offset" && i + 1 < argc &&
             hex16(argv[++i], offset) && offset <= 2)
      ;
    else {
      std::fprintf(stderr,
                   "usage: %s [id|off|power|efuse|fw|mac|phy|chan|tssi|"
                   "tssi-bb|tssi-thermal|tssi-analog|tssi-enable] "
                   "[--vid N --pid N] [--bus N] [--address N] "
                   "[--channel N --width 5|10|20|40 --offset 0|1|2]\n",
                   argv[0]);
      return 2;
    }
  }
  if ((want_vid == 0) != (want_pid == 0)) {
    std::fprintf(stderr, "--vid and --pid must be supplied together\n");
    return 2;
  }
  const int wanted = stage == "off"     ? -1
                     : stage == "tssi-enable" ? 11
                     : stage == "tssi-analog" ? 10
                     : stage == "tssi-thermal" ? 9
                     : stage == "tssi-bb" ? 8
                     : stage == "tssi"  ? 7
                     : stage == "chan"  ? 6
                     : stage == "phy"   ? 5
                     : stage == "mac"   ? 4
                     : stage == "fw"    ? 3
                     : stage == "efuse" ? 2
                     : stage == "power" ? 1
                                          : 0;
  auto logger = std::make_shared<Logger>();
  devourer::DeviceSession session(logger);
  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) < 0)
    return 1;
  session.adopt_context(ctx);

  libusb_device_handle *handle = nullptr;
  uint16_t vid = want_vid, pid = want_pid;
  if (want_vid) {
    handle = open_matching(ctx, want_vid, want_pid, want_bus, want_address);
  } else {
    for (const rtl8733b::UsbId &id : rtl8733b::kUsbIds) {
      handle = open_matching(ctx, id.vid, id.pid, want_bus, want_address);
      if (handle) {
        vid = id.vid;
        pid = id.pid;
        break;
      }
    }
  }
  if (!handle) {
    logger->error(
        "no matching RTL8733B USB adapter found (expected 0bda:f72b or "
        "0bda:b733; bus={} address={})",
        want_bus, want_address);
    return 1;
  }

  libusb_device *opened = libusb_get_device(handle);
  const uint8_t bus = libusb_get_bus_number(opened);
  const uint8_t address = libusb_get_device_address(opened);

  std::shared_ptr<devourer::UsbDeviceLock> lock;
  if (devourer::claim_interface_then_reset(handle, 0, logger, true, lock) !=
      0) {
    session.adopt_handle(handle, 0);
    return 1;
  }
  session.adopt_handle(handle, 0);
  session.adopt_lock(lock);

  RtlAdapter adapter(handle, logger, ctx, lock);
  rtl8733b::Rtl8733bBringup dev(adapter, logger);
  PowerDownGuard power_down(dev);
  try {
    const rtl8733b::ChipInfo info = dev.read_chip_info();
    const bool id_ok = info.matches();
    logger->info(
        "id: {:04x}:{:04x} bus={} address={} die=0x{:02x} cut={} "
        "SYS_CFG1=0x{:08x} "
        "secure=0x{:02x} CR=0x{:02x} SYS_STATUS1=0x{:08x} "
        "MCUFW_CTRL=0x{:04x} autoload={}",
        vid, pid, bus, address, info.die_id, info.cut, info.sys_cfg1,
        info.secure_ctrl, info.cr, info.sys_status1, info.mcufw_ctrl,
        info.efuse_autoload_ok);
    devourer::Ev(logger->events(), "rtl8733b.id")
        .f("ok", id_ok).hexf("vid", vid, 4).hexf("pid", pid, 4)
        .f("bus", bus).f("address", address)
        .hexf("die_id", info.die_id, 2).f("cut", info.cut)
        .hexf("sys_cfg1", info.sys_cfg1, 8)
        .hexf("secure_ctrl", info.secure_ctrl, 2).hexf("cr", info.cr, 2)
        .hexf("sys_status1", info.sys_status1, 8)
        .hexf("mcufw", info.mcufw_ctrl, 4);
    log_snapshot(logger, dev, "identity");
    if (id_ok && wanted < 0) {
      const bool off_ok = dev.power_off();
      log_snapshot(logger, dev, "power-off");
      return off_ok ? 0 : 1;
    }
    if (!id_ok || wanted < 1)
      return id_ok ? 0 : 1;

    const bool power_ok = dev.power_on();
    power_down.arm();
    const uint8_t cr = adapter.rtw_read8(0x0100);
    logger->info("power: ok={} CR=0x{:02x}", power_ok, cr);
    devourer::Ev(logger->events(), "rtl8733b.power")
        .f("ok", power_ok).hexf("cr", cr, 2);
    log_snapshot(logger, dev, "power");
    if (!power_ok || wanted < 2)
      return power_ok ? 0 : 1;

    rtl8733b::Halmac8733bMac mac(adapter, logger);
    rtl8733b::EfuseInfo efuse;
    const bool efuse_ok = mac.read_efuse(efuse);
    devourer::Ev(logger->events(), "rtl8733b.efuse")
        .f("ok", efuse_ok)
        .f("used_bytes", efuse.used_bytes)
        .hexf("id", efuse.id, 4)
        .hexf("vid", efuse.vid, 4)
        .hexf("pid", efuse.pid, 4)
        .hexf("rfe_type", efuse.rfe_type, 2)
        .hexf("xtal", efuse.xtal, 2)
        .hexf("thermal", efuse.thermal, 2)
        .hexf("trx_path", efuse.trx_path, 2)
        .hexf("tx_power_calibrate", efuse.tx_power_calibrate, 2)
        .f("tx_power_selector", efuse.tx_power_tracking_mode)
        .f("tx_power_mode", tx_power_mode_name(efuse.tx_power_mode))
        .f("tssi_path_a_programmed", efuse.tssi_power.path_a_programmed)
        .f("tssi_path_b_programmed", efuse.tssi_power.path_b_programmed)
        .f("tssi_trim_programmed", efuse.tssi_power.trim_programmed)
        .f("tssi_ch6_cck_a", efuse.tssi_power.path_a_de[2])
        .f("tssi_ch6_cck_b", efuse.tssi_power.path_b_de[2])
        .f("tssi_ch6_ofdm_a", efuse.tssi_power.path_a_de[8])
        .f("tssi_ch6_ofdm_b", efuse.tssi_power.path_b_de[8])
        .f("tssi_ch6_trim_a", efuse.tssi_power.trim[0][0])
        .f("tssi_ch6_trim_b", efuse.tssi_power.trim[0][1]);
    if (!efuse_ok || wanted < 3)
      return efuse_ok ? 0 : 1;

    const bool fw_ok = dev.download_default_firmware(info.cut);
    const uint16_t mcufw = adapter.rtw_read16(0x0080);
    logger->info("fw: ok={} checksum={} ready={} MCUFW_CTRL=0x{:04x}", fw_ok,
                 dev.checksum_ok(), dev.ready_ok(), mcufw);
    devourer::Ev(logger->events(), "rtl8733b.fw")
        .f("ok", fw_ok).f("checksum", dev.checksum_ok())
        .f("ready", dev.ready_ok()).hexf("mcufw", mcufw, 4);
    log_snapshot(logger, dev, "firmware");
    if (!fw_ok || wanted < 4)
      return fw_ok ? 0 : 1;

    bool mac_ok = mac.initialize(efuse);
    bool stopped = false;
    bool reinit_ok = false;
    uint16_t stopped_cr = 0xffff;
    if (mac_ok) {
      mac.stop();
      const rtl8733b::MacState stopped_state = mac.read_mac_state();
      stopped_cr = adapter.rtw_read16(0x0100);
      stopped = stopped_cr == 0 &&
                (stopped_state.pq_map & (1u << 2)) == 0;
      reinit_ok = stopped && mac.initialize(efuse);
      mac_ok = mac_ok && stopped && reinit_ok;
      devourer::Ev(logger->events(), "rtl8733b.mac_lifecycle")
          .f("stopped", stopped)
          .hexf("stopped_cr", stopped_cr, 4)
          .f("reinit", reinit_ok);
    }
    const rtl8733b::MacState mac_state = mac.read_mac_state();
    devourer::Ev(logger->events(), "rtl8733b.mac")
        .f("ok", mac_ok)
        .hexf("pq_map", mac_state.pq_map, 4)
        .hexf("rqpn_hlpq", mac_state.rqpn_hlpq, 8)
        .hexf("rqpn_npq", mac_state.rqpn_npq, 8)
        .hexf("boundary", mac_state.reserved_boundary, 2)
        .hexf("rx_boundary", mac_state.rx_boundary, 4)
        .hexf("cr", mac_state.cr, 2)
        .hexf("rxdma_mode", mac_state.rxdma_mode, 2)
        .hexf("rx_agg", mac_state.rx_agg, 4)
        .hexf("rcr", mac_state.rcr, 8);
    if (!mac_ok || wanted < 5)
      return mac_ok ? 0 : 1;

    rtl8733b::Phy8733b phy(adapter, logger);
    const bool phy_ok = phy.initialize(info.cut, efuse);
    devourer::ThermalStatus thermal;
    if (phy_ok) {
      thermal.raw = phy.read_thermal();
      thermal.baseline = efuse.thermal;
      thermal.valid = thermal.raw != 0 && thermal.baseline != 0xff;
      thermal.delta = thermal.valid
                          ? static_cast<int>(thermal.raw) -
                                static_cast<int>(thermal.baseline)
                          : 0;
      const char *bucket = devourer::ThermalBucket(thermal);
      logger->info("RTL8733B thermal: raw={} baseline={} delta={} status={}",
                   thermal.raw, thermal.baseline, thermal.delta, bucket);
      devourer::Ev(logger->events(), "thermal")
          .f("raw", thermal.raw)
          .f("baseline", thermal.valid ? static_cast<int>(thermal.baseline)
                                        : -1)
          .f("delta", thermal.delta)
          .f("status", bucket);
      /* Reported, not enforced. The meter is a PA-bias tracking index, not a
       * calibrated junction temperature, and it is not a validated degradation
       * predictor (docs/warm-tx-degradation.md) — so it grades the emitted
       * `thermal` event (ThermalBucket labels this band "critical") and the
       * abort decision stays with whoever is driving the probe. */
      if (thermal.valid && thermal.delta >= 25)
        logger->warn("RTL8733B thermal: delta {} is in the 'critical' bucket — "
                     "telemetry only, continuing",
                     thermal.delta);
    }
    if (!phy_ok || wanted < 6)
      return phy_ok ? 0 : 1;
    const ChannelWidth_t selected_width =
        width == 5    ? CHANNEL_WIDTH_5
        : width == 10 ? CHANNEL_WIDTH_10
        : width == 40 ? CHANNEL_WIDTH_40
                      : CHANNEL_WIDTH_20;
    const SelectedChannel selected{static_cast<uint8_t>(channel),
                                   static_cast<uint8_t>(offset),
                                   selected_width};
    const bool channel_ok = phy.set_channel(selected);
    if (!channel_ok || wanted < 7)
      return channel_ok ? 0 : 1;
    const bool bb_ok = wanted == 7 || phy.prepare_tssi_bb(selected, efuse);
    const bool thermal_ok =
        bb_ok && (wanted < 9 || phy.prepare_tssi_thermal(efuse, false));
    const bool de_ok =
        thermal_ok && phy.prepare_tssi_offsets(selected, efuse);
    const bool analog_ok =
        de_ok && (wanted < 10 || phy.audit_tssi_analog(selected, efuse));
    const bool enable_ok =
        analog_ok &&
        (wanted < 11 || phy.audit_tssi_enable(selected, efuse, 64));
    const uint8_t final_thermal = phy.read_thermal();
    const int final_delta =
        final_thermal != 0 && efuse.thermal != 0xff
            ? static_cast<int>(final_thermal) - static_cast<int>(efuse.thermal)
            : 0;
    logger->info("RTL8733B TSSI thermal: final={} baseline={} delta={}",
                 final_thermal, efuse.thermal, final_delta);
    devourer::Ev(logger->events(), "rtl8733b.tssi_thermal")
        .f("raw", final_thermal)
        .f("baseline", efuse.thermal)
        .f("delta", final_delta);
    if (final_thermal == 0 || final_delta >= 25) {
      logger->error("RTL8733B TSSI thermal: unsafe or unreadable final sample");
      return 1;
    }
    return enable_ok ? 0 : 1;
  } catch (const std::exception &e) {
    logger->error("RTL8733B {} stage threw: {}", stage, e.what());
    return 1;
  }
}
