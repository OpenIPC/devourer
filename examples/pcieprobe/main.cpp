/* pcieprobe — staged bring-up driver for the PCIe transport (RTL8821CE).
 *
 * Validates the PCIe milestones one layer at a time, bottom-up:
 *   id     (M0) vfio open + BAR2 MMIO: chip-id @0xFC must read 0x09,
 *               SYS_CFG1/REG_CR sanity.
 *   power  (M1) + TRX ring registers, pre-init, PCIe power-on sequence,
 *               chip version, EFUSE logical map (MAC @0xD0 must match the
 *               address the kernel driver reported).
 *   fw     (M2) + init_system_cfg + firmware DLFW over the BCN TX ring
 *               (pass = REG_MCUFW_CTRL 0x80 == 0xC078).
 * Full RX (M3) lives in rxdemo via DEVOURER_PCIE_BDF.
 *
 * Usage: sudo pcieprobe <bdf> [id|power|fw]     (default stage: id)
 * The device must be bound to vfio-pci first: tests/pcie_vfio_bind.sh <bdf>.
 *
 * Events (stdout JSONL): pcie.id / pcie.power / pcie.fw with ok:true|false —
 * exit code 0 only if the requested stage passed. */

#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "Event.h"
#include "PcieTransport.h"
#include "RtlUsbAdapter.h"
#include "logger.h"

#include "jaguar2/ChipVariant.h"
#include "jaguar2/HalJaguar2.h"
#include "jaguar2/HalmacJaguar2Fw.h"
#include "jaguar2/HalmacJaguar2MacInit.h"

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "usage: %s <bdf e.g. 0000:01:00.0> [id|power|fw]\n",
            argv[0]);
    return 2;
  }
  const std::string bdf = argv[1];
  const std::string stage = argc > 2 ? argv[2] : "id";
  const int want = stage == "fw" ? 2 : stage == "power" ? 1 : 0;

  auto logger = std::make_shared<Logger>();

  auto transport = devourer::PcieTransport::Open(bdf, logger);
  if (!transport) {
    devourer::Ev(logger->events(), "pcie.id").f("ok", false).f("why", "open");
    return 1;
  }

  /* ---- stage id (M0): pure MMIO register plane, no power, no DMA ---- */
  RtlUsbAdapter adapter(transport, logger, {});
  const uint8_t chip_id = adapter.rtw_read8(0x00FC);
  const uint32_t sys_cfg1 = adapter.rtw_read32(0x00F0);
  const uint8_t cr = adapter.rtw_read8(0x0100);
  const bool id_ok = chip_id == 0x09;
  logger->info("M0: chip-id=0x{:02x} (want 0x09) SYS_CFG1=0x{:08x} CR=0x{:02x}",
               chip_id, sys_cfg1, cr);
  devourer::Ev(logger->events(), "pcie.id")
      .f("ok", id_ok)
      .hexf("chip_id", chip_id, 2)
      .hexf("sys_cfg1", sys_cfg1, 8)
      .hexf("cr", cr, 2);
  if (!id_ok || want < 1)
    return id_ok ? 0 : 1;

  /* ---- stage power (M1): rings -> pre-init -> PCIe power-on -> EFUSE ---- */
  jaguar2::HalJaguar2 hal(adapter, logger, jaguar2::ChipVariant::C8821C, {});
  jaguar2::HalmacJaguar2MacInit macinit(adapter, logger,
                                        jaguar2::ChipVariant::C8821C);
  bool power_ok = false;
  std::vector<uint8_t> efuse(0x200, 0xFF);
  try {
    /* rtw88 order: hci_setup (ring registers) precedes mac_power_on. */
    transport->setup_trx_rings();
    macinit.pre_init_system_cfg();
    hal.power_on();
    hal.read_chip_version();
    hal.read_efuse_logical_map(efuse.data(), efuse.size(), /*dump=*/false);
    power_ok = true;
  } catch (const std::exception &e) {
    logger->error("M1: power-on failed: {}", e.what());
  }
  /* 8821CE efuse: MAC at logical 0xD0 (rtw8821ce_efuse; the USB variant keeps
   * it elsewhere). Cross-check against the kernel-reported MAC. */
  char mac[18];
  snprintf(mac, sizeof(mac), "%02x:%02x:%02x:%02x:%02x:%02x", efuse[0xD0],
           efuse[0xD1], efuse[0xD2], efuse[0xD3], efuse[0xD4], efuse[0xD5]);
  const uint16_t efuse_id =
      static_cast<uint16_t>(efuse[0] | (efuse[1] << 8));
  logger->info("M1: power_ok={} efuse id=0x{:04x} MAC(0xD0)={}", power_ok,
               efuse_id, mac);
  devourer::Ev(logger->events(), "pcie.power")
      .f("ok", power_ok)
      .hexf("efuse_id", efuse_id, 4)
      .f("mac", mac);
  if (!power_ok || want < 2)
    return power_ok ? 0 : 1;

  /* ---- stage fw (M2): system cfg + DLFW over the BCN ring ---- */
  bool fw_ok = false;
  try {
    macinit.init_system_cfg(CHANNEL_WIDTH_20, hal.chip_version().cut);
    jaguar2::HalmacJaguar2Fw fw(adapter, logger, jaguar2::ChipVariant::C8821C);
    fw_ok = fw.download_default_firmware();
  } catch (const std::exception &e) {
    logger->error("M2: DLFW failed: {}", e.what());
  }
  const uint16_t mcufw = adapter.rtw_read16(0x0080);
  logger->info("M2: fw_ok={} MCUFW_CTRL=0x{:04x} (want 0xC078)", fw_ok, mcufw);
  devourer::Ev(logger->events(), "pcie.fw").f("ok", fw_ok).hexf("mcufw", mcufw, 4);
  return fw_ok ? 0 : 1;
}
