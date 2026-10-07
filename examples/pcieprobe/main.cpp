/* pcieprobe — staged bring-up driver for the PCIe transport (RTL8821CE on
 * the 88xx ring plane; RTL8852CE on the AX plane — see probe_8852c below).
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
#include "RtlAdapter.h"
#include "logger.h"

#include "jaguar2/ChipVariant.h"
#if defined(DEVOURER_HAVE_JAGUAR2_8821C)
#include "jaguar2/HalJaguar2.h"
#include "jaguar2/HalmacJaguar2Fw.h"
#include "jaguar2/HalmacJaguar2MacInit.h"
#endif
#if defined(DEVOURER_HAVE_KESTREL_8852C)
#include "kestrel/ChipVariant.h"
#include "kestrel/HalKestrel.h"
#endif
#include <fstream>

/* PCI device id from sysfs — the AX dies dispatch id-first (kestrel/CLAUDE.md:
 * 0x00FC is R_AX_SYS_CHIPINFO there, not the Jaguar chip-id). */
static uint16_t pci_device_id(const std::string &bdf) {
  std::ifstream f("/sys/bus/pci/devices/" + bdf + "/device");
  unsigned v = 0;
  f >> std::hex >> v;
  return static_cast<uint16_t>(v);
}

#if defined(DEVOURER_HAVE_KESTREL_8852C)
/* RTL8852CE (Kestrel, AX): M0 die-id + cut, M1 = HalKestrel PCIe power-on
 * (mac_pwr_on_nic_pcie_8852c) + EFUSE, cross-checking the efuse copy of the
 * PCI ids against config space; M2 = firmware download over the AX HAXI
 * FWCMD ring (HalKestrel::download_firmware runs the PCIe pre-init, which
 * programs the rings through hci_setup, then the three-phase FWDL). */
static int probe_8852c(RtlAdapter &adapter, Logger_t logger, int want) {
  const uint8_t die_id = adapter.rtw_read8(0x00FC);
  kestrel::HalKestrel hal(adapter, logger, kestrel::ChipVariant::C8852C);
  const uint8_t cut = hal.read_cut();
  const bool id_ok = die_id == 0x52;
  logger->info("M0 (8852C): die-id=0x{:02x} (want 0x52) cut={}", die_id, cut);
  devourer::Ev(logger->events(), "pcie.id")
      .f("ok", id_ok)
      .f("chip", "8852c")
      .hexf("chip_id", die_id, 2)
      .f("cut", static_cast<int>(cut));
  if (!id_ok || want < 1)
    return id_ok ? 0 : 1;

  bool power_ok = false;
  kestrel::EfuseInfo ef;
  try {
    power_ok = hal.power_on() && hal.read_efuse(ef);
  } catch (const std::exception &e) {
    logger->error("M1 (8852C): power-on failed: {}", e.what());
  }
  char mac[18];
  snprintf(mac, sizeof(mac), "%02x:%02x:%02x:%02x:%02x:%02x", ef.mac[0],
           ef.mac[1], ef.mac[2], ef.mac[3], ef.mac[4], ef.mac[5]);
  const bool ids_ok = ef.pci_vid == 0x10EC && ef.pci_did == 0xC852;
  const bool ok = power_ok && ef.autoload_ok && ids_ok;
  logger->info("M1 (8852C): power_ok={} autoload={} efuse pci={:04x}:{:04x} "
               "MAC(0x400)={} rfe={} xtal=0x{:02x}",
               power_ok, ef.autoload_ok, ef.pci_vid, ef.pci_did, mac,
               ef.rfe_type, ef.xtal_cap);
  devourer::Ev(logger->events(), "pcie.power")
      .f("ok", ok)
      .hexf("efuse_vid", ef.pci_vid, 4)
      .hexf("efuse_did", ef.pci_did, 4)
      .f("mac", mac)
      .f("rfe", static_cast<int>(ef.rfe_type));
  if (!ok || want < 2)
    return ok ? 0 : 1;

  bool fw_ok = false;
  try {
    fw_ok = hal.download_firmware(cut);
  } catch (const std::exception &e) {
    logger->error("M2 (8852C): FWDL failed: {}", e.what());
  }
  const uint32_t wcpu = adapter.rtw_read32(0x01E0); /* R_AX_WCPU_FW_CTRL */
  const uint32_t err = hal.fw_err_state("pcieprobe-fw");
  logger->info("M2 (8852C): fw_ok={} WCPU_FW_CTRL=0x{:08x} ser-err=0x{:08x}",
               fw_ok, wcpu, err);
  devourer::Ev(logger->events(), "pcie.fw")
      .f("ok", fw_ok)
      .hexf("wcpu_fw_ctrl", wcpu, 8)
      .hexf("ser_err", err, 8);
  if (!fw_ok)
    return 1;
  if (want >= 3) {
    /* M3 (bb): is the BB register window (+0x10000, halbb/halrf's plane)
     * reachable through BAR2? set_enable_bb_rf then read a few BB registers
     * both ways (adapter wide path vs raw BAR offset) plus a MAC register. */
    hal.enable_bb_rf();
    /* Alias check: does the BAR decode more than 16 address bits? */
    const uint32_t mac0 = adapter.rtw_read32(0x0000);
    const uint32_t mac0c = adapter.rtw_read32(0x000c);
    const uint32_t a2 = adapter.rtw_read32_wide(0x20000);
    const uint32_t a3 = adapter.rtw_read32_wide(0x30000);
    const uint32_t a4 = adapter.rtw_read32_wide(0x40000);
    logger->info("M3 (8852C): alias check MAC[0x0]=0x{:08x} MAC[0xc]=0x{:08x} "
                 "[0x20000]=0x{:08x} [0x30000]=0x{:08x} [0x40000]=0x{:08x}",
                 mac0, mac0c, a2, a3, a4);
    /* The post-init releases the PCIe IO stop; test the window both before
     * and after it. */
    const uint32_t pre4004 = adapter.rtw_read32_wide(0x14004);
    adapter.rtw_write32_wide(0x14004, 0xCA014000u);
    const uint32_t pre_wr = adapter.rtw_read32_wide(0x14004);
    const uint32_t stop1 = adapter.rtw_read32(0x1010);
    hal.pcie_init();
    logger->info("M3 (8852C): before post-init: 0x4004={:08x} write->{:08x} "
                 "HAXI_DMA_STOP1=0x{:08x}; now 0x{:08x}",
                 pre4004, pre_wr, stop1, adapter.rtw_read32(0x1010));
    const uint32_t bb0 = adapter.rtw_read32_wide(0x10000);
    const uint32_t bb4004 = adapter.rtw_read32_wide(0x14004);
    const uint32_t bb000c = adapter.rtw_read32_wide(0x1000c);
    const uint32_t raw4004 = adapter.rtw_read32_wide(0x14004);
    const uint32_t mac4004 = adapter.rtw_read32(0x4004);
    logger->info("M3 (8852C): BB window: [0x10000]=0x{:08x} [0x14004]=0x{:08x} "
                 "[0x1000c]=0x{:08x} raw[0x14004]=0x{:08x} MAC[0x4004]=0x{:08x}",
                 bb0, bb4004, bb000c, raw4004, mac4004);
    /* Write/readback through the window: does a BB write land? */
    adapter.rtw_write32_wide(0x14004, 0xCA014000u);
    const uint32_t wr4004 = adapter.rtw_read32_wide(0x14004);
    const uint32_t old000c = bb000c;
    adapter.rtw_write32_wide(0x1000c, old000c ^ 0x00000f00u);
    const uint32_t wr000c = adapter.rtw_read32_wide(0x1000c);
    adapter.rtw_write32_wide(0x1000c, old000c);
    logger->info("M3 (8852C): BB write/readback: 0x4004 <- CA014000 reads "
                 "0x{:08x}; 0x000c ^0xf00 reads 0x{:08x} (was 0x{:08x})",
                 wr4004, wr000c, old000c);
    devourer::Ev(logger->events(), "pcie.bb")
        .hexf("bb_0", bb0, 8)
        .hexf("bb_4004", bb4004, 8)
        .hexf("bb_000c", bb000c, 8)
        .hexf("wr_4004", wr4004, 8)
        .hexf("wr_000c", wr000c, 8);
  }
  hal.pcie_deinit();
  return 0;
}
#endif

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "usage: %s <bdf e.g. 0000:01:00.0> [id|power|fw|bb]\n",
            argv[0]);
    return 2;
  }
  const std::string bdf = argv[1];
  const std::string stage = argc > 2 ? argv[2] : "id";
  const int want = stage == "bb" ? 3 : stage == "fw" ? 2 : stage == "power" ? 1 : 0;

  auto logger = std::make_shared<Logger>();

  auto transport = devourer::PcieTransport::Open(bdf, logger);
  if (!transport) {
    devourer::Ev(logger->events(), "pcie.id").f("ok", false).f("why", "open");
    return 1;
  }

  /* ---- stage id (M0): pure MMIO register plane, no power, no DMA ---- */
  RtlAdapter adapter(transport, logger, {});
  if (pci_device_id(bdf) == 0xC852) {
#if defined(DEVOURER_HAVE_KESTREL_8852C)
    return probe_8852c(adapter, logger, want);
#else
    logger->error("RTL8852CE found but 8852C support not compiled in");
    return 1;
#endif
  }
#if !defined(DEVOURER_HAVE_JAGUAR2_8821C)
  logger->error("PCI device {:04x} is not an RTL8852CE and 8821C support is not "
                "compiled in", pci_device_id(bdf));
  return 1;
#else
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
#endif /* DEVOURER_HAVE_JAGUAR2_8821C */
}
