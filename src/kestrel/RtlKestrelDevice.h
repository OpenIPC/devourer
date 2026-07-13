#ifndef RTL_KESTREL_DEVICE_H
#define RTL_KESTREL_DEVICE_H

#include <cstdint>

#include "logger.h"
#include "DeviceConfig.h"
#include "IRtlDevice.h"
#include "RtlAdapter.h"
#include "SelectedChannel.h"

#include "ChipVariant.h"
#include "HalKestrel.h"

namespace kestrel {

/* AX-native chip identity, read via ReadChipInfo(). On this generation the
 * byte at 0x00FC is R_AX_SYS_CHIPINFO (the D-die id: 0x51 = 8852B, 0x52 =
 * 8852C) and the cut version is the high nibble of R_AX_SYS_CFG1+1 (0x00F1):
 * 0 = CAV, 1 = CBV, 2 = CCV, ... — the mac_ax get_chip_info() read pair. */
struct ChipInfo {
  uint8_t die_id = 0; /* R_AX_SYS_CHIPINFO (0x00FC) */
  uint8_t cut = 0;    /* rtw_cv: R_AX_SYS_CFG1[15:12] */
  bool matches(ChipVariant v) const {
    return (v == ChipVariant::C8852B && die_id == 0x51) ||
           (v == ChipVariant::C8852C && die_id == 0x52);
  }
};

} /* namespace kestrel */

/* RtlKestrelDevice is the orchestrator for the Realtek "Kestrel" Wi-Fi 6 /
 * 802.11ax family (G6 "phl" vendor architecture) — RTL8852BU/8832BU and
 * RTL8852CU/8832CU. It implements the same IRtlDevice contract as the three
 * Jaguar (11ac) generations so the demos and WiFiDriver factory treat all
 * four uniformly.
 *
 * Bring-up is ported from the vendor trees reference/rtl8852bu (v1.19.21) and
 * reference/rtl8852cu (v1.19.22): mac_ax power-on / firmware download / TRX
 * init, halbb PHY tables, halrf calibration. Milestone status:
 *   M0 (this skeleton): identity — PID dispatch + die-id/cut confirmation.
 *   M1: power-on, FW download (NICCE image), efuse.       [not yet ported]
 *   M2: MAC TRX init, monitor RX, 11ax frame parser.      [not yet ported]
 *   M3: channel/BW via halbb, calibration minimum set.    [not yet ported]
 *   M4: TX (mac_ax TXWD), TSSI power-by-rate.             [not yet ported]
 * Unported entry points throw std::runtime_error naming the milestone. */
class RtlKestrelDevice : public IRtlDevice {
public:
  RtlKestrelDevice(RtlAdapter device, Logger_t logger,
                   kestrel::ChipVariant variant = kestrel::ChipVariant::C8852B,
                   devourer::DeviceConfig cfg = {});

  void Init(Action_ParsedRadioPacket packetProcessor,
            SelectedChannel channel) override;
  void InitWrite(SelectedChannel channel) override;
  void StartRxLoop(Action_ParsedRadioPacket packetProcessor) override;
  void StopRxLoop() override { _rx_stop = true; }
  void SetMonitorChannel(SelectedChannel channel) override;
  bool send_packet(const uint8_t *packet, size_t length) override;
  SelectedChannel GetSelectedChannel() override { return _channel; }

  /* AX-native identity read (no power-on needed — the identity block is alive
   * as soon as the USB function enumerates). kestrelprobe stage "id" and the
   * constructor's confirmation log both come through here. */
  kestrel::ChipInfo ReadChipInfo();

  kestrel::ChipVariant variant() const { return _variant; }

  /* M1a bring-up, exposed for kestrelprobe's "power" stage: power the MAC on
   * and dump the efuse. Not part of the IRtlDevice contract (Init/InitWrite
   * drive the full sequence once M1b+M2 land). Returns false on failure. */
  bool PowerOnAndReadEfuse(kestrel::EfuseInfo &out);

  /* M1b bring-up for kestrelprobe's "fw" stage: power on, read efuse, then
   * download firmware. Returns false at the first failing step. */
  bool PowerOnEfuseAndFw(kestrel::EfuseInfo &out);

  /* M2a bring-up for kestrelprobe's "trx" stage: everything in "fw" plus the
   * DMAC-half MAC TRX init. Returns false at the first failing step. */
  bool PowerOnFwAndTrx(kestrel::EfuseInfo &out);

  /* M3 bring-up for kestrelprobe's "phy" stage: everything in "trx" plus the
   * BB + RF table apply. Returns false at the first failing step. */
  bool PowerOnTrxAndPhy(kestrel::EfuseInfo &out);

  kestrel::HalKestrel &hal() { return _hal; }

  /* Full monitor bring-up (power -> efuse -> fw -> MAC TRX -> BB/RF -> channel)
   * without starting the RX loop. Returns false at the first failing step.
   * Init() calls this then StartRxLoop; a probe "rx"/"chan" stage can call it
   * standalone. */
  bool BringUpMonitor(SelectedChannel channel);

private:
  [[noreturn]] void not_ported(const char *entry, const char *milestone) const;

  RtlAdapter _device;
  Logger_t _logger;
  kestrel::ChipVariant _variant;
  devourer::DeviceConfig _cfg;
  kestrel::HalKestrel _hal;
  SelectedChannel _channel{};
  kestrel::EfuseInfo _efuse{};
  volatile bool _rx_stop = false;
  uint8_t _tx_mgmt_ep = 0; /* band-0 mgmt bulk-OUT ep (BULKOUTID0), 0=TX not up */
  uint8_t _tx_data_ep = 0; /* AC0 data bulk-OUT ep (BULKOUTID3) */
  uint16_t _tx_seq = 0;    /* rolling 12-bit wifi sequence for injected frames */
};

#endif /* RTL_KESTREL_DEVICE_H */
