#ifndef RTL_JAGUAR3_DEVICE_H
#define RTL_JAGUAR3_DEVICE_H

#include <atomic>
#include <mutex>
#include <optional>
#include <thread>

#include "logger.h"
#include "IRtlDevice.h"
#include "TxMode.h"
#include "RtlUsbAdapter.h"
#include "SelectedChannel.h"
#include "ChipVariant.h"
#include "HalJaguar3.h"
#include "RadioManagementJaguar3.h"

/* RtlJaguar3Device is the orchestrator for the Realtek "Jaguar3" 802.11ac family
 * — RTL8822CU, RTL8812EU, RTL8822EU. It is the Jaguar3 sibling of
 * RtlJaguarDevice (Jaguar1) and implements the same IRtlDevice contract so the
 * demos and WiFiDriver factory treat both uniformly.
 *
 * Bring-up is ported from Realtek vendor source (rtl88x2cu/phydm/halrf):
 * power-on, firmware download, MAC/BB/RF config, IQK and DACK drive RX,
 * channel/bandwidth (incl. 5/10 MHz narrowband) and on-air TX. send_packet is
 * on-air; sustained continuous TX is kept alive by the coex runtime thread
 * (coex_runtime_loop) — see CLAUDE.md. */
class RtlJaguar3Device : public IRtlDevice {
public:
  RtlJaguar3Device(RtlUsbAdapter device, Logger_t logger,
                   jaguar3::ChipVariant variant = jaguar3::ChipVariant::C8822C);
  ~RtlJaguar3Device() override;

  void Init(Action_ParsedRadioPacket packetProcessor,
            SelectedChannel channel) override;
  /* Blocking RX worker loop on an already-brought-up chip (see IRtlDevice).
   * Init = bring-up + BFEE arm + StartRxLoop; a TX+RX caller (self-sounding
   * single-radio ground station) does InitWrite once, then runs this on its
   * own std::thread next to the TX loop. NB: for reliable RX the TX+RX intent
   * must be declared at InitWrite time (DEVOURER_TX_WITH_RX set) — InitWrite
   * then keeps the RX filters open and enables the RX path during bring-up.
   * On a plain TX-only bring-up this falls back to a best-effort retrofit
   * (filter re-open + enable_rx_path), which has proven unreliable on 8822E.
   * Takes over the bulk-IN endpoint from the coex thread's C2H drain for as
   * long as it runs. */
  void StartRxLoop(Action_ParsedRadioPacket packetProcessor) override;
  void StopRxLoop() override { _rx_stop = true; }
  void SetMonitorChannel(SelectedChannel channel) override;
  void InitWrite(SelectedChannel channel) override;
  void SetTxPower(uint8_t power) override;
  bool send_packet(const uint8_t *packet, size_t length) override;
  SelectedChannel GetSelectedChannel() override;
  void Stop() override;
  /* Runtime TX-mode default — applied in send_packet when the radiotap carries
   * no rate. Without this the Jaguar3 TX path fell back to MGN_1M for rate-less
   * frames (so DEVOURER_TX_RATE/an MCS flood went on-air at 1 Mbps): the feature
   * was ported for Jaguar1 (RtlJaguarDevice) but not Jaguar3. */
  void SetTxMode(const devourer::TxMode &mode) override;
  void ClearTxMode() override;
  bool should_stop = false;

private:
  RtlUsbAdapter _device;
  Logger_t _logger;
  jaguar3::ChipVariant _variant;
  jaguar3::HalJaguar3 _hal;
  jaguar3::RadioManagementJaguar3 _radioManagement;
  SelectedChannel _channel{};
  Action_ParsedRadioPacket _packetProcessor = nullptr;
  /* Optional flat TXAGC override from SetTxPower(); -1 = use the chip's
   * efuse-calibrated power. Applied during InitWrite (so it may be set before
   * the device is brought up). */
  int _tx_pwr_override = -1;
  /* Runtime TX-mode default (SetTxMode/ClearTxMode). */
  std::optional<devourer::TxMode> _tx_mode_default;
  /* Coex runtime: a background thread that drains bulk-IN, dispatches firmware
   * C2H reports (BT-info etc.) and runs the periodic coex decision so the FW's
   * PTA keeps the antenna with WLAN during sustained TX.
   * THREADING CONTRACT: while a TX session is active (between InitWrite and Stop)
   * this thread owns all chip register access; the ~2 s housekeeping tick and
   * StartRxLoop's one-shot RX-filter restore serialize on _reg_mu. The TX hot
   * path (send_packet) does no register I/O, so it runs lock-free alongside it;
   * callers must NOT invoke SetMonitorChannel/SetTxPower (which do register RMW)
   * during an active TX session, or those reads-modify-writes will race the coex
   * thread. Bulk-IN has exactly one reader at a time: while StartRxLoop is
   * active (_rx_loop_active) the coex thread skips its C2H drain — the RX async
   * loop sees the C2H reports as part of its stream. */
  std::thread _coex_thread;
  volatile bool _coex_stop = false;
  void coex_runtime_loop();
  /* StartRxLoop stop request (StopRxLoop). */
  volatile bool _rx_stop = false;
  /* True while StartRxLoop owns bulk-IN (gates the coex thread's drain). */
  std::atomic<bool> _rx_loop_active{false};
  /* Set when InitWrite zeroes the RX filters (0x6A0-0x6A4) for TX-only
   * throughput; StartRxLoop restores the monitor_rx_cfg values and clears it. */
  bool _rx_filters_closed = false;
  /* Serializes the coex housekeeping tick against StartRxLoop's register
   * restore (the only two register writers during an active TX session). */
  std::mutex _reg_mu;
};

#endif /* RTL_JAGUAR3_DEVICE_H */
