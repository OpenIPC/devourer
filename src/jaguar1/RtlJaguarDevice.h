#ifndef RTL_JAGUAR_DEVICE_H
#define RTL_JAGUAR_DEVICE_H

#include <array>
#include <atomic>
#include <functional>
#include <iostream>
#include <iomanip>
#include <memory>
#include <optional>
#include <thread>

#include "logger.h"
#include "BbDbgportReader.h"
#include "HalModule.h"
#include "IRtlDevice.h"
#include "SelectedChannel.h"
#include "EepromManager.h"
#include "RadioManagementModule.h"
#include "FrameParser.h"
#include "TxMode.h"

extern "C"
{
#include "ieee80211_radiotap.h"
}

/* Action_ParsedRadioPacket is declared in IRtlDevice.h (shared with the
 * Jaguar3 device and the factory). */

/* RtlJaguarDevice is the orchestrator for the Realtek "Jaguar" 802.11ac family
 * — RTL8812AU (2T2R), RTL8811AU (1T1R cut), and RTL8814AU (4T4R RF / 3-SS
 * baseband). The chip is identified at construction time via SYS_CFG bits and
 * USB PID; this class drives bring-up, RX, and TX for whichever member of the
 * family is present. */
class RtlJaguarDevice : public IRtlDevice {
  std::shared_ptr<EepromManager> _eepromManager;
  std::shared_ptr<RadioManagementModule> _radioManagement;
  /* Last channel handed to SetMonitorChannel. Value-initialised so the
   * 5GHz CCK clamp in send_packet reads Channel=0 (clamp off) rather than
   * indeterminate garbage before the first channel set. */
  SelectedChannel _channel{};
  RtlUsbAdapter _device;
  HalModule _halModule;
  Logger_t _logger;
  uint8_t debug;
  Action_ParsedRadioPacket _packetProcessor = nullptr;
  /* Runtime TX-mode default (SetTxMode/ClearTxMode); applied in send_packet
   * only when the frame's radiotap carries no rate. */
  std::optional<devourer::TxMode> _tx_mode_default;

public:
  RtlJaguarDevice(RtlUsbAdapter device, Logger_t logger);
  ~RtlJaguarDevice() override;
  void Init(Action_ParsedRadioPacket packetProcessor,
            SelectedChannel channel) override;
  /* Blocking RX worker loop on an already-brought-up chip (see IRtlDevice).
   * Init = bring-up + BFEE arm + StartRxLoop; a TX+RX caller does InitWrite
   * once, then runs this on its own std::thread next to the TX loop. */
  void StartRxLoop(Action_ParsedRadioPacket packetProcessor) override;
  void StopRxLoop() override { should_stop = true; }
  void SetMonitorChannel(SelectedChannel channel) override;
  /* Lean frequency-hop retune: switches the RF channel only, skipping the
   * per-rate TX-power loop, bandwidth post-set, and thermal pwrtrk tick that
   * SetMonitorChannel runs — none of which change across an intra-band, 20 MHz
   * hop. Falls back to the full SetMonitorChannel automatically when the hop
   * crosses the 2.4/5 GHz boundary or the current bandwidth isn't 20 MHz.
   * cache_rf=true additionally avoids the per-write 20 ms C-cut RF-read sleep
   * by writing RF_CHNLBW from a cached value. Intended for channel hopping;
   * keeps the device channel state in sync for the 5 GHz CCK clamp. */
  void FastRetune(uint8_t channel, bool cache_rf = true);
  void InitWrite(SelectedChannel channel) override;
  void SetTxPower(uint8_t power) override;
  /* Force the per-rate TXAGC index (0..63), bypassing the EFUSE per-rate
   * table; -1 restores normal behaviour. Re-applied on the next
   * SetMonitorChannel. Used by the thermal-vs-gain ramp in WiFiDriverTxDemo. */
  void SetTxPowerOverride(int idx);
  /* Re-apply the per-rate TX power for the current channel immediately (no
   * channel switch). Needed because SetMonitorChannel early-returns when the
   * channel is unchanged. Call after SetTxPowerOverride to make it take. */
  void ApplyTxPower();
  /* Read a baseband register (debug/diagnostic). Thin passthrough to the
   * radio manager's BB read — handy for confirming a TXAGC write landed. */
  uint32_t ReadBBReg(uint16_t addr, uint32_t mask);

  /* Runtime TX-mode default. send_packet honours a frame's own radiotap rate
   * fields per-packet; when a frame's radiotap carries no rate, this mode
   * supplies the modulation / MCS / BW / GI / FEC / STBC instead of the
   * built-in 1M-CCK fallback. Pure state — applied on the next send_packet,
   * no USB I/O. ClearTxMode() reverts to the built-in default. */
  void SetTxMode(const devourer::TxMode& mode);
  void ClearTxMode();

  bool send_packet(const uint8_t* packet, size_t length) override;
  SelectedChannel GetSelectedChannel() override;

  bool should_stop = false;

  /* Per-queue free-page snapshot read from REG_FIFOPAGE_INFO_1..5
   * (0x0230 / 0x0234 / 0x0238 / 0x023C / 0x0240). 8814A-only — these
   * registers don't exist on 8812 / 8821, so the snapshot stays at all
   * zeros on those chips and start_queue_depth_poller() is a no-op.
   *
   * Polling cadence is set via interval_ms (DEVOURER_QUEUE_POLL_MS in
   * the demos). 0 = disabled. The poller spawns a worker thread that
   * issues a vendor-control read for each register every interval and
   * stores the raw 32-bit word — interpretation (which bits are free
   * pages vs reserved) is left to the consumer, since the per-queue
   * layout differs slightly between chip cuts. */
  void start_queue_depth_poller(uint32_t interval_ms);
  std::array<uint32_t, 5> get_queue_depth() const;

  /* Read the chip thermal meter (RF[A][0x42][15:10]) paired with the EFUSE
   * baseline. Read-only — leaves the TX-power-tracking BB-swing registers
   * untouched. Works on every Jaguar member. Safe to call from the thread
   * that owns the device (e.g. inline in a TX loop) — no USB contention.
   * See ThermalStatus in RadioManagementModule.h for field semantics. */
  ThermalStatus GetThermalStatus();

  /* Spawn a background thread that samples the thermal meter every
   * interval_ms and stores a snapshot (queryable via get_thermal_snapshot).
   * Emits a logger->warn when delta >= warn_delta. 0 interval = disabled.
   * Intended for the RX demo, whose Init() blocks the main thread.
   *
   * CONCURRENCY: an RF read is a multi-step BB register sequence over the
   * shared libusb handle. Background phydm-style polling has wedged the chip
   * before (ch100 second-channel-set), so this poller is opt-in and should
   * use a conservative cadence (>= 1 s). A TX loop on the owning thread
   * should prefer the synchronous GetThermalStatus() instead. */
  void start_thermal_poller(uint32_t interval_ms, int warn_delta);
  ThermalStatus get_thermal_snapshot() const;

  /* F2 research helper: read a u32 from the BB debug port at `selector`,
   * with save/restore around register 0x8FC. Lazy-constructs the reader
   * on first call. Returns 0 if the chip wedged on a prior call. See
   * BbDbgportReader.h for the brick-risk caveats. */
  uint32_t read_bb_dbgport(uint32_t selector);
  bool bb_dbgport_wedged() const;

private:
  void StartWithMonitorMode(SelectedChannel selectedChannel);
  bool NetDevOpen(SelectedChannel selectedChannel);

  std::array<std::atomic<uint32_t>, 5> _qd_snap{};
  std::thread _qd_thread;
  std::atomic<bool> _qd_stop{false};

  /* DEVOURER_RX_PATHS toggle thread — cycles the RX-path mask (0x808) for the
   * mobile/fading combining measurement. */
  void start_rx_path_toggle(const std::vector<uint8_t> &masks,
                            uint32_t interval_ms);
  std::thread _rxmask_thread;
  std::atomic<bool> _rxmask_stop{false};

  std::thread _therm_thread;
  std::atomic<bool> _therm_stop{false};
  /* Packed last thermal snapshot: bit0 = valid, [8:15] = raw,
   * [16:23] = baseline, [24:31] = signed delta (clamped to int8). Stored as
   * one atomic so a reader sees a consistent tuple without a mutex. */
  std::atomic<uint32_t> _therm_snap{0};

  std::unique_ptr<devourer::BbDbgportReader> _bb_dbgport;
};

/* Backwards-compatibility alias. External callers using the old name still
 * compile but get a deprecation warning. Remove after one release cycle. */
using Rtl8812aDevice [[deprecated("renamed to RtlJaguarDevice")]] =
    RtlJaguarDevice;

#endif /* RTL_JAGUAR_DEVICE_H */
