#ifndef MT7612U_RADIO_H
#define MT7612U_RADIO_H

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <libusb.h>

#include "DeviceConfig.h"
#include "IRadio.h"
#include "RxPacket.h"
#include "UsbDeviceLock.h"
#include "logger.h"
#include "mt7612u/Mt7612uRxQueue.h"
#include "mt7612u/mt7612u.h"

/*
 * The MediaTek MT7612U behind IRadio.
 *
 * It derives from IRadio and NOT from IRtlRadio: the Realtek-only members
 * (phydm energy counters, the crystal-cap trim, EFUSE stability, the canary
 * register dump) describe a register plane this silicon does not have, and
 * inheriting them would mean answering for hardware that is not here.
 *
 * What it does NOT do is reuse RtlAdapter. That transport is shaped around
 * 16-bit Realtek registers and Realtek bulk endpoints; MT7612U is 32-bit
 * registers over EP0 vendor requests plus an in-band MCU on EP8/EP5, with
 * firmware to upload before any of it answers. The C library under this
 * directory is that transport, and this class owns one `mt7612u_dev`.
 *
 * THREE ORDERING RULES, each of which has cost real hardware time:
 *
 *  1. RX ring first, receiver second. Enabling MAC RX with nothing draining
 *     the bulk-IN endpoint wedges this part BELOW the USB level:
 *     libusb_reset_device, the sysfs authorized toggle and rebinding the
 *     kernel driver all fail, and only a physical replug recovers it.
 *     Teardown is the mirror image — quiesce the receiver, then remove the
 *     drain (mt7612u_rx_quiesce, then mt7612u_rx_stop).
 *
 *  2. The monitor filter goes on AFTER mt7612u_start(), which rewrites
 *     MT_RX_FILTR_CFG to mt76's managed-station value. Measured against the
 *     bring-up harness in the same minute on the same silicon: 0 OFDM frames
 *     of 244 with the managed filter, 103 of 402 with the monitor filter. A
 *     single-path test would have called 244 beacons a working receiver.
 *
 *  3. The 1 Hz PHY tick is not optional. Measured against a peer 20 cm away
 *     airing 3037 fps: a receiver with no tick takes 3 frames in 10 s; with it,
 *     5415-5470/s. It runs on a thread this class owns rather than inside
 *     StartRxLoop, because a transmit-only consumer (InitWrite with no RX loop)
 *     needs it too.
 *
 * Locking. `_mu` guards the control plane — bring-up, channel sets, power, and
 * the tick, all of which issue MCU commands that share one 4-bit sequence
 * number and one response endpoint and so need a single user. The RX callback
 * deliberately does NOT take it: it runs on the C layer's libusb event thread
 * and only reads `_rx_processor`, which is written before the ring starts and
 * not touched again until after it stops.
 */
class Mt7612uRadio : public IRadio {
public:
  Mt7612uRadio(libusb_device_handle *handle, libusb_context *ctx,
               std::shared_ptr<devourer::UsbDeviceLock> usb_lock,
               Logger_t logger, devourer::DeviceConfig cfg);
  ~Mt7612uRadio() override;

  Mt7612uRadio(const Mt7612uRadio &) = delete;
  Mt7612uRadio &operator=(const Mt7612uRadio &) = delete;

  /* --- the pure-virtual core --- */
  void Init(Action_ParsedRadioPacket packetProcessor,
            SelectedChannel channel) override;
  void InitWrite(SelectedChannel channel) override;
  void StartRxLoop(Action_ParsedRadioPacket packetProcessor) override;
  /* Safe to call from the packet processor: since delivery moved to this
   * thread, StopRxLoop no longer joins a thread the processor is running on. */
  void StopRxLoop() override;
  void SetMonitorChannel(SelectedChannel channel) override;
  bool send_packet(const uint8_t *packet, size_t length) override;
  size_t send_packets(const TxPacketView *pkts, size_t count) override;
  SelectedChannel GetSelectedChannel() override;
  void SetCcaMode(bool disabled) override;

  /* --- optional members this silicon can actually answer for --- */
  void Stop() override;
  devourer::AdapterCaps GetAdapterCaps() override;
  devourer::TxCaps GetTxCaps() override;
  devourer::TxPowerCaps GetTxPowerCaps() override;
  void SetTxPower(uint8_t power) override;
  void SetTxPowerIndexOverride(int idx) override;
  int SetTxPowerOffsetQdb(int qdb) override;
  void SetTxMode(const devourer::TxMode &mode) override;
  void ClearTxMode() override;
  bool SetAmpduMode(const devourer::AmpduMode &mode) override;
  bool GetPermanentMacAddress(uint8_t out[6]) override;
  uint64_t ReadTsf() override;
  void WriteTsf(uint64_t tsf) override;
  devourer::TxStats GetTxStats() override;
  bool SetAckResponder(const devourer::MacAddr &mac) override;
  bool StartBeacon(const uint8_t *beacon, size_t len, int interval_tu) override;
  bool UpdateBeaconPayload(const uint8_t *beacon, size_t len) override;
  bool StopBeacon() override;
  /* Refuse loudly rather than report a 0 us shift that was never applied. */
  int32_t AdjustBeaconTiming(int32_t microseconds) override;
  int32_t AdjustBeaconTimingFine(int32_t microseconds) override;
  int32_t PinBeaconTbtt(int32_t offset_us) override;
  void ClearAckResponder() override;

private:
  void bring_up(SelectedChannel channel);           /* _mu held */
  void apply_config();                              /* _mu held */
  int  txpower_target_dbm() const;                  /* _mu held */
  void start_tick();                                /* _mu held */
  void stop_tick();                                 /* _mu NOT held */
  void tick_loop();
  static void rx_trampoline(void *user, const void *frame, size_t len,
                            const struct mt7612u_rx_info *info);
  void on_rx(const void *frame, size_t len,
             const struct mt7612u_rx_info *info);
  static void log_trampoline(void *user, char level, const char *line);
  /* The C library's diagnostic sink is process-global, so the routing has to
   * be too. See the constructor for why this is a registry and not `this`. */
  static std::mutex &sink_mu();
  static std::vector<Mt7612uRadio *> &sink_registry();

  libusb_device_handle *_handle;
  libusb_context *_ctx;
  /* Held, never used: it keeps the exclusive USB lock WiFiDriver took alive
   * for this object's lifetime. */
  std::shared_ptr<devourer::UsbDeviceLock> _usb_lock;
  Logger_t _logger;
  devourer::DeviceConfig _cfg;

  std::recursive_mutex _mu;
  struct mt7612u_dev *_dev = nullptr;
  SelectedChannel _channel{};
  Action_ParsedRadioPacket _rx_processor;

  /* Serialises the whole RX teardown. StopRxLoop is documented to have torn
   * the ring down and joined the event thread BEFORE it returns, so a second
   * caller has to WAIT for the first rather than see a cleared flag and return
   * early - the early return let Stop() close and free the device out from
   * under a thread still inside mt7612u_rx_stop(). Never held while _mu is
   * held. */
  std::mutex _teardown_mu;
  std::atomic<bool> _rx_stop{false};
  std::atomic<bool> _rx_active{false};
  std::atomic<uint64_t> _rx_frames{0};

  /* Frames cross from the C library's event thread to the StartRxLoop thread
   * here, rather than the processor being invoked where the frame arrives.
   * That is not a style choice - it is what keeps a transmitting processor
   * from wedging the part below the USB level. The queue's own header carries
   * the argument and the two properties (drop the newest and count it; the
   * popped slot outlives the lock) that its selftest holds. */
  mt7612u::RxQueue _rx_q;
  std::atomic<uint64_t> _tx_submitted{0};
  std::atomic<uint64_t> _tx_failed{0};

  /* The tick thread and the gate that lets StopRxLoop / Stop wake it early
   * instead of waiting out a whole second. */
  std::thread _tick;
  std::mutex _tick_mu;
  std::condition_variable _tick_cv;
  bool _tick_stop = false;

  /* StopBeacon/UpdateBeaconPayload are documented to return false when no
   * beacon is active, and the MAC keeps beaconing after the host process dies,
   * so the destructor needs to know too. Guarded by _mu like the rest of the
   * control plane. */
  bool _beacon_active = false;

  int _txpwr_dbm = 20;    /* the absolute dBm limit mt7612u_set_txpower takes */
  int _txpwr_offset_qdb = 0; /* sticky, folded onto _txpwr_dbm */
};

#endif /* MT7612U_RADIO_H */
