#ifndef MT76X0U_RADIO_H
#define MT76X0U_RADIO_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include <libusb.h>

#include "DeviceConfig.h"
#include "IRadio.h"
#include "RxPacket.h"
#include "UsbDeviceLock.h"
#include "logger.h"
#include "mt7612u/Mt7612uRxQueue.h" /* header-only, chip-neutral hand-off queue */
#include "mt76x0u/mt76x0u.h"

/*
 * The MediaTek MT76x0U (MT7610U / MT7630U / MT7650U) behind IRadio.
 *
 * Receive-only by design: monitor capture is the target, so
 * send_packet() refuses and the TX/beacon/station surface falls through to
 * IRadio's not-ported defaults. Derives from IRadio, not IRtlRadio, for the
 * reason Mt7612uRadio gives: there is no Realtek register plane here.
 *
 * Lifecycle, as a monitor daemon drives it: InitWrite() brings the chip up and tunes
 * (no receiver); StartRxLoop() starts the ring, then the MAC, then delivers
 * frames on the calling thread until StopRxLoop(). Init() is both.
 *
 * The rules are Mt7612uRadio's, which were each paid for on hardware:
 *  1. Ring first, receiver second; on the way down, receiver off, then ring.
 *  2. The monitor filter goes on after the MAC start (mt76x0u_start does it).
 *  3. Frames cross from the libusb event thread to the StartRxLoop thread
 *     through a queue (mt7612u::RxQueue); the processor never runs on the
 *     event thread.
 *  4. The ring teardown and the device close join the event thread, so they
 *     run WITHOUT _mu held (a processor may call back in and take it).
 *
 * The calibration tick follows the kernel's cal_work timing: 1 s after a tune,
 * then every 4 s (MT_CALIBRATE_INTERVAL, 4 * MT_CALIBRATE_INTERVAL).
 *
 * NOTHING here has run on an MT7610U yet.
 */
class Mt76x0uRadio : public IRadio {
public:
  Mt76x0uRadio(libusb_device_handle *handle, libusb_context *ctx,
               std::shared_ptr<devourer::UsbDeviceLock> usb_lock,
               Logger_t logger, devourer::DeviceConfig cfg, bool no_2ghz);
  ~Mt76x0uRadio() override;

  void Init(Action_ParsedRadioPacket packetProcessor,
            SelectedChannel channel) override;
  void InitWrite(SelectedChannel channel) override;
  void StartRxLoop(Action_ParsedRadioPacket packetProcessor) override;
  void StopRxLoop() override;
  void SetMonitorChannel(SelectedChannel channel) override;
  bool send_packet(const uint8_t *packet, size_t length) override;
  SelectedChannel GetSelectedChannel() override;
  void SetCcaMode(bool disabled) override;
  void Stop() override;
  devourer::AdapterCaps GetAdapterCaps() override;
  bool GetPermanentMacAddress(uint8_t out[6]) override;

private:
  void bring_up(SelectedChannel channel); /* _mu held */
  void tune(SelectedChannel channel);     /* _mu held, _dev open */
  void start_rx_loop(Action_ParsedRadioPacket packetProcessor, bool clear_stale);
  void start_tick();                      /* _mu NOT held */
  void stop_tick();                       /* _mu NOT held */
  void tick_loop();
  static void rx_trampoline(void *user, const void *frame, size_t len,
                            const struct mt76x0u_rx_info *info);
  void on_rx(const void *frame, size_t len, const struct mt76x0u_rx_info *info);

  libusb_device_handle *_handle;
  libusb_context *_ctx;
  /* Held, never used: keeps WiFiDriver's exclusive USB lock alive. */
  std::shared_ptr<devourer::UsbDeviceLock> _usb_lock;
  Logger_t _logger;
  devourer::DeviceConfig _cfg;
  bool _no_2ghz;

  std::recursive_mutex _mu;
  struct mt76x0u_dev *_dev = nullptr;
  SelectedChannel _channel{};
  bool _has_2g = false, _has_5g = false;
  Action_ParsedRadioPacket _rx_processor;

  /* Serialises the RX teardown (Mt7612uRadio's _teardown_mu, same reason). */
  std::mutex _teardown_mu;
  std::atomic<bool> _rx_stop{false};
  std::atomic<bool> _rx_active{false};
  std::atomic<uint64_t> _rx_frames{0};
  mt7612u::RxQueue _rx_q;

  std::thread _tick;
  std::mutex _tick_mu;
  std::condition_variable _tick_cv;
  bool _tick_stop = false;
  /* When the next calibration pass is due; a tune moves it to now + 1 s.
   * Under _mu. */
  std::chrono::steady_clock::time_point _cal_due{};
};

#endif /* MT76X0U_RADIO_H */
