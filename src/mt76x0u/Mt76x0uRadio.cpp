#include "mt76x0u/Mt76x0uRadio.h"

#include <cstdio>
#include <exception>
#include <filesystem>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "SignalStop.h"
#include "mt76x0u/Mt76x0uMapping.h"

namespace {

/*
 * Where the firmware lives, resolved as Mt7612uRadio does (and for the same
 * reason: linux-firmware's licence and zstd compression mean it can be neither
 * vendored nor assumed at a fixed path). A directory qualifies if it holds
 * either file, since the library falls back from mt7610e.bin to mt7610u.bin
 * the way the kernel does (some distributions ship only mt7610u.bin).
 */
std::string resolve_fw_dir(const devourer::DeviceConfig &cfg,
                           const Logger_t &logger) {
  std::vector<std::string> tried;
  if (cfg.mt76x0u.firmware_dir)
    tried.emplace_back(*cfg.mt76x0u.firmware_dir);
  tried.emplace_back("/lib/firmware/mediatek");
  tried.emplace_back("firmware");

  std::error_code ec;
  for (const std::string &dir : tried) {
    for (const char *name : {"mt7610e.bin", "mt7610u.bin"}) {
      if (std::filesystem::exists(std::filesystem::path(dir) / name, ec)) {
        logger->info("MT7610U firmware directory {} (has {})", dir, name);
        return dir;
      }
    }
  }
  std::string all;
  for (const std::string &dir : tried)
    all += (all.empty() ? "" : ", ") + dir;
  logger->error("MT7610U firmware (mt7610e.bin or mt7610u.bin) not found in: "
                "{}. They ship zstd-compressed in linux-firmware; decompress "
                "one and set DeviceConfig mt76x0u.firmware_dir (demos: "
                "DEVOURER_MT76X0U_FW_DIR).",
                all);
  return {};
}

} // namespace

Mt76x0uRadio::Mt76x0uRadio(libusb_device_handle *handle, libusb_context *ctx,
                           std::shared_ptr<devourer::UsbDeviceLock> usb_lock,
                           Logger_t logger, devourer::DeviceConfig cfg,
                           bool no_2ghz)
    : _handle(handle), _ctx(ctx), _usb_lock(std::move(usb_lock)),
      _logger(std::move(logger)), _cfg(std::move(cfg)), _no_2ghz(no_2ghz) {}

Mt76x0uRadio::~Mt76x0uRadio() {
  /* Nothing may escape a destructor. */
  try {
    Stop();
  } catch (...) {
  }
}

/* --- bring-up and tuning (_mu held) ---------------------------------------- */

void Mt76x0uRadio::bring_up(SelectedChannel channel) {
  if (_dev) {
    /* Already up (InitWrite then Init, or a second InitWrite): just tune. */
    tune(channel);
    return;
  }
  const char *err = nullptr;
  _dev = mt76x0u_open_handle(_handle, _ctx, _no_2ghz ? 1 : 0, &err);
  if (!_dev)
    throw std::runtime_error(std::string("MT7610U open failed: ") +
                             (err ? err : "unknown error"));
  char rev[16];
  std::snprintf(rev, sizeof rev, "%08x",
                static_cast<unsigned>(mt76x0u_asic_version(_dev)));

  const std::string fw_dir = resolve_fw_dir(_cfg, _logger);
  if (fw_dir.empty())
    throw std::runtime_error("MT7610U firmware not found");

  /* Power-on, firmware, EFUSE, MAC/BBP/RF init. MAC RX stays off. */
  if (mt76x0u_bring_up(_dev, fw_dir.c_str(), _cfg.mt76x0u.force_fw_upload ? 1 : 0))
    throw std::runtime_error(std::string("MT7610U bring-up failed (ASIC ") + rev +
                             "); the log above names the step");
  mt76x0u_set_tune_twice(_dev, _cfg.mt76x0u.tune_twice ? 1 : 0);
  int has_2g = 0, has_5g = 0;
  mt76x0u_bands(_dev, &has_2g, &has_5g);
  _has_2g = has_2g != 0;
  _has_5g = has_5g != 0;
  const uint8_t *mac = mt76x0u_mac_addr(_dev);
  char macs[18] = "?";
  if (mac)
    std::snprintf(macs, sizeof macs, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0],
                  mac[1], mac[2], mac[3], mac[4], mac[5]);
  _logger->info("MT7610U up: ASIC {} MAC {} bands{}{}", rev, macs,
                _has_2g ? " 2.4GHz" : "", _has_5g ? " 5GHz" : "");
  tune(channel);
}

/* SelectedChannel -> (primary, ht40). ChannelOffset follows devourer's
 * Realtek convention: HAL_PRIME_CHNL_OFFSET_LOWER (1) = the primary is the
 * lower 20 MHz, secondary above = HT40+; UPPER (2) = HT40-. A 40 MHz request
 * with offset 0 picks the side as iw does for a bare "40". */
void Mt76x0uRadio::tune(SelectedChannel channel) {
  int ht40 = 0;
  if (channel.ChannelWidth == CHANNEL_WIDTH_40) {
    if (channel.ChannelOffset == 1)
      ht40 = +1;
    else if (channel.ChannelOffset == 2)
      ht40 = -1;
    else if (channel.Channel <= 14)
      ht40 = channel.Channel + 4 <= 13 ? +1 : -1;
    else /* 5 GHz pairs start at 36, 44, ... and at 149, 157, 165 */
      ht40 = (((channel.Channel < 149 ? channel.Channel - 36
                                      : channel.Channel - 149) / 4) % 2 == 0)
                 ? +1
                 : -1;
  } else if (channel.ChannelWidth != CHANNEL_WIDTH_20) {
    throw std::runtime_error("MT7610U channel width refused: the backend "
                             "serves 20 and 40 MHz");
  }
  if (mt76x0u_set_channel(_dev, channel.Channel, ht40))
    throw std::runtime_error("MT7610U channel set failed (channel " +
                             std::to_string(channel.Channel) + ")");
  _channel = channel;
  /* mt76x0_phy_set_channel queues cal_work MT_CALIBRATE_INTERVAL out. */
  _cal_due = std::chrono::steady_clock::now() + std::chrono::seconds(1);
}

/* --- calibration tick ------------------------------------------------------ */

void Mt76x0uRadio::start_tick() {
  if (_tick.joinable() || !_cfg.mt76x0u.cal_tick)
    return;
  {
    std::lock_guard<std::mutex> lock(_tick_mu);
    _tick_stop = false;
  }
  _tick = std::thread(&Mt76x0uRadio::tick_loop, this);
}

void Mt76x0uRadio::stop_tick() {
  if (!_tick.joinable())
    return;
  {
    std::lock_guard<std::mutex> lock(_tick_mu);
    _tick_stop = true;
  }
  _tick_cv.notify_all();
  _tick.join();
}

void Mt76x0uRadio::tick_loop() {
  for (;;) {
    {
      std::unique_lock<std::mutex> lock(_tick_mu);
      _tick_cv.wait_for(lock, std::chrono::milliseconds(250),
                        [this] { return _tick_stop; });
      if (_tick_stop)
        return;
    }
    std::lock_guard<std::recursive_mutex> lock(_mu);
    const auto now = std::chrono::steady_clock::now();
    if (!_dev || now < _cal_due)
      continue;
    mt76x0u_cal_tick(_dev);
    /* mt76x0_phy_calibration_work requeues itself 4 * MT_CALIBRATE_INTERVAL. */
    _cal_due = now + std::chrono::seconds(4);
  }
}

/* --- IRadio ---------------------------------------------------------------- */

void Mt76x0uRadio::InitWrite(SelectedChannel channel) {
  /* Stop() on failure, OUTSIDE _mu: it joins the tick, which takes _mu. */
  std::exception_ptr failed;
  {
    std::lock_guard<std::recursive_mutex> lock(_mu);
    try {
      bring_up(channel);
    } catch (...) {
      failed = std::current_exception();
    }
  }
  if (failed) {
    Stop();
    std::rethrow_exception(failed);
  }
  start_tick();
}

void Mt76x0uRadio::Init(Action_ParsedRadioPacket packetProcessor,
                        SelectedChannel channel) {
  _rx_stop = false;
  InitWrite(channel);
  try {
    start_rx_loop(std::move(packetProcessor), false);
  } catch (...) {
    Stop();
    throw;
  }
}

void Mt76x0uRadio::StartRxLoop(Action_ParsedRadioPacket packetProcessor) {
  start_rx_loop(std::move(packetProcessor), true);
}

void Mt76x0uRadio::start_rx_loop(Action_ParsedRadioPacket packetProcessor,
                                 bool clear_stale) {
  if (g_devourer_should_stop)
    return;

  struct mt76x0u_dev *mac_failed = nullptr;
  {
    /* Ordering _teardown_mu -> _mu, as in StopRxLoop (Mt7612uRadio). */
    std::lock_guard<std::mutex> teardown(_teardown_mu);
    std::lock_guard<std::recursive_mutex> lock(_mu);
    if (!_dev)
      throw std::runtime_error("MT7610U RX loop requires initialized hardware");
    if (_rx_active.load())
      throw std::runtime_error("MT7610U RX loop is already active");
    _rx_processor = std::move(packetProcessor);
    if (clear_stale)
      _rx_stop = false;
    else if (_rx_stop.load())
      return;
    _rx_q.reset(64);
    _rx_frames.store(0, std::memory_order_relaxed);

    /* Rule 1: ring first, receiver second. */
    if (mt76x0u_rx_start(_dev, &Mt76x0uRadio::rx_trampoline, this) != 0)
      throw std::runtime_error("MT7610U RX ring failed to start");
    if (mt76x0u_start(_dev, _cfg.mt76x0u.monitor_control ? 1 : 0) != 0) {
      mt76x0u_rx_quiesce(_dev);
      mac_failed = _dev;
    } else {
      _rx_active.store(true, std::memory_order_release);
    }
  }
  if (mac_failed) {
    /* Rule 4: the ring teardown joins the event thread - not under _mu. */
    mt76x0u_rx_stop(mac_failed);
    _rx_processor = nullptr;
    throw std::runtime_error("MT7610U MAC start failed");
  }
  _logger->info("MT7610U RX on channel {}", _channel.Channel);

  /* Rule 3: the consumer, on this thread. Returns, as the IRadio contract
   * expects, when the ring dies: the adapter was unplugged (NO_DEVICE), or
   * every transfer failed to resubmit - nothing more can arrive. */
  auto idle_since = std::chrono::steady_clock::now();
  for (;;) {
    if (_rx_stop.load() || g_devourer_should_stop)
      break;
    mt7612u::RxQueue::Slot *slot =
        _rx_q.pop_begin(std::chrono::milliseconds(20), _rx_stop);
    if (!slot) {
      const auto now = std::chrono::steady_clock::now();
      if (now - idle_since >= std::chrono::milliseconds(200)) {
        idle_since = now;
        /* _teardown_mu (not _mu, which a tune holds for a second): Stop()
         * nulls and closes _dev under it. Skip the check if it is busy. */
        struct mt76x0u_stats st{};
        {
          std::unique_lock<std::mutex> td(_teardown_mu, std::try_to_lock);
          if (td.owns_lock() && _dev)
            mt76x0u_get_stats(_dev, &st);
        }
        if (st.rx_no_device || st.rx_ring_dead) {
          _logger->error("MT7610U RX ring stopped ({}, {} failed transfers) - "
                         "adapter lost; ending the RX loop",
                         st.rx_no_device ? "device gone" : "no transfer in flight",
                         st.rx_err);
          break;
        }
      }
      continue;
    }
    idle_since = std::chrono::steady_clock::now();
    Packet packet{};
    packet.RxAtrib = slot->attrib;
    packet.Data = std::span<uint8_t>(slot->data.data(), slot->data.size());
    if (_rx_processor)
      _rx_processor(packet);
    _rx_q.pop_commit(slot);
  }
  StopRxLoop();
}

void Mt76x0uRadio::StopRxLoop() {
  _rx_stop = true;
  _rx_q.wake();

  std::lock_guard<std::mutex> teardown(_teardown_mu);
  if (!_rx_active.load(std::memory_order_acquire))
    return;
  struct mt76x0u_dev *dev = nullptr;
  {
    std::lock_guard<std::recursive_mutex> lock(_mu);
    dev = _dev;
    if (dev)
      mt76x0u_rx_quiesce(dev); /* rule 1, the mirror image */
  }
  if (dev)
    mt76x0u_rx_stop(dev); /* rule 4 */
  _rx_active.store(false, std::memory_order_release);

  const uint64_t dropped = _rx_q.dropped();
  if (dropped)
    _logger->warn("MT7610U RX stopped after {} frames received, of which {} "
                  "DROPPED at the hand-off queue - the packet processor could "
                  "not keep up",
                  _rx_frames.load(std::memory_order_relaxed), dropped);
  else
    _logger->info("MT7610U RX stopped after {} frames",
                  _rx_frames.load(std::memory_order_relaxed));
}

void Mt76x0uRadio::rx_trampoline(void *user, const void *frame, size_t len,
                                 const struct mt76x0u_rx_info *info) {
  static_cast<Mt76x0uRadio *>(user)->on_rx(frame, len, info);
}

/* On the libusb event thread: no _mu, no blocking. */
void Mt76x0uRadio::on_rx(const void *frame, size_t len,
                         const struct mt76x0u_rx_info *info) {
  rx_pkt_attrib attrib{};
  const auto *f = static_cast<const uint8_t *>(frame);
  mt76x0u::fill_attrib(*info, f, len, attrib);
  _rx_frames.fetch_add(1, std::memory_order_relaxed);
  _rx_q.push(attrib, f, len);
}

void Mt76x0uRadio::SetMonitorChannel(SelectedChannel channel) {
  std::lock_guard<std::recursive_mutex> lock(_mu);
  if (!_dev)
    throw std::runtime_error("MT7610U channel set requires initialized hardware");
  tune(channel);
}

bool Mt76x0uRadio::send_packet(const uint8_t * /*packet*/, size_t /*length*/) {
  return false; /* receive-only backend */
}

SelectedChannel Mt76x0uRadio::GetSelectedChannel() {
  std::lock_guard<std::recursive_mutex> lock(_mu);
  return _channel;
}

void Mt76x0uRadio::SetCcaMode(bool /*disabled*/) {
  /* CCA gates transmission; this backend does not transmit. */
}

void Mt76x0uRadio::Stop() {
  try {
    StopRxLoop();
  } catch (...) {
  }
  stop_tick(); /* joins; must not run with _mu held */

  /* Take the device out under the locks, close it outside (rule 4). */
  struct mt76x0u_dev *dev = nullptr;
  {
    std::lock_guard<std::mutex> teardown(_teardown_mu);
    std::lock_guard<std::recursive_mutex> lock(_mu);
    dev = _dev;
    _dev = nullptr;
    if (dev)
      mt76x0u_stop(dev);
  }
  if (dev)
    mt76x0u_close(dev);
}

devourer::AdapterCaps Mt76x0uRadio::GetAdapterCaps() {
  std::lock_guard<std::recursive_mutex> lock(_mu);
  const bool known = _dev != nullptr;
  devourer::AdapterCaps c{};
  c.supported = true;
  c.chip_name = "MT7610U";
  c.marketing_names = "MT7610U/MT7630U/MT7650U";
  c.chip_id = 0; /* no SYS_CFG2 equivalent - dispatch is VID:PID */
  c.generation = devourer::ChipGeneration::Mt76x0u;
  c.variant = "MT76x0U";
  c.transport = "usb";
  c.tx_chains = 0; /* receive-only backend */
  c.rx_chains = 1; /* 1T1R family */
  c.bw_mask = devourer::bw_mask_for_generation(c.generation);
  /* Before bring-up only the USB-id quirk is known; after it, the EFUSE
   * board type and the MT7630 strap too. */
  const bool g = known ? _has_2g : !_no_2ghz;
  const bool a = known ? _has_5g : true;
  c.tune_2g4 = {g, uint16_t(g ? 2412 : 0), uint16_t(g ? 2484 : 0)};
  c.tune_5g = {a, uint16_t(a ? 5180 : 0), uint16_t(a ? 5865 : 0)};
  return c;
}

bool Mt76x0uRadio::GetPermanentMacAddress(uint8_t out[6]) {
  std::lock_guard<std::recursive_mutex> lock(_mu);
  const uint8_t *mac = _dev ? mt76x0u_mac_addr(_dev) : nullptr;
  if (!mac)
    return false;
  for (int i = 0; i < 6; ++i)
    out[i] = mac[i];
  return true;
}
