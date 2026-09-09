#include "mt7612u/Mt7612uRadio.h"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <span>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include "mt7612u/Mt7612uMapping.h"

extern volatile bool g_devourer_should_stop;

namespace {

/*
 * Where mt7662_rom_patch.bin and mt7662.bin live.
 *
 * Unlike the Realtek backends, whose firmware is generated into hal/ and
 * compiled in, MediaTek's ships in linux-firmware under its own licence and is
 * zstd-compressed on most distributions - so it can be neither vendored here
 * nor assumed ready at a fixed path. Resolved in the order a caller would
 * expect, and the failure names every place that was tried rather than just
 * saying no.
 */
std::string resolve_fw_dir(const devourer::DeviceConfig &cfg,
                           const Logger_t &logger) {
  std::vector<std::string> tried;

  if (cfg.mt7612u.firmware_dir)
    tried.emplace_back(*cfg.mt7612u.firmware_dir);
  tried.emplace_back("/lib/firmware/mediatek");
  tried.emplace_back("firmware"); /* the bring-up harness's own directory */

  std::error_code ec;
  for (const std::string &dir : tried) {
    const std::filesystem::path patch =
        std::filesystem::path(dir) / "mt7662_rom_patch.bin";
    const std::filesystem::path fw = std::filesystem::path(dir) / "mt7662.bin";
    if (std::filesystem::exists(patch, ec) &&
        std::filesystem::exists(fw, ec)) {
      logger->info("MT7612U firmware from {}", dir);
      return dir;
    }
  }

  std::string all;
  for (const std::string &dir : tried)
    all += (all.empty() ? "" : ", ") + dir;
  logger->error("MT7612U firmware (mt7662_rom_patch.bin + mt7662.bin) not "
                "found in: {}. They ship zstd-compressed in linux-firmware; "
                "decompress them and set DeviceConfig mt7612u.firmware_dir "
                "(demos: DEVOURER_MT7612U_FW_DIR).",
                all);
  return tried.back();
}

} // namespace

Mt7612uRadio::Mt7612uRadio(libusb_device_handle *handle, libusb_context *ctx,
                           std::shared_ptr<devourer::UsbDeviceLock> usb_lock,
                           Logger_t logger, devourer::DeviceConfig cfg)
    : _handle(handle), _ctx(ctx), _usb_lock(std::move(usb_lock)),
      _logger(std::move(logger)), _cfg(std::move(cfg)) {
  /* Before any library call, and before any thread the library might start:
   * the sink pointer is read from the RX event thread without
   * synchronisation, which is the contract mt7612u_set_log_sink documents.
   * Without this the subtree's diagnostics go straight to stderr, bypassing
   * the log level, a redirected diag stream, and __android_log_write. */
  mt7612u_set_log_sink(&Mt7612uRadio::log_trampoline, this);
}

Mt7612uRadio::~Mt7612uRadio() {
  Stop();
  /* Nothing may reach a destroyed `this` afterwards. */
  mt7612u_set_log_sink(nullptr, nullptr);
}

void Mt7612uRadio::log_trampoline(void *user, char level, const char *line) {
  auto *self = static_cast<Mt7612uRadio *>(user);
  if (!self || !self->_logger || !line)
    return;
  /* The library hands over the bare message; Logger re-adds "devourer [X] "
   * and applies the level gating and stream this consumer configured. */
  switch (level) {
  case 'E':
    self->_logger->error("mt7612u: {}", line);
    break;
  case 'W':
    self->_logger->warn("mt7612u: {}", line);
    break;
  default:
    self->_logger->info("mt7612u: {}", line);
    break;
  }
}

void Mt7612uRadio::bring_up(SelectedChannel channel) {
  enum mt7612u_bw bw = MT7612U_BW_20;
  const char *why = "";

  if (!mt7612u::width_to_bw(channel.ChannelWidth, bw, why))
    throw std::runtime_error(std::string("MT7612U channel width refused: ") +
                             why);

  if (!_dev) {
    const char *err = nullptr;
    /* Adopts the caller's handle: WiFiDriver already opened, reset and claimed
     * it, and holds the exclusive lock this object carries. Reopening would
     * race that lock, and libusb_reset_device() here would invalidate the
     * caller's own handle. */
    const std::string fw_dir = resolve_fw_dir(_cfg, _logger);
    _dev = mt7612u_open_handle(_handle, _ctx, fw_dir.c_str(), &err);
    if (!_dev)
      throw std::runtime_error(std::string("MT7612U bring-up failed: ") +
                               (err ? err : "unknown"));
    _logger->info("MT7612U up: ASIC 0x{:08x}", mt7612u_asic_version(_dev));
    if (_txpwr_dbm != 20)
      mt7612u_set_txpower(_dev, _txpwr_dbm);
  }

  if (mt7612u_set_channel(_dev, channel.Channel, bw) != 0)
    throw std::runtime_error("MT7612U channel set failed");
  _channel = channel;
  start_tick();
}

/* --- the 1 Hz PHY tick ---------------------------------------------------
 *
 * One round of mt76's cal_work. Not optional and not cosmetic: without it a
 * receiver 20 cm from a peer airing 3037 fps takes 3 frames in 10 s; with it,
 * 5415-5470/s.
 *
 * On a thread this class owns rather than inside StartRxLoop, because a
 * transmit-only consumer (InitWrite, no RX loop) needs it too - TSSI
 * temperature compensation rides the same tick. It takes _mu because the tick
 * issues MCU commands, which share one 4-bit sequence number and one response
 * endpoint with everything else here; running one alongside a channel change's
 * calibration burst is how those get crossed. */
void Mt7612uRadio::start_tick() {
  if (_tick.joinable())
    return;
  {
    std::lock_guard<std::mutex> lock(_tick_mu);
    _tick_stop = false;
  }
  _tick = std::thread(&Mt7612uRadio::tick_loop, this);
}

void Mt7612uRadio::stop_tick() {
  if (!_tick.joinable())
    return;
  {
    std::lock_guard<std::mutex> lock(_tick_mu);
    _tick_stop = true;
  }
  _tick_cv.notify_all();
  _tick.join();
}

void Mt7612uRadio::tick_loop() {
  for (;;) {
    {
      std::unique_lock<std::mutex> lock(_tick_mu);
      /* Waits out the interval, but wakes immediately on teardown so a stop
       * never has to sit through a whole second. */
      _tick_cv.wait_for(lock, std::chrono::seconds(1),
                        [this] { return _tick_stop; });
      if (_tick_stop)
        return;
    }
    std::lock_guard<std::recursive_mutex> lock(_mu);
    if (!_dev)
      continue;
    /* Returns -1 before a channel is set, which is not an error worth a line
     * every second - bring_up() sets one before this thread can do anything. */
    mt7612u_phy_tick(_dev);
  }
}

void Mt7612uRadio::Init(Action_ParsedRadioPacket packetProcessor,
                        SelectedChannel channel) {
  {
    std::lock_guard<std::recursive_mutex> lock(_mu);
    bring_up(channel);
  }
  /* Deliberately outside the lock: StartRxLoop blocks until StopRxLoop. */
  StartRxLoop(std::move(packetProcessor));
}

void Mt7612uRadio::InitWrite(SelectedChannel channel) {
  std::lock_guard<std::recursive_mutex> lock(_mu);
  bring_up(channel);
  /* No RX ring, so mt7612u_start() enables TX only - which is the point of
   * this entry point, and also what keeps the chip out of the undrained-
   * receiver wedge. */
  if (mt7612u_start(_dev) != 0)
    throw std::runtime_error("MT7612U MAC start failed");
}

void Mt7612uRadio::StartRxLoop(Action_ParsedRadioPacket packetProcessor) {
  {
    std::lock_guard<std::recursive_mutex> lock(_mu);
    if (!_dev)
      throw std::runtime_error("MT7612U RX loop requires initialized hardware");
    if (_rx_active.load())
      throw std::runtime_error("MT7612U RX loop is already active");
    _rx_processor = std::move(packetProcessor);
    _rx_stop = false;

    /* Ring first, receiver second - see rule 1 in the header. */
    if (mt7612u_rx_start(_dev, &Mt7612uRadio::rx_trampoline, this) != 0)
      throw std::runtime_error("MT7612U RX ring failed to start");
    if (mt7612u_start(_dev) != 0) {
      mt7612u_rx_stop(_dev);
      throw std::runtime_error("MT7612U MAC start failed");
    }
    /* AFTER mt7612u_start(), which rewrites the filter to mt76's managed-mode
     * value - see rule 2. Before it, this write is simply overwritten. */
    if (mt7612u_set_monitor_rx(_dev, _cfg.rx.keep_corrupted ? 1 : 0) != 0)
      _logger->warn("MT7612U monitor RX filter not applied");
    /* Arms the channel timers and zeroes the MIB counters. */
    mt7612u_link_stats_start(_dev);
    _rx_active = true;
  }

  _logger->info("MT7612U monitor RX on channel {}", _channel.Channel);

  /* The C layer drives RX from its own libusb event thread, so this loop has
   * nothing to poll - it exists to give StartRxLoop the blocking contract
   * every other backend has, and to notice Stop() and SIGINT.
   *
   * Delivery therefore happens on the event thread rather than on this one,
   * which differs from the Realtek backends. The guarantee that matters is
   * preserved: StopRxLoop tears the ring down and joins that thread before
   * returning, so no callback can arrive after StartRxLoop returns. */
  while (!_rx_stop.load() && !g_devourer_should_stop)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

  StopRxLoop();
}

void Mt7612uRadio::StopRxLoop() {
  _rx_stop = true;
  if (!_rx_active.exchange(false))
    return;

  /* Quiesce under the lock: it is a register write, and it must not interleave
   * with the tick's MCU traffic. */
  {
    std::lock_guard<std::recursive_mutex> lock(_mu);
    if (_dev)
      mt7612u_rx_quiesce(_dev);
  }
  /* Ring teardown WITHOUT the lock. mt7612u_rx_stop() joins the event thread,
   * and a processor still in flight on that thread may call back into this
   * object - SetMonitorChannel, say - which takes _mu. Holding _mu across the
   * join is therefore a deadlock, not a theoretical one. */
  if (_dev)
    mt7612u_rx_stop(_dev);

  _logger->info("MT7612U RX stopped after {} frames",
                _rx_frames.load(std::memory_order_relaxed));
}

void Mt7612uRadio::rx_trampoline(void *user, const void *frame, size_t len,
                                 const struct mt7612u_rx_info *info) {
  static_cast<Mt7612uRadio *>(user)->on_rx(frame, len, info);
}

void Mt7612uRadio::on_rx(const void *frame, size_t len,
                         const struct mt7612u_rx_info *info) {
  if (!_rx_processor)
    return;

  Packet packet{};
  packet.RxAtrib.pkt_len = static_cast<uint16_t>(len);
  packet.RxAtrib.crc_err = info->crc_err != 0;
  packet.RxAtrib.seq_num = info->seq;
  packet.RxAtrib.data_rate = mt7612u::desc_rate(*info);
  packet.RxAtrib.bw = mt7612u::bw_to_desc(info->bw);
  packet.RxAtrib.stbc = info->stbc;
  packet.RxAtrib.ldpc = info->ldpc;
  packet.RxAtrib.sgi = info->sgi;
  packet.RxAtrib.paggr = info->ampdu != 0;
  packet.RxAtrib.pkt_rpt_type = RX_PACKET_TYPE::NORMAL_RX;
  /* THE MediaTek divergence, and the reason rx_pkt_attrib carries this flag at
   * all: this MAC strips the FCS. The four bytes after the MPDU in the DMA
   * buffer are the FCE info trailer, not a checksum - CRC-32 matched them on 0
   * of 4263 measured frames. A consumer that trims four bytes here would
   * delete real payload. */
  packet.RxAtrib.fcs_present = false;
  /* Per-chain RSSI and SNR, from n_chains rather than the array's extent:
   * rssi[2] is the noise floor and rssi[3] is unidentified. */
  mt7612u::copy_signal(*info, packet.RxAtrib);

  if (len >= 2) {
    const uint8_t *f = static_cast<const uint8_t *>(frame);
    packet.RxAtrib.qos = (f[0] & 0x0c) == 0x08 && (f[0] & 0x80) != 0;
    /* The TID lives in the first QoS Control byte, which follows the 24-byte
     * base header. Set alongside qos rather than left zero, so a consumer
     * cannot read "QoS frame, TID 0" for every frame. */
    if (packet.RxAtrib.qos && len >= 26)
      packet.RxAtrib.priority = f[24] & 0x0f;
  }

  /* The span points into the ring buffer the libusb event thread owns and
   * reuses the moment this returns, so the processor must not retain it - the
   * same contract every other backend's parser has. const_cast because
   * Packet::Data is a mutable span and the buffer genuinely is ours. */
  packet.Data = std::span<uint8_t>(
      const_cast<uint8_t *>(static_cast<const uint8_t *>(frame)), len);

  _rx_frames.fetch_add(1, std::memory_order_relaxed);
  _rx_processor(packet);
}

void Mt7612uRadio::SetMonitorChannel(SelectedChannel channel) {
  std::lock_guard<std::recursive_mutex> lock(_mu);
  enum mt7612u_bw bw = MT7612U_BW_20;
  const char *why = "";

  if (!_dev) {
    _channel = channel; /* remembered until bring-up */
    return;
  }
  if (!mt7612u::width_to_bw(channel.ChannelWidth, bw, why)) {
    _logger->error("MT7612U channel width refused: {}", why);
    return;
  }
  if (mt7612u_set_channel(_dev, channel.Channel, bw) != 0) {
    _logger->error("MT7612U channel set to {} failed", channel.Channel);
    return;
  }
  /* Only on success, so GetSelectedChannel never reports a channel the
   * hardware did not reach. */
  _channel = channel;
}

SelectedChannel Mt7612uRadio::GetSelectedChannel() {
  std::lock_guard<std::recursive_mutex> lock(_mu);
  return _channel;
}

/* Both send paths take _mu because the TX width clamp in the library reads the
 * tuned channel and width, which SetMonitorChannel writes under this same
 * lock. Without it a retune concurrent with a burst can have a frame read a
 * half-updated width. */
bool Mt7612uRadio::send_packet(const uint8_t *packet, size_t length) {
  std::lock_guard<std::recursive_mutex> lock(_mu);
  if (!_dev)
    return false;
  return mt7612u_send_packet(_dev, packet, length) == 0;
}

size_t Mt7612uRadio::send_packets(const TxPacketView *pkts, size_t count) {
  std::lock_guard<std::recursive_mutex> lock(_mu);
  if (!_dev || !pkts)
    return 0;
  /* TxPacketView and mt7612u_tx_view are the same two fields in the same
   * order, but a reinterpret_cast across a language boundary is the kind of
   * thing that breaks silently when one side gains a member. Copy. */
  std::vector<struct mt7612u_tx_view> views(count);
  for (size_t i = 0; i < count; ++i) {
    views[i].data = pkts[i].data;
    views[i].len = pkts[i].len;
  }
  return mt7612u_send_packets(_dev, views.data(), count);
}

void Mt7612uRadio::SetCcaMode(bool disabled) {
  /* Refuses rather than no-ops. MT7612U does have an ED-CCA enable
   * (MT_TXOP_CTRL_CFG / MT_TXOP_ED_CCA_EN, which mac_stop already clears), but
   * "disable CCA" on the Realtek backends means a specific, measured set of
   * writes, and nothing here has been measured against an on-air carrier-sense
   * test. Claiming it on the strength of one plausible-looking bit is how an
   * unverified regulatory-adjacent behaviour ships. */
  _logger->error("MT7612U: SetCcaMode({}) not implemented - the ED-CCA enable "
                 "exists but no on-air carrier-sense measurement backs it",
                 disabled);
}

void Mt7612uRadio::Stop() {
  StopRxLoop();
  stop_tick(); /* joins; must not run with _mu held */
  std::lock_guard<std::recursive_mutex> lock(_mu);
  if (_dev) {
    mt7612u_stop(_dev);
    mt7612u_close(_dev);
    _dev = nullptr;
  }
}

void Mt7612uRadio::SetTxPower(uint8_t power) {
  std::lock_guard<std::recursive_mutex> lock(_mu);
  /* Deliberately NOT forwarded to SetTxPowerIndexOverride the way the base
   * class does: there is no TXAGC index here, so the argument is read as the
   * dBm limit it actually maps to. */
  _txpwr_dbm = static_cast<int>(power);
  if (_dev && mt7612u_set_txpower(_dev, _txpwr_dbm) != 0)
    _logger->error("MT7612U TX power {} dBm refused (valid range 0-30)",
                   _txpwr_dbm);
}

void Mt7612uRadio::SetTxPowerIndexOverride(int idx) {
  _logger->error("MT7612U has no TXAGC index to override (asked for {}); TX "
                 "power here is an absolute dBm limit - use SetTxPower()",
                 idx);
}

bool Mt7612uRadio::GetPermanentMacAddress(uint8_t out[6]) {
  std::lock_guard<std::recursive_mutex> lock(_mu);
  if (!_dev)
    return false;
  const uint8_t *mac = mt7612u_mac_addr(_dev);
  if (!mac)
    return false;
  for (int i = 0; i < 6; ++i)
    out[i] = mac[i];
  return true;
}

uint64_t Mt7612uRadio::ReadTsf() {
  std::lock_guard<std::recursive_mutex> lock(_mu);
  return _dev ? mt7612u_read_tsf(_dev) : 0;
}

void Mt7612uRadio::WriteTsf(uint64_t tsf) {
  std::lock_guard<std::recursive_mutex> lock(_mu);
  if (_dev)
    mt7612u_write_tsf(_dev, tsf);
}

bool Mt7612uRadio::SetAckResponder(const devourer::MacAddr &mac) {
  std::lock_guard<std::recursive_mutex> lock(_mu);
  if (!_dev)
    return false;
  return mt7612u_set_ack_responder(_dev, mac.data()) == 0;
}

void Mt7612uRadio::ClearAckResponder() {
  std::lock_guard<std::recursive_mutex> lock(_mu);
  if (_dev)
    mt7612u_clear_ack_responder(_dev);
}

devourer::TxCaps Mt7612uRadio::GetTxCaps() {
  devourer::TxCaps c{};
  c.supported = true;
  c.n_ss = 2;
  c.stbc_ok = true;
  c.ldpc_ok = true;
  c.sgi_ok = true;
  c.bw_max_mhz = 80;
  return c;
}

devourer::TxPowerCaps Mt7612uRadio::GetTxPowerCaps() {
  devourer::TxPowerCaps c{};
  c.supported = true;
  /* index_max 0 = the dBm model, per the field's own contract. There is no
   * TXAGC index on this part: TX power is an absolute dBm limit feeding the
   * per-rate table, plus a 4-bit per-frame trim in the descriptor. */
  c.index_max = 0;
  c.step_qdb = 2; /* the limit is carried in 0.5 dB units */
  c.step_measured = false;
  c.offset_min_qdb = -80; /* down to 0 dBm from the 20 dBm default */
  c.offset_max_qdb = 40;  /* up to 30 dBm, the API's own ceiling */
  c.rate_diffs = false;
  return c;
}

devourer::AdapterCaps Mt7612uRadio::GetAdapterCaps() {
  struct mt7612u_caps hw {};
  bool have_hw = false;
  {
    std::lock_guard<std::recursive_mutex> lock(_mu);
    if (_dev) {
      mt7612u_get_caps(_dev, &hw);
      have_hw = true;
    }
  }

  devourer::AdapterCaps c{};
  c.supported = true;
  c.chip_name = have_hw && hw.chip_name ? hw.chip_name : "MT7612U";
  c.marketing_names = "MT7612U/MT7662U";
  c.chip_id = 0; /* no SYS_CFG2 equivalent - dispatch is VID:PID */
  c.generation = devourer::ChipGeneration::Mt7612u;
  c.variant = "MT7612U";
  c.transport = "usb";
  /* Read from the library rather than restated as literals here: it derives
   * them from the EEPROM and the register programming, and a second copy is a
   * second thing to drift. Falls back only when the device is not open. */
  c.tx_chains = have_hw ? hw.nss_tx : 2;
  c.rx_chains = have_hw ? hw.nss_rx : 2;
  c.tx = GetTxCaps();
  c.txpwr = GetTxPowerCaps();
  /* 5/10 MHz stay out: MT_RATE_BW has no encoding for them. Must agree with
   * GetTxCaps().bw_max_mhz - the two travel together in one adapter.caps
   * event, and a consumer gating on the mask would never ask for a width the
   * ceiling advertises. */
  c.bw_mask = devourer::kBw20 | devourer::kBw40 | devourer::kBw80;
  c.tune_5g = {true, have_hw ? hw.band_5g_min_mhz : uint16_t(5180),
               have_hw ? hw.band_5g_max_mhz : uint16_t(5825)};
  c.tune_2g4 = {true, have_hw ? hw.band_2g_min_mhz : uint16_t(2412),
                have_hw ? hw.band_2g_max_mhz : uint16_t(2484)};
  c.characterized_5g = c.tune_5g;
  c.characterized_2g4 = c.tune_2g4;
  c.ldpc_rx_ht = true;
  c.ldpc_rx_vht = true;
  c.ldpc_rx_flag = true;     /* the RXWI carries the per-frame LDPC bit */
  c.per_chain_rssi = true;
  c.hw_rx_timestamp = false; /* the RXWI TSF field is not parsed */
  c.hw_beacon_txtsf = false; /* no hardware beacon function ported */
  /* Measured on air: 0 frames at the stimulus radio unarmed, 3500+ armed. */
  c.ack_responder_ok = true;
  /* Unmeasured, so false rather than optimistic - nothing here drives the
   * hardware retry counter. */
  c.tx_retry_limit_ok = false;
  c.narrowband_ok = false;
  /* Measured 526 ms full / 48 ms with calibration skipped, against 0.5-2.5 ms
   * on the Realtek parts: the RF plane lives behind the MCU. Not "fast". */
  c.fastretune_ok = false;
  c.per_packet_txpower = true;
  c.per_pkt_txpwr_steps = 0;
  c.per_pkt_txpwr_step_qdb = 4; /* MT_TX_PWR_ADJ is a 4-bit dB trim */
  c.per_pkt_txpwr_min_qdb = -32;
  c.per_pkt_txpwr_max_qdb = 28;
  c.per_pkt_txpwr_measured = false;
  c.vht_2g4_ok = false; /* unmeasured on this part */
  return c;
}
