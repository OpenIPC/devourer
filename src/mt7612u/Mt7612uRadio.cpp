#include "mt7612u/Mt7612uRadio.h"

#include <chrono>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <span>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include "TxPower.h"
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
  {
    std::lock_guard<std::mutex> lock(sink_mu());
    /* The sink is a process-global pair in the C library, so with two radios
     * open the last constructor would own BOTH libraries' diagnostics and the
     * first destructor would unhook the survivor's - leaving it writing to raw
     * stderr for the rest of its life, which is the exact failure the sink
     * exists to prevent. Route by a registry instead of by `this`: install
     * once, and keep it installed until the last radio goes. */
    if (sink_registry().empty())
      mt7612u_set_log_sink(&Mt7612uRadio::log_trampoline, nullptr);
    sink_registry().push_back(this);
  }
}

Mt7612uRadio::~Mt7612uRadio() {
  /* Everything, not just StopRxLoop: stop_tick()'s join throws on EDEADLK or
   * EINVAL, and a destructor is implicitly noexcept, so an escape here is
   * std::terminate rather than a caught error. */
  try {
    Stop();
  } catch (...) {
  }

  /* Deregister AFTER Stop(), not before. Teardown is when the library has the
   * most to say - "async stop: N TX and M RX transfers still in flight after
   * 2 s, leaking the ring and the USB handle with it" is the single most
   * important line it can print - and deregistering first sent all of it to
   * raw stderr, bypassing the log level, a redirected stream and
   * __android_log_write, which is the whole reason the sink exists. Safe in
   * this order because log_trampoline holds sink_mu() for its entire call and
   * _logger outlives the body of this destructor. */
  {
    std::lock_guard<std::mutex> lock(sink_mu());
    auto &reg = sink_registry();
    for (auto it = reg.begin(); it != reg.end(); ++it)
      if (*it == this) {
        reg.erase(it);
        break;
      }
    if (reg.empty())
      mt7612u_set_log_sink(nullptr, nullptr);
  }
}

/* Deliberately leaked. A Mt7612uRadio destroyed during static destruction would
 * otherwise touch a destroyed mutex and vector; leaking two small objects at
 * exit is the cheaper failure. */
std::mutex &Mt7612uRadio::sink_mu() {
  static std::mutex *m = new std::mutex();
  return *m;
}

std::vector<Mt7612uRadio *> &Mt7612uRadio::sink_registry() {
  static std::vector<Mt7612uRadio *> *reg = new std::vector<Mt7612uRadio *>();
  return *reg;
}

void Mt7612uRadio::log_trampoline(void *user, char level, const char *line) {
  (void)user;
  if (!line)
    return;
  /* The C library has one sink for the process, so a line cannot be attributed
   * to a particular adapter - it is routed to the first live radio's logger,
   * which in the single-adapter case (every shipping consumer today) is the
   * only one there is. Taking the registry lock here also means a destructor
   * cannot deregister mid-call. */
  std::lock_guard<std::mutex> lock(sink_mu());
  auto &reg = sink_registry();
  if (reg.empty())
    return;
  Mt7612uRadio *self = reg.front();
  if (!self->_logger)
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
    /* Replays base AND offset, so neither is lost across a Stop()/re-Init().  */
    if (_txpwr_dbm != 20 || _txpwr_offset_qdb != 0)
      mt7612u_set_txpower(_dev, txpower_target_dbm());
  }

  if (mt7612u_set_channel(_dev, channel.Channel, bw) != 0)
    throw std::runtime_error("MT7612U channel set failed");
  _channel = channel;
  apply_config();
  start_tick();
}

/* Every DeviceConfig knob this backend can reach, and a loud line for each one
 * it cannot.
 *
 * The rule is the sibling backend's: a config value that reads as applied while
 * the radio runs something else is the one failure worse than an unported knob.
 * Sited in bring_up so an RX-only session is told too, and so each fires once
 * per bring-up rather than once per frame. */
void Mt7612uRadio::apply_config() {
  /* Opt-in only, never a default: it turns a passive monitor into an active
   * SIFS-timed transmitter. The caller asked for a responder, not a monitor, so
   * a refusal is fatal rather than swallowed - otherwise the operator gets a
   * green init and a session that silently answers nothing, and debugs the RF
   * link instead of the config. */
  if (_cfg.rx.ack_responder && !SetAckResponder(*_cfg.rx.ack_responder))
    throw std::runtime_error("MT7612U ACK responder could not be armed");

  /* Not programmable here. DeviceConfig calls this "ONE default, 128 us,
   * programmed identically on every generation at bring-up", and that sentence
   * stops being true the moment this adapter is attached - so say so rather
   * than let a range budget be assumed. It matters on this part specifically:
   * docs/mt7612u.md attributes the 40x unicast-injection cliff to the ACK
   * timeout. */
  if (_cfg.tx.ack_timeout_us != 128)
    _logger->warn("MT7612U: tx.ack_timeout_us={} is not programmable by this "
                  "backend - the MAC keeps its own default, so the range "
                  "budget this knob implies does not apply here",
                  _cfg.tx.ack_timeout_us);

  if (_cfg.tuning.disable_cca)
    _logger->warn("MT7612U: DEVOURER_DIS_CCA / tuning.disable_cca is not "
                  "implemented by this backend - carrier-sense stays ENABLED "
                  "for this session");

  /* rx.pool_exhaust defaults to Backpressure, and DeviceConfig says the
   * non-SpscFat modes "never drop host-side ... which backpressures the chip by
   * construction". Not available on this part: backpressure here means not
   * draining EP4, and an undrained receiver wedges this silicon below the USB
   * level where only a physical replug recovers it. The hand-off queue drops
   * the newest frame and counts it instead, and the teardown line reports any
   * loss. Said once, rather than left to be inferred from a frame count. */
  if (_cfg.rx.pool_exhaust == devourer::PoolExhaust::Backpressure)
    _logger->warn("MT7612U: rx.pool_exhaust=backpressure cannot be honoured - "
                  "host-side backpressure on this part means an undrained "
                  "receiver, which wedges it below the USB level. The RX "
                  "hand-off queue drops the newest frame instead, and counts "
                  "every drop.");

  if (_cfg.tx.usb_agg_max > 0)
    _logger->warn("MT7612U: tx.usb_agg_max={} is not consulted - send_packets "
                  "always chains frames into shared bulk-OUT URBs on this "
                  "part, and send_packet never does",
                  _cfg.tx.usb_agg_max);
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
  /* Stop() on the way out, as the sibling backends do: a throw from the channel
   * set or the config would otherwise leave the device open and the tick thread
   * running until the destructor happens to run, and a caller that catches and
   * retries would get a half-open object.
   *
   * The cleanup MUST run with _mu released. Stop() joins the tick thread, and
   * the tick takes _mu once it wakes - so calling Stop() from inside the locked
   * scope deadlocks whenever the tick is already blocked on that lock. Hence
   * the exception_ptr rather than a plain catch-and-rethrow. */
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
  /* Deliberately outside the lock: StartRxLoop blocks until StopRxLoop. Its
   * four throws get the same cleanup as bring_up's - without this, "RX ring
   * failed to start" left the device open and the tick running, which is the
   * half-open object this whole guard exists to prevent. */
  try {
    StartRxLoop(std::move(packetProcessor));
  } catch (...) {
    Stop();
    throw;
  }
}

void Mt7612uRadio::InitWrite(SelectedChannel channel) {
  /* Same shape as Init, and for the same lock-ordering reason: the cleanup runs
   * outside _mu because Stop() joins the tick thread. */
  std::exception_ptr failed;
  {
    std::lock_guard<std::recursive_mutex> lock(_mu);
    try {
      bring_up(channel);
      /* No RX ring, so mt7612u_start() enables TX only - which is the point of
       * this entry point, and also what keeps the chip out of the undrained-
       * receiver wedge. */
      if (mt7612u_start(_dev) != 0)
        throw std::runtime_error("MT7612U MAC start failed");
    } catch (...) {
      failed = std::current_exception();
    }
  }
  if (failed) {
    Stop();
    std::rethrow_exception(failed);
  }
}

void Mt7612uRadio::StartRxLoop(Action_ParsedRadioPacket packetProcessor) {
  struct mt7612u_dev *mac_failed = nullptr;
  {
    /* The WHOLE prologue, arming through the failure teardown, under the same
     * lock StopRxLoop uses - and released before the sleep loop below, which
     * calls StopRxLoop and would otherwise self-deadlock on it.
     *
     * Without this, moving mt7612u_rx_stop() out of the _mu scope reopened the
     * use-after-free _teardown_mu exists to close: between releasing _mu and
     * the teardown, a concurrent Stop() sees _rx_active still false, takes _mu
     * and calls mt7612u_close(), which frees the device this thread is about
     * to hand to mt7612u_rx_stop(). Ordering is _teardown_mu -> _mu here and in
     * StopRxLoop; nothing takes them the other way round. */
    std::lock_guard<std::mutex> teardown(_teardown_mu);
    std::lock_guard<std::recursive_mutex> lock(_mu);
    if (!_dev)
      throw std::runtime_error("MT7612U RX loop requires initialized hardware");
    if (_rx_active.load())
      throw std::runtime_error("MT7612U RX loop is already active");
    _rx_processor = std::move(packetProcessor);
    _rx_stop = false;
    /* 64 slots is about 45 ms of headroom at the measured 1400 fps - enough to
     * ride out a slow processor without letting the producer block. The ring
     * itself is allocated here; each slot's payload buffer then grows to the
     * largest frame that slot has held (~245 KB in total at the part's
     * 3836-byte max MPDU), so the event thread does still allocate during the
     * first pass round the ring and on any frame-size step-up. */
    _rx_q.reset(64);

    /* Ring first, receiver second - see rule 1 in the header. */
    if (mt7612u_rx_start(_dev, &Mt7612uRadio::rx_trampoline, this) != 0)
      throw std::runtime_error("MT7612U RX ring failed to start");
    if (mt7612u_start(_dev) != 0) {
      /* Same two rules as StopRxLoop, and for the same reasons: quiesce before
       * removing the drain, and finish the teardown OUTSIDE _mu - the ring was
       * armed one line above, so a frame can already be in a processor that
       * takes this lock, and mt7612u_rx_stop() joins that thread. Recorded
       * here and acted on below rather than unlocking by hand mid-scope. */
      mt7612u_rx_quiesce(_dev);
      mac_failed = _dev;
    } else {
      /* AFTER mt7612u_start(), which rewrites the filter to mt76's managed-mode
       * value - see rule 2. Before it, this write is simply overwritten. */
      if (mt7612u_set_monitor_rx(_dev, _cfg.rx.keep_corrupted ? 1 : 0) != 0)
        _logger->warn("MT7612U monitor RX filter not applied");
      /* Arms the channel timers and zeroes the MIB counters. */
      mt7612u_link_stats_start(_dev);
      _rx_active.store(true, std::memory_order_release);
    }
    if (mac_failed) {
      mt7612u_rx_stop(mac_failed);
      /* Only now that the ring is down: clearing it while transfers are armed
       * would destroy a std::function the event thread can be inside. */
      _rx_processor = nullptr;
    }
  }
  if (mac_failed)
    throw std::runtime_error("MT7612U MAC start failed");

  _logger->info("MT7612U monitor RX on channel {}", _channel.Channel);

  /* THE consumer. The C layer parses on its own event thread and enqueues; the
   * processor runs here, on the thread that called StartRxLoop, which is the
   * contract every other backend keeps. See the queue's comment in the header
   * for why delivering on the event thread wedges the hardware. */
  for (;;) {
    mt7612u::RxQueue::Slot *slot =
        _rx_q.pop_begin(std::chrono::milliseconds(20), _rx_stop);
    if (!slot) {
      if (_rx_stop.load() || g_devourer_should_stop)
        break;
      continue;
    }

    /* Outside the queue lock: user code runs here, and it may call back into
     * this object. The producer never writes the slot at the tail, so this
     * reference stays valid until pop_commit() below. */
    Packet packet{};
    packet.RxAtrib = slot->attrib;
    packet.Data = std::span<uint8_t>(slot->data.data(), slot->data.size());
    if (_rx_processor)
      _rx_processor(packet);

    _rx_q.pop_commit(slot);
  }

  StopRxLoop();
}

void Mt7612uRadio::StopRxLoop() {
  _rx_stop = true;
  /* Wake the consumer immediately rather than leaving it to time out. */
  _rx_q.wake();

  /* Held across the WHOLE teardown, so a second caller blocks here instead of
   * returning while the first is still inside mt7612u_rx_stop(). That early
   * return was a use-after-free: the loser reported the ring down, went on to
   * Stop(), and mt7612u_close() freed the device while the winner was still
   * cancelling its transfers and joining the event thread. */
  std::lock_guard<std::mutex> teardown(_teardown_mu);
  if (!_rx_active.load(std::memory_order_acquire))
    return;

  struct mt7612u_dev *dev = nullptr;
  {
    /* Quiesce under _mu: it is a register write, and it must not interleave
     * with the tick's MCU traffic. The device pointer is read here too - _dev
     * is written under _mu, so reading it outside would race Stop(). */
    std::lock_guard<std::recursive_mutex> lock(_mu);
    dev = _dev;
    if (dev)
      mt7612u_rx_quiesce(dev);
  }
  /* Ring teardown WITHOUT _mu. mt7612u_rx_stop() joins the event thread, and a
   * processor still in flight on that thread may call back into this object -
   * SetMonitorChannel, say - which takes _mu. Holding _mu across the join is a
   * deadlock, not a theoretical one. */
  if (dev)
    mt7612u_rx_stop(dev);

  /* Cleared only now that the ring is genuinely down. Clearing it up front let
   * a restart pass the "already active" guard while the old ring still
   * existed, and mt7612u_rx_start() then returned success WITHOUT arming. */
  _rx_active.store(false, std::memory_order_release);

  const uint64_t dropped = _rx_q.dropped();
  if (dropped)
    /* "received" is the count off the air, so it INCLUDES the dropped ones -
     * spelling that out because "N frames, D DROPPED" reads as N delivered. */
    _logger->warn("MT7612U RX stopped after {} frames received, of which {} "
                  "DROPPED at the hand-off queue - the packet processor could "
                  "not keep up",
                  _rx_frames.load(std::memory_order_relaxed), dropped);
  else
    _logger->info("MT7612U RX stopped after {} frames",
                  _rx_frames.load(std::memory_order_relaxed));
}

void Mt7612uRadio::rx_trampoline(void *user, const void *frame, size_t len,
                                 const struct mt7612u_rx_info *info) {
  static_cast<Mt7612uRadio *>(user)->on_rx(frame, len, info);
}

void Mt7612uRadio::on_rx(const void *frame, size_t len,
                         const struct mt7612u_rx_info *info) {
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

  {
    const uint8_t *f = static_cast<const uint8_t *>(frame);
    uint8_t tid = 0;
    /* qos_tid finds the QoS Control field at the RIGHT offset - 30, not 24, on
     * a 4-address frame - so priority is a TID and never the low nibble of an
     * address. */
    packet.RxAtrib.qos = mt7612u::qos_tid(f, len, tid);
    packet.RxAtrib.priority = tid;
  }

  _rx_frames.fetch_add(1, std::memory_order_relaxed);

  /* Copy and hand off. The library's buffer is reused the moment this returns,
   * and the consumer now runs on another thread, so the frame cannot be passed
   * by reference the way it could when the processor ran here. A full queue
   * drops this frame and counts it rather than blocking - blocking here is
   * precisely the wedge the queue exists to prevent. */
  _rx_q.push(packet.RxAtrib, static_cast<const uint8_t *>(frame), len);
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
  const bool ok = mt7612u_send_packet(_dev, packet, length) == 0;
  (ok ? _tx_submitted : _tx_failed).fetch_add(1, std::memory_order_relaxed);
  return ok;
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
  const size_t sent = mt7612u_send_packets(_dev, views.data(), count);
  _tx_submitted.fetch_add(sent, std::memory_order_relaxed);
  _tx_failed.fetch_add(count - sent, std::memory_order_relaxed);
  return sent;
}

void Mt7612uRadio::SetCcaMode(bool disabled) {
  /* Asking for carrier-sense ENABLED is asking for the state this chip is
   * already in, so it succeeds quietly - a caller that asserts the default
   * unconditionally must not be punished for asking. Only the disable is
   * refused. */
  if (!disabled)
    return;
  /* Refuses rather than no-ops. MT7612U does have an ED-CCA enable
   * (MT_TXOP_CTRL_CFG / MT_TXOP_ED_CCA_EN, which mac_stop already clears), but
   * "disable CCA" on the Realtek backends means a specific, measured set of
   * writes, and nothing here has been measured against an on-air carrier-sense
   * test. Claiming it on the strength of one plausible-looking bit is how an
   * unverified regulatory-adjacent behaviour ships. */
  _logger->error("MT7612U: SetCcaMode(true) not implemented - the ED-CCA "
                 "enable exists but no on-air carrier-sense measurement backs "
                 "it; carrier-sense stays ENABLED for this session");
}

void Mt7612uRadio::Stop() {
  /* Nothing here may escape: Stop() is called from the destructor, and a throw
   * that skipped stop_tick() would leave _tick joinable, whose destructor calls
   * std::terminate. The logging inside StopRxLoop is the realistic thrower. */
  try {
    StopRxLoop();
  } catch (...) {
  }
  stop_tick(); /* joins; must not run with _mu held */

  /* Take the device out under _mu, then close it OUTSIDE - mt7612u_close()
   * runs mt_async_stop(), which joins the libusb event thread, and holding _mu
   * across that join is the same deadlock StopRxLoop documents avoiding. The
   * teardown lock is what keeps a concurrent StopRxLoop from using the pointer
   * after this steals it. */
  struct mt7612u_dev *dev = nullptr;
  {
    std::lock_guard<std::mutex> teardown(_teardown_mu);
    std::lock_guard<std::recursive_mutex> lock(_mu);
    dev = _dev;
    _dev = nullptr;
    if (dev)
      mt7612u_stop(dev);
  }
  if (dev)
    mt7612u_close(dev);
}

void Mt7612uRadio::SetTxPower(uint8_t power) {
  std::lock_guard<std::recursive_mutex> lock(_mu);
  /* Deliberately NOT forwarded to SetTxPowerIndexOverride the way the base
   * class does: there is no TXAGC index here, so the argument is read as the
   * dBm limit it actually maps to. */
  /* Clamped HERE, not only at the actuator. Storing the raw byte let a value
   * above 30 - including a negative that narrowed into uint8_t - sit in the
   * base while txpower_target_dbm() quietly clamped the hardware to maximum
   * output, so every later offset composed against a base the radio never
   * used. */
  int dbm = static_cast<int>(power);
  if (dbm > 30) {
    _logger->warn("MT7612U TX power {} dBm is above this part's 30 dBm ceiling "
                  "- clamping", dbm);
    dbm = 30;
  }
  _txpwr_dbm = dbm;
  /* Composes with a live offset rather than discarding it - IRadio says the two
   * compose, and writing the bare base would silently undo an offset while
   * _txpwr_offset_qdb still reported it as applied. */
  if (_dev && mt7612u_set_txpower(_dev, txpower_target_dbm()) != 0)
    _logger->error("MT7612U TX power {} dBm refused (valid range 0-30)",
                   _txpwr_dbm);
}

/* Real on this silicon, so implemented rather than advertised and ignored.
 * There is no TXAGC index to trim, but mt7612u_set_txpower() is an absolute
 * dBm limit that feeds the per-rate table, so an offset folds onto it.
 *
 * The library takes whole dBm, so the applied value is quantized to 4 qdB and
 * that quantized figure is what comes back - the contract is "returns the
 * APPLIED qdB ... so a closed-loop controller knows exactly what moved", and a
 * controller told -20 when -20 was rounded to -20 but only -16 landed would
 * integrate against a number the radio never used. Sticky across
 * SetMonitorChannel because the base is re-applied from _txpwr_dbm. */
int Mt7612uRadio::SetTxPowerOffsetQdb(int qdb) {
  std::lock_guard<std::recursive_mutex> lock(_mu);
  /* The family's own quantizer, not a local q/4. It rounds to nearest with
   * ties away from zero, which is the documented rule every other backend
   * follows; truncating toward zero instead turned a -6 qdB request into -4
   * (LESS attenuation than asked for) and any sub-step request into 0 - which
   * is also the "refused" return, so a small request was indistinguishable
   * from a refusal. That was the exact fault this override exists to remove. */
  const devourer::TxPowerCaps caps = GetTxPowerCaps();
  const int want_qdb = devourer::quantize_offset_qdb(qdb, caps, nullptr);

  /* The applied value is derived AFTER the rail clamp, never before. The
   * actuator is an absolute 0-30 dBm limit, so an offset that would drive it
   * past a rail is only partly applied - and returning the requested figure
   * there tells a closed-loop controller the radio moved further than it did.
   * With base 5 dBm, a -20 dB request lands at 0 dBm, i.e. -5 dB, and that is
   * what comes back. */
  int dbm = _txpwr_dbm + want_qdb / 4;
  if (dbm < 0)
    dbm = 0;
  if (dbm > 30)
    dbm = 30;
  const int applied_qdb = (dbm - _txpwr_dbm) * 4;

  if (_dev && mt7612u_set_txpower(_dev, dbm) != 0) {
    _logger->error("MT7612U TX power offset {} qdB -> {} dBm refused", qdb, dbm);
    return 0;
  }
  /* REQUESTED is what is remembered; APPLIED is what is returned. Storing the
   * rail-clamped figure instead would shrink the offset permanently: ask for
   * -20 dB against a 5 dBm base, get -5 dB at the 0 dBm rail, and a later
   * SetTxPower(20) would compose with -5 rather than restoring the -20 that is
   * still configured. Recorded even with no device open, so an offset set
   * before Init survives to the first tune instead of being swallowed. */
  _txpwr_offset_qdb = want_qdb;
  return applied_qdb;
}

/* The rate a frame airs at when its radiotap carries none.
 *
 * REFUSED, loudly, because this backend cannot honour it and a silent no-op
 * here is a measurement that lies: examples/tx builds rate-less frames on
 * purpose and calls SetTxMode to drive an MCS sweep, and the library's
 * radiotap parser defaults an un-rated frame to OFDM MCS0 (radiotap.cpp) - so
 * an MCS7 sweep would report MCS7 and air 6 Mbps. Confirmed on this bench: an
 * RTL8812AU witnessing our transmit logged rate 4 (OFDM 6M) for every frame.
 *
 * Honouring it needs a session-default rate in the C library, which has none -
 * mt7612u_send_packet() re-derives the rate from each frame's radiotap. Until
 * then, put the rate in the radiotap header, where it always wins. */
void Mt7612uRadio::SetTxMode(const devourer::TxMode &mode) {
  (void)mode;
  _logger->error("MT7612U: SetTxMode is not implemented - the C library has no "
                 "session-default rate, so a rate-less frame airs at OFDM "
                 "6 Mbps. Put the rate in each frame's radiotap header.");
}

void Mt7612uRadio::ClearTxMode() {
  /* Nothing was ever set, and "cleared" is the state this backend is always
   * in - so this one is genuinely a no-op rather than a hidden refusal. */
}

bool Mt7612uRadio::SetAmpduMode(const devourer::AmpduMode &mode) {
  (void)mode;
  /* Aggregation on this part is per-frame descriptor state set on the TXWI at
   * build time, not MAC state - so there is nothing to program here. It works:
   * measured 326/326 frames aggregated and 2.21x throughput at 200 B
   * (docs/mt7612u.md). Reaching it through IRadio needs the radiotap A-MPDU
   * field plumbed into the library's frame builder, which is not done. Refused
   * rather than accepting a mode that would never reach the air. */
  _logger->error("MT7612U: A-MPDU works on this part (docs/mt7612u.md, 2.21x at "
                 "200 B) but is not wired through send_packet yet - refusing "
                 "rather than accepting a mode that would not reach the air");
  return false;
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

devourer::TxStats Mt7612uRadio::GetTxStats() {
  devourer::TxStats out{};

  /* INCOMPLETE, deliberately, and the incompleteness is documented rather than
   * papered over: `failed` counts frames the transport REFUSED, not frames
   * that died on the wire. An earlier cut added the ring's tx_err to close
   * that, and it was wrong three ways - mt_async_tx_submit increments tx_err
   * AND returns -1, so a submit failure counted twice; tx_err counts URBs
   * while this counts frames, and an aggregated URB carries up to 32; and
   * mt_async_stop deletes the ring, so the counter restarts at zero and
   * `failed` went BACKWARDS across an RX restart, which a consumer differencing
   * a uint64_t reads as ~1.8e19. Reading it also took _mu, which send_packet
   * holds across a blocking submit - so a stats poll from another thread
   * stalled for as long as the TX ring was saturated. A documented gap beats
   * four wrong numbers; the fix belongs in the library, which needs a
   * monotonic wire-failure counter that outlives a ring.
   *
   * Counted here rather than read from mt7612u_get_stats(), which reports the
   * ASYNC RING's counters. mt_tx_raw() only uses that ring when one is running
   * (tx.cpp), and the TX-only bring-up this backend offers - InitWrite with no
   * StartRxLoop - starts no ring, so those counters read 0 while frames are
   * going out. Measured: txdemo on this part reported submitted=0 against
   * twelve tx.frame events with rc=1. A stat that reads zero while the radio
   * transmits is worse than no stat, so this counts what devourer actually
   * handed the transport.
   *
   * last_error_rc and last_was_timeout stay at their defaults: the library
   * returns a count, not the libusb rc of the last failure, and inventing one
   * would be the same fault in a different field. */
  out.submitted = _tx_submitted.load(std::memory_order_relaxed);
  out.failed = _tx_failed.load(std::memory_order_relaxed);
  return out;
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

/* The absolute dBm the actuator should carry: the base plus whatever offset is
 * live, clamped to the part's 0-30 range. One place, so the base setter, the
 * offset setter and the bring-up replay cannot drift apart. */
int Mt7612uRadio::txpower_target_dbm() const {
  int dbm = _txpwr_dbm + _txpwr_offset_qdb / 4;
  if (dbm < 0)
    return 0;
  if (dbm > 30)
    return 30;
  return dbm;
}

devourer::TxCaps Mt7612uRadio::GetTxCaps() {
  devourer::TxCaps c{};
  c.supported = true;
  /* From the library, so this cannot disagree with GetAdapterCaps().tx_chains,
   * which reads the same field - the two travel together in one adapter.caps
   * event and a consumer comparing them would have no way to pick. */
  {
    std::lock_guard<std::recursive_mutex> lock(_mu);
    struct mt7612u_caps hw {};
    if (_dev) {
      mt7612u_get_caps(_dev, &hw);
      c.n_ss = hw.nss_tx;
    } else {
      c.n_ss = 2;
    }
  }
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
  /* 4, not 2. The limit is carried internally in 0.5 dB units, but the only
   * actuator reachable from here - mt7612u_set_txpower() - takes WHOLE dBm, so
   * half-dB is not a step this backend can take. Advertising 2 would have
   * TxPower.h's quantize_offset_qdb round requests to a granularity the
   * hardware cannot honour. */
  c.step_qdb = 4;
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
  /* Tunable is not characterized. The TX-power registers were verified equal to
   * the vendor driver's at ch149 only, and docs/mt7612u.md says plainly that
   * "register equality is not dBm" - nothing has been measured against a power
   * meter across either band. Left invalid rather than claiming the whole
   * tunable span is table-backed. */
  c.characterized_5g = {};
  c.characterized_2g4 = {};
  c.ldpc_rx_ht = true;
  /* HT and VHT are separate decoder paths in silicon, which is why AdapterCaps
   * splits them - and docs/mt7612u.md lists VHT on air as unexercised. The one
   * LDPC-RX measurement on this part is an HT frame, so the VHT half is
   * unmeasured and says so. */
  c.ldpc_rx_vht = false;
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
  /* MT_TX_PWR_ADJ is a real 4-bit per-frame dB trim, and mt7612u_tx() takes it
   * as mt7612u_tx_rate.power_adj - but nothing reaches it from here. The
   * library's radiotap path PARSES DBM_TX_POWER and explicitly discards it
   * (radiotap.cpp), the session route is gated on an enable_tpc field that no
   * code ever assigns (internal.h), and this backend calls send_packet, not
   * mt7612u_tx(). So the mechanism exists and has no caller; advertising it
   * would hand a rate controller a knob with nothing behind it. false until
   * one of those paths is wired, at which point the ranges below come back. */
  c.per_packet_txpower = false;
  c.vht_2g4_ok = false; /* unmeasured on this part */
  return c;
}
