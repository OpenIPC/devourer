#include "RtlJaguarDevice.h"
#include "EepromManager.h"
#include "RadioManagementModule.h"
#include "SignalStop.h"

#include <chrono>
#include <cstdlib>
#include <mutex>
#include <thread>
#include <vector>

/* Per-queue free-page registers (8814A only — 0x0230..0x0240 don't decode
 * the same way on 8812/8821). Cross-checked against hal/rtl8814a_spec.h
 * and src/HalModule.cpp's REG_FIFOPAGE_INFO_*_8814A constants. */
static constexpr uint16_t kFifoPageInfoRegs_8814A[5] = {
    0x0230, 0x0234, 0x0238, 0x023C, 0x0240,
};

RtlJaguarDevice::RtlJaguarDevice(RtlUsbAdapter device, Logger_t logger)
    : _device{device},
      _eepromManager{std::make_shared<EepromManager>(device, logger)},
      _radioManagement{std::make_shared<RadioManagementModule>(
          device, _eepromManager, logger)},
      _halModule{device, _eepromManager, _radioManagement, logger},
      _logger{logger} {}

void RtlJaguarDevice::InitWrite(SelectedChannel channel) {
  StartWithMonitorMode(channel);
  SetMonitorChannel(channel);
  _logger->info("In Monitor Mode");
}

/* Map a radiotap CHANNEL frequency (MHz) to a Wi-Fi channel number. Returns 0
 * for a frequency outside the bands devourer drives (caller ignores it). */
static int freq_to_chan(uint16_t freq_mhz) {
  if (freq_mhz == 2484)
    return 14;
  if (freq_mhz >= 2412 && freq_mhz <= 2472)
    return (freq_mhz - 2407) / 5;        /* 2.4 GHz ch1..13 */
  if (freq_mhz >= 5000 && freq_mhz <= 5895)
    return (freq_mhz - 5000) / 5;        /* 5 GHz UNII ch */
  return 0;
}

SelectedChannel RtlJaguarDevice::GetSelectedChannel() { return _channel; }

bool RtlJaguarDevice::send_packet(const uint8_t *packet, size_t length) {
  struct tx_desc *ptxdesc;
  bool resp;
  uint8_t *usb_frame;
  int real_packet_length, usb_frame_length, radiotap_length;

  bool vht = false;
  int ret = 0;
  int qos_len = 0;
  u8 fixed_rate = MGN_1M, sgi = 0, bwidth = 0, ldpc = 0, stbc = 0;
  u16 txflags = 0;
  int rate_id = 0;
  /* True once the radiotap carries a rate (RATE / HT-MCS-index / VHT). When it
   * stays false the device TX-mode default (SetTxMode) — else MGN_1M — applies. */
  bool rate_from_radiotap = false;
  /* Per-packet hop target from a radiotap CHANNEL field (0 = none present). */
  int radiotap_channel = 0;
  if (length < sizeof(struct ieee80211_radiotap_header)) {
    return false;
  }
  radiotap_length = get_unaligned_le16(packet + 2);
  if (radiotap_length == 0 || (size_t)radiotap_length >= length) {
    return false;
  }
  real_packet_length = length - radiotap_length;

  if (radiotap_length != 0x0d)
    vht = true;

  usb_frame_length = real_packet_length + TXDESC_SIZE;

  _logger->debug("radiotap length is {}, 80211 length is {}, usb_frame length "
                "should be {}",
                radiotap_length, real_packet_length, usb_frame_length);

  struct ieee80211_radiotap_header *rtap_hdr;
  rtap_hdr = (struct ieee80211_radiotap_header *)packet;
  struct ieee80211_radiotap_iterator iterator;
  ret = ieee80211_radiotap_iterator_init(&iterator, rtap_hdr, radiotap_length,
                                         NULL);
  while (!ret) {
    ret = ieee80211_radiotap_iterator_next(&iterator);

    if (ret)
      continue;

    /* see if this argument is something we can use */
    switch (iterator.this_arg_index) {

    case IEEE80211_RADIOTAP_RATE:
      fixed_rate = *iterator.this_arg;
      rate_from_radiotap = true;
      break;

    case IEEE80211_RADIOTAP_TX_FLAGS:
      txflags = get_unaligned_le16(iterator.this_arg);
      break;

    case IEEE80211_RADIOTAP_CHANNEL:
      /* 2 x __le16: frequency (MHz), then flags. Frequency is authoritative
       * for the per-packet hop target; flags are ignored (rate/BW come from
       * the RATE/MCS/VHT fields). */
      radiotap_channel = freq_to_chan(get_unaligned_le16(iterator.this_arg));
      break;

    case IEEE80211_RADIOTAP_MCS: {
      u8 mcs_known = iterator.this_arg[0];
      u8 mcs_flags = iterator.this_arg[1];

      uint8_t mcs_bw_field = mcs_flags & IEEE80211_RADIOTAP_MCS_BW_MASK;
      if (mcs_bw_field == IEEE80211_RADIOTAP_MCS_BW_40) {
        bwidth = CHANNEL_WIDTH_40;
      } else if (mcs_bw_field == IEEE80211_RADIOTAP_MCS_BW_20 ||
                 mcs_bw_field == IEEE80211_RADIOTAP_MCS_BW_20L ||
                 mcs_bw_field == IEEE80211_RADIOTAP_MCS_BW_20U) {
        bwidth = CHANNEL_WIDTH_20;
      }

      if (mcs_flags & 0x04) {
        sgi = 1;
      } else {
        sgi = 0;
      }

      /* STBC (radiotap MCS known bit5 / flags bits5-6) and FEC=LDPC (known bit4 /
       * flags bit4). The HT branch previously read neither, so an HT frame tagged
       * STBC/LDPC in radiotap silently transmitted as BCC SISO -- only the VHT
       * branch honoured them. Reading them here lets WiFiDriverTxDemo emit a real
       * HT STBC / HT LDPC frame (needed as a chip reference for the gr-ieee802-11
       * fork's modern-format TX). */
      if (mcs_known & 0x20) {
        stbc = (mcs_flags >> 5) & 0x3;
      }
      if ((mcs_known & 0x10) && (mcs_flags & 0x10)) {
        ldpc = 1;
      }

      /* The radiotap is authoritative for per-packet rate: honour the HT MCS
       * index from byte 2 unconditionally. (Previously gated behind the
       * DEVOURER_TX_HT_MCS env var, so a valid HT radiotap silently fell back
       * to 1M CCK — now replaced by the programmatic SetTxMode default, applied
       * after this loop only when no rate is present in the radiotap.) */
      if (mcs_known & IEEE80211_RADIOTAP_MCS_HAVE_MCS) {
        uint8_t mcs_index = iterator.this_arg[2];
        if (mcs_index <= 31) {
          fixed_rate = MGN_MCS0 + mcs_index;
          rate_from_radiotap = true;
        }
      }
    } break;

    case IEEE80211_RADIOTAP_VHT: {
      u8 known = iterator.this_arg[0];
      u8 flags = iterator.this_arg[2];
      unsigned int mcs, nss;
      if ((known & 4) && (flags & 4))
        sgi = 1;
      if ((known & 1) && (flags & 1))
        stbc = 1;
      if (known & 0x40) {
        auto bw = iterator.this_arg[3] & 0x1f;
        /* Map radiotap VHT bandwidth codes to CHANNEL_WIDTH enums — the
         * descriptor BW switch below compares against the enums (as the
         * HT path above does). The previous MHz literals (40/80) never
         * matched CHANNEL_WIDTH_40(1)/CHANNEL_WIDTH_80(2), so VHT 40/80
         * silently transmitted as 20MHz. */
        if (bw >= 1 && bw <= 3)
          bwidth = CHANNEL_WIDTH_40;
        else if (bw >= 4 && bw <= 10)
          bwidth = CHANNEL_WIDTH_80;
        else
          bwidth = CHANNEL_WIDTH_20;
      }

      if (iterator.this_arg[8] & 1)
        ldpc = 1;
      mcs = (iterator.this_arg[4] >> 4) & 0x0f;
      nss = iterator.this_arg[4] & 0x0f;
      if (nss > 0) {
        if (nss > 4)
          nss = 4;
        if (mcs > 9)
          mcs = 9;
        fixed_rate = MGN_VHT1SS_MCS0 + ((nss - 1) * 10 + mcs);
        rate_from_radiotap = true;
      }
    } break;

    default:
      break;
    }
  }

  /* Radiotap CHANNEL is authoritative for per-packet frequency, exactly as
   * RATE/MCS are for rate: a frame asking for a different channel triggers a
   * lean FastRetune before TX (intra-band 20 MHz ~1.6 ms; cross-band/non-20MHz
   * falls back to the full path). A no-op when it equals the current channel,
   * so an informational CHANNEL field costs nothing. This makes frequency
   * hopping radiotap-driven — any caller can hop per-packet without an env knob
   * — at the honest cost of stalling this send by the retune latency. Done
   * before the 5 GHz CCK clamp below so the clamp keys off the new channel. */
  if (radiotap_channel > 0 && radiotap_channel != _channel.Channel) {
    FastRetune(static_cast<uint8_t>(radiotap_channel));
  }

  /* The radiotap carried no rate → apply the runtime TX-mode default set via
   * SetTxMode (modulation / MCS / BW / GI / FEC / STBC). With no default set,
   * the MGN_1M fallback from above stands. Per-packet radiotap always wins. */
  if (!rate_from_radiotap && _tx_mode_default.has_value()) {
    const devourer::TxParams tp =
        devourer::tx_mode_to_params(*_tx_mode_default);
    fixed_rate = tp.fixed_rate;
    vht = tp.vht;
    sgi = tp.sgi;
    ldpc = tp.ldpc;
    stbc = tp.stbc;
    bwidth = static_cast<u8>(tp.bwidth);
  }

  /* CCK rates (1/2/5.5/11M) do not exist at 5GHz. The RTL8814AU silently
   * drops a CCK-rated frame on a 5GHz channel — the bulk-OUT completes but
   * nothing goes on-air (verified on hardware: default MGN_1M beacon = 0
   * frames at ch36/ch100, but ~14k on-air once the rate is OFDM). 2.4GHz
   * CCK is fine. So on a 5GHz channel, clamp a CCK rate to the lowest OFDM
   * rate. (The 8812 chip happens to auto-fall-back CCK->OFDM at 5G; the
   * 8814 does not, so we must do it in software.) */
  if (_channel.Channel > 14 &&
      (fixed_rate == MGN_1M || fixed_rate == MGN_2M ||
       fixed_rate == MGN_5_5M || fixed_rate == MGN_11M)) {
    fixed_rate = MGN_6M;
  }

  usb_frame = new uint8_t[usb_frame_length]();

  ptxdesc = (struct tx_desc *)usb_frame;

  _logger->debug("fixed rate:{}, sgi:{}, radiotap_bwidth:{}, ldpc:{}, stbc:{}",
                (int)fixed_rate, (int)sgi, (int)bwidth, (int)ldpc, (int)stbc);

  uint8_t BWSettingOfDesc;
  if (bwidth == CHANNEL_WIDTH_40) {
    BWSettingOfDesc = 1;
  } else if (bwidth == CHANNEL_WIDTH_80) {
    BWSettingOfDesc = 2;
  } else {
    BWSettingOfDesc = 0;
  }
  _logger->debug("TX DESC BW decision: _channel.ChannelWidth(RX)={}, radiotap_bwidth(TX)={}, BWSettingOfDesc(TX_DESC)={}",
                (int)_channel.ChannelWidth, (int)bwidth, (int)BWSettingOfDesc);

  SET_TX_DESC_DATA_BW_8812(usb_frame, BWSettingOfDesc);

  /* The SET_TX_DESC_*_8812 macros have bit-identical positions to the
   * SET_TX_DESC_*_8814A macros (verified against hal/rtl8814a_xmit.h). But
   * a few of the field NAMES differ on 8814A, and a usbmon byte-diff
   * against a working VM-passthrough 88XXau monitor-injection session shows
   * three field-value mismatches on 8814A:
   *
   *   Dword 0 bit 31 — 8812 calls it OWN, 8814A calls it DISQSELSEQ.
   *     88XXau leaves bit 31 = 0 for monitor-injected frames; devourer's
   *     SET_TX_DESC_OWN_8812(usb_frame, 1) sets it to 1, which on 8814A
   *     means DISQSELSEQ=1 (disable queue-select-based sequence numbering).
   *   Dword 2 bits 24-29 (GID) — 88XXau leaves at 0 for injection;
   *     devourer writes 0x3F.
   *   Dword 4 bits 18-23 (DATA_RETRY_LIMIT) — 88XXau leaves at 0 for
   *     injection; devourer writes 12.
   *
   * Skip those writes on 8814A to byte-match aircrack-ng's reference
   * monitor-injection descriptor. Does NOT resolve #50 (on-air silence
   * has a different root cause that vendor-control-write replay can't
   * reach), but aligns devourer's TX descriptor with the working
   * kernel-driver format. Override with DEVOURER_TX_LEGACY_8812_DESC=1
   * to restore the old behaviour without rebuilding.
   *
   * 8812AU and 8821AU paths are bit-for-bit identical to current master --
   * is_8814a is false there and all writes fire as before. */
  const bool is_8814a =
      _eepromManager->version_id.ICType == CHIP_8814A &&
      !std::getenv("DEVOURER_TX_LEGACY_8812_DESC");

  /* Single-fragment frame: LAST_SEG=1 (no FIRST_SEG). */
  SET_TX_DESC_LAST_SEG_8812(usb_frame, 1);
  if (!is_8814a) {
    /* OWN=1 needed on 8812/8821 so chip processes the descriptor. On
     * 8814A the same bit is DISQSELSEQ -- leave at 0 to match 88XXau. */
    SET_TX_DESC_OWN_8812(usb_frame, 1);
  }

  SET_TX_DESC_PKT_SIZE_8812(usb_frame,
                            static_cast<uint32_t>(real_packet_length));

  SET_TX_DESC_OFFSET_8812(usb_frame,
                          static_cast<uint8_t>(TXDESC_SIZE + OFFSET_SZ));

  /* Match kernel-driver TX descriptor field-for-field. Verified by
   * usbmon capture of working kernel-driver TX in monitor mode +
   * byte-for-byte diff. Previous values were based on speculative
   * comments; the chip silently dropped frames whose descriptor didn't
   * match the kernel's. */
  SET_TX_DESC_MACID_8812(usb_frame, static_cast<uint8_t>(0x01));

  if (!vht) {
    rate_id = 8;
  } else {
    rate_id = 9;
  }

  SET_TX_DESC_BMC_8812(usb_frame, 1);
  SET_TX_DESC_RATE_ID_8812(usb_frame, static_cast<uint8_t>(rate_id));

  SET_TX_DESC_QUEUE_SEL_8812(usb_frame, 0x12);
  SET_TX_DESC_HWSEQ_EN_8812(usb_frame, static_cast<uint8_t>(1));
  if (!is_8814a) {
    /* 88XXau leaves GID=0 for monitor injection on 8814A. */
    SET_TX_DESC_GID_8812(usb_frame, static_cast<uint8_t>(0x3F));
  }
  SET_TX_DESC_SW_DEFINE_8812(usb_frame, static_cast<uint16_t>(0x001));
  SET_TX_DESC_RETRY_LIMIT_ENABLE_8812(usb_frame, 1);
  if (!is_8814a) {
    /* 88XXau leaves DATA_RETRY_LIMIT=0 for monitor injection on 8814A
     * (RETRY_LIMIT_ENABLE stays set to 1 in both). */
    SET_TX_DESC_DATA_RETRY_LIMIT_8812(usb_frame, 12);
  }
  if (sgi) {
    _logger->info("short gi enabled,set sgi");
    SET_TX_DESC_DATA_SHORT_8812(usb_frame, 1);
  }
  SET_TX_DESC_USE_RATE_8812(usb_frame, 1);
  SET_TX_DESC_TX_RATE_8812(usb_frame,
                           static_cast<uint8_t>(MRateToHwRate(
                               fixed_rate)));

  if (ldpc) {
    SET_TX_DESC_DATA_LDPC_8812(usb_frame, ldpc);
  }

  SET_TX_DESC_DATA_STBC_8812(usb_frame, stbc & 3);

  rtl8812a_cal_txdesc_chksum(usb_frame);
  _logger->debug("tx desc formed");
#ifdef DEBUG
  for (size_t i = 0; i < usb_frame_length; ++i) {
    std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<int>(usb_frame[i]);

    if (i < usb_frame_length - 1) {
      std::cout << ",";
    }
  }
  std::cout << std::dec << std::endl;
#endif
  uint8_t *addr = usb_frame + TXDESC_SIZE;
  memcpy(addr, packet + radiotap_length, real_packet_length);
  _logger->debug("packet formed");
#ifdef DEBUG
  for (size_t i = 0; i < usb_frame_length; ++i) {
    std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<int>(usb_frame[i]);

    if (i < usb_frame_length - 1) {
      std::cout << ",";
    }
  }
  std::cout << std::dec << std::endl;
#endif
  resp = _device.send_packet(usb_frame, usb_frame_length);
  delete[] usb_frame;

  return resp;
}

std::vector<Packet> RtlJaguarDevice::read_frames() {
  /* Cover one full chip-side RX aggregate (the 8814 pairs a 20K DMA-agg
   * threshold with a 32K host read). 32K buffer, an exact multiple of
   * wMaxPacketSize so no short packet truncates the transfer. */
  static constexpr int BUF_SIZE = 32 * 1024;
  uint8_t buffer[BUF_SIZE] = {};
  int n = _device.bulk_read_raw(buffer, sizeof(buffer), USB_TIMEOUT * 10);
  if (n < 0) {
    /* Rate-limit the error log + sleep so a fast-failing rc (e.g. NO_DEVICE
     * after the chip drops off USB) can't spin the RX loop at full CPU. */
    static uint64_t err_count = 0;
    if ((err_count++ % 100) == 0)
      _logger->error("bulk_read_raw failed with error: {} (count={})", n,
                     err_count);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    n = 0;
  }
  FrameParser fp{_logger};
  return fp.recvbuf2recvframe(std::span<uint8_t>{buffer, (size_t)n});
}

void RtlJaguarDevice::Init(Action_ParsedRadioPacket packetProcessor,
                          SelectedChannel channel) {
  _packetProcessor = packetProcessor;

  StartWithMonitorMode(channel);
  SetMonitorChannel(channel);

  _logger->info("Listening air...");
  /* Keep several bulk-IN transfers in flight at once (mirrors the kernel's ~4
   * always-posted RX URBs — confirmed via usbmon: rtw88 re-submits each URB
   * ~20us after completion and cycles 4 of them). With a single synchronous
   * transfer there is a window between transfers where NO URB is posted; the
   * 8814's RX-DMA ring pauses in that gap and, under sparse traffic,
   * intermittently wedges after a short burst — the "RX stalls at ~10 frames"
   * bug (8812/8821 tolerate it, the 8814 does not). read_frames() is
   * reentrant (local buffer + local FrameParser; libusb is thread-safe), so a
   * small worker pool restores the always-posted behaviour. Frame order across
   * workers is not preserved, which is fine for monitor-mode RX. Set
   * DEVOURER_RX_URBS=1 to restore the old single-transfer behaviour. */
  int rx_workers = 4;
  if (const char *e = std::getenv("DEVOURER_RX_URBS")) {
    rx_workers = std::atoi(e);
    if (rx_workers < 1) rx_workers = 1;
  }
  std::mutex proc_mu;
  std::vector<std::thread> workers;
  for (int i = 0; i < rx_workers; ++i) {
    workers.emplace_back([this, &proc_mu]() {
      while (!should_stop && !g_devourer_should_stop) {
        auto packets = read_frames();
        if (packets.empty())
          continue;
        std::lock_guard<std::mutex> lk(proc_mu);
        for (auto &p : packets) {
          if (should_stop || g_devourer_should_stop)
            break;
          _packetProcessor(p);
        }
      }
    });
  }
  for (auto &t : workers)
    t.join();

#if 0
  _device.UsbDevice.SetBulkDataHandler(BulkDataHandler);
  _readTask = Task.Run(() = > _device.UsbDevice.InfinityRead());
#endif
}

void RtlJaguarDevice::SetMonitorChannel(SelectedChannel channel) {
  /* Keep the device-level channel state current: send_packet's 5GHz
   * CCK->OFDM clamp keys off _channel.Channel. Before this assignment
   * existed, _channel was never written anywhere — the clamp read an
   * uninitialised member and fired nondeterministically. */
  _channel = channel;
  _radioManagement->set_channel_bwmode(channel.Channel, channel.ChannelOffset,
                                       channel.ChannelWidth);
}

void RtlJaguarDevice::FastRetune(uint8_t channel, bool cache_rf) {
  if (_radioManagement->fast_retune(channel, cache_rf)) {
    _channel.Channel = channel;
    return;
  }
  /* Fast path declined (band change / non-20MHz) — do the full channel set,
   * preserving the current bandwidth + offset. */
  SetMonitorChannel(SelectedChannel{.Channel = channel,
                                    .ChannelOffset = _channel.ChannelOffset,
                                    .ChannelWidth = _channel.ChannelWidth});
}

void RtlJaguarDevice::StartWithMonitorMode(SelectedChannel selectedChannel) {
  if (NetDevOpen(selectedChannel) == false) {
    throw std::ios_base::failure("StartWithMonitorMode failed NetDevOpen");
  }

  _radioManagement->SetMonitorMode();
}

void RtlJaguarDevice::SetTxPower(uint8_t power) {
  _radioManagement->SetTxPower(power);
}

void RtlJaguarDevice::SetTxPowerOverride(int idx) {
  _radioManagement->SetTxPowerOverride(idx);
}

void RtlJaguarDevice::ApplyTxPower() { _radioManagement->ApplyTxPower(); }

void RtlJaguarDevice::SetTxMode(const devourer::TxMode& mode) {
  _tx_mode_default = mode;
}

void RtlJaguarDevice::ClearTxMode() { _tx_mode_default.reset(); }

uint32_t RtlJaguarDevice::ReadBBReg(uint16_t addr, uint32_t mask) {
  return _radioManagement->phy_query_bb_reg_public(addr, mask);
}

bool RtlJaguarDevice::NetDevOpen(SelectedChannel selectedChannel) {
  auto status = _halModule.rtw_hal_init(selectedChannel);
  if (status == false) {
    return false;
  }

  return true;
}

RtlJaguarDevice::~RtlJaguarDevice() {
  _qd_stop.store(true);
  if (_qd_thread.joinable()) {
    _qd_thread.join();
  }
  _therm_stop.store(true);
  if (_therm_thread.joinable()) {
    _therm_thread.join();
  }
}

void RtlJaguarDevice::start_queue_depth_poller(uint32_t interval_ms) {
  if (interval_ms == 0) return;
  if (_qd_thread.joinable()) {
    _logger->warn("queue-depth poller already running");
    return;
  }
  if (_eepromManager->version_id.ICType != CHIP_8814A) {
    _logger->warn(
        "DEVOURER_QUEUE_POLL_MS set but chip is not 8814A — REG_FIFOPAGE_INFO_*"
        " registers don't decode as per-queue free pages on this chip; poller"
        " disabled");
    return;
  }
  _qd_thread = std::thread([this, interval_ms]() {
    /* libusb-1.0 allows concurrent transfers on different endpoints from
     * different threads — vendor control (ep 0) doesn't conflict with the
     * RX-loop's bulk-IN. The reads here are synchronous control transfers
     * via _device.rtw_read32, so no completion-callback plumbing needed. */
    while (!_qd_stop.load()) {
      for (size_t i = 0; i < 5; ++i) {
        if (_qd_stop.load()) break;
        uint32_t v = _device.rtw_read32(kFifoPageInfoRegs_8814A[i]);
        _qd_snap[i].store(v, std::memory_order_relaxed);
      }
      /* Sleep in short slices so destruction doesn't block for a full
       * interval after _qd_stop is set. */
      for (uint32_t slept = 0; slept < interval_ms && !_qd_stop.load();
           slept += 50) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
  });
}

std::array<uint32_t, 5> RtlJaguarDevice::get_queue_depth() const {
  std::array<uint32_t, 5> out{};
  for (size_t i = 0; i < 5; ++i) {
    out[i] = _qd_snap[i].load(std::memory_order_relaxed);
  }
  return out;
}

static uint32_t pack_thermal(const ThermalStatus &s) {
  int8_t d = static_cast<int8_t>(
      s.delta > 127 ? 127 : (s.delta < -128 ? -128 : s.delta));
  return (s.valid ? 1u : 0u) | (uint32_t(s.raw) << 8) |
         (uint32_t(s.baseline) << 16) |
         (uint32_t(static_cast<uint8_t>(d)) << 24);
}

static ThermalStatus unpack_thermal(uint32_t v) {
  ThermalStatus s;
  s.valid = (v & 1u) != 0;
  s.raw = static_cast<uint8_t>((v >> 8) & 0xFF);
  s.baseline = static_cast<uint8_t>((v >> 16) & 0xFF);
  s.delta = static_cast<int8_t>((v >> 24) & 0xFF);
  return s;
}

ThermalStatus RtlJaguarDevice::GetThermalStatus() {
  return _radioManagement->ReadThermalStatus();
}

ThermalStatus RtlJaguarDevice::get_thermal_snapshot() const {
  return unpack_thermal(_therm_snap.load(std::memory_order_relaxed));
}

void RtlJaguarDevice::start_thermal_poller(uint32_t interval_ms,
                                           int warn_delta) {
  if (interval_ms == 0) return;
  if (_therm_thread.joinable()) {
    _logger->warn("thermal poller already running");
    return;
  }
  _therm_thread = std::thread([this, interval_ms, warn_delta]() {
    bool warned = false;
    bool baseline_note = false;
    while (!_therm_stop.load()) {
      ThermalStatus s = _radioManagement->ReadThermalStatus();
      _therm_snap.store(pack_thermal(s), std::memory_order_relaxed);
      if (!s.valid) {
        if (!baseline_note) {
          _logger->info(
              "thermal: no EFUSE baseline (0xFF) — reporting raw only "
              "(raw={})",
              unsigned(s.raw));
          baseline_note = true;
        }
      } else if (s.delta >= warn_delta) {
        if (!warned) {
          _logger->warn(
              "thermal: chip running hot ({}) — raw={} baseline={} delta=+{} "
              "(>= {}); TX power tracking backing off, sustained TX may "
              "degrade the PA",
              ThermalBucket(s), unsigned(s.raw), unsigned(s.baseline), s.delta,
              warn_delta);
          warned = true;
        }
      } else {
        warned = false; /* re-arm once it cools back under the threshold */
      }
      /* Sleep in short slices so destruction doesn't block for a full
       * interval after _therm_stop is set. */
      for (uint32_t slept = 0; slept < interval_ms && !_therm_stop.load();
           slept += 50) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
  });
}

uint32_t RtlJaguarDevice::read_bb_dbgport(uint32_t selector) {
  if (!_bb_dbgport) {
    _bb_dbgport = std::make_unique<devourer::BbDbgportReader>(_device, _logger);
  }
  return _bb_dbgport->read_dbgport(selector);
}

bool RtlJaguarDevice::bb_dbgport_wedged() const {
  return _bb_dbgport && _bb_dbgport->is_wedged();
}
