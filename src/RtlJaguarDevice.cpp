#include "RtlJaguarDevice.h"
#include "EepromManager.h"
#include "RadioManagementModule.h"

#include <chrono>
#include <cstdlib>

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
      break;

    case IEEE80211_RADIOTAP_TX_FLAGS:
      txflags = get_unaligned_le16(iterator.this_arg);
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

      /* DEVOURER_TX_HT_MCS=1: honour the HT MCS index from radiotap byte 2
       * and set fixed_rate accordingly. Without this knob the historic
       * behaviour kicks in — fixed_rate stays at the MGN_1M default and the
       * chip transmits 1 Mbps CCK regardless of the HT-MCS field (see PR
       * #80's README: "send_packet only wires fixed_rate from the radiotap
       * RATE/VHT fields — never the HT MCS index"). Gated because flipping
       * this unconditionally would silently change the regression matrix's
       * rate sweeps and the precoder PoC's locked-in 6 Mbps OFDM carrier. */
      static const bool ht_mcs_enabled =
          std::getenv("DEVOURER_TX_HT_MCS") != nullptr;
      if (ht_mcs_enabled && (mcs_known & IEEE80211_RADIOTAP_MCS_HAVE_MCS)) {
        uint8_t mcs_index = iterator.this_arg[2];
        if (mcs_index <= 31) {
          fixed_rate = MGN_MCS0 + mcs_index;
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
        if (bw >= 1 && bw <= 3)
          bwidth = 40;
        else if (bw >= 4 && bw <= 10)
          bwidth = 80;
        else
          bwidth = 20;
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
      }
    } break;

    default:
      break;
    }
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

void RtlJaguarDevice::Init(Action_ParsedRadioPacket packetProcessor,
                          SelectedChannel channel) {
  _packetProcessor = packetProcessor;

  StartWithMonitorMode(channel);
  SetMonitorChannel(channel);

  _logger->info("Listening air...");
  while (!should_stop) {
    auto packets = _device.infinite_read();
    for (auto &p : packets) {
      _packetProcessor(p);
    }
  }

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

void RtlJaguarDevice::StartWithMonitorMode(SelectedChannel selectedChannel) {
  if (NetDevOpen(selectedChannel) == false) {
    throw std::ios_base::failure("StartWithMonitorMode failed NetDevOpen");
  }

  _radioManagement->SetMonitorMode();
}

void RtlJaguarDevice::SetTxPower(uint8_t power) {
  _radioManagement->SetTxPower(power);
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

uint32_t RtlJaguarDevice::read_bb_dbgport(uint32_t selector) {
  if (!_bb_dbgport) {
    _bb_dbgport = std::make_unique<devourer::BbDbgportReader>(_device, _logger);
  }
  return _bb_dbgport->read_dbgport(selector);
}

bool RtlJaguarDevice::bb_dbgport_wedged() const {
  return _bb_dbgport && _bb_dbgport->is_wedged();
}
