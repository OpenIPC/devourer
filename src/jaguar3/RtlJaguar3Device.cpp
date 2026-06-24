#include "RtlJaguar3Device.h"

#include <cstring>
#include <utility>
#include <vector>

#include "FrameParser8822c.h"
#include "Jaguar3Common.h"
#include "RadioManagementModule.h" /* MGN_* rate enum (shared across the family) */

extern "C" {
#include "ieee80211_radiotap.h" /* MRateToHwRate + radiotap iterator */
}

RtlJaguar3Device::RtlJaguar3Device(RtlUsbAdapter device, Logger_t logger)
    : _device{device}, _logger{logger}, _hal{device, logger},
      _radioManagement{device, logger} {
  _logger->info("RtlJaguar3Device constructed (Jaguar3 port is WIP — see "
                "docs/jaguar3-bringup.md)");
}

void RtlJaguar3Device::Init(Action_ParsedRadioPacket packetProcessor,
                            SelectedChannel channel) {
  _packetProcessor = std::move(packetProcessor);
  _channel = channel;
  /* TODO(M4): _hal.rtw_hal_init(channel); SetMonitorChannel(channel); then the
   * infinite_read() RX loop (reuse RtlUsbAdapter, parse via FrameParser8822c). */
  jaguar3::jaguar3_todo("RX bring-up (Init)", jaguar3::Milestone::M4_RxFirst);
}

void RtlJaguar3Device::InitWrite(SelectedChannel channel) {
  _channel = channel;
  /* TODO(M5): _hal.rtw_hal_init(channel); SetMonitorChannel(channel). */
  jaguar3::jaguar3_todo("TX bring-up (InitWrite)", jaguar3::Milestone::M5_Tx20MHz);
}

void RtlJaguar3Device::SetMonitorChannel(SelectedChannel channel) {
  _channel = channel;
  _radioManagement.set_channel_bwmode(channel.Channel, channel.ChannelOffset,
                                      channel.ChannelWidth);
}

void RtlJaguar3Device::SetTxPower(uint8_t /*power*/) {
  jaguar3::jaguar3_todo("SetTxPower", jaguar3::Milestone::M5_Tx20MHz);
}

bool RtlJaguar3Device::send_packet(const uint8_t *packet, size_t length) {
  /* Parse the radiotap header (same fields the Jaguar1 path reads) and build an
   * 8822C TX descriptor + bulk-OUT. Reachable only after M2/M5 bring-up
   * (InitWrite throws until then), so this is the real M5 TX path. */
  if (length < sizeof(struct ieee80211_radiotap_header))
    return false;
  uint16_t radiotap_length = get_unaligned_le16(packet + 2);
  if (radiotap_length == 0 || static_cast<size_t>(radiotap_length) >= length)
    return false;
  const size_t frame_len = length - radiotap_length;

  uint8_t fixed_rate = MGN_1M;
  uint8_t sgi = 0, ldpc = 0, stbc = 0;
  ChannelWidth_t bwidth = CHANNEL_WIDTH_20;
  bool vht = (radiotap_length != 0x0d);

  auto *rtap_hdr = reinterpret_cast<struct ieee80211_radiotap_header *>(
      const_cast<uint8_t *>(packet));
  struct ieee80211_radiotap_iterator it;
  int ret = ieee80211_radiotap_iterator_init(&it, rtap_hdr, radiotap_length,
                                             nullptr);
  while (!ret) {
    ret = ieee80211_radiotap_iterator_next(&it);
    if (ret)
      continue;
    switch (it.this_arg_index) {
    case IEEE80211_RADIOTAP_RATE:
      fixed_rate = *it.this_arg;
      break;
    case IEEE80211_RADIOTAP_MCS: {
      uint8_t mcs_known = it.this_arg[0];
      uint8_t mcs_flags = it.this_arg[1];
      if ((mcs_flags & IEEE80211_RADIOTAP_MCS_BW_MASK) ==
          IEEE80211_RADIOTAP_MCS_BW_40)
        bwidth = CHANNEL_WIDTH_40;
      sgi = (mcs_flags & 0x04) ? 1 : 0;
      if (mcs_known & IEEE80211_RADIOTAP_MCS_HAVE_MCS) {
        uint8_t idx = it.this_arg[2];
        if (idx <= 31)
          fixed_rate = MGN_MCS0 + idx;
      }
    } break;
    case IEEE80211_RADIOTAP_VHT: {
      uint8_t known = it.this_arg[0];
      uint8_t flags = it.this_arg[2];
      if ((known & 4) && (flags & 4))
        sgi = 1;
      if ((known & 1) && (flags & 1))
        stbc = 1;
      if (known & 0x40) {
        auto bw = it.this_arg[3] & 0x1f;
        if (bw >= 1 && bw <= 3)
          bwidth = CHANNEL_WIDTH_40;
        else if (bw >= 4 && bw <= 10)
          bwidth = CHANNEL_WIDTH_80;
      }
      if (it.this_arg[8] & 1)
        ldpc = 1;
      unsigned mcs = (it.this_arg[4] >> 4) & 0x0f;
      unsigned nss = it.this_arg[4] & 0x0f;
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

  uint8_t bw_desc = (bwidth == CHANNEL_WIDTH_40)   ? 1
                    : (bwidth == CHANNEL_WIDTH_80) ? 2
                                                   : 0;
  uint8_t rate_id = vht ? 9 : 8;

  std::vector<uint8_t> usb_frame(jaguar3::TXDESC_SIZE_8822C + frame_len, 0);
  jaguar3::fill_data_tx_desc_8822c(
      usb_frame.data(), static_cast<uint16_t>(frame_len),
      MRateToHwRate(fixed_rate), rate_id, bw_desc, sgi != 0, ldpc != 0, stbc);
  std::memcpy(usb_frame.data() + jaguar3::TXDESC_SIZE_8822C,
              packet + radiotap_length, frame_len);

  return _device.send_packet(usb_frame.data(), usb_frame.size());
}

SelectedChannel RtlJaguar3Device::GetSelectedChannel() { return _channel; }
