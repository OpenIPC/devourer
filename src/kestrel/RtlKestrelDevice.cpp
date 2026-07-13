#include "RtlKestrelDevice.h"

#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

#include "FrameParserKestrel.h"
#include "RadiotapPeek.h"
#include "RateDefinitions.h" /* MGN_* rate enum */
#include "MacRegAx.h"
#include "RxPacket.h"
#include "TxDescKestrel.h"
#include "TxReportKestrel.h"
#include "ieee80211_radiotap.h" /* radiotap iterator + field ids */

namespace {

/* mac_ax get_chip_info() register pair (reference/rtl8852bu
 * phl/hal_g6/mac/mac_ax.c + mac_reg_ax.h). R_AX_SYS_CHIPINFO shares the
 * 0x00FC address with the 11ac SYS_CFG2 dispatch byte — but the value space
 * is the AX D-die id, not the 11ac chip-id. */
constexpr uint16_t kRegSysChipInfo = 0x00FC; /* R_AX_SYS_CHIPINFO */
constexpr uint16_t kRegSysCfg1Hi = 0x00F1;   /* R_AX_SYS_CFG1 + 1 */

const char *die_name(uint8_t die_id) {
  switch (die_id) {
  case 0x50:
    return "8852A";
  case 0x51:
    return "8852B";
  case 0x52:
    return "8852C";
  case 0x54:
    return "8851B";
  default:
    return "unknown";
  }
}

} /* namespace */

RtlKestrelDevice::RtlKestrelDevice(RtlAdapter device, Logger_t logger,
                                   kestrel::ChipVariant variant,
                                   devourer::DeviceConfig cfg)
    : _device{device}, _logger{logger}, _variant{variant},
      _cfg{std::move(cfg)}, _hal{device, logger, variant} {
  /* Confirm the PID-selected variant against the on-chip die id (the PID
   * table is authoritative for dispatch; this catches a mislabeled board or a
   * stale table row before any bring-up write happens). */
  const auto info = ReadChipInfo();
  if (!info.matches(_variant))
    _logger->warn("Kestrel: PID-selected variant {} but die-id 0x{:02x} ({}) "
                  "— check the KestrelUsbIds table row for this adapter",
                  _variant == kestrel::ChipVariant::C8852B ? "8852B" : "8852C",
                  info.die_id, die_name(info.die_id));
  else
    _logger->info("Kestrel: die-id 0x{:02x} ({}), cut {}", info.die_id,
                  die_name(info.die_id), info.cut);
}

RtlKestrelDevice::~RtlKestrelDevice() {
  _rx_stop = true;
  stop_wp_drain();
}

void RtlKestrelDevice::start_wp_drain() {
  if (_wp_drain_thread.joinable())
    return; /* already draining */
  _wp_drain_stop = false;
  _wp_drain_thread = std::thread([this]() {
    /* Consume the bulk-IN so the fw's TX WP-release reports (rpkt_type=7,
     * TX_PD_RELEASE_HOST) recycle the transmitted frames' WD/PLE pages. A no-op
     * callback suffices — completing the bulk-IN URB is what does the recycle. */
    _device.bulk_read_async_loop(
        16384, 8, [](const uint8_t *, int) {}, _wp_drain_stop);
  });
  _logger->info("Kestrel: WP-release drain started (recycles TX pages)");
}

void RtlKestrelDevice::stop_wp_drain() {
  if (!_wp_drain_thread.joinable())
    return;
  _wp_drain_stop = true;
  _wp_drain_thread.join();
}

kestrel::ChipInfo RtlKestrelDevice::ReadChipInfo() {
  kestrel::ChipInfo info{};
  info.die_id = _device.rtw_read8(kRegSysChipInfo);
  info.cut = static_cast<uint8_t>(_device.rtw_read8(kRegSysCfg1Hi) >> 4);
  return info;
}

bool RtlKestrelDevice::PowerOnAndReadEfuse(kestrel::EfuseInfo &out) {
  if (!_hal.power_on())
    return false;
  return _hal.read_efuse(out);
}

bool RtlKestrelDevice::PowerOnEfuseAndFw(kestrel::EfuseInfo &out) {
  /* Vendor order: efuse is processed AFTER the MAC hal init
   * (rtl8852b_halinit.c:556) — FWDL first, then the efuse dump. */
  if (!_hal.power_on())
    return false;
  if (!_hal.download_firmware(_hal.read_cut()))
    return false;
  return _hal.read_efuse(out);
}

bool RtlKestrelDevice::PowerOnFwAndTrx(kestrel::EfuseInfo &out) {
  /* mac_hal_init order (init.c:336): pwr -> [pre-init+FWDL inside
   * download_firmware] -> set_enable_bb_rf -> sys_init -> trx_init ->
   * feat_init (host-side no-op) -> intf_init (usb) -> [efuse after]. */
  if (!_hal.power_on())
    return false;
  if (!_hal.download_firmware(_hal.read_cut()))
    return false;
  _hal.enable_bb_rf();
  _hal.mac_sys_init();
  if (!_hal.trx_dmac_init())
    return false;
  if (!_hal.trx_cmac_rx_init())
    return false;
  _hal.usb_intf_init();
  _hal.fw_err_state("post-mac-hal-init");
  return _hal.read_efuse(out);
}

bool RtlKestrelDevice::PowerOnTrxAndPhy(kestrel::EfuseInfo &out) {
  if (!PowerOnFwAndTrx(out))
    return false;
  return _hal.phy_bb_rf_init(out.rfe_type, _hal.read_cut());
}

void RtlKestrelDevice::not_ported(const char *entry,
                                  const char *milestone) const {
  _logger->error("Kestrel: {} is not ported yet ({})", entry, milestone);
  throw std::runtime_error(std::string("Kestrel ") + entry +
                           " not ported yet (" + milestone + ")");
}

bool RtlKestrelDevice::BringUpMonitor(SelectedChannel channel) {
  _channel = channel;
  /* hal_start_8852b order (rtl8852b_halinit.c:508): mac_hal_init [pwr ->
   * FWDL -> enable_bb_rf -> sys_init -> trx_init -> intf_init] -> efuse ->
   * BB/RF tables -> channel. */
  if (!PowerOnFwAndTrx(_efuse))
    return false;
  if (!_hal.phy_bb_rf_init(_efuse.rfe_type, _hal.read_cut()))
    return false;
  /* Vendor timing: arm the SER error IMR only after the BB/RF tables. */
  _hal.enable_ser_imr();
  /* set_host_rpr (mac_trx_init tail, after the IMR) — enable the fw's WD/PLE
   * page-release path so sustained TX doesn't stall when the bulk-OUT page pool
   * fills after ~103 frames. */
  _hal.set_host_rpr();
  return _hal.set_channel(channel.Channel, channel.ChannelWidth);
}

void RtlKestrelDevice::Init(Action_ParsedRadioPacket packetProcessor,
                            SelectedChannel channel) {
  if (!BringUpMonitor(channel)) {
    _logger->error("Kestrel: monitor bring-up failed");
    return;
  }
  StartRxLoop(std::move(packetProcessor));
}

void RtlKestrelDevice::InitWrite(SelectedChannel channel) {
  /* TX bring-up = the full monitor bring-up (the CMAC init already stands up
   * the scheduler/tmac/trxptcl/ptcl TX path) minus the RX loop. Then resolve
   * the band-0 mgmt bulk-OUT endpoint (BULKOUTID0 = 0th bulk-OUT per
   * get_bulkout_id_8852b). */
  if (!BringUpMonitor(channel)) {
    _logger->error("Kestrel: TX bring-up failed");
    return;
  }
  _tx_mgmt_ep = _device.nth_bulk_out_ep(0); /* B0MG -> BULKOUTID0 */
  _tx_data_ep = _device.nth_bulk_out_ep(3); /* ACH0 -> BULKOUTID3 */
  if (_tx_mgmt_ep == 0) {
    _logger->error("Kestrel: no bulk-OUT endpoint for mgmt TX");
    return;
  }
  /* Register a station-role MACID with the fw (mac_fw_role_maintain, CREATE)
   * so the per-MACID frame-stat engine tracks our injected frames — the
   * linchpin for the USR_TX_RPT report firing (and, later, data TX + power-by-
   * rate). Then enable the per-user TX report (freerun TX-egress timestamps);
   * the C2H reports are decoded in handle_c2h when the RX loop is up. */
  _hal.register_sta_role(0, 0, 0);
  _hal.enable_tx_report(kestrel::reg::USR_TX_RPT_MODE_PERIOD, 0, 0);
  _logger->info("Kestrel: TX ready on ch{} — mgmt ep 0x{:02x} data ep 0x{:02x}",
                channel.Channel, _tx_mgmt_ep, _tx_data_ep);
  /* Start draining the bulk-IN so the fw's per-frame WP-release reports recycle
   * the TX pages (else sustained mgmt TX stalls after ~103 frames). If the
   * caller later runs StartRxLoop, it takes over the bulk-IN (handoff). */
  start_wp_drain();
}

void RtlKestrelDevice::StartRxLoop(Action_ParsedRadioPacket packetProcessor) {
  /* Take the bulk-IN over from the TX WP-release drain (this loop also consumes
   * rpkt_type=7 releases) so the two never read the endpoint concurrently. */
  stop_wp_drain();
  _rx_stop = false;
  _logger->info("Kestrel: starting RX loop on ch{}", _channel.Channel);
  /* Async bulk-IN ring; walk each aggregate with the 11ax rxd parser. */
  int reads = 0;
  _device.bulk_read_async_loop(
      16384, 8,
      [&](const uint8_t *data, int n) {
        if (++reads <= 12)
          _logger->info("Kestrel RX: bulk-IN completion #{} -> {} bytes "
                        "(rxd0=0x{:08x})",
                        reads, n,
                        n >= 4 ? (data[0] | (data[1] << 8) | (data[2] << 16) |
                                  (data[3] << 24))
                               : 0);
        uint32_t off = 0;
        while (off + 16 <= static_cast<uint32_t>(n)) {
          kestrel::KestrelRxFrame f;
          if (!kestrel::parse_rx_8852b(data + off, static_cast<size_t>(n) - off,
                                       f))
            break;
          if (f.rpkt_type == kestrel::RPKT_TYPE_WIFI && packetProcessor) {
            Packet p{};
            p.RxAtrib.pkt_len = static_cast<uint16_t>(f.payload_len);
            p.RxAtrib.crc_err = f.crc_err;
            p.RxAtrib.icv_err = f.icv_err;
            p.RxAtrib.data_rate = static_cast<uint8_t>(f.rx_rate);
            p.RxAtrib.tsfl = f.freerun_cnt;
            p.Data = std::span<uint8_t>(const_cast<uint8_t *>(f.payload),
                                        f.payload_len);
            packetProcessor(p);
          } else if (f.rpkt_type == kestrel::RPKT_TYPE_C2H) {
            handle_c2h(f.payload, f.payload_len);
          }
          if (f.next_offset == 0)
            break;
          off += f.next_offset;
        }
      },
      _rx_stop);
}

void RtlKestrelDevice::SetMonitorChannel(SelectedChannel channel) {
  _channel = channel;
  _hal.set_channel(channel.Channel, channel.ChannelWidth);
}

void RtlKestrelDevice::handle_c2h(const uint8_t *payload, uint32_t len) {
  namespace r = kestrel::reg;
  if (payload == nullptr || len < 8)
    return;
  /* C2H fwcmd header (8B): dword0 = del_type | cat[1:0] | class[7:2] |
   * func[15:8] | seq (same layout as H2C). Route on class+func. */
  const uint32_t h0 = payload[0] | (payload[1] << 8) | (payload[2] << 16) |
                      (static_cast<uint32_t>(payload[3]) << 24);
  const uint8_t cls = (h0 >> r::H2C_HDR_CLASS_SH) & 0x3f;
  const uint8_t func = (h0 >> r::H2C_HDR_FUNC_SH) & 0xff;
  if (cls == r::FWCMD_H2C_CL_FW_OFLD &&
      func == r::FWCMD_C2H_FUNC_USR_TX_RPT_INFO) {
    kestrel::KestrelTxReport rpt;
    if (kestrel::parse_usr_tx_rpt(payload + 8, len - 8, rpt)) {
      /* freerun_first_out is the HW TX-egress timestamp on the same clock the
       * RX parser reports (RxAtrib.tsfl) — the scheduled-TX air-departure time
       * 11ac/Jaguar could not expose. */
      _logger->info("Kestrel tx.report: macid={} egress_freerun=0x{:08x} "
                    "(in=0x{:08x} last_out=0x{:08x}) pend[BE/BK/VI/VO]={}/{}/{}/{}",
                    rpt.macid, rpt.freerun_first_out, rpt.freerun_first_in,
                    rpt.freerun_last_out, rpt.pending_1k[0], rpt.pending_1k[1],
                    rpt.pending_1k[2], rpt.pending_1k[3]);
    }
  }
}

devourer::TxCaps RtlKestrelDevice::GetTxCaps() {
  /* RTL8852B is 2T2R (halrf_8852b loops path A/B), 20/40/80 MHz. HE MCS via
   * radiotap lands with the M5 HE grammar; STBC/LDPC/SGI supported. */
  return devourer::tx_caps_for_chains(2, /*ldpc=*/true, /*sgi=*/true,
                                      /*bw_max_mhz=*/80);
}

devourer::AdapterCaps RtlKestrelDevice::GetAdapterCaps() {
  devourer::AdapterCaps c;
  c.supported = true;
  c.generation = devourer::ChipGeneration::Kestrel;
  c.transport = _device.is_usb() ? "usb" : "pcie";
  c.tx = GetTxCaps();
  c.txpwr = GetTxPowerCaps();
  c.tx_chains = 2; /* 8852B/8852C are 2T2R */
  c.rx_chains = 2;
  c.per_chain_rssi = true; /* FrameParserKestrel fills per-chain rssi */
  c.bw_mask = devourer::bw_mask_for_generation(c.generation);
  c.per_packet_txpower = false; /* AX power is TSSI, not the J2 descriptor LUT */
  c.narrowband_ok = false;      /* 5/10 MHz re-clock not yet ported for Kestrel */
  c.fastretune_ok = false;      /* FastRetune not yet ported for Kestrel */
  c.hw_rx_timestamp = true;     /* rx freerun_cnt -> RxAtrib.tsfl */
  /* hw_beacon_txtsf needs the AX beacon engine (StartBeacon, M5) — false for
   * now. Kestrel's host-visible TX-egress timestamp rides USR_TX_RPT instead. */
  c.hw_beacon_txtsf = false;
  c.xtal_cap_default = _efuse.xtal_cap; /* efuse crystal-cap (no runtime trim wired) */
  devourer::set_standard_freq_ranges(c);

  if (_variant == kestrel::ChipVariant::C8852C) {
    c.chip_name = "RTL8852C";
    c.marketing_names = "RTL8852CU/RTL8832CU";
    c.chip_id = 0x52; /* R_AX_SYS_CHIPINFO die-id */
    c.variant = "C8852C";
  } else {
    c.chip_name = "RTL8852B";
    c.marketing_names = "RTL8852BU/RTL8832BU";
    c.chip_id = 0x51;
    c.variant = "C8852B";
  }
  return c;
}

bool RtlKestrelDevice::send_packet(const uint8_t *packet, size_t length) {
  if (_tx_mgmt_ep == 0) {
    _logger->error("Kestrel: send_packet before InitWrite (TX not up)");
    return false;
  }
  /* devourer convention: packet = [radiotap header][802.11 MPDU]. Strip the
   * radiotap; the AX descriptor carries the TX parameters instead. */
  const uint16_t rlen = devourer::radiotap_hdr_len(packet, length);
  if (rlen == 0 || static_cast<size_t>(rlen) >= length) {
    _logger->error("Kestrel: send_packet bad radiotap (len={}, rtap={})",
                   length, rlen);
    return false;
  }
  const uint8_t *frame = packet + rlen;
  const uint32_t flen = static_cast<uint32_t>(length - rlen);

  /* Radiotap-driven rate/BW: read RATE / MCS / VHT fields into a devourer MGN_*
   * rate + BW + SGI + LDPC/STBC, then map to the AX descriptor encoding. A
   * rate-less frame stays at 6M OFDM. HE rates land in M5. */
  uint8_t mgn = MGN_6M;
  bool he = false; /* HE rate set directly on tr.rate (no MGN_HE code) */
  kestrel::TxRate tr{}; /* defaults: 6M, 20MHz, LGI */
  auto *rth = reinterpret_cast<struct ieee80211_radiotap_header *>(
      const_cast<uint8_t *>(packet));
  struct ieee80211_radiotap_iterator it;
  if (ieee80211_radiotap_iterator_init(&it, rth, rlen, nullptr) == 0) {
    while (ieee80211_radiotap_iterator_next(&it) == 0) {
      switch (it.this_arg_index) {
      case IEEE80211_RADIOTAP_RATE:
        mgn = *it.this_arg;
        break;
      case IEEE80211_RADIOTAP_MCS: {
        uint8_t flags = it.this_arg[1];
        if ((flags & IEEE80211_RADIOTAP_MCS_BW_MASK) ==
            IEEE80211_RADIOTAP_MCS_BW_40)
          tr.bw = 1;
        if (flags & 0x04)
          tr.gi_ltf = 1; /* SGI */
        if (it.this_arg[0] & IEEE80211_RADIOTAP_MCS_HAVE_MCS)
          mgn = static_cast<uint8_t>(MGN_MCS0 + it.this_arg[2]);
      } break;
      case IEEE80211_RADIOTAP_VHT: {
        uint8_t known = it.this_arg[0], flags = it.this_arg[2];
        if ((known & 0x04) && (flags & 0x04))
          tr.gi_ltf = 1;
        if ((known & 0x01) && (flags & 0x01))
          tr.stbc = true;
        if (known & 0x40) {
          uint8_t bw = it.this_arg[3] & 0x1f;
          tr.bw = (bw >= 4) ? 2 : (bw >= 1) ? 1 : 0;
        }
        if (it.this_arg[8] & 0x01)
          tr.ldpc = true;
        unsigned mcs = (it.this_arg[4] >> 4) & 0xf, nss = it.this_arg[4] & 0xf;
        if (nss >= 1 && nss <= 4 && mcs <= 9)
          mgn = static_cast<uint8_t>(MGN_VHT1SS_MCS0 + (nss - 1) * 10 + mcs);
      } break;
      case IEEE80211_RADIOTAP_HE: {
        /* 802.11ax HE: six LE u16 words data1..data6. Read MCS/coding/STBC
         * (data3), BW+GI+LTF (data5) and NSTS (data6); map to the AX rate
         * (0x180 | (nss-1)<<4 | mcs) and the 3-bit GI/LTF code. */
        auto rd16 = [&](int w) -> uint16_t {
          return static_cast<uint16_t>(it.this_arg[w * 2] |
                                       (it.this_arg[w * 2 + 1] << 8));
        };
        const uint16_t d3 = rd16(2), d5 = rd16(4), d6 = rd16(5);
        const unsigned mcs =
            (d3 & IEEE80211_RADIOTAP_HE_DATA3_DATA_MCS) >> 8;
        unsigned nss = d6 & IEEE80211_RADIOTAP_HE_DATA6_NSTS;
        if (nss < 1) nss = 1;
        if (nss > 4) nss = 4;
        if (mcs <= 11) {
          tr.rate = static_cast<uint16_t>(0x180 + (nss - 1) * 16 + mcs);
          he = true;
        }
        if (d3 & IEEE80211_RADIOTAP_HE_DATA3_CODING) tr.ldpc = true;
        if (d3 & IEEE80211_RADIOTAP_HE_DATA3_STBC) tr.stbc = true;
        const unsigned bw = d5 & IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC;
        tr.bw = static_cast<uint8_t>(bw > 2 ? 2 : bw); /* 0=20 1=40 2=80 */
        const unsigned gi =
            (d5 & IEEE80211_RADIOTAP_HE_DATA5_GI) >>
            IEEE80211_RADIOTAP_HE_DATA5_GI_SHIFT;
        const unsigned ltf =
            (d5 & IEEE80211_RADIOTAP_HE_DATA5_LTF_SIZE) >>
            IEEE80211_RADIOTAP_HE_DATA5_LTF_SIZE_SHIFT;
        /* radiotap (GI,LTF) -> enum rtw_gi_ltf 3-bit code. GI 0=0.8 1=1.6 2=3.2;
         * LTF 1=1x 2=2x 3=4x. Defaults to 2xLTF+0.8us (code 3). */
        uint8_t code = 3;
        if (ltf == 3) code = (gi == 2) ? 0 : 1;        /* 4x: 3.2->0, 0.8->1 */
        else if (ltf == 1) code = (gi == 1) ? 4 : 5;   /* 1x: 1.6->4, 0.8->5 */
        else code = (gi == 1) ? 2 : 3;                 /* 2x: 1.6->2, 0.8->3 */
        tr.gi_ltf = code;
      } break;
      default:
        break;
      }
    }
  }
  if (!he)
    tr.rate = kestrel::mgn_to_ax_rate(mgn);
  /* Route by 802.11 frame type: data frames ride the AC0 queue (BULKOUTID3) so
   * they exercise the data power-by-rate path; mgmt/beacon uses the MG0 queue
   * (BULKOUTID0). Falls back to the mgmt ep if the data ep was not resolved. */
  const bool is_data = kestrel::frame_is_data(frame, flen);
  uint8_t ep = _tx_mgmt_ep;
  auto buf = is_data && _tx_data_ep
                 ? kestrel::build_data_txdesc(frame, flen, tr, 0,
                                              _tx_seq++ & 0xfff)
                 : kestrel::build_mgnt_txdesc(frame, flen, tr, 0,
                                              _tx_seq++ & 0xfff);
  if (is_data && _tx_data_ep)
    ep = _tx_data_ep;
  int rc = _device.bulk_send_sync_ep(ep, buf.data(),
                                     static_cast<int>(buf.size()), 1000);
  if (rc < 0 || static_cast<size_t>(rc) != buf.size()) {
    _logger->error("Kestrel: send_packet bulk-OUT ep 0x{:02x} failed (rc={}, "
                   "wanted {})",
                   ep, rc, buf.size());
    return false;
  }
  return true;
}
