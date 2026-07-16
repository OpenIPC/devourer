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
#include "SignalStop.h" /* g_devourer_should_stop — set by demo signal handlers */
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
  /* DEVOURER_TX_PWR on Kestrel = the fixed BB TX power in whole dBm (distinct
   * from the Jaguar2 TXAGC-index meaning). Applied at every set_channel. */
  if (_cfg.tx.power_index.has_value())
    _hal.set_default_txpwr_dbm(*_cfg.tx.power_index);
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
     * callback suffices — completing each bulk-IN URB does the recycle. */
    _device.bulk_read_async_loop(
        16384, 8, [](const uint8_t *, int) {},
        [this]() -> bool { return _wp_drain_stop || g_devourer_should_stop; });
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


bool RtlKestrelDevice::BringUpMonitor(SelectedChannel channel) {
  _channel = channel;
  /* hal_start_8852b order (rtl8852b_halinit.c:508): mac_hal_init [pwr ->
   * FWDL -> enable_bb_rf -> sys_init -> trx_init -> intf_init] -> efuse ->
   * BB/RF tables -> channel. */
  if (!PowerOnFwAndTrx(_efuse))
    return false;
  if (!_hal.phy_bb_rf_init(_efuse.rfe_type, _hal.read_cut()))
    return false;
  /* halrf RFK: DAC/ADC DC-offset calibration (vendored halrf DACK + RX-DCK)
   * — once after the BB/RF tables, before the channel set. Removes the ADC DC
   * term the CCA energy detector reads as busy. */
  _hal.dac_cal();
  /* Vendor timing: arm the SER error IMR only after the BB/RF tables. */
  _hal.enable_ser_imr();
  /* set_host_rpr (mac_trx_init tail, after the IMR) — enable the fw's WD/PLE
   * page-release path so sustained TX doesn't stall when the bulk-OUT page pool
   * fills after ~103 frames. */
  _hal.set_host_rpr();
  if (!_hal.set_channel(channel.Channel, channel.ChannelWidth, channel.ChannelOffset, channel.Band))
    return false;
  return true;
}

/* TX-scheduler enablement — split out of BringUpMonitor because sch_tx_en()
 * clears the CCA gates (R_AX_CCA_CFG_0 B_AX_CCA_ALL_EN) to unfreeze the CSMA
 * backoff for injection, and clearing CCA ALSO disables RX energy detection —
 * a pure-monitor RX loop that ran it went deaf (zero bulk-IN completions on
 * every band). Only the TX path (InitWrite) needs it; RX-only keeps CCA on.
 * Carrier-sense TX (CCA on during TX) returns with the RX-DCK/DACK-backed
 * energy-detector calibration. */
void RtlKestrelDevice::EnableTxScheduler() {
  /* mac_port_init (mport.c) — enable the CMAC PORT so the transmit engine has a
   * BSS/PTCL context. Without it the CMAC queues frames but never airs them,
   * and the mgmt bulk-OUT stalls at ~103 (the page pool fills with never-
   * released frames). NO_LINK port-0 is enough for self-sourced injection. */
  _hal.port_init();
  /* Enable the scheduler contention TX queues (R_AX_CTN_TXEN) so mgmt/data
   * frames can actually contend for the medium; also clears the CCA TX gates. */
  _hal.sch_tx_en();
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
  /* TX-only: enable the CMAC port + scheduler contention queues (this clears
   * the CCA gates — see EnableTxScheduler). Kept out of BringUpMonitor so the
   * pure-monitor RX path keeps CCA on and can actually hear frames. */
  _hal.set_cca_on(_cfg.debug.kestrel_cca_on);
  EnableTxScheduler();
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
  /* The rest of _add_role: program the ADDR_CAM entry + CMAC control table for
   * macid 0 (the WD macid our descriptors carry). This is what actually gives
   * the TX engine an antenna path (cctl ntx_path_en) + BSS/rate context, so
   * queued frames air and their PLE pages release — without it the mgmt
   * bulk-OUT stalls deterministically at ~103. SA = the canonical injection
   * source address (examples/tx/main.cpp beacon SA). */
  static const uint8_t kInjectSA[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
  _hal.add_self_sta(kInjectSA, /*macid=*/0);
  _hal.enable_tx_report(kestrel::reg::USR_TX_RPT_MODE_PERIOD, 0, 0);
  /* Diagnostic (DEVOURER_KESTREL_FWLOG): route the fw log to C2H packets to
   * probe whether async packet-C2H reaches the host at all (task #12/#236). */
  if (_cfg.debug.kestrel_fw_log)
    _hal.enable_fw_log_c2h();
  _logger->info("Kestrel: TX ready on ch{} — mgmt ep 0x{:02x} data ep 0x{:02x}",
                channel.Channel, _tx_mgmt_ep, _tx_data_ep);
  /* One-shot thermal snapshot at bring-up (RF 0x42 meter vs efuse baseline) —
   * the PA-heating baseline for the TX-heavy link. */
  {
    const auto t = GetThermalStatus();
    _logger->info("Kestrel thermal: raw={} baseline={} delta={}", t.raw,
                  t.valid ? t.baseline : 0xFF, t.valid ? t.delta : 0);
  }
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
  /* Async bulk-IN ring; walk each aggregate with the 11ax rxd parser. The
   * buffer must hold a full RXAGG aggregate (the 8852C LEN_TH is ~20 KB), or
   * large aggregates overflow/truncate and RX delivery stalls. */
  /* Per-die drv_info granularity (vendor RX_DESC_DRV_INFO_UNIT_8852{B,C}):
   * 8 bytes on the 8852B, 16 on the 8852C — the one RX-descriptor layout
   * divergence between the dies. */
  const uint16_t drv_info_unit =
      _variant == kestrel::ChipVariant::C8852C ? 16 : 8;
  _device.bulk_read_async_loop(
      32768, 8,
      [&, drv_info_unit](const uint8_t *data, int n) {
        uint32_t off = 0;
        while (off + 16 <= static_cast<uint32_t>(n)) {
          kestrel::KestrelRxFrame f;
          if (!kestrel::parse_rx_8852b(data + off, static_cast<size_t>(n) - off,
                                       f, drv_info_unit))
            break;
          if (f.rpkt_type == kestrel::RPKT_TYPE_PPDU && f.payload_len >= 6) {
            /* physts header (halbb physts_hdr_info): byte3 = rssi_avg_td, byte4+
             * = per-path rssi_td, all U(8,1) (RSSI% = dBm+110 = raw>>1). Cache
             * for the following WIFI frame(s). */
            _last_rssi[0] = static_cast<uint8_t>(f.payload[4] >> 1);
            _last_rssi[1] = static_cast<uint8_t>(f.payload[5] >> 1);
          } else if (f.rpkt_type == kestrel::RPKT_TYPE_WIFI && packetProcessor) {
            Packet p{};
            p.RxAtrib.pkt_len = static_cast<uint16_t>(f.payload_len);
            p.RxAtrib.crc_err = f.crc_err;
            p.RxAtrib.icv_err = f.icv_err;
            p.RxAtrib.data_rate = f.rx_rate; /* 9-bit AX code (HE >= 0x180) */
            p.RxAtrib.bw = f.bw;
            p.RxAtrib.ppdu_type = f.ppdu_type; /* 7=HE_SU 8=HE_ERSU */
            p.RxAtrib.ppdu_cnt = f.ppdu_cnt;
            p.RxAtrib.tsfl = f.freerun_cnt;
            p.RxAtrib.rssi[0] = _last_rssi[0];
            p.RxAtrib.rssi[1] = _last_rssi[1];
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
      /* Stop on StopRxLoop() or the demos' SIGINT/SIGTERM flag — the same
       * signal-flag pattern every generation's RX loop honours (without it the
       * harness's `timeout` SIGTERM never unblocks the loop). */
      [this]() -> bool { return _rx_stop || g_devourer_should_stop; });
}

void RtlKestrelDevice::SetMonitorChannel(SelectedChannel channel) {
  _channel = channel;
  _hal.set_channel(channel.Channel, channel.ChannelWidth, channel.ChannelOffset, channel.Band);
}

void RtlKestrelDevice::FastRetune(uint8_t channel, bool /*cache_rf*/) {
  /* Lean path only for a same-band 20 MHz hop (RF channel switch); anything else
   * (band change or wide/narrow BW, where the center + BB config differ) takes
   * the full SetMonitorChannel. */
  const bool same_band = (_channel.Channel <= 14) == (channel <= 14);
  if (!same_band || _channel.ChannelWidth != CHANNEL_WIDTH_20) {
    SelectedChannel c = _channel;
    c.Channel = channel;
    SetMonitorChannel(c);
    return;
  }
  _hal.fast_retune(channel);
  _channel.Channel = channel;
}

void RtlKestrelDevice::FastSetBandwidth(ChannelWidth_t bw) {
  /* Lean path only for the 20 <-> 5/10 MHz narrowband toggle (BB small-BW
   * field, same channel); 40/80 MHz change the center + RF bw so take the full
   * SetMonitorChannel. */
  if (bw != CHANNEL_WIDTH_20 && bw != CHANNEL_WIDTH_5 && bw != CHANNEL_WIDTH_10) {
    SelectedChannel c = _channel;
    c.ChannelWidth = bw;
    SetMonitorChannel(c);
    return;
  }
  _hal.fast_set_bw(bw);
  _channel.ChannelWidth = bw;
}

void RtlKestrelDevice::SetTxMode(const devourer::TxMode &mode) {
  _tx_mode_default = mode;
}
void RtlKestrelDevice::ClearTxMode() { _tx_mode_default.reset(); }

devourer::ThermalStatus RtlKestrelDevice::GetThermalStatus() {
  devourer::ThermalStatus s;
  s.raw = _hal.read_thermal(/*path=*/0); /* RF path A */
  s.baseline = _efuse.thermal_a;
  s.valid = (s.baseline != 0xFF);
  if (s.valid)
    s.delta = static_cast<int>(s.raw) - static_cast<int>(s.baseline);
  return s;
}

void RtlKestrelDevice::handle_c2h(const uint8_t *payload, uint32_t len) {
  namespace r = kestrel::reg;
  if (payload == nullptr || len < 8)
    return;
  /* C2H fwcmd header (8B): dword0 = del_type | cat[1:0] | class[7:2] |
   * func[15:8] | seq (same layout as H2C). Route on class+func. */
  const uint32_t h0 = payload[0] | (payload[1] << 8) | (payload[2] << 16) |
                      (static_cast<uint32_t>(payload[3]) << 24);
  const uint8_t cat = (h0 >> r::H2C_HDR_CAT_SH) & 0x3;
  const uint8_t cls = (h0 >> r::H2C_HDR_CLASS_SH) & 0x3f;
  const uint8_t func = (h0 >> r::H2C_HDR_FUNC_SH) & 0xff;
  /* Diagnostic: every packet-C2H (rpkt_type=10) that reaches the host — proves
   * on-hardware whether the fw emits async packet-C2H at all, and its cat/cls/
   * func, independent of the specific report match below. */
  _logger->debug("Kestrel c2h.rx: cat={} cls=0x{:02x} func=0x{:02x} len={}",
                 cat, cls, func, len);
  /* The C2H class enum differs from H2C: FW_OFLD is 0x1 for C2H (not the H2C
   * 0x9). USR_TX_RPT_INFO arrives as cat=MAC, class=FW_OFLD(0x1), func=0x7. */
  if (cls == r::FWCMD_C2H_CL_FW_OFLD &&
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
  /* RTL8852B/C are 2T2R (halrf loops path A/B). HE MCS via radiotap; STBC/
   * LDPC/SGI supported. Max bandwidth is per-die (vendor bw_sup): the 8852C
   * does 160 MHz — on-air-validated at 5 GHz (SDR MCS7/160) — the 8852B tops
   * at 80 (rtl8852b_halinit.c). NOTE: 6 GHz 160 MHz TX does not radiate on
   * the C8852C (SDR-confirmed 0% duty vs 45% at 6G-80 / 40% at 5G-160); the
   * RF synth locks but the 6G+160 TX-enable path is an un-ported gap — 6 GHz
   * tops out at 80 MHz for TX in practice. */
  return devourer::tx_caps_for_chains(
      2, /*ldpc=*/true, /*sgi=*/true,
      /*bw_max_mhz=*/_variant == kestrel::ChipVariant::C8852C ? 160 : 80);
}

/* Effective BB power is clamped to [0, 23] dBm (=[0, 92] quarter-dB); the
 * 8852B PA saturates well below the s(9,2) field's nominal +63 dBm ceiling. */
static constexpr int16_t kKestrelTxMaxQdb = 23 * 4;
static constexpr int16_t kKestrelTxMinQdb = 0;

devourer::TxPowerCaps RtlKestrelDevice::GetTxPowerCaps() {
  devourer::TxPowerCaps c;
  c.supported = true;
  c.index_max = 0;      /* dBm-target model, not an index */
  c.step_qdb = 1;       /* s(9,2) resolution = 0.25 dB */
  c.step_measured = false;
  const int16_t base = _hal.txpwr_base_qdb();
  c.offset_min_qdb = static_cast<int16_t>(kKestrelTxMinQdb - base);
  c.offset_max_qdb = static_cast<int16_t>(kKestrelTxMaxQdb - base);
  return c;
}

int RtlKestrelDevice::SetTxPowerOffsetQdb(int qdb) {
  /* Clamp so the effective power stays in the PA-valid dBm window, then apply
   * the offset (folded into the base at the current channel). Returns the
   * APPLIED qdB (may differ from the request after clamping). */
  const int16_t base = _hal.txpwr_base_qdb();
  int eff = base + qdb;
  if (eff < kKestrelTxMinQdb) eff = kKestrelTxMinQdb;
  if (eff > kKestrelTxMaxQdb) eff = kKestrelTxMaxQdb;
  const int16_t applied = static_cast<int16_t>(eff - base);
  _hal.set_txpwr_offset_qdb(applied);
  _logger->info("Kestrel: SetTxPowerOffsetQdb({}) -> applied {} qdB "
                "(effective {} dBm)",
                qdb, applied, eff / 4);
  return applied;
}

devourer::TxPowerState RtlKestrelDevice::GetTxPowerState() {
  devourer::TxPowerState s;
  s.valid = true;
  s.flat_index = -1; /* dBm model, no per-rate index */
  s.offset_qdb = _hal.txpwr_offset_qdb();
  s.offset_steps = s.offset_qdb; /* 1 step == 1 qdB here */
  s.saturated_low = _hal.txpwr_effective_qdb() <= kKestrelTxMinQdb;
  s.saturated_high = _hal.txpwr_effective_qdb() >= kKestrelTxMaxQdb;
  return s;
}

uint64_t RtlKestrelDevice::ReadTsf() {
  namespace r = kestrel::reg;
  /* mac_get_tsf (twt.c) band-0 port-0. hi/lo/hi wrap guard against a low-32
   * rollover between the two reads (the vendor reads hi then lo; the guard is
   * the same robustness the Jaguar2/3 ReadTsf uses). */
  uint32_t hi = _device.rtw_read32(r::R_AX_TSFTR_HIGH_P0);
  uint32_t lo = _device.rtw_read32(r::R_AX_TSFTR_LOW_P0);
  const uint32_t hi2 = _device.rtw_read32(r::R_AX_TSFTR_HIGH_P0);
  if (hi2 != hi) {
    hi = hi2;
    lo = _device.rtw_read32(r::R_AX_TSFTR_LOW_P0);
  }
  return (static_cast<uint64_t>(hi) << 32) | lo;
}

bool RtlKestrelDevice::StartBeacon(const uint8_t *beacon, size_t len,
                                   int interval_tu) {
  if (_tx_mgmt_ep == 0) {
    _logger->error("Kestrel: StartBeacon before InitWrite");
    return false;
  }
  if (beacon == nullptr || len == 0) {
    _logger->error("Kestrel: StartBeacon empty beacon");
    return false;
  }
  const uint16_t iv = interval_tu > 0 ? static_cast<uint16_t>(interval_tu) : 100;
  /* OFDM 6M beacon (MAC_AX_OFDM6). bss_color 0 (no HE-BSS coloring). */
  return _hal.start_beacon(beacon, static_cast<uint32_t>(len), iv,
                           /*bss_color=*/0, kestrel::reg::MAC_AX_OFDM6);
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
  c.per_chain_rssi = true; /* per-path RSSI from the PPDU-status physts header */
  c.bw_mask = devourer::bw_mask_for_generation(c.generation);
  if (_variant == kestrel::ChipVariant::C8852C)
    c.bw_mask |= devourer::kBw160; /* 8852C-only (vendor bw_sup BW_CAP_160M) */
  c.per_packet_txpower = false; /* AX power is TSSI, not the J2 descriptor LUT */
  c.narrowband_ok = true; /* 5/10 MHz BB small-BW (SDR-validated); FastSetBandwidth toggle */
  c.fastretune_ok = true; /* lean intra-band 20 MHz retune (FastRetune) */
  /* HE ER SU + DCM: both dies (AX_TXD_DATA_ER/_BW_ER/_DCM in the TX WD; RX
   * classifies via the descriptor ppdu_type nibble). */
  c.he_er_su_ok = true;
  c.hw_rx_timestamp = true;     /* rx freerun_cnt -> RxAtrib.tsfl */
  /* The AX beacon engine (StartBeacon) airs a HW-timed beacon with the live TSF
   * inserted by the MAC at TX — on-air validated on the 8852BU. */
  c.hw_beacon_txtsf = true;
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
  bool rate_from_radiotap = false; /* a per-packet radiotap rate overrides the
                                    * SetTxMode default */
  kestrel::TxRate tr{}; /* defaults: 6M, 20MHz, LGI */
  auto *rth = reinterpret_cast<struct ieee80211_radiotap_header *>(
      const_cast<uint8_t *>(packet));
  struct ieee80211_radiotap_iterator it;
  if (ieee80211_radiotap_iterator_init(&it, rth, rlen, nullptr) == 0) {
    while (ieee80211_radiotap_iterator_next(&it) == 0) {
      switch (it.this_arg_index) {
      case IEEE80211_RADIOTAP_RATE:
        mgn = *it.this_arg;
        rate_from_radiotap = true;
        break;
      case IEEE80211_RADIOTAP_MCS: {
        rate_from_radiotap = true;
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
        rate_from_radiotap = true;
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
        rate_from_radiotap = true;
        /* 802.11ax HE: six LE u16 words data1..data6. Read MCS/coding/STBC
         * (data3), BW+GI+LTF (data5) and NSTS (data6); map to the AX rate
         * (0x180 | (nss-1)<<4 | mcs) and the 3-bit GI/LTF code. */
        auto rd16 = [&](int w) -> uint16_t {
          return static_cast<uint16_t>(it.this_arg[w * 2] |
                                       (it.this_arg[w * 2 + 1] << 8));
        };
        const uint16_t d1 = rd16(0), d3 = rd16(2), d5 = rd16(4), d6 = rd16(5);
        unsigned mcs = (d3 & IEEE80211_RADIOTAP_HE_DATA3_DATA_MCS) >> 8;
        unsigned nss = d6 & IEEE80211_RADIOTAP_HE_DATA6_NSTS;
        if (nss < 1) nss = 1;
        if (nss > 4) nss = 4;
        if (d3 & IEEE80211_RADIOTAP_HE_DATA3_CODING) tr.ldpc = true;
        if (d3 & IEEE80211_RADIOTAP_HE_DATA3_STBC) tr.stbc = true;
        const unsigned bw = d5 & IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC;
        tr.bw = static_cast<uint8_t>(bw > 2 ? 2 : bw); /* 0=20 1=40 2=80 */
        /* HE ER SU (extended range): FORMAT=EXT_SU selects the ER SU PPDU
         * (AX_TXD_DATA_ER); a BW_RU_ALLOC of 106-tone picks the upper-bandwidth
         * ER variant (AX_TXD_DATA_BW_ER). DCM (data3 bit 12) stacks on SU or
         * ER SU. Out-of-spec combos are clamped with one W line rather than
         * handed to the MAC — an unresolvable rate word stalls the scheduler
         * and NAKs the bulk-OUT. */
        if ((d1 & IEEE80211_RADIOTAP_HE_DATA1_FORMAT_MASK) ==
            IEEE80211_RADIOTAP_HE_DATA1_FORMAT_EXT_SU) {
          tr.er = true;
          tr.er_106 = (bw == IEEE80211_RADIOTAP_HE_DATA5_RU_106);
          tr.bw = 0; /* ER SU is a 20 MHz-only format */
          const unsigned mcs_max = tr.er_106 ? 0 : 2; /* 242-tone: MCS0-2 */
          if (nss > 1 || mcs > mcs_max) {
            _logger->warn("Kestrel HE ER SU clamp: nss {}->1 mcs {}->{} ({}-tone)",
                          nss, mcs, mcs > mcs_max ? mcs_max : mcs,
                          tr.er_106 ? 106 : 242);
            nss = 1;
            if (mcs > mcs_max)
              mcs = mcs_max;
          }
        }
        if (d3 & IEEE80211_RADIOTAP_HE_DATA3_DATA_DCM) {
          /* DCM pairs with MCS 0/1/3/4 (0/1 under ER SU) and excludes STBC. */
          const bool dcm_mcs_ok =
              tr.er ? (mcs <= 1) : (mcs <= 1 || mcs == 3 || mcs == 4);
          if (!dcm_mcs_ok) {
            _logger->warn("Kestrel HE DCM dropped: mcs {} not DCM-capable", mcs);
          } else {
            tr.dcm = true;
            if (tr.stbc) {
              _logger->warn("Kestrel HE DCM excludes STBC: dropping STBC");
              tr.stbc = false;
            }
          }
        }
        if (mcs <= 11) {
          tr.rate = static_cast<uint16_t>(0x180 + (nss - 1) * 16 + mcs);
          he = true;
        }
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
  /* No per-packet radiotap rate -> apply the SetTxMode default (DEVOURER_TX_RATE)
   * so a rate-less frame (the demo beacon) airs at the requested rate/BW rather
   * than the 6M/20MHz fallback. Mirrors the Jaguar path. */
  if (!rate_from_radiotap && !he && _tx_mode_default.has_value()) {
    const devourer::TxMode &m = *_tx_mode_default;
    if (m.mode == devourer::TxMode::Mode::HE) {
      /* HE expressed natively (tx_mode_to_params has no HE mapping — it would
       * silently fall back to VHT and the frame would air as a VHT_SU PPDU,
       * bench-caught via the RX ppdu_type nibble). ER/DCM limits were already
       * clamped at parse time; a programmatic SetTxMode is clamped here the
       * same way. */
      unsigned nss = m.he_nss < 1 ? 1 : (m.he_nss > 4 ? 4 : m.he_nss);
      unsigned mcs = m.he_mcs <= 11 ? m.he_mcs : 0;
      tr.bw = m.bw_mhz >= 160 ? 3 : m.bw_mhz >= 80 ? 2 : m.bw_mhz >= 40 ? 1 : 0;
      tr.gi_ltf = m.he_gi_ltf & 0x7;
      tr.ldpc = m.ldpc;
      tr.stbc = m.stbc;
      if (m.he_er) {
        tr.er = true;
        tr.er_106 = (m.he_er == 2);
        tr.bw = 0;
        nss = 1;
        const unsigned mcs_max = tr.er_106 ? 0 : 2;
        if (mcs > mcs_max)
          mcs = mcs_max;
      }
      if (m.he_dcm && (mcs <= 1 || (!m.he_er && (mcs == 3 || mcs == 4)))) {
        tr.dcm = true;
        tr.stbc = false; /* DCM excludes STBC */
      }
      tr.rate = static_cast<uint16_t>(0x180 + (nss - 1) * 16 + mcs);
      he = true;
    } else {
      const devourer::TxParams tp = devourer::tx_mode_to_params(m);
      mgn = tp.fixed_rate;
      tr.bw = tp.bwidth == CHANNEL_WIDTH_40   ? 1
              : tp.bwidth == CHANNEL_WIDTH_80 ? 2
                                              : 0; /* narrowband -> DATA_BW 20 */
      tr.gi_ltf = tp.sgi ? 1 : 0;
      tr.ldpc = tp.ldpc;
      tr.stbc = tp.stbc;
    }
  }
  if (!he)
    tr.rate = kestrel::mgn_to_ax_rate(mgn);
  /* Route by 802.11 frame type: data frames ride the AC0 queue (BULKOUTID3) so
   * they exercise the data power-by-rate path; mgmt/beacon uses the MG0 queue
   * (BULKOUTID0). Falls back to the mgmt ep if the data ep was not resolved. */
  const bool is_data = kestrel::frame_is_data(frame, flen);
  uint8_t ep = _tx_mgmt_ep;
  /* The 8852C data/mgmt TX descriptor is the 32-byte wd_body_t_v1; the 8852B is
   * the 24-byte wd_body_t. */
  const uint32_t wd_len = (_variant == kestrel::ChipVariant::C8852C)
                              ? kestrel::WD_BODY_LEN_V1
                              : kestrel::WD_BODY_LEN;
  auto buf = is_data && _tx_data_ep
                 ? kestrel::build_data_txdesc(frame, flen, tr, 0,
                                              _tx_seq++ & 0xfff, wd_len)
                 : kestrel::build_mgnt_txdesc(frame, flen, tr, 0,
                                              _tx_seq++ & 0xfff, wd_len);
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
