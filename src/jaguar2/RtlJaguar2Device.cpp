#include "RtlJaguar2Device.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <span>
#include <thread>
#include <stdexcept>
#include <utility>
#include <vector>

#include "FrameParserJaguar2.h"
#include "Halrf8822b.h"
#include "RateDefinitions.h"
#include "RxPacket.h"
#include "SignalStop.h" /* g_devourer_should_stop */
extern "C" {
#include "ieee80211_radiotap.h" /* MRateToHwRate + radiotap iterator */
}

/* M0 scaffold: the orchestrator compiles and constructs, but bring-up is not yet
 * wired. Each entry point logs and (for the data-path calls) throws/returns so a
 * premature use is loud rather than silently wrong. Milestones M2..M7 replace
 * these bodies with the ported HalMAC / phydm / halrf sub-modules. */

RtlJaguar2Device::RtlJaguar2Device(RtlUsbAdapter device, Logger_t logger)
    : _device{device}, _logger{logger}, _hal{device, logger},
      _macinit{device, logger}, _fw{device, logger} {}

RtlJaguar2Device::~RtlJaguar2Device() { stop_dig(); }

void RtlJaguar2Device::bring_up(SelectedChannel channel) {
  /* Cold bring-up shared by Init (RX) and InitWrite (TX). Order mirrors the
   * HalMAC _halmac_init_hal / vendor rtl88x2bu flow: pre-init -> power-on ->
   * chip-version -> init_system_cfg -> firmware DLFW -> post-DLFW MAC cfg +
   * USB RX-DMA + BB/RF enable -> BB/AGC/RF phydm tables -> TRX mode -> channel
   * -> LCK -> IQK -> coex WL grant -> enable RX/TX MAC engine. */
  const uint8_t bw = static_cast<uint8_t>(channel.ChannelWidth);
  _macinit.pre_init_system_cfg();
  _hal.power_on();
  _hal.read_chip_version();
  _macinit.init_system_cfg(channel.ChannelWidth, _hal.chip_version().cut);
  /* Firmware download BEFORE trx/queue config (HalMAC order): running init_trx
   * first over-allocates the FIFOPAGE queues and wedges the DLFW bcn-valid. */
  if (!_fw.download_default_firmware())
    throw std::runtime_error("RtlJaguar2Device: firmware DLFW failed");
  _logger->info("RtlJaguar2Device: firmware booted (bw={})", (int)bw);

  if (getenv("DEVOURER_DLFW_ONLY")) {
    /* Stop after DLFW so a from-scratch kernel-write replay (DEVOURER_TX_REPLAY)
     * can reconstruct the ENTIRE post-DLFW init in the kernel's exact order,
     * without devourer's own bring-up first setting any internal latch that an
     * on-top replay can't clear. Tests true sequence parity. */
    _logger->info("RtlJaguar2Device: DLFW_ONLY — skipping devourer init");
    return;
  }

  if (!_macinit.init_mac_cfg(channel.ChannelWidth))
    throw std::runtime_error("RtlJaguar2Device: init_mac_cfg failed");
  _macinit.init_usb_cfg();
  _macinit.enable_bb_rf(true);
  _logger->info("RtlJaguar2Device: MAC cfg + BB/RF enabled");

  uint8_t rfe = _hal.read_efuse_rfe();
  /* The kernel uses rfe_type=3 for the T3U even with an unprogrammed efuse
   * (devourer reads 0xff -> defaults 0). rfe_type selects the BB/RF phydm
   * conditional blocks AND the RFE antenna-switch pins; the wrong variant leaves
   * the TX front-end mis-routed. DEVOURER_RFE=N overrides. */
  if (const char *e = getenv("DEVOURER_RFE"))
    rfe = static_cast<uint8_t>(strtol(e, nullptr, 0));
  _hal.apply_bb_rf_agc_tables(rfe);
  _logger->info("RtlJaguar2Device: PHY tables applied");

  _hal.config_trx_mode(); /* RF mode table + TX/RX antenna-path HW blocks */
  _hal.set_channel_bw(static_cast<uint8_t>(channel.Channel), bw, rfe);
  _hal.do_lck(); /* LC calibration — lock the RF LO */

  if (!getenv("DEVOURER_SKIP_IQK")) {
    jaguar2::Halrf8822b halrf(_device, _logger, _hal.chip_version().cut,
                              _hal.chip_version().rf_2t2r != 0);
    halrf.iqk_trigger(channel.Channel <= 14);
  } else {
    _logger->info("Jaguar2: IQK SKIPPED (DEVOURER_SKIP_IQK)");
  }
  /* Grant the antenna to WLAN (combo chip) — must precede enable. */
  if (!getenv("DEVOURER_SKIP_COEX"))
    _hal.coex_wlan_only();
  else
    _logger->info("Jaguar2: coex WL grant SKIPPED (DEVOURER_SKIP_COEX)");
  _hal.enable_rx(); /* CR MACTX|MACRX + RCR + IGI — enables both TX and RX */
}

void RtlJaguar2Device::Init(Action_ParsedRadioPacket packetProcessor,
                            SelectedChannel channel) {
  _packetProcessor = std::move(packetProcessor);
  _channel = channel;
  bring_up(channel);

  /* RX bring-up register dump (DEVOURER_RX_DEBUG): confirms the enable_rx /
   * channel / RF-mode / coex writes landed. The phydm BB decode counters
   * (0xF04/0xF14/0xF08/0xF48) are hold-type and only advance when the DIG/FA
   * thread pulses their reset, so they are NOT a reliable live-RX signal here —
   * dumped for reference only. */
  if (const char *pf = getenv("DEVOURER_BB_PATCH")) {
    /* Canary bisect: apply "0xADDR 0xVAL" BB-register overrides from a file
     * (kernel golden values) right before RX, to find the decode-critical
     * register(s). */
    FILE *f = fopen(pf, "r");
    if (f) {
      char line[128];
      int n = 0;
      while (fgets(line, sizeof(line), f)) {
        unsigned addr, val;
        if (line[0] == 'A' && sscanf(line + 1, "%x %x", &addr, &val) == 2) {
          _hal.dbg_rf_write(0, addr, val);
          n++;
        } else if (line[0] == 'B' &&
                   sscanf(line + 1, "%x %x", &addr, &val) == 2) {
          _hal.dbg_rf_write(1, addr, val);
          n++;
        } else if (sscanf(line, "%x %x", &addr, &val) == 2) {
          _device.rtw_write32(static_cast<uint16_t>(addr), val);
          n++;
        }
      }
      fclose(f);
      _logger->info("Jaguar2: BB_PATCH applied {} registers from {}", n, pf);
    }
  }
  if (getenv("DEVOURER_BB_DUMP")) {
    /* Full BB/RF register dump in the vendor rtw_proc format for canary diff
     * against the kernel driver (tests/jaguar2_rx_canary.sh). */
    for (uint32_t a = 0x0; a <= 0x7fc; a += 0x10)
      _logger->info("BBDUMP 0x{:04x} 0x{:08x} 0x{:08x} 0x{:08x} 0x{:08x}", a,
                    _device.rtw_read32(a), _device.rtw_read32(a + 4),
                    _device.rtw_read32(a + 8), _device.rtw_read32(a + 12));
    for (uint32_t a = 0x800; a <= 0xffc; a += 0x10)
      _logger->info("BBDUMP 0x{:04x} 0x{:08x} 0x{:08x} 0x{:08x} 0x{:08x}", a,
                    _device.rtw_read32(a), _device.rtw_read32(a + 4),
                    _device.rtw_read32(a + 8), _device.rtw_read32(a + 12));
    for (uint32_t a = 0x1800; a <= 0x1afc; a += 0x10)
      _logger->info("BBDUMP 0x{:04x} 0x{:08x} 0x{:08x} 0x{:08x} 0x{:08x}", a,
                    _device.rtw_read32(a), _device.rtw_read32(a + 4),
                    _device.rtw_read32(a + 8), _device.rtw_read32(a + 12));
    for (uint32_t r = 0x0; r <= 0xff; r++)
      _logger->info("RFDUMP 0x{:02x} A=0x{:05x} B=0x{:05x}", r,
                    _hal.dbg_rf_read(0, r), _hal.dbg_rf_read(1, r));
  }
  if (getenv("DEVOURER_RX_DEBUG")) {
    _logger->info(
        "Jaguar2 RXDBG: CR=0x{:04x} RCR=0x{:08x} RXpath(0x808)=0x{:08x} "
        "RF0A/B(0x0)=0x{:05x}/0x{:05x} RF18A/B=0x{:05x}/0x{:05x} "
        "RFmodeTbl(0xc08/0xe08)=0x{:08x}/0x{:08x}",
        _device.rtw_read16(0x0100), _device.rtw_read32(0x0608),
        _device.rtw_read32(0x0808), _hal.dbg_rf_read(0, 0x0),
        _hal.dbg_rf_read(1, 0x0), _hal.dbg_rf_read(0, 0x18),
        _hal.dbg_rf_read(1, 0x18), _device.rtw_read32(0x0c08),
        _device.rtw_read32(0x0e08));
    /* MAC RX-FIFO occupancy poll (0x0284): RXPKT_NUM[31:24] = packets buffered
     * awaiting DMA, RXDMA_IDLE=BIT17, RXPKT_FULL=BIT16. Distinguishes WMAC-drop
     * (stays 0) from USB-DMA-stall (climbs). Also poll RXFF read/write ptrs
     * 0x0288 (RXFF_RD_PTR)/0x028c-ish via 0x0284 dword. */
    for (int i = 0; i < 10; i++) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      uint32_t r284 = _device.rtw_read32(0x0284);
      _logger->info("Jaguar2 MACRX[{}]: 0x284=0x{:08x} (RXPKT_NUM={} "
                    "RXDMA_IDLE={} FULL={})",
                    i, r284, (r284 >> 24) & 0xff, (r284 >> 17) & 1,
                    (r284 >> 16) & 1);
    }
  }
  /* Start the DIG thread: track IGI to the false-alarm rate so weak signals are
   * caught without an FA storm (a fixed IGI can't span the range). */
  _dig_stop = false;
  if (!getenv("DEVOURER_SKIP_DIG")) {
    const bool dig_dbg = getenv("DEVOURER_RX_DEBUG") != nullptr;
    _dig_thread = std::thread([this, dig_dbg] {
      int tick = 0;
      while (!_dig_stop && !g_devourer_should_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        _hal.dig_step();
        if (dig_dbg && (++tick % 10) == 0)
          _logger->info("Jaguar2 DIG: IGI=0x{:02x} FA={}", _hal.dbg_igi(),
                        _hal.dbg_last_fa());
      }
    });
    _logger->info("RtlJaguar2Device: DIG thread started");
  }

  _logger->info("RtlJaguar2Device: entering RX loop (ch={})", channel.Channel);

  /* RX loop: async bulk-IN URB queue; walk the aggregated 8822B RX descriptors
   * per completion and hand each PSDU to the packet processor. */
  uint64_t frames = 0, reads = 0;
  auto on_data = [&](const uint8_t *data, int n) {
    if (++reads <= 8)
      _logger->info("Jaguar2 RX: completion #{} -> {} bytes", reads, n);
    uint32_t off = 0;
    while (off + jaguar2::RXDESC_SIZE_8822B <= static_cast<uint32_t>(n)) {
      jaguar2::Rx8822bFrame f;
      if (!jaguar2::parse_rx_8822b(data + off, static_cast<size_t>(n) - off, f))
        break;
      if (_packetProcessor) {
        Packet p{};
        p.RxAtrib.pkt_len = static_cast<uint16_t>(f.frame_len);
        p.RxAtrib.crc_err = f.crc_err;
        p.RxAtrib.icv_err = f.icv_err;
        p.RxAtrib.data_rate = f.rx_rate;
        p.RxAtrib.drvinfo_sz = static_cast<uint8_t>(f.drvinfo_size);
        p.RxAtrib.shift_sz = f.shift;
        p.RxAtrib.pkt_rpt_type = RX_PACKET_TYPE::NORMAL_RX;
        p.Data =
            std::span<uint8_t>(const_cast<uint8_t *>(f.frame), f.frame_len);
        _packetProcessor(p);
      }
      ++frames;
      if (getenv("DEVOURER_RX_DEBUG"))
        _logger->info("Jaguar2 RX: frame len={} rate={} crc={}", f.frame_len,
                      f.rx_rate, f.crc_err);
      if (f.next_offset == 0)
        break;
      off += f.next_offset;
    }
  };
  _device.bulk_read_async_loop(32 * 1024, 8, on_data, g_devourer_should_stop);
  stop_dig();
  _logger->info("RtlJaguar2Device: RX loop exited ({} frames, {} reads)", frames,
                reads);
}

void RtlJaguar2Device::stop_dig() {
  _dig_stop = true;
  if (_dig_thread.joinable())
    _dig_thread.join();
}

void RtlJaguar2Device::InitWrite(SelectedChannel channel) {
  _channel = channel;
  /* TX shares the full cold bring-up (config_trx_mode enables the TX antenna
   * paths, enable_rx sets CR MACTXEN). The chip transmits at its
   * efuse/table-calibrated TXAGC; DEVOURER_TX_PWR=0xNN forces a flat reference
   * (SDR-visibility debug knob). */
  bring_up(channel);
  if (getenv("DEVOURER_TX_FWINFO")) {
    /* halmac post-DLFW FW handshake the kernel does (hal_halmac.c
     * _send_general_info + phydm_info) that devourer skipped entirely — sent via
     * the H2C-packet queue (32-byte FW-offload packet, QSEL_H2C_CMD=0x13, bulk-
     * OUT with a TX descriptor), NOT the 8-byte HMEBOX path. These tell the FW
     * the TX boundary + chip/RF/antenna identity; without them the FW's host-TX
     * handling may stay disabled (RX works, BB never keys TX). Format from
     * halmac_fw_88xx.c proc_send_general_info/phydm_info + halmac_fw_offload_
     * h2c_nic.h bit layout. */
    auto send_fw_h2c = [&](const uint8_t pkt[32]) {
      /* Delivery check: REG_H2C_PKT_WRITEADDR(0x10D4) advances when the packet
       * enters the H2C queue; REG_H2C_PKT_READADDR(0x10D0) advances when the FW
       * CONSUMES it. If WRITE doesn't move -> bad QSEL routing; if WRITE moves
       * but READ doesn't -> FW isn't reading the queue (handshake dead). */
      uint32_t w0 = _device.rtw_read32(0x10D4) & 0x3FFFF;
      uint32_t r0 = _device.rtw_read32(0x10D0) & 0x3FFFF;
      std::vector<uint8_t> buf(jaguar2::TXDESC_SIZE_8822B + 32, 0);
      SET_TX_DESC_TXPKTSIZE_8822B(buf.data(), 32);
      SET_TX_DESC_QSEL_8822B(buf.data(), 0x13); /* QSEL_H2C_CMD */
      std::memcpy(buf.data() + jaguar2::TXDESC_SIZE_8822B, pkt, 32);
      jaguar2::cal_txdesc_chksum_8822b(buf.data());
      _device.bulk_send_sync_ep(_device.first_bulk_out_ep(), buf.data(),
                                static_cast<int>(buf.size()), 20);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      uint32_t w1 = _device.rtw_read32(0x10D4) & 0x3FFFF;
      uint32_t r1 = _device.rtw_read32(0x10D0) & 0x3FFFF;
      _logger->info("Jaguar2 H2C-deliver: WPTR {:05x}->{:05x} RPTR {:05x}->{:05x}"
                    " ({})", w0, w1, r0, r1,
                    (r1 != r0) ? "FW CONSUMED" : (w1 != w0) ? "queued, FW IDLE"
                                                            : "NOT queued");
    };
    auto put32 = [](uint8_t *p, uint32_t v) {
      p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
    };
    const uint8_t cut = _hal.chip_version().cut;
    const uint8_t ant = _hal.chip_version().rf_2t2r ? 0x3 : 0x1; /* AB / A */
    /* general_info: sub=0x0D. FW_TX_BOUNDARY = rsvd_fw_txbuf_addr - rsvd_boundary;
     * kernel-observed value is 0x30 (my earlier 0x38 double-counted CPU_INSTR/
     * H2CQ pages). Override with DEVOURER_TXBND if the page alloc changes. */
    uint32_t txbnd = 0x30;
    if (const char *b = getenv("DEVOURER_TXBND")) txbnd = strtol(b, nullptr, 0);
    uint8_t gi[32] = {0};
    put32(gi + 0, 0x01u | (0xFFu << 8) | (0x0Du << 16));
    put32(gi + 4, 12u | (0u << 16)); /* total_len=12, seq=0 */
    put32(gi + 8, txbnd << 16);
    send_fw_h2c(gi);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    /* phydm_info: sub=0x11. Kernel(T3U): rfe_type=3, rf_type=4(1T1R), cut=3,
     * rx_ant=tx_ant=1(A). DEVOURER_PHYDM=0xNN overrides rfe_type; rf_type/ant
     * follow rf_2t2r (force with DEVOURER_FORCE_1T1R). */
    uint8_t rfe_pi = 0x03;
    if (const char *r = getenv("DEVOURER_PHYDM")) rfe_pi = strtol(r, nullptr, 0);
    uint8_t rftype = _hal.chip_version().rf_2t2r ? 0x2 : 0x4;
    uint8_t pi[32] = {0};
    put32(pi + 0, 0x01u | (0xFFu << 8) | (0x11u << 16));
    put32(pi + 4, 16u | (1u << 16)); /* total_len=16, seq=1 */
    put32(pi + 8, static_cast<uint32_t>(rfe_pi) |
                      (static_cast<uint32_t>(rftype) << 8) |
                      (static_cast<uint32_t>(cut) << 16) |
                      (static_cast<uint32_t>(ant) << 24) |
                      (static_cast<uint32_t>(ant) << 28));
    put32(pi + 12, 0u); /* ext_pa=0, package_type=0, mp_mode=0 */
    send_fw_h2c(pi);
    _logger->info("Jaguar2: TX_FWINFO general_info+phydm_info H2C sent (FW "
                  "handshake, QSEL_H2C_CMD)");
  }
  if (getenv("DEVOURER_TX_RSVD")) {
    /* FW reserved-page download — the kernel does this on interface-up (usbmon:
     * FIFOPAGE_CTRL_2 beacon-head arm + bulk-OUT template + bcn-valid). The FW
     * requires its rsvd page before it enables the MAC TX scheduler. This dummy
     * template (null-data + qos-null placeholders) exercises the mechanism but
     * is NOT the real template set — a minimal blob did not unlock TX, so the
     * FW likely needs the true probe_rsp/null/qos_null templates (per
     * _rtw_hal_set_fw_rsvd_page) and/or the iddma copy step. Opt-in for now. */
    std::vector<uint8_t> tmpl(256, 0);
    tmpl[0] = 0x48;   /* null-data FC */
    tmpl[128] = 0x88; /* qos-null FC */
    tmpl[129] = 0x01;
    uint16_t pg = _macinit.rsvd_boundary();
    bool ok = _fw.download_rsvd_page(pg, tmpl.data(),
                                     static_cast<uint32_t>(tmpl.size()));
    _logger->info("Jaguar2: FW rsvd-page download {} (pg={})",
                  ok ? "OK" : "FAIL", pg);
  }
  if (getenv("DEVOURER_NO_DROPDATA")) {
    /* Clear BIT_DROP_DATA_EN (0x020C[9], set by init_usb_cfg). The MAC drops TX
     * frames whose length fails the desc OFFSET/size check; a mismatch would
     * silently prevent the BB from ever keying TX. Test knob. */
    uint16_t v = _device.rtw_read16(0x020C);
    _device.rtw_write16(0x020C, v & ~(1u << 9));
    _logger->info("Jaguar2: DROP_DATA_EN cleared (0x020C=0x{:04x})",
                  v & ~(1u << 9));
  }
  if (getenv("DEVOURER_TX_H2C")) {
    /* Send the 2 phydm FW H2C the kernel issues at monitor-up (usbmon golden):
     * element 0x4c = PHYDM_H2C_FW_GENERAL_INIT, via the HMEBOX box protocol
     * (poll REG_HMETFR box0 free -> write HMEBOX_EXT0 0x1f0 -> write HMEBOX0
     * 0x1d0, which triggers the FW read). The prior register-replay failed
     * because it wrote both back-to-back with no box-free poll, so the 2nd
     * clobbered the 1st before the FW consumed it. */
    auto send_h2c = [&](uint32_t box0, uint32_t ext) {
      for (int i = 0; i < 100; i++) {
        if ((_device.rtw_read8(0x01cc) & 0x1) == 0)
          break; /* REG_HMETFR bit0 = box0 busy */
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      _device.rtw_write32(0x01f0, ext);  /* HMEBOX_EXT0 (h2c[4..7]) */
      _device.rtw_write32(0x01d0, box0); /* HMEBOX0 (h2c[0..3]) triggers */
    };
    send_h2c(0x0300034c, 0x00000011);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    send_h2c(0x0302034c, 0x00000033);
    _logger->info("Jaguar2: TX_H2C FW_GENERAL_INIT (0x4c) x2 sent via HMEBOX");
  }
  if (getenv("DEVOURER_TX_DEBUG")) {
    _logger->info("Jaguar2 TXstate: CR(0x100)=0x{:04x} TXPAUSE(0x522)=0x{:02x} "
                  "TXDMA_OFFCHK(0x20c)=0x{:04x}",
                  _device.rtw_read16(0x0100), _device.rtw_read8(0x0522),
                  _device.rtw_read16(0x020C));
    /* TSF-tick check: read the free-running TSF timer (0x560/0x564) twice with a
     * 5 ms gap. If it does not advance, the MAC timing engine is frozen and the
     * EDCA backoff can never count down -> TX never scheduled (RX unaffected). */
    uint32_t tsf0 = _device.rtw_read32(0x0560);
    uint32_t tsfh0 = _device.rtw_read32(0x0564);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    uint32_t tsf1 = _device.rtw_read32(0x0560);
    uint32_t tsfh1 = _device.rtw_read32(0x0564);
    _logger->info("Jaguar2 TSF: {:08x}{:08x} -> {:08x}{:08x} ({})", tsfh0, tsf0,
                  tsfh1, tsf1,
                  (tsf0 != tsf1 || tsfh0 != tsfh1) ? "TICKING" : "FROZEN");
  }
  if (const char *rfp = getenv("DEVOURER_RF_REPLAY")) {
    /* Replay kernel RF-register state via the 3-wire LSSI ("path addr val"
     * lines). RF writes go through BB LSSI regs (0xc90/0xe90), so RF state is
     * invisible to the vendor-reg-write set diff and was never replayed. RF
     * 0x08 reads 0 on devourer vs 0x9c060 on the kernel — a whole RF register
     * that may gate the TX path. */
    FILE *f = fopen(rfp, "r");
    if (f) {
      unsigned path, addr, val;
      int n = 0;
      while (fscanf(f, "%x %x %x", &path, &addr, &val) == 3) {
        _hal.dbg_rf_write(static_cast<uint8_t>(path), addr, val);
        n++;
      }
      fclose(f);
      _logger->info("Jaguar2: RF_REPLAY applied {} RF writes", n);
    }
  }
  if (const char *rf = getenv("DEVOURER_TX_REPLAY")) {
    /* Replay the kernel's captured TX-enable write sequence (usbmon golden):
     * "0xREG WIDTH 0xVAL" lines, applied in order at the native width. */
    FILE *f = fopen(rf, "r");
    if (f) {
      unsigned reg, w, val;
      int n = 0;
      while (fscanf(f, "%x %u %x", &reg, &w, &val) == 3) {
        if (w == 1)
          _device.rtw_write8(static_cast<uint16_t>(reg), static_cast<uint8_t>(val));
        else if (w == 2)
          _device.rtw_write16(static_cast<uint16_t>(reg), static_cast<uint16_t>(val));
        else
          _device.rtw_write32(static_cast<uint16_t>(reg), val);
        n++;
      }
      fclose(f);
      _logger->info("Jaguar2: TX_REPLAY applied {} kernel writes", n);
    }
  }
  if (const char *e = getenv("DEVOURER_TX_PWR")) {
    uint8_t idx = static_cast<uint8_t>(strtol(e, nullptr, 0) & 0x3f);
    _hal.set_tx_power_flat(idx);
  }
  if (getenv("DEVOURER_TX_DRAIN")) {
    /* Drain bulk-IN (FW C2H reports) during TX. The RX demo submits async
     * bulk-IN URBs; the TX demo never reads bulk-IN, so the FW's C2H buffer can
     * back up and stall the FW's TX flow-control (RX works, TX never keys the
     * BB). This thread continuously reads + discards bulk-IN to keep C2H space
     * free — the jaguar2 analogue of the Jaguar3 coex_runtime C2H drain. */
    _dig_thread = std::thread([this] {
      std::vector<uint8_t> buf(16 * 1024);
      while (!g_devourer_should_stop) {
        _device.bulk_read_raw(buf.data(), static_cast<int>(buf.size()), 20);
      }
    });
    _logger->info("Jaguar2: TX_DRAIN bulk-IN C2H drain thread started");
  }
  if (getenv("DEVOURER_TX_DEBUG")) {
    /* Post-replay chip-liveness: TSF advancing => the (possibly replayed) init
     * produced a functional MAC. Distinguishes "chip dead / replay incomplete"
     * from "chip live but TX still won't key". */
    uint32_t a = _device.rtw_read32(0x0560);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    uint32_t b = _device.rtw_read32(0x0560);
    _logger->info("Jaguar2 TSF(post-init): {:08x}->{:08x} ({}) CR=0x{:04x}", a, b,
                  a != b ? "TICKING" : "FROZEN", _device.rtw_read16(0x0100));
  }
  _logger->info("Jaguar2: ready for TX (monitor inject, ch={})",
                channel.Channel);
}

void RtlJaguar2Device::SetMonitorChannel(SelectedChannel channel) {
  _channel = channel;
}

void RtlJaguar2Device::SetTxPower(uint8_t power) { _tx_pwr_override = power; }

bool RtlJaguar2Device::send_packet(const uint8_t *packet, size_t length) {
  /* Parse the radiotap header for rate/bw/coding, build an 8822B TX descriptor
   * (48 B) + the 802.11 frame, and synchronously bulk-OUT. The TX antenna paths
   * are enabled during bring_up, so frames go on-air at the tuned channel.
   * Ported from RtlJaguar3Device::send_packet (shared halmac 88xx descriptor). */
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
  bool rate_from_radiotap = false;

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
      rate_from_radiotap = true;
      break;
    case IEEE80211_RADIOTAP_MCS: {
      uint8_t mcs_flags = it.this_arg[1];
      if ((mcs_flags & IEEE80211_RADIOTAP_MCS_BW_MASK) ==
          IEEE80211_RADIOTAP_MCS_BW_40)
        bwidth = CHANNEL_WIDTH_40;
      sgi = (mcs_flags & 0x04) ? 1 : 0;
      if (it.this_arg[0] & IEEE80211_RADIOTAP_MCS_HAVE_MCS) {
        uint8_t idx = it.this_arg[2];
        if (idx <= 31) {
          fixed_rate = MGN_MCS0 + idx;
          rate_from_radiotap = true;
        }
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
        auto b = it.this_arg[3] & 0x1f;
        if (b >= 1 && b <= 3)
          bwidth = CHANNEL_WIDTH_40;
        else if (b >= 4 && b <= 10)
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
        rate_from_radiotap = true;
      }
    } break;
    default:
      break;
    }
  }

  /* Rate-less frame -> apply the SetTxMode default; per-packet radiotap wins. */
  if (!rate_from_radiotap && _tx_mode_default.has_value()) {
    const devourer::TxParams tp = devourer::tx_mode_to_params(*_tx_mode_default);
    fixed_rate = tp.fixed_rate;
    vht = tp.vht;
    sgi = tp.sgi ? 1 : 0;
    ldpc = tp.ldpc ? 1 : 0;
    stbc = tp.stbc ? 1 : 0;
    bwidth = static_cast<ChannelWidth_t>(tp.bwidth);
  }

  uint8_t bw_desc = (bwidth == CHANNEL_WIDTH_40)   ? 1
                    : (bwidth == CHANNEL_WIDTH_80) ? 2
                                                   : 0;
  uint8_t rate_id = vht ? 9 : 8;
  const uint8_t *dot11 = packet + radiotap_length;
  bool bmc = frame_len >= 6 && (dot11[4] & 0x01);

  /* 802.11 header length for the descriptor WHEADER_LEN field: base 24 B, +2
   * for QoS data, +6 for 4-address (ToDS+FromDS). */
  uint8_t hdrlen = 24;
  if (frame_len >= 2) {
    uint8_t fc0 = dot11[0], fc1 = dot11[1];
    if ((fc0 & 0x0c) == 0x08 && (fc0 & 0x80))
      hdrlen += 2; /* QoS data */
    if ((fc1 & 0x03) == 0x03)
      hdrlen += 6; /* 4-address */
  }

  std::vector<uint8_t> usb_frame(jaguar2::TXDESC_SIZE_8822B + frame_len, 0);
  jaguar2::fill_data_tx_desc_8822b(
      usb_frame.data(), static_cast<uint16_t>(frame_len),
      MRateToHwRate(fixed_rate), rate_id, bw_desc, sgi != 0, ldpc != 0, stbc,
      bmc, static_cast<uint8_t>(hdrlen >> 1));
  std::memcpy(usb_frame.data() + jaguar2::TXDESC_SIZE_8822B, dot11, frame_len);

  /* Test knob: override QSEL (default 0x12 MGNT). The mgmt/HQ queue may not be
   * serviced by the scheduler in monitor mode; a data AC (BE=0x00/VO=0x07) uses
   * the EDCA path. Re-fill the QSEL field + recompute the desc checksum. */
  if (const char *q = getenv("DEVOURER_TX_QSEL")) {
    uint8_t qsel = static_cast<uint8_t>(strtol(q, nullptr, 0) & 0x1f);
    SET_TX_DESC_QSEL_8822B(usb_frame.data(), qsel);
    if (const char *rid = getenv("DEVOURER_TX_RATEID"))
      SET_TX_DESC_RATE_ID_8822B(usb_frame.data(),
                                strtol(rid, nullptr, 0) & 0x1f);
    if (const char *mid = getenv("DEVOURER_TX_MACID"))
      SET_TX_DESC_MACID_8822B(usb_frame.data(), strtol(mid, nullptr, 0) & 0x7f);
    jaguar2::cal_txdesc_chksum_8822b(usb_frame.data());
  }

  uint8_t tx_ep = _device.first_bulk_out_ep();
  if (const char *e = getenv("DEVOURER_TX_EP"))
    tx_ep = static_cast<uint8_t>(strtol(e, nullptr, 0));
  int rc = _device.bulk_send_sync_ep(tx_ep, usb_frame.data(), usb_frame.size(),
                                     /*timeout_ms=*/20);
  /* TX diagnostic (DEVOURER_TX_DEBUG): every 400 frames, read the BB TX-enable
   * counters (0x2de0 OFDM txen/txon, 0x2de4 CCK txen/txon). Non-zero => the BB
   * is keying TX (RF-radiate issue is downstream); zero => the frame is dropped
   * before reaching the BB TX (descriptor/queue/MAC issue). */
  if (getenv("DEVOURER_TX_DEBUG")) {
    static int c = 0;
    if ((++c % 60) == 0) {
      uint32_t ofdm = _device.rtw_read32(0x2de0);
      uint32_t cck = _device.rtw_read32(0x2de4);
      _logger->info("Jaguar2 TXDBG[{}]: OFDM(txen={} txon={}) CCK(txen={} "
                    "txon={})",
                    c, ofdm & 0xffff, ofdm >> 16, cck & 0xffff, cck >> 16);
    }
  }
  return rc >= 0;
}

SelectedChannel RtlJaguar2Device::GetSelectedChannel() { return _channel; }

void RtlJaguar2Device::Stop() { stop_dig(); }

void RtlJaguar2Device::SetTxMode(const devourer::TxMode &mode) {
  _tx_mode_default = mode;
}

void RtlJaguar2Device::ClearTxMode() { _tx_mode_default.reset(); }
