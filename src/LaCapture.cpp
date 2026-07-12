#include "LaCapture.h"

#include <chrono>
#include <thread>

namespace devourer {

namespace {
constexpr uint16_t kRegSysCfg = 0x00F0; /* liveness canary (BbDbgportReader) */

/* MAC-side LA block — identical across families
 * (phydm_la_set_mac_iq_dump / phydm_la_get_tx_pkt_buf). */
constexpr uint16_t kRegLaCtrl = 0x07C0; /* [0]=en [1]/[2]=poll [5]=manual
                                         * [7:6]=mac sig [14:8]=trig time
                                         * [30:16]=finish [31]=wrap */
constexpr uint16_t kRegLaMacMask = 0x07C4;
constexpr uint16_t kRegLaMacSig = 0x07C8;
constexpr uint16_t kRegLaConf = 0x07CC;  /* trig-time unit; [30]=full buff */
constexpr uint16_t kRegTxffPage = 0x0140; /* page = 0x780 + (addr>>12) */
constexpr uint16_t kRegTxffDbgEn = 0x0106; /* 0x69 = TXFF debug access */
constexpr uint16_t kTxffWindow = 0x8000;   /* 4 KB readback window */
constexpr uint16_t kRegMacDbgEn = 0x00F4;  /* [16] MAC dbg port enable */
constexpr uint16_t kRegMacDbgSel = 0x0038; /* [23:16] MAC dbg port */

/* 11AC dialect (phydm_la_set_bb, phydm_debug.c dbg-port routing). */
constexpr uint16_t kAcLaEngine = 0x095C; /* [4:0]=trig bit [7:5]=rate
                                          * [11:8]=dma [23]=la clk [31]=edge */
constexpr uint16_t kAcRptUpdate = 0x08B4;  /* [7] update rpt every pkt */
constexpr uint16_t kAcDbgPortSel = 0x08FC; /* selector (dword) */
constexpr uint16_t kAcDbgPortHdr = 0x08F8; /* [25:22] header mux */
constexpr uint16_t kAcDbgPortClk = 0x198C; /* [2:0] dbg-port clock (11AC-2) */

/* JGR3 dialect. */
constexpr uint16_t kJ3LaEngine = 0x1CE4; /* [5:0]=dma [7:6]=mac-phy timing
                                          * [10:8]=rate [17:13]=trig bit
                                          * [26]=edge [27],[31:28]=adv AND */
constexpr uint16_t kJ3LaOn = 0x1CF4;     /* [23] LA mode on */
constexpr uint16_t kJ3RptUpdate = 0x1EB4; /* [23] update rpt every pkt */
constexpr uint16_t kJ3DbgPortSel = 0x1C3C; /* [19:8] selector */
constexpr uint16_t kJ3AdvAnd123 = 0x1CE8;  /* adv-trigger AND1..3 (reset 0) */
constexpr uint16_t kJ3AdvAnd4 = 0x1CF0;    /* adv-trigger AND4 mask */
}  // namespace

LaCapture::LaCapture(RtlAdapter device, Logger_t logger, LaRegs regs)
    : _device{device}, _logger{logger}, _regs{regs} {}

bool LaCapture::is_chip_alive() {
  uint32_t v = _device.rtw_read32(kRegSysCfg);
  if (v == 0 || v == 0xFFFFFFFFu) {
    _logger->error("LaCapture: SYS_CFG read 0x{:08x} — chip wedged. "
                   "Recovery: libusb_reset_device, usbreset, power-cycle.",
                   v);
    return false;
  }
  return true;
}

void LaCapture::set_reg(uint16_t addr, uint32_t mask, uint32_t val) {
  _device.phy_set_bb_reg(addr, mask, val);
}

void LaCapture::snapshot() {
  _saved.clear();
  auto save = [&](uint16_t addr) {
    _saved.emplace_back(addr, _device.rtw_read32(addr));
  };
  save(kRegLaCtrl);
  save(kRegLaMacMask);
  save(kRegLaMacSig);
  save(kRegLaConf);
  save(kRegTxffPage);
  save(kRegMacDbgEn);
  save(kRegMacDbgSel);
  if (_regs.dialect == LaBbDialect::Ac11) {
    save(kAcLaEngine);
    save(kAcRptUpdate);
    save(kAcDbgPortSel);
    save(kAcDbgPortHdr);
    if (_regs.dbgport_clk)
      save(kAcDbgPortClk);
  } else {
    save(kJ3LaEngine);
    save(kJ3LaOn);
    save(kJ3RptUpdate);
    save(kJ3DbgPortSel);
    save(kJ3AdvAnd123);
    save(kJ3AdvAnd4);
  }
}

void LaCapture::restore() {
  /* Stop the LA block first so nothing samples while BB regs move, then
   * put every touched register back (reverse order). 0x0106 goes back to
   * 0 explicitly — the vendor flow leaks it set. */
  _device.rtw_write8(kRegLaCtrl, 0);
  _device.rtw_write8(kRegTxffDbgEn, 0);
  for (auto it = _saved.rbegin(); it != _saved.rend(); ++it)
    _device.rtw_write32(it->first, it->second);
}

void LaCapture::setup_trigger_time(const LaParams &p) {
  /* phydm_la_set_mac_trigger_time: pick unit u so count < 128. */
  uint32_t unit = 0;
  while (unit < 6 && (p.trigger_time_us >> unit) >= 128)
    unit++;
  uint32_t count = (p.trigger_time_us >> unit) & 0x7f;
  set_reg(kRegLaConf, 0x7u << _regs.trig_time_unit_lsb, unit);
  set_reg(kRegLaCtrl, 0x7f00u, count);
}

void LaCapture::setup_bb(const LaParams &p) {
  if (_regs.dialect == LaBbDialect::Ac11) {
    set_reg(kAcDbgPortHdr, 0x3c00000u, p.hdr_sel);
    set_reg(kAcRptUpdate, 1u << 7, 1); /* update rpt every pkt */
    set_reg(kAcLaEngine, 0xf00u, p.dma_type);
    set_reg(kAcLaEngine, 1u << 31, p.edge);
    set_reg(kAcLaEngine, 0xe0u, p.smp_rate);
    if (_regs.needs_la_clk) /* 8821C cut B+ (caller skips cut A) */
      set_reg(kAcLaEngine, 1u << 23, 1);
  } else {
    set_reg(kJ3RptUpdate, 1u << 23, 1); /* update rpt every pkt */
    set_reg(kJ3LaEngine, 0xc0u, 0);     /* MAC-PHY timing */
    set_reg(kJ3LaOn, 1u << 23, 1);      /* LA mode on */
    set_reg(kJ3LaEngine, 0x3fu, p.dma_type);
    set_reg(kJ3LaEngine, 1u << 26, p.edge);
    set_reg(kJ3LaEngine, 0x700u, p.smp_rate);
    /* Advanced AND-gate trigger at pass-through defaults
     * (phydm_la_bb_adv_trig_setting_jgr3, adv table zeroed). */
    set_reg(kJ3LaEngine, 1u << 27, 0);
    set_reg(kJ3LaEngine, 0xf0000000u, 0);
    set_reg(kJ3AdvAnd123, 1u << 5, 0);
    set_reg(kJ3AdvAnd123, 0x3c0u, 0);
    set_reg(kJ3AdvAnd123, 1u << 15, 0);
    set_reg(kJ3AdvAnd123, 0xf0000u, 0);
    set_reg(kJ3AdvAnd123, 1u << 25, 0);
    set_reg(kJ3AdvAnd4, 0xffffffffu, 0);
    set_reg(kJ3AdvAnd123, 1u << 26, 0);
  }
}

void LaCapture::setup_dbg_port(const LaParams &p) {
  /* phydm_set_bb_dbg_port(DBGPORT_PRI_3) + the trig-sel bit. */
  if (_regs.dialect == LaBbDialect::Ac11) {
    if (_regs.dbgport_clk)
      set_reg(kAcDbgPortClk, 0x7u, 0x7);
    _device.rtw_write32(kAcDbgPortSel, p.dbg_port);
  } else {
    set_reg(kJ3DbgPortSel, 0xfff00u, p.dbg_port);
  }
  uint32_t trig_sel =
      (p.trig_mode == LaTrigMode::MacDump) ? 0 : p.trig_sel;
  if (_regs.dialect == LaBbDialect::Ac11)
    set_reg(kAcLaEngine, 0x1fu, trig_sel);
  else
    set_reg(kJ3LaEngine, 0x3e000u, trig_sel);
}

void LaCapture::setup_mac(const LaParams &p) {
  /* phydm_la_set_mac_iq_dump. */
  _device.rtw_write8(kRegLaCtrl, 0);
  set_reg(kRegLaCtrl, 1u << 0, 1); /* LA block on */

  if (p.trig_mode == LaTrigMode::MacDump) {
    set_reg(kRegLaCtrl, 1u << 2, 1); /* poll bit, MAC mode */
    set_reg(kRegLaCtrl, 0x18u, p.edge);
    set_reg(kRegMacDbgEn, 1u << 16, 1);
    set_reg(kRegMacDbgSel, 0xff0000u, p.dbg_port);
    _device.rtw_write32(kRegLaMacMask, 0xffffffffu);
    _device.rtw_write32(kRegLaMacSig, 0);
  } else {
    if (p.trig_mode == LaTrigMode::AdcMacTrig) {
      set_reg(kRegLaCtrl, 1u << 3, 1); /* MAC-event trigger source */
      set_reg(kRegLaCtrl, 0xc0u, static_cast<uint32_t>(p.mac_sig));
    }
    set_reg(kRegLaCtrl, 1u << 1, 1); /* poll bit, BB-ADC mode */
    if (p.trig_mode == LaTrigMode::AdcMacTrig &&
        p.mac_sig == LaMacSig::Manual) {
      /* Fire the register trigger (0x7c0[5] 0->1) only after the dump
       * enable above is armed — the vendor sets it earlier in the same
       * function, but hardware-bisected here: an edge before the poll
       * bit arms is consumed and the capture never completes. */
      set_reg(kRegLaCtrl, 1u << 5, 0);
      set_reg(kRegLaCtrl, 1u << 5, 1);
    }
  }
}

bool LaCapture::poll(const LaParams &p) {
  const uint8_t polling_bit =
      (p.trig_mode == LaTrigMode::MacDump) ? (1u << 2) : (1u << 1);
  for (uint32_t i = 0; i < p.poll_tries; i++) {
    uint8_t v = _device.rtw_read8(kRegLaCtrl);
    _logger->debug("LaCapture: poll[{}] 0x7c0[7:0]=0x{:02x} (wait bit 0x{:02x} "
                   "clear)", i, v, polling_bit);
    if (!(v & polling_bit)) {
      _logger->debug("LaCapture: capture complete after {} polls", i);
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(p.poll_ms));
  }
  return false;
}

void LaCapture::readback(const LaParams &p, LaResult &r) {
  /* phydm_la_get_tx_pkt_buf. */
  uint32_t buf_start = _regs.buf_start;
  const uint32_t buf_end = _regs.buf_end;
  if (p.buff_all && _regs.full_buff_capable)
    buf_start = buf_end - 2 * (buf_end - buf_start);

  _device.rtw_write8(kRegTxffDbgEn, 0x69);
  const uint32_t v = _device.rtw_read32(kRegLaCtrl);
  r.round_up = (v >> 31) & 1;
  r.finish_addr = (v & 0x7FFF0000u) >> 16; /* unit: 8 bytes */

  uint32_t addr;   /* byte address of the oldest sample */
  uint32_t count;  /* samples to read */
  const uint32_t window_samples = (buf_end - buf_start) >> 3;
  if (r.round_up) {
    addr = (r.finish_addr + 2) << 3; /* vendor: "+1 or +2 ??" */
    count = window_samples;
  } else {
    addr = buf_start;
    const uint32_t start8 = buf_start >> 3;
    count = r.finish_addr > start8 ? r.finish_addr - start8
                                   : start8 - r.finish_addr;
  }
  if (count > window_samples)
    count = window_samples;
  if (p.max_samples && count > p.max_samples) {
    /* Keep the NEWEST samples: the trigger event (and the frame that
     * fired it) sits at the finish end of the ring, so a cap that kept
     * the oldest end would drop exactly the content of interest. */
    uint32_t skip = count - p.max_samples;
    addr += skip * 8;
    while (addr >= buf_end)
      addr -= (buf_end - buf_start);
    count = p.max_samples;
  }

  _logger->debug("LaCapture: readback {} samples from 0x{:05x} (finish=0x{:04x}"
                 " wrap={})", count, addr, r.finish_addr, r.round_up);

  r.samples.reserve(count);
  uint32_t page = 0xFFFFFFFFu;
  for (uint32_t i = 0; i < count; i++) {
    if (addr >= buf_end)
      addr = buf_start; /* ring */
    const uint32_t pg = addr >> 12;
    if (pg != page) {
      page = pg;
      _device.rtw_write16(kRegTxffPage, static_cast<uint16_t>(0x780 + pg));
    }
    const uint16_t off = static_cast<uint16_t>(kTxffWindow + (addr & 0xfff));
    const uint32_t data_l = _device.rtw_read32(off);
    const uint32_t data_h = _device.rtw_read32(off + 4);
    r.samples.push_back((static_cast<uint64_t>(data_h) << 32) | data_l);
    addr += 8;
  }
}

LaResult LaCapture::run(const LaParams &p) {
  LaResult r;
  if (_wedged) {
    r.wedged = true;
    return r;
  }

  snapshot();
  if (p.buff_all && _regs.full_buff_capable)
    set_reg(kRegLaConf, 1u << 30, 1);
  else if (_regs.full_buff_capable)
    set_reg(kRegLaConf, 1u << 30, 0);

  setup_trigger_time(p);
  setup_bb(p);
  setup_dbg_port(p);
  setup_mac(p);

  if (!is_chip_alive()) {
    _wedged = true;
    r.wedged = true;
    restore();
    return r;
  }

  /* Post-arm register dump — the LA state the trigger sees. */
  if (_regs.dialect == LaBbDialect::Ac11)
    _logger->debug("LaCapture: armed 0x7c0=0x{:08x} 0x7cc=0x{:08x} "
                   "0x95c=0x{:08x} 0x8b4=0x{:08x} 0x8fc=0x{:08x} "
                   "0x8f8=0x{:08x} 0x198c=0x{:08x}",
                   _device.rtw_read32(kRegLaCtrl),
                   _device.rtw_read32(kRegLaConf),
                   _device.rtw_read32(kAcLaEngine),
                   _device.rtw_read32(kAcRptUpdate),
                   _device.rtw_read32(kAcDbgPortSel),
                   _device.rtw_read32(kAcDbgPortHdr),
                   _device.rtw_read32(kAcDbgPortClk));
  else
    _logger->debug("LaCapture: armed 0x7c0=0x{:08x} 0x7cc=0x{:08x} "
                   "0x1ce4=0x{:08x} 0x1cf4=0x{:08x} 0x1eb4=0x{:08x} "
                   "0x1c3c=0x{:08x}",
                   _device.rtw_read32(kRegLaCtrl),
                   _device.rtw_read32(kRegLaConf),
                   _device.rtw_read32(kJ3LaEngine),
                   _device.rtw_read32(kJ3LaOn),
                   _device.rtw_read32(kJ3RptUpdate),
                   _device.rtw_read32(kJ3DbgPortSel));

  /* The arm sequence set 0x7c0[0]=1 (+poll bit). A die without the LA
   * block reads the whole register back as zero — everything after would
   * be a false-positive read of stale TXFF content. */
  if ((_device.rtw_read32(kRegLaCtrl) & 0x3u) == 0) {
    r.no_la_block = true;
    _logger->warn("LaCapture: 0x7c0 did not latch the arm bits — no LA "
                  "capture block on this die");
    restore();
    return r;
  }

  if (poll(p)) {
    readback(p, r);
    r.ok = !r.samples.empty();
  } else {
    r.poll_timeout = true;
    _logger->warn("LaCapture: polling timeout after {} x {} ms — trigger "
                  "never fired or no LA block on this die",
                  p.poll_tries, p.poll_ms);
  }

  restore();
  if (!is_chip_alive()) {
    _wedged = true;
    r.wedged = true;
    r.ok = false;
  }
  return r;
}

}  // namespace devourer
