/* LA-mode (phydm "logic analyzer" / ADC-sampling) IQ capture — the debug
 * escape hatch that DMAs a raw baseband sample bus into the top of the TX
 * packet buffer, host-readable through the 0x0140-page + 0x8000 window.
 * An offline FFT of a captured L-LTF is the full per-tone channel estimate
 * H(k), so this turns a dongle into a one-shot per-subcarrier channel
 * sounder (docs/la-capture.md; offline tools: tools/la_decode.py,
 * tools/la_csi.py).
 *
 * Ported from phydm_adc_sampling.c (vendored: reference/rtl88x2bu for the
 * 11AC dialect, reference/rtl88x2cu + rtl88x2eu for JGR3); the dbg-port
 * routing is phydm_debug.c (phydm_set_bb_dbg_port /
 * phydm_release_bb_dbg_port). The MAC half (trigger time, 0x7c0 capture
 * control, poll, TX-buffer readback) is identical across families; the BB
 * half has exactly two register dialects, so one implementation serves all
 * generations — LaRegs carries the per-chip parameters, mirroring
 * NhmReader's NhmRegs pattern.
 *
 * SILICON SUPPORT (PHYDM_IC_SUPPORT_LA_MODE, phydm_pre_define.h): 8814A,
 * 8822B, 8821C (cut B+ for the LA clock), 8822C, 8822E. The 8812A and
 * 8821A are NOT in the macro and have no buffer-geometry case — on those
 * dies la_regs_8812a_experimental() is a probe (expect a poll timeout).
 *
 * BRICK RISK — same class as BbDbgportReader: this pokes the BB dbg-port
 * mux (0x8fc / 0x1c3c) and LA engine registers while the demod may be
 * live. All touched registers are snapshotted at run() entry and restored
 * on every exit path, and chip liveness (SYS_CFG readable) is checked
 * after each phase; a dead read latches _wedged and the instance refuses
 * further register writes. Recovery ladder if RX stalls after a capture:
 *   1. libusb_reset_device (set DEVOURER_SKIP_RESET=0 on next launch)
 *   2. USB port-level usbreset (tests/regress.py style)
 *   3. power-cycle / replug
 * Treat first runs on a new chip as destructive until proven otherwise.
 *
 * THREADING / TX CONTRACT: single-control-thread like every control-plane
 * entry point. run() blocks for up to poll_ms*poll_tries + the readback
 * (two vendor-control reads per 8-byte sample: 16 K samples ~ tens of
 * seconds on USB2). The capture lands in the TOP of the TX packet buffer
 * (above the pages the TX path uses), but do not run a capture concurrent
 * with heavy TX — arm first, keep TX quiesced until readback completes.
 */

#ifndef DEVOURER_LA_CAPTURE_H
#define DEVOURER_LA_CAPTURE_H

#include <cstdint>
#include <vector>

#include "RtlAdapter.h"
#include "logger.h"

namespace devourer {

enum class LaBbDialect : uint8_t {
  Ac11, /* Jaguar1/2: 0x95c engine, 0x8fc dbg-port mux */
  Jgr3, /* Jaguar3: 0x1ce4/0x1cf4 engine, 0x1c3c dbg-port mux */
};

/* Per-chip parameters (buffer geometry per phydm_la_set_buff_mode; HALF
 * mode window = [buf_end - size, buf_end)). */
struct LaRegs {
  LaBbDialect dialect;
  uint32_t buf_start;    /* HALF-mode window start (byte addr in TXFF) */
  uint32_t buf_end;      /* window end (exclusive) */
  bool dbgport_clk;      /* 11AC-2 dbg-port clock gate 0x198c[2:0] */
  bool needs_la_clk;     /* 8821C cut B+: LA clock 0x95c[23] */
  bool full_buff_capable;/* 0x7cc[30] doubles the window (8821C/8822C/E) */
  uint8_t trig_time_unit_lsb; /* 0x7cc trigger-time unit field LSB: 18
                               * (11AC / Jaguar2) or 16 (JGR3) */
};

inline LaRegs la_regs_8814a() {
  return LaRegs{LaBbDialect::Ac11, 0x30000, 0x40000, true, false, false, 18};
}
inline LaRegs la_regs_8822b() {
  return LaRegs{LaBbDialect::Ac11, 0x20000, 0x40000, true, false, false, 18};
}
inline LaRegs la_regs_8821c() {
  /* needs_la_clk: 0x95c[23], skipped by the caller on cut A (vendor
   * phydm_la_clk_en does the same). */
  return LaRegs{LaBbDialect::Ac11, 0x8000, 0x10000, true, true, true, 18};
}
inline LaRegs la_regs_jgr3() { /* 8822C + 8822E */
  return LaRegs{LaBbDialect::Jgr3, 0x20000, 0x40000, false, false, true, 16};
}
/* Probe map for dies the vendor macro excludes (8812A/8821A): 8814A
 * geometry, no dbg-port clock gate (they are 11AC-1). A poll timeout here
 * is the expected "no LA block" answer, not a failure of the plumbing. */
inline LaRegs la_regs_8812a_experimental() {
  return LaRegs{LaBbDialect::Ac11, 0x30000, 0x40000, false, false, false, 18};
}

/* Vendor rt_adcsmp_trig_sel, minus the RF0/RF1 chain taps (reachable via
 * hdr_sel = 9/8 on 11AC if ever needed). */
enum class LaTrigMode : uint8_t {
  BbTrig = 0,     /* trigger on a BB dbg-port bit (trig_sel picks the bit) */
  AdcMacTrig = 1, /* ADC dump triggered by a MAC event (mac_sig) */
  MacDump = 4,    /* pure MAC dbg-port dump (not an IQ capture) */
};

enum class LaMacSig : uint8_t {
  CrcOk = 0,   /* RX frame FCS pass */
  CrcFail = 1, /* RX frame FCS fail */
  Cca = 2,     /* CCA assertion */
  Manual = 3,  /* immediate register trigger (0x7c0[5]) */
};

struct LaParams {
  LaTrigMode trig_mode = LaTrigMode::AdcMacTrig;
  LaMacSig mac_sig = LaMacSig::Manual;
  /* 0=80M 1=40M 2=20M 3=10M 4=5M 5=2.5M 6=1.25M 7=160M */
  uint8_t smp_rate = 2;
  /* 11AC 0x95c[11:8] / JGR3 0x1ce4[5:0] — selects the sampled bus. */
  uint8_t dma_type = 0;
  /* BB dbg-port mux value (11AC: full 0x8fc dword; JGR3: 0x1c3c[19:8]). */
  uint32_t dbg_port = 0;
  /* 11AC dbg-port header mux 0x8f8[25:22] (0 = ofdm_dbg[31:0]). */
  uint8_t hdr_sel = 0;
  /* BbTrig: which dbg-port bit arms the trigger. */
  uint8_t trig_sel = 0;
  uint8_t edge = 0; /* 0 = posedge, 1 = negedge */
  /* Post-trigger sampling time; the rest of the ring is pre-trigger. */
  uint32_t trigger_time_us = 100;
  bool buff_all = false; /* double the window (full_buff_capable only) */
  uint32_t poll_ms = 100;
  uint32_t poll_tries = 20;
  /* Cap readback (8-byte samples read, oldest-first); 0 = whole window. */
  uint32_t max_samples = 0;
};

struct LaResult {
  bool ok = false;
  bool wedged = false;
  bool poll_timeout = false;
  /* 0x7c0 read back all-zero after arm: the LA capture block is not
   * implemented on this die (observed on 8812A — vendor's support macro
   * excludes it). Distinct from poll_timeout (block exists, no trigger). */
  bool no_la_block = false;
  bool round_up = false;    /* ring wrapped (capture filled the window) */
  uint32_t finish_addr = 0; /* 0x7c0[30:16], unit 8 bytes */
  /* Time-ordered (oldest first, wrap unrolled); each entry is one 8-byte
   * LA word, (data_h << 32) | data_l — same order the vendor's
   * "%08x%08x" dump prints. Packing of I/Q inside the word is dma_type-
   * and dialect-dependent (tools/la_decode.py). */
  std::vector<uint64_t> samples;
};

class LaCapture {
 public:
  LaCapture(RtlAdapter device, Logger_t logger, LaRegs regs);

  /* Full one-shot sequence: arm → (trigger) → poll → readback → restore.
   * Safe to call repeatedly; after a wedge every call returns immediately
   * with wedged=true. */
  LaResult run(const LaParams &p);

  bool is_wedged() const { return _wedged; }

 private:
  void set_reg(uint16_t addr, uint32_t mask, uint32_t val);
  void setup_trigger_time(const LaParams &p);
  void setup_bb(const LaParams &p);
  void setup_dbg_port(const LaParams &p);
  void setup_mac(const LaParams &p);
  bool poll(const LaParams &p);
  void readback(const LaParams &p, LaResult &r);
  void snapshot();
  void restore();
  bool is_chip_alive();

  RtlAdapter _device;
  Logger_t _logger;
  LaRegs _regs;
  bool _wedged = false;
  /* run()-scoped snapshot of every register the sequence touches. */
  std::vector<std::pair<uint16_t, uint32_t>> _saved;
};

}  // namespace devourer

#endif /* DEVOURER_LA_CAPTURE_H */
