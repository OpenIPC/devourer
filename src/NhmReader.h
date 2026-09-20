#ifndef DEVOURER_NHM_READER_H
#define DEVOURER_NHM_READER_H

#include <chrono>
#include <cstdint>
#include <functional>
#include <thread>

#include "NhmEnvMath.h"
#include "RxSense.h"

/* NHM (Noise Histogram Measurement) — a frame-free, hardware in-band
 * power-distribution readout, ported from Realtek's phydm CCX
 * (phydm_ccx.c in each reference tree). The BB autonomously bins received signal
 * power into 12 IGI-referenced buckets over a measurement window; the driver
 * sets 11 thresholds, pulses a trigger, polls a ready bit, and reads back the
 * 12 one-byte bucket counters. Bucket 0 = power below threshold[0], bucket i
 * (1..10) = th[i-1]..th[i], bucket 11 = above threshold[10]. This is the richer
 * companion to the scalar FA/CCA/IGI energy sensor: a coarse spectrum-free power
 * histogram whose mass shifts up when an in-band interferer raises the floor.
 *
 * CLM (Channel Load Measurement) rides the same armed window: it shares the
 * ccx_en bit and the period register with NHM, and differs only in its trigger
 * bit and its own ready/result register. Reading it costs one extra masked
 * write and one extra register read on a window we already paid for, so this
 * reader always arms both.
 *
 * The algorithm is identical across chip generations; only the register map
 * differs (11AC: Jaguar1/2; JGR3: Jaguar3). NhmRegs carries the per-generation
 * addresses so a single implementation serves all three. */
namespace devourer {

struct NhmRegs {
  uint16_t ctrl;      /* [0]=CLM trigger, [1]=NHM trigger, [8]=ccx_en,
                         [11:8]=cfg, [31:16]=th9|th10<<8 */
  uint16_t period;    /* [31:16]=NHM period, [15:0]=CLM period (4us units) */
  uint16_t th0_3;     /* th[0..3], one byte each */
  uint16_t th4_7;     /* th[4..7] */
  uint16_t th8;       /* th[8] at th8_shift */
  uint8_t th8_shift;  /* 0 (11AC, byte0) / 16 (JGR3, byte2) */
  uint16_t ready;     /* [16]=ready, [15:0]=duration */
  uint16_t res0_3;    /* bucket[0..3] */
  uint16_t res4_7;    /* bucket[4..7] */
  uint16_t res8_11;   /* bucket[8..11] */
  uint16_t clm;       /* [16]=CLM ready, [15:0]=CLM busy ticks */
};

/* 11AC map — Jaguar1 (8812/8814/8821AU) and Jaguar2 (8822BU/8821CU). */
inline NhmRegs nhm_regs_11ac() {
  return NhmRegs{0x994, 0x990, 0x998, 0x99c, 0x9a0, 0,
                 0xfb4, 0xfa8, 0xfac, 0xfb0, 0xfa4};
}

/* JGR3 map — Jaguar3 (8822CU/8822EU). */
inline NhmRegs nhm_regs_jgr3() {
  return NhmRegs{0x1e60, 0x1e40, 0x1e44, 0x1e48, 0x1e5c, 16,
                 0x2d4c, 0x2d40, 0x2d44, 0x2d48, 0x2d88};
}

/* Run one CCX measurement window, filling e.nhm[]/nhm_duration/nhm_th[]/
 * nhm_ratio_pct/nhm_env_ratio_pct/valid_nhm, and the e.clm_ fields with
 * valid_clm.
 *   read32(addr)          — read a 32-bit BB register
 *   set_bb(addr,mask,val) — masked BB-register write (phy_set_bb_reg)
 *   igi7                  — current 7-bit IGI (0xC50/0x1d70), the histogram reference
 *   period                — measurement window in 4us units (default 500 = ~2ms)
 * The thresholds follow phydm's NHM_BACKGROUND recipe (IGI-relative), so the
 * buckets are referenced to the receiver's own noise floor, not absolute dBm.
 * CLM needs no such reference: it counts busy ticks, so its ratio is comparable
 * across channels and adapters in a way the NHM numbers are not. */
inline void read_nhm(const NhmRegs& r, uint8_t igi7,
                     const std::function<uint32_t(uint16_t)>& read32,
                     const std::function<void(uint16_t, uint32_t, uint32_t)>& set_bb,
                     RxEnergy& e, uint16_t period = 500) {
  /* Thresholds (phydm_nhm_th_update_chk, NHM_BACKGROUND): unit PWdB U(8,1).
   * th[0] = (igi - CCA_CAP) * 2, th[i] = th[0] + 4*i for i=1..10. */
  constexpr int kCcaCap = 14;
  int base = (static_cast<int>(igi7) - kCcaCap) * 2;
  if (base < 0) base = 0;
  uint8_t th[11];
  for (int i = 0; i < 11; i++) {
    int v = base + 4 * i;
    th[i] = v > 255 ? 255 : static_cast<uint8_t>(v);
  }

  /* Config [11:8] = (divi<<3)|(inc_tx<<2)|(inc_cca<<1)|ccx_en.
   * ccx_en=1, include_cca=1 (count busy so a CW interferer registers),
   * include_tx=0, divider=NHM_CNT_ALL(0) -> 0b0011. */
  set_bb(r.ctrl, 0xf00u, 0x3u);
  set_bb(r.period, 0xffff0000u, static_cast<uint32_t>(period));
  /* CLM shares the period dword (low half) and ccx_en above; give it the same
   * window so the two numbers describe the same slice of time. */
  set_bb(r.period, 0x0000ffffu, static_cast<uint32_t>(period));

  set_bb(r.th0_3, 0xffffffffu,
         th[0] | (th[1] << 8) | (th[2] << 16) | (uint32_t(th[3]) << 24));
  set_bb(r.th4_7, 0xffffffffu,
         th[4] | (th[5] << 8) | (th[6] << 16) | (uint32_t(th[7]) << 24));
  /* set_bb (phy_set_bb_reg -> PHY_SetBBReg8812) shifts the value left by the
   * mask's own bit position, so every value here is passed RELATIVE to its
   * mask — never pre-shifted, or it is shifted twice and the field lands as
   * zero. Matches the vendor, which passes th[8] and the packed th9|th10<<8
   * raw (phydm_nhm_set_th_reg). */
  set_bb(r.th8, 0xffu << r.th8_shift, th[8]);
  set_bb(r.ctrl, 0xffff0000u, th[9] | (uint32_t(th[10]) << 8));

  /* Trigger both engines (pulse each bit 0->1: CLM bit0, NHM bit1). */
  set_bb(r.ctrl, 0x1u, 0);
  set_bb(r.ctrl, 0x2u, 0);
  set_bb(r.ctrl, 0x1u, 1);
  set_bb(r.ctrl, 0x2u, 1);

  /* Poll ready (bit16). Window ~period*4us; cap the wait so a stuck read never
   * stalls the caller (holds a register lock on Jaguar3). */
  bool ready = false;
  for (int i = 0; i < 15 && !ready; i++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (read32(r.ready) & (1u << 16))
      ready = true;
  }
  /* CLM finishes on the same window, but poll it on its own bit — a failed
   * NHM must not be reported as a failed CLM, or vice versa. */
  const uint32_t clm_raw = read32(r.clm);
  if (clm_raw & (1u << 16)) {
    e.clm_result = static_cast<uint16_t>(clm_raw & 0xffff);
    e.clm_period = period;
    e.clm_ratio_pct = ccx_rpt_ratio(e.clm_result, period);
    e.valid_clm = true;
  }

  if (!ready) {
    e.valid_nhm = false;
    return;
  }

  const uint32_t a = read32(r.res0_3), b = read32(r.res4_7),
                 c = read32(r.res8_11);
  const uint32_t words[3] = {a, b, c};
  for (int w = 0; w < 3; w++)
    for (int k = 0; k < 4; k++)
      e.nhm[w * 4 + k] = (words[w] >> (8 * k)) & 0xff;
  e.nhm_duration = static_cast<uint16_t>(read32(r.ready) & 0xffff);
  for (int i = 0; i < 11; i++)
    e.nhm_th[i] = th[i];
  e.valid_nhm = true;

  /* Reduce the raw histogram the way the vendor's ACS does. Both ratios are
   * derived, never hardware state, so a consumer that wants the buckets can
   * still have them. */
  const NhmUtility u = nhm_utility(e.nhm, e.nhm_th);
  if (u.valid) {
    e.nhm_ratio_pct = u.nhm_ratio;
    e.nhm_env_ratio_pct = u.env_ratio;
  } else {
    e.valid_nhm = false;
  }
}


/* CLM alone, armed now and read later.
 *
 * read_nhm() above arms CLM and NHM on ONE window and polls a ready bit for at
 * most 15 ms, so it can neither program nor read a window longer than that.
 * CLM's period is its own field (the low half of the period dword) and reaches
 * 65535 ticks of 4 us, which is long enough to integrate a survey dwell. These
 * two split that into arm and read so the caller owns the window; the state
 * machine that keeps such a window honest is devourer::ClmWindow
 * (src/BusyWindow.h).
 *
 * NHM's half of the period dword and its trigger bit are deliberately not
 * touched: the histogram is referenced to live IGI, so lengthening ITS window
 * would make nhm_env less stable, not more. */
inline void arm_clm_only(
    const NhmRegs &r, uint16_t period,
    const std::function<void(uint16_t, uint32_t, uint32_t)> &set_bb) {
  /* cfg as read_nhm programs it — bit8 of ctrl is ccx_en, which both engines
   * share. */
  set_bb(r.ctrl, 0xf00u, 0x3u);
  set_bb(r.period, 0x0000ffffu, static_cast<uint32_t>(period));
  /* Pulse CLM's trigger (bit0) 0->1. NHM's bit1 is left alone. */
  set_bb(r.ctrl, 0x1u, 0);
  set_bb(r.ctrl, 0x1u, 1);
}

struct ClmRead {
  bool ready = false;  /* the programmed window has completed */
  uint16_t ticks = 0;  /* busy ticks, 4 us each */
};

/* One read of the CLM result register. Non-destructive and latched: the
 * register holds the last COMPLETED window until the next trigger, so two
 * reads without an intervening arm return the same value (measured on
 * Jaguar1/2/3). The caller must therefore know whether its own window has
 * elapsed — an early read returns the PREVIOUS one, not an error. */
inline ClmRead read_clm_only(const NhmRegs &r,
                             const std::function<uint32_t(uint16_t)> &read32) {
  const uint32_t raw = read32(r.clm);
  ClmRead c;
  if (raw & (1u << 16)) {
    c.ready = true;
    c.ticks = static_cast<uint16_t>(raw & 0xffffu);
  }
  return c;
}

}  // namespace devourer

#endif /* DEVOURER_NHM_READER_H */
