/* Pure register arithmetic for RtlJaguar3Device::ArmNhmBusy/ReadNhmBusy:
 * the busy-airtime NHM recipe (inc_cca on, inc_tx off, absolute dBm
 * thresholds) composed as FULL dwords from the registers' current values,
 * so the arm path is a batched write group with no masked read-modify-
 * write (JGR3 double-shift gotcha, jaguar3/CLAUDE.md). NhmReader.h's
 * polling readers (masked writes + a sleeping poll) are deliberately not
 * built on this. */
#ifndef DEVOURER_JAGUAR3_NHM_BUSY_MATH_H
#define DEVOURER_JAGUAR3_NHM_BUSY_MATH_H

#include <cstdint>

namespace devourer {
namespace jgr3 {

/* cfg nibble for ctrl [11:8] = (divi<<3)|(inc_tx<<2)|(inc_cca<<1)|ccx_en:
 * count busy (802.11 frames register), exclude our own TX. */
constexpr uint32_t kNhmBusyCfg = 0x3;

struct NhmProgram {
  uint32_t w1e40; /* period [31:16] */
  uint32_t w1e44; /* th0..th3 */
  uint32_t w1e48; /* th4..th7 */
  uint32_t w1e5c; /* th8 at [23:16] */
  /* ctrl: cfg [11:8], th9 [23:16], th10 [31:24], trigger [1] = 0 */
  uint32_t w1e60;
};

/* The full recipe as five complete dwords. Bits outside period / th / cfg /
 * trigger round-trip from cur_*; the trigger is left low (pulse it with
 * nhm_trigger_low/high). */
inline NhmProgram compose_nhm_program(uint32_t cur_1e40, uint32_t cur_1e5c,
                                      uint32_t cur_1e60, const uint8_t th[11],
                                      uint32_t cfg4, uint16_t period) {
  NhmProgram p;
  p.w1e40 = (cur_1e40 & 0x0000ffffu) | (static_cast<uint32_t>(period) << 16);
  p.w1e44 = th[0] | (th[1] << 8) | (th[2] << 16) |
            (static_cast<uint32_t>(th[3]) << 24);
  p.w1e48 = th[4] | (th[5] << 8) | (th[6] << 16) |
            (static_cast<uint32_t>(th[7]) << 24);
  p.w1e5c = (cur_1e5c & ~0x00ff0000u) | (static_cast<uint32_t>(th[8]) << 16);
  p.w1e60 = (cur_1e60 & 0x0000f0fdu) | ((cfg4 & 0xfu) << 8) |
            (static_cast<uint32_t>(th[9]) << 16) |
            (static_cast<uint32_t>(th[10]) << 24);
  return p;
}

/* 0x1e40 with only the period field [31:16] replaced (re-arm path). */
inline uint32_t nhm_with_period(uint32_t w1e40, uint16_t period) {
  return (w1e40 & 0x0000ffffu) | (static_cast<uint32_t>(period) << 16);
}

/* The two halves of the 0->1 trigger pulse on 0x1e60[1] that starts a
 * window. */
inline uint32_t nhm_trigger_low(uint32_t w1e60) { return w1e60 & ~0x2u; }
inline uint32_t nhm_trigger_high(uint32_t w1e60) { return w1e60 | 0x2u; }

struct NhmResult {
  bool ready = false;
  uint8_t buckets[12] = {};
  uint16_t duration = 0;
};

/* Decode a finished window: 0x2d4c[16] = ready, 0x2d4c[15:0] = duration
 * (4 us units), 0x2d40/44/48 = buckets 0..11, LSB-first. */
inline NhmResult parse_nhm_result(uint32_t r2d4c, uint32_t r2d40,
                                  uint32_t r2d44, uint32_t r2d48) {
  NhmResult r;
  r.ready = (r2d4c & (1u << 16)) != 0;
  r.duration = static_cast<uint16_t>(r2d4c & 0xffffu);
  const uint32_t w[3] = {r2d40, r2d44, r2d48};
  for (int i = 0; i < 3; ++i)
    for (int k = 0; k < 4; ++k)
      r.buckets[i * 4 + k] = (w[i] >> (8 * k)) & 0xffu;
  return r;
}

} // namespace jgr3
} // namespace devourer

#endif /* DEVOURER_JAGUAR3_NHM_BUSY_MATH_H */
