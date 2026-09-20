/* BusyWindow — the CCX CLM measurement window as a state machine.
 *
 * IRadio::GetChannelBusy() as shipped reads whatever window the shared NHM
 * read happened to arm: ~2 ms, once. That is a sample, not a measurement of
 * the caller's dwell, and on bursty traffic it is bimodal — measured on an
 * RTL8822BU against a 50 ms-on/450 ms-off interferer, one 2 ms read per 300 ms
 * read ZERO in 55 of 71 windows and ~63% in the rest, while 300-400 frames
 * were decoded in nearly every one of those windows.
 *
 * CLM's period field is its own (the low half of the period dword) and reaches
 * 65535 ticks of 4 us, so the hardware can integrate a whole survey dwell in
 * one window. This type owns that: arm at the point the caller resets its
 * counters, read when the dwell ends.
 *
 * It also owns the three ways such a window stops describing what the caller
 * thinks it describes. All three are measured, not theorised:
 *
 *   1. An NHM read inside the window. read_nhm() re-arms BOTH engines, and the
 *      damage is register-map-dependent: on the JGR3 map (Jaguar3) the long
 *      window is DESTROYED and the read returns the 2 ms re-arm (311-326 of
 *      62500 ticks where a clean window read 38000), while on the 11AC map
 *      (Jaguar1/2) it survives but reads 3-4 points HIGH — the intruding
 *      measurement appears to add busy time. Neither is a reading about the
 *      window that was asked for.
 *   2. A retune inside the window. The counter keeps running across it, so the
 *      result is a blend of two channels: measured 44-47% on a Jaguar3 and
 *      60-62% on a Jaguar1 where the channel itself was 61% and 71% busy.
 *   3. A read before the window has elapsed. The result register latches the
 *      LAST COMPLETED window, and reads are non-destructive (two reads without
 *      re-arming return identical values), so an early read silently returns a
 *      stale measurement rather than failing.
 *
 * Own transmission is carried rather than corrected. CLM counts receive-side
 * deferral only: a transmitting sensor never counts its own airtime and is
 * deaf while it transmits, so it reads LOW — measured 60.9% -> 18.5% on a
 * Jaguar3 and 70.9% -> 25% on a Jaguar1, same channel, same interferer. The
 * MediaTek channel timers are biased the other way (they count own TX as
 * busy), which is exactly why this is a fact on the reading and not a fudge
 * factor inside it.
 *
 * No hardware access of its own: the owner passes register accessors, so the
 * whole state machine is reachable from a selftest with no radio.
 */
#ifndef DEVOURER_BUSY_WINDOW_H
#define DEVOURER_BUSY_WINDOW_H

#include <cstdint>
#include <functional>

#include "NhmReader.h"
#include "RxSense.h"

namespace devourer {

/* 4 us per tick, and the period field is 16 bits. */
inline constexpr uint32_t kClmTickUs = 4;
/* The vendor reports rpt == period as 100% busy, so the top of the range
 * cannot be told from a saturated window that wrapped. Cap below it: 60000
 * ticks = 240 ms, which still spans a survey dwell. */
inline constexpr uint16_t kClmMaxPeriodTicks = 60000;
/* A floor rather than a refusal: below ~1 ms the caller is asking for what
 * the sampled path already gives, and silently arming something shorter than
 * the hardware can time would be worse than rounding up to it. The arm
 * returns the window it actually programmed, so the caller can see it. */
inline constexpr uint16_t kClmMinPeriodTicks = 250;

inline uint16_t clm_period_for_us(uint32_t window_us) {
  const uint32_t ticks = window_us / kClmTickUs;
  if (ticks < kClmMinPeriodTicks)
    return kClmMinPeriodTicks;
  if (ticks > kClmMaxPeriodTicks)
    return kClmMaxPeriodTicks;
  return static_cast<uint16_t>(ticks);
}

class ClmWindow {
public:
  using Read32 = std::function<uint32_t(uint16_t)>;
  using SetBb = std::function<void(uint16_t, uint32_t, uint32_t)>;

  /* Arm a CLM-only window. Returns the window actually armed, in
   * microseconds, which is the clamped period and NOT what the caller asked
   * for. 0 means nothing was armed. `tx_submitted` is the device's running
   * TX-submit counter, sampled so the read can tell whether this window was
   * measured while the sensor itself was transmitting. */
  uint32_t arm(const NhmRegs &regs, uint32_t window_us, uint64_t tx_submitted,
               const SetBb &set_bb) {
    /* A zero window is a caller bug, not a request for the minimum: flooring
     * it would arm hardware for a measurement nobody asked for and hand back
     * a period they never chose. Nothing is armed and nothing is disturbed. */
    if (window_us == 0)
      return 0;
    period_ = clm_period_for_us(window_us);
    arm_clm_only(regs, period_, set_bb);
    armed_ = true;
    spoil_ = BusySpoil::None;
    tx_at_arm_ = tx_submitted;
    return static_cast<uint32_t>(period_) * kClmTickUs;
  }

  /* Read an armed window. An unarmed, spoiled or not-yet-elapsed window
   * yields an INVALID reading rather than a number: "no reading" and "quiet
   * channel" are different facts and every consumer downstream acts on the
   * difference.
   *
   * A completed or spoiled window is consumed by the read. A NOT-ELAPSED one
   * is not: the hardware is still counting it, so the caller reads again
   * when its dwell ends. Consuming it here would send that retry down the
   * sampled path, which re-arms the engine for 2 ms and reports THAT as a
   * valid reading — on the JGR3 map it also destroys the window still
   * running. */
  ChannelBusy read(const NhmRegs &regs, uint64_t tx_submitted,
                   const Read32 &read32) {
    ChannelBusy b;
    if (!armed_)
      return b;

    /* An earlier reason wins: a window that something re-armed mid-flight is
     * "interrupted", and the fact that it is consequently not elapsed either
     * is a symptom of that, not a second finding. Checked BEFORE the ready
     * bit for exactly that reason. */
    if (spoil_ != BusySpoil::None) {
      armed_ = false;
      last_spoil_ = spoil_;
      b.spoil = spoil_;
      return b;
    }

    const ClmRead c = read_clm_only(regs, read32);
    if (!c.ready) {
      /* The ready bit is still clear: the window has not finished. The result
       * register holds the PREVIOUS window, so reporting it would be a stale
       * reading wearing this window's timestamp. Still armed: read again. */
      last_spoil_ = BusySpoil::NotElapsed;
      b.spoil = BusySpoil::NotElapsed;
      return b;
    }
    armed_ = false;
    b.valid = true;
    b.source = BusySource::Clm;
    b.valid_busy = true;
    b.window_us = static_cast<uint32_t>(period_) * kClmTickUs;
    const uint32_t ticks = c.ticks > period_ ? period_ : c.ticks;
    b.busy_pct =
        static_cast<uint8_t>((static_cast<uint32_t>(ticks) * 100u + period_ / 2u) /
                             period_);
    /* The counter is 64-bit and the field is 32. A window cannot hold four
     * billion frames, but a counter that jumped (a reset, a wrap) would
     * otherwise TRUNCATE into a small plausible count — and 3 frames reads as
     * "barely transmitting" where the truth is "unknown". Saturate instead:
     * the flag is what consumers act on, and it stays set. */
    const uint64_t sent = tx_submitted > tx_at_arm_ ? tx_submitted - tx_at_arm_ : 0;
    b.own_tx_frames = sent > UINT32_MAX ? UINT32_MAX
                                        : static_cast<uint32_t>(sent);
    b.own_tx_in_window = sent > 0;
    last_spoil_ = BusySpoil::None;
    return b;
  }

  /* The owner calls these from the paths that are known to spoil a window.
   * Cheap enough to call unconditionally: they do nothing when none is
   * armed. */
  void note_nhm_read() {
    if (armed_ && spoil_ == BusySpoil::None)
      spoil_ = BusySpoil::Interrupted;
  }
  void note_retune() {
    if (armed_ && spoil_ == BusySpoil::None)
      spoil_ = BusySpoil::Retuned;
  }

  bool armed() const { return armed_; }
  /* Why the last read returned nothing (None when it returned a reading). */
  BusySpoil last_spoil() const { return last_spoil_; }

private:
  bool armed_ = false;
  uint16_t period_ = 0;
  uint64_t tx_at_arm_ = 0;
  BusySpoil spoil_ = BusySpoil::None;
  BusySpoil last_spoil_ = BusySpoil::None;
};

} // namespace devourer

#endif /* DEVOURER_BUSY_WINDOW_H */
