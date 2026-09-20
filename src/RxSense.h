#ifndef RX_SENSE_H
#define RX_SENSE_H

#include <cstdint>

/* Frame-free RX sensing. This header holds TWO types with different audiences:
 * `RxEnergy` below — the Realtek phydm counter set, reached through
 * IRtlRadio::GetRxEnergy — and `ChannelBusy` at the end, the vendor-NEUTRAL
 * reduction reached through IRadio::GetChannelBusy. A consumer that must work
 * on an arbitrary backend reads the second; one that wants the phydm detail
 * reads the first and accepts being Realtek-only.
 *
 * RxEnergy — a frame-free RX energy / channel-busy snapshot. This is the read
 * side of the DEVOURER_CW_TONE emitter: a coarse "how much in-band energy /
 * channel activity is here" measurement that does NOT require receiving a frame.
 *
 * Filled by IRtlRadio::GetRxEnergy() from the chip's phydm facilities:
 *   - false-alarm (FA) + CCA (channel-busy) counters,
 *   - the DIG initial-gain index (a noise-floor proxy),
 *   - and, where triggered, the NHM in-band power histogram and the CLM
 *     busy-airtime measurement that shares its window.
 *
 * All values are channel-wide scalars — no Realtek 88xx chip exports
 * per-subcarrier CSI to the host, so this is energy, not a spectrum. Build a
 * coarse spectrum by sweeping channels/bins and sampling this per bin.
 *
 * FA/CCA counts are the DELTA since the previous GetRxEnergy() call (each read
 * resets the hardware counters), so a strong in-band carrier shows up as a jump
 * in cca_ofdm / fa_ofdm and a rise in igi. Every field carries a valid_* flag
 * because the facilities differ by chip generation.
 *
 * The read splits into two very different costs, which is why the caller picks
 * (`GetRxEnergy(bool with_nhm)`): the scalars below are a handful of register
 * reads, while the NHM histogram arms a ~2 ms measurement window and then polls
 * a ready bit at 1 ms granularity. Anything sampling at a sub-second cadence —
 * or throwing the read away to reset the counters before an observation
 * window — wants with_nhm=false. */
struct RxEnergy {
  /* phydm false-alarm + CCA counters (delta since the previous read). */
  bool valid_fa = false;
  uint32_t fa_ofdm = 0;  /* OFDM false-alarm count */
  uint32_t fa_cck = 0;   /* CCK false-alarm count */
  uint32_t cca_ofdm = 0; /* OFDM CCA (channel-busy) count */
  uint32_t cca_cck = 0;  /* CCK CCA count */

  /* DIG initial-gain index (0x0c50[6:0] on the AC BB): the AGC backs the gain
   * off as the in-band floor rises, so a higher IGI means a busier/noisier
   * channel. Noise-floor proxy. */
  bool valid_igi = false;
  uint8_t igi = 0;

  /* NHM in-band power histogram: 12 IGI-referenced power buckets and the
   * measurement duration. A frame-free power distribution (fuller than the
   * scalar FA counts). Only populated when the caller asked for it
   * (GetRxEnergy(with_nhm=true)). */
  bool valid_nhm = false;
  uint8_t nhm[12] = {};
  uint16_t nhm_duration = 0;
  /* The thresholds that produced those buckets (U(8,1) PWdB, IGI-referenced).
   * Kept because the histogram alone does not say where in absolute power it
   * sits — devourer::nhm_utility() needs both. */
  uint8_t nhm_th[11] = {};
  /* Vendor utility reductions of the histogram above (devourer::nhm_utility):
   * nhm_ratio is the mass above bucket 0, nhm_env_ratio the mass above the
   * receiver's OWN noise floor. The second is the one that means "something is
   * transmitting here"; the first rails busy on a quiet channel. */
  uint8_t nhm_ratio_pct = 0;
  uint8_t nhm_env_ratio_pct = 0;

  /* CLM (Channel Load Measurement): the fraction of the measurement window in
   * which the baseband asserted CCA busy, counted by hardware in 4 us ticks.
   * Unlike the FA/CCA event counters this is AIRTIME, directly comparable
   * across channels and adapters, and unlike NHM it needs no gain reference.
   *
   * Read CLM against nhm_env_ratio_pct rather than alone: CLM counts the
   * channel held by anything the BB recognises as a signal it must defer to,
   * while NHM-env counts energy above the floor whether or not it looked like
   * one. Energy that raises NHM-env without raising CLM is the non-802.11
   * emitter a frame sniffer cannot see at all.
   *
   * Rides the same armed window as the NHM histogram, so it is only populated
   * when the caller asked for NHM (GetRxEnergy(with_nhm=true)). */
  bool valid_clm = false;
  uint8_t clm_ratio_pct = 0;  /* 0..100 busy airtime */
  uint16_t clm_result = 0;    /* raw busy ticks, 4 us each */
  uint16_t clm_period = 0;    /* window length in the same 4 us ticks */

  /* Active/frame-free ABSOLUTE noise floor (dBm) — the vendor idle-noise
   * monitor, distinct from the passive rssi-snr floor in RxQuality. Heavy
   * (~10 ms of USB round-trips), so it is only filled when the caller opted in
   * (DEVOURER_RX_NOISE_FLOOR). Jaguar2 fills it live (HW idle-noise report,
   * wedge-free); Jaguar1 8812A/8821A fill it from an RX-idle CAL measurement;
   * Jaguar3 and others leave it invalid (no vendor path). */
  bool valid_noise_floor = false;
  int8_t abs_noise_floor_dbm = 0;
};

namespace devourer {

/* Where a ChannelBusy reading came from. Provenance is part of the reading, not
 * metadata about it: the two facilities define "busy" differently (see
 * ChannelBusy::busy_pct), so a consumer ranking channels across a MIXED pair of
 * adapters is comparing two rulers and needs to know. Ranking within one
 * adapter — the chanscout / hopset case — is unaffected. */
enum class BusySource : uint8_t {
  None = 0,  /* no reading */
  Clm = 1,   /* Realtek CCX CLM: baseband-asserted CCA busy, 4 us ticks */
  ChTime = 2 /* MediaTek MAC channel timers: TX+RX+NAV+EIFS counted busy */
};

/* Why an armed busy window produced no reading. Carried ON the reading
 * because "this backend has no sensor" and "this window was spoiled" are
 * different facts with different fixes, and a consumer that logs one as the
 * other will chase the wrong bug. Named for what happened to the window, not
 * for the facility that did it, so a second family's equivalent fits.
 *
 *   Interrupted  another measurement re-armed the shared engine mid-window
 *                (on Realtek, an NHM read: destructive on the JGR3 map, a 3-4
 *                point overcount on the 11AC map).
 *   Retuned      the radio changed channel mid-window, so the count is a
 *                blend of two channels.
 *   NotElapsed   read before the window finished. The hardware latches the
 *                PREVIOUS window's result, so this would be a stale reading
 *                wearing a fresh timestamp. */
enum class BusySpoil : uint8_t {
  None = 0,
  Interrupted,
  Retuned,
  NotElapsed
};

/* ChannelBusy — the vendor-NEUTRAL frame-free channel-occupancy reading, and
 * the only energy evidence src/chanmig/ and src/hopset/ can ask an arbitrary
 * backend for.
 *
 * It exists because every counter in RxEnergy except CLM is defined by phydm
 * (false-alarm classes, the DIG index, IGI-referenced NHM buckets) and has no
 * meaning on another vendor's silicon, while busy AIRTIME does: both families
 * count it in hardware, it is a time fraction rather than an event count, and
 * it therefore needs no per-adapter normalisation to compare two channels.
 *
 * Read-and-clear DELTA semantics, like RxEnergy: each call returns the window
 * since the previous call on that backend, so two pollers on one radio steal
 * each other's counts. IRadio::ArmChannelBusy replaces that implicit window
 * with an explicit one: after an arm, the next reading covers arm-to-read and
 * nothing else, and window_us says so.
 *
 * Every field carries a validity flag. A backend that cannot answer reports
 * valid=false, NEVER a zero reading — "quiet channel" and "no sensor" are
 * different facts and the fusion layers act on the difference. */
struct ChannelBusy {
  /* false = no reading at all (backend unported, chip not brought up, or the
   * window came back implausible). Nothing below is meaningful. */
  bool valid = false;
  BusySource source = BusySource::None;

  /* The window the reading spans, in microseconds. Realtek: the CLM period
   * (clm_period * 4 us, a HARDWARE window, ~2 ms at the default). MediaTek:
   * the HOST-measured interval since the previous call, i.e. as long as the
   * caller's own cadence. A consumer rejecting a too-short dwell reads this
   * rather than its own clock. 0 = unknown (the first call on the MediaTek
   * path, which has no previous mark). */
  uint32_t window_us = 0;

  /* Busy airtime over the window, 0..100. What "busy" means differs by source,
   * and the difference is documented rather than papered over:
   *   - Clm: fraction of 4 us ticks in which the baseband asserted CCA busy —
   *     receive-side deferral only.
   *   - ChTime: the MAC channel timers with TX, RX, NAV and EIFS all counted
   *     busy (mt76's mt76x02_mac_cc_reset() configuration), so on a radio that
   *     is also TRANSMITTING this includes its own airtime. A TX-side sensor
   *     must read it inside a quiet window, which src/hopset/ already does.
   *
   * One window is a sample, not a measurement: CLM measured a per-window sd of
   * 36 on an ambient-traffic channel (docs/rx-spectrum-sensing.md). Average
   * several before ranking anything. */
  bool valid_busy = false;
  uint8_t busy_pct = 0;

  /* In-band energy above the receiver's OWN noise floor, 0..100 — energy that
   * was there whether or not the baseband recognised a signal it had to defer
   * to. The contrast with busy_pct is the point: energy that raises this
   * without raising busy_pct is the non-802.11 emitter a frame sniffer cannot
   * see at all.
   *
   * Realtek fills it from nhm_env_ratio_pct. MediaTek leaves it INVALID: the
   * only candidate there is MT_RX_STAT_1's false-CCA field, which is
   * read-and-clear and OWNED by the 1 Hz AGC loop, so a second reader at
   * caller cadence would both misreport it and blind the gain tracking. An
   * unowned counter is not available for reuse. */
  bool valid_energy = false;
  uint8_t energy_pct = 0;

  /* Did the radio TRANSMIT inside the measured window, and how many frames.
   *
   * Carried rather than corrected, because the two families are biased in
   * OPPOSITE directions and only the consumer knows which it can live with:
   *   - Clm counts receive-side deferral only, and a transmitting radio is
   *     deaf to the channel while its own PA is up, so the reading comes back
   *     LOW. Measured on one 61%-busy channel: 60.9% silent, 18.4-18.6% while
   *     the sensor transmitted (Jaguar3); 70.9% -> 24.4-25.8% on a Jaguar1.
   *   - ChTime counts TX, RX, NAV and EIFS alike, so the same session reads
   *     HIGH by its own airtime.
   * A ranker comparing channels must not mix a hot sample with a quiet one in
   * either direction. window_us with own_tx_in_window set is a sample that
   * was taken, not a channel that was measured. */
  bool own_tx_in_window = false;
  uint32_t own_tx_frames = 0;

  /* Set when an ARMED window came back unusable (valid stays false). None on
   * a backend that simply has no sensor — that is the absence of a reading,
   * not a spoiled one. */
  BusySpoil spoil = BusySpoil::None;
};

/* RxEnergy -> ChannelBusy. Pure. CLM and NHM-env are the only two fields in
 * RxEnergy whose meaning survives leaving the Realtek family; everything else
 * is dropped on purpose. Yields valid=false for an energy read that carried
 * neither — a with_nhm=false read, or a generation with no CCX engine
 * (Kestrel's NHM rides halbb, not NhmReader, so it has no CLM; the RTL8733B
 * has no GetRxEnergy at all). */
inline ChannelBusy busy_from_rx_energy(const RxEnergy &e) {
  ChannelBusy b;
  if (!e.valid_clm)
    return b; /* no airtime figure -> no reading, not a quiet channel */
  b.valid = true;
  b.source = BusySource::Clm;
  b.valid_busy = true;
  b.busy_pct = e.clm_ratio_pct > 100 ? 100 : e.clm_ratio_pct;
  b.window_us = static_cast<uint32_t>(e.clm_period) * 4u; /* ticks are 4 us */
  if (e.valid_nhm) {
    b.valid_energy = true;
    b.energy_pct =
        e.nhm_env_ratio_pct > 100 ? 100 : e.nhm_env_ratio_pct;
  }
  return b;
}

/* MediaTek MAC channel timers -> ChannelBusy. Pure, so the whole conversion is
 * ctest-covered without an adapter. `busy` and `idle` are the raw
 * read-and-clear MT_CH_BUSY / MT_CH_IDLE deltas in MAC clock units; the ratio
 * is taken against their SUM, which cancels the clock unit and needs no MAC
 * clock constant. `interval_us` is the host-measured window (0 = unknown).
 *
 * Refuses (valid=false) when busy+idle == 0: that is a counter that was never
 * armed, or a read that did not happen — NOT an idle channel. */
inline ChannelBusy busy_from_ch_time(uint32_t busy, uint32_t idle,
                                     uint32_t interval_us) {
  ChannelBusy b;
  const uint64_t total = static_cast<uint64_t>(busy) + idle;
  if (total == 0)
    return b;
  b.valid = true;
  b.source = BusySource::ChTime;
  b.valid_busy = true;
  /* 64-bit: busy alone reaches UINT32_MAX on a long interval. */
  const uint64_t pct = (static_cast<uint64_t>(busy) * 100u + total / 2u) / total;
  b.busy_pct = static_cast<uint8_t>(pct > 100 ? 100 : pct);
  b.window_us = interval_us;
  /* energy_pct deliberately left invalid — see the field doc. */
  return b;
}

/* An armed MediaTek busy window, as far as the conversion below needs to know
 * about it. The backend owns the lifetime; this is the snapshot it decides on.
 *
 * These timers have no ready bit, so everything that makes an armed reading
 * honest has to be carried in software: what the caller asked for (or a short
 * glance at the channel reads as a finished window), whether a retune ran
 * through it, and the TX baseline, because unlike CLM these timers count own
 * transmission as busy. */
struct ChTimeWindow {
  bool armed = false;
  bool spoiled = false;   /* a retune ran through the window */
  uint32_t window_us = 0; /* what the caller requested at arm */
  uint64_t tx_at_arm = 0;
};

/* The armed-window decision for the MediaTek timers, pure so the refusals are
 * reachable from a selftest with no radio — the backend that owns the state is
 * hardware-only, and these rules are exactly the part worth testing.
 *
 * `disturbed` says mt7612u_link_stats() read-and-cleared the same registers
 * inside the window, which takes the counts this reading would otherwise
 * claim: the MediaTek equivalent of an NHM read re-arming the shared CCX
 * engine, and reported the same way.
 *
 * An unarmed call is the sampled path and falls through to busy_from_ch_time
 * unchanged. */
inline ChannelBusy busy_from_ch_time_window(const ChTimeWindow &w,
                                            uint32_t busy, uint32_t idle,
                                            uint32_t interval_us,
                                            bool disturbed, uint64_t tx_now) {
  if (w.armed) {
    if (w.spoiled) {
      ChannelBusy b;
      b.spoil = BusySpoil::Retuned;
      return b;
    }
    /* Interruption outranks "not elapsed", matching the Realtek ordering in
     * devourer::ClmWindow::read: when something took the counters, the short
     * interval is a SYMPTOM of that, and the caller's fix is its own
     * sequencing rather than a longer wait. */
    if (disturbed) {
      ChannelBusy b;
      b.spoil = BusySpoil::Interrupted;
      return b;
    }
    if (interval_us < w.window_us) {
      ChannelBusy b;
      b.spoil = BusySpoil::NotElapsed;
      return b;
    }
  }
  ChannelBusy b = busy_from_ch_time(busy, idle, interval_us);
  /* An armed, elapsed, undisturbed window whose timers read nothing is a
   * window that was LOST (a MAC that stopped counting), not "no sensor":
   * that shape is reserved for a backend without one. */
  if (w.armed && !b.valid) {
    b.spoil = BusySpoil::Interrupted;
    return b;
  }
  if (w.armed && b.valid) {
    const uint64_t sent = tx_now > w.tx_at_arm ? tx_now - w.tx_at_arm : 0;
    b.own_tx_frames = sent > UINT32_MAX ? UINT32_MAX
                                        : static_cast<uint32_t>(sent);
    b.own_tx_in_window = sent > 0;
  }
  return b;
}


} /* namespace devourer */

#endif /* RX_SENSE_H */
