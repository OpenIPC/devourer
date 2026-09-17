#ifndef RX_SENSE_H
#define RX_SENSE_H

#include <cstdint>

/* RxEnergy — a frame-free RX energy / channel-busy snapshot. This is the read
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

#endif /* RX_SENSE_H */
