#ifndef IRTL_RADIO_H
#define IRTL_RADIO_H

#include "IRadio.h"

#include "AdapterHealth.h" /* EfuseStability */
#include "RxSense.h"       /* RxEnergy */

/* IRtlRadio is the Realtek-family extension of IRadio: the members whose
 * semantics are defined by Realtek silicon (phydm false-alarm / CCA / IGI / NHM
 * counters, the EFUSE logical map and its 0x8129 EEPROM id, the AFE crystal-cap
 * register, the rtw canary register set) rather than by a vendor-neutral
 * concept. Every Realtek backend derives from it:
 *   - RtlJaguarDevice   — Realtek "Jaguar" wave-1 (8812AU/8811AU/8821AU/8814AU)
 *   - RtlJaguar2Device  — Realtek "Jaguar2" (8822BU/8812BU)
 *   - RtlJaguar3Device  — Realtek "Jaguar3" (8822CU/8812EU/8822EU)
 *   - Rtl8733bDevice    — Realtek HALMAC 87xx 11n (RTL8731BU/RTL8733BU)
 *   - RtlKestrelDevice  — Realtek G6 11ax (RTL8852BU/RTL8852CU)
 *
 * WiFiDriver::CreateRadio returns an IRadio; a caller that needs one of these
 * members dynamic_casts to IRtlRadio and treats nullptr as "not a Realtek
 * radio" — skip the feature with one diagnostic, never fake a reading.
 * Per-generation research helpers (BB-debug-port reads, the 8814 queue poller,
 * the CW tone) stay on the concrete classes: the same convention one level
 * further down. Every member here keeps the IRadio rule — virtual with a
 * not-ported default, never pure. */
class IRtlRadio : public IRadio {
public:
  /* Crystal (XTAL) load-capacitance trim — the CFO lever. Writes the AFE
   * crystal-cap field (a per-chip register), pulling the chip's reference
   * oscillator a few ppm to align a marginal TX/RX crystal pair; the payoff
   * is narrowband at the edge of its CFO budget (5 MHz at 5 GHz). `cap` is a
   * raw trim code in [0, GetAdapterCaps().xtal_cap_max]; cap < 0 reverts to
   * the efuse/default value. Both physical caps (Xi/Xo) are set together.
   * Returns the applied code, or -1 when unsupported. Sticky across channel
   * changes (an AFE register, untouched by the RF retune). */
  virtual int SetXtalCap(int cap) {
    (void)cap;
    return -1;
  }

  /* Current crystal-cap code (the last SetXtalCap value, or the efuse default
   * at bring-up). -1 when unsupported. */
  virtual int GetXtalCap() { return -1; }

  /* Frame-free RX energy / channel-busy snapshot (see RxSense.h) — the read side
   * of the DEVOURER_CW_TONE emitter, used for spectrum-sensing / interferer
   * detection. Reads the chip's phydm false-alarm + CCA counters, DIG/IGI, and
   * (when asked) the NHM power histogram. FA/CCA counts are the delta since the
   * previous call. Default returns an all-invalid snapshot; each generation
   * overrides with a real reader.
   *
   * `with_nhm` is a cost decision, not a preference: the NHM read arms a ~2 ms
   * measurement window and then polls a ready bit at 1 ms granularity
   * (src/NhmReader.h), so it dominates the call — the scalar FA/CCA/IGI path is
   * a handful of register reads. Pass false for the throwaway read that resets
   * the delta counters before an observation window, and for any caller
   * sampling faster than a few times a second. */
  virtual RxEnergy GetRxEnergy(bool with_nhm) { (void)with_nhm; return {}; }

  /* Perform `reads` fresh PHYSICAL EFUSE logical-map reads (each pass re-runs
   * the efuse-controller read sequence — not the cached shadow) and
   * cross-compare them. Dying silicon returns different content per read;
   * healthy silicon is byte-identical every time. Post-bring-up only: returns
   * supported=false before Init/InitWrite (on the 8814AU a pre-fwdl EFUSE
   * read breaks the RSVD-page firmware download). Control-plane threading
   * contract applies (same as SetMonitorChannel). */
  virtual devourer::EfuseStability ProbeEfuseStability(int reads = 4) {
    (void)reads;
    return {};
  }

  /* Dump the chip's canary register set (BB / MAC / per-path RF) to the
   * diagnostic plane. Reads only — no writes, no calibration, no bring-up.
   *
   * The point is that it is callable on a device that has NOT been Init'ed, so
   * a chip left in whatever state a previous session abandoned it in can be
   * inspected AS IT IS. Every other path into this driver reconfigures the chip
   * on the way in, which destroys exactly the evidence a state bug leaves
   * behind. Pair it with an open that skips libusb_reset_device
   * (claim_interface_then_reset's `do_reset=false`) — a USB reset re-runs the
   * chip's own boot and is just as destructive.
   *
   * Output format matches DEVOURER_DUMP_CANARY, so two dumps diff directly with
   * tests/canary_diff.py. Reading a powered-down chip yields garbage or throws;
   * interpreting that is the caller's job. No-op where unsupported (default). */
  virtual void DumpChipState() {}

  /* The MAC carrier-sense gate, one bit at a time.
   *
   * SetCcaMode is all-or-nothing, and on this family it is two gates:
   * 0x520[14] primary CCA (defers to a decodable preamble) and 0x520[15]
   * EDCCA (defers to raw in-band energy). They answer different questions
   * and they do not behave the same way — tests/dis_cca_tx_onair.sh measured
   * primary CCA costing a Jaguar3 injector 41-45% against a co-channel
   * flooder while the energy bit alone was null, and on Jaguar1 the result
   * inverts (see below). A caller that needs one of them should not have to
   * turn off both, and a caller diagnosing a deferral needs to tell them
   * apart.
   *
   * `true` means DISABLED, matching SetCcaMode's argument sense and the
   * register's own polarity (bit set = gate off). SetCcaMode is exactly
   * SetCcaGates(d, d) and writes the same bytes it always did. Returns false
   * where the split is not ported; SetCcaMode remains the portable call. */
  virtual bool SetCcaGates(bool primary_disabled, bool edcca_disabled) {
    (void)primary_disabled;
    (void)edcca_disabled;
    return false;
  }

  /* Current gate state, read back from the hardware rather than remembered. */
  virtual bool GetCcaGates(bool &primary_disabled, bool &edcca_disabled) {
    (void)primary_disabled;
    (void)edcca_disabled;
    return false;
  }
};

#endif /* IRTL_RADIO_H */
