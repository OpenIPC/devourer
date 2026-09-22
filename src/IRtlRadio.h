#ifndef IRTL_RADIO_H
#define IRTL_RADIO_H

#include "IRadio.h"

#include "AdapterHealth.h" /* EfuseStability */
#include <mutex>

#include "BusyWindow.h"
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
 * not-ported default, never pure.
 *
 * One member here is the other direction: GetChannelBusy is DECLARED on IRadio
 * (it is a vendor-neutral concept) and merely IMPLEMENTED here, once, in terms
 * of this family's own facility. The split is about where a member is
 * declared, not where a family implements it — so this is not a re-mixing of
 * it, and it saves five identical per-backend overrides. */
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

  /* The neutral reading (IRadio::GetChannelBusy), expressed in this family's
   * own facility: CLM busy airtime plus NHM-env, both of which ride
   * GetRxEnergy's armed CCX window.
   *
   * Implemented ONCE here rather than five times because every Realtek
   * backend's answer is the same function of its GetRxEnergy. A generation
   * that overrides GetRxEnergy gets this for free; one that does not gets a
   * correct "no reading" instead of a fabricated zero — which is the right
   * answer for the RTL8733B (no override at all) and for Kestrel (whose
   * GetRxEnergy fills only the absolute noise floor: its NHM rides the halbb
   * glue, not NhmReader, so there is no CLM).
   *
   * Left non-final on purpose: CLM's period field is independent of the NHM
   * window and reaches ~262 ms, so a generation may later want a longer,
   * lower-variance window than the shared read gives.
   *
   * COST + CONTENTION: see the IRadio declaration. This arms the ~2 ms NHM
   * window and consumes the same delta GetRxEnergy and GetRxQuality read. */
  devourer::ChannelBusy GetChannelBusy() override {
    /* Sampled OUTSIDE the CCX lock. On these backends GetTxStats is a
     * lock-free counter read, but the rule stands on the ordering, not on
     * that: nothing may reach for another lock while holding this one. */
    const uint64_t tx = GetTxStats().submitted;
    devourer::ChannelBusy armed_reading;
    bool was_armed = false;
    const bool have_ccx =
        with_ccx([&](const devourer::NhmRegs &regs, const Read32 &rd,
                     const SetBb &) {
          if (!_busy_window.armed())
            return;
          was_armed = true;
          armed_reading = _busy_window.read(regs, tx, rd);
        });
    if (have_ccx && was_armed)
      return armed_reading;
    /* Unarmed: the shipped sampled path, unchanged — GetRxEnergy takes its own
     * locks, so it must not be called with the CCX lock held. */
    return devourer::busy_from_rx_energy(GetRxEnergy(/*with_nhm=*/true));
  }

  /* IRadio::ArmChannelBusy on this family: a CLM-only window, armed without
   * touching the NHM half of the shared engine. Implemented once here for the
   * same reason GetChannelBusy is — every generation's answer is the same
   * function of its CCX register map, which ccx_access() supplies. A
   * generation with no CCX map returns 0 (not ported), and the caller keeps
   * the sampled path. */
  uint32_t ArmChannelBusy(uint32_t window_us) override {
    const uint64_t tx = GetTxStats().submitted; /* see GetChannelBusy */
    uint32_t armed = 0;
    const bool have_ccx =
        with_ccx([&](const devourer::NhmRegs &regs, const Read32 &,
                     const SetBb &wr) {
          armed = _busy_window.arm(regs, window_us, tx, wr);
        });
    return have_ccx ? armed : 0;
  }

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
   * SetCcaMode is all-or-nothing, and on Jaguar1 and Jaguar3 it is two
   * gates: 0x520[14] primary CCA (defers to a decodable preamble) and
   * 0x520[15] EDCCA (defers to raw in-band energy). They answer different
   * questions, and the two families measured so far DISAGREE about which one
   * stops an injector — so a caller diagnosing a deferral has to tell them
   * apart, and one that needs a single gate should not have to turn off
   * both. CLAUDE.md summarises the on-air delivery figures and
   * tests/dis_cca_tx_onair.sh is the harness behind them;
   * tests/cca_gates_regcheck.sh is the register-level check that this
   * contract holds, not a delivery measurement.
   *
   * `true` means DISABLED, matching SetCcaMode's argument sense and the
   * register's own polarity (bit set = gate off). SetCcaMode is exactly
   * SetCcaGates(d, d) and writes the same bytes it always did; it stays the
   * portable call, and is all a backend without the split offers.
   *
   * CONTRACT, because both halves of this have bitten:
   *
   *  - POST-BRING-UP ONLY. Both calls return false before Init/InitWrite:
   *    0x520 is meaningless until the MAC is configured, so reading it would
   *    be a fabricated gate state and writing it would poke an uninitialised
   *    MAC. `false` therefore means EITHER "not ported on this backend" OR
   *    "not brought up yet"; a caller probing capability at construction
   *    cannot tell those apart and must re-ask after bring-up. On a refusal
   *    GetCcaGates leaves its out-parameters untouched.
   *
   *    SetCcaMode is NOT the same, and the difference is pre-existing rather
   *    than something the split introduced: it returns void, so a
   *    pre-bring-up call cannot report anything, and what it does with one
   *    is per-backend. The way to ask for a gate state from bring-up is the
   *    tuning.disable_cca config knob, which Init applies once the MAC is
   *    up.
   *
   *  - STICKINESS SURVIVES A RETUNE ON BOTH, BUT ONLY ONE OF THEM MEANS IT.
   *    Measured on an 8812AU and an 8822C, the gate state is intact after
   *    SetMonitorChannel AND after FastRetune, both within a band and across
   *    a 5 GHz/2.4 GHz change, at 0x520, 0x524 and Jaguar1's BB thresholds.
   *    The mechanisms are not equivalent: Jaguar3 records the pair and
   *    re-asserts it in SetMonitorChannel (its FastRetune fallback does not,
   *    and does not need to), while Jaguar1 records nothing and survives
   *    only because its channel path happens not to rewrite those registers.
   *    Do not build on the Jaguar1 case — re-read with GetCcaGates rather
   *    than assume. Bring-up IS a reset on Jaguar1: Init/InitWrite
   *    unconditionally re-run SetCcaMode(_cfg.tuning.disable_cca), so a
   *    re-Init puts the gates back to the configured default. */
  virtual bool SetCcaGates(bool primary_disabled, bool edcca_disabled) {
    (void)primary_disabled;
    (void)edcca_disabled;
    return false;
  }

  /* Current gate state, read back from the hardware rather than remembered.
   * Same contract as SetCcaGates above: post-bring-up only, false where the
   * split is unavailable, out-parameters untouched on a refusal. */
  virtual bool GetCcaGates(bool &primary_disabled, bool &edcca_disabled) {
    (void)primary_disabled;
    (void)edcca_disabled;
    return false;
  }

protected:
  using Read32 = std::function<uint32_t(uint16_t)>;
  using SetBb = std::function<void(uint16_t, uint32_t, uint32_t)>;

  using CcxFn = std::function<void(const devourer::NhmRegs &, const Read32 &,
                                   const SetBb &)>;

  /* Run `fn` with the generation's CCX register map and register accessors,
   * holding every lock that access needs: the family's own register lock
   * first (the CCX registers are read-modify-written by phy_set_bb_reg, and
   * on the Jaguar3 the coex thread writes registers under _reg_mu), then
   * busy_window_mutex(). A generation without a CCX engine leaves this false
   * and its callers get "no reading" rather than a fabricated zero.
   *
   * It takes the locks rather than handing them out because the arm is FOUR
   * register writes that must not interleave with another CCX user; lending
   * bare accessors made every caller's sequence non-atomic. Protected: this
   * is a family lending the base class its register access, not a
   * register-poke API on the contract. */
  virtual bool with_ccx(const CcxFn & /*fn*/) { return false; }

  /* A generation calls these from the two paths measured to spoil an armed
   * window: its NHM read (which re-arms the shared engine) and its retune.
   * No-ops when nothing is armed, so they are safe to call unconditionally.
   *
   * THE CALLER MUST HOLD busy_window_mutex(), and for the NHM read it must
   * hold it across the note AND the read itself. Otherwise the note can land
   * before a concurrent arm sets its flag while the re-arm lands after it,
   * which is precisely the destroyed window this reports — silently valid. */
  void busy_window_note_nhm_read() { _busy_window.note_nhm_read(); }
  void busy_window_note_retune() { _busy_window.note_retune(); }

  /* Forget any armed window. Bring-up AND teardown paths call it: a window
   * armed before a re-Init — or before a Stop() — describes a chip state that
   * no longer exists, and leaving it armed would make the next unrelated
   * GetChannelBusy() take the armed branch and report a stale period as if it
   * were its own dwell. Stop() is the half that is easy to miss, because
   * clearing the flag with_ccx gates on does not stand in for it: on a
   * backend whose retune re-runs bring-up, that flag comes straight back.
   * See IRadio::ArmChannelBusy for the contract. */
  void busy_window_reset() { _busy_window = devourer::ClmWindow{}; }

  /* Serialises the CCX engine: the armed window's state, the arm/read, and
   * the NHM read that re-arms the same hardware. Ordering is always the
   * family's register lock FIRST, then this one. */
  std::mutex &busy_window_mutex() { return _ccx_mu; }

  std::mutex _ccx_mu;
  devourer::ClmWindow _busy_window; /* guarded by _ccx_mu */

};

#endif /* IRTL_RADIO_H */
