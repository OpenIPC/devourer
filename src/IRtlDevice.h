#ifndef IRTL_DEVICE_H
#define IRTL_DEVICE_H

#include <cstddef>
#include <cstdint>
#include <functional>

#include "AdapterCaps.h"
#include "AmpduMode.h"
#include "DeviceConfig.h"
#include "AdapterHealth.h"
#include "RxQuality.h"
#include "RxSense.h"
#include "SelectedChannel.h"
#include "ThermalStatus.h"
#include "Sounding.h"
#include "TriggerTwt.h"
#include "TxCaps.h"
#include "TxMode.h"
#include "TxPower.h"
#include "TxStats.h"

/* Packet is the parsed-RX type handed to the RX callback. Forward-declared here
 * (a reference in the std::function signature needs only an incomplete type) so
 * the interface header stays light; the full definition lives in FrameParser.h. */
struct Packet;

using Action_ParsedRadioPacket = std::function<void(const Packet &)>;

/* One TX frame handed to send_packets: radiotap header + 802.11 MPDU, the
 * same buffer contract as send_packet. */
struct TxPacketView {
  const uint8_t *data;
  size_t len;
};

/* IRtlDevice is the chip-family-agnostic device contract used by the demos and
 * the WiFiDriver factory. Three implementations exist:
 *   - RtlJaguarDevice   — Realtek "Jaguar" wave-1 (8812AU/8811AU/8821AU/8814AU)
 *   - RtlJaguar2Device  — Realtek "Jaguar2" (8822BU/8812BU)
 *   - RtlJaguar3Device  — Realtek "Jaguar3" (8822CU/8812EU/8822EU)
 *
 * Chip-family-specific research helpers (BB-debug-port reads, the 8814 queue
 * poller, ...) are intentionally NOT part of this interface — callers that need
 * them dynamic_cast down to the concrete type. */
class IRtlDevice {
public:
  virtual ~IRtlDevice() = default;

  virtual void Init(Action_ParsedRadioPacket packetProcessor,
                    SelectedChannel channel) = 0;
  virtual void InitWrite(SelectedChannel channel) = 0;

  /* Blocking RX worker loop. Assumes the chip is already brought up (a prior
   * Init or InitWrite on this device) — performs NO bring-up, channel set, or
   * beamforming arming. Runs on the CALLER's thread and returns once
   * StopRxLoop() is called or the global stop flag is set; it is restartable
   * after it returns. This is the piece that lets one process bring up once
   * (InitWrite) and then run TX and RX concurrently on the same claimed handle
   * — Init is the RX-only convenience wrapper (bring-up + StartRxLoop). */
  virtual void StartRxLoop(Action_ParsedRadioPacket packetProcessor) = 0;

  /* Ask a running StartRxLoop to exit (sets a flag; the caller then joins
   * whatever thread runs StartRxLoop). Default no-op. */
  virtual void StopRxLoop() {}
  virtual void SetMonitorChannel(SelectedChannel channel) = 0;

  /* Lean intra-band, same-bandwidth channel retune for hop/sweep dwells: the RF
   * channel switch only, skipping the per-rate TX-power loop and bandwidth
   * post-set a hop doesn't need (see docs/frequency-hopping.md). Generations
   * with a fast path override this (Jaguar1, Jaguar3 — both fall back to the
   * full path internally on a band change); the default is the full
   * SetMonitorChannel at the current width/offset, so callers may hop through
   * this unconditionally on any chip. `cache_rf` selects the cached-RF-write
   * variant where one exists (false = re-read per hop, for A/B measurement);
   * the default argument binds here, at the interface. */
  virtual void FastRetune(uint8_t channel, bool cache_rf = true) {
    (void)cache_rf;
    SelectedChannel c = GetSelectedChannel();
    c.Channel = channel;
    SetMonitorChannel(c);
  }

  /* Lean same-channel bandwidth switch between 20 MHz and 5/10 MHz narrowband —
   * the bandwidth analogue of FastRetune. On the chips that support it the
   * whole switch collapses to a single BB dword write, because across a
   * 20<->5/10 toggle the RF stays in 20 MHz mode and only the baseband ADC/DAC
   * re-clock register changes (everything else — RF bandwidth, MAC BW, TX
   * power, IQK — is invariant, so it is skipped). Generations with a fast path
   * override this; the default falls back to a full SetMonitorChannel at the
   * current channel/offset, so callers may use it unconditionally. */
  virtual void FastSetBandwidth(ChannelWidth_t bw) {
    SelectedChannel c = GetSelectedChannel();
    c.ChannelWidth = bw;
    SetMonitorChannel(c);
  }

  /* Force a flat absolute TXAGC index across all rates (the debug /
   * SDR-visibility knob — same knob as SetTxPowerIndexOverride, kept for
   * source compatibility; the override form has the explicit clear). NB this
   * is a REAL flat override on every generation now — it previously had three
   * divergent semantics (Jaguar1: pre-efuse fallback, silently ignored with
   * loaded EFUSE; Jaguar2: dead store; Jaguar3: flat reference). */
  virtual void SetTxPower(uint8_t power) {
    SetTxPowerIndexOverride(static_cast<int>(power));
  }

  /* --- Runtime TX-power control (see src/TxPower.h for the full model) ---
   *
   * THREADING CONTRACT: like every other control-plane entry point
   * (SetMonitorChannel, FastRetune, SetTxMode, Start/StopCwTone), these are
   * single-control-thread calls — invoke them from the thread that owns the
   * device's control plane, never concurrently with a channel set. Jaguar3
   * additionally serializes against its coex/thermal tick internally; on
   * Jaguar1/2 the USB-touching apply itself is the caller's to sequence.
   * GetTxPowerCaps and the cached (non-readback) part of GetTxPowerState are
   * safe from any thread. */

  /* Static capabilities of this family's TX-power knobs; supported=false on
   * the default (a generation without the API wired). */
  virtual devourer::TxPowerCaps GetTxPowerCaps() { return {}; }

  /* Adjust TX power RELATIVE to the efuse-calibrated per-rate table (or the
   * flat override when one is active) in quarter-dB. Quantized to the family
   * step and clamped to the caps range; returns the APPLIED qdB (0 when
   * unsupported), so a closed-loop controller knows exactly what moved.
   * Sticky: survives SetMonitorChannel (re-folded against the new channel's
   * table) and FastRetune (hop paths never rewrite TXAGC). Refused (returns
   * 0, logs) while a CW tone holds the chip — TXAGC does not modulate a bare
   * LO carrier. */
  virtual int SetTxPowerOffsetQdb(int qdb) {
    (void)qdb;
    return 0;
  }

  /* Force / clear the flat absolute TXAGC index: idx >= 0 forces it for all
   * rates (composes with the offset), idx < 0 reverts to the efuse per-rate
   * baseline. The primary knob — SetTxPower forwards here; a generation
   * without the API wired ignores it (caps.supported=false). */
  virtual void SetTxPowerIndexOverride(int idx) { (void)idx; }

  /* Re-program the TX-power registers from the current knob state at the
   * CURRENT channel — the hook tests use to force a re-apply without moving
   * any knob. Returns false when unsupported or the chip isn't brought up. */
  virtual bool ReApplyTxPower() { return false; }

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

  /* Snapshot of the knob state + representative effective indices (register
   * readback where the family's TXAGC block is readable). */
  virtual devourer::TxPowerState GetTxPowerState() { return {}; }

  /* Chip thermal-meter snapshot (RF 0x42 family; efuse baseline where the
   * family wires one — see src/ThermalStatus.h). The PA-heating input of the
   * adaptive-link controller. Default returns an all-invalid reading. */
  virtual devourer::ThermalStatus GetThermalStatus() { return {}; }

  /* Per-chip TX capability report (see src/TxCaps.h): spatial streams, STBC /
   * LDPC / SGI support, max bandwidth — derived from the chip identity resolved
   * at construction. A caller (or send_packet) uses it to avoid requesting a
   * feature the silicon can't do (e.g. STBC on a 1T1R part, which produces a
   * frame that never decodes). Default returns supported=false. */
  virtual devourer::TxCaps GetTxCaps() { return {}; }

  /* Aggregate STATIC adapter-capability report (see src/AdapterCaps.h):
   * identity (chip name, generation, variant, transport, chip-id), TX/RX chain
   * counts, the composed TxCaps + TxPowerCaps, the supported channel-width set,
   * per-band tunable + characterized frequency spans, and the per-family
   * feature flags (per-packet TX power, narrowband, fast retune, per-chain
   * RSSI). Resolved at construction — safe from any thread and callable BEFORE
   * Init/InitWrite (the demos emit it as the `adapter.caps` event right after
   * CreateRtlDevice). Default returns supported=false. */
  virtual devourer::AdapterCaps GetAdapterCaps() { return {}; }

  /* Best-effort live estimate of which RX chains are actually carrying signal
   * (see ActiveRxPaths in src/RxQuality.h) — derived from the per-frame
   * per-chain RSSI the frame parser fills, so it needs an RX loop running and
   * ambient traffic to mean anything. A chain whose mean RSSI sits far below
   * the strongest is reported inactive (a disconnected/blocked antenna, or a
   * software-masked path). Delta semantics like GetRxQuality: the window drains
   * on read. Default is an all-invalid snapshot; a generation with >=2 RX
   * chains overrides. NOT part of AdapterCaps, which is static. */
  virtual devourer::ActiveRxPaths GetActiveRxPaths() { return {}; }

  virtual bool send_packet(const uint8_t *packet, size_t length) = 0;

  /* Hardware ACK responder (src/AckResponder.h): program the port identity
   * (MACID/BSSID = `mac`, net_type = AP) so the MAC auto-ACKs — SIFS-timed,
   * zero host involvement — any unicast frame addressed to `mac`, while
   * monitor RX/injection continue unchanged. The reliable-unicast enabler:
   * a peer TXing to `mac` with normal ack-policy gets hardware
   * retransmissions until the ACK (its tx.report shows retries~0). `mac`
   * must be unicast (I/G clear). Turning a passive monitor into an active
   * transmitter is opt-in only — never a default. Returns false where
   * unsupported. Clear = net_type back to No Link. */
  virtual bool SetAckResponder(const devourer::MacAddr &mac) {
    (void)mac;
    return false;
  }
  virtual void ClearAckResponder() {}

  /* 802.11 A-MPDU TX mode (src/AmpduMode.h): the first-class bundle of the
   * recipe the spike + pacing sweep proved on-air. When enabled, every data
   * frame is marked aggregatable (data QSEL + AGG_EN + MAX_AGG_NUM +
   * AMPDU_DENSITY, and — for the no-BlockAck-peer broadcast case — a per-frame
   * retry limit of 0), and the call programs the MAC aggregate-fill timer +
   * burst-mode gate live (the pacing that gates net goodput). Amortizes one
   * PHY preamble over many MPDUs: +30% on-air goodput at MCS7/20, more at
   * higher rates. The caller must keep the TX queue fed deep enough for the
   * MAC to aggregate (send_packets with enough in flight). Off keeps every TX
   * byte-identical. Returns false where unsupported (USB HalMAC + Jaguar1;
   * PCIe not wired). The lower-level DEVOURER_TX_QSEL / DEVOURER_TX_AMPDU
   * spike knobs still compose on top for register-level experimentation. */
  virtual bool SetAmpduMode(const devourer::AmpduMode & /*mode*/) {
    return false;
  }
  virtual void ClearAmpduMode() {}
  virtual devourer::AmpduMode GetAmpduMode() { return {}; }

  /* 802.11ax trigger-based UL + TWT (src/TriggerTwt.h) — the standards-native
   * scheduled, contention-free UL access. Kestrel (RTL8852) only: the AP airs
   * an HE Trigger frame that grants each STA a resource unit and a start time
   * (SIFS after the trigger), and the STA MAC fires its UL-OFDMA TX at that
   * instant in hardware. Every method returns false where unsupported (the
   * Jaguar generations have no HE trigger/TWT firmware surface). */

  /* Air one HE Basic Trigger (UL-OFDMA grant) via the firmware F2P command. */
  virtual bool SendTrigger(const devourer::TriggerConfig & /*cfg*/) {
    return false;
  }

  /* Create/modify a TWT agreement (wake window, interval, absolute target-wake
   * TSF); TeardownTwt deletes it. The AP owns the timetable. */
  virtual bool ConfigureTwt(const devourer::TwtConfig & /*cfg*/) { return false; }
  virtual bool TeardownTwt(const devourer::TwtConfig & /*cfg*/) { return false; }
  /* Bind/unbind a STA (macid) to a TWT config (add/del/terminate/...). */
  virtual bool TwtBindSta(const devourer::TwtStaAct & /*act*/) { return false; }

  /* Program the fw-autonomous trigger cadence inside a TWT service period
   * (TWT-OFDMA). Returns false where the fw lacks the (non-canonical) command —
   * ConfigureUlOfdma is the canonical fallback. */
  virtual bool ConfigureTwtOfdma(const devourer::TwtOfdmaConfig & /*cfg*/) {
    return false;
  }

  /* Program the production UL-OFDMA scheduler table (UL_FIXINFO). With
   * mode=tf_periodic the fw airs Triggers autonomously at the configured
   * interval — the transport-independent scheduled-UL primitive. */
  virtual bool ConfigureUlOfdma(const devourer::UlOfdmaConfig & /*cfg*/) {
    return false;
  }

  /* Register an associated peer STA (a scheduled-UL client): its macid + ADDR_CAM
   * so a Trigger's per-user grant scores against it. `peer_mac` is the STA's
   * address, `addr_cam_idx` must be unique per peer. The AP-side companion to
   * SendTrigger/ConfigureUlOfdma for the end-to-end UL path. Returns false where
   * unsupported. */
  virtual bool RegisterPeerSta(const uint8_t /*peer_mac*/[6], uint8_t /*macid*/,
                               uint8_t /*addr_cam_idx*/) {
    return false;
  }

  /* Register an associated HE STA (macid) as a beamformee to sound: program its
   * per-STA CSI/bf params (nc/nr/ng/cb/cs) + the sounding-status/CSI-buffer maps
   * so a BFRP to it solicits a decodable compressed-beamforming report. The
   * AP-side companion to StartSounding. Returns false where unsupported. */
  virtual bool RegisterBeamformee(const uint8_t /*peer_mac*/[6], uint8_t /*macid*/,
                                  uint8_t /*addr_cam_idx*/,
                                  const devourer::StaBfCaps & /*bf*/) {
    return false;
  }

  /* Drive one HE sounding: hand the fw the NDPA -> NDP -> BFRP descriptor set (a
   * BFRP is an 802.11ax Trigger-frame variant) to build and air, soliciting the
   * beamformee's report as a hardware-scheduled HE TB PPDU (RxAtrib.ppdu_type ==
   * 10). Measured: the shipped client NIC firmware accepts the H2C but does not
   * air the sequence (the fw sounding-transmit engine is AP-firmware-only) — the
   * path that actually airs a Trigger on this firmware is host-injection
   * (SendTrigger). Returns false where unsupported. See docs/he-trigger-ul.md. */
  virtual bool StartSounding(const devourer::SoundingConfig & /*cfg*/) {
    return false;
  }

  /* Batch TX: submit `count` frames (each buffer = radiotap header + 802.11
   * MPDU, the send_packet contract) in one call. With USB TX aggregation
   * enabled (DeviceConfig tx.usb_agg_max > 0) the USB generations pack
   * consecutive frames into shared bulk-OUT URBs (see src/TxAggPlan.h) — one
   * transfer per burst instead of one per frame, and the frames land in the
   * TXDMA back-to-back. The default (and the PCIe / agg-off behaviour) is a
   * plain send_packet loop, so callers may use this unconditionally.
   * Semantics notes: a frame whose radiotap CHANNEL differs from the current
   * channel flushes the pending URB before the retune (per-packet hopping
   * stays radiotap-driven); a malformed frame is skipped. Returns the number
   * of frames successfully submitted. */
  virtual size_t send_packets(const TxPacketView *pkts, size_t count) {
    size_t ok = 0;
    for (size_t i = 0; i < count; ++i)
      if (pkts[i].data != nullptr && send_packet(pkts[i].data, pkts[i].len))
        ++ok;
    return ok;
  }

  virtual SelectedChannel GetSelectedChannel() = 0;

  /* Read the 64-bit hardware TSF (Timing Synchronization Function) timer — the
   * 802.11 MAC's free-running microsecond clock (REG_TSFTR). It runs off the
   * chip's crystal and is latched into every RX descriptor at receive
   * (rx_pkt_attrib::tsfl, the low 32 bits), so it is a precise, host-jitter-free
   * timing reference for multi-radio sync / TDOA / scheduled bursts. Returns 0
   * where unsupported (default). NB: a register read is a control transfer —
   * calling it concurrently with a heavy RX bulk-IN load can race (catch the
   * exception). */
  virtual uint64_t ReadTsf() { return 0; }

  /* Write the 64-bit MAC TSF (REG_TSFTR). Sets the free-running microsecond clock
   * — the primitive for TSF *adoption* (a slave slewing its clock onto the
   * master's, so its per-frame `tsfl` reads in the master's timebase). The
   * counter keeps running, so a read-add-write shifts by an approximate delta (a
   * control loop absorbs the read→write latency). NOTE: this moves the reported
   * TSF (and the beacon-body timestamp) but NOT the beacon TBTT air-time — a
   * separate per-port timer drives the TBTT (bench-proven). To steer the
   * hardware-timed beacon (the uplink timing-advance actuator) use
   * AdjustBeaconTiming. No-op where unsupported. */
  virtual void WriteTsf(uint64_t tsf) { (void)tsf; }

  /* Load a beacon into the beacon reserved-page + enable the MAC beacon function,
   * so the chip AUTO-TRANSMITS it at each TBTT — hardware-timed and
   * hardware-TSF-stamped (the MAC inserts the live 64-bit TSF into the beacon
   * timestamp at TX), fully host-jitter-free. `beacon` is a full 802.11 beacon
   * MPDU (a leading radiotap header, if present, is stripped); addr2/addr3 set
   * the port MAC/BSSID. `interval_tu` is the beacon interval in TU (1 TU =
   * 1024 µs). One call suffices — the hardware beacons indefinitely. Implemented
   * on all three generations (Jaguar2/3: HalMAC reserved-page download;
   * Jaguar1 incl. the 8814A: the pre-HalMAC BCNQ-boundary store bracket —
   * note the 8814A's stored beacon airs with the 802.11 sequence pinned at 0,
   * kernel rtw88 parity). See docs/time-distribution.md. */
  virtual bool StartBeacon(const uint8_t *beacon, size_t len,
                           int interval_tu) {
    (void)beacon; (void)len; (void)interval_tu;
    return false;
  }

  /* Replace the ACTIVE beacon's content in place — the dynamic-grant delivery
   * primitive (a scheduled MAC carries its DCI-style grant map in the beacon
   * body). Same buffer contract as StartBeacon (a leading radiotap header is
   * stripped; the raw 802.11 MPDU lands in the reserved page); the beacon
   * interval, TBTT phase and port identity are NOT touched — changing
   * addr2/addr3 mid-flight is unsupported (the port registers keep the
   * StartBeacon identity). Requires an active StartBeacon; returns false
   * otherwise. Rides the same reserved-page re-download the TBTT steers use,
   * so the cost bound is the steer's: at most one skipped beacon per update
   * while the valid latch re-arms, and the swap is NOT atomic versus TBTT (a
   * beacon airing during the download may still carry the previous content).
   * Measured per-generation skip/latency numbers: docs/scheduled-mac.md. */
  virtual bool UpdateBeaconPayload(const uint8_t *beacon, size_t len) {
    (void)beacon; (void)len;
    return false;
  }

  /* Stop the hardware beacon: EN_BCN_FUNCTION off + net_type back to No Link.
   * The chip beacons AUTONOMOUSLY once StartBeacon arms it — killing the host
   * process does NOT silence it (bench-bitten: a killed probe's beacon kept
   * airing and contaminated the next test's witness) — so any beaconing
   * session that ends without a device power-cycle must call this. Idempotent;
   * returns false when no beacon was active. */
  virtual bool StopBeacon() { return false; }

  /* Disable / restore the MAC EDCCA energy-detect gate (the vendor dis_cca
   * recipe). With EDCCA off the MAC does not defer TX to carrier-sense, so a
   * TBTT beacon airs exactly on schedule instead of after a CSMA backoff — the
   * lever that collapses the hardware-beacon downlink residual to sub-µs on a
   * shared channel (the master owns the channel). Also DEVOURER_DIS_CCA at
   * construction. Implemented on Jaguar2/3; a DELIBERATE no-op on Jaguar1,
   * whose baseband EDCCA is already disabled by its init table (0x8A4 =
   * 0x7F7F7F7F) — its hardware-beacon downlink measures ~0.34 µs RMS on a
   * crowded channel with no MAC-side gate, and porting the J2 register
   * recipe was bench-refuted (no gain; the 0x524[11] clear conflicts with
   * the vendor beacon-enable state). */
  virtual void SetCcaMode(bool disabled) { (void)disabled; }

  /* Shift the next hardware beacon TBTT by `microseconds` (>0 = later/retard,
   * <0 = earlier/advance), quantized to whole TU (1 TU = 1024 µs). One-shot
   * REG_BCN_INTERVAL tweak: runs one beacon interval at (nominal + round(µs/1024))
   * TU then restores nominal, so the next TBTT — and the cadence thereafter —
   * shifts by that many TU. This is the beacon-timing / uplink timing-advance
   * actuator: WriteTsf moves the reported TSF but NOT the TBTT air-time (a
   * separate per-port timer drives it), whereas the interval tweak steers it
   * deterministically (the 802.11 IBSS/TSF-merge mechanism; bench-proven to the
   * microsecond). Requires an active StartBeacon. BLOCKS the caller ~one beacon
   * interval (the tweaked interval must latch and fire once before restore).
   * Returns the actual applied shift in µs (TU-quantized); 0 if no active beacon
   * or |microseconds| < 512. Jaguar3 and Jaguar2: the J2 beacon engine loses its
   * bcn-valid latch on any TBTT re-latch (bench-proven), so the J2 path follows
   * the steer with a reserved-page re-download of the retained beacon — one
   * skipped beacon per correction. On Jaguar1 the interval tweak is inert
   * (bench-proven on the 8821AU), so the TU-quantized shift rides the fine
   * TSF-toggle mechanism instead — which also moves the reported TSF, unlike
   * J2/J3. Base is a no-op. */
  virtual int32_t AdjustBeaconTiming(int32_t microseconds) {
    (void)microseconds;
    return 0;
  }

  /* Fine (sub-TU, microsecond-granular) variant of AdjustBeaconTiming. Shifts the
   * next beacon TBTT by `microseconds` (>0 = later/retard, <0 = earlier/advance)
   * by toggling the MAC beacon function off, shifting the port-0 TSF, and toggling
   * it back on so the TBTT counter re-derives from the shifted TSF. Unlike the
   * interval-tweak AdjustBeaconTiming (quantized to whole TU), this steers at
   * microsecond resolution — the µs-fine uplink timing-advance actuator. Note it
   * also shifts this port's reported TSF (and the beacon-body timestamp) by the
   * same amount, which is the intended behaviour for a UE advancing its own
   * timebase. The USB read→write latency adds a sub-ms offset (~0.5–1.2 ms) that
   * a closed timing-advance loop absorbs — the *resolution* is microseconds.
   * Requires an active StartBeacon; returns the applied shift in µs. All three
   * generations: the J2 and J1 beacon engines drop their bcn-valid latch on the
   * toggle, so those paths re-download the retained reserved-page beacon after
   * the re-latch — one skipped beacon per correction. The 8814A additionally
   * pulses DUAL_TSF_RST (its TBTT counter free-runs across the toggle), which
   * re-derives the grid ABSOLUTELY from the shifted TSF — each steer also
   * cancels accumulated drift, so the TBTT ends up TSF-locked. Base is a
   * no-op. */
  virtual int32_t AdjustBeaconTimingFine(int32_t microseconds) {
    (void)microseconds;
    return 0;
  }

  /* TSF-PRESERVING µs-fine TBTT actuator: pin the beacon TBTT so it fires at
   * TSF % interval == offset_us, WITHOUT disturbing the reported TSF.
   * AdjustBeaconTimingFine necessarily jumps the TSF (the TBTT re-derives from
   * a shifted TSF), which corrupts any controller whose phase estimate is a
   * fit against this port's TSF (ref = a·tsf + b — every steer breaks the
   * slope, and a high-authority loop chases the corrupted estimate into a
   * limit cycle; bench-observed on the 8821CE AP↔PTP loop). This variant does
   * the same shift + re-latch, then immediately writes the TSF back onto its
   * original timeline — a bare TSF write does not move the TBTT (bench-proven),
   * so the steered TBTT phase survives while the clock the loop reads stays
   * continuous. The residual TSF discontinuity is one register-write latency:
   * ~µs over PCIe MMIO, ~0.5–1 ms over USB (so on USB this only pays off for
   * steers larger than that).
   *
   * ABSOLUTE semantics (unlike the incremental AdjustBeaconTimingFine): each
   * call re-pins the TBTT to the TSF grid, so consecutive calls don't
   * accumulate, and a PTP-disciplined TSF drags the pinned TBTT with it
   * between corrections — the fit-free discipline pattern. Requires an active
   * StartBeacon; returns the applied offset (normalized into the beacon
   * period), 0 if no active beacon.
   *
   * Per generation (all bench-proven): Jaguar2 — full support, ~10 µs TSF
   * disturbance over PCIe MMIO (8821CE; the AP↔PTP loop holds ~±1 µs with
   * it). Jaguar3 — full support over USB (~0.5–1.5 ms restore-write
   * disturbance; still far below a fine steer's full-magnitude jump).
   * Jaguar1 — offset 0 only (arm/re-derive): its TBTT is hardware-locked to
   * the TSF grid, so a nonzero TSF-preserving pin cannot hold (refused); the
   * flip side is that steering/disciplining the J1 TSF steers the TBTT with
   * it in hardware, no actuator needed. Base is a no-op. */
  virtual int32_t PinBeaconTbtt(int32_t offset_us) {
    (void)offset_us;
    return 0;
  }

  /* Clean shutdown: halt TRX DMA and power the chip down to a re-enumerable
   * state (mirrors the kernel driver's card-disable on unbind). Call after the
   * RX/TX loop exits and BEFORE releasing/closing the USB interface, so the
   * adapter isn't abandoned with its USB core hung. Default no-op. */
  virtual void Stop() {}

  /* Runtime TX-mode default: the modulation/rate/BW/GI/FEC used when a frame's
   * radiotap carries no rate (per-packet radiotap always wins). Both chip
   * families implement it; default no-op keeps other impls unaffected. Without
   * it, a rate-less frame falls back to MGN_1M — e.g. an MCS7 flood would
   * silently go on-air at 1 Mbps. */
  virtual void SetTxMode(const devourer::TxMode & /*mode*/) {}
  virtual void ClearTxMode() {}

  /* TX submission health snapshot (see TxStats.h) — the driver-side drop /
   * congestion signal an adaptive-link controller uses to detect a full TX FIFO
   * (a bulk-OUT TIMEOUT = recoverable back-pressure) vs a hard error. Counted at
   * the shared USB bulk-OUT layer, so every generation reports it. Default is an
   * all-zero snapshot. */
  virtual devourer::TxStats GetTxStats() { return {}; }

  /* Frame-free RX energy / channel-busy snapshot (see RxSense.h) — the read side
   * of the DEVOURER_CW_TONE emitter, used for spectrum-sensing / interferer
   * detection. Reads the chip's phydm false-alarm + CCA counters, DIG/IGI, and
   * (optionally) the NHM power histogram. FA/CCA counts are the delta since the
   * previous call. Default returns an all-invalid snapshot; each generation
   * overrides with a real reader. */
  virtual RxEnergy GetRxEnergy() { return {}; }

  /* Consolidated windowed RX link-quality snapshot (see RxQuality.h) — the
   * runtime feed a closed-loop adaptive-link controller reads instead of
   * scraping the demo's stdout. Fuses the per-frame RSSI/SNR/EVM aggregate the
   * device accumulates internally, a passive noise-floor estimate (rssi - snr,
   * the self-jamming signal), the frame-free FA/CCA/IGI energy, and the
   * LinkHealth verdict. Drains the window (delta semantics) and SUBSUMES
   * GetRxEnergy (it calls it internally + consumes the FA/CCA delta — don't also
   * poll GetRxEnergy separately on the same cadence). Default is an all-invalid
   * snapshot; each generation overrides. */
  virtual devourer::RxQuality GetRxQuality() { return {}; }

  /* --- Adapter-health probes (see src/AdapterHealth.h; examples/doctor is
   * the reference consumer) --- */

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

  /* Outcome of the most recent firmware download (populated during
   * Init/InitWrite). On Jaguar1 a failed FW boot does not abort bring-up —
   * this is the only place the failure is visible to a caller. */
  virtual devourer::FwBootStatus GetFwBootStatus() { return {}; }
};

#endif /* IRTL_DEVICE_H */
