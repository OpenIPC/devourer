#ifndef IRTL_DEVICE_H
#define IRTL_DEVICE_H

#include <cstddef>
#include <cstdint>
#include <functional>

#include "RxSense.h"
#include "SelectedChannel.h"
#include "ThermalStatus.h"
#include "TxMode.h"
#include "TxPower.h"

/* Packet is the parsed-RX type handed to the RX callback. Forward-declared here
 * (a reference in the std::function signature needs only an incomplete type) so
 * the interface header stays light; the full definition lives in FrameParser.h. */
struct Packet;

using Action_ParsedRadioPacket = std::function<void(const Packet &)>;

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

  /* Force a flat absolute TXAGC index across all rates (the debug /
   * SDR-visibility knob — kept for source compatibility; new callers should
   * prefer SetTxPowerIndexOverride, which is the same knob with an explicit
   * clear). Live on a brought-up chip, recorded-and-applied-at-InitWrite
   * otherwise. */
  virtual void SetTxPower(uint8_t power) = 0;

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
   * baseline. Default forwards to the legacy SetTxPower for generations that
   * predate the unified API. */
  virtual void SetTxPowerIndexOverride(int idx) {
    if (idx >= 0)
      SetTxPower(static_cast<uint8_t>(idx));
  }

  /* Re-program the TX-power registers from the current knob state at the
   * CURRENT channel — the hook tests use to force a re-apply without moving
   * any knob. Returns false when unsupported or the chip isn't brought up. */
  virtual bool ReApplyTxPower() { return false; }

  /* Snapshot of the knob state + representative effective indices (register
   * readback where the family's TXAGC block is readable). */
  virtual devourer::TxPowerState GetTxPowerState() { return {}; }

  /* Chip thermal-meter snapshot (RF 0x42 family; efuse baseline where the
   * family wires one — see src/ThermalStatus.h). The PA-heating input of the
   * adaptive-link controller. Default returns an all-invalid reading. */
  virtual devourer::ThermalStatus GetThermalStatus() { return {}; }

  virtual bool send_packet(const uint8_t *packet, size_t length) = 0;
  virtual SelectedChannel GetSelectedChannel() = 0;

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

  /* Frame-free RX energy / channel-busy snapshot (see RxSense.h) — the read side
   * of the DEVOURER_CW_TONE emitter, used for spectrum-sensing / interferer
   * detection. Reads the chip's phydm false-alarm + CCA counters, DIG/IGI, and
   * (optionally) the NHM power histogram. FA/CCA counts are the delta since the
   * previous call. Default returns an all-invalid snapshot; each generation
   * overrides with a real reader. */
  virtual RxEnergy GetRxEnergy() { return {}; }
};

#endif /* IRTL_DEVICE_H */
