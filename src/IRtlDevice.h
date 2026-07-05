#ifndef IRTL_DEVICE_H
#define IRTL_DEVICE_H

#include <cstddef>
#include <cstdint>
#include <functional>

#include "RxSense.h"
#include "SelectedChannel.h"
#include "TxMode.h"

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
  virtual void SetTxPower(uint8_t power) = 0;
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
