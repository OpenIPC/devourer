#ifndef RTL_JAGUAR2_DEVICE_H
#define RTL_JAGUAR2_DEVICE_H

#include <atomic>
#include <optional>
#include <thread>

#include "logger.h"
#include "IRtlDevice.h"
#include "TxMode.h"
#include "RtlUsbAdapter.h"
#include "SelectedChannel.h"
#include "HalJaguar2.h"
#include "HalmacJaguar2MacInit.h"
#include "HalmacJaguar2Fw.h"
#include "ChipVariant.h"

/* RtlJaguar2Device is the orchestrator for the Realtek "Jaguar2" 802.11ac family
 * — RTL8822BU (chip 8822B, 2T2R, USB). It is the Jaguar2 sibling of
 * RtlJaguarDevice (Jaguar1) and RtlJaguar3Device (Jaguar3) and implements the
 * same IRtlDevice contract so the demos and the WiFiDriver factory treat all
 * three uniformly.
 *
 * Jaguar2 is a hybrid of the two existing generations: firmware download, MAC
 * init and power sequencing follow the HalMAC path (like Jaguar3), while the
 * phydm BB/AGC/RF register tables use the older `check_positive` format (like
 * Jaguar1, via the shared PhyTableLoader). Bring-up is ported from the vendor
 * rtl88x2bu (8822B) / rtl8821cu (8821C) trees.
 *
 * Multi-chip via jaguar2::ChipVariant (C8822B 2T2R / C8821C 1T1R), selected in
 * the WiFiDriver factory from the SYS_CFG2 chip-id and threaded into HalJaguar2
 * (Jaguar2PhyTables table data + RF-path count), HalmacJaguar2Fw (blob) and the
 * Jaguar2Calibration IQK factory — the same strategy-dispatch shape as the
 * Jaguar3 8822C/8822E HAL. */
class RtlJaguar2Device : public IRtlDevice {
public:
  RtlJaguar2Device(RtlUsbAdapter device, Logger_t logger,
                   jaguar2::ChipVariant variant = jaguar2::ChipVariant::C8822B);
  ~RtlJaguar2Device() override;

  void Init(Action_ParsedRadioPacket packetProcessor,
            SelectedChannel channel) override;
  /* Blocking RX worker loop on an already-brought-up chip (see IRtlDevice).
   * Init = bring_up + StartRxLoop; a TX+RX caller does InitWrite once, then
   * runs this on its own std::thread next to the TX loop. Starts (and on exit
   * stops) the DIG thread — TX-only sessions stay DIG-free. */
  void StartRxLoop(Action_ParsedRadioPacket packetProcessor) override;
  void StopRxLoop() override { _rx_stop = true; }
  void SetMonitorChannel(SelectedChannel channel) override;
  /* Lean frequency-hop retune (the Jaguar2 port of the Jaguar1 FastRetune —
   * see docs/frequency-hopping.md): composed cached writes for RF18 + the
   * on-change channel constants, via HalJaguar2::fast_retune; falls back to
   * the full set_channel_bw on a band change. Same concurrency model as
   * SetMonitorChannel: no lock — the DIG thread (RX sessions only) writes its
   * own registers (IGI/FA), and TX-only sessions have no concurrent register
   * writer. cache_rf=false re-primes the compose cache per hop (A/B
   * measurement). */
  void FastRetune(uint8_t channel, bool cache_rf) override;
  void InitWrite(SelectedChannel channel) override;
  void SetTxPower(uint8_t power) override;
  bool send_packet(const uint8_t *packet, size_t length) override;
  SelectedChannel GetSelectedChannel() override;
  void Stop() override;
  void SetTxMode(const devourer::TxMode &mode) override;
  void ClearTxMode() override;

  /* Realtek MP single-tone (CW carrier) — radiate a bare RF local-oscillator
   * carrier at the tuned channel center. Path A; both Jaguar2 variants, per the
   * vendor hal_mpt_SetSingleToneTx() branches: OFDM/CCK modulators off, RF path A
   * to TX mode at `gain` (RF 0x00[4:0]) with the LO enabled. 8822B forces the
   * RFE pinmux (0x77777777) + RFE-inverse; 8821C uses its own path-A pinmux
   * (0xCB0[0xF0F0]=0x707) and gates the LO via RF 0x75[16] on 2.4 GHz (BTG) /
   * RF 0x58[1] on 5 GHz. StopCwTone() restores the saved state and disables the
   * LO. Idempotent. A controllable narrowband interferer / MP tone source. */
  void StartCwTone(uint8_t gain);
  void StopCwTone();

  /* Modulated continuous TX — sibling of StartCwTone. Streams a 100%-duty
   * modulated OFDM carrier via the vendor 0x914 continuous mode. The 0x914 bit
   * alone wedges the USB TX FIFO; the fix is rCCAonSec (0x838)=0x6d first, after
   * which the carrier self-radiates from BB state (no send_packet feed needed —
   * the demo idle-holds). SDR-verified as a flat ~18 MHz OFDM block.
   * StopContinuousTx clears the mode, pulses a BB reset, restores 0x838. */
  void StartContinuousTx(const devourer::TxMode& mode);
  void StopContinuousTx();

  /* Frame-free RX energy snapshot (see RxSense.h) — the FA/CCA/IGI values
   * dig_step samples over its ~100 ms window, plus a fresh NHM power histogram.
   * The read side of the CW tone. */
  RxEnergy GetRxEnergy() override;

private:
  RtlUsbAdapter _device;
  Logger_t _logger;
  jaguar2::ChipVariant _variant;
  jaguar2::HalJaguar2 _hal;
  jaguar2::HalmacJaguar2MacInit _macinit;
  jaguar2::HalmacJaguar2Fw _fw;
  SelectedChannel _channel{};
  Action_ParsedRadioPacket _packetProcessor = nullptr;
  int _tx_pwr_override = -1;
  /* rfe_type resolved during bring_up (efuse + DEVOURER_RFE), cached so
   * SetMonitorChannel can retune (set_channel_bw needs it). */
  uint8_t _rfe = 0;
  std::optional<devourer::TxMode> _tx_mode_default;

  /* CW single-tone (StartCwTone/StopCwTone) saved state for a clean restore:
   * the pre-tone RF 0x00 (path A) and the four RFE-pinmux BB words
   * (0xCB0/0xEB0/0xCB4/0xEB4). _cw_active guards double start/stop. */
  bool _cw_active = false;
  uint32_t _cw_rf00 = 0;
  uint32_t _cw_bb[4] = {0, 0, 0, 0};

  /* Modulated continuous TX (StartContinuousTx/StopContinuousTx) guard + saved
   * pre-continuous rCCAonSec (0x838) for a clean restore. */
  bool _cont_active = false;
  uint32_t _cont_cca838 = 0;

  /* DIG (dynamic initial gain) background thread — periodically runs
   * HalJaguar2::dig_step so IGI tracks the false-alarm rate for weak-signal RX. */
  std::thread _dig_thread;
  std::atomic<bool> _dig_stop{false};
  void stop_dig();

  /* StartRxLoop stop request (StopRxLoop). volatile (not atomic) to match the
   * signal-flag pattern used across the library (g_devourer_should_stop). */
  volatile bool _rx_stop = false;

  /* Shared cold bring-up (power-on -> DLFW -> MAC/BB/RF -> channel -> LCK ->
   * IQK -> coex -> enable RX/TX engine). Used by both Init (RX) and InitWrite
   * (TX). Leaves the chip fully calibrated on `channel`. */
  void bring_up(SelectedChannel channel);
};

#endif /* RTL_JAGUAR2_DEVICE_H */
