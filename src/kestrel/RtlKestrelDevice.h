#ifndef RTL_KESTREL_DEVICE_H
#define RTL_KESTREL_DEVICE_H

#include <cstdint>
#include <optional>
#include <thread>

#include "TxMode.h"

#include "logger.h"
#include "DeviceConfig.h"
#include "IRtlDevice.h"
#include "RtlAdapter.h"
#include "RxQuality.h" /* RxQualityAccumulator + build_rx_quality */
#include "SelectedChannel.h"

#include "ChipVariant.h"
#include "HalKestrel.h"

namespace kestrel {

/* AX-native chip identity, read via ReadChipInfo(). On this generation the
 * byte at 0x00FC is R_AX_SYS_CHIPINFO (the D-die id: 0x51 = 8852B, 0x52 =
 * 8852C) and the cut version is the high nibble of R_AX_SYS_CFG1+1 (0x00F1):
 * 0 = CAV, 1 = CBV, 2 = CCV, ... — the mac_ax get_chip_info() read pair. */
struct ChipInfo {
  uint8_t die_id = 0; /* R_AX_SYS_CHIPINFO (0x00FC) */
  uint8_t cut = 0;    /* rtw_cv: R_AX_SYS_CFG1[15:12] */
  bool matches(ChipVariant v) const {
    return (v == ChipVariant::C8852B && die_id == 0x51) ||
           (v == ChipVariant::C8852C && die_id == 0x52);
  }
};

} /* namespace kestrel */

/* RtlKestrelDevice is the orchestrator for the Realtek "Kestrel" Wi-Fi 6 /
 * 802.11ax family (G6 "phl" vendor architecture) — RTL8852BU/8832BU and
 * RTL8852CU/8832CU. It implements the same IRtlDevice contract as the three
 * Jaguar (11ac) generations so the demos and WiFiDriver factory treat all
 * four uniformly.
 *
 * Ported from the vendor trees reference/rtl8852bu (v1.19.21) and
 * reference/rtl8852cu (v1.19.22). The layer split: the mac_ax plane (identity,
 * power-on, firmware download, efuse, MAC TRX init, H2C/C2H, descriptors) is
 * hand-ported C++ here + HalKestrel/KestrelFw; the halbb PHY and halrf
 * calibration planes are the vendor C compiled verbatim (hal/halbb, hal/halrf)
 * behind the kestrel glue. */
class RtlKestrelDevice : public IRtlDevice {
public:
  RtlKestrelDevice(RtlAdapter device, Logger_t logger,
                   kestrel::ChipVariant variant = kestrel::ChipVariant::C8852B,
                   devourer::DeviceConfig cfg = {});
  ~RtlKestrelDevice() override;

  void Init(Action_ParsedRadioPacket packetProcessor,
            SelectedChannel channel) override;
  void InitWrite(SelectedChannel channel) override;
  void StartRxLoop(Action_ParsedRadioPacket packetProcessor) override;
  void StopRxLoop() override { _rx_stop = true; }
  /* Join the WP-release drain thread (started by InitWrite) before the caller
   * tears libusb down — a thread still inside libusb_handle_events when
   * libusb_exit runs trips libusb's usbi_mutex_destroy assertion (SIGABRT at
   * demo shutdown). The destructor joins too, but the demos destroy the
   * device object after libusb_exit, so Stop() is the ordered join point. */
  void Stop() override {
    _rx_stop = true;
    stop_wp_drain();
  }
  void SetMonitorChannel(SelectedChannel channel) override;
  /* Disable / restore the MAC carrier-sense gate (R_AX_CCA_CFG_0 all-CCA-EN:
   * primary + sec20/40/80 + EDCCA). Injection is already CCA-off by default here
   * (EnableTxScheduler clears these gates), so this is a runtime toggle, not the
   * co-channel-deferral fix it is on Jaguar. */
  void SetCcaMode(bool disabled) override;
  bool send_packet(const uint8_t *packet, size_t length) override;
  devourer::TxStats GetTxStats() override { return _device.GetTxStats(); }
  SelectedChannel GetSelectedChannel() override { return _channel; }

  /* Static capability aggregate (resolved from chip identity; thread-safe,
   * callable pre-Init). The demos emit it as the adapter.caps JSONL event. */
  devourer::TxCaps GetTxCaps() override;
  devourer::AdapterCaps GetAdapterCaps() override;

  /* Runtime TX-power lever: fixed-dBm BB power (halbb_set_txpwr_dbm) with a
   * quarter-dB offset relative to the DEVOURER_TX_PWR base. Sticky across
   * SetMonitorChannel. Per-packet: a radiotap DBM_TX_POWER dB-delta on a
   * frame overrides this for that frame via a fixed-dBm rewrite in
   * send_packet (2 BB RMWs on value change, free while constant). */
  devourer::TxPowerCaps GetTxPowerCaps() override;
  int SetTxPowerOffsetQdb(int qdb) override;
  devourer::TxPowerState GetTxPowerState() override;

  /* Runtime TX-mode default (DEVOURER_TX_RATE): rate/BW/GI/LDPC/STBC applied to
   * a rate-less frame (e.g. the demo beacon). A per-packet radiotap rate always
   * wins. Mirrors the Jaguar behaviour so DEVOURER_TX_RATE works uniformly. */
  void SetTxMode(const devourer::TxMode &mode) override;
  void ClearTxMode() override;

  /* 64-bit free-running MAC TSF (band-0 port-0). mac_get_tsf (twt.c). */
  uint64_t ReadTsf() override;

  /* Lean intra-band 20 MHz retune (frequency hopping) — RF channel only, skips
   * the BB bandwidth config + RX-DCK. Falls back to SetMonitorChannel on a band
   * change or a non-20 MHz width. */
  void FastRetune(uint8_t channel, bool cache_rf = true) override;

  /* Lean same-channel 20 <-> 5/10 MHz narrowband toggle (BB small-BW field
   * only). Falls back to SetMonitorChannel for 40/80 MHz. */
  void FastSetBandwidth(ChannelWidth_t bw) override;

  /* Chip thermal-meter snapshot (halrf_get_thermal_8852b, RF path-A 0x42) with
   * the efuse baseline. Control-thread only (does an RF read). */
  devourer::ThermalStatus GetThermalStatus() override;

  /* Frame-free RX energy snapshot. On Kestrel this carries only the active
   * absolute noise floor (halbb NHM env-monitor) when DEVOURER_RX_NOISE_FLOOR is
   * set — there is no phydm FA/CCA/IGI DIG monitor on this generation. */
  RxEnergy GetRxEnergy() override;
  /* Windowed RX link-quality: the passive rssi-snr floor + LinkHealth verdict
   * (fed per frame via _rxq) fused with the active NHM floor from GetRxEnergy. */
  devourer::RxQuality GetRxQuality() override;

  /* Arm the AX HW beacon engine (mac_send_bcn_h2c + AP port timing). Requires a
   * prior InitWrite. `beacon` is a full 802.11 beacon; the MAC airs it every
   * `interval_tu` TU with the live TSF inserted. */
  bool StartBeacon(const uint8_t *beacon, size_t len, int interval_tu) override;

  /* 802.11ax scheduled UL: air an HE Basic Trigger (UL-OFDMA grant), program
   * TWT agreements + STA binds, and the fw-autonomous trigger cadence
   * (TWT-OFDMA or the UL_FIXINFO table). Require a prior InitWrite. */
  bool SendTrigger(const devourer::TriggerConfig &cfg) override;
  bool ConfigureTwt(const devourer::TwtConfig &cfg) override;
  bool TeardownTwt(const devourer::TwtConfig &cfg) override;
  bool TwtBindSta(const devourer::TwtStaAct &act) override;
  bool ConfigureTwtOfdma(const devourer::TwtOfdmaConfig &cfg) override;
  bool ConfigureUlOfdma(const devourer::UlOfdmaConfig &cfg) override;

  /* Register an associated peer STA (macid/addr-cam) so a Trigger's per-user
   * grant scores against it — the AP-side companion to SendTrigger for the
   * end-to-end UL path. */
  bool RegisterPeerSta(const uint8_t peer_mac[6], uint8_t macid,
                       uint8_t addr_cam_idx) override;

  /* HE sounding command surface: register an associated beamformee, then drive a
   * sounding intended to air NDPA -> NDP -> BFRP and receive the report HE TB
   * PPDU. NB the shipped client NIC fw accepts SET_SND_PARA but does not air it
   * (AP-firmware-only transmit engine); see docs/he-trigger-ul.md. */
  bool RegisterBeamformee(const uint8_t peer_mac[6], uint8_t macid,
                          uint8_t addr_cam_idx,
                          const devourer::StaBfCaps &bf) override;
  bool StartSounding(const devourer::SoundingConfig &cfg) override;

  /* AX-native identity read (no power-on needed — the identity block is alive
   * as soon as the USB function enumerates). kestrelprobe stage "id" and the
   * constructor's confirmation log both come through here. */
  kestrel::ChipInfo ReadChipInfo();

  kestrel::ChipVariant variant() const { return _variant; }

  /* Staged bring-up, exposed for kestrelprobe's "power" stage: power the MAC
   * on and dump the efuse. Not part of the IRtlDevice contract (Init/InitWrite
   * drive the full sequence). Returns false on failure. */
  bool PowerOnAndReadEfuse(kestrel::EfuseInfo &out);

  /* kestrelprobe "fw" stage: power on, read efuse, then download firmware.
   * Returns false at the first failing step. */
  bool PowerOnEfuseAndFw(kestrel::EfuseInfo &out);

  /* kestrelprobe "trx" stage: everything in "fw" plus the DMAC-half MAC TRX
   * init. Returns false at the first failing step. */
  bool PowerOnFwAndTrx(kestrel::EfuseInfo &out);

  /* kestrelprobe "phy" stage: everything in "trx" plus the BB + RF table
   * apply. Returns false at the first failing step. */
  bool PowerOnTrxAndPhy(kestrel::EfuseInfo &out);

  kestrel::HalKestrel &hal() { return _hal; }

  /* Full monitor bring-up (power -> efuse -> fw -> MAC TRX -> BB/RF -> channel)
   * without starting the RX loop. Returns false at the first failing step.
   * Init() calls this then StartRxLoop; a probe "rx"/"chan" stage can call it
   * standalone. */
  bool BringUpMonitor(SelectedChannel channel);

  /* TX-only scheduler enablement (CMAC port + CTN_TXEN); clears the CCA gates,
   * so it is kept out of BringUpMonitor to leave RX-only CCA on. InitWrite
   * calls it after the shared bring-up. */
  void EnableTxScheduler();

private:
  /* Route a C2H firmware message (RPKT_TYPE_C2H payload). Currently decodes the
   * USR_TX_RPT_INFO report and emits its freerun TX-egress timestamp. */
  void handle_c2h(const uint8_t *payload, uint32_t len);

  RtlAdapter _device;
  Logger_t _logger;
  kestrel::ChipVariant _variant;
  devourer::DeviceConfig _cfg;
  kestrel::HalKestrel _hal;
  SelectedChannel _channel{};
  kestrel::EfuseInfo _efuse{};
  volatile bool _rx_stop = false;
  /* WP-release drain (Kestrel STF USB): the fw returns each transmitted frame's
   * WD/PLE pages as an RX packet (rpkt_type=7, TX_PD_RELEASE_HOST) on the bulk-
   * IN. A TX-only path must still drain the bulk-IN or the pages never recycle
   * and the mgmt bulk-OUT stalls after ~103 frames. InitWrite starts this
   * background drain; StartRxLoop hands the bulk-IN over to itself (which also
   * drains type-7), so the two never read the endpoint concurrently. */
  std::thread _wp_drain_thread;
  volatile bool _wp_drain_stop = true;
  void start_wp_drain();
  void stop_wp_drain();
  uint8_t _tx_mgmt_ep = 0; /* band-0 mgmt bulk-OUT ep (BULKOUTID0), 0=TX not up */
  uint8_t _tx_data_ep = 0; /* AC0 data bulk-OUT ep (BULKOUTID3) */
  uint16_t _tx_seq = 0;    /* rolling 12-bit wifi sequence for injected frames */
  std::optional<devourer::TxMode> _tx_mode_default; /* SetTxMode default */
  int16_t _sess_pwr_qdb = 0; /* offset applied by SetTxPowerOffsetQdb — the
                              * restore target for frames without a radiotap
                              * DBM_TX_POWER field */
  /* Per-chain RSSI (RSSI% = dBm+110) cached from the last PPDU-status physts
   * header, attached to the following WIFI frame(s) in the aggregate. */
  uint8_t _last_rssi[2] = {0, 0};
  /* Per-frame SNR (raw U(8,1) = dB*2), cached from the physts header alongside
   * RSSI for the passive noise floor (rssi_dbm - snr_db). 0 = unknown. */
  uint8_t _last_snr = 0;
  /* Windowed RX link-quality accumulator (passive noise floor + LinkHealth),
   * fed per decoded frame from the RX loop; drained by GetRxQuality. */
  devourer::RxQualityAccumulator _rxq;
};

#endif /* RTL_KESTREL_DEVICE_H */
