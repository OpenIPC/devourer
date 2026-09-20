#ifndef RTL8733B_DEVICE_H
#define RTL8733B_DEVICE_H

#include <atomic>
#include <mutex>
#include <optional>

#include "DeviceConfig.h"
#include "FrameParser8733b.h"
#include "Halmac8733bMac.h"
#include "IRtlRadio.h"
#include "Phy8733b.h"
#include "Rtl8733bBringup.h"
#include "RtlAdapter.h"
#include "SelectedChannel.h"
#include "logger.h"

/* Dedicated RTL8733B IRadio boundary. Power, firmware, EFUSE, HALMAC,
 * PHY/RF, monitor RX, and bounded legacy/HT injection all use the production
 * path. Unsupported optional controls refuse loudly rather than silently
 * no-opping, but a refusal never tears the session down — asking for a knob
 * this backend has not ported is not a hardware-safety event. */
class Rtl8733bDevice : public IRtlRadio {
public:
  Rtl8733bDevice(RtlAdapter device, Logger_t logger,
                 devourer::DeviceConfig cfg = {});
  ~Rtl8733bDevice() override;

  void Init(Action_ParsedRadioPacket packetProcessor,
            SelectedChannel channel) override;
  void InitWrite(SelectedChannel channel) override;
  void StartRxLoop(Action_ParsedRadioPacket packetProcessor) override;
  void StopRxLoop() override { _rx_stop = true; }
  void SetMonitorChannel(SelectedChannel channel) override;
  /* Lean intra-band, same-bandwidth hop (see Phy8733b::fast_retune — the
   * profile that sized it and the TSSI in-place contract live there). Falls
   * back to the full SetMonitorChannel on a band/width change or a cold
   * radio, per the IRadio contract. The cache_rf default binds at the
   * interface declaration. */
  void FastRetune(uint8_t channel, bool cache_rf) override;
  bool send_packet(const uint8_t *packet, size_t length) override;
  /* USB TX aggregation (cfg.tx.usb_agg_max / DEVOURER_TX_USB_AGG): pack
   * consecutive frames into one bulk-OUT URB. Measured on the CV610 craft,
   * this is the lever that matters on an embedded host — a single submission
   * costs ~248 us of CPU there against ~22 us on x86, and ~87% of that is the
   * kernel USB submit/completion path, so folding three frames into one URB
   * removes two of every three (measured: 248 -> 148 us per frame on the
   * craft, frame rate unchanged). Knob off / non-USB falls back to the
   * interface-default per-frame loop, byte-identical.
   *
   * Note when reading TX stats against this: GetTxStats().submitted counts
   * bulk-OUT transfers, so an aggregated session reports roughly frames/3 —
   * the same accounting Jaguar1/2/3 have, not a throughput drop. The
   * per-URB `tx.agg` event carries the real frame count. */
  size_t send_packets(const TxPacketView *pkts, size_t count) override;
  void SetTxMode(const devourer::TxMode &mode) override;
  void ClearTxMode() override;
  SelectedChannel GetSelectedChannel() override;
  bool SetAckResponder(const devourer::MacAddr &mac) override;
  void ClearAckResponder() override;
  /* rxdemo's RTL8733B-only live-disarm measurement hook. The delay starts
   * after Init has completed the locked bring-up (including a configured
   * SetAckResponder), never from process start. */
  void ScheduleAckResponderDisarmForTest(uint32_t delay_ms) {
    _ack_disarm_after_ms = delay_ms;
  }
  /* Shared by ClearAckResponder and SetAckResponder rollback. */
  bool disarm_ack_responder();
  void SetCcaMode(bool disabled) override;
  void Stop() override;

  devourer::TxCaps GetTxCaps() override;
  devourer::AdapterCaps GetAdapterCaps() override;
  /* Runtime TX power. Only the relative offset is ported: on a TSSI-offset PG
   * unit the closed loop is the power control, and moving its target is the
   * one lever this part has that was measured to work. The flat-index and
   * per-rate-diff knobs stay on IRadio's not-ported defaults —
   * kSafeTxAgcIndex8733b was witnessed unable to carry HT at all, and no
   * dB-per-step slope has been measured for the index. */
  devourer::TxPowerCaps GetTxPowerCaps() override;
  int SetTxPowerOffsetQdb(int qdb) override;
  devourer::TxPowerState GetTxPowerState() override;
  /* Overridden only to refuse out loud. IRadio's default returns void and
   * ignores the value, so on this backend — where the flat index is genuinely
   * unported — silence would be the caller's only answer, and a knob that
   * looks granted is precisely the defect this family's offset knob was added
   * to fix. SetTxPowerRateDiffs needs no such override: its `false` return
   * already says it. */
  void SetTxPowerIndexOverride(int idx) override;
  devourer::TxStats GetTxStats() override { return _device.GetTxStats(); }
  devourer::ThermalStatus GetThermalStatus() override;
  bool GetPermanentMacAddress(uint8_t out[6]) override;
  devourer::EfuseStability ProbeEfuseStability(int reads) override;
  devourer::FwBootStatus GetFwBootStatus() override;

private:
  /* This generation's CCX map and register access, under its locks — see
   * IRtlRadio::with_ccx. Private: the base class calls it, nobody else.
   *
   * The JGR3 map, and the proof is in this tree: hal/hal8733b_tables.c:792
   * onward, the shipped 8733B phy_reg init table, programs 0x1e40, 0x1e44,
   * 0x1e48, 0x1e5c and 0x1e60 at bring-up — the CCX block, at the JGR3
   * addresses, on this die. The vendor agrees: at the pinned
   * reference/rtl8733bu-20230626, hal/phydm/phydm_pre_define.h:513 lists
   * ODM_RTL8733B in PHYDM_IC_SUPPORT_IFS_CLM, and :523-525 define
   * PHYDM_IC_JGR3_SERIES_SUPPORT when RTL8733B_SUPPORT is set.
   *
   * That init table is also the only other writer of these registers:
   * nothing in src/rtl8733b/ touches 0x1e40-0x1e60 or 0x2d88 at runtime,
   * so the masked read-modify-writes below cannot corrupt another
   * subsystem's state. arm_clm_only never touches 0x1e5c, where the table
   * leaves a non-zero value.
   *
   * CLM is reachable here even though GetRxEnergy is not overridden:
   * arm_clm_only() and read_clm_only() take no IGI argument, because busy
   * airtime is a tick count and needs no receiver-noise reference the way
   * NHM's IGI-referenced thresholds do. So this backend answers
   * GetChannelBusy through an armed window while its sampled path still
   * reports no reading, and rx_energy_ok stays false. */
  bool with_ccx(const CcxFn &fn) override {
    std::lock_guard<std::recursive_mutex> reg(_reg_mu);
    /* Nothing to lend before the BB is programmed: a window armed against it
     * would be forgotten by Init/InitWrite's reset. _phy_ready is this
     * family's _brought_up, and it goes true the moment _phy.initialize()
     * succeeds rather than at the end of bring_up_to_phy() — which is the
     * right point for THIS question, because everything after it is MAC-level
     * (the ACK window, the ACK responder) and does not bear on whether a BB
     * register is safe to touch. Cleared by Stop().
     *
     * Read UNDER _reg_mu, unlike the Jaguar2/3 overrides this is modelled on.
     * It is a plain bool written under that lock by bring_up_to_phy() and
     * Stop(), so checking it before taking the lock is both a data race and a
     * TOCTOU: the check passes, the lock then blocks behind a concurrent
     * Stop(), and the register access below proceeds against a card that has
     * just been powered down. Costs nothing to do it in the right order. */
    if (!_phy_ready)
      return false;
    std::lock_guard<std::mutex> ccx(busy_window_mutex());
    const Read32 rd = [this](uint16_t a) {
      return _device.rtw_read<uint32_t>(a);
    };
    const SetBb wr = [this](uint16_t a, uint32_t m, uint32_t v) {
      _device.phy_set_bb_reg(a, m, v);
    };
    fn(devourer::nhm_regs_jgr3(), rd, wr);
    return true;
  }

  void bring_up_to_phy();
  bool configure_tx_power(SelectedChannel channel);
  /* Fill one [txdesc][frame] block at `out`. `agg_num` is the USB TX
   * aggregation block count and belongs on the FIRST descriptor of a packed
   * URB only; 0 everywhere else, which is what keeps the single-frame path
   * byte-identical. */
  size_t build_tx_block(const uint8_t *packet, size_t length, uint8_t *out,
                        uint8_t packet_offset, uint8_t agg_num = 0);

  RtlAdapter _device;
  Logger_t _logger;
  devourer::DeviceConfig _cfg;
  rtl8733b::Rtl8733bBringup _bringup;
  rtl8733b::Halmac8733bMac _mac;
  rtl8733b::Phy8733b _phy;
  rtl8733b::EfuseInfo _efuse{};
  SelectedChannel _channel{1, 0, CHANNEL_WIDTH_20};
  rtl8733b::ChipInfo _chip{};
  devourer::FwBootStatus _fw_boot{.supported = true};
  bool _power_ready = false;
  bool _firmware_ready = false;
  bool _mac_ready = false;
  bool _phy_ready = false;
  bool _tx_ready = false;
  bool _tssi_tracking = false;
  /* Session TX-power offset in qdB, over the int8 delta field's full
   * [-128, +127] (GetTxPowerCaps argues the range and records where the chip
   * stops responding at each end), plus the rails the last apply hit. Sticky
   * by construction: configure_tx_power folds it back in on every channel set,
   * and FastRetune passes it to the in-place hop rewrite. */
  int16_t _tx_offset_qdb = 0;
  bool _tx_sat_low = false;
  bool _tx_sat_high = false;
  bool _tx_readback_warned = false;
  std::atomic<bool> _rx_stop{false};
  std::atomic<bool> _rx_active{false};
  std::atomic<uint8_t> _rx_configured_bw{0};
  std::atomic<uint64_t> _tx_submits{0};
  mutable std::recursive_mutex _reg_mu;
  std::optional<devourer::TxMode> _tx_mode_default;
  std::optional<uint32_t> _ack_disarm_after_ms;
};

#endif /* RTL8733B_DEVICE_H */
