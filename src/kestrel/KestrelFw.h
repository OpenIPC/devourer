#ifndef KESTREL_KESTREL_FW_H
#define KESTREL_KESTREL_FW_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "ChipVariant.h"
#include "RtlAdapter.h"
#include "Sounding.h"
#include "TriggerTwt.h"
#include "logger.h"

namespace kestrel {

/* KestrelFw owns the firmware-download layer, ported from
 * reference/rtl8852bu mac_ax: the DMAC/DLE/HFC pre-init that stands up the H2C
 * (CH12) transport, then the three-phase FWDL state machine (header H2C ->
 * section chunks -> WCPU boot poll). Runs after HalKestrel::power_on.
 *
 * The firmware image is the embedded NICCE blob (hal8852b_fw.c), cut-selected:
 * cut CCV (>=2) -> array_8852b_u3_nicce, cut CBV -> u2. */
class KestrelFw {
public:
  KestrelFw(RtlAdapter device, Logger_t logger, ChipVariant variant);

  /* mac_hal_init pre-FWDL half: hci_func_en + dmac_pre_init (DLE/HFC for
   * DLFW). The caller then runs usb_pre_init (intf_pre_init, vendor order)
   * before download_firmware. */
  bool fw_pre_init();

  /* The FWDL state machine: WDT config + disable/enable WCPU + download the
   * NICCE image for `cut` + fw-ready poll. `mss_idx` selects the secure-boot
   * signature (HalKestrel::read_mss_index). `is_sec_ic` (OTP 0x5ED[7]) gates
   * the non-secure IDMEM + CPU-clock FWDL patch. Returns false on any poll
   * timeout or FW error status (checksum/security/cut mismatch), all logged. */
  bool download_firmware(uint8_t cut, uint8_t mss_idx, bool is_sec_ic);

  /* Send one radio-parameter page as an OUTSRC H2C (halrf radio-to-fw): `cls` =
   * OUTSRC_CL_RADIO_A/B, `page` = page index (H2C func), `packed` = the
   * (addr<<20|data) u32 array for this page, `count` entries. */
  bool radio_page_to_fw(uint8_t cls, uint8_t page, const uint32_t *packed,
                        uint16_t count);

  bool ch12_ready() const { return _ch12_ep != 0; }

  /* Enable the per-user TX report (mac_cfg_usr_tx_rpt): H2C cat=MAC,
   * class=FW_OFLD, func=USR_TX_RPT. `mode` = mac_ax_usr_tx_rpt_mode (PERIOD /
   * LAST_PKT), started immediately (RTP_START). The firmware then emits C2H
   * USR_TX_RPT_INFO reports carrying the freerun TX-egress timestamps. */
  bool enable_usr_tx_rpt(uint8_t mode, uint8_t macid, uint8_t port,
                         uint32_t period_us = 100000);

  /* Route the firmware log to C2H packets (mac_fw_log_cfg, class=FW_INFO
   * func=LOG_CFG, output=C2H). Diagnostic: a decisive probe of whether async
   * packet-C2H (rpkt_type=10) reaches the host at all — if fw-log C2H arrives,
   * packet-C2H delivery works and USR_TX_RPT is a fw-side gating quirk. */
  bool enable_fw_log_c2h();

  /* mac_fw_role_maintain (role.c): H2C cat=MAC, class=MEDIA_RPT,
   * func=FWROLE_MAINTAIN — makes the firmware create/track a MACID role. This
   * is the registration linchpin: without a fw role the per-MACID frame-stat
   * engine (USR_TX_RPT), the data-frame path, and power-by-rate do not engage
   * for an injected MACID. `self_role` = mac_ax_self_role, `wifi_role` =
   * mac_ax_wifi_role, `upd_mode` = mac_ax_upd_mode (CREATE/REMOVE). */
  bool fw_role_maintain(uint8_t macid, uint8_t self_role, uint8_t wifi_role,
                        uint8_t upd_mode, uint8_t band, uint8_t port);

  /* mac_upd_addr_cam (addr_cam.c): H2C cat=MAC, class=ADDR_CAM_UPDATE,
   * func=ADDRCAM_INFO — programs the address-CAM entry (BSSID+SMA+TMA+macid+
   * net_type) for `macid`. `self_mac` is the 6-byte transmitter address (our
   * SA). `net_type` = mac_ax_net_type. `addr_cam_idx`/`bssid_cam_idx` index the
   * CAM entries (0 for the first/self STA). The frame's SA/BSSID resolve from
   * this entry; without it the CMAC cannot build the PPDU. */
  bool fw_upd_addr_cam(uint8_t macid, const uint8_t self_mac[6], uint8_t net_type,
                       uint8_t addr_cam_idx, uint8_t bssid_cam_idx);

  /* mac_upd_cctl_info (tblupd.c): H2C cat=MAC, class=FR_EXCHG, func=CCTLINFO_UD
   * — the per-MACID CMAC control table. Sets the default TX rate, the TX
   * antenna path enable (ntx_path_en — without it a frame has NO antenna and
   * never airs), the addr-cam index, and disables rate fallback. `bmc` marks a
   * broadcast/multicast MACID. Read-modify-write (operation=1). */
  bool fw_upd_cctl_basic(uint8_t macid, uint8_t addr_cam_idx, uint16_t datarate,
                         uint8_t ntx_path_en, uint8_t path_map_a, bool bmc);

  /* mac_send_bcn_h2c (beacon.c): H2C cat=MAC, class=FR_EXCHG, func=BCN_UPD_V1 —
   * hand the firmware the full 802.11 beacon body; the fw stores it and airs it
   * hardware-timed at each TBTT (the MAC inserts the live TSF into the beacon
   * timestamp). 9-dword header (port/band, macid/rate, ntx_path) + body. */
  bool fw_send_beacon(const uint8_t *body, uint32_t len, uint8_t macid,
                      uint16_t rate_ax, uint8_t ntx_path_en, uint8_t path_map_a);

  /* ---- 802.11ax trigger-based UL + TWT ----
   * The encoders below are implemented in KestrelFwSched.cpp (a second TU of
   * this class) to keep this file focused on FWDL/bring-up. They all funnel
   * through send_h2c_cmd, so the per-die descriptor + rolling seq are shared. */

  /* mac_f2p_test_cmd: H2C cat=MAC, class=FR_EXCHG, func=F2P_TEST — makes the fw
   * build and air one HE Trigger frame (UL-OFDMA grant) as a frame exchange.
   * The 8852B lays out the v0 (4-user, 236B) content, the 8852C the v1 (8-user,
   * 288B) — selected by _variant. Only the trigger-frame parameters are set;
   * the DL-burst / SIG-B dwords stay zero for a plain UL Basic Trigger. */
  bool f2p_trigger(const devourer::TriggerConfig &cfg);

  /* mac_twt_info_upd (class=TWT, func=TWTINFO_UPD): create/modify a TWT
   * agreement (wake window, interval, absolute target-wake TSF). */
  bool twt_info_upd(const devourer::TwtConfig &cfg, uint8_t act);

  /* mac_twt_act (class=TWT, func=TWT_STANSP_UPD): bind/unbind a STA macid to a
   * TWT config (add/del/terminate/suspend/resume). */
  bool twt_act(const devourer::TwtStaAct &act);

  /* mac_twt_staanno (class=TWT, func=TWT_ANNOUNCE_UPD): tell the fw to send a
   * TWT announce to a macid (AP-side). */
  bool twt_announce(uint8_t macid);

  /* mac_twt_ofdma_info_upd (class=TWT, func=0x03): the fw-autonomous trigger
   * cadence inside a TWT SP. func 0x03 is non-canonical (may be absent from the
   * shipped fw) — the caller treats a fw-ignore as "fall back to ul_fixinfo". */
  bool twt_ofdma_info_upd(const devourer::TwtOfdmaConfig &cfg);

  /* mac_upd_ul_fixinfo (class=FR_EXCHG, func=TBLUD, table CLASS_UL_FIXINFO):
   * the production UL-OFDMA scheduler table. mode=tf_periodic makes the fw air
   * Triggers autonomously at the configured interval. */
  bool ul_fixinfo(const devourer::UlOfdmaConfig &cfg);

  /* mac_set_snd_para (class=CL_SOUND, func=SET_SND_PARA v0 / _V1 0xD): the
   * *production* trigger-airing path. The fw builds and airs the NDPA -> NDP ->
   * BFRP sequence from this H2C content and arms RX for the beamforming-report
   * HE TB PPDU. A BFRP is an 802.11ax Trigger-frame variant, so this solicits a
   * genuine hardware-scheduled UL transmission — unlike f2p_trigger (the MP-only
   * F2P_TEST entry the shipped client fw has no handler for). 8852B lays out the
   * v0 (72-dword) content, the 8852C the v1 (98-dword), selected by _variant. */
  bool set_snd_para(const devourer::SoundingConfig &cfg);

  /* mac_set_csi_para_cctl (mac_upd_cctl_info with the CSI/bf dword): program the
   * per-macid CSI/beamformee params (nc/nr/ng/cb/cs + csi_para_en) so a BFRP to
   * this STA solicits a decodable compressed-beamforming report. csi_txbf_en is
   * forced 0 (vendor BB HW-bug note). */
  bool fw_upd_cctl_bf(uint8_t macid, uint8_t addr_cam_idx,
                      const devourer::StaBfCaps &bf);

private:
  /* Generic H2C over CH12: [WD 24B][fwcmd_hdr 8B][content]. */
  bool send_h2c_cmd(uint8_t cat, uint8_t h2c_class, uint8_t func,
                    const uint8_t *content, uint32_t len);

  /* --- register-op helpers (mirror HalKestrel; kept local to this TU) --- */
  void set32(uint16_t reg, uint32_t bits);
  void clr32(uint16_t reg, uint32_t bits);
  bool poll_wcpu(uint32_t mask, uint32_t expect, const char *what);

  /* --- pre-init --- */
  bool hci_func_en();
  bool dmac_pre_init();        /* dmac_func_pre_en + dle_init + hfc_init */
  bool dle_init_dlfw();
  bool hfc_init_dlfw();
  bool chk_dle_rdy(uint16_t status_reg, uint32_t rdy_bits, const char *what);

  /* --- FWDL state machine --- */
  bool disable_cpu();
  bool enable_cpu(uint8_t boot_reason);
  bool mac_fwdl(const uint8_t *fw, uint32_t len, uint8_t mss_idx);
  /* Non-secure-IC FWDL patches (fwdl.c): SEC_CTRL IDMEM-share to default, and
   * the fw CPU-clock write via the indirect-access IDMEM window. */
  void idmem_share_mode_check();
  void fwdl_patch_fw_delay();
  bool check_fw_rdy();
  /* Build [WD 24B] (+ [fwcmd_hdr 8B] when is_header) + payload and bulk-send
   * to the CH12 endpoint. seq is the H2C sequence counter (header only). */
  bool send_fwdl_packet(const uint8_t *payload, uint32_t payload_len,
                        bool is_header, uint8_t seq);

  RtlAdapter _device;
  Logger_t _logger;
  ChipVariant _variant;
  uint8_t _ch12_ep = 0; /* resolved bulk-OUT endpoint for CH12 (BULKOUTID2) */
  std::vector<uint8_t> _txbuf; /* reused H2C packet scratch */
  uint8_t _h2c_seq = 0; /* fwinfo->h2c_seq: 8-bit rolling, all runtime H2Cs */
  bool _is_sec_ic = false; /* OTP 0x5ED[7]: gates the non-secure FWDL patch */
};

} /* namespace kestrel */

#endif /* KESTREL_KESTREL_FW_H */
