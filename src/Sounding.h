#pragma once

/* Sounding — the 802.11ax HE sounding (NDPA -> NDP -> BFRP) session knobs.
 *
 * This is the *production* trigger-airing path on the RTL8852 client firmware:
 * the fw builds and airs the NDPA/NDP/BFRP frames from a single H2C
 * (CL_SOUND / SET_SND_PARA, content-in-H2C, no pkt_ofld) and arms RX for the
 * beamforming-report HE TB PPDU. A BFRP (Beamforming Report Poll) is an
 * 802.11ax Trigger-frame variant, so it solicits a genuine hardware-scheduled,
 * contention-free UL transmission (the STA's compressed beamforming report,
 * sent as an HE TB PPDU at trigger+SIFS).
 *
 * Unlike the F2P_TEST command (an MP/hardware-verification entry with no
 * shipped-fw handler), sounding is a normal client feature. These aggregates
 * are devourer-native; the Kestrel HAL maps them onto the vendor
 * mac_ax_fwcmd_snd via the SchedEncode.h encoder. The raw 72-/98-dword vendor
 * struct never surfaces here.
 *
 * The plane split mirrors the rest of the AX MAC surface: the AP (this driver)
 * owns the sounding schedule and the fw airs the frames; the STA MAC fires its
 * TB-PPDU report at trigger+SIFS in hardware. */

#include <array>
#include <cstdint>
#include <cstring>

namespace devourer {

/* One beamformee's STA Info in the HE NDPA (802.11ax NDP Announcement). */
struct SoundingStaInfo {
  uint16_t aid = 0;     /* 11-bit AID of the beamformee */
  uint16_t bw = 0;      /* Sounding bandwidth (0 = 20 MHz) */
  uint8_t fb_ng = 0;    /* Feedback Ng (subcarrier grouping: 0 = Ng4) */
  bool cb = false;      /* Codebook size */
  uint8_t nc = 0;       /* Nc-1 (number of columns of the feedback matrix) */
};

/* One beamformee's User Info in the HE BFRP Trigger (what TB PPDU to send). */
struct SoundingBfrpUser {
  uint16_t aid12 = 0;    /* 12-bit AID of the polled STA */
  uint8_t ru_pos = 0;    /* 8-bit RU Allocation subfield (61 = full 20 MHz) */
  bool ul_fec_code = false; /* false = BCC, true = LDPC */
  uint8_t ul_mcs = 0;    /* 0..11 */
  bool ul_dcm = false;
  uint8_t ss_alloc = 0;  /* SS Allocation subfield (0 = 1 SS starting at SS0) */
  uint8_t fbseg_rexmit_bmp = 0;
  uint8_t ul_tgt_rssi = 127; /* 7-bit target RSSI (127 = transmit at max power) */
};

/* One entry of the sounding WD list (the TX descriptor the fw uses for each
 * frame in the NDPA/NDP/BFRP sequence). The frexgtype-specific fields
 * (txpktsize / snd_pkt_sel / ndpa / sifs_tx) are set by make_he_bfrp_sounding
 * to match the vendor mac_set_snd_para AX_MU_BFRP1 case. */
struct SoundingWd {
  uint16_t txpktsize = 0;
  uint16_t ndpa_duration = 0;
  uint16_t datarate = 0;  /* AX 9-bit rate code the frame airs at (0 = OFDM6) */
  uint8_t macid = 0;
  uint8_t data_bw = 0;
  uint8_t gi_ltf = 0;
  bool stf_mode = false;
  bool disdatafb = false;
  bool data_txcnt_lmt_sel = false;
  uint8_t data_txcnt_lmt = 0;
  bool sifs_tx = false;
  uint8_t snd_pkt_sel = 0; /* 1 = NDPA, 3 = NDP, 5 = BFRP (fw frame selector) */
  uint8_t ndpa = 0;        /* 0 = none, 2 = VHT, 3 = HE */
};

/* HE BFRP Trigger common info (the Trigger frame Common Info field). */
struct SoundingBfrpCommon {
  uint8_t trigger_type = 0; /* 1 = BFRP */
  uint16_t ul_len = 0;      /* L-SIG length of the solicited TB PPDU */
  uint8_t ul_bw = 0;        /* 0 = 20 MHz */
  uint8_t gi_ltf = 0;
  uint8_t num_he_ltf = 0;
  uint8_t ap_tx_pwr = 0;
  uint16_t ul_sr = 0;
};

/* A complete HE sounding command (mirrors mac_ax_fwcmd_snd for the devourer
 * surface). Defaults are zero; use make_he_bfrp_sounding() to build a valid
 * single-user HE BFRP sounding. */
struct SoundingConfig {
  uint8_t frexgtype = 36; /* FRAME_EXCHANGE_SND_AX_MU_BFRP1 */
  uint8_t mode = 0;
  uint8_t bfrp0_user_num = 0;
  uint8_t bfrp1_user_num = 0;
  std::array<uint8_t, 16> macid{}; /* beamformee macids (macid[0] = STA) */

  /* NDPA frame + STA-info list. */
  uint16_t ndpa_frame_ctl = 0x0054;
  uint16_t ndpa_duration = 0;
  std::array<uint8_t, 6> ndpa_addr1{}; /* RA (the beamformee / broadcast) */
  std::array<uint8_t, 6> ndpa_addr2{}; /* TA (AP SA) */
  std::array<uint8_t, 6> ndpa_addr3{}; /* BSSID */
  uint16_t ndpa_seq_control = 0;
  bool ndpa_dialog_he = true;
  uint8_t ndpa_dialog_token = 0;
  std::array<SoundingStaInfo, 8> he_sta{}; /* HE NDPA STA Info (index 0..7) */

  /* BFRP frame headers (up to 3: BFRP0/1/2) + per-BFRP common+user. */
  std::array<uint16_t, 3> bfrp_frame_ctl{{0x0024, 0x0024, 0x0024}};
  std::array<uint16_t, 3> bfrp_duration{};
  std::array<std::array<uint8_t, 6>, 3> bfrp_addr1{}; /* RA */
  std::array<std::array<uint8_t, 6>, 3> bfrp_addr2{}; /* TA (AP SA) */
  std::array<SoundingBfrpCommon, 2> bfrp_common{};
  std::array<std::array<SoundingBfrpUser, 8>, 2> bfrp_user{};

  /* WD list (5 entries) + F2P(BFRP) + SFP. */
  std::array<SoundingWd, 5> wd{};
  std::array<uint16_t, 2> csi_len_bfrp{};    /* f2p[0/1].csi_len_bfrp */
  std::array<uint8_t, 2> tb_t_pe_bfrp{};
  uint8_t f2p_type = 0;   /* sfp.f2p_type (0 = add/one-shot) */
  uint8_t f2p_index = 0;
  uint16_t f2p_period = 0; /* 0 = one-shot; >0 = periodic sounding (units of TU) */
  bool sounding_en = false; /* sfp.sounding_en (periodic-sounding enable) */
};

/* Per-STA beamformee capability the AP programs into the CCTL bf fields so a
 * BFRP to this STA solicits a decodable report. Values are the STA's advertised
 * HE beamformee params (Nc, Ng, codebook) + the CSI report rate/BW. */
struct StaBfCaps {
  uint8_t nc = 0;   /* Nc-1 */
  uint8_t nr = 0;   /* Nr-1 (AP receive chains - 1) */
  uint8_t ng = 0;   /* subcarrier grouping (0 = Ng4) */
  uint8_t cb = 1;   /* codebook */
  uint8_t cs = 0;   /* codebook subset */
  uint8_t csi_bw = 0;      /* 0 = 20 MHz */
  uint8_t csi_gi_ltf = 0;  /* CSI report GI/LTF */
  uint16_t csi_fix_rate = 0; /* 0 = auto (from RRSC) */
};

/* Build a single-user HE BFRP sounding: the AP (ap_mac / bssid) sounds one
 * associated beamformee (sta_mac, aid, fw macid). Fills the NDPA + BFRP frame
 * content and the WD list per the vendor mac_set_snd_para AX_MU_BFRP1 case
 * (wd[0]=NDPA, wd[1]=NDP, wd[2]=BFRP). `dialog` is the rolling sounding dialog
 * token; `csi_bw` selects the sounding bandwidth (0 = 20 MHz). `air_rate` is
 * the AX rate code the NDPA/NDP/BFRP air at (0 = OFDM6, non-HT so any monitor
 * decodes the trigger). */
/* `period` selects the fw sounding cadence: 0 = a one-shot sounding (SNDF2P_ONCE,
 * the caller re-issues each time — beware flooding the CH12 H2C queue if fast);
 * >0 = fw-autonomous periodic sounding (SNDF2P_ADD + sounding_en), so ONE H2C
 * arms repeated soundings at the fw's own cadence (units are the vendor
 * f2p_period field) — the analogue of UL_FIXINFO tf_periodic and the way to
 * sustain sounding without per-sounding H2Cs. */
inline SoundingConfig make_he_bfrp_sounding(const uint8_t ap_mac[6],
                                            const uint8_t bssid[6],
                                            const uint8_t sta_mac[6],
                                            uint16_t aid, uint8_t macid,
                                            uint8_t dialog,
                                            uint16_t period = 0,
                                            uint8_t ap_macid = 0,
                                            uint16_t air_rate = 0,
                                            uint8_t ul_mcs = 0) {
  SoundingConfig c;
  c.frexgtype = 36; /* AX_MU_BFRP1 */
  c.bfrp0_user_num = 1;
  c.macid[0] = macid;

  /* NDPA (type1/subtype5, FC 0x0054): RA = STA, TA = AP, BSSID. */
  c.ndpa_frame_ctl = 0x0054;
  std::memcpy(c.ndpa_addr1.data(), sta_mac, 6);
  std::memcpy(c.ndpa_addr2.data(), ap_mac, 6);
  std::memcpy(c.ndpa_addr3.data(), bssid, 6);
  c.ndpa_dialog_he = true;
  c.ndpa_dialog_token = dialog & 0x3f;
  c.he_sta[0].aid = aid & 0x7ff;
  c.he_sta[0].bw = 0;
  c.he_sta[0].fb_ng = 0;
  c.he_sta[0].cb = false;
  c.he_sta[0].nc = 0; /* Nc-1 = 0 -> 1 column */

  /* BFRP0 Trigger (type1/subtype2, FC 0x0024): RA = STA, TA = AP. */
  c.bfrp_frame_ctl[0] = 0x0024;
  std::memcpy(c.bfrp_addr1[0].data(), sta_mac, 6);
  std::memcpy(c.bfrp_addr2[0].data(), ap_mac, 6);
  c.bfrp_common[0].trigger_type = 1; /* BFRP */
  c.bfrp_common[0].ul_len = 0x1ff;   /* placeholder L-SIG len (tuned on-air) */
  c.bfrp_common[0].ul_bw = 0;
  c.bfrp_common[0].gi_ltf = 0;
  c.bfrp_common[0].num_he_ltf = 0;
  c.bfrp_common[0].ap_tx_pwr = 0x3f;
  c.bfrp_user[0][0].aid12 = aid & 0xfff;
  c.bfrp_user[0][0].ru_pos = 61; /* full 20 MHz (242-tone) */
  c.bfrp_user[0][0].ul_mcs = ul_mcs & 0xf;
  c.bfrp_user[0][0].ss_alloc = 0; /* 1 SS */
  c.bfrp_user[0][0].ul_tgt_rssi = 127; /* max power */
  c.csi_len_bfrp[0] = 0x1ff;

  /* WD list per mac_set_snd_para AX_MU_BFRP1 (users = 1):
   * wd[0]=NDPA, wd[1]=NDP, wd[2]=BFRP. */
  const uint8_t users = 1;
  auto &w0 = c.wd[0];
  w0.txpktsize = static_cast<uint16_t>(17 + 4 * users);
  w0.stf_mode = true; w0.disdatafb = true;
  w0.data_txcnt_lmt_sel = true; w0.data_txcnt_lmt = 1;
  w0.sifs_tx = true; w0.snd_pkt_sel = 1; w0.ndpa = 3;
  w0.datarate = air_rate; w0.macid = ap_macid;
  auto &w1 = c.wd[1];
  w1.txpktsize = 0;
  w1.stf_mode = true; w1.disdatafb = true;
  w1.data_txcnt_lmt_sel = true; w1.data_txcnt_lmt = 1;
  w1.sifs_tx = true; w1.snd_pkt_sel = 3; w1.ndpa = 3;
  w1.datarate = air_rate; w1.macid = ap_macid;
  auto &w2 = c.wd[2];
  w2.txpktsize = static_cast<uint16_t>(24 + 6 * users);
  w2.stf_mode = true; w2.disdatafb = true;
  w2.data_txcnt_lmt_sel = true; w2.data_txcnt_lmt = 1;
  w2.sifs_tx = false; w2.snd_pkt_sel = 5; w2.ndpa = 3;
  w2.datarate = air_rate; w2.macid = ap_macid;

  if (period > 0) {
    c.f2p_type = 1;          /* SNDF2P_ADD — fw-autonomous periodic sounding */
    c.f2p_period = period;
    c.sounding_en = true;
  } else {
    c.f2p_type = 0;          /* SNDF2P_ONCE — one-shot (caller re-issues) */
    c.f2p_period = 0;
    c.sounding_en = false;
  }
  return c;
}

} /* namespace devourer */
