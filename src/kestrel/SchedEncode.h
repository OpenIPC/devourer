#ifndef KESTREL_SCHED_ENCODE_H
#define KESTREL_SCHED_ENCODE_H

#include <cstdint>
#include <vector>

#include "KestrelLe.h"
#include "MacRegAx.h"
#include "Sounding.h"
#include "TriggerTwt.h"

/* Pure LE-byte encoders for the 802.11ax trigger-UL + TWT H2C content buffers.
 * Split out of the KestrelFw methods so the byte layout is unit-testable
 * without a device (tests/kestrel_sched_selftest.cpp) — the KestrelFw methods
 * are just encode_* + send_h2c_cmd + a log line. Every function mirrors a
 * vendor mac_ax command byte-for-byte; field layouts are the MacRegAx.h
 * constants (verbatim from the vendor headers). */

namespace kestrel::sched {

namespace r = kestrel::reg;

/* SET_WORD(v, FIELD) == (v & FIELD_MSK) << FIELD_SH. */
inline uint32_t sw(uint32_t v, uint32_t msk, uint8_t sh) {
  return (v & msk) << sh;
}

/* 802.11ax SS Allocation subfield: bits[2:0]=start SS(0), bits[5:3]=NSS-1. */
inline uint32_t ss_alloc_enc(uint8_t ss) {
  uint32_t nss = ss == 0 ? 0 : static_cast<uint32_t>(ss - 1);
  return (nss & 0x7) << 3;
}

/* F2P trigger content (mac_f2p_test_cmd): v1=true lays out the 8852C 8-user
 * 288-byte fwcmd_test_para_v1; v1=false the 8852B 4-user 236-byte
 * fwcmd_test_para. Only the trigger-frame parameters are set; the DL-burst /
 * SIG-B dwords stay zero (a plain UL Basic Trigger). */
inline std::vector<uint8_t> encode_f2p_trigger(const devourer::TriggerConfig &cfg,
                                              bool v1) {
  const size_t content_len = v1 ? r::F2P_V1_CONTENT_LEN : r::F2P_V0_CONTENT_LEN;
  const uint8_t max_users = v1 ? r::F2P_V1_MAX_USERS : r::F2P_V0_MAX_USERS;
  const size_t tfwd_off = v1 ? r::F2P_V1_TFWD_OFF : r::F2P_V0_TFWD_OFF;
  const size_t depuser_off = v1 ? r::F2P_V1_DEPUSER_OFF : r::F2P_V0_DEPUSER_OFF;
  uint8_t n = cfg.n_users == 0 ? 1 : cfg.n_users;
  if (n > max_users)
    n = max_users;

  std::vector<uint8_t> c(content_len, 0);
  uint8_t *b = c.data();

  /* dword0 — trigger-frame packet common. pktnum=0 (no paired DL data). */
  put_le32(b + 0,
           sw(cfg.ul_bw, r::FWCMD_F2PTEST_ULBW_MSK, r::FWCMD_F2PTEST_ULBW_SH) |
               sw(cfg.gi_ltf, r::FWCMD_F2PTEST_GILTF_MSK,
                  r::FWCMD_F2PTEST_GILTF_SH) |
               sw(cfg.num_he_ltf, r::FWCMD_F2PTEST_NUMLTF_MSK,
                  r::FWCMD_F2PTEST_NUMLTF_SH) |
               sw(cfg.ap_tx_power, r::FWCMD_F2PTEST_TXPWR_MSK,
                  r::FWCMD_F2PTEST_TXPWR_SH) |
               sw(n, r::FWCMD_F2PTEST_USERNUM_MSK, r::FWCMD_F2PTEST_USERNUM_SH) |
               sw(cfg.pri20_bitmap, r::FWCMD_F2PTEST_BITMAP_MSK,
                  r::FWCMD_F2PTEST_BITMAP_SH));

  /* Per-user pair: dwordA at (1+2u), dwordB at (2+2u) — same in v0 and v1. */
  for (uint8_t u = 0; u < n; ++u) {
    const devourer::TrigUser &tu = cfg.users[u];
    const size_t a = static_cast<size_t>(1 + 2 * u) * 4;
    const size_t bb = static_cast<size_t>(2 + 2 * u) * 4;
    put_le32(b + a,
             sw(tu.aid12, r::FWCMD_F2PTEST_AID12_MSK, r::FWCMD_F2PTEST_AID12_SH) |
                 sw(tu.ul_mcs, r::FWCMD_F2PTEST_ULMCS_MSK,
                    r::FWCMD_F2PTEST_ULMCS_SH) |
                 sw(tu.macid, r::FWCMD_F2PTEST_MACID_MSK,
                    r::FWCMD_F2PTEST_MACID_SH) |
                 sw(tu.ru_alloc, r::FWCMD_F2PTEST_RUPOS_MSK,
                    r::FWCMD_F2PTEST_RUPOS_SH));
    put_le32(b + bb,
             sw(tu.ldpc ? 1 : 0, r::FWCMD_F2PTEST_ULFEC_MSK,
                r::FWCMD_F2PTEST_ULFEC_SH) |
                 sw(tu.dcm ? 1 : 0, r::FWCMD_F2PTEST_ULDCM_MSK,
                    r::FWCMD_F2PTEST_ULDCM_SH) |
                 sw(ss_alloc_enc(tu.ss), r::FWCMD_F2PTEST_SS_ALLOC_MSK,
                    r::FWCMD_F2PTEST_SS_ALLOC_SH) |
                 sw(devourer::he_tgt_rssi_enc(tu.tgt_rssi_dbm),
                    r::FWCMD_F2PTEST_UL_TGTRSSI_MSK,
                    r::FWCMD_F2PTEST_UL_TGTRSSI_SH));
    /* dep-user pref_AC — one byte per user (v0: byte9..; v1: dword17_0..). */
    b[depuser_off + u] = tu.pref_ac & 0x3;
  }

  /* tf_wd dword — the trigger frame's own legacy-PPDU rate + mode/frexch. */
  put_le32(b + tfwd_off,
           sw(cfg.trig_rate, r::FWCMD_F2PTEST_DATARATE_MSK,
              r::FWCMD_F2PTEST_DATARATE_SH) |
               sw(cfg.mode, r::FWCMD_F2PTEST_MODE_MSK, r::FWCMD_F2PTEST_MODE_SH) |
               sw(cfg.frexch_type, r::FWCMD_F2PTEST_TYPE_MSK,
                  r::FWCMD_F2PTEST_TYPE_SH));

  /* f2p_wd (dword14 v0 / dword20 v1) — the TX descriptor for the frame the fw
   * builds. Without it (length 0, fs/ls clear) the fw airs nothing. One
   * complete Basic Trigger on the band-0 mgmt queue: fs=ls=1, one segment,
   * length = the built frame's dwords (16 B MAC hdr + 8 B Common Info + 5 B per
   * User Info + 4 B FCS, rounded up). */
  const uint32_t frame_bytes = 16u + 8u + 5u * n + 4u;
  const uint32_t frame_dwords = (frame_bytes + 3u) / 4u;
  put_le32(b + tfwd_off + 4,
           sw(r::MAC_AX_MG0_SEL, r::F2P_WD_CMD_QSEL_MSK, r::F2P_WD_CMD_QSEL_SH) |
               r::F2P_WD_FS | r::F2P_WD_LS |
               sw(1, r::F2P_WD_TOTAL_NUMBER_MSK, r::F2P_WD_TOTAL_NUMBER_SH) |
               sw(frame_dwords, r::F2P_WD_LENGTH_MSK, r::F2P_WD_LENGTH_SH));
  return c;
}

/* TWT info (mac_twt_info_upd) — 6-dword fwcmd_twtinfo_upd (vendor sets 0..4). */
inline std::vector<uint8_t> encode_twtinfo(const devourer::TwtConfig &cfg,
                                          uint8_t act) {
  std::vector<uint8_t> c(24, 0);
  uint8_t *b = c.data();
  const uint32_t nego = cfg.broadcast ? 2u : 0u;
  put_le32(b + 0,
           sw(nego, r::FWCMD_H2C_TWTINFO_UPD_NEGOTYPE_MSK,
              r::FWCMD_H2C_TWTINFO_UPD_NEGOTYPE_SH) |
               (cfg.trigger ? r::FWCMD_H2C_TWTINFO_UPD_TRIGGER : 0) |
               (cfg.announced ? r::FWCMD_H2C_TWTINFO_UPD_FLOWTYPE : 0) |
               (cfg.wake_dur_us256 ? r::FWCMD_H2C_TWTINFO_UPD_WAKEDURUNIT : 0) |
               sw(cfg.flow_id, r::FWCMD_H2C_TWTINFO_UPD_FLOWID_MSK,
                  r::FWCMD_H2C_TWTINFO_UPD_FLOWID_SH) |
               (cfg.protection ? r::FWCMD_H2C_TWTINFO_UPD_PROT : 0) |
               sw(act, r::FWCMD_H2C_TWTINFO_UPD_ACT_MSK,
                  r::FWCMD_H2C_TWTINFO_UPD_ACT_SH) |
               sw(cfg.config_id, r::FWCMD_H2C_TWTINFO_UPD_ID_MSK,
                  r::FWCMD_H2C_TWTINFO_UPD_ID_SH) |
               (cfg.band ? r::FWCMD_H2C_TWTINFO_UPD_BAND : 0) |
               sw(cfg.port, r::FWCMD_H2C_TWTINFO_UPD_PORT_MSK,
                  r::FWCMD_H2C_TWTINFO_UPD_PORT_SH) |
               (cfg.ap_role ? r::FWCMD_H2C_TWTINFO_UPD_NIC_ROLE : 0));
  put_le32(b + 4,
           sw(cfg.wake_man, r::FWCMD_H2C_TWTINFO_UPD_WAKE_MAN_MSK,
              r::FWCMD_H2C_TWTINFO_UPD_WAKE_MAN_SH) |
               sw(cfg.wake_exp, r::FWCMD_H2C_TWTINFO_UPD_WAKE_EXP_MSK,
                  r::FWCMD_H2C_TWTINFO_UPD_WAKE_EXP_SH) |
               sw(cfg.min_wake_dur, r::FWCMD_H2C_TWTINFO_UPD_DUR_MSK,
                  r::FWCMD_H2C_TWTINFO_UPD_DUR_SH));
  put_le32(b + 8, static_cast<uint32_t>(cfg.trgt_tsf & 0xffffffffu));
  put_le32(b + 12, static_cast<uint32_t>(cfg.trgt_tsf >> 32));
  /* dword4 (ptt/dma/early) + dword5 stay zero. */
  return c;
}

/* TWT STA act (mac_twt_act) — 2-dword fwcmd_twt_stansp_upd. */
inline std::vector<uint8_t> encode_twt_act(const devourer::TwtStaAct &a) {
  std::vector<uint8_t> c(8, 0);
  put_le32(c.data() + 0,
           sw(a.macid, r::FWCMD_H2C_TWT_STANSP_UPD_MACID_MSK,
              r::FWCMD_H2C_TWT_STANSP_UPD_MACID_SH) |
               sw(a.config_id, r::FWCMD_H2C_TWT_STANSP_UPD_ID_MSK,
                  r::FWCMD_H2C_TWT_STANSP_UPD_ID_SH) |
               sw(a.action, r::FWCMD_H2C_TWT_STANSP_UPD_ACT_MSK,
                  r::FWCMD_H2C_TWT_STANSP_UPD_ACT_SH));
  return c;
}

/* TWT announce (mac_twt_staanno) — 1-dword fwcmd_twt_announce_upd. */
inline std::vector<uint8_t> encode_twt_announce(uint8_t macid) {
  std::vector<uint8_t> c(4, 0);
  put_le32(c.data() + 0, sw(macid, r::FWCMD_H2C_TWT_ANNOUNCE_UPD_MACID_MSK,
                            r::FWCMD_H2C_TWT_ANNOUNCE_UPD_MACID_SH));
  return c;
}

/* TWT-OFDMA cadence (mac_twt_ofdma_info_upd) — 2-dword fwcmd_twt_ofdma_info_upd. */
inline std::vector<uint8_t> encode_twt_ofdma(const devourer::TwtOfdmaConfig &cfg) {
  std::vector<uint8_t> c(8, 0);
  put_le32(c.data() + 0,
           sw(cfg.option, r::FWCMD_H2C_TWT_OFDMA_OPTION_MSK,
              r::FWCMD_H2C_TWT_OFDMA_OPTION_SH) |
               sw(cfg.twt_id, r::FWCMD_H2C_TWT_OFDMA_TWT_ID_MSK,
                  r::FWCMD_H2C_TWT_OFDMA_TWT_ID_SH) |
               sw(cfg.max_tf_retry, r::FWCMD_H2C_TWT_OFDMA_MAX_TF_RETRY_MSK,
                  r::FWCMD_H2C_TWT_OFDMA_MAX_TF_RETRY_SH) |
               sw(cfg.max_dl_retry, r::FWCMD_H2C_TWT_OFDMA_MAX_DL_RETRY_MSK,
                  r::FWCMD_H2C_TWT_OFDMA_MAX_DL_RETRY_SH) |
               sw(cfg.round_num, r::FWCMD_H2C_TWT_OFDMA_ROUND_NUM_MSK,
                  r::FWCMD_H2C_TWT_OFDMA_ROUND_NUM_SH) |
               sw(cfg.preferred_ac, r::FWCMD_H2C_TWT_OFDMA_PREFERRED_AC_MSK,
                  r::FWCMD_H2C_TWT_OFDMA_PREFERRED_AC_SH) |
               (cfg.htc_bsr_ctrl_en ? r::FWCMD_H2C_TWT_OFDMA_HTC_BSR_CTRL_EN : 0));
  put_le32(c.data() + 4, sw(cfg.round_interval_us,
                            r::FWCMD_H2C_TWT_OFDMA_ROUND_INTERVAL_MSK,
                            r::FWCMD_H2C_TWT_OFDMA_ROUND_INTERVAL_SH));
  return c;
}

/* UL_FIXINFO table (mac_upd_ul_fixinfo) — 27-dword content for 8 RU slots. */
inline std::vector<uint8_t> encode_ul_fixinfo(const devourer::UlOfdmaConfig &cfg) {
  constexpr uint8_t RU = r::UL_FIXINFO_MAX_RU_NUM; /* 8 */
  const size_t ndw = 5 + RU / 2 + 2 + RU * 2;
  std::vector<uint8_t> c(ndw * 4, 0);
  uint8_t *b = c.data();
  size_t o = 0;
  auto push = [&](uint32_t v) {
    put_le32(b + o, v);
    o += 4;
  };
  uint8_t nsta = cfg.n_stas == 0 ? 1 : cfg.n_stas;
  if (nsta > RU)
    nsta = RU;

  /* dword0 — tbl_hdr: write, group 0, offset 0, length = ndw, table class. */
  push(r::FWCMD_H2C_TBLUD_R_W |
       sw(static_cast<uint32_t>(ndw), r::FWCMD_H2C_TBLUD_LENGTH_MSK,
          r::FWCMD_H2C_TBLUD_LENGTH_SH) |
       sw(r::CLASS_UL_FIXINFO, r::FWCMD_H2C_TBLUD_TABLE_CLASS_MSK,
          r::FWCMD_H2C_TBLUD_TABLE_CLASS_SH));
  /* dword1 — cfg: mode / interval / bsr_thold / storemode. */
  push(sw(cfg.mode, r::FWCMD_H2C_UL_FIXINFO_CFG_MODE_MSK,
          r::FWCMD_H2C_UL_FIXINFO_CFG_MODE_SH) |
       sw(cfg.interval_s, r::FWCMD_H2C_UL_FIXINFO_CFG_INTERVAL_MSK,
          r::FWCMD_H2C_UL_FIXINFO_CFG_INTERVAL_SH) |
       sw(cfg.bsr_thold, r::FWCMD_H2C_UL_FIXINFO_CFG_BSR_THOLD_MSK,
          r::FWCMD_H2C_UL_FIXINFO_CFG_BSR_THOLD_SH) |
       sw(cfg.store_mode, r::FWCMD_H2C_UL_FIXINFO_CFG_STOREMODE_MSK,
          r::FWCMD_H2C_UL_FIXINFO_CFG_STOREMODE_SH));
  /* dword2 — ulinfo A: tf_type / gi_ltf. */
  push(sw(cfg.tf_type, r::FWCMD_H2C_UL_FIXINFO_ULINFO_TF_TYPE_MSK,
          r::FWCMD_H2C_UL_FIXINFO_ULINFO_TF_TYPE_SH) |
       sw(cfg.gi_ltf, r::FWCMD_H2C_UL_FIXINFO_ULINFO_GI_LTF_MSK,
          r::FWCMD_H2C_UL_FIXINFO_ULINFO_GI_LTF_SH));
  /* dword3 — ulinfo B: data_rate / apep_len / istwt. */
  push(sw(cfg.data_rate, r::FWCMD_H2C_UL_FIXINFO_ULINFO_DATART_MSK,
          r::FWCMD_H2C_UL_FIXINFO_ULINFO_DATART_SH) |
       sw(cfg.apep_len, r::FWCMD_H2C_UL_FIXINFO_ULINFO_APEPLEN_MSK,
          r::FWCMD_H2C_UL_FIXINFO_ULINFO_APEPLEN_SH) |
       (cfg.istwt ? r::FWCMD_H2C_UL_FIXINFO_ULINFO_ISTWT : 0));
  /* dword4 — ulinfo C: multiport_id. */
  push(sw(cfg.multiport_id, r::FWCMD_H2C_UL_FIXINFO_ULINFO_MULTIPORT_MSK,
          r::FWCMD_H2C_UL_FIXINFO_ULINFO_MULTIPORT_SH));

  /* sta_info pairs — macid/pref_AC, two STAs per dword, RU/2 dwords. */
  for (uint8_t i = 0; i < RU; i += 2) {
    const devourer::UlOfdmaSta &s0 = cfg.stas[i];
    const devourer::UlOfdmaSta &s1 = cfg.stas[i + 1];
    push(sw(s0.macid, r::FWCMD_H2C_UL_FIXINFO_STA_INFO_MACID_0_MSK,
            r::FWCMD_H2C_UL_FIXINFO_STA_INFO_MACID_0_SH) |
         sw(s0.pref_ac, r::FWCMD_H2C_UL_FIXINFO_STA_INFO_PREF_AC_0_MSK,
            r::FWCMD_H2C_UL_FIXINFO_STA_INFO_PREF_AC_0_SH) |
         sw(s1.macid, r::FWCMD_H2C_UL_FIXINFO_STA_INFO_MACID_1_MSK,
            r::FWCMD_H2C_UL_FIXINFO_STA_INFO_MACID_1_SH) |
         sw(s1.pref_ac, r::FWCMD_H2C_UL_FIXINFO_STA_INFO_PREF_AC_1_MSK,
            r::FWCMD_H2C_UL_FIXINFO_STA_INFO_PREF_AC_1_SH));
  }

  /* ulrua header A — ppdu_bw / gi_ltf / sta_num. */
  push(sw(cfg.ppdu_bw, r::FWCMD_H2C_UL_FIXINFO_ULRUA_PPDU_BW_MSK,
          r::FWCMD_H2C_UL_FIXINFO_ULRUA_PPDU_BW_SH) |
       sw(cfg.gi_ltf, r::FWCMD_H2C_UL_FIXINFO_ULRUA_GI_LTF_MSK,
          r::FWCMD_H2C_UL_FIXINFO_ULRUA_GI_LTF_SH) |
       sw(nsta, r::FWCMD_H2C_UL_FIXINFO_ULRUA_STANUM_MSK,
          r::FWCMD_H2C_UL_FIXINFO_ULRUA_STANUM_SH));
  /* ulrua header B — grp_mode/grp_id/fix_mode stay 0 (explicit per-RU list). */
  push(0);

  /* Per-RU entries — 2 dwords each, RU_NUM slots (unused slots stay zero). */
  for (uint8_t i = 0; i < RU; ++i) {
    const devourer::UlOfdmaSta &s = cfg.stas[i];
    const bool used = i < nsta;
    uint32_t a = 0, d = 0;
    if (used) {
      a = sw(devourer::he_tgt_rssi_enc(s.tgt_rssi_dbm),
             r::FWCMD_H2C_UL_FIXINFO_UL_RUA_STA_ENT_TGT_RSSI_MSK,
             r::FWCMD_H2C_UL_FIXINFO_UL_RUA_STA_ENT_TGT_RSSI_SH) |
          sw(s.macid, r::FWCMD_H2C_UL_FIXINFO_UL_RUA_STA_ENT_MAC_ID_MSK,
             r::FWCMD_H2C_UL_FIXINFO_UL_RUA_STA_ENT_MAC_ID_SH) |
          sw(s.ru_pos, r::FWCMD_H2C_UL_FIXINFO_UL_RUA_STA_ENT_RU_POS_MSK,
             r::FWCMD_H2C_UL_FIXINFO_UL_RUA_STA_ENT_RU_POS_SH) |
          (s.ldpc ? r::FWCMD_H2C_UL_FIXINFO_UL_RUA_STA_ENT_CODE : 0);
      d = sw(s.bsr_length, r::FWCMD_H2C_UL_FIXINFO_UL_RUA_STA_ENT_BSRLEN_MSK,
             r::FWCMD_H2C_UL_FIXINFO_UL_RUA_STA_ENT_BSRLEN_SH) |
          (s.dcm ? r::FWCMD_H2C_UL_FIXINFO_UL_RUA_STA_ENT_DCM : 0) |
          sw(s.ss == 0 ? 0 : (s.ss - 1u),
             r::FWCMD_H2C_UL_FIXINFO_UL_RUA_STA_ENT_SS_MSK,
             r::FWCMD_H2C_UL_FIXINFO_UL_RUA_STA_ENT_SS_SH) |
          sw(s.mcs, r::FWCMD_H2C_UL_FIXINFO_UL_RUA_STA_ENT_MCS_MSK,
             r::FWCMD_H2C_UL_FIXINFO_UL_RUA_STA_ENT_MCS_SH);
    }
    push(a);
    push(d);
  }
  return c;
}

/* HE sounding content (mac_set_snd_para / build_snd_h2c): the 72-dword v0
 * (8852B) or 98-dword v1 (8852C) fwcmd_set_snd_para. The two layouts differ
 * only in section base offsets (v1 spreads macid over 4 dwords, carries 16 HE
 * STA-info + 8 users/BFRP vs v0's 8 + 4) and the dword0 user-num field width;
 * the per-field bit packing is identical, so one code path drives both via the
 * `L` offset table. Airs NDPA -> NDP -> BFRP for frexgtype AX_MU_BFRP1. */
inline std::vector<uint8_t> encode_set_snd_para(const devourer::SoundingConfig &cfg,
                                               bool v1) {
  const size_t ndw = v1 ? 98 : 72;
  std::vector<uint8_t> c(ndw * 4, 0);
  uint8_t *b = c.data();
  auto put = [&](size_t dwi, uint32_t v) { put_le32(b + dwi * 4, v); };
  auto pk4 = [](const uint8_t *p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
  };
  auto pk2 = [](const uint8_t *p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8);
  };

  /* Section base dwords (v0 / v1). */
  const size_t B_NDPA = v1 ? 5 : 3;      /* frame_ctl+dur, then 6 more dwords */
  const size_t B_VHT = v1 ? 12 : 10;     /* 2 dwords */
  const size_t B_HESTA = v1 ? 14 : 12;   /* v1:16 / v0:8 dwords (1 per STA) */
  const size_t B_BFRP_HDR = v1 ? 30 : 20;/* 3 hdrs x 4 dwords */
  const size_t B_BFRP0 = v1 ? 42 : 32;   /* common(2) + users(2 each) */
  const uint8_t USERS_PER = v1 ? 8 : 4;
  const size_t B_BFRP1 = B_BFRP0 + 2 + 2 * USERS_PER;
  const size_t B_VHTBFRP = B_BFRP1 + 2 + 2 * USERS_PER;
  const size_t B_WD = B_VHTBFRP + 1;     /* 5 WDs x 3 dwords */
  const size_t B_F2P = B_WD + 15;        /* 2 dwords */
  const size_t B_SFP = B_F2P + 2;        /* 2 dwords */

  /* dword0. */
  uint32_t d0 = sw(cfg.frexgtype, r::SND_FREXCH_TYPE_MSK, r::SND_FREXCH_TYPE_SH) |
                sw(cfg.mode, r::SND_MODE_MSK, r::SND_MODE_SH);
  if (v1)
    d0 |= sw(cfg.bfrp0_user_num, 0xff, 8) | sw(cfg.bfrp1_user_num, 0xff, 16);
  else
    d0 |= sw(cfg.bfrp0_user_num, r::SND_BFRP0_USER_NUM_MSK,
             r::SND_BFRP0_USER_NUM_SH) |
          sw(cfg.bfrp1_user_num, r::SND_BFRP1_USER_NUM_MSK,
             r::SND_BFRP1_USER_NUM_SH);
  put(0, d0);

  /* macid[] (v0: dwords 1,2 hold macid[0..3],[8..11]; v1: dwords 1..4 all 16). */
  if (v1) {
    put(1, pk4(&cfg.macid[0]));
    put(2, pk4(&cfg.macid[4]));
    put(3, pk4(&cfg.macid[8]));
    put(4, pk4(&cfg.macid[12]));
  } else {
    put(1, pk4(&cfg.macid[0]));
    put(2, pk4(&cfg.macid[8]));
  }

  /* NDPA: fc+dur, addr1/addr2 (byte-packed), dialog, addr3+seq. */
  put(B_NDPA, sw(cfg.ndpa_frame_ctl, r::SND_NDPA_FRAME_CTRL_MSK,
                 r::SND_NDPA_FRAME_CTRL_SH) |
                  sw(cfg.ndpa_duration, r::SND_NDPA_DURATION_MSK,
                     r::SND_NDPA_DURATION_SH));
  put(B_NDPA + 1, pk4(&cfg.ndpa_addr1[0]));
  put(B_NDPA + 2, pk2(&cfg.ndpa_addr1[4]) | (pk2(&cfg.ndpa_addr2[0]) << 16));
  put(B_NDPA + 3, pk4(&cfg.ndpa_addr2[2]));
  put(B_NDPA + 4, (cfg.ndpa_dialog_he ? r::SND_NDPA_SND_DLG_HE : 0) |
                      sw(cfg.ndpa_dialog_token, r::SND_NDPA_SND_DLG_DIALOG_MSK,
                         r::SND_NDPA_SND_DLG_DIALOG_SH));
  put(B_NDPA + 5, pk4(&cfg.ndpa_addr3[0]));
  put(B_NDPA + 6, pk2(&cfg.ndpa_addr3[4]) |
                      sw(cfg.ndpa_seq_control, r::SND_HT_SEQ_CONTROL_MSK,
                         r::SND_HT_SEQ_CONTROL_SH));

  /* NDPA HE STA-info (one per dword). VHT sta-info left zero (HE path). */
  const uint8_t nhe = v1 ? 16 : 8;
  for (uint8_t i = 0; i < 8 && i < nhe; ++i) {
    const devourer::SoundingStaInfo &s = cfg.he_sta[i];
    put(B_HESTA + i,
        sw(s.aid, r::SND_HE_STA_AID11_MSK, r::SND_HE_STA_AID11_SH) |
            sw(s.bw, r::SND_HE_STA_BW_MSK, r::SND_HE_STA_BW_SH) |
            sw(s.fb_ng, r::SND_HE_STA_FB_NG_MSK, r::SND_HE_STA_FB_NG_SH) |
            r::SND_HE_STA_DISAMBIGUATION | /* vendor forces disambiguation=1 */
            (s.cb ? r::SND_HE_STA_CB : 0) |
            sw(s.nc, r::SND_HE_STA_NC_MSK, r::SND_HE_STA_NC_SH));
  }

  /* BFRP frame headers (3): fc+dur, addr1/addr2 byte-packed. */
  for (uint8_t h = 0; h < 3; ++h) {
    const size_t base = B_BFRP_HDR + static_cast<size_t>(h) * 4;
    put(base, sw(cfg.bfrp_frame_ctl[h], r::SND_BFRP_FRAME_CTL_MSK,
                 r::SND_BFRP_FRAME_CTL_SH) |
                  sw(cfg.bfrp_duration[h], r::SND_BFRP_DURATION_MSK,
                     r::SND_BFRP_DURATION_SH));
    put(base + 1, pk4(&cfg.bfrp_addr1[h][0]));
    put(base + 2, pk2(&cfg.bfrp_addr1[h][4]) | (pk2(&cfg.bfrp_addr2[h][0]) << 16));
    put(base + 3, pk4(&cfg.bfrp_addr2[h][2]));
  }

  /* BFRP HE common + per-user (2 BFRP groups). */
  auto enc_bfrp_para = [&](size_t base, uint8_t g) {
    const devourer::SoundingBfrpCommon &cm = cfg.bfrp_common[g];
    put(base,
        sw(cm.trigger_type, r::SND_HE_BFRP_TRIGGER_INFO_MSK,
           r::SND_HE_BFRP_TRIGGER_INFO_SH) |
            sw(cm.ul_len, r::SND_HE_BFRP_UL_LENGTH_MSK,
               r::SND_HE_BFRP_UL_LENGTH_SH) |
            sw(cm.ul_bw, r::SND_HE_BFRP_UL_BW_MSK, r::SND_HE_BFRP_UL_BW_SH) |
            sw(cm.gi_ltf, r::SND_HE_BFRP_GI_LTF_MSK, r::SND_HE_BFRP_GI_LTF_SH) |
            sw(cm.num_he_ltf, r::SND_HE_BFRP_NUM_OF_HE_LTF_MSK,
               r::SND_HE_BFRP_NUM_OF_HE_LTF_SH));
    put(base + 1,
        sw(cm.ap_tx_pwr, r::SND_HE_BFRP_AP_TX_POWER_MSK,
           r::SND_HE_BFRP_AP_TX_POWER_SH) |
            sw(cm.ul_sr, r::SND_HE_BFRP_UL_SPATIAL_REUSE_MSK,
               r::SND_HE_BFRP_UL_SPATIAL_REUSE_SH));
    for (uint8_t u = 0; u < USERS_PER; ++u) {
      const devourer::SoundingBfrpUser &us = cfg.bfrp_user[g][u];
      put(base + 2 + 2 * u,
          sw(us.aid12, r::SND_HE_BFRP_U_AID12_MSK, r::SND_HE_BFRP_U_AID12_SH) |
              sw(us.ru_pos, r::SND_HE_BFRP_U_RU_POS_MSK,
                 r::SND_HE_BFRP_U_RU_POS_SH) |
              (us.ul_fec_code ? r::SND_HE_BFRP_U_UL_FEC_CODE : 0) |
              sw(us.ul_mcs, r::SND_HE_BFRP_U_UL_MCS_MSK,
                 r::SND_HE_BFRP_U_UL_MCS_SH) |
              (us.ul_dcm ? r::SND_HE_BFRP_U_UL_DCM : 0) |
              sw(us.ss_alloc, r::SND_HE_BFRP_U_SS_ALLOC_MSK,
                 r::SND_HE_BFRP_U_SS_ALLOC_SH));
      put(base + 3 + 2 * u,
          sw(us.fbseg_rexmit_bmp, r::SND_HE_BFRP_U_FB_REXMIT_MSK,
             r::SND_HE_BFRP_U_FB_REXMIT_SH) |
              sw(us.ul_tgt_rssi, r::SND_HE_BFRP_U_UL_TGT_RSSI_MSK,
                 r::SND_HE_BFRP_U_UL_TGT_RSSI_SH));
    }
  };
  enc_bfrp_para(B_BFRP0, 0);
  enc_bfrp_para(B_BFRP1, 1);
  /* VHT BFRP retransmission bitmap dword left zero (HE path). */

  /* WD list (5 entries x 3 dwords). */
  for (uint8_t i = 0; i < 5; ++i) {
    const devourer::SoundingWd &w = cfg.wd[i];
    const size_t base = B_WD + static_cast<size_t>(i) * 3;
    put(base, sw(w.txpktsize, r::SND_WD_TXPKTSIZE_MSK, r::SND_WD_TXPKTSIZE_SH) |
                  sw(w.ndpa_duration, r::SND_WD_NDPA_DURATION_MSK,
                     r::SND_WD_NDPA_DURATION_SH));
    put(base + 1,
        sw(w.datarate, r::SND_WD_DATARATE_MSK, r::SND_WD_DATARATE_SH) |
            sw(w.macid, r::SND_WD_MACID_MSK, r::SND_WD_MACID_SH) |
            sw(w.data_bw, r::SND_WD_DATA_BW_MSK, r::SND_WD_DATA_BW_SH) |
            sw(w.gi_ltf, r::SND_WD_GI_LTF_MSK, r::SND_WD_GI_LTF_SH));
    put(base + 2,
        (w.stf_mode ? r::SND_WD_STF_MODE : 0) |
            (w.disdatafb ? r::SND_WD_DISDATAFB : 0) |
            (w.data_txcnt_lmt_sel ? r::SND_WD_DATA_TXCNT_LMT_SEL : 0) |
            sw(w.data_txcnt_lmt, r::SND_WD_DATA_TXCNT_LMT_MSK,
               r::SND_WD_DATA_TXCNT_LMT_SH) |
            (w.sifs_tx ? r::SND_WD_SIFS_TX : 0) |
            sw(w.snd_pkt_sel, r::SND_WD_SND_PKT_SEL_MSK,
               r::SND_WD_SND_PKT_SEL_SH) |
            sw(w.ndpa, r::SND_WD_NDPA_MSK, r::SND_WD_NDPA_SH));
  }

  /* F2P(BFRP) sub-content (2 dwords) + SFP period (2 dwords). */
  for (uint8_t i = 0; i < 2; ++i)
    put(B_F2P + i, sw(cfg.csi_len_bfrp[i], r::SND_F2P_CSI_LEN_BFRP_MSK,
                      r::SND_F2P_CSI_LEN_BFRP_SH) |
                       sw(cfg.tb_t_pe_bfrp[i], r::SND_F2P_TB_T_PE_BFRP_MSK,
                          r::SND_F2P_TB_T_PE_BFRP_SH));
  put(B_SFP, sw(cfg.f2p_type, r::SND_SFP_F2P_TYPE_MSK, r::SND_SFP_F2P_TYPE_SH) |
                 sw(cfg.f2p_index, r::SND_SFP_F2P_INDEX_MSK,
                    r::SND_SFP_F2P_INDEX_SH) |
                 sw(cfg.f2p_period, r::SND_SFP_F2P_PERIOD_MSK,
                    r::SND_SFP_F2P_PERIOD_SH));
  put(B_SFP + 1, cfg.sounding_en ? r::SND_SFP_SOUNDING_EN : 0);
  return c;
}

} /* namespace kestrel::sched */

#endif /* KESTREL_SCHED_ENCODE_H */
