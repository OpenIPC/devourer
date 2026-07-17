#ifndef KESTREL_SCHED_ENCODE_H
#define KESTREL_SCHED_ENCODE_H

#include <cstdint>
#include <vector>

#include "KestrelLe.h"
#include "MacRegAx.h"
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

} /* namespace kestrel::sched */

#endif /* KESTREL_SCHED_ENCODE_H */
