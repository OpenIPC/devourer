/* Headless golden-bytes guard for the Kestrel 802.11ax trigger-UL + TWT H2C
 * content encoders (src/kestrel/SchedEncode.h). Each check builds a known
 * config and asserts the emitted LE dwords equal the hand-computed vendor
 * SET_WORD layout (mac_ax/tblupd.c, twt.c, mac_8852c/tblupd_8852c.c). Pure
 * byte-level — no hardware. On-air validation (an fw-aired trigger decoded by a
 * witness) is tier-A. Compiled only when a Kestrel variant is built. */
#include <cstdint>
#include <cstdio>
#include <vector>

#include "kestrel/SchedEncode.h"

using namespace kestrel;

static int failures = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      failures++;                                                              \
    }                                                                          \
  } while (0)

static uint32_t dw(const std::vector<uint8_t> &b, size_t i) {
  return kestrel::le32(b.data() + i * 4);
}

int main() {
  /* ---- F2P trigger, 8852B v0: 1 user, BW20, MCS0, full-20 RU (61), AID 1 ---- */
  {
    devourer::TriggerConfig cfg;
    cfg.ul_bw = 0;
    cfg.gi_ltf = 1;
    cfg.num_he_ltf = 2;
    cfg.ap_tx_power = 20;
    cfg.pri20_bitmap = 0x1;
    cfg.n_users = 1;
    cfg.users[0].aid12 = 1;
    cfg.users[0].macid = 3;
    cfg.users[0].ru_alloc = 61;
    cfg.users[0].ul_mcs = 0;
    cfg.users[0].ss = 1;
    cfg.users[0].tgt_rssi_dbm = -60; /* -> 50 */
    cfg.users[0].pref_ac = 2;
    cfg.trig_rate = 0;
    cfg.mode = 1;
    cfg.frexch_type = 0;

    std::vector<uint8_t> b = sched::encode_f2p_trigger(cfg, /*v1=*/false);
    CHECK(b.size() == 236); /* sizeof(fwcmd_test_para) */
    /* dword0: ulbw[1:0]=0, giltf[3:2]=1, numltf[6:4]=2, txpwr[14:9]=20,
     * usernum[19:16]=1, bitmap[31:24]=1. */
    uint32_t d0 = (1u << 2) | (2u << 4) | (20u << 9) | (1u << 16) | (1u << 24);
    CHECK(dw(b, 0) == d0);
    /* dword1 (user A): aid12[11:0]=1, mcs[15:12]=0, macid[23:16]=3, ru[31:24]=61. */
    uint32_t d1 = 1u | (3u << 16) | (61u << 24);
    CHECK(dw(b, 1) == d1);
    /* dword2 (user B): fec[0]=0, dcm[1]=0, ss_alloc[7:2]=0 (1 SS), tgtrssi[14:8]=50. */
    uint32_t d2 = (50u << 8);
    CHECK(dw(b, 2) == d2);
    /* dep-user pref_AC byte at offset 36 (byte9). */
    CHECK(b[36] == 2);
    /* tf_wd dword13 at offset 40: datarate=0, mode[17:16]=1, frexch[23:18]=0. */
    CHECK(dw(b, 10) == (1u << 16));
    /* users 1..3 + DL-burst dwords stay zero. */
    CHECK(dw(b, 3) == 0 && dw(b, 4) == 0);
    /* f2p_wd dword14: cmd_qsel=MG0(18) | FS(bit11) | LS(bit10) |
     * total_number=1(<<12) | length=9(<<24) — frame = 16+8+5+4 = 33 B = 9 dw. */
    CHECK(dw(b, 11) == (18u | (1u << 10) | (1u << 11) | (1u << 12) | (9u << 24)));
    CHECK(dw(b, 12) == 0); /* dword15 (rsvd) + DL-burst tx_cmd stay zero */
  }

  /* ---- F2P trigger, 8852C v1: user block extends to dword16, tf_wd at dword19,
   * pref_AC bytes at offset 68, size 288. ---- */
  {
    devourer::TriggerConfig cfg;
    cfg.ul_bw = 2; /* 80 MHz */
    cfg.n_users = 2;
    cfg.users[0].aid12 = 1;
    cfg.users[0].macid = 5;
    cfg.users[0].ru_alloc = 61;
    cfg.users[0].pref_ac = 3;
    cfg.users[1].aid12 = 2;
    cfg.users[1].macid = 6;
    cfg.users[1].ru_alloc = 65;
    cfg.users[1].pref_ac = 1;
    cfg.mode = 2;

    std::vector<uint8_t> b = sched::encode_f2p_trigger(cfg, /*v1=*/true);
    CHECK(b.size() == 288); /* sizeof(fwcmd_test_para_v1) */
    CHECK(dw(b, 0) == ((2u << 0) | (1u << 4) /*numltf default 1*/ |
                       (20u << 9) /*ap_tx default 20*/ | (2u << 16) | (1u << 24)));
    /* user0 at dword1/2, user1 at dword3/4. */
    CHECK(dw(b, 1) == (1u | (5u << 16) | (61u << 24)));
    CHECK(dw(b, 3) == (2u | (6u << 16) | (65u << 24)));
    /* pref_AC bytes at offset 68 (dword17_0) and 69. */
    CHECK(b[68] == 3 && b[69] == 1);
    /* tf_wd dword19 at offset 76: mode[17:16]=2. */
    CHECK(dw(b, 19) == (2u << 16));
  }

  /* ---- TWT info: individual, trigger-enabled, AP role, config 1, flow 0 ---- */
  {
    devourer::TwtConfig cfg;
    cfg.broadcast = false;
    cfg.trigger = true;
    cfg.config_id = 1;
    cfg.flow_id = 0;
    cfg.port = 0;
    cfg.ap_role = true;
    cfg.wake_exp = 10;
    cfg.wake_man = 512;
    cfg.min_wake_dur = 255;
    cfg.trgt_tsf = 0x1122334455667788ull;

    std::vector<uint8_t> b = sched::encode_twtinfo(cfg, /*act=*/0);
    CHECK(b.size() == 24); /* 6-dword fwcmd_twtinfo_upd */
    /* dword0: nego=0, trigger BIT(2), id[16:14]=1, nic_role BIT(25). */
    uint32_t d0 = (1u << 2) | (1u << 14) | (1u << 25);
    CHECK(dw(b, 0) == d0);
    /* dword1: wake_man[15:0]=512, wake_exp[20:16]=10, dur[31:24]=255. */
    uint32_t d1 = 512u | (10u << 16) | (255u << 24);
    CHECK(dw(b, 1) == d1);
    CHECK(dw(b, 2) == 0x55667788u); /* trgt_l */
    CHECK(dw(b, 3) == 0x11223344u); /* trgt_h */
    CHECK(dw(b, 4) == 0 && dw(b, 5) == 0);
  }

  /* ---- TWT act: bind macid 7 to config 1 (add) ---- */
  {
    devourer::TwtStaAct a;
    a.macid = 7;
    a.config_id = 1;
    a.action = 0;
    std::vector<uint8_t> b = sched::encode_twt_act(a);
    CHECK(b.size() == 8);
    CHECK(dw(b, 0) == (7u | (1u << 8))); /* macid | id<<8 | act(0)<<11 */
    CHECK(dw(b, 1) == 0);
  }

  /* ---- TWT announce ---- */
  {
    std::vector<uint8_t> b = sched::encode_twt_announce(9);
    CHECK(b.size() == 4);
    CHECK(dw(b, 0) == 9u);
  }

  /* ---- TWT-OFDMA cadence ---- */
  {
    devourer::TwtOfdmaConfig cfg;
    cfg.twt_id = 1;
    cfg.max_tf_retry = 3;
    cfg.max_dl_retry = 0;
    cfg.round_num = 4;
    cfg.preferred_ac = 2;
    cfg.htc_bsr_ctrl_en = true;
    cfg.round_interval_us = 1000;
    std::vector<uint8_t> b = sched::encode_twt_ofdma(cfg);
    CHECK(b.size() == 8);
    /* dword0: twt_id[4:2]=1, max_tf[12:5]=3, round_num[28:21]=4,
     * preferred_ac[30:29]=2, htc_bsr BIT(31). */
    uint32_t d0 = (1u << 2) | (3u << 5) | (4u << 21) | (2u << 29) | (1u << 31);
    CHECK(dw(b, 0) == d0);
    CHECK(dw(b, 1) == 1000u); /* round_interval[15:0] */
  }

  /* ---- UL_FIXINFO: tf_periodic, 1 STA (macid 4, RU 61, MCS2, 1 SS) ---- */
  {
    devourer::UlOfdmaConfig cfg;
    cfg.mode = 3; /* tf_periodic */
    cfg.interval_s = 1;
    cfg.tf_type = 0;
    cfg.ppdu_bw = 0;
    cfg.n_stas = 1;
    cfg.stas[0].macid = 4;
    cfg.stas[0].pref_ac = 0;
    cfg.stas[0].ru_pos = 61;
    cfg.stas[0].tgt_rssi_dbm = -60; /* -> 50 */
    cfg.stas[0].mcs = 2;
    cfg.stas[0].ss = 1;

    std::vector<uint8_t> b = sched::encode_ul_fixinfo(cfg);
    CHECK(b.size() == 27 * 4); /* 108 bytes */
    /* dword0 tbl_hdr: R_W BIT(0), length[22:13]=27, table_class[31:24]=7. */
    uint32_t d0 = 1u | (27u << 13) | (7u << 24);
    CHECK(dw(b, 0) == d0);
    /* dword1 cfg: mode[1:0]=3, interval[7:2]=1. */
    CHECK(dw(b, 1) == (3u | (1u << 2)));
    /* sta_info pair at dword5: macid_0[7:0]=4. */
    CHECK(dw(b, 5) == 4u);
    /* ulrua header A at dword9: sta_num[14:11]=1. */
    CHECK(dw(b, 9) == (1u << 11));
    /* first per-RU entry at dword11: tgt_rssi[7:1]=50, mac_id[15:8]=4,
     * ru_pos[23:16]=61. */
    uint32_t rua_a = (50u << 1) | (4u << 8) | (61u << 16);
    CHECK(dw(b, 11) == rua_a);
    /* per-RU entry B at dword12: ss[19:17]=0 (1 SS), mcs[23:20]=2. */
    CHECK(dw(b, 12) == (2u << 20));
    /* second RU slot (dword13/14) unused -> zero. */
    CHECK(dw(b, 13) == 0 && dw(b, 14) == 0);
  }

  if (failures) {
    std::fprintf(stderr, "kestrel_sched: %d FAILURES\n", failures);
    return 1;
  }
  std::printf("kestrel_sched: all checks passed\n");
  return 0;
}
