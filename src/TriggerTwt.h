#pragma once

/* TriggerTwt — the 802.11ax trigger-based UL + TWT session knobs.
 *
 * These are devourer-native aggregates: the small set of fields a caller
 * actually sets to make the firmware air an HE Trigger frame (UL-OFDMA grant)
 * or to program a TWT / UL-OFDMA schedule. The Kestrel HAL maps them onto the
 * vendor fw command surface (F2P trigger, TWT info/act/announce/OFDMA,
 * UL_FIXINFO); the raw 46-/72-dword vendor structs never surface here.
 *
 * The plane split, same as the rest of the AX MAC surface:
 *   - The AP (this driver) owns the schedule and builds the frames the fw airs.
 *   - The STA MAC fires its UL-OFDMA transmission at trigger+SIFS in hardware.
 * TWT negotiation (the S1G Action frames) is host-built like any mgmt frame;
 * what the fw provides is the *scheduling* machinery these structs drive.
 *
 * Only trigger frames are 802.11 CONTROL frames (FC type 1, subtype 2), aired
 * in a legacy/non-HT PPDU — so any monitor, including an 11ac witness, captures
 * and decodes the bytes (see TriggerParse.h); no HE PHY is needed to validate
 * what flew.  Docs: docs/he-extended-range.md is the sibling per-packet path;
 * this is the scheduled-UL path. */

#include <array>
#include <cstddef>
#include <cstdint>

namespace devourer {

/* One user's UL grant inside a Basic Trigger (one HE User Info field). */
struct TrigUser {
  uint16_t aid12 = 0;  /* 12-bit association id of the granted STA (0 = the
                        * unassociated/random-access slot) */
  uint8_t macid = 0;   /* fw macid the grant is scored against */
  /* 8-bit RU Allocation subfield (802.11ax Trigger User Info). 61 = 242-tone =
   * a full 20 MHz RU. Build it with he_ru_alloc(). NOTE: whether the fw's
   * ru_pos field wants this exact 802.11 code or its own RU index is verified
   * on-air from the aired trigger's decoded User Info (runtime discovery). */
  uint8_t ru_alloc = 61;
  uint8_t ul_mcs = 0;  /* 0..11 */
  uint8_t ss = 1;      /* spatial-stream count for this user (1..8) */
  bool ldpc = false;   /* UL FEC: false = BCC, true = LDPC */
  bool dcm = false;    /* dual-carrier modulation */
  int8_t tgt_rssi_dbm = -60; /* AP's requested UL receive level (dBm) */
  uint8_t pref_ac = 0; /* 0 = BE, 1 = BK, 2 = VI, 3 = VO */
};

/* A Basic Trigger (UL-OFDMA) — the one-shot F2P command config. Defaults air a
 * single-user, full-20 MHz, MCS0 Basic Trigger, so a demo needs only a macid /
 * aid to fire one. */
struct TriggerConfig {
  uint8_t ul_bw = 0;         /* solicited UL BW: 0 = 20, 1 = 40, 2 = 80, 3 = 160 */
  uint8_t gi_ltf = 0;        /* HE GI+LTF combo (0 = 1x/1.6us short) */
  uint8_t num_he_ltf = 1;    /* HE-LTF symbol count (0..7) */
  uint8_t ap_tx_power = 20;  /* AP TX power field, 0..63 (Common Info) */
  /* UL Length (Common Info B4-B15): the L-SIG Length the solicited TB PPDU
   * carries — a station computes its TB-PPDU duration from it, so a Basic
   * Trigger with UL Length 0 elicits NO response. 0x1F4 (~a few-hundred-us TB
   * PPDU, enough for a QoS-Null even with no buffered data) is a safe default;
   * tune to the granted RU/MCS/duration for a data grant. */
  uint16_t ul_length = 0x1F4;
  uint8_t pri20_bitmap = 0x1; /* primary-20 punctured-channel bitmap */
  uint16_t trig_rate = 0;    /* the trigger frame's OWN legacy-PPDU TX rate
                              * (AX 9-bit rate code; 0 = lowest OFDM). Trigger
                              * frames air non-HT so any RX can decode them. */
  uint8_t mode = 0;          /* fw f2p "mode" (2-bit) — undocumented, swept in
                              * kestrelprobe (runtime discovery) */
  uint8_t frexch_type = 0;   /* fw frame-exchange type (6-bit) — undocumented */
  /* Transmitter Address of the aired Trigger (host-injection path). A station
   * only answers a Trigger whose TA equals the BSSID it associated to, so this
   * MUST be the AP's own address; all-zero falls back to the injection SA. */
  std::array<uint8_t, 6> ta{};
  /* Receiver Address. Zero = broadcast (a Basic Trigger addresses its users by
   * AID12, so broadcast RA is standard); set to a STA MAC to unicast. */
  std::array<uint8_t, 6> ra{};
  std::array<TrigUser, 8> users{}; /* up to 4 on the 8852B (v0), 8 on the 8852C
                                    * (v1); n_users above the die max is clamped */
  uint8_t n_users = 1;
};

/* Encode a target-RSSI dBm into the 7-bit 802.11ax Target RSSI subfield:
 * 0..90 = -110..-20 dBm (encoded = dBm + 110); 127 = "transmit at max power"
 * (no RSSI target, selected by dbm >= 0). Out-of-range dBm clamps into 0..90. */
inline uint8_t he_tgt_rssi_enc(int dbm) {
  if (dbm >= 0)
    return 127;
  int v = dbm + 110;
  if (v < 0)
    v = 0;
  if (v > 90)
    v = 90;
  return static_cast<uint8_t>(v);
}

/* Encode the 8-bit RU Allocation subfield (802.11ax Trigger User Info) for a
 * whole-RU-size allocation. `index` selects among equal-size RUs in the BW.
 * 242-tone (index 0) = a full 20 MHz RU = 61 (the TriggerConfig default). This
 * is the 802.11 standard code; the fw's own ru_pos convention is verified
 * on-air (see TrigUser::ru_alloc). */
inline uint8_t he_ru_alloc(unsigned tones, unsigned index = 0) {
  switch (tones) {
  case 26:
    return static_cast<uint8_t>(index <= 36 ? index : 0); /* 26-tone RU #index */
  case 52:
    return static_cast<uint8_t>(37 + (index & 0xf));  /* 52-tone */
  case 106:
    return static_cast<uint8_t>(53 + (index & 0x7));  /* 106-tone */
  case 242:
    return static_cast<uint8_t>(61 + (index & 0x3));  /* 242-tone / full 20 MHz */
  case 484:
    return static_cast<uint8_t>(65 + (index & 0x1));  /* 484-tone / full 40 MHz */
  case 996:
    return 67;                                        /* 996-tone / full 80 MHz */
  default:
    return 61;                                        /* fallback: full 20 MHz */
  }
}

/* TWT agreement config (mac_twt_info_upd). The AP owns the timetable; trgt_tsf
 * is an absolute 64-bit TSF instant (read ReadTsf(), add lead time). */
struct TwtConfig {
  bool broadcast = false; /* false = individual TWT, true = broadcast TWT */
  bool trigger = true;    /* trigger-enabled TWT (AP sends triggers in the SP) */
  bool announced = false; /* announced (true) vs unannounced (false) flow */
  bool protection = false;
  uint8_t flow_id = 0;    /* TWT flow id (0..7) */
  uint8_t config_id = 0;  /* fw TWT config slot (0..7) */
  uint8_t port = 0;       /* MAC port (0..3) */
  uint8_t band = 0;       /* 0 = band0/2.4-5G, 1 = band1 */
  uint8_t wake_exp = 10;  /* wake-interval exponent (interval = man << exp us) */
  uint16_t wake_man = 512; /* wake-interval mantissa */
  uint8_t min_wake_dur = 255; /* nominal minimum wake duration (unit below) */
  bool wake_dur_us256 = false; /* dur unit: false = 256us, true = 1 TU */
  uint64_t trgt_tsf = 0;  /* absolute target wake time (TSF microseconds) */
  bool ap_role = true;    /* nic_role: AP (true) vs STA (false) */
};

/* Bind / unbind a STA (macid) to a TWT agreement (mac_twt_act). */
struct TwtStaAct {
  uint8_t macid = 0;
  uint8_t config_id = 0;
  uint8_t action = 0; /* 0 add, 1 del, 2 terminate, 3 suspend, 4 resume */
};

/* TWT-OFDMA cadence (mac_twt_ofdma_info_upd, fw func 0x03). This is the "fw
 * autonomously airs trigger rounds inside the TWT SP" lever. The func is
 * non-canonical in the vendor tree (gated MAC_FEAT_TWT_OFDMA_EN, default 0) —
 * whether the shipped fw implements it is a runtime discovery; UlOfdmaConfig is
 * the canonical fallback. */
struct TwtOfdmaConfig {
  uint8_t option = 0;
  uint8_t twt_id = 0;         /* which TWT config to schedule */
  uint8_t max_tf_retry = 3;   /* max Basic-Trigger retries per round */
  uint8_t max_dl_retry = 0;
  uint8_t round_num = 1;      /* UL-OFDMA rounds per TWT service period */
  uint8_t preferred_ac = 0;   /* 0 BE / 1 BK / 2 VI / 3 VO */
  bool htc_bsr_ctrl_en = false;
  uint16_t round_interval_us = 0; /* spacing between rounds */
};

/* One STA in the UL_FIXINFO scheduler table. */
struct UlOfdmaSta {
  uint8_t macid = 0;
  uint8_t pref_ac = 0;
  uint8_t ru_pos = 0;          /* fw RU index (verified on-air, see he_ru_alloc) */
  int8_t tgt_rssi_dbm = -60;
  uint8_t mcs = 0;
  uint8_t ss = 1;
  bool ldpc = false;
  bool dcm = false;
  uint16_t bsr_length = 0;     /* buffer-status hint (0 = fw decides) */
};

/* UL_FIXINFO — the fw's production UL-OFDMA scheduler table (mac_upd_ul_fixinfo,
 * CLASS_UL_FIXINFO). mode=tf_periodic makes the fw air Triggers autonomously at
 * `interval` seconds. Canonical (MAC_FEAT_ULOFDMA), so it is the fallback when
 * TWT-OFDMA is absent from the shipped fw. */
struct UlOfdmaConfig {
  uint8_t mode = 3;      /* 0 periodic, 1 normal, 2 non-trigger, 3 tf_periodic */
  uint8_t interval_s = 1; /* schedule interval (seconds) */
  uint8_t bsr_thold = 0;
  uint8_t store_mode = 0;
  uint8_t tf_type = 0;   /* trigger type (0 = Basic) */
  uint16_t data_rate = 0; /* trigger PPDU rate (AX 9-bit code) */
  uint16_t apep_len = 0;
  bool istwt = false;
  uint8_t ppdu_bw = 0;   /* 0 = 20, 1 = 40, 2 = 80, 3 = 160 */
  uint8_t gi_ltf = 0;
  uint8_t multiport_id = 0;
  std::array<UlOfdmaSta, 8> stas{};
  uint8_t n_stas = 1;
};

/* Decoded TWT C2H event (twt.notify / twt.wait_anno), emitted by the RX loop. */
struct TwtNotify {
  uint8_t type = 0;    /* fw notify type (TWT_NOTIFY_EVT) */
  uint8_t twt_id = 0;
  uint64_t tsf = 0;    /* TSF-stamped notify instant */
  bool wait_anno = false; /* true = WAIT_ANNOUNCE (macid list), false = NOTIFY */
  uint8_t wait_case = 0;
  uint8_t macid0 = 0, macid1 = 0, macid2 = 0;
};

/* -------- host-side Basic Trigger frame builder (step-0 raw injection) --------
 *
 * Assembles an 802.11 Basic Trigger control frame body (no radiotap, no FCS —
 * the caller prepends radiotap for send_packet and the MAC appends the FCS)
 * into `out` (>= 32 bytes for one user). Returns the byte length written, or 0
 * on overflow. Symmetric with parse_trigger() (TriggerParse.h); the round-trip
 * is a headless selftest. This path exists to (a) validate witness tooling
 * before the fw path, and (b) probe whether the Kestrel TX path even accepts a
 * control-frame subtype. The fw F2P path (SendTrigger) is the production one. */
inline size_t build_basic_trigger(const TriggerConfig &cfg,
                                  const uint8_t ra[6], const uint8_t ta[6],
                                  uint8_t *out, size_t cap) {
  const uint8_t nuser = cfg.n_users == 0 ? 1 : cfg.n_users;
  /* Basic-Trigger User Info is 6 bytes (5 common + 1 Trigger Dependent). A
   * Padding field (a User Info with AID12=4095 then 0xFF octets) marks the end
   * of the list and gives the responder preparation time. */
  const size_t kPad = 4;
  const size_t need = 2 /*FC*/ + 2 /*dur*/ + 6 /*RA*/ + 6 /*TA*/ + 8 /*common*/ +
                      static_cast<size_t>(nuser) * 6 /*user info*/ +
                      2 /*pad marker*/ + kPad /*padding*/;
  if (out == nullptr || cap < need)
    return 0;
  size_t o = 0;
  out[o++] = 0x24; /* FC: version0 | type1(control) | subtype2(trigger) */
  out[o++] = 0x00; /* FC flags */
  out[o++] = 0x00; /* Duration lo (fw/HW may overwrite) */
  out[o++] = 0x00; /* Duration hi */
  for (int i = 0; i < 6; ++i)
    out[o++] = ra ? ra[i] : 0xff; /* RA (default broadcast) */
  for (int i = 0; i < 6; ++i)
    out[o++] = ta ? ta[i] : 0x00; /* TA */
  /* Common Info (8 bytes, little-endian bitfield): Trigger Type[3:0]=0 (Basic),
   * UL Length[15:4], More TF[16], CS Required[17], UL BW[19:18], GI/LTF[21:20],
   * MU-MIMO LTF[22], Num HE-LTF[25:23], ... AP TX Power[33:28]. */
  uint64_t common = 0;
  common |= (static_cast<uint64_t>(cfg.ul_length) & 0xfff) << 4;
  common |= (static_cast<uint64_t>(cfg.ul_bw) & 0x3) << 18;
  common |= (static_cast<uint64_t>(cfg.gi_ltf) & 0x3) << 20;
  common |= (static_cast<uint64_t>(cfg.num_he_ltf) & 0x7) << 23;
  common |= (static_cast<uint64_t>(cfg.ap_tx_power) & 0x3f) << 28;
  for (int i = 0; i < 8; ++i)
    out[o++] = static_cast<uint8_t>((common >> (8 * i)) & 0xff);
  /* Per-user info (5 bytes = 40 bits, little-endian): AID12[11:0],
   * RU Alloc[19:12], UL FEC/coding[20], UL MCS[24:21], UL DCM[25],
   * SS Alloc[31:26], Target RSSI[38:32]. */
  for (uint8_t u = 0; u < nuser; ++u) {
    const TrigUser &tu = cfg.users[u];
    uint64_t ui = 0;
    ui |= static_cast<uint64_t>(tu.aid12) & 0xfff;
    ui |= (static_cast<uint64_t>(tu.ru_alloc) & 0xff) << 12;
    ui |= (static_cast<uint64_t>(tu.ldpc ? 1 : 0)) << 20;
    ui |= (static_cast<uint64_t>(tu.ul_mcs) & 0xf) << 21;
    ui |= (static_cast<uint64_t>(tu.dcm ? 1 : 0)) << 25;
    ui |= (static_cast<uint64_t>(tu.ss) & 0x3f) << 26;
    ui |= (static_cast<uint64_t>(he_tgt_rssi_enc(tu.tgt_rssi_dbm)) & 0x7f) << 32;
    for (int i = 0; i < 5; ++i)
      out[o++] = static_cast<uint8_t>((ui >> (8 * i)) & 0xff);
    /* Trigger Dependent User Info (Basic Trigger): MPDU MU Spacing Factor[1:0],
     * TID Aggregation Limit[4:2], reserved[5], Preferred AC[7:6]. */
    out[o++] = static_cast<uint8_t>((tu.pref_ac & 0x3) << 6);
  }
  /* Padding: a User Info whose AID12 = 4095 marks the end of the user list,
   * then 0xFF octets give the responder time to prepare its TB PPDU. */
  out[o++] = 0xff;
  out[o++] = 0x0f; /* AID12 = 0xFFF (4095) in the low 12 bits */
  for (size_t i = 0; i < kPad; ++i)
    out[o++] = 0xff;
  return o;
}

} /* namespace devourer */
