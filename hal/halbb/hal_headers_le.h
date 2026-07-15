/* SHIM replacement for the vendor phl/hal_g6/hal_headers_le.h.
 *
 * The vendored halbb-G6 8852C source (hal/halbb/rtl8852c/vendor/, copied
 * verbatim by tools/vendor_halbb_8852c.sh) reaches this file via its own
 * `#include "../../hal_headers_le.h"`. The real header pulls in the entire
 * PHL/HAL/MAC/BTC type universe; we replace it with the minimal surface the RX
 * bring-up functions actually touch and route every register/OS primitive to
 * devourer's RtlAdapter via kestrel_halbb_bridge (halbb_bridge.h).
 *
 * Matching the real header's include guard (_HAL_HEADERS_LE_H_) makes any
 * stray direct include of the real file a no-op too.
 */
#ifndef _HAL_HEADERS_LE_H_
#define _HAL_HEADERS_LE_H_

#include <stddef.h>
#include <stdarg.h>
#include <stdlib.h>
#include "rtl8852c/shim/halbb_bridge.h"

/* ---- base scalar types (vendor phl_types.h spellings) ------------------- */
#define u8  unsigned char
#define s8  signed char
#define u16 unsigned short
#define s16 short
#define u32 unsigned int
#define s32 int
#define u64 unsigned long long
#define s64 long long
typedef u8 boolean;
#ifndef __cplusplus
  #ifndef bool
    #define bool  u8
  #endif
  #ifndef true
    #define true  1
  #endif
  #ifndef false
    #define false 0
  #endif
#endif

#ifndef BIT
#define BIT(x) (1u << (x))
#endif
/* BIT0..BIT31 come from the vendored rtw_general_def.h / hal_general_def.h. */

/* OS glue types the vendor structs embed (halbb_timer_list etc.). We never run
 * the timer/mutex machinery from the RX bring-up path — opaque stubs suffice so
 * struct layouts compile. */
typedef struct { void *p; } _os_timer;
typedef struct { void *p; } _os_mutex;
typedef struct { int v; } _os_atomic;

/* ---- PHL enums --------------------------------------------------------- */
enum phl_phy_idx { HW_PHY_0, HW_PHY_1, HW_PHY_MAX };

/* Vendored verbatim (pure enums/defines, no includes, no structs): rf_path,
 * band_type, channel_width, chan_offset, wlan_mode, rtw_gi_ltf, rtw_data_rate,
 * rtw_ac, rtw_rate_mode, ... — the whole general enum set at correct values. */
#include "rtl8852c/shim/rtw_general_def.h"

/* ---- PHL sizing constants (must match vendor for array layout) ---------- */
#define PHL_MAX_STA_NUM     128
#define RTW_PHL_MAX_RF_PATH 4

/* Only the notify events halbb emits are needed (values arbitrary — the
 * cmd_notify sink is inert on our RX path). */
enum phl_msg_evt_id {
  PHL_MSG_EVT_ID_NONE = 0, MSG_EVT_NOTIFY_BB, MSG_EVT_SCAN_START,
  MSG_EVT_P2P_SCAN_START
};
enum role_type { PHL_RTYPE_NONE = 0 };

/* CFO/RSSI per-STA stats + EDCCA cap (verbatim field sets the vendor accesses). */
struct rtw_cfo_info {
  s32 cfo_tail; s32 pre_cfo_avg; s32 cfo_avg; u16 cfo_cnt; u32 tp;
};
struct rtw_rssi_info { s16 snr_ma; }; /* only .snr_ma read (inert per-STA path) */
struct rtw_edcca_cap_t {
  u8 edcca_adap_th_2g, edcca_adap_th_5g, edcca_cbp_th_6g, edcca_carrier_sense_th;
};

enum phl_band_idx { HW_BAND_0 = 0, HW_BAND_1, HW_BAND_MAX };
#define MAX_BAND_NUM 2

/* Vendored verbatim (pure enums/defines, no includes): rtw_cv (CAV/CBV/..),
 * rtw_dv_sel (DAV/DDV), rtw_efuse_info (EFUSE_INFO_*), rtw_chip_id, n_rf_entity. */
#include "rtl8852c/shim/hal_general_def.h"

/* MP-mode predicates — always false on the monitor-RX path (normal, not MP).
 * void* params so any phl_com pointer type passes cleanly. */
static inline int phl_is_mp_mode(void *c) { (void)c; return 0; }
static inline int phl_mp_is_tmac_mode(void *c) { (void)c; return 0; }

enum rtw_hal_status {
  RTW_HAL_STATUS_SUCCESS = 0, RTW_HAL_STATUS_FAILURE, RTW_HAL_STATUS_RESOURCE,
  RTW_HAL_STATUS_IO_INIT_FAILURE, RTW_HAL_STATUS_MAC_INIT_FAILURE,
  RTW_HAL_STATUS_BB_INIT_FAILURE, RTW_HAL_STATUS_RF_INIT_FAILURE,
  RTW_HAL_STATUS_BTC_INIT_FAILURE, RTW_HAL_STATUS_HAL_INIT_FAILURE,
  RTW_HAL_STATUS_EFUSE_UNINIT, RTW_HAL_STATUS_EFUSE_IVALID_OFFSET,
  RTW_HAL_STATUS_EFUSE_PG_FAIL, RTW_HAL_STATUS_MAC_API_FAILURE,
  RTW_HAL_STATUS_BB_CH_INFO_LAST_SEG, RTW_HAL_STATUS_UNKNOWN_RFE_TYPE,
  RTW_HAL_STATUS_TIMEOUT, RTW_HAL_STATUS_NOT_SUPPORT
};

/* ---- external PHL structs ----------------------------------------------- */
/* rtw_phl_com_t: halbb reads only dev_cap.{rfe_type,dbcc_sup}. rfe_type drives
 * the RFE-specific gpio/gain branch, so the bb_info factory MUST set it from the
 * device's efuse RFE type. */
struct rtw_chan_def {
  enum band_type band;
  u8 chan;
  enum channel_width bw;
  enum chan_offset offset;
  u8 center_ch;
  u16 hw_value;
  u32 center_freq1;
  u32 center_freq2;
  bool is_dfs;
};

enum rtw_drv_mode {
  RTW_DRV_MODE_NORMAL = 0, RTW_DRV_MODE_EQC = 1, RTW_DRV_MODE_HIGH_THERMAL = 2,
  RTW_DRV_MODE_SNIFFER = 3
};
enum tx_pause_rson {
  PAUSE_RSON_NOR_SCAN, PAUSE_RSON_UNSPEC_BY_MACID, PAUSE_RSON_RFK,
  PAUSE_RSON_PSD, PAUSE_RSON_DFS, PAUSE_RSON_DFS_CSA, PAUSE_RSON_DFS_CSA_MG,
  PAUSE_RSON_DFS_CAC, PAUSE_RSON_MAX
};

struct dev_cap_t {
  u8 rfe_type; bool dbcc_sup; u8 xcap; u8 nb_config; u8 antdiv_sup; u8 io_ofld;
  struct rtw_edcca_cap_t edcca_cap;
};
struct rtw_phy_cap_t { u8 tx_path_num, rx_path_num; };
/* phy_sw_cap only read by the gc'd-out halbb_init_reg, but its TU must compile:
 * reproduce rtw_para_info_t verbatim. */
enum rtw_para_src {
  RTW_PARA_SRC_INTNAL, RTW_PARA_SRC_EXTNAL, RTW_PARA_SRC_EXTNAL_BUF,
  RTW_PARA_SRC_CUSTOM, RTW_PARA_SRC_MAX
};
struct rtw_para_info_t {
  enum rtw_para_src para_src;
  char para_path[256];
  char *hal_phy_folder;
  char postfix[33];
  u8 *ext_para_file_buf;
  u32 ext_para_file_buf_len;
  u32 para_data_len;
  u32 *para_data;
  bool loaded;
};
struct phy_sw_cap_t {
  struct rtw_para_info_t bb_phy_reg_info;
  struct rtw_para_info_t bb_phy_reg_gain_info;
};
struct rtw_phl_com_t {
  struct dev_cap_t dev_cap;
  struct rtw_phy_cap_t phy_cap[MAX_BAND_NUM]; /* [band].tx_path_num (inert cmac) */
  struct phy_sw_cap_t phy_sw_cap[2];
  enum rtw_drv_mode drv_mode;
  u8 edcca_mode;
};
/* Per-STA info — only touched by inert per-STA CMAC/BF/stat helpers on RX. */
struct rtw_wifi_role_link_t { u8 hw_band; };
struct rtw_hal_stainfo_t {
  struct rtw_rssi_info rssi_stat;
  struct rtw_cfo_info cfo_stat;
};
struct rtw_phl_stainfo_t {
  struct rtw_wifi_role_link_t *rlink;
  struct rtw_hal_stainfo_t *hal_sta;
  struct rtw_chan_def chandef;
  u16 macid;
  bool active;
  struct { u32 rx_tf_cnt; } stats; /* only .rx_tf_cnt read (inert per-STA) */
};

/* MAC per-STA CMAC control (real one is a bitfield struct). Only the inert
 * halbb_config_cmac_tbl_ax helper writes it — plain u32 fields suffice. */
struct rtw_hal_mac_ax_cctl_info {
  u32 txpwr_mode, ntx_path_en, path_map_a, path_map_b, path_map_c, path_map_d,
      antsel_a, antsel_b, antsel_c, antsel_d, doppler_ctrl, txpwr_tolerence;
};

/* External-FEM presence (verbatim halrf_outsrc_def.h). elna_* drives the
 * is_efem gpio/gain branch — the factory can override; default (all zero =
 * non-EFEM) matches the compact USB dongles we target. */
struct halrf_fem_info { u8 elna_2g, elna_5g, elna_6g, epa_2g, epa_5g, epa_6g; };
static inline struct halrf_fem_info rtw_hal_rf_efem_info(void *h) {
  struct halrf_fem_info f = {0, 0, 0, 0, 0, 0}; (void)h; return f;
}

/* halrf TSSI (TX) entry — inert on the RX path (behind phl_is_mp_mode). */
static inline u32 rtw_hal_rf_tssi_scan_ch(void *h, enum phl_phy_idx p,
                                          enum rf_path rp) {
  (void)h; (void)p; (void)rp; return 0;
}

enum phl_rf_mode {
  RF_MODE_NORMAL = 0, RF_MODE_SHUTDOWN = 1, RF_MODE_STANDBY = 2,
  RF_MODE_RX = 3, RF_MODE_TX = 4, RF_MODE_MAX
};

/* RF synth config (TX/standby) — inert on monitor RX. */
static inline enum rtw_hal_status rtw_hal_rf_syn_config(void *h,
    u8 syn_id, enum phl_phy_idx phy, u8 path, u8 turn_on) {
  (void)h; (void)syn_id; (void)phy; (void)path; (void)turn_on;
  return RTW_HAL_STATUS_SUCCESS;
}

/* Efuse read — return FAILURE so RX gain-K falls back to table defaults (the
 * efuse RX-gain-K trim is a sensitivity refinement, not a hearing gate; the
 * 8852B first-light RX ran without it). Route to real efuse later. */
static inline enum rtw_hal_status rtw_hal_efuse_get_info(void *h,
    enum rtw_efuse_info info, void *value, u8 size) {
  (void)h; (void)info; (void)value; (void)size; return RTW_HAL_STATUS_FAILURE;
}

/* Memory helpers (vendor _os_mem_*). Byte loop — no libc dependency in the
 * shim-compiled vendor C. */
static inline void _os_mem_set(void *drv, void *d, int v, u32 n) {
  unsigned char *p = (unsigned char *)d; (void)drv; while (n--) *p++ = (unsigned char)v;
}
static inline void _os_mem_cpy(void *drv, void *d, const void *s, u32 n) {
  unsigned char *dp = (unsigned char *)d; const unsigned char *sp = (const unsigned char *)s;
  (void)drv; while (n--) *dp++ = *sp++;
}
static inline int _os_mem_cmp(void *drv, const void *a, const void *b, u32 n) {
  const unsigned char *x = (const unsigned char *)a, *y = (const unsigned char *)b;
  (void)drv; while (n--) { if (*x != *y) return (int)*x - (int)*y; x++; y++; } return 0;
}
static inline u64 _os_division64(u64 a, u64 b) { return b ? a / b : 0; }

/* String helpers (vendor _os_*): inert on RX (debug/parse paths only). */
static inline int _os_snprintf(char *b, size_t n, const char *f, ...) {
  (void)f; if (n) b[0] = 0; return 0;
}
#define _os_sscanf(...) (0)
static inline int _os_strcmp(const char *a, const char *b) {
  while (*a && *a == *b) { a++; b++; }
  return (int)*(const unsigned char *)a - (int)*(const unsigned char *)b;
}

/* Direct phy-efuse read — FAILURE so callers fall back to defaults (RX gain-K
 * refinement, not a hearing gate; route to real efuse later). */
static inline enum rtw_hal_status rtw_hal_mac_read_phy_efuse(void *h, u32 addr,
    u32 size, u8 *value) {
  (void)h; (void)addr; (void)size; (void)value; return RTW_HAL_STATUS_FAILURE;
}

/* Inert MAC/BF/notify entries — never exercised on the monitor-RX bring-up. */
static inline enum rtw_hal_status rtw_hal_cmd_notify(struct rtw_phl_com_t *c,
    enum phl_msg_evt_id e, void *cmd, u8 band) {
  (void)c; (void)e; (void)cmd; (void)band; return RTW_HAL_STATUS_SUCCESS;
}
static inline enum rtw_hal_status rtw_hal_tx_pause(void *h, u8 band, u8 pause,
    enum tx_pause_rson rson) {
  (void)h; (void)band; (void)pause; (void)rson; return RTW_HAL_STATUS_SUCCESS;
}
static inline enum rtw_hal_status rtw_hal_mac_get_xcap(void *h, u8 sc, u32 *v) {
  (void)h; (void)sc; if (v) *v = 0; return RTW_HAL_STATUS_SUCCESS;
}
static inline enum rtw_hal_status rtw_hal_mac_set_xcap(void *h, u8 sc, u32 v) {
  (void)h; (void)sc; (void)v; return RTW_HAL_STATUS_SUCCESS;
}
static inline enum rtw_hal_status rtw_hal_mac_ax_init_bf_role(void *h, u8 r, u8 b) {
  (void)h; (void)r; (void)b; return RTW_HAL_STATUS_SUCCESS;
}
static inline enum rtw_hal_status rtw_hal_mac_ax_deinit_bfee(void *h, u8 b) {
  (void)h; (void)b; return RTW_HAL_STATUS_SUCCESS;
}
static inline enum rtw_hal_status rtw_hal_mac_write_pwr_ref_reg(void *h, u8 b) {
  (void)h; (void)b; return RTW_HAL_STATUS_SUCCESS;
}
static inline enum rtw_hal_status rtw_hal_mac_tx_path_map_cfg(void *h, void *c) {
  (void)h; (void)c; return RTW_HAL_STATUS_SUCCESS;
}

/* Heap glue (vendor hal_mem_ / _os_mem_ alloc). halbb allocates a few
 * sub-structs at init; back them with libc malloc. */
static inline void *_os_mem_alloc(void *drv, u32 sz) {
  (void)drv; return malloc(sz);
}
static inline void _os_mem_free(void *drv, void *p, u32 sz) {
  (void)drv; (void)sz; free(p);
}
#define hal_mem_alloc(h, sz)      _os_mem_alloc(halcom_to_drvpriv(h), sz)
#define hal_mem_free(h, p, sz)    _os_mem_free(halcom_to_drvpriv(h), p, sz)


/* Large PHL structs embedded by value but never read on the monitor-RX bring-up
 * path — opaque-ish stubs keep the enclosing halbb structs a valid size. If a
 * later TU actually reads a field, replace with the verbatim vendor definition. */
/* rtw_ext_pwr_lmt_info is vendor-defined (halbb_bb_wrapper_outsrc.h); only these
 * two are pure-PHL and absent from the vendored headers. */
struct rtw_phl_ext_pwr_lmt_info { u32 _stub[16]; };
struct rtw_mac_ax_sr_info { u32 _stub[16]; };

/* MU/H2C types touched by inert C2H/H2C helpers in the same TU (never called on
 * the RX path — stubbed to the accessed member paths only). */
#define H2CB_TYPE_LONG_DATA 1
#define HAL_MAX_MU_STA_NUM  8
struct rtw_g6_h2c_hdr {
  u8 h2c_class, h2c_func, type, rec_ack, done_ack;
  u16 content_len;
};
struct hal_mu_score_tbl {
  struct { u8 mu_sc_thr, mu_opt; } mu_ctrl;
  struct { u8 score[HAL_MAX_MU_STA_NUM]; } mu_score[HAL_MAX_MU_STA_NUM];
};

/* Minimal rtw_hal_com_t — only the fields the RX bring-up reads. drv_priv is
 * the kestrel_halbb_bridge*. Extend as the compile frontier requires. */
/* Vendor BB "outsrc" type headers (the real hal_headers_le.h includes these):
 * rtw_tpu_info + bb_tpu_all_info (bb_wrapper) and bb_tx/rx_path_en_info
 * (outsrc_def). Pulled early — guards make the later precomp re-includes no-ops
 * — so rtw_hw_band can embed the TPU types and the trx-path prototype matches. */
#include "halbb_outsrc_def.h"
struct rtw_hw_band {
  struct rtw_chan_def cur_chandef;
  struct rtw_tpu_info rtw_tpu_i;
  union bb_tpu_all_info bb_tpu_all_i; /* vendor union (from bb_wrapper_outsrc) */
};

/* MAC tx-path map cfg (bitfield) — passed to the inert tx_path_map_cfg helper. */
struct hal_txmap_cfg { u32 macid:8, n_tx_en:4, map_a:2, map_b:2, map_c:2, map_d:2; };

struct rtw_hal_com_t {
  void *drv_priv;      /* -> struct kestrel_halbb_bridge */
  u8   cv;             /* chip cut version */
  u8   dbcc_en;        /* dual-band concurrent (0 for us) */
  enum rtw_chip_id chip_id;             /* from vendored hal_general_def.h */
  struct dev_cap_t dev_hw_cap;          /* hw capability (mirror of dev_cap) */
  u8   scanofld_en;
  u8   assoc_sta_cnt;
  struct hal_mu_score_tbl bb_mu_score_tbl;
  struct rtw_hw_band band[MAX_BAND_NUM]; /* band[0].cur_chandef read by 8852c api */
};

#define halcom_to_drvpriv(_hcom) ((struct kestrel_halbb_bridge *)((_hcom)->drv_priv))

/* MAC-plane calls: power-reg -> bridge; h2c -> inert stub (no FW path on RX). */
static inline u32 rtw_hal_mac_set_pwr_reg(struct rtw_hal_com_t *h, u8 band,
                                          u32 off, u32 val) {
  struct kestrel_halbb_bridge *b = halcom_to_drvpriv(h);
  (void)band; if (b->write_pwr) b->write_pwr(b->dev, off, val); return 0;
}
static inline u32 rtw_hal_mac_write_msk_pwr_reg(struct rtw_hal_com_t *h, u8 band,
                                                u32 off, u32 mask, u32 val) {
  struct kestrel_halbb_bridge *b = halcom_to_drvpriv(h);
  u32 cur = b->read_pwr ? b->read_pwr(b->dev, off) : 0;
  u32 sh = 0; u32 m = mask; while (m && !(m & 1u)) { m >>= 1; sh++; }
  (void)band; cur = (cur & ~mask) | ((val << sh) & mask);
  if (b->write_pwr) b->write_pwr(b->dev, off, cur); return 0;
}
static inline u32 rtw_hal_mac_get_pwr_reg(struct rtw_hal_com_t *h, u8 band,
                                          u32 off, u32 *val) {
  struct kestrel_halbb_bridge *b = halcom_to_drvpriv(h);
  (void)band; if (val) *val = b->read_pwr ? b->read_pwr(b->dev, off) : 0; return 0;
}
static inline u32 rtw_hal_mac_send_h2c(struct rtw_hal_com_t *h,
                                       struct rtw_g6_h2c_hdr *hdr, u32 *pv) {
  (void)h; (void)hdr; (void)pv; return 0;
}


/* ---- register plane: route to the bridge -------------------------------- */
static inline u32 hal_read32(struct rtw_hal_com_t *h, u32 addr) {
  struct kestrel_halbb_bridge *b = halcom_to_drvpriv(h);
  return b->read32(b->dev, addr);
}
static inline void hal_write32(struct rtw_hal_com_t *h, u32 addr, u32 val) {
  struct kestrel_halbb_bridge *b = halcom_to_drvpriv(h);
  b->write32(b->dev, addr, val);
}
static inline u16 hal_read16(struct rtw_hal_com_t *h, u32 addr) {
  return (u16)hal_read32(h, addr & ~0x3u);
}
static inline u8 hal_read8(struct rtw_hal_com_t *h, u32 addr) {
  return (u8)(hal_read32(h, addr & ~0x3u) >> ((addr & 0x3u) * 8));
}

/* ---- OS primitives ------------------------------------------------------ */
static inline void _os_delay_us(void *drv, u32 us) {
  struct kestrel_halbb_bridge *b = (struct kestrel_halbb_bridge *)drv;
  if (b && b->delay_us) b->delay_us(b->dev, us);
}
static inline void _os_delay_ms(void *drv, u32 ms) { _os_delay_us(drv, ms * 1000u); }
#define hal_mdelay(h, ms) _os_delay_ms(halcom_to_drvpriv(h), ms)
#define hal_udelay(h, us) _os_delay_us(halcom_to_drvpriv(h), us)

/* Timer/diagnostic glue — inert on the RX bring-up path (no watchdog started).
 * Variadic dbgdump routes to the bridge logline when present. */
#define _os_init_timer(drv, t, cb, ctx, id)  do { (void)(drv); (void)(t); } while (0)
#define _os_set_timer(drv, t, ms)            do { (void)(drv); (void)(t); } while (0)
#define _os_cancel_timer(drv, t)             do { (void)(drv); (void)(t); } while (0)
#define _os_release_timer(drv, t)            do { (void)(drv); (void)(t); } while (0)
static inline void _os_dbgdump(const char *fmt, ...) { (void)fmt; }

#endif /* _HAL_HEADERS_LE_H_ */
