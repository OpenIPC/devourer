/* SPDX-License-Identifier: BSD-3-Clause-Clear */
#ifndef MT76X0U_INTERNAL_H
#define MT76X0U_INTERNAL_H

/* Private to src/mt76x0u. Shaped after src/mt7612u/internal.h, cut down to
 * what phase 1 uses: the device, the sync-transfer pool, the RX ring and the
 * MCU channel. Internal symbols are mt0_ (mt7612u's are mt_) so both subtrees
 * link into one libdevourer without colliding. */

#if defined(__has_include)
#  if __has_include(<libusb.h>)
#    include <libusb.h>
#  else
#    include <libusb-1.0/libusb.h>
#  endif
#else
#  include <libusb-1.0/libusb.h>
#endif
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include "regs.h"
#include "include/mt76x0u/mt76x0u.h"

#define MT_SYNC_POOL 4   /* pooled transfers for the sync helpers */
#define MT_RX_RING  16
#define MT_RX_BUFSZ 4096

struct mt76x0u_dev;
struct mt0_async;
/* Each slot names its own ring, not dev->a: if a teardown has to leak a ring
 * whose transfers are still in flight, their completions must keep touching
 * the leaked ring and never a replacement one (as in mt7612u). */
struct mt0_slot {
	struct mt76x0u_dev *d; struct mt0_async *a; int idx;
};

/* The MT7612U ring minus its TX half: the MT76x0U port is receive-only, and
 * MCU commands go out synchronously on EP 0x08 (mcu.cpp), not through here. */
struct mt0_async {
	struct libusb_transfer *rx[MT_RX_RING];
	struct mt0_slot rx_slot[MT_RX_RING];
	uint8_t rx_buf[MT_RX_RING][MT_RX_BUFSZ];
	/* Guards running, rx_active, rx_inflight, rx_slot[].d and the counters. */
	std::mutex lock;
	std::condition_variable_any cv;
	std::thread evt;
	int evt_started;
	int running, rx_active;
	int rx_inflight;
	mt76x0u_rx_cb cb;
	void *cb_user;
	uint64_t rx_frames, rx_err, rx_dropped, rx_invalid;
	int no_device;            /* a transfer completed or resubmitted NO_DEVICE */
};

#define MT_EP4_NONE 0        /* mt76x0u_dev::ep4_owner values */
#define MT_EP4_RING 1
#define MT_EP4_SYNC 2

/* mt76_reg_pair: one register of a random read/write. `reg` is relative to
 * the base the call names (MT_MCU_MEMMAP_WLAN, MT_MCU_MEMMAP_RF). */
struct mt0_reg_pair {
	uint32_t reg;
	uint32_t value;
};

/* What mt76x0_phy_set_channel() derives from a chandef, for 20 and 40 MHz. */
struct mt0_chan_shape {
	uint8_t  primary;     /* control channel (chan->hw_value) */
	uint8_t  center;      /* channel the RF synthesiser is programmed to */
	uint8_t  width;       /* 20 or 40 (MHz) */
	uint8_t  ch_group;    /* ch_group_index: 0 = 20 MHz / HT40+, 1 = HT40- */
	uint16_t rf_bw_band;  /* RF_G_BAND or RF_A_BAND, | RF_BW_20 or RF_BW_40 */
	int      is_5ghz;
	uint32_t ext_cca;     /* MT_EXT_CCA_CFG value for ch_group */
	int      radar;       /* a DFS channel (gain adjust, see chan.cpp) */
};

/* Test seam: when a device carries one, every EP0 vendor request and every
 * synchronous bulk transfer goes here instead of to libusb (the RX ring does
 * not - it is libusb's async API and is not used by the transfer-log tests).
 * Return values are libusb's: bytes transferred / 0, or LIBUSB_ERROR_*.
 * Production code never sets it. */
struct mt0_io_hook {
	int (*ctrl)(void *user, uint8_t type, uint8_t req, uint16_t val,
	            uint16_t idx, uint8_t *data, uint16_t len);
	int (*bulk)(void *user, uint8_t ep, uint8_t *buf, int len, int *xfered,
	            unsigned timeout_ms);
	void *user;
};

struct mt76x0u_dev {
	libusb_context       *ctx;
	libusb_device_handle *h;
	const struct mt0_io_hook *io_hook;  /* tests only; see mt0_io_hook */

	uint32_t rev;             /* MT_ASIC_VERSION, e.g. 0x76100044 */
	uint32_t mac_rev;         /* MT_MAC_CSR0 */
	int      is_mt7630;       /* mt76_chip() == 0x7630 (mt76x0.h) */
	int      no_2ghz;         /* USB-id quirk (kernel driver_info = 1) */

	/* From the EFUSE (eeprom.cpp), as mt76x0_eeprom_init() derives them. */
	uint8_t  eeprom[MT76X0_EEPROM_SIZE];
	int      eeprom_loaded;
	uint8_t  ee_version, ee_fae;
	uint8_t  macaddr[6];
	int      has_2ghz, has_5ghz;
	uint16_t chainmask;       /* 0x101: mt76x02_init_device for !is_mt76x2 */
	uint8_t  freq_offset;     /* cal.rx.freq_offset (RF B0.R22 trim) */
	int8_t   temp_offset;     /* cal.rx.temp_offset */
	uint32_t rxfilter;        /* MT_RX_FILTR_CFG after init (mt76.rxfilter) */

	/* Channel state (chan.cpp). Written by the channel set and the
	 * calibration tick, both under the caller's control-plane lock. */
	struct mt0_chan_shape shape;  /* valid once shape_valid */
	int      shape_valid;
	int8_t   lna_gain;        /* cal.rx.lna_gain for the current channel */
	int8_t   rssi_offset[2];  /* cal.rx.rssi_offset */
	/* lna_gain and rssi_offset[0] packed (lna << 8 | offset), published by
	 * the channel set and read per frame on the event thread: one relaxed
	 * load gives a consistent pair (the MT7612U's rx_corr lesson). */
	std::atomic<uint32_t> rx_corr{0};
	int      mac_started;     /* MAC RX on (mt0_mac_start) */
	int      tune_once;       /* mt76x0u_set_tune_twice(d, 0); see chan.cpp */
	uint8_t  agc_gain_init[2], agc_gain_cur[2];
	uint8_t  agc_gain_adjust;
	int      low_gain;        /* -1 until the first tick, as the kernel */
	uint32_t false_cca;
	int8_t   temp_vco, temp;  /* last temperatures that triggered a cal */
	int      tssi_noted;      /* the TSSI-card notice is logged once */
	uint8_t  mcu_seq;
	/* Set when an MCU wait gave up, so its reply may still land on EP 5 and
	 * desync the next command; see mt0_mcu_send(). */
	uint8_t  mcu_stale_pending;
	/* MT76_STATE_MCU_RUNNING: set once the firmware answers (uploaded here,
	 * or found running). Routes register pairs through the MCU, as
	 * mt76u_wr_rp/rd_rp do; before it they are plain EP0 accesses. */
	int      mcu_running;
	/* Q_SELECT has been sent since the firmware came up. wifit3 measured on
	 * an MT7610U that CMD_RANDOM_READ times out if it goes first, and the
	 * kernel always sends Q_SELECT first (mt76x0_init_hardware), so the
	 * random read/write commands refuse until it has. */
	int      q_selected;
	/* An in-flight CMD_RANDOM_READ's pairs, filled from the matching reply
	 * by mcu_wait_resp() (the kernel's usb->mcu.rp). Under io_lock. */
	struct mt0_reg_pair *rp;
	int      rp_len;
	uint32_t rp_base;
	int      rp_filled;
	/* Who reads EP 4 (0x84): the RX ring, a synchronous drainer, or nobody.
	 * mcu_wait_resp() drains EP 4 only when nobody does - data parked there
	 * holds back MCU replies on EP 5 (measured on the MT7612U, same DMA). */
	std::atomic<int> ep4_owner{MT_EP4_NONE};
	/* Recursive: an MCU send holds it and nests vendor requests. */
	std::recursive_mutex io_lock;

	unsigned io_err;          /* EP0 transfers that exhausted their retries */
	int      ep0_quiet;       /* set: an EP0 request that exhausts its retries
	                           * is not logged (the caller expects some) */
	int      ep0_tries;       /* 0: VEND_RETRIES; else this many attempts */
	int      transfers_stranded; /* libusb still owns a cancelled transfer */
	std::mutex sync_pool_mu;
	struct libusb_transfer *sync_pool[MT_SYNC_POOL];
	int      sync_pool_n;

	struct mt0_async *a;
	FILE    *wrlog;           /* oracle-diff log of EP0 writes, or NULL */
	FILE    *mculog;          /* oracle-diff log of MCU commands, or NULL */
};

/* --- usb.cpp --- */
/* Open a device whose I/O all goes through `hook` (tests/mt76x0u_wire_harness). */
struct mt76x0u_dev *mt0_open_hooked(const struct mt0_io_hook *hook, int no_2ghz,
                                    const char **err);
void     mt0_dev_state_init(struct mt76x0u_dev *d);
void     mt0_dev_state_destroy(struct mt76x0u_dev *d);
int      mt0_adopt(struct mt76x0u_dev *d, libusb_device_handle *h,
                   libusb_context *ctx, const char **err);
void     mt0_close(struct mt76x0u_dev *d);
int      mt0_rr_chk(struct mt76x0u_dev *d, uint32_t addr, uint32_t *val);
uint32_t mt0_rr(struct mt76x0u_dev *d, uint32_t addr);
void     mt0_wr(struct mt76x0u_dev *d, uint32_t addr, uint32_t val);
int      mt0_rmw(struct mt76x0u_dev *d, uint32_t addr, uint32_t mask, uint32_t val);
int      mt0_wr_chk(struct mt76x0u_dev *d, uint32_t addr, uint32_t val);
void     mt0_io_clear(struct mt76x0u_dev *d);
void     mt0_io_restore(struct mt76x0u_dev *d, unsigned v);
unsigned mt0_io_errors(struct mt76x0u_dev *d);
#define  mt0_set(d, a, v)   mt0_rmw(d, a, v, v)
#define  mt0_clear(d, a, v) mt0_rmw(d, a, v, 0)
int      mt0_poll(struct mt76x0u_dev *d, uint32_t addr, uint32_t mask,
                  uint32_t val, int timeout_us);
int      mt0_vendor_req(struct mt76x0u_dev *d, uint8_t req, uint8_t type,
                        uint16_t val, uint16_t idx, void *buf, size_t len);
void     mt0_wr_copy(struct mt76x0u_dev *d, uint32_t offset, const void *data, int len);
void     mt0_single_wr(struct mt76x0u_dev *d, uint8_t req, uint16_t off, uint32_t val);
int      mt0_bulk(struct mt76x0u_dev *d, uint8_t ep, void *buf, int len,
                  int *xfered, unsigned timeout_ms);
int      mt0_wait_for_mac(struct mt76x0u_dev *d);
void     mt0_usleep(unsigned us);
/* Drain EP 4 synchronously; see the definition. */
size_t   mt0_rx_flush(struct mt76x0u_dev *d, int *reads_left = nullptr);

/* --- mcu.cpp --- */
int mt0_mcu_send(struct mt76x0u_dev *d, int cmd, const void *data, int len, int wait_resp);
int mt0_mcu_function_select(struct mt76x0u_dev *d, int func, uint32_t val);
int mt0_mcu_set_radio_state(struct mt76x0u_dev *d, int on);
int mt0_mcu_calibrate(struct mt76x0u_dev *d, int type, uint32_t param);
/* Register pairs at `base` + reg (mt76u_wr_rp / mt76u_rd_rp). With the MCU
 * running they go through CMD_RANDOM_WRITE / CMD_RANDOM_READ, at most
 * MT0_RP_MAX_PER_CMD pairs per command (a read refuses more); before it, as
 * EP0 register accesses. 0 on success, -1 on failure. */
int mt0_mcu_wr_rp(struct mt76x0u_dev *d, uint32_t base,
                  const struct mt0_reg_pair *pairs, int n);
int mt0_mcu_rd_rp(struct mt76x0u_dev *d, uint32_t base,
                  struct mt0_reg_pair *pairs, int n);
/* RF registers through the MCU (mt76x0_rf_wr/rr on USB). `offset` is
 * MT_RF(bank, reg). rr returns the value, or -1. */
int mt0_rf_wr(struct mt76x0u_dev *d, uint32_t offset, uint8_t val);
int mt0_rf_rr(struct mt76x0u_dev *d, uint32_t offset);

/* Pure helpers, exposed for tests/mt76x0u_fw_selftest.cpp. */
/* Pack `n` pairs as CMD_RANDOM_* payload: (base + reg, value) LE32 each.
 * Returns the byte count (8 * n). */
int mt0_mcu_pack_rp(uint8_t *out, uint32_t base,
                    const struct mt0_reg_pair *pairs, int n);
/* Fill `pairs` from a CMD_RANDOM_READ reply payload (the bytes after the
 * 4-byte RX FCE info word, trailer excluded). 0 on success; -1 when the
 * count or any echoed address disagrees with what was asked. */
int mt0_mcu_parse_rp(const uint8_t *payload, int len, uint32_t base,
                     struct mt0_reg_pair *pairs, int n);

/* --- eeprom.cpp --- */
/* Pure, over a 512-byte EFUSE image (tests/mt76x0u_eeprom_selftest.cpp). */
uint16_t mt0_ee_get(const uint8_t *ee, unsigned field);
int      mt0_field_valid(uint8_t val);
int      mt0_sign_extend(uint32_t val, unsigned size);
/* cal.rx.lna_gain for `chan` (mt76x02_get_rx_gain + mt76x02_get_lna_gain):
 * the per-channel correction wifit3 found 5 GHz RX deaf without. */
int8_t   mt0_ee_lna_gain(const uint8_t *ee, uint8_t chan);
void     mt0_ee_rssi_offset(const uint8_t *ee, uint8_t chan, int8_t out[2]);
/* Fill d's derived fields from d->eeprom (no I/O). */
int      mt0_ee_parse(struct mt76x0u_dev *d);
/* Read the EFUSE into d->eeprom and parse it. */
int      mt0_eeprom_init(struct mt76x0u_dev *d);

/* --- phy.cpp --- */
int      mt0_phy_wait_bbp_ready(struct mt76x0u_dev *d);
/* mt76x0_rf_patch_reg_array's per-register override, USB arms. Pure. */
int      mt0_rf_patch_value(uint32_t reg, uint8_t val, int is_mt7630);
int      mt0_phy_init(struct mt76x0u_dev *d);

/* --- init.cpp --- */
void     mt0_chip_onoff(struct mt76x0u_dev *d, int enable, int reset);
/* Power-on through PHY init; see the order in init.cpp. 0 on success. */
int      mt0_bring_up(struct mt76x0u_dev *d, const char *fw_dir, int force_fw);

/* --- chan.cpp --- */
/* Pure: the shape for primary channel `primary` at 20 MHz (ht40 == 0) or
 * 40 MHz with the secondary above (+1, HT40+) or below (-1, HT40-).
 * 0 on success; -1 with *err set for anything this backend does not tune
 * (unsupported channel, a 40 MHz pair that does not exist, channel 14 at
 * 40 MHz). 80 MHz is refused before it gets here. */
int mt0_chan_shape_for(uint8_t primary, int ht40, struct mt0_chan_shape *out,
                       const char **err);
/* mt76x0_set_channel(): mt76x0_phy_set_channel + cc_reset + edcca_init.
 * Leaves the first calibration tick due 1 s later (the caller runs it). */
int mt0_set_channel(struct mt76x0u_dev *d, uint8_t primary, int ht40);
/* mt76x0_phy_calibrate(power_on). The TSSI DC step of the power-on arm is
 * not run: it calibrates transmit power, and this backend does not send. */
int mt0_phy_calibrate(struct mt76x0u_dev *d, int power_on);
/* One mt76x0_phy_calibration_work pass (RX gain, then temperature unless
 * the card does TSSI). The kernel runs it 1 s after a tune, then every 4 s. */
int mt0_cal_tick(struct mt76x0u_dev *d);
/* Monitor RX: the kernel's filter for a monitor vif (PROMISC and OTHER_BSS
 * drops cleared; control frames too when accept_control) plus wifit3's
 * measured address-match override. Programs MT_RX_FILTR_CFG only - MAC RX
 * enable is the RX start's job (phase 5). */
void mt0_set_monitor_filter(struct mt76x0u_dev *d, int accept_control);
/* Pure: the filter value mt0_set_monitor_filter() writes. */
uint32_t mt0_monitor_filter_value(uint32_t rxfilter, int accept_control);

/* --- fw.cpp --- */
struct mt0_fw_image {
	const uint8_t *ivb;   /* MT_MCU_IVB_SIZE bytes: the ILM's first 64 */
	const uint8_t *ilm;   /* ILM after the IVB */
	uint32_t ilm_len;     /* hdr ilm_len - MT_MCU_IVB_SIZE */
	const uint8_t *dlm;
	uint32_t dlm_len;
	uint16_t fw_ver, build_ver;
	char build_time[17];
};
/* Validate a firmware blob as mt76x0u_load_firmware() does. 0 on success. */
int mt0_fw_parse(const uint8_t *buf, size_t n, struct mt0_fw_image *img,
                 const char **err);
/* Encode one upload chunk into `scratch` (4-byte header, payload, 4 zero
 * bytes) exactly as __mt76x02u_mcu_fw_send_data(). Returns the bulk length
 * (4 + roundup(len, 4) + 4). */
int mt0_fw_build_chunk(uint8_t *scratch, const uint8_t *src, int len);
/* MT_MCU_COM_REG0 == 1 (mt76x0_firmware_running). -1 on a failed read. */
int mt0_fw_running(struct mt76x0u_dev *d);
/* After the start command: wait up to timeout_us for MT_MCU_COM_REG0 bit 0,
 * riding out EP0 reads that fail while the firmware boots. 0 = running. */
int mt0_fw_wait_running(struct mt76x0u_dev *d, int timeout_us);
/* mt76x0u_load_firmware() with the blob already in memory: enable USB bulk
 * DMA, return early if the firmware is running (unless `force`), otherwise
 * FCE set-up, ILM + DLM upload, IVB, wait for start. Sets mcu_running on
 * success. *uploaded (may be NULL) tells the two successes apart. 0 on
 * success. See the warm-start note in fw.cpp for why `force` exists. */
int mt0_fw_load(struct mt76x0u_dev *d, const uint8_t *blob, size_t n,
                int force, int *uploaded);

/* --- rx.cpp --- */
#define MT_RX_PARSE_INVALID (-2)  /* the rate word's PHY field is invalid */
/* One completed EP 0x84 transfer -> one MPDU. Returns its length (no FCS:
 * the MAC strips it), 0 if unusable, MT_RX_PARSE_INVALID for a bad PHY.
 * *frame points into buf (the L2 pad is folded out in place). Takes no lock
 * and counts nothing: the ring calls it under its own lock. */
int  mt0_rx_parse(struct mt76x0u_dev *d, uint8_t *buf, int n,
                  const uint8_t **frame, struct mt76x0u_rx_info *info);
/* ieee80211_hdrlen() for the L2-pad fold. Pure. */
int  mt0_hdrlen_from_fc(const uint8_t *frame);

/* --- mac.cpp --- */
/* mt76x02u_mac_start. Refuses unless the RX ring is draining EP 0x84. */
int  mt0_mac_start(struct mt76x0u_dev *d);
void mt0_mac_rx_disable(struct mt76x0u_dev *d);
int  mt0_mac_stop(struct mt76x0u_dev *d);

/* --- async.cpp --- */
int  mt0_async_start(struct mt76x0u_dev *d, mt76x0u_rx_cb cb, void *user);
/* 0 when every transfer came back, -1 when the ring had to be leaked. */
int  mt0_async_stop(struct mt76x0u_dev *d);
/* Test hook: runs the RX completion callback on a hand-filled transfer.
 * Exists only for tests/mt76x0u_rx_ring_selftest.cpp; not API. */
void mt0_async_rx_done_for_test(struct libusb_transfer *t);

/* Diagnostics: one line "devourer [X] mt76x0u: ..." on stderr unless a sink
 * is installed (mt76x0u_set_log_sink). */
void mt0_diag(char level, const char *fmt, ...)
#if defined(__GNUC__)
	__attribute__((format(printf, 2, 3)))
#endif
	;
#define LOG(...)   mt0_diag('I', __VA_ARGS__)
#define WARN(...)  mt0_diag('W', __VA_ARGS__)
#define ERR(...)   mt0_diag('E', __VA_ARGS__)

#endif
