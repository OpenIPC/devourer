/* SPDX-License-Identifier: BSD-3-Clause-Clear */
#ifndef MT7612U_INTERNAL_H
#define MT7612U_INTERNAL_H

/* The rest of this project includes <libusb.h> and lets the build system
 * supply the include directory; the standalone Makefile here has no
 * pkg-config step, and most distributions install the header under
 * libusb-1.0/. Try the project's spelling first, fall back to the
 * distribution path. */
#if defined(__has_include)
#  if __has_include(<libusb.h>)
#    include <libusb.h>
#  else
#    include <libusb-1.0/libusb.h>
#  endif
#else
#  include <libusb-1.0/libusb.h>
#endif
#include <pthread.h>
#include <time.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include "regs.h"
#include "include/mt7612u/mt7612u.h"

/* Per-rate TX power, 0.5 dB units, exactly mt76x02_rate_power's layout. */
struct mt_rate_power {
	union {
		struct { int8_t cck[4], ofdm[8], ht[16], vht[2]; };
		int8_t all[30];
	};
};

struct mt_tx_power_info {
	uint8_t target_power;
	int8_t  delta_bw40, delta_bw80;
	struct { int8_t tssi_slope, tssi_offset, target_power, delta; } chain[2];
};

/* EEPROM-derived values the host computes with (firmware does the rest). */
struct mt7612u_cal {
	int8_t  rssi_offset[2];
	int8_t  lna_gain;
	int8_t  high_gain[2];
	uint8_t init_cal_done;
	uint8_t channel_cal_done;
	uint32_t mcu_gain;
	uint8_t  agc_gain_init[2];
	uint8_t  tssi_cal_done;
};

#define MT_RX_RING  16
/* 16 slots, not 32: the slots now carry a full aggregate, so this is the
 * difference between 256 KB and 512 KB of ring. Depth is not what buys
 * throughput here - see "Async rings bought no throughput" in
 * docs/mt7612u.md - and soak/ampdu were re-measured after the change. */
#define MT_TX_RING  16
#define MT_RX_BUFSZ 4096
#define MT_USB_AGG_BUF  16384   /* one aggregated bulk-OUT transfer */

/*
 * ONE ceiling for everything that reaches the async ring.
 *
 * This was 2048 while mt_tx_raw() built up to MT_TX_BUF_MAX (4096) and
 * mt7612u_send_packets() up to MT_USB_AGG_BUF (16384), and both route through
 * the ring whenever an RX loop is running - which is the normal integrated
 * shape. So a frame over ~2 KB was refused after being built, and every
 * multi-frame batch reported zero accepted. The sync path has no ring, which
 * is exactly why the mtu sweep never saw it.
 *
 * Sized to the aggregate buffer and checked at compile time below, so a future
 * change to either builder cannot silently reintroduce the mismatch.
 */
#define MT_TX_BUFSZ MT_USB_AGG_BUF

/*
 * One page, matching MT_RX_BUFSZ. The old 2048 silently capped a single frame
 * at 2016 B while mt7612u_send_packets() bounded only against the 16 KB
 * aggregate buffer, so the two public TX entry points disagreed about the
 * largest frame this backend accepts.
 *
 * 4096 is a deliberate choice, not the hardware's limit: measured on air, this
 * part transmits at least 7900 B and an RTL8812AU decoded 60/60 at every size
 * up to that with zero CRC errors - far past 802.11's 2304 non-A-MSDU MPDU
 * ceiling. It is capped here to match what this backend can RECEIVE, because
 * MT_RX_BUFSZ is one page and a frame larger than that is dropped without a
 * counter moving. Raising this means raising the RX buffer too.
 */
#define MT_TX_BUF_MAX 4096

/* Neither TX builder may outgrow the ring. A mismatch here is what made every
 * aggregate fail silently once an RX loop was up, so it is a build error now
 * rather than a runtime refusal. */
typedef char mt_tx_ceiling_covers_single_frame[MT_TX_BUF_MAX <= MT_TX_BUFSZ ? 1 : -1];
typedef char mt_tx_ceiling_covers_aggregate[MT_USB_AGG_BUF <= MT_TX_BUFSZ ? 1 : -1];
#define MT_USB_AGG_MAX  32      /* frames chained per transfer */

struct mt7612u_dev;
struct mt_async;
/* Each slot names its own ring, not dev->a: if a teardown has to leak a ring
 * whose transfers are still in flight, their completions must keep touching
 * the leaked ring and never a replacement one. */
struct mt_slot { struct mt7612u_dev *d; struct mt_async *a; int idx; };

struct mt_async {
	struct libusb_transfer *rx[MT_RX_RING], *tx[MT_TX_RING];
	struct mt_slot rx_slot[MT_RX_RING], tx_slot[MT_TX_RING];
	uint8_t rx_buf[MT_RX_RING][MT_RX_BUFSZ];
	uint8_t tx_buf[MT_TX_RING][MT_TX_BUFSZ];
	int     tx_busy[MT_TX_RING];
	/* Guards running, rx_active, tx_busy[], tx_inflight and rx_inflight -
	 * all of which the event thread writes and the caller reads. */
	pthread_mutex_t lock;
	pthread_cond_t  cv;
	pthread_t evt;
	int evt_started;
	int running, rx_active;
	int tx_inflight, rx_inflight;
	mt7612u_rx_cb cb;
	void *cb_user;
	uint64_t tx_submitted, tx_done_n, tx_err, rx_frames, rx_err, rx_invalid;
	uint64_t rx_dropped;      /* rejected on length: truncated, or > MT_RX_BUFSZ */
};

struct mt7612u_dev {
	libusb_context       *ctx;
	libusb_device_handle *h;
	int      kernel_was_attached;
	int      keep_detached;
	/* 0 when the handle and context were handed in by a caller that keeps
	 * ownership of them - mt_close() must then release the interface but
	 * neither close the handle nor exit the context. */
	int      owns_handle;

	uint32_t rev;             /* MT_ASIC_VERSION, e.g. 0x76120044 */
	uint8_t  eeprom[MT7612U_EEPROM_SIZE];
	uint8_t  macaddr[6];
	uint16_t chainmask;       /* 0x202 = 2T2R */
	uint8_t  mcu_seq;
	uint8_t  chan;
	uint8_t  bw;
	uint8_t  bw_clamp_warned;   /* the "never widen" notice is once, not per frame */
	int8_t   txpower_conf;      /* limit, 0.5 dB units (dBm * 2) */
	int8_t   target_power;
	int8_t   target_power_delta[2];
	int      enable_tpc;        /* per-packet TX_PWR_ADJ; mt76 defaults it off */
	struct mt_rate_power rate_power;
	struct mt7612u_cal cal;

	unsigned io_err;          /* EP0 transfers that exhausted their retries */
	int      transfers_stranded; /* libusb still owns a cancelled ring */
	uint16_t max_mpdu_rx;     /* from MT_MAX_LEN_CFG at init, less the FCS */
	uint64_t stats_last_us;   /* previous mt7612u_link_stats() mark */

	/* Oracle-diff log: every EP0 write we emit, in order. */
	uint8_t  ack_saved_mac[6];
	int      ack_saved;
	struct mt_async *a;
	FILE    *wrlog;
	FILE    *mculog;
};

/* --- usb.c --- */
int      mt_open(struct mt7612u_dev *d, const char **err);
/* Adopt a handle the caller already opened, reset and claimed. */
int      mt_adopt(struct mt7612u_dev *d, libusb_device_handle *h,
                  libusb_context *ctx, const char **err);
void     mt_close(struct mt7612u_dev *d);
/* Checked read: 0 on success with *val filled, -1 on transport failure.
 * Prefer this anywhere the value drives a decision - 0xffffffff is a real
 * register value here and cannot serve as an error sentinel. */
int      mt_rr_chk(struct mt7612u_dev *d, uint32_t addr, uint32_t *val);
uint32_t mt_rr(struct mt7612u_dev *d, uint32_t addr);
void     mt_wr(struct mt7612u_dev *d, uint32_t addr, uint32_t val);
/* Returns -1 without writing when the read half fails. */
int      mt_rmw(struct mt7612u_dev *d, uint32_t addr, uint32_t mask, uint32_t val);
int      mt_wr_chk(struct mt7612u_dev *d, uint32_t addr, uint32_t val);
/* Register-I/O failure accumulator; see the comment above mt_io_clear(). */
void     mt_io_clear(struct mt7612u_dev *d);
unsigned mt_io_errors(struct mt7612u_dev *d);
#define  mt_set(d, a, v)   mt_rmw(d, a, v, v)
#define  mt_clear(d, a, v) mt_rmw(d, a, v, 0)
/* Poll until (rr(addr) & mask) == val. Returns 1 on success, 0 on timeout. */
int      mt_poll(struct mt7612u_dev *d, uint32_t addr, uint32_t mask,
                 uint32_t val, int timeout_us);
int      mt_vendor_req(struct mt7612u_dev *d, uint8_t req, uint8_t type,
                       uint16_t val, uint16_t idx, void *buf, size_t len);
/* Two 16-bit halves, as mt76u_single_wr(). Used for the FCE DMA regs. */
void     mt_wr_copy(struct mt7612u_dev *d, uint32_t offset, const void *data, int len);
void     mt_single_wr(struct mt7612u_dev *d, uint8_t req, uint16_t off, uint32_t val);
int      mt_bulk(struct mt7612u_dev *d, uint8_t ep, void *buf, int len,
                 int *xfered, unsigned timeout_ms);
int      mt_wait_for_mac(struct mt7612u_dev *d);
void     mt_usleep(unsigned us);

/* --- mcu.c --- */
int mt_mcu_send(struct mt7612u_dev *d, int cmd, const void *data, int len, int wait_resp);
int mt_mcu_function_select(struct mt7612u_dev *d, int func, uint32_t val);
int mt_mcu_set_radio_state(struct mt7612u_dev *d, int on);
int mt_mcu_calibrate(struct mt7612u_dev *d, int type, uint32_t param);
int mt_mcu_load_cr(struct mt7612u_dev *d, uint8_t type, uint8_t temp, uint8_t ch);
int mt_mcu_set_channel(struct mt7612u_dev *d, uint8_t ch, uint8_t bw, uint8_t bw_index, int scan);
int mt_mcu_init_gain(struct mt7612u_dev *d, uint8_t ch, uint32_t gain, int force);

/* --- fw.c --- */
int mt_fw_init(struct mt7612u_dev *d, const char *fw_dir);

/* --- eeprom.c --- */
void mt_get_rate_power(struct mt7612u_dev *d, struct mt_rate_power *t, int band);
void mt_get_power_info(struct mt7612u_dev *d, struct mt_tx_power_info *t,
                       uint8_t chan, int band);
void mt_read_rx_gain(struct mt7612u_dev *d, uint8_t chan, int band);
int mt_eeprom_init(struct mt7612u_dev *d);
uint16_t mt_ee(const struct mt7612u_dev *d, unsigned off);

/* --- init.c --- */
void mt_power_cycle(struct mt7612u_dev *d);
int mt_init_hardware(struct mt7612u_dev *d, const char *fw_dir);
int mt_mac_start(struct mt7612u_dev *d, int enable_rx);
void mt_rx_flush(struct mt7612u_dev *d);
int mt_mac_stop(struct mt7612u_dev *d);

/* --- tx.c --- */
uint16_t mt_tx_rate_word(const struct mt7612u_tx_rate *r);
/* ieee80211_hdrlen(), ported. Shared by TX (where to insert the L2 pad) and
 * RX (how many bytes to move when folding it back out) - they must agree. */
int mt_hdrlen_from_fc(const uint8_t *frame);
#define MT_TXOPT_RATE_LUT  0x01  /* set MT_TXWI_FLAGS_TX_RATE_LUT */
#define MT_TXOPT_AMPDU     0x02  /* AMPDU flag + density + BA window */
#define MT_TXOPT_QSEL_MGMT 0x04  /* mt76 uses MT_QSEL_MGMT for aggregated TX */
int mt_tx_build(struct mt7612u_dev *d, uint8_t *buf, size_t bufsz,
                const void *frame, size_t len,
                const struct mt7612u_tx_rate *rate, uint8_t wcid, unsigned opts,
                int next_vld, int trailer);
int mt_tx_raw(struct mt7612u_dev *d, const void *frame, size_t len,
              const struct mt7612u_tx_rate *rate, uint8_t wcid, unsigned opts);
void mt_wcid_setup(struct mt7612u_dev *d, uint8_t idx, const uint8_t *mac);

/* --- radiotap.c --- */
int mt_radiotap_parse(const uint8_t *buf, size_t len, struct mt7612u_tx_rate *r);

/* --- async.c --- */
struct mt_async_stats {
	uint64_t tx_submitted, tx_done, tx_err, rx_frames, rx_err, rx_invalid;
	uint64_t rx_dropped;
};
void mt_async_stats(struct mt7612u_dev *d, struct mt_async_stats *out);
void mt_async_note_invalid(struct mt7612u_dev *d);
int  mt_async_start(struct mt7612u_dev *d, mt7612u_rx_cb cb, void *user);
void mt_async_stop(struct mt7612u_dev *d);
int  mt_async_tx_submit(struct mt7612u_dev *d, const uint8_t *buf, int len);

/* --- rx.c --- */
int mt_rx_parse(struct mt7612u_dev *d, uint8_t *buf, int n,
                const uint8_t **frame, struct mt7612u_rx_info *info);
int mt_rx_one(struct mt7612u_dev *d, uint8_t *buf, int bufsize,
              const uint8_t **frame, struct mt7612u_rx_info *info,
              unsigned timeout_ms);

/* --- phy.c / chan.c --- */
void mt_phy_set_rxpath(struct mt7612u_dev *d);
void mt_phy_set_txpower(struct mt7612u_dev *d, int band);
int  mt_tssi_enabled(struct mt7612u_dev *d);
int8_t mt_tx_get_max_txpwr_adj(struct mt7612u_dev *d,
                               const struct mt7612u_tx_rate *r);
int8_t mt_tx_get_txpwr_adj(struct mt7612u_dev *d, int8_t txpwr, int8_t max_adj);
void mt_phy_set_txdac(struct mt7612u_dev *d);
int  mt_set_channel(struct mt7612u_dev *d, uint8_t chan, uint8_t bw);
int  mt_set_channel_ex(struct mt7612u_dev *d, uint8_t chan, uint8_t bw, int fast);
int  mt_chan_group(uint8_t chan, uint8_t bw, uint8_t *hw_chan,
                   uint8_t *bw_index, uint8_t *ch_group);

/*
 * The diagnostic plane, per docs/logging.md: stderr, one line of
 * `devourer [X] message` with X in T/D/I/W/E.
 *
 * The old macros broke that contract three ways. They used a private
 * `[mt7612u]` prefix rather than the documented one, so a consumer filtering
 * on level saw nothing; they emitted the text and its newline as two separate
 * stdio calls, so a line written from the libusb event thread could be split
 * by one from a caller; and they never flushed, so a piped reader could sit on
 * a full buffer while the device was mid-bring-up. mt_diag() formats the whole
 * line first and emits it with a single fwrite + fflush, which is what
 * src/Event.h does for the machine plane and for the same reason.
 */
void mt_diag(char level, const char *fmt, ...)
#if defined(__GNUC__)
	__attribute__((format(printf, 2, 3)))
#endif
	;
#define LOG(...)   mt_diag('I', __VA_ARGS__)
#define WARN(...)  mt_diag('W', __VA_ARGS__)
#define ERR(...)   mt_diag('E', __VA_ARGS__)

#endif
