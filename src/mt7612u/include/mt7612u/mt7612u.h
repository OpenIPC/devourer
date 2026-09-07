/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * mt7612u-hal - minimal userspace HAL for MediaTek MT7612U over libusb.
 *
 * Descriptor layouts and register sequences are derived from openwrt/mt76
 * (mt76x2/, mt76x02*), Copyright (C) 2016 Felix Fietkau, (C) 2018 Lorenzo
 * Bianconi / Stanislaw Gruszka.
 *
 * Design notes and the evidence behind every constant here: ../INVESTIGATION.md
 */
#ifndef MT7612U_H
#define MT7612U_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MT7612U_VID 0x0e8d
#define MT7612U_PID 0x7612

/* txwi.rate bits 15:13 - the PHY type the frame is sent with. */
enum mt7612u_phy {
	MT7612U_PHY_CCK   = 0,
	MT7612U_PHY_OFDM  = 1,
	MT7612U_PHY_HT    = 2,
	MT7612U_PHY_HT_GF = 3,
	MT7612U_PHY_VHT   = 4,
};

/* txwi.rate bits 8:7. A frame may narrow below the channel width, not exceed it. */
enum mt7612u_bw {
	MT7612U_BW_20 = 0,
	MT7612U_BW_40 = 1,
	MT7612U_BW_80 = 2,
};

/*
 * Per-packet PHY selection. Every field here maps into the 16-bit txwi.rate
 * word plus one bit of txwi.ack_ctl, so all of it is genuinely per frame -
 * nothing is cached in firmware or in a per-station rate table.
 */
struct mt7612u_tx_rate {
	enum mt7612u_phy phy;
	uint8_t  mcs;      /* legacy index, HT MCS 0-31, or VHT MCS 0-9 */
	uint8_t  nss;      /* 1 or 2; folded into the rate index for HT/VHT */
	enum mt7612u_bw bw;
	unsigned sgi  : 1;
	unsigned ldpc : 1;
	unsigned stbc : 1;  /* hardware honours it only at nss == 1 */
	unsigned no_ack : 1;
	int8_t   power_adj; /* txwi.ctl2 MT_TX_PWR_ADJ, 4-bit relative offset */
};

/* Filled from the 32-byte RXWI in front of every received frame. */
struct mt7612u_rx_info {
	enum mt7612u_phy phy;
	uint8_t  mcs;
	uint8_t  nss;
	enum mt7612u_bw bw;
	unsigned sgi  : 1;
	unsigned ldpc : 1;
	unsigned stbc : 1;
	unsigned crc_err : 1;
	unsigned ampdu   : 1;
	int8_t   rssi[4];      /* per chain, already EEPROM-corrected */
	/*
	 * Noise floor in dBm, from RXWI byte 14 - the `rssi[2]` slot that mt76
	 * declares and never reads (mt76x02_mac_process_rx uses only [0] and
	 * [1]). Identified by measurement, not by documentation:
	 *
	 *  - It is signal-independent. On ch36 it read -92, -94, -92, -91, -91
	 *    dBm across five bands whose signal spanned 43 dB.
	 *  - It orders with how noisy the channel is, against the MAC's own
	 *    false-CCA count: ch36 (689 false CCA) -92 dBm, ch1 (28567) -88,
	 *    ch6 (27313) -85, ch11 (17256) -81.
	 *
	 * `snr_db` below is simply rssi[0] - noise.
	 *
	 * CAVEAT, unresolved: on a quiet 5 GHz channel carrying only our own
	 * very strong transmitter this reads a fixed -116 dBm, which is below
	 * the thermal floor of a 20 MHz channel and so cannot be a real
	 * measurement. Treat a reading below about -100 dBm as "no valid
	 * estimate" rather than as an extraordinarily quiet channel; noise_valid
	 * says so.
	 */
	int8_t   noise;
	int8_t   snr_db;       /* rssi[0] - noise; 0 when !noise_valid */
	unsigned noise_valid : 1;
	uint8_t  n_chains;
	uint16_t mpdu_len;
	uint16_t seq;
	/* RXWI bytes 16-31, the `bbp_rxinfo[4]` that mt76 declares in
	 * mt76x02_mac.h and never reads. Raw and unparsed: exposed because
	 * whatever the baseband puts here is the only per-frame quality
	 * signal this MAC offers beyond RSSI, and it cannot be identified
	 * without looking at it. Do not build on the layout until it is. */
	uint32_t bbp[4];
};

struct mt7612u_dev;

/*
 * open: claims the interface, detaching mt76x2u if it holds it, then runs the
 * full power-on, firmware load and MAC/PHY init. fw_dir may be NULL for the
 * system default. Returns NULL on failure; err (optional) receives a message.
 */
struct mt7612u_dev *mt7612u_open(const char *fw_dir, const char **err);

/*
 * Same, but adopting a libusb handle the caller already opened, reset and
 * claimed interface 0 on. Neither the handle nor the context is closed by
 * mt7612u_close() - the caller keeps ownership of both, and of any exclusive
 * lock it took over them.
 *
 * Adopting rather than reopening matters: reopening would race whatever lock
 * the caller holds, and libusb_reset_device() here would invalidate the
 * caller's own handle.
 *
 * `h` is `libusb_device_handle *` and `ctx` is `libusb_context *`, spelled
 * void * so this header does not force libusb on a consumer that only wants
 * the descriptor types above.
 */
struct mt7612u_dev *mt7612u_open_handle(void *h, void *ctx, const char *fw_dir,
                                        const char **err);

void mt7612u_close(struct mt7612u_dev *dev);

/* Reattaches the kernel driver on close unless this is set. */
void mt7612u_keep_detached(struct mt7612u_dev *dev, int keep);

/*
 * Channel + width. `chan` is always the *control* channel, at every width,
 * and is an 802.11 channel number rather than a frequency. Issues
 * CMD_SWITCH_CHANNEL_OP and the firmware calibration burst, so it is not
 * cheap - it is a setup call, not a per-frame one.
 *
 * 20, 40 and 80 MHz are implemented. The hardware tunes the *centre* of a
 * widened channel, derived here from the control channel: at 80 MHz, channel
 * 36 tunes centre 42. A control channel that cannot carry the requested
 * width is refused rather than tuned near - that covers an off-grid channel,
 * one whose widened span would leave the band `mt7612u_caps` declares, and
 * 40 MHz in 2.4 GHz outside channels 4-11, where a bare channel number
 * cannot say which side the secondary sits on.
 */
int mt7612u_set_channel(struct mt7612u_dev *dev, uint8_t chan, enum mt7612u_bw bw);

/* Absolute TX power base, dBm. Per-frame trim is mt7612u_tx_rate.power_adj. */
int mt7612u_set_txpower(struct mt7612u_dev *dev, int dbm);

/* 0x202 = 2T2R (default), 0x101 = 1T1R. Global; takes effect at next channel set. */
int mt7612u_set_chainmask(struct mt7612u_dev *dev, uint16_t chainmask);

/*
 * Enable the MAC. TX always; RX only if mt7612u_rx_start() is already
 * running. That condition is not a convenience: with the receiver on and
 * nothing draining the bulk-IN endpoint, this part wedges below the USB
 * level and no software reset recovers it - only a physical replug.
 */
int mt7612u_start(struct mt7612u_dev *dev);
int mt7612u_stop(struct mt7612u_dev *dev);

/*
 * Inject one complete 802.11 frame (no FCS - the MAC appends it). Builds the
 * TXWI + TXINFO and submits a single bulk transfer. Fire-and-forget.
 */
int mt7612u_tx(struct mt7612u_dev *dev, const void *frame, size_t len,
               const struct mt7612u_tx_rate *rate);

/*
 * RX callback, invoked from the libusb event thread. `frame`/`len` are the
 * 802.11 MPDU with the RXWI removed and **without the trailing FCS** - the
 * MAC strips it, and the four bytes that follow the MPDU in the DMA buffer
 * are the FCE info trailer, not a checksum (measured: CRC-32 matched them on
 * 0 of 4263 frames). Must not block and must not call back into the device.
 */
typedef void (*mt7612u_rx_cb)(void *user, const void *frame, size_t len,
                              const struct mt7612u_rx_info *info);
int mt7612u_rx_start(struct mt7612u_dev *dev, mt7612u_rx_cb cb, void *user);
int mt7612u_rx_stop(struct mt7612u_dev *dev);

/*
 * Put the receive filter into monitor mode: pass everything the PHY decodes,
 * dropping only PHY errors and (unless keep_corrupted) frames that failed FCS.
 *
 * This is NOT the default. mt7612u_start() leaves the filter mt76's managed
 * -mode value, which drops control frames and anything not addressed to this
 * station - on ambient 2.4 GHz traffic that is the difference between seeing
 * the whole mix and seeing almost nothing but beacons. Call this after
 * mt7612u_start(), which rewrites the register.
 */
int mt7612u_set_monitor_rx(struct mt7612u_dev *dev, int keep_corrupted);

/*
 * Radiotap-framed inject, matching devourer's send_packet() contract: one
 * buffer holding a radiotap header followed by the 802.11 MPDU, with the
 * per-frame rate carried in the header.
 */
int mt7612u_send_packet(struct mt7612u_dev *dev, const void *buf, size_t len);

/* One radiotap-framed MPDU, as handed to mt7612u_send_packets(). */
struct mt7612u_tx_view { const uint8_t *data; size_t len; };

/*
 * Submit several frames in one call. MT7612U chains them into a single
 * bulk-OUT transfer via the TXDMA's "next valid" bit, so a burst costs one USB
 * transaction instead of one per frame. Returns the number accepted.
 */
size_t mt7612u_send_packets(struct mt7612u_dev *dev,
                            const struct mt7612u_tx_view *pkts, size_t count);

/*
 * Hardware ACK responder: make the MAC answer frames addressed to `mac` with
 * a SIFS-timed ACK, with no host involvement. `mac` must be unicast.
 * Returns 0 on success, negative when unsupported or the arm cannot be
 * verified. Clear is best effort.
 */
int  mt7612u_set_ack_responder(struct mt7612u_dev *dev, const uint8_t mac[6]);
void mt7612u_clear_ack_responder(struct mt7612u_dev *dev);

/*
 * TX/RX counters from the async rings. Zeroed when no ring is running, and
 * taken under the ring's own lock - reading the fields directly would race
 * the libusb event thread.
 */
struct mt7612u_stats {
	uint64_t tx_submitted, tx_done, tx_err, rx_frames, rx_err;
	/* Frames dropped because the rate word named no valid PHY. MT_RATE_PHY
	 * is three bits, so 5-7 are representable and mean nothing; mt76 drops
	 * them too. Nonzero here means the RX path is seeing garbage, not that
	 * the radio is slow. */
	uint64_t rx_invalid;
	/* Frames the parser rejected on length - a short or malformed
	 * transfer. Counted because such a frame used to move no counter at
	 * all, which is indistinguishable from one that was never sent.
	 *
	 * It does NOT count an oversize frame: those are discarded by the MAC
	 * above max_mpdu_rx, before USB, so they raise nothing here. That
	 * limit is a capability to read, not an error to count. */
	uint64_t rx_dropped;
};
void mt7612u_get_stats(struct mt7612u_dev *dev, struct mt7612u_stats *out);

/*
 * Per-interval link statistics from the MAC's MIB counters.
 *
 * These are the rich part of this MAC's link reporting, and none of it is per
 * frame: the RX descriptor carries RSSI and nothing else - its
 * `bbp_rxinfo[4]`, which mt76 declares and never reads, turns out to be two
 * words of zero plus a duplicate of the same two RSSI values, so there is no
 * per-frame SNR or EVM to surface.
 *
 * Every counter below is READ-AND-CLEAR in hardware. Each call therefore
 * returns the interval since the previous call, not a running total, and two
 * readers would steal each other's counts. mt7612u_link_stats_start() enables
 * the channel timers and zeroes everything; call it once, then poll.
 */
struct mt7612u_link_stats {
	uint32_t interval_us;    /* host-measured, for turning counts into rates */

	/* Channel occupancy, in the MAC's own clock units. busy counts TX, RX,
	 * NAV and EIFS as busy, which is the definition mt76 configures. */
	uint32_t ch_busy, ch_idle;

	/* Receive error classes. false_cca is the interference signal: energy
	 * that started a receive and did not become a frame. */
	uint16_t rx_crc_err, rx_phy_err, rx_false_cca, rx_plcp_err;
	uint16_t rx_dup_err, rx_overflow;

	/* A-MPDU length histogram: agg_cnt[n] is the number of aggregates that
	 * carried n+1 MPDUs, up to 32. */
	uint16_t agg_cnt[32];

	int8_t   temp_c;         /* die temperature, degrees C; INT8_MIN if unread */
};

/*
 * Enable the channel timers and clear every counter. Safe to call again to
 * re-zero. Returns 0 on success.
 */
int mt7612u_link_stats_start(struct mt7612u_dev *dev);

/* Read and clear. Returns 0 on success; fills the interval since the previous
 * call to this function or to _start(). */
int mt7612u_link_stats(struct mt7612u_dev *dev, struct mt7612u_link_stats *out);

/* TSF, the hardware microsecond clock. Two register reads. */
uint64_t mt7612u_read_tsf(struct mt7612u_dev *dev);
void     mt7612u_write_tsf(struct mt7612u_dev *dev, uint64_t tsf);

/* What this adapter can do, so a caller need not assume. */
struct mt7612u_caps {
	const char *chip_name;
	uint32_t rev;
	uint8_t  nss_rx, nss_tx;
	uint8_t  bw_mask;          /* bit0 = 20, bit1 = 40, bit2 = 80 MHz */
	/*
	 * Largest MPDU, excluding FCS, in each direction. They differ, and the
	 * asymmetry is real rather than an oversight:
	 *
	 *   max_mpdu_tx  what this backend will submit. A buffer choice, not a
	 *                silicon limit - measured on air, the part transmits at
	 *                least 7900 B and an RTL8812AU decoded 60/60 at every
	 *                size up to that with zero CRC errors, far past the
	 *                802.11 non-A-MSDU ceiling of 2304.
	 *   max_mpdu_rx  what the MAC will hand back. Set by MT_MAX_LEN_CFG,
	 *                whose low 12 bits are the on-air length INCLUDING the
	 *                4-byte FCS; at the 0xf00 this driver programs that is
	 *                3840 on air, so 3836 of MPDU. Measured to the byte:
	 *                3836 arrives, 3837 does not.
	 *
	 * A frame above max_mpdu_rx is discarded by the MAC before USB. Nothing
	 * counts it - not rx_err, not rx_dropped - so a caller that needs to
	 * know must read this rather than watch for an error.
	 */
	uint16_t max_mpdu_tx, max_mpdu_rx;
	uint16_t band_5g_min_mhz, band_5g_max_mhz;
	uint16_t band_2g_min_mhz, band_2g_max_mhz;
	unsigned ampdu_tx : 1;     /* aggregation works on injected frames */
	unsigned per_chain_rssi : 1;
	unsigned narrowband : 1;   /* 5/10 MHz - not available on this part */
	unsigned fast_retune : 1;  /* sub-ms channel change - not on this part */
};
void mt7612u_get_caps(const struct mt7612u_dev *dev, struct mt7612u_caps *caps);

/* Identity, for logging and for refusing to run on an unexpected revision. */
uint32_t mt7612u_asic_version(const struct mt7612u_dev *dev); /* 0x76120044 here */
const uint8_t *mt7612u_mac_addr(const struct mt7612u_dev *dev);

#ifdef __cplusplus
}
#endif
#endif /* MT7612U_H */
