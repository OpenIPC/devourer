/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * mt76x0u - userspace HAL for the MediaTek MT76x0U (MT7610U / MT7630U /
 * MT7650U) over libusb.
 *
 * Register sequences are derived from openwrt/mt76 (mt76x0/, mt76x02*,
 * usb.c), Copyright (C) 2016 Felix Fietkau, (C) 2018 Lorenzo Bianconi /
 * Stanislaw Gruszka, pinned as reference/mt76 @ be5ce79.
 *
 * STATUS: phase 5 of the port. What exists is the transport (register I/O
 * over EP0 vendor requests), chip identification, the in-band MCU channel
 * with register-pair commands, firmware upload, the power-on sequence, EFUSE
 * calibration data, MAC/BBP/RF init (mt76x0u_bring_up), the 20/40 MHz
 * channel set with its calibration, the periodic calibration pass, the
 * monitor RX filter, the EP 0x84 RX ring with RXWI parsing, and MAC
 * start/stop (phase 5). NONE of it has run on hardware.
 */
#ifndef MT76X0U_H
#define MT76X0U_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MT76X0U_VID 0x0e8d
#define MT76X0U_PID 0x7610

struct mt76x0u_dev;

/*
 * Adopt a libusb handle the caller already opened, reset and claimed interface
 * 0 on. The caller keeps ownership of the handle, the context and any
 * exclusive lock: mt76x0u_close() releases neither. No reset here - it would
 * invalidate the caller's handle.
 *
 * Identifies the chip (MT_ASIC_VERSION >> 16 must be 0x7610, 0x7630 or 0x7650,
 * mt76x02.h is_mt76x0) and returns NULL with *err set otherwise.
 *
 * `no_2ghz` is the kernel table's driver_info quirk (TP-Link Archer T1U); the
 * caller derives it from the USB id (mt76x0u::usb_id_has_no_2ghz).
 *
 * `h` is `libusb_device_handle *` and `ctx` is `libusb_context *`, spelled
 * void * so this header does not force libusb on a consumer, as in mt7612u.h.
 */
struct mt76x0u_dev *mt76x0u_open_handle(void *h, void *ctx, int no_2ghz,
                                        const char **err);
/* Stops the RX ring if one is running, then frees the device. Safe on NULL. */
void mt76x0u_close(struct mt76x0u_dev *dev);

/* MT_ASIC_VERSION as read at open, e.g. 0x76100044. 0 on NULL. */
uint32_t mt76x0u_asic_version(const struct mt76x0u_dev *dev);
/* MT_MAC_CSR0 as read at open (wifit3 records 0x76502000 on an MT7610U). */
uint32_t mt76x0u_mac_version(const struct mt76x0u_dev *dev);
/* The 2.4 GHz + Bluetooth combo strap: mt76_chip() == 0x7630 (mt76x0.h). */
int mt76x0u_is_mt7630(const struct mt76x0u_dev *dev);
/* The USB id's no_2ghz quirk, as passed to mt76x0u_open_handle(). */
int mt76x0u_no_2ghz(const struct mt76x0u_dev *dev);

/*
 * Firmware (mt76x0u_load_firmware in mt76/mt76x0/usb_mcu.c).
 *
 * If MT_MCU_COM_REG0 already reads 1, the running firmware is kept and
 * nothing is uploaded - the kernel's warm path; *uploaded is then 0. `force`
 * no longer re-uploads onto running firmware: that is refused with a warning,
 * because on the bench it jammed the chip's RX DMA (src/mt76x0u/fw.cpp).
 * Otherwise `fw_dir`/mt7610e.bin is loaded, falling back to
 * `fw_dir`/mt7610u.bin as the kernel does, and *uploaded is 1. `fw_dir` NULL
 * means "./firmware".
 *
 * Precondition, not checked here: the chip is powered and its MAC answers.
 * That is the caller's power-on sequence (bring-up, a later phase), which the
 * kernel runs before this in mt76x0u_init_hardware.
 *
 * 0 on success (the MCU then answers commands), -1 on failure.
 */
int mt76x0u_load_firmware(struct mt76x0u_dev *dev, const char *fw_dir,
                          int force, int *uploaded);
/* MT_MCU_COM_REG0 == 1. 1 / 0, or -1 when the register cannot be read. */
int mt76x0u_firmware_running(struct mt76x0u_dev *dev);
/* Whether this library considers the MCU up (loaded or adopted). */
int mt76x0u_mcu_running(const struct mt76x0u_dev *dev);

/*
 * Bring the chip up as mt76x0u_probe() + mt76x0u_init_hardware() do, plus one
 * step of our own: on a warm start (firmware kept), right after the firmware
 * step, drain EP 0x84 if RX_BUSY and pulse the DMA/PBF reset, which clears a
 * jammed receive DMA (bench-found; see init.cpp). Order: chip off, chip on with
 * reset, firmware (mt76x0u_load_firmware, with `force_fw`), the warm reset,
 * USB DMA, MAC/BBP tables, key and WCID clear, EFUSE, PHY init. MAC RX stays
 * off. 0 on success, -1 on failure (the log says which step).
 */
int mt76x0u_bring_up(struct mt76x0u_dev *dev, const char *fw_dir, int force_fw);

/* After bring-up: the 512-byte EFUSE image (returns 512, or -1), the MAC
 * address (NULL before), and the bands the EFUSE, the USB-id quirk and the
 * MT7630 strap leave (0 on success). */
int mt76x0u_eeprom(const struct mt76x0u_dev *dev, uint8_t *out, size_t len);
const uint8_t *mt76x0u_mac_addr(const struct mt76x0u_dev *dev);
int mt76x0u_bands(const struct mt76x0u_dev *dev, int *has_2ghz, int *has_5ghz);

/*
 * Tune (mt76x0_set_channel): `primary` is the control channel, `ht40` 0 for
 * 20 MHz, +1 for 40 MHz with the secondary above (HT40+), -1 below (HT40-).
 * 80 MHz is not offered. Runs the channel calibration. 0 on success.
 */
int mt76x0u_set_channel(struct mt76x0u_dev *dev, uint8_t primary, int ht40);
/* By default mt76x0u_set_channel() runs the channel set twice, as the kernel
 * does on the wire (and as wifit3 found the hardware needs - see chan.cpp).
 * 0 here makes it a single pass, e.g. for a transfer-log comparison. */
void mt76x0u_set_tune_twice(struct mt76x0u_dev *dev, int twice);
/*
 * One pass of the kernel's periodic calibration (RX gain tracking, then the
 * temperature-triggered recalibration unless the card does TSSI). The kernel
 * runs it 1 s after a tune and every 4 s after that; the caller owns the
 * timing and must not overlap it with other calls on the same device.
 */
int mt76x0u_cal_tick(struct mt76x0u_dev *dev);
/*
 * Program the monitor RX filter (kernel monitor semantics; control frames
 * kept when `accept_control`) and wifit3's address-match override. Does not
 * enable MAC RX. 0 on success.
 */
int mt76x0u_set_monitor_filter(struct mt76x0u_dev *dev, int accept_control);

/*
 * Diagnostics sink. Default: stderr, one line "devourer [X] mt76x0u: msg".
 * Process-global; set it before opening a device or starting a ring - it is
 * read from the libusb event thread without synchronisation (same contract as
 * mt7612u_set_log_sink).
 */
typedef void (*mt76x0u_log_sink)(void *user, char level, const char *line);
void mt76x0u_set_log_sink(mt76x0u_log_sink sink, void *user);

/* rate word bits 15:13 (mt76x02 MT_RATE_PHY) */
enum mt76x0u_phy {
	MT76X0U_PHY_CCK   = 0,
	MT76X0U_PHY_OFDM  = 1,
	MT76X0U_PHY_HT    = 2,
	MT76X0U_PHY_HT_GF = 3,
	MT76X0U_PHY_VHT   = 4,
};
enum mt76x0u_bw { MT76X0U_BW_20 = 0, MT76X0U_BW_40 = 1, MT76X0U_BW_80 = 2 };

/* Parsed from the 32-byte RXWI in front of every received frame. */
struct mt76x0u_rx_info {
	enum mt76x0u_phy phy;
	uint8_t  mcs;          /* legacy index, HT MCS, or VHT MCS */
	uint8_t  nss;
	enum mt76x0u_bw bw;
	unsigned sgi  : 1;
	unsigned ldpc : 1;
	unsigned stbc : 1;
	unsigned crc_err : 1;
	unsigned ampdu   : 1;
	/* Chain 0 (the only chain: 1T1R), dBm, corrected as
	 * mt76x02_mac_get_rssi: raw + EFUSE rssi_offset[0] - per-channel LNA
	 * gain. Whether it matches the kernel's reading is a bench question. */
	int8_t   rssi_dbm;
	int8_t   rssi_raw;     /* RXWI rssi[0] as read */
	uint16_t mpdu_len;
	uint16_t seq;
};

/*
 * RX callback, on the libusb event thread. `frame`/`len` are one 802.11 MPDU
 * with the L2 pad folded out and WITHOUT an FCS: this MAC strips it (the
 * MT7612U, same MAC family, was measured: the 4 bytes after the MPDU are an
 * FCE trailer, not a CRC). Valid only for the call. Must not block: the ring
 * resubmits after it returns.
 */
typedef void (*mt76x0u_rx_cb)(void *user, const void *frame, size_t len,
                              const struct mt76x0u_rx_info *info);
/* Start the EP 0x84 ring. Start it BEFORE mt76x0u_start(): a receiver with
 * nothing draining its endpoint wedges the MT7612U below the USB level, and
 * this family shares the DMA design. 0 on success (also if running). */
int mt76x0u_rx_start(struct mt76x0u_dev *dev, mt76x0u_rx_cb cb, void *user);
/* Cancels and reaps the ring. 0 when every transfer came back, -1 when some
 * were stranded (the device is then unusable and leaks its USB objects
 * rather than freeing memory libusb still owns). Call mt76x0u_rx_quiesce()
 * first when the MAC receiver is on. */
int mt76x0u_rx_stop(struct mt76x0u_dev *dev);

/*
 * mt76x0u_start(): mt76x02u_mac_start (MAC TX + RX on, as the kernel - the
 * TX engine idles with nothing queued), then the monitor RX filter (after the
 * start, which writes the init filter back), then
 * mt76x0_phy_calibrate(power_on). REFUSES unless the RX ring is running.
 * A channel must have been set. 0 on success.
 */
int mt76x0u_start(struct mt76x0u_dev *dev, int accept_control);
/* MAC RX off, TX untouched. Before mt76x0u_rx_stop(), so the receiver never
 * runs with nothing draining it. */
int mt76x0u_rx_quiesce(struct mt76x0u_dev *dev);
/* mt76x0_mac_stop: wait for TX queues, MAC TX/RX off, wait for idle. */
int mt76x0u_stop(struct mt76x0u_dev *dev);

struct mt76x0u_stats {
	uint64_t rx_frames;      /* parsed and delivered */
	uint64_t rx_dropped;     /* completed but unusable (short/malformed) */
	uint64_t rx_invalid;     /* rate word named no valid PHY (also dropped) */
	uint64_t rx_err;         /* failed (not cancelled) RX transfers */
	unsigned io_errors;      /* EP0 register accesses that exhausted retries */
	int rx_no_device;        /* an RX transfer reported the device gone */
	int rx_ring_dead;        /* ring started but no transfer left in flight */
};
void mt76x0u_get_stats(struct mt76x0u_dev *dev, struct mt76x0u_stats *out);

#ifdef __cplusplus
}
#endif

#endif /* MT76X0U_H */
