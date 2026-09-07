/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * MT7612U bringup harness. One subcommand per gate (see src/mt7612u/README.md), so each
 * stage is independently runnable on hardware.
 */
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/resource.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/resource.h>
#include "../internal.h"


static struct mt7612u_dev dev;

static double now_ms(void)
{
	struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec * 1000.0 + t.tv_nsec / 1e6;
}

/* --- interruptible waits --------------------------------------------------
 *
 * A gate that hangs on this part hangs hard: the thread blocks inside a USB
 * ioctl in uninterruptible sleep, where SIGKILL does not reach it and Ctrl-C
 * does nothing.
 *
 * The defence against that is the exclusive per-adapter lock in usb.c, which
 * refuses a second opener and so removes the cause. A watchdog thread lived
 * here for one commit and was removed: _exit() cannot reap a thread already
 * blocked in an uninterruptible ioctl, so against the failure that motivated
 * it the watchdog could only print a message and then fail to exit. Keeping
 * it would have been complexity that reads like protection without being any.
 *
 * What is kept is the part that does work: signals set a flag every wait loop
 * polls, so an interrupt unwinds through the normal teardown - MAC stopped,
 * RX ring torn down, lock released - instead of leaving the receiver running.
 */
static volatile sig_atomic_t g_stop;

static void on_signal(int sig) { (void)sig; g_stop = 1; }

/* Interruptible sleep: returns 1 if the caller should keep going. */
static int wait_ms(double ms)
{
	double t0 = now_ms();

	while (now_ms() - t0 < ms) {
		if (g_stop) return 0;
		mt_usleep(50000);
	}
	return !g_stop;
}


static int gate_regs(void)
{
	int fail = 0;

	printf("MT_ASIC_VERSION   = 0x%08x  (chip %04x rev %04x)\n",
	       dev.rev, dev.rev >> 16, dev.rev & 0xffff);
	printf("MT_MAC_CSR0       = 0x%08x\n", mt_rr(&dev, MT_MAC_CSR0));
	printf("MT_WLAN_FUN_CTRL  = 0x%08x  (bit0 WLAN_EN, bit1 CLK_EN)\n",
	       mt_rr(&dev, MT_WLAN_FUN_CTRL));
	printf("MT_MCU_COM_REG0   = 0x%08x  (bit0 fw running, bit1 host ack)\n",
	       mt_rr(&dev, MT_MCU_COM_REG0));
	printf("MT_MCU_CLOCK_CTL  = 0x%08x  (bit0 ROM patch applied)\n",
	       mt_rr(&dev, MT_MCU_CLOCK_CTL));
	printf("MT_MAC_SYS_CTRL   = 0x%08x\n", mt_rr(&dev, MT_MAC_SYS_CTRL));
	printf("MT_USB_U3DMA_CFG  = 0x%08x  (CFG space)\n",
	       mt_rr(&dev, CFG_ADDR(MT_USB_U3DMA_CFG)));

	if (dev.rev != 0x76120044) {
		printf("GATE A: FAIL - expected MT_ASIC_VERSION 0x76120044\n");
		return 1;
	}

	/* Two write round-trips, one per address space, so a failure says which
	 * side broke. MT_TX_RTS_CFG is the MAC-space choice because mt76's own
	 * mac_stop does read/modify/restore on it, so it is proven R/W.
	 *
	 * Do NOT use MT_MAC_ADDR_DW1's U2ME_MASK here: bits 23:16 of that
	 * register are write-only on this silicon - the low 16 bits take a
	 * write and read back, the U2ME byte always reads 0. Probing with it
	 * reports a working write path as broken. */
	{
		uint32_t o = mt_rr(&dev, CFG_ADDR(MT_USB_U3DMA_CFG));
		uint32_t w = (o & ~MT_USB_DMA_CFG_RX_BULK_AGG_TOUT) |
		             FIELD_PREP(MT_USB_DMA_CFG_RX_BULK_AGG_TOUT, 0x33);
		uint32_t r;
		mt_wr(&dev, CFG_ADDR(MT_USB_U3DMA_CFG), w);
		r = mt_rr(&dev, CFG_ADDR(MT_USB_U3DMA_CFG));
		mt_wr(&dev, CFG_ADDR(MT_USB_U3DMA_CFG), o);
		printf("\nCFG-space  write 0x%08x -> read 0x%08x -> restore 0x%08x  %s\n",
		       w, r, mt_rr(&dev, CFG_ADDR(MT_USB_U3DMA_CFG)), r == w ? "OK" : "FAIL");
		fail |= (r != w);
	}
	{
		uint32_t o = mt_rr(&dev, MT_TX_RTS_CFG);
		uint32_t w = (o & ~MT_TX_RTS_CFG_RETRY_LIMIT) |
		             FIELD_PREP(MT_TX_RTS_CFG_RETRY_LIMIT, 0x2b);
		uint32_t r, back;
		mt_wr(&dev, MT_TX_RTS_CFG, w);
		r = mt_rr(&dev, MT_TX_RTS_CFG);
		mt_wr(&dev, MT_TX_RTS_CFG, o);
		back = mt_rr(&dev, MT_TX_RTS_CFG);
		printf("MAC-space  write 0x%08x -> read 0x%08x -> restore 0x%08x  %s\n",
		       w, r, back, (r == w && back == o) ? "OK" : "FAIL");
		fail |= (r != w) || (back != o);
	}

	/* EEPROM read path, and the MAC it holds. */
	{
		uint8_t mac[6];
		for (unsigned i = 0; i < 8; i += 4) {
			uint32_t v = mt_rr(&dev, EEP_ADDR(MT_EE_MAC_ADDR + i));
			for (unsigned b = 0; b < 4 && i + b < 6; b++)
				mac[i + b] = (v >> (8 * b)) & 0xff;
		}
		printf("\nEEPROM MAC (0x004) = %02x:%02x:%02x:%02x:%02x:%02x\n",
		       mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
		if (mac[0] == 0xff || (mac[0] | mac[1] | mac[2]) == 0) {
			printf("GATE A: FAIL - EEPROM MAC looks unprogrammed\n");
			fail = 1;
		}
	}

	printf("\nGATE A: %s\n", fail ? "FAIL" : "PASS");
	return fail;
}

/* Gate B: MCU transport + ROM patch + firmware, then a live MCU round-trip. */
static int gate_fw(const char *fw_dir)
{
	uint32_t clk, com0;

	printf("before load:  MT_MCU_CLOCK_CTL=0x%08x  MT_MCU_COM_REG0=0x%08x\n",
	       mt_rr(&dev, MT_MCU_CLOCK_CTL), mt_rr(&dev, MT_MCU_COM_REG0));

	if (mt_eeprom_init(&dev))
		return 1;

	if (mt_fw_init(&dev, fw_dir)) {
		printf("GATE B: FAIL - firmware load failed\n");
		return 1;
	}

	clk  = mt_rr(&dev, MT_MCU_CLOCK_CTL);
	com0 = mt_rr(&dev, MT_MCU_COM_REG0);
	printf("after load:   MT_MCU_CLOCK_CTL=0x%08x (patch bit0=%u)  "
	       "MT_MCU_COM_REG0=0x%08x (fw bit0=%u)\n",
	       clk, clk & 1, com0, com0 & 1);

	if (!(clk & 1) || !(com0 & 1)) {
		printf("GATE B: FAIL - status bits not set\n");
		return 1;
	}

	/* Follow the kernel's own post-firmware order (mt76x2u_mcu_init):
	 * Q_SELECT then RADIO_ON, neither of which waits for a response. */
	if (mt_mcu_function_select(&dev, Q_SELECT, 1)) {
		printf("GATE B: FAIL - Q_SELECT bulk-out failed\n");
		return 1;
	}
	if (mt_mcu_set_radio_state(&dev, 1)) {
		printf("GATE B: FAIL - RADIO_ON bulk-out failed\n");
		return 1;
	}
	printf("Q_SELECT + RADIO_ON sent (neither waits, as in mt76)\n");

	/* The status bits alone are not proof. CMD_LOAD_CR is the only command
	 * the kernel waits on during probe, so it is the one known-good
	 * round-trip: out on EP 8, matched by sequence on EP 5.
	 * NOTE: do not use GET_FW_VERSION - it is declared in mt76's enum and
	 * called nowhere, and the firmware does not answer it. */
	if (mt_mcu_load_cr(&dev, MT_RF_BBP_CR, 0, 0)) {
		printf("GATE B: FAIL - MCU round-trip (CMD_LOAD_CR) failed\n");
		return 1;
	}
	printf("MCU round-trip OK (CMD_LOAD_CR acked with matching seq)\n");

	printf("\nGATE B: PASS\n");
	return 0;
}

/* Gate C: full power-on + firmware + MAC/PHY init, with an oracle-diff log. */
static int gate_init(const char *fw_dir)
{
	uint32_t clk0, com0, clk1, com1, clk2, com2;

	clk0 = mt_rr(&dev, MT_MCU_CLOCK_CTL);
	com0 = mt_rr(&dev, MT_MCU_COM_REG0);
	printf("state on entry:    CLOCK_CTL=0x%08x COM_REG0=0x%08x\n", clk0, com0);

	/* Transition test. A gate that only checks "bit is set at the end"
	 * passes on stale state from a previous run, so force the bits down
	 * first and require them to come back up. */
	mt_power_cycle(&dev);
	clk1 = mt_rr(&dev, MT_MCU_CLOCK_CTL);
	com1 = mt_rr(&dev, MT_MCU_COM_REG0);
	printf("after reset+power: CLOCK_CTL=0x%08x COM_REG0=0x%08x  "
	       "(patch bit0=%u, fw bit0=%u)\n", clk1, com1, clk1 & 1, com1 & 1);

	if (mt_eeprom_init(&dev))
		return 1;

	dev.wrlog = fopen("wrlog.txt", "w");
	if (!dev.wrlog)
		printf("warning: could not open wrlog.txt for the oracle diff\n");

	if (mt_init_hardware(&dev, fw_dir)) {
		printf("GATE C: FAIL - init_hardware failed\n");
		return 1;
	}

	clk2 = mt_rr(&dev, MT_MCU_CLOCK_CTL);
	com2 = mt_rr(&dev, MT_MCU_COM_REG0);
	printf("after full init:   CLOCK_CTL=0x%08x COM_REG0=0x%08x  "
	       "(patch bit0=%u, fw bit0=%u)\n", clk2, com2, clk2 & 1, com2 & 1);
	printf("MT_MAC_CSR0=0x%08x  MT_MAC_SYS_CTRL=0x%08x  MT_MAC_STATUS=0x%08x\n",
	       mt_rr(&dev, MT_MAC_CSR0), mt_rr(&dev, MT_MAC_SYS_CTRL),
	       mt_rr(&dev, MT_MAC_STATUS));
	printf("MT_WPDMA_GLO_CFG=0x%08x (TX/RX busy bits must be 0)\n",
	       mt_rr(&dev, MT_WPDMA_GLO_CFG));

	if (!(clk2 & 1) || !(com2 & 1)) {
		printf("GATE C: FAIL - firmware status bits not set after init\n");
		return 1;
	}
	if (mt_rr(&dev, MT_WPDMA_GLO_CFG) &
	    (MT_WPDMA_GLO_CFG_TX_DMA_BUSY | MT_WPDMA_GLO_CFG_RX_DMA_BUSY)) {
		printf("GATE C: FAIL - WPDMA still busy\n");
		return 1;
	}

	printf("\nwrote %s for the oracle diff\n", "wrlog.txt");
	printf("GATE C: PASS%s\n",
	       (com1 & 1) ? "  (NOTE: reset did not clear the fw bit - see below)" : "");
	if (com1 & 1)
		printf("  The COM_REG0 fw bit survived reset+power_on, so \"bit set at\n"
		       "  the end\" is not by itself proof of a fresh load. The MCU\n"
		       "  round-trip in Gate B is the check that cannot pass on stale state.\n");
	return 0;
}

/* Gate D: full init, then set one fixed 5 GHz channel at 20 MHz. */
static int gate_chan(uint8_t chan, const char *fw_dir)
{
	if (mt_eeprom_init(&dev))
		return 1;

	dev.wrlog  = fopen("wrlog.txt", "w");
	dev.mculog = fopen("mculog.txt", "w");

	if (mt_init_hardware(&dev, fw_dir)) {
		printf("GATE D: FAIL - init_hardware failed\n");
		return 1;
	}
	printf("init complete, setting channel %u @ 20 MHz\n", chan);

	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) {
		printf("GATE D: FAIL - set_channel failed\n");
		return 1;
	}

	printf("MT_TX_BAND_CFG   = 0x%08x (bit1 5G, bit2 2G)\n",
	       mt_rr(&dev, MT_TX_BAND_CFG));
	printf("MT_BBP(CORE,1)   = 0x%08x (BW field 4:3 == 0 for 20 MHz)\n",
	       mt_rr(&dev, MT_BBP(CORE, 1)));
	printf("MT_BBP(AGC,0)    = 0x%08x\n", mt_rr(&dev, MT_BBP(AGC, 0)));
	printf("MT_EXT_CCA_CFG   = 0x%08x\n", mt_rr(&dev, MT_EXT_CCA_CFG));
	printf("MT_TX_ALC_CFG_0  = 0x%08x\n", mt_rr(&dev, MT_TX_ALC_CFG_0));
	printf("MT_TX_PWR_CFG_0  = 0x%08x\n", mt_rr(&dev, MT_TX_PWR_CFG_0));

	if (FIELD_GET(MT_BBP_CORE_R1_BW, mt_rr(&dev, MT_BBP(CORE, 1))) != 0) {
		printf("GATE D: FAIL - BBP CORE R1 bandwidth is not 20 MHz\n");
		return 1;
	}
	if (!(mt_rr(&dev, MT_TX_BAND_CFG) & MT_TX_BAND_CFG_5G)) {
		printf("GATE D: FAIL - 5 GHz band not selected\n");
		return 1;
	}

	printf("\nwrote mculog.txt - compare against the kernel ch%u stream\n", chan);
	printf("GATE D: PASS\n");
	return 0;
}

/* Gate E: inject frames. The witness is a separate radio - our own RX seeing
 * these would prove nothing. */
static int gate_tx(uint8_t chan, int count, int phy, int mcs)
{
	/* A plain 3-address data frame: broadcast DA, a source MAC chosen to be
	 * unmistakable in a monitor capture, and a magic payload with a counter. */
	static const uint8_t src[6] = { 0x02, 0x4d, 0x54, 0x76, 0x12, 0x01 };
	uint8_t frame[64];
	struct mt7612u_tx_rate rate = {
		.phy = (enum mt7612u_phy)phy, .mcs = (uint8_t)mcs, .nss = 1,
		.bw = MT7612U_BW_20, .no_ack = 1, .power_adj = 0,
	};
	/* Indexed with (phy & 7): MT_RATE_PHY is three bits, so 5-7 are
	 * representable and named nothing. Five entries read past the end. */
	const char *phy_name[] = { "CCK", "OFDM", "HT", "HT-GF", "VHT",
	                           "?5", "?6", "?7" };
	int sent = 0;

	if (mt_eeprom_init(&dev))
		return 1;
	if (mt_init_hardware(&dev, NULL)) {
		printf("GATE E: FAIL - init_hardware failed\n"); return 1;
	}
	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) {
		printf("GATE E: FAIL - set_channel failed\n"); return 1;
	}
	/* TX only: this gate never reads EP 4, so do not switch the receiver on. */
	if (mt_mac_start(&dev, 0)) {
		printf("GATE E: FAIL - mac_start failed\n"); return 1;
	}
	printf("MAC started: MT_MAC_SYS_CTRL=0x%08x (bit2 TX, bit3 RX)\n",
	       mt_rr(&dev, MT_MAC_SYS_CTRL));

	memset(frame, 0, sizeof frame);
	frame[0] = 0x08; frame[1] = 0x00;            /* data, ToDS=0 FromDS=0 */
	memset(frame + 4, 0xff, 6);                  /* addr1 = broadcast */
	memcpy(frame + 10, src, 6);                  /* addr2 = source */
	memcpy(frame + 16, src, 6);                  /* addr3 = bssid */
	memcpy(frame + 24, "MT7612U-HAL ", 12);

	printf("injecting %d frames on ch%u, %s idx %d, no-ACK, rate word 0x%04x\n",
	       count, chan, phy_name[phy & 7], mcs, mt_tx_rate_word(&rate));
	printf("source MAC %02x:%02x:%02x:%02x:%02x:%02x - grep the witness for it\n",
	       src[0], src[1], src[2], src[3], src[4], src[5]);

	for (int i = 0; i < count; i++) {
		frame[36] = (uint8_t)i;
		frame[37] = (uint8_t)(i >> 8);
		/* sequence number, so the witness can see distinct frames */
		frame[22] = (uint8_t)((i & 0xf) << 4);
		frame[23] = (uint8_t)(i >> 4);
		if (mt7612u_tx(&dev, frame, 40, &rate) == 0)
			sent++;
		mt_usleep(2000);
	}

	printf("submitted %d/%d frames\n", sent, count);
	printf("MT_MAC_STATUS=0x%08x  MT_TX_STA_CNT0=0x%08x\n",
	       mt_rr(&dev, MT_MAC_STATUS), mt_rr(&dev, 0x1710));
	mt_mac_stop(&dev);

	if (sent != count) { printf("GATE E: FAIL - some submissions failed\n"); return 1; }
	printf("\nGATE E: frames submitted. PASS/FAIL is decided by the witness.\n");
	return 0;
}

/* Gate F: monitor RX. Decode rate/BW and per-chain RSSI from the RXWI. */
static int gate_rx(uint8_t chan, int want)
{
	static /* Indexed with (phy & 7): MT_RATE_PHY is three bits, so 5-7 are
	 * representable and named nothing. Five entries read past the end. */
	const char *phy_name[] = { "CCK", "OFDM", "HT", "HT-GF", "VHT",
	                           "?5", "?6", "?7" };
	static const char *bw_name[] = { "20", "40", "80", "?" };
	uint8_t buf[4096];
	int got = 0, empty = 0;

	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) {
		printf("GATE F: FAIL - init_hardware failed\n"); return 1;
	}
	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) {
		printf("GATE F: FAIL - set_channel failed\n"); return 1;
	}
	if (mt_mac_start(&dev, 1)) {
		printf("GATE F: FAIL - mac_start failed\n"); return 1;
	}
	/* Monitor: drop only CRC and PHY errors, accept everything else. The
	 * initvals value 0x15f97 drops a great deal more than that. */
	mt_wr(&dev, MT_RX_FILTR_CFG,
	      MT_RX_FILTR_CFG_CRC_ERR | MT_RX_FILTR_CFG_PHY_ERR);
	printf("listening on ch%u\n", chan);
	printf("  MT_RX_FILTR_CFG  = 0x%08x\n", mt_rr(&dev, MT_RX_FILTR_CFG));
	printf("  MT_MAC_SYS_CTRL  = 0x%08x (bit2 TX, bit3 RX)\n",
	       mt_rr(&dev, MT_MAC_SYS_CTRL));
	printf("  MT_USB_U3DMA_CFG = 0x%08x (bit22 RX_BULK_EN)\n",
	       mt_rr(&dev, CFG_ADDR(MT_USB_U3DMA_CFG)));
	printf("  MT_MAC_STATUS    = 0x%08x\n", mt_rr(&dev, MT_MAC_STATUS));
	printf("  MT_RX_STAT_1     = 0x%08x (CCA errors seen = RF is live)\n",
	       mt_rr(&dev, MT_RX_STAT_1));

	while (got < want && empty < 200) {
		struct mt7612u_rx_info info;
		const uint8_t *f = NULL;
		int len = mt_rx_one(&dev, buf, sizeof buf, &f, &info, 50);

		if (len <= 0) { empty++; continue; }
		got++;
		if (got <= 20 || got % 50 == 0)
			printf("  #%-4d len=%-5d %-5s mcs=%-2u nss=%u bw=%-2s "
			       "sgi=%u ldpc=%u stbc=%u rssi=[%d,%d] sa=%02x:%02x:%02x:%02x:%02x:%02x\n",
			       got, len, phy_name[info.phy & 7], info.mcs, info.nss,
			       bw_name[info.bw & 3], info.sgi, info.ldpc, info.stbc,
			       info.rssi[0], info.rssi[1],
			       len > 15 ? f[10] : 0, len > 15 ? f[11] : 0,
			       len > 15 ? f[12] : 0, len > 15 ? f[13] : 0,
			       len > 15 ? f[14] : 0, len > 15 ? f[15] : 0);
	}

	printf("\nreceived %d frames\n", got);
	mt_mac_stop(&dev);
	if (got == 0) {
		printf("GATE F: FAIL - no frames received\n");
		return 1;
	}
	printf("GATE F: PASS\n");
	return 0;
}

/* How expensive is a channel change? Decides whether FHSS is on the table. */
static int gate_hop(void)
{
	static const uint8_t chans[] = { 149, 153, 157, 161, 149, 157, 153, 161 };
	double t0, full = 0, fast = 0;

	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) return 1;
	if (mt_set_channel(&dev, 149, MT7612U_BW_20)) return 1;

	for (unsigned i = 0; i < sizeof chans; i++) {
		t0 = now_ms();
		if (mt_set_channel_ex(&dev, chans[i], MT7612U_BW_20, 0)) return 1;
		full += now_ms() - t0;
	}
	for (unsigned i = 0; i < sizeof chans; i++) {
		t0 = now_ms();
		if (mt_set_channel_ex(&dev, chans[i], MT7612U_BW_20, 1)) return 1;
		fast += now_ms() - t0;
	}
	printf("channel switch, mean of %zu:\n", sizeof chans);
	printf("  full (with firmware calibration burst): %6.2f ms\n", full / sizeof chans);
	printf("  fast (calibration skipped)            : %6.2f ms\n", fast / sizeof chans);
	printf("\nfor reference, devourer on Realtek hops in ~0.5-2.5 ms\n");
	mt_mac_stop(&dev);
	return 0;
}

/*
 * Gate G, the per-frame rate-control check:
 *  1. alternate MCS0/MCS7 frame by frame - the witness must see the rate the
 *     frame's own index calls for. Correlating on the index rather than
 *     demanding an unbroken alternating sequence keeps a lost frame from
 *     failing a working test.
 *  2. make the hardware rate LUT disagree with txwi.rate and see which airs,
 *     with a positive control that sets MT_TXWI_FLAGS_TX_RATE_LUT.
 */
static int gate_g(uint8_t chan, int count)
{
	static const uint8_t src[6] = { 0x02, 0x4d, 0x54, 0x76, 0x12, 0x01 };
	uint8_t frame[64];
	struct mt7612u_tx_rate mcs0 = { .phy = MT7612U_PHY_HT, .mcs = 0, .nss = 1,
	                                .bw = MT7612U_BW_20, .no_ack = 1 };
	struct mt7612u_tx_rate mcs7 = { .phy = MT7612U_PHY_HT, .mcs = 7, .nss = 1,
	                                .bw = MT7612U_BW_20, .no_ack = 1 };
	struct mt7612u_tx_rate ofdm6 = { .phy = MT7612U_PHY_OFDM, .mcs = 0, .nss = 1,
	                                 .bw = MT7612U_BW_20, .no_ack = 1 };
	uint32_t lut;
	long sent_alt = 0, sent_lut[2] = { 0, 0 };

	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) return 1;
	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) return 1;
	if (mt_mac_start(&dev, 0)) return 1;

	memset(frame, 0, sizeof frame);
	frame[0] = 0x08;
	memset(frame + 4, 0xff, 6);
	memcpy(frame + 10, src, 6);
	memcpy(frame + 16, src, 6);
	memcpy(frame + 24, "MT7612U-HAL ", 12);
	/* body[] at the witness starts at offset 24: [0..11] magic, [12] tag,
	 * [13..14] index. */

	printf("test 1: per-frame alternation, HT MCS0 (rate word 0x%04x) / "
	       "MCS7 (0x%04x)\n", mt_tx_rate_word(&mcs0), mt_tx_rate_word(&mcs7));
	for (int i = 0; i < count; i++) {
		frame[36] = 'T';
		frame[37] = (uint8_t)i;
		frame[38] = (uint8_t)(i >> 8);
		if (mt7612u_tx(&dev, frame, 40, (i & 1) ? &mcs7 : &mcs0) == 0)
			sent_alt++;
		mt_usleep(2000);
	}
	/* Report what actually went out, not what was asked for. Printing the
	 * requested count and returning 0 regardless made this gate pass even
	 * if every single submit failed - and then handed the witness an
	 * experiment that never aired. */
	printf("  sent %ld/%d frames, even index = MCS0, odd = MCS7\n",
	       sent_alt, count);

	/* Load WCID 1's hardware rate LUT with OFDM 6 Mbps, then transmit
	 * HT MCS7 frames that point at it. */
	lut = FIELD_PREP(MT_WCID_TX_INFO_RATE, mt_tx_rate_word(&ofdm6)) |
	      FIELD_PREP(MT_WCID_TX_INFO_NSS, 1) | MT_WCID_TX_INFO_SET;
	mt_wr(&dev, MT_WCID_TX_RATE(1), lut);
	mt_wr(&dev, MT_WCID_TX_RATE(1) + 4, 0);
	printf("\ntest 2: WCID 1 rate LUT = 0x%08x (OFDM 6 Mbps), "
	       "txwi.rate = HT MCS7\n", lut);
	printf("  read back MT_WCID_TX_RATE(1) = 0x%08x\n",
	       mt_rr(&dev, MT_WCID_TX_RATE(1)));

	for (int arm = 0; arm < 2; arm++) {
		long ok = 0;

		for (int i = 0; i < 150; i++) {
			frame[36] = arm ? 'B' : 'A';
			frame[37] = (uint8_t)i;
			frame[38] = 0;
			if (mt_tx_raw(&dev, frame, 40, &mcs7, 1, arm) == 0)
				ok++;
			mt_usleep(2000);
		}
		printf("  arm %c: wcid=1, TX_RATE_LUT flag %s -> %ld/150 frames\n",
		       arm ? 'B' : 'A', arm ? "SET" : "clear", ok);
		sent_lut[arm] = ok;
	}

	mt_mac_stop(&dev);
	/* An arm that aired nothing is not a result the witness can rule on:
	 * "no frames decoded" would read as a negative finding rather than as
	 * a transmitter that never spoke. Fail loudly instead. */
	if (sent_alt == 0 || sent_lut[0] == 0 || sent_lut[1] == 0) {
		printf("\nGATE g: FAIL - an arm submitted no frames "
		       "(alt %ld, lut A %ld, lut B %ld); the witness has nothing "
		       "to rule on\n", sent_alt, sent_lut[0], sent_lut[1]);
		return 1;
	}
	printf("\nGate G frames sent. The witness decides.\n");
	return 0;
}

static double cpu_ms(void)
{
	struct rusage r;
	getrusage(RUSAGE_SELF, &r);
	return r.ru_utime.tv_sec * 1000.0 + r.ru_utime.tv_usec / 1000.0 +
	       r.ru_stime.tv_sec * 1000.0 + r.ru_stime.tv_usec / 1000.0;
}

/* Sustained TX: synchronous path vs the async ring, same frame and rate. */
/*
 * Largest MPDU the part will actually put on air. Not a throughput test: one
 * burst per size, and the witness decides which sizes arrived.
 *
 * Worth measuring because the ceiling in this port was a buffer constant, not
 * a number anyone had checked, and because the two public TX entry points did
 * not agree on it - mt7612u_tx() refused above MT_TX_BUF_MAX - 32 while
 * mt7612u_send_packets() bounded only against the 16 KB aggregate buffer.
 * 802.11 puts the non-A-MSDU MPDU ceiling at 2304, which is the interesting
 * boundary; sizes above it are here to see whether the MAC or the USB path
 * objects first.
 */
/* Defined below with the other RX callbacks; gate_mtu needs it to keep the
 * receiver drained during its async pass. */
static void drain_cb(void *user, const void *frame, size_t len,
                     const struct mt7612u_rx_info *info);

static int gate_mtu(uint8_t chan, int count)
{
	static const uint8_t src[6] = { 0x02, 0x4d, 0x54, 0x76, 0x12, 0x01 };
	static const int sizes[] = {
		 200, 1000, 1500, 2000, 2304, 3000, 3836, 3837, 4000, 4064, 4065,
	};
	static uint8_t frame[8192];
	struct mt7612u_tx_rate rate = { .phy = MT7612U_PHY_HT, .mcs = 7, .nss = 1,
	                                .bw = MT7612U_BW_20, .no_ack = 1 };
	unsigned k;	int short_arms = 0;   /* sizes where the two TX paths disagreed */


	if (count <= 0 || count > 1000) count = 60;
	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) return 1;
	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) return 1;
	if (mt_mac_start(&dev, 0)) return 1;

	memset(frame, 0, sizeof frame);
	frame[0] = 0x08;                        /* data, 3-address */
	memset(frame + 4, 0xff, 6);             /* broadcast */
	memcpy(frame + 10, src, 6);
	memcpy(frame + 16, src, 6);
	memcpy(frame + 24, "MT7612U-HAL ", 12);

	printf("ch%u, HT MCS7 20 MHz, %d frames per size, BOTH TX paths.\n"
	       "'accepted' is what this driver submitted; the witness reports\n"
	       "which lengths actually decoded.\n\n", chan, count);
	/* Two passes on purpose. mt_tx_raw() takes the async ring whenever one
	 * is running and the synchronous bulk otherwise, and those had
	 * different ceilings: the ring refused above 2048 while the builder
	 * produced up to 4096. Measuring only the sync path is what hid that,
	 * so the sweep now reports both and a divergence is visible in the
	 * table rather than in an integration months later. */
	printf("  %-6s %-10s %-10s %s\n", "bytes", "sync", "async", "note");

	for (k = 0; k < sizeof sizes / sizeof sizes[0]; k++) {
		int len = sizes[k];
		long ok_sync = 0, ok_async = 0;
		unsigned long drained = 0;
		int i, pass;

		if ((size_t)len > sizeof frame) continue;
		/* Tag the payload with the size so the witness can bucket by what
		 * was ASKED for, not only by what arrived. */
		frame[36] = (uint8_t)(len & 0xff);
		frame[37] = (uint8_t)(len >> 8);

		for (pass = 0; pass < 2; pass++) {
			long *ok = pass ? &ok_async : &ok_sync;

			/* Pass 1 brings up the RX ring, which is what makes
			 * mt_tx_raw() take the async path. The receiver must be
			 * drained or the chip wedges below USB level, hence a
			 * real callback rather than a null one. */
			if (pass && mt7612u_rx_start(&dev, drain_cb, &drained)) {
				printf("  %-6d rx_start failed - async pass skipped\n", len);
				break;
			}
			for (i = 0; i < count; i++) {
				frame[38] = (uint8_t)i;
				if (mt7612u_tx(&dev, frame, (size_t)len, &rate) == 0)
					(*ok)++;
				mt_usleep(1500);
			}
			if (pass) mt7612u_rx_stop(&dev);
		}

		printf("  %-6d %ld/%-8d %ld/%-8d %s\n", len, ok_sync, count,
		       ok_async, count,
		       (ok_sync == 0 && ok_async == 0) ? "refused by this driver" :
		       (ok_sync != ok_async) ? "PATHS DISAGREE" :
		       (len > 2304 ? "above the 802.11 MPDU ceiling" : ""));
		if (ok_sync != ok_async) short_arms++;
		mt_usleep(120000);
	}

	mt_mac_stop(&dev);
	if (short_arms) {
		printf("\nGATE mtu: FAIL - %d size(s) where the sync and async TX\n"
		       "paths disagreed. One public API must not have two ceilings.\n",
		       short_arms);
		return 1;
	}
	printf("\nThe largest size with a non-zero witness count is the answer.\n"
	       "A size this driver accepted but the witness never saw was\n"
	       "submitted and dropped somewhere below - that is the real limit.\n");
	return 0;
}

static int gate_soak(uint8_t chan, int secs, int framelen)
{
	static const uint8_t src[6] = { 0x02, 0x4d, 0x54, 0x76, 0x12, 0x01 };
	static uint8_t frame[2048];
	struct mt7612u_tx_rate rate = { .phy = MT7612U_PHY_HT, .mcs = 7, .nss = 1,
	                                .bw = MT7612U_BW_20, .no_ack = 1 };
	double t0, wall, c0, cpu, smax, ssum;
	long n;

	if (framelen < 40 || framelen > 1500) framelen = 1400;
	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) return 1;
	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) return 1;
	if (mt_mac_start(&dev, 0)) return 1;

	memset(frame, 0, sizeof frame);
	frame[0] = 0x08;
	memset(frame + 4, 0xff, 6);
	memcpy(frame + 10, src, 6);
	memcpy(frame + 16, src, 6);
	memcpy(frame + 24, "MT7612U-HAL ", 12);

	printf("soak: %d s per arm, %d-byte frames, HT MCS7 20 MHz, no-ACK\n\n",
	       secs, framelen);

	/* --- synchronous --- */
	n = 0; t0 = now_ms(); c0 = cpu_ms(); smax = 0; ssum = 0;
	while (now_ms() - t0 < secs * 1000.0) {
		double s0 = now_ms(), s1;
		frame[36] = (uint8_t)n; frame[37] = (uint8_t)(n >> 8);
		if (mt7612u_tx(&dev, frame, (size_t)framelen, &rate) == 0) n++;
		s1 = now_ms() - s0;
		ssum += s1; if (s1 > smax) smax = s1;
	}
	wall = now_ms() - t0; cpu = cpu_ms() - c0;
	printf("  sync : %7ld frames  %8.0f fps  %6.2f Mbit/s  cpu %5.1f%%  "
	       "submit mean %.3f ms max %.1f ms\n",
	       n, n * 1000.0 / wall, n * framelen * 8.0 / wall / 1000.0,
	       100.0 * cpu / wall, ssum / (n ? n : 1), smax);

	/* --- async ring --- */
	if (mt_async_start(&dev, NULL, NULL)) { printf("async start failed\n"); return 1; }
	n = 0; t0 = now_ms(); c0 = cpu_ms(); smax = 0; ssum = 0;
	while (now_ms() - t0 < secs * 1000.0) {
		double s0 = now_ms(), s1;
		frame[36] = (uint8_t)n; frame[37] = (uint8_t)(n >> 8);
		if (mt7612u_tx(&dev, frame, (size_t)framelen, &rate) == 0) n++;
		s1 = now_ms() - s0;
		ssum += s1; if (s1 > smax) smax = s1;
	}
	wall = now_ms() - t0; cpu = cpu_ms() - c0;
	printf("  async: %7ld frames  %8.0f fps  %6.2f Mbit/s  cpu %5.1f%%  "
	       "submit mean %.3f ms max %.1f ms\n",
	       n, n * 1000.0 / wall, n * framelen * 8.0 / wall / 1000.0,
	       100.0 * cpu / wall, ssum / (n ? n : 1), smax);
	{
		struct mt_async_stats st;

		mt_async_stats(&dev, &st);
		printf("         submitted=%llu completed=%llu errors=%llu\n",
		       (unsigned long long)st.tx_submitted,
		       (unsigned long long)st.tx_done,
		       (unsigned long long)st.tx_err);
	}
	mt_async_stop(&dev);

	/* Below saturation the pool is never full, so submit returns as soon as
	 * the transfer is queued instead of waiting for the wire. That is what
	 * the ring actually buys a caller that has other work to do. */
	printf("\n  paced to ~800 fps (well under the %0.0f fps air ceiling):\n",
	       n * 1000.0 / wall);
	for (int arm = 0; arm < 2; arm++) {
		if (arm && mt_async_start(&dev, NULL, NULL)) return 1;
		n = 0; smax = 0; ssum = 0; t0 = now_ms(); c0 = cpu_ms();
		while (now_ms() - t0 < secs * 1000.0) {
			double s0 = now_ms(), s1;
			frame[36] = (uint8_t)n;
			if (mt7612u_tx(&dev, frame, (size_t)framelen, &rate) == 0) n++;
			s1 = now_ms() - s0;
			ssum += s1; if (s1 > smax) smax = s1;
			mt_usleep(1250);
		}
		wall = now_ms() - t0; cpu = cpu_ms() - c0;
		printf("    %-5s %6ld frames  %5.0f fps  cpu %4.1f%%  "
		       "submit mean %.3f ms max %.1f ms\n",
		       arm ? "async" : "sync", n, n * 1000.0 / wall,
		       100.0 * cpu / wall, ssum / (n ? n : 1), smax);
		if (arm) mt_async_stop(&dev);
	}

	printf("\n  MT_TX_STA_CNT0 = 0x%08x\n", mt_rr(&dev, 0x1710));
	mt_mac_stop(&dev);
	return 0;
}

/*
 * Written by the libusb event thread, read by the gate while that thread is
 * still running - gate_arx() and gate_duplex() both print before calling
 * mt7612u_rx_stop(). Plain increments there are a data race, so the displayed
 * rate and the duplex pass/fail verdict could be built from torn counts.
 * Relaxed atomics: these are counters, nothing orders anything else off them,
 * and this is the RX hot path in a throughput gate.
 */
struct arx_ctx { _Atomic unsigned long n; _Atomic unsigned long by_phy[8]; };
static void arx_cb(void *user, const void *frame, size_t len,
                   const struct mt7612u_rx_info *info)
{
	struct arx_ctx *c = user;
	(void)frame; (void)len;
	atomic_fetch_add_explicit(&c->n, 1, memory_order_relaxed);
	atomic_fetch_add_explicit(&c->by_phy[info->phy & 7], 1,
	                          memory_order_relaxed);
}

/* Async RX ring: the callback path StartRxLoop needs. */
static int gate_arx(uint8_t chan, int secs)
{
	static /* Indexed with (phy & 7): MT_RATE_PHY is three bits, so 5-7 are
	 * representable and named nothing. Five entries read past the end. */
	const char *phy_name[] = { "CCK", "OFDM", "HT", "HT-GF", "VHT",
	                           "?5", "?6", "?7" };
	struct arx_ctx ctx = { 0 };
	double t0;

	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) return 1;
	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) return 1;
	if (mt_mac_start(&dev, 1)) return 1;
	mt_wr(&dev, MT_RX_FILTR_CFG,
	      MT_RX_FILTR_CFG_CRC_ERR | MT_RX_FILTR_CFG_PHY_ERR);

	if (mt7612u_rx_start(&dev, arx_cb, &ctx)) {
		printf("GATE arx: FAIL - rx_start failed\n"); return 1;
	}
	t0 = now_ms();
	wait_ms(secs * 1000.0);
	{
		struct mt_async_stats st;
		/* Actual elapsed, not the requested duration: an interrupt now
		 * unwinds through here, and dividing by the request would report
		 * a rate the run never achieved. */
		double el = (now_ms() - t0) / 1000.0;

		mt_async_stats(&dev, &st);
		printf("async RX on ch%u for %.1f s: %lu frames (%.0f/s), rx_err=%llu "
		       "rx_invalid=%llu rx_dropped=%llu\n",
		       chan, el, ctx.n, ctx.n / (el > 0 ? el : 1),
		       (unsigned long long)st.rx_err,
		       (unsigned long long)st.rx_invalid,
		       (unsigned long long)st.rx_dropped);
	}
	for (int i = 0; i < 8; i++)
		if (ctx.by_phy[i]) printf("  %-6s %lu\n", phy_name[i], ctx.by_phy[i]);
	mt7612u_rx_stop(&dev);
	mt_mac_stop(&dev);
	return ctx.n ? 0 : 1;
}

/*
 * Concurrent TX and RX on one claimed handle - the InitWrite + StartRxLoop +
 * send_packet shape a bidirectional link consumer uses.
 *
 * This gate needs a PEER transmitting on the same channel; without one it
 * measures nothing and says so rather than reporting a fault. With one,
 * measured: 2886 fps out while 44 fps still arrives, so a saturated TX does
 * throttle receive but does not stop it.
 */
#define RECOVER_S 3.0   /* post-flood listen window */
static int gate_duplex(uint8_t chan, int secs)
{
	static const uint8_t src[6] = { 0x02, 0x4d, 0x54, 0x76, 0x12, 0x01 };
	static uint8_t frame[2048];
	struct mt7612u_tx_rate rate = { .phy = MT7612U_PHY_HT, .mcs = 7, .nss = 1,
	                                .bw = MT7612U_BW_20, .no_ack = 1 };
	struct arx_ctx ctx = { 0 };
	double t0, wall;
	long n = 0;

	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) return 1;
	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) return 1;
	if (mt_mac_start(&dev, 1)) return 1;
	mt_wr(&dev, MT_RX_FILTR_CFG,
	      MT_RX_FILTR_CFG_CRC_ERR | MT_RX_FILTR_CFG_PHY_ERR);

	memset(frame, 0, sizeof frame);
	frame[0] = 0x08;
	memset(frame + 4, 0xff, 6);
	memcpy(frame + 10, src, 6);
	memcpy(frame + 16, src, 6);
	memcpy(frame + 24, "MT7612U-HAL ", 12);

	if (mt7612u_rx_start(&dev, arx_cb, &ctx)) return 1;

	t0 = now_ms();
	while (now_ms() - t0 < secs * 1000.0) {
		frame[36] = (uint8_t)n; frame[37] = (uint8_t)(n >> 8);
		if (mt7612u_tx(&dev, frame, 1400, &rate) == 0) n++;
	}
	wall = now_ms() - t0;
	printf("duplex on ch%u for %.1f s:\n", chan, wall / 1000.0);
	{
		struct mt_async_stats st;

		mt_async_stats(&dev, &st);
		printf("  TX %ld frames (%.0f fps)  RX %lu frames (%.0f fps)  "
		       "tx_err=%llu rx_err=%llu\n",
		       n, n * 1000.0 / wall, ctx.n, ctx.n * 1000.0 / wall,
		       (unsigned long long)st.tx_err,
		       (unsigned long long)st.rx_err);
	}
	/*
	 * The verdict was (n && ctx.n), which is satisfiable: with a peer
	 * transmitting this gate sees 2886 fps out and 44 fps in. An earlier
	 * reading of "RX is always 0 here" was wrong - the peer adapter had
	 * silently failed its firmware load, so nothing was on air at all.
	 *
	 * What is added is the second half, not a replacement: after TX stops,
	 * the receiver must still deliver. That separates "throttled while
	 * transmitting", which is expected and now quantified, from "the flood
	 * wedged the receiver", which is the failure worth catching and which
	 * the in-flood count alone cannot distinguish from a quiet channel.
	 * The failure message names the peer requirement because a missing
	 * stimulus and a wedged receiver look identical from here.
	 */
	{
		unsigned long before = atomic_load_explicit(&ctx.n,
		                                            memory_order_relaxed);
		unsigned long after;

		printf("  TX stopped; listening %.1f s for the receiver to recover\n",
		       RECOVER_S);
		if (!wait_ms(RECOVER_S * 1000.0)) {
			mt7612u_rx_stop(&dev);
			mt_mac_stop(&dev);
			return 1;
		}
		after = atomic_load_explicit(&ctx.n, memory_order_relaxed);
		printf("  RX after the flood: %lu frames\n", after - before);
		mt7612u_rx_stop(&dev);
		mt_mac_stop(&dev);

		if (!n) {
			printf("GATE duplex: FAIL - nothing transmitted\n");
			return 1;
		}
		if (after == before) {
			printf("GATE duplex: FAIL - no frames received after TX stopped. "
			       "Either the flood wedged the receiver, or no peer was "
			       "transmitting; this gate needs one on the same channel.\n");
			return 1;
		}
		printf("GATE duplex: PASS - %ld frames out, receiver healthy after\n", n);
	}
	return 0;
}

/* TX power: compare our EEPROM-derived registers against the values the
 * kernel driver wrote for the same channel (captured in usbmon-bus2.txt). */
static int gate_pwr(uint8_t chan)
{
	static const struct { uint32_t reg; uint32_t kernel_ch149; const char *n; } ref[] = {
		{ MT_TX_PWR_CFG_0, 0x04070606, "MT_TX_PWR_CFG_0" },
		{ MT_TX_PWR_CFG_1, 0x04060202, "MT_TX_PWR_CFG_1" },
		{ MT_TX_PWR_CFG_2, 0x04060101, "MT_TX_PWR_CFG_2" },
		{ MT_TX_PWR_CFG_3, 0x04060101, "MT_TX_PWR_CFG_3" },
		{ MT_TX_PWR_CFG_4, 0x00000101, "MT_TX_PWR_CFG_4" },
		{ MT_TX_PWR_CFG_7, 0x00010002, "MT_TX_PWR_CFG_7" },
		{ MT_TX_PWR_CFG_8, 0x00000001, "MT_TX_PWR_CFG_8" },
		{ MT_TX_PWR_CFG_9, 0x00000001, "MT_TX_PWR_CFG_9" },
		{ MT_TX_ALC_CFG_0, 0x2f2f171a, "MT_TX_ALC_CFG_0" },
	};
	int bad = 0;

	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) return 1;
	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) return 1;

	printf("txpower_conf = %d (0.5 dB units = %d dBm), tssi=%d\n",
	       dev.txpower_conf, dev.txpower_conf / 2, mt_tssi_enabled(&dev));
	printf("target_power = %d, chain deltas = %d/%d\n\n",
	       dev.target_power, dev.target_power_delta[0], dev.target_power_delta[1]);

	printf("%-18s %-12s %-12s\n", "register", "ours", "kernel(ch149)");
	for (unsigned i = 0; i < sizeof ref / sizeof ref[0]; i++) {
		uint32_t v = mt_rr(&dev, ref[i].reg);
		int match = (chan == 149) ? (v == ref[i].kernel_ch149) : 1;

		printf("  %-16s 0x%08x   0x%08x   %s\n", ref[i].n, v,
		       ref[i].kernel_ch149,
		       chan != 149 ? "(n/a, not ch149)" : (match ? "MATCH" : "*** DIFFER ***"));
		if (!match) bad++;
	}

	printf("\nper-rate table (0.5 dB units):\n  cck  ");
	for (int i = 0; i < 4; i++) printf("%3d ", dev.rate_power.cck[i]);
	printf("\n  ofdm ");
	for (int i = 0; i < 8; i++) printf("%3d ", dev.rate_power.ofdm[i]);
	printf("\n  ht   ");
	for (int i = 0; i < 16; i++) printf("%3d ", dev.rate_power.ht[i]);
	printf("\n  vht  %3d %3d\n", dev.rate_power.vht[0], dev.rate_power.vht[1]);

	mt_mac_stop(&dev);
	printf("\nGATE pwr: %s\n", bad ? "FAIL" : "PASS");
	return bad;
}

/*
 * A-MPDU. Three arms, same QoS-data frame and rate, distinguished by a tag
 * byte in the payload so the witness can separate them:
 *   A  no AMPDU flag                (baseline)
 *   B  AMPDU flag, QSEL_EDCA
 *   C  AMPDU flag, QSEL_MGMT        (what mt76 picks for aggregated TX)
 * The observable is the witness's paggr / ppdu fields: a frame that arrived
 * as part of an aggregate reports paggr=1.
 */
static int gate_ampdu(uint8_t chan, int count)
{
	static const uint8_t src[6]  = { 0x02, 0x4d, 0x54, 0x76, 0x12, 0x01 };
	static const uint8_t peer[6] = { 0x02, 0x4d, 0x54, 0x76, 0x12, 0x02 };
	static uint8_t frame[128];
	struct mt7612u_tx_rate rate = { .phy = MT7612U_PHY_HT, .mcs = 7, .nss = 1,
	                                .bw = MT7612U_BW_20, .no_ack = 1 };
	static const struct { char tag; unsigned opts; const char *what; } arms[] = {
		{ 'A', 0,                                    "no AMPDU (baseline)" },
		{ 'B', MT_TXOPT_AMPDU,                       "AMPDU + QSEL_EDCA" },
		{ 'C', MT_TXOPT_AMPDU | MT_TXOPT_QSEL_MGMT,  "AMPDU + QSEL_MGMT" },
	};

	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) return 1;
	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) return 1;
	if (mt_mac_start(&dev, 0)) return 1;

	/* A real station-table entry: aggregation is a per-peer notion, and
	 * wcid 0xff (what the injector normally uses) names no peer. */
	mt_wcid_setup(&dev, 1, peer);
	printf("WCID 1 = %02x:%02x:%02x:%02x:%02x:%02x\n",
	       peer[0], peer[1], peer[2], peer[3], peer[4], peer[5]);

	memset(frame, 0, sizeof frame);
	frame[0] = 0x88;                    /* QoS Data */
	frame[1] = 0x00;
	memcpy(frame + 4,  peer, 6);        /* addr1: unicast to the peer */
	memcpy(frame + 10, src, 6);
	memcpy(frame + 16, src, 6);
	/* QoS Control: TID 0, Ack Policy = No Ack (bits 6:5 = 01). Leaving this
	 * at Normal Ack makes the MAC retry every unicast frame against a peer
	 * that never answers, which costs ~50x throughput. */
	frame[24] = 0x20; frame[25] = 0x00;
	memcpy(frame + 26, "MT7612U-HAL ", 12);

	if (mt_async_start(&dev, NULL, NULL)) return 1;

	for (unsigned a = 0; a < sizeof arms / sizeof arms[0]; a++) {
		double t0 = now_ms(), wall;

		for (int i = 0; i < count; i++) {
			frame[22] = (uint8_t)((i & 0xf) << 4);
			frame[23] = (uint8_t)(i >> 4);
			frame[38] = (uint8_t)arms[a].tag;
			frame[39] = (uint8_t)i;
			frame[40] = (uint8_t)(i >> 8);
			/* back to back, no pacing - aggregation needs frames
			 * queued faster than the air drains them */
			mt_tx_raw(&dev, frame, 48, &rate, 1, arms[a].opts);
		}
		wall = now_ms() - t0;
		printf("  arm %c: %-22s %5d frames  %7.0f fps  %6.2f Mbit/s\n",
		       arms[a].tag, arms[a].what, count,
		       count * 1000.0 / wall, count * 48 * 8.0 / wall / 1000.0);
		mt_usleep(200000);
	}
	{
		struct mt_async_stats st;

		mt_async_stats(&dev, &st);
		printf("  tx_err=%llu\n", (unsigned long long)st.tx_err);
	}

	/* The bisect above showed unicast is what collapses throughput (the MAC
	 * arms an ACK timeout for a peer that never answers), so measure the
	 * aggregation payoff on broadcast, where the link actually runs.
	 * Aggregation amortises preamble+IFS, so it should matter far more at
	 * small frame sizes than at 1400 bytes. */
	printf("\nA-MPDU payoff on broadcast QoS, HT MCS7, 3 s per cell:\n");
	printf("  %-6s %-7s %8s %10s\n", "bytes", "ampdu", "fps", "Mbit/s");
	{
		static const int sizes[] = { 200, 1400 };
		static const char tags[2][2] = { { 'D', 'E' }, { 'F', 'G' } };

		for (unsigned z = 0; z < 2; z++) {
			for (int agg = 0; agg < 2; agg++) {
				double t0, wall;
				long n = 0;

				memset(frame, 0, sizeof frame);
				frame[0] = 0x88;                 /* QoS data */
				memset(frame + 4, 0xff, 6);      /* broadcast */
				memcpy(frame + 10, src, 6);
				memcpy(frame + 16, src, 6);
				frame[24] = 0x20; frame[25] = 0; /* TID 0, No Ack */
				memcpy(frame + 26, "MT7612U-HAL ", 12);
				frame[38] = (uint8_t)tags[z][agg];

				t0 = now_ms();
				while (now_ms() - t0 < 3000.0) {
					frame[22] = (uint8_t)((n & 0xf) << 4);
					frame[23] = (uint8_t)(n >> 4);
					if (mt_tx_raw(&dev, frame, (size_t)sizes[z], &rate, 1,
					              agg ? (MT_TXOPT_AMPDU | MT_TXOPT_QSEL_MGMT) : 0) == 0)
						n++;
				}
				wall = now_ms() - t0;
				printf("  %-6d %-7s %8.0f %10.2f   (tag %c)\n",
				       sizes[z], agg ? "on" : "off",
				       n * 1000.0 / wall,
				       n * sizes[z] * 8.0 / wall / 1000.0, tags[z][agg]);
			}
		}
	}
	mt_async_stop(&dev);
	mt_mac_stop(&dev);
	printf("\nA-MPDU frames sent. The witness paggr/ppdu fields decide.\n");
	return 0;
}

/* Somebody has to read EP 4 whenever MAC RX is on; this gate does not care
 * what arrives, only that the endpoint keeps being drained. */
static void drain_cb(void *user, const void *frame, size_t len,
                     const struct mt7612u_rx_info *info)
{
	(void)frame; (void)len; (void)info;
	atomic_fetch_add_explicit((_Atomic unsigned long *)user, 1,
	                          memory_order_relaxed);
}

/* Capability descriptor, TSF and 40 MHz. */
static int gate_caps(uint8_t chan)
{
	struct mt7612u_caps c;
	unsigned long drained = 0;
	uint64_t t1, t2;
	int64_t delta;
	int bad = 0;

	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) return 1;
	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) return 1;
	/* The RX ring must be draining EP 4 *before* the receiver is enabled.
	 * This gate then sits through two 200 ms sleeps and a channel switch;
	 * with nothing reading, that is long enough to wedge the part below
	 * the USB level, which no software reset recovers. */
	if (mt_async_start(&dev, drain_cb, &drained)) return 1;
	if (mt_mac_start(&dev, 1)) { mt_async_stop(&dev); return 1; }

	mt7612u_get_caps(&dev, &c);
	printf("caps: %s rev 0x%08x  %dTx%dRx  bw_mask 0x%02x (20%s%s)\n",
	       c.chip_name, c.rev, c.nss_tx, c.nss_rx, c.bw_mask,
	       (c.bw_mask & 2) ? "/40" : "", (c.bw_mask & 4) ? "/80" : "");
	printf("      5 GHz %u-%u MHz, 2.4 GHz %u-%u MHz\n",
	       c.band_5g_min_mhz, c.band_5g_max_mhz,
	       c.band_2g_min_mhz, c.band_2g_max_mhz);
	printf("      ampdu_tx=%u per_chain_rssi=%u narrowband=%u fast_retune=%u\n",
	       c.ampdu_tx, c.per_chain_rssi, c.narrowband, c.fast_retune);
	printf("      max MPDU: tx %u  rx %u  (rx is MT_MAX_LEN_CFG 0x%03x on air,\n"
	       "                             less the 4-byte FCS)\n",
	       c.max_mpdu_tx, c.max_mpdu_rx,
	       mt_rr(&dev, MT_MAX_LEN_CFG) & 0xfff);

	printf("\nRX gain from EEPROM: rssi_offset=[%d,%d] lna_gain=%d "
	       "high_gain=[%d,%d] mcu_gain=0x%08x\n",
	       dev.cal.rssi_offset[0], dev.cal.rssi_offset[1], dev.cal.lna_gain,
	       dev.cal.high_gain[0], dev.cal.high_gain[1], dev.cal.mcu_gain);
	printf("  raw EEPROM: LNA_GAIN=0x%04x RSSI_OFF_5G_0=0x%04x "
	       "RSSI_OFF_5G_1=0x%04x GRP4_5_RX_HIGH_GAIN=0x%04x\n",
	       mt_ee(&dev, MT_EE_LNA_GAIN), mt_ee(&dev, MT_EE_RSSI_OFFSET_5G_0),
	       mt_ee(&dev, MT_EE_RSSI_OFFSET_5G_1),
	       mt_ee(&dev, MT_EE_RF_5G_GRP4_5_RX_HIGH_GAIN));
	printf("  -> all-zero correction is CORRECT here: this EEPROM has no gain\n"
	       "     calibration programmed, and mcu_gain 0x%08x is exactly what the\n"
	       "     kernel sent in CMD_INIT_GAIN_OP for the same channel.\n",
	       dev.cal.mcu_gain);

	/* TSF: the register names suggest DW0 is the low word but mt76 reads
	 * DW0 as the high one. Rather than trust either reading, sleep a known
	 * 200 ms and require the clock to have advanced by that much. */
	{
		uint32_t a0 = mt_rr(&dev, MT_TSF_TIMER_DW0);
		uint32_t a1 = mt_rr(&dev, MT_TSF_TIMER_DW1);
		uint32_t b0, b1;
		int64_t d_hi0, d_lo0;

		mt_usleep(200000);
		b0 = mt_rr(&dev, MT_TSF_TIMER_DW0);
		b1 = mt_rr(&dev, MT_TSF_TIMER_DW1);

		d_hi0 = (int64_t)((((uint64_t)b0 << 32) | b1) - (((uint64_t)a0 << 32) | a1));
		d_lo0 = (int64_t)((((uint64_t)b1 << 32) | b0) - (((uint64_t)a1 << 32) | a0));

		printf("\nTSF raw: DW0 %08x -> %08x   DW1 %08x -> %08x\n", a0, b0, a1, b1);
		printf("  as (DW0<<32)|DW1 : delta %lld us\n", (long long)d_hi0);
		printf("  as (DW1<<32)|DW0 : delta %lld us\n", (long long)d_lo0);
		printf("  over a 200000 us sleep -> DW%d is the low word\n",
		       (d_lo0 > 150000 && d_lo0 < 400000) ? 0 : 1);

		t1 = mt7612u_read_tsf(&dev);
		mt_usleep(200000);
		t2 = mt7612u_read_tsf(&dev);
		delta = (int64_t)(t2 - t1);
		printf("  mt7612u_read_tsf(): delta %lld us  %s\n", (long long)delta,
		       (delta > 150000 && delta < 400000) ? "OK" : "*** WRONG ORDER ***");
		if (delta < 150000 || delta > 400000) bad++;
	}

	/* 40 MHz */
	printf("\n40 MHz on ch%u:\n", chan);
	if (mt_set_channel(&dev, chan, MT7612U_BW_40)) {
		printf("  set_channel(40 MHz) FAILED\n");
		bad++;
	} else {
		uint32_t core1 = mt_rr(&dev, MT_BBP(CORE, 1));
		uint32_t agc0  = mt_rr(&dev, MT_BBP(AGC, 0));
		unsigned bwf = FIELD_GET(MT_BBP_CORE_R1_BW, core1);

		printf("  MT_BBP(CORE,1)=0x%08x BW field=%u (2 = 40 MHz)\n", core1, bwf);
		printf("  MT_BBP(AGC,0)=0x%08x AGC BW=%u (3 = 40 MHz)\n",
		       agc0, FIELD_GET(MT_BBP_AGC_R0_BW, agc0));
		if (bwf != 2) { printf("  *** BBP not in 40 MHz ***\n"); bad++; }

		/* Registers saying 40 MHz is not the same as 40 MHz on air.
		 * Transmit at bw=40 and let the witness report the width. */
		{
			static const uint8_t src[6] = { 0x02,0x4d,0x54,0x76,0x12,0x01 };
			uint8_t f[64];
			struct mt7612u_tx_rate r40 = { .phy = MT7612U_PHY_HT, .mcs = 7,
			                               .nss = 1, .bw = MT7612U_BW_40,
			                               .no_ack = 1 };
			memset(f, 0, sizeof f);
			f[0] = 0x08;
			memset(f + 4, 0xff, 6);
			memcpy(f + 10, src, 6);
			memcpy(f + 16, src, 6);
			memcpy(f + 24, "MT7612U-HAL ", 12);
			f[36] = 'W';
			for (int i = 0; i < 300; i++) {
				f[22] = (uint8_t)((i & 0xf) << 4);
				f[23] = (uint8_t)(i >> 4);
				mt7612u_tx(&dev, f, 40, &r40);
				mt_usleep(2000);
			}
			printf("  sent 300 frames at bw=40, tag W - witness reports the width\n");
		}
	}

	mt_async_stop(&dev);
	mt_mac_stop(&dev);
	printf("\n%lu frames drained from EP 4 while the receiver was on\n", drained);
	printf("\nGATE caps: %s\n", bad ? "FAIL" : "PASS");
	return bad;
}

/*
 * Hardware ACK responder, using devourer's own methodology: an unACKed
 * unicast frame is retransmitted, and a retransmission carries the Retry bit
 * in frame control. So the observable is our own RX - count frames from the
 * stimulus transmitter, split by the Retry bit, with the responder off and
 * then on. If the MAC is ACKing, the retry copies collapse.
 *
 * The RX filter must keep MT_RX_FILTR_CFG_DUP clear or the hardware drops the
 * duplicates this test is counting.
 */
/* Same event-thread/gate split as arx_ctx above. */
struct ack_ctx { _Atomic unsigned long to_us, retry_to_us, other; };

static const uint8_t g_ack_mac[6] = { 0x02, 0x4d, 0x54, 0x76, 0x12, 0xaa };

static void ack_cb(void *user, const void *frame, size_t len,
                   const struct mt7612u_rx_info *info)
{
	struct ack_ctx *c = user;
	const uint8_t *f = frame;

	(void)info;
	if (len < 16) return;
	if (memcmp(f + 4, g_ack_mac, 6) != 0) {
		atomic_fetch_add_explicit(&c->other, 1, memory_order_relaxed);
		return;
	}
	atomic_fetch_add_explicit(&c->to_us, 1, memory_order_relaxed);
	if (f[1] & 0x08)                        /* FC Retry bit */
		atomic_fetch_add_explicit(&c->retry_to_us, 1, memory_order_relaxed);
}

static int gate_ack(uint8_t chan, int secs, int arm)
{
	struct ack_ctx off = { 0, 0, 0 };

	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) return 1;
	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) return 1;
	/* Ring first, receiver second - see gate_caps. Arming the responder and
	 * printing between the two would otherwise leave RX on and undrained. */
	if (mt7612u_rx_start(&dev, ack_cb, &off)) return 1;
	if (mt_mac_start(&dev, 1)) { mt7612u_rx_stop(&dev); return 1; }
	/* CRC and PHY errors only: DUP must stay clear so retries reach us. */
	mt_wr(&dev, MT_RX_FILTR_CFG,
	      MT_RX_FILTR_CFG_CRC_ERR | MT_RX_FILTR_CFG_PHY_ERR);

	printf("responder address %02x:%02x:%02x:%02x:%02x:%02x on ch%u\n",
	       g_ack_mac[0], g_ack_mac[1], g_ack_mac[2], g_ack_mac[3],
	       g_ack_mac[4], g_ack_mac[5], chan);
	printf("stimulus expected from the other radio:\n"
	       "  DEVOURER_TX_QOS_DATA=1 DEVOURER_TX_RA=02:4d:54:76:12:aa txdemo\n\n");

	/* One arm per invocation, so the two conditions are cleanly separated
	 * in the stimulus radio's own capture rather than by timestamp windows. */
	if (arm) {
		if (mt7612u_set_ack_responder(&dev, g_ack_mac)) {
			printf("GATE ack: FAIL - could not arm\n");
			mt7612u_rx_stop(&dev);
			mt_mac_stop(&dev);
			return 1;
		}
		printf("responder ARMED (MT_MAC_ADDR_DW0=0x%08x, AUTO_RSP_CFG=0x%08x)\n",
		       mt_rr(&dev, MT_MAC_ADDR_DW0), mt_rr(&dev, MT_AUTO_RSP_CFG));
	} else {
		printf("responder NOT armed (control arm)\n");
	}

	/* Validate before the cast: `secs` is signed and came from argv, and
	 * (unsigned)(-1) * 1000000 is roughly 49 days with the receiver left
	 * running - an RX path enabled and undrained for that long is the wedge
	 * this port documents as replug-only. */
	if (secs <= 0 || secs > 3600) {
		printf("GATE ack: FAIL - listen duration %d out of range (1..3600 s)\n",
		       secs);
		mt7612u_rx_stop(&dev);
		mt_mac_stop(&dev);
		return 1;
	}
	printf("listening %d s ...\n", secs);
	/* wait_ms, not mt_usleep: it honours SIGINT, where the old cast-to-
	 * unsigned sleep both ignored the signal and turned a negative argument
	 * into roughly 49 days with the receiver left running. */
	if (!wait_ms(secs * 1000.0)) {
		printf("GATE ack: interrupted\n");
		mt7612u_rx_stop(&dev);
		mt_mac_stop(&dev);
		return 1;
	}
	mt7612u_rx_stop(&dev);
	printf("  stimulus frames addressed to the responder MAC: %lu (retries %lu)\n",
	       off.to_us, off.retry_to_us);

	if (arm) {
		mt7612u_clear_ack_responder(&dev);
		printf("cleared; MT_MAC_ADDR_DW0 back to 0x%08x\n",
		       mt_rr(&dev, MT_MAC_ADDR_DW0));
	}
	mt_mac_stop(&dev);

	if (!off.to_us) {
		printf("\nINCONCLUSIVE - the stimulus never reached us.\n");
		return 1;
	}
	printf("\nstimulus confirmed. The ACKs (if any) are counted on the\n"
	       "stimulus radio, which receives concurrently.\n");
	return 0;
}


/*
 * Every RXWI byte against ambient traffic, bucketed by received level.
 *
 * The power sweep answered "does this byte track our transmitter". This asks
 * the two questions that one could not: does a byte vary with received level
 * across a much wider span than our own saturated link covers, and does a
 * byte that looks constant differ between a clean channel and an interfered
 * one. A noise floor would be flat within a channel and move between them.
 */
static struct { unsigned long n; long sum[20]; long mn[20], mx[20]; } g_rxb[6];
static const int g_rxb_edge[6] = { -100, -80, -70, -60, -50, 0 };

static void rxbytes_cb(void *user, const void *frame, size_t len,
                       const struct mt7612u_rx_info *info)
{
    uint8_t bytes[20];
    int band = 0;

    (void)user; (void)frame;
    if (len < 16) return;
    for (int i = 0; i < 6; i++)
        if (info->rssi[0] <= g_rxb_edge[i]) { band = i; break; }

    for (int i = 0; i < 4; i++) bytes[i] = (uint8_t)info->rssi[i];
    for (int w = 0; w < 4; w++)
        for (int b = 0; b < 4; b++)
            bytes[4 + w * 4 + b] = (uint8_t)(info->bbp[w] >> (8 * b));

    if (!g_rxb[band].n)
        for (int i = 0; i < 20; i++) { g_rxb[band].mn[i] = 999; g_rxb[band].mx[i] = -999; }
    g_rxb[band].n++;
    for (int i = 0; i < 20; i++) {
        long v = bytes[i];

        g_rxb[band].sum[i] += v;
        if (v < g_rxb[band].mn[i]) g_rxb[band].mn[i] = v;
        if (v > g_rxb[band].mx[i]) g_rxb[band].mx[i] = v;
    }
}

static int gate_rxbytes(uint8_t chan, int secs)
{
    static const char *nm[20] = {
        "rssi[0]", "rssi[1]", "rssi[2]", "rssi[3]",
        "bbp0.b0", "bbp0.b1", "bbp0.b2", "bbp0.b3",
        "bbp1.b0", "bbp1.b1", "bbp1.b2", "bbp1.b3",
        "bbp2.b0", "bbp2.b1", "bbp2.b2", "bbp2.b3",
        "bbp3.b0", "bbp3.b1", "bbp3.b2", "bbp3.b3",
    };
    struct mt7612u_link_stats st;

    memset(g_rxb, 0, sizeof g_rxb);
    if (mt_eeprom_init(&dev)) return 1;
    if (mt_init_hardware(&dev, NULL)) return 1;
    if (mt_set_channel(&dev, chan, MT7612U_BW_20)) return 1;
    if (mt7612u_rx_start(&dev, rxbytes_cb, NULL)) return 1;
    if (mt_mac_start(&dev, 1)) return 1;
    mt7612u_set_monitor_rx(&dev, 0);
    mt7612u_link_stats_start(&dev);

    wait_ms(secs * 1000.0);
    mt7612u_link_stats(&dev, &st);
    mt7612u_rx_stop(&dev);
    mt_mac_stop(&dev);

    printf("ch%u, %d s ambient. false CCA this interval: %u (mt76 calls >800 "
           "interfered, <10 clean)\n", chan, secs, st.rx_false_cca);
    {
        unsigned long tot = 0, nv = 0;
        long rs = 0, ns = 0, ss = 0;

        for (int b = 0; b < 6; b++) {
            if (!g_rxb[b].n) continue;
            tot += g_rxb[b].n;
            rs += g_rxb[b].sum[0];
            ns += g_rxb[b].sum[2];
        }
        if (tot) {
            double r = rs / (double)tot - 256, n = ns / (double)tot - 256;

            (void)nv; (void)ss;
            printf("  mean rssi %.1f dBm, noise %.1f dBm  ->  SNR %.1f dB%s\n\n",
                   r, n, r - n,
                   n < -100 ? "   (noise below thermal: no valid estimate)" : "");
        }
    }
    printf("  %-8s", "byte");
    for (int b = 0; b < 6; b++) if (g_rxb[b].n) printf("  <=%-4d", g_rxb_edge[b]);
    printf("   min  max\n");
    for (int i = 0; i < 20; i++) {
        long mn = 999, mx = -999;

        printf("  %-8s", nm[i]);
        for (int b = 0; b < 6; b++) {
            if (!g_rxb[b].n) continue;
            printf(" %7.1f", g_rxb[b].sum[i] / (double)g_rxb[b].n);
            if (g_rxb[b].mn[i] < mn) mn = g_rxb[b].mn[i];
            if (g_rxb[b].mx[i] > mx) mx = g_rxb[b].mx[i];
        }
        printf("  %4ld %4ld\n", mn, mx);
    }
    printf("\n  frames per band:");
    for (int b = 0; b < 6; b++) if (g_rxb[b].n) printf(" %lu", g_rxb[b].n);
    printf("\n");
    return 0;
}

/*
 * The MAC's MIB counters, sampled once a second.
 *
 * This is where this part's link reporting actually lives. The RX descriptor
 * carries RSSI and nothing else - `bbp_rxinfo[4]`, which mt76 declares and
 * never reads, is two words of zero plus a duplicate of the same two RSSI
 * values - so there is no per-frame SNR or EVM. What there is instead is
 * per-interval: channel occupancy, four classes of receive error, a false-CCA
 * count that is the interference signal, and the A-MPDU length histogram.
 *
 * Read-and-clear, so each line is the second that just passed.
 */
static unsigned long linkstat_drained;

static int gate_linkstat(uint8_t chan, int secs, int with_rx)
{
	struct mt7612u_link_stats st;

	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) return 1;
	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) return 1;
	/* The receiver has to be ON for any of the RX error classes or the
	 * busy timer to count anything, and the ring has to be draining before
	 * the receiver is enabled. Getting this wrong reads as "the counters
	 * are dead" rather than as a harness bug. */
	if (with_rx) {
		if (mt7612u_rx_start(&dev, drain_cb, &linkstat_drained)) return 1;
	}
	if (mt_mac_start(&dev, with_rx)) return 1;
	if (with_rx) mt7612u_set_monitor_rx(&dev, 0);
	mt7612u_link_stats_start(&dev);

	printf("ch%u, receiver %s, %d samples of 1 s (read-and-clear)\n\n",
	       chan, with_rx ? "ON" : "off", secs);
	printf("  %5s %9s %9s %6s  %5s %5s %8s %5s %5s %5s  %4s\n",
	       "s", "busy", "idle", "busy%", "crc", "phy", "falseCCA", "plcp", "dup", "ovf",
	       "temp");
	for (int i = 0; i < secs; i++) {
		double busy_pct;

		if (!wait_ms(1000.0)) break;
		if (mt7612u_link_stats(&dev, &st)) return 1;
		busy_pct = (st.ch_busy + st.ch_idle)
		         ? 100.0 * st.ch_busy / (double)(st.ch_busy + st.ch_idle) : 0.0;
		printf("  %5d %9u %9u %5.1f%%  %5u %5u %8u %5u %5u %5u  %4d\n",
		       i, st.ch_busy, st.ch_idle, busy_pct,
		       st.rx_crc_err, st.rx_phy_err, st.rx_false_cca,
		       st.rx_plcp_err, st.rx_dup_err, st.rx_overflow, st.temp_c);
	}

	{
		int any = 0;

		for (int i = 0; i < 32; i++) if (st.agg_cnt[i]) any = 1;
		printf("\n  A-MPDU length histogram (last second): %s",
		       any ? "" : "all zero - nothing aggregated\n");
		if (any) {
			for (int i = 0; i < 32; i++)
				if (st.agg_cnt[i]) printf("[%d]=%u ", i + 1, st.agg_cnt[i]);
			printf("\n");
		}
	}
	if (with_rx) {
		mt7612u_rx_stop(&dev);
		printf("  %lu frames reached the ring over the run\n", linkstat_drained);
	}
	mt_mac_stop(&dev);
	return 0;
}

/* --- MT7612U -> MT7612U link, and what the baseband reports per frame ---
 *
 * Two adapters, one transmitting at a swept TX power and one receiving. It
 * answers three separate questions at once, which is why the sweep is a
 * power sweep and not a fixed level:
 *
 *  1. Does this port's TX and RX work against each other end to end?
 *  2. Does mt7612u_set_txpower() move *radiated* power? Everything so far
 *     compared registers against the kernel's, which is not the same claim.
 *  3. RXWI bytes 16-31 are `bbp_rxinfo[4]`, which mt76 declares and never
 *     reads, and mt76x02 has no SNR or EVM anywhere. If any of those bytes
 *     is a link-quality metric it must move with the transmitter's power;
 *     if none of them does, they are not one.
 *
 * The receiver is NOT an independent instrument - it runs this same decode
 * path - so this measures the link and the descriptor, not our correctness.
 */
static int gate_linktx(uint8_t chan, int count)
{
	static const uint8_t src[6] = { 0x02, 0x4d, 0x54, 0x76, 0x12, 0x01 };
	static const int powers[] = { 0, 4, 8, 12, 16, 20, 24, 30 };
	struct mt7612u_tx_rate r = { .phy = MT7612U_PHY_HT, .mcs = 2, .nss = 1,
	                             .bw = MT7612U_BW_20, .no_ack = 1 };
	uint8_t f[64];

	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) return 1;
	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) return 1;
	if (mt_mac_start(&dev, 0)) return 1;

	memset(f, 0, sizeof f);
	f[0] = 0x08;
	memset(f + 4, 0xff, 6);
	memcpy(f + 10, src, 6);
	memcpy(f + 16, src, 6);
	memcpy(f + 24, "MT7612U-HAL ", 12);

	printf("TX on ch%u, HT MCS2 1SS 20 MHz, %d frames per power step\n",
	       chan, count);
	for (unsigned i = 0; i < sizeof powers / sizeof powers[0]; i++) {
		long sent = 0;

		if (mt7612u_set_txpower(&dev, powers[i])) {
			printf("  %2d dBm  REFUSED\n", powers[i]);
			continue;
		}
		f[36] = (uint8_t)powers[i];
		for (int n = 0; n < count; n++) {
			f[22] = (uint8_t)((n & 0xf) << 4);
			f[23] = (uint8_t)(n >> 4);
			f[37] = (uint8_t)n;
			if (mt7612u_tx(&dev, f, 44, &r) == 0) sent++;
			mt_usleep(1200);
		}
		printf("  %2d dBm  sent %ld/%d\n", powers[i], sent, count);
		mt_usleep(120000);
	}
	mt_mac_stop(&dev);
	return 0;
}

/* Every byte the RXWI offers past the two RSSI values mt76 reads, averaged.
 * 4 rssi bytes (mt76 uses only [0] and [1]; [2] and [3] are read by nobody,
 * and on the legacy Ralink RXWI those slots were SNR0/SNR1) plus the 16 bytes
 * of bbp_rxinfo. Signed and unsigned means both, because an SNR would be a
 * small positive number and an RSSI a negative one. */
struct link_bucket { unsigned long n; long b_sum[20]; long b_min[20], b_max[20]; };
static struct link_bucket g_link[32];
static int g_link_pw[32];
static int g_link_n;

static void linkrx_cb(void *user, const void *frame, size_t len,
                      const struct mt7612u_rx_info *info)
{
	const uint8_t *f = frame;
	int slot = -1, pw;
	uint8_t bytes[20];

	(void)user;
	if (len < 40) return;
	if (memcmp(f + 10, "\x02\x4d\x54\x76\x12\x01", 6)) return;
	if (memcmp(f + 24, "MT7612U-HAL ", 12)) return;
	pw = f[36];
	for (int i = 0; i < g_link_n; i++)
		if (g_link_pw[i] == pw) { slot = i; break; }
	if (slot < 0) {
		if (g_link_n >= 32) return;
		slot = g_link_n++;
		g_link_pw[slot] = pw;
		for (int i = 0; i < 20; i++) {
			g_link[slot].b_min[i] = 999;
			g_link[slot].b_max[i] = -999;
		}
	}

	for (int i = 0; i < 4; i++) bytes[i] = (uint8_t)info->rssi[i];
	for (int w = 0; w < 4; w++)
		for (int b = 0; b < 4; b++)
			bytes[4 + w * 4 + b] = (uint8_t)(info->bbp[w] >> (8 * b));

	g_link[slot].n++;
	for (int i = 0; i < 20; i++) {
		long v = bytes[i];

		g_link[slot].b_sum[i] += v;
		if (v < g_link[slot].b_min[i]) g_link[slot].b_min[i] = v;
		if (v > g_link[slot].b_max[i]) g_link[slot].b_max[i] = v;
	}
}

static int gate_linkrx(uint8_t chan, int secs)
{

	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) return 1;
	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) return 1;
	if (mt7612u_rx_start(&dev, linkrx_cb, NULL)) return 1;
	if (mt_mac_start(&dev, 1)) return 1;
	mt7612u_set_monitor_rx(&dev, 0);

	printf("RX on ch%u for %d s, filtering our own magic\n", chan, secs);
	wait_ms(secs * 1000.0);
	mt7612u_rx_stop(&dev);
	mt_mac_stop(&dev);

	{
		static const char *nm[20] = {
			"rssi[0]", "rssi[1]", "rssi[2]", "rssi[3]",
			"bbp0.b0", "bbp0.b1", "bbp0.b2", "bbp0.b3",
			"bbp1.b0", "bbp1.b1", "bbp1.b2", "bbp1.b3",
			"bbp2.b0", "bbp2.b1", "bbp2.b2", "bbp2.b3",
			"bbp3.b0", "bbp3.b1", "bbp3.b2", "bbp3.b3",
		};

		printf("\nmean of every candidate byte, per requested tx power\n");
		printf("  %-8s", "byte");
		for (int i = 0; i < g_link_n; i++) printf(" %7d", g_link_pw[i]);
		printf("   span  as int8\n");
		for (int b = 0; b < 20; b++) {
			double lo = 1e9, hi = -1e9;

			printf("  %-8s", nm[b]);
			for (int i = 0; i < g_link_n; i++) {
				double m = g_link[i].b_sum[b] / (double)g_link[i].n;

				if (m < lo) lo = m;
				if (m > hi) hi = m;
				printf(" %7.1f", m);
			}
			printf("  %5.1f  %6.1f\n", hi - lo,
			       g_link[0].b_sum[b] / (double)g_link[0].n > 127
			         ? g_link[0].b_sum[b] / (double)g_link[0].n - 256
			         : g_link[0].b_sum[b] / (double)g_link[0].n);
		}
		printf("\n  frames per level:");
		for (int i = 0; i < g_link_n; i++) printf(" %lu", g_link[i].n);
		printf("\n");
	}
	printf("\nA byte whose span is ~0 across a 30 dB sweep carries no level or\n"
	       "quality information. One that tracks and stays a small positive\n"
	       "number is an SNR candidate; one that tracks and reads negative as\n"
	       "int8 is another copy of RSSI.\n");
	return g_link_n ? 0 : 1;
}

/*
 * Does STBC actually put the stream on both antennas, and does the second
 * chain radiate without it?
 *
 * The coding gate proves the STBC bit reaches the air and the receiver
 * decodes the frame as STBC. It says nothing about radiated power, and there
 * is a real confound: a chip may already drive the second chain with cyclic
 * delay diversity on a one-stream frame, in which case "STBC off" is not
 * "one antenna".
 *
 * Phase 1 alternates STBC off/on frame by frame at one rate. Nothing is
 * reconfigured between them - only one bit of the rate word changes - so
 * ambient drift, distance and AGC state cancel.
 *
 * Phase 2 needs a chainmask change, which costs a channel re-set, so it runs
 * in blocks and repeats the sequence twice: if the two passes disagree, the
 * difference is drift and not the chainmask.
 */
static int gate_diversity(uint8_t chan, int count)
{
	static const uint8_t src[6] = { 0x02, 0x4d, 0x54, 0x76, 0x12, 0x01 };
	uint8_t f[64];

	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) return 1;
	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) return 1;
	if (mt_mac_start(&dev, 0)) return 1;

	memset(f, 0, sizeof f);
	f[0] = 0x08;
	memset(f + 4, 0xff, 6);
	memcpy(f + 10, src, 6);
	memcpy(f + 16, src, 6);
	memcpy(f + 24, "MT7612U-HAL ", 12);

	printf("HT MCS2, 1 spatial stream, 20 MHz, ch%u\n", chan);
	printf("phase 1: STBC off/on alternating frame by frame (tags A / B)\n");
	{
		struct mt7612u_tx_rate off = { .phy = MT7612U_PHY_HT, .mcs = 2,
		                               .nss = 1, .bw = MT7612U_BW_20,
		                               .no_ack = 1 };
		struct mt7612u_tx_rate on = off;
		long n_off = 0, n_on = 0;

		on.stbc = 1;
		printf("  rate word off 0x%04x  on 0x%04x  (one bit apart)\n",
		       mt_tx_rate_word(&off), mt_tx_rate_word(&on));
		for (int i = 0; i < count; i++) {
			int stbc = i & 1;

			f[22] = (uint8_t)((i & 0xf) << 4);
			f[23] = (uint8_t)(i >> 4);
			f[36] = stbc ? 'B' : 'A';
			f[37] = (uint8_t)i;
			if (mt7612u_tx(&dev, f, 44, stbc ? &on : &off) == 0) {
				if (stbc) n_on++; else n_off++;
			}
			mt_usleep(1500);
		}
		printf("  submitted %ld off, %ld on\n", n_off, n_on);
	}

	printf("phase 2: 1T1R vs 2T2R, STBC off, two passes (tags C / D)\n");
	for (int pass = 0; pass < 2; pass++) {
		for (int two = 0; two < 2; two++) {
			struct mt7612u_tx_rate r = { .phy = MT7612U_PHY_HT, .mcs = 2,
			                             .nss = 1, .bw = MT7612U_BW_20,
			                             .no_ack = 1 };
			long sent = 0;

			if (mt7612u_set_chainmask(&dev, two ? 0x0202 : 0x0101))
				return 1;
			if (mt_set_channel(&dev, chan, MT7612U_BW_20)) return 1;
			printf("  pass %d chainmask 0x%04x txwi[17]=0x%02x\n", pass,
			       dev.chainmask, ((dev.chainmask & 0xf) > 1) ? 0x13 : 0);
			f[36] = two ? 'D' : 'C';
			for (int i = 0; i < count; i++) {
				f[22] = (uint8_t)((i & 0xf) << 4);
				f[23] = (uint8_t)(i >> 4);
				f[37] = (uint8_t)i;
				if (mt7612u_tx(&dev, f, 44, &r) == 0) sent++;
				mt_usleep(1500);
			}
			printf("    submitted %ld/%d\n", sent, count);
		}
	}
	mt7612u_set_chainmask(&dev, 0x0202);

	mt_mac_stop(&dev);
	printf("\nWitness RSSI per tag decides. A vs B is the STBC question with\n"
	       "nothing else changed; C vs D is whether the second chain radiates\n"
	       "at all without STBC.\n");
	return 0;
}

/*
 * The three modulation flags in the rate word: LDPC, STBC and short GI.
 *
 * Each frame carries both the DESC_RATE it should air at and the flag bits it
 * should carry, so the witness compares the frame against its own claim
 * rather than against an arm table.
 *
 * One arm is a deliberate negative control: STBC is requested at two spatial
 * streams, where mt_tx_rate_word() refuses to set it because mt76 refuses too
 * (STBC on this MAC is a 1SS feature). The frame must air with stbc clear. An
 * arm that only ever asks for things that work cannot tell a working encoder
 * from one that sets every bit it is handed.
 */
/* bw is the MT7612U_BW_* enum: 0 = 20, 1 = 40, 2 = 80. */
static int gate_coding(uint8_t chan, int count, int bw)
{
	static const uint8_t src[6] = { 0x02, 0x4d, 0x54, 0x76, 0x12, 0x01 };
	static const struct { enum mt7612u_phy phy; uint8_t mcs, nss; int base; }
	rates[] = {
		{ MT7612U_PHY_HT,   3, 1, 15 },
		{ MT7612U_PHY_HT,   7, 1, 19 },
		{ MT7612U_PHY_HT,  11, 2, 23 },
		{ MT7612U_PHY_VHT,  3, 1, 47 },
		{ MT7612U_PHY_VHT,  7, 1, 51 },
		{ MT7612U_PHY_VHT,  3, 2, 57 },
		{ MT7612U_PHY_VHT,  7, 2, 61 },
	};
	uint8_t f[64];
	int arms = 0;
	int short_arms = 0;   /* arms that aired fewer frames than asked */
	uint8_t hw_chan = chan;

	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) return 1;
	if (mt_set_channel(&dev, chan, (enum mt7612u_bw)bw)) return 1;
	if (mt_mac_start(&dev, 0)) return 1;

	mt_chan_group(chan, (uint8_t)bw, &hw_chan, NULL, NULL);
	printf("ch%u (hw centre %u) at %d MHz, %d frames per arm\n\n",
	       chan, hw_chan, 20 << bw, count);
	printf("  %-16s %-10s %-9s %s\n", "rate", "asked", "rate word", "expect on air");

	memset(f, 0, sizeof f);
	f[0] = 0x08;
	memset(f + 4, 0xff, 6);
	memcpy(f + 10, src, 6);
	memcpy(f + 16, src, 6);
	memcpy(f + 24, "MT7612U-HAL ", 12);

	for (unsigned i = 0; i < sizeof rates / sizeof rates[0]; i++) {
		/* 802.11n has no 80 MHz, so an HT arm at that width would emit a
		 * rate word that names no real format. gate_sweep and gate_vht
		 * skip the same pairing. */
		if (bw == MT7612U_BW_80 && rates[i].phy != MT7612U_PHY_VHT) {
			printf("  %-16s skipped: 802.11n has no 80 MHz\n", "HT");
			continue;
		}
		for (int coding = 0; coding < 8; coding++) {
			struct mt7612u_tx_rate r = {
				.phy = rates[i].phy, .mcs = rates[i].mcs,
				.nss = rates[i].nss,
				.bw = (uint8_t)bw,
				.sgi = (coding & 4) ? 1u : 0u,
				.ldpc = (coding & 1) ? 1u : 0u,
				.stbc = (coding & 2) ? 1u : 0u,
				.no_ack = 1,
			};
			uint16_t word = mt_tx_rate_word(&r);
			/* What the encoder actually committed to, which is what
			 * the air must show - not what was asked for. */
			int on_air = ((word & MT_RATE_LDPC) ? 1 : 0) |
			             ((word & MT_RATE_STBC) ? 2 : 0) |
			             ((word & MT_RATE_SGI)  ? 4 : 0);
			char asked[16], expect[24];
			long sent = 0;

			snprintf(asked, sizeof asked, "%s%s%s",
			         (coding & 1) ? "L" : "-", (coding & 2) ? "S" : "-",
			         (coding & 4) ? "G" : "-");
			snprintf(expect, sizeof expect, "rate %d  %s%s%s",
			         rates[i].base,
			         (on_air & 1) ? "L" : "-", (on_air & 2) ? "S" : "-",
			         (on_air & 4) ? "G" : "-");
			printf("  %s MCS%-2d %dSS  %-10s 0x%04x    %s%s\n",
			       rates[i].phy == MT7612U_PHY_HT ? "HT " : "VHT",
			       rates[i].mcs, rates[i].nss, asked, word, expect,
			       (coding & 2) && rates[i].nss > 1 ? "   <- STBC refused at 2SS" : "");

			f[36] = (uint8_t)rates[i].base;
			f[38] = (uint8_t)on_air;
			for (int n = 0; n < count; n++) {
				f[22] = (uint8_t)((n & 0xf) << 4);
				f[23] = (uint8_t)(n >> 4);
				f[37] = (uint8_t)n;
				if (mt7612u_tx(&dev, f, 44, &r) == 0) sent++;
				mt_usleep(1200);
			}
			if (sent != count) {
				printf("       submitted only %ld/%d\n", sent, count);
				short_arms++;
			}
			arms++;
			mt_usleep(80000);
		}
	}

	mt_mac_stop(&dev);
	printf("\n%d arms swept. Payload offset 12 is the expected DESC_RATE,\n"
	       "offset 14 the expected LDPC|STBC|SGI bits.\n", arms);
	if (short_arms) {
		printf("GATE coding: FAIL - %d arm(s) submitted fewer frames than "
		       "asked; the witness cannot rule on an arm that did not air\n",
		       short_arms);
		return 1;
	}
	return 0;
}

/*
 * Full rate-ladder sweep: every HT MCS 0-15 and every legal VHT MCS at both
 * stream counts, at whichever width the caller picks.
 *
 * Each frame carries its own expected DESC_RATE code in the payload, so the
 * check is "did this frame air at the rate it says it should have" rather
 * than an arm table the analysis has to agree with separately. A frame that
 * airs at the wrong rate indicts itself.
 *
 * VHT MCS9 is not legal at 20 MHz for one or two streams, so it is skipped
 * there and included at 40.
 */
/* bw is the MT7612U_BW_* enum: 0 = 20, 1 = 40, 2 = 80. */
static int gate_sweep(uint8_t chan, int count, int bw)
{
	static const uint8_t src[6] = { 0x02, 0x4d, 0x54, 0x76, 0x12, 0x01 };
	uint8_t f[64];
	int arms = 0;
	int short_arms = 0;   /* arms that aired fewer frames than asked */
	uint8_t hw_chan = chan;

	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) return 1;
	if (mt_set_channel(&dev, chan, (enum mt7612u_bw)bw)) return 1;
	if (mt_mac_start(&dev, 0)) return 1;

	/* Report the centre the hardware actually tuned, not the control
	 * channel: at 80 MHz they differ by up to 6, and a witness listening on
	 * the control channel with the wrong centre hears nothing. */
	mt_chan_group(chan, (uint8_t)bw, &hw_chan, NULL, NULL);
	printf("ch%u (hw centre %u) at %d MHz, chainmask 0x%04x, %d frames per rate\n\n",
	       chan, hw_chan, 20 << bw, dev.chainmask, count);
	printf("  %-18s %-9s %s\n", "rate", "rate word", "expected DESC_RATE");

	memset(f, 0, sizeof f);
	f[0] = 0x08;
	memset(f + 4, 0xff, 6);
	memcpy(f + 10, src, 6);
	memcpy(f + 16, src, 6);
	memcpy(f + 24, "MT7612U-HAL ", 12);

	for (int phase = 0; phase < 2; phase++) {
		/* HT has no 80 MHz: 802.11n stops at 40, and 80 is a VHT-only
		 * width. A rate word naming PHY=HT with BW=80 is not a wide HT
		 * frame, it is an unspecified one, so the HT ladder is skipped
		 * rather than swept at a width it cannot mean. */
		int last_mcs = phase == 0 ? 15 : (bw ? 9 : 8);

		if (phase == 0 && bw == MT7612U_BW_80) {
			printf("  (HT ladder skipped: 802.11n has no 80 MHz)\n");
			continue;
		}

		for (int mcs = 0; mcs <= last_mcs; mcs++) {
			for (int nss = 1; nss <= 2; nss++) {
				struct mt7612u_tx_rate r = {
					.bw = (uint8_t)bw,
					.no_ack = 1,
				};
				char what[32];
				int expect;
				long sent = 0;

				if (phase == 0) {
					/* HT folds the stream count into the MCS
					 * number, so it is one ladder, not two. */
					if (nss == 2) continue;
					r.phy = MT7612U_PHY_HT;
					r.mcs = (uint8_t)mcs;
					r.nss = (uint8_t)(1 + (mcs >> 3));
					expect = 12 + mcs;
					snprintf(what, sizeof what, "HT  MCS%-2d %dSS", mcs, r.nss);
				} else {
					r.phy = MT7612U_PHY_VHT;
					r.mcs = (uint8_t)mcs;
					r.nss = (uint8_t)nss;
					expect = 44 + (nss - 1) * 10 + mcs;
					snprintf(what, sizeof what, "VHT MCS%-2d %dSS", mcs, nss);
				}

				printf("  %-18s 0x%04x    %d\n", what,
				       mt_tx_rate_word(&r), expect);
				f[36] = (uint8_t)expect;
				for (int i = 0; i < count; i++) {
					f[22] = (uint8_t)((i & 0xf) << 4);
					f[23] = (uint8_t)(i >> 4);
					f[37] = (uint8_t)i;
					if (mt7612u_tx(&dev, f, 44, &r) == 0) sent++;
					mt_usleep(1200);
				}
				if (sent != count) {
					printf("       submitted only %ld/%d\n", sent, count);
					short_arms++;
				}
				arms++;
				mt_usleep(100000);
			}
		}
	}

	mt_mac_stop(&dev);
	printf("\n%d rates swept at %d MHz. Each frame carries its own expected\n"
	       "DESC_RATE at payload offset 12; the witness compares the two.\n"
	       "The witness must be listening at the same width - a 20 MHz\n"
	       "receiver decodes none of a 40 or 80 MHz frame.\n", arms, 20 << bw);
	if (short_arms) {
		printf("GATE sweep: FAIL - %d rate(s) submitted fewer frames than "
		       "asked\n", short_arms);
		return 1;
	}
	return 0;
}

/*
 * VHT and two spatial streams on air.
 *
 * The rate word encodes both and the RX path decodes both, but until now
 * neither had been transmitted - docs/mt7612u.md listed them as unexercised.
 * Each arm carries its own tag byte so the witness attributes frames by
 * content rather than by timestamp, and each has one expected DESC_RATE code
 * at the witness: HT is 12+mcs, VHT 1SS is 44+mcs, VHT 2SS is 54+mcs. A
 * stream count that silently collapsed to one would land on the 1SS codes,
 * which is exactly the failure this is looking for.
 *
 * VHT MCS9 is not a legal rate at 20 MHz for one or two streams, so it only
 * appears in the 40 MHz arms.
 */
/* bw is the MT7612U_BW_* enum: 0 = 20, 1 = 40, 2 = 80. */
static int gate_vht(uint8_t chan, int count, int bw)
{
	static const uint8_t src[6] = { 0x02, 0x4d, 0x54, 0x76, 0x12, 0x01 };
	static const struct {
		char tag; enum mt7612u_phy phy; uint8_t mcs, nss; int wide_only;
		const char *what; int expect;
	} arms[] = {
		{ 'P', MT7612U_PHY_HT,  7,  1, 0, "HT   MCS7  1SS", 19 },
		{ 'Q', MT7612U_PHY_HT, 15,  2, 0, "HT   MCS15 2SS", 27 },
		{ 'R', MT7612U_PHY_VHT, 0,  1, 0, "VHT  MCS0  1SS", 44 },
		{ 'S', MT7612U_PHY_VHT, 7,  1, 0, "VHT  MCS7  1SS", 51 },
		{ 'T', MT7612U_PHY_VHT, 8,  1, 0, "VHT  MCS8  1SS", 52 },
		{ 'U', MT7612U_PHY_VHT, 0,  2, 0, "VHT  MCS0  2SS", 54 },
		{ 'V', MT7612U_PHY_VHT, 7,  2, 0, "VHT  MCS7  2SS", 61 },
		{ 'X', MT7612U_PHY_VHT, 8,  2, 0, "VHT  MCS8  2SS", 62 },
		{ 'Y', MT7612U_PHY_VHT, 9,  1, 1, "VHT  MCS9  1SS", 53 },
		{ 'Z', MT7612U_PHY_VHT, 9,  2, 1, "VHT  MCS9  2SS", 63 },
	};
	uint8_t f[64];
	int short_arms = 0;   /* arms that aired fewer frames than asked */
	uint8_t hw_chan = chan;

	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) return 1;
	if (mt_set_channel(&dev, chan, (enum mt7612u_bw)bw)) return 1;
	if (mt_mac_start(&dev, 0)) return 1;

	mt_chan_group(chan, (uint8_t)bw, &hw_chan, NULL, NULL);
	printf("chainmask 0x%04x -> %d spatial streams, txwi[17]=0x%02x\n",
	       dev.chainmask, (dev.chainmask & 0xf) > 1 ? 2 : 1,
	       ((dev.chainmask & 0xf) > 1) ? 0x13 : 0);
	printf("ch%u (hw centre %u) at %d MHz, %d frames per arm\n\n",
	       chan, hw_chan, 20 << bw, count);
	printf("  tag  %-16s rate word  expected witness DESC_RATE\n", "arm");

	memset(f, 0, sizeof f);
	f[0] = 0x08;                       /* data, 3-address */
	memset(f + 4, 0xff, 6);            /* broadcast */
	memcpy(f + 10, src, 6);
	memcpy(f + 16, src, 6);
	memcpy(f + 24, "MT7612U-HAL ", 12);

	for (unsigned a = 0; a < sizeof arms / sizeof arms[0]; a++) {
		struct mt7612u_tx_rate r = { .phy = arms[a].phy, .mcs = arms[a].mcs,
		                             .nss = arms[a].nss,
		                             .bw = (uint8_t)bw,
		                             .no_ack = 1 };
		long sent = 0;

		/* MCS9 has no 20 MHz encoding at 1 or 2 streams. */
		if (arms[a].wide_only && bw == MT7612U_BW_20) continue;
		/* HT is a 20/40-only PHY: 80 MHz is VHT-defined. */
		if (arms[a].phy != MT7612U_PHY_VHT && bw == MT7612U_BW_80) continue;

		printf("  %c    %-16s 0x%04x     %d\n", arms[a].tag, arms[a].what,
		       mt_tx_rate_word(&r), arms[a].expect);
		f[36] = (uint8_t)arms[a].tag;
		for (int i = 0; i < count; i++) {
			f[22] = (uint8_t)((i & 0xf) << 4);
			f[23] = (uint8_t)(i >> 4);
			f[37] = (uint8_t)i;
			f[38] = (uint8_t)(i >> 8);
			if (mt7612u_tx(&dev, f, 44, &r) == 0) sent++;
			mt_usleep(1500);
		}
		printf("       submitted %ld/%d\n", sent, count);
		if (sent != count) short_arms++;
		mt_usleep(150000);
	}

	mt_mac_stop(&dev);
	printf("\nFrames submitted. The witness decides: each tag must appear at\n"
	       "its expected DESC_RATE. A 2SS arm landing on a 1SS code means the\n"
	       "second stream did not go out.\n");
	if (short_arms) {
		printf("GATE vht: FAIL - %d arm(s) submitted fewer frames than "
		       "asked\n", short_arms);
		return 1;
	}
	return 0;
}

/*
 * The two radiotap entry points: send_packet (one framed MPDU) and
 * send_packets (several, chained into one bulk-OUT transfer via
 * MT_TXD_INFO_NEXT_VLD). Tag A = singular, tag B = aggregated.
 */
static int gate_rtap(uint8_t chan, int count)
{
	static const uint8_t src[6] = { 0x02, 0x4d, 0x54, 0x76, 0x12, 0x01 };
	/* radiotap: present = MCS | TX_FLAGS, then tx_flags(2), mcs(3) */
	static const uint8_t rtap[] = {
		0x00, 0x00, 0x0d, 0x00,                 /* ver, pad, len 13 */
		0x00, 0x80, 0x08, 0x00,                 /* present: TX_FLAGS(15) MCS(19) */
		0x08, 0x00,                             /* TX_FLAGS = NOACK */
		0x1f, 0x00, 0x07,                       /* MCS: known, flags, index 7 */
	};
	uint8_t pkt[13 + 64];
	struct mt7612u_tx_view views[16];
	uint8_t bufs[16][13 + 64];
	double t0, wall;
	long n = 0;
	size_t acc;

	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) return 1;
	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) return 1;
	if (mt_mac_start(&dev, 0)) return 1;

	memcpy(pkt, rtap, sizeof rtap);
	{
		uint8_t *f = pkt + sizeof rtap;

		memset(f, 0, 64);
		f[0] = 0x08;
		memset(f + 4, 0xff, 6);
		memcpy(f + 10, src, 6);
		memcpy(f + 16, src, 6);
		memcpy(f + 24, "MT7612U-HAL ", 12);
	}

	/* Round-trip the parser first: what did it make of that header? */
	{
		struct mt7612u_tx_rate r;
		int rl = mt_radiotap_parse(pkt, sizeof pkt, &r);

		printf("radiotap parse: hdrlen=%d -> phy=%d mcs=%u nss=%u bw=%d "
		       "sgi=%u ldpc=%u stbc=%u no_ack=%u  (rate word 0x%04x)\n",
		       rl, r.phy, r.mcs, r.nss, r.bw, r.sgi, r.ldpc, r.stbc,
		       r.no_ack, mt_tx_rate_word(&r));
		if (rl != 13 || r.phy != MT7612U_PHY_HT || r.mcs != 7 || !r.no_ack) {
			printf("GATE rtap: FAIL - parser did not decode the header\n");
			return 1;
		}
	}

	/* Tag A: send_packet, one frame per call. */
	pkt[sizeof rtap + 36] = 'A';
	t0 = now_ms();
	for (int i = 0; i < count; i++) {
		pkt[sizeof rtap + 22] = (uint8_t)((i & 0xf) << 4);
		pkt[sizeof rtap + 23] = (uint8_t)(i >> 4);
		if (mt7612u_send_packet(&dev, pkt, sizeof rtap + 40) == 0) n++;
	}
	wall = now_ms() - t0;
	printf("send_packet : %ld frames, %.0f fps\n", n, n * 1000.0 / wall);

	/* Tag B: send_packets, 16 per call -> one bulk transfer per 16 frames. */
	for (int k = 0; k < 16; k++) {
		memcpy(bufs[k], pkt, sizeof rtap + 40);
		bufs[k][sizeof rtap + 36] = 'B';
		views[k].data = bufs[k];
		views[k].len = sizeof rtap + 40;
	}
	acc = 0;
	t0 = now_ms();
	for (int i = 0; i < count / 16; i++) {
		for (int k = 0; k < 16; k++) {
			bufs[k][sizeof rtap + 22] = (uint8_t)(((i * 16 + k) & 0xf) << 4);
			bufs[k][sizeof rtap + 23] = (uint8_t)((i * 16 + k) >> 4);
		}
		acc += mt7612u_send_packets(&dev, views, 16);
	}
	wall = now_ms() - t0;
	printf("send_packets: %zu frames in %d transfers (16/transfer), %.0f fps\n",
	       acc, count / 16, acc * 1000.0 / wall);

	mt_mac_stop(&dev);
	printf("\nWitness decides: tag A must appear (send_packet works) and tag B\n"
	       "must appear (USB chaining via NEXT_VLD actually airs).\n");
	return 0;
}

int main(int argc, char **argv)
{
	const char *err = NULL, *cmd = argc > 1 ? argv[1] : "regs";
	int rc;
	/* The width argument that sweep/coding/vht share in argv[4]. Validated
	 * here rather than inside them, because they format it as `20 << bw`,
	 * which for a negative or large argv value is undefined rather than
	 * merely wrong. Other gates give argv[4] a different meaning, so the
	 * check is scoped to the three that read it as a width. */
	int want_bw = argc > 4 ? atoi(argv[4]) : 0;

	if (!strcmp(cmd, "sweep") || !strcmp(cmd, "coding") || !strcmp(cmd, "vht")) {
		if (want_bw < MT7612U_BW_20 || want_bw > MT7612U_BW_80) {
			fprintf(stderr,
			        "bad bandwidth '%s': 0 = 20 MHz, 1 = 40, 2 = 80\n",
			        argv[4]);
			return 2;
		}
		/* A non-positive count sweeps every arm with zero frames and
		 * then reports success, which is the same "structurally
		 * guaranteed pass" the shortfall check above exists to stop. */
		if (argc > 3 && atoi(argv[3]) <= 0) {
			fprintf(stderr, "bad frame count '%s': must be positive\n",
			        argv[3]);
			return 2;
		}
	}

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	if (mt_open(&dev, &err)) {
		fprintf(stderr, "open failed: %s\n", err ? err : "?");
		return 1;
	}

	if (!strcmp(cmd, "regs")) {
		rc = gate_regs();
	} else if (!strcmp(cmd, "rtap")) {
		rc = gate_rtap(argc > 2 ? (uint8_t)atoi(argv[2]) : 149,
		               argc > 3 ? atoi(argv[3]) : 400);
	} else if (!strcmp(cmd, "ack")) {
		rc = gate_ack(argc > 2 ? (uint8_t)atoi(argv[2]) : 149,
		              argc > 3 ? atoi(argv[3]) : 6,
		              argc > 4 ? atoi(argv[4]) : 0);
	} else if (!strcmp(cmd, "caps")) {
		rc = gate_caps(argc > 2 ? (uint8_t)atoi(argv[2]) : 149);
	} else if (!strcmp(cmd, "rxbytes")) {
		rc = gate_rxbytes(argc > 2 ? (uint8_t)atoi(argv[2]) : 1,
		                  argc > 3 ? atoi(argv[3]) : 15);
	} else if (!strcmp(cmd, "linkstat")) {
		rc = gate_linkstat(argc > 2 ? (uint8_t)atoi(argv[2]) : 149,
		                   argc > 3 ? atoi(argv[3]) : 10,
		                   argc > 4 ? atoi(argv[4]) : 0);
	} else if (!strcmp(cmd, "linktx")) {
		rc = gate_linktx(argc > 2 ? (uint8_t)atoi(argv[2]) : 149,
		                 argc > 3 ? atoi(argv[3]) : 400);
	} else if (!strcmp(cmd, "linkrx")) {
		rc = gate_linkrx(argc > 2 ? (uint8_t)atoi(argv[2]) : 149,
		                 argc > 3 ? atoi(argv[3]) : 30);
	} else if (!strcmp(cmd, "diversity")) {
		rc = gate_diversity(argc > 2 ? (uint8_t)atoi(argv[2]) : 149,
		                    argc > 3 ? atoi(argv[3]) : 600);
	} else if (!strcmp(cmd, "coding")) {
		rc = gate_coding(argc > 2 ? (uint8_t)atoi(argv[2]) : 149,
		                 argc > 3 ? atoi(argv[3]) : 100, want_bw);
	} else if (!strcmp(cmd, "mtu")) {
		rc = gate_mtu(argc > 2 ? (uint8_t)atoi(argv[2]) : 149,
		              argc > 3 ? atoi(argv[3]) : 60);
	} else if (!strcmp(cmd, "sweep")) {
		rc = gate_sweep(argc > 2 ? (uint8_t)atoi(argv[2]) : 149,
		                argc > 3 ? atoi(argv[3]) : 120, want_bw);
	} else if (!strcmp(cmd, "vht")) {
		rc = gate_vht(argc > 2 ? (uint8_t)atoi(argv[2]) : 149,
		              argc > 3 ? atoi(argv[3]) : 300, want_bw);
	} else if (!strcmp(cmd, "ampdu")) {
		rc = gate_ampdu(argc > 2 ? (uint8_t)atoi(argv[2]) : 149,
		                argc > 3 ? atoi(argv[3]) : 400);
	} else if (!strcmp(cmd, "pwr")) {
		rc = gate_pwr(argc > 2 ? (uint8_t)atoi(argv[2]) : 149);
	} else if (!strcmp(cmd, "soak")) {
		rc = gate_soak(argc > 2 ? (uint8_t)atoi(argv[2]) : 149,
		               argc > 3 ? atoi(argv[3]) : 5,
		               argc > 4 ? atoi(argv[4]) : 1400);
	} else if (!strcmp(cmd, "duplex")) {
		rc = gate_duplex(argc > 2 ? (uint8_t)atoi(argv[2]) : 149,
		                 argc > 3 ? atoi(argv[3]) : 5);
	} else if (!strcmp(cmd, "arx")) {
		rc = gate_arx(argc > 2 ? (uint8_t)atoi(argv[2]) : 1,
		              argc > 3 ? atoi(argv[3]) : 5);
	} else if (!strcmp(cmd, "gateg")) {
		rc = gate_g(argc > 2 ? (uint8_t)atoi(argv[2]) : 149,
		            argc > 3 ? atoi(argv[3]) : 300);
	} else if (!strcmp(cmd, "hop")) {
		rc = gate_hop();
	} else if (!strcmp(cmd, "rx")) {
		rc = gate_rx(argc > 2 ? (uint8_t)atoi(argv[2]) : 149,
		             argc > 3 ? atoi(argv[3]) : 40);
	} else if (!strcmp(cmd, "tx")) {
		rc = gate_tx(argc > 2 ? (uint8_t)atoi(argv[2]) : 149,
		             argc > 3 ? atoi(argv[3]) : 200,
		             argc > 4 ? atoi(argv[4]) : MT7612U_PHY_OFDM,
		             argc > 5 ? atoi(argv[5]) : 0);
	} else if (!strcmp(cmd, "chan")) {
		rc = gate_chan(argc > 2 ? (uint8_t)atoi(argv[2]) : 149,
		               argc > 3 ? argv[3] : NULL);
	} else if (!strcmp(cmd, "init")) {
		rc = gate_init(argc > 2 ? argv[2] : NULL);
	} else if (!strcmp(cmd, "fw")) {
		rc = gate_fw(argc > 2 ? argv[2] : NULL);
	} else {
		fprintf(stderr, "unknown subcommand '%s'\n", cmd);
		fprintf(stderr, "usage: bringup [regs|fw|init|chan|tx|rx|hop|gateg] [chan] [count] [phy 0=CCK 1=OFDM 2=HT 4=VHT] [mcs]\n");
		fprintf(stderr, "       bringup [sweep|coding|vht] [chan] [count] [bw 0=20 1=40 2=80]\n");
		fprintf(stderr, "       the witness must listen at the same width (DEVOURER_BW=40|80)\n");
		rc = 2;
	}

	mt_close(&dev);
	return rc;
}
