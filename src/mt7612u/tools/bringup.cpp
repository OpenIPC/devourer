/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * MT7612U bringup harness. One subcommand per gate (see src/mt7612u/README.md), so each
 * stage is independently runnable on hardware.
 */
#include <atomic>
#include <chrono>
#include <thread>
#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
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
#include "../Mt7612uTsfRead.h"


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

/* wait_ms() plus mt7612u_phy_tick() once a second - what every receiving
 * loop must do, see the public header.  Same return contract as wait_ms(). */
static int wait_ticking(double ms)
{
	double t0 = now_ms();

	while (now_ms() - t0 < ms) {
		double slice = ms - (now_ms() - t0);

		if (slice > 1000.0) slice = 1000.0;
		if (!wait_ms(slice)) return 0;
		mt7612u_phy_tick(&dev);
	}
	return !g_stop;
}

/*
 * RX off, then the ring.  mt_async_stop() reaps the EP 4 drainer and
 * mt_mac_stop() does not clear ENABLE_RX until after its own flush and TX-idle
 * wait, so cancelling the ring first leaves the MAC filling a receive pipe
 * nobody reads - the FIFO overflow that stops RX DMA for good, reached at the
 * end of every gate that receives.  One helper rather than the pair open-coded
 * at each of the twenty teardowns, error paths included: an invariant spelled
 * out twenty times is one that gets half-enforced.  See mt_mac_rx_disable().
 */
static void rx_teardown(void)
{
	mt_mac_rx_disable(&dev);
	mt7612u_rx_stop(&dev);
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
/*
 * The vendor RTMPSwReset() sequence, from docs/mt7612u-usb-wedge.md.
 * MT76x2U has SEPARATE UDMA TX/RX and IFDMA/FCE resets that neither mt76 nor
 * this port ever touched - CFG 0x9014 and CFG 0x0064[22:21].  Every earlier
 * failed attempt only ever hit CFG 0x9018 and MAC 0x0400, which is why they
 * could not cover every block.
 *
 * `swreset 0` observes and repairs nothing: the failing control.
 * `swreset 1` runs the sequence.  Either way the verdict is a real firmware
 * load afterwards, not an idle status register.
 */
#define CFG_UDMA_RESET   0x9014
#define CFG_UDMA_CFG     0x9018
#define CFG_EP_DROP      0x9080
#define CFG_IFDMA_RESET  0x0064
#define CFG_UDMA_TX_STAT 0x9100

static void swreset_dump(const char *when)
{
	static const uint16_t epq[] = { 0x2240, 0x2250, 0x2260, 0x2270, 0x2280, 0x2290 };
	uint32_t v;
	int idle = 1;

	printf("  %-7s U3DMA=0x%08x UDMA_RST=0x%08x EP_DROP=0x%08x IFDMA=0x%08x\n",
	       when, mt_rr(&dev, CFG_ADDR(CFG_UDMA_CFG)),
	       mt_rr(&dev, CFG_ADDR(CFG_UDMA_RESET)),
	       mt_rr(&dev, CFG_ADDR(CFG_EP_DROP)),
	       mt_rr(&dev, CFG_ADDR(CFG_IFDMA_RESET)));
	v = mt_rr(&dev, CFG_ADDR(CFG_UDMA_TX_STAT));
	printf("          UDMA_TX_STATE=0x%08x (idle=%d)  PBF=0x%08x  DESC_IDX=0x%08x\n",
	       v, (v & 0x07f00000u) == 0, mt_rr(&dev, MT_PBF_SYS_CTRL),
	       mt_rr(&dev, MT_TX_CPU_FROM_FCE_CPU_DESC_IDX));
	/* FCE TX1/TX2 fill: MAC 0x0a30/0x0a34, per docs/mt7612u-usb-wedge.md. */
	printf("          FCE_TX1=0x%08x FCE_TX2=0x%08x  EP4-9 empty:",
	       mt_rr(&dev, 0x0a30), mt_rr(&dev, 0x0a34));
	for (unsigned i = 0; i < sizeof epq / sizeof epq[0]; i++) {
		int e = !!(mt_rr(&dev, CFG_ADDR(epq[i])) & BIT(17));

		printf(" %d", e);
		if (!e) idle = 0;
	}
	printf("%s\n", idle ? "  (all empty)" : "  (NOT all empty)");
}

static void swreset_pulse(uint32_t addr, uint32_t mask)
{
	mt_set(&dev, addr, mask);
	mt_usleep(15000);
	mt_clear(&dev, addr, mask);
	mt_usleep(15000);
}

static int gate_swreset(int apply)
{
	printf("swreset: %s\n\n", apply ? "running the vendor sequence"
	                                  : "OBSERVE ONLY (failing control)");
	swreset_dump("before");

	if (apply) {
		/* The helper's surrounding contract: stop the MAC and let TX
		 * drain before touching the DMA.  Bounded - this is the fault
		 * under investigation, so it must not become an infinite wait. */
		mt_wr(&dev, MT_MAC_SYS_CTRL, 0);
		/* MT_MAC_STATUS_TX, not BIT(0) - BIT(0) is _RX.  Waiting on RX
		 * idle pulsed the UDMA TX reset with TX possibly still in
		 * flight, which is the one precondition the vendor sequence
		 * states, so a FAIL below could not separate "the sequence does
		 * not work" from "it ran too early".  Same test init.c polls,
		 * and the same 100 ms budget as the 50 x 2 ms loop it replaces. */
		int tx_idle = mt_poll(&dev, MT_MAC_STATUS, MT_MAC_STATUS_TX, 0,
		                      100000);
		printf("  MAC stopped, TX idle=%d\n", tx_idle);
		if (!tx_idle)
			printf("  TX never went idle - the sequence ran without its "
			       "precondition, so a FAIL below is about that\n");

		mt_clear(&dev, CFG_ADDR(CFG_UDMA_CFG), 0x00c00000);  /* 1 */
		swreset_pulse(CFG_ADDR(CFG_EP_DROP),    0x03f00000); /* 2 */
		swreset_pulse(CFG_ADDR(CFG_UDMA_RESET), 0x00000040); /* 3 UDMA TX */
		swreset_pulse(CFG_ADDR(CFG_IFDMA_RESET),0x00600000); /* 4 IFDMA/FCE */
		swreset_pulse(MT_PBF_SYS_CTRL,          0x0000000c); /* 5 MAC/PBF */
		swreset_pulse(CFG_ADDR(CFG_UDMA_RESET), 0x00000020); /* 6 UDMA RX */
		mt_set(&dev, CFG_ADDR(CFG_UDMA_CFG),    0x00c00000); /* 7 */
		mt_usleep(15000);
		printf("  sequence applied\n");
		swreset_dump("after");
	}

	/* The only verdict that counts: does the chip take firmware again? */
	if (mt_eeprom_init(&dev)) { printf("SWRESET: eeprom failed\n"); return 1; }
	if (mt_fw_init(&dev, NULL)) {
		printf("SWRESET: FAIL - firmware still will not load\n");
		return 1;
	}
	printf("SWRESET: PASS - firmware loaded\n");
	return 0;
}

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

/*
 * Exercise the adopt path - how a libusb-owning consumer (the IRadio
 * backend) reaches this subtree. Everything else in this tool arrives through
 * mt_open(), so without this gate the second entry point is never opened on
 * hardware at all.
 *
 * That mattered: the wedge recovery used to live inside mt_open() alone, so a
 * caller adopting a handle after a run died mid-transfer paid both mt_fw_init()
 * attempts and failed with the exact error the recovery removes. A PASS here
 * after a killed run is the evidence that the recovery is shared.
 *
 * It runs mt_adopt() + mt_init_hardware() on bringup's own device rather than
 * calling mt7612u_open_handle() - that function is exactly those two steps, and
 * driving them directly is what lets MT7612U_NO_AUTORECOVER reach this gate:
 * the public entry point allocates the device itself, so an observe-only wedge
 * experiment could never be set up through it.
 */
static int gate_adopt(const char *sel)
{
	libusb_context *ctx = NULL;
	libusb_device_handle *h = NULL;
	libusb_device **list = NULL;
	const char *err = NULL;
	ssize_t n;
	int detached = 0, matches = 0, rrc, rc = 1;

	if (libusb_init(&ctx)) { printf("ADOPT: FAIL - libusb_init\n"); return 1; }

	/* Same selector spellings open_selected() accepts - a "bus-port" like
	 * "2-1", or a bare index - so a two-adapter bench does not silently test
	 * the other unit, and MT7612U_DEV=0 does not read as "no adapter". */
	n = libusb_get_device_list(ctx, &list);
	for (ssize_t i = 0; i < n && !h; i++) {
		struct libusb_device_descriptor desc;
		uint8_t ports[8];
		char id[32], idx[8];
		int np, off;

		if (libusb_get_device_descriptor(list[i], &desc))
			continue;
		if (desc.idVendor != 0x0e8d || desc.idProduct != 0x7612)
			continue;
		off = snprintf(id, sizeof id, "%u", libusb_get_bus_number(list[i]));
		np = libusb_get_port_numbers(list[i], ports, sizeof ports);
		for (int p = 0; p < np && off > 0 && off < (int)sizeof id; p++)
			off += snprintf(id + off, sizeof id - (size_t)off, "%s%u",
			                p ? "." : "-", ports[p]);
		snprintf(idx, sizeof idx, "%d", matches);
		if (sel && *sel && strcmp(sel, id) && strcmp(sel, idx)) {
			matches++;
			continue;
		}
		if (!libusb_open(list[i], &h))
			printf("adopting the caller-owned handle at %s\n", id);
		matches++;
	}
	if (list) libusb_free_device_list(list, 1);
	if (!h) {
		printf("ADOPT: FAIL - no MT7612U%s%s\n", sel ? " at " : "", sel ? sel : "");
		libusb_exit(ctx);
		return 1;
	}

	if (libusb_kernel_driver_active(h, 0) == 1 &&
	    libusb_detach_kernel_driver(h, 0) == 0)
		detached = 1;

	/* Claim BEFORE resetting. This gate opens libusb itself - that is the
	 * point of it - so it cannot take the exclusive adapter lock that
	 * open_selected() uses, and a reset would otherwise yank the device out
	 * from under a live capture in another process and only fail afterwards.
	 * A failed claim here is that other process still holding the interface. */
	if (libusb_claim_interface(h, 0)) {
		printf("ADOPT: FAIL - interface 0 is claimed elsewhere; not resetting\n");
		goto out;
	}
	libusb_release_interface(h, 0);

	rrc = libusb_reset_device(h);
	if (rrc) {
		printf("ADOPT: FAIL - libusb_reset_device: %s\n", libusb_error_name(rrc));
		goto out;
	}
	if (libusb_claim_interface(h, 0)) {
		printf("ADOPT: FAIL - could not claim interface 0 after reset\n");
		goto out;
	}

	/* The caller owns the handle and the context; mt_adopt() records that and
	 * mt_close() then leaves both to us. */
	if (mt_adopt(&dev, h, ctx, &err)) {
		printf("ADOPT: FAIL - mt_adopt: %s\n", err ? err : "?");
		printf("  (a wedged adapter failing HERE but not via `bringup regs` is\n"
		       "   the recovery being unreachable from the adopt path)\n");
		goto out_release;
	}
	if (mt_eeprom_init(&dev) || mt_init_hardware(&dev, NULL)) {
		printf("ADOPT: FAIL - bring-up after adopt\n");
		goto out_close;
	}

	printf("U3DMA_CFG after adopt = 0x%08x (0x00c00020 = soft wedge, "
	       "0x80c00020 = hard)\n", mt_rr(&dev, CFG_ADDR(MT_USB_U3DMA_CFG)));
	printf("ADOPT: PASS - the adopt path brought the device up; the wedge "
	       "recovery runs here too\n");
	rc = 0;

out_close:
	mt_mac_stop(&dev);
	mt_close(&dev);      /* adopted: releases nothing of ours, drops io_lock */
out_release:
	libusb_release_interface(h, 0);
out:
	if (detached)
		libusb_attach_kernel_driver(h, 0);
	libusb_close(h);
	libusb_exit(ctx);
	return rc;
}

/*
 * Stage A: a static beacon on air. The MAC auto-transmits it from the reserved
 * page, so there is nothing to loop over here except watching the TSF advance;
 * the RTL8812AU witness (rxdemo) and a kernel station's `iw scan` decide
 * PASS/FAIL. The beacon is ALWAYS disabled before returning - a beacon left
 * armed keeps airing after the process exits and contaminates the next run.
 */
static int build_beacon(uint8_t chan, const uint8_t *bssid, uint8_t *out,
                        size_t outsz)
{
	/* 5 GHz: OFDM basic set. 2.4 GHz: CCK + OFDM basic set. */
	static const uint8_t rates_5g[] = { 0x8c, 0x12, 0x98, 0x24,
	                                    0xb0, 0x48, 0x60, 0x6c };
	static const uint8_t rates_2g[] = { 0x82, 0x84, 0x8b, 0x96,
	                                    0x0c, 0x12, 0x18, 0x24 };
	static const char ssid[] = "MT7612U-AP";
	const uint8_t *rates = chan <= 14 ? rates_2g : rates_5g;
	const int ssidlen = (int)sizeof ssid - 1;
	uint8_t *p = out;

	if (outsz < 128)
		return -1;

	*p++ = 0x80; *p++ = 0x00;                 /* FC: mgmt, beacon */
	*p++ = 0x00; *p++ = 0x00;                 /* duration */
	memset(p, 0xff, 6); p += 6;               /* addr1 = broadcast */
	memcpy(p, bssid, 6); p += 6;              /* addr2 = SA (BSSID) */
	memcpy(p, bssid, 6); p += 6;              /* addr3 = BSSID */
	*p++ = 0x00; *p++ = 0x00;                 /* seq ctl (HW assigns) */

	memset(p, 0, 8); p += 8;                  /* timestamp (HW fills) */
	*p++ = 0x64; *p++ = 0x00;                 /* beacon interval = 100 TU */
	*p++ = 0x01; *p++ = 0x00;                 /* capability: ESS */

	*p++ = 0; *p++ = (uint8_t)ssidlen;                    /* SSID IE */
	memcpy(p, ssid, (size_t)ssidlen); p += ssidlen;
	*p++ = 1; *p++ = 8; memcpy(p, rates, 8); p += 8;      /* Supported Rates */
	*p++ = 3; *p++ = 1; *p++ = chan;                      /* DS Parameter Set */
	*p++ = 5; *p++ = 4;                                   /* TIM (DTIM=1, empty) */
	*p++ = 0; *p++ = 1; *p++ = 0; *p++ = 0;

	return (int)(p - out);
}

static int gate_beacon(uint8_t chan, int secs)
{
	/* The AP's BSSID is the device's own MAC, which mac_setaddr() has already
	 * programmed into MT_MAC_ADDR (what the MAC auto-ACKs against) and
	 * MT_MAC_BSSID. Advertising anything else in the beacon would leave a
	 * station addressing auth to an address the MAC does not answer for. It
	 * is a real, unicast address, which is what a STA requires (an I/G-set
	 * BSSID makes it drop auth before the air - docs/ap-mode.md). */
	const uint8_t *bssid;
	struct mt7612u_tx_rate rate = {
		.phy = MT7612U_PHY_OFDM, .mcs = 0, .nss = 1,
		.bw = MT7612U_BW_20, .no_ack = 1,
	};
	uint8_t bcn[128];
	int n, rc = 1;

	if (mt_eeprom_init(&dev)) return 1;
	bssid = dev.macaddr;
	n = build_beacon(chan, bssid, bcn, sizeof bcn);
	if (n < 0) { printf("GATE A: FAIL - beacon build\n"); return 1; }
	if (mt_init_hardware(&dev, NULL)) {
		printf("GATE A: FAIL - init_hardware\n"); return 1;
	}
	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) {
		printf("GATE A: FAIL - set_channel\n"); return 1;
	}
	/* TX-only: beaconing never reads EP 4. */
	if (mt_mac_start(&dev, MT_RX_DRAIN_NONE)) {
		printf("GATE A: FAIL - mac_start\n"); mt_mac_stop(&dev); return 1;
	}

	mt_beacon_init(&dev);
	if (mt_beacon_write(&dev, bcn, (size_t)n, &rate)) {
		printf("GATE A: FAIL - beacon_write\n"); goto out;
	}
	if (mt_beacon_set_enable(&dev, 1, 100)) {
		printf("GATE A: FAIL - beacon_set_enable\n"); goto out;
	}

	printf("beacon armed: ch%u, BSSID %02x:%02x:%02x:%02x:%02x:%02x, "
	       "SSID \"MT7612U-AP\", 100 TU, OFDM 6M, %d B MPDU\n",
	       chan, bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5], n);
	printf("MT_BEACON_TIME_CFG=0x%08x (bit16 TIMER bit19 TBTT bit20 TX)\n",
	       mt_rr(&dev, MT_BEACON_TIME_CFG));
	printf("MT_MAC_BSSID_DW1  =0x%08x (MBSS_MODE 17:16 should read 3)\n",
	       mt_rr(&dev, MT_MAC_BSSID_DW1));
	printf("witness: run rxdemo on the 8812AU and grep the BSSID; "
	       "or `iw dev <sta> scan | grep MT7612U-AP`\n");

	/* Watch the TSF advance - proof the beacon timer is running. The checked
	 * read matters here: an unchecked one returns all-ones on a failed
	 * transfer, which is "greater than the previous sample" and would count a
	 * dead transport as a live timer. A failed read breaks the chain instead. */
	{
		uint64_t prev = 0, tsf;
		bool have = false;
		int good = 0, failed = 0;

		for (int s = 0; s < secs && !g_stop; s++) {
			if (mt7612u_read_tsf_chk(&dev, &tsf)) {
				printf("  t=%ds TSF read failed\n", s);
				failed++;
				have = false;
			} else {
				if (have)
					printf("  t=%ds TSF=%llu (+%llu us)\n", s,
					       (unsigned long long)tsf,
					       (unsigned long long)(tsf - prev));
				if (have && tsf > prev)
					good++;
				prev = tsf;
				have = true;
			}
			if (!wait_ms(1000))
				break;
		}
		/* A running TSF is necessary, not sufficient - the witness is the
		 * real gate - but a frozen TSF means no beacons are being sent. */
		if (failed) {
			printf("GATE A: FAIL - %d TSF read(s) failed; no timer verdict on a failing transport\n",
			       failed);
			goto out;
		}
		if (good == 0) {
			printf("GATE A: FAIL - TSF did not advance; beacon timer is dead\n");
			goto out;
		}
		printf("TSF advanced on %d sample(s) - beacon timer is live\n", good);
	}
	rc = 0;
	/* This is a LOCAL precondition only: an advancing TSF proves the beacon
	 * timer runs, not that a frame reaches the air. The witness (rxdemo /
	 * `iw scan`) is the actual Gate A. */
	printf("\nGATE A (local): beacon armed, timer live. On-air PASS/FAIL is "
	       "the witness's call - grep the 8812AU for our SSID/BSSID.\n");

out:
	mt_beacon_set_enable(&dev, 0, 0);   /* never leave a beacon airing */
	mt_mac_stop(&dev);
	return rc;
}

/*
 * Stage B: the beacon plus a receiver, so a real station can probe, authenticate
 * and associate against us.
 *
 * The measurement that matters is the RETRY BIT. An ACK is SIFS-timed and can
 * only come from the MAC, so it cannot be observed directly from userspace -
 * but a station that does not get one retransmits with FC Retry set. Auth
 * arriving at retry=0 is therefore the proof that the hardware auto-ACKed it;
 * a pile of retry=1 auths is the proof it did not.
 */
struct ap_ctx {
	/* std::atomic, not C11 _Atomic: this file is C++ since the subtree
	 * migration, and ap_cb runs on the RX event thread while the gate's own
	 * thread reads the counters. */
	std::atomic<unsigned> probe_req{0}, auth{0}, auth_retry{0};
	std::atomic<unsigned> assoc{0}, assoc_retry{0};
	std::atomic<unsigned> data_to_us{0}, mgmt_other{0};
	uint8_t bssid[6];
};

static void ap_cb(void *user, const void *frame, size_t len,
                  const struct mt7612u_rx_info *info)
{
	struct ap_ctx *c = static_cast<struct ap_ctx *>(user);
	const uint8_t *f = static_cast<const uint8_t *>(frame);
	unsigned fc, type, subtype;
	int retry, to_us;

	(void)info;
	if (len < 16) return;
	fc = (unsigned)f[0] | ((unsigned)f[1] << 8);
	type = (fc >> 2) & 3;
	subtype = (fc >> 4) & 0xf;
	retry = (f[1] & 0x08) != 0;        /* FC Retry */
	to_us = memcmp(f + 4, c->bssid, 6) == 0;   /* addr1 == our BSSID */

	if (type == 2) {                   /* data */
		if (to_us)
			c->data_to_us.fetch_add(1, std::memory_order_relaxed);
		return;
	}
	if (type != 0) return;             /* control */

	switch (subtype) {
	case 4:                            /* probe request (usually broadcast) */
		c->probe_req.fetch_add(1, std::memory_order_relaxed);
		break;
	case 11:                           /* authentication */
		if (!to_us) break;
		c->auth.fetch_add(1, std::memory_order_relaxed);
		if (retry)
			c->auth_retry.fetch_add(1, std::memory_order_relaxed);
		break;
	case 0: case 2:                    /* (re)association request */
		if (!to_us) break;
		c->assoc.fetch_add(1, std::memory_order_relaxed);
		if (retry)
			c->assoc_retry.fetch_add(1, std::memory_order_relaxed);
		break;
	default:
		if (to_us)
			c->mgmt_other.fetch_add(1, std::memory_order_relaxed);
		break;
	}
}

static int gate_ap(uint8_t chan, int secs)
{
	struct ap_ctx ctx{};
	struct mt7612u_tx_rate rate = {
		.phy = MT7612U_PHY_OFDM, .mcs = 0, .nss = 1,
		.bw = MT7612U_BW_20, .no_ack = 1,
	};
	uint8_t bcn[128];
	int n, rc = 1, rx_up = 0;
	unsigned pr, au, aur, as, asr, dt;

	if (secs <= 0 || secs > 3600) {
		printf("GATE B: FAIL - duration %d out of range (1..3600 s)\n", secs);
		return 1;
	}
	if (mt_eeprom_init(&dev)) return 1;
	memcpy(ctx.bssid, dev.macaddr, 6);
	n = build_beacon(chan, dev.macaddr, bcn, sizeof bcn);
	if (n < 0) { printf("GATE B: FAIL - beacon build\n"); return 1; }

	if (mt_init_hardware(&dev, NULL)) {
		printf("GATE B: FAIL - init_hardware\n"); return 1;
	}
	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) {
		printf("GATE B: FAIL - set_channel\n"); return 1;
	}
	/* Ring first, receiver second - RX must never run with EP 4 undrained. */
	if (mt7612u_rx_start(&dev, ap_cb, &ctx)) {
		printf("GATE B: FAIL - rx_start\n"); return 1;
	}
	rx_up = 1;
	if (mt_mac_start(&dev, MT_RX_DRAIN_RING)) {
		printf("GATE B: FAIL - mac_start\n");
		rx_teardown(); mt_mac_stop(&dev); return 1;
	}
	/*
	 * AP receive filter. The managed default mt_mac_start() just wrote already
	 * leaves OTHER_BSS, BCAST and MCAST undropped, so a probe request with a
	 * wildcard BSSID reaches us - mt76 clears OTHER_BSS for every mode too.
	 * The one change an AP needs is DUP: dropping duplicates would hide exactly
	 * the retransmissions this gate measures. Clear the bit in place rather
	 * than re-write a copied literal, so this cannot drift from the default.
	 */
	mt_clear(&dev, MT_RX_FILTR_CFG, MT_RX_FILTR_CFG_DUP);

	/*
	 * The address-match half of "being an AP": the MAC auto-ACKs against
	 * MT_MAC_ADDR (already our MAC) and matches the BSS against this slot,
	 * which mac_setaddr() zeroed. There is no separate AP op-mode register on
	 * this part - mt76 sets none either; address match + beacon IS the AP.
	 *
	 * Slot 0 is only right for a globally-administered MAC. Under MBSS_MODE=3
	 * the hardware takes the BSS index from the address bits, and mt76 uses
	 * 1 + (((macaddr[0] ^ addr[0]) >> 2) & 7) whenever the locally-administered
	 * bit is set (mt76x02_util.c). Refuse loudly rather than guess: a cloned
	 * 02:/06:/0a: MAC would match nothing and void every result below.
	 */
	if (dev.macaddr[0] & 0x02) {
		printf("GATE B: FAIL - MAC %02x:.. is locally administered; APC slot 0 "
		       "is not the slot this MAC selects (mt76 derives 1+n)\n",
		       dev.macaddr[0]);
		goto out;
	}
	if (mt_ap_set_bssid(&dev, 0, dev.macaddr)) {
		printf("GATE B: FAIL - could not program the APC BSSID slot\n");
		goto out;
	}

	mt_beacon_init(&dev);
	if (mt_beacon_write(&dev, bcn, (size_t)n, &rate)) {
		printf("GATE B: FAIL - beacon_write\n"); goto out;
	}
	if (mt_beacon_set_enable(&dev, 1, 100)) {
		printf("GATE B: FAIL - beacon_set_enable\n"); goto out;
	}

	printf("AP up: ch%u  BSSID/MAC %02x:%02x:%02x:%02x:%02x:%02x  SSID \"MT7612U-AP\"\n",
	       chan, dev.macaddr[0], dev.macaddr[1], dev.macaddr[2],
	       dev.macaddr[3], dev.macaddr[4], dev.macaddr[5]);
	/* Read back BOTH halves of the BSSID: mt_rmw() skips its write when the
	 * read fails, so printing only the L half would show a correct-looking
	 * address for a BSSID whose top two bytes never landed. */
	printf("MT_RX_FILTR_CFG=0x%08x  APC_BSSID(0)=%04x%08x  AUTO_RSP_CFG=0x%08x\n",
	       mt_rr(&dev, MT_RX_FILTR_CFG),
	       (unsigned)(mt_rr(&dev, MT_MAC_APC_BSSID_H(0)) & MT_MAC_APC_BSSID_H_ADDR),
	       mt_rr(&dev, MT_MAC_APC_BSSID_L(0)),
	       mt_rr(&dev, MT_AUTO_RSP_CFG));
	printf("stimulus: on a station radio run\n"
	       "  sudo iw dev <sta> scan          (probe requests)\n"
	       "  sudo wpa_supplicant ... / iw dev <sta> connect MT7612U-AP\n");
	printf("listening %d s ...\n", secs);

	if (!wait_ticking(secs * 1000.0))
		printf("(interrupted)\n");

	/*
	 * Receiver loss belongs next to the verdict: a retried auth we simply
	 * missed biases the result toward PASS, which is the direction that
	 * produces a false hardware conclusion. Sample it while the ring still
	 * EXISTS - mt7612u_rx_stop() tears the ring down and takes its counters
	 * with it, which reads back as a flat zero and looks like a clean capture.
	 */
	{
		struct mt7612u_stats st;

		mt7612u_get_stats(&dev, &st);
		printf("rx frames %llu  err %llu  invalid %llu  dropped %llu\n",
		       (unsigned long long)st.rx_frames,
		       (unsigned long long)st.rx_err,
		       (unsigned long long)st.rx_invalid,
		       (unsigned long long)st.rx_dropped);
	}

	/*
	 * Now stop the producer, BEFORE reading the verdict counters. ap_cb() runs
	 * on the RX event thread, and auth/auth_retry are two independent relaxed
	 * atomics - sampling them live can catch one increment half-applied and
	 * invert the verdict outright (auth=0 with auth_retry=1 reads as "no auth
	 * reached us"; auth_retry>auth reads as "every auth was a retry").
	 * mt_async_stop() joins the event thread, so after this no callback can
	 * run. gate_ack orders it the same way.
	 */
	mt7612u_rx_stop(&dev);
	rx_up = 0;

	pr  = ctx.probe_req.load(std::memory_order_relaxed);
	au  = ctx.auth.load(std::memory_order_relaxed);
	aur = ctx.auth_retry.load(std::memory_order_relaxed);
	as  = ctx.assoc.load(std::memory_order_relaxed);
	asr = ctx.assoc_retry.load(std::memory_order_relaxed);
	dt  = ctx.data_to_us.load(std::memory_order_relaxed);

	printf("\nprobe-req %u | auth %u (retry %u) | assoc %u (retry %u) | data-to-us %u | other-mgmt %u\n",
	       pr, au, aur, as, asr, dt,
	       ctx.mgmt_other.load(std::memory_order_relaxed));

	if (!au) {
		printf("GATE B: INCONCLUSIVE - no auth reached us "
		       "(probe-req %u). Did a station try to connect?\n", pr);
	} else if (aur == 0) {
		printf("GATE B: PASS - %u auth frame(s), none retried: "
		       "the MAC auto-ACKed them\n", au);
		rc = 0;
	} else if (aur < au) {
		printf("GATE B: PARTIAL - %u auth, %u retried: ACKs land but not always\n",
		       au, aur);
		rc = 0;
	} else {
		printf("GATE B: FAIL - every auth (%u) was a retry: nothing is ACKing\n", au);
	}

out:
	mt_beacon_set_enable(&dev, 0, 0);   /* never leave a beacon airing */
	/* Retract the BSS address too, so the teardown matches the contract the
	 * beacon half states. Inert in practice (the MAC is stopped and
	 * mac_setaddr() re-zeroes every slot on the next bring-up), but leaving
	 * half the AP identity programmed contradicts what this gate promises. */
	{
		static const uint8_t zero[6] = { 0 };

		mt_ap_set_bssid(&dev, 0, zero);
	}
	/* rx_up: the verdict path already stopped the ring so the counters could
	 * be read with the producer joined; stopping twice must not happen. */
	if (rx_up)
		mt7612u_rx_stop(&dev);
	mt_mac_stop(&dev);
	return rc;
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
	if (mt_mac_start(&dev, MT_RX_DRAIN_NONE)) {
		printf("GATE E: FAIL - mac_start failed\n"); mt_mac_stop(&dev); return 1;
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
	if (mt_mac_start(&dev, MT_RX_DRAIN_SYNC)) {
		printf("GATE F: FAIL - mac_start failed\n");
		mt_mac_rx_disable(&dev); mt_mac_stop(&dev); return 1;
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
	/*
	 * Declared, not fixed.  The tick below runs on this thread, which under
	 * MT_RX_DRAIN_SYNC is the only EP 4 drainer, and mt7612u_phy_tick() can
	 * block in the MCU for ~3.3 s when the part answers late under RF load -
	 * so the gate opens the very undrained-receiver window it exists to
	 * observe.  Gating RX around the tick would close that window, but the
	 * re-enable runs through mt_mac_start(), which rewrites MT_RX_FILTR_CFG
	 * back to the initvals value and would silently undo the monitor filter
	 * set above - a quieter gate measuring something else.  A sync gate that
	 * can no longer reproduce the hazard is worth less than one that reports
	 * it, and gate_arx is the witness that holds anyway: its ring drains on
	 * the libusb event thread, and its notick arm is the measured control.
	 */
	printf("  NOTE: the 1 Hz tick blocks this thread, the only EP 4 drainer -\n"
	       "        under load this gate is NOT a valid tick witness. Use\n"
	       "        `arx <ch> <secs>` for that (`arx <ch> <secs> 1` is its\n"
	       "        negative control).\n");

	double last_tick = now_ms();

	while (got < want && empty < 200) {
		if (now_ms() - last_tick >= 1000.0) {
			mt7612u_phy_tick(&dev);
			last_tick = now_ms();
		}
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
	/* The same invariant every ring-cancelling gate now follows, and it
	 * applies here even though there is no ring: mt_mac_stop() does not clear
	 * ENABLE_RX until after its own mt_rx_flush() and a TX-idle wait of up to
	 * 150 ms, and this thread has already stopped draining EP 4 - on the busy
	 * channel this gate was just listening to, that IS the undrained window. */
	mt_mac_rx_disable(&dev);
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
	if (mt_mac_start(&dev, MT_RX_DRAIN_NONE)) { mt_mac_stop(&dev); return 1; }

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
	if (mt_mac_start(&dev, MT_RX_DRAIN_NONE)) { mt_mac_stop(&dev); return 1; }

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
		/* atomic because drain_cb increments it from the RX event thread. */
		std::atomic<unsigned long> drained{0};
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
			if (pass) rx_teardown();
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
	if (mt_mac_start(&dev, MT_RX_DRAIN_NONE)) { mt_mac_stop(&dev); return 1; }

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
struct arx_ctx {
	std::atomic<unsigned long> n;
	std::atomic<unsigned long> by_phy[8];
};
static void arx_cb(void *user, const void *frame, size_t len,
                   const struct mt7612u_rx_info *info)
{
	struct arx_ctx *c = (struct arx_ctx *)user;
	(void)frame; (void)len;
	c->n.fetch_add(1, std::memory_order_relaxed);
	c->by_phy[info->phy & 7].fetch_add(1, std::memory_order_relaxed);
}

/* Async RX ring: the callback path StartRxLoop needs. */
static int gate_arx(uint8_t chan, int secs, int notick)
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
	if (mt7612u_rx_start(&dev, arx_cb, &ctx)) {
		printf("GATE arx: FAIL - rx_start failed\n"); return 1;
	}
	if (mt_mac_start(&dev, MT_RX_DRAIN_RING)) {
		rx_teardown(); mt_mac_stop(&dev); return 1;
	}
	mt7612u_set_monitor_rx(&dev, 0);
	t0 = now_ms();
	/* notick is the negative control: without the 1 Hz PHY tick this gate
	 * reads 3 frames in 10 s from a strong nearby peer (reproduced eight
	 * times); with it the rate quoted on mt7612u_phy_tick().  See that comment. */
	if (notick) wait_ms(secs * 1000.0); else wait_ticking(secs * 1000.0);
	{
		struct mt_async_stats st;
		/* Actual elapsed, not the requested duration: an interrupt now
		 * unwinds through here, and dividing by the request would report
		 * a rate the run never achieved. */
		double el = (now_ms() - t0) / 1000.0;

		mt_async_stats(&dev, &st);
		/* ring=rx_frames is what the ring accepted, cb=ctx.n is what the
		 * callback saw.  They must agree; printing only the second one
		 * cannot tell "nothing arrived" from "arrived, not delivered". */
		printf("async RX on ch%u for %.1f s: %lu frames (%.0f/s), "
		       "ring=%llu rx_err=%llu "
		       "rx_invalid=%llu rx_dropped=%llu\n",
		       chan, el, ctx.n.load(), ctx.n.load() / (el > 0 ? el : 1),
		       (unsigned long long)st.rx_frames,
		       (unsigned long long)st.rx_err,
		       (unsigned long long)st.rx_invalid,
		       (unsigned long long)st.rx_dropped);
	}
	for (int i = 0; i < 8; i++)
		if (ctx.by_phy[i])
			printf("  %-6s %lu\n", phy_name[i], ctx.by_phy[i].load());
	rx_teardown();
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

	memset(frame, 0, sizeof frame);
	frame[0] = 0x08;
	memset(frame + 4, 0xff, 6);
	memcpy(frame + 10, src, 6);
	memcpy(frame + 16, src, 6);
	memcpy(frame + 24, "MT7612U-HAL ", 12);

	if (mt7612u_rx_start(&dev, arx_cb, &ctx)) return 1;
	if (mt_mac_start(&dev, MT_RX_DRAIN_RING)) {
		rx_teardown(); mt_mac_stop(&dev); return 1;
	}
	mt7612u_set_monitor_rx(&dev, 0);

	t0 = now_ms();
	{
		double last_tick = t0;

		while (now_ms() - t0 < secs * 1000.0) {
			frame[36] = (uint8_t)n; frame[37] = (uint8_t)(n >> 8);
			if (mt7612u_tx(&dev, frame, 1400, &rate) == 0) n++;
			/* RX stays enabled through the flood; without the 1 Hz tick the
			 * receiver decays and the concurrent-RX figure is confounded. */
			if (now_ms() - last_tick >= 1000.0) {
				mt7612u_phy_tick(&dev);
				last_tick = now_ms();
			}
		}
	}
	wall = now_ms() - t0;
	printf("duplex on ch%u for %.1f s:\n", chan, wall / 1000.0);
	{
		struct mt_async_stats st;

		mt_async_stats(&dev, &st);
		printf("  TX %ld frames (%.0f fps)  RX %lu frames (%.0f fps)  "
		       "tx_err=%llu rx_err=%llu\n",
		       n, n * 1000.0 / wall, ctx.n.load(),
		       ctx.n.load() * 1000.0 / wall,
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
		unsigned long before = ctx.n.load(std::memory_order_relaxed);
		unsigned long after;

		printf("  TX stopped; listening %.1f s for the receiver to recover\n",
		       RECOVER_S);
		if (!wait_ticking(RECOVER_S * 1000.0)) {
			rx_teardown();
			mt_mac_stop(&dev);
			return 1;
		}
		after = ctx.n.load(std::memory_order_relaxed);
		printf("  RX after the flood: %lu frames\n", after - before);
		rx_teardown();
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
	if (mt_mac_start(&dev, MT_RX_DRAIN_NONE)) { mt_mac_stop(&dev); return 1; }

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

/*
 * gate_txs's receiver, for the receiver-ON half. An 802.11 ACK is FC 0xd4
 * 0x00, duration, addr1 - ten bytes, and this part does not deliver the FCS,
 * so `len` is 10 here rather than the 14 a Realtek witness reports. addr1 of
 * an ACK is the address that solicited it, i.e. OUR addr2, which is what
 * distinguishes our peer's ACKs from the ambient ACK traffic any busy channel
 * carries. The gate transmits from TWO addr2s - the port's own address on
 * the ownSA arms and a static source on the rest - so both are matched.
 */
struct ucast_ack_count {
	std::atomic<unsigned long> acks{0};
	std::atomic<unsigned long> frames{0};
	uint8_t ta[2][6];   /* written before the RX ring starts, read-only after */
};

static void ucast_rx_cb(void *user, const void *frame, size_t len,
                        const struct mt7612u_rx_info *info)
{
	struct ucast_ack_count *c = (struct ucast_ack_count *)user;
	const uint8_t *f = (const uint8_t *)frame;

	(void)info;
	c->frames.fetch_add(1, std::memory_order_relaxed);
	if (len < 10 || len > 16) return;
	if (f[0] != 0xd4 || f[1] != 0x00) return;
	if (memcmp(f + 4, c->ta[0], 6) != 0 && memcmp(f + 4, c->ta[1], 6) != 0)
		return;
	c->acks.fetch_add(1, std::memory_order_relaxed);
}

static int parse_mac6(const char *s, uint8_t out[6])
{
	unsigned v[6];
	int i, used = -1;

	if (!s) return -1;
	/* %n pins the whole string: "02:...:0a:ff" or "02:...:0azz" is a typo,
	 * not a MAC with trailing decoration. */
	if (sscanf(s, "%x:%x:%x:%x:%x:%x%n",
	           &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &used) != 6 ||
	    used < 0 || s[used] != '\0')
		return -1;
	for (i = 0; i < 6; i++) {
		if (v[i] > 0xff) return -1;
		out[i] = (uint8_t)v[i];
	}
	return 0;
}

/*
 * gate_txs - read the retry count off the chip instead of inferring it.
 *
 * The retry ladder behind the unicast cliff (docs/mt7612u.md) can be derived
 * by arithmetic - 15 retries from MT_TX_RETRY_CFG, CWmin 15 / CWmax 1023 from
 * MT_WMM_CWMIN/CWMAX, a 9 us slot, ~46 ms against ~45.5 ms measured - but
 * that is an inference. The MAC counts the retries itself, in
 * MT_TX_STAT_FIFO, for every frame sent with a non-zero txwi pktid
 * (MT_TXOPT_TXS); this gate reads that count.
 *
 * Two questions this answers directly:
 *
 *  1. Does the ladder run to exhaustion when no ACK can arrive? Expect a retry
 *     count near the 15 limit with SUCCESS clear.
 *  2. Why do the No-Ack arms (txwi ACK_CTL_REQ clear AND QoS Ack Policy = No
 *     Ack) still sit far below the broadcast ceiling instead of at it? If their
 *     entries show retries, the no-ack request is not reaching the retry engine
 *     - a devourer-side defect. If they show retry 0, the cost is elsewhere and
 *     the ladder is not the explanation for those arms.
 *
 * And it verifies the retry-limit knob: DEVOURER_TX_RETRY_LIMIT=N applies
 * mt7612u_set_retry_limit() before the arms run, exactly as Mt7612uRadio
 * does, so an unacknowledged Normal arm must then settle at N+1 attempts.
 *
 * On receiver-ON passes the 1 Hz PHY tick runs from the per-frame and settle
 * waits (txs_tick), as in every receiving gate here.
 *
 * Frames go out ONE AT A TIME: each waits for its own status entry (bounded)
 * before the next is submitted, since a submit only means the USB transfer
 * is queued. At the
 * ~20-60 fps these configurations run, a drain costs nothing next to a 45 ms
 * frame. (Draining alone does NOT keep arms apart - an unsettled arm's status
 * can arrive after the next arm starts; the per-arm pktid below does.) The
 * status FIFO is
 * shallow and mt76 polls it, so batch-then-drain would lose most of it.
 *
 * The peer is an independent radio armed as a hardware ACK responder for
 * `peer` (e.g. an RTL8812AU running rxdemo with DEVOURER_ACK_RESPONDER), on
 * the same channel. Usage: txs [chan] [frames/arm] [peer MAC].
 *
 * Every arm, in each receiver pass, sends with its OWN txwi pktid
 * (txs_arm_pktid) and counts only status entries echoing it. A shared pktid
 * would let late status from an unsettled arm - an unacknowledged Normal arm
 * can still owe entries when its deadline passes - land in the NEXT arm's
 * columns, making a No-Ack arm look as if it retried to exactly the
 * configured limit and displacing one of its own entries from entr/sent.
 * Entries carrying the previous arm's pktid are reported as late; any other
 * pktid as foreign - with one exception, the stale EXT word (txs_drain).
 *
 * Exit: 0 reported, 1 device failure OR no status entry filed at all (the
 * measurement did not happen), 2 bad argument or retry limit refused,
 * 3 interrupted.
 */
struct txs_sum {
	long entries, success, retry_total, retry_max;
	long late_prev; /* entries carrying the PREVIOUS arm's pktid */
	long foreign;   /* entries with any other pktid */
	long stale_ext; /* own entries popped with a stale EXT word: counted in
	                 * entries/success, kept out of the retry columns */
};

/* mt76's skb pktid range starts at MT_PACKET_ID_FIRST (3) and the id must
 * stay under bit 7 (MT_PACKET_ID_HAS_RATE): 3 + 8 * pass + arm gives 3..18
 * for two passes of up to eight arms. */
static unsigned txs_arm_pktid(int rx_on, unsigned arm)
{
	return 3u + 8u * (unsigned)rx_on + arm;
}
#define TXS_NO_PKTID 0x100u   /* matches no 8-bit EXT_PKTID */
#define TXS_ANY_PKTID 0x200u  /* txs_drain's stale_id: unknown, so any */
/* Slack on top of frame_budget_ms for one frame's status wait: USB submit
 * latency plus the drain's two control reads. */
#define TXS_FRAME_MARGIN_MS 50.0

/* Returns 0 when the FIFO was drained (or is empty), -1 when a status read
 * failed - the caller must not report the arm as measured then.
 *
 * The EXT-then-main read is two USB transfers, not one atomic read. When the
 * FIFO is EMPTY at the EXT read and an entry is filed before the main read,
 * the main read pops that entry but the EXT word read just before it is
 * stale - it still describes the last entry popped. Within an arm that is
 * harmless (same pktid). On an arm's FIRST entry it is the previous arm's
 * pktid (or, on the session's first arm, whatever EXT held), so the entry was
 * counted late/foreign, the arm stayed one short for good, and every
 * per-frame wait then timed out - the "one-step status lag" rows of
 * docs/mt7612u-tx-retry.md (N-1/N, a timeout on every frame, ~5-6 fps, and
 * the late entry in the SAME arm's row, after a fully settled arm).
 *
 * `stale_id` takes that one entry back. It is the pktid a stale EXT word
 * would carry: the last arm that SENT a frame (an arm that sent none popped
 * nothing, so EXT still describes the arm before it), or TXS_ANY_PKTID on the
 * session's first. The caller passes it only once this arm has submitted a
 * frame and that arm owed no entries, and TXS_NO_PKTID otherwise (no
 * claim). The arm's first popped entry, carrying stale_id, can then only be
 * ours. Its main word is fresh, so its SUCCESS bit counts; its retry count is
 * the stale word's, so it is kept out of the retry columns. */
static int txs_drain(struct mt7612u_dev *d, struct txs_sum *o,
                     unsigned want, unsigned prev, unsigned stale_id)
{
	int guard;

	/* Bounded: a stuck VALID bit must not become an infinite loop inside a
	 * gate holding the only USB lock for this adapter. */
	for (guard = 0; guard < 64; guard++) {
		uint32_t st = 0, ext = 0;
		long r;

		/* Read order matters, and it is mt76's
		 * (mt76x02_mac_load_tx_status): EXT FIRST, then the main word.
		 * Reading MT_TX_STAT_FIFO pops the entry, so an EXT read after it
		 * would describe the NEXT head, not the entry just popped. */
		if (mt_rr_chk(d, MT_TX_STAT_FIFO_EXT, &ext)) return -1;
		if (mt_rr_chk(d, MT_TX_STAT_FIFO, &st)) return -1;
		if (!(st & MT_TX_STAT_FIFO_VALID)) return 0;
		/* Only the CURRENT arm's frames are averaged in: the previous
		 * arm's late status and anything else that files status are
		 * counted apart. */
		{
			const unsigned id =
				(unsigned)FIELD_GET(MT_TX_STAT_FIFO_EXT_PKTID, ext);

			if (id != want) {
				if (stale_id != TXS_NO_PKTID && o->entries == 0 &&
				    (stale_id == TXS_ANY_PKTID || id == stale_id)) {
					o->entries++;
					o->stale_ext++;
					if (st & MT_TX_STAT_FIFO_SUCCESS)
						o->success++;
					continue;
				}
				if (id == prev) o->late_prev++;
				else            o->foreign++;
				continue;
			}
		}
		o->entries++;
		if (st & MT_TX_STAT_FIFO_SUCCESS) o->success++;
		r = (long)FIELD_GET(MT_TX_STAT_FIFO_EXT_RETRY, ext);
		o->retry_total += r;
		if (r > o->retry_max) o->retry_max = r;
	}
	return 0;
}

/* The whole string one number (base auto-detect, leading and trailing
 * whitespace allowed, as strtol and isspace define them) - the rule
 * env_config's env_long_strict() applies. 0 and *out on success, -1 when no
 * digit was consumed, anything but whitespace follows, or it overflows. The
 * no-digits check comes BEFORE the trailing-whitespace skip: after it, a
 * whitespace-only string would look consumed and read as 0. */
static int txs_parse_long(const char *s, long *out)
{
	char *end = NULL;
	long v;

	if (!s || !*s) return -1;
	errno = 0;
	v = strtol(s, &end, 0);
	if (!end || end == s || errno == ERANGE) return -1;
	while (isspace((unsigned char)*end)) end++;
	if (*end != '\0') return -1;
	*out = v;
	return 0;
}

/* The receiving gates' 1 Hz PHY tick (gate_rx, gate_duplex use exactly this
 * last-tick form; wait_ticking() the sleeping one): on a receiver
 * pass, at most once a second, from wherever the gate is waiting. Without it
 * the receiver decays - see mt7612u_phy_tick() in the public header. Its
 * return is ignored, as every other gate here ignores it. */
static void txs_tick(int rx_on, double *last_tick)
{
	if (rx_on && now_ms() - *last_tick >= 1000.0) {
		mt7612u_phy_tick(&dev);
		*last_tick = now_ms();
	}
}

/* An operator-supplied string on one output line: \n \r \t as escapes, any
 * other control byte (< 0x20, 0x7f) as \xNN - env_config's rule. */
static void txs_print_escaped(const char *s)
{
	for (; s && *s; s++) {
		const unsigned char c = (unsigned char)*s;

		if (c == '\n')                 fputs("\\n", stdout);
		else if (c == '\r')            fputs("\\r", stdout);
		else if (c == '\t')            fputs("\\t", stdout);
		else if (c < 0x20 || c == 0x7f) printf("\\x%02x", c);
		else                           putchar(c);
	}
}

/* DEVOURER_TX_RETRY_LIMIT, read with env_config's strictness: the whole
 * string one number (base auto-detect), trailing whitespace by isspace()
 * exactly as env_long_strict() takes it, clamped to the config's 0..63.
 * Returns 1 and sets *out when present and valid, 0 when unset, -1 when
 * present but not a number. */
static int txs_retry_limit_env(int *out)
{
	const char *e = getenv("DEVOURER_TX_RETRY_LIMIT");
	long v;

	if (!e || !*e) return 0;
	if (txs_parse_long(e, &v)) return -1;
	*out = (int)(v < 0 ? 0 : (v > 63 ? 63 : v));
	return 1;
}

static int gate_txs(uint8_t chan, int frames, const char *peer_str)
{
	static const uint8_t src[6]   = { 0x02, 0x4d, 0x54, 0x76, 0x12, 0x01 };
	static const uint8_t bcast[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	uint8_t peer[6] = { 0x02, 0x4d, 0x54, 0x76, 0x12, 0x0a };
	static uint8_t frame[1600];
	const size_t flen = 1400;
	static struct ucast_ack_count ctr;
	int rx_on, rl = 0, rl_given;
	long total_entries = 0, total_foreign = 0;
	unsigned prev_pktid = TXS_NO_PKTID;
	/* txs_drain's stale_id: the last arm that sent a frame, and whether it
	 * settled with no entry owed. */
	unsigned stale_pktid = TXS_ANY_PKTID;
	int stale_settled = 1;
	double frame_budget_ms;
	int io_fail = 0;   /* status read / WCID setup failed: teardown, exit 1 */
	double last_tick = 0.0;  /* receiver passes: last mt7612u_phy_tick() */

	/*
	 * Arms a-d are the ones docs/mt7612u-tx-retry.md records. Arms e-h
	 * attack what is left of its open question.
	 *
	 * The receiver-OFF No-Ack arm (c) settles at 0.0 retries and 100%
	 * success and STILL costs ~20 ms a frame, so whatever that cost is, it
	 * is not the retry engine. The candidates that can be separated with a
	 * register write and a txwi bit are: the no-station WCID index, the
	 * transmit queue the frame is filed into, and aggregation. Each gets an
	 * arm against the same reference.
	 *
	 * `wcid` 1 means a real WCID-table entry installed with mt_wcid_setup()
	 * - no library path installs one. The published bisect (docs/mt7612u.md)
	 * measured wcid=1 as WORSE than 0xff against a dead peer, which is itself
	 * unexplained, so this is a re-measurement under known-good accounting
	 * rather than a repeat.
	 */
	static const struct {
		char tag; int own_sa; int bcast_a1; int no_ack;
		uint8_t wcid; unsigned opts; const char *what;
	} arms[] = {
		{ 'a', 0, 1, 1, 0xff, 0, "broadcast,       No Ack" },
		{ 'b', 0, 0, 0, 0xff, 0, "ucast peer,      Normal" },
		{ 'c', 0, 0, 1, 0xff, 0, "ucast peer,      No Ack" },
		{ 'd', 1, 0, 0, 0xff, 0, "ucast peer ownSA Normal" },
		{ 'e', 1, 0, 1, 0x01, 0, "ucast peer ownSA NoAck wcid1" },
		{ 'f', 1, 0, 1, 0xff, MT_TXOPT_QSEL_MGMT, "ucast NoAck QSEL_MGMT" },
		{ 'g', 1, 0, 1, 0xff, MT_TXOPT_AMPDU | MT_TXOPT_QSEL_MGMT,
		  "ucast NoAck AMPDU+MGMT" },
		{ 'h', 0, 1, 1, 0x01, 0, "broadcast, wcid1 control" },
	};
	static_assert(sizeof arms / sizeof arms[0] <= 8,
	              "txs_arm_pktid gives each pass 8 distinct pktids");

	if (frames <= 0) {
		printf("GATE TXS: FAIL - frames must be positive\n");
		return 2;
	}
	if (peer_str && parse_mac6(peer_str, peer)) {
		printf("GATE TXS: FAIL - bad peer MAC '%s'\n", peer_str);
		return 2;
	}
	/* Parsed before any device work, so a typo costs nothing. */
	rl_given = txs_retry_limit_env(&rl);
	if (rl_given < 0) {
		printf("GATE TXS: FAIL - DEVOURER_TX_RETRY_LIMIT='");
		txs_print_escaped(getenv("DEVOURER_TX_RETRY_LIMIT"));
		printf("' is not a number\n");
		return 2;
	}

	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) return 1;
	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) return 1;
	/* DEVOURER_TX_RETRY_LIMIT=N: programmed with the setter Mt7612uRadio
	 * uses, so this gate can verify it - an unacknowledged Normal arm must
	 * then report a mean retry of N+1 (the limit plus the first attempt).
	 * Unset, the gate does NOT program the register: it runs the initvals
	 * (short 15 / long 31), which is not what a library session runs - that
	 * programs tx.retry_limit, default 0. The word is printed either way, so
	 * every table says which register value its arms ran with. */
	{
		uint32_t cfg = 0;

		if (rl_given && mt7612u_set_retry_limit(&dev, rl)) {
			printf("GATE TXS: FAIL - retry limit %d not set\n", rl);
			return 2;
		}
		if (mt_rr_chk(&dev, MT_TX_RETRY_CFG, &cfg))
			printf("MT_TX_RETRY_CFG read failed\n");
		else if (rl_given)
			printf("retry limit set to %d (MT_TX_RETRY_CFG %08x)\n",
			       rl, cfg);
		else
			printf("retry limit: initvals, not programmed - "
			       "DEVOURER_TX_RETRY_LIMIT unset (MT_TX_RETRY_CFG "
			       "%08x)\n", cfg);
	}

	/* Per-frame time budget for the send and settle deadlines, from the
	 * EFFECTIVE limit (the initvals' short limit 15 when unset). 60 ms
	 * covers the measured ~45 ms 16-attempt ladder and stays the floor, so
	 * a lower limit never shortens the wait. Attempts past the 16th all
	 * back off from CWmax (1023 slots x 9 us, ~4.6 ms mean) plus airtime,
	 * so each one adds 8 ms - at 63 that is 444 ms a frame. */
	{
		const int eff = rl_given ? rl : 15;

		frame_budget_ms = 60.0 + (eff > 15 ? (eff - 15) * 8.0 : 0.0);
	}

	printf("chan %u, HT MCS7 BW20, %zu-byte QoS data, wcid 0xff, %d frames/arm\n",
	       chan, flen, frames);
	printf("peer %02x:%02x:%02x:%02x:%02x:%02x\n",
	       peer[0], peer[1], peer[2], peer[3], peer[4], peer[5]);

	/* The same arms with the MAC receiver off, then on, in one session. The
	 * receiver decides whether an ACK can terminate the ladder, so it is the
	 * variable under test, not a setting. */
	for (rx_on = 0; rx_on <= 1; rx_on++) {
		unsigned a;

		memcpy(ctr.ta[0], dev.macaddr, 6);   /* ownSA arms' addr2 */
		memcpy(ctr.ta[1], src, 6);           /* every other arm's addr2 */
		ctr.acks.store(0);
		ctr.frames.store(0);

		if (mt_mac_start(&dev, MT_RX_DRAIN_NONE)) {
			mt_mac_stop(&dev);
			return 1;
		}
		if (mt_async_start(&dev, rx_on ? ucast_rx_cb : NULL,
		                   rx_on ? (void *)&ctr : NULL)) {
			mt_mac_stop(&dev);
			return 1;
		}
		if (rx_on) {
			if (mt_mac_start(&dev, MT_RX_DRAIN_RING)) {
				/* RX may be half-enabled: silence it before the ring
				 * goes (rx_teardown), or the undrained EP 4 wedges
				 * RX DMA. */
				rx_teardown();
				mt_mac_stop(&dev);
				return 1;
			}
			mt7612u_set_monitor_rx(&dev, 0);
		}
		/* Receiver pass: the 1 Hz PHY tick runs from the waiting loops. */
		last_tick = now_ms();

		/* A WCID entry has to exist before an arm can select it; without
		 * this, wcid 1 names an empty slot and the arm measures nothing
		 * it claims to. */
		if (mt_wcid_setup(&dev, 1, peer)) {
			printf("\nGATE TXS: WCID 1 did not read back as installed - "
			       "arms e and h would report against an empty slot\n");
			io_fail = 1;
		}

		printf("\n  MAC receiver %s\n", rx_on ? "ON" : "OFF");
		printf("  arm  %-28s %7s %9s %8s %9s %6s\n", "configuration",
		       "fps", "entr/sent", "success", "mean rtry", "max");

		for (a = 0; !io_fail && a < sizeof arms / sizeof arms[0]; a++) {
			struct mt7612u_tx_rate rate = { };
			struct txs_sum sum = { 0, 0, 0, 0, 0, 0, 0 };
			const unsigned pktid = txs_arm_pktid(rx_on, a);
			const uint8_t *sa = arms[a].own_sa ? dev.macaddr : src;
			const uint8_t *a1 = arms[a].bcast_a1 ? bcast : peer;
			double t0, wall, send_deadline;
			long n = 0, attempts = 0, submit_fail = 0;
			long status_timeouts = 0;
			int settled = 0;

			rate.phy = MT7612U_PHY_HT;
			rate.mcs = 7;
			rate.nss = 1;
			rate.bw = MT7612U_BW_20;
			rate.no_ack = (unsigned)arms[a].no_ack;

			memset(frame, 0, sizeof frame);
			frame[0] = 0x88;
			memcpy(frame + 4, a1, 6);
			memcpy(frame + 10, sa, 6);
			memcpy(frame + 16, sa, 6);
			frame[24] = arms[a].no_ack ? 0x20 : 0x00;
			memcpy(frame + 26, "MT7612U-TXS", 11);

			/* The previous arm's tail: counted as late, never as ours. */
			if (txs_drain(&dev, &sum, pktid, prev_pktid, TXS_NO_PKTID))
				io_fail = 1;

			/* Bounded twice, like gate_ampdu's wall clock: a submit
			 * that keeps failing must end the arm, not spin it. The
			 * attempt cap allows 3 failures per frame; the clock
			 * allows every frame its full per-frame status wait. */
			t0 = now_ms();
			send_deadline = t0 + frames * (frame_budget_ms +
			                               TXS_FRAME_MARGIN_MS) + 5000.0;
			while (n < frames && attempts < (long)frames * 4 &&
			       now_ms() < send_deadline && !g_stop && !io_fail) {
				frame[22] = (uint8_t)((n & 0xf) << 4);
				frame[23] = (uint8_t)(n >> 4);
				attempts++;
				if (mt_tx_raw(&dev, frame, flen, &rate,
				              arms[a].wcid,
				              MT_TXOPT_TXS | MT_TXOPT_PKTID(pktid) |
				              arms[a].opts) != 0) {
					submit_fail++;
					if (txs_drain(&dev, &sum, pktid, prev_pktid,
					              n > 0 && stale_settled ?
					              stale_pktid : TXS_NO_PKTID))
						io_fail = 1;
					continue;
				}
				n++;
				/*
				 * One frame at a time, for real. mt_tx_raw returns
				 * once the USB transfer is SUBMITTED (the async
				 * pool), not once the MAC has transmitted, so
				 * without this wait submissions run ahead of the
				 * air and can overflow the shallow status FIFO
				 * between drains - entries lost for good. Wait for
				 * this frame's own entry, bounded by the ladder at
				 * the effective limit; on expiry count a status
				 * timeout and go on. (If one entry never arrives,
				 * every later frame of the arm also waits its full
				 * bound - the count then says how many frames were
				 * waited on without their entry, not which one is
				 * missing.)
				 */
				{
					const double fdl = now_ms() + frame_budget_ms +
					                   TXS_FRAME_MARGIN_MS;

					do {
						if (txs_drain(&dev, &sum, pktid,
						              prev_pktid,
						              stale_settled ? stale_pktid
						                            : TXS_NO_PKTID)) {
							io_fail = 1;
							break;
						}
						if (sum.entries >= n) break;
						txs_tick(rx_on, &last_tick);
						mt_usleep(500);
					} while (now_ms() < fdl && !g_stop);
					if (!io_fail && sum.entries < n)
						status_timeouts++;
				}
			}
			/*
			 * Wait for any status the MAC still owes us before
			 * moving on. With the per-frame wait above this is
			 * mostly a no-op; it stays for the frames whose own
			 * wait timed out, whose entries would otherwise be
			 * counted against the NEXT arm (late) instead of this
			 * one.
			 *
			 * Bounded by the worst case that matters: `frames` at
			 * the full ladder for the effective retry limit
			 * (frame_budget_ms), plus slack.
			 */
			{
				double deadline = now_ms() + frames * frame_budget_ms +
				                  2000.0;

				while (sum.entries < n && now_ms() < deadline
				       && !g_stop && !io_fail) {
					txs_tick(rx_on, &last_tick);
					mt_usleep(2000);
					if (txs_drain(&dev, &sum, pktid, prev_pktid,
					              n > 0 && stale_settled ?
					              stale_pktid : TXS_NO_PKTID))
						io_fail = 1;
				}
				settled = (sum.entries >= n);
			}
			wall = now_ms() - t0;

			/* fps here is frames over the whole arm INCLUDING each
			 * frame's wait for its own status entry - i.e. per-frame
			 * submit-to-status time, plus any status timeouts and the
			 * final settle. It is NOT a steady-state throughput
			 * figure; a row with status timeouts or UNSETTLED is
			 * dominated by the waits. The retry columns are the
			 * point of this gate. entr is this arm's own-pktid
			 * entries, uncapped: more than `sent` would mean the MAC
			 * filed duplicates, and is shown as such rather than
			 * clipped. */
			{
				/* The retry mean is over the entries whose EXT
				 * word is their own. */
				const long rtry_n = sum.entries - sum.stale_ext;

				printf("  %c    %-28s %7.0f %4ld/%-4ld %8ld %9.1f %6ld%s\n",
				       arms[a].tag, arms[a].what, n * 1000.0 / wall,
				       sum.entries, n, sum.success,
				       rtry_n ? (double)sum.retry_total / rtry_n : 0.0,
				       sum.retry_max, settled ? "" : "  UNSETTLED");
			}
			if (submit_fail || status_timeouts || sum.late_prev ||
			    sum.foreign || sum.stale_ext || n < frames)
				printf("       (pktid %u: %ld submit failures, %ld/%d "
				       "frames sent, %ld per-frame status timeouts, "
				       "%ld late entries from the previous arm, "
				       "%ld foreign, %ld stale-EXT entries claimed)\n",
				       pktid, submit_fail, n, frames, status_timeouts,
				       sum.late_prev, sum.foreign, sum.stale_ext);
			total_entries += sum.entries;
			total_foreign += sum.foreign + sum.late_prev;
			prev_pktid = pktid;
			/* More entries than frames would be MAC duplicates: then a
			 * stale-pktid entry is not provably ours either. An arm that
			 * sent nothing popped nothing and leaves both as they were. */
			if (n > 0) {
				stale_pktid = pktid;
				stale_settled = (sum.entries == n);
			}
			if (io_fail) {
				printf("       (arm %c: a status-FIFO read FAILED - "
				       "the row above is incomplete)\n", arms[a].tag);
				break;
			}
			if (g_stop) break;
			mt_usleep(100000);
		}
		if (rx_on)
			printf("  (receiver saw %lu frames, %lu ACKs to our TAs)\n",
			       (unsigned long)ctr.frames.load(),
			       (unsigned long)ctr.acks.load());

		/* Receiver pass: RX off before the EP 4 ring goes (rx_teardown).
		 * TX-only pass: nothing was receiving, so the ring just stops. */
		if (rx_on)
			rx_teardown();
		else
			mt_async_stop(&dev);
		mt_mac_stop(&dev);
		if (g_stop || io_fail) break;
	}

	/* After the pass's normal teardown: a failed status read or an
	 * uninstalled WCID means the columns are not a measurement, so it is a
	 * device failure, not a report. */
	if (io_fail) {
		printf("\nGATE TXS: FAIL - a status-FIFO read or the WCID setup "
		       "failed (see above); the measurement is incomplete\n");
		return 1;
	}
	if (g_stop) {
		printf("\nGATE TXS: INTERRUPTED\n");
		return 3;
	}
	if (total_entries == 0) {
		/* Every column above is then a default, not a reading. */
		printf("\nGATE TXS: FAIL - no arm filed a single status entry "
		       "with its own pktid (%ld late/foreign). The measurement did "
		       "not happen - check MT_TXOPT_TXS reached the txwi pktid.\n",
		       total_foreign);
		return 1;
	}
	printf("\nGATE TXS: reported. An arm with a zero entry count filed no\n"
	       "status - read nothing into its retry columns.\n");
	return 0;
}

/* Somebody has to read EP 4 whenever MAC RX is on; this gate does not care
 * what arrives, only that the endpoint keeps being drained. */
static void drain_cb(void *user, const void *frame, size_t len,
                     const struct mt7612u_rx_info *info)
{
	(void)frame; (void)len; (void)info;
	((std::atomic<unsigned long> *)user)->fetch_add(
	    1, std::memory_order_relaxed);
}

/* Capability descriptor, TSF and 40 MHz. */
static int gate_caps(uint8_t chan)
{
	struct mt7612u_caps c;
	/* atomic: drain_cb runs on the RX event thread. */
	std::atomic<unsigned long> drained{0};
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
	if (mt_mac_start(&dev, MT_RX_DRAIN_RING)) {
		rx_teardown(); mt_mac_stop(&dev); return 1;
	}

	mt7612u_get_caps(&dev, &c);
	printf("caps: %s rev 0x%08x  %dTx%dRx  bw_mask 0x%02x (20%s%s)\n",
	       c.chip_name, c.rev, c.nss_tx, c.nss_rx, c.bw_mask,
	       (c.bw_mask & 2) ? "/40" : "", (c.bw_mask & 4) ? "/80" : "");
	printf("      5 GHz %u-%u MHz, 2.4 GHz %u-%u MHz\n",
	       c.band_5g_min_mhz, c.band_5g_max_mhz,
	       c.band_2g_min_mhz, c.band_2g_max_mhz);
	printf("      ampdu_tx=%u per_chain_rssi=%u narrowband=%u fast_retune=%u tsf_write=%u\n",
	       c.ampdu_tx, c.per_chain_rssi, c.narrowband, c.fast_retune, c.tsf_write);
	printf("      max MPDU: tx %u  rx %u  (rx is MT_MAX_LEN_CFG 0x%03x on air,\n"
	       "                             less the 4-byte FCS)\n",
	       c.max_mpdu_tx, c.max_mpdu_rx,
	       mt_rr(&dev, MT_MAX_LEN_CFG) & 0xfff);

	int8_t rssi_offset[2], lna_gain;
	mt_rx_corr_unpack(dev.cal.rx_corr.load(std::memory_order_relaxed),
	                  rssi_offset, &lna_gain);
	printf("\nRX gain from EEPROM: rssi_offset=[%d,%d] lna_gain=%d "
	       "high_gain=[%d,%d] mcu_gain=0x%08x\n",
	       rssi_offset[0], rssi_offset[1], lna_gain,
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
	 * 200 ms and require the clock to have advanced by that much. The raw
	 * words stay raw on purpose - this is the measurement of the word order -
	 * but they are checked: an all-ones failure would otherwise pose as a
	 * word-order answer. */
	{
		uint32_t a0 = 0, a1 = 0, b0 = 0, b1 = 0;
		int64_t d_hi0, d_lo0;
		bool raw_ok;

		raw_ok = !mt_rr_chk(&dev, MT_TSF_TIMER_DW0, &a0) &&
		         !mt_rr_chk(&dev, MT_TSF_TIMER_DW1, &a1);
		mt_usleep(200000);
		raw_ok = raw_ok && !mt_rr_chk(&dev, MT_TSF_TIMER_DW0, &b0) &&
		         !mt_rr_chk(&dev, MT_TSF_TIMER_DW1, &b1);
		if (!raw_ok) {
			printf("\nTSF raw: read failed - no word-order verdict\n");
			bad++;
		}

		d_hi0 = (int64_t)((((uint64_t)b0 << 32) | b1) - (((uint64_t)a0 << 32) | a1));
		d_lo0 = (int64_t)((((uint64_t)b1 << 32) | b0) - (((uint64_t)a1 << 32) | a0));

		if (raw_ok) {
			printf("\nTSF raw: DW0 %08x -> %08x   DW1 %08x -> %08x\n", a0, b0, a1, b1);
			printf("  as (DW0<<32)|DW1 : delta %lld us\n", (long long)d_hi0);
			printf("  as (DW1<<32)|DW0 : delta %lld us\n", (long long)d_lo0);
			printf("  over a 200000 us sleep -> DW%d is the low word\n",
			       (d_lo0 > 150000 && d_lo0 < 400000) ? 0 : 1);
		}

		int rc = mt7612u_read_tsf_chk(&dev, &t1);

		mt_usleep(200000);
		rc |= mt7612u_read_tsf_chk(&dev, &t2);
		if (rc) {
			printf("  mt7612u_read_tsf_chk(): read failed\n");
			bad++;
		} else {
			delta = (int64_t)(t2 - t1);
			printf("  mt7612u_read_tsf_chk(): delta %lld us  %s\n", (long long)delta,
			       (delta > 150000 && delta < 400000) ? "OK" : "*** WRONG ORDER ***");
			if (delta < 150000 || delta > 400000) bad++;
		}
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

	rx_teardown();
	mt_mac_stop(&dev);
	printf("\n%lu frames drained from EP 4 while the receiver was on\n",
	       drained.load());
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
struct ack_ctx {
	std::atomic<unsigned long> to_us, retry_to_us, other;
};

static const uint8_t g_ack_mac[6] = { 0x02, 0x4d, 0x54, 0x76, 0x12, 0xaa };

static void ack_cb(void *user, const void *frame, size_t len,
                   const struct mt7612u_rx_info *info)
{
	struct ack_ctx *c = (struct ack_ctx *)user;
	const uint8_t *f = (const uint8_t *)frame;

	(void)info;
	if (len < 16) return;
	if (memcmp(f + 4, g_ack_mac, 6) != 0) {
		c->other.fetch_add(1, std::memory_order_relaxed);
		return;
	}
	c->to_us.fetch_add(1, std::memory_order_relaxed);
	if (f[1] & 0x08)                        /* FC Retry bit */
		c->retry_to_us.fetch_add(1, std::memory_order_relaxed);
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
	if (mt_mac_start(&dev, MT_RX_DRAIN_RING)) {
		rx_teardown(); mt_mac_stop(&dev); return 1;
	}
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
			rx_teardown();
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
		rx_teardown();
		mt_mac_stop(&dev);
		return 1;
	}
	printf("listening %d s ...\n", secs);
	/* wait_ms, not mt_usleep: it honours SIGINT, where the old cast-to-
	 * unsigned sleep both ignored the signal and turned a negative argument
	 * into roughly 49 days with the receiver left running. */
	if (!wait_ticking(secs * 1000.0)) {
		printf("GATE ack: interrupted\n");
		rx_teardown();
		mt_mac_stop(&dev);
		return 1;
	}
	rx_teardown();
	printf("  stimulus frames addressed to the responder MAC: %lu (retries %lu)\n",
	       off.to_us.load(), off.retry_to_us.load());

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
    if (mt_mac_start(&dev, MT_RX_DRAIN_RING)) {
        rx_teardown(); mt_mac_stop(&dev); return 1;
    }
    mt7612u_set_monitor_rx(&dev, 0);
    mt7612u_link_stats_start(&dev);

    wait_ticking(secs * 1000.0);
    mt7612u_link_stats(&dev, &st);
    rx_teardown();
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
/* atomic: drain_cb runs on the RX event thread. */
static std::atomic<unsigned long> linkstat_drained{0};

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
	if (mt_mac_start(&dev, with_rx)) {
		if (with_rx) rx_teardown();
		mt_mac_stop(&dev);
		return 1;
	}
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
		if (mt7612u_link_stats(&dev, &st)) {
			if (with_rx) rx_teardown();
			return 1;
		}
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
		rx_teardown();
		printf("  %lu frames reached the ring over the run\n",
		       linkstat_drained.load());
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
	if (mt_mac_start(&dev, MT_RX_DRAIN_NONE)) { mt_mac_stop(&dev); return 1; }

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
	const uint8_t *f = (const uint8_t *)frame;
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
	if (mt_mac_start(&dev, MT_RX_DRAIN_RING)) {
		rx_teardown(); mt_mac_stop(&dev); return 1;
	}
	mt7612u_set_monitor_rx(&dev, 0);

	printf("RX on ch%u for %d s, filtering our own magic\n", chan, secs);
	wait_ticking(secs * 1000.0);
	rx_teardown();
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
	if (mt_mac_start(&dev, MT_RX_DRAIN_NONE)) { mt_mac_stop(&dev); return 1; }

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
	if (mt_mac_start(&dev, MT_RX_DRAIN_NONE)) { mt_mac_stop(&dev); return 1; }

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
				.bw = (enum mt7612u_bw)bw,
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
	if (mt_mac_start(&dev, MT_RX_DRAIN_NONE)) { mt_mac_stop(&dev); return 1; }

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
					.bw = (enum mt7612u_bw)bw,
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
	if (mt_mac_start(&dev, MT_RX_DRAIN_NONE)) { mt_mac_stop(&dev); return 1; }

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
		                             .bw = (enum mt7612u_bw)bw,
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
	if (mt_mac_start(&dev, MT_RX_DRAIN_NONE)) { mt_mac_stop(&dev); return 1; }

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

/* Gate TSF-WRITE: characterises whether this part has a TSF load path at all.
 * The DW0/DW1 registers hold the free-running counter and do not load it:
 * every sequence tried below was ignored on two units (docs/mt7612u.md), with
 * the clock first verified alive (a dead or wedged counter would also read as
 * "no load path").
 * That measurement is why this backend reports AdapterCaps::tsf_write_ok false
 * and leaves WriteTsf on the refusing IRadio default. A future firmware that
 * enables loading must fail this gate so the contract is revisited. Every arm
 * builds its target from a fresh read so a stale write cannot look like a
 * take. */

/* The wall-rate window the clock control and every arm's liveness check
 * share: a kTsfLiveSleepUs sleep must advance the counter by more than
 * kTsfLiveMinUs and less than kTsfLiveMaxUs. */
static const unsigned kTsfLiveSleepUs = 50000;
static const int64_t kTsfLiveMinUs = 10000;
static const int64_t kTsfLiveMaxUs = 200000;

/* How far from its target a readback may land and still count as a take: the
 * 20 ms settle plus control round trips, with margin. */
static const int64_t kTsfTakeWindowUs = 200000;

/* How close to a low-word wrap an arm may start: the 5 s target plus the arm's
 * own time, which kTsfArmMaxMs bounds, with margin. */
static const uint64_t kTsfWrapGuardUs = 6000000;

/* Host-time bounds that keep an arm's verdict meaningful. mt_vendor_req
 * retries a timed-out EP0 transfer silently (no io_err on eventual success),
 * so a stall between a real load and the readback could push the error past
 * kTsfTakeWindowUs and print no-op on a part that loads - and a slow enough arm
 * could outlast the wrap guard. An arm slower than either bound is
 * inconclusive, never "ignored". */
static const double kTsfWriteToReadbackMaxMs = 150.0; /* < kTsfTakeWindowUs */
static const double kTsfArmMaxMs = 1000.0;             /* << kTsfWrapGuardUs */

static bool tsf_live(int64_t delta)
{
	return delta > kTsfLiveMinUs && delta < kTsfLiveMaxUs;
}

/* The library's checked, wrap-safe TSF read. A failed transfer must stay
 * distinguishable from a stopped clock here, or a flaky cable is reported as
 * a dead timer. False means a transfer failed and *out is not a TSF. */
static bool tsf_read_chk(uint64_t *out)
{
	return mt7612u_read_tsf_chk(&dev, out) == 0;
}

/* A fresh base for one arm, clear of a low-word wrap. Each arm judges a take
 * with a 64-bit compare against base + offset. Near a wrap that compare lies
 * in both directions: a DW0-only load whose 5 s target carries into DW1 reads
 * back ~2^32 short and prints no-op (a false PASS), and a natural carry
 * between the base read and the readback makes the DW1-only arm read as a
 * take. Waiting out the last kTsfWrapGuardUs before a wrap removes both. */
static bool tsf_fresh_base(uint64_t *base)
{
	uint32_t lo;

	if (!tsf_read_chk(base))
		return false;
	lo = (uint32_t)*base;
	if (lo > 0xffffffffull - kTsfWrapGuardUs) {
		mt_usleep((unsigned)(0x100000000ull - lo) + 100000);
		if (!tsf_read_chk(base))
			return false;
	}
	return true;
}

/* 1 = alive, 0 = the counter is not advancing, -1 = a read failed (the
 * transport, not the clock). */
static int tsf_clock_control(void)
{
	uint64_t t0, t1;
	int64_t d;

	if (!tsf_read_chk(&t0))
		return -1;
	mt_usleep(kTsfLiveSleepUs);
	if (!tsf_read_chk(&t1))
		return -1;
	d = (int64_t)(t1 - t0);
	printf("  %-34s t0=%llu t1=%llu delta=%+lld  %s\n",
	       "clock control", (unsigned long long)t0, (unsigned long long)t1,
	       (long long)d, tsf_live(d) ? "ALIVE" : "DEAD");
	return tsf_live(d) ? 1 : 0;
}

enum tsf_write_order {
	TSF_DW0_THEN_DW1,
	TSF_DW1_THEN_DW0,
	TSF_DW0_ONLY,
	TSF_DW1_ONLY,
};

struct tsf_arm {
	const char *label;
	enum tsf_write_order order;
	uint64_t offset;  /* target = fresh base + offset */
	/* Clear MT_BEACON_TIME_CFG_TIMER_EN around the write. Restoring TIMER_EN
	 * restarts the counter from ~0, which would wipe a load before the normal
	 * readback, so this arm also reads the counter while the timer is still
	 * off and judges the take on that read too. */
	bool stop_timer;
};

/* One load attempt. Returns 1 on a take, 0 on a no-op with the clock still
 * running, and -1 when the arm produced no usable conclusion: a failed read,
 * or a counter that stopped during the arm. A stalled counter prints no-op and
 * would otherwise let the gate PASS on a dead clock, so -1 is a failure of the
 * run rather than a "the write was ignored" datum. A take is the readback
 * landing on the target; what the clock does after a take does not decide, so
 * a load that takes and then stalls is still a take. */
static int tsf_arm_run(const struct tsf_arm *a)
{
	uint32_t cfg = 0;
	uint64_t base, target, r1, r2, held = 0;
	int64_t err, rate;
	double t_base, t_write, t_r1, t_end;
	bool took, live, slow;

	if (!tsf_fresh_base(&base)) {
		printf("  %-34s skipped: TSF read failed\n", a->label);
		return -1;
	}
	t_base = now_ms();
	target = base + a->offset;

	if (a->stop_timer) {
		/* mt_rr returns 0xffffffff on a failed transfer, which must never be
		 * written back as configuration. If the read fails, skip the arm: the
		 * accumulated I/O error makes the gate FAIL at the sweep boundary. */
		if (mt_rr_chk(&dev, MT_BEACON_TIME_CFG, &cfg)) {
			printf("  %-34s skipped: MT_BEACON_TIME_CFG read failed\n", a->label);
			return -1;
		}
		mt_wr(&dev, MT_BEACON_TIME_CFG, cfg & ~MT_BEACON_TIME_CFG_TIMER_EN);
	}

	t_write = now_ms();
	switch (a->order) {
	case TSF_DW0_THEN_DW1:
		mt_wr(&dev, MT_TSF_TIMER_DW0, (uint32_t)target);
		mt_wr(&dev, MT_TSF_TIMER_DW1, (uint32_t)(target >> 32));
		break;
	case TSF_DW1_THEN_DW0:
		mt_wr(&dev, MT_TSF_TIMER_DW1, (uint32_t)(target >> 32));
		mt_wr(&dev, MT_TSF_TIMER_DW0, (uint32_t)target);
		break;
	case TSF_DW0_ONLY:
		mt_wr(&dev, MT_TSF_TIMER_DW0, (uint32_t)target);
		break;
	case TSF_DW1_ONLY:
		mt_wr(&dev, MT_TSF_TIMER_DW1, (uint32_t)(target >> 32));
		break;
	}

	if (a->stop_timer) {
		bool held_ok = tsf_read_chk(&held);

		/* Restore before judging, so a failed read cannot leave the
		 * timer off. */
		mt_wr(&dev, MT_BEACON_TIME_CFG, cfg);
		if (!held_ok) {
			printf("  %-34s TSF read with the timer off failed\n", a->label);
			return -1;
		}
	}

	mt_usleep(20000);
	if (!tsf_read_chk(&r1)) {
		printf("  %-34s TSF readback failed\n", a->label);
		return -1;
	}
	t_r1 = now_ms();
	mt_usleep(kTsfLiveSleepUs);
	if (!tsf_read_chk(&r2)) {
		printf("  %-34s TSF liveness read failed\n", a->label);
		return -1;
	}
	t_end = now_ms();
	err = (int64_t)(r1 - target);
	rate = (int64_t)(r2 - r1);
	took = llabs(err) < kTsfTakeWindowUs ||
	       (a->stop_timer && llabs((int64_t)(held - target)) < kTsfTakeWindowUs);
	live = tsf_live(rate);
	slow = t_r1 - t_write > kTsfWriteToReadbackMaxMs ||
	       t_end - t_base > kTsfArmMaxMs;
	if (a->stop_timer)
		printf("  %-34s held=%llu (read with TIMER_EN clear)\n", "",
		       (unsigned long long)held);
	printf("  %-34s target=%llu read=%llu err=%+lld delta50ms=%+lld  %s\n",
	       a->label, (unsigned long long)target, (unsigned long long)r1,
	       (long long)err, (long long)rate,
	       took ? "TOOK" : slow ? "inconclusive (arm too slow)"
	            : live ? "no-op" : "no-op (clock stalled)");
	if (took)
		return 1;
	if (slow) {
		printf("  %-34s write->readback %.1f ms (max %.0f), arm %.1f ms (max %.0f)\n",
		       "", t_r1 - t_write, kTsfWriteToReadbackMaxMs, t_end - t_base,
		       kTsfArmMaxMs);
		return -1;
	}
	return live ? 0 : -1;
}

/* Runs every arm, even after a take, so the printout is the whole picture. */
static void tsf_arms_run(const struct tsf_arm *arms, size_t n, bool *any,
                         bool *invalid)
{
	for (size_t i = 0; i < n; i++) {
		int r = tsf_arm_run(&arms[i]);

		if (r > 0)
			*any = true;
		if (r < 0)
			*invalid = true;
	}
}

static int gate_tsfwrite(uint8_t chan)
{
	/* With the MAC running (the state a live link is in): both word orders,
	 * then each word alone. The high-word-only target sets a high word that
	 * differs from the live one, so the arm is not vacuous. */
	static const struct tsf_arm mac_on[] = {
		{ "MAC on, both words DW0,DW1", TSF_DW0_THEN_DW1, 5000000, false },
		{ "MAC on, both words DW1,DW0", TSF_DW1_THEN_DW0, 5000000, false },
		{ "MAC on, high word (DW1) only", TSF_DW1_ONLY, 1ull << 32, false },
		{ "MAC on, low word (DW0) only", TSF_DW0_ONLY, 5000000, false },
	};
	/* With the MAC stopped, and with the free-running timer disabled. */
	static const struct tsf_arm mac_off[] = {
		{ "MAC off, both words", TSF_DW0_THEN_DW1, 5000000, false },
		{ "MAC off, timer off, both words", TSF_DW0_THEN_DW1, 5000000, true },
	};
	uint32_t cfg;
	bool any = false, invalid = false;
	int alive;

	if (mt_eeprom_init(&dev)) {
		printf("GATE TSF-WRITE: FAIL - eeprom_init failed\n");
		return 1;
	}
	if (mt_init_hardware(&dev, NULL)) {
		printf("GATE TSF-WRITE: FAIL - init_hardware failed\n");
		return 1;
	}
	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) {
		printf("GATE TSF-WRITE: FAIL - set_channel failed\n");
		return 1;
	}

	if (mt_rr_chk(&dev, MT_BEACON_TIME_CFG, &cfg)) {
		printf("GATE TSF-WRITE: FAIL - MT_BEACON_TIME_CFG read failed\n");
		return 1;
	}
	printf("MT_BEACON_TIME_CFG=0x%08x TIMER_EN=%u TBTT_EN=%u BEACON_TX=%u SYNC_MODE=%u\n",
	       cfg, !!(cfg & MT_BEACON_TIME_CFG_TIMER_EN),
	       !!(cfg & MT_BEACON_TIME_CFG_TBTT_EN),
	       !!(cfg & MT_BEACON_TIME_CFG_BEACON_TX),
	       (unsigned)FIELD_GET(MT_BEACON_TIME_CFG_SYNC_MODE, cfg));

	/* A dead counter reads exactly like a counter that ignores loads, and a
	 * failed read reads like a dead counter - keep the three apart. */
	alive = tsf_clock_control();
	if (alive < 0) {
		printf("\nGATE TSF-WRITE: FAIL - TSF read failed (%u USB transfer error(s)); "
		       "a transport fault, not a clock verdict\n", mt_io_errors(&dev));
		return 1;
	}
	if (alive == 0) {
		printf("\nGATE TSF-WRITE: FAIL - the TSF clock is not running; no load conclusion\n");
		return 1;
	}

	if (mt_mac_start(&dev, MT_RX_DRAIN_NONE)) {
		/* The start enables TX control before its DMA-idle poll, so stop
		 * before bailing out rather than leaving the MAC half-started. */
		mt_mac_stop(&dev);
		printf("\nGATE TSF-WRITE: FAIL - mt_mac_start failed\n");
		return 1;
	}
	tsf_arms_run(mac_on, sizeof mac_on / sizeof mac_on[0], &any, &invalid);
	mt_mac_stop(&dev);
	tsf_arms_run(mac_off, sizeof mac_off / sizeof mac_off[0], &any, &invalid);

	/* A mid-run transport failure reads as all-ones, which every arm would
	 * otherwise report as "ignored" - the exact false conclusion this gate
	 * exists to prevent. A stalled clock is the same class of false
	 * conclusion: no arm can be read as "the write was ignored" if the
	 * counter was not advancing while the arm ran. */
	if (invalid || mt_io_errors(&dev) != 0) {
		printf("\nGATE TSF-WRITE: FAIL - %u USB transfer(s) failed%s; no load conclusion\n",
		       mt_io_errors(&dev),
		       invalid ? " and/or an arm was inconclusive (failed read, stalled clock, or too slow)"
		               : " during the sweep");
		return 1;
	}

	printf("\nGATE TSF-WRITE: %s\n", any
	       ? "FAIL - a write sequence takes; tsf_write_ok and WriteTsf must be revisited"
	       : "PASS - confirmed: no sequence loads the TSF, so tsf_write_ok is false");
	return any ? 1 : 0;
}

/* Gate TSF-WRAP: the TSF read across the 2^32 us low-word wrap, judged
 * against a truth that is not the read under test.
 *
 * WHY A GATE. The two TSF halves are not latched, so a read is only wrong for
 * the few hundred microseconds around a low-word wrap, and bring-up restarts
 * the counter near 0, so the first wrap is 71.6 min in. The headless
 * mt7612u_tsf_read cell holds the read discipline against a scripted counter;
 * this holds it against the part. A PASS takes ~72 min.
 *
 * TRUTH. A least-squares host-clock model, tsf = t0 + a + b * (host - h0),
 * fitted over one read per 100 ms for the preceding 60 s. A read latches at an
 * unknown instant inside its control transfer, and one transfer can take 10 ms
 * on a busy USB 2.0 bus, so a read is judged against the model over the host
 * interval that bracketed it (+-kTsfWrapTolUs), never at one timestamp. A torn
 * read misses by 2^32 us.
 *
 * THREE PARTS.
 *  1. Continuous: mt7612u_read_tsf_chk in a tight loop for the whole run. Any
 *     failed read or backwards step fails the gate, and every read within
 *     kTsfWrapWindowUs of the wrap must sit within kTsfWrapTolUs of the model.
 *  2. Forced: once the counter is within 3 s of the wrap, the mt7612u::tsf_read template
 *     the library compiles runs with a reader that sleeps to a schedule, so the
 *     wrap lands in the chosen gap of the read (gap 1: between the first high
 *     and the low read; gap 2: between the low and the second high). It must
 *     take the retry and land within kTsfWrapTolUs of the model.
 *  3. Positive control, interleaved with part 2: a plain DW0-then-DW1 read with
 *     the wrap between its halves. It must miss the model by ~2^32 us, or the
 *     rig cannot see the tear it exists to catch - and a DW0 read that froze
 *     DW1 would make it read correctly.
 *
 * WHAT IT DOES NOT SHOW. Part 2 occupies the wrap instant, so the exported
 * function in part 1 never itself takes the retry across it; the forced read
 * is the same template, driven through a different reader.
 *
 * Smoke mode: wrap_bits < 32 treats the carry out of that bit of the low word
 * as the "wrap" (16.7 s at 24). That checks the schedule and the model in
 * seconds but cannot tear a read, so it reports SMOKE, never PASS. */
static const int64_t kTsfWrapTolUs = 5000;
/* A gone device fails every read at loop speed; stop rather than spin for the
 * rest of the run (an interrupted run reached 192 million failed reads). */
static const uint64_t kTsfWrapMaxConsecFails = 100;
/* A read this slow still judges (it is judged over its own interval), but it is
 * too coarse to fit the model from. Decoupled from the tolerance: on a busy
 * USB 2.0 bus a 3-transfer read can take longer than the tolerance, and
 * refusing to fit from those would leave the model empty. */
static const int64_t kTsfWrapFeedMaxUs = 20000;
/* The counter runs at the host's rate to within a crystal's error. A fit
 * outside this band is a frozen or wedged counter, not a clock to predict a
 * wrap from. */
static const double kTsfWrapRateMin = 0.9, kTsfWrapRateMax = 1.1;
/* How far either side of the wrap the forced read places its accesses. Wide
 * enough that one slow (~10 ms) control transfer cannot move a latch across
 * the wrap. */
static const int64_t kTsfWrapMarginUs = 40000;
static const int64_t kTsfWrapWindowUs = 120000000;
static const int kTsfWrapModelPts = 600;

static int64_t mono_us(void)
{
	struct timespec t;

	clock_gettime(CLOCK_MONOTONIC, &t);
	return (int64_t)t.tv_sec * 1000000 + t.tv_nsec / 1000;
}

/* Sleep until a mono_us() instant. Not clock_nanosleep(TIMER_ABSTIME): macOS
 * has neither, and reaching for a second clock source would put an epoch
 * difference between the schedule and every timestamp around it. Sleeping the
 * remaining delta and re-checking keeps mono_us() the only clock; an
 * interrupted or short sleep just goes round again. */
static void sleep_until_us(int64_t at)
{
	for (;;) {
		const int64_t left = at - mono_us();

		if (left <= 0 || g_stop)
			return;
		std::this_thread::sleep_for(std::chrono::microseconds(left));
	}
}

/* A ring of (host, tsf) points and the line through them. */
struct tsf_model {
	double h[kTsfWrapModelPts], t[kTsfWrapModelPts];
	int n, head;
	double h0, t0, a, b;
	bool ok;
};

static void tsf_model_add(struct tsf_model *m, double h, double t)
{
	m->h[m->head] = h;
	m->t[m->head] = t;
	m->head = (m->head + 1) % kTsfWrapModelPts;
	if (m->n < kTsfWrapModelPts)
		m->n++;
}

static void tsf_model_fit(struct tsf_model *m)
{
	const int first = m->n < kTsfWrapModelPts ? 0 : m->head;
	double sx = 0, sy = 0, sxx = 0, sxy = 0, den;

	m->ok = false;
	if (m->n < 50)
		return;
	m->h0 = m->h[first];
	m->t0 = m->t[first];
	for (int i = 0; i < m->n; i++) {
		int k = (first + i) % kTsfWrapModelPts;
		double x = m->h[k] - m->h0, y = m->t[k] - m->t0;

		sx += x; sy += y; sxx += x * x; sxy += x * y;
	}
	den = m->n * sxx - sx * sx;
	if (den == 0)
		return;
	m->b = (m->n * sxy - sx * sy) / den;
	m->a = (sy - m->b * sx) / m->n;
	m->ok = true;
}

static double tsf_model_at(const struct tsf_model *m, double h)
{
	return m->t0 + m->a + m->b * (h - m->h0);
}

/* Whether a value read between host instants h_start and h_end is on the
 * model: between the model at the start and at the end, give or take the
 * tolerance. *err gets the miss against the interval's midpoint, for printing. */
static bool tsf_model_holds(const struct tsf_model *m, double v, double h_start,
                            double h_end, double *err)
{
	*err = v - tsf_model_at(m, (h_start + h_end) / 2);
	return v >= tsf_model_at(m, h_start) - kTsfWrapTolUs &&
	       v <= tsf_model_at(m, h_end) + kTsfWrapTolUs;
}

static int gate_tsfwrap(int gap, int wrap_bits, double max_min)
{
	static struct tsf_model model;
	int64_t last_pt = 0, last_status = 0, wrap_host = 0;
	uint64_t prev = 0, reads = 0, fails = 0, backwards = 0, checked = 0, off_model = 0;
	uint64_t consec_fails = 0;
	bool gone = false;
	double worst = 0;
	bool have = false, forced = false, f_retried = false, f_held = false, c_held = false;
	/* The forced read's first low word, for the gap check at the verdict. */
	uint32_t lo_first = 0;
	bool have_lo_first = false;
	int f_rc = -1;
	double f_err = 0, c_err = 0;

	/* Argument checks come before the arithmetic they feed: 1ull << wrap_bits
	 * is undefined for a wrap_bits outside the word, and a non-finite or huge
	 * max_min has no int64 to convert to. atof() gives 0 for a non-number,
	 * which the positive check below refuses. */
	if (gap != 1 && gap != 2) {
		printf("GATE TSF-WRAP: FAIL - gap must be 1 or 2\n");
		return 2;
	}
	if (wrap_bits < 20 || wrap_bits > 32) {
		printf("GATE TSF-WRAP: FAIL - wrap_bits must be 20..32\n");
		return 2;
	}
	if (!isfinite(max_min) || max_min <= 0 || max_min > 24 * 60) {
		printf("GATE TSF-WRAP: FAIL - max_min must be a positive number of minutes, at most a day\n");
		return 2;
	}

	const uint64_t period = 1ull << wrap_bits, mask = period - 1;
	const int64_t t_start = mono_us();
	int64_t deadline = t_start + (int64_t)(max_min * 60e6);
	/* Register reads only: the MAC stays as mt_init_hardware left it (stopped),
	 * so there is nothing for an early return to unwind. */
	if (mt_eeprom_init(&dev) || mt_init_hardware(&dev, NULL) ||
	    mt_set_channel(&dev, 6, MT7612U_BW_20)) {
		printf("GATE TSF-WRAP: FAIL - bring-up failed\n");
		return 1;
	}

	while (!g_stop && mono_us() < deadline) {
		uint64_t v;
		const int64_t h0 = mono_us();

		if (mt7612u_read_tsf_chk(&dev, &v)) {
			fails++;
			have = false;
			if (++consec_fails >= kTsfWrapMaxConsecFails) {
				gone = true;
				break;
			}
			continue;
		}
		consec_fails = 0;
		const int64_t h1 = mono_us();
		const double hm = (h0 + h1) / 2.0;

		reads++;
		if (have && (int64_t)(v - prev) < 0) {
			backwards++;
			printf("  backwards: %llu -> %llu\n", (unsigned long long)prev,
			       (unsigned long long)v);
		}
		prev = v;
		have = true;

		const bool near = wrap_host
			? h1 < wrap_host + kTsfWrapWindowUs
			: (v & mask) > mask - ((uint64_t)kTsfWrapWindowUs % period);
		double e = 0;
		bool holds = true;

		tsf_model_fit(&model);
		if (model.ok)
			holds = tsf_model_holds(&model, (double)v, h0, h1, &e);
		if (model.ok && near) {
			checked++;
			if (fabs(e) > worst)
				worst = fabs(e);
			if (!holds && ++off_model <= 5)
				printf("  off model: tsf=%llu err=%+.0f us (read took %lld us)\n",
				       (unsigned long long)v, e, (long long)(h1 - h0));
		}
		/* Feed the model only fast reads it agrees with, once it exists. */
		if (h1 - last_pt > 100000 && h1 - h0 <= kTsfWrapFeedMaxUs && holds) {
			last_pt = h1;
			tsf_model_add(&model, hm, (double)v);
		}
		if (h1 - last_status > 60000000) {
			last_status = h1;
			printf("  t=%4.0fs tsf=%llu reads=%llu fails=%llu backwards=%llu checked=%llu off_model=%llu worst=%.0f us\n",
			       (h1 - t_start) / 1e6, (unsigned long long)v,
			       (unsigned long long)reads, (unsigned long long)fails,
			       (unsigned long long)backwards, (unsigned long long)checked,
			       (unsigned long long)off_model, worst);
			fflush(stdout);
		}

		if (forced || !model.ok || (v & mask) <= mask - (3000000ull % period))
			continue;

		/* Parts 2 and 3, once. */
		if (model.b < kTsfWrapRateMin || model.b > kTsfWrapRateMax) {
			printf("\nGATE TSF-WRAP: FAIL - the counter runs at %.6f x the host clock; not a clock to time a wrap from\n",
			       model.b);
			return 1;
		}
		forced = true;
		const int64_t w = (int64_t)(hm + (double)(period - (v & mask)) / model.b);
		const int64_t m1 = kTsfWrapMarginUs, m2 = m1 + 5000, m3 = m1 + 10000;
		const int64_t sched_gap1[4] = { w - m1, w + m1, w + m2, w + m3 };
		const int64_t sched_gap2[4] = { w - m2, w - m1, w + m1, w + m2 };
		const int64_t *sched = gap == 1 ? sched_gap1 : sched_gap2;
		const int first_post = gap == 1 ? 1 : 2;
		int64_t lo_start = 0, lo_end = 0, c_start, c_end, at_ms[4] = { 0 };
		uint32_t c_lo = 0, c_hi = 0;
		bool c_ok = true;
		int k = 0;
		uint64_t fv = 0;

		wrap_host = w;
		if (w - m3 > deadline) {
			printf("\nGATE TSF-WRAP: FAIL - the predicted wrap (%.1f min away) is past the deadline\n",
			       (w - mono_us()) / 60e6);
			return 1;
		}
		sleep_until_us(w - m3);
		c_start = mono_us();
		c_ok = !mt_rr_chk(&dev, MT_TSF_TIMER_DW0, &c_lo);
		c_end = mono_us();

		auto rd = [&](uint32_t addr, uint32_t *val) {
			int64_t start;
			int r;

			if (k < 4) {
				if (k == first_post) {
					/* After the wrap, and before the forced read's own first
					 * post-wrap access, with room for a slow transfer. */
					sleep_until_us(w + m1 - 5000);
					c_ok = c_ok && !mt_rr_chk(&dev, MT_TSF_TIMER_DW1, &c_hi);
				}
				sleep_until_us(sched[k]);
			}
			start = mono_us();
			r = mt_rr_chk(&dev, addr, val);
			if (k < 4)
				at_ms[k] = start;
			if (addr == MT_TSF_TIMER_DW0) {
				lo_start = start;
				lo_end = mono_us();
				/* Which side of the wrap the FIRST low read landed on is what
				 * says which gap the wrap actually fell in - the requested one
				 * is only where it was aimed. */
				if (!have_lo_first && !r) {
					lo_first = *val;
					have_lo_first = true;
				}
			}
			k++;
			return r;
		};
		f_rc = mt7612u::tsf_read(rd, &fv, &f_retried);
		if (!c_ok)
			fails++;
		/* A coherent read carries the instant its (last) low word latched. */
		f_held = tsf_model_holds(&model, (double)fv, (double)lo_start, (double)lo_end, &f_err);
		/* The control's low word latched in its own transfer; a coherent value
		 * would sit there, a torn one 2^32 above. */
		c_held = tsf_model_holds(&model, (double)(((uint64_t)c_hi << 32) | c_lo),
		                         (double)c_start, (double)c_end, &c_err);
		printf("  forced read (gap %d, wrap_bits %d): rc=%d retried=%d err=%+.0f us  accesses at",
		       gap, wrap_bits, f_rc, f_retried ? 1 : 0, f_err);
		for (int i = 0; i < k && i < 4; i++)
			printf(" %+.2f", (at_ms[i] - w) / 1e3);
		printf(" ms\n  control DW0,DW1 across the wrap: err=%+.0f us\n", c_err);
		fflush(stdout);
		have = false;                        /* the sleeps are a hole in part 1 */
		deadline = mono_us() + kTsfWrapWindowUs;
	}

	printf("  reads=%llu fails=%llu backwards=%llu checked=%llu off_model=%llu worst=%.0f us\n",
	       (unsigned long long)reads, (unsigned long long)fails,
	       (unsigned long long)backwards, (unsigned long long)checked,
	       (unsigned long long)off_model, worst);

	if (gone) {
		printf("\nGATE TSF-WRAP: FAIL - %llu reads in a row failed; the adapter is gone\n",
		       (unsigned long long)consec_fails);
		return 1;
	}
	if (fails || backwards || off_model) {
		printf("\nGATE TSF-WRAP: FAIL - %llu failed read(s), %llu backwards step(s), %llu read(s) off the model\n",
		       (unsigned long long)fails, (unsigned long long)backwards,
		       (unsigned long long)off_model);
		return 1;
	}
	/* 0 PASS, 1 FAIL, 2 bad invocation (as everywhere else in this tool),
	 * 3 no verdict: interrupted, or the wrap missed the gap. A wrapper reruns
	 * a 3; a 1 is a defect and a 2 is the operator's. */
	if (g_stop) {
		printf("\nGATE TSF-WRAP: INTERRUPTED - no verdict\n");
		return 3;
	}
	if (!forced) {
		printf("\nGATE TSF-WRAP: FAIL - no wrap reached before the deadline (%.0f min)\n",
		       max_min);
		return 1;
	}
	if (f_rc || !f_held) {
		printf("\nGATE TSF-WRAP: FAIL - the forced read %s\n",
		       f_rc ? "failed" : "missed the model");
		return 1;
	}
	if (wrap_bits < 32) {
		printf("\nGATE TSF-WRAP: SMOKE - schedule and model check out (control err %+.0f us); no wrap verdict below 32 bits\n",
		       c_err);
		return c_held ? 0 : 1;
	}
	/* Defensive: the forced read only runs on a fitted model, and the model
	 * needs reads from inside the same 120 s window, so this cannot be 0 today.
	 * It is the one thing a PASS silently rests on, so it is checked. */
	if (checked == 0) {
		printf("\nGATE TSF-WRAP: FAIL - no continuous read was checked near the wrap\n");
		return 1;
	}
	/* The control reads the low word before the wrap and the high word after,
	 * so a torn value is one whole high-word step ABOVE the truth. The sign
	 * matters: a read that is systematically 2^32 low would also fail on
	 * magnitude alone, and the model would have absorbed it. Checked before the
	 * retry verdict, so a run that misses the gap still reports whether the rig
	 * can see a tear at all. */
	if (c_err < 2147483648.0 || c_err > 6442450944.0) {
		printf("\nGATE TSF-WRAP: FAIL - the control is %+.0f us off, not the +2^32 us a torn read gives; the rig cannot see a tear\n",
		       c_err);
		return 1;
	}
	/* A retry says the wrap fell somewhere inside the read, not that it fell
	 * where it was aimed: a mistimed wrap lands in the other gap and still
	 * retries. The first low word says which - post-wrap it reads small,
	 * pre-wrap it reads just under the mask. Crediting the wrong gap would
	 * report a gap as covered when it never was. */
	if (f_retried && have_lo_first) {
		const int actual = ((uint64_t)lo_first & mask) < period / 2 ? 1 : 2;

		if (actual != gap) {
			printf("\nGATE TSF-WRAP: INCONCLUSIVE - the wrap landed in gap %d, not the requested gap %d (first low word 0x%08x). Re-run.\n",
			       actual, gap, lo_first);
			return 3;
		}
	}
	if (!f_retried) {
		/* The read holds against the model (checked above) and the control did
		 * tear, so the wrap simply did not land in the gap - transfer jitter
		 * can do that. Not a defect: re-run. */
		printf("\nGATE TSF-WRAP: INCONCLUSIVE - the forced read holds and the control tore, but the read took no retry; the wrap missed gap %d. Re-run.\n",
		       gap);
		return 3;
	}
	printf("\nGATE TSF-WRAP: PASS - retried across the wrap in gap %d, %+.0f us off the model; control tore by %+.0f us\n",
	       gap, f_err, c_err);
	return 0;
}

/* ---------------------------------------------------------------------------
 * Station-identity gates: `sta`, `staack`, `staid`, `norsp`, `bssen`.
 * docs/mt7612u-station-identity.md is the record they produced; the harnesses
 * are tests/mt7612u_sta_identity.sh and tests/mt7612u_sta_autoack.sh (the
 * uplink half is gate_txs, driven by tests/mt7612u_sta_uplink.sh).
 */

/* ---------------------------------------------------------------- gate_sta
 *
 * Does programming the joined BSSID anywhere - MT_MAC_BSSID or the
 * MT_MAC_APC_BSSID slot table - change what a MANAGED STATION receives, and
 * is a wrong value silent, harmless, or fatal? Six arms; every write is read
 * back after mt_mac_start(), every other slot is checked empty, and an arm
 * whose state does not read back as written is marked UNVERIFIED and makes
 * the gate INCONCLUSIVE (rc 2) - as does an all-zero to_us column, which
 * would mean the table measured broadcast reception only.
 *
 * The station's slot is sta_station_slot() - slot 0 on a factory address, so
 * arms C and D write the same slot there. Arms B, E and F move MT_MAC_BSSID,
 * which mt76's station configuration never does; they ask whether a wrong
 * base matters, not what mt76 would program.
 *
 * The receive filter is what makes this a question at all. The managed value
 * mt_mac_start() programs, 0x00015f97, has bit 2 (PROMISC) SET - mt76x2
 * sets it whenever the phy is not in monitor mode, and it drops unicast not
 * addressed to MT_MAC_ADDR (mt76x2u_config()). Bit 3 (OTHER_BSS) is clear
 * (regs.h has the full decode). Do not reason about
 * this register from one bit: the gate prints the full value per arm for the
 * reader, and flags an arm whose PROMISC drop bit is clear (the monitor
 * filter).
 *
 * The AP-side finding (docs/mt7612u-ap-mode.md: a wrong APC slot "beacons
 * perfectly, acknowledges nobody") is about acknowledgement, not reception,
 * and does not transfer to a station's receive path.
 *
 * NO ARM TOUCHES MT_MAC_ADDR. The auto-response engine matches address 1
 * against it, and moving it is what breaks a station
 * (docs/mt7612u-station-identity.md).
 *
 * WHAT THIS GATE CANNOT SEE. It counts RX only. Whether the MAC auto-ACKed is
 * a property of what the transmitter observed, and this process cannot ask -
 * tests/mt7612u_sta_autoack.sh asks the transmitter. A healthy RX arm is not
 * evidence about ACKing.
 *
 *   bringup sta <chan> <secs-per-arm> <ap-bssid>
 */
struct sta_rx_count {
	std::atomic<unsigned long> total{0};     /* every frame off the ring */
	std::atomic<unsigned long> from_bss{0};  /* addr2 == the AP          */
	std::atomic<unsigned long> to_us{0};     /* addr1 == our own MAC     */
	std::atomic<unsigned long> to_us_data{0};
	std::atomic<unsigned long> beacons{0};
	uint8_t bssid[6];
	uint8_t own[6];
};

static void sta_rx_cb(void *user, const void *frame, size_t len,
                      const struct mt7612u_rx_info *info)
{
	struct sta_rx_count *c = (struct sta_rx_count *)user;
	const uint8_t *f = (const uint8_t *)frame;

	(void)info;
	c->total.fetch_add(1, std::memory_order_relaxed);
	if (len < 24) return;

	/* addr1 at 4, addr2 at 10, addr3 at 16 - true for every non-4-address
	 * frame, which is all an infrastructure station ever sees. */
	if (memcmp(f + 10, c->bssid, 6) == 0)
		c->from_bss.fetch_add(1, std::memory_order_relaxed);
	if (memcmp(f + 4, c->own, 6) == 0) {
		c->to_us.fetch_add(1, std::memory_order_relaxed);
		if ((f[0] & 0x0c) == 0x08)
			c->to_us_data.fetch_add(1, std::memory_order_relaxed);
	}
	if (f[0] == 0x80 && memcmp(f + 16, c->bssid, 6) == 0)
		c->beacons.fetch_add(1, std::memory_order_relaxed);
}

/*
 * The APC slot a STATION's BSSID lives in, by mt76's rule - which keys the
 * slot on the station's OWN address, not on the BSSID:
 *
 *   mt76x02_add_interface(): idx = 0, or 1 + (((macaddr[0] ^ vif->addr[0])
 *     >> 2) & 7) when vif->addr is locally administered; a STATION then gets
 *     idx += 8 ("bssidx 8-15 for client mode");
 *   mt76x02_bss_info_changed() -> mt76x02_mac_set_bssid(mvif->idx, bssid),
 *     which writes APC slot (idx & 7).
 *
 * `macaddr` there is the MBSS base, which mt76x02_mac_setaddr() keeps equal to
 * the station's own address - mt76's station configuration never moves it.
 * So the slot is computed against the base init leaves (the station's own
 * address), and arms that reprogram MT_MAC_BSSID are, by construction, not an
 * mt76 station configuration. A station on a factory (globally administered)
 * address - this tree's case - is slot 0 whatever the base holds. The AP-side
 * rule in beacon.cpp keys on the AP's own address, which for an AP is the
 * BSSID; applying that rule to a station's BSSID picks the wrong slot.
 */
static int sta_station_slot(const uint8_t *base, const uint8_t *own)
{
	int idx = 0;

	if (own[0] & 0x02)
		idx = 1 + (((base[0] ^ own[0]) >> 2) & 7);
	return (idx + 8) & 7;
}

/* Local copies: beacon.cpp's equivalents are static to that file. */
static int sta_set_bss_base(struct mt7612u_dev *d, const uint8_t *a)
{
	const uint32_t dw0 = (uint32_t)a[0] | ((uint32_t)a[1] << 8) |
	                     ((uint32_t)a[2] << 16) | ((uint32_t)a[3] << 24);
	const uint32_t dw1 = (uint32_t)a[4] | ((uint32_t)a[5] << 8);

	if (mt_wr_chk(d, MT_MAC_BSSID_DW0, dw0))
		return -1;
	return mt_rmw(d, MT_MAC_BSSID_DW1, MT_MAC_BSSID_DW1_ADDR, dw1);
}

/* Read a slot back into `out`. Without the read-back an arm could be writing
 * a slot the hardware never consults, and the table would look identical
 * either way. */
static int sta_read_apc(struct mt7612u_dev *d, int idx, uint8_t *out)
{
	uint32_t lo = 0, hi = 0;

	if (mt_rr_chk(d, MT_MAC_APC_BSSID_L(idx), &lo) ||
	    mt_rr_chk(d, MT_MAC_APC_BSSID_H(idx), &hi))
		return -1;
	out[0] = (uint8_t)(lo & 0xff);
	out[1] = (uint8_t)((lo >> 8) & 0xff);
	out[2] = (uint8_t)((lo >> 16) & 0xff);
	out[3] = (uint8_t)((lo >> 24) & 0xff);
	out[4] = (uint8_t)(hi & 0xff);
	out[5] = (uint8_t)((hi >> 8) & 0xff);
	return 0;
}

/* The slot high register's BIT(16) is MT_MAC_APC_BSSID0_H_EN upstream in mt76
 * (defined, never written there); this tree does not define it and gate_sta
 * does not set it. If a per-slot enable is real on this part, a "slot
 * programmed" arm may have written a slot the engine was not consulting,
 * which would make a null result here much weaker than it appears. This gate
 * reports the bit; gate_bssen sets it and measures. Returns -1 on a failed
 * read. */
static int sta_apc_high_raw(struct mt7612u_dev *d, int idx, uint32_t *hi)
{
	return mt_rr_chk(d, MT_MAC_APC_BSSID_H(idx), hi) ? -1 : 0;
}

static int sta_write_apc(struct mt7612u_dev *d, int idx, const uint8_t *a)
{
	const uint32_t lo = (uint32_t)a[0] | ((uint32_t)a[1] << 8) |
	                    ((uint32_t)a[2] << 16) | ((uint32_t)a[3] << 24);
	const uint32_t hi = (uint32_t)a[4] | ((uint32_t)a[5] << 8);

	if (mt_wr_chk(d, MT_MAC_APC_BSSID_L(idx), lo))
		return -1;
	return mt_rmw(d, MT_MAC_APC_BSSID_H(idx), MT_MAC_APC_BSSID_H_ADDR, hi);
}

/* The MBSS base (MT_MAC_BSSID's address halves) back into `out`. */
static int sta_read_bss_base(struct mt7612u_dev *d, uint8_t *out)
{
	uint32_t dw0 = 0, dw1 = 0;

	if (mt_rr_chk(d, MT_MAC_BSSID_DW0, &dw0) ||
	    mt_rr_chk(d, MT_MAC_BSSID_DW1, &dw1))
		return -1;
	out[0] = (uint8_t)(dw0 & 0xff);
	out[1] = (uint8_t)((dw0 >> 8) & 0xff);
	out[2] = (uint8_t)((dw0 >> 16) & 0xff);
	out[3] = (uint8_t)((dw0 >> 24) & 0xff);
	out[4] = (uint8_t)(dw1 & 0xff);
	out[5] = (uint8_t)((dw1 >> 8) & 0xff);
	return 0;
}

/*
 * Put BOTH register families back to their init state, so an arm cannot
 * inherit anything - from its predecessor, or from an earlier process: the
 * chip keeps register state across bring-up tool runs, and mac_setaddr()
 * rewrites only the address halves of the APC slots, so a BIT(16) that
 * gate_bssen set survives into the next gate_sta unless it is cleared here.
 * The whole high word is written, enable bit included. Returns -1 if any
 * write fails, in which case the arm has not started from a known state.
 */
static int sta_reset_bss(struct mt7612u_dev *d, const uint8_t *own)
{
	int rc = sta_set_bss_base(d, own);

	for (int z = 0; z < 8; z++) {
		if (mt_wr_chk(d, MT_MAC_APC_BSSID_L(z), 0) ||
		    mt_wr_chk(d, MT_MAC_APC_BSSID_H(z), 0))
			rc = -1;
	}
	return rc;
}

/* sta_reset_bss(), then read it all back: the base holds `own` and both words
 * of every slot read zero. The chip keeps these registers across processes, so
 * a reset that did not land is reported, not assumed. 0 when verified. */
static int sta_reset_bss_verified(struct mt7612u_dev *d, const uint8_t *own,
                                  const char *gate)
{
	uint8_t base[6] = { 0 };
	int ok = sta_reset_bss(d, own) == 0 &&
	         sta_read_bss_base(d, base) == 0 && memcmp(base, own, 6) == 0;

	for (int z = 0; ok && z < 8; z++) {
		uint32_t lo = 1, hi = 1;

		if (mt_rr_chk(d, MT_MAC_APC_BSSID_L(z), &lo) ||
		    mt_rr_chk(d, MT_MAC_APC_BSSID_H(z), &hi) || lo || hi)
			ok = 0;
	}
	if (!ok)
		printf("GATE %s: FAIL - MT_MAC_BSSID / the APC slots could not be "
		       "restored to their init state; the next process inherits "
		       "them\n", gate);
	return ok ? 0 : -1;
}

/* The receive filter the gate is about to measure under, read back and printed.
 * 0 when the PROMISC drop bit is set (the managed filter mt_mac_start()
 * programs); -1 on a failed read or a clear bit (the monitor filter), which
 * voids an arm that claims to run managed. */
static int sta_check_managed_filter(const char *gate)
{
	uint32_t filtr = 0;

	if (mt_rr_chk(&dev, MT_RX_FILTR_CFG, &filtr)) {
		printf("GATE %s: FAIL - MT_RX_FILTR_CFG unreadable\n", gate);
		return -1;
	}
	if (!(filtr & MT_RX_FILTR_CFG_PROMISC)) {
		printf("GATE %s: FAIL - filtr=%08x: PROMISC drop bit clear, the "
		       "monitor filter - this arm would not run managed\n",
		       gate, filtr);
		return -1;
	}
	printf("filtr=%08x (managed: PROMISC drop bit set)\n", filtr);
	return 0;
}

/* Set MT_AUTO_RSP_EN again and read it back. 0 once it verifiably holds -
 * the state init leaves and everything else on the part assumes. `gate` names
 * the caller in the failure line. */
static int sta_restore_auto_rsp(const char *gate)
{
	uint32_t v = 0;

	if (mt_rmw(&dev, MT_AUTO_RSP_CFG, MT_AUTO_RSP_EN, MT_AUTO_RSP_EN) ||
	    mt_rr_chk(&dev, MT_AUTO_RSP_CFG, &v) || !(v & MT_AUTO_RSP_EN)) {
		printf("GATE %s: FAIL - MT_AUTO_RSP_EN could not be restored "
		       "(read %08x); the chip is left with auto-response OFF\n",
		       gate, v);
		return -1;
	}
	return 0;
}

/* 0 when MT_MAC_ADDR reads back as this adapter's own address (DW0 and the
 * low half of DW1; the U2ME byte above it is write-only). -1 on a failed read
 * or any other address - the port identity did not come back. */
static int sta_port_is_own(void)
{
	uint32_t dw0 = 0, dw1 = 0;
	const uint8_t *m = dev.macaddr;

	if (mt_rr_chk(&dev, MT_MAC_ADDR_DW0, &dw0) ||
	    mt_rr_chk(&dev, MT_MAC_ADDR_DW1, &dw1))
		return -1;
	if (dw0 != ((uint32_t)m[0] | ((uint32_t)m[1] << 8) |
	            ((uint32_t)m[2] << 16) | ((uint32_t)m[3] << 24)) ||
	    (dw1 & 0xffff) != ((uint32_t)m[4] | ((uint32_t)m[5] << 8)))
		return -1;
	return 0;
}

static int gate_sta(uint8_t chan, int secs, const char *bssid_str)
{
	static const uint8_t wrong[6] = { 0x02, 0x00, 0x00, 0xde, 0xad, 0x01 };
	static const uint8_t zero[6] = { 0 };
	static struct sta_rx_count ctr;
	uint8_t bssid[6];
	unsigned long base_bss = 0, base_bcn = 0, any_to_us = 0;
	int any_beacon = 0, unverified = 0, slot;

	/* mbss: write `want` into MT_MAC_BSSID. apc0: into APC slot 0.
	 * apc_sta: into the station's slot by mt76's rule (sta_station_slot). */
	static const struct {
		char tag; int mbss; int apc_sta; int apc0; int bad;
		const char *what;
	} arms[] = {
		{ 'A', 0, 0, 0, 0, "init only - nothing programmed" },
		{ 'B', 1, 0, 0, 0, "MT_MAC_BSSID = AP" },
		{ 'C', 0, 0, 1, 0, "APC slot 0 = AP" },
		{ 'D', 0, 1, 0, 0, "APC station slot (mt76 rule) = AP" },
		{ 'E', 1, 1, 0, 0, "MT_MAC_BSSID + station slot = AP" },
		{ 'F', 1, 1, 0, 1, "both programmed WRONG (is it silent?)" },
	};

	if (parse_mac6(bssid_str, bssid)) {
		printf("GATE STA: FAIL - need the AP's BSSID, e.g.\n"
		       "  bringup sta 6 20 02:42:75:05:d6:00\n");
		return 2;
	}
	if (bssid[0] & 0x01) {
		printf("GATE STA: FAIL - %02x:%02x:%02x:%02x:%02x:%02x is multicast\n",
		       bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
		return 2;
	}

	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) return 1;
	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) return 1;

	/* Against the base init leaves, which is the station's own address. */
	slot = sta_station_slot(dev.macaddr, dev.macaddr);

	printf("=== GATE STA: what the BSSID registers do for a managed station ===\n");
	printf("chan %u, %d s per arm, AP %02x:%02x:%02x:%02x:%02x:%02x, "
	       "own %02x:%02x:%02x:%02x:%02x:%02x, station APC slot %d\n",
	       chan, secs, bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5],
	       dev.macaddr[0], dev.macaddr[1], dev.macaddr[2],
	       dev.macaddr[3], dev.macaddr[4], dev.macaddr[5], slot);
	printf("RX ONLY - whether the MAC auto-ACKed is not visible from here.\n");
	printf("No arm touches MT_MAC_ADDR.\n");
	/*
	 * The bring-up - its calibrations above all - is done. Say so, flushed,
	 * and give a harness time to start its unicast stimulus before arm A.
	 * The MT7612U's calibration replies come late under a strong nearby
	 * transmitter (mcu.cpp, mcu_wait_resp), and the stimulus here is a
	 * monitor-vif flood from 20 cm; starting it only after this line keeps
	 * the two from overlapping (tests/mt7612u_sta_identity.sh waits for it).
	 */
	printf("GATE STA: bring-up done - start the stimulus\n\n");
	fflush(stdout);
	if (!wait_ms(3000)) return 2;
	printf("  arm  %-38s %5s %8s %8s %7s %8s\n",
	       "configuration", "slot", "rx_total", "from_bss", "beacons", "to_us");

	for (unsigned a = 0; a < sizeof arms / sizeof arms[0]; a++) {
		const uint8_t *want = arms[a].bad ? wrong : bssid;
		const uint8_t *want_base = arms[a].mbss ? want : dev.macaddr;
		uint32_t filtr = 0, apc_hi = 0;
		uint8_t apc_rb[6] = { 0 }, base_rb[6] = { 0 };
		int wrote_slot = -1, ok = 1, base_ok, apc_ok = 1, hi_ok = 0;

		memcpy(ctr.bssid, bssid, 6);
		memcpy(ctr.own, dev.macaddr, 6);
		ctr.total = 0; ctr.from_bss = 0; ctr.to_us = 0;
		ctr.to_us_data = 0; ctr.beacons = 0;

		if (sta_reset_bss(&dev, dev.macaddr)) ok = 0;
		if (arms[a].mbss && sta_set_bss_base(&dev, want)) ok = 0;
		if (arms[a].apc0) wrote_slot = 0;
		if (arms[a].apc_sta) wrote_slot = slot;
		if (wrote_slot >= 0 && sta_write_apc(&dev, wrote_slot, want))
			ok = 0;

		/* Every early exit leaves the registers as init does: the chip
		 * keeps them across runs. */
		if (mt_mac_start(&dev, MT_RX_DRAIN_NONE)) {
			mt_mac_stop(&dev); sta_reset_bss(&dev, dev.macaddr); return 1;
		}
		if (mt_async_start(&dev, sta_rx_cb, &ctr)) {
			mt_mac_stop(&dev); sta_reset_bss(&dev, dev.macaddr); return 1;
		}
		/* The RECEIVER must be on: with ENABLE_RX clear the MAC filters
		 * nothing and the gate could not test its own claim. */
		if (mt_mac_start(&dev, MT_RX_DRAIN_RING)) {
			rx_teardown(); mt_mac_stop(&dev);
			sta_reset_bss(&dev, dev.macaddr); return 1;
		}
		/*
		 * DO NOT call mt7612u_set_monitor_rx() here. It writes
		 * MT_RX_FILTR_CFG = PHY_ERR|CRC_ERR and nothing else - every
		 * address and BSS drop bit OFF - so every arm would run
		 * PROMISCUOUS, the hardware would never consult MT_MAC_BSSID or
		 * the APC table, and six identical arms would be guaranteed
		 * before the dwell began (docs/mt7612u-station-identity.md, the
		 * retraction). The filter under test is what mt_mac_start()
		 * already left: 0x00015f97, mt76's managed-station value.
		 */
		if (mt_rr_chk(&dev, MT_RX_FILTR_CFG, &filtr)) ok = 0;

		/* Read everything back AFTER mt_mac_start(): anything written
		 * before it could have been overwritten since. The base must
		 * hold `want_base`; every slot but the one written must be
		 * empty, and that one must hold `want` with BIT(16) reported. */
		base_ok = sta_read_bss_base(&dev, base_rb) == 0 &&
		          memcmp(base_rb, want_base, 6) == 0;
		if (!base_ok) ok = 0;
		for (int z = 0; z < 8; z++) {
			uint8_t rb[6] = { 0 };

			if (sta_read_apc(&dev, z, rb) ||
			    memcmp(rb, z == wrote_slot ? want : zero, 6) != 0) {
				apc_ok = 0;
				ok = 0;
			}
			if (z == wrote_slot)
				memcpy(apc_rb, rb, 6);
		}
		if (wrote_slot >= 0)
			hi_ok = sta_apc_high_raw(&dev, wrote_slot, &apc_hi) == 0;

		/* The receiving dwell ticks the PHY about once a second, as the
		 * public header requires of every receiving consumer. */
		wait_ticking(secs * 1000.0);

		rx_teardown();
		mt_mac_stop(&dev);

		printf("  %c    %-38s %5d %8lu %8lu %7lu %8lu%s\n",
		       arms[a].tag, arms[a].what, wrote_slot,
		       ctr.total.load(), ctr.from_bss.load(),
		       ctr.beacons.load(), ctr.to_us.load(),
		       ok ? "" : "  UNVERIFIED");
		printf("       base %02x:%02x:%02x:%02x:%02x:%02x%s  filtr=%08x%s  "
		       "to_us_data=%lu\n",
		       base_rb[0], base_rb[1], base_rb[2], base_rb[3], base_rb[4],
		       base_rb[5], base_ok ? " (as written)" : " *** NOT AS WRITTEN ***",
		       filtr,
		       /* The label reads the way the BIT does, not the way the
		        * word sounds: these are DROP bits, so PROMISC SET means
		        * "drop frames not addressed here" - the managed state we
		        * want. Clear means promiscuous: the monitor filter, under
		        * which this gate measures nothing. */
		       (filtr & MT_RX_FILTR_CFG_PROMISC)
		           ? "" : "  *** MONITOR FILTER - THIS ARM IS PROMISCUOUS ***",
		       ctr.to_us_data.load());
		if (!apc_ok)
			printf("       APC table NOT AS WRITTEN - a slot other than %d "
			       "is non-empty, or slot %d reads "
			       "%02x:%02x:%02x:%02x:%02x:%02x\n", wrote_slot,
			       wrote_slot, apc_rb[0], apc_rb[1], apc_rb[2],
			       apc_rb[3], apc_rb[4], apc_rb[5]);
		else if (wrote_slot >= 0)
			printf("       APC slot %d verified, others empty, high reg "
			       "%08x (bit16 %s - mt76's per-slot enable)\n",
			       wrote_slot, apc_hi,
			       !hi_ok ? "UNREAD" :
			       (apc_hi & (1u << 16)) ? "SET" : "clear");
		if (!(filtr & MT_RX_FILTR_CFG_PROMISC)) ok = 0;
		if (!ok) unverified++;

		if (ctr.beacons.load()) any_beacon = 1;
		any_to_us += ctr.to_us.load();
		if (a == 0) { base_bss = ctr.from_bss.load(); base_bcn = ctr.beacons.load(); }
		if (g_stop) break;
	}
	/* Leave the registers as init does, verified, so the next process starts
	 * clean. */
	if (sta_reset_bss_verified(&dev, dev.macaddr, "STA"))
		return 1;
	/* 0 measured, 1 FAIL, 2 inconclusive or bad invocation, 3 no verdict:
	 * interrupted, as gate_txs and gate_tsfwrap. A table cut short mid-arm
	 * is not a measurement, whatever the arms before it saw. */
	if (g_stop) {
		printf("\nGATE STA: INTERRUPTED - no verdict\n");
		return 3;
	}

	printf("\nHow to read this:\n");
	if (!any_beacon) {
		printf("  NO BEACONS IN ANY ARM. The AP was not on channel %u, or its\n"
		       "  BSSID is not the one given. Nothing here is comparable and\n"
		       "  the run says NOTHING about the BSSID registers - fix the rig\n"
		       "  and re-run.\n", chan);
		printf("GATE STA: INCONCLUSIVE\n");
		return 2;
	}
	if (!any_to_us) {
		printf("  NO UNICAST TO US IN ANY ARM. Beacons arrived, but nothing was\n"
		       "  addressed to this station, so the table measures broadcast\n"
		       "  reception only - not the question. Drive unicast at the DUT\n"
		       "  (tests/sta_unicast_inject.py) and re-run.\n");
		printf("GATE STA: INCONCLUSIVE\n");
		return 2;
	}
	if (unverified) {
		printf("  %d arm(s) UNVERIFIED: a write did not read back, or the\n"
		       "  managed filter was not in force. Those rows test nothing.\n",
		       unverified);
		printf("GATE STA: INCONCLUSIVE\n");
		return 2;
	}
	printf("  arm A baseline: from_bss=%lu beacons=%lu\n", base_bss, base_bcn);
	printf("  - if B..E match A, the BSSID registers do not gate a station's\n");
	printf("    RX on this MAC, and the answer for receive is 'nothing'.\n");
	printf("  - if arm F (deliberately WRONG) also matches, a wrong BSSID\n");
	printf("    is HARMLESS for RX here - the opposite of the AP-side finding.\n");
	printf("  - the ACK half is measured from the transmitter\n");
	printf("    (tests/mt7612u_sta_autoack.sh).\n");
	printf("GATE STA: measured (verdict is the operator's - see above)\n");
	return 0;
}

/* ------------------------------------------------------------- gate_staack
 *
 * Probe-response retry counting, from the DUT alone - kept for the register
 * state it prints and for its arm C, NOT for its auto-ACK verdict.
 *
 * Method: a directed probe request from our own address makes hostapd answer
 * with a unicast probe response addressed to us. If we ACK it the AP is done
 * (one copy, FC Retry clear); if not, the AP retransmits and we see the same
 * response again with FC Retry SET.
 *
 * WHY ITS VERDICT IS NOT EVIDENCE. The single-variable control (arm B: clear
 * MT_AUTO_RSP_EN, hold reception constant) does not move against hostapd,
 * because that AP does not retransmit an unacknowledged probe response - so
 * the method cannot fail and therefore cannot measure. The auto-ACK answer
 * comes from the transmitter instead: tests/mt7612u_sta_autoack.sh reads a
 * Realtek peer's per-frame CCX reports. docs/mt7612u-station-identity.md
 * records both failed methods.
 *
 * WHAT ARM C DOES SHOW. Arm C retargets MT_MAC_ADDR with
 * mt7612u_set_ack_responder() - what SetAckResponder(bssid) would do to a
 * station. Under the managed filter the station then receives none of the
 * AP's responses: it goes deaf, not merely silent.
 *
 *   bringup staack <chan> <secs> <ap-bssid>
 */
struct staack_count {
	std::atomic<unsigned long> resp{0};      /* probe responses to us      */
	std::atomic<unsigned long> resp_retry{0};/* ... with FC Retry set      */
	std::atomic<unsigned long> other_to_us{0};
	std::atomic<unsigned long> other_retry{0};
	uint8_t own[6];
	uint8_t bssid[6];
};

static void staack_rx_cb(void *user, const void *frame, size_t len,
                         const struct mt7612u_rx_info *info)
{
	struct staack_count *c = (struct staack_count *)user;
	const uint8_t *f = (const uint8_t *)frame;
	int retry;

	(void)info;
	if (len < 24) return;
	if (memcmp(f + 4, c->own, 6) != 0) return;       /* addr1 must be us */
	if (memcmp(f + 10, c->bssid, 6) != 0) return;    /* from the AP      */

	retry = (f[1] & 0x08) ? 1 : 0;                   /* FC Retry */
	if (f[0] == 0x50) {                              /* probe response */
		c->resp.fetch_add(1, std::memory_order_relaxed);
		if (retry) c->resp_retry.fetch_add(1, std::memory_order_relaxed);
	} else {
		c->other_to_us.fetch_add(1, std::memory_order_relaxed);
		if (retry) c->other_retry.fetch_add(1, std::memory_order_relaxed);
	}
}

static int gate_staack(uint8_t chan, int secs, const char *bssid_str)
{
	static struct staack_count ctr;
	struct mt7612u_tx_rate rate = { };
	uint8_t bssid[6];
	static uint8_t probe[128];
	size_t plen;
	double t0, last_tick, a_frac = -1.0, b_frac = -1.0, c_frac = -1.0;
	unsigned long sent = 0, resp = 0, retried = 0;
	unsigned long a_resp = 0, b_resp = 0, c_resp = 0;

	if (parse_mac6(bssid_str, bssid)) {
		printf("GATE STAACK: FAIL - need the AP's BSSID\n");
		return 2;
	}

	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) return 1;
	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) return 1;

	memcpy(ctr.own, dev.macaddr, 6);
	memcpy(ctr.bssid, bssid, 6);
	ctr.resp = 0; ctr.resp_retry = 0; ctr.other_to_us = 0; ctr.other_retry = 0;

	/* Directed probe request: addr1 = addr3 = the AP, addr2 = US. Addressed
	 * to the AP rather than broadcast so the response comes back unicast to
	 * our address, which is the frame whose acknowledgement we are testing. */
	memset(probe, 0, sizeof probe);
	probe[0] = 0x40;                       /* probe request */
	memcpy(probe + 4,  bssid, 6);
	memcpy(probe + 10, dev.macaddr, 6);
	memcpy(probe + 16, bssid, 6);
	plen = 24;
	probe[plen++] = 0x00;                  /* SSID element, wildcard */
	probe[plen++] = 0x00;
	probe[plen++] = 0x01;                  /* supported rates */
	probe[plen++] = 0x04;
	probe[plen++] = 0x82; probe[plen++] = 0x84;
	probe[plen++] = 0x8b; probe[plen++] = 0x96;

	rate.phy = MT7612U_PHY_OFDM;
	rate.mcs = 0;                          /* 6 Mbit/s - robust */
	rate.nss = 1;
	rate.bw = MT7612U_BW_20;
	rate.no_ack = 0;

	printf("=== GATE STAACK: does this MAC auto-ACK unicast to its own address? ===\n");
	printf("chan %u, %d s per arm, AP %02x:%02x:%02x:%02x:%02x:%02x, own %02x:%02x:%02x:%02x:%02x:%02x\n",
	       chan, secs, bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5],
	       dev.macaddr[0], dev.macaddr[1], dev.macaddr[2],
	       dev.macaddr[3], dev.macaddr[4], dev.macaddr[5]);
	printf("\n");

	/*
	 * THREE ARMS. Arm A is the claim; arm B is the control that lets it
	 * mean anything; arm C is a diagnostic.
	 *
	 * Retargeting MT_MAC_ADDR is not a control: under the MANAGED receive
	 * filter it also makes the filter drop the AP's responses, so it cannot
	 * tell "we did not acknowledge" from "we did not receive".
	 *
	 * The clean control changes ONE thing: clear MT_AUTO_RSP_EN and leave
	 * MT_MAC_ADDR alone. Reception is then identical to arm A - same port
	 * identity, same filter, the AP's responses still addressed to us and
	 * still accepted - and the only difference is that the MAC stops
	 * answering them. Retried copies must rise. If they do not, the
	 * retried-copy signal does not track acknowledgement on this rig and
	 * arm A proves nothing.
	 *
	 * Arm C demonstrates the MT_MAC_ADDR hazard rather than asserting it:
	 * what SetAckResponder(bssid) would do to a station. Under the managed
	 * filter the station goes deaf as well as silent.
	 */
	for (int armi = 0; armi < 3; armi++) {
		static const uint8_t foreign[6] =
			{ 0x02, 0x00, 0x00, 0xac, 0x1d, 0x01 };
		double frac;

		ctr.resp = 0; ctr.resp_retry = 0;
		ctr.other_to_us = 0; ctr.other_retry = 0;
		sent = 0;

		/* mt_mac_start() sets ENABLE_TX before its WPDMA poll, so a failed
		 * start can leave TX on: stop the MAC on that path too. */
		if (mt_mac_start(&dev, MT_RX_DRAIN_NONE)) { mt_mac_stop(&dev); return 1; }
		if (mt_async_start(&dev, staack_rx_cb, &ctr)) { mt_mac_stop(&dev); return 1; }
		if (mt_mac_start(&dev, MT_RX_DRAIN_RING)) {
			rx_teardown(); mt_mac_stop(&dev); return 1;
		}
		/* As in gate_sta: mt7612u_set_monitor_rx() installs the MONITOR
		 * filter, not the managed one. Leave what
		 * mt_mac_start() programmed. The auto-ACK conclusion does not rest
		 * on the filter - a probe response addressed to us is accepted
		 * either way - but the arm should still run in the configuration a
		 * station uses. */

		if (armi == 1) {
			/* The single-variable control: stop answering, keep
			 * receiving. */
			if (mt_rmw(&dev, MT_AUTO_RSP_CFG, MT_AUTO_RSP_EN, 0)) {
				printf("  B  could not clear MT_AUTO_RSP_EN - no control\n");
				rx_teardown(); mt_mac_stop(&dev);
				return 2;
			}
		} else if (armi == 2 && mt7612u_set_ack_responder(&dev, foreign)) {
			printf("  C  could not retarget MT_MAC_ADDR\n");
			rx_teardown(); mt_mac_stop(&dev);
			return 2;
		}

		printf("  %c  %s\n", (char)('A' + armi),
		       armi == 0 ? "nothing armed - MT_MAC_ADDR as init left it" :
		       armi == 1 ? "MT_AUTO_RSP_EN CLEARED (control: same RX, no ACK)"
		                 : "MT_MAC_ADDR RETARGETED away (the station hazard)");

		t0 = now_ms();
		last_tick = t0;
		while (now_ms() - t0 < secs * 1000.0 && !g_stop) {
			/* Receiving: tick the PHY about once a second. */
			txs_tick(1, &last_tick);
			probe[22] = (uint8_t)((sent & 0xf) << 4);
			probe[23] = (uint8_t)(sent >> 4);
			if (mt_tx_raw(&dev, probe, plen, &rate, 0xff, 0) == 0)
				sent++;
			usleep(200000);            /* 5/s - inside any AP's rate */
		}

		resp = ctr.resp.load();
		retried = ctr.resp_retry.load();
		frac = resp ? 100.0 * (double)retried / (double)resp : -1.0;

		/* Put the arm's change back, verified: the next arm (or the next
		 * process - the chip keeps registers) must not start with
		 * auto-response off or the port identity elsewhere. */
		if (armi == 1 && sta_restore_auto_rsp("STAACK")) {
			rx_teardown(); mt_mac_stop(&dev);
			return 1;
		}
		if (armi == 2) {
			mt7612u_clear_ack_responder(&dev);
			if (sta_port_is_own()) {
				printf("GATE STAACK: FAIL - MT_MAC_ADDR did not come back "
				       "to this adapter's own address after arm C\n");
				rx_teardown(); mt_mac_stop(&dev);
				return 1;
			}
		}
		rx_teardown();
		mt_mac_stop(&dev);

		printf("     sent %lu, responses to us %lu, retried %lu",
		       sent, resp, retried);
		if (frac >= 0.0) printf("  -> %.1f%% retried\n", frac);
		else             printf("  -> no responses\n");
		printf("     other unicast to us %lu (retried %lu)\n",
		       ctr.other_to_us.load(), ctr.other_retry.load());

		if (armi == 0)      { a_resp = resp; a_frac = frac; }
		else if (armi == 1) { b_resp = resp; b_frac = frac; }
		else                { c_resp = resp; c_frac = frac; }
		if (g_stop) break;
	}

	/* Interrupted: no verdict (rc 3), as gate_sta. */
	if (g_stop) {
		printf("\nGATE STAACK: INTERRUPTED - no verdict\n");
		return 3;
	}
	printf("\n");
	if (a_resp == 0) {
		printf("Arm A got no probe response at all. Either the AP is not on this\n"
		       "channel/BSSID or our probe requests are not reaching it. This says\n"
		       "NOTHING about acknowledgement - do not read it as a failure to ACK.\n"
		       "GATE STAACK: INCONCLUSIVE\n");
		return 2;
	}
	if (b_resp == 0) {
		printf("Arm B got no probe response, so the control could not run and\n"
		       "arm A's %.1f%% is UNCONTROLLED - do not quote it. Clearing\n"
		       "MT_AUTO_RSP_EN should not have changed what we RECEIVE, so if\n"
		       "this happens the assumption behind the control is wrong too.\n"
		       "GATE STAACK: INCONCLUSIVE\n", a_frac);
		return 2;
	}
	printf("A (nothing armed)       : %5.1f%% retried over %lu responses\n", a_frac, a_resp);
	printf("B (AUTO_RSP_EN cleared) : %5.1f%% retried over %lu responses\n", b_frac, b_resp);
	if (c_resp)
		printf("C (MT_MAC_ADDR moved)   : %5.1f%% retried over %lu responses\n",
		       c_frac, c_resp);
	else
		printf("C (MT_MAC_ADDR moved)   : received NOTHING - under the managed\n"
		       "                          filter the station goes DEAF as well\n"
		       "                          as silent. A larger failure than the\n"
		       "                          one this seam was designed around.\n");

	if (b_frac > a_frac + 10.0) {
		printf("\nB rose with reception held constant, so the retried-copy signal\n"
		       "does track acknowledgement here and A is meaningful: this MAC\n"
		       "DOES auto-ACK unicast addressed to its own address with nothing\n"
		       "armed at all.\n");
		printf("GATE STAACK: PASS\n");
		return 0;
	}
	printf("\nB did NOT rise above A even though only the answering engine was\n"
	       "disabled. Either this MAC acknowledges by some path MT_AUTO_RSP_EN\n"
	       "does not gate, or retried copies do not track acknowledgement on\n"
	       "this rig. Either way the method did not demonstrate it can fail, so\n"
	       "A's number proves nothing.\n");
	printf("GATE STAACK: INCONCLUSIVE\n");
	return 2;
}

/* -------------------------------------------------------------- gate_staid
 *
 * The SetStationIdentity contract, checked against real hardware. No AP and
 * no peer: every property here is about what this MAC holds and what the
 * function refuses, which is the whole of the job on this part.
 *
 * The case that matters is 5. mt7612u_set_ack_responder() retargets
 * MT_MAC_ADDR, which is the register the auto-response engine matches address
 * 1 against - and under the managed receive filter gate_staack's arm C shows
 * reception itself going to zero when it moves. So a station identity armed
 * while an ACK responder holds the port identity would be a station that
 * cannot acknowledge anything, silently. It must be REFUSED, and this checks
 * that it is, on the hardware, rather than trusting the branch to be right.
 *
 *   bringup staid
 */
/*
 * Every identity register a station identity could plausibly write: the port
 * identity (MT_MAC_ADDR), the MBSS base (MT_MAC_BSSID) and both words of all
 * eight APC slots. The seam writes none of them; gate_staid compares a
 * snapshot before and after. Returns -1 on any failed read - an unreadable
 * register cannot be shown unchanged. The one register it DOES write, the
 * receive filter, is checked separately (staid_filtr_is).
 */
struct staid_regs { uint32_t w[20]; };

static int staid_snapshot(struct staid_regs *r)
{
	int n = 0;

	if (mt_rr_chk(&dev, MT_MAC_ADDR_DW0, &r->w[n++]) ||
	    mt_rr_chk(&dev, MT_MAC_ADDR_DW1, &r->w[n++]) ||
	    mt_rr_chk(&dev, MT_MAC_BSSID_DW0, &r->w[n++]) ||
	    mt_rr_chk(&dev, MT_MAC_BSSID_DW1, &r->w[n++]))
		return -1;
	for (int z = 0; z < 8; z++) {
		if (mt_rr_chk(&dev, MT_MAC_APC_BSSID_L(z), &r->w[n++]) ||
		    mt_rr_chk(&dev, MT_MAC_APC_BSSID_H(z), &r->w[n++]))
			return -1;
	}
	return 0;
}

/* 1 when MT_RX_FILTR_CFG reads back as `want`. */
static int staid_filtr_is(uint32_t want)
{
	uint32_t v = 0;

	if (mt_rr_chk(&dev, MT_RX_FILTR_CFG, &v)) return 0;
	if (v != want)
		printf("        (MT_RX_FILTR_CFG = %08x, expected %08x)\n", v, want);
	return v == want;
}

/* 1 when both snapshots were taken and are identical. */
static int staid_unchanged(const struct staid_regs *a, int a_ok,
                           const struct staid_regs *b, int b_ok)
{
	return a_ok && b_ok && memcmp(a->w, b->w, sizeof a->w) == 0;
}

static int gate_staid(void)
{
	static const uint8_t bssid[6]   = { 0x02, 0x42, 0x75, 0x05, 0xd6, 0xaa };
	static const uint8_t foreign[6] = { 0x02, 0x00, 0x00, 0xac, 0x1d, 0x01 };
	static const uint8_t mcast[6]   = { 0x01, 0x00, 0x5e, 0x00, 0x00, 0x01 };
	uint8_t own[6], got[6];
	int pass = 0, fail = 0, snap0_ok, snap1_ok;
	struct staid_regs snap0, snap1;

	/* The monitor filter Mt7612uRadio's RX loop installs - the pre-arm
	 * state every filter check below is measured against. */
	const uint32_t mon = MT_RX_FILTR_CFG_CRC_ERR | MT_RX_FILTR_CFG_PHY_ERR;

	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) return 1;
	mt7612u_set_monitor_rx(&dev, 0);

	memcpy(own, dev.macaddr, 6);
	printf("=== GATE STAID: the SetStationIdentity contract on hardware ===\n");
	printf("own %02x:%02x:%02x:%02x:%02x:%02x   bssid %02x:%02x:%02x:%02x:%02x:%02x\n\n",
	       own[0], own[1], own[2], own[3], own[4], own[5],
	       bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);

#define CHK(cond, what) do {                                            \
		if (cond) { pass++; printf("  ok    %s\n", what); }     \
		else      { fail++; printf("  FAIL  %s\n", what); }     \
	} while (0)

	/* 1. the ordinary case - and the seam's defining property: arming
	 * writes no identity register (MT_MAC_ADDR, MT_MAC_BSSID and all eight
	 * APC slots read the same before and after), and installs the managed
	 * receive filter in place of the monitor one. */
	CHK(staid_filtr_is(mon), "pre-arm: the monitor receive filter");
	snap0_ok = staid_snapshot(&snap0) == 0;
	CHK(mt7612u_set_station_identity(&dev, own, bssid) == 0,
	    "arms with the factory address as own");
	snap1_ok = staid_snapshot(&snap1) == 0;
	CHK(mt7612u_station_bssid(&dev, got) == 0 && memcmp(got, bssid, 6) == 0,
	    "records the BSSID it was given");
	CHK(staid_unchanged(&snap0, snap0_ok, &snap1, snap1_ok),
	    "arming writes no identity register (MT_MAC_ADDR, MT_MAC_BSSID, "
	    "APC slots read back unchanged)");
	CHK(staid_filtr_is(MT_RX_FILTR_CFG_MANAGED),
	    "arming installs the managed receive filter 00015f97");

	/* 1b. a receiver (re)started under the arm: the monitor request is
	 * recorded, not installed. */
	mt7612u_set_monitor_rx(&dev, 0);
	CHK(staid_filtr_is(MT_RX_FILTR_CFG_MANAGED),
	    "a monitor-filter request while armed keeps the managed filter");

	/* 2. an address this MAC is not holding */
	CHK(mt7612u_set_station_identity(&dev, foreign, bssid) != 0,
	    "refuses an `own` that is not the port identity");

	/* 3. malformed arguments */
	/* Not an isolating test: `mcast` is also not the port identity, so the
	 * later branch would refuse it even if the multicast branch were
	 * deleted. Kept because the refusal is still the required behaviour,
	 * and labelled so nobody reads it as coverage of that branch. */
	CHK(mt7612u_set_station_identity(&dev, mcast, bssid) != 0,
	    "refuses a multicast own (not an isolating test - see comment)");
	CHK(mt7612u_set_station_identity(&dev, own, mcast) != 0,
	    "refuses a multicast bssid");
	CHK(mt7612u_set_station_identity(&dev, own, own) != 0,
	    "refuses own == bssid");

	CHK(staid_filtr_is(MT_RX_FILTR_CFG_MANAGED),
	    "the refusals left the (still armed) managed filter alone");

	/* 4. clear - no identity register; the monitor filter back */
	snap0_ok = staid_snapshot(&snap0) == 0;
	CHK(mt7612u_clear_station_identity(&dev) == 0,
	    "clear reports the pre-arm filter restored");
	snap1_ok = staid_snapshot(&snap1) == 0;
	CHK(mt7612u_station_bssid(&dev, got) != 0,
	    "reports no BSSID once cleared");
	CHK(staid_unchanged(&snap0, snap0_ok, &snap1, snap1_ok),
	    "clearing writes no identity register (same registers read back "
	    "unchanged)");
	CHK(staid_filtr_is(mon), "clearing restores the monitor receive filter");

	/* 4b. a refusal with nothing armed writes no filter either. */
	CHK(mt7612u_set_station_identity(&dev, foreign, bssid) != 0 &&
	    staid_filtr_is(mon),
	    "a refused arm leaves the monitor filter untouched");

	/*
	 * 5. THE ONE THAT MATTERS. Arm an ACK responder on a foreign address -
	 * which moves MT_MAC_ADDR - and the station arm must refuse, because a
	 * station whose port identity points elsewhere acknowledges nothing.
	 */
	if (mt7612u_set_ack_responder(&dev, foreign) == 0) {
		CHK(mt7612u_set_station_identity(&dev, own, bssid) != 0,
		    "REFUSES while an ACK responder holds the port identity");
		mt7612u_clear_ack_responder(&dev);
		CHK(mt7612u_set_station_identity(&dev, own, bssid) == 0,
		    "arms again once the responder has given it back");
	} else {
		printf("  SKIP  could not arm an ACK responder - case 5 not run\n");
		fail++;   /* the most important case did not run; do not pass. */
	}
	CHK(mt7612u_clear_station_identity(&dev) == 0 && staid_filtr_is(mon),
	    "clear after case 5 restores the monitor receive filter");

	/*
	 * 6. THE OTHER ORDERING, the one a real caller is likelier to hit. Case
	 * 5 covers "responder first, station second" - refused. This covers
	 * "station first, responder second", where the arm-time check cannot
	 * help: the responder moves MT_MAC_ADDR out from under a live station.
	 *
	 * It is not refused - a station arm does not veto the beacon and
	 * responder paths - but it must not be silent, and the armed state must
	 * not go on claiming a station is configured once its identity has been
	 * taken.
	 */
	if (mt7612u_set_station_identity(&dev, own, bssid) == 0 &&
	    mt7612u_set_ack_responder(&dev, foreign) == 0) {
		CHK(mt7612u_station_bssid(&dev, got) != 0,
		    "drops the armed station when a responder takes the identity");
		CHK(staid_filtr_is(mon),
		    "the drop gives the receiver back the monitor filter");
		mt7612u_clear_ack_responder(&dev);
	} else {
		printf("  SKIP  could not set up case 6\n");
		fail++;
	}
	/* After a drop: the clear re-writes the pre-arm filter and verifies it. */
	CHK(mt7612u_clear_station_identity(&dev) == 0 && staid_filtr_is(mon),
	    "clear after the drop verifies the monitor receive filter");

#undef CHK
	printf("\nGATE STAID: %d passed, %d failed\n", pass, fail);
	return fail ? 1 : 0;
}

/* -------------------------------------------------------------- gate_norsp
 *
 * Receive with MT_AUTO_RSP_EN CLEARED, for the single-variable arm of
 * tests/mt7612u_sta_autoack.sh.
 *
 * That harness asks a peer whether this MAC acknowledges unicast addressed to
 * it. SetStationIdentity refuses to arm when MT_AUTO_RSP_EN is clear; this
 * gate is what tests that the bit matters: same receiver, same port identity,
 * same filter, one bit different. If the peer's ok rate collapses with the
 * bit clear, the refusal is justified; if it does not, the refusal rests on a
 * bit that does not gate acknowledgement here.
 *
 * The third argument selects which side of the comparison this is:
 *   1 (default) - clear MT_AUTO_RSP_EN: the CONTROL
 *   0           - leave it set: the CLAIM
 * Both run the SAME code path with the SAME managed filter, so the two arms
 * differ by exactly one bit. (Pairing this with `bringup arx`, which installs
 * the MONITOR filter at the top of gate_arx, would vary the filter and the
 * init path as well.)
 *
 *   bringup norsp <chan> <secs> [clear_rsp]
 */
static void norsp_rx_cb(void *user, const void *frame, size_t len,
                        const struct mt7612u_rx_info *info)
{
	(void)user; (void)frame; (void)len; (void)info;
}

static int gate_norsp(uint8_t chan, int secs, int clear_rsp)
{
	uint32_t before = 0, after = 0;
	int rc = 0;

	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) return 1;
	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) return 1;
	/* mt_mac_start() sets ENABLE_TX before its WPDMA poll, so a failed start
	 * can leave TX on: stop the MAC on that path too. */
	if (mt_mac_start(&dev, MT_RX_DRAIN_NONE)) { mt_mac_stop(&dev); return 1; }
	/* A NON-NULL callback, because mt_async_start(NULL) starts the TX slots
	 * and NOT the RX ring - and mac_start(MT_RX_DRAIN_RING) then refuses,
	 * correctly, with "no ring draining EP4". */
	if (mt_async_start(&dev, norsp_rx_cb, NULL)) { mt_mac_stop(&dev); return 1; }
	if (mt_mac_start(&dev, MT_RX_DRAIN_RING)) {
		rx_teardown(); mt_mac_stop(&dev); return 1;
	}
	/* Managed filter left exactly as mt_mac_start() programmed it - do NOT
	 * call mt7612u_set_monitor_rx(), which installs the monitor value (see
	 * gate_sta). Read back, not assumed. */
	if (sta_check_managed_filter("NORSP")) {
		rx_teardown(); mt_mac_stop(&dev); return 2;
	}

	if (mt_rr_chk(&dev, MT_AUTO_RSP_CFG, &before)) {
		printf("GATE NORSP: FAIL - cannot read MT_AUTO_RSP_CFG\n");
		rx_teardown(); mt_mac_stop(&dev); return 1;
	}
	if (clear_rsp) {
		if (mt_rmw(&dev, MT_AUTO_RSP_CFG, MT_AUTO_RSP_EN, 0)) {
			printf("GATE NORSP: FAIL - cannot clear MT_AUTO_RSP_EN\n");
			rx_teardown(); mt_mac_stop(&dev); return 1;
		}
		/* Checked: mt_rr_chk() leaves `after` untouched on a failed read,
		 * and 0 would read as "EN cleared". */
		if (mt_rr_chk(&dev, MT_AUTO_RSP_CFG, &after)) {
			printf("GATE NORSP: FAIL - cannot read MT_AUTO_RSP_CFG back "
			       "after clearing EN\n");
			sta_restore_auto_rsp("NORSP");
			rx_teardown(); mt_mac_stop(&dev); return 1;
		}
		if (after & MT_AUTO_RSP_EN) {
			printf("GATE NORSP: FAIL - MT_AUTO_RSP_EN did not stay clear "
			       "(%08x -> %08x); the arm would measure nothing\n",
			       before, after);
			sta_restore_auto_rsp("NORSP");
			rx_teardown(); mt_mac_stop(&dev); return 2;
		}
	} else {
		after = before;
		if (!(after & MT_AUTO_RSP_EN)) {
			printf("GATE NORSP: FAIL - asked to LEAVE MT_AUTO_RSP_EN set but "
			       "it is already clear (%08x); this arm would be the control, "
			       "not the claim\n", after);
			rx_teardown(); mt_mac_stop(&dev); return 2;
		}
	}

	printf("MT_AUTO_RSP_CFG %08x -> %08x (EN %s), managed filter, "
	       "receiving %d s on ch%u\n", before, after,
	       clear_rsp ? "CLEARED" : "left SET", secs, chan);

	/* The receiving dwell ticks the PHY about once a second, as the public
	 * header requires of every receiving consumer. 0 means interrupted. */
	const int completed = wait_ticking(secs * 1000.0);

	/* Put it back, verified - interrupted or not. */
	if (clear_rsp && sta_restore_auto_rsp("NORSP"))
		rc = 1;
	rx_teardown();
	mt_mac_stop(&dev);
	if (rc)
		return rc;
	if (!completed) {
		printf("GATE NORSP: INTERRUPTED - no verdict (restored)\n");
		return 3;
	}
	printf("GATE NORSP: done (restored)\n");
	return 0;
}

/* -------------------------------------------------------------- gate_bssen
 *
 * A WRONG BSSID in the APC slot a station's BSSID lives in, with that slot's
 * BIT(16) SET - the DUT arm for tests/mt7612u_sta_autoack.sh arm E.
 *
 * gate_sta leaves BIT(16) clear - upstream mt76 calls it
 * MT_MAC_APC_BSSID0_H_EN and never writes it; this tree does not define it.
 * So a "slot programmed" arm there may write a slot the engine is not
 * consulting, and "a wrong BSSID changes nothing" would be uninteresting if
 * nothing was reading the BSSID.
 *
 * Which slot: the STATION slot by mt76's rule (sta_station_slot), keyed on
 * the station's own address against the MBSS base - slot 0 for a factory
 * address. MT_MAC_BSSID is left at the base init programs (the station's own
 * address), exactly as mt76's station configuration leaves it, so the slot
 * the hardware derives is the slot written. Every other slot is emptied,
 * enable bit included. Base, slot, bit and emptiness are all read back; if
 * any of them does not hold, the arm refuses: an unsettable or misplaced
 * write is not evidence about anything.
 *
 * The peer (tests/mt7612u_sta_autoack.sh) transmits unicast at this station's
 * own address throughout. If acknowledgement and reception survive, the BSSID
 * plane does not gate a station on this part even when its enable is set.
 *
 *   bringup bssen <chan> <secs>
 */
static int gate_bssen(uint8_t chan, int secs)
{
	static const uint8_t wrong[6] = { 0x02, 0x00, 0x00, 0xde, 0xad, 0x02 };
	static const uint8_t zero[6] = { 0 };
	uint8_t rb[6] = { 0 }, base_rb[6] = { 0 };
	uint32_t hi = 0;
	/* bad: 1 a register write / read-back failed (a defect, rc 1);
	 *      2 BIT(16) would not stay set (inconclusive, rc 2). */
	int idx, bad = 0;

	if (mt_eeprom_init(&dev)) return 1;
	if (mt_init_hardware(&dev, NULL)) return 1;
	if (mt_set_channel(&dev, chan, MT7612U_BW_20)) return 1;
	/* As in gate_norsp: a failed start can leave TX on. */
	if (mt_mac_start(&dev, MT_RX_DRAIN_NONE)) { mt_mac_stop(&dev); return 1; }
	if (mt_async_start(&dev, norsp_rx_cb, NULL)) { mt_mac_stop(&dev); return 1; }
	if (mt_mac_start(&dev, MT_RX_DRAIN_RING)) {
		rx_teardown(); mt_mac_stop(&dev); return 1;
	}
	/* Managed filter as mt_mac_start() left it - read back, not assumed. */
	if (sta_check_managed_filter("BSSEN")) {
		rx_teardown(); mt_mac_stop(&dev); return 2;
	}

	idx = sta_station_slot(dev.macaddr, dev.macaddr);
	if (sta_reset_bss(&dev, dev.macaddr) ||
	    sta_write_apc(&dev, idx, wrong) ||
	    mt_rmw(&dev, MT_MAC_APC_BSSID_H(idx), 1u << 16, 1u << 16)) {
		printf("GATE BSSEN: FAIL - a register write failed\n");
		bad = 1;
	}
	if (!bad && (sta_read_bss_base(&dev, base_rb) ||
	             memcmp(base_rb, dev.macaddr, 6) != 0)) {
		printf("GATE BSSEN: FAIL - MT_MAC_BSSID does not hold the station's "
		       "own address, so the derived slot is not certain\n");
		bad = 1;
	}
	for (int z = 0; !bad && z < 8; z++) {
		if (sta_read_apc(&dev, z, rb) ||
		    memcmp(rb, z == idx ? wrong : zero, 6) != 0) {
			printf("GATE BSSEN: FAIL - APC slot %d did not read back as "
			       "written\n", z);
			bad = 1;
		}
	}
	if (!bad && sta_apc_high_raw(&dev, idx, &hi)) {
		printf("GATE BSSEN: FAIL - APC slot %d high register unreadable\n", idx);
		bad = 1;
	}
	if (!bad && !(hi & (1u << 16))) {
		printf("GATE BSSEN: INCONCLUSIVE - BIT(16) of the APC high register "
		       "would not stay set (%08x). Either it is not a per-slot "
		       "enable on this part, or it is not writable here; either way "
		       "this arm proves nothing about an enabled slot.\n", hi);
		bad = 2;
	}
	if (bad) {
		const int restore_failed =
			sta_reset_bss_verified(&dev, dev.macaddr, "BSSEN");

		rx_teardown(); mt_mac_stop(&dev);
		/* A failed restore is a defect whatever the arm's own verdict. */
		return restore_failed ? 1 : bad;
	}

	printf("WRONG BSSID %02x:%02x:%02x:%02x:%02x:%02x in station APC slot %d, "
	       "BIT(16) SET (high reg %08x), other slots empty, MT_MAC_BSSID = own "
	       "address (verified)\n",
	       wrong[0], wrong[1], wrong[2], wrong[3], wrong[4], wrong[5], idx, hi);
	printf("receiving %d s on ch%u as %02x:%02x:%02x:%02x:%02x:%02x\n",
	       secs, chan, dev.macaddr[0], dev.macaddr[1], dev.macaddr[2],
	       dev.macaddr[3], dev.macaddr[4], dev.macaddr[5]);

	/* The receiving dwell ticks the PHY about once a second, as the public
	 * header requires of every receiving consumer. 0 means interrupted. */
	const int completed = wait_ticking(secs * 1000.0);

	/* Leave the registers as init does, verified - interrupted or not: the
	 * chip keeps them across runs. */
	{
		const int restore_failed =
			sta_reset_bss_verified(&dev, dev.macaddr, "BSSEN");

		rx_teardown();
		mt_mac_stop(&dev);
		if (restore_failed)
			return 1;
	}
	if (!completed) {
		printf("GATE BSSEN: INTERRUPTED - no verdict (restored)\n");
		return 3;
	}
	printf("GATE BSSEN: done\n");
	return 0;
}

/*
 * The station gates' numeric arguments, parsed before any device I/O with
 * txs_parse_long()'s strictness: [chan] 1..255, [secs] 1..3600, and norsp's
 * [clear_rsp] 0 or 1. A malformed or out-of-range value is a usage error -
 * atoi() would turn it into 0, and a zero dwell skips the measurement while
 * the gate still reports "done". Absent arguments keep the defaults passed
 * in. Returns 0, or 2 after printing the usage line.
 */
static int sta_parse_args(int argc, char **argv, const char *usage,
                          long *chan, long *secs, long *flag)
{
	if ((argc > 2 && (txs_parse_long(argv[2], chan) ||
	                  *chan < 1 || *chan > 255))) {
		fprintf(stderr, "bad channel '%s': a number 1..255\n", argv[2]);
		fprintf(stderr, "usage: %s\n", usage);
		return 2;
	}
	if (argc > 3 && (txs_parse_long(argv[3], secs) ||
	                 *secs < 1 || *secs > 3600)) {
		fprintf(stderr, "bad duration '%s': seconds, 1..3600\n", argv[3]);
		fprintf(stderr, "usage: %s\n", usage);
		return 2;
	}
	if (flag && argc > 4 && (txs_parse_long(argv[4], flag) ||
	                         (*flag != 0 && *flag != 1))) {
		fprintf(stderr, "bad clear_rsp '%s': 0 or 1\n", argv[4]);
		fprintf(stderr, "usage: %s\n", usage);
		return 2;
	}
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

	/* txs: every argument refused BEFORE mt_open(), which already resets the
	 * device - an atoi'd "abc" or a (uint8_t) 256 would otherwise become
	 * channel 0 and reach hardware setup. Channel 1..255 (at 20 MHz,
	 * mt_chan_group() does not validate the control channel, so there is no
	 * finer "does this chip tune it" check to reuse), frames a positive int,
	 * peer a whole MAC. */
	long txs_chan = 149, txs_frames = 40;
	/* The station gates' arguments (sta_parse_args): defaults per gate. */
	long sta_chan = 6, sta_secs = 15, sta_flag = 1;
	if (!strcmp(cmd, "txs")) {
		uint8_t mac[6];

		if (argc > 5) {
			fprintf(stderr, "too many arguments for txs\n");
			fprintf(stderr, "usage: bringup txs [chan] [frames] [peer MAC]\n");
			return 2;
		}
		if (argc > 2 && (txs_parse_long(argv[2], &txs_chan) ||
		                 txs_chan < 1 || txs_chan > 255)) {
			fprintf(stderr, "bad channel '%s': a number 1..255\n", argv[2]);
			fprintf(stderr, "usage: bringup txs [chan] [frames] [peer MAC]\n");
			return 2;
		}
		if (argc > 3 && (txs_parse_long(argv[3], &txs_frames) ||
		                 txs_frames < 1 || txs_frames > INT_MAX)) {
			fprintf(stderr, "bad frame count '%s': a positive number\n",
			        argv[3]);
			fprintf(stderr, "usage: bringup txs [chan] [frames] [peer MAC]\n");
			return 2;
		}
		if (argc > 4 && parse_mac6(argv[4], mac)) {
			fprintf(stderr, "bad peer MAC '%s'\n", argv[4]);
			fprintf(stderr, "usage: bringup txs [chan] [frames] [peer MAC]\n");
			return 2;
		}
	}

	if (!strcmp(cmd, "sta") || !strcmp(cmd, "staack") ||
	    !strcmp(cmd, "norsp") || !strcmp(cmd, "bssen")) {
		const int norsp = !strcmp(cmd, "norsp");

		sta_secs = !strcmp(cmd, "sta") ? 15 :
		           !strcmp(cmd, "staack") ? 20 : 25;
		if (sta_parse_args(argc, argv,
		        norsp ? "bringup norsp [chan] [secs] [clear 1|0]" :
		        !strcmp(cmd, "bssen") ? "bringup bssen [chan] [secs]" :
		        !strcmp(cmd, "sta") ? "bringup sta [chan] [secs] <ap-bssid>" :
		                              "bringup staack [chan] [secs] <ap-bssid>",
		        &sta_chan, &sta_secs, norsp ? &sta_flag : NULL))
			return 2;
	}

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	/* The knob lives here, not in the library: mt_recover_usb() reads the
	 * field, and setting it before mt_open() is what makes the wedge
	 * experiments observe-only.  Same spelling as before. */
	if (getenv("MT7612U_NO_AUTORECOVER"))
		dev.no_autorecover = 1;
	/* Same shape, same reason: the library takes a selector, this tool is what
	 * reads the environment for it. Operator-facing spelling is unchanged. */
	dev.dev_selector = getenv("MT7612U_DEV");

	/* Runs before the global mt_open() below, because it IS an open - of the
	 * other public entry point. */
	if (!strcmp(cmd, "adopt"))
		return gate_adopt(getenv("MT7612U_DEV"));
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
	} else if (!strcmp(cmd, "tsfwrite")) {
		rc = gate_tsfwrite(argc > 2 ? (uint8_t)atoi(argv[2]) : 149);
	} else if (!strcmp(cmd, "tsfwrap")) {
		rc = gate_tsfwrap(argc > 2 ? atoi(argv[2]) : 1,
		                  argc > 3 ? atoi(argv[3]) : 32,
		                  argc > 4 ? atof(argv[4]) : 80.0);
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
	} else if (!strcmp(cmd, "txs")) {
		/* Validated before mt_open() above. */
		rc = gate_txs((uint8_t)txs_chan, (int)txs_frames,
		              argc > 4 ? argv[4] : NULL);
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
		              argc > 3 ? atoi(argv[3]) : 5,
		              argc > 4 ? atoi(argv[4]) : 0);
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
	} else if (!strcmp(cmd, "beacon")) {
		rc = gate_beacon(argc > 2 ? (uint8_t)atoi(argv[2]) : 149,
		                 argc > 3 ? atoi(argv[3]) : 10);
	} else if (!strcmp(cmd, "ap")) {
		rc = gate_ap(argc > 2 ? (uint8_t)atoi(argv[2]) : 149,
		             argc > 3 ? atoi(argv[3]) : 30);
	} else if (!strcmp(cmd, "sta")) {
		rc = gate_sta((uint8_t)sta_chan, (int)sta_secs,
		              argc > 4 ? argv[4] : NULL);
	} else if (!strcmp(cmd, "staack")) {
		rc = gate_staack((uint8_t)sta_chan, (int)sta_secs,
		                 argc > 4 ? argv[4] : NULL);
	} else if (!strcmp(cmd, "staid")) {
		rc = gate_staid();
	} else if (!strcmp(cmd, "norsp")) {
		rc = gate_norsp((uint8_t)sta_chan, (int)sta_secs, (int)sta_flag);
	} else if (!strcmp(cmd, "bssen")) {
		rc = gate_bssen((uint8_t)sta_chan, (int)sta_secs);
	} else if (!strcmp(cmd, "chan")) {
		rc = gate_chan(argc > 2 ? (uint8_t)atoi(argv[2]) : 149,
		               argc > 3 ? argv[3] : NULL);
	} else if (!strcmp(cmd, "init")) {
		rc = gate_init(argc > 2 ? argv[2] : NULL);
	} else if (!strcmp(cmd, "swreset")) {
		rc = gate_swreset(argc > 2 ? atoi(argv[2]) : 1);
	} else if (!strcmp(cmd, "fw")) {
		rc = gate_fw(argc > 2 ? argv[2] : NULL);
	} else {
		fprintf(stderr, "unknown subcommand '%s'\n", cmd);
		fprintf(stderr, "usage: bringup [regs|fw|init|chan|tx|rx|hop|gateg|tsfwrite|tsfwrap] [chan] [count] [phy 0=CCK 1=OFDM 2=HT 4=VHT] [mcs]\n");
		fprintf(stderr, "       bringup adopt                  (the mt_adopt path a libusb-owning consumer uses)\n");
		fprintf(stderr, "       bringup tsfwrite [chan]        (confirm this part has no TSF load path)\n");
		fprintf(stderr, "       bringup tsfwrap [gap 1|2] [wrap_bits] [max_min]  (TSF read across the low-word wrap, ~72 min; rc 3 = no verdict, re-run)\n");
		fprintf(stderr, "       bringup beacon [chan] [secs]   (Stage A: static AP beacon on air)\n");
		fprintf(stderr, "       bringup ap     [chan] [secs]   (Stage B: beacon + RX, probe/auth/assoc)\n");
		fprintf(stderr, "       bringup txs    [chan] [frames] [peer MAC]  (per-frame retry count off MT_TX_STAT_FIFO; honours DEVOURER_TX_RETRY_LIMIT)\n");
		fprintf(stderr, "       bringup staid                  (the SetStationIdentity contract on hardware, no AP)\n");
		fprintf(stderr, "       bringup sta    [chan] [secs] <ap-bssid>  (what the BSSID registers do for a managed station)\n");
		fprintf(stderr, "       bringup staack [chan] [secs] <ap-bssid>  (probe-response retry count; register state, not a verdict)\n");
		fprintf(stderr, "       bringup norsp  [chan] [secs] [clear 1|0] (receive with MT_AUTO_RSP_EN cleared or left set)\n");
		fprintf(stderr, "       bringup bssen  [chan] [secs]   (receive with a WRONG BSSID in an ENABLED APC slot)\n");
		fprintf(stderr, "       bringup [sweep|coding|vht] [chan] [count] [bw 0=20 1=40 2=80]\n");
		fprintf(stderr, "       the witness must listen at the same width (DEVOURER_BW=40|80)\n");
		rc = 2;
	}

	mt_close(&dev);
	return rc;
}
