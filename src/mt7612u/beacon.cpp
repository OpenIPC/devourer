/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * Static reserved-page beacon.
 *
 * The MT76 MAC auto-transmits a beacon written to MT_BEACON_BASE at every
 * TBTT, filling the TSF timestamp and the 802.11 sequence number in hardware
 * (MT_TXWI_FLAGS_TS + MT_TXWI_ACK_CTL_NSEQ, requested via MT_TXOPT_BEACON).
 * So an always-on AP needs no host involvement per beacon: no pre-TBTT timer,
 * no worker thread. That is the one USB-userspace limitation the AP design
 * engineers out - see docs/mt7612u-ap-mode.md.
 *
 * Dynamic beacon content (a live TIM bitmap for power-saving clients) is NOT
 * covered here; it would need the pre-TBTT machinery mt76 runs on PCIe/USB.
 *
 * Ported from mt76/mt76x02_beacon.c and mt76x02_usb_core.c @ be5ce79.
 * Copyright (C) 2016 Felix Fietkau, (C) 2018 Lorenzo Bianconi / Stanislaw Gruszka.
 */
#include <cstring>
#include "internal.h"

/* mt76x02u: 5 USB beacon slots, each (8192 / 5) & ~63 = 1600 bytes. The 8 kB
 * reserved page is shared with PS-buffered frames upstream; we use slot 0. */
#define MT_BCN_NSLOTS    5
#define MT_BCN_SLOT_SIZE ((8192 / MT_BCN_NSLOTS) & ~63)

/* mt76x02_set_beacon_offsets(): each slot's (byte offset / 64) is packed into
 * MT_BCN_OFFSET, four slots to a 32-bit register. */
static void mt_beacon_set_offsets(struct mt7612u_dev *d)
{
	uint32_t regs[4] = { 0 };
	int i;

	for (i = 0; i < MT_BCN_NSLOTS; i++) {
		uint32_t val = (uint32_t)i * MT_BCN_SLOT_SIZE;

		regs[i / 4] |= (val / 64) << (8 * (i % 4));
	}
	for (i = 0; i < 4; i++)
		mt_wr(d, MT_BCN_OFFSET(i), regs[i]);
}

/*
 * mt76x02_init_beacon_config(): quiet the beacon engine, select sync mode,
 * suppress every beacon slot while the page is being set up, and lay out the
 * slot offsets. Run once before the first beacon is written. The address
 * programming (BSSID, MBSS mode, per-slot beacon count) is done by
 * mac_setaddr() at init. BCN_BYPASS_MASK=0xffff suppresses all slots here;
 * mt_beacon_write() clears the bit for the slot it loads so that one airs.
 */
void mt_beacon_init(struct mt7612u_dev *d)
{
	mt_clear(d, MT_BEACON_TIME_CFG,
	         MT_BEACON_TIME_CFG_TIMER_EN | MT_BEACON_TIME_CFG_TBTT_EN |
	         MT_BEACON_TIME_CFG_BEACON_TX);
	mt_set(d, MT_BEACON_TIME_CFG, MT_BEACON_TIME_CFG_SYNC_MODE);
	mt_wr(d, MT_BCN_BYPASS_MASK, 0xffff);   /* suppress all while we set up */
	mt_beacon_set_offsets(d);
}

/* The slot has to hold the body, its TXWI and the DMA header. Factored out so
 * mt7612u_beacon_update() can apply it BEFORE it suppresses the slot - a
 * refusal after the guard is up leaves the AP off the air. Returns non-zero
 * when the frame does not fit, and says so. */
static int len_fits_slot(size_t len)
{
	if (len + MT_TXWI_LEN + MT_DMA_HDR_LEN > MT_BCN_SLOT_SIZE) {
		ERR("beacon %zu B + TXWI exceeds the %d B slot", len,
		    (int)MT_BCN_SLOT_SIZE);
		return -1;
	}
	return 0;
}

/*
 * mt76x02_mac_set_beacon(): write [TXWI][beacon MPDU] into slot 0.
 *
 * mt_tx_build() emits [TXINFO 4][TXWI 20][802.11][pad], which is the shape the
 * TX queue wants; the reserved page wants no TXINFO and no trailing zero word,
 * so skip the first 4 bytes and copy from the TXWI onward. A beacon header is
 * 24 bytes (4-aligned), so mt_tx_build() inserts no interior L2 pad and the
 * copied region is exactly [TXWI][MPDU] rounded up to a word.
 */
int mt_beacon_write(struct mt7612u_dev *d, const void *frame, size_t len,
                    const struct mt7612u_tx_rate *rate)
{
	uint8_t buf[MT_BCN_SLOT_SIZE];
	int total;

	/* Same rule mt7612u_beacon_update() applies before it suppresses the slot;
	 * kept here too because beacon_start reaches this directly. */
	if (len_fits_slot(len))
		return -1;
	total = mt_tx_build(d, buf, sizeof buf, frame, len, rate, 0xff,
	                    MT_TXOPT_BEACON, 0, 0);
	if (total < 0)
		return -1;

	/* Checked by the io_err delta, because mt_wr_copy() returns void and
	 * gives up mid-loop on the first failed vendor request - leaving a HALF
	 * WRITTEN beacon in the page, which then airs. That is worse than no
	 * beacon, and the paragraph below already argues why USB writes here get
	 * checked when mt76's MMIO ones do not. */
	{
		const unsigned before = mt_io_errors(d);
		mt_wr_copy(d, MT_BEACON_BASE, buf + MT_DMA_HDR_LEN,
		           total - MT_DMA_HDR_LEN);
		if (mt_io_errors(d) != before) {
			ERR("beacon: the reserved-page copy failed part way");
			return -1;
		}
	}

	/*
	 * Unsuppress the slot just written. BCN_BYPASS_MASK is inverted: a set
	 * bit suppresses that slot (mt_beacon_init() set all 16). mt76 clears the
	 * low N bits down from bit 7 for N written beacons - 0xff00 | ~(0xff00 >>
	 * beacon_data_count) in mt76x02u_pre_tbtt_work() - and the static path
	 * writes exactly one (slot 0), so N = 1. Without this the beacon never
	 * airs even though the TSF and beacon timer run.
	 *
	 * Checked, unlike mt_wr(): this single write decides whether the beacon
	 * airs at all, and mt76 can leave it unchecked because MMIO cannot fail
	 * while USB can - the same argument mt_ap_set_bssid() makes below. A
	 * silent failure here is an AP that beacons nothing while every other
	 * step reports success.
	 */
	return mt_wr_chk(d, MT_BCN_BYPASS_MASK, 0xff00u | ~(0xff00u >> 1));
}

/*
 * mt76x02_mac_set_bssid(): the per-BSS address the MAC matches receptions
 * against. mac_setaddr() zeroes all eight APC slots at init, which is right for
 * an injector; an AP must publish its own BSSID in the slot its MBSS index
 * selects (0 for a single BSS) or the MAC matches nothing for the BSS - a
 * station's auth is then neither accepted nor auto-ACKed, and it retries
 * forever. Called by the AP path; the injector never needs it.
 */
int mt_ap_set_bssid(struct mt7612u_dev *d, uint8_t idx, const uint8_t *addr)
{
	uint32_t lo = (uint32_t)addr[0] | ((uint32_t)addr[1] << 8) |
	              ((uint32_t)addr[2] << 16) | ((uint32_t)addr[3] << 24);
	uint32_t hi = (uint32_t)addr[4] | ((uint32_t)addr[5] << 8);

	idx &= 7;
	if (mt_wr_chk(d, MT_MAC_APC_BSSID_L(idx), lo))
		return -1;
	/* mt_rmw() skips the write entirely when its read half fails, so an
	 * unchecked call can leave bytes 4-5 zero - a half-programmed BSSID that
	 * matches nothing while the L half still reads back correct. mt76 returns
	 * void here because MMIO cannot fail; USB can. */
	return mt_rmw(d, MT_MAC_APC_BSSID_H(idx), MT_MAC_APC_BSSID_H_ADDR, hi);
}

/*
 * mt76x02_mac_set_beacon_enable(), static path. No pre-TBTT timer: the MAC
 * transmits the reserved-page beacon on its own once BEACON_TX|TBTT_EN|TIMER_EN
 * are set. interval_tu is the beacon interval in TU (1024 us); the register
 * counts in 1/16 TU, so it is shifted left by 4 (mt76x02_bss_info_changed).
 */
int mt_beacon_set_enable(struct mt7612u_dev *d, int on, unsigned interval_tu)
{
	const uint32_t bits = MT_BEACON_TIME_CFG_BEACON_TX |
	                      MT_BEACON_TIME_CFG_TBTT_EN |
	                      MT_BEACON_TIME_CFG_TIMER_EN;

	if (on) {
		/* INTVAL is 16 bits of 1/16-TU, so the interval caps at 4095 TU. */
		if (interval_tu == 0 || interval_tu > 0xffffu / 16) {
			ERR("beacon interval %u TU out of range (1..4095)", interval_tu);
			return -1;
		}
		mt_rmw(d, MT_BEACON_TIME_CFG, MT_BEACON_TIME_CFG_INTVAL,
		       FIELD_PREP(MT_BEACON_TIME_CFG_INTVAL, interval_tu << 4));
		mt_set(d, MT_BEACON_TIME_CFG, bits);
	} else {
		mt_clear(d, MT_BEACON_TIME_CFG, bits);
	}
	return 0;
}

/* --- public ABI ---------------------------------------------------------
 *
 * The three calls devourer's IRadio beacon surface maps onto. Everything they
 * do is the sequence bringup's Stage A and Stage B gates run and that was
 * device-verified on 2026-09-08 (docs/mt7612u-ap-mode.md); this is that
 * sequence behind the public header, so a consumer does not have to reach into
 * internal.h to be an AP.
 */

/* Split a radiotap-framed buffer into rate + MPDU, exactly as
 * mt7612u_send_packet() does. A bare MPDU (no radiotap) is not an error here -
 * IRadio's contract strips the header "if present" - and takes the rate a
 * beacon wants: OFDM 6 Mbps, the basic rate every station must decode. */
static int beacon_split(const void *buf, size_t len, const uint8_t **mpdu,
                        size_t *mpdu_len, struct mt7612u_tx_rate *r)
{
	const uint8_t *p = (const uint8_t *)buf;
	int rlen;

	if (!p || len == 0) return -1;

	/* ZEROED FIRST. The bare-MPDU branch below sets five of this struct's
	 * nine fields, and sgi/ldpc/stbc go straight into the 16-bit rate word
	 * the MAC transmits verbatim, while power_adj short-circuits the derived
	 * per-rate TX power in mt_tx_build(). Left indeterminate, a bare-MPDU
	 * beacon - which is what tests/ap_responder.cpp hands us - airs with
	 * whatever was on the stack. The radiotap branch only escaped this
	 * because mt_radiotap_parse() memsets its output. */
	*r = mt7612u_tx_rate{};

	/*
	 * Which shape is this? An 802.11 beacon's first byte is its frame control,
	 * 0x80 - never 0. A radiotap header's first byte is its version, which must
	 * be 0. So byte 0 decides, and each shape is then held to its own rules
	 * rather than being allowed to fall back to the other:
	 *
	 * mt_radiotap_parse() returns 0 for BOTH "not a radiotap header" and "is
	 * one and it is malformed" (bad version, bad length, truncated present
	 * map), and negative for "declared a field past its own length". Treating
	 * any of those as a bare MPDU parses radiotap bytes as an 802.11 header and
	 * loads them into the beacon page - which then airs.
	 */
	if (p[0] == 0) {
		rlen = mt_radiotap_parse(p, len, r);
		if (rlen <= 0) {
			ERR("beacon: radiotap header is malformed");
			return -1;
		}
		if ((size_t)rlen >= len) {
			ERR("beacon: %d B of radiotap and no frame after it", rlen);
			return -1;
		}
		*mpdu = p + rlen;
		*mpdu_len = len - (size_t)rlen;
	} else {
		/* A bare MPDU. OFDM 6 Mbps is the basic rate every station must
		 * decode, which is what a beacon wants. */
		r->phy = MT7612U_PHY_OFDM;
		r->mcs = 0;
		r->nss = 1;
		r->bw = MT7612U_BW_20;
		*mpdu = p;
		*mpdu_len = len;
	}

	/* Unconditionally, whatever the caller's radiotap said: a beacon is
	 * broadcast, and mt_tx_build() turns a cleared no_ack into
	 * MT_TXWI_ACK_CTL_REQ - an ACK request on a frame no one may ACK. Both
	 * bring-up gates hardcode this; the ABI must not be weaker. */
	r->no_ack = 1;

	/* addr3 lives at offset 16, so anything shorter has no BSSID to publish. */
	if (*mpdu_len < 24) {
		ERR("beacon: %zu B is too short for an 802.11 header", *mpdu_len);
		return -1;
	}
	/* mt_beacon_write() documents that it relies on a 24-byte, 4-aligned
	 * header so mt_tx_build() inserts no interior L2 pad. A QoS-data or
	 * 4-address frame (26 or 30) passes every check above and would land in
	 * the reserved page as [TXWI][hdr][2 pad][body] - a layout that function
	 * is written not to expect. Enforce what it assumes. */
	if (mt_hdrlen_from_fc(*mpdu) != 24) {
		ERR("beacon: header is %d B, not the 24 a beacon has - the reserved "
		    "page needs an unpadded [TXWI][MPDU]",
		    mt_hdrlen_from_fc(*mpdu));
		return -1;
	}
	return 0;
}

/*
 * The MBSS base address: MT_MAC_BSSID_DW0/DW1's address halves, leaving
 * MBSS_MODE / MBEACON_N / LOCAL_BIT alone.
 *
 * This is the half of "retarget the identity" that mt76 does and devourer's
 * ACK responder does not. mt76x02_mac_setaddr() moves mphy.macaddr,
 * MT_MAC_ADDR and MT_MAC_BSSID together, and the whole per-BSS index
 * derivation is written against that invariant. Move only MT_MAC_ADDR - which
 * is all mt7612u_set_ack_responder() does, and all this function used to do -
 * and the MBSS base is still the factory address, so the hardware derives the
 * BSS index from a different address than the host thinks it does. That is
 * silent: the AP beacons perfectly and matches nobody.
 */
static int mt_mac_set_bss_base(struct mt7612u_dev *d, const uint8_t *a)
{
	const uint32_t dw0 = (uint32_t)a[0] | ((uint32_t)a[1] << 8) |
	                     ((uint32_t)a[2] << 16) | ((uint32_t)a[3] << 24);
	const uint32_t dw1 = (uint32_t)a[4] | ((uint32_t)a[5] << 8);

	if (mt_wr_chk(d, MT_MAC_BSSID_DW0, dw0))
		return -1;
	return mt_rmw(d, MT_MAC_BSSID_DW1, MT_MAC_BSSID_DW1_ADDR, dw1);
}

/* Put the identity back if THIS call was what moved it. Shared by every
 * failure path in beacon_start and by beacon_stop. */
static void unwind_identity(struct mt7612u_dev *d, int took)
{
	const unsigned before = mt_io_errors(d);

	/*
	 * The two registers have different ownership and cannot share one flag.
	 *
	 * MT_MAC_BSSID has exactly two writers in the whole backend - mac_setaddr
	 * at init, and mt_mac_set_bss_base() here - so a beacon that moved it
	 * always owns it, and it is restored unconditionally. Gating it on `took`
	 * left it pointing at the beacon's addr2 after any hand-off, and nothing
	 * else in the library ever writes it back: the hardware then derives the
	 * BSS index from an address the host no longer believes it is using.
	 *
	 * MT_MAC_ADDR is co-owned with the ACK responder, so it is restored only
	 * while the beacon still holds it.
	 */
	mt_mac_set_bss_base(d, d->macaddr);
	if (took)
		mt7612u_clear_ack_responder(d);

	/* Flags survive a restore that did not land, so Stop()'s documented retry
	 * has something left to retry. Clearing them regardless made the second
	 * and third attempts no-ops against a still-leaked identity.
	 *
	 * This only reaches MT_MAC_ADDR because mt7612u_clear_ack_responder()
	 * keeps `ack_saved` on its own failure for the same reason - it early-
	 * returns on !ack_saved, so a retry that found the flag cleared would do
	 * nothing at all no matter what this function decides. The two halves of
	 * the retry have to agree. */
	if (mt_io_errors(d) != before)
		return;
	if (took)
		d->beacon_took_identity = 0;
}

int mt7612u_beacon_start(struct mt7612u_dev *dev, const void *buf, size_t len,
                         unsigned interval_tu)
{
	struct mt7612u_tx_rate rate;
	const uint8_t *mpdu = NULL;
	size_t mpdu_len = 0;
	static const uint8_t zero6[6] = { 0 };
	const uint8_t *ta, *bssid;
	uint8_t idx;
	unsigned before;
	int took = 0;

	if (!dev) return -1;
	if (beacon_split(buf, len, &mpdu, &mpdu_len, &rate)) return -1;

	ta = mpdu + 10;     /* addr2 - the transmitter, i.e. the port identity */
	bssid = mpdu + 16;  /* addr3 */

	if (ta[0] & 0x01) {
		ERR("beacon addr2 must be unicast; a station cannot unicast-auth to "
		    "a multicast BSSID");
		return -1;
	}

	/*
	 * mt76x02_add_interface(): the port identity FOLLOWS the interface
	 * address. IRadio says the same thing - "addr2/addr3 set the port
	 * MAC/BSSID" - and devourer's AP harnesses rely on it ("MACID = BSSID,
	 * set by StartBeacon", tests/ap_responder.cpp). Without this the MAC
	 * would keep ACKing for the adapter's factory MAC while beaconing a
	 * different BSSID, so a station's auth is never acknowledged and it
	 * retries until it gives up.
	 *
	 * mt7612u_set_ack_responder() is that register write, and it saves the
	 * factory identity so ClearAckResponder() can put it back. The two share
	 * one identity and one save slot by construction - there is only one
	 * MT_MAC_ADDR on this part - so a caller that arms a responder AND
	 * beacons is setting the same thing twice, and the restore is whichever
	 * of the two runs last.
	 */
	/* Unconditional, and BOTH registers. Unconditional because the old
	 * `ta != dev->macaddr` guard compared against the FACTORY address -
	 * dev->macaddr is written once, from the EEPROM, and nothing moves it -
	 * so a caller who had armed an ACK responder and then beaconed as the
	 * factory MAC kept the responder's address in MT_MAC_ADDR and ACKed for
	 * the wrong station all session. The retarget is idempotent and costs two
	 * EP0 writes; there is nothing to save by skipping it.
	 *
	 * Both registers because the index below is derived from the MBSS base,
	 * not from MT_MAC_ADDR. */

	/* Snapshot before the FIRST hardware write, so the delta covers the
	 * identity writes too - MT_MAC_ADDR_DW1 goes out as a bare mt_wr and the
	 * readback checks DW0 only, so a failed DW1 would otherwise be invisible
	 * and the AP would beacon with half an address. */
	before = mt_io_errors(dev);
	{
		/*
		 * `took` is decided BEFORE the writes and the device flag is set
		 * AFTER them, and the split is load-bearing in both directions.
		 *
		 * Before, because both calls move MT_MAC_ADDR and can then fail -
		 * set_ack_responder writes the register and readback-verifies
		 * afterwards, and set_bss_base is only reachable once that write
		 * landed - so the unwind needs to know we own it while those failures
		 * are still in flight.
		 *
		 * After, because mt7612u_set_ack_responder() CLEARS
		 * beacon_took_identity itself: that is how a caller arming a responder
		 * takes ownership away from a beacon. Setting the device flag first
		 * meant the call immediately below wiped it, and StopBeacon then never
		 * restored the identity on the success path - which is the previous
		 * round's fix for the failure paths breaking the success one.
		 */
		/*
		 * Always 1. The retarget below is unconditional, so this call always
		 * moves the identity and therefore always owns it at this instant.
		 *
		 * It used to be `!dev->ack_saved`, meaning "somebody else got here
		 * first, leave it to them" - but ack_saved is also set by THIS
		 * function's own call below, and is only cleared by a successful
		 * beacon_stop. So a re-arm over a live beacon, and any session whose
		 * config arms rx.ack_responder, both took the "somebody else" branch
		 * and disabled the restore for the rest of the session. And the branch
		 * protected nothing even when it fired: the retarget had already
		 * overwritten that responder's address in hardware, so declining to
		 * restore left MT_MAC_ADDR at the beacon's addr2 - neither the
		 * responder's address nor the factory one.
		 *
		 * Hand-off is the responder's job, not ours: mt7612u_set_ack_responder()
		 * clears beacon_took_identity, so a caller arming one AFTER the beacon
		 * takes ownership and beacon_stop then leaves it alone.
		 */
		took = 1;
		/*
		 * fail_post, not fail_pre, and that is the whole point of there being
		 * only one failure label past this line.
		 *
		 * -1 is contracted as "nothing was touched, whatever was airing still
		 * is", and these two exits cannot honour it: set_ack_responder has
		 * already written MT_MAC_ADDR before it readback-verifies, and
		 * set_bss_base is only reachable once that write landed. The unwind
		 * puts the FACTORY address back - the only address saved anywhere -
		 * so over a live beacon a failed re-arm restored an identity that
		 * beacon never had, while its page and timers kept airing it. The
		 * caller, told -1, left _beacon_active true and went on believing in
		 * an AP that beacons perfectly and acknowledges nobody.
		 *
		 * There is no atomic re-arm to offer here: one MT_MAC_ADDR, one save
		 * slot, and the previous occupant's address is not in it. So a failure
		 * after the identity moves takes the beacon down deliberately - engine
		 * disarmed, APC slots zeroed, identity retracted, -2 - which the
		 * caller can act on. Silence is a worse outcome than a deaf AP only if
		 * you are not told about it.
		 */
		if (mt7612u_set_ack_responder(dev, ta))
			goto fail_post;
		if (mt_mac_set_bss_base(dev, ta))
			goto fail_post;
		dev->beacon_took_identity = took;
	}

	/*
	 * The APC slot the hardware will match this BSS in. Under MBSS_MODE=3 the
	 * index comes from the address bits, and mt76 computes
	 *   idx = 1 + (((macaddr[0] ^ addr[0]) >> 2) & 7)
	 * for a locally-administered address, 0 otherwise (mt76x02_util.c:310).
	 * mt76 runs that AFTER mt76x02_mac_setaddr(), so its macaddr IS addr and
	 * the XOR is zero, collapsing the expression to 1. mt_mac_set_bss_base()
	 * above is what makes the same thing true here - without it the base
	 * stays the factory address, the hardware derives 1 + ((factory[0] ^
	 * ta[0]) >> 2 & 7), and this constant is right only for the adapters
	 * where that happens to be 1. Getting it wrong is silent: the AP beacons
	 * perfectly and acknowledges nobody.
	 */
	idx = (ta[0] & 0x02) ? 1 : 0;

	/*
	 * From here the hardware is being changed, so every exit unwinds. Two
	 * things are being unwound, and they are different:
	 *
	 *  - the identity, if THIS call took it. Leaving it retargeted after a
	 *    failed start means the adapter answers for a BSS that does not exist.
	 *  - the beacon engine, once mt_beacon_init() has disarmed it.
	 *
	 * Both unwind through the single fail_post label and return -2. -1 is
	 * reserved for the refusals ABOVE the first hardware write - bad input, a
	 * malformed frame, a multicast addr2 - so it can keep meaning "nothing was
	 * touched, whatever was airing still is" without qualification. An earlier
	 * draft returned -1 from inside the identity block, where that promise was
	 * already false.
	 *
	 * And the io_err delta, because several writes on this path report only
	 * their read halves or nothing at all - mt_beacon_set_enable()'s
	 * mt_rmw()/mt_set(), and MT_MAC_ADDR_DW1, which set_ack_responder writes
	 * with a bare mt_wr and whose readback checks DW0 only. Without the delta
	 * a failed DW1 transfer returns 0 and the AP beacons with half an address
	 * and acknowledges nobody. The snapshot is taken before the identity
	 * writes, above, for exactly that reason.
	 */

	/*
	 * No RX-filter change, and the reason is mt7612u_set_monitor_rx(), NOT a
	 * default. The init value is 0x00015f97 (initvals.h, re-written at
	 * init.cpp:290) and BIT(7) - MT_RX_FILTR_CFG_DUP - is SET in it. What
	 * leaves DUP clear is mt7612u_set_monitor_rx(), which rewrites the
	 * register as PHY_ERR and nothing else - these are DROP bits, so CRC_ERR
	 * joins it when the caller does NOT want corrupted frames kept
	 * (init.cpp:566, `if (!keep_corrupted)`). init.cpp:557 says dropping DUP
	 * there is deliberate, because "duplicate suppression would hide the
	 * retransmissions an ACK-responder test counts".
	 *
	 * Every AP path goes through it: Mt7612uRadio::StartRxLoop() calls it, and
	 * an AP has to receive. A TX-only consumer - InitWrite() with no RX loop -
	 * does not, and there DUP stays set; that costs it nothing, because with
	 * no receiver there are no retransmissions to count.
	 *
	 * So touching the filter here would be wrong in both directions: clearing
	 * DUP is a no-op on every path that beacons, and RESTORING it on the way
	 * out would switch duplicate filtering on in a session that deliberately
	 * had it off - destroying the retry=0 evidence the AP harness measures.
	 * Both were here for one round; neither belongs.
	 */
	mt_beacon_init(dev);
	/*
	 * Kept below mt_beacon_init(). It writes MT_MAC_APC_BSSID_L then _H and
	 * nothing in unwind_identity() touches the APC slots, so a failure between
	 * the two leaves a half-programmed slot: over a live beacon that
	 * half-overwrites the airing AP's own entry and it silently stops
	 * acknowledging. fail_post zeroes both slots, which is the only unwind
	 * that covers it, and everything from here down exits that way.
	 */
	if (mt_ap_set_bssid(dev, idx, bssid))
		goto fail_post;
	if (mt_beacon_write(dev, mpdu, mpdu_len, &rate))
		goto fail_post;
	if (mt_beacon_set_enable(dev, 1, interval_tu))
		goto fail_post;
	if (mt_io_errors(dev) != before) {
		ERR("beacon: a USB transfer failed while arming");
		goto fail_post;
	}
	/* Recorded only once the arm has succeeded. Written before the last
	 * failure exit, a -1 re-arm replaced it while the OLD beacon was still
	 * airing - and beacon_update then refused the live beacon's real addr2 and
	 * accepted one that was not on the air. Host state describing the hardware
	 * is part of what -1 promises not to change.
	 *
	 * addr2 and addr3 together: `ta` is mpdu + 10 and `bssid` is mpdu + 16, so
	 * twelve bytes from `ta` are exactly the pair, and both were programmed
	 * above - addr2 into the identity registers, addr3 into APC slot `idx`. */
	memcpy(dev->beacon_ident, ta, 12);
	return 0;

/*
 * The only failure label past the first hardware write, so -2 covers every one
 * of them and -1 is left to mean exactly what it says: refused on its input,
 * nothing touched. Disarm, erase, retract, in that order.
 */
fail_post:
	mt_beacon_set_enable(dev, 0, 0);
	/* The APC slot was programmed above; leave no BSS the MAC still matches.
	 * After a -2 the caller clears _beacon_active, so StopBeacon early-returns
	 * and this residue would be unreachable for the rest of the session. */
	mt_ap_set_bssid(dev, 0, zero6);
	mt_ap_set_bssid(dev, 1, zero6);
	unwind_identity(dev, took);
	return -2;
}

int mt7612u_beacon_update(struct mt7612u_dev *dev, const void *buf, size_t len)
{
	struct mt7612u_tx_rate rate;
	const uint8_t *mpdu = NULL;
	size_t mpdu_len = 0;
	int rc;

	if (!dev) return -1;
	if (beacon_split(buf, len, &mpdu, &mpdu_len, &rate)) return -1;

	/*
	 * Refuse a beacon that would change the identity. IRadio says addr2/addr3
	 * are not changeable mid-flight and that the port registers keep the
	 * StartBeacon identity - so loading one with a different BSSID airs a
	 * beacon that no longer matches the programmed APC slot or MT_MAC_ADDR.
	 * It beacons perfectly and acknowledges nobody, which is the failure the
	 * start path goes to some length to prevent.
	 *
	 * BOTH addresses, in one 12-byte compare over the adjacent addr2/addr3
	 * pair. Checking addr2 alone still admitted a changed addr3, and addr3 is
	 * the half that goes into the APC slot - the update would air a BSSID the
	 * slot does not hold, producing precisely the deaf AP described above
	 * through the guard meant to stop it.
	 */
	if (memcmp(mpdu + 10, dev->beacon_ident, 12) != 0) {
		ERR("beacon: an in-place update cannot change addr2 or addr3 - the "
		    "port identity and the APC slot keep what beacon_start programmed");
		return -1;
	}

	/*
	 * Everything that can refuse this payload runs BEFORE the slot is
	 * suppressed. mt_beacon_write() checks a caller-controlled length, and
	 * checking it after the guard was up meant a rejected payload left every
	 * slot suppressed with no path to lower them again - the AP silently off
	 * the air while _beacon_active still said otherwise.
	 */
	if (len_fits_slot(mpdu_len))
		return -1;

	if (mt_wr_chk(dev, MT_BCN_BYPASS_MASK, 0xffff)) {
		/* If the guard never lands the copy below runs against a LIVE slot,
		 * and a TBTT mid-copy airs a torn beacon - the one outcome this
		 * bracket exists to prevent, so failing to raise it is not the quiet
		 * path. Nothing to unwind: the mask is whatever it already was. */
		ERR("beacon: could not suppress the slot for an in-place update");
		return -1;
	}

	rc = mt_beacon_write(dev, mpdu, mpdu_len, &rate);
	if (rc) {
		/* Lower the guard again rather than leaving the AP dark. A failed
		 * update should cost the update, not the beacon. */
		mt_wr(dev, MT_BCN_BYPASS_MASK, 0xff00u | ~(0xff00u >> 1));
		return rc;
	}
	return 0;
}

int mt7612u_beacon_stop(struct mt7612u_dev *dev)
{
	static const uint8_t zero[6] = { 0 };
	unsigned before;
	int rc;

	if (!dev) return -1;

	/* The io_err delta is what makes a failed stop VISIBLE. mt_beacon_set_enable's
	 * off path is mt_clear() -> mt_rmw(), which reports only its READ half, and
	 * mt_ap_set_bssid()'s writes are checked but their returns were dropped. So
	 * this function used to be incapable of returning non-zero, which made
	 * Mt7612uRadio::StopBeacon's whole failure branch unreachable and the
	 * harness assertion for it vacuous - while the real hazard (an EP0 stall
	 * during teardown leaving the MAC beaconing) reported success. */
	before = mt_io_errors(dev);
	rc = mt_beacon_set_enable(dev, 0, 0);

	/*
	 * Retract the WHOLE identity, not just the timer. Leaving the APC slot
	 * programmed means the MAC keeps matching and auto-ACKing for a BSS that
	 * no longer exists, so a session that stops beaconing and carries on as an
	 * injector or a monitor drags that residue with it. The bring-up gate
	 * already zeroes the slot on every exit path for exactly this reason; the
	 * public path was the weaker of the two.
	 *
	 * Slot 1 and slot 0 are both cleared because beacon_start picks between
	 * them by the address's locally-administered bit, and stop does not have
	 * the beacon any more to re-derive which one it used.
	 */
	if (mt_ap_set_bssid(dev, 0, zero)) rc = -1;
	if (mt_ap_set_bssid(dev, 1, zero)) rc = -1;

	/* And the port MAC, if this call's opposite number was what retargeted
	 * it. mt7612u_clear_ack_responder() is the restore. */
	unwind_identity(dev, dev->beacon_took_identity);
	if (mt_io_errors(dev) != before) rc = -1;
	return rc;
}
