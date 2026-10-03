/*
 * station.cpp — the MT7612U half of IRadio::SetStationIdentity.
 *
 * This file is small, and it is small BECAUSE of a measurement rather than in
 * spite of one. docs/mt7612u-station-identity.md has the numbers; the short
 * version is that on this part a station needs almost nothing programmed, and
 * the one thing it must not do is the thing that looks most like the job.
 *
 * WHAT WAS MEASURED (mt7612uprobe's `sta` and `staack` gates, against
 * hostapd on independent silicon):
 *
 *   - MT_MAC_BSSID does not gate a managed station's receive: programmed
 *     deliberately WRONG, the DUT received 5877 unicast frames addressed to
 *     it against 6250 with nothing programmed; the AP's BSSID in the
 *     station's APC slot (slot 0) changed nothing either. So this function
 *     does NOT write them. A wrong BSSID in the station slot with its enable
 *     bit set was acknowledged 867/867 by a peer (one run); its effect on
 *     reception is not measured (docs/mt7612u-station-identity.md).
 *
 *   - Moving MT_MAC_ADDR makes a station DEAF. With the managed receive
 *     filter in force, retargeting the port identity took reception of the
 *     AP's unicast from 103 frames to ZERO. So this function does NOT write
 *     MT_MAC_ADDR either - it CHECKS it, and refuses if it has moved.
 *
 * That second number is the whole reason this seam exists separately from
 * mt7612u_set_ack_responder(). Arming an ACK responder on this part retargets
 * the port identity, so SetAckResponder(bssid) on a station would silence AND
 * deafen it. A station must leave MT_MAC_ADDR exactly where MAC bring-up put
 * it.
 *
 * AUTO-ACK NEEDS NO CALL. tests/mt7612u_sta_autoack.sh asks the TRANSMITTER,
 * the only party that knows whether its frame was acknowledged: a Realtek
 * peer injects unicast at this MAC and reads its own CCX reports. With nothing
 * armed, 100% acknowledged at 0.45 mean retries, against controls pinned at
 * the peer's retry limit - including MT_AUTO_RSP_EN cleared, which is why
 * this function refuses when that bit is clear.
 *
 * WHAT THIS DOES NOT DO: anything about transmission. A station's unicast
 * needs ACK-requesting radiotap and a nonzero tx.retry_limit - see
 * Mt7612uRadio::SetStationIdentity.
 *
 * THE ONE REGISTER IT WRITES is the receive filter. Every cell above ran the
 * managed filter, while Mt7612uRadio's RX loop installs the monitor filter;
 * left there, an armed station received promiscuously, and "moving the port
 * identity makes a station deaf" did not hold for it. So the arm installs
 * MT_RX_FILTR_CFG_MANAGED and the clear (or a drop) puts back what it found.
 *
 * So the useful work here is refusal and verification, plus that one filter.
 */
#include <string.h>

#include "StationIdentity.h"
#include "internal.h"
#include "regs.h"

/*
 * Read the port identity back out of the hardware. This is what the
 * auto-response engine matches an incoming frame's address 1 against, and on
 * this part it is the entire mechanism behind a station's auto-ACK.
 */
static int sta_read_port_identity(struct mt7612u_dev *d, uint8_t out[6])
{
	uint32_t dw0 = 0, dw1 = 0;

	/* Checked, not assumed. mt_rr_chk() leaves *val untouched when the
	 * transfer fails, so the zero-initialised locals would otherwise turn a
	 * failed read into the address 00:00:00:00:00:00 and refuse for the
	 * wrong reason. */
	if (mt_rr_chk(d, MT_MAC_ADDR_DW0, &dw0) != 0 ||
	    mt_rr_chk(d, MT_MAC_ADDR_DW1, &dw1) != 0)
		return -1;
	out[0] = (uint8_t)(dw0 & 0xff);
	out[1] = (uint8_t)((dw0 >> 8) & 0xff);
	out[2] = (uint8_t)((dw0 >> 16) & 0xff);
	out[3] = (uint8_t)((dw0 >> 24) & 0xff);
	out[4] = (uint8_t)(dw1 & 0xff);
	out[5] = (uint8_t)((dw1 >> 8) & 0xff);
	return 0;
}

/* Write the receive filter and read it back. */
static int sta_write_filter(struct mt7612u_dev *d, uint32_t v)
{
	uint32_t got = 0;

	if (mt_wr_chk(d, MT_RX_FILTR_CFG, v) != 0 ||
	    mt_rr_chk(d, MT_RX_FILTR_CFG, &got) != 0)
		return -1;
	return got == v ? 0 : -1;
}

int mt7612u_set_station_identity(struct mt7612u_dev *dev,
                                 const uint8_t own[6], const uint8_t bssid[6])
{
	uint8_t port[6] = { 0 };
	uint32_t rsp = 0, filtr = 0;
	int port_ok, rsp_ok;
	enum mt7612u_sta_verdict v;

	if (!dev)
		return -1;

	/* Arguments first: a caller mistake costs no USB transfer. */
	v = mt7612u_sta_check_args(own, bssid);
	if (v != MT7612U_STA_OK)
		goto refused;

	/* Read, then decide. The deciding is in StationIdentity.h so that every
	 * branch below - both failed-read paths included - is exercised
	 * headlessly by tests/mt7612u_station_selftest.cpp rather than only on a
	 * device. */
	port_ok = (sta_read_port_identity(dev, port) == 0);
	rsp_ok  = (mt_rr_chk(dev, MT_AUTO_RSP_CFG, &rsp) == 0);

	v = mt7612u_sta_decide(own, bssid, port, port_ok, rsp,
	                       rsp_ok, MT_AUTO_RSP_EN);
refused:
	switch (v) {
	case MT7612U_STA_OK:
		break;
	case MT7612U_STA_BAD_ARGS:
		WARN("station identity refused: null address");
		return -1;
	case MT7612U_STA_MULTICAST:
		WARN("station identity refused: own and bssid must both be unicast");
		return -1;
	case MT7612U_STA_SAME_ADDR:
		WARN("station identity refused: own == bssid");
		return -1;
	case MT7612U_STA_READ_FAILED:
		WARN("station identity refused: could not read the MAC back, so "
		     "there is nothing to verify against - refusing rather than "
		     "arming a station whose ability to receive and acknowledge is "
		     "unknown");
		return -1;
	case MT7612U_STA_PORT_MISMATCH:
		WARN("station identity refused: the MAC's port identity is "
		     "%02x:%02x:%02x:%02x:%02x:%02x, not the requested "
		     "%02x:%02x:%02x:%02x:%02x:%02x. Something else owns it (a "
		     "beacon or an ACK responder). A station on any other address "
		     "is not acknowledged, and under the managed receive filter "
		     "does not receive either.",
		     port[0], port[1], port[2], port[3], port[4], port[5],
		     own[0], own[1], own[2], own[3], own[4], own[5]);
		return -1;
	case MT7612U_STA_AUTO_RSP_OFF:
		WARN("station identity refused: MT_AUTO_RSP_EN is CLEAR (cfg %08x) "
		     "- the auto-response engine is switched off", rsp);
		return -1;
	}

	/*
	 * NOT written, deliberately: MT_MAC_ADDR, MT_MAC_BSSID and the
	 * MT_MAC_APC_BSSID slot table. The first must not move - that is the
	 * measured "reception goes to zero" failure. The other two made no
	 * measurable difference to what a managed station receives in the arms
	 * measured, and MT_MAC_BSSID already has two owners; a third writer on a
	 * register nothing needs would recreate the hazard this seam exists to
	 * avoid. docs/mt7612u-station-identity.md.
	 *
	 * The BSSID is recorded for the host: it is addr3 on every frame a
	 * station transmits. Power save, TIM parsing and per-BSS key lookup,
	 * which could give it a hardware use, are untested on this part.
	 */
	/*
	 * The managed receive filter - the receiver the cells measured, not the
	 * monitor one the RX loop installs. Read first, so the clear can put it
	 * back and so a failure leaves the register as found; nothing above this
	 * line writes, so every refusal before it leaves the filter untouched.
	 */
	if (mt_rr_chk(dev, MT_RX_FILTR_CFG, &filtr) != 0) {
		WARN("station identity refused: MT_RX_FILTR_CFG unreadable, so the "
		     "filter the clear must restore is unknown");
		return -1;
	}
	if (sta_write_filter(dev, MT_RX_FILTR_CFG_MANAGED) != 0) {
		/* Best effort: put back what it held. A previous arm, if any,
		 * stands - this call changed nothing it can confirm. */
		mt_wr(dev, MT_RX_FILTR_CFG, filtr);
		WARN("station identity refused: the managed receive filter "
		     "%08x did not read back", MT_RX_FILTR_CFG_MANAGED);
		return -1;
	}
	mt7612u_sta_arm(&dev->sta, own, bssid, filtr);
	return 0;
}

int mt7612u_clear_station_identity(struct mt7612u_dev *dev)
{
	if (!dev)
		return 0;
	/* Re-written for a lost arm too: its drop restored the filter best
	 * effort, and this is where that gets verified. */
	if ((dev->sta.armed || dev->sta.lost) &&
	    sta_write_filter(dev, dev->sta.rx_filtr_restore) != 0) {
		WARN("station identity clear: the pre-arm receive filter %08x did "
		     "not read back - the arm stays recorded so a second clear "
		     "retries it", dev->sta.rx_filtr_restore);
		return -1;
	}
	mt7612u_sta_clear(&dev->sta);
	return 0;
}

/*
 * Called after every write to MT_MAC_ADDR (mt7612u_set_ack_responder_as(),
 * which the beacon path goes through, and the beacon start's unwind), with
 * the register read back. It decides from what the register HOLDS, not from
 * what the caller meant to do, so a write that failed and left the identity
 * where it was does not drop anything.
 *
 * It does not refuse: a station arm does not veto the beacon and responder
 * paths. It makes the consequence audible and keeps `sta.armed` true to the
 * hardware. `allow_restore` is for an operation that failed and unwound the
 * identity back: the arm it dropped comes back with it.
 */
void mt7612u_station_identity_check(struct mt7612u_dev *dev, const char *who,
                                    int allow_restore)
{
	uint8_t port[6] = { 0 };
	unsigned io;
	int port_ok, filtr_rc = 0;
	uint32_t filtr_want = 0;
	enum mt7612u_sta_event ev;

	if (!dev || (!dev->sta.armed && !dev->sta.lost))
		return;
	/* Kept out of the I/O-error accumulator: this read, and the filter
	 * write below, must not fail an operation (a beacon start counts io
	 * errors) that did its own job. */
	io = mt_io_errors(dev);
	port_ok = sta_read_port_identity(dev, port) == 0;
	ev = mt7612u_sta_port_observed(&dev->sta,
	        mt7612u_port_compare(port, port_ok, dev->sta.own), allow_restore);
	/* The filter follows the arm: a dropped station gives the receiver back
	 * to the pre-arm filter (a beacon or responder that took the identity
	 * wants the monitor filter, DUP clear - beacon.cpp relies on it); a
	 * restored one takes it again. Read back, and a miss is said - it does
	 * not fail the caller's operation, and the clear re-writes and
	 * verifies. */
	if (ev == MT7612U_STA_EV_DROPPED || ev == MT7612U_STA_EV_RESTORED) {
		filtr_want = ev == MT7612U_STA_EV_DROPPED ? dev->sta.rx_filtr_restore
		                                          : MT_RX_FILTR_CFG_MANAGED;
		filtr_rc = sta_write_filter(dev, filtr_want);
	}
	mt_io_restore(dev, io);
	if (filtr_rc != 0)
		WARN("station identity %s after %s: the receive filter %08x did "
		     "not read back - the receiver may still run the %s filter",
		     ev == MT7612U_STA_EV_DROPPED ? "drop" : "restore", who,
		     filtr_want,
		     ev == MT7612U_STA_EV_DROPPED ? "managed (DUP set)" : "pre-arm");

	switch (ev) {
	case MT7612U_STA_EV_NONE:
		break;
	case MT7612U_STA_EV_DROPPED:
		WARN("station identity DROPPED: %s moved MT_MAC_ADDR to "
		     "%02x:%02x:%02x:%02x:%02x:%02x, away from this station's own "
		     "address. The MAC no longer acknowledges the AP's unicast to "
		     "the station, and the receive filter is back to its pre-arm "
		     "value (the monitor filter, under Mt7612uRadio's RX loop): "
		     "frames still arrive but go unacknowledged. Re-arm the "
		     "station identity after %s releases it.",
		     who, port[0], port[1], port[2], port[3], port[4], port[5], who);
		break;
	case MT7612U_STA_EV_RESTORED:
		WARN("station identity RESTORED: %s put MT_MAC_ADDR back to this "
		     "station's own address", who);
		break;
	case MT7612U_STA_EV_UNVERIFIED:
		WARN("station identity UNVERIFIED after %s: MT_MAC_ADDR could not "
		     "be read back, so whether it still holds this station's own "
		     "address is unknown. The arm is kept; re-arm to re-check it.",
		     who);
		break;
	}
}

int mt7612u_station_bssid(struct mt7612u_dev *dev, uint8_t out[6])
{
	if (!dev || !out || !dev->sta.armed)
		return -1;
	memcpy(out, dev->sta.bssid, 6);
	return 0;
}
