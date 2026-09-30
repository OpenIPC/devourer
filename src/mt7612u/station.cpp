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
 * So the useful work here is refusal and verification, not configuration.
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

int mt7612u_set_station_identity(struct mt7612u_dev *dev,
                                 const uint8_t own[6], const uint8_t bssid[6])
{
	uint8_t port[6] = { 0 };
	uint32_t rsp = 0;
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
	mt7612u_sta_arm(&dev->sta, own, bssid);
	return 0;
}

void mt7612u_clear_station_identity(struct mt7612u_dev *dev)
{
	if (!dev)
		return;
	/* Nothing to undo in hardware - this seam never wrote any. That is a
	 * property of this part and not a promise of the interface. */
	mt7612u_sta_clear(&dev->sta);
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
	int port_ok;

	if (!dev || (!dev->sta.armed && !dev->sta.lost))
		return;
	/* Kept out of the I/O-error accumulator: this read must not fail an
	 * operation (a beacon start counts io errors) that did its own job. */
	io = mt_io_errors(dev);
	port_ok = sta_read_port_identity(dev, port) == 0;
	mt_io_restore(dev, io);

	switch (mt7612u_sta_port_observed(&dev->sta,
	            mt7612u_port_compare(port, port_ok, dev->sta.own),
	            allow_restore)) {
	case MT7612U_STA_EV_NONE:
		break;
	case MT7612U_STA_EV_DROPPED:
		WARN("station identity DROPPED: %s moved MT_MAC_ADDR to "
		     "%02x:%02x:%02x:%02x:%02x:%02x, away from this station's own "
		     "address. The MAC no longer acknowledges the AP's unicast to "
		     "the station; under the MANAGED receive filter it no longer "
		     "receives it either (measured: reception goes to zero). Under "
		     "the monitor filter Mt7612uRadio's RX loop installs, frames "
		     "still arrive but go unacknowledged. Re-arm the station "
		     "identity after %s releases it.",
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
