/*
 * StationIdentity.h - the decision half of SetStationIdentity, with no I/O.
 *
 * station.cpp reads three things off the chip (the port identity, whether that
 * read worked, and MT_AUTO_RSP_CFG) and then decides whether to arm. The
 * reading needs a device; the deciding does not, and the deciding is where
 * the easy mistakes are - a failed read that fails OPEN (arming a station
 * whose ability to acknowledge is unknown), or a failed read laundered into a
 * mismatch against 00:00:00:00:00:00 (refusing for the wrong reason). Neither
 * is visible in a register trace. Split out here, every branch runs in ctest:
 * tests/mt7612u_station_selftest.cpp.
 *
 * Pure: no device type, no register access, no logging. Safe to include from
 * a test with nothing but -I src/mt7612u.
 */
#ifndef MT7612U_STATION_IDENTITY_H
#define MT7612U_STATION_IDENTITY_H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

enum mt7612u_sta_verdict {
	MT7612U_STA_OK = 0,
	/* A caller mistake: null pointers, a multicast address, or `own` and
	 * `bssid` the same - a station whose own address is its BSSID is not a
	 * station. */
	MT7612U_STA_BAD_ARGS,
	MT7612U_STA_MULTICAST,
	MT7612U_STA_SAME_ADDR,
	/* Could not read the chip. Refused, not assumed: if we cannot find out
	 * whether this MAC will acknowledge anything, we do not get to claim it
	 * will. */
	MT7612U_STA_READ_FAILED,
	/* `own` is not the address the MAC is holding. Something else owns the
	 * port identity - a beacon, or an ACK responder - and moving it here
	 * would make this station deaf: measured, reception goes to zero.
	 * docs/mt7612u-station-identity.md. */
	MT7612U_STA_PORT_MISMATCH,
	/* The auto-response engine is switched off. Whatever did that did it
	 * deliberately, so this refuses rather than silently re-enabling it. */
	MT7612U_STA_AUTO_RSP_OFF
};

/*
 * `port_ok` / `rsp_ok` are whether the corresponding register read SUCCEEDED,
 * not whether its value is acceptable. Passing 0 for either must refuse -
 * that asymmetry is the bug this file exists to make testable.
 *
 * `auto_rsp_en_mask` is MT_AUTO_RSP_EN, passed in so this header needs no
 * register definitions.
 */
/* The argument half of the decision, callable before any register I/O so a
 * caller mistake costs no USB transfer and is reported as what it is. */
static inline enum mt7612u_sta_verdict
mt7612u_sta_check_args(const uint8_t *own, const uint8_t *bssid)
{
	if (!own || !bssid)
		return MT7612U_STA_BAD_ARGS;
	if ((own[0] & 0x01) || (bssid[0] & 0x01))
		return MT7612U_STA_MULTICAST;
	if (memcmp(own, bssid, 6) == 0)
		return MT7612U_STA_SAME_ADDR;
	return MT7612U_STA_OK;
}

static inline enum mt7612u_sta_verdict
mt7612u_sta_decide(const uint8_t *own, const uint8_t *bssid,
                   const uint8_t *port, int port_ok,
                   uint32_t auto_rsp_cfg, int rsp_ok,
                   uint32_t auto_rsp_en_mask)
{
	enum mt7612u_sta_verdict a = mt7612u_sta_check_args(own, bssid);

	if (a != MT7612U_STA_OK)
		return a;
	if (!port)
		return MT7612U_STA_BAD_ARGS;
	if (!port_ok)
		return MT7612U_STA_READ_FAILED;
	if (memcmp(port, own, 6) != 0)
		return MT7612U_STA_PORT_MISMATCH;
	if (!rsp_ok)
		return MT7612U_STA_READ_FAILED;
	if (!(auto_rsp_cfg & auto_rsp_en_mask))
		return MT7612U_STA_AUTO_RSP_OFF;
	return MT7612U_STA_OK;
}

/*
 * The ownership hand-off, as a pure state machine.
 *
 * SetStationIdentity's check is one-shot: it verifies the port identity when
 * it arms and has no further say. A beacon or an ACK responder armed LATER
 * can move MT_MAC_ADDR out from under a live station - the ordering a real
 * caller is likelier to hit than the one the arm-time check covers.
 *
 * This does not veto those paths; a station arm does not refuse them. After
 * any write to MT_MAC_ADDR the caller reads the register back, compares it
 * with the station's own address (mt7612u_port_compare) and hands the verdict
 * here:
 *
 *   SAME       - the identity did not move: the arm stands. If this operation
 *                had dropped it and then put the identity back (a start that
 *                failed and unwound), `allow_restore` re-arms it.
 *   DIFFERENT  - the identity verifiably moved: the arm is dropped, and its
 *                own/BSSID kept aside so an unwind can restore it.
 *   UNKNOWN    - the read failed: nothing is known, so the arm stands and the
 *                caller says so.
 */
enum mt7612u_port_cmp {
	MT7612U_PORT_SAME = 0,
	MT7612U_PORT_DIFFERENT,
	MT7612U_PORT_UNKNOWN
};

enum mt7612u_sta_event {
	MT7612U_STA_EV_NONE = 0,     /* nothing changed */
	MT7612U_STA_EV_DROPPED,      /* armed -> dropped: the identity moved */
	MT7612U_STA_EV_RESTORED,     /* dropped -> armed: it came back */
	MT7612U_STA_EV_UNVERIFIED    /* armed, but the read failed */
};

struct mt7612u_sta_state {
	uint8_t own[6];
	uint8_t bssid[6];
	int armed;
	/* Dropped by a port-identity move, own/bssid kept for a restore. */
	int lost;
};

/* `port` as read from MT_MAC_ADDR (DW0 + the low half of DW1) against `own`.
 * A failed read is UNKNOWN, never DIFFERENT. */
static inline enum mt7612u_port_cmp
mt7612u_port_compare(const uint8_t *port, int read_ok, const uint8_t *own)
{
	if (!read_ok || !port || !own)
		return MT7612U_PORT_UNKNOWN;
	return memcmp(port, own, 6) == 0 ? MT7612U_PORT_SAME
	                                 : MT7612U_PORT_DIFFERENT;
}

static inline void mt7612u_sta_arm(struct mt7612u_sta_state *s,
                                   const uint8_t *own, const uint8_t *bssid)
{
	memcpy(s->own, own, 6);
	memcpy(s->bssid, bssid, 6);
	s->armed = 1;
	s->lost = 0;
}

static inline void mt7612u_sta_clear(struct mt7612u_sta_state *s)
{
	memset(s, 0, sizeof *s);
}

/* Idempotent: observing the same move twice is one loss, not two. */
static inline enum mt7612u_sta_event
mt7612u_sta_port_observed(struct mt7612u_sta_state *s,
                          enum mt7612u_port_cmp c, int allow_restore)
{
	if (s->armed) {
		if (c == MT7612U_PORT_DIFFERENT) {
			s->armed = 0;
			s->lost = 1;
			return MT7612U_STA_EV_DROPPED;
		}
		return c == MT7612U_PORT_UNKNOWN ? MT7612U_STA_EV_UNVERIFIED
		                                 : MT7612U_STA_EV_NONE;
	}
	if (s->lost && allow_restore && c == MT7612U_PORT_SAME) {
		s->armed = 1;
		s->lost = 0;
		return MT7612U_STA_EV_RESTORED;
	}
	return MT7612U_STA_EV_NONE;
}

#ifdef __cplusplus
}
#endif

#endif /* MT7612U_STATION_IDENTITY_H */
