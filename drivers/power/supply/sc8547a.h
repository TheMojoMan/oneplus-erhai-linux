/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * sc8547a.h — Southchip SC8547A drive/state interface usable by other
 * charger drivers (erhai_pps).  Only state queries, never register access.
 */
#ifndef _SC8547A_H_
#define _SC8547A_H_

#include <linux/types.h>

/*
 * Is the serial VOOC ladder currently DELIVERING (CP enabled, real SuperVOOC
 * current flowing)?  The PPS host engine stands down while this is true; the
 * genuine serial-SVOOC path keeps ownership of the plug.
 */
bool sc8547a_fastchg_delivering(void);

/*
 * Is the serial VOOC ladder in the identify or deliver phase (see sc8547a.c)?
 * The PPS host engine stands down while this is true, not just while
 * sc8547a_fastchg_delivering() reports real current — the identify window must
 * get its full 5 V chance before PPS negotiates.
 */
bool sc8547a_fastchg_identifying(void);

/*
 * PPS host-engine charge-pump API (b5).  Lets erhai_pps run the SC8547A 2:1
 * charge pump from a PD/PPS contract: it servos the PPS voltage so the chip
 * input sits at ~2*vbat + margin, then engages the CP with the same hardware
 * sequence the serial-VOOC deliver path uses, and feeds the 1000 ms REG_09
 * watchdog every poll.  All gated on the master instance; the serial ladder
 * (sc8547a_fastchg_delivering) trumps this path.
 */
int sc8547a_pps_snapshot(int *vbus_mv, int *vbat_mv, int *ibus_ma);
bool sc8547a_pps_cp_engaged(void);
int sc8547a_pps_cp_engage(int *need_mv);
int sc8547a_pps_cp_feed(int *vbus_mv, int *vbat_mv);
void sc8547a_pps_cp_disengage(void);

#endif /* _SC8547A_H_ */
