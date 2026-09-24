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

#endif /* _SC8547A_H_ */
