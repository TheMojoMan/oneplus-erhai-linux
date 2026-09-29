// SPDX-License-Identifier: GPL-2.0-only
// PPS negotiation/limit logic ported to Linux from the Android charger_pd
// PPS handling in vendor/oplus/kernel/charger/,
// in
//   github.com/OnePlusOSS/android_kernel_modules_and_devicetree_oneplus_sm8750
//   commit e888579 (OnePlus OPD2415_15.0.1.701(EX01))
// which has license GPL-2.0-only
//   (Copyright (C) 2020-2022 Oplus. All rights reserved.)
//
/*
 * erhai_pps.c — ADSP PPS fast-charge host engine (PD/PPS-only bricks).
 *
 * The SC8547A driver owns the proprietary serial (S)VOOC ladder (genuine
 * OnePlus bricks).  PD/PPS-only chargers (3rd-party bricks Android labels
 * "SuperVOOC", e.g. a 100 W Anker) never speak serial VOOC; Android fast-
 * charges them by renegotiating a PPS APDO through the ADSP charger_pd
 * firmware that is also alive on our boot, raising VBUS to ~8.4 V so the
 * PMIC buck converts (8.4 V x 2 A ≈ 17-19 W -> 4.1 V x ~4.4 A pack), then
 * granting the pack demand through the battery-psy profile props.
 *
 * That negotiation is just battery_charger property writes on the pmic_glink
 * edge this file already shares with erhai_otg_glink (v2 usb_property_id /
 * battery_property_id from oplus_hal_adsp.h):
 *
 *	USB (opcode 0x33):  49 USB_PPS_VOOCPHY_ENABLE, 34 USB_SET_PPS_VOLT mV,
 *		35 USB_SET_PPS_CURR mA, 5 USB_INPUT_CURR_LIMIT uA,
 *		39 USB_PPS_MOS_CTRL, 40 USB_PPS_CP_MODE_INIT
 *	Battery (opcode 0x31): 46 BATT_AP_FASTCHG_ALLOW, 10 BATT_CHG_CTRL_LIM uA
 *		(FCC / pack demand), 24 BATT_CHG_EN
 *
 * Live-proven subset (2026-09-23, 100 W Anker): 49/34/35/5 -> VBUS 4.8->8.4 V,
 * pack 1.0->3.6 A when the ADSP is fresh; the battery-psy props are the
 * profile gate Android's engine also drives (the total handshake).  This
 * module automates the whole sequence behind a guarded state machine so a
 * failure always falls back to the ordinary 5 V contract.
 *
 * Design / safety:
 *  - engages ONLY for PD_PPS adapters (ADSP USB_REAL_TYPE == 8) with PPS
 *    support (USB_GET_PPS_TYPE == 1) and NO serial VOOC active
 *    (USB_VOOC_FAST_CHG_TYPE below pps_vooc_min AND sc8547a not delivering) —
 *    the genuine OnePlus brick keeps its existing serial-SVOOC path,
 *  - starts PPS only after a debounce long enough for the serial ladder to
 *    decline (pps_identify_ms), so a slow-starting genuine brick wins,
 *  - steps VBUS up in pps_volt_step_mv increments (vendor order), servos the
 *    PPS current, tapers at vbat/pSoC limits and ALWAYS reverts to the 5 V
 *    contract on exit/detach/abnormal,
 *  - module params mirror the sc8547a driver style; every state crossing and
 *    exit reason is logged; counters/substates behind debugfs.
 *
 * No DT change.  Built as a separate module so shipping/removing it does not
 * touch the OTG/SC8547 drivers.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/fs.h>			/* simple_write_to_buffer() */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>

#include "erhai_otg_glink.h"
#include "sc8547a.h"

/* sc8547a PPS-CP API (see drivers/power/supply/sc8547a.c) — the 2:1 charge-pump
 * entry points this engine drives when the chip window is reachable.  Resolved
 * lazily via symbol_get() exactly like pps_sc_delivering(): this module must
 * load (and degrade to the buck path) even when the sc8547a module is absent. */
struct erhai_pps_sc_api {
	int	(*snapshot)(int *vbus_mv, int *vbat_mv, int *ibus_ma);
	bool	(*cp_engaged)(void);
	int	(*cp_engage)(int *need_mv);
	int	(*cp_feed)(int *vbus_mv, int *vbat_mv);
	void	(*cp_disengage)(void);
};

static const struct erhai_pps_sc_api *pps_sc_cp_get(void)
{
	static struct erhai_pps_sc_api api;
	static bool once;

	if (!once) {
		once = true;
		api.snapshot = (void *)symbol_get(sc8547a_pps_snapshot);
		api.cp_engaged = (void *)symbol_get(sc8547a_pps_cp_engaged);
		api.cp_engage = (void *)symbol_get(sc8547a_pps_cp_engage);
		api.cp_feed = (void *)symbol_get(sc8547a_pps_cp_feed);
		api.cp_disengage = (void *)symbol_get(sc8547a_pps_cp_disengage);
		if (!api.snapshot || !api.cp_engaged || !api.cp_engage ||
		    !api.cp_feed || !api.cp_disengage) {
			symbol_put(sc8547a_pps_snapshot);
			symbol_put(sc8547a_pps_cp_engaged);
			symbol_put(sc8547a_pps_cp_engage);
			symbol_put(sc8547a_pps_cp_feed);
			symbol_put(sc8547a_pps_cp_disengage);
			memset(&api, 0, sizeof(api));
			return NULL;
		}
	}
	return api.snapshot ? &api : NULL;
}

/* returns true while the CP is engaged AND the API is reachable */
static bool pps_cp_is_engaged(void)
{
	const struct erhai_pps_sc_api *a = pps_sc_cp_get();

	return a && a->cp_engaged();
}

static void pps_cp_drop(void)
{
	const struct erhai_pps_sc_api *a = pps_sc_cp_get();

	if (a && a->cp_engaged())
		a->cp_disengage();
}

/* ---- ADSP v2 property ids (oplus_hal_adsp.h), our subset ---- */
/* usb_property_id (BC_USB_PROPERTY_* / opcode 0x32,0x33) */
#define PPS_PROP_USB_INPUT_CURR_LIMIT	5
#define PPS_PROP_USB_ADAP_TYPE		7
#define PPS_PROP_USB_REAL_TYPE		11
#define PPS_PROP_USB_VOOC_FAST_CHG_TYPE	26
#define PPS_PROP_USB_GET_PPS_TYPE	32
#define PPS_PROP_USB_GET_PPS_STATUS	33
#define PPS_PROP_USB_SET_PPS_VOLT	34
#define PPS_PROP_USB_SET_PPS_CURR	35
#define PPS_PROP_USB_GET_PPS_MAX_CURR	36
#define PPS_PROP_USB_PPS_MOS_CTRL	39
#define PPS_PROP_USB_PPS_CP_MODE_INIT	40
#define PPS_PROP_USB_PPS_VOOCPHY_ENABLE	49
/* battery_property_id (BC_BATTERY_PROPERTY_* / opcode 0x30,0x31) */
#define PPS_PROP_BATT_CHG_CTRL_LIM	10
#define PPS_PROP_BATT_TEMP		12
#define PPS_PROP_BATT_CHG_EN		24
#define PPS_PROP_BATT_AP_FASTCHG_ALLOW	46
/* ADSP battery GET subset (battmgr ids, mirror erhai_otg_glink.c) */
#define PPS_PROP_BATT_STATUS		0
#define PPS_PROP_BATT_CAPACITY		4
#define PPS_PROP_BATT_VOLT_NOW		7
#define PPS_PROP_BATT_CURR_NOW		9

#define PPS_USB_TYPE_PD_PPS		8	/* POWER_SUPPLY_USB_TYPE_PD_PPS */

#define PPS_USB_OP_GET			0x32
#define PPS_USB_OP_SET			0x33
#define PPS_BATT_OP_GET			0x30
#define PPS_BATT_OP_SET			0x31


/* ---- module parameters (sc8547a-style knobs) ---- */
static bool pps_enable = true;
module_param(pps_enable, bool, 0644);
MODULE_PARM_DESC(pps_enable, "master kill switch for the PPS fast-charge engine");

static unsigned int pps_poll_ms = 2000;
module_param(pps_poll_ms, uint, 0644);
MODULE_PARM_DESC(pps_poll_ms, "engine poll period (ms)");

static unsigned int pps_identify_ms = 10000;
module_param(pps_identify_ms, uint, 0644);
MODULE_PARM_DESC(pps_identify_ms,
		 "wait for the serial-VOOC ladder to decline before PPS engages (ms)");

static unsigned int pps_start_volt_mv = 5500;
module_param(pps_start_volt_mv, uint, 0644);
MODULE_PARM_DESC(pps_start_volt_mv,
		 "legacy PPS entry VBUS (mV) - entry now writes the TARGET directly (validated "
		 "2026-09-24: the ADSP latches the PPS APDO range to the first 34 write, so a "
		 "5.5 V seed + step-ramp caps VBUS at ~6.6 V); kept as the ramp liveness floor");

static unsigned int pps_target_volt_mv = 8400;
module_param(pps_target_volt_mv, uint, 0644);
MODULE_PARM_DESC(pps_target_volt_mv,
		 "PPS target VBUS (mV) - Android measures 8.4-8.6 V with the 100 W Anker");

static unsigned int pps_max_volt_mv = 8900;
module_param(pps_max_volt_mv, uint, 0644);
MODULE_PARM_DESC(pps_max_volt_mv, "hard PPS VBUS ceiling (mV)");

static unsigned int pps_volt_step_mv = 400;
module_param(pps_volt_step_mv, uint, 0644);
MODULE_PARM_DESC(pps_volt_step_mv, "VBUS ramp step (mV) per poll");

static unsigned int pps_start_curr_ma = 1000;
module_param(pps_start_curr_ma, uint, 0644);
MODULE_PARM_DESC(pps_start_curr_ma, "PPS entry current (mA)");

static unsigned int pps_target_curr_ma = 3000;
module_param(pps_target_curr_ma, uint, 0644);
MODULE_PARM_DESC(pps_target_curr_ma,
		 "PPS input current target (mA) - live-validated 3000 with the 100 W Anker "
		 "(8.2-8.6 V x 3 A in = ~24 W); Android's 2 A ICL is a floor, not a ceiling");

static unsigned int pps_max_curr_ma = 3000;
module_param(pps_max_curr_ma, uint, 0644);
MODULE_PARM_DESC(pps_max_curr_ma, "hard PPS input current ceiling (mA)");

static unsigned int pps_iclim_ma = 3000;
module_param(pps_iclim_ma, uint, 0644);
MODULE_PARM_DESC(pps_iclim_ma,
		 "USB_INPUT_CURR_LIMIT written on PPS entry (mA); the ADSP honours 3000 when "
		 "the battery demand is granted (validated 2026-09-24)");

static unsigned int pps_fcc_ma = 4400;
module_param(pps_fcc_ma, uint, 0644);
MODULE_PARM_DESC(pps_fcc_ma,
		 "pack charge demand granted via BATT_CHG_CTRL_LIM (mA); Android draws ~4.4 A at 2 A/8.4 V in");

static unsigned int pps_soc_stop = 80;
module_param(pps_soc_stop, uint, 0644);
MODULE_PARM_DESC(pps_soc_stop, "exit PPS at this pSoC (%)");

static unsigned int pps_soc_restart = 75;
module_param(pps_soc_restart, uint, 0644);
MODULE_PARM_DESC(pps_soc_restart,
		 "below this pSoC a stopped PPS session may resume (%)");

static unsigned int pps_vbat_max_mv = 4490;
module_param(pps_vbat_max_mv, uint, 0644);
MODULE_PARM_DESC(pps_vbat_max_mv, "taper/exit at this pack voltage (mV)");

static unsigned int pps_vbat_min_mv = 3300;
module_param(pps_vbat_min_mv, uint, 0644);
MODULE_PARM_DESC(pps_vbat_min_mv, "abort below this pack voltage (mV)");

static unsigned int pps_vbus_collapse_mv = 3800;
module_param(pps_vbus_collapse_mv, uint, 0644);
MODULE_PARM_DESC(pps_vbus_collapse_mv,
		 "VBUS below this while targeting >5 V means a bad cable/adapter -> exit (mV)");

static int pps_temp_min_c = -5;
module_param(pps_temp_min_c, int, 0644);
MODULE_PARM_DESC(pps_temp_min_c, "PPS pack temp floor (C)");

static int pps_temp_max_c = 45;
module_param(pps_temp_max_c, int, 0644);
MODULE_PARM_DESC(pps_temp_max_c, "PPS pack temp ceiling (C)");

static unsigned int pps_vooc_min = 2;
module_param(pps_vooc_min, uint, 0644);
MODULE_PARM_DESC(pps_vooc_min,
		 "ADSP USB_VOOC_FAST_CHG_TYPE at or above this means serial VOOC owns the plug; never engage PPS");

static unsigned int pps_stall_polls = 15;
module_param(pps_stall_polls, uint, 0644);
MODULE_PARM_DESC(pps_stall_polls,
		 "consecutive polls with ~no pack current at >7 V VBUS before exiting (0 = never)");

static bool pps_cp_init = true;
module_param(pps_cp_init, bool, 0644);
MODULE_PARM_DESC(pps_cp_init,
		 "also write USB_PPS_CP_MODE_INIT(40)=1 + USB_PPS_MOS_CTRL(39)=1 on entry (vendor PHY init)");

/* ---- SC8547A charge-pump 2:1 (CP direct-charge from the PPS bus) ---- */
static bool pps_cp_enable = true;
module_param(pps_cp_enable, bool, 0644);
MODULE_PARM_DESC(pps_cp_enable,
		 "drive the SC8547A charge pump 2:1 from the PPS bus when the chip window is reachable "
		 "(b5; owner-testing).  Disable = pure buck path (b4 behaviour).");

static unsigned int pps_cp_target_mv = 150;
module_param(pps_cp_target_mv, uint, 0644);
MODULE_PARM_DESC(pps_cp_target_mv,
		 "CP servo target for vbus - 2*vbat at the chip (mV; engage window is 110..700)");

static unsigned int pps_cp_vstep_mv = 50;
module_param(pps_cp_vstep_mv, uint, 0644);
MODULE_PARM_DESC(pps_cp_vstep_mv, "CP PPS-voltage servo step (mV) per poll");

static unsigned int pps_cp_max_volt_mv = 9500;
module_param(pps_cp_max_volt_mv, uint, 0644);
MODULE_PARM_DESC(pps_cp_max_volt_mv, "CP servo PPS VBUS ceiling (mV)");

static unsigned int pps_cp_min_volt_mv = 8400;
module_param(pps_cp_min_volt_mv, uint, 0644);
MODULE_PARM_DESC(pps_cp_min_volt_mv, "CP servo PPS VBUS floor (mV) - never undershoot the buck point");

static unsigned int pps_cp_feed_ms = 500;
module_param(pps_cp_feed_ms, uint, 0644);
MODULE_PARM_DESC(pps_cp_feed_ms,
		 "engine poll period (ms) while the CP is engaged - must stay under the "
		 "SC8547A 1000 ms REG_09 watchdog window");

/* ---- state machine ---- */
enum pps_state {
	PPS_IDLE = 0,		/* nothing attached / not a PPS adapter */
	PPS_ARMED,		/* PD_PPS seen; waiting for serial ladder to decline */
	PPS_NEGOTIATE,		/* writing profile gate + PPS props */
	PPS_RAMP,		/* stepping VBUS up toward the target */
	PPS_CHARGING,		/* servoing PPS current against vbat/pSoC */
	PPS_TAPER,		/* near CV: reducing current before exit */
	PPS_EXIT,		/* reverting to the 5 V contract */
	PPS_LAST_STATE,
};

static const char *const pps_state_names[PPS_LAST_STATE] = {
	[PPS_IDLE]	= "idle",
	[PPS_ARMED]	= "armed",
	[PPS_NEGOTIATE]	= "negotiate",
	[PPS_RAMP]	= "ramp",
	[PPS_CHARGING]	= "charging",
	[PPS_TAPER]	= "taper",
	[PPS_EXIT]	= "exit",
};

/* exit reasons (also the log tag) */
#define PPS_EXIT_REASON_DETACH		"detach"
#define PPS_EXIT_REASON_NOT_PPS		"not-a-pps-adapter"
#define PPS_EXIT_REASON_VOOC_ENGAGED	"serial-vooc-engaged"
#define PPS_EXIT_REASON_SOC		"soc-limit"
#define PPS_EXIT_REASON_VBAT_MAX	"vbat-cv"
#define PPS_EXIT_REASON_VBAT_MIN	"vbat-low"
#define PPS_EXIT_REASON_TEMP		"temp-out-of-window"
#define PPS_EXIT_REASON_COLLAPSE	"vbus-collapse"
#define PPS_EXIT_REASON_STALL		"pack-stall"
#define PPS_EXIT_REASON_SET_FAIL	"prop-set-failed"
#define PPS_EXIT_REASON_RAMP_TIMEOUT	"vbus-not-rising"
#define PPS_EXIT_REASON_FORCED		"forced"

struct erhai_pps {
	struct delayed_work	work;
	struct dentry		*debugfs;

	enum pps_state		state;
	const char		*last_reason;
	unsigned long		state_since;	/* jiffies state was entered */
	unsigned long		armed_since;
	unsigned long		ramp_since;
	unsigned int		stall_count;

	/* what we last commanded / observed */
	unsigned int		set_volt_mv;
	unsigned int		set_curr_ma;
	unsigned int		target_curr_ma;
	int			vbus_mv;
	int			vbat_mv;
	int			ibat_ma;
	int			soc;
	int			temp_c;

	/* stats */
	u64			sessions;
	u32			entries;
	unsigned int		exits;
	unsigned long		charg_ticks;
	unsigned int		cp_engages;
	unsigned int		cp_drops;
};

static struct erhai_pps *g_pps;
static DEFINE_MUTEX(pps_lock);

static void pps_enter(struct erhai_pps *p, enum pps_state st, const char *why);

/* optional serial-VOOC gate (sc8547a module) */
static bool pps_sc_delivering(void)
{
	static bool (*fn)(void);
	bool delivering = false;

	if (!fn) {
		fn = (bool (*)(void))symbol_get(sc8547a_fastchg_delivering);
		if (fn)
			pr_debug("erhai_pps: sc8547a delivering() gate linked\n");
	}
	if (fn)
		delivering = fn();

	return delivering;
}

/* optional "serial ladder still identifying/delivering" gate (sc8547a module) —
 * the engine must not capture the connector while the ladder is mid-identify
 * (2026-09-25): a genuine SuperVOOC brick identifies in its 5 V window and is
 * only lost when the engine's debounce laps it and lifts VBUS to PPS first. */
static bool pps_sc_identifying(void)
{
	static bool (*fn)(void);
	bool identifying = false;

	if (!fn) {
		fn = (bool (*)(void))symbol_get(sc8547a_fastchg_identifying);
		if (fn)
			pr_debug("erhai_pps: sc8547a identifying() gate linked\n");
	}
	if (fn)
		identifying = fn();

	return identifying;
}

/* ---- ADSP prop helpers ---- */
static int pps_usb_get(u32 prop)
{
	return erhai_otg_adsp_prop_get(PPS_USB_OP_GET, prop);
}

static int pps_usb_set(u32 prop, u32 value)
{
	return erhai_otg_adsp_prop_set(PPS_USB_OP_SET, prop, value);
}

static int pps_batt_set(u32 prop, u32 value)
{
	return erhai_otg_adsp_prop_set(PPS_BATT_OP_SET, prop, value);
}

static void pps_prop_err(struct erhai_pps *p, const char *what, int rc)
{
	pr_warn("erhai_pps: %s failed rc=%d (state %s); safe exit to 5 V\n",
		what, rc, pps_state_names[p->state]);
}

/* ---- pack reads: prefer the real pack gauge, fall back to the ADSP battery ---- */
static struct power_supply *pps_bat_psy(void)
{
	struct power_supply *psy = power_supply_get_by_name("bq27z561-0");

	if (!psy)
		psy = power_supply_get_by_name("erhai-adsp-bat");
	return psy;
}

static int pps_psy_int(struct power_supply *psy, enum power_supply_property prop)
{
	union power_supply_propval val = { };
	int rc;

	rc = power_supply_get_property(psy, prop, &val);
	if (rc)
		return rc;
	return val.intval;
}

/* fills p->vbat_mv / p->ibat_ma / p->soc / p->temp_c; returns 0 on success */
static int pps_read_battery(struct erhai_pps *p)
{
	struct power_supply *psy = pps_bat_psy();
	int v, i, s, t;

	if (!psy)
		return -ENODEV;

	v = pps_psy_int(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	i = pps_psy_int(psy, POWER_SUPPLY_PROP_CURRENT_NOW);
	s = pps_psy_int(psy, POWER_SUPPLY_PROP_CAPACITY);
	t = pps_psy_int(psy, POWER_SUPPLY_PROP_TEMP);
	power_supply_put(psy);
	if (v < 0 || i < 0 || s < 0 || t < 0)
		return -EIO;

	/* sysfs battery: voltage_now uV -> mV; current_now uA -> mA; temp 0.1C -> C */
	p->vbat_mv = v / 1000;
	p->ibat_ma = i / 1000;
	p->soc = s;
	p->temp_c = t / 10;
	return 0;
}

/* reads VBUS through the ADSP (USB_VOLT_NOW in uV) */
static int pps_read_vbus(struct erhai_pps *p)
{
	int v = pps_usb_get(1);			/* USB_VOLT_NOW, uV */

	if (v < 0)
		return v;
	p->vbus_mv = v / 1000;
	return 0;
}

static bool pps_deployable(struct erhai_pps *p, bool *vooc_engaged)
{
	int real, pps_type, fct;

	*vooc_engaged = false;

	real = pps_usb_get(PPS_PROP_USB_REAL_TYPE);
	pps_type = pps_usb_get(PPS_PROP_USB_GET_PPS_TYPE);
	fct = pps_usb_get(PPS_PROP_USB_VOOC_FAST_CHG_TYPE);

	if (real != PPS_USB_TYPE_PD_PPS || pps_type != 1)
		return false;
	if (fct >= (int)pps_vooc_min || pps_sc_delivering() ||
	    pps_sc_identifying()) {
		*vooc_engaged = true;
		return false;
	}
	if (pps_read_battery(p))
		return false;
	if (p->soc >= (int)pps_soc_stop)
		return false;
	if (p->vbat_mv < (int)pps_vbat_min_mv ||
	    p->vbat_mv > (int)pps_vbat_max_mv)
		return false;
	if (p->temp_c < pps_temp_min_c || p->temp_c > pps_temp_max_c)
		return false;
	return true;
}


/* ---- the 5 V revert ---- */
static void pps_revert(struct erhai_pps *p)
{
	int rc;

	/* Order matters: current+voltage back to 5 V first, then limits. */
	rc = pps_usb_set(PPS_PROP_USB_SET_PPS_CURR, 2000);
	if (rc)
		pps_prop_err(p, "revert PPS_CURR", rc);
	rc = pps_usb_set(PPS_PROP_USB_SET_PPS_VOLT, 5000);
	if (rc)
		pps_prop_err(p, "revert PPS_VOLT", rc);
	if (pps_cp_init) {
		rc = pps_usb_set(PPS_PROP_USB_PPS_MOS_CTRL, 0);
		if (rc)
			pps_prop_err(p, "revert MOS_CTRL", rc);
	}
	/* Release the profile gate; keep charging enabled for normal 5 V. */
	rc = pps_batt_set(PPS_PROP_BATT_AP_FASTCHG_ALLOW, 0);
	if (rc)
		pps_prop_err(p, "revert FASTCHG_ALLOW", rc);
	rc = pps_batt_set(PPS_PROP_BATT_CHG_CTRL_LIM, 0);
	if (rc)
		pps_prop_err(p, "revert CHG_CTRL_LIM", rc);
	rc = pps_batt_set(PPS_PROP_BATT_CHG_EN, 1);
	if (rc)
		pps_prop_err(p, "revert CHG_EN", rc);
	rc = pps_usb_set(PPS_PROP_USB_INPUT_CURR_LIMIT, 1500000);
	if (rc)
		pps_prop_err(p, "revert ICL", rc);
	rc = pps_usb_set(PPS_PROP_USB_PPS_VOOCPHY_ENABLE, 0);
	if (rc)
		pps_prop_err(p, "revert VOOCPHY", rc);

	p->set_volt_mv = 0;
	p->set_curr_ma = 0;
	p->stall_count = 0;
}

/* ---- state transitions ---- */
static void pps_enter(struct erhai_pps *p, enum pps_state st, const char *why)
{
	if (p->state != st && st != PPS_EXIT)
		pr_info("erhai_pps: %s -> %s%s%s%s\n",
			pps_state_names[p->state], pps_state_names[st],
			why ? " (" : "", why ?: "", why ? ")" : "");
	p->state = st;
	p->state_since = jiffies;
	if (st == PPS_IDLE)
		p->last_reason = NULL;
	if (st == PPS_ARMED)
		p->armed_since = jiffies;
	if (st == PPS_RAMP)
		p->ramp_since = jiffies;
}

static void pps_exit(struct erhai_pps *p, const char *reason)
{
	if (p->state == PPS_IDLE || p->state == PPS_EXIT)
		return;
	p->last_reason = reason;
	p->exits++;
	p->charg_ticks = 0;
	pr_info("erhai_pps: exit: %s\n", reason);
	/* drop the charge pump BEFORE the 5 V revert: at 5 V the 2:1 window is
	 * gone and a latched CHG_EN has nothing to drive but also no watchdog. */
	if (pps_cp_is_engaged())
		pps_cp_drop();
	pps_revert(p);
	pps_enter(p, PPS_IDLE, NULL);
}


/* ---- the poll ---- */
static void pps_poll(struct erhai_pps *p)
{
	int online, rc;

	if (!pps_enable)
		return;

	online = pps_usb_get(0);	/* USB_ONLINE */
	if (pps_read_vbus(p) < 0)
		online = -EINVAL;	/* ADSP not up yet -> treat as offline */

	switch (p->state) {
	case PPS_IDLE: {
		bool vooc;

		if (online <= 0)
			return;
		if (!pps_deployable(p, &vooc))
			return;
		pps_enter(p, PPS_ARMED, "PD_PPS adapter");
		return;
	}
	case PPS_ARMED: {
		bool vooc;

		if (online <= 0) {
			pps_exit(p, PPS_EXIT_REASON_DETACH);
			return;
		}
		if (pps_deployable(p, &vooc) && !vooc) {
			if (time_after_eq(jiffies, p->armed_since +
					  msecs_to_jiffies(pps_identify_ms)))
				pps_enter(p, PPS_NEGOTIATE, NULL);
			return;
		}
		if (vooc) {
			/* serial VOOC won: stand down for this plug */
			pps_exit(p, PPS_EXIT_REASON_VOOC_ENGAGED);
			return;
		}
		/* adapter vanished or no longer deployable -> back to idle */
		p->state = PPS_IDLE;
		return;
	}
	case PPS_NEGOTIATE: {
		/*
		 * Entry sequence — ORDER IS LOAD-BEARING (live-validated 2026-09-24,
		 * 100 W Anker, b3 kernel): the ADSP charger_pd latches its PPS APDO
		 * from the first writes and grants the pack demand only after the
		 * battery profile gate is set.  The working order is:
		 *
		 *   49 PPS_EN -> 34 VOLT at TARGET (first!) -> 35 CURR -> 5 ICL,
		 * then the battery profile gate (46/24/10) and the CP PHY init
		 * (40/39) AFTER the PDO is established.
		 *
		 * Measured with the reverse/enhanced variants: CURR-before-VOLT plus
		 * either a 5.5 V seed+step-ramp (VBUS caps at ~6.64 V = ~9 W pack)
		 * or a one-shot 8.4 V request (VBUS 4.6 V); granting the gate before
		 * the PDO caps input at ~1.3 A.  The sequence below delivers
		 * 8.2-8.6 V x 3 A in ~= 24 W, pack 3.6-4 A and climbing.
		 */
		rc = pps_usb_set(PPS_PROP_USB_PPS_VOOCPHY_ENABLE, 1);
		if (rc)
			goto fail;
		rc = pps_usb_set(PPS_PROP_USB_SET_PPS_VOLT, pps_target_volt_mv);
		if (rc)
			goto fail;
		rc = pps_usb_set(PPS_PROP_USB_SET_PPS_CURR, pps_start_curr_ma);
		if (rc)
			goto fail;
		rc = pps_usb_set(PPS_PROP_USB_INPUT_CURR_LIMIT,
				 pps_iclim_ma * 1000);
		if (rc)
			goto fail;

		/* profile gate + CP PHY init AFTER the PDO (validated order) */
		rc = pps_batt_set(PPS_PROP_BATT_AP_FASTCHG_ALLOW, 1);
		if (rc)
			goto fail;
		rc = pps_batt_set(PPS_PROP_BATT_CHG_EN, 1);
		if (rc)
			goto fail;
		rc = pps_batt_set(PPS_PROP_BATT_CHG_CTRL_LIM, pps_fcc_ma * 1000);
		if (rc)
			goto fail;
		if (pps_cp_init) {
			rc = pps_usb_set(PPS_PROP_USB_PPS_CP_MODE_INIT, 1);
			if (rc)
				goto fail;
			rc = pps_usb_set(PPS_PROP_USB_PPS_MOS_CTRL, 1);
			if (rc)
				goto fail;
		}

		p->set_volt_mv = pps_target_volt_mv;
		p->set_curr_ma = pps_start_curr_ma;
		p->target_curr_ma = min(pps_target_curr_ma, pps_max_curr_ma);
		p->sessions++;
		p->entries++;
		pps_enter(p, PPS_RAMP, NULL);
		return;

fail:
		pps_prop_err(p, "PPS ENTER", rc);
		pps_exit(p, PPS_EXIT_REASON_SET_FAIL);
		return;
	}
	case PPS_RAMP: {
		unsigned int next, ci;

		if (online <= 0) {
			pps_exit(p, PPS_EXIT_REASON_DETACH);
			return;
		}
		/* did we actually leave 5 V in time? */
		if (time_after_eq(jiffies, p->ramp_since +
				  msecs_to_jiffies(pps_poll_ms * 6)) &&
		    p->vbus_mv < (int)pps_start_volt_mv) {
			pps_exit(p, PPS_EXIT_REASON_RAMP_TIMEOUT);
			return;
		}
		if (p->vbus_mv < (int)pps_vbus_collapse_mv) {
			pps_exit(p, PPS_EXIT_REASON_COLLAPSE);
			return;
		}
		if (p->set_volt_mv >= pps_target_volt_mv) {
			pps_enter(p, PPS_CHARGING, "VBUS at target");
			return;
		}
		next = min(p->set_volt_mv + pps_volt_step_mv, pps_target_volt_mv);
		next = min(next, pps_max_volt_mv);
		if (pps_usb_set(PPS_PROP_USB_SET_PPS_VOLT, next)) {
			pps_exit(p, PPS_EXIT_REASON_SET_FAIL);
			return;
		}
		/* keep the input current moving toward the target */
		if (p->set_curr_ma < p->target_curr_ma) {
			ci = min(p->set_curr_ma + 250U, p->target_curr_ma);
			if (!pps_usb_set(PPS_PROP_USB_SET_PPS_CURR, ci))
				p->set_curr_ma = ci;
		}
		p->set_volt_mv = next;
		pr_info("erhai_pps: ramp VBUS %u mV curr %u mA (read %d mV)\n",
			p->set_volt_mv, p->set_curr_ma, p->vbus_mv);
		return;
	}
	case PPS_CHARGING:
	case PPS_TAPER: {
		bool charging = (p->state == PPS_CHARGING);

		if (online <= 0) {
			pps_exit(p, PPS_EXIT_REASON_DETACH);
			return;
		}
		rc = pps_read_battery(p);
		if (rc)
			return;

		/* safety exits first */
		if (p->vbat_mv < (int)pps_vbat_min_mv) {
			pps_exit(p, PPS_EXIT_REASON_VBAT_MIN);
			return;
		}
		if (p->soc >= (int)pps_soc_stop) {
			pps_exit(p, PPS_EXIT_REASON_SOC);
			return;
		}
		if (p->temp_c < pps_temp_min_c || p->temp_c > pps_temp_max_c) {
			pps_exit(p, PPS_EXIT_REASON_TEMP);
			return;
		}
		if (p->vbus_mv < (int)pps_vbus_collapse_mv) {
			pps_exit(p, PPS_EXIT_REASON_COLLAPSE);
			return;
		}
		/* pack-stall watchdog: nothing flowing at high VBUS */
		if (pps_stall_polls && p->vbus_mv > 7000 &&
		    p->ibat_ma < 100) {
			if (++p->stall_count >= pps_stall_polls) {
				pps_exit(p, PPS_EXIT_REASON_STALL);
				return;
			}
		} else {
			p->stall_count = 0;
		}

		if (p->vbat_mv >= (int)pps_vbat_max_mv) {
			if (charging)
				pps_enter(p, PPS_TAPER, "vbat at CV");
			/* taper: shed current in steps, then hand to 5 V */
			if (p->set_curr_ma > 500) {
				unsigned int n = max(500U,
						     p->set_curr_ma - 250U);

				if (!pps_usb_set(PPS_PROP_USB_SET_PPS_CURR, n))
					p->set_curr_ma = n;
			} else {
				pps_exit(p, PPS_EXIT_REASON_VBAT_MAX);
			}
			return;
		}

		/*
		 * current servo: the ADSP fills the pack up to the ICL/FCC
		 * grant; we nudge PPS curr to the input target, never yank.
		 */
		if (p->set_curr_ma < p->target_curr_ma) {
			unsigned int n = min(p->set_curr_ma + 250U,
					     p->target_curr_ma);

			if (!pps_usb_set(PPS_PROP_USB_SET_PPS_CURR, n))
				p->set_curr_ma = n;
		}

		/*
		 * CP 2:1 direct-charge (b5, owner-testing): run the SC8547A pump
		 * from the PPS bus when the chip's vbus - 2*vbat sits inside the
		 * engage window, servoing the PPS VOLT setpoint in small steps
		 * toward pps_cp_target_mv (the ADSP's settle is its own closed-loop
		 * choice — open-loop setpoints wander ~8.1-8.3 V — so the chip ADC
		 * is the only reliable steering input).  On any excursion the CP
		 * disengages and we keep the buck path (stay in PPS).  Never runs
		 * while the serial VOOC ladder is delivering; the engine's
		 * collapse/soc/temp/vbat exits drop the CP via pps_exit BEFORE
		 * the 5 V revert.
		 */
		if (pps_cp_enable && charging) {
			const struct erhai_pps_sc_api *a = pps_sc_cp_get();

			if (a) {
				int cvbus = 0, cvbat = 0, diff = 0, rc;

				rc = a->snapshot(&cvbus, &cvbat, NULL);
				if (rc == 0) {
					diff = cvbus - cvbat * 2;
					if (a->cp_engaged()) {
						rc = a->cp_feed(&cvbus, &cvbat);
						if (rc) {
							pr_info("erhai_pps: CP feed drop rc=%d (diff %d mV) - buck path\n",
								rc, diff);
							p->cp_drops++;
						}
					} else {
						/* engage only once the chip window is
						 * (at least) reachable */
						if (diff >= 0 &&
						    p->set_volt_mv <= pps_cp_max_volt_mv) {
							int need = 0;

							rc = a->cp_engage(&need);
							if (rc == 0) {
								p->cp_engages++;
								pr_info("erhai_pps: CP 2:1 ENGAGED (chip vbus %d mV vbat %d mV diff %d mV)\n",
								    cvbus, cvbat, diff);
							}
						}
						/* b6: servo PPS VOLT toward 2*vbat + target
						 * even when diff < 0.  The ADSP settle is
						 * its own closed loop and at higher SOC it
						 * can sit UNDER 2*vbat (measured -470 mV on
						 * the b5 boot); b5 only servoed after a
						 * -EAGAIN engage attempt, which can never
						 * happen with diff < 0, so the engine never
						 * even tried to reach the window.  Keep
						 * walking toward the target (ceiling
						 * pps_cp_max_volt_mv, floor
						 * pps_cp_min_volt_mv); the engage gate
						 * above fires the chip the moment the window
						 * appears. */
						if (p->charg_ticks > 3) {
							unsigned int nv;
							int target = pps_cp_target_mv;

							if (diff < target) {
								nv = min(p->set_volt_mv +
								     pps_cp_vstep_mv,
								     pps_cp_max_volt_mv);
								if (nv > p->set_volt_mv &&
								    !pps_usb_set(PPS_PROP_USB_SET_PPS_VOLT, nv))
								    p->set_volt_mv = nv;
							} else if (diff > target + (int)pps_cp_vstep_mv) {
								nv = p->set_volt_mv - pps_cp_vstep_mv;
								if (nv >= pps_cp_min_volt_mv &&
								    nv < p->set_volt_mv &&
								    !pps_usb_set(PPS_PROP_USB_SET_PPS_VOLT, nv))
								    p->set_volt_mv = nv;
							}
						}
					}
				}
			}
		}
		p->charg_ticks++;
		if (p->charg_ticks < 60)	/* ~first 2 minutes */
			pr_info("erhai_pps: %s vbus %d mV vbat %d mV ibat %d mA soc %d%% t %dC curr %u mA\n",
				charging ? "charging" : "taper",
				p->vbus_mv, p->vbat_mv, p->ibat_ma, p->soc,
				p->temp_c, p->set_curr_ma);
		return;
	}
	default:
		return;
	}
}

static void pps_ws(struct work_struct *work)
{
	struct erhai_pps *p = container_of(to_delayed_work(work),
					   struct erhai_pps, work);

	mutex_lock(&pps_lock);
	pps_poll(p);
	mutex_unlock(&pps_lock);
	/* while the charge pump is engaged, poll at the REG_09 watchdog feed
	 * interval (must stay under the SC8547A 1000 ms window); otherwise the
	 * normal telemetry cadence is fine.  A dropped poll = a watchdog expiry
	 * self-clears REG_07, so this can only slow the CP down, never strand it. */
	schedule_delayed_work(&p->work,
			      msecs_to_jiffies(pps_cp_is_engaged() ?
					      pps_cp_feed_ms : pps_poll_ms));
}


/* ---- debugfs ---- */
static int pps_state_show(struct seq_file *s, void *v)
{
	struct erhai_pps *p = s->private;

	seq_printf(s, "pps_enable %d\nstate %s\nlast_exit %s\n",
		   pps_enable, pps_state_names[p->state],
		   p->last_reason ?: "-");
	seq_printf(s, "set volt %u mV curr %u mA target %u mA\n",
		   p->set_volt_mv, p->set_curr_ma, p->target_curr_ma);
	if (p->state > PPS_IDLE)
		seq_printf(s, "read vbus %d mV vbat %d mV ibat %d mA soc %d%% t %d C\n",
			   p->vbus_mv, p->vbat_mv, p->ibat_ma, p->soc,
			   p->temp_c);
	seq_printf(s, "sessions %llu entries %u exits %u cp %s (eng %u drop %u)\n",
		   p->sessions, p->entries, p->exits,
		   pps_cp_is_engaged() ? "on" : "off",
		   p->cp_engages, p->cp_drops);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(pps_state);

static ssize_t pps_cmd_write(struct file *f, const char __user *ubuf,
			     size_t count, loff_t *off)
{
	struct erhai_pps *p = f->private_data;
	char buf[32];
	ssize_t n;

	n = simple_write_to_buffer(buf, sizeof(buf) - 1, off, ubuf, count);
	if (n < 0)
		return n;
	buf[n] = '\0';

	mutex_lock(&pps_lock);
	if (strncmp(buf, "start", 5) == 0) {
		if (p->state == PPS_IDLE)
			pps_enter(p, PPS_ARMED, "manual");
	} else if (strncmp(buf, "stop", 4) == 0) {
		pps_exit(p, PPS_EXIT_REASON_FORCED);
	}
	mutex_unlock(&pps_lock);
	return n;
}

static const struct file_operations pps_cmd_fops = {
	.open	= simple_open,
	.write	= pps_cmd_write,
	.llseek	= noop_llseek,
};

static int __init erhai_pps_init(void)
{
	struct erhai_pps *p;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->state = PPS_IDLE;
	INIT_DELAYED_WORK(&p->work, pps_ws);
	g_pps = p;

	p->debugfs = debugfs_create_dir("erhai_pps", NULL);
	debugfs_create_file("state", 0444, p->debugfs, p, &pps_state_fops);
	debugfs_create_file("cmd", 0200, p->debugfs, p, &pps_cmd_fops);
	/* the engine polls even before the ADSP is up (returns cleanly) */
	schedule_delayed_work(&p->work, msecs_to_jiffies(5000));

	pr_info("erhai_pps: PPS host engine loaded (poll %u ms, target %u mV/%u mA, FCC %u mA, soc<=%u%%)\n",
		pps_poll_ms, pps_target_volt_mv, pps_target_curr_ma, pps_fcc_ma,
		pps_soc_stop);
	return 0;
}

static void __exit erhai_pps_exit(void)
{
	struct erhai_pps *p = g_pps;

	if (!p)
		return;
	mutex_lock(&pps_lock);
	cancel_delayed_work_sync(&p->work);
	if (p->state != PPS_IDLE)
		pps_revert(p);
	mutex_unlock(&pps_lock);
	debugfs_remove_recursive(p->debugfs);
	kfree(p);
	g_pps = NULL;
}

module_init(erhai_pps_init);
module_exit(erhai_pps_exit);

MODULE_DESCRIPTION("erhai (OnePlus Pad 3) ADSP PPS fast-charge host engine");
MODULE_LICENSE("GPL");

