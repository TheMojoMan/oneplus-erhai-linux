// SPDX-License-Identifier: GPL-2.0-only
/*
 * erhai (OnePlus Pad 3 / SM8750) USB-OTG arming over pmic_glink.
 *
 * On erhai the USB-OTG VBUS boost is owned by the ADSP charger/PD firmware
 * (DT: qcom,otg_boost_src=0 => PMIC-internal boost driven by the ADSP). There
 * is no AP-side regulator to enable. OxygenOS arms OTG by sending proprietary
 * OnePlus battery_charger "set property" commands over pmic_glink to the ADSP;
 * mainline qcom_battmgr does not implement this, so nothing arms OTG and VBUS
 * never sources.
 *
 * This auxiliary driver binds the "pmic_glink.power-supply" aux device,
 * registers as the battmgr glink owner, and once the charger_pd protection
 * domain is up sends:
 *     USB_OTG_AP_ENABLE = 1   (AP is present and will drive the boost)
 *     USB_OTG_SWITCH    = 1   (arm OTG; scheme=0 => straight to the ADSP)
 * Arming alone is not enough: when a sink is plugged the ADSP detects CC and
 * sends the AP a BC_OTG_ENABLE notification, and the AP must answer with
 * USB_OTG_VBUS_REGULATOR_ENABLE=1 to actually turn the PMIC boost on (and =0 on
 * BC_OTG_DISABLE). This driver performs that round-trip in erhai_otg_callback,
 * matching the vendor charger driver (otg_notification_handler ->
 * smbchg_otg_enable). It also flips the dwc3 data-role to host and raises the
 * boost current limit in the vendor's order.
 *
 * The usb_property_id values are from the V2 charger framework
 * (oplus_hal_adsp.h) the SM8750 ADSP actually runs; the older sm8450 enum is +3
 * lower from index 10 on, which pointed the OTG-enable write at a read-only
 * status property that the ADSP accepted and ignored.
 *
 * A debugfs knob (erhai_otg/cmd: "<property> <value> [opcode]") drives an
 * arbitrary property SET, used to raise the ADSP input current limit so the
 * tablet charges while acting as a data host on a powered hub.
 *
 * On gauge-absent SKUs (Qualcomm QBG battery gauge — e.g. DMI 24976 units
 * with no pack-side bq27z561 fitted) the same glink edge answers battery
 * property GETs; when the bq27z561 psy never shows up this driver also
 * registers ADSP-backed "erhai-adsp-bat"/"erhai-adsp-usb" power supplies,
 * so the desktop battery icon works on both hardware variants
 * (2026-09-10, see power_gauge-2026-09-10.md in the workspace).
 */

#include <linux/auxiliary_bus.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/usb/role.h>
#include <linux/workqueue.h>

#include <linux/soc/qcom/pdr.h>
#include <linux/soc/qcom/pmic_glink.h>

/* battery_charger glink opcodes */
#define BC_BATTERY_PROPERTY_GET		0x30	/* BC_BATTERY_STATUS_GET (req/resp) */
#define BC_BATTERY_PROPERTY_SET		0x31	/* BC_BATTERY_STATUS_SET (req/resp) */
#define BC_USB_PROPERTY_GET		0x32	/* BC_USB_STATUS_GET (req/resp) */
#define BC_USB_PROPERTY_SET		0x33	/* BC_USB_STATUS_SET (req/resp) */
#define BC_NOTIFY_IND			0x07	/* async ADSP -> AP notification */
#define BC_SET_NOTIFY_REQ		0x04	/* register AP for async notifications */

/* The vendor charger waits this long for each glink ack (battery_chg_write). */
#define BC_WAIT_TIME_MS			2500

/* notify_msg->notification values the ADSP sends when a sink is (un)plugged */
#define BC_OTG_ENABLE			0x50
#define BC_OTG_DISABLE			0x51

/* usb_property_id enum (V2 charger framework, oplus_hal_adsp.h). */
#define USB_OTG_AP_ENABLE		17
#define USB_OTG_SWITCH			18
#define USB_CHARGE_OUTPUT_SUSPEND	19
#define USB_OTG_VBUS_REGULATOR_ENABLE	24
#define USB_OTG_BOOST_CURRENT		64

/*
 * battery_property_id enum (battmgr family — the same ids upstream
 * qcom_battmgr sends; subset used by the ADSP battery psy below).
 */
#define BC_BATT_STATUS			0
#define BC_BATT_PRESENT			2
#define BC_BATT_CAPACITY		4
#define BC_BATT_VOLT_NOW		7
#define BC_BATT_CURR_NOW		9
#define BC_BATT_TEMP			12
#define BC_USB_ONLINE			0

/* battery notification values (BC_NOTIFY_IND payload). */
#define BC_NOTIF_BAT_PROPERTY		0x30
#define BC_NOTIF_BAT_STATUS		0x80
#define BC_NOTIF_BAT_CHARGING_STATE	0x83

/* Charge-limit (max-SoC) policy. */
#define ERHAI_CHGLIM_DEFAULT_MAXSOC	80
#define ERHAI_CHGLIM_HYST		3	/* resume this many %% below the cap */
#define ERHAI_CHGLIM_POLL_MS		10000

/*
 * Boost current limit the vendor sends right before the regulator-enable (V2
 * oplus_chg_8350_set_otg_boost_curr_limit); the boost voltage is configured
 * inside the ADSP.
 */
#define OTG_BOOST_CURR_MA		1250

/*
 * dwc3 device->host DRD transition (dwc3_host_init, async via drd_work) hangs
 * if it runs while the PMIC VBUS boost is already sourcing. So on BC_OTG_ENABLE
 * flip to host and wait for host_init/xhci to settle BEFORE enabling VBUS.
 */
#define OTG_HOST_SETTLE_MS		2500

/*
 * Debounce the ADSP's reflexive BC_OTG_DISABLE: a just-VBUS-powered device
 * often fails its first descriptor read (-71) before its HS PHY settles, and
 * the ADSP reacts with a DISABLE then re-detects CC and re-ENABLEs ~0.5s later.
 * That VBUS "blink" tears xhci down each cycle so the retries never converge.
 * Delay acting on a DISABLE; an ENABLE within the window cancels the teardown.
 */
#define OTG_DISABLE_DEBOUNCE_MS		1500

/*
 * Self-heal delay after a detach-edge disarm. The drd-sync disarm kills
 * the ADSP session, and CC attach detection dies with it. Userspace
 * (erhai-otg-rearm.service) re-arms on the detach edge, but that can lose a
 * race when the host-exit is slow (2026-08-28: userspace re-arm at T+3 s,
 * dwc3 host-exit drd sync disarm at T+4 s ate it -- every later plug was
 * invisible until a manual debugfs re-arm). Re-arm once after this delay if
 * the session is still down; an attach in the meantime sets armed=true and
 * makes it a no-op. 0 disables the self-heal.
 */
static int erhai_rearm_delay_ms = 2000;
module_param(erhai_rearm_delay_ms, int, 0644);
MODULE_PARM_DESC(erhai_rearm_delay_ms,
		 "re-arm the ADSP OTG session this long after a detach disarm (0=off)");

/*
 * Max-SoC charge limit. When 1..99, charging is inhibited at/above this SoC
 * (battery held, system runs from input via USB_CHARGE_OUTPUT_SUSPEND) and
 * resumed once SoC falls ERHAI_CHGLIM_HYST below it. 0 or 100 disables the
 * limit (charge to full). Uses the reliable bq27z561 AP-I2C gauge.
 */
static int max_soc = ERHAI_CHGLIM_DEFAULT_MAXSOC;
module_param(max_soc, int, 0644);
MODULE_PARM_DESC(max_soc, "Stop charging at this SoC %% (bypass, system on input); 0/100 = charge to full (default 80)");

struct erhai_otg_set_msg {
	struct pmic_glink_hdr hdr;
	__le32 battery;
	__le32 property;
	__le32 value;
};

/* battery_charger_notify_msg: hdr + a single notification code. */
struct erhai_otg_notify_msg {
	struct pmic_glink_hdr hdr;
	__le32 notification;
};

/* battery_charger_resp_msg: the ADSP reply to a property GET/SET. */
struct erhai_otg_resp_msg {
	struct pmic_glink_hdr hdr;
	__le32 property;
	__le32 value;
	__le32 ret_code;
};

/* battery_charger_set_notify_msg: registers the AP for async ADSP notifications. */
struct erhai_otg_notify_req_msg {
	struct pmic_glink_hdr hdr;
	__le32 battery_id;
	__le32 power_state;
	__le32 low_capacity;
	__le32 high_capacity;
};

struct erhai_otg {
	struct device *dev;
	struct pmic_glink_client *client;

	struct mutex lock;		/* serialize requests (fw dislikes concurrency) */
	struct completion ack;
	int error;

	bool service_up;
	struct work_struct arm_work;

	/*
	 * VBUS boost round-trip: the ADSP asks us to (dis)enable the PMIC boost
	 * via BC_OTG_ENABLE/DISABLE; we answer with USB_OTG_VBUS_REGULATOR_ENABLE.
	 * vbus_work brings the session up; vbus_off_work tears it down, delayed by
	 * OTG_DISABLE_DEBOUNCE_MS so a blink (DISABLE->ENABLE) is absorbed.
	 * vbus_lock serializes the two; vbus_on makes a repeat ENABLE idempotent.
	 */
	struct mutex vbus_lock;
	struct work_struct vbus_work;
	struct delayed_work vbus_off_work;
	/*
	 * Detach-edge self-heal: re-arm the ADSP session a short while after a
	 * disarm so a lost re-arm race (or no userspace re-arm at all) cannot
	 * leave CC attach detection dead. See erhai_otg_rearm_work().
	 */
	struct delayed_work rearm_work;
	bool vbus_on;
	bool armed;

	/* dwc3 USB data-role switch, looked up lazily by the dwc3 fwnode. */
	struct usb_role_switch *role_sw;

	/* Max-SoC charge limit (see max_soc module param). */
	struct delayed_work chglim_work;
	int chglim_state;		/* -1 unknown, 0 charging allowed, 1 inhibited */

	/*
	 * ADSP battery psy (gauge-absent QBG SKUs): property GET replies are
	 * decoded by the callback into resp_property/resp_value; the ack and
	 * otg->lock are shared with the OTG property SETs.
	 */
	struct power_supply *bat_psy;
	struct power_supply *usb_psy;
	bool psy_registered;
	u32 resp_property;
	u32 resp_value;

	struct dentry *debugfs;
};

/*
 * Send a property SET to the ADSP. opcode selects the target psy:
 * BC_USB_PROPERTY_SET (USB) or BC_BATTERY_PROPERTY_SET (battery). The 24-byte
 * request layout and the response are identical for both.
 */
static int erhai_otg_set_op(struct erhai_otg *otg, u32 opcode, u32 property,
			    u32 value)
{
	struct erhai_otg_set_msg msg = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(opcode),
		.battery = cpu_to_le32(0),
		.property = cpu_to_le32(property),
		.value = cpu_to_le32(value),
	};
	unsigned long left;
	int ret;

	mutex_lock(&otg->lock);

	if (!otg->service_up) {
		mutex_unlock(&otg->lock);
		return -ENODEV;
	}

	reinit_completion(&otg->ack);
	otg->error = 0;

	ret = pmic_glink_send(otg->client, &msg, sizeof(msg));
	if (ret < 0) {
		mutex_unlock(&otg->lock);
		return ret;
	}

	/*
	 * The ADSP replies with a ret_code decoded by the callback into
	 * otg->error (0 = accepted, -EINVAL = rejected). Distinguish a total
	 * non-response as -ETIMEDOUT.
	 */
	left = wait_for_completion_timeout(&otg->ack,
					   msecs_to_jiffies(BC_WAIT_TIME_MS));
	ret = left ? otg->error : -ETIMEDOUT;

	mutex_unlock(&otg->lock);

	if (!left)
		dev_warn(otg->dev, "no ack for op %#x property %u=%u (timed out)\n",
			 opcode, property, value);
	return ret;
}

static int erhai_otg_set(struct erhai_otg *otg, u32 property, u32 value)
{
	return erhai_otg_set_op(otg, BC_USB_PROPERTY_SET, property, value);
}

/* Read SoC (%) from the reliable bq27z561 AP-I2C gauge, or negative errno. */
static int erhai_otg_gauge_soc(void)
{
	struct power_supply *psy = power_supply_get_by_name("bq27z561-0");
	union power_supply_propval v;
	int ret;

	if (!psy)
		return -ENODEV;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &v);
	power_supply_put(psy);
	return ret ? ret : v.intval;
}

/*
 * Read a battery_charger property from the ADSP (req/resp; the same 24-byte
 * request layout as a SET — see erhai_otg_set_msg). The reply echoes the
 * property and carries its value plus a ret_code, decoded by the callback
 * into otg->error / otg->resp_property / otg->resp_value. Used by the
 * ADSP-gauge battery reads on gauge-absent SKUs. Returns the raw value, or
 * a negative errno. Serialized by otg->lock like every other ADSP
 * round-trip (the firmware stops responding to concurrency).
 */
static int erhai_otg_get_op(struct erhai_otg *otg, u32 opcode, u32 property)
{
	struct erhai_otg_set_msg msg = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(opcode),
		.battery = cpu_to_le32(0),
		.property = cpu_to_le32(property),
		.value = cpu_to_le32(0),
	};
	unsigned long left;
	int ret;

	mutex_lock(&otg->lock);

	if (!otg->service_up) {
		mutex_unlock(&otg->lock);
		return -ENODEV;
	}

	reinit_completion(&otg->ack);
	otg->error = 0;
	otg->resp_property = (u32)-1;
	otg->resp_value = 0;

	ret = pmic_glink_send(otg->client, &msg, sizeof(msg));
	if (ret < 0) {
		mutex_unlock(&otg->lock);
		return ret;
	}

	left = wait_for_completion_timeout(&otg->ack,
					   msecs_to_jiffies(BC_WAIT_TIME_MS));
	if (!left) {
		mutex_unlock(&otg->lock);
		dev_warn(otg->dev,
			 "no reply for GET %#x property %u (timed out)\n",
			 opcode, property);
		return -ETIMEDOUT;
	}

	if (otg->error)
		ret = otg->error;
	else if (otg->resp_property == property)
		ret = (int)otg->resp_value;
	else
		ret = -ENODATA;

	mutex_unlock(&otg->lock);
	return ret;
}

/* SoC percent (raw) from the ADSP gauge (battmgr BATT_CAPACITY GET). */
static int erhai_otg_adsp_soc(struct erhai_otg *otg)
{
	return erhai_otg_get_op(otg, BC_BATTERY_PROPERTY_GET, BC_BATT_CAPACITY);
}

/*
 * Normalize the ADSP SoC to percent. The sm8350/sm8550 battmgr family
 * reports BATT_CAPACITY in centi-percent (value / 100 == %), but the
 * vendor ADSP on erhai has only been observed live on pack-gauge units;
 * guard both encodings instead of trusting the scaling (2026-09-10).
 */
static int erhai_otg_soc_pct(int raw)
{
	if (raw < 0)
		return 0;
	if (raw > 100)
		return min(raw / 100, 100);
	return raw;
}

/*
 * Charge-limiter SoC source: the reliable AP-I2C bq27z561 gauge when
 * fitted (pack-gauge SKUs), else the ADSP gauge (QBG SKUs — units with
 * no pack chip; see power_gauge-2026-09-10.md).
 */
static int erhai_otg_chglimit_soc(struct erhai_otg *otg)
{
	int soc = erhai_otg_gauge_soc();

	if (soc == -ENODEV)
		soc = erhai_otg_soc_pct(erhai_otg_adsp_soc(otg));
	return soc;
}

/*
 * Max-SoC charge limit. Holds the pack at the max_soc ceiling by suspending the
 * charge output above it (system keeps running from input) and resuming below
 * the recharge point. Re-asserts the suspend every poll while above the cap: the
 * ADSP may clear it on its own charge events.
 */
static void erhai_otg_chglim_work(struct work_struct *work)
{
	struct erhai_otg *otg = container_of(to_delayed_work(work),
					     struct erhai_otg, chglim_work);
	int max = max_soc;
	int soc;

	if (max < 1 || max >= 100) {
		/* limit disabled -> release any standing inhibit */
		if (otg->chglim_state == 1) {
			if (otg->service_up)
				erhai_otg_set(otg, USB_CHARGE_OUTPUT_SUSPEND, 0);
			otg->chglim_state = 0;
		}
		goto out;
	}

	if (!otg->service_up)
		goto out;		/* ADSP not up yet; retry next poll */

	soc = erhai_otg_chglimit_soc(otg);
	if (soc < 0)
		goto out;		/* gauge/ADSP not ready; retry */

	if (soc >= max) {
		erhai_otg_set(otg, USB_CHARGE_OUTPUT_SUSPEND, 1);
		otg->chglim_state = 1;
	} else if (soc <= max - ERHAI_CHGLIM_HYST) {
		if (otg->chglim_state != 0) {
			erhai_otg_set(otg, USB_CHARGE_OUTPUT_SUSPEND, 0);
			otg->chglim_state = 0;
		}
	}
	/* in the [max-hyst, max) band: hold the current state */

out:
	schedule_delayed_work(&otg->chglim_work,
			      msecs_to_jiffies(ERHAI_CHGLIM_POLL_MS));
}

/*
 * The dwc3 core registers its USB-role switch as "a600000.usb-role-switch" with
 * the switch's class-device fwnode set to the dwc3 device fwnode. We have no DT
 * phandle to it (this is a pmic_glink aux device), so find the switch by the
 * dwc3 device's fwnode and cache it. Done lazily because dwc3 may register the
 * switch after this driver probes.
 *
 * usb_role_switch_find_by_fwnode() (matches the switch's registration fwnode),
 * NOT fwnode_usb_role_switch_get() -- the latter is the consumer lookup and
 * expects a "usb-role-switch" graph link the dwc3 device does not have.
 */
#define OTG_DWC3_DEV_NAME	"a600000.usb"

static struct usb_role_switch *erhai_otg_role_sw(struct erhai_otg *otg)
{
	struct usb_role_switch *sw;
	struct device *dwc3;

	if (otg->role_sw)
		return otg->role_sw;

	dwc3 = bus_find_device_by_name(&platform_bus_type, NULL, OTG_DWC3_DEV_NAME);
	if (!dwc3) {
		dev_warn(otg->dev, "OTG role-switch: dwc3 %s not found\n",
			 OTG_DWC3_DEV_NAME);
		return NULL;
	}

	sw = usb_role_switch_find_by_fwnode(dev_fwnode(dwc3));
	put_device(dwc3);
	if (!sw) {
		dev_warn(otg->dev, "OTG role-switch: switch for %s fwnode not found\n",
			 OTG_DWC3_DEV_NAME);
		return NULL;
	}

	otg->role_sw = sw;
	return sw;
}

/* Flip the dwc3 USB data-role directly in-kernel (same path as the sysfs write). */
static void erhai_otg_set_role(struct erhai_otg *otg, enum usb_role role)
{
	struct usb_role_switch *sw = erhai_otg_role_sw(otg);

	if (sw)
		usb_role_switch_set_role(sw, role);
}

extern bool erhai_dwc3_host_active(void);

static bool erhai_detach_gate_enable = true;
module_param(erhai_detach_gate_enable, bool, 0644);
MODULE_PARM_DESC(erhai_detach_gate_enable,
		 "ignore ADSP OTG detach while a UCSI-owned host session is live");
/*
 * Apply the requested VBUS-boost / data-role state. Idempotent: a repeat ENABLE
 * while a session is up (the ADSP blink) is a no-op that does NOT re-toggle VBUS
 * or re-incur the host-settle sleep, so the USB core can keep retrying
 * enumeration of a hub that is still settling. Runs in process context;
 * vbus_lock serializes the enable and the debounced disable.
 */
static void erhai_otg_vbus_apply(struct erhai_otg *otg, bool on)
{
	int ret;

	if (!on && erhai_detach_gate_enable && erhai_dwc3_host_active()) {
		/*
		 * Charger-session churn (sc8547a VOOC/BC1.2 renegotiation
		 * with a DP alt-mode partner) makes the ADSP withdraw
		 * BC_OTG_ENABLE while UCSI still owns a live host session.
		 * That is NOT a physical detach: the data-role flip below
		 * tears xhci down mid-enumeration (hub + card reader killed
		 * ~1 s after attach — boot5 storm log, 19:16). Suppress the
		 * whole sequence and leave vbus_on/armed as they are so the
		 * follow-up attach re-enable is an idempotent no-op. The DRD
		 * host-exit path disarms the session via
		 * erhai_otg_session_sync() when UCSI really exits.
		 */
		dev_info(otg->dev,
			 "otg: detach suppressed: host session live (ucsi owns role)\n");
		return;
	}

	mutex_lock(&otg->vbus_lock);
	if (on == otg->vbus_on) {
		mutex_unlock(&otg->vbus_lock);
		return;
	}

	/*
	 * On enable: bring the data-role to host FIRST (xhci up and ready), then
	 * mirror the vendor's order -- boost current limit, then regulator
	 * enable. On disable VBUS goes off first, then role back to device.
	 */
	if (on) {
		if (!otg->armed) {
			/*
			 * A DISABLE fully disarms the ADSP session (below);
			 * an ENABLE therefore re-arms it first so the ADSP's
			 * lane/VBUS state machine starts from a clean slate.
			 */
			ret = erhai_otg_set(otg, USB_OTG_AP_ENABLE, 1);
			if (ret)
				dev_err(otg->dev, "otg: AP_ENABLE failed %d\n", ret);
			ret = erhai_otg_set(otg, USB_OTG_SWITCH, 1);
			if (ret)
				dev_err(otg->dev, "otg: SWITCH failed %d\n", ret);
			otg->armed = (ret == 0);
			msleep(100);
			dev_info(otg->dev, "otg: session armed (attach)\n");
		}
		erhai_otg_set_role(otg, USB_ROLE_HOST);
		/* Let dwc3_host_init + xhci settle before VBUS sources. */
		msleep(OTG_HOST_SETTLE_MS);
		dev_info(otg->dev, "otg: data-role host settled, boosting\n");
		erhai_otg_set(otg, USB_OTG_BOOST_CURRENT, OTG_BOOST_CURR_MA);
	} else {
		dev_info(otg->dev, "otg: detach: VBUS off\n");
	}

	erhai_otg_set(otg, USB_OTG_VBUS_REGULATOR_ENABLE, on);

	if (!on) {
		msleep(500);
		dev_info(otg->dev, "otg: detach: data-role device\n");
		erhai_otg_set_role(otg, USB_ROLE_DEVICE);
		msleep(200);

		/* Fully disarm the ADSP OTG session until the next attach. */
		if (otg->armed) {
			ret = erhai_otg_set(otg, USB_OTG_SWITCH, 0);
			if (ret)
				dev_err(otg->dev, "otg: SWITCH off failed %d\n", ret);
			ret = erhai_otg_set(otg, USB_OTG_AP_ENABLE, 0);
			if (ret)
				dev_err(otg->dev, "otg: AP_ENABLE off failed %d\n", ret);
			otg->armed = false;
			dev_info(otg->dev, "otg: session disarmed (detach)\n");
		}
		/* Self-heal: revive CC attach detection (see erhai_rearm_delay_ms). */
		if (erhai_rearm_delay_ms > 0)
			mod_delayed_work(system_wq, &otg->rearm_work,
					 msecs_to_jiffies(erhai_rearm_delay_ms));
	}

	otg->vbus_on = on;
	mutex_unlock(&otg->vbus_lock);
}

static struct erhai_otg *erhai_otg_glink_dev;
static bool erhai_session_sync_enable = true;
module_param(erhai_session_sync_enable, bool, 0644);
MODULE_PARM_DESC(erhai_session_sync_enable,
		 "disarm/rearm the ADSP OTG session on DRD detach/attach");

extern int (*erhai_session_sync)(bool arm);

/*
 * Vendor-flow parity for UCSI-driven role changes: the ADSP OTG session is
 * disarmed per detach and re-armed per attach. Only touches the session
 * pair (SWITCH/AP_ENABLE) — never VBUS boost or the data role, which stay
 * owned by the ucsi/role-switch path.
 */
static int erhai_otg_session_sync(bool arm)
{
	struct erhai_otg *otg = erhai_otg_glink_dev;
	int ret = 0;

	if (!otg || !erhai_session_sync_enable)
		return 0;

	mutex_lock(&otg->vbus_lock);
	if (arm) {
		if (otg->armed) {
			mutex_unlock(&otg->vbus_lock);
			return 0;
		}
		ret = erhai_otg_set(otg, USB_OTG_AP_ENABLE, 1);
		if (!ret)
			ret = erhai_otg_set(otg, USB_OTG_SWITCH, 1);
		if (ret)
			erhai_otg_set(otg, USB_OTG_AP_ENABLE, 0);
		else
			otg->armed = true;
		dev_info(otg->dev, "otg: session %s (drd sync, ret=%d)\n",
			 ret ? "arm FAILED" : "armed", ret);
	} else {
		if (otg->armed) {
			ret = erhai_otg_set(otg, USB_OTG_SWITCH, 0);
			erhai_otg_set(otg, USB_OTG_AP_ENABLE, 0);
			otg->armed = false;
			dev_info(otg->dev,
				 "otg: session disarmed (drd sync, ret=%d)\n",
				 ret);
			/*
			 * Self-heal: this disarm can land on top of (and eat)
			 * a userspace detach-edge re-arm when the host-exit is
			 * slow, leaving CC attach detection dead. Re-arm once
			 * later if the session is still down.
			 */
			if (erhai_rearm_delay_ms > 0)
				mod_delayed_work(system_wq, &otg->rearm_work,
						 msecs_to_jiffies(erhai_rearm_delay_ms));
		}
	}
	mutex_unlock(&otg->vbus_lock);
	return ret;
}

/*
 * Detach-edge self-heal: runs erhai_rearm_delay_ms after the last disarm.
 * No-op when an attach already recovered the session (armed=true, or a live
 * host session exists), otherwise re-arms so the ADSP keeps seeing CC
 * attaches after a detach no matter which disarm path fired.
 */
static void erhai_otg_rearm_work(struct work_struct *work)
{
	struct erhai_otg *otg = container_of(to_delayed_work(work),
					     struct erhai_otg, rearm_work);

	if (!otg->service_up || erhai_rearm_delay_ms <= 0)
		return;
	if (erhai_detach_gate_enable && erhai_dwc3_host_active())
		return;	/* host session is back: attach recovered already */
	if (!otg->armed)
		erhai_otg_session_sync(true);
}

/* BC_OTG_ENABLE: bring the OTG session up (immediate). */
static void erhai_otg_vbus_work(struct work_struct *work)
{
	struct erhai_otg *otg = container_of(work, struct erhai_otg, vbus_work);
	erhai_otg_vbus_apply(otg, true);
}

/*
 * BC_OTG_DISABLE: tear the session down -- but only fires
 * OTG_DISABLE_DEBOUNCE_MS after the DISABLE, and is cancelled if an ENABLE
 * arrives first (the blink), so a transient ADSP disable does not drop VBUS out
 * from under a settling hub.
 */
static void erhai_otg_vbus_off_work(struct work_struct *work)
{
	struct erhai_otg *otg = container_of(to_delayed_work(work),
					     struct erhai_otg, vbus_off_work);

	erhai_otg_vbus_apply(otg, false);
}

static void erhai_otg_callback(const void *data, size_t len, void *priv)
{
	struct erhai_otg *otg = priv;
	const struct pmic_glink_hdr *hdr = data;
	u32 opcode;

	if (len < sizeof(*hdr))
		return;

	opcode = le32_to_cpu(hdr->opcode);

	/*
	 * Response to our property SET/GET. For a SET the ret_code tells us
	 * whether the property was accepted (0) or rejected (non-zero). The ADSP
	 * echoes the request opcode.
	 */
	if (opcode == BC_USB_PROPERTY_SET || opcode == BC_BATTERY_PROPERTY_SET ||
	    opcode == BC_USB_PROPERTY_GET || opcode == BC_BATTERY_PROPERTY_GET) {
		if (len >= sizeof(struct erhai_otg_resp_msg)) {
			const struct erhai_otg_resp_msg *r = data;

			otg->error = le32_to_cpu(r->ret_code) ? -EINVAL : 0;
			otg->resp_property = le32_to_cpu(r->property);
			otg->resp_value = le32_to_cpu(r->value);
		} else {
			otg->error = 0;
		}
		complete(&otg->ack);
		return;
	}

	/*
	 * Async notification from the ADSP. After OTG is armed, the ADSP asks the
	 * AP to turn the PMIC boost on (sink plugged) or off (unplugged); we must
	 * answer with USB_OTG_VBUS_REGULATOR_ENABLE or VBUS never sources.
	 *
	 * Debounce the blink: an ENABLE cancels any pending teardown; a DISABLE
	 * only schedules the teardown into the future so a quick ENABLE-after-
	 * DISABLE keeps VBUS + host up uninterrupted. cancel_delayed_work /
	 * mod_delayed_work are non-sleeping and safe in this atomic callback.
	 */
	if (opcode == BC_NOTIFY_IND &&
	    len >= sizeof(struct erhai_otg_notify_msg)) {
		const struct erhai_otg_notify_msg *n = data;
		u32 ev = le32_to_cpu(n->notification);

		if (ev == BC_OTG_ENABLE) {
			cancel_delayed_work(&otg->vbus_off_work);
			schedule_work(&otg->vbus_work);
		} else if (ev == BC_OTG_DISABLE) {
			mod_delayed_work(system_wq, &otg->vbus_off_work,
					 msecs_to_jiffies(OTG_DISABLE_DEBOUNCE_MS));
		} else if (otg->psy_registered &&
			   (ev == BC_NOTIF_BAT_PROPERTY ||
			    ev == BC_NOTIF_BAT_STATUS ||
			    ev == BC_NOTIF_BAT_CHARGING_STATE)) {
			/*
			 * ADSP battery push update (QBG SKU): force UPower
			 * to re-read the battery properties.
			 */
			power_supply_changed(otg->bat_psy);
		}
	}
}

/*
 * Register the AP to receive async ADSP notifications, mirroring the vendor
 * charger's battery_chg_notify_enable(). Fire-and-forget (type=NOTIFY).
 */
static void erhai_otg_notify_enable(struct erhai_otg *otg)
{
	struct erhai_otg_notify_req_msg msg = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_NOTIFY),
		.hdr.opcode = cpu_to_le32(BC_SET_NOTIFY_REQ),
	};

	pmic_glink_send(otg->client, &msg, sizeof(msg));
}

/*
 * ---- ADSP battery psy (gauge-absent QBG SKUs, 2026-09-10) ----
 *
 * This unit carries the pack-side bq27z561 and bq27xxx owns the battery.
 * The friend's QBG SKU (DMI 24976) has NO pack chip fitted — Android's
 * oplus-chg-qbg.dtsi reads SoC/voltage/current from the ADSP
 * (oplus_adsp_gauge on charger_pd). The same battery_charger glink edge
 * answers property GETs, so expose an ADSP-backed battery + USB psy ONLY
 * when the pack gauge never appeared. Value scaling follows upstream
 * qcom_battmgr sm8350 family semantics; unverified assumptions are noted
 * at each read — verify against Android on the first live boot.
 */

static int erhai_otg_bat_get_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval *val)
{
	struct erhai_otg *otg = power_supply_get_drvdata(psy);
	int ret;

	if (!otg->service_up)
		return -EAGAIN;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		return 0;
	case POWER_SUPPLY_PROP_STATUS:
		/* POWER_SUPPLY_STATUS_* space, passed raw by upstream too. */
		ret = erhai_otg_get_op(otg, BC_BATTERY_PROPERTY_GET,
				       BC_BATT_STATUS);
		if (ret < 0)
			return ret;
		val->intval = ret;
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = erhai_otg_adsp_soc(otg);
		if (ret < 0)
			return ret;
		val->intval = erhai_otg_soc_pct(ret);
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = erhai_otg_get_op(otg, BC_BATTERY_PROPERTY_GET,
				       BC_BATT_VOLT_NOW);
		if (ret < 0)
			return ret;
		val->intval = ret;	/* µV assumed (sm8350 family) */
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = erhai_otg_get_op(otg, BC_BATTERY_PROPERTY_GET,
				       BC_BATT_CURR_NOW);
		if (ret < 0)
			return ret;
		val->intval = ret;	/* µA assumed (sm8350 family) */
		return 0;
	case POWER_SUPPLY_PROP_TEMP:
		ret = erhai_otg_get_op(otg, BC_BATTERY_PROPERTY_GET,
				       BC_BATT_TEMP);
		if (ret < 0)
			return ret;
		/* sm8350 family: raw is centi-degrees; psy wants tenths. */
		val->intval = DIV_ROUND_CLOSEST(ret, 10);
		return 0;
	default:
		return -EINVAL;
	}
}

static const enum power_supply_property erhai_adsp_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

static const struct power_supply_desc erhai_adsp_bat_psy_desc = {
	.name		= "erhai-adsp-bat",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.properties	= erhai_adsp_bat_props,
	.num_properties	= ARRAY_SIZE(erhai_adsp_bat_props),
	.get_property	= erhai_otg_bat_get_property,
};

static int erhai_otg_usb_get_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval *val)
{
	struct erhai_otg *otg = power_supply_get_drvdata(psy);
	int ret;

	if (!otg->service_up)
		return -EAGAIN;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = erhai_otg_get_op(otg, BC_USB_PROPERTY_GET,
				       BC_USB_ONLINE);
		if (ret < 0)
			return ret;
		val->intval = ret;
		return 0;
	default:
		return -EINVAL;
	}
}

static const enum power_supply_property erhai_adsp_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc erhai_adsp_usb_psy_desc = {
	.name		= "erhai-adsp-usb",
	.type		= POWER_SUPPLY_TYPE_USB,
	.properties	= erhai_adsp_usb_props,
	.num_properties	= ARRAY_SIZE(erhai_adsp_usb_props),
	.get_property	= erhai_otg_usb_get_property,
};

/*
 * Runtime SKU arbitration: register the ADSP bat+usb psy only when the
 * pack-side bq27z561 gauge never appeared (QBG SKUs). Called from
 * arm_work on charger_pd PDR UP — the i2c gauge probes long before that.
 * If registration is ever lost against a slow gauge (duplicate battery
 * evidence in dmesg), move this behind a gauge-settle delay.
 */
static void erhai_otg_psy_init(struct erhai_otg *otg)
{
	struct power_supply_config cfg = { };

	if (otg->psy_registered)
		return;

	cfg.drv_data = otg;

	otg->bat_psy = devm_power_supply_register(otg->dev,
						  &erhai_adsp_bat_psy_desc,
						  &cfg);
	if (IS_ERR(otg->bat_psy)) {
		dev_warn(otg->dev, "ADSP battery psy registration failed %ld\n",
			 PTR_ERR(otg->bat_psy));
		otg->bat_psy = NULL;
		return;
	}

	otg->usb_psy = devm_power_supply_register(otg->dev,
						  &erhai_adsp_usb_psy_desc,
						  &cfg);
	if (IS_ERR(otg->usb_psy)) {
		dev_warn(otg->dev, "ADSP USB psy registration failed %ld\n",
			 PTR_ERR(otg->usb_psy));
		otg->usb_psy = NULL;
		return;
	}

	otg->psy_registered = true;
	dev_info(otg->dev,
		 "no bq27z561 pack gauge: ADSP battery/USB psy registered (QBG SKU)\n");
}

static void erhai_otg_arm_work(struct work_struct *work)
{
	struct erhai_otg *otg = container_of(work, struct erhai_otg, arm_work);
	struct power_supply *psy;

	/* SKU arbitration: ADSP bat+usb psy only when no pack gauge fitted. */
	psy = power_supply_get_by_name("bq27z561-0");
	if (psy) {
		power_supply_put(psy);
	} else {
		erhai_otg_psy_init(otg);
		if (!otg->psy_registered)
			dev_warn(otg->dev,
				 "no pack gauge and ADSP psy init failed\n");
	}

	/* Register for the BC_OTG_ENABLE/DISABLE notifications, then arm OTG. */
	erhai_otg_notify_enable(otg);
	erhai_otg_set(otg, USB_OTG_AP_ENABLE, 1);
	erhai_otg_set(otg, USB_OTG_SWITCH, 1);
	otg->armed = true;
	dev_info(otg->dev, "otg: session armed at boot\n");
}

/* Called from pmic_glink under spinlock/irqsave -- must not sleep. */
static void erhai_otg_pdr_notify(void *priv, int state)
{
	struct erhai_otg *otg = priv;

	if (state == SERVREG_SERVICE_STATE_UP) {
		otg->service_up = true;
		schedule_work(&otg->arm_work);
	} else {
		otg->service_up = false;
	}
}

/*
 * debugfs: write "<property> <value> [opcode]" to send an arbitrary property
 * SET. opcode (hex or decimal) defaults to BC_USB_PROPERTY_SET (0x33, USB psy);
 * pass 0x31 to target the BATTERY psy. Used by userspace to raise the ADSP input
 * current limit so the tablet charges while acting as a data host.
 */
static ssize_t erhai_otg_cmd_write(struct file *file, const char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	struct erhai_otg *otg = file->private_data;
	char buf[32];
	u32 property, value, opcode = BC_USB_PROPERTY_SET;
	int n, ret;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	n = sscanf(buf, "%u %u %i", &property, &value, &opcode);
	if (n < 2)
		return -EINVAL;

	ret = erhai_otg_set_op(otg, opcode, property, value);
	if (ret < 0)
		return ret;

	/*
	 * Keep the kernel's armed flag coherent with manual userspace
	 * re-arms/disarms, otherwise a later drd-sync disarm happily kills a
	 * session userspace just armed (2026-08-28 detach race).
	 */
	if (opcode == BC_USB_PROPERTY_SET && property == USB_OTG_SWITCH) {
		mutex_lock(&otg->vbus_lock);
		otg->armed = (value != 0);
		mutex_unlock(&otg->vbus_lock);
	}

	return count;
}

static const struct file_operations erhai_otg_cmd_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = erhai_otg_cmd_write,
	.llseek = noop_llseek,
};

static int erhai_otg_probe(struct auxiliary_device *adev,
			   const struct auxiliary_device_id *id)
{
	struct device *dev = &adev->dev;
	struct erhai_otg *otg;

	otg = devm_kzalloc(dev, sizeof(*otg), GFP_KERNEL);
	if (!otg)
		return -ENOMEM;

	otg->dev = dev;
	mutex_init(&otg->lock);
	mutex_init(&otg->vbus_lock);
	erhai_otg_glink_dev = otg;
	erhai_session_sync = erhai_otg_session_sync;
	init_completion(&otg->ack);
	INIT_WORK(&otg->arm_work, erhai_otg_arm_work);
	INIT_WORK(&otg->vbus_work, erhai_otg_vbus_work);
	INIT_DELAYED_WORK(&otg->vbus_off_work, erhai_otg_vbus_off_work);
	INIT_DELAYED_WORK(&otg->rearm_work, erhai_otg_rearm_work);
	INIT_DELAYED_WORK(&otg->chglim_work, erhai_otg_chglim_work);
	otg->chglim_state = -1;
	/* Start the charge-limit poller after a settle delay (ADSP PDR + gauge). */
	schedule_delayed_work(&otg->chglim_work, msecs_to_jiffies(15000));

	otg->debugfs = debugfs_create_dir("erhai_otg", NULL);
	debugfs_create_file("cmd", 0200, otg->debugfs, otg, &erhai_otg_cmd_fops);

	otg->client = devm_pmic_glink_client_alloc(dev, PMIC_GLINK_OWNER_BATTMGR,
						   erhai_otg_callback,
						   erhai_otg_pdr_notify,
						   otg);
	if (IS_ERR(otg->client)) {
		debugfs_remove_recursive(otg->debugfs);
		return PTR_ERR(otg->client);
	}

	auxiliary_set_drvdata(adev, otg);

	/* May immediately call back erhai_otg_pdr_notify() if PD is already up. */
	pmic_glink_client_register(otg->client);

	return 0;
}

static void erhai_otg_remove(struct auxiliary_device *adev)
{
	erhai_session_sync = NULL;
	erhai_otg_glink_dev = NULL;
	struct erhai_otg *otg = auxiliary_get_drvdata(adev);

	cancel_work_sync(&otg->arm_work);
	cancel_work_sync(&otg->vbus_work);
	cancel_delayed_work_sync(&otg->vbus_off_work);
	cancel_delayed_work_sync(&otg->rearm_work);
	cancel_delayed_work_sync(&otg->chglim_work);
	usb_role_switch_put(otg->role_sw);
	debugfs_remove_recursive(otg->debugfs);
}

static const struct auxiliary_device_id erhai_otg_id_table[] = {
	{ .name = "pmic_glink.power-supply", },
	{},
};
MODULE_DEVICE_TABLE(auxiliary, erhai_otg_id_table);

static struct auxiliary_driver erhai_otg_driver = {
	.name = "erhai_otg_glink",
	.probe = erhai_otg_probe,
	.remove = erhai_otg_remove,
	.id_table = erhai_otg_id_table,
};

module_auxiliary_driver(erhai_otg_driver);

MODULE_DESCRIPTION("erhai (OnePlus Pad 3) USB-OTG arming over pmic_glink");
MODULE_LICENSE("GPL");
