// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024 Linaro Ltd.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/pwrseq/provider.h>
#include <linux/string.h>
#include <linux/types.h>

struct pwrseq_qcom_wcn_pdata {
	const char *const *vregs;
	size_t num_vregs;
	unsigned int pwup_delay_ms;
	unsigned int gpio_enable_delay_ms;
	const struct pwrseq_target_data **targets;
	bool has_vddio; /* separate VDD IO regulator */
	int (*match)(struct pwrseq_device *pwrseq, struct device *dev);
};

/*
 * erhai/WCN7880 cold-boot-cal lever: the working Android cnss stack does NOT
 * hold WLAN_EN statically asserted from the first power-on. It asserts, then
 * de-asserts ~166ms later, then re-asserts for the real power-up (a clean
 * power-cycle of the WCN WLAN subsystem). Mainline pwrseq just asserts once.
 * Replicate the cnss power-cycle so the WCN internal power/PMM state machine
 * gets the same clean restart before cold-boot calibration. Default ON for the
 * full Android replay; prune via these params. Only fires on wcn7850 pdata.
 *
 * Timing is taken VERBATIM from the OxygenOS cnss boot log
 * (captures/cnss-qmi-oxygenos/cnss_ipc_boot_bringup.log):
 *   2.180755  cnss_select_pinctrl_state: Assert WLAN_EN GPIO (Value 1)
 *   2.346659  cnss_select_pinctrl_state: De-assert WLAN_EN GPIO (Value 0) -> ON  = 166ms
 *   5.409353  cnss_select_pinctrl_state: Assert WLAN_EN GPIO (Value 1)    -> OFF = 3062ms
 * The 3.06s OFF is what actually resets the WCN power/PMM domain before the
 * real bring-up; our prior 50ms guess was ~60x too short to be a real reset.
 */
static bool pwrseq_qcom_wcn_wlan_en_toggle = true;
module_param_named(wlan_en_toggle, pwrseq_qcom_wcn_wlan_en_toggle, bool, 0644);
MODULE_PARM_DESC(wlan_en_toggle,
	"WCN7850/7880: power-cycle WLAN_EN (assert/de-assert/re-assert) at power-on, matching cnss (default on)");

static unsigned int pwrseq_qcom_wcn_wlan_en_hold_ms = 166;
module_param_named(wlan_en_hold_ms, pwrseq_qcom_wcn_wlan_en_hold_ms, uint, 0644);
MODULE_PARM_DESC(wlan_en_hold_ms,
	"WLAN_EN initial-assert hold before the de-assert (ms, default 166 = cnss timing)");

static unsigned int pwrseq_qcom_wcn_wlan_en_gap_ms = 3062;
module_param_named(wlan_en_gap_ms, pwrseq_qcom_wcn_wlan_en_gap_ms, uint, 0644);
MODULE_PARM_DESC(wlan_en_gap_ms,
	"WLAN_EN de-assert (OFF) duration before the final re-assert (ms, default 3062 = cnss-measured)");

/*
 * erhai/WCN7880 cold-boot-cal lever: "something in our code keeps the client
 * awake." The cal RDDM is the WCN's on-die PMM refusing the l1ss_sleep grant at
 * cal init ("l1ss_sleep unvoted by client" -> wal_pm_wmac_assert_phy_off ->
 * NOC error -> RDDM). For the PMM to grant l1ss_sleep the WCN must be able to
 * GATE its reference clock (CLKREQ# handshake). The working Android cnss stack
 * makes NO Linux clk vote on the WCN refclk:
 *   cnss_get_clk_single: Failed to get optional clock rf_clk, err = -2
 * RF_CLK1 is left entirely to AOP/PDC, and the WCN gates it via CLKREQ# when
 * its PMM enters l1ss_sleep. Mainline pwrseq, by contrast, holds a static
 * clk_prepare_enable(RPMH_RF_CLK1) vote for the WHOLE WCN-up window -> the
 * refclk can never gate -> the PMM grant times out -> phy_off assert. Match
 * Android: on wcn7880, do NOT hold the RF_CLK1 vote (let AOP/CLKREQ# manage it).
 * Default OFF (Android-faithful, part of the full replay). Set rf_clk_hold=1 to
 * restore the mainline always-on vote if the WCN fails to clock up without it.
 */
static bool pwrseq_qcom_wcn_rf_clk_hold;
module_param_named(rf_clk_hold, pwrseq_qcom_wcn_rf_clk_hold, bool, 0644);
MODULE_PARM_DESC(rf_clk_hold,
	"WCN7880: hold a static RPMH_RF_CLK1 vote across the whole WCN-up window (mainline behaviour). Android/cnss makes no such vote; default off lets the WCN PMM gate its refclk for l1ss_sleep during cold-boot-cal. Set 1 to restore the always-on vote.");

/*
 * erhai/WCN7880 cold-boot-cal lever #2 (the rail analogue of rf_clk_hold).
 * cnss votes the 6 WCN rails ON, sends mode=CALIBRATION, then ~171 ms later
 * REMOVES every rail vote ("cnss_vreg_unvote_single: Removing vote", all six
 * are need_unvote:1), so for the WHOLE 8.4 s cal the rails are owned ONLY by
 * the WCN's own AOP/PDC. That hand-off is load-bearing: when the WCN PMM runs
 * the cal phy_off it asks its PDC to GATE the RF rail (vddrfa) as part of
 * l1ss_sleep; the PDC's low/off vote only takes effect if no Linux RPMh vote
 * max-aggregates against it. Mainline pwrseq instead votes the rails at probe
 * and HOLDS a static min-voltage floor for the whole session -> the PDC can
 * never drop the rail -> the PMM's "rail gated" precondition never holds ->
 * the H2S grant times out -> wal_pm_wmac_assert_phy_off -> RDDM ~330 ms into
 * cal. Match cnss: a short delay after the final WLAN_EN re-assert (by which
 * time ath12k has sent the AOP wlan_pdc votes that keep the rails up), drop
 * our SW min-voltage floor + load so the AOP/PDC alone drives the rails and
 * can gate them during phy_off. Default ON (part of the full Android replay).
 */
static bool pwrseq_qcom_wcn_unvote_rails = true;
module_param_named(unvote_rails, pwrseq_qcom_wcn_unvote_rails, bool, 0644);
MODULE_PARM_DESC(unvote_rails,
	"WCN7880: after the WCN is up, remove the SW min-voltage floor on the rails (cnss need_unvote) so the AOP/PDC can gate them during the cold-boot-cal phy_off. Default on. Set 0 to hold the mainline static floor.");

static unsigned int pwrseq_qcom_wcn_unvote_delay_ms = 1000;
module_param_named(unvote_delay_ms, pwrseq_qcom_wcn_unvote_delay_ms, uint, 0644);
MODULE_PARM_DESC(unvote_delay_ms,
	"WCN7880: delay after the final WLAN_EN re-assert before removing the rail floors (ms, default 1000 = after ath12k's AOP votes land, before the cal phy_off).");

/*
 * cnss drops all 6 PRIM rail votes after cal-start; replay that in full (default
 * on). The IO + digital-core rails (vddio/vddio1p2/vdddig) feed the PCIe link;
 * if our AOP wlan_pdc votes don't back them the floor-drop could collapse the
 * link (link-down handling is unimplemented) — but Android unvotes all 6 and the
 * link survives (the fw/AOP holds them), and the prior "subset only" default was
 * a safety guard set on an OLD base (pre full PDC/AOP parity), not a test result.
 * Set unvote_all=0 to fall back to the AOP-backed RF/analog subset
 * {vddaon,vddrfa1p2,vddrfa1p8} if the link drops.
 */
static bool pwrseq_qcom_wcn_unvote_all = true;
module_param_named(unvote_all, pwrseq_qcom_wcn_unvote_all, bool, 0644);
MODULE_PARM_DESC(unvote_all,
	"WCN7880: unvote ALL 6 rails (cnss-faithful full replay, default on); 0 = AOP-backed RF/analog subset only.");

static bool pwrseq_wcn7880_rail_unvotable(const char *name)
{
	if (pwrseq_qcom_wcn_unvote_all)
		return true;
	return !strcmp(name, "vddaon") ||
	       !strcmp(name, "vddrfa1p2") ||
	       !strcmp(name, "vddrfa1p8");
}

struct pwrseq_qcom_wcn_ctx {
	struct pwrseq_device *pwrseq;
	struct device_node *of_node;
	const struct pwrseq_qcom_wcn_pdata *pdata;
	struct regulator_bulk_data *regs;
	struct regulator *vddio;
	struct gpio_desc *bt_gpio;
	struct gpio_desc *wlan_gpio;
	struct gpio_desc *xo_clk_gpio;
	struct clk *clk;
	unsigned long last_gpio_enable_jf;
	struct device *dev;
	bool is_wcn7880; /* erhai opt-in (qcom,wcn7880-rf-voltages present) */
	struct delayed_work unvote_work; /* cnss-style post-cal rail unvote */
};

/* The wcn7880 per-rail voltage/load table (cnss-matched). */
struct pwrseq_qcom_wcn_vreg_cfg {
	const char *name;
	int min_uv;
	int max_uv;
	int load_ua;
};

static const struct pwrseq_qcom_wcn_vreg_cfg pwrseq_wcn7880_vreg_cfg[] = {
	{ "vddio",     1800000, 1800000, 30000 },
	{ "vddio1p2",  1200000, 1200000, 30000 },
	{ "vddaon",     876000, 1036000, 0 },
	{ "vdddig",     876000, 1000000, 0 },
	{ "vddrfa1p2", 1312000, 1340000, 0 },
	{ "vddrfa1p8", 1860000, 2000000, 0 },
};

static const struct pwrseq_qcom_wcn_vreg_cfg *
pwrseq_wcn7880_find_cfg(const char *supply)
{
	size_t j;

	for (j = 0; j < ARRAY_SIZE(pwrseq_wcn7880_vreg_cfg); j++)
		if (!strcmp(supply, pwrseq_wcn7880_vreg_cfg[j].name))
			return &pwrseq_wcn7880_vreg_cfg[j];
	return NULL;
}

/*
 * Apply the cnss per-rail min-voltage floor + load to every WCN rail. Used at
 * probe and re-applied at each power-up so a prior unvote (floor dropped to 0)
 * can't leave the rails under-volted on a re-probe.
 */
static void pwrseq_wcn7880_apply_floors(struct pwrseq_qcom_wcn_ctx *ctx)
{
	int i, ret;

	for (i = 0; i < ctx->pdata->num_vregs; i++) {
		const struct pwrseq_qcom_wcn_vreg_cfg *cfg =
			pwrseq_wcn7880_find_cfg(ctx->regs[i].supply);

		if (!cfg)
			continue;

		ret = regulator_set_voltage(ctx->regs[i].consumer,
					    cfg->min_uv, cfg->max_uv);
		if (ret)
			dev_warn(ctx->dev, "wcn7880: set_voltage %s %d-%d failed %d\n",
				 cfg->name, cfg->min_uv, cfg->max_uv, ret);
		if (cfg->load_ua) {
			ret = regulator_set_load(ctx->regs[i].consumer,
						 cfg->load_ua);
			if (ret < 0)
				dev_warn(ctx->dev, "wcn7880: set_load %s %d failed %d\n",
					 cfg->name, cfg->load_ua, ret);
		}
	}
}

/*
 * cnss need_unvote: once the WCN is up and ath12k has installed the AOP
 * wlan_pdc votes (which hold the rails), remove our SW min-voltage floor +
 * load so the AOP/PDC alone owns the rails and can GATE them when the WCN PMM
 * runs the cold-boot-cal phy_off. A held floor max-aggregates against the
 * PDC's sleep vote -> rail never drops -> H2S grant withheld -> RDDM.
 */
static void pwrseq_qcom_wcn_unvote_work(struct work_struct *work)
{
	struct pwrseq_qcom_wcn_ctx *ctx = container_of(to_delayed_work(work),
				struct pwrseq_qcom_wcn_ctx, unvote_work);
	int i, ret;

	for (i = 0; i < ctx->pdata->num_vregs; i++) {
		const struct pwrseq_qcom_wcn_vreg_cfg *cfg =
			pwrseq_wcn7880_find_cfg(ctx->regs[i].supply);

		if (!cfg || !pwrseq_wcn7880_rail_unvotable(cfg->name))
			continue;

		ret = regulator_set_voltage(ctx->regs[i].consumer, 0, cfg->max_uv);
		if (ret)
			dev_warn(ctx->dev, "wcn7880: unvote set_voltage %s failed %d\n",
				 cfg->name, ret);
		regulator_set_load(ctx->regs[i].consumer, 0);
		dev_info(ctx->dev, "wcn7880: unvoted %s floor (AOP/PDC now owns it)\n",
			 cfg->name);
	}
	dev_info(ctx->dev,
		 "WCN7880: removed SW rail floors (cnss need_unvote) -> AOP/PDC owns rails, can gate for cal phy_off\n");
}

static void pwrseq_qcom_wcn_ensure_gpio_delay(struct pwrseq_qcom_wcn_ctx *ctx)
{
	unsigned long diff_jiffies;
	unsigned int diff_msecs;

	if (!ctx->pdata->gpio_enable_delay_ms)
		return;

	diff_jiffies = jiffies - ctx->last_gpio_enable_jf;
	diff_msecs = jiffies_to_msecs(diff_jiffies);

	if (diff_msecs < ctx->pdata->gpio_enable_delay_ms)
		msleep(ctx->pdata->gpio_enable_delay_ms - diff_msecs);
}

static int pwrseq_qcom_wcn_vddio_enable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	return regulator_enable(ctx->vddio);
}

static int pwrseq_qcom_wcn_vddio_disable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	return regulator_disable(ctx->vddio);
}

static const struct pwrseq_unit_data pwrseq_qcom_wcn_vddio_unit_data = {
	.name = "vddio-enable",
	.enable = pwrseq_qcom_wcn_vddio_enable,
	.disable = pwrseq_qcom_wcn_vddio_disable,
};

static int pwrseq_qcom_wcn_vregs_enable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	/*
	 * Restore the cnss min-voltage floors before each power-up so a prior
	 * post-cal unvote (floor dropped to 0) can't leave the WCN under-volted
	 * on a re-probe.
	 */
	if (ctx->is_wcn7880) {
		cancel_delayed_work_sync(&ctx->unvote_work);
		pwrseq_wcn7880_apply_floors(ctx);
	}

	return regulator_bulk_enable(ctx->pdata->num_vregs, ctx->regs);
}

static int pwrseq_qcom_wcn_vregs_disable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	return regulator_bulk_disable(ctx->pdata->num_vregs, ctx->regs);
}

static const struct pwrseq_unit_data pwrseq_qcom_wcn_vregs_unit_data = {
	.name = "regulators-enable",
	.enable = pwrseq_qcom_wcn_vregs_enable,
	.disable = pwrseq_qcom_wcn_vregs_disable,
};

static int pwrseq_qcom_wcn_clk_enable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	/*
	 * erhai/WCN7880: match cnss, which makes no Linux rf_clk vote and lets
	 * AOP/CLKREQ# gate RF_CLK1 so the WCN PMM can enter l1ss_sleep during
	 * cold-boot-cal. A held vote pins the refclk on -> PMM grant times out ->
	 * phy_off assert -> RDDM. Skip the static vote unless rf_clk_hold=1.
	 */
	if (ctx->is_wcn7880 && !pwrseq_qcom_wcn_rf_clk_hold) {
		dev_info(ctx->dev,
			 "WCN7880: not holding RPMH_RF_CLK1 (match cnss; AOP/CLKREQ# gates it)\n");
		return 0;
	}

	return clk_prepare_enable(ctx->clk);
}

static int pwrseq_qcom_wcn_clk_disable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	/* Balanced with clk_enable: skip unprepare if we never prepared. */
	if (ctx->is_wcn7880 && !pwrseq_qcom_wcn_rf_clk_hold)
		return 0;

	clk_disable_unprepare(ctx->clk);

	return 0;
}

static const struct pwrseq_unit_data pwrseq_qcom_wcn_clk_unit_data = {
	.name = "clock-enable",
	.enable = pwrseq_qcom_wcn_clk_enable,
	.disable = pwrseq_qcom_wcn_clk_disable,
};

static const struct pwrseq_unit_data *pwrseq_qcom_wcn3990_unit_deps[] = {
	&pwrseq_qcom_wcn_vddio_unit_data,
	&pwrseq_qcom_wcn_vregs_unit_data,
	NULL,
};

static const struct pwrseq_unit_data pwrseq_qcom_wcn3990_unit_data = {
	.name = "clock-enable",
	.deps = pwrseq_qcom_wcn3990_unit_deps,
	.enable = pwrseq_qcom_wcn_clk_enable,
	.disable = pwrseq_qcom_wcn_clk_disable,
};

static const struct pwrseq_unit_data *pwrseq_qcom_wcn_unit_deps[] = {
	&pwrseq_qcom_wcn_vregs_unit_data,
	&pwrseq_qcom_wcn_clk_unit_data,
	NULL
};

static int pwrseq_qcom_wcn6855_clk_assert(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	if (!ctx->xo_clk_gpio)
		return 0;

	msleep(1);

	gpiod_set_value_cansleep(ctx->xo_clk_gpio, 1);
	usleep_range(100, 200);

	return 0;
}

static const struct pwrseq_unit_data pwrseq_qcom_wcn6855_xo_clk_assert = {
	.name = "xo-clk-assert",
	.enable = pwrseq_qcom_wcn6855_clk_assert,
};

static const struct pwrseq_unit_data *pwrseq_qcom_wcn6855_unit_deps[] = {
	&pwrseq_qcom_wcn_vregs_unit_data,
	&pwrseq_qcom_wcn_clk_unit_data,
	&pwrseq_qcom_wcn6855_xo_clk_assert,
	NULL
};

static int pwrseq_qcom_wcn_bt_enable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	pwrseq_qcom_wcn_ensure_gpio_delay(ctx);
	gpiod_set_value_cansleep(ctx->bt_gpio, 1);
	ctx->last_gpio_enable_jf = jiffies;

	return 0;
}

static int pwrseq_qcom_wcn_bt_disable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	gpiod_set_value_cansleep(ctx->bt_gpio, 0);

	return 0;
}

static const struct pwrseq_unit_data pwrseq_qcom_wcn_bt_unit_data = {
	.name = "bluetooth-enable",
	.deps = pwrseq_qcom_wcn_unit_deps,
	.enable = pwrseq_qcom_wcn_bt_enable,
	.disable = pwrseq_qcom_wcn_bt_disable,
};

static const struct pwrseq_unit_data pwrseq_qcom_wcn6855_bt_unit_data = {
	.name = "bluetooth-enable",
	.deps = pwrseq_qcom_wcn6855_unit_deps,
	.enable = pwrseq_qcom_wcn_bt_enable,
	.disable = pwrseq_qcom_wcn_bt_disable,
};

static int pwrseq_qcom_wcn_wlan_enable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	pwrseq_qcom_wcn_ensure_gpio_delay(ctx);

	/* Replicate the cnss WLAN_EN power-cycle (see wlan_en_toggle above). */
	if (ctx->is_wcn7880 && pwrseq_qcom_wcn_wlan_en_toggle) {
		dev_info(ctx->dev, "wcn7880: WLAN_EN power-cycle (assert %ums -> de-assert %ums -> re-assert)\n",
			 pwrseq_qcom_wcn_wlan_en_hold_ms,
			 pwrseq_qcom_wcn_wlan_en_gap_ms);
		gpiod_set_value_cansleep(ctx->wlan_gpio, 1);
		msleep(pwrseq_qcom_wcn_wlan_en_hold_ms);
		gpiod_set_value_cansleep(ctx->wlan_gpio, 0);
		msleep(pwrseq_qcom_wcn_wlan_en_gap_ms);
	}

	gpiod_set_value_cansleep(ctx->wlan_gpio, 1);
	ctx->last_gpio_enable_jf = jiffies;

	/*
	 * cnss removes its rail votes ~171 ms after sending mode=CALIBRATION,
	 * i.e. ~860 ms after this final WLAN_EN re-assert. Schedule the same
	 * hand-off: by the time it fires ath12k has installed the AOP wlan_pdc
	 * votes (which keep the rails up), so dropping our SW floor lets the
	 * AOP/PDC gate the RF rail during the cal phy_off instead of us pinning
	 * it -> the H2S grant can resolve. (See unvote_rails above.)
	 */
	if (ctx->is_wcn7880 && pwrseq_qcom_wcn_unvote_rails)
		schedule_delayed_work(&ctx->unvote_work,
			msecs_to_jiffies(pwrseq_qcom_wcn_unvote_delay_ms));

	return 0;
}

static int pwrseq_qcom_wcn_wlan_disable(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	if (ctx->is_wcn7880)
		cancel_delayed_work_sync(&ctx->unvote_work);

	gpiod_set_value_cansleep(ctx->wlan_gpio, 0);

	return 0;
}

/*
 * erhai WCN7880 cal->mission cold power-cycle. cnss2 calls this at the
 * cold-boot-cal -> mission handoff (after it has brought the PCIe link down and
 * powered MHI off, before the mission power-up) to replicate downstream
 * Android's WLAN_EN toggle: de-assert WLAN_EN so the WCN power/PMM domain fully
 * resets, wait the same gap the boot cold-cycle uses, then re-assert. Without
 * this the WCN stays warm in MISSION EE across the MHI power-cycle, the mission
 * MHI power-up never sees an EE transition, the IPCR/qrtr channel is not
 * restarted and the WLFW QMI server never re-arrives -> cnss "Timeout waiting
 * for FW ready". Built-in (=y) so EXPORT_SYMBOL_GPL for the cnss2 module.
 */
static struct pwrseq_qcom_wcn_ctx *pwrseq_qcom_wcn_erhai_ctx;

void pwrseq_qcom_wcn_erhai_power_cycle(void)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_qcom_wcn_erhai_ctx;

	if (!ctx || !ctx->is_wcn7880 || !ctx->wlan_gpio)
		return;

	dev_info(ctx->dev,
		 "erhai: WLAN_EN cal->mission power-cycle (de-assert %u ms -> re-assert)\n",
		 pwrseq_qcom_wcn_wlan_en_gap_ms);

	cancel_delayed_work_sync(&ctx->unvote_work);
	gpiod_set_value_cansleep(ctx->wlan_gpio, 0);
	msleep(pwrseq_qcom_wcn_wlan_en_gap_ms);
	/*
	 * Restore the cnss rail min-voltage floors (the post-cal unvote work may
	 * have dropped them to 0) before the WCN powers back up for mission.
	 */
	pwrseq_wcn7880_apply_floors(ctx);
	gpiod_set_value_cansleep(ctx->wlan_gpio, 1);
	ctx->last_gpio_enable_jf = jiffies;
}
EXPORT_SYMBOL_GPL(pwrseq_qcom_wcn_erhai_power_cycle);

/*
 * Split WLAN_EN control for the faithful cnss_set_pci_link() port: the WCN
 * cold power-cycle is spread across cnss's own suspend_pci_link(DOWN) /
 * resume_pci_link(UP) pair so it brackets the PERST assert/deassert in
 * pcie-qcom (matches Android msm_pcie L2/L3 + WLAN_EN toggle). on=false:
 * de-assert WLAN_EN (rails collapse, chip drops to unpowered). on=true:
 * restore rail floors and re-assert WLAN_EN so the chip cold-boots from PBL.
 */
void pwrseq_qcom_wcn_erhai_wlan_en(bool on)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_qcom_wcn_erhai_ctx;

	if (!ctx || !ctx->is_wcn7880 || !ctx->wlan_gpio)
		return;

	if (on) {
		pwrseq_wcn7880_apply_floors(ctx);
		gpiod_set_value_cansleep(ctx->wlan_gpio, 1);
		ctx->last_gpio_enable_jf = jiffies;
		dev_info(ctx->dev, "erhai: WLAN_EN re-assert (cold mission boot)\n");
	} else {
		cancel_delayed_work_sync(&ctx->unvote_work);
		gpiod_set_value_cansleep(ctx->wlan_gpio, 0);
		dev_info(ctx->dev, "erhai: WLAN_EN de-assert (cal->mission link down)\n");
	}
}
EXPORT_SYMBOL_GPL(pwrseq_qcom_wcn_erhai_wlan_en);

static const struct pwrseq_unit_data pwrseq_qcom_wcn_wlan_unit_data = {
	.name = "wlan-enable",
	.deps = pwrseq_qcom_wcn_unit_deps,
	.enable = pwrseq_qcom_wcn_wlan_enable,
	.disable = pwrseq_qcom_wcn_wlan_disable,
};

static const struct pwrseq_unit_data pwrseq_qcom_wcn6855_wlan_unit_data = {
	.name = "wlan-enable",
	.deps = pwrseq_qcom_wcn6855_unit_deps,
	.enable = pwrseq_qcom_wcn_wlan_enable,
	.disable = pwrseq_qcom_wcn_wlan_disable,
};

static int pwrseq_qcom_wcn_pwup_delay(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	if (ctx->pdata->pwup_delay_ms)
		msleep(ctx->pdata->pwup_delay_ms);

	return 0;
}

static int pwrseq_qcom_wcn6855_xo_clk_deassert(struct pwrseq_device *pwrseq)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);

	if (ctx->xo_clk_gpio) {
		usleep_range(2000, 5000);
		gpiod_set_value_cansleep(ctx->xo_clk_gpio, 0);
	}

	return pwrseq_qcom_wcn_pwup_delay(pwrseq);
}

static const struct pwrseq_target_data pwrseq_qcom_wcn_bt_target_data = {
	.name = "bluetooth",
	.unit = &pwrseq_qcom_wcn_bt_unit_data,
	.post_enable = pwrseq_qcom_wcn_pwup_delay,
};

static const struct pwrseq_target_data pwrseq_qcom_wcn_wlan_target_data = {
	.name = "wlan",
	.unit = &pwrseq_qcom_wcn_wlan_unit_data,
	.post_enable = pwrseq_qcom_wcn_pwup_delay,
};

/* There are no separate BT and WLAN enablement pins */
static const struct pwrseq_target_data pwrseq_qcom_wcn3990_bt_target_data = {
	.name = "bluetooth",
	.unit = &pwrseq_qcom_wcn3990_unit_data,
};

static const struct pwrseq_target_data pwrseq_qcom_wcn3990_wlan_target_data = {
	.name = "wlan",
	.unit = &pwrseq_qcom_wcn3990_unit_data,
};

static const struct pwrseq_target_data pwrseq_qcom_wcn6855_bt_target_data = {
	.name = "bluetooth",
	.unit = &pwrseq_qcom_wcn6855_bt_unit_data,
	.post_enable = pwrseq_qcom_wcn6855_xo_clk_deassert,
};

static const struct pwrseq_target_data pwrseq_qcom_wcn6855_wlan_target_data = {
	.name = "wlan",
	.unit = &pwrseq_qcom_wcn6855_wlan_unit_data,
	.post_enable = pwrseq_qcom_wcn6855_xo_clk_deassert,
};

static const struct pwrseq_target_data *pwrseq_qcom_wcn_targets[] = {
	&pwrseq_qcom_wcn_bt_target_data,
	&pwrseq_qcom_wcn_wlan_target_data,
	NULL
};

static const struct pwrseq_target_data *pwrseq_qcom_wcn3990_targets[] = {
	&pwrseq_qcom_wcn3990_bt_target_data,
	&pwrseq_qcom_wcn3990_wlan_target_data,
	NULL
};

static const struct pwrseq_target_data *pwrseq_qcom_wcn6855_targets[] = {
	&pwrseq_qcom_wcn6855_bt_target_data,
	&pwrseq_qcom_wcn6855_wlan_target_data,
	NULL
};

static const char *const pwrseq_qca6390_vregs[] = {
	"vddio",
	"vddaon",
	"vddpmu",
	"vddrfa0p95",
	"vddrfa1p3",
	"vddrfa1p9",
	"vddpcie1p3",
	"vddpcie1p9",
};

static const struct pwrseq_qcom_wcn_pdata pwrseq_qca6390_of_data = {
	.vregs = pwrseq_qca6390_vregs,
	.num_vregs = ARRAY_SIZE(pwrseq_qca6390_vregs),
	.pwup_delay_ms = 60,
	.gpio_enable_delay_ms = 100,
	.targets = pwrseq_qcom_wcn_targets,
};

static const char *const pwrseq_wcn3990_vregs[] = {
	/* vddio is handled separately */
	"vddxo",
	"vddrf",
	"vddch0",
	"vddch1",
};

static int pwrseq_qcom_wcn3990_match(struct pwrseq_device *pwrseq,
				     struct device *dev);

static const struct pwrseq_qcom_wcn_pdata pwrseq_wcn3990_of_data = {
	.vregs = pwrseq_wcn3990_vregs,
	.num_vregs = ARRAY_SIZE(pwrseq_wcn3990_vregs),
	.pwup_delay_ms = 50,
	.targets = pwrseq_qcom_wcn3990_targets,
	.has_vddio = true,
	.match = pwrseq_qcom_wcn3990_match,
};

static const char *const pwrseq_wcn6750_vregs[] = {
	"vddaon",
	"vddasd",
	"vddpmu",
	"vddrfa0p8",
	"vddrfa1p2",
	"vddrfa1p7",
	"vddrfa2p2",
};

static const struct pwrseq_qcom_wcn_pdata pwrseq_wcn6750_of_data = {
	.vregs = pwrseq_wcn6750_vregs,
	.num_vregs = ARRAY_SIZE(pwrseq_wcn6750_vregs),
	.pwup_delay_ms = 50,
	.gpio_enable_delay_ms = 5,
	.targets = pwrseq_qcom_wcn_targets,
};

static const char *const pwrseq_wcn6855_vregs[] = {
	"vddio",
	"vddaon",
	"vddpmu",
	"vddpmumx",
	"vddpmucx",
	"vddrfa0p95",
	"vddrfa1p3",
	"vddrfa1p9",
	"vddpcie1p3",
	"vddpcie1p9",
};

static const struct pwrseq_qcom_wcn_pdata pwrseq_wcn6855_of_data = {
	.vregs = pwrseq_wcn6855_vregs,
	.num_vregs = ARRAY_SIZE(pwrseq_wcn6855_vregs),
	.pwup_delay_ms = 50,
	.gpio_enable_delay_ms = 5,
	.targets = pwrseq_qcom_wcn6855_targets,
};

static const char *const pwrseq_wcn7850_vregs[] = {
	"vdd",
	"vddio",
	"vddio1p2",
	"vddaon",
	"vdddig",
	"vddrfa1p2",
	"vddrfa1p8",
};

static const struct pwrseq_qcom_wcn_pdata pwrseq_wcn7850_of_data = {
	.vregs = pwrseq_wcn7850_vregs,
	.num_vregs = ARRAY_SIZE(pwrseq_wcn7850_vregs),
	.pwup_delay_ms = 50,
	.targets = pwrseq_qcom_wcn_targets,
};

/*
 * WCN7880 (OnePlus Pad 3 "peach") RF/AON rail voltages, taken byte-for-byte
 * from the OxygenOS cnss vreg table (captures/cnss-qmi-oxygenos). The mainline
 * pwrseq driver only ENABLES the shared rpmh rails (leaving them at their rpmh
 * floor); the WCN7880 RF-analog rails need higher voltages than the WCN7850 it
 * shares a compatible with -- rfa2 is 1.31-1.34V (not 1.2V), rfa1 1.86-2.0V,
 * aon >=0.876V. Left at the WCN7850 floor (rfa2 1.224V, aon 0.852V) the WCN7880
 * RF PHY fails cold-boot calibration: fw RDDM-asserts in wal_pm_wmac_assert_phy_off
 * (whal_wmac_phy_off_check / enable_chain_ldo_v2). cnss votes these per-rail
 * voltages+loads; we replicate that, but ONLY when the DT node opts in via
 * "qcom,wcn7880-rf-voltages" so real WCN7850 boards are unaffected.
 */
static int pwrseq_qcom_wcn_match_regulator(struct pwrseq_device *pwrseq,
					   struct device *dev,
					   const char *name)
{
	struct pwrseq_qcom_wcn_ctx *ctx = pwrseq_device_get_drvdata(pwrseq);
	struct device_node *dev_node = dev->of_node;

	/*
	 * The PMU supplies power to the Bluetooth and WLAN modules. both
	 * consume the PMU AON output so check the presence of the
	 * 'vddaon-supply' property and whether it leads us to the right
	 * device.
	 */
	if (!of_property_present(dev_node, name))
		return PWRSEQ_NO_MATCH;

	struct device_node *reg_node __free(device_node) =
			of_parse_phandle(dev_node, name, 0);
	if (!reg_node)
		return PWRSEQ_NO_MATCH;

	/*
	 * `reg_node` is the PMU AON regulator, its parent is the `regulators`
	 * node and finally its grandparent is the PMU device node that we're
	 * looking for.
	 */
	if (!reg_node->parent || !reg_node->parent->parent ||
	    reg_node->parent->parent != ctx->of_node)
		return PWRSEQ_NO_MATCH;

	return PWRSEQ_MATCH_OK;
}

static int pwrseq_qcom_wcn_match(struct pwrseq_device *pwrseq,
				 struct device *dev)
{
	return pwrseq_qcom_wcn_match_regulator(pwrseq, dev, "vddaon-supply");
}

static int pwrseq_qcom_wcn3990_match(struct pwrseq_device *pwrseq,
				     struct device *dev)
{
	int ret;

	/* BT device */
	ret = pwrseq_qcom_wcn_match_regulator(pwrseq, dev, "vddio-supply");
	if (ret == PWRSEQ_MATCH_OK)
		return ret;

	/* WiFi device match */
	return pwrseq_qcom_wcn_match_regulator(pwrseq, dev, "vdd-1.8-xo-supply");
}

static int pwrseq_qcom_wcn_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pwrseq_qcom_wcn_ctx *ctx;
	struct pwrseq_config config;
	int i, ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->of_node = dev->of_node;
	ctx->dev = dev;

	ctx->pdata = device_get_match_data(dev);
	if (!ctx->pdata)
		return dev_err_probe(dev, -ENODEV,
				     "Failed to obtain platform data\n");

	ctx->regs = devm_kcalloc(dev, ctx->pdata->num_vregs,
				 sizeof(*ctx->regs), GFP_KERNEL);
	if (!ctx->regs)
		return -ENOMEM;

	for (i = 0; i < ctx->pdata->num_vregs; i++)
		ctx->regs[i].supply = ctx->pdata->vregs[i];

	ret = devm_regulator_bulk_get(dev, ctx->pdata->num_vregs, ctx->regs);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Failed to get all regulators\n");

	/*
	 * WCN7880: vote the cnss per-rail voltages+loads on the shared rpmh
	 * rails (the mainline path only enables them, leaving the RF rails
	 * under-volted vs what the WCN7880 RF PHY needs for cold-boot cal).
	 * Opt-in via DT so real WCN7850 boards keep the enable-only behaviour.
	 */
	ctx->is_wcn7880 = of_property_read_bool(dev->of_node,
						"qcom,wcn7880-rf-voltages");
	if (ctx->is_wcn7880) {
		pwrseq_wcn7880_apply_floors(ctx);

		ret = devm_delayed_work_autocancel(dev, &ctx->unvote_work,
						   pwrseq_qcom_wcn_unvote_work);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to init the rail-unvote work\n");
	}

	if (ctx->pdata->has_vddio) {
		ctx->vddio = devm_regulator_get(dev, "vddio");
		if (IS_ERR(ctx->vddio))
			return dev_err_probe(dev, PTR_ERR(ctx->vddio), "Failed to get VDDIO\n");
	}

	ctx->bt_gpio = devm_gpiod_get_optional(dev, "bt-enable", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->bt_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->bt_gpio),
				     "Failed to get the Bluetooth enable GPIO\n");

	/*
	 * FIXME: This should actually be GPIOD_OUT_LOW, but doing so would
	 * cause the WLAN power to be toggled, resulting in PCIe link down.
	 * Since the PCIe controller driver is not handling link down currently,
	 * the device becomes unusable. So we need to keep this workaround until
	 * the link down handling is implemented in the controller driver.
	 */
	ctx->wlan_gpio = devm_gpiod_get_optional(dev, "wlan-enable",
						 GPIOD_ASIS);
	if (IS_ERR(ctx->wlan_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->wlan_gpio),
				     "Failed to get the WLAN enable GPIO\n");

	ctx->xo_clk_gpio = devm_gpiod_get_optional(dev, "xo-clk",
						   GPIOD_OUT_LOW);
	if (IS_ERR(ctx->xo_clk_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->xo_clk_gpio),
				     "Failed to get the XO_CLK GPIO\n");

	/*
	 * Set direction to output but keep the current value in order to not
	 * disable the WLAN module accidentally if it's already powered on.
	 */
	gpiod_direction_output(ctx->wlan_gpio,
			       gpiod_get_value_cansleep(ctx->wlan_gpio));

	ctx->clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(ctx->clk))
		return dev_err_probe(dev, PTR_ERR(ctx->clk),
				     "Failed to get the reference clock\n");

	memset(&config, 0, sizeof(config));

	config.parent = dev;
	config.owner = THIS_MODULE;
	config.drvdata = ctx;
	config.match = ctx->pdata->match ? : pwrseq_qcom_wcn_match;
	config.targets = ctx->pdata->targets;

	ctx->pwrseq = devm_pwrseq_device_register(dev, &config);
	if (IS_ERR(ctx->pwrseq))
		return dev_err_probe(dev, PTR_ERR(ctx->pwrseq),
				     "Failed to register the power sequencer\n");

	/* single WCN instance: publish ctx for the cnss2 cal->mission cold-cycle */
	if (ctx->is_wcn7880)
		pwrseq_qcom_wcn_erhai_ctx = ctx;

	return 0;
}

static const struct of_device_id pwrseq_qcom_wcn_of_match[] = {
	{
		.compatible = "qcom,wcn3950-pmu",
		.data = &pwrseq_wcn3990_of_data,
	},
	{
		.compatible = "qcom,wcn3988-pmu",
		.data = &pwrseq_wcn3990_of_data,
	},
	{
		.compatible = "qcom,wcn3990-pmu",
		.data = &pwrseq_wcn3990_of_data,
	},
	{
		.compatible = "qcom,wcn3991-pmu",
		.data = &pwrseq_wcn3990_of_data,
	},
	{
		.compatible = "qcom,wcn3998-pmu",
		.data = &pwrseq_wcn3990_of_data,
	},
	{
		.compatible = "qcom,qca6390-pmu",
		.data = &pwrseq_qca6390_of_data,
	},
	{
		.compatible = "qcom,wcn6855-pmu",
		.data = &pwrseq_wcn6855_of_data,
	},
	{
		.compatible = "qcom,wcn7850-pmu",
		.data = &pwrseq_wcn7850_of_data,
	},
	{
		.compatible = "qcom,wcn6750-pmu",
		.data = &pwrseq_wcn6750_of_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, pwrseq_qcom_wcn_of_match);

static struct platform_driver pwrseq_qcom_wcn_driver = {
	.driver = {
		.name = "pwrseq-qcom_wcn",
		.of_match_table = pwrseq_qcom_wcn_of_match,
	},
	.probe = pwrseq_qcom_wcn_probe,
};
module_platform_driver(pwrseq_qcom_wcn_driver);

MODULE_AUTHOR("Bartosz Golaszewski <bartosz.golaszewski@linaro.org>");
MODULE_DESCRIPTION("Qualcomm WCN PMU power sequencing driver");
MODULE_LICENSE("GPL");
