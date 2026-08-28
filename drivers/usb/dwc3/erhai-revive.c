// SPDX-License-Identifier: GPL-2.0
/*
 * erhai-revive.c - last-resort recovery for the SM8750 USB register island.
 *
 * #22 evidence: BCR pulse revives the register fabric but the command
 * engine stays dead (soft_reset -110); a GDSC collapse/restore cycle does
 * not restore it either; the clock framework believes every USB clock is
 * still enabled, so nothing re-gates the clock tree. A later full mode
 * transition heals the controller — consistent with the ADSP/AOP session
 * state being the out-of-band owner of the island.
 *
 * revive-v3 ladder (each step gated by a real functional probe):
 *   1. GCC_USB30_PRIM_BCR pulse              (fabric back)
 *   2. GDSC collapse/restore, if regs dead   (power island retry)
 *   3. glue clock re-cycle via dwc3-qcom     (CCF re-gates the tree)
 *   4. dwc3_core_soft_reset                  (functional gate)
 *   5. PHY power-cycle + re-init, if reset   (qusb2/qmp utmi/pipe clock)
 *      still times out; soft reset again
 * Verdict requires GHWPARAMS0 + GRXFIFOSIZ(0) alive AND soft reset == 0.
 */
#include <linux/bits.h>
#include <linux/sizes.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/phy/phy.h>

#include "core.h"
#include "io.h"

#define ERHAI_GCC_BASE		0x00100000
#define ERHAI_GCC_USB30_BCR	(ERHAI_GCC_BASE + 0x39000)

#define GDSCR_SW_COLLAPSE	BIT(0)
#define CFG_POWER_UP_COMPLETE	BIT(16)

static bool erhai_qmp_force;
module_param_named(qmp_force, erhai_qmp_force, bool, 0644);

static bool erhai_revive_enable = true;
module_param(erhai_revive_enable, bool, 0444);
MODULE_PARM_DESC(erhai_revive_enable,
		 "attempt recovery when the USB island reads dead");

static bool erhai_phy_reinit = true;
module_param(erhai_phy_reinit, bool, 0444);
MODULE_PARM_DESC(erhai_phy_reinit,
		 "power-cycle usb2/usb3 phys as last revive resort");

static bool erhai_glue_cycle;
module_param(erhai_glue_cycle, bool, 0644);
MODULE_PARM_DESC(erhai_glue_cycle,
		 "glue clock re-cycle in revive (BYPASSES the clock framework — "
		 "breaks shared QMP pipe-clk accounting, DP stops retraining)");

static unsigned int erhai_revive_retry_ms = 250;
module_param(erhai_revive_retry_ms, uint, 0644);
MODULE_PARM_DESC(erhai_revive_retry_ms, "ms between soft-reset retry polls");

static unsigned int erhai_revive_retries = 32;
module_param(erhai_revive_retries, uint, 0644);
MODULE_PARM_DESC(erhai_revive_retries,
		 "max soft-reset retry polls in revive (0 = single attempt)");

bool erhai_dwc3_revive(struct dwc3 *dwc);

static inline bool erhai_island_alive(struct dwc3 *dwc)
{
	return dwc3_readl(dwc, DWC3_GHWPARAMS0) &&
	       dwc3_readl(dwc, DWC3_GRXFIFOSIZ(0));
}

static void erhai_dwc3_gcc_dance_once(struct dwc3 *dwc, bool full)
{
	u32 __iomem *g;
	u32 __iomem *gdscr, *cfg;

	g = ioremap(ERHAI_GCC_USB30_BCR, SZ_64K);
	if (!g) {
		dev_err(dwc->dev, "erhai revive: ioremap failed\n");
		return;
	}
	gdscr = g + 1;
	cfg = g + 2;

	if (!full) {
		dev_info(dwc->dev,
			 "erhai revive: pulsing GCC_USB30_PRIM_BCR\n");
		writel(1, g);
		usleep_range(50000, 60000);
		writel(0, g);
		usleep_range(300000, 400000);
	} else {
		int i;

		dev_info(dwc->dev,
			 "erhai revive: cycling USB30_PRIM GDSC\n");
		writel(readl(gdscr) | GDSCR_SW_COLLAPSE, gdscr);
		usleep_range(300000, 400000);
		writel(readl(gdscr) & ~GDSCR_SW_COLLAPSE, gdscr);
		for (i = 0; i < 20; i++) {
			if (readl(cfg) & CFG_POWER_UP_COMPLETE)
				break;
			usleep_range(10000, 15000);
		}
		usleep_range(200000, 300000);
	}

	iounmap(g);
}

/* provided by phy-qcom-qmp-combo.c; bypasses stale phy-core counts */
int qmp_combo_force_recover(struct phy *phy);

static void erhai_dwc3_phy_cycle(struct dwc3 *dwc)
{
	struct phy *phys[2];
	int i, ret;

	phys[0] = dwc->usb2_generic_phy[0];
	phys[1] = dwc->usb3_generic_phy[0];

	/*
	 * v6: the USB3 (QMP combo) side is cycled through a dedicated
	 * force-recover call: the phy-core init_count is stale across the
	 * DP-only teardown, so the refcounted phy_exit/phy_init below
	 * silently no-oped and the dwc3 soft reset never completed (no
	 * pipe clock). The USB2 (M31 + eUSB2 repeater) side is unaffected
	 * and keeps the plain cycle.
	 */
	if (erhai_qmp_force) {
		ret = qmp_combo_force_recover(phys[1]);
		dev_info(dwc->dev, "erhai revive: qmp force recover ret=%d\n", ret);
	}

	if (!phys[0] || IS_ERR(phys[0]))
		return;
	phy_power_off(phys[0]);
	phy_exit(phys[0]);
	ret = phy_init(phys[0]);
	if (!ret)
		ret = phy_power_on(phys[0]);
	dev_info(dwc->dev, "erhai revive: phy0 reinit ret=%d\n", ret);
}

bool erhai_dwc3_revive(struct dwc3 *dwc)
{
	u32 v;
	int r;

	if (!erhai_revive_enable || !dwc->regs)
		return false;
	if (erhai_island_alive(dwc))
		return false;

	dev_err(dwc->dev, "erhai revive: island dead (GHWPARAMS0=%#x FIFO0=%#x)\n",
		dwc3_readl(dwc, DWC3_GHWPARAMS0),
		dwc3_readl(dwc, DWC3_GRXFIFOSIZ(0)));

	erhai_dwc3_gcc_dance_once(dwc, false);
	if (!erhai_island_alive(dwc))
		erhai_dwc3_gcc_dance_once(dwc, true);

	/*
	 * v4: the glue-clock step gated RAW GCC writes that bypass the clock
	 * framework and desynced the enable-count of
	 * gcc_usb3_prim_phy_pipe_clk — a clock the DP half of the shared QMP
	 * combo PHY also gates. The desync fired "pipe_clk already disabled"
	 * WARNINGS in qmp_combo_dp_exit and DP never re-trained afterwards
	 * (#24 boot log 19:52:56). Hook kept for experiments, default OFF.
	 */
	if (erhai_glue_cycle && erhai_glue_clk_recover) {
		r = erhai_glue_clk_recover(dwc);
		dev_info(dwc->dev, "erhai revive: glue clock cycle ret=%d\n",
			 r);
	}

	r = dwc3_core_soft_reset(dwc);
	if (r && erhai_phy_reinit) {
		dev_info(dwc->dev,
			 "erhai revive: soft_reset=%d, re-initing phys\n", r);
		erhai_dwc3_phy_cycle(dwc);
		r = dwc3_core_soft_reset(dwc);
	}

	/*
	 * v4: #24 evidence — after the BCR pulse the fabric reads alive but
	 * the soft reset only starts completing tens of seconds later
	 * (19:52:53 INCOMPLETE -> 19:53:58 soft_reset=0). Instead of
	 * continuing device entry against maxpacket=0 endpoints (g_ncm -19,
	 * dead command engine), poll for completion. Runs in the DRD mode
	 * work; the bounded sleep replaces the -110 timeout storms that made
	 * the tablet unresponsive for minutes.
	 */
	if (r && erhai_revive_retries) {
		unsigned int i;

		for (i = 0; i < erhai_revive_retries && r; i++) {
			msleep(erhai_revive_retry_ms);
			if (!erhai_island_alive(dwc)) {
				dev_info(dwc->dev,
					 "erhai revive: island still dead, re-pulsing BCR\n");
				erhai_dwc3_gcc_dance_once(dwc, false);
			}
			r = dwc3_core_soft_reset(dwc);
		}
		if (!r)
			dev_info(dwc->dev,
				 "erhai revive: soft reset completed after %u retries\n",
				 i);
	}

	v = dwc3_readl(dwc, DWC3_GHWPARAMS0);
	if (v && erhai_island_alive(dwc) && !r)
		dev_info(dwc->dev,
			 "erhai revive: SUCCESS GHWPARAMS0=%#x soft_reset=0\n",
			 v);
	else
		dev_err(dwc->dev,
			"erhai revive: INCOMPLETE GHWPARAMS0=%#x soft_reset=%d\n",
			v, r);

	return true;
}
