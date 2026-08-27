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
#include <linux/moduleparam.h>
#include <linux/phy/phy.h>

#include "core.h"
#include "io.h"

#define ERHAI_GCC_BASE		0x00100000
#define ERHAI_GCC_USB30_BCR	(ERHAI_GCC_BASE + 0x39000)

#define GDSCR_SW_COLLAPSE	BIT(0)
#define CFG_POWER_UP_COMPLETE	BIT(16)

static bool erhai_revive_enable = true;
module_param(erhai_revive_enable, bool, 0444);
MODULE_PARM_DESC(erhai_revive_enable,
		 "attempt recovery when the USB island reads dead");

static bool erhai_phy_reinit = true;
module_param(erhai_phy_reinit, bool, 0444);
MODULE_PARM_DESC(erhai_phy_reinit,
		 "power-cycle usb2/usb3 phys as last revive resort");

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

static void erhai_dwc3_phy_cycle(struct dwc3 *dwc)
{
	struct phy *phys[2];
	int i, ret;

	phys[0] = dwc->usb2_generic_phy[0];
	phys[1] = dwc->usb3_generic_phy[0];

	for (i = 0; i < 2; i++) {
		if (!phys[i] || IS_ERR(phys[i]))
			continue;
		phy_power_off(phys[i]);
		phy_exit(phys[i]);
		ret = phy_init(phys[i]);
		if (!ret)
			ret = phy_power_on(phys[i]);
		dev_info(dwc->dev, "erhai revive: phy%d reinit ret=%d\n",
			 i, ret);
	}
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

	if (erhai_glue_clk_recover) {
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
