// SPDX-License-Identifier: GPL-2.0
/*
 * erhai-revive.c - last-resort recovery for the SM8750 USB register island.
 *
 * Live forensics on the unplug wedge (#21 capture): the dwc3/xhci block dies
 * inside xhci unregister; all clocks/GDSC/ICC/BCR software state stays ON.
 * A GCC_USB30_PRIM_BCR pulse revives the register fabric (GHWPARAMS0/FIFO
 * regs read sane again) but the command engine stays dead (DEPCMD/xhci
 * setup -110) until a proper core soft reset runs against LIVE hardware.
 * Kernel #21 fired the pulse and continued mid-flight: gadget bound but
 * every command timed out until the next full mode transition.
 *
 * revive-v2: pulse BCR -> dwc3_core_soft_reset() as the FUNCTIONAL gate
 * (a soft reset against dead regs returns fake success, so this one runs
 * only after the pulse) -> if that times out, full GDSC collapse/restore
 * cycle -> soft reset again. Caller then continues device-mode entry with
 * event buffers + gadget init against a reset, alive controller.
 */
#include <linux/bits.h>
#include <linux/sizes.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>

#include "core.h"
#include "io.h"

#define ERHAI_GCC_BASE		0x00100000
#define ERHAI_GCC_USB30_BCR	(ERHAI_GCC_BASE + 0x39000)

#define GDSCR_SW_COLLAPSE	BIT(0)
#define CFG_POWER_UP_COMPLETE	BIT(16)

static bool erhai_revive_enable = true;
module_param(erhai_revive_enable, bool, 0444);
MODULE_PARM_DESC(erhai_revive_enable,
		 "attempt BCR/GDSC dance when the USB island reads dead");

bool erhai_dwc3_revive(struct dwc3 *dwc);

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

bool erhai_dwc3_revive(struct dwc3 *dwc)
{
	u32 v;
	int r;

	if (!erhai_revive_enable || !dwc->regs)
		return false;
	if (dwc3_readl(dwc, DWC3_GHWPARAMS0))
		return false;

	dev_err(dwc->dev, "erhai revive: island dead (GHWPARAMS0=0)\n");

	erhai_dwc3_gcc_dance_once(dwc, false);
	if (!dwc3_readl(dwc, DWC3_GHWPARAMS0)) {
		dev_info(dwc->dev,
			 "erhai revive: BCR insufficient, cycling GDSC\n");
		erhai_dwc3_gcc_dance_once(dwc, true);
	}
	r = dwc3_core_soft_reset(dwc);
	if (r) {
		dev_info(dwc->dev,
			 "erhai revive: soft_reset=%d, cycling GDSC\n", r);
		erhai_dwc3_gcc_dance_once(dwc, true);
		r = dwc3_core_soft_reset(dwc);
	}

	v = dwc3_readl(dwc, DWC3_GHWPARAMS0);
	if (v && !r)
		dev_info(dwc->dev,
			 "erhai revive: SUCCESS GHWPARAMS0=%#x soft_reset=0\n",
			 v);
	else
		dev_err(dwc->dev,
			"erhai revive: INCOMPLETE GHWPARAMS0=%#x soft_reset=%d\n",
			v, r);

	return true;
}
