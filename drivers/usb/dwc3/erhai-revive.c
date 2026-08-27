// SPDX-License-Identifier: GPL-2.0
/*
 * erhai-revive.c - last-resort recovery for the SM8750 USB register island.
 *
 * Live /dev/mem forensics on the unplug wedge showed the dwc3/xhci block
 * reading 0x0 while clocks, GDSCs, interconnect votes and BCRs all verified
 * ON from software. Pulsing GCC_USB30_PRIM_BCR partially revived the block;
 * a GDSC collapse/restore cycle revived more. This is the same dance done
 * from kernel context so a wedged DRD transition can attempt recovery
 * instead of leaving a dead controller until the next hard reset.
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

void erhai_dwc3_revive(struct dwc3 *dwc);

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
			 "erhai revive: BCR insufficient, cycling GDSC\n");
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

void erhai_dwc3_revive(struct dwc3 *dwc)
{
	u32 v;
	int round;

	if (!erhai_revive_enable || !dwc->regs)
		return;
	if (dwc3_readl(dwc, DWC3_GHWPARAMS0))
		return;

	dev_err(dwc->dev, "erhai revive: island dead (GHWPARAMS0=0)\n");
	for (round = 1; round <= 2 && !dwc3_readl(dwc, DWC3_GHWPARAMS0); round++)
		erhai_dwc3_gcc_dance_once(dwc, round == 2);

	v = dwc3_readl(dwc, DWC3_GHWPARAMS0);
	if (v)
		dev_info(dwc->dev, "erhai revive: SUCCESS GHWPARAMS0=%#x\n", v);
	else
		dev_err(dwc->dev,
			"erhai revive: island STILL DEAD after BCR+GDSC\n");
}
