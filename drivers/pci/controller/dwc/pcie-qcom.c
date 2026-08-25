// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm PCIe root complex driver
 *
 * Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 * Copyright 2015 Linaro Limited.
 *
 * Author: Stanimir Varbanov <svarbanov@mm-sol.com>
 */

#include <linux/clk.h>
#include <linux/crc8.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interconnect.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/pci-ecam.h>
#include <linux/pci-pwrctrl.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/phy/pcie.h>
#include <linux/phy/phy.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/units.h>

#include "../../pci.h"
#include "../pci-host-common.h"
#include "pcie-designware.h"
#include "pcie-qcom-common.h"

/* PARF registers */
#define PARF_SYS_CTRL				0x00
#define PARF_PM_CTRL				0x20
#define PARF_PCS_DEEMPH				0x34
#define PARF_PCS_SWING				0x38
#define PARF_PHY_CTRL				0x40
#define PARF_PHY_REFCLK				0x4c
#define PARF_CONFIG_BITS			0x50
#define PARF_DBI_BASE_ADDR			0x168
#define PARF_SLV_ADDR_SPACE_SIZE		0x16c
#define PARF_MHI_CLOCK_RESET_CTRL		0x174
#define PARF_AXI_MSTR_WR_ADDR_HALT		0x178
#define PARF_AXI_MSTR_WR_ADDR_HALT_V2		0x1a8
#define PARF_Q2A_FLUSH				0x1ac
#define PARF_LTSSM				0x1b0
#define PARF_INT_ALL_MASK			0x22c
#define PARF_CFG_BITS_3				0x2c4
#define PARF_SID_OFFSET				0x234
#define PARF_BDF_TRANSLATE_CFG			0x24c
#define PARF_DBI_BASE_ADDR_V2			0x350
#define PARF_DBI_BASE_ADDR_V2_HI		0x354
#define PARF_SLV_ADDR_SPACE_SIZE_V2		0x358
#define PARF_SLV_ADDR_SPACE_SIZE_V2_HI		0x35c
#define PARF_NO_SNOOP_OVERRIDE			0x3d4
#define PARF_ATU_BASE_ADDR			0x634
#define PARF_ATU_BASE_ADDR_HI			0x638
#define PARF_DEVICE_TYPE			0x1000
#define PARF_BDF_TO_SID_TABLE_N			0x2000
#define PARF_BDF_TO_SID_CFG			0x2c00
/*
 * CESTA clkreq/l1ss-timeout routing config. Offset matches the downstream MSM
 * PCIe driver's "qcom,pcie-clkreq-offset" for the SM8750 WCN RC (sun-pcie.dtsi
 * pcie0@1c00000 = 0x2c48). bit0 selects where the controller takes CLKREQ from:
 * 1 = from the CESTA hardware state-machine, 0 = from the direct clkreq gpio.
 * bit3 enables forwarding the L1ss-sleep timeout indication to the CESTA SM.
 */
#define PARF_CESTA_CONFIG			0x2c48
#define PARF_CESTA_CLKREQ_SEL			BIT(0)
#define PARF_CESTA_L1SUB_TIMEOUT_EXT_INT_EN	BIT(3)

/*
 * CESTA l1ss-sleep timeout timer (PARF_L1SUB_AHB_CLK_MAX_TIMER). Arming it makes
 * the controller signal "l1ss timeout" to the CESTA SM (forwarded by the bit3
 * above) so the SM enters l1ss-sleep and grants the fw's request. Value =
 * RESET | (l1ss_timeout_us * aux_clk_freq_MHz); downstream sun-pcie.dtsi pcie0:
 * qcom,drv-l1ss-timeout-us=5000, qcom,aux-clk-freq=20 -> 100000 ticks.
 */
#define PARF_L1SUB_AHB_CLK_MAX_TIMER		0x180
#define PARF_L1SUB_AHB_CLK_MAX_TIMER_RESET	BIT(31)
#define CESTA_L1SS_TIMEOUT_TICKS		(5000 * 20)

/*
 * CESTA PCIe State-Manager register block ("pcie_sm" reg in sun-pcie.dtsi
 * pcie0 = phys 0x01D07000 / 0x7000). The SM instruction memory is at offset 0;
 * the downstream loads its instruction + branch sequence here and starts it.
 */
#define PCIE_SM_PHYS				0x01d07000
#define PCIE_SM_SIZE				0x7000
#define PCIE_SM_PWR_CTRL			0x1040
#define PCIE_SM_PWR_CTRL_MASK			0x1048
#define PCIE_SM_BRANCH_OFFSET			0x1000
#define PCIE_SM_START_OFFSET			0x1090
/* SM status regs (read-only) to confirm the SM is actually executing. */
#define PCIE_SM_PC_VAL				0x1088
#define PCIE_SM_PWR_CTRL_STATUS			0x1070
#define PCIE_SM_CLKREQ_GATE_REQ_STATUS		0x109c

/*
 * pcie_crm hardware ("qcom,pcie-crm-v2" @ crm@1d01000 in sun.dtsi). The CESTA
 * SM votes resources through this block; without it the SM's perf-ol votes go
 * nowhere. Probe it to decide whether the full CRM port is needed: read the
 * common region (reg[5] @0x1d05f00) version/cfg/enable + HW-DRV0 perf-ol status.
 */
#define PCIE_CRM_BASE_PHYS			0x01d01000	/* reg "base"   */
#define PCIE_CRM_BASE_SIZE			0x2000
#define PCIE_CRM_COMMON_PHYS			0x01d05f00	/* reg "common" */
#define PCIE_CRM_COMMON_SIZE			0x100
#define PCIE_CRM_CRMC_PHYS			0x01d03800	/* reg "crm_c"  */
#define PCIE_CRM_CRMC_SIZE			0x2000
#define CRM_COMMON_VERSION			0x0
#define CRM_COMMON_CFG_PARAM_1			0x4
#define CRM_COMMON_CFG_PARAM_2			0x8
#define CRM_COMMON_ENABLE			0xc
/* HW-DRV perf-ol VCD regs live in the "base" region: drv0 @0, drv1 @0x1000. */
#define CRM_HWDRV_DISTANCE			0x1000
#define CRM_HWDRV_PERFOL_STATUS			0x28
#define CRM_HWDRV_PERFOL_PWR_IDX_STATUS		0x37c
/* crmc aggregated/current perf-ol for the PCIe resource. */
#define CRM_CRMC_AGGR_PERF_OL			0x4
#define CRM_CRMC_CURR_PERF_OL			0x24

/*
 * HW-DRV perf-ol VCD write layout (in the "base" region, drv0 @ offset 0).
 * crm_get_offset() = PWR_STn + ch*PWR_ST_CHN_DISTANCE + resource*RES_DISTANCE.
 * Mirrors crm_write_pwr_states()/crm_flush_cache() for HW DRV0, resource 0.
 */
#define CRM_PERFOL_PWR_ST0			0x0	/* PWR_ST0; ST1..4 stride 4 */
#define CRM_PERFOL_PWR_ST_STRIDE		0x4
#define CRM_PERFOL_CHN_DISTANCE			0x14	/* PWR_ST_CHN_DISTANCE */
#define CRM_PERFOL_RES_DISTANCE			0x2c	/* DRV_RESOURCE_DISTANCE */
#define CRM_NUM_PERFOL_RES			2	/* perfol_vcd=2 (CFG1) */
#define CRM_NUM_PWR_STATES			5	/* pwrst/ch=5 (CFG1) */
#define CRM_CHN_BUSY				0x370
#define CRM_CHN_UPDATE				0x374
/* perf-ol vote levels (enum msm_pcie_cesta_perf_idx). */
#define CRM_PERF_LVL_D3COLD			0
#define CRM_PERF_LVL_L1SS			1
#define CRM_PERF_LVL_GEN1			2
#define CRM_PERF_LVL_GEN2			3
#define CRM_PERF_LVL_GEN3			4

/* ELBI registers */
#define ELBI_SYS_CTRL				0x04
#define ELBI_SYS_STTS				0x08

/* DBI registers */
#define AXI_MSTR_RESP_COMP_CTRL0		0x818
#define AXI_MSTR_RESP_COMP_CTRL1		0x81c

/* MHI registers */
#define PARF_DEBUG_CNT_PM_LINKST_IN_L2		0xc04
#define PARF_DEBUG_CNT_PM_LINKST_IN_L1		0xc0c
#define PARF_DEBUG_CNT_PM_LINKST_IN_L0S		0xc10
#define PARF_DEBUG_CNT_AUX_CLK_IN_L1SUB_L1	0xc84
#define PARF_DEBUG_CNT_AUX_CLK_IN_L1SUB_L2	0xc88

/* PARF_SYS_CTRL register fields */
#define MAC_PHY_POWERDOWN_IN_P2_D_MUX_EN	BIT(29)
#define MST_WAKEUP_EN				BIT(13)
#define SLV_WAKEUP_EN				BIT(12)
#define MSTR_ACLK_CGC_DIS			BIT(10)
#define SLV_ACLK_CGC_DIS			BIT(9)
#define CORE_CLK_CGC_DIS			BIT(6)
#define AUX_PWR_DET				BIT(4)
#define L23_CLK_RMV_DIS				BIT(2)
#define L1_CLK_RMV_DIS				BIT(1)

/* PARF_PM_CTRL register fields */
#define REQ_NOT_ENTR_L1				BIT(5)

/* PARF_PCS_DEEMPH register fields */
#define PCS_DEEMPH_TX_DEEMPH_GEN1(x)		FIELD_PREP(GENMASK(21, 16), x)
#define PCS_DEEMPH_TX_DEEMPH_GEN2_3_5DB(x)	FIELD_PREP(GENMASK(13, 8), x)
#define PCS_DEEMPH_TX_DEEMPH_GEN2_6DB(x)	FIELD_PREP(GENMASK(5, 0), x)

/* PARF_PCS_SWING register fields */
#define PCS_SWING_TX_SWING_FULL(x)		FIELD_PREP(GENMASK(14, 8), x)
#define PCS_SWING_TX_SWING_LOW(x)		FIELD_PREP(GENMASK(6, 0), x)

/* PARF_PHY_CTRL register fields */
#define PHY_CTRL_PHY_TX0_TERM_OFFSET_MASK	GENMASK(20, 16)
#define PHY_CTRL_PHY_TX0_TERM_OFFSET(x)		FIELD_PREP(PHY_CTRL_PHY_TX0_TERM_OFFSET_MASK, x)
#define PHY_TEST_PWR_DOWN			BIT(0)

/* PARF_PHY_REFCLK register fields */
#define PHY_REFCLK_SSP_EN			BIT(16)
#define PHY_REFCLK_USE_PAD			BIT(12)

/* PARF_CONFIG_BITS register fields */
#define PHY_RX0_EQ(x)				FIELD_PREP(GENMASK(26, 24), x)

/* PARF_SLV_ADDR_SPACE_SIZE register value */
#define SLV_ADDR_SPACE_SZ			0x80000000

/* PARF_MHI_CLOCK_RESET_CTRL register fields */
#define AHB_CLK_EN				BIT(0)
#define MSTR_AXI_CLK_EN				BIT(1)
#define BYPASS					BIT(4)

/* PARF_AXI_MSTR_WR_ADDR_HALT register fields */
#define EN					BIT(31)

/* PARF_LTSSM register fields */
#define LTSSM_EN				BIT(8)
#define PARF_LTSSM_STATE_MASK			GENMASK(5, 0)

/* PARF_NO_SNOOP_OVERRIDE register fields */
#define WR_NO_SNOOP_OVERRIDE_EN			BIT(1)
#define RD_NO_SNOOP_OVERRIDE_EN			BIT(3)

/* PARF_DEVICE_TYPE register fields */
#define DEVICE_TYPE_RC				0x4

/* PARF_BDF_TO_SID_CFG fields */
#define BDF_TO_SID_BYPASS			BIT(0)

/* ELBI_SYS_CTRL register fields */
#define ELBI_SYS_CTRL_LT_ENABLE			BIT(0)
#define ELBI_SYS_CTRL_PME_TURNOFF_MSG		BIT(4)

/* ELBI_SYS_STTS register fields */
#define ELBI_SYS_STTS_LTSSM_STATE_MASK		GENMASK(17, 12)

/* AXI_MSTR_RESP_COMP_CTRL0 register fields */
#define CFG_REMOTE_RD_REQ_BRIDGE_SIZE_2K	0x4
#define CFG_REMOTE_RD_REQ_BRIDGE_SIZE_4K	0x5

/* AXI_MSTR_RESP_COMP_CTRL1 register fields */
#define CFG_BRIDGE_SB_INIT			BIT(0)

/* PCI_EXP_SLTCAP register fields */
#define PCIE_CAP_SLOT_POWER_LIMIT_VAL		FIELD_PREP(PCI_EXP_SLTCAP_SPLV, 250)
#define PCIE_CAP_SLOT_POWER_LIMIT_SCALE		FIELD_PREP(PCI_EXP_SLTCAP_SPLS, 1)
#define PCIE_CAP_SLOT_VAL			(PCI_EXP_SLTCAP_ABP | \
						PCI_EXP_SLTCAP_PCP | \
						PCI_EXP_SLTCAP_MRLSP | \
						PCI_EXP_SLTCAP_AIP | \
						PCI_EXP_SLTCAP_PIP | \
						PCI_EXP_SLTCAP_HPS | \
						PCI_EXP_SLTCAP_EIP | \
						PCIE_CAP_SLOT_POWER_LIMIT_VAL | \
						PCIE_CAP_SLOT_POWER_LIMIT_SCALE)

#define PERST_DELAY_US				1000

#define QCOM_PCIE_CRC8_POLYNOMIAL		(BIT(2) | BIT(1) | BIT(0))

#define QCOM_PCIE_LINK_SPEED_TO_BW(speed) \
		Mbps_to_icc(PCIE_SPEED2MBS_ENC(pcie_get_link_speed(speed)))

struct qcom_pcie_resources_1_0_0 {
	struct clk_bulk_data *clks;
	int num_clks;
	struct reset_control *core;
	struct regulator *vdda;
};

#define QCOM_PCIE_2_1_0_MAX_RESETS		6
#define QCOM_PCIE_2_1_0_MAX_SUPPLY		3
struct qcom_pcie_resources_2_1_0 {
	struct clk_bulk_data *clks;
	int num_clks;
	struct reset_control_bulk_data resets[QCOM_PCIE_2_1_0_MAX_RESETS];
	int num_resets;
	struct regulator_bulk_data supplies[QCOM_PCIE_2_1_0_MAX_SUPPLY];
};

#define QCOM_PCIE_2_3_2_MAX_SUPPLY		2
struct qcom_pcie_resources_2_3_2 {
	struct clk_bulk_data *clks;
	int num_clks;
	struct regulator_bulk_data supplies[QCOM_PCIE_2_3_2_MAX_SUPPLY];
};

#define QCOM_PCIE_2_3_3_MAX_RESETS		7
struct qcom_pcie_resources_2_3_3 {
	struct clk_bulk_data *clks;
	int num_clks;
	struct reset_control_bulk_data rst[QCOM_PCIE_2_3_3_MAX_RESETS];
};

#define QCOM_PCIE_2_4_0_MAX_RESETS		12
struct qcom_pcie_resources_2_4_0 {
	struct clk_bulk_data *clks;
	int num_clks;
	struct reset_control_bulk_data resets[QCOM_PCIE_2_4_0_MAX_RESETS];
	int num_resets;
};

#define QCOM_PCIE_2_7_0_MAX_SUPPLIES		2
struct qcom_pcie_resources_2_7_0 {
	struct clk_bulk_data *clks;
	int num_clks;
	struct regulator_bulk_data supplies[QCOM_PCIE_2_7_0_MAX_SUPPLIES];
	struct reset_control *rst;
};

struct qcom_pcie_resources_2_9_0 {
	struct clk_bulk_data *clks;
	int num_clks;
	struct reset_control *rst;
};

union qcom_pcie_resources {
	struct qcom_pcie_resources_1_0_0 v1_0_0;
	struct qcom_pcie_resources_2_1_0 v2_1_0;
	struct qcom_pcie_resources_2_3_2 v2_3_2;
	struct qcom_pcie_resources_2_3_3 v2_3_3;
	struct qcom_pcie_resources_2_4_0 v2_4_0;
	struct qcom_pcie_resources_2_7_0 v2_7_0;
	struct qcom_pcie_resources_2_9_0 v2_9_0;
};

struct qcom_pcie;

struct qcom_pcie_ops {
	int (*get_resources)(struct qcom_pcie *pcie);
	int (*init)(struct qcom_pcie *pcie);
	int (*post_init)(struct qcom_pcie *pcie);
	void (*host_post_init)(struct qcom_pcie *pcie);
	void (*deinit)(struct qcom_pcie *pcie);
	void (*ltssm_enable)(struct qcom_pcie *pcie);
	int (*config_sid)(struct qcom_pcie *pcie);
	enum dw_pcie_ltssm (*get_ltssm)(struct qcom_pcie *pcie);
};

 /**
  * struct qcom_pcie_cfg - Per SoC config struct
  * @ops: qcom PCIe ops structure
  * @override_no_snoop: Override NO_SNOOP attribute in TLP to enable cache
  * snooping
  * @firmware_managed: Set if the Root Complex is firmware managed
  */
struct qcom_pcie_cfg {
	const struct qcom_pcie_ops *ops;
	bool override_no_snoop;
	bool firmware_managed;
	bool no_l0s;
};

struct qcom_pcie_perst {
	struct list_head list;
	struct gpio_desc *desc;
};

struct qcom_pcie_port {
	struct list_head list;
	struct phy *phy;
	u32 l1ss_t_power_on;
	struct list_head perst;
};

struct qcom_pcie {
	struct dw_pcie *pci;
	void __iomem *parf;			/* DT parf */
	void __iomem *mhi;
	union qcom_pcie_resources res;
	struct icc_path *icc_mem;
	struct icc_path *icc_cpu;
	const struct qcom_pcie_cfg *cfg;
	struct dentry *debugfs;
	struct list_head ports;
	struct gpio_desc *reset;
	bool use_pm_opp;
	bool wcn_clk_hwgated;	/* D3: SW vote dropped on HWCG AXI clocks */
	/* v472: WCN L1SS programming (msm_pcie_config_l1ss faithful). Values from the
	 * Android pcie0 DT (qcom,l1-2-th-value=0x96 / qcom,l1-2-th-scale=2); applied to
	 * EP+RC when the WCN endpoint binds (bus notifier) since pcie-qcom has no
	 * post-enumeration hook. */
	struct notifier_block wcn_l1ss_nb;
	u32 wcn_l1_2_th_value;
	u32 wcn_l1_2_th_scale;
};

#define to_qcom_pcie(x)		dev_get_drvdata((x)->dev)

/*
 * erhai WCN7880 cal->mission faithful re-link. cnss's cnss_set_pci_link() is
 * stubbed on this mainline port (pci_qcom.o excluded), so its own cal->mission
 * suspend_pci_link(DOWN)/resume_pci_link(UP) recovery never actually drops and
 * cold-retrains the WCN link -> the chip stays warm in MISSION EE, WLFW never
 * re-arrives. Export a real link toggle that cnss's stub calls.
 *
 * A bare PERST deassert does NOT retrain a power-cycled EP (boot#38/#39: LTSSM
 * stuck in PRE_DETECT_QUIET, -110) because the RC PHY/controller are left in a
 * stale state. The only path proven to train this link is the full host
 * bring-up. So reuse the DWC system-PM machinery:
 *   DOWN = dw_pcie_stop_link + pp->ops->deinit (qcom_pcie_host_deinit: PERST
 *          assert, pci_pwrctrl powers the WCN OFF via pwrseq/WLAN_EN, PHY off,
 *          clocks off) + mark pci->suspended.
 *   UP   = dw_pcie_resume_noirq (pp->ops->init = qcom_pcie_host_init re-powers
 *          the WCN via pwrctrl/pwrseq, re-inits clocks+PHY, deasserts PERST;
 *          then setup_rc + start_link + wait_for_link + post_init).
 * resume_noirq is a no-op unless we set pci->suspended, so a stray UP without a
 * matching DOWN can't disturb a live link. pwrctrl owns WLAN_EN throughout.
 * Gated for A/B.
 */
static struct qcom_pcie *qcom_pcie_wcn_erhai;
static bool qcom_pcie_wcn_relink = true;
module_param(qcom_pcie_wcn_relink, bool, 0644);
MODULE_PARM_DESC(qcom_pcie_wcn_relink,
		"erhai: cnss_set_pci_link cold-cycles the WCN at cal->mission (default on)");
static unsigned int qcom_pcie_wcn_relink_settle_ms = 100;
module_param(qcom_pcie_wcn_relink_settle_ms, uint, 0644);

static void __qcom_pcie_perst_assert(struct qcom_pcie *pcie, bool assert)
{
	struct qcom_pcie_perst *perst;
	struct qcom_pcie_port *port;
	int val = assert ? 1 : 0;

	list_for_each_entry(port, &pcie->ports, list) {
		list_for_each_entry(perst, &port->perst, list)
			gpiod_set_value_cansleep(perst->desc, val);
	}

	usleep_range(PERST_DELAY_US, PERST_DELAY_US + 500);
}

static void qcom_pcie_perst_assert(struct qcom_pcie *pcie)
{
	__qcom_pcie_perst_assert(pcie, true);
}

static void qcom_pcie_perst_deassert(struct qcom_pcie *pcie)
{
	/* Ensure that PERST# has been asserted for at least 100 ms */
	msleep(PCIE_T_PVPERL_MS);
	__qcom_pcie_perst_assert(pcie, false);
}

static struct pci_dev *qcom_pcie_wcn_find_rootport(struct qcom_pcie *pcie);
static struct pci_dev *qcom_pcie_wcn_find_ep(struct qcom_pcie *pcie);

/*
 * cnss calls set_pci_link(DOWN) then, some time later, set_pci_link(UP), and
 * between the two it runs its own shutdown bookkeeping + MHI teardown that
 * touches the EP/RC. If we drop the link in DOWN, those accesses hit a
 * clock-gated RC -> NoC/SError (boot#44 crashed even with the portdrv IRQ
 * masked -- the fault is the DOWN *window*, not just the teardown instant;
 * mode 3 only survived because the whole cycle was one atomic call with
 * nothing else touching the device).
 *
 * So DON'T expose a down link to cnss at all: DOWN just records intent and
 * leaves the link electrically UP; the entire power-cycle (teardown + chip
 * WLAN_EN cold-cycle via pwrctrl + link retrain) is done ATOMICALLY inside the
 * following UP, exactly reproducing the mode-3 conditions that ran clean. The
 * chip still genuinely power-cycles (fresh SBL->AMSS) -- just deferred to UP.
 */
static bool qcom_pcie_wcn_cycle_pending;

int qcom_pcie_wcn_set_link(bool up)
{
	struct qcom_pcie *pcie = qcom_pcie_wcn_erhai;
	struct pci_dev *rp, *ep = NULL;
	struct dw_pcie *pci;
	int rp_irq = 0, ret;

	if (!qcom_pcie_wcn_relink)	/* A/B: behave like the old cnss stub */
		return 0;

	if (!pcie) {
		pr_warn("erhai: qcom_pcie_wcn_set_link: WCN pcie not registered\n");
		return -ENODEV;
	}
	pci = pcie->pci;

	if (!up) {
		/* Defer: leave the link up while cnss does its bookkeeping. */
		qcom_pcie_wcn_cycle_pending = true;
		dev_info(pci->dev, "erhai-cc: 0 WCN DOWN deferred (atomic cold-cycle happens at UP)\n");
		return 0;
	}

	/* UP with no pending DOWN => nothing to do, don't disturb the link. */
	if (!qcom_pcie_wcn_cycle_pending) {
		dev_info(pci->dev, "erhai: WCN UP (no pending cycle, link_up=%d)\n",
			 dw_pcie_link_up(pci));
		return 0;
	}
	qcom_pcie_wcn_cycle_pending = false;

	/*
	 * Atomic cold-cycle. Mask the root-port PME/AER/bwctrl IRQ so a portdrv
	 * service can't read the clock-gated RC during the ~200ms the link is
	 * down (boot#43 mode-3 root cause), then teardown + full re-init.
	 */
	rp = qcom_pcie_wcn_find_rootport(pcie);
	ep = qcom_pcie_wcn_find_ep(pcie);
	if (rp && rp->irq) {
		rp_irq = rp->irq;
		disable_irq(rp_irq);
	}

	/*
	 * Root cause of the ~50% post-cal SoC reset (breadcrumb-3 crash boot): after
	 * deinit() cuts PHY+clocks and drops the link, the WCN RC/EP config block is
	 * unclocked for the whole teardown+settle+retrain window. A CONCURRENT config
	 * access from another context -- udev/NetworkManager probing the just-renamed
	 * wlp1s0 EP, or a portdrv (PME/AER) config read -- hits the dead block and
	 * raises an unrecoverable external abort -> SoC reset. Masking rp_irq only
	 * stops the RC's own IRQ handler, not these. pci_cfg_access_lock() is the
	 * primitive pci_reset_function() uses for exactly this: concurrent
	 * pci_{read,write}_config on the locked dev block (pci_wait_cfg) until we
	 * unlock, instead of touching the unclocked device. Lock both RC and EP; the
	 * cold-cycle's own DBI accesses use readl/writel (not the config API) so they
	 * are unaffected and cannot deadlock.
	 */
	if (rp)
		pci_cfg_access_lock(rp);
	if (ep)
		pci_cfg_access_lock(ep);

	/*
	 * erhai-cc breadcrumbs: localize the faulting sub-step if it still resets
	 * (DRAM wiped by Mu -> only fsync'd journald lines survive). Pair with the
	 * fsync-per-line /dev/kmsg logger on the device.
	 */
	dev_info(pci->dev, "erhai-cc: 1 rp_irq=%d masked, cfg locked (rp=%d ep=%d), stop_link...\n",
		 rp_irq, !!rp, !!ep);
	dw_pcie_stop_link(pci);
	dev_info(pci->dev, "erhai-cc: 2 stop_link done, deinit (PERST+WLAN_EN off+PHY/clk off)...\n");
	if (pci->pp.ops->deinit)
		pci->pp.ops->deinit(&pci->pp);	/* PERST + WLAN_EN off + PHY/clk off */
	pci->suspended = true;
	dev_info(pci->dev, "erhai-cc: 3 deinit done, settle %ums...\n",
		 qcom_pcie_wcn_relink_settle_ms);
	if (qcom_pcie_wcn_relink_settle_ms)
		msleep(qcom_pcie_wcn_relink_settle_ms);	/* chip powered off; rails drain */

	dev_info(pci->dev, "erhai-cc: 4 resume_noirq (WLAN_EN cold-cycle + clk/PHY + retrain)...\n");
	ret = dw_pcie_resume_noirq(pci);	/* WLAN_EN cold-cycle + clk/PHY + retrain */
	dev_info(pci->dev, "erhai-cc: 5 resume_noirq returned %d\n", ret);

	if (rp_irq)
		enable_irq(rp_irq);
	if (ep) {
		pci_cfg_access_unlock(ep);
		pci_dev_put(ep);
	}
	if (rp) {
		pci_cfg_access_unlock(rp);
		pci_dev_put(rp);
	}

	dev_info(pci->dev, "erhai: WCN atomic cold-cycle done (portdrv irq %d, resume ret=%d link_up=%d)\n",
		 rp_irq, ret, dw_pcie_link_up(pci));

	/*
	 * dw_pcie_resume_noirq() can return 0 on this marginal WCN link even when
	 * the LTSSM never reached L0 (retrain trained to nowhere). cnss's caller
	 * (cnss_resume_pci_link) treats ret==0 as success and immediately pokes
	 * the EP config space (set_power_state(D0) + RESTORE_PCI_CONFIG_SPACE +
	 * pci_enable_device). Those accesses to a dead link raise an unrecoverable
	 * external abort / AER-uncorrectable -> full SoC reset (the ~50% post-cal
	 * reboot). Gate on link_up so a bad retrain becomes a clean cnss abort
	 * (-EAGAIN, mission retries) instead of a hard reboot.
	 */
	if (!ret && !dw_pcie_link_up(pci)) {
		dev_err(pci->dev, "erhai: WCN retrain reported OK but link is DOWN -> fail (avoid dead-EP SoC reset)\n");
		ret = -EIO;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(qcom_pcie_wcn_set_link);

/*
 * Standalone link DOWN->UP tester, driven with cnss NOT loaded:
 *   echo 1 > .../qcom_pcie_wcn_relink_test   bare RC teardown/re-init
 *   echo 2 > .../qcom_pcie_wcn_relink_test   same, but first put the WCN EP
 *      (01:00.0) into cnss's exact pre-DOWN state (save cfg + clear master +
 *      disable + D3hot) and restore it after (D0 + enable + restore + master).
 *      Mode 2 reproduces the boot#40 crash condition (cnss quiesces the EP to
 *      D3hot before calling set_pci_link(DOWN)) WITHOUT cnss/MHI, so a fault
 *      isolates whether the D3hot-EP + portdrv(PME/AER/bwctrl on IRQ 228)
 *      interaction is what tears down.
 * Every micro-step is logged BEFORE it runs so the last line to reach disk
 * localizes a hard fault. Reads that touch the RC block are only done while
 * its clocks are known on.
 */
static struct pci_dev *qcom_pcie_wcn_find_ep(struct qcom_pcie *pcie)
{
	struct pci_host_bridge *bridge = pcie->pci->pp.bridge;
	struct pci_dev *pdev = NULL;

	if (!bridge || !bridge->bus)
		return NULL;
	/* WCN EP is the single endpoint on bus 01 under this RC. */
	for_each_pci_dev(pdev) {
		if (pci_domain_nr(pdev->bus) == pci_domain_nr(bridge->bus) &&
		    pci_pcie_type(pdev) == PCI_EXP_TYPE_ENDPOINT)
			return pdev;
	}
	return NULL;
}

static struct pci_dev *qcom_pcie_wcn_find_rootport(struct qcom_pcie *pcie)
{
	struct pci_host_bridge *bridge = pcie->pci->pp.bridge;

	if (!bridge || !bridge->bus)
		return NULL;
	/* Root port = 00:00.0 on the RC's root bus. */
	return pci_get_slot(bridge->bus, PCI_DEVFN(0, 0));
}

static int qcom_pcie_wcn_relink_test_set(const char *val,
					 const struct kernel_param *kp)
{
	struct qcom_pcie *pcie = qcom_pcie_wcn_erhai;
	struct pci_dev *ep = NULL, *rp = NULL;
	struct dw_pcie *pci;
	int mode = 1, ret;

	if (!pcie) {
		pr_warn("erhai relink_test: WCN pcie not registered\n");
		return -ENODEV;
	}
	pci = pcie->pci;
	if (val && val[0] == '2')
		mode = 2;
	else if (val && val[0] == '3')
		mode = 3;

	if (mode >= 2) {
		ep = qcom_pcie_wcn_find_ep(pcie);
		dev_info(pci->dev, "erhai relink_test[%d]: EP=%s -- quiescing to cnss pre-DOWN state (D3hot)\n",
			 mode, ep ? pci_name(ep) : "NONE");
		if (ep) {
			pci_clear_master(ep);
			pci_save_state(ep);
			pci_disable_device(ep);
			pci_set_power_state(ep, PCI_D3hot);
		}
	}
	if (mode == 3) {
		rp = qcom_pcie_wcn_find_rootport(pcie);
		if (rp && rp->irq) {
			dev_info(pci->dev, "erhai relink_test[3]: masking root-port portdrv IRQ %u (PME/AER/bwctrl) around the cycle\n",
				 rp->irq);
			disable_irq(rp->irq);
		} else {
			dev_info(pci->dev, "erhai relink_test[3]: root-port irq NOT found (rp=%s)\n",
				 rp ? pci_name(rp) : "NONE");
		}
	}

	dev_info(pci->dev, "erhai relink_test[%d]: STEP1 pre-down link_up=%d\n",
		 mode, dw_pcie_link_up(pci));
	dev_info(pci->dev, "erhai relink_test[%d]: STEP2 dw_pcie_stop_link\n", mode);
	dw_pcie_stop_link(pci);
	dev_info(pci->dev, "erhai relink_test[%d]: STEP3 calling ops->deinit (PHY/clocks/pwrctrl OFF) -- fault here => teardown unsafe\n", mode);
	if (pci->pp.ops->deinit)
		pci->pp.ops->deinit(&pci->pp);
	pci->suspended = true;
	dev_info(pci->dev, "erhai relink_test[%d]: STEP4 deinit returned OK; down complete, sleep 200ms\n", mode);
	msleep(200);
	dev_info(pci->dev, "erhai relink_test[%d]: STEP5 calling dw_pcie_resume_noirq\n", mode);
	ret = dw_pcie_resume_noirq(pci);
	dev_info(pci->dev, "erhai relink_test[%d]: STEP6 resume ret=%d link_up=%d\n",
		 mode, ret, dw_pcie_link_up(pci));

	if (mode == 3 && rp && rp->irq) {
		dev_info(pci->dev, "erhai relink_test[3]: unmasking root-port IRQ %u\n", rp->irq);
		enable_irq(rp->irq);
	}
	if (rp)
		pci_dev_put(rp);
	if (mode >= 2 && ep) {
		dev_info(pci->dev, "erhai relink_test[%d]: STEP7 restoring EP (D0 + restore + enable + master)\n", mode);
		pci_set_power_state(ep, PCI_D0);
		pci_restore_state(ep);
		if (pci_enable_device(ep) == 0)
			pci_set_master(ep);
		dev_info(pci->dev, "erhai relink_test[%d]: STEP8 EP restored\n", mode);
	}
	return 0;
}

static const struct kernel_param_ops qcom_pcie_wcn_relink_test_ops = {
	.set = qcom_pcie_wcn_relink_test_set,
};
module_param_cb(qcom_pcie_wcn_relink_test, &qcom_pcie_wcn_relink_test_ops,
		NULL, 0644);

static int qcom_pcie_start_link(struct dw_pcie *pci)
{
	struct qcom_pcie *pcie = to_qcom_pcie(pci);

	qcom_pcie_common_set_equalization(pci);

	if (pcie_get_link_speed(pci->max_link_speed) == PCIE_SPEED_16_0GT)
		qcom_pcie_common_set_16gt_lane_margining(pci);

	/* Enable Link Training state machine */
	if (pcie->cfg->ops->ltssm_enable)
		pcie->cfg->ops->ltssm_enable(pcie);

	return 0;
}

static void qcom_pcie_clear_aspm_l0s(struct dw_pcie *pci)
{
	struct qcom_pcie *pcie = to_qcom_pcie(pci);
	u16 offset;
	u32 val;

	if (!pcie->cfg->no_l0s)
		return;

	offset = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);

	dw_pcie_dbi_ro_wr_en(pci);

	val = readl(pci->dbi_base + offset + PCI_EXP_LNKCAP);
	val &= ~PCI_EXP_LNKCAP_ASPM_L0S;
	writel(val, pci->dbi_base + offset + PCI_EXP_LNKCAP);

	dw_pcie_dbi_ro_wr_dis(pci);
}

static void qcom_pcie_set_slot_nccs(struct dw_pcie *pci)
{
	u16 offset = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	u32 val;

	dw_pcie_dbi_ro_wr_en(pci);

	/*
	 * Qcom PCIe Root Ports do not support generating command completion
	 * notifications for the Hot-Plug commands. So set the NCCS field to
	 * avoid waiting for the completions.
	 */
	val = readl(pci->dbi_base + offset + PCI_EXP_SLTCAP);
	val |= PCI_EXP_SLTCAP_NCCS;
	writel(val, pci->dbi_base + offset + PCI_EXP_SLTCAP);

	dw_pcie_dbi_ro_wr_dis(pci);
}

static void qcom_pcie_configure_dbi_base(struct qcom_pcie *pcie)
{
	struct dw_pcie *pci = pcie->pci;

	if (pci->dbi_phys_addr) {
		/*
		 * PARF_DBI_BASE_ADDR register is in CPU domain and require to
		 * be programmed with CPU physical address.
		 */
		writel(lower_32_bits(pci->dbi_phys_addr), pcie->parf +
							PARF_DBI_BASE_ADDR);
		writel(SLV_ADDR_SPACE_SZ, pcie->parf +
						PARF_SLV_ADDR_SPACE_SIZE);
	}
}

static void qcom_pcie_configure_dbi_atu_base(struct qcom_pcie *pcie)
{
	struct dw_pcie *pci = pcie->pci;

	if (pci->dbi_phys_addr) {
		/*
		 * PARF_DBI_BASE_ADDR_V2 and PARF_ATU_BASE_ADDR registers are
		 * in CPU domain and require to be programmed with CPU
		 * physical addresses.
		 */
		writel(lower_32_bits(pci->dbi_phys_addr), pcie->parf +
							PARF_DBI_BASE_ADDR_V2);
		writel(upper_32_bits(pci->dbi_phys_addr), pcie->parf +
						PARF_DBI_BASE_ADDR_V2_HI);

		if (pci->atu_phys_addr) {
			writel(lower_32_bits(pci->atu_phys_addr), pcie->parf +
							PARF_ATU_BASE_ADDR);
			writel(upper_32_bits(pci->atu_phys_addr), pcie->parf +
							PARF_ATU_BASE_ADDR_HI);
		}

		writel(0x0, pcie->parf + PARF_SLV_ADDR_SPACE_SIZE_V2);
		writel(SLV_ADDR_SPACE_SZ, pcie->parf +
					PARF_SLV_ADDR_SPACE_SIZE_V2_HI);
	}
}

static void qcom_pcie_2_1_0_ltssm_enable(struct qcom_pcie *pcie)
{
	struct dw_pcie *pci = pcie->pci;
	u32 val;

	if (!pci->elbi_base) {
		dev_err(pci->dev, "ELBI is not present\n");
		return;
	}
	/* enable link training */
	val = readl(pci->elbi_base + ELBI_SYS_CTRL);
	val |= ELBI_SYS_CTRL_LT_ENABLE;
	writel(val, pci->elbi_base + ELBI_SYS_CTRL);
}

static enum dw_pcie_ltssm qcom_pcie_2_1_0_get_ltssm(struct qcom_pcie *pcie)
{
	struct dw_pcie *pci = pcie->pci;
	u32 val;

	val = readl(pci->elbi_base + ELBI_SYS_STTS);
	return (enum dw_pcie_ltssm)FIELD_GET(ELBI_SYS_STTS_LTSSM_STATE_MASK, val);
}

static int qcom_pcie_get_resources_2_1_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_1_0 *res = &pcie->res.v2_1_0;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	bool is_apq = of_device_is_compatible(dev->of_node, "qcom,pcie-apq8064");
	int ret;

	res->supplies[0].supply = "vdda";
	res->supplies[1].supply = "vdda_phy";
	res->supplies[2].supply = "vdda_refclk";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(res->supplies),
				      res->supplies);
	if (ret)
		return ret;

	res->num_clks = devm_clk_bulk_get_all(dev, &res->clks);
	if (res->num_clks < 0) {
		dev_err(dev, "Failed to get clocks\n");
		return res->num_clks;
	}

	res->resets[0].id = "pci";
	res->resets[1].id = "axi";
	res->resets[2].id = "ahb";
	res->resets[3].id = "por";
	res->resets[4].id = "phy";
	res->resets[5].id = "ext";

	/* ext is optional on APQ8016 */
	res->num_resets = is_apq ? 5 : 6;
	ret = devm_reset_control_bulk_get_exclusive(dev, res->num_resets, res->resets);
	if (ret < 0)
		return ret;

	return 0;
}

static void qcom_pcie_deinit_2_1_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_1_0 *res = &pcie->res.v2_1_0;

	clk_bulk_disable_unprepare(res->num_clks, res->clks);
	reset_control_bulk_assert(res->num_resets, res->resets);

	writel(1, pcie->parf + PARF_PHY_CTRL);

	regulator_bulk_disable(ARRAY_SIZE(res->supplies), res->supplies);
}

static int qcom_pcie_init_2_1_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_1_0 *res = &pcie->res.v2_1_0;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	int ret;

	/* reset the PCIe interface as uboot can leave it undefined state */
	ret = reset_control_bulk_assert(res->num_resets, res->resets);
	if (ret < 0) {
		dev_err(dev, "cannot assert resets\n");
		return ret;
	}

	ret = regulator_bulk_enable(ARRAY_SIZE(res->supplies), res->supplies);
	if (ret < 0) {
		dev_err(dev, "cannot enable regulators\n");
		return ret;
	}

	ret = reset_control_bulk_deassert(res->num_resets, res->resets);
	if (ret < 0) {
		dev_err(dev, "cannot deassert resets\n");
		regulator_bulk_disable(ARRAY_SIZE(res->supplies), res->supplies);
		return ret;
	}

	return 0;
}

static int qcom_pcie_post_init_2_1_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_1_0 *res = &pcie->res.v2_1_0;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	struct device_node *node = dev->of_node;
	u32 val;
	int ret;

	/* Force PHY out of lowest power state */
	val = readl(pcie->parf + PARF_PHY_CTRL);
	val &= ~PHY_TEST_PWR_DOWN;
	writel(val, pcie->parf + PARF_PHY_CTRL);

	ret = clk_bulk_prepare_enable(res->num_clks, res->clks);
	if (ret)
		return ret;

	if (of_device_is_compatible(node, "qcom,pcie-ipq8064") ||
	    of_device_is_compatible(node, "qcom,pcie-ipq8064-v2")) {
		writel(PCS_DEEMPH_TX_DEEMPH_GEN1(24) |
			       PCS_DEEMPH_TX_DEEMPH_GEN2_3_5DB(24) |
			       PCS_DEEMPH_TX_DEEMPH_GEN2_6DB(34),
		       pcie->parf + PARF_PCS_DEEMPH);
		writel(PCS_SWING_TX_SWING_FULL(120) |
			       PCS_SWING_TX_SWING_LOW(120),
		       pcie->parf + PARF_PCS_SWING);
		writel(PHY_RX0_EQ(4), pcie->parf + PARF_CONFIG_BITS);
	}

	if (of_device_is_compatible(node, "qcom,pcie-ipq8064")) {
		/* set TX termination offset */
		val = readl(pcie->parf + PARF_PHY_CTRL);
		val &= ~PHY_CTRL_PHY_TX0_TERM_OFFSET_MASK;
		val |= PHY_CTRL_PHY_TX0_TERM_OFFSET(7);
		writel(val, pcie->parf + PARF_PHY_CTRL);
	}

	/* enable external reference clock */
	val = readl(pcie->parf + PARF_PHY_REFCLK);
	/* USE_PAD is required only for ipq806x */
	if (!of_device_is_compatible(node, "qcom,pcie-apq8064"))
		val &= ~PHY_REFCLK_USE_PAD;
	val |= PHY_REFCLK_SSP_EN;
	writel(val, pcie->parf + PARF_PHY_REFCLK);

	/* wait for clock acquisition */
	usleep_range(1000, 1500);

	/* Set the Max TLP size to 2K, instead of using default of 4K */
	writel(CFG_REMOTE_RD_REQ_BRIDGE_SIZE_2K,
	       pci->dbi_base + AXI_MSTR_RESP_COMP_CTRL0);
	writel(CFG_BRIDGE_SB_INIT,
	       pci->dbi_base + AXI_MSTR_RESP_COMP_CTRL1);

	qcom_pcie_set_slot_nccs(pcie->pci);

	return 0;
}

static int qcom_pcie_get_resources_1_0_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_1_0_0 *res = &pcie->res.v1_0_0;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;

	res->vdda = devm_regulator_get(dev, "vdda");
	if (IS_ERR(res->vdda))
		return PTR_ERR(res->vdda);

	res->num_clks = devm_clk_bulk_get_all(dev, &res->clks);
	if (res->num_clks < 0) {
		dev_err(dev, "Failed to get clocks\n");
		return res->num_clks;
	}

	res->core = devm_reset_control_get_exclusive(dev, "core");
	return PTR_ERR_OR_ZERO(res->core);
}

static void qcom_pcie_deinit_1_0_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_1_0_0 *res = &pcie->res.v1_0_0;

	reset_control_assert(res->core);
	clk_bulk_disable_unprepare(res->num_clks, res->clks);
	regulator_disable(res->vdda);
}

static int qcom_pcie_init_1_0_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_1_0_0 *res = &pcie->res.v1_0_0;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	int ret;

	ret = reset_control_deassert(res->core);
	if (ret) {
		dev_err(dev, "cannot deassert core reset\n");
		return ret;
	}

	ret = clk_bulk_prepare_enable(res->num_clks, res->clks);
	if (ret) {
		dev_err(dev, "cannot prepare/enable clocks\n");
		goto err_assert_reset;
	}

	ret = regulator_enable(res->vdda);
	if (ret) {
		dev_err(dev, "cannot enable vdda regulator\n");
		goto err_disable_clks;
	}

	return 0;

err_disable_clks:
	clk_bulk_disable_unprepare(res->num_clks, res->clks);
err_assert_reset:
	reset_control_assert(res->core);

	return ret;
}

static int qcom_pcie_post_init_1_0_0(struct qcom_pcie *pcie)
{
	qcom_pcie_configure_dbi_base(pcie);

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		u32 val = readl(pcie->parf + PARF_AXI_MSTR_WR_ADDR_HALT);

		val |= EN;
		writel(val, pcie->parf + PARF_AXI_MSTR_WR_ADDR_HALT);
	}

	qcom_pcie_set_slot_nccs(pcie->pci);

	return 0;
}

static void qcom_pcie_2_3_2_ltssm_enable(struct qcom_pcie *pcie)
{
	u32 val;

	/* enable link training */
	val = readl(pcie->parf + PARF_LTSSM);
	val |= LTSSM_EN;
	writel(val, pcie->parf + PARF_LTSSM);
}

static int qcom_pcie_get_resources_2_3_2(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_3_2 *res = &pcie->res.v2_3_2;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	int ret;

	res->supplies[0].supply = "vdda";
	res->supplies[1].supply = "vddpe-3v3";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(res->supplies),
				      res->supplies);
	if (ret)
		return ret;

	res->num_clks = devm_clk_bulk_get_all(dev, &res->clks);
	if (res->num_clks < 0) {
		dev_err(dev, "Failed to get clocks\n");
		return res->num_clks;
	}

	return 0;
}

static void qcom_pcie_deinit_2_3_2(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_3_2 *res = &pcie->res.v2_3_2;
	u32 val;

	/* Force PHY to lowest power state*/
	val = readl(pcie->parf + PARF_PHY_CTRL);
	val |= PHY_TEST_PWR_DOWN;
	writel(val, pcie->parf + PARF_PHY_CTRL);

	clk_bulk_disable_unprepare(res->num_clks, res->clks);
	regulator_bulk_disable(ARRAY_SIZE(res->supplies), res->supplies);
}

static int qcom_pcie_init_2_3_2(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_3_2 *res = &pcie->res.v2_3_2;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(res->supplies), res->supplies);
	if (ret < 0) {
		dev_err(dev, "cannot enable regulators\n");
		return ret;
	}

	ret = clk_bulk_prepare_enable(res->num_clks, res->clks);
	if (ret) {
		dev_err(dev, "cannot prepare/enable clocks\n");
		regulator_bulk_disable(ARRAY_SIZE(res->supplies), res->supplies);
		return ret;
	}

	return 0;
}

static int qcom_pcie_post_init_2_3_2(struct qcom_pcie *pcie)
{
	u32 val;

	/* Force PHY out of lowest power state */
	val = readl(pcie->parf + PARF_PHY_CTRL);
	val &= ~PHY_TEST_PWR_DOWN;
	writel(val, pcie->parf + PARF_PHY_CTRL);

	qcom_pcie_configure_dbi_base(pcie);

	/* MAC PHY_POWERDOWN MUX DISABLE  */
	val = readl(pcie->parf + PARF_SYS_CTRL);
	val &= ~MAC_PHY_POWERDOWN_IN_P2_D_MUX_EN;
	writel(val, pcie->parf + PARF_SYS_CTRL);

	val = readl(pcie->parf + PARF_MHI_CLOCK_RESET_CTRL);
	val |= BYPASS;
	writel(val, pcie->parf + PARF_MHI_CLOCK_RESET_CTRL);

	val = readl(pcie->parf + PARF_AXI_MSTR_WR_ADDR_HALT_V2);
	val |= EN;
	writel(val, pcie->parf + PARF_AXI_MSTR_WR_ADDR_HALT_V2);

	qcom_pcie_set_slot_nccs(pcie->pci);

	return 0;
}

static int qcom_pcie_get_resources_2_4_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_4_0 *res = &pcie->res.v2_4_0;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	bool is_ipq = of_device_is_compatible(dev->of_node, "qcom,pcie-ipq4019");
	int ret;

	res->num_clks = devm_clk_bulk_get_all(dev, &res->clks);
	if (res->num_clks < 0) {
		dev_err(dev, "Failed to get clocks\n");
		return res->num_clks;
	}

	res->resets[0].id = "axi_m";
	res->resets[1].id = "axi_s";
	res->resets[2].id = "axi_m_sticky";
	res->resets[3].id = "pipe_sticky";
	res->resets[4].id = "pwr";
	res->resets[5].id = "ahb";
	res->resets[6].id = "pipe";
	res->resets[7].id = "axi_m_vmid";
	res->resets[8].id = "axi_s_xpu";
	res->resets[9].id = "parf";
	res->resets[10].id = "phy";
	res->resets[11].id = "phy_ahb";

	res->num_resets = is_ipq ? 12 : 6;

	ret = devm_reset_control_bulk_get_exclusive(dev, res->num_resets, res->resets);
	if (ret < 0)
		return ret;

	return 0;
}

static void qcom_pcie_deinit_2_4_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_4_0 *res = &pcie->res.v2_4_0;
	u32 val;

	/* Force PHY to lowest power state*/
	val = readl(pcie->parf + PARF_PHY_CTRL);
	val |= PHY_TEST_PWR_DOWN;
	writel(val, pcie->parf + PARF_PHY_CTRL);

	reset_control_bulk_assert(res->num_resets, res->resets);
	clk_bulk_disable_unprepare(res->num_clks, res->clks);
}

static int qcom_pcie_init_2_4_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_4_0 *res = &pcie->res.v2_4_0;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	int ret;

	ret = reset_control_bulk_assert(res->num_resets, res->resets);
	if (ret < 0) {
		dev_err(dev, "cannot assert resets\n");
		return ret;
	}

	usleep_range(10000, 12000);

	ret = reset_control_bulk_deassert(res->num_resets, res->resets);
	if (ret < 0) {
		dev_err(dev, "cannot deassert resets\n");
		return ret;
	}

	usleep_range(10000, 12000);

	ret = clk_bulk_prepare_enable(res->num_clks, res->clks);
	if (ret) {
		reset_control_bulk_assert(res->num_resets, res->resets);
		return ret;
	}

	return 0;
}

static int qcom_pcie_get_resources_2_3_3(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_3_3 *res = &pcie->res.v2_3_3;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	int ret;

	res->num_clks = devm_clk_bulk_get_all(dev, &res->clks);
	if (res->num_clks < 0) {
		dev_err(dev, "Failed to get clocks\n");
		return res->num_clks;
	}

	res->rst[0].id = "axi_m";
	res->rst[1].id = "axi_s";
	res->rst[2].id = "pipe";
	res->rst[3].id = "axi_m_sticky";
	res->rst[4].id = "sticky";
	res->rst[5].id = "ahb";
	res->rst[6].id = "sleep";

	ret = devm_reset_control_bulk_get_exclusive(dev, ARRAY_SIZE(res->rst), res->rst);
	if (ret < 0)
		return ret;

	return 0;
}

static void qcom_pcie_deinit_2_3_3(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_3_3 *res = &pcie->res.v2_3_3;
	u32 val;

	/* Force PHY to lowest power state */
	val = readl(pcie->parf + PARF_PHY_CTRL);
	val |= PHY_TEST_PWR_DOWN;
	writel(val, pcie->parf + PARF_PHY_CTRL);

	clk_bulk_disable_unprepare(res->num_clks, res->clks);
}

static int qcom_pcie_init_2_3_3(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_3_3 *res = &pcie->res.v2_3_3;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	int ret;

	ret = reset_control_bulk_assert(ARRAY_SIZE(res->rst), res->rst);
	if (ret < 0) {
		dev_err(dev, "cannot assert resets\n");
		return ret;
	}

	usleep_range(2000, 2500);

	ret = reset_control_bulk_deassert(ARRAY_SIZE(res->rst), res->rst);
	if (ret < 0) {
		dev_err(dev, "cannot deassert resets\n");
		return ret;
	}

	/*
	 * Don't have a way to see if the reset has completed.
	 * Wait for some time.
	 */
	usleep_range(2000, 2500);

	ret = clk_bulk_prepare_enable(res->num_clks, res->clks);
	if (ret) {
		dev_err(dev, "cannot prepare/enable clocks\n");
		goto err_assert_resets;
	}

	return 0;

err_assert_resets:
	/*
	 * Not checking for failure, will anyway return
	 * the original failure in 'ret'.
	 */
	reset_control_bulk_assert(ARRAY_SIZE(res->rst), res->rst);

	return ret;
}

static int qcom_pcie_post_init_2_3_3(struct qcom_pcie *pcie)
{
	struct dw_pcie *pci = pcie->pci;
	u16 offset = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	u32 val;

	/* Force PHY out of lowest power state */
	val = readl(pcie->parf + PARF_PHY_CTRL);
	val &= ~PHY_TEST_PWR_DOWN;
	writel(val, pcie->parf + PARF_PHY_CTRL);

	qcom_pcie_configure_dbi_atu_base(pcie);

	writel(MST_WAKEUP_EN | SLV_WAKEUP_EN | MSTR_ACLK_CGC_DIS
		| SLV_ACLK_CGC_DIS | CORE_CLK_CGC_DIS |
		AUX_PWR_DET | L23_CLK_RMV_DIS | L1_CLK_RMV_DIS,
		pcie->parf + PARF_SYS_CTRL);
	writel(0, pcie->parf + PARF_Q2A_FLUSH);

	writel(PCI_COMMAND_MASTER, pci->dbi_base + PCI_COMMAND);

	dw_pcie_dbi_ro_wr_en(pci);

	writel(PCIE_CAP_SLOT_VAL, pci->dbi_base + offset + PCI_EXP_SLTCAP);

	val = readl(pci->dbi_base + offset + PCI_EXP_LNKCAP);
	val &= ~PCI_EXP_LNKCAP_ASPMS;
	writel(val, pci->dbi_base + offset + PCI_EXP_LNKCAP);

	writel(PCI_EXP_DEVCTL2_COMP_TMOUT_DIS, pci->dbi_base + offset +
		PCI_EXP_DEVCTL2);

	dw_pcie_dbi_ro_wr_dis(pci);

	return 0;
}

static int qcom_pcie_get_resources_2_7_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_7_0 *res = &pcie->res.v2_7_0;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	int ret;

	res->rst = devm_reset_control_array_get_exclusive(dev);
	if (IS_ERR(res->rst))
		return PTR_ERR(res->rst);

	res->supplies[0].supply = "vdda";
	res->supplies[1].supply = "vddpe-3v3";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(res->supplies),
				      res->supplies);
	if (ret)
		return ret;

	res->num_clks = devm_clk_bulk_get_all(dev, &res->clks);
	if (res->num_clks < 0) {
		dev_err(dev, "Failed to get clocks\n");
		return res->num_clks;
	}

	return 0;
}

/*
 * WCN7880 cold-boot-cal RDDM root-cause (2026-06-24, GATE-1 source diff vs the
 * downstream OnePlus SM8750 pci-msm.c + sun-pcie.dtsi): the downstream driver
 * runs the WCN PCIe RC under CESTA ("qcom,drv-name = cesta", a "pcie_sm" reg
 * region @0x01D07000, a *_PCIE_CRM_HW_* interconnect). The CESTA hardware
 * state-machine answers the WCN amss cold-boot-cal phy_off l1ss-sleep / CLKREQ
 * handshake (pcie_core_ctrl_l1ss_sleep_mode). Mainline pcie-qcom has no CESTA,
 * so when the cal fw requests PCIe l1ss-sleep nothing answers ("l1ss_sleep
 * unvoted by client") -> PCSS watchdog -> NoC error ~270ms into cal.
 *
 * Hypothesis-#1 (clear CLKREQ_SEL so CLKREQ uses the direct gpio) was REFUTED on
 * v442: PARF_CESTA_CONFIG reset = 0x80000000, CLKREQ_SEL already 0, cal RDDM
 * byte-identical. So routing alone is not enough -- the CESTA state-machine has
 * to actually run.
 *
 * v443 SM-only (load SM + route CLKREQ, skip CRM votes) was REFUTED: cal RDDM
 * byte-identical. v444 diagnostic boot then showed WHY: the pcie_crm HW block is
 * ALIVE (CRM v2.0 ENABLE=1, chans=2 hwdrvs=2 perfol_vcd=2 pwrst/ch=5) but its
 * PCIe-resource CURR_PERF_OL = 0 (D3COLD) -- the SM runs (PC_VAL!=0) yet has no
 * perf-ol vote to act on, so it can never drive the link's l1ss state. The fix is
 * to push the perf-ol vote the way the downstream crm_write_pwr_states() does.
 *
 * Theory (now fully coherent): the fw cold-boot-cal phy_off path deasserts CLKREQ
 * to request l1ss-sleep and polls for the host "client" to grant it. Only when the
 * host CESTA is voted to DRV_STATE = {POWER_STATE_0=L1SS, POWER_STATE_1=GEN3} does
 * the SM grant l1ss on CLKREQ-deassert. With no vote (or D0={GEN3,GEN3}) the host
 * pins the link up, never grants l1ss -> the fw polls to its fixed PCSS watchdog
 * (~330ms, host-invariant -- matching every refuted generic ASPM/L1ss/retrain
 * lever). Gated qcom_pcie_wcn_cesta (default OFF); recover a wedged boot via the
 * previous EFI at rEFInd.
 *
 * DEFAULT OFF (2026-06-25): the full CESTA/CRM machinery was faithfully
 * replicated (SM runs, CRM perf-ol DRV vote verified landing CURR_PERF_OL->GEN3)
 * and REFUTED as a cal-RDDM fix (ledger C3) -- the host PCIe link-power plane is
 * exonerated. Worse, its init poking destabilises the WCN EP link-training: the
 * l1ss-timeout/SM gating intermittently leaves the EP un-enumerated ("Device not
 * found", no ath12k, no cal). Since it neither fixes the bug nor is load-bearing,
 * keep it OFF so the WCN PCIe reverts to the reliable mainline bring-up. Flip
 * pcie_qcom.qcom_pcie_wcn_cesta=1 only to re-run the (refuted) CESTA A/B.
 */
/*
 * v458: default-ON as the HW clock-manager that makes lever D3 (HW-gated PCIe AXI
 * clocks, qcom_pcie_wcn_clk_hwgate) SAFE. D3 alone (v457) dropped the SW clock
 * vote and SoC-reset during WCN link bring-up: with the AXI clocks HW-gated and
 * nothing managing the clock requests, the DBI/link access hung the NoC. Android
 * gates those clocks AND runs the RC under CESTA -- the SM grants the clock on
 * CLKREQ#, so accesses don't hang. CESTA alone (C3) was refuted as a cal fix and
 * D3 alone breaks boot; this is the Android-faithful COMBINATION (full-replay).
 * A/B via pcie_qcom.qcom_pcie_wcn_cesta=0 (also disable D3 then).
 */
static bool qcom_pcie_wcn_cesta = true;
module_param(qcom_pcie_wcn_cesta, bool, 0644);
MODULE_PARM_DESC(qcom_pcie_wcn_cesta,
	"SM8750 WCN PCIe: load+start the CESTA pcie_sm SM + CRM perf-ol vote + route CLKREQ/l1ss to it (HW clock-manager that makes D3 gated-AXI-clocks safe; default 1)");

/*
 * CESTA usage scenario to vote: 1 = DRV {L1SS,GEN3} (CESTA drops to L1SS perf on
 * CLKREQ deassert); 0 = D0 {GEN3,GEN3} (CLKREQ deassert drops NOTHING; link
 * resources pinned while host active).
 *
 * DEFAULT FLIPPED 1->0 (lever #23, 2026-07-08 Android PARF/source ground truth):
 * downstream msm_pcie keeps the ACTIVE host in the D0 map -- cesta_map[D0] =
 * {MAX,MAX} at boot, and the BW_REQ Gen1 scale does map_save(GEN1) +
 * map_apply(D0) = {GEN1,GEN1}. The DRV map {L1SS,GEN1} (+ l1ss-to timer + CLKREQ
 * wake-GPIO) is applied ONLY at cnss DRV suspend (host hands the link away).
 * Our old DRV{L1SS,*} vote during active mission let the CESTA SM power-drop the
 * link the instant the fw deasserts CLKREQ at its dev-SOL-LOW l1ss_sleep entry
 * (mission, fw_ready#2+~250ms) -- mid-L1.2-handshake, under live MHI traffic ->
 * fw "polling for L1ss failed" -> PCSS watchdog -> NOC 437:0x8 RDDM. On Android
 * the same fw CLKREQ deassert hits PS0=GEN1 (no drop) and mission lives.
 * Settable via cmdline pcie_qcom.qcom_pcie_wcn_cesta_state= for A/B.
 */
static int qcom_pcie_wcn_cesta_state;
module_param(qcom_pcie_wcn_cesta_state, int, 0644);
MODULE_PARM_DESC(qcom_pcie_wcn_cesta_state,
	"SM8750 WCN PCIe CESTA perf-ol vote: 0=D0{GEN3,GEN3} (default, Android-faithful active-host map), 1=DRV{L1SS,GEN3}");

/*
 * Arm the CESTA l1ss-timeout timer at .init (default OFF). At init it fires
 * during EP link-training idle and breaks enumeration (v446); ath12k arms it at
 * cal-start instead. Kept gated for A/B via pcie_qcom.qcom_pcie_wcn_cesta_l1ss_to=1.
 */
static bool qcom_pcie_wcn_cesta_l1ss_to;
module_param(qcom_pcie_wcn_cesta_l1ss_to, bool, 0644);
MODULE_PARM_DESC(qcom_pcie_wcn_cesta_l1ss_to,
	"SM8750 WCN PCIe: arm the CESTA l1ss-timeout timer at init (default 0; breaks EP enum)");

/* CESTA SM instruction + branch sequence, from the downstream SM8750
 * sun-pcie.dtsi pcie0 (qcom,pcie-sm-seq / qcom,pcie-sm-branch-seq). */
static const u32 qcom_pcie_wcn_sm_seq[] = {
	0x1c018081, 0x70074002, 0x50028000, 0x28007003, 0x80804002,
	0x70021c01, 0x18001802, 0x70005000, 0x10004000, 0x80814002,
	0x18001c01, 0x1c018080, 0x00000100,
};
static const u32 qcom_pcie_wcn_sm_branch[] = {
	0x4, 0x1c, 0x24, 0x2c, 0x0, 0x0, 0x0,
};

/*
 * Load + start the CESTA state-machine for the SM8750 WCN RC and route the
 * CLKREQ + L1ss-sleep timeout to it. Called from .init before link training,
 * matching the downstream order (cesta_init at probe, before enable_link).
 */
/*
 * Read-only probe of the pcie_crm HW block. Logs version/config/enable and the
 * HW-DRV0 perf-ol status so we can tell whether the CRM is alive and what the
 * CESTA SM's votes would land on. No writes — purely diagnostic for now.
 */
static void qcom_pcie_wcn_crm_probe(struct qcom_pcie *pcie)
{
	struct device *dev = pcie->pci->dev;
	void __iomem *common, *base, *crmc;
	u32 cfg1, cfg2, en, ver;

	common = ioremap(PCIE_CRM_COMMON_PHYS, PCIE_CRM_COMMON_SIZE);
	if (!common) {
		dev_warn(dev, "wcn7880: CRM: ioremap(common) failed\n");
		return;
	}
	ver  = readl(common + CRM_COMMON_VERSION);
	cfg1 = readl(common + CRM_COMMON_CFG_PARAM_1);
	cfg2 = readl(common + CRM_COMMON_CFG_PARAM_2);
	en   = readl(common + CRM_COMMON_ENABLE);
	dev_info(dev,
		 "wcn7880: CRM v%u.%u ENABLE=0x%08x CFG1=0x%08x (chans=%u hwdrvs=%u swdrvs=%u perfol_vcd=%u pwrst/ch=%u) CFG2=0x%08x\n",
		 (ver >> 16) & 0xff, (ver >> 8) & 0xff, en, cfg1,
		 (cfg1 >> 4) & 0xf, (cfg1 >> 16) & 0xf, (cfg1 >> 20) & 0xf,
		 (cfg1 >> 8) & 0xf, cfg1 & 0xf, cfg2);
	iounmap(common);

	/* HW-DRV0 perf-ol status (resource 0) lives in the "base" region. */
	base = ioremap(PCIE_CRM_BASE_PHYS, PCIE_CRM_BASE_SIZE);
	if (base) {
		dev_info(dev,
			 "wcn7880: CRM HWDRV0 perfol STATUS=0x%08x PWR_IDX_STATUS=0x%08x\n",
			 readl(base + CRM_HWDRV_PERFOL_STATUS),
			 readl(base + CRM_HWDRV_PERFOL_PWR_IDX_STATUS));
		iounmap(base);
	}

	/* crmc aggregated/current perf-ol for the PCIe resource. */
	crmc = ioremap(PCIE_CRM_CRMC_PHYS, PCIE_CRM_CRMC_SIZE);
	if (crmc) {
		dev_info(dev,
			 "wcn7880: CRM crmc AGGR_PERF_OL=0x%08x CURR_PERF_OL=0x%08x\n",
			 readl(crmc + CRM_CRMC_AGGR_PERF_OL),
			 readl(crmc + CRM_CRMC_CURR_PERF_OL));
		iounmap(crmc);
	}
}

/*
 * Push the perf-ol vote for the WCN PCIe resource into the CRM HW DRV0, the way
 * the downstream crm_write_pwr_states()/crm_flush_cache() does: write the per-
 * power-state perf levels into the free channel, then switch to it. This gives
 * the CESTA SM a valid l1ss/Gen3 mapping so it can grant the fw's l1ss-sleep.
 *
 *   crm_get_offset(perf_ol, ch, res) = PWR_STn + ch*0x14 + res*0x2c  (drv0 @ 0)
 *   resource 0 = the WCN PCIe link (rc_idx 0). DRV vote = {L1SS, GEN3}.
 */
/*
 * Low-level CRM perf-ol writer: write the per-power-state perf levels {st0, st1}
 * into the free HW-DRV0 channel and switch to it (== crm_write_pwr_states +
 * crm_switch_channel for drv0/resource0). @dev may be NULL (BW_REQ re-vote path
 * has no qcom_pcie handle); falls back to pr_*.
 */
static void __qcom_pcie_wcn_crm_write(struct device *dev, const char *tag,
				      u32 st0, u32 st1)
{
	void __iomem *base, *crmc;
	u32 map[CRM_NUM_PWR_STATES] = { 0 };
	u32 chn_update, ch, busy = 0;
	int res, j, retry = 100;

	map[0] = st0;
	map[1] = st1;

	base = ioremap(PCIE_CRM_BASE_PHYS, PCIE_CRM_BASE_SIZE);
	if (!base) {
		if (dev)
			dev_warn(dev, "wcn7880: CRM vote: ioremap(base) failed\n");
		else
			pr_warn("wcn7880: CRM vote: ioremap(base) failed\n");
		return;
	}

	/* crm_get_channel(CHN_FREE): pick the unused channel (drv0 chn regs @ base). */
	chn_update = readl(base + CRM_CHN_UPDATE);
	if (!chn_update)
		ch = 0;
	else if (chn_update & BIT(0))		/* CH0 busy -> use CH1 */
		ch = 1;
	else					/* CH1 busy (or other) -> use CH0 */
		ch = 0;

	/* crm_flush_cache(): perf-ol VCD, resources 0..N, pwr states 0..4. */
	for (res = 0; res < CRM_NUM_PERFOL_RES; res++) {
		for (j = 0; j < CRM_NUM_PWR_STATES; j++) {
			u32 data = (res == 0) ? map[j] : 0;
			u32 off = CRM_PERFOL_PWR_ST0 + j * CRM_PERFOL_PWR_ST_STRIDE +
				  ch * CRM_PERFOL_CHN_DISTANCE +
				  res * CRM_PERFOL_RES_DISTANCE;

			writel(data, base + off);
		}
	}

	/* crm_switch_channel(): request the new channel, poll CHN_BUSY. */
	writel(BIT(ch), base + CRM_CHN_UPDATE);
	do {
		busy = readl(base + CRM_CHN_BUSY) & BIT(ch);
		udelay(100);
	} while (busy != BIT(ch) && --retry);

	if (dev)
		dev_info(dev,
			 "wcn7880: CRM perf-ol vote applied: %s ch=%u ST0=%u ST1=%u switch_retry=%d busy=0x%x\n",
			 tag, ch, map[0], map[1], retry, busy);
	else
		pr_info("wcn7880: CRM perf-ol vote applied: %s ch=%u ST0=%u ST1=%u switch_retry=%d busy=0x%x\n",
			tag, ch, map[0], map[1], retry, busy);
	iounmap(base);

	/* Verify the vote landed: crmc CURR_PERF_OL for resource 0 should track it. */
	crmc = ioremap(PCIE_CRM_CRMC_PHYS, PCIE_CRM_CRMC_SIZE);
	if (crmc) {
		if (dev)
			dev_info(dev,
				 "wcn7880: CRM after vote: AGGR_PERF_OL=0x%08x CURR_PERF_OL=0x%08x\n",
				 readl(crmc + CRM_CRMC_AGGR_PERF_OL),
				 readl(crmc + CRM_CRMC_CURR_PERF_OL));
		else
			pr_info("wcn7880: CRM after vote: AGGR_PERF_OL=0x%08x CURR_PERF_OL=0x%08x\n",
				readl(crmc + CRM_CRMC_AGGR_PERF_OL),
				readl(crmc + CRM_CRMC_CURR_PERF_OL));
		iounmap(crmc);
	}
}

static void qcom_pcie_wcn_crm_vote(struct qcom_pcie *pcie)
{
	struct device *dev = pcie->pci->dev;

	if (qcom_pcie_wcn_cesta_state)		/* DRV {L1SS, GEN3} */
		__qcom_pcie_wcn_crm_write(dev, "DRV{L1SS,GEN3}",
					  CRM_PERF_LVL_L1SS, CRM_PERF_LVL_GEN3);
	else					/* D0  {GEN3, GEN3} */
		__qcom_pcie_wcn_crm_write(dev, "D0{GEN3,GEN3}",
					  CRM_PERF_LVL_GEN3, CRM_PERF_LVL_GEN3);
}

/*
 * BW_REQ re-vote: when the WCN cold-boot-cal firmware emits its MHI BW_REQ asking
 * for Gen1 (the last host<->fw exchange before the phy_off/l1ss-sleep RDDM), the
 * downstream host does NOT just ack it -- msm_pcie_set_link_bandwidth() ->
 * msm_pcie_scale_link_bandwidth() -> cesta_map_save(GEN1) + cesta_map_apply(D0)
 * re-votes the CRM perf-ol for the active/DRV power states DOWN from GEN3 to GEN1.
 * Mainline pcie-qcom holds the static GEN3 vote (qcom_pcie_wcn_crm_vote) and never
 * re-scales, so the CESTA SM services the fw's commanded l1ss-sleep at the GEN3
 * perf level while the fw expects Gen1 -> the fw's pcie_core_check_pcie_clock_on
 * poll fails -> NoC -> RDDM. Replicate the GEN1 re-vote here; ath12k's bw_scale
 * work calls it on BW_REQ. WCN7880-specific (fixed CRM phys; rc_idx 0).
 */
void qcom_pcie_wcn_crm_revote_gen1(void)
{
	if (!qcom_pcie_wcn_cesta)
		return;
	/* Follow the boot vote's scenario (lever #23): Android's BW_REQ path is
	 * map_save(GEN1) + map_apply(D0) = {GEN1,GEN1} -- PS0 stays full-on so the
	 * fw's later CLKREQ deassert (mission l1ss_sleep) drops nothing. The old
	 * unconditional DRV{L1SS,GEN1} re-vote is kept for the cesta_state=1 A/B.
	 */
	if (qcom_pcie_wcn_cesta_state)
		__qcom_pcie_wcn_crm_write(NULL, "BW_REQ DRV{L1SS,GEN1}",
					  CRM_PERF_LVL_L1SS, CRM_PERF_LVL_GEN1);
	else
		__qcom_pcie_wcn_crm_write(NULL, "BW_REQ D0{GEN1,GEN1}",
					  CRM_PERF_LVL_GEN1, CRM_PERF_LVL_GEN1);
}
EXPORT_SYMBOL(qcom_pcie_wcn_crm_revote_gen1);

static void qcom_pcie_wcn_cesta_enable(struct qcom_pcie *pcie)
{
	struct device *dev = pcie->pci->dev;
	void __iomem *sm;
	u32 v;
	int i;

	if (!qcom_pcie_wcn_cesta ||
	    !of_device_is_compatible(dev->of_node, "qcom,pcie-sm8750"))
		return;

	sm = ioremap(PCIE_SM_PHYS, PCIE_SM_SIZE);
	if (!sm) {
		dev_warn(dev, "wcn7880: CESTA: ioremap(pcie_sm) failed\n");
		return;
	}

	/* Clear the PWR_CTRL / PWR_CTRL_MASK overrides (rc_idx 0). */
	writel(0, sm + PCIE_SM_PWR_CTRL);
	writel(0, sm + PCIE_SM_PWR_CTRL_MASK);

	/* Load the SM instruction sequence (base offset 0) and branch sequence. */
	for (i = 0; i < ARRAY_SIZE(qcom_pcie_wcn_sm_seq); i++)
		writel(qcom_pcie_wcn_sm_seq[i], sm + 4 * i);
	for (i = 0; i < ARRAY_SIZE(qcom_pcie_wcn_sm_branch); i++)
		writel(qcom_pcie_wcn_sm_branch[i],
		       sm + PCIE_SM_BRANCH_OFFSET + 4 * i);

	/* Start the state-machine. */
	v = readl(sm + PCIE_SM_START_OFFSET);
	writel(v | BIT(0), sm + PCIE_SM_START_OFFSET);
	dev_info(dev,
		 "wcn7880: CESTA SM loaded+started (start=0x%08x; CRM votes skipped)\n",
		 readl(sm + PCIE_SM_START_OFFSET));
	/* Is the SM actually executing? PC_VAL should advance if it runs. */
	dev_info(dev,
		 "wcn7880: SM status: PC_VAL=0x%08x PWR_CTRL_STATUS=0x%08x CLKREQ_GATE_REQ=0x%08x\n",
		 readl(sm + PCIE_SM_PC_VAL),
		 readl(sm + PCIE_SM_PWR_CTRL_STATUS),
		 readl(sm + PCIE_SM_CLKREQ_GATE_REQ_STATUS));

	iounmap(sm);

	/*
	 * Probe the pcie_crm HW block (alive?), then push the perf-ol vote so the
	 * CESTA SM has a valid l1ss/Gen3 mapping to grant the fw's l1ss-sleep.
	 */
	qcom_pcie_wcn_crm_probe(pcie);
	qcom_pcie_wcn_crm_vote(pcie);

	/* Route CLKREQ + the L1ss-sleep timeout to the CESTA SM (Android value). */
	v = readl(pcie->parf + PARF_CESTA_CONFIG);
	dev_info(dev, "wcn7880: PARF_CESTA_CONFIG before=0x%08x\n", v);
	v |= PARF_CESTA_CLKREQ_SEL | PARF_CESTA_L1SUB_TIMEOUT_EXT_INT_EN;
	writel(v, pcie->parf + PARF_CESTA_CONFIG);
	dev_info(dev,
		 "wcn7880: PARF_CESTA_CONFIG after=0x%08x (CLKREQ+l1ss-timeout -> CESTA SM)\n",
		 readl(pcie->parf + PARF_CESTA_CONFIG));

	/*
	 * Arm the l1ss-sleep timeout timer (msm_pcie_cesta_enable_l1ss_to): write
	 * RESET, then RESET|ticks. This is what makes the controller hand the SM
	 * the l1ss-TO so it actually enters l1ss-sleep and grants the fw request.
	 *
	 * DEFAULT OFF at init: armed here, it fires on the idle gaps DURING the WCN
	 * EP's link training and the SM gates the link -> the EP never enumerates
	 * ("Device not found", no ath12k, no cal -- proven on v446). The timer must
	 * instead be armed AFTER enumeration, right before cold-boot-cal, from
	 * ath12k (ath12k_wcn7880_arm_cesta_l1ss_to(), which maps PARF 0x01c00000).
	 */
	if (qcom_pcie_wcn_cesta_l1ss_to) {
		writel(PARF_L1SUB_AHB_CLK_MAX_TIMER_RESET,
		       pcie->parf + PARF_L1SUB_AHB_CLK_MAX_TIMER);
		writel(PARF_L1SUB_AHB_CLK_MAX_TIMER_RESET | CESTA_L1SS_TIMEOUT_TICKS,
		       pcie->parf + PARF_L1SUB_AHB_CLK_MAX_TIMER);
		dev_info(dev, "wcn7880: CESTA l1ss-timeout armed @init @PARF+0x180 = 0x%08x\n",
			 readl(pcie->parf + PARF_L1SUB_AHB_CLK_MAX_TIMER));
	}
}

/*
 * SM8750 WCN PCIe RC — keep the PCIe core clock running through L1/L1ss.
 *
 * Cold-boot rwmmio diff (captures/wcn-fw-cal-trace/android-coldboot-mmio/) of the
 * working OxygenOS cal-powerup vs ours: the ONLY un-refuted PARF divergence is
 * PARF_SYS_CTRL — Android = 0x365E, ours (mainline 2_7_0 RMW) = 0x361C. The two
 * extra bits Android sets are CORE_CLK_CGC_DIS (BIT6, disable the core clock
 * gating-cell) and L1_CLK_RMV_DIS (BIT1, disable clock removal while the link is
 * in L1). Both KEEP the PCIe core clock alive in low-power; mainline's older
 * 2_3_3 path sets them, but the 2_7_0 path sm8750 uses does not. Downstream
 * pci-msm.c writes PARF_SYS_CTRL=0x365E verbatim (line 6014).
 *
 * This is dead-on the decoded WCN cold-boot-cal RDDM: the cal fw enters a routine
 * inter-step l1ss-sleep, then pcie_core_check_pcie_clock_on reads
 * GCC_SNOC_PCIE_AXI_CBCR clk_off (clock removed in L1 on our side) -> touches a
 * PCIE_NOC target with the clock gated -> "NOC ERROR DETECTED 0x8" -> PCSS
 * watchdog -> RDDM. Keeping the core clock on (as Android does) should let that
 * first l1ss-sleep complete. Also replays the two other cold-boot PARF writes
 * Android does that we don't (INT_ALL_MASK, CFG_BITS_3) for full parity.
 *
 * Gated default-ON (full-replay-then-prune); A/B via pcie_qcom.qcom_pcie_wcn_clk_keepon=0.
 */
static bool qcom_pcie_wcn_clk_keepon = true;
module_param(qcom_pcie_wcn_clk_keepon, bool, 0644);
MODULE_PARM_DESC(qcom_pcie_wcn_clk_keepon,
	"SM8750 WCN: keep PCIe core clock on in L1 (PARF_SYS_CTRL 0x365E) like Android (default on)");

static void qcom_pcie_wcn_clk_keepon_apply(struct qcom_pcie *pcie)
{
	struct device *dev = pcie->pci->dev;
	u32 v;

	if (!qcom_pcie_wcn_clk_keepon ||
	    !of_device_is_compatible(dev->of_node, "qcom,pcie-sm8750"))
		return;

	/* PARF_SYS_CTRL: add the two clock-keep-on bits -> matches Android 0x365E. */
	v = readl(pcie->parf + PARF_SYS_CTRL);
	dev_info(dev, "wcn7880: PARF_SYS_CTRL before=0x%08x\n", v);
	v |= CORE_CLK_CGC_DIS | L1_CLK_RMV_DIS;
	writel(v, pcie->parf + PARF_SYS_CTRL);
	dev_info(dev, "wcn7880: PARF_SYS_CTRL after=0x%08x (clock kept on in L1)\n",
		 readl(pcie->parf + PARF_SYS_CTRL));

	/* Replay the two other cold-boot PARF writes Android does (full parity).
	 *
	 * Lever #26 (b25, 2026-07-08): the Android cold-boot rwmmio trace writes
	 * CFG_BITS_3 TWICE back-to-back right before LTSSM_EN — 0x68 then 0x168
	 * (bit8 added; only writes to 0x2c4 in the whole boot). Our replay stopped
	 * at the first write, so bit8 was never set — the ONLY missing PARF init
	 * bit after the full b24/b25 parity audit (PHY table 128/128 byte-equal,
	 * PARF init otherwise equal; 0x508 INT_ALL_2_MASK BW_MGT skipped
	 * deliberately: msm's own retrain-IRQ plumbing, we poll). Suspected the
	 * source of the standing read-only divergences CLKREQ_OVERRIDE
	 * 0x10-vs-0x20 and PM_STTS[10:8] 4-vs-3, i.e. the CLKREQ/refclk plane the
	 * fw's mission l1ss_sleep entry dies on (NOC 437:0x8 25ms after
	 * dev-SOL-LOW, L1/L1.2 entry retry loop).
	 */
	writel(0x7f80d202, pcie->parf + PARF_INT_ALL_MASK);
	writel(0x68, pcie->parf + PARF_CFG_BITS_3);
	writel(0x168, pcie->parf + PARF_CFG_BITS_3);
	dev_info(dev, "wcn7880: PARF_INT_ALL_MASK=0x%08x PARF_CFG_BITS_3=0x%08x\n",
		 readl(pcie->parf + PARF_INT_ALL_MASK),
		 readl(pcie->parf + PARF_CFG_BITS_3));
}

/*
 * NOTE (lever D2): the SM8750 WCN PCIe RC votes its PCIe->mem_noc->LLCC->DDR
 * bandwidth via the OPP table (use_pm_opp=true), NOT via icc_mem (icc_init is
 * skipped on this platform). The avg-bw FLOOR that keeps the NoC->DDR route
 * clocked through the idle/L1.2 cold-boot-cal window is therefore set in the
 * DTS pcie0_opp_table (opp-avg-kBps), not here. See STATE-DIFF-FINDINGS.
 */

/*
 * Lever D3 (WCN cold-boot-cal): let the HWCG-capable PCIe AXI clocks GATE during
 * the idle cal window like Android, instead of SW-pinning them on.
 *
 * BROAD STATE diff: in-cal `clk_summary` shows Android's `gcc_pcie_0_{slv,mstr}_axi`
 * + `cfg_ahb` at **hw_en=N** (HW-gated, link idle), while ours sit at **hw_en=Y**
 * because mainline `qcom_pcie_init_2_7_0` does `clk_bulk_prepare_enable` on ALL DT
 * clocks -- holding the CBCR SW-enable so HWCG never gates them. These three CBCRs
 * already carry HWCG (gcc-sm8750 hwcg_reg 0x6b020/030/040, matches the downstream
 * gcc), so DROPPING our SW vote lets the hardware gate them when the AXI block is
 * idle (and auto-ungate on activity -- safe: HWCG keeps them on whenever the block
 * is busy, so enum/datapath are unaffected). Matches Android's hw_en=N during cal.
 *
 * (The non-HWCG PCIe clocks -- aux/pipe/phy_rchng/slv_q2a -- are left SW-enabled;
 * Android gates those via CESTA HW clock management, which is lever C3, already
 * replicated + refuted. So this is the only independently-testable slice of D3.)
 * Gated default-ON (full-replay); A/B via pcie_qcom.qcom_pcie_wcn_clk_hwgate=0.
 */
static bool qcom_pcie_wcn_clk_hwgate = true;
module_param(qcom_pcie_wcn_clk_hwgate, bool, 0644);
MODULE_PARM_DESC(qcom_pcie_wcn_clk_hwgate,
	"SM8750 WCN: drop the SW vote on the HWCG PCIe AXI clocks so they HW-gate during idle cal like Android (default on)");

/*
 * Full clock-release (v459, full-replay of Android): drop the SW vote on ALL
 * eight controller PCIe clocks, not just the original three. Android holds
 * enable_count=0 on every gcc_pcie_0 clock during cold-boot cal (managed by
 * HWCG/CESTA, link parked in L1.2); D3 originally dropped only bus_master/
 * bus_slave/cfg. The other five (aux/slave_q2a/ddrss_sf_tbu/noc_aggr/
 * cnoc_sf_axi) now all carry HWCG in gcc-sm8750 (aux + slv_q2a HWCG added in
 * the same change; the rest already had it), so dropping their vote lets the
 * hardware gate them when the link idles and auto-ungate on activity -- the
 * proven D3 pattern, extended to the whole controller clock set.
 */
static const char * const qcom_pcie_wcn_hwgate_clks[] = {
	"aux", "cfg", "bus_master", "bus_slave",
	"slave_q2a", "ddrss_sf_tbu", "noc_aggr", "cnoc_sf_axi",
};

static bool qcom_pcie_wcn_clk_is_hwgate(const char *id)
{
	int i;

	if (!id)
		return false;
	for (i = 0; i < ARRAY_SIZE(qcom_pcie_wcn_hwgate_clks); i++)
		if (!strcmp(id, qcom_pcie_wcn_hwgate_clks[i]))
			return true;
	return false;
}

/* Drop the SW enable on the HWCG AXI clocks (called after clk_bulk_prepare_enable). */
static void qcom_pcie_wcn_clk_hwgate_apply(struct qcom_pcie *pcie,
					   struct clk_bulk_data *clks, int num)
{
	struct device *dev = pcie->pci->dev;
	int i, n = 0;

	if (!qcom_pcie_wcn_clk_hwgate ||
	    !of_device_is_compatible(dev->of_node, "qcom,pcie-sm8750"))
		return;

	for (i = 0; i < num; i++) {
		if (qcom_pcie_wcn_clk_is_hwgate(clks[i].id)) {
			clk_disable_unprepare(clks[i].clk);
			n++;
		}
	}
	pcie->wcn_clk_hwgated = true;
	dev_info(dev, "wcn7880: dropped SW vote on %d HWCG AXI clocks (D3: let them HW-gate in idle cal)\n", n);
}

/* Re-balance the bulk refcount before clk_bulk_disable_unprepare in deinit. */
static void qcom_pcie_wcn_clk_hwgate_undo(struct qcom_pcie *pcie,
					  struct clk_bulk_data *clks, int num)
{
	int i;

	if (!pcie->wcn_clk_hwgated)
		return;

	for (i = 0; i < num; i++)
		if (qcom_pcie_wcn_clk_is_hwgate(clks[i].id))
			clk_prepare_enable(clks[i].clk);
	pcie->wcn_clk_hwgated = false;
}

/*
 * v472/v473: replicate downstream msm_pcie's L1SS programming
 * (msm_pcie_config_l1_2_threshold + msm_pcie_config_l1ss) for the WCN7880 link,
 * so the cnss port sees the same link config Android's msm_pcie leaves behind.
 * msm_pcie writes the LTR_L1.2 threshold (LTR_L12_TH_VALUE/SCALE, verbatim from
 * DT qcom,l1-2-th-value / -scale, shifts 16/29) and enables all four L1SS
 * substates on RC + endpoint, plus DEVCTL2.LTR_EN. The live OxygenOS value is
 * 0xDF (EP CTL1=0x40df000f, RC=0x40df460f) -- that is what the DT now carries.
 *
 * Gated to the cnss_pci driver ONLY: ath12k self-programs L1SS
 * (ath12k_wcn7880_program_l1ss), so firing this for an ath12k bind would collide
 * with it. NB: L1SS does NOT gate fw-ready -- ath12k reaches fw-ready with L1SS
 * both disabled and enabled -- so this is a faithfulness match that matters for
 * the later cal phy-off L1.2-entry step, not for the QMI preamble.
 */
static void qcom_pcie_wcn_program_l1ss(struct qcom_pcie *pcie,
				       struct pci_dev *pdev)
{
	int l1ss = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_L1SS);
	u32 ctl1;

	if (!l1ss)
		return;

	/* LTR must be enabled for ASPM L1.2 to be entered. */
	pcie_capability_set_word(pdev, PCI_EXP_DEVCTL2, PCI_EXP_DEVCTL2_LTR_EN);

	pci_read_config_dword(pdev, l1ss + PCI_L1SS_CTL1, &ctl1);
	ctl1 &= ~(PCI_L1SS_CTL1_LTR_L12_TH_VALUE |
		  PCI_L1SS_CTL1_LTR_L12_TH_SCALE);
	ctl1 |= (PCI_L1SS_CTL1_LTR_L12_TH_VALUE &
		 (pcie->wcn_l1_2_th_value << 16)) |
		(PCI_L1SS_CTL1_LTR_L12_TH_SCALE &
		 (pcie->wcn_l1_2_th_scale << 29)) |
		PCI_L1SS_CTL1_L1SS_MASK;	/* PCIPM + ASPM, L1.1 + L1.2 enable */
	pci_write_config_dword(pdev, l1ss + PCI_L1SS_CTL1, ctl1);

	pci_read_config_dword(pdev, l1ss + PCI_L1SS_CTL1, &ctl1);
	pci_info(pdev, "wcn7880: L1SS CTL1 = 0x%08x (LTR_L1.2 th=0x%x scale=%u)\n",
		 ctl1, pcie->wcn_l1_2_th_value, pcie->wcn_l1_2_th_scale);
}

static int qcom_pcie_wcn_l1ss_notify(struct notifier_block *nb,
				     unsigned long action, void *data)
{
	struct qcom_pcie *pcie = container_of(nb, struct qcom_pcie, wcn_l1ss_nb);
	struct pci_dev *pdev = to_pci_dev(data);
	struct pci_dev *rc;

	if (action != BUS_NOTIFY_BOUND_DRIVER)
		return NOTIFY_DONE;
	/* WCN7880 (peach) endpoint only. */
	if (pdev->vendor != 0x17cb || pdev->device != 0x110e)
		return NOTIFY_DONE;
	/*
	 * cnss_pci only. ath12k (ath12k_wifi7_pci) force-programs L1SS itself;
	 * doing it here too would collide. This notifier exists to do for the
	 * cnss port what msm_pcie does on Android, so scope it to that driver.
	 */
	if (!pdev->dev.driver || strcmp(pdev->dev.driver->name, "cnss_pci"))
		return NOTIFY_DONE;

	qcom_pcie_wcn_program_l1ss(pcie, pdev);		/* endpoint */
	rc = pci_upstream_bridge(pdev);
	if (rc)
		qcom_pcie_wcn_program_l1ss(pcie, rc);	/* root port */

	return NOTIFY_DONE;
}

static int qcom_pcie_init_2_7_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_7_0 *res = &pcie->res.v2_7_0;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	u32 val;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(res->supplies), res->supplies);
	if (ret < 0) {
		dev_err(dev, "cannot enable regulators\n");
		return ret;
	}

	ret = clk_bulk_prepare_enable(res->num_clks, res->clks);
	if (ret < 0)
		goto err_disable_regulators;

	/* D3: let the HWCG PCIe AXI clocks gate in idle cal like Android (safe
	 * only with CESTA managing the clock requests -- see qcom_pcie_wcn_cesta).
	 */
	qcom_pcie_wcn_clk_hwgate_apply(pcie, res->clks, res->num_clks);

	ret = reset_control_assert(res->rst);
	if (ret) {
		dev_err(dev, "reset assert failed (%d)\n", ret);
		goto err_disable_clocks;
	}

	usleep_range(1000, 1500);

	ret = reset_control_deassert(res->rst);
	if (ret) {
		dev_err(dev, "reset deassert failed (%d)\n", ret);
		goto err_disable_clocks;
	}

	/* Wait for reset to complete, required on SM8450 */
	usleep_range(1000, 1500);

	/* configure PCIe to RC mode */
	writel(DEVICE_TYPE_RC, pcie->parf + PARF_DEVICE_TYPE);

	/* Force PHY out of lowest power state */
	val = readl(pcie->parf + PARF_PHY_CTRL);
	val &= ~PHY_TEST_PWR_DOWN;
	writel(val, pcie->parf + PARF_PHY_CTRL);

	qcom_pcie_configure_dbi_atu_base(pcie);

	/* MAC PHY_POWERDOWN MUX DISABLE  */
	val = readl(pcie->parf + PARF_SYS_CTRL);
	val &= ~MAC_PHY_POWERDOWN_IN_P2_D_MUX_EN;
	writel(val, pcie->parf + PARF_SYS_CTRL);

	/* SM8750 WCN RC: keep the PCIe core clock on in L1 (Android 0x365E). */
	qcom_pcie_wcn_clk_keepon_apply(pcie);

	val = readl(pcie->parf + PARF_MHI_CLOCK_RESET_CTRL);
	val |= BYPASS;
	writel(val, pcie->parf + PARF_MHI_CLOCK_RESET_CTRL);

	/* Enable L1 and L1SS */
	val = readl(pcie->parf + PARF_PM_CTRL);
	val &= ~REQ_NOT_ENTR_L1;
	writel(val, pcie->parf + PARF_PM_CTRL);

	pci->l1ss_support = true;

	/* SM8750 WCN RC: load+start the CESTA SM (SM-only, no CRM votes). */
	qcom_pcie_wcn_cesta_enable(pcie);

	/* v472: if the pcie node carries the Android L1SS threshold props, program
	 * L1.2 the msm_pcie way once the WCN endpoint binds (bus notifier — no
	 * post-enum hook exists here). Only the WCN pcie0 node has these props. */
	if (!of_property_read_u32(dev->of_node, "qcom,l1-2-th-value",
				  &pcie->wcn_l1_2_th_value)) {
		pcie->wcn_l1_2_th_scale = 2;
		of_property_read_u32(dev->of_node, "qcom,l1-2-th-scale",
				     &pcie->wcn_l1_2_th_scale);
		pcie->wcn_l1ss_nb.notifier_call = qcom_pcie_wcn_l1ss_notify;
		bus_register_notifier(&pci_bus_type, &pcie->wcn_l1ss_nb);
		dev_info(dev,
			 "wcn7880: L1SS programmer armed (LTR_L1.2 th=0x%x scale=%u, applied on EP bind)\n",
			 pcie->wcn_l1_2_th_value, pcie->wcn_l1_2_th_scale);
		/* Only the WCN pcie0 node carries these props: stash it for the
		 * exported cal->mission link toggle (qcom_pcie_wcn_set_link). */
		qcom_pcie_wcn_erhai = pcie;
	}

	val = readl(pcie->parf + PARF_AXI_MSTR_WR_ADDR_HALT_V2);
	val |= EN;
	writel(val, pcie->parf + PARF_AXI_MSTR_WR_ADDR_HALT_V2);

	return 0;
err_disable_clocks:
	clk_bulk_disable_unprepare(res->num_clks, res->clks);
err_disable_regulators:
	regulator_bulk_disable(ARRAY_SIZE(res->supplies), res->supplies);

	return ret;
}

static int qcom_pcie_post_init_2_7_0(struct qcom_pcie *pcie)
{
	const struct qcom_pcie_cfg *pcie_cfg = pcie->cfg;

	if (pcie_cfg->override_no_snoop)
		writel(WR_NO_SNOOP_OVERRIDE_EN | RD_NO_SNOOP_OVERRIDE_EN,
				pcie->parf + PARF_NO_SNOOP_OVERRIDE);

	qcom_pcie_set_slot_nccs(pcie->pci);

	return 0;
}

static int qcom_pcie_enable_aspm(struct pci_dev *pdev, void *userdata)
{
	/*
	 * Downstream devices need to be in D0 state before enabling PCI PM
	 * substates.
	 */
	pci_set_power_state_locked(pdev, PCI_D0);
	pci_enable_link_state_locked(pdev, PCIE_LINK_STATE_ALL);

	return 0;
}

/*
 * WCN cold-boot-cal: the complete PCIe config-space diff vs OxygenOS found the
 * one remaining host divergence on the WCN EP -- Android enables ASPM **L0s**
 * (LinkCtl 0x43 = L0s+L1, RCB=64) in both baseline and cal-done, while our
 * kernel leaves the EP at L1-only (0x4a, L0s off): qcom_pcie_enable_aspm()
 * requests PCIE_LINK_STATE_ALL but aspm.c masks L0s off on the latency check.
 * This matches the EP LinkCtl to Android exactly (set L0s, clear RCB) so the
 * link's L0s behaviour during cal replays Android (ours shows a one-directional
 * L0s storm in the post-bw_req freeze that Android's symmetric L0s does not).
 * Only the EP (non-bridge) is touched; the RC already matches Android (0x4b).
 * Gated default-on; A/B via pcie_qcom.qcom_pcie_wcn_force_l0s=0.
 */
static bool qcom_pcie_wcn_force_l0s = true;
module_param(qcom_pcie_wcn_force_l0s, bool, 0644);
MODULE_PARM_DESC(qcom_pcie_wcn_force_l0s,
	"SM8750 WCN: force ASPM L0s on the WCN endpoint (match Android EP LinkCtl 0x43) (default on)");

static int qcom_pcie_wcn_force_l0s_dev(struct pci_dev *pdev, void *data)
{
	u16 lnkctl, want;

	if (pci_is_bridge(pdev))	/* only the WCN endpoint; RC already 0x4b */
		return 0;

	pcie_capability_read_word(pdev, PCI_EXP_LNKCTL, &lnkctl);
	want = (lnkctl | PCI_EXP_LNKCTL_ASPM_L0S) & ~PCI_EXP_LNKCTL_RCB;
	if (want != lnkctl) {
		pcie_capability_write_word(pdev, PCI_EXP_LNKCTL, want);
		pci_info(pdev, "wcn7880: forced EP ASPM L0s (LnkCtl 0x%04x -> 0x%04x, match Android 0x43)\n",
			 lnkctl, want);
	}
	return 0;
}

static void qcom_pcie_host_post_init_2_7_0(struct qcom_pcie *pcie)
{
	struct dw_pcie_rp *pp = &pcie->pci->pp;

	pci_walk_bus(pp->bridge->bus, qcom_pcie_enable_aspm, NULL);

	if (qcom_pcie_wcn_force_l0s &&
	    of_device_is_compatible(pcie->pci->dev->of_node, "qcom,pcie-sm8750"))
		pci_walk_bus(pp->bridge->bus, qcom_pcie_wcn_force_l0s_dev, NULL);
}

static void qcom_pcie_deinit_2_7_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_7_0 *res = &pcie->res.v2_7_0;
	u32 val;

	/* Force PHY to lowest power state */
	val = readl(pcie->parf + PARF_PHY_CTRL);
	val |= PHY_TEST_PWR_DOWN;
	writel(val, pcie->parf + PARF_PHY_CTRL);

	/* D3: re-balance the clocks we dropped in init before the bulk disable. */
	qcom_pcie_wcn_clk_hwgate_undo(pcie, res->clks, res->num_clks);

	clk_bulk_disable_unprepare(res->num_clks, res->clks);

	regulator_bulk_disable(ARRAY_SIZE(res->supplies), res->supplies);
}

static int qcom_pcie_config_sid_1_9_0(struct qcom_pcie *pcie)
{
	/* iommu map structure */
	struct {
		u32 bdf;
		u32 phandle;
		u32 smmu_sid;
		u32 smmu_sid_len;
	} *map;
	void __iomem *bdf_to_sid_base = pcie->parf + PARF_BDF_TO_SID_TABLE_N;
	struct device *dev = pcie->pci->dev;
	u8 qcom_pcie_crc8_table[CRC8_TABLE_SIZE];
	int i, nr_map, size = 0;
	u32 smmu_sid_base;
	u32 val;

	of_get_property(dev->of_node, "iommu-map", &size);
	if (!size)
		return 0;

	/* Enable BDF to SID translation by disabling bypass mode (default) */
	val = readl(pcie->parf + PARF_BDF_TO_SID_CFG);
	val &= ~BDF_TO_SID_BYPASS;
	writel(val, pcie->parf + PARF_BDF_TO_SID_CFG);

	map = kzalloc(size, GFP_KERNEL);
	if (!map)
		return -ENOMEM;

	of_property_read_u32_array(dev->of_node, "iommu-map", (u32 *)map,
				   size / sizeof(u32));

	nr_map = size / (sizeof(*map));

	crc8_populate_msb(qcom_pcie_crc8_table, QCOM_PCIE_CRC8_POLYNOMIAL);

	/* Registers need to be zero out first */
	memset_io(bdf_to_sid_base, 0, CRC8_TABLE_SIZE * sizeof(u32));

	/* Extract the SMMU SID base from the first entry of iommu-map */
	smmu_sid_base = map[0].smmu_sid;

	/* Look for an available entry to hold the mapping */
	for (i = 0; i < nr_map; i++) {
		__be16 bdf_be = cpu_to_be16(map[i].bdf);
		u32 val;
		u8 hash;

		hash = crc8(qcom_pcie_crc8_table, (u8 *)&bdf_be, sizeof(bdf_be), 0);

		val = readl(bdf_to_sid_base + hash * sizeof(u32));

		/* If the register is already populated, look for next available entry */
		while (val) {
			u8 current_hash = hash++;
			u8 next_mask = 0xff;

			/* If NEXT field is NULL then update it with next hash */
			if (!(val & next_mask)) {
				val |= (u32)hash;
				writel(val, bdf_to_sid_base + current_hash * sizeof(u32));
			}

			val = readl(bdf_to_sid_base + hash * sizeof(u32));
		}

		/* BDF [31:16] | SID [15:8] | NEXT [7:0] */
		val = map[i].bdf << 16 | (map[i].smmu_sid - smmu_sid_base) << 8 | 0;
		writel(val, bdf_to_sid_base + hash * sizeof(u32));
	}

	kfree(map);

	return 0;
}

static int qcom_pcie_get_resources_2_9_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_9_0 *res = &pcie->res.v2_9_0;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;

	res->num_clks = devm_clk_bulk_get_all(dev, &res->clks);
	if (res->num_clks < 0) {
		dev_err(dev, "Failed to get clocks\n");
		return res->num_clks;
	}

	res->rst = devm_reset_control_array_get_exclusive(dev);
	if (IS_ERR(res->rst))
		return PTR_ERR(res->rst);

	return 0;
}

static void qcom_pcie_deinit_2_9_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_9_0 *res = &pcie->res.v2_9_0;
	u32 val;

	/* Force PHY to lowest power state */
	val = readl(pcie->parf + PARF_PHY_CTRL);
	val |= PHY_TEST_PWR_DOWN;
	writel(val, pcie->parf + PARF_PHY_CTRL);

	clk_bulk_disable_unprepare(res->num_clks, res->clks);
}

static int qcom_pcie_init_2_9_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_9_0 *res = &pcie->res.v2_9_0;
	struct device *dev = pcie->pci->dev;
	int ret;

	ret = reset_control_assert(res->rst);
	if (ret) {
		dev_err(dev, "reset assert failed (%d)\n", ret);
		return ret;
	}

	/*
	 * Delay periods before and after reset deassert are working values
	 * from downstream Codeaurora kernel
	 */
	usleep_range(2000, 2500);

	ret = reset_control_deassert(res->rst);
	if (ret) {
		dev_err(dev, "reset deassert failed (%d)\n", ret);
		return ret;
	}

	usleep_range(2000, 2500);

	return clk_bulk_prepare_enable(res->num_clks, res->clks);
}

static int qcom_pcie_post_init_2_9_0(struct qcom_pcie *pcie)
{
	struct dw_pcie *pci = pcie->pci;
	u16 offset = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	u32 val;
	int i;

	/* Force PHY out of lowest power state */
	val = readl(pcie->parf + PARF_PHY_CTRL);
	val &= ~PHY_TEST_PWR_DOWN;
	writel(val, pcie->parf + PARF_PHY_CTRL);

	qcom_pcie_configure_dbi_atu_base(pcie);

	writel(DEVICE_TYPE_RC, pcie->parf + PARF_DEVICE_TYPE);
	writel(BYPASS | MSTR_AXI_CLK_EN | AHB_CLK_EN,
		pcie->parf + PARF_MHI_CLOCK_RESET_CTRL);
	writel(GEN3_RELATED_OFF_RXEQ_RGRDLESS_RXTS |
		GEN3_RELATED_OFF_GEN3_ZRXDC_NONCOMPL,
		pci->dbi_base + GEN3_RELATED_OFF);

	writel(MST_WAKEUP_EN | SLV_WAKEUP_EN | MSTR_ACLK_CGC_DIS |
		SLV_ACLK_CGC_DIS | CORE_CLK_CGC_DIS |
		AUX_PWR_DET | L23_CLK_RMV_DIS | L1_CLK_RMV_DIS,
		pcie->parf + PARF_SYS_CTRL);

	writel(0, pcie->parf + PARF_Q2A_FLUSH);

	dw_pcie_dbi_ro_wr_en(pci);

	writel(PCIE_CAP_SLOT_VAL, pci->dbi_base + offset + PCI_EXP_SLTCAP);

	val = readl(pci->dbi_base + offset + PCI_EXP_LNKCAP);
	val &= ~PCI_EXP_LNKCAP_ASPMS;
	writel(val, pci->dbi_base + offset + PCI_EXP_LNKCAP);

	writel(PCI_EXP_DEVCTL2_COMP_TMOUT_DIS, pci->dbi_base + offset +
			PCI_EXP_DEVCTL2);

	dw_pcie_dbi_ro_wr_dis(pci);

	for (i = 0; i < 256; i++)
		writel(0, pcie->parf + PARF_BDF_TO_SID_TABLE_N + (4 * i));

	return 0;
}

static bool qcom_pcie_link_up(struct dw_pcie *pci)
{
	u16 offset = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	u16 val = readw(pci->dbi_base + offset + PCI_EXP_LNKSTA);

	return val & PCI_EXP_LNKSTA_DLLLA;
}

static enum dw_pcie_ltssm qcom_pcie_get_ltssm(struct dw_pcie *pci)
{
	struct qcom_pcie *pcie = to_qcom_pcie(pci);
	u32 val;

	if (pcie->cfg->ops->get_ltssm)
		return pcie->cfg->ops->get_ltssm(pcie);

	val = readl(pcie->parf + PARF_LTSSM);

	return (enum dw_pcie_ltssm)FIELD_GET(PARF_LTSSM_STATE_MASK, val);
}

static void qcom_pcie_phy_power_off(struct qcom_pcie *pcie)
{
	struct qcom_pcie_port *port;

	list_for_each_entry(port, &pcie->ports, list)
		phy_power_off(port->phy);
}

static int qcom_pcie_phy_power_on(struct qcom_pcie *pcie)
{
	struct qcom_pcie_port *port;
	int ret;

	list_for_each_entry(port, &pcie->ports, list) {
		ret = phy_set_mode_ext(port->phy, PHY_MODE_PCIE, PHY_MODE_PCIE_RC);
		if (ret)
			return ret;

		ret = phy_power_on(port->phy);
		if (ret) {
			qcom_pcie_phy_power_off(pcie);
			return ret;
		}
	}

	return 0;
}

static void qcom_pcie_configure_ports(struct qcom_pcie *pcie)
{
	struct qcom_pcie_port *port;

	list_for_each_entry(port, &pcie->ports, list)
		dw_pcie_program_t_power_on(pcie->pci, port->l1ss_t_power_on);
}

static int qcom_pcie_host_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct qcom_pcie *pcie = to_qcom_pcie(pci);
	int ret;

	qcom_pcie_perst_assert(pcie);

	ret = pcie->cfg->ops->init(pcie);
	if (ret)
		return ret;

	ret = qcom_pcie_phy_power_on(pcie);
	if (ret)
		goto err_deinit;

	if (!pci->suspended) {
		ret = pci_pwrctrl_create_devices(pci->dev);
		if (ret)
			goto err_disable_phy;
	}

	if (!pp->skip_pwrctrl_off) {
		ret = pci_pwrctrl_power_on_devices(pci->dev);
		if (ret)
			goto err_pwrctrl_destroy;
	}

	if (pcie->cfg->ops->post_init) {
		ret = pcie->cfg->ops->post_init(pcie);
		if (ret)
			goto err_pwrctrl_power_off;
	}

	qcom_pcie_clear_aspm_l0s(pcie->pci);
	dw_pcie_remove_capability(pcie->pci, PCI_CAP_ID_MSIX);
	dw_pcie_remove_ext_capability(pcie->pci, PCI_EXT_CAP_ID_DPC);

	qcom_pcie_configure_ports(pcie);

	qcom_pcie_perst_deassert(pcie);

	if (pcie->cfg->ops->config_sid) {
		ret = pcie->cfg->ops->config_sid(pcie);
		if (ret)
			goto err_assert_reset;
	}

	return 0;

err_assert_reset:
	qcom_pcie_perst_assert(pcie);
err_pwrctrl_power_off:
	if (!pp->skip_pwrctrl_off)
		pci_pwrctrl_power_off_devices(pci->dev);
err_pwrctrl_destroy:
	if (ret != -EPROBE_DEFER && !pci->suspended)
		pci_pwrctrl_destroy_devices(pci->dev);
err_disable_phy:
	qcom_pcie_phy_power_off(pcie);
err_deinit:
	pcie->cfg->ops->deinit(pcie);

	return ret;
}

static void qcom_pcie_host_deinit(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct qcom_pcie *pcie = to_qcom_pcie(pci);

	qcom_pcie_perst_assert(pcie);

	if (!pci->pp.skip_pwrctrl_off) {
		/*
		 * No need to destroy pwrctrl devices as this function only
		 * gets called during system suspend as of now.
		 */
		pci_pwrctrl_power_off_devices(pci->dev);
	}

	qcom_pcie_phy_power_off(pcie);
	pcie->cfg->ops->deinit(pcie);
}

static void qcom_pcie_host_post_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct qcom_pcie *pcie = to_qcom_pcie(pci);

	/*
	 * During system suspend, the Qcom RC driver may turn off the
	 * analog circuitry of PHY and remove controller votes to save
	 * power. If the link is in L1SS and the endpoint asserts CLKREQ#
	 * to exit L1SS, the time required to wake the system and restore
	 * the PHY/REFCLK may exceed the L1SS exit timing (L10_REFCLK_ON +
	 * T_COMMONMODE), resulting in Link Down (LDn) and a reset of the
	 * endpoint. Set this flag to indicate this limitation to client
	 * drivers so that they can avoid relying on device state being
	 * preserved during system suspend.
	 */
	pp->bridge->broken_l1ss_resume = true;

	if (pcie->cfg->ops->host_post_init)
		pcie->cfg->ops->host_post_init(pcie);
}

static void qcom_pcie_host_pme_turn_off(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	writel(ELBI_SYS_CTRL_PME_TURNOFF_MSG, pci->elbi_base + ELBI_SYS_CTRL);
}

static const struct dw_pcie_host_ops qcom_pcie_dw_ops = {
	.init		= qcom_pcie_host_init,
	.deinit		= qcom_pcie_host_deinit,
	.post_init	= qcom_pcie_host_post_init,
	.pme_turn_off	= qcom_pcie_host_pme_turn_off,
};

/* Qcom IP rev.: 2.1.0	Synopsys IP rev.: 4.01a */
static const struct qcom_pcie_ops ops_2_1_0 = {
	.get_resources = qcom_pcie_get_resources_2_1_0,
	.init = qcom_pcie_init_2_1_0,
	.post_init = qcom_pcie_post_init_2_1_0,
	.deinit = qcom_pcie_deinit_2_1_0,
	.ltssm_enable = qcom_pcie_2_1_0_ltssm_enable,
	.get_ltssm = qcom_pcie_2_1_0_get_ltssm,
};

/* Qcom IP rev.: 1.0.0	Synopsys IP rev.: 4.11a */
static const struct qcom_pcie_ops ops_1_0_0 = {
	.get_resources = qcom_pcie_get_resources_1_0_0,
	.init = qcom_pcie_init_1_0_0,
	.post_init = qcom_pcie_post_init_1_0_0,
	.deinit = qcom_pcie_deinit_1_0_0,
	.ltssm_enable = qcom_pcie_2_1_0_ltssm_enable,
	.get_ltssm = qcom_pcie_2_1_0_get_ltssm,
};

/* Qcom IP rev.: 2.3.2	Synopsys IP rev.: 4.21a */
static const struct qcom_pcie_ops ops_2_3_2 = {
	.get_resources = qcom_pcie_get_resources_2_3_2,
	.init = qcom_pcie_init_2_3_2,
	.post_init = qcom_pcie_post_init_2_3_2,
	.deinit = qcom_pcie_deinit_2_3_2,
	.ltssm_enable = qcom_pcie_2_3_2_ltssm_enable,
};

/* Qcom IP rev.: 2.4.0	Synopsys IP rev.: 4.20a */
static const struct qcom_pcie_ops ops_2_4_0 = {
	.get_resources = qcom_pcie_get_resources_2_4_0,
	.init = qcom_pcie_init_2_4_0,
	.post_init = qcom_pcie_post_init_2_3_2,
	.deinit = qcom_pcie_deinit_2_4_0,
	.ltssm_enable = qcom_pcie_2_3_2_ltssm_enable,
};

/* Qcom IP rev.: 2.3.3	Synopsys IP rev.: 4.30a */
static const struct qcom_pcie_ops ops_2_3_3 = {
	.get_resources = qcom_pcie_get_resources_2_3_3,
	.init = qcom_pcie_init_2_3_3,
	.post_init = qcom_pcie_post_init_2_3_3,
	.deinit = qcom_pcie_deinit_2_3_3,
	.ltssm_enable = qcom_pcie_2_3_2_ltssm_enable,
};

/* Qcom IP rev.: 2.7.0	Synopsys IP rev.: 4.30a */
static const struct qcom_pcie_ops ops_2_7_0 = {
	.get_resources = qcom_pcie_get_resources_2_7_0,
	.init = qcom_pcie_init_2_7_0,
	.post_init = qcom_pcie_post_init_2_7_0,
	.deinit = qcom_pcie_deinit_2_7_0,
	.ltssm_enable = qcom_pcie_2_3_2_ltssm_enable,
};

/* Qcom IP rev.: 1.9.0 */
static const struct qcom_pcie_ops ops_1_9_0 = {
	.get_resources = qcom_pcie_get_resources_2_7_0,
	.init = qcom_pcie_init_2_7_0,
	.post_init = qcom_pcie_post_init_2_7_0,
	.host_post_init = qcom_pcie_host_post_init_2_7_0,
	.deinit = qcom_pcie_deinit_2_7_0,
	.ltssm_enable = qcom_pcie_2_3_2_ltssm_enable,
	.config_sid = qcom_pcie_config_sid_1_9_0,
};

/* Qcom IP rev.: 1.21.0  Synopsys IP rev.: 5.60a */
static const struct qcom_pcie_ops ops_1_21_0 = {
	.get_resources = qcom_pcie_get_resources_2_7_0,
	.init = qcom_pcie_init_2_7_0,
	.post_init = qcom_pcie_post_init_2_7_0,
	.host_post_init = qcom_pcie_host_post_init_2_7_0,
	.deinit = qcom_pcie_deinit_2_7_0,
	.ltssm_enable = qcom_pcie_2_3_2_ltssm_enable,
};

/* Qcom IP rev.: 2.9.0  Synopsys IP rev.: 5.00a */
static const struct qcom_pcie_ops ops_2_9_0 = {
	.get_resources = qcom_pcie_get_resources_2_9_0,
	.init = qcom_pcie_init_2_9_0,
	.post_init = qcom_pcie_post_init_2_9_0,
	.deinit = qcom_pcie_deinit_2_9_0,
	.ltssm_enable = qcom_pcie_2_3_2_ltssm_enable,
};

static const struct qcom_pcie_cfg cfg_1_0_0 = {
	.ops = &ops_1_0_0,
};

static const struct qcom_pcie_cfg cfg_1_9_0 = {
	.ops = &ops_1_9_0,
};

static const struct qcom_pcie_cfg cfg_1_34_0 = {
	.ops = &ops_1_9_0,
	.override_no_snoop = true,
	.no_l0s = true,
};

static const struct qcom_pcie_cfg cfg_2_1_0 = {
	.ops = &ops_2_1_0,
};

static const struct qcom_pcie_cfg cfg_2_3_2 = {
	.ops = &ops_2_3_2,
	.no_l0s = true,
};

static const struct qcom_pcie_cfg cfg_2_3_3 = {
	.ops = &ops_2_3_3,
};

static const struct qcom_pcie_cfg cfg_2_4_0 = {
	.ops = &ops_2_4_0,
};

static const struct qcom_pcie_cfg cfg_2_7_0 = {
	.ops = &ops_2_7_0,
};

static const struct qcom_pcie_cfg cfg_2_9_0 = {
	.ops = &ops_2_9_0,
};

static const struct qcom_pcie_cfg cfg_sc8280xp = {
	.ops = &ops_1_21_0,
	.no_l0s = true,
};

static const struct qcom_pcie_cfg cfg_fw_managed = {
	.firmware_managed = true,
};

static const struct dw_pcie_ops dw_pcie_ops = {
	.link_up = qcom_pcie_link_up,
	.start_link = qcom_pcie_start_link,
	.get_ltssm = qcom_pcie_get_ltssm,
};

static int qcom_pcie_icc_init(struct qcom_pcie *pcie)
{
	struct dw_pcie *pci = pcie->pci;
	int ret;

	pcie->icc_mem = devm_of_icc_get(pci->dev, "pcie-mem");
	if (IS_ERR(pcie->icc_mem))
		return PTR_ERR(pcie->icc_mem);

	pcie->icc_cpu = devm_of_icc_get(pci->dev, "cpu-pcie");
	if (IS_ERR(pcie->icc_cpu))
		return PTR_ERR(pcie->icc_cpu);
	/*
	 * Some Qualcomm platforms require interconnect bandwidth constraints
	 * to be set before enabling interconnect clocks.
	 *
	 * Set an initial peak bandwidth corresponding to single-lane Gen 1
	 * for the pcie-mem path.
	 */
	ret = icc_set_bw(pcie->icc_mem, 0, QCOM_PCIE_LINK_SPEED_TO_BW(1));
	if (ret) {
		dev_err(pci->dev, "Failed to set bandwidth for PCIe-MEM interconnect path: %d\n",
			ret);
		return ret;
	}

	/*
	 * Since the CPU-PCIe path is only used for activities like register
	 * access of the host controller and endpoint Config/BAR space access,
	 * HW team has recommended to use a minimal bandwidth of 1KBps just to
	 * keep the path active.
	 */
	ret = icc_set_bw(pcie->icc_cpu, 0, kBps_to_icc(1));
	if (ret) {
		dev_err(pci->dev, "Failed to set bandwidth for CPU-PCIe interconnect path: %d\n",
			ret);
		icc_set_bw(pcie->icc_mem, 0, 0);
		return ret;
	}

	return 0;
}

static void qcom_pcie_icc_opp_update(struct qcom_pcie *pcie)
{
	u32 offset, status, width, speed;
	struct dw_pcie *pci = pcie->pci;
	struct dev_pm_opp_key key = {};
	unsigned long freq_kbps;
	struct dev_pm_opp *opp;
	int ret, freq_mbps;

	offset = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	status = readw(pci->dbi_base + offset + PCI_EXP_LNKSTA);

	/* Only update constraints if link is up. */
	if (!(status & PCI_EXP_LNKSTA_DLLLA))
		return;

	speed = FIELD_GET(PCI_EXP_LNKSTA_CLS, status);
	width = FIELD_GET(PCI_EXP_LNKSTA_NLW, status);

	if (pcie->icc_mem) {
		ret = icc_set_bw(pcie->icc_mem, 0,
				 width * QCOM_PCIE_LINK_SPEED_TO_BW(speed));
		if (ret) {
			dev_err(pci->dev, "Failed to set bandwidth for PCIe-MEM interconnect path: %d\n",
				ret);
		}
	} else if (pcie->use_pm_opp) {
		freq_mbps = pcie_dev_speed_mbps(pcie_get_link_speed(speed));
		if (freq_mbps < 0)
			return;

		freq_kbps = freq_mbps * KILO;
		opp = dev_pm_opp_find_level_exact(pci->dev, speed);
		if (IS_ERR(opp)) {
			 /* opp-level is not defined use only frequency */
			opp = dev_pm_opp_find_freq_exact(pci->dev, freq_kbps * width,
							 true);
		} else {
			/* put opp-level OPP */
			dev_pm_opp_put(opp);

			key.freq = freq_kbps * width;
			key.level = speed;
			key.bw = 0;
			opp = dev_pm_opp_find_key_exact(pci->dev, &key, true);
		}
		if (!IS_ERR(opp)) {
			ret = dev_pm_opp_set_opp(pci->dev, opp);
			if (ret)
				dev_err(pci->dev, "Failed to set OPP for freq (%lu): %d\n",
					freq_kbps * width, ret);
			dev_pm_opp_put(opp);
		}
	}
}

static int qcom_pcie_set_max_opp(struct device *dev)
{
	unsigned long max_freq = ULONG_MAX;
	struct dev_pm_opp *opp;
	int ret;

	opp = dev_pm_opp_find_freq_floor(dev, &max_freq);
	if (IS_ERR(opp))
		return PTR_ERR(opp);

	ret = dev_pm_opp_set_opp(dev, opp);
	dev_pm_opp_put(opp);

	return ret;
}

static int qcom_pcie_link_transition_count(struct seq_file *s, void *data)
{
	struct qcom_pcie *pcie = (struct qcom_pcie *)dev_get_drvdata(s->private);

	seq_printf(s, "L0s transition count: %u\n",
		   readl_relaxed(pcie->mhi + PARF_DEBUG_CNT_PM_LINKST_IN_L0S));

	seq_printf(s, "L1 transition count: %u\n",
		   readl_relaxed(pcie->mhi + PARF_DEBUG_CNT_PM_LINKST_IN_L1));

	seq_printf(s, "L1.1 transition count: %u\n",
		   readl_relaxed(pcie->mhi + PARF_DEBUG_CNT_AUX_CLK_IN_L1SUB_L1));

	seq_printf(s, "L1.2 transition count: %u\n",
		   readl_relaxed(pcie->mhi + PARF_DEBUG_CNT_AUX_CLK_IN_L1SUB_L2));

	seq_printf(s, "L2 transition count: %u\n",
		   readl_relaxed(pcie->mhi + PARF_DEBUG_CNT_PM_LINKST_IN_L2));

	return 0;
}

static void qcom_pcie_init_debugfs(struct qcom_pcie *pcie)
{
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	char *name;

	name = devm_kasprintf(dev, GFP_KERNEL, "%pOFP", dev->of_node);
	if (!name)
		return;

	pcie->debugfs = debugfs_create_dir(name, NULL);
	debugfs_create_devm_seqfile(dev, "link_transition_count", pcie->debugfs,
				    qcom_pcie_link_transition_count);
}

static void qcom_pci_free_msi(void *ptr)
{
	struct dw_pcie_rp *pp = (struct dw_pcie_rp *)ptr;

	if (pp && pp->use_imsi_rx)
		dw_pcie_free_msi(pp);
}

static int qcom_pcie_ecam_host_init(struct pci_config_window *cfg)
{
	struct device *dev = cfg->parent;
	struct dw_pcie_rp *pp;
	struct dw_pcie *pci;
	int ret;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pci->dev = dev;
	pp = &pci->pp;
	pci->dbi_base = cfg->win;
	pp->num_vectors = MSI_DEF_NUM_VECTORS;

	/*
	 * dw_pcie_msi_host_init() is called directly here, bypassing
	 * dw_pcie_host_init() where pp->lock is normally initialized.
	 */
	raw_spin_lock_init(&pp->lock);

	ret = dw_pcie_msi_host_init(pp);
	if (ret)
		return ret;

	pp->use_imsi_rx = true;
	dw_pcie_msi_init(pp);

	return devm_add_action_or_reset(dev, qcom_pci_free_msi, pp);
}

static const struct pci_ecam_ops pci_qcom_ecam_ops = {
	.init		= qcom_pcie_ecam_host_init,
	.pci_ops	= {
		.map_bus	= pci_ecam_map_bus,
		.read		= pci_generic_config_read,
		.write		= pci_generic_config_write,
	}
};

/* Parse PERST# from all nodes in depth first manner starting from @np */
static int qcom_pcie_parse_perst(struct qcom_pcie *pcie,
				 struct qcom_pcie_port *port,
				 struct device_node *np)
{
	struct device *dev = pcie->pci->dev;
	struct qcom_pcie_perst *perst;
	struct gpio_desc *reset;
	int ret;

	if (pcie->reset) {
		dev_warn_once(dev,
			      "Reusing PERST# from Root Complex node. DT needs to be fixed!\n");
		reset = pcie->reset;
		goto skip_perst_parsing;
	}

	if (!of_find_property(np, "reset-gpios", NULL))
		goto parse_child_node;

	reset = devm_fwnode_gpiod_get(dev, of_fwnode_handle(np), "reset",
				      GPIOD_OUT_HIGH, "PERST#");
	if (IS_ERR(reset)) {
		/*
		 * FIXME: GPIOLIB currently supports exclusive GPIO access only.
		 * Non exclusive access is broken. But shared PERST# requires
		 * non-exclusive access. So once GPIOLIB properly supports it,
		 * implement it here.
		 */
		if (PTR_ERR(reset) == -EBUSY)
			dev_err(dev, "Shared PERST# is not supported\n");

		return PTR_ERR(reset);
	}

skip_perst_parsing:
	perst = devm_kzalloc(dev, sizeof(*perst), GFP_KERNEL);
	if (!perst)
		return -ENOMEM;

	INIT_LIST_HEAD(&perst->list);
	perst->desc = reset;
	list_add_tail(&perst->list, &port->perst);

parse_child_node:
	for_each_available_child_of_node_scoped(np, child) {
		ret = qcom_pcie_parse_perst(pcie, port, child);
		if (ret)
			return ret;
	}

	return 0;
}

static int qcom_pcie_parse_port(struct qcom_pcie *pcie, struct device_node *node)
{
	struct device *dev = pcie->pci->dev;
	struct qcom_pcie_port *port;
	struct phy *phy;
	int ret;

	phy = devm_of_phy_get(dev, node, NULL);
	if (IS_ERR(phy))
		return PTR_ERR(phy);

	port = devm_kzalloc(dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	ret = phy_init(phy);
	if (ret)
		return ret;

	INIT_LIST_HEAD(&port->perst);

	ret = qcom_pcie_parse_perst(pcie, port, node);
	if (ret)
		return ret;

	/* TODO: Move to DWC core after multi Root Port support is added */
	of_property_read_u32(node, "t-power-on-us", &port->l1ss_t_power_on);

	port->phy = phy;
	INIT_LIST_HEAD(&port->list);
	list_add_tail(&port->list, &pcie->ports);

	return 0;
}

static int qcom_pcie_parse_ports(struct qcom_pcie *pcie)
{
	struct qcom_pcie_perst *perst, *tmp_perst;
	struct qcom_pcie_port *port, *tmp_port;
	struct device *dev = pcie->pci->dev;
	int ret = -ENODEV;

	if (of_find_property(dev->of_node, "perst-gpios", NULL)) {
		pcie->reset = devm_gpiod_get_optional(dev, "perst",
						      GPIOD_OUT_HIGH);
		if (IS_ERR(pcie->reset))
			return PTR_ERR(pcie->reset);
	}

	for_each_available_child_of_node_scoped(dev->of_node, of_port) {
		if (!of_node_is_type(of_port, "pci"))
			continue;
		ret = qcom_pcie_parse_port(pcie, of_port);
		if (ret)
			goto err_port_del;
	}

	return ret;

err_port_del:
	list_for_each_entry_safe(port, tmp_port, &pcie->ports, list) {
		list_for_each_entry_safe(perst, tmp_perst, &port->perst, list)
			list_del(&perst->list);
		phy_exit(port->phy);
		list_del(&port->list);
	}

	return ret;
}

static int qcom_pcie_parse_legacy_binding(struct qcom_pcie *pcie)
{
	struct device *dev = pcie->pci->dev;
	struct qcom_pcie_perst *perst;
	struct qcom_pcie_port *port;
	struct phy *phy;
	int ret;

	phy = devm_phy_optional_get(dev, "pciephy");
	if (IS_ERR(phy))
		return PTR_ERR(phy);

	ret = phy_init(phy);
	if (ret)
		return ret;

	port = devm_kzalloc(dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	perst = devm_kzalloc(dev, sizeof(*perst), GFP_KERNEL);
	if (!perst)
		return -ENOMEM;

	port->phy = phy;
	INIT_LIST_HEAD(&port->list);
	list_add_tail(&port->list, &pcie->ports);

	perst->desc = pcie->reset;
	INIT_LIST_HEAD(&port->perst);
	INIT_LIST_HEAD(&perst->list);
	list_add_tail(&perst->list, &port->perst);

	return 0;
}

static int qcom_pcie_probe(struct platform_device *pdev)
{
	struct qcom_pcie_perst *perst, *tmp_perst;
	struct qcom_pcie_port *port, *tmp_port;
	const struct qcom_pcie_cfg *pcie_cfg;
	struct device *dev = &pdev->dev;
	struct qcom_pcie *pcie;
	struct dw_pcie_rp *pp;
	struct resource *res;
	struct dw_pcie *pci;
	int ret;

	pcie_cfg = of_device_get_match_data(dev);
	if (!pcie_cfg) {
		dev_err(dev, "No platform data\n");
		return -ENODATA;
	}

	if (!pcie_cfg->firmware_managed && !pcie_cfg->ops) {
		dev_err(dev, "No platform ops\n");
		return -ENODATA;
	}

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0)
		goto err_pm_runtime_put;

	if (pcie_cfg->firmware_managed) {
		struct pci_host_bridge *bridge;
		struct pci_config_window *cfg;

		bridge = devm_pci_alloc_host_bridge(dev, 0);
		if (!bridge) {
			ret = -ENOMEM;
			goto err_pm_runtime_put;
		}

		/* Parse and map our ECAM configuration space area */
		cfg = pci_host_common_ecam_create(dev, bridge,
				&pci_qcom_ecam_ops);
		if (IS_ERR(cfg)) {
			ret = PTR_ERR(cfg);
			goto err_pm_runtime_put;
		}

		bridge->sysdata = cfg;
		bridge->ops = (struct pci_ops *)&pci_qcom_ecam_ops.pci_ops;
		bridge->msi_domain = true;

		ret = pci_host_probe(bridge);
		if (ret)
			goto err_pm_runtime_put;

		return 0;
	}

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie) {
		ret = -ENOMEM;
		goto err_pm_runtime_put;
	}

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci) {
		ret = -ENOMEM;
		goto err_pm_runtime_put;
	}

	INIT_LIST_HEAD(&pcie->ports);

	pci->dev = dev;
	pci->ops = &dw_pcie_ops;
	pp = &pci->pp;

	pcie->pci = pci;

	pcie->cfg = pcie_cfg;

	pcie->parf = devm_platform_ioremap_resource_byname(pdev, "parf");
	if (IS_ERR(pcie->parf)) {
		ret = PTR_ERR(pcie->parf);
		goto err_pm_runtime_put;
	}

	/* MHI region is optional */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "mhi");
	if (res) {
		pcie->mhi = devm_ioremap_resource(dev, res);
		if (IS_ERR(pcie->mhi)) {
			ret = PTR_ERR(pcie->mhi);
			goto err_pm_runtime_put;
		}
	}

	/* OPP table is optional */
	ret = devm_pm_opp_of_add_table(dev);
	if (ret && ret != -ENODEV) {
		dev_err_probe(dev, ret, "Failed to add OPP table\n");
		goto err_pm_runtime_put;
	}

	/*
	 * Before the PCIe link is initialized, vote for highest OPP in the OPP
	 * table, so that we are voting for maximum voltage corner for the
	 * link to come up in maximum supported speed. At the end of the
	 * probe(), OPP will be updated using qcom_pcie_icc_opp_update().
	 */
	if (!ret) {
		ret = qcom_pcie_set_max_opp(dev);
		if (ret) {
			dev_err_probe(dev, ret, "Failed to set max OPP\n");
			goto err_pm_runtime_put;
		}

		pcie->use_pm_opp = true;
	} else {
		/* Skip ICC init if OPP is supported as it is handled by OPP */
		ret = qcom_pcie_icc_init(pcie);
		if (ret)
			goto err_pm_runtime_put;
	}

	ret = pcie->cfg->ops->get_resources(pcie);
	if (ret)
		goto err_pm_runtime_put;

	pp->ops = &qcom_pcie_dw_ops;

	ret = qcom_pcie_parse_ports(pcie);
	if (ret) {
		if (ret != -ENODEV) {
			dev_err_probe(pci->dev, ret,
				      "Failed to parse Root Port: %d\n", ret);
			goto err_pm_runtime_put;
		}

		/*
		 * In the case of properties not populated in Root Port node,
		 * fallback to the legacy method of parsing the Host Bridge
		 * node. This is to maintain DT backwards compatibility.
		 */
		ret = qcom_pcie_parse_legacy_binding(pcie);
		if (ret)
			goto err_pm_runtime_put;
	}

	platform_set_drvdata(pdev, pcie);

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err_probe(dev, ret, "cannot initialize host\n");
		goto err_phy_exit;
	}

	qcom_pcie_icc_opp_update(pcie);

	if (pcie->mhi)
		qcom_pcie_init_debugfs(pcie);

	return 0;

err_phy_exit:
	list_for_each_entry_safe(port, tmp_port, &pcie->ports, list) {
		list_for_each_entry_safe(perst, tmp_perst, &port->perst, list)
			list_del(&perst->list);
		phy_exit(port->phy);
		list_del(&port->list);
	}
err_pm_runtime_put:
	pm_runtime_put(dev);
	pm_runtime_disable(dev);

	return ret;
}

static int qcom_pcie_suspend_noirq(struct device *dev)
{
	struct qcom_pcie *pcie;
	int ret = 0;

	pcie = dev_get_drvdata(dev);
	if (!pcie)
		return 0;

	ret = dw_pcie_suspend_noirq(pcie->pci);
	if (ret)
		return ret;

	if (pcie->pci->suspended) {
		ret = icc_disable(pcie->icc_mem);
		if (ret)
			dev_err(dev, "Failed to disable PCIe-MEM interconnect path: %d\n", ret);

		ret = icc_disable(pcie->icc_cpu);
		if (ret)
			dev_err(dev, "Failed to disable CPU-PCIe interconnect path: %d\n", ret);

		if (pcie->use_pm_opp)
			dev_pm_opp_set_opp(pcie->pci->dev, NULL);
	} else {
		/*
		 * Set minimum bandwidth required to keep data path
		 * functional during suspend.
		 */
		if (pcie->icc_mem) {
			ret = icc_set_bw(pcie->icc_mem, 0, kBps_to_icc(1));
			if (ret) {
				dev_err(dev,
					"Failed to set bandwidth for PCIe-MEM interconnect path: %d\n",
					ret);
				return ret;
			}
		}

		/*
		 * Only disable CPU-PCIe interconnect path if the suspend
		 * is non-S2RAM.  On some platforms, DBI access can happen
		 * very late during S2RAM and a non-active CPU-PCIe
		 * interconnect path may lead to NoC error.
		 */
		if (pm_suspend_target_state != PM_SUSPEND_MEM) {
			ret = icc_disable(pcie->icc_cpu);
			if (ret)
				dev_err(dev, "Failed to disable CPU-PCIe interconnect path: %d\n",
					ret);

			if (pcie->use_pm_opp)
				dev_pm_opp_set_opp(pcie->pci->dev, NULL);
		}
	}
	return ret;
}

static int qcom_pcie_resume_noirq(struct device *dev)
{
	struct qcom_pcie *pcie;
	int ret;

	pcie = dev_get_drvdata(dev);
	if (!pcie)
		return 0;

	if (pcie->pci->suspended) {
		if (pcie->use_pm_opp) {
			ret = qcom_pcie_set_max_opp(dev);
			if (ret) {
				dev_err(dev, "Failed to set max OPP: %d\n", ret);
				return ret;
			}
		}

		ret = icc_enable(pcie->icc_cpu);
		if (ret) {
			dev_err(dev, "Failed to enable CPU-PCIe interconnect path: %d\n", ret);
			return ret;
		}

		ret = icc_enable(pcie->icc_mem);
		if (ret) {
			dev_err(dev, "Failed to enable PCIe-MEM interconnect path: %d\n", ret);
			goto disable_icc_cpu;
		}

		/*
		 * Ignore -ENODEV & -EIO here since it is expected when no
		 * endpoint is connected to the PCIe link.
		 */
		ret = dw_pcie_resume_noirq(pcie->pci);
		if (ret && ret != -ENODEV && ret != -EIO)
			goto disable_icc_mem;
	} else {
		if (pm_suspend_target_state != PM_SUSPEND_MEM) {
			if (pcie->use_pm_opp) {
				ret = qcom_pcie_set_max_opp(dev);
				if (ret) {
					dev_err(dev, "Failed to set max OPP: %d\n", ret);
					return ret;
				}
			}

			ret = icc_enable(pcie->icc_cpu);
			if (ret) {
				dev_err(dev, "Failed to enable CPU-PCIe interconnect path: %d\n",
					ret);
				return ret;
			}
		}
	}

	qcom_pcie_icc_opp_update(pcie);

	return 0;
disable_icc_mem:
	icc_disable(pcie->icc_mem);
disable_icc_cpu:
	icc_disable(pcie->icc_cpu);

	return ret;
}

static const struct of_device_id qcom_pcie_match[] = {
	{ .compatible = "qcom,pcie-apq8064", .data = &cfg_2_1_0 },
	{ .compatible = "qcom,pcie-apq8084", .data = &cfg_1_0_0 },
	{ .compatible = "qcom,pcie-ipq4019", .data = &cfg_2_4_0 },
	{ .compatible = "qcom,pcie-ipq5018", .data = &cfg_2_9_0 },
	{ .compatible = "qcom,pcie-ipq6018", .data = &cfg_2_9_0 },
	{ .compatible = "qcom,pcie-ipq8064", .data = &cfg_2_1_0 },
	{ .compatible = "qcom,pcie-ipq8064-v2", .data = &cfg_2_1_0 },
	{ .compatible = "qcom,pcie-ipq8074", .data = &cfg_2_3_3 },
	{ .compatible = "qcom,pcie-ipq8074-gen3", .data = &cfg_2_9_0 },
	{ .compatible = "qcom,pcie-ipq9574", .data = &cfg_2_9_0 },
	{ .compatible = "qcom,pcie-msm8996", .data = &cfg_2_3_2 },
	{ .compatible = "qcom,pcie-qcs404", .data = &cfg_2_4_0 },
	{ .compatible = "qcom,pcie-sa8255p", .data = &cfg_fw_managed },
	{ .compatible = "qcom,pcie-sa8540p", .data = &cfg_sc8280xp },
	{ .compatible = "qcom,pcie-sa8775p", .data = &cfg_1_34_0},
	{ .compatible = "qcom,pcie-sc7280", .data = &cfg_1_9_0 },
	{ .compatible = "qcom,pcie-sc8180x", .data = &cfg_1_9_0 },
	{ .compatible = "qcom,pcie-sc8280xp", .data = &cfg_sc8280xp },
	{ .compatible = "qcom,pcie-sdm845", .data = &cfg_2_7_0 },
	{ .compatible = "qcom,pcie-sdx55", .data = &cfg_1_9_0 },
	{ .compatible = "qcom,pcie-sm8150", .data = &cfg_1_9_0 },
	{ .compatible = "qcom,pcie-sm8250", .data = &cfg_1_9_0 },
	{ .compatible = "qcom,pcie-sm8350", .data = &cfg_1_9_0 },
	{ .compatible = "qcom,pcie-sm8450-pcie0", .data = &cfg_1_9_0 },
	{ .compatible = "qcom,pcie-sm8450-pcie1", .data = &cfg_1_9_0 },
	{ .compatible = "qcom,pcie-sm8550", .data = &cfg_1_9_0 },
	{ .compatible = "qcom,pcie-x1e80100", .data = &cfg_sc8280xp },
	{ }
};

static void qcom_fixup_class(struct pci_dev *dev)
{
	dev->class = PCI_CLASS_BRIDGE_PCI_NORMAL;
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_QCOM, 0x0101, qcom_fixup_class);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_QCOM, 0x0104, qcom_fixup_class);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_QCOM, 0x0106, qcom_fixup_class);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_QCOM, 0x0107, qcom_fixup_class);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_QCOM, 0x0302, qcom_fixup_class);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_QCOM, 0x1000, qcom_fixup_class);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_QCOM, 0x1001, qcom_fixup_class);

static const struct dev_pm_ops qcom_pcie_pm_ops = {
	NOIRQ_SYSTEM_SLEEP_PM_OPS(qcom_pcie_suspend_noirq, qcom_pcie_resume_noirq)
};

static struct platform_driver qcom_pcie_driver = {
	.probe = qcom_pcie_probe,
	.driver = {
		.name = "qcom-pcie",
		.suppress_bind_attrs = true,
		.of_match_table = qcom_pcie_match,
		.pm = &qcom_pcie_pm_ops,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};
builtin_platform_driver(qcom_pcie_driver);
