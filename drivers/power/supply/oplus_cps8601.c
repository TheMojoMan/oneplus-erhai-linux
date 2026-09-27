// SPDX-License-Identifier: GPL-2.0-only
/*
 * OnePlus "wireless pen" charger - CPS8601 WPT (Qi) TX chip driver.
 *
 * Standalone mainline port of the OnePlus vendor charger backend
 * (vendor/oplus/kernel/charger/wireless_pen/oplus_cps8601.c) for the
 * OnePlus Pad 3 (erhai, SM8750), in the style of the in-tree SC8547A port.
 * The chip is a wireless-power TX controller at I2C 0x41 on the qupv3 i2c
 * hub ("pencil-wireless-charger@41" in the erhai device tree): it transmits
 * Qi ping/charge to the pen (an IDT/Renesas P9418-class RX) and the pen
 * reports its own BLE MAC back over the power link - which is what stock's
 * "OplusPen" service used to auto-bond the pen over Bluetooth.
 *
 * This port implements the run-time charging state machine:
 *   - the chip's 16-bit register-access window (chip id, interrupt flags,
 *     thresholds, ADC, ASK/SSP/IDP/BC packets),
 *   - the five platform GPIOs (charge-int interrupt, sw_en, sleep, scan,
 *     off-state) requested from the device tree,
 *   - the H-BoOST supply rail over pmic_glink (owner 32785, opcode
 *     0x10007): the PMIC must acknowledge the boosted rail voltage
 *     (default 7.0 V, 10.0 V while a pen is attached) before charging is
 *     allowed,
 *   - Q-value calibration on first attach, EPT/FOD/OCP/OVP handling, a
 *     power-expired limit, and (optional) the pencil_status / pencil_addr /
 *     pencil_mislocated uevents stock emitted.
 *
 * Deliberately NOT ported (v1): the 32-bit SRAM/OTP paths (firmware
 * update of the cps8601 itself - endian/password/reprogram/flash flows).
 * Flashing is owner-gated follow-up work; charging does not need it.
 *
 * Copyright (C) 2020-2022 Oplus.  Mainline port 2026.
 */


#define cps_log(fmt, ...) do { if (!cps_silent) printk(KERN_INFO "oplus_cps8601: " fmt, ##__VA_ARGS__); } while (0)
#define cps_err(fmt, ...) printk(KERN_ERR "oplus_cps8601: " fmt, ##__VA_ARGS__)

#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/auxiliary_bus.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/timekeeping.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include <linux/gpio/legacy.h>

/* -------------------------------------------------------------------- */
/* Chip identity / protocol constants (verbatim from the vendor driver). */
/* -------------------------------------------------------------------- */

#define CPS_WLS_FAIL		-1
#define CPS_WLS_SUCCESS		0

/* 32-bit (long-reg) windows - reserved for fw-update follow-up, kept for
 * documentation parity with the vendor driver. */
#define ADDR_EN_32BIT_IIC	0xFFFFFF00
#define ADDR_IIC_ENDIAN		0x40040030
#define ADDR_PASSWORD		0x40040070
#define ADDR_SYSTEM_CMD		0x40040008
#define ADDR_DISABLE_DMA	0x4000E000
#define ADDR_WDOG_COUNT_OVER	0x40040048

#define PASSWORD		0x0000A061
#define CMD_EN_32BIT_IIC	0x0000000E
#define CMD_IIC_LITTLE_ENDIAN	0xFFFFFFFF
#define CMD_RESET_ALL_SYS	0x00000001

/* tx interrupt types */
#define TX_INT_WAKEUP		(0x01 << 0)
#define TX_INT_PING		(0x01 << 1)
#define TX_INT_SSP		(0x01 << 2)
#define TX_INT_IDP		(0x01 << 3)
#define TX_INT_CFGP		(0x01 << 4)
#define TX_INT_ASK_PKT		(0x01 << 5)
#define TX_INT_EPT		(0x01 << 6)
#define TX_INT_RPP_TO		(0x01 << 7)
#define TX_INT_CEP_TO		(0x01 << 8)
#define TX_INT_RX_ATTACH	(0x01 << 9)
#define TX_INT_RX_REMOVED	(0x01 << 10)
#define TX_INT_Q_CALI		(0x01 << 12)
#define TX_INT_MIS_LOC		(0x01 << 13)

/* tx commands */
#define ENTER_DEEP_LOW_POWER_MODE	(0x01 << 0)
#define TX_CMD_ENTER_TX_MODE		(0x01 << 1)
#define TX_CMD_CRC_CHERCK		(0x01 << 2)
#define TX_CMD_EXIT_TX_MODE		(0x01 << 3)
#define TX_CMD_SEND_FSK			(0x01 << 4)
#define TX_CMD_RESET_SYS		(0x01 << 5)
#define TX_CMD_START_Q_CAIL		(0x01 << 6)

/* tx EPT codes */
#define EPT_OCP			(0x01 << 6)
#define EPT_OVP			(0x01 << 7)
#define EPT_UVP			(0x01 << 8)
#define EPT_FOD			(0x01 << 9)
#define EPT_OTP			(0x01 << 10)
#define EPT_POCP		(0x01 << 11)

/* ASK/BC packet markers from the pen (carries BLE MAC + SoC) */
#define MAC_PKG_HEAD		0x48
#define MAC_PKG_CMD_CKECK	0xC1
#define MAC_PKG_CMD_ADD1	0xB6
#define MAC_PKG_CMD_ADD2	0xB7
#define POWER_PKG_HEAD		0x28
#define POWER_PKG_CMD		0x17
#define TX_PKG_LEN		11

#define CPS8601_CHIPID		0x8601

#define P9418_WAIT_TIME		50
#define MIN_TO_MS		60000
#define MAX_RETRY		5
#define BOOST_SET_RETRY		60
#define BOOST_SET_DELAY_MS	500
#define BOOST_WORK_MAX_RETRY	10
#define BOOST_DEFAULT_VOLT_MV	7000
#define BOOST_ATTACH_VOLT_MV	10000
#define MAX_BOOST_VOLT_MV	14750
#define MIN_BOOST_VOLT_MV	2000
#define DEFAULT_BOOST_VOLT_MV	5800
#define SOC_THRESHOLD		90
#define POWER_EXPIRED_DEFAULT	120

#define CPS_OCP_THRESHOLD	500
#define CPS_OVP_THRESHOLD	12000
#define CPS_LVP_THRESHOLD	4000
#define CPS_FOD_THRESHOLD	400

#define CPS_WLS_CHRG_DRV_NAME	"cps-wls-charger"
#define CPS8601_CHIPID_STR	"0x8601"

enum pen_status {
	PEN_STATUS_UNDEFINED,
	PEN_STATUS_NEAR,
	PEN_STATUS_FAR,
};

enum power_disable_reason {
	PEN_REASON_UNKNOWN,
	PEN_REASON_FAR,
	PEN_REASON_CHARGE_FULL,
	PEN_REASON_CHARGE_TIMEOUT,
	PEN_REASON_CHARGE_STOP,
	PEN_REASON_CHARGE_EPT,
	PEN_REASON_CHARGE_OCP,
	PEN_OFF_REASON_MAX,
};

enum q_cali_status {
	Q_CALI_UNKNOWN,
	Q_CALI_IN_PROGRESS,
	Q_CALI_SUCCESS,
	Q_CALI_FAIL,
	Q_CALI_CALIBRATED,
	Q_CALI_UNCALIBRATED,
};

enum hboost_status {
	HBOOST_UNKNOWN,
	HBOOST_SET_FAIL,
	HBOOST_SET_SUCCESS,
	HBOOST_IS_SETTING_BOOST,
};

#define ENTER_CALI_MODE		0xAA
#define EXIT_CALI_MODE		0xFF
#define SYS_MODE_TX		1


/* Register map of the chip's 16-bit access window. */
enum {
	CPS_TX_REG_PPP_HEADER,
	CPS_TX_REG_PPP_COMMAND,
	CPS_TX_REG_PPP_DATA0,
	CPS_TX_REG_PPP_DATA1,
	CPS_TX_REG_PPP_DATA2,
	CPS_TX_REG_PPP_DATA3,
	CPS_TX_REG_PPP_DATA4,
	CPS_TX_REG_PPP_DATA5,
	CPS_TX_REG_PPP_DATA6,
	CPS_TX_REG_PPP_DATA7,
	CPS_TX_REG_PPP_DATA8,
	CPS_TX_REG_BC_HEADER,
	CPS_TX_REG_BC_COMMAND,
	CPS_TX_REG_BC_DATA0,
	CPS_TX_REG_BC_DATA1,
	CPS_TX_REG_BC_DATA2,
	CPS_TX_REG_BC_DATA3,
	CPS_TX_REG_BC_DATA4,
	CPS_TX_REG_BC_DATA5,
	CPS_TX_REG_BC_DATA6,
	CPS_TX_REG_BC_DATA7,
	CPS_TX_REG_BC_DATA8,
	CPS_TX_REG_CNT_VARIANCE,
	CPS_TX_REG_WIDTH_VARIANCE,
	CPS_TX_REG_CALI_TIMES,
	CPS_TX_REG_NO_RX_CNT,
	CPS_TX_REG_NO_RX_WIDTH,
	CPS_TX_REG_OCP_TH,
	CPS_TX_REG_UVP_TH,
	CPS_TX_REG_OVP_TH,
	CPS_TX_REG_FOP_MIN,
	CPS_TX_REG_FOP_MAX,
	CPS_TX_REG_PING_FREQ,
	CPS_TX_REG_PING_DUTY,
	CPS_TX_REG_PING_TIME,
	CPS_TX_REG_PING_INTERVAL,
	CPS_TX_REG_FOD0_TH,
	CPS_TX_REG_FOP_VAL,
	CPS_TX_REG_ADC_VIN,
	CPS_TX_REG_ADC_VRECT,
	CPS_TX_REG_ADC_I_IN,
	CPS_TX_REG_EPT_RSN,
	CPS_TX_REG_ADC_DIE_TEMP,
	CPS_TX_REG_EPT_CODE,
	CPS_TX_REG_MAX
};

enum {
	CPS_COMM_REG_CHIP_ID,
	CPS_COMM_REG_FW_VER,
	CPS_COMM_REG_SYS_MODE,
	CPS_COMM_REG_INT_EN,
	CPS_COMM_REG_INT_FLAG,
	CPS_COMM_REG_INT_CLR,
	CPS_COMM_REG_CMD,
	CPS_COMN_REG_Q_CALI_CNT,
	CPS_COMN_REG_Q_CALI_WIDTH,
	CPS_COMM_REG_Q_CALI_RES,
	CPS_COMM_REG_Q_CALI_MODE,
	CPS_COMM_REG_MAX
};

struct cps_reg_s {
	unsigned short reg_name;
	unsigned short reg_bytes_len;
	unsigned int reg_addr;
};

static const struct cps_reg_s cps_comm_reg[CPS_COMM_REG_MAX] = {
	{CPS_COMM_REG_CHIP_ID,		2, 0x0000},
	{CPS_COMM_REG_FW_VER,		2, 0x0002},
	{CPS_COMM_REG_SYS_MODE,		1, 0x0004},
	{CPS_COMM_REG_INT_EN,		2, 0x0005},
	{CPS_COMM_REG_INT_FLAG,		2, 0x0007},
	{CPS_COMM_REG_INT_CLR,		2, 0x0009},
	{CPS_COMM_REG_CMD,		1, 0x000B},
	{CPS_COMN_REG_Q_CALI_CNT,	1, 0x000D},
	{CPS_COMN_REG_Q_CALI_WIDTH,	2, 0x000E},
	{CPS_COMM_REG_Q_CALI_RES,	1, 0x0013},
	{CPS_COMM_REG_Q_CALI_MODE,	1, 0x0015},
};

static const struct cps_reg_s cps_tx_reg[CPS_TX_REG_MAX] = {
	{CPS_TX_REG_PPP_HEADER,		1, 0x0040},
	{CPS_TX_REG_PPP_COMMAND,	1, 0x0041},
	{CPS_TX_REG_PPP_DATA0,		1, 0x0042},
	{CPS_TX_REG_PPP_DATA1,		1, 0x0043},
	{CPS_TX_REG_PPP_DATA2,		1, 0x0044},
	{CPS_TX_REG_PPP_DATA3,		1, 0x0045},
	{CPS_TX_REG_PPP_DATA4,		1, 0x0046},
	{CPS_TX_REG_PPP_DATA5,		1, 0x0047},
	{CPS_TX_REG_PPP_DATA6,		1, 0x0048},
	{CPS_TX_REG_PPP_DATA7,		1, 0x0049},
	{CPS_TX_REG_PPP_DATA8,		1, 0x004A},
	{CPS_TX_REG_BC_HEADER,		1, 0x004B},
	{CPS_TX_REG_BC_COMMAND,		1, 0x004C},
	{CPS_TX_REG_BC_DATA0,		1, 0x004D},
	{CPS_TX_REG_BC_DATA1,		1, 0x004E},
	{CPS_TX_REG_BC_DATA2,		1, 0x004F},
	{CPS_TX_REG_BC_DATA3,		1, 0x0050},
	{CPS_TX_REG_BC_DATA4,		1, 0x0051},
	{CPS_TX_REG_BC_DATA5,		1, 0x0052},
	{CPS_TX_REG_BC_DATA6,		1, 0x0053},
	{CPS_TX_REG_BC_DATA7,		1, 0x0054},
	{CPS_TX_REG_BC_DATA8,		1, 0x0055},
	{CPS_TX_REG_CNT_VARIANCE,	2, 0x0056},
	{CPS_TX_REG_WIDTH_VARIANCE,	2, 0x0058},
	{CPS_TX_REG_CALI_TIMES,		1, 0x005A},
	{CPS_TX_REG_NO_RX_CNT,		1, 0x0010},
	{CPS_TX_REG_NO_RX_WIDTH,		2, 0x0011},
	{CPS_TX_REG_OCP_TH,		2, 0x0028},
	{CPS_TX_REG_UVP_TH,		2, 0x002C},
	{CPS_TX_REG_OVP_TH,		2, 0x002A},
	{CPS_TX_REG_FOP_MIN,		2, 0x0020},
	{CPS_TX_REG_FOP_MAX,		2, 0x0022},
	{CPS_TX_REG_PING_FREQ,		2, 0x0024},
	{CPS_TX_REG_PING_DUTY,		2, 0x001E},
	{CPS_TX_REG_PING_TIME,		2, 0x0016},
	{CPS_TX_REG_PING_INTERVAL,	2, 0x0018},
	{CPS_TX_REG_FOD0_TH,		2, 0x0034},
	{CPS_TX_REG_FOP_VAL,		2, 0x0036},
	{CPS_TX_REG_ADC_VIN,		2, 0x003C},
	{CPS_TX_REG_ADC_VRECT,		2, 0x003E},
	{CPS_TX_REG_ADC_I_IN,		2, 0x005E},
	{CPS_TX_REG_EPT_RSN,		1, 0x003A},
	{CPS_TX_REG_ADC_DIE_TEMP,	1, 0x0033},
	{CPS_TX_REG_EPT_CODE,		1, 0x003B},
};

static const struct regmap_config cps8601_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
};

static const struct regmap_config cps8601_regmap32_config = {
	.reg_bits = 32,
	.val_bits = 8,
};


/* -------------------------------------------------------------------- */
/* Driver object + tunables                                              */
/* -------------------------------------------------------------------- */

struct cps_wls_chip;
struct cps_glink_aux;

struct cps_wls_chip {
	struct i2c_client *client;
	struct device *dev;
	struct regmap *regmap;
	struct regmap *regmap32;

	int wls_charge_int;
	int cps_wls_irq;
	int wls_off_state_gpio;
	int wls_sleep_gpio;
	int wls_scan_gpio;
	int wls_sw_en_gpio;

	int ocp_threshold;
	int ovp_threshold;
	int lvp_threshold;
	int fod_threshold;
	int power_expired_time;
	int is_supply_by_hboost;
	int hboost_default_volt;
	int hboost_attach_volt;
	int hboost_volt;
	int hboost_status;
	int soc_threshold;
	int pen_support_track;

	int chip_id;
	unsigned short fw_version;
	int pen_present;
	unsigned int pen_status;
	unsigned int charge_allow;
	unsigned int power_disable_reason;
	int q_cali_status;
	int cali_times;
	u64 ble_mac_addr;
	u64 mac_check_data;
	int tx_voltage;
	int tx_current;
	int pen_soc;
	u64 ping_succ_time;
	u64 tx_start_time;

	bool i2c_ready;
	bool init_done;
	bool charging;
	bool led_on;
	bool fw_update_warned;

	struct mutex irq_lock;
	struct mutex i2c_lock;
	struct mutex glk_lock;

	struct pmic_glink_client *glk_client;
	struct cps_glink_aux *glk_aux;
	struct delayed_work glk_retry_work;
	unsigned int glk_tries;
	struct completion glk_ack;
	int glk_ready;

	struct work_struct init_work;
	struct work_struct restate_work;
	struct delayed_work expire_work;
	struct dentry *debug_root;
};

struct cps_wls_chip *g_chip;

static unsigned int cps_enabled = 1;
module_param(cps_enabled, uint, 0644);
MODULE_PARM_DESC(cps_enabled,
		 "0 disables the wireless-pen charger entirely (default 1)");

static unsigned int cps_uevent = 0;
module_param(cps_uevent, uint, 0644);
MODULE_PARM_DESC(cps_uevent,
		 "emit pencil_status/pencil_addr kobj uevents (default 0)");

static unsigned int cps_silent = 0;
module_param(cps_silent, uint, 0644);

/* =0 disables the pmic_glink H-BoOST rail path entirely (rail never set) */
static unsigned int cps_hboost_glink = 1;
module_param_named(erhai_hboost_glink, cps_hboost_glink, uint, 0644);
MODULE_PARM_DESC(cps_silent,
		 "1 = quiet (errors only) logging (default 0)");


/* -------------------------------------------------------------------- */
/* Register access (16-bit app window + 32-bit raw, like the vendor drv) */
/* -------------------------------------------------------------------- */

static int cps_wls_l_write_reg(struct cps_wls_chip *chip, int reg, int value)
{
	int ret;

	if (!chip)
		return CPS_WLS_FAIL;
	mutex_lock(&chip->i2c_lock);
	ret = regmap_write(chip->regmap, (unsigned int)reg, (unsigned int)value);
	mutex_unlock(&chip->i2c_lock);
	if (ret < 0) {
		cps_err("i2c write reg 0x%x error %d\n", reg, ret);
		return CPS_WLS_FAIL;
	}
	return CPS_WLS_SUCCESS;
}

static int cps_wls_l_read_reg(struct cps_wls_chip *chip, int reg)
{
	unsigned int value = 0;
	int ret;

	if (!chip)
		return CPS_WLS_FAIL;
	mutex_lock(&chip->i2c_lock);
	ret = regmap_read(chip->regmap, (unsigned int)reg, &value);
	mutex_unlock(&chip->i2c_lock);
	if (ret < 0) {
		cps_err("i2c read reg 0x%x error %d\n", reg, ret);
		return CPS_WLS_FAIL;
	}
	return (int)value;
}

static int cps_wls_read_reg(struct cps_wls_chip *chip, int reg, int byte_len)
{
	int i, tmp, read_value = 0;

	for (i = 0; i < byte_len; i++) {
		tmp = cps_wls_l_read_reg(chip, (reg & 0xffff) + i);
		if (tmp == CPS_WLS_FAIL)
			return CPS_WLS_FAIL;
		read_value |= (tmp << (8 * i));
	}
	return read_value;
}

static int cps_wls_write_reg(struct cps_wls_chip *chip, int reg, int value, int byte_len)
{
	int i, tmp;

	for (i = 0; i < byte_len; i++) {
		tmp = (value >> (i * 8)) & 0xff;
		if (cps_wls_l_write_reg(chip, (reg & 0xffff) + i, tmp) == CPS_WLS_FAIL)
			return CPS_WLS_FAIL;
	}
	return CPS_WLS_SUCCESS;
}

/* 32-bit raw word access (unused in v1 but kept for the fw-update follow-up). */
static int cps_wls_write_word(struct cps_wls_chip *chip, int reg, int value)
{
	u8 data[4];
	int ret;

	data[3] = (value >> 24) & 0xff;
	data[2] = (value >> 16) & 0xff;
	data[1] = (value >> 8) & 0xff;
	data[0] = value & 0xff;
	mutex_lock(&chip->i2c_lock);
	ret = regmap_raw_write(chip->regmap32, (unsigned int)reg, data, 4);
	mutex_unlock(&chip->i2c_lock);
	return ret < 0 ? CPS_WLS_FAIL : CPS_WLS_SUCCESS;
}

static int cps_wls_read_word(struct cps_wls_chip *chip, int addr, u32 *out)
{
	u8 data[4];
	int ret;

	mutex_lock(&chip->i2c_lock);
	ret = regmap_raw_read(chip->regmap32, (unsigned int)addr, data, 4);
	mutex_unlock(&chip->i2c_lock);
	if (ret < 0)
		return CPS_WLS_FAIL;
	*out = (u32)data[0] | ((u32)data[1] << 8) | ((u32)data[2] << 16) |
	       ((u32)data[3] << 24);
	return CPS_WLS_SUCCESS;
}


/* -------- common-register wrappers -------- */

static int cps_wls_get_chip_id(struct cps_wls_chip *chip)
{
	return cps_wls_read_reg(chip,
		cps_comm_reg[CPS_COMM_REG_CHIP_ID].reg_addr,
		cps_comm_reg[CPS_COMM_REG_CHIP_ID].reg_bytes_len);
}

static int cps_wls_get_sys_fw_version(struct cps_wls_chip *chip)
{
	return cps_wls_read_reg(chip,
		cps_comm_reg[CPS_COMM_REG_FW_VER].reg_addr,
		cps_comm_reg[CPS_COMM_REG_FW_VER].reg_bytes_len);
}

static int cps_wls_set_int_enable(struct cps_wls_chip *chip)
{
	return cps_wls_write_reg(chip,
		cps_comm_reg[CPS_COMM_REG_INT_EN].reg_addr, 0xFFFF,
		cps_comm_reg[CPS_COMM_REG_INT_EN].reg_bytes_len);
}

static int cps_wls_get_int_flag(struct cps_wls_chip *chip)
{
	return cps_wls_read_reg(chip,
		cps_comm_reg[CPS_COMM_REG_INT_FLAG].reg_addr,
		cps_comm_reg[CPS_COMM_REG_INT_FLAG].reg_bytes_len);
}

static int cps_wls_set_int_clr(struct cps_wls_chip *chip, int value)
{
	return cps_wls_write_reg(chip,
		cps_comm_reg[CPS_COMM_REG_INT_CLR].reg_addr, value,
		cps_comm_reg[CPS_COMM_REG_INT_CLR].reg_bytes_len);
}

static int cps_wls_get_cmd(struct cps_wls_chip *chip)
{
	return cps_wls_read_reg(chip,
		cps_comm_reg[CPS_COMM_REG_CMD].reg_addr,
		cps_comm_reg[CPS_COMM_REG_CMD].reg_bytes_len);
}

static int cps_wls_set_cmd(struct cps_wls_chip *chip, int value)
{
	return cps_wls_write_reg(chip,
		cps_comm_reg[CPS_COMM_REG_CMD].reg_addr, value,
		cps_comm_reg[CPS_COMM_REG_CMD].reg_bytes_len);
}

static int cps_wls_enable_tx_mode(struct cps_wls_chip *chip)
{
	int cmd = cps_wls_get_cmd(chip);

	if (cmd == CPS_WLS_FAIL)
		return CPS_WLS_FAIL;
	return cps_wls_set_cmd(chip, cmd | TX_CMD_ENTER_TX_MODE);
}

static int cps_wls_reset_sys(struct cps_wls_chip *chip)
{
	int cmd = cps_wls_get_cmd(chip);

	if (cmd == CPS_WLS_FAIL)
		return CPS_WLS_FAIL;
	return cps_wls_set_cmd(chip, cmd | TX_CMD_RESET_SYS);
}

static int cps_wls_disable_tx_mode(struct cps_wls_chip *chip)
{
	int cmd = cps_wls_get_cmd(chip);

	if (cmd == CPS_WLS_FAIL)
		return CPS_WLS_FAIL;
	return cps_wls_set_cmd(chip, cmd | TX_CMD_EXIT_TX_MODE);
}

static int cps_wls_start_q_cali(struct cps_wls_chip *chip)
{
	int cmd = cps_wls_get_cmd(chip);

	if (cmd == CPS_WLS_FAIL)
		return CPS_WLS_FAIL;
	return cps_wls_set_cmd(chip, cmd | TX_CMD_START_Q_CAIL);
}

/* -------- tx-register wrappers -------- */

static int cps_wls_get_tx_i_in(struct cps_wls_chip *chip)
{
	return cps_wls_read_reg(chip,
		cps_tx_reg[CPS_TX_REG_ADC_I_IN].reg_addr,
		cps_tx_reg[CPS_TX_REG_ADC_I_IN].reg_bytes_len);
}

static int cps_wls_get_tx_vin(struct cps_wls_chip *chip)
{
	return cps_wls_read_reg(chip,
		cps_tx_reg[CPS_TX_REG_ADC_VIN].reg_addr,
		cps_tx_reg[CPS_TX_REG_ADC_VIN].reg_bytes_len);
}

static int cps_wls_get_tx_ept_type(struct cps_wls_chip *chip)
{
	return cps_wls_read_reg(chip,
		cps_tx_reg[CPS_TX_REG_EPT_CODE].reg_addr,
		cps_tx_reg[CPS_TX_REG_EPT_CODE].reg_bytes_len);
}

static int cps_wls_set_tx_ocp_threshold(struct cps_wls_chip *chip, int value)
{
	return cps_wls_write_reg(chip,
		cps_tx_reg[CPS_TX_REG_OCP_TH].reg_addr, value,
		cps_tx_reg[CPS_TX_REG_OCP_TH].reg_bytes_len);
}

static int cps_wls_set_tx_uvp_threshold(struct cps_wls_chip *chip, int value)
{
	return cps_wls_write_reg(chip,
		cps_tx_reg[CPS_TX_REG_UVP_TH].reg_addr, value,
		cps_tx_reg[CPS_TX_REG_UVP_TH].reg_bytes_len);
}

static int cps_wls_set_tx_ovp_threshold(struct cps_wls_chip *chip, int value)
{
	return cps_wls_write_reg(chip,
		cps_tx_reg[CPS_TX_REG_OVP_TH].reg_addr, value,
		cps_tx_reg[CPS_TX_REG_OVP_TH].reg_bytes_len);
}

static int cps_wls_set_tx_fod_thresh(struct cps_wls_chip *chip, int value)
{
	return cps_wls_write_reg(chip,
		cps_tx_reg[CPS_TX_REG_FOD0_TH].reg_addr, value,
		cps_tx_reg[CPS_TX_REG_FOD0_TH].reg_bytes_len);
}

static int cps_wls_get_q_cali_count(struct cps_wls_chip *chip)
{
	return cps_wls_read_reg(chip,
		cps_comm_reg[CPS_COMN_REG_Q_CALI_CNT].reg_addr,
		cps_comm_reg[CPS_COMN_REG_Q_CALI_CNT].reg_bytes_len);
}

static int cps_wls_get_q_cali_width(struct cps_wls_chip *chip)
{
	return cps_wls_read_reg(chip,
		cps_comm_reg[CPS_COMN_REG_Q_CALI_WIDTH].reg_addr,
		cps_comm_reg[CPS_COMN_REG_Q_CALI_WIDTH].reg_bytes_len);
}

static int cps_wls_get_cali_times(struct cps_wls_chip *chip)
{
	return cps_wls_read_reg(chip,
		cps_tx_reg[CPS_TX_REG_CALI_TIMES].reg_addr,
		cps_tx_reg[CPS_TX_REG_CALI_TIMES].reg_bytes_len);
}

static int cps_wls_get_cnt_variance(struct cps_wls_chip *chip)
{
	return cps_wls_read_reg(chip,
		cps_tx_reg[CPS_TX_REG_CNT_VARIANCE].reg_addr,
		cps_tx_reg[CPS_TX_REG_CNT_VARIANCE].reg_bytes_len);
}

static int cps_wls_get_width_variance(struct cps_wls_chip *chip)
{
	return cps_wls_read_reg(chip,
		cps_tx_reg[CPS_TX_REG_WIDTH_VARIANCE].reg_addr,
		cps_tx_reg[CPS_TX_REG_WIDTH_VARIANCE].reg_bytes_len);
}

static int cps_wls_get_is_cali_success(struct cps_wls_chip *chip)
{
	return cps_wls_read_reg(chip,
		cps_comm_reg[CPS_COMM_REG_Q_CALI_RES].reg_addr,
		cps_comm_reg[CPS_COMM_REG_Q_CALI_RES].reg_bytes_len);
}

static int cps_wls_set_cali_mode(struct cps_wls_chip *chip, bool cali_mode)
{
	return cps_wls_write_reg(chip,
		cps_comm_reg[CPS_COMM_REG_Q_CALI_MODE].reg_addr,
		cali_mode ? ENTER_CALI_MODE : EXIT_CALI_MODE,
		cps_comm_reg[CPS_COMM_REG_Q_CALI_MODE].reg_bytes_len);
}


/* -------------------------------------------------------------------- */
/* GPIO + device-tree plumbing                                           */
/* -------------------------------------------------------------------- */

static int request_named_gpio(struct cps_wls_chip *chip, const char *prop,
			      const char *label, int output)
{
	struct device_node *node = chip->dev->of_node;
	int gp, rc;

	if (!node)
		return -EINVAL;
	gp = of_get_named_gpio(node, prop, 0);
	if (gp < 0) {
		cps_err("%s not specified in DT\n", prop);
		return -EINVAL;
	}
	if (!gpio_is_valid(gp))
		return 0;
	rc = gpio_request(gp, label);
	if (rc) {
		cps_err("failed to request gpio %d (%s): %d\n", gp, label, rc);
		return rc;
	}
	if (output)
		gpio_direction_output(gp, 0);
	else
		gpio_direction_input(gp);
	return gp;
}

static void free_named_gpio(struct cps_wls_chip *chip)
{
	int *pins[] = { &chip->wls_charge_int, &chip->wls_sw_en_gpio,
			 &chip->wls_sleep_gpio, &chip->wls_scan_gpio,
			 &chip->wls_off_state_gpio };
	int i;

	for (i = 0; i < sizeof(pins) / sizeof(pins[0]); i++)
		if (gpio_is_valid(*pins[i])) {
			gpio_free(*pins[i]);
			*pins[i] = -EINVAL;
		}
}

static int cps_wls_gpio_init(struct cps_wls_chip *chip)
{
	int gp;

	gp = request_named_gpio(chip, "qcom,cps_wls_int-gpio", "cps-wls-int", 0);
	if (gp < 0)
		return gp;
	chip->wls_charge_int = gp;
	chip->cps_wls_irq = gpio_to_irq(gp);

	gp = request_named_gpio(chip, "qcom,wls_sw_en-gpio", "cps-wls-swen", 1);
	if (gp < 0)
		return gp;
	chip->wls_sw_en_gpio = gp;

	gp = request_named_gpio(chip, "qcom,wls_sleep-gpio", "cps-wls-sleep", 1);
	if (gp < 0)
		return gp;
	chip->wls_sleep_gpio = gp;

	gp = request_named_gpio(chip, "qcom,wls_scan-gpio", "cps-wls-scan", 1);
	if (gp < 0)
		return gp;
	chip->wls_scan_gpio = gp;

	gp = request_named_gpio(chip, "qcom,wls_off_state-gpio", "cps-wls-off", 1);
	if (gp < 0)
		return gp;
	chip->wls_off_state_gpio = gp;

	cps_log("GPIOs: int=%d sw_en=%d sleep=%d scan=%d off=%d irq=%d\n",
		chip->wls_charge_int, chip->wls_sw_en_gpio, chip->wls_sleep_gpio,
		chip->wls_scan_gpio, chip->wls_off_state_gpio, chip->cps_wls_irq);
	return 0;
}

static void cps_set_hw_state(struct cps_wls_chip *chip, bool charging)
{
	/* sw_en powers the WPT subsystem; scan tells the chip to ping; the
	 * off-state line marks "not charging" and the sleep line stays low. */
	if (gpio_is_valid(chip->wls_sw_en_gpio))
		gpio_direction_output(chip->wls_sw_en_gpio, 1);
	if (gpio_is_valid(chip->wls_scan_gpio))
		gpio_direction_output(chip->wls_scan_gpio, charging ? 0 : 1);
	if (gpio_is_valid(chip->wls_off_state_gpio))
		gpio_direction_output(chip->wls_off_state_gpio, charging ? 0 : 1);
	if (gpio_is_valid(chip->wls_sleep_gpio))
		gpio_direction_output(chip->wls_sleep_gpio, 0);
}

static void cps_set_hw_idle(struct cps_wls_chip *chip)
{
	if (gpio_is_valid(chip->wls_sw_en_gpio))
		gpio_direction_output(chip->wls_sw_en_gpio, 0);
	if (gpio_is_valid(chip->wls_scan_gpio))
		gpio_direction_output(chip->wls_scan_gpio, 0);
	if (gpio_is_valid(chip->wls_off_state_gpio))
		gpio_direction_output(chip->wls_off_state_gpio, 0);
	if (gpio_is_valid(chip->wls_sleep_gpio))
		gpio_direction_output(chip->wls_sleep_gpio, 0);
}

static int cps_wls_parse_dt(struct cps_wls_chip *chip)
{
	struct device_node *node = chip->dev->of_node;
	unsigned int v;

	if (!node)
		return -EINVAL;

	chip->ocp_threshold = CPS_OCP_THRESHOLD;
	chip->ovp_threshold = CPS_OVP_THRESHOLD;
	chip->lvp_threshold = CPS_LVP_THRESHOLD;
	chip->fod_threshold = CPS_FOD_THRESHOLD;
	chip->power_expired_time = POWER_EXPIRED_DEFAULT;
	chip->soc_threshold = SOC_THRESHOLD;
	chip->is_supply_by_hboost = 1;
	chip->hboost_default_volt = BOOST_DEFAULT_VOLT_MV;
	chip->hboost_attach_volt = BOOST_ATTACH_VOLT_MV;

	if (!of_property_read_u32(node, "qcom,ocp_threshold", &v))
		chip->ocp_threshold = (int)v;
	if (!of_property_read_u32(node, "qcom,ovp_threshold", &v))
		chip->ovp_threshold = (int)v;
	if (!of_property_read_u32(node, "qcom,lvp_threshold", &v))
		chip->lvp_threshold = (int)v;
	if (!of_property_read_u32(node, "qcom,fod_threshold", &v))
		chip->fod_threshold = (int)v;
	if (!of_property_read_u32(node, "tx,enable_expired_time", &v))
		chip->power_expired_time = (int)v;
	if (!of_property_read_u32(node, "rx,soc_threshold", &v))
		chip->soc_threshold = (int)v;
	if (!of_property_read_u32(node, "qcom,pen_is_supply_by_hboost", &v))
		chip->is_supply_by_hboost = (int)v;
	if (!of_property_read_u32(node, "qcom,hboost_default_volt_mv", &v)) {
		chip->hboost_default_volt = (int)v;
		chip->hboost_volt = (int)v;
	}
	if (!of_property_read_u32(node, "qcom,hboost_attach_volt_mv", &v))
		chip->hboost_attach_volt = (int)v;

	cps_log("DT: ocp=%d ovp=%d uvp=%d fod=%d expired=%ds soc=%d%% hboost=%d/%dmV\n",
		chip->ocp_threshold, chip->ovp_threshold, chip->lvp_threshold,
		chip->fod_threshold, chip->power_expired_time, chip->soc_threshold,
		chip->hboost_default_volt, chip->hboost_attach_volt);
	return 0;
}

static int cps_wls_set_protect_parameter(struct cps_wls_chip *chip)
{
	cps_wls_set_tx_ocp_threshold(chip, chip->ocp_threshold);
	cps_wls_set_tx_uvp_threshold(chip, chip->lvp_threshold);
	cps_wls_set_tx_ovp_threshold(chip, chip->ovp_threshold);
	cps_wls_set_tx_fod_thresh(chip, chip->fod_threshold);
	return 0;
}

/* -------------------------------------------------------------------- */
/* H-BoOST supply rail over pmic_glink (owner 32785 = wireless pen)      */
/* -------------------------------------------------------------------- */

#define MSG_OWNER_HBOOST		32785
#define MSG_TYPE_REQ_RESP		1
#define OEM_OPCODE_SET_HBOOST_VOLT	0x10007
#define BC_WAIT_TIME_MS			2500

struct wireless_pen_set_hboost_vout_req {
	struct pmic_glink_hdr hdr;
	u8 reg_vout;
};

struct wireless_pen_set_hboost_vout_resp {
	struct pmic_glink_hdr hdr;
	u32 ret_code;
};

static void cps_glink_cb(const void *data, size_t len, void *priv)
{
	struct cps_wls_chip *chip = priv;
	const struct wireless_pen_set_hboost_vout_resp *resp = data;

	if (!chip || !data)
		return;
	if (len < sizeof(struct pmic_glink_hdr))
		return;
	if (resp->hdr.opcode == OEM_OPCODE_SET_HBOOST_VOLT) {
		if (resp->ret_code == 0)
			complete(&chip->glk_ack);
		else
			cps_err("hboost set rejected by PMIC (ret 0x%x)\n",
				resp->ret_code);
	}
}

static void cps_glink_pdr(void *priv, int state)
{
	struct cps_wls_chip *chip = priv;

	if (!chip)
		return;
	chip->glk_ready = (state == 0);
	if (state == 0)
		complete(&chip->glk_ack);
}

/*
 * The glink client is created against an AUXILIARY device on the
 * pmic_glink platform device.  devm_pmic_glink_client_alloc() derives the
 * pmic_glink pointer from dev->parent's drvdata, so the client MUST hang
 * off the pmic_glink device -- an i2c client's parent is the i2c adapter,
 * and locking its "state_lock" offset would deadlock the whole boot (the
 * +b10 hang).  This device is created asynchronously from init_work so the
 * i2c probe never blocks on the glink bus; if pmic_glink never appears the
 * rail path just stays unset and charging stays blocked (safe).
 */
#define GLK_RETRY_MAX		15
#define GLK_RETRY_DELAY_MS	1000

struct cps_glink_aux {
	struct auxiliary_device adev;
	struct cps_wls_chip *chip;
};

static int cps_glink_aux_probe(struct auxiliary_device *adev,
			       const struct auxiliary_device_id *id)
{
	struct cps_glink_aux *ga =
		container_of(adev, struct cps_glink_aux, adev);
	struct cps_wls_chip *chip = ga->chip;

	if (!chip || chip->glk_client)
		return 0;

	chip->glk_client = devm_pmic_glink_client_alloc(&adev->dev,
						       MSG_OWNER_HBOOST,
						       cps_glink_cb,
						       cps_glink_pdr,
						       chip);
	if (IS_ERR(chip->glk_client)) {
		cps_err("hboost: glink client alloc failed %ld\n",
			PTR_ERR(chip->glk_client));
		chip->glk_client = NULL;
		return 0;
	}
	pmic_glink_client_register(chip->glk_client);
	chip->glk_ready = 0;
	cps_log("hboost glink client up (owner %u)\n", MSG_OWNER_HBOOST);
	return 0;
}

static void cps_glink_aux_remove(struct auxiliary_device *adev)
{
	struct cps_glink_aux *ga =
		container_of(adev, struct cps_glink_aux, adev);

	if (ga && ga->chip)
		ga->chip->glk_client = NULL;
}

static const struct auxiliary_device_id cps_glink_aux_ids[] = {
	{ .name = "pmic_glink.cps8601-wls" },
	{}
};
MODULE_DEVICE_TABLE(auxiliary, cps_glink_aux_ids);

static struct auxiliary_driver cps_glink_aux_driver = {
	.probe = cps_glink_aux_probe,
	.remove = cps_glink_aux_remove,
	.id_table = cps_glink_aux_ids,
	.driver = {
		.name = "cps8601_hboost_glk",
	},
};

static int cps_glink_device_create(struct cps_wls_chip *chip)
{
	const char *const compat[] = {
		"qcom,pmic-glink", "qcom,glymur-pmic-glink",
		"qcom,kaanapali-pmic-glink"
	};
	struct device_node *pg_node = NULL;
	struct platform_device *pg_pdev;
	struct cps_glink_aux *ga;
	int i, ret;

	if (!cps_hboost_glink || chip->glk_aux)
		return 0;

	for (i = 0; i < ARRAY_SIZE(compat); i++) {
		pg_node = of_find_compatible_node(NULL, NULL, compat[i]);
		if (pg_node)
			break;
	}
	if (!pg_node) {
		cps_log("hboost: pmic_glink node not bound yet\n");
		return -EAGAIN;
	}
	pg_pdev = of_find_device_by_node(pg_node);
	of_node_put(pg_node);
	if (!pg_pdev) {
		cps_log("hboost: pmic_glink platform device not created yet\n");
		return -EAGAIN;
	}

	ga = kzalloc(sizeof(*ga), GFP_KERNEL);
	if (!ga) {
		put_device(&pg_pdev->dev);
		return -ENOMEM;
	}
	ga->chip = chip;
	ga->adev.name = "cps8601-wls";
	ga->adev.dev.parent = &pg_pdev->dev;

	ret = auxiliary_device_init(&ga->adev);
	if (ret)
		goto fail_free;

	/* modname "pmic_glink" (not KBUILD_MODNAME) so the device binds our
	 * "pmic_glink.cps8601-wls" id_table entry. */
	ret = __auxiliary_device_add(&ga->adev, "pmic_glink");
	if (ret) {
		auxiliary_device_uninit(&ga->adev);
		goto fail_free;
	}

	chip->glk_aux = ga;
	put_device(&pg_pdev->dev);
	cps_log("hboost glink aux device on pmic_glink (probe pending)\n");
	return 0;

fail_free:
	kfree(ga);
	put_device(&pg_pdev->dev);
	return ret;
}

static void cps_glink_destroy(struct cps_wls_chip *chip)
{
	struct cps_glink_aux *ga = chip->glk_aux;

	if (!ga)
		return;
	chip->glk_aux = NULL;
	auxiliary_device_delete(&ga->adev);
	auxiliary_device_uninit(&ga->adev);
	kfree(ga);
}

static void cps_glink_retry_work_func(struct work_struct *work)
{
	struct cps_wls_chip *chip =
		container_of(work, struct cps_wls_chip, glk_retry_work.work);
	int ret;

	if (!chip || chip->glk_aux)
		return;
	if (!cps_hboost_glink) {
		cps_err("hboost: rail disabled by erhai_hboost_glink=0\n");
		return;
	}
	if (chip->glk_tries++ >= GLK_RETRY_MAX) {
		cps_err("hboost: pmic_glink never appeared; H-BoOST rail "
			"unavailable\n");
		return;
	}
	ret = cps_glink_device_create(chip);
	if (ret == -EAGAIN)
		schedule_delayed_work(&chip->glk_retry_work,
				      msecs_to_jiffies(GLK_RETRY_DELAY_MS));
	else if (ret)
		cps_err("hboost: glink aux setup failed %d\n", ret);
}

static int cps_set_boost_volt_mv(struct cps_wls_chip *chip, int volt_mv)
{
	struct wireless_pen_set_hboost_vout_req req = { { 0 } };
	int ret, rc = CPS_WLS_SUCCESS;

	if (volt_mv > MAX_BOOST_VOLT_MV)
		volt_mv = MAX_BOOST_VOLT_MV;
	else if (volt_mv < MIN_BOOST_VOLT_MV)
		volt_mv = MIN_BOOST_VOLT_MV;

	req.reg_vout = (u8)((volt_mv - 2000) / 50);
	req.hdr.owner = MSG_OWNER_HBOOST;
	req.hdr.type = MSG_TYPE_REQ_RESP;
	req.hdr.opcode = OEM_OPCODE_SET_HBOOST_VOLT;

	if (!chip->glk_client) {
		cps_err("hboost: no glink client\n");
		return CPS_WLS_FAIL;
	}
	mutex_lock(&chip->glk_lock);
	reinit_completion(&chip->glk_ack);
	ret = pmic_glink_send(chip->glk_client, &req, sizeof(req));
	if (!ret) {
		if (!wait_for_completion_timeout(&chip->glk_ack,
						 msecs_to_jiffies(BC_WAIT_TIME_MS))) {
			cps_err("hboost: PMIC did not answer the voltage request\n");
			rc = CPS_WLS_FAIL;
		}
	} else {
		cps_err("hboost: pmic_glink_send failed %d\n", ret);
		rc = CPS_WLS_FAIL;
	}
	mutex_unlock(&chip->glk_lock);
	return rc;
}


/* -------------------------------------------------------------------- */
/* Pen-presence / MAC exchange (ASK packets over the power link)         */
/* -------------------------------------------------------------------- */

static u64 cps_recv_ble_mac_addr(u8 *data)
{
	u64 ble_addr;
	u8 decode_addr[6];

	decode_addr[5] = ((data[9] & 0x0F) << 4) | ((data[4] & 0xF0) >> 4);
	decode_addr[4] = ((data[8] & 0x0F) << 4) | ((data[3] & 0xF0) >> 4);
	decode_addr[3] = ((data[7] & 0x0F) << 4) | ((data[2] & 0xF0) >> 4);
	decode_addr[2] = ((data[4] & 0x0F) << 4) | ((data[9] & 0xF0) >> 4);
	decode_addr[1] = ((data[3] & 0x0F) << 4) | ((data[8] & 0xF0) >> 4);
	decode_addr[0] = ((data[2] & 0x0F) << 4) | ((data[7] & 0xF0) >> 4);

	ble_addr = (u64)decode_addr[5] << 40 | (u64)decode_addr[4] << 32 |
		   (u64)decode_addr[3] << 24 | (u64)decode_addr[2] << 16 |
		   (u64)decode_addr[1] << 8 | (u64)decode_addr[0];
	return ble_addr;
}

static u64 cps_recv_mac_check_data(u8 *data)
{
	return (u64)(data[4] << 16 | data[3] << 8 | data[2]);
}

static bool cps_ble_mac_valid(struct cps_wls_chip *chip)
{
	u64 addr = chip->ble_mac_addr, check = chip->mac_check_data;

	if (addr == 0 || addr == (u64)CPS_WLS_FAIL)
		return false;
	if ((((addr >> 40) & 0xFF) ^ ((addr >> 16) & 0xFF)) != ((check >> 16) & 0xFF))
		return false;
	if ((((addr >> 32) & 0xFF) ^ ((addr >> 8) & 0xFF)) != ((check >> 8) & 0xFF))
		return false;
	if ((((addr >> 24) & 0xFF) ^ (addr & 0xFF)) != (check & 0xFF))
		return false;
	return true;
}

static void cps_ask_packet(struct cps_wls_chip *chip)
{
	u8 data[TX_PKG_LEN];
	int i, tmp;

	for (i = 0; i < TX_PKG_LEN; i++) {
		tmp = cps_wls_read_reg(chip,
			cps_tx_reg[CPS_TX_REG_PPP_HEADER].reg_addr + i,
			cps_tx_reg[CPS_TX_REG_PPP_HEADER].reg_bytes_len);
		if (tmp == CPS_WLS_FAIL) {
			cps_err("ask pkt read failed\n");
			return;
		}
		data[i] = (u8)tmp;
	}

	if (data[0] == MAC_PKG_HEAD && data[1] == MAC_PKG_CMD_CKECK)
		chip->mac_check_data = cps_recv_mac_check_data(data);
	if (data[0] == MAC_PKG_HEAD && data[1] == MAC_PKG_CMD_ADD1 &&
	    data[5] == MAC_PKG_HEAD && data[6] == MAC_PKG_CMD_ADD2) {
		chip->ble_mac_addr = cps_recv_ble_mac_addr(data);
		cps_log("pen BLE addr mx%llx valid=%d\n", chip->ble_mac_addr,
			cps_ble_mac_valid(chip));
	}

	/* POWER package: byte 2 = pen state of charge (%). */
	if (data[0] == POWER_PKG_HEAD && data[1] == POWER_PKG_CMD) {
		chip->pen_soc = data[2];
		cps_log("pen SoC %d%%\n", chip->pen_soc);
	}
}

/* -------------------------------------------------------------------- */
/* Charge control                                                        */
/* -------------------------------------------------------------------- */

static void cps_send_pencil_uevent(struct cps_wls_chip *chip, bool present)
{
	char status_string[64], addr_string[64];
	char *envp[] = { status_string, addr_string, NULL };

	if (!cps_uevent)
		return;
	sprintf(status_string, "pencil_status=%d", present ? 1 : 0);
	sprintf(addr_string, "pencil_addr=%llx", (unsigned long long)chip->ble_mac_addr);
	kobject_uevent_env(&chip->dev->kobj, KOBJ_CHANGE, envp);
}

static void cps_reset_chip_status(struct cps_wls_chip *chip)
{
	chip->pen_status = PEN_STATUS_FAR;
	chip->pen_present = 0;
	chip->ping_succ_time = 0;
	chip->tx_start_time = 0;
	chip->ble_mac_addr = 0;
	chip->mac_check_data = 0;
	chip->tx_voltage = 0;
	chip->tx_current = 0;
	chip->power_disable_reason = PEN_REASON_UNKNOWN;
}

static int cps_q_cali_run(struct cps_wls_chip *chip)
{
	int i, times = 0;

	if (chip->q_cali_status == Q_CALI_CALIBRATED)
		return CPS_WLS_SUCCESS;

	cps_log("q-cali: start\n");
	cps_wls_set_cali_mode(chip, true);
	cps_wls_start_q_cali(chip);
	for (i = 0; i < 50; i++) {
		msleep(100);
		times = cps_wls_get_cali_times(chip);
		if (times != CPS_WLS_FAIL && times > 0)
			break;
	}
	cps_wls_set_cali_mode(chip, false);

	if (times > 0) {
		chip->cali_times = times;
		chip->q_cali_status = Q_CALI_CALIBRATED;
		cps_log("q-cali: calibrated (times=%d cnt=%d width=%d)\n", times,
			cps_wls_get_q_cali_count(chip), cps_wls_get_q_cali_width(chip));
		return CPS_WLS_SUCCESS;
	}
	chip->q_cali_status = Q_CALI_UNCALIBRATED;
	cps_err("q-cali: not calibrated (timed out)\n");
	return CPS_WLS_FAIL;
}

static void cps_handle_ept(struct cps_wls_chip *chip)
{
	int code = cps_wls_get_tx_ept_type(chip);

	if (code == CPS_WLS_FAIL)
		return;
	chip->power_disable_reason = PEN_REASON_CHARGE_EPT;
	if (code & (EPT_OCP | EPT_OVP | EPT_FOD | EPT_OTP | EPT_POCP)) {
		chip->power_disable_reason = PEN_REASON_CHARGE_OCP;
		chip->charge_allow = 0;
	} else if (code & EPT_UVP && chip->is_supply_by_hboost) {
		chip->charge_allow = 0;
	}
	chip->charging = false;
	cps_err("EPT code 0x%x, reason %d - charge stopped\n", code,
		chip->power_disable_reason);
}

static int cps_set_boost_and_allow(struct cps_wls_chip *chip, bool attach)
{
	int volt, retry = BOOST_SET_RETRY, rc = CPS_WLS_FAIL;

	if (!chip->is_supply_by_hboost)
		return CPS_WLS_SUCCESS;
	volt = attach ? chip->hboost_attach_volt : chip->hboost_default_volt;
	chip->hboost_volt = volt;
	chip->hboost_status = HBOOST_IS_SETTING_BOOST;
	while (retry-- > 0) {
		if (cps_set_boost_volt_mv(chip, volt) == CPS_WLS_SUCCESS) {
			chip->hboost_status = HBOOST_SET_SUCCESS;
			rc = CPS_WLS_SUCCESS;
			break;
		}
		msleep(BOOST_SET_DELAY_MS);
	}
	if (rc != CPS_WLS_SUCCESS)
		chip->hboost_status = HBOOST_SET_FAIL;
	return rc;
}

static void cps_disable_charge(struct cps_wls_chip *chip, int reason)
{
	chip->charging = false;
	chip->charge_allow = 0;
	chip->power_disable_reason = reason;
	if (gpio_is_valid(chip->wls_scan_gpio))
		gpio_direction_output(chip->wls_scan_gpio, 1);
	if (gpio_is_valid(chip->wls_off_state_gpio))
		gpio_direction_output(chip->wls_off_state_gpio, 1);
	cps_wls_disable_tx_mode(chip);
	cps_set_boost_and_allow(chip, false);
}

static void cps_restate_work_func(struct work_struct *work)
{
	struct cps_wls_chip *chip =
		container_of(work, struct cps_wls_chip, restate_work);

	if (!chip || !chip->init_done)
		return;

	if (!cps_enabled) {
		cps_set_hw_idle(chip);
		cps_wls_disable_tx_mode(chip);
		return;
	}

	cps_set_hw_state(chip, chip->charging);

	/* re-assert the ping/charge window if we are idly scanning */
	if (!chip->charging) {
		cps_wls_enable_tx_mode(chip);
		cps_send_pencil_uevent(chip, chip->pen_present);
	}

	/* if EPT already faulted and the pen is gone, re-arm scanning */
	if (!chip->pen_present && chip->power_disable_reason != PEN_REASON_UNKNOWN)
		cps_reset_chip_status(chip);
}

static void cps_expire_work_func(struct work_struct *work)
{
	struct cps_wls_chip *chip =
		container_of(work, struct cps_wls_chip, expire_work.work);

	if (!chip)
		return;
	if (chip->charging || chip->pen_present) {
		cps_err("power expired (%ds) - stopping charge\n",
			chip->power_expired_time);
		cps_disable_charge(chip, PEN_REASON_CHARGE_TIMEOUT);
	}
}


/* -------------------------------------------------------------------- */
/* Interrupt handler - the chip raises wls_charge_int on every event     */
/* -------------------------------------------------------------------- */

static inline u64 get_current_time_ms(void)
{
	struct timespec64 ts;

	ktime_get_real_ts64(&ts);
	return (u64)ts.tv_sec * 1000 + (u64)ts.tv_nsec / 1000000;
}

static void cps_wls_tx_irq_handler(struct cps_wls_chip *chip, int irq_flag)
{
	if (!chip)
		return;

	cps_log("tx irq flag 0x%x (pen_present=%d charging=%d)\n", irq_flag,
		chip->pen_present, chip->charging);

	if (irq_flag & TX_INT_WAKEUP) {
		chip->i2c_ready = true;
		cps_wls_enable_tx_mode(chip);
	}

	if (irq_flag & TX_INT_SSP) {
		chip->pen_present = 1;
		chip->pen_status = PEN_STATUS_NEAR;
		if (!chip->charging) {
			chip->tx_start_time = get_current_time_ms();
			if (cps_set_boost_and_allow(chip, true) == CPS_WLS_SUCCESS) {
				chip->charge_allow = 1;
				chip->charging = true;
				cps_set_hw_state(chip, true);
				schedule_delayed_work(&chip->expire_work,
					round_jiffies_relative(msecs_to_jiffies(
						chip->power_expired_time * MIN_TO_MS)));
				cps_log("pen attached: charging (boost %dmV)\n",
					chip->hboost_volt);
			}
		}
		cps_send_pencil_uevent(chip, true);
	}

	if (irq_flag & TX_INT_RX_ATTACH) {
		chip->pen_status = PEN_STATUS_NEAR;
		chip->tx_start_time = get_current_time_ms();
	}

	if (irq_flag & TX_INT_RX_REMOVED) {
		chip->pen_present = 0;
		chip->pen_status = PEN_STATUS_FAR;
		chip->charging = false;
		chip->charge_allow = 0;
		cancel_delayed_work(&chip->expire_work);
		cps_set_hw_idle(chip);
		cps_send_pencil_uevent(chip, false);
		cps_reset_chip_status(chip);
		cps_set_boost_and_allow(chip, false);
		cps_log("pen removed\n");
	}

	if (irq_flag & TX_INT_ASK_PKT)
		cps_ask_packet(chip);

	if (irq_flag & TX_INT_EPT)
		cps_handle_ept(chip);

	if (irq_flag & TX_INT_Q_CALI)
		cps_q_cali_run(chip);

	if (irq_flag & TX_INT_MIS_LOC)
		cps_err("pen mislocated (FOD)\n");
}

static irqreturn_t cps_wls_irq_handler(int irq, void *dev_id)
{
	struct cps_wls_chip *chip = (struct cps_wls_chip *)dev_id;
	int irq_flag;

	if (!chip)
		return IRQ_HANDLED;
	mutex_lock(&chip->irq_lock);
	cps_wls_set_int_enable(chip);
	irq_flag = cps_wls_get_int_flag(chip);
	if (irq_flag == CPS_WLS_FAIL) {
		mutex_unlock(&chip->irq_lock);
		return IRQ_HANDLED;
	}
	cps_wls_set_int_clr(chip, irq_flag);
	mutex_unlock(&chip->irq_lock);

	cps_wls_tx_irq_handler(chip, irq_flag);
	return IRQ_HANDLED;
}

/* -------------------------------------------------------------------- */
/* Asynchronous bring-up (mirrors the vendor init_work_func)             */
/* -------------------------------------------------------------------- */

static int cps_wls_chipid_check(struct cps_wls_chip *chip)
{
	int retry;

	for (retry = 0; retry < MAX_RETRY; retry++) {
		chip->chip_id = cps_wls_get_chip_id(chip);
		if (chip->chip_id == CPS8601_CHIPID)
			return CPS_WLS_SUCCESS;
		msleep(100);
	}
	cps_err("chip id mismatch: got 0x%x, want 0x%x\n", chip->chip_id,
		CPS8601_CHIPID);
	return CPS_WLS_FAIL;
}

static void cps_init_work_func(struct work_struct *work)
{
	struct cps_wls_chip *chip =
		container_of(work, struct cps_wls_chip, init_work);
	int ret;

	if (!chip)
		return;

	/* Bring the chip up on the 16-bit window: identity first, then
	 * thresholds.  sw_en stays OFF until everything below succeeded, so
	 * a dead/mis-wired chip leaves the coil exactly as it was before. */
	if (cps_wls_chipid_check(chip) != CPS_WLS_SUCCESS)
		goto init_fail;
	chip->fw_version = (unsigned short)cps_wls_get_sys_fw_version(chip);

	ret = cps_wls_set_protect_parameter(chip);
	if (ret != CPS_WLS_SUCCESS) {
		cps_err("protect parameter write failed %d\n", ret);
		goto init_fail;
	}

	cps_set_hw_state(chip, false);
	cps_wls_enable_tx_mode(chip);
	cps_wls_set_int_enable(chip);

	if (chip->cps_wls_irq) {
		ret = devm_request_threaded_irq(chip->dev, chip->cps_wls_irq, NULL,
						 cps_wls_irq_handler,
						 IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						 dev_name(chip->dev), chip);
		if (ret) {
			cps_err("irq request failed %d\n", ret);
			goto init_fail;
		}
	}

	chip->init_done = true;
	cps_log("CPS8601 wireless pen charger up (chipid 0x%x fw %u.%u)\n",
		chip->chip_id, chip->fw_version >> 8, chip->fw_version & 0xff);

	if (chip->is_supply_by_hboost) {
		/* Start the pmic_glink aux device (H-BoOST rail client). */
		schedule_delayed_work(&chip->glk_retry_work,
				      msecs_to_jiffies(200));
		cps_set_boost_and_allow(chip, false);
	} else {
		chip->charge_allow = 1;
	}
	return;

init_fail:
	cps_set_hw_idle(chip);
	cps_err("init failed - wireless pen charge disabled\n");
}


/* -------------------------------------------------------------------- */
/* debugfs                                                              */
/* -------------------------------------------------------------------- */

static int soc_get(void *data, u64 *val)
{
	struct cps_wls_chip *chip = data;
	*val = chip ? (chip->pen_soc > 0 ? (u64)chip->pen_soc : 0) : 0;
	return 0;
}

static int pen_present_get(void *data, u64 *val)
{
	struct cps_wls_chip *chip = data;
	*val = chip ? (u64)chip->pen_present : 0;
	return 0;
}

static int ble_mac_addr_get(void *data, u64 *val)
{
	struct cps_wls_chip *chip = data;
	*val = chip ? chip->ble_mac_addr : 0;
	return 0;
}

static int fw_version_get(void *data, u64 *val)
{
	struct cps_wls_chip *chip = data;
	*val = chip ? chip->fw_version : 0;
	return 0;
}

static int chip_id_get(void *data, u64 *val)
{
	struct cps_wls_chip *chip = data;
	*val = chip ? (u64)chip->chip_id : 0;
	return 0;
}

static int state_get(void *data, u64 *val)
{
	struct cps_wls_chip *chip = data;
	*val = chip ? ((u64)chip->charging << 1 | (u64)chip->charge_allow) : 0;
	return 0;
}

static int q_cali_get(void *data, u64 *val)
{
	struct cps_wls_chip *chip = data;
	*val = chip ? (u64)chip->cali_times : 0;
	return 0;
}

static int ept_get(void *data, u64 *val)
{
	struct cps_wls_chip *chip = data;
	*val = chip ? (u64)chip->power_disable_reason : 0;
	return 0;
}

static int reg_get(void *data, u64 *val)
{
	/* last-read value cache; see reg_set for the request path */
	struct cps_wls_chip *chip = data;
	*val = chip ? (u64)chip->chip_id : 0;
	return 0;
}

static int reg_set(void *data, u64 val)
{
	struct cps_wls_chip *chip = data;
	unsigned int addr = (unsigned int)(val >> 16);
	int len = (int)((val >> 8) & 0xff);
	int rw = (int)(val & 1);
	int res;

	if (!chip || len <= 0 || len > 4)
		return -EINVAL;
	cps_err("reg %s 0x%x len %d\n", rw ? "write" : "read", addr, len);
	if (rw)
		res = cps_wls_write_reg(chip, (int)addr, (int)(val >> 32), len);
	else
		res = cps_wls_read_reg(chip, (int)addr, len);
	return res == CPS_WLS_FAIL ? -EIO : 0;
}

static int enable_get(void *data, u64 *val)
{
	*val = cps_enabled;
	return 0;
}

static int enable_set(void *data, u64 val)
{
	struct cps_wls_chip *chip = data;

	cps_enabled = (unsigned int)!!val;
	cps_log("cps8601 %s\n", cps_enabled ? "enabled" : "disabled");
	if (chip && chip->init_done)
		schedule_work(&chip->restate_work);
	return 0;
}

static int q_cali_set(void *data, u64 val)
{
	struct cps_wls_chip *chip = data;
	if (!chip)
		return -EINVAL;
	if (val && chip->q_cali_status != Q_CALI_IN_PROGRESS)
		cps_q_cali_run(chip);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(pen_present_fops, pen_present_get, NULL, "%llu\n");
DEFINE_DEBUGFS_ATTRIBUTE(soc_fops, soc_get, NULL, "%llu\n");
DEFINE_DEBUGFS_ATTRIBUTE(ble_mac_addr_fops, ble_mac_addr_get, NULL, "%llx\n");
DEFINE_DEBUGFS_ATTRIBUTE(fw_version_fops, fw_version_get, NULL, "%u\n");
DEFINE_DEBUGFS_ATTRIBUTE(chip_id_fops, chip_id_get, NULL, "%x\n");
DEFINE_DEBUGFS_ATTRIBUTE(state_fops, state_get, NULL, "%llu\n");
DEFINE_DEBUGFS_ATTRIBUTE(q_cali_fops, q_cali_get, q_cali_set, "%llu\n");
DEFINE_DEBUGFS_ATTRIBUTE(ept_fops, ept_get, NULL, "%llu\n");
DEFINE_DEBUGFS_ATTRIBUTE(reg_fops, reg_get, reg_set, "%llx\n");
DEFINE_DEBUGFS_ATTRIBUTE(enable_fops, enable_get, enable_set, "%llu\n");

static void cps_create_debugfs(struct cps_wls_chip *chip)
{
	chip->debug_root = debugfs_create_dir(dev_name(chip->dev), NULL);
	debugfs_create_file("pen_present", 0444, chip->debug_root, chip,
			    &pen_present_fops);
	debugfs_create_file("soc", 0444, chip->debug_root, chip, &soc_fops);
	debugfs_create_file("ble_mac_addr", 0444, chip->debug_root, chip,
			    &ble_mac_addr_fops);
	debugfs_create_file("fw_version", 0444, chip->debug_root, chip,
			    &fw_version_fops);
	debugfs_create_file("chip_id", 0444, chip->debug_root, chip,
			    &chip_id_fops);
	debugfs_create_file("state", 0444, chip->debug_root, chip,
			    &state_fops);
	debugfs_create_file("q_cali", 0644, chip->debug_root, chip,
			    &q_cali_fops);
	debugfs_create_file("ept_reason", 0444, chip->debug_root, chip,
			    &ept_fops);
	debugfs_create_file("reg", 0644, chip->debug_root, chip, &reg_fops);
	debugfs_create_file("enable", 0644, chip->debug_root, chip,
			    &enable_fops);
}

/* -------------------------------------------------------------------- */
/* probe / remove / power management                                     */
/* -------------------------------------------------------------------- */

static int cps_wls_chrg_probe(struct i2c_client *client)
{
	struct cps_wls_chip *chip;
	int ret;

	if (!cps_enabled)
		return dev_err_probe(&client->dev, -ENODEV,
				     "cps8601 disabled by module param\n");

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->client = client;
	chip->dev = &client->dev;
	chip->pen_soc = -1;
	mutex_init(&chip->irq_lock);
	mutex_init(&chip->i2c_lock);
	mutex_init(&chip->glk_lock);
	init_completion(&chip->glk_ack);
	INIT_WORK(&chip->init_work, cps_init_work_func);
	INIT_DELAYED_WORK(&chip->glk_retry_work, cps_glink_retry_work_func);
	INIT_WORK(&chip->restate_work, cps_restate_work_func);
	INIT_DELAYED_WORK(&chip->expire_work, cps_expire_work_func);
	i2c_set_clientdata(client, chip);
	g_chip = chip;

	chip->regmap = devm_regmap_init_i2c(client, &cps8601_regmap_config);
	if (IS_ERR(chip->regmap)) {
		ret = PTR_ERR(chip->regmap);
		goto free_source;
	}

	chip->regmap32 = devm_regmap_init_i2c(client, &cps8601_regmap32_config);
	if (IS_ERR(chip->regmap32)) {
		ret = PTR_ERR(chip->regmap32);
		goto free_source;
	}

	if (cps_wls_parse_dt(chip))
		goto free_source;
	if (cps_wls_gpio_init(chip))
		goto free_source;

	if (chip->is_supply_by_hboost) {
		/* glink rail is set up asynchronously from init_work: the
		 * pmic_glink client must live on an auxiliary device under
		 * qcom_pmic_glink, never be touched from this i2c probe. */
		chip->charge_allow = 0;
	}

	cps_create_debugfs(chip);
	schedule_work(&chip->init_work);

	cps_log("CPS8601 wireless pen charger probed (addr 0x%02x)\n", client->addr);
	return 0;

free_source:
	cps_set_hw_idle(chip);
	free_named_gpio(chip);
	i2c_set_clientdata(client, NULL);
	g_chip = NULL;
	devm_kfree(&client->dev, chip);
	return ret;
}

static void cps_wls_chrg_remove(struct i2c_client *client)
{
	struct cps_wls_chip *chip = i2c_get_clientdata(client);

	if (!chip)
		return;
	cps_set_hw_idle(chip);
	cancel_delayed_work(&chip->expire_work);
	cancel_delayed_work_sync(&chip->glk_retry_work);
	cps_glink_destroy(chip);
	free_named_gpio(chip);
	debugfs_remove_recursive(chip->debug_root);
	i2c_set_clientdata(client, NULL);
	if (g_chip == chip)
		g_chip = NULL;
	devm_kfree(&client->dev, chip);
}

static int cps8601_pm_resume(struct device *dev)
{
	struct cps_wls_chip *chip = dev_get_drvdata(dev);

	if (chip) {
		chip->i2c_ready = true;
		cps_set_hw_state(chip, chip->charging);
	}
	return 0;
}

static int cps8601_pm_suspend(struct device *dev)
{
	struct cps_wls_chip *chip = dev_get_drvdata(dev);

	if (chip) {
		chip->i2c_ready = false;
		chip->charge_allow = 0;
		chip->charging = false;
		cps_set_hw_idle(chip);
	}
	return 0;
}

static const struct dev_pm_ops cps8601_pm_ops = {
	.resume = cps8601_pm_resume,
	.suspend = cps8601_pm_suspend,
};

static const struct of_device_id cps_wls_chrg_of_tbl[] = {
	{ .compatible = "oplus,wls-charger-cps8601", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, cps_wls_chrg_of_tbl);

static const struct i2c_device_id cps_wls_charger_id[] = {
	{ "cps-wls-charger" },
	{},
};
MODULE_DEVICE_TABLE(i2c, cps_wls_charger_id);

static struct i2c_driver cps8601_driver = {
	.driver = {
		.name = CPS_WLS_CHRG_DRV_NAME,
		.of_match_table = cps_wls_chrg_of_tbl,
		.pm = &cps8601_pm_ops,
	},
	.probe = cps_wls_chrg_probe,
	.remove = cps_wls_chrg_remove,
	.id_table = cps_wls_charger_id,
};
static int __init cps8601_module_init(void)
{
	int ret;

	ret = auxiliary_driver_register(&cps_glink_aux_driver);
	if (ret)
		return ret;
	ret = i2c_add_driver(&cps8601_driver);
	if (ret)
		auxiliary_driver_unregister(&cps_glink_aux_driver);
	return ret;
}
module_init(cps8601_module_init);

static void __exit cps8601_module_exit(void)
{
	i2c_del_driver(&cps8601_driver);
	auxiliary_driver_unregister(&cps_glink_aux_driver);
}
module_exit(cps8601_module_exit);

MODULE_DESCRIPTION("OnePlus CPS8601 wireless pen charger (erhai)");
MODULE_LICENSE("GPL");
MODULE_LICENSE("GPL");
