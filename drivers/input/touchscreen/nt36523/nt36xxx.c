/*
 * Copyright (C) 2010 - 2018 Novatek, Inc.
 * Copyright (C) 2021 XiaoMi, Inc.
 *
 * $Revision: 73033 $
 * $Date: 2020-11-26 10:09:14 +0800 (週四, 26 十一月 2020) $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/input/mt.h>
#include <linux/debugfs.h>
#include <linux/of_irq.h>

#ifdef NVT_XIAOMI_MI_DRM_NOTIFIER
#include <drm/drm_notifier.h>
#endif

#include "nt36xxx.h"

#if NVT_TOUCH_ESD_PROTECT
#include <linux/jiffies.h>
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if NVT_TOUCH_ESD_PROTECT
static struct delayed_work nvt_esd_check_work;
static struct workqueue_struct *nvt_esd_check_wq;
static unsigned long irq_timer = 0;
uint8_t esd_check = false;
uint8_t esd_retry = 0;
#endif /* #if NVT_TOUCH_ESD_PROTECT */

struct nvt_ts_data *ts;

/*
 * erhai stylus knobs.
 *
 * pen_mode_on_cmd — the extended-command sub-command used to enter pen scan
 *   mode.  0x10 (TYPE_PENCIL_HAVON) is what Android's touch HAL writes for the
 *   default pencil type; if a sticker does not respond, try the other families
 *   (0x12/0x13/0x18 maxeye, 0x15 Xingwangda) at runtime instead of rebuilding:
 *     echo 0x12 > /sys/module/nt36523_ts/parameters/pen_mode_on_cmd
 *     echo 1    > /sys/module/nt36523_ts/parameters/pen_resend
 * pen_resend — write anything to re-send the pen-mode command now (the verdict
 *   from the FW lands in dmesg next to "pen cmd 0x.. acked" / "not accepted").
 * pen_dump — non-zero prints the raw 14-byte pen report block (event buffer
 *   bytes 66..79) whenever it changes: the direct way to see whether the IC
 *   emits stylus packets at all, before any coordinate interpretation.
 */
static uint pen_mode_on_cmd = EVENTBUFFER_EXT_PEN_MODE_ON;
module_param(pen_mode_on_cmd, uint, 0644);
MODULE_PARM_DESC(pen_mode_on_cmd, "NT36536 extended pen-mode sub-command (0x10 havon default, 0x12/0x13/0x18 maxeye, 0x15 xingwangda)");
static uint pen_dump;
module_param(pen_dump, uint, 0644);
MODULE_PARM_DESC(pen_dump, "non-zero: print the raw 14-byte pen report block whenever it changes");
static uint pen_dump_all;
module_param(pen_dump_all, uint, 0644);
MODULE_PARM_DESC(pen_dump_all, "non-zero: print the WHOLE event frame (0..79) whenever any byte changes");
/*
 * set on every pen-mode arm so the NEXT frame prints the pen block even when it
 * did not change — otherwise a capture of a run where the IC never sees a pen
 * contains no "pen buf" line at all and the IC's status stays invisible.
 */
static u8 pen_dump_force;
static uint pen_resend;
static int nvt_pen_resend_set(const char *val, const struct kernel_param *kp);
static const struct kernel_param_ops nvt_pen_resend_ops = {
	.set = nvt_pen_resend_set,
	.get = param_get_uint,
};
module_param_cb(pen_resend, &nvt_pen_resend_ops, &pen_resend, 0200);
MODULE_PARM_DESC(pen_resend, "write any value to re-send the pen-mode command");

static int32_t disable_pen_input_device(bool disable);

#ifdef NVT_XIAOMI_MI_DRM_NOTIFIER
static int nvt_drm_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#endif

static int32_t nvt_ts_suspend(struct device *dev);
static int32_t nvt_ts_resume(struct device *dev);

uint32_t ENG_RST_ADDR  = 0x7FFF80;
uint32_t SWRST_N8_ADDR = 0; //read from dtsi
uint32_t SPI_RD_FAST_ADDR = 0;	//read from dtsi
uint32_t BLD_SPE_PUPS_ADDR = 0;	//set from trim table at chip detect (NT36536 CRC-reboot recovery)

#if TOUCH_KEY_NUM > 0
const uint16_t touch_key_array[TOUCH_KEY_NUM] = {
	KEY_BACK,
	KEY_HOME,
	KEY_MENU
};
#endif

static uint8_t bTouchIsAwake = 0;

/*******************************************************
Description:
	Novatek touchscreen irq enable/disable function.

return:
	n.a.
*******************************************************/
static void nvt_irq_enable(bool enable)
{
	if (enable) {
		if (!ts->irq_enabled) {
			enable_irq(ts->client->irq);
			ts->irq_enabled = true;
		}
	} else {
		if (ts->irq_enabled) {
			disable_irq(ts->client->irq);
			ts->irq_enabled = false;
		}
	}
}

static inline int32_t spi_read_write(struct spi_device *client, uint8_t *buf, size_t len , NVT_SPI_RW rw)
{
	struct spi_message m;
	struct spi_transfer t = {
		.len    = len,
	};

	memset(ts->xbuf, 0, len + DUMMY_BYTES);
	memcpy(ts->xbuf, buf, len);

	switch (rw) {
		case NVTREAD:
			t.tx_buf = ts->xbuf;
			t.rx_buf = ts->rbuf;
			t.len    = (len + DUMMY_BYTES);
			break;

		case NVTWRITE:
			t.tx_buf = ts->xbuf;
			break;
	}

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(client, &m);
}

/*******************************************************
Description:
	Novatek touchscreen spi read function.

return:
	Executive outcomes. 2---succeed. -5---I/O error
*******************************************************/
int32_t CTP_SPI_READ(struct spi_device *client, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;
	int32_t retries = 0;

	mutex_lock(&ts->xbuf_lock);

	buf[0] = SPI_READ_MASK(buf[0]);

	while (retries < 5) {
		ret = spi_read_write(client, buf, len, NVTREAD);
		if (ret == 0) break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("read error, ret=%d\n", ret);
		ret = -EIO;
	} else {
		memcpy((buf+1), (ts->rbuf+2), (len-1));
	}

	mutex_unlock(&ts->xbuf_lock);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen spi write function.

return:
	Executive outcomes. 1---succeed. -5---I/O error
*******************************************************/
int32_t CTP_SPI_WRITE(struct spi_device *client, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;
	int32_t retries = 0;

	mutex_lock(&ts->xbuf_lock);

	buf[0] = SPI_WRITE_MASK(buf[0]);

	while (retries < 5) {
		ret = spi_read_write(client, buf, len, NVTWRITE);
		if (ret == 0)	break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("error, ret=%d\n", ret);
		ret = -EIO;
	}

	mutex_unlock(&ts->xbuf_lock);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen set index/page/addr address.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t nvt_set_page(uint32_t addr)
{
	uint8_t buf[4] = {0};

	buf[0] = 0xFF;	//set index/page/addr command
	buf[1] = (addr >> 15) & 0xFF;
	buf[2] = (addr >> 7) & 0xFF;

	return CTP_SPI_WRITE(ts->client, buf, 3);
}

/*******************************************************
Description:
	Novatek touchscreen write data to specify address.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t nvt_write_addr(uint32_t addr, uint8_t data)
{
	int32_t ret = 0;
	uint8_t buf[4] = {0};

	//---set xdata index---
	buf[0] = 0xFF;	//set index/page/addr command
	buf[1] = (addr >> 15) & 0xFF;
	buf[2] = (addr >> 7) & 0xFF;
	ret = CTP_SPI_WRITE(ts->client, buf, 3);
	if (ret) {
		NVT_ERR("set page 0x%06X failed, ret = %d\n", addr, ret);
		return ret;
	}

	//---write data to index---
	buf[0] = addr & (0x7F);
	buf[1] = data;
	ret = CTP_SPI_WRITE(ts->client, buf, 2);
	if (ret) {
		NVT_ERR("write data to 0x%06X failed, ret = %d\n", addr, ret);
		return ret;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen enable hw bld crc function.

return:
	N/A.
*******************************************************/
void nvt_bld_crc_enable(void)
{
	uint8_t buf[4] = {0};

	//---set xdata index to BLD_CRC_EN_ADDR---
	nvt_set_page(ts->mmap->BLD_CRC_EN_ADDR);

	//---read data from index---
	buf[0] = ts->mmap->BLD_CRC_EN_ADDR & (0x7F);
	buf[1] = 0xFF;
	CTP_SPI_READ(ts->client, buf, 2);

	//---write data to index---
	buf[0] = ts->mmap->BLD_CRC_EN_ADDR & (0x7F);
	buf[1] = buf[1] | (0x01 << 7);
	CTP_SPI_WRITE(ts->client, buf, 2);
}

/*******************************************************
Description:
	Novatek touchscreen clear status & enable fw crc function.

return:
	N/A.
*******************************************************/
void nvt_fw_crc_enable(void)
{
	uint8_t buf[4] = {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

	//---clear fw reset status---
	buf[0] = EVENT_MAP_RESET_COMPLETE & (0x7F);
	buf[1] = 0x00;
	CTP_SPI_WRITE(ts->client, buf, 2);

	//---enable fw crc---
	buf[0] = EVENT_MAP_HOST_CMD & (0x7F);
	buf[1] = 0xAE;	//enable fw crc command
	CTP_SPI_WRITE(ts->client, buf, 2);
}

/*******************************************************
Description:
	Novatek touchscreen set boot ready function.

return:
	N/A.
*******************************************************/
void nvt_boot_ready(void)
{
	//---write BOOT_RDY status cmds---
	nvt_write_addr(ts->mmap->BOOT_RDY_ADDR, 1);

	mdelay(5);

	if (!ts->hw_crc) {
		//---write BOOT_RDY status cmds---
		nvt_write_addr(ts->mmap->BOOT_RDY_ADDR, 0);

		//---write POR_CD cmds---
		nvt_write_addr(ts->mmap->POR_CD_ADDR, 0xA0);
	}
}

/*******************************************************
Description:
	Novatek touchscreen enable auto copy mode function.

return:
	N/A.
*******************************************************/
void nvt_tx_auto_copy_mode(void)
{
	if (ts->auto_copy == CHECK_SPI_DMA_TX_INFO) {
		//---write TX_AUTO_COPY_EN cmds---
		nvt_write_addr(ts->mmap->TX_AUTO_COPY_EN, 0x69);
	} else if (ts->auto_copy == CHECK_TX_AUTO_COPY_EN) {
		//---write SPI_MST_AUTO_COPY cmds (NT36536)---
		nvt_write_addr(ts->mmap->TX_AUTO_COPY_EN, 0x56);
	}

	NVT_LOG("tx auto copy mode %d enable\n", ts->auto_copy);
}

/*******************************************************
Description:
	Novatek touchscreen check tx auto copy state function.
(NT36536 cascade SRAM-mirror completion polled on TX_AUTO_COPY_EN)

return:
	Executive outcomes. 0---succeed. -1---fail.
*******************************************************/
int32_t nvt_check_tx_auto_copy(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 200;

	if (ts->mmap->TX_AUTO_COPY_EN == 0) {
		NVT_ERR("error, TX_AUTO_COPY_EN = 0\n");
		return -1;
	}

	for (i = 0; i < retry; i++) {
		//---set xdata index to TX_AUTO_COPY_EN---
		nvt_set_page(ts->mmap->TX_AUTO_COPY_EN);

		//---read auto copy status---
		buf[0] = ts->mmap->TX_AUTO_COPY_EN & 0x7F;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);

		if (buf[1] == 0x00)
			break;

		usleep_range(1000, 1000);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen wait auto copy finished function.
Dispatches to the per-chip completion-check method.

return:
	Executive outcomes. 0---succeed. -1---fail.
*******************************************************/
int32_t nvt_wait_auto_copy(void)
{
	if (ts->auto_copy == CHECK_SPI_DMA_TX_INFO) {
		return nvt_check_spi_dma_tx_info();
	} else if (ts->auto_copy == CHECK_TX_AUTO_COPY_EN) {
		return nvt_check_tx_auto_copy();
	} else {
		NVT_ERR("failed, not support mode %d!\n", ts->auto_copy);
		return -1;
	}
}

/*******************************************************
Description:
	Novatek touchscreen stop CRC-fail reboot loop function.
NT36536 fail-path recovery: clear CRC_ERR_FLAG via BLD_SPE_PUPS.

return:
	N/A.
*******************************************************/
void nvt_stop_crc_reboot(void)
{
	uint8_t buf[8] = {0};
	int32_t retry = 0;

	if (!BLD_SPE_PUPS_ADDR) {
		NVT_ERR("BLD_SPE_PUPS_ADDR not set, skip stop_crc_reboot\n");
		return;
	}

	/* IC is in CRC fail reboot loop, needs to be stopped! */
	for (retry = 5; retry > 0; retry--) {
		//---reset idle : 1st---
		nvt_write_addr(SWRST_N8_ADDR, 0xAA);
		//---reset idle : 2nd---
		nvt_write_addr(SWRST_N8_ADDR, 0xAA);

		msleep(1);

		//---clear CRC_ERR_FLAG---
		nvt_set_page(BLD_SPE_PUPS_ADDR);
		buf[0] = BLD_SPE_PUPS_ADDR & 0xFF;
		buf[1] = 0xA5;
		CTP_SPI_WRITE(ts->client, buf, 2);

		//---check CRC_ERR_FLAG---
		nvt_set_page(BLD_SPE_PUPS_ADDR);
		buf[0] = BLD_SPE_PUPS_ADDR & 0xFF;
		buf[1] = 0x00;
		CTP_SPI_READ(ts->client, buf, 2);

		if (buf[1] == 0xA5)
			break;
	}

	if (retry == 0)
		NVT_ERR("CRC auto reboot not stopped! buf[1]=0x%02X\n", buf[1]);
}

/*******************************************************
Description:
	Novatek touchscreen check spi dma tx info function.

return:
	N/A.
*******************************************************/
int32_t nvt_check_spi_dma_tx_info(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 200;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(ts->mmap->SPI_DMA_TX_INFO);

		//---read fw status---
		buf[0] = ts->mmap->SPI_DMA_TX_INFO & 0x7F;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);

		if (buf[1] == 0x00)
			break;

		usleep_range(1000, 1000);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen eng reset cmd
    function.

return:
	n.a.
*******************************************************/
void nvt_eng_reset(void)
{
	//---eng reset cmds to ENG_RST_ADDR---
	nvt_write_addr(ENG_RST_ADDR, 0x5A);

	mdelay(1);	//wait tMCU_Idle2TP_REX_Hi after TP_RST
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU
    function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset(void)
{
	//---software reset cmds to SWRST_N8_ADDR---
	nvt_write_addr(SWRST_N8_ADDR, 0x55);

	msleep(10);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU then into idle mode
    function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset_idle(void)
{
	//---MCU idle cmds to SWRST_N8_ADDR---
	nvt_write_addr(SWRST_N8_ADDR, 0xAA);

	msleep(15);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU (boot) function.

return:
	n.a.
*******************************************************/
void nvt_bootloader_reset(void)
{
	//---reset cmds to SWRST_N8_ADDR---
	nvt_write_addr(SWRST_N8_ADDR, 0x69);

	mdelay(5);	//wait tBRST2FR after Bootload RST

	if (SPI_RD_FAST_ADDR) {
		/* disable SPI_RD_FAST */
		nvt_write_addr(SPI_RD_FAST_ADDR, 0x00);
	}
}

/*******************************************************
Description:
	Novatek touchscreen clear FW status function.

return:
	Executive outcomes. 0---succeed. -1---fail.
*******************************************************/
int32_t nvt_clear_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 20;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		//---clear fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_SPI_WRITE(ts->client, buf, 2);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);

		if (buf[1] == 0x00)
			break;

		usleep_range(10000, 10000);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW status function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 50;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_SPI_READ(ts->client, buf, 2);

		if ((buf[1] & 0xF0) == 0xA0)
			break;

		usleep_range(10000, 10000);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW reset state function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;
	int32_t retry = 0;
	int32_t retry_max = (check_reset_state == RESET_STATE_INIT) ? 10 : 50;

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_RESET_COMPLETE);

	while (1) {
		//---read reset state---
		buf[0] = EVENT_MAP_RESET_COMPLETE;
		buf[1] = 0x00;
		CTP_SPI_READ(ts->client, buf, 6);

		if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX)) {
			ret = 0;
			break;
		}

		retry++;
		if(unlikely(retry > retry_max)) {
			NVT_ERR("error, retry=%d, buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
				retry, buf[1], buf[2], buf[3], buf[4], buf[5]);
			ret = -1;
			break;
		}

		usleep_range(10000, 10000);
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get novatek project id information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t nvt_read_pid(void)
{
	uint8_t buf[4] = {0};
	int32_t ret = 0;

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_PROJECTID);

	//---read project id---
	buf[0] = EVENT_MAP_PROJECTID;
	buf[1] = 0x00;
	buf[2] = 0x00;
	CTP_SPI_READ(ts->client, buf, 3);

	ts->nvt_pid = (buf[2] << 8) + buf[1];

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

	NVT_LOG("PID=%04X\n", ts->nvt_pid);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get firmware related information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t nvt_get_fw_info(void)
{
	uint8_t buf[64] = {0};
	uint32_t retry_count = 0;
	int32_t ret = 0;

info_retry:
	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_FWINFO);

	//---read fw info---
	buf[0] = EVENT_MAP_FWINFO;
	CTP_SPI_READ(ts->client, buf, 39);
	ts->fw_ver = buf[1];
	ts->x_num = buf[3];
	ts->y_num = buf[4];
	ts->abs_x_max = (uint16_t)((buf[5] << 8) | buf[6]);
	ts->abs_y_max = (uint16_t)((buf[7] << 8) | buf[8]);
	ts->max_button_num = buf[11];
	ts->fw_eventbuf_prot = buf[13];
	ts->high_reso = (buf[13] == NVT_EVENTBUF_PROT_HIGH_RESO);
	ts->cascade = buf[34] & 0x01;
	if (ts->pen_support) {
		ts->x_gang_num = buf[37];
		ts->y_gang_num = buf[38];
	}

	//---clear x_num, y_num if fw info is broken---
	if ((buf[1] + buf[2]) != 0xFF) {
		NVT_ERR("FW info is broken! fw_ver=0x%02X, ~fw_ver=0x%02X\n", buf[1], buf[2]);
		ts->fw_ver = 0;
		ts->x_num = 18;
		ts->y_num = 32;
		ts->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
		ts->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;
		ts->max_button_num = TOUCH_KEY_NUM;

		if(retry_count < 3) {
			retry_count++;
			NVT_ERR("retry_count=%d\n", retry_count);
			goto info_retry;
		} else {
			NVT_ERR("Set default fw_ver=%d, x_num=%d, y_num=%d, "
					"abs_x_max=%d, abs_y_max=%d, max_button_num=%d!\n",
					ts->fw_ver, ts->x_num, ts->y_num,
					ts->abs_x_max, ts->abs_y_max, ts->max_button_num);
			ret = -1;
		}
	} else {
		ret = 0;
	}

	NVT_LOG("fw_ver = 0x%02X, fw_type = 0x%02X, x_num=%d, y_num=%d, eventbuf_prot=0x%02X, high_reso=%d, abs=%dx%d\n",
			ts->fw_ver, buf[14], ts->x_num, ts->y_num, ts->fw_eventbuf_prot, ts->high_reso, ts->abs_x_max, ts->abs_y_max);

	//---Get Novatek PID---
	nvt_read_pid();

	return ret;
}

static void release_pen_event(void) {
	if (ts && ts->pen_input_dev) {
		input_report_abs(ts->pen_input_dev, ABS_X, 0);
		input_report_abs(ts->pen_input_dev, ABS_Y, 0);
		input_report_abs(ts->pen_input_dev, ABS_PRESSURE, 0);
		input_report_abs(ts->pen_input_dev, ABS_TILT_X, 0);
		input_report_abs(ts->pen_input_dev, ABS_TILT_Y, 0);
		input_report_abs(ts->pen_input_dev, ABS_DISTANCE, 0);
		input_report_key(ts->pen_input_dev, BTN_TOUCH, 0);
		input_report_key(ts->pen_input_dev, BTN_TOOL_PEN, 0);
		input_report_key(ts->pen_input_dev, BTN_STYLUS, 0);
		input_report_key(ts->pen_input_dev, BTN_STYLUS2, 0);
		input_sync(ts->pen_input_dev);
	}
}

/*
 * erhai: (re)apply the pen input device's coordinate ranges.
 *
 * The pen reports in the SAME space as the fingers (FW advertises
 * abs_x_max x abs_y_max = 33920 x 24000; the vendor DT's
 * touchpanel,pen-panel-coords = <24000 33920> is that very grid in portrait
 * axes), so the pen limits are abs_{x,y}_max - 1 and nothing is scaled.  The
 * retired nabu "wgp-stylus" path used to multiply by 8 here, which left the
 * cursor confined to a small corner of the screen.
 *
 * This runs again whenever the ranges become known/changed: at pen device
 * creation they are still TOUCH_DEFAULT_MAX_* (the FW info / FW download land
 * later on this in-cell panel).
 */
static void nvt_pen_abs_config(void)
{
	if (!ts || !ts->pen_input_dev)
		return;

	input_set_abs_params(ts->pen_input_dev, ABS_X, 0, ts->abs_x_max - 1, 0, 0);
	input_set_abs_params(ts->pen_input_dev, ABS_Y, 0, ts->abs_y_max - 1, 0, 0);
	input_abs_set_res(ts->pen_input_dev, ABS_X, (ts->abs_x_max - 1) / PANEL_DEFAULT_WIDTH_MM);
	input_abs_set_res(ts->pen_input_dev, ABS_Y, (ts->abs_y_max - 1) / PANEL_DEFAULT_HEIGHT_MM);

	NVT_LOG("pen ABS range set to %dx%d\n", ts->abs_x_max - 1, ts->abs_y_max - 1);
}

/*******************************************************
Description:
	Novatek touchscreen parse device tree function.

return:
	n.a.
*******************************************************/
#ifdef CONFIG_OF
static int32_t nvt_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	int32_t ret = 0;

#if NVT_TOUCH_SUPPORT_HW_RST
	ts->reset_gpio = of_get_named_gpio_flags(np, "novatek,reset-gpio", 0, &ts->reset_flags);
	NVT_LOG("novatek,reset-gpio=%d\n", ts->reset_gpio);
#endif
	/*
	 * The legacy integer-GPIO OF API (of_get_named_gpio / of_gpio.h) was
	 * removed in 7.1. We take the touch IRQ from the DT "interrupts"
	 * property via the SPI core (client->irq); the INT line itself is
	 * configured as input by the ts_int_active pinctrl. No GPIO request.
	 */
	ts->irq_gpio = -1;

	ts->pen_support = of_property_read_bool(np, "novatek,pen-support");
	NVT_LOG("novatek,pen-support=%d\n", ts->pen_support);

	ret = of_property_read_u32(np, "novatek,swrst-n8-addr", &SWRST_N8_ADDR);
	if (ret) {
		NVT_ERR("error reading novatek,swrst-n8-addr. ret=%d\n", ret);
		return ret;
	} else {
		NVT_LOG("SWRST_N8_ADDR=0x%06X\n", SWRST_N8_ADDR);
	}

	ret = of_property_read_u32(np, "novatek,spi-rd-fast-addr", &SPI_RD_FAST_ADDR);
	if (ret) {
		NVT_LOG("not support novatek,spi-rd-fast-addr\n");
		SPI_RD_FAST_ADDR = 0;
		ret = 0;
	} else {
		NVT_LOG("SPI_RD_FAST_ADDR=0x%06X\n", SPI_RD_FAST_ADDR);
	}

	ret = of_property_read_string(np, "firmware-name", &ts->fw_name);
	if (ret) {
		NVT_LOG("Unable to get touchscreen firmware name\n");
		ts->fw_name = DEFAULT_BOOT_UPDATE_FIRMWARE_NAME;
	}

	ret = of_property_read_u32(np, "spi-max-frequency", &ts->spi_max_freq);
	if (ret) {
		NVT_LOG("Unable to get spi freq\n");
		return ret;
	} else {
		NVT_LOG("spi-max-frequency: %u\n", ts->spi_max_freq);
	}

	return ret;
}
#else
static int32_t nvt_parse_dt(struct device *dev)
{
#if NVT_TOUCH_SUPPORT_HW_RST
	ts->reset_gpio = NVTTOUCH_RST_PIN;
#endif
	ts->irq_gpio = NVTTOUCH_INT_PIN;
	return 0;
}
#endif

/*******************************************************
Description:
	Novatek touchscreen config and request gpio

return:
	Executive outcomes. 0---succeed. not 0---failed.
*******************************************************/
static int nvt_gpio_config(struct nvt_ts_data *ts)
{
	int32_t ret = 0;

#if NVT_TOUCH_SUPPORT_HW_RST
	/* request RST-pin (Output/High) */
	if (gpio_is_valid(ts->reset_gpio)) {
		ret = gpio_request_one(ts->reset_gpio, GPIOF_OUT_INIT_LOW, "NVT-tp-rst");
		if (ret) {
			NVT_ERR("Failed to request NVT-tp-rst GPIO\n");
			goto err_request_reset_gpio;
		}
	}
#endif

	/* request INT-pin (Input) */
	if (gpio_is_valid(ts->irq_gpio)) {
		ret = gpio_request_one(ts->irq_gpio, GPIOF_IN, "NVT-int");
		if (ret) {
			NVT_ERR("Failed to request NVT-int GPIO\n");
			goto err_request_irq_gpio;
		}
	}

	return ret;

err_request_irq_gpio:
#if NVT_TOUCH_SUPPORT_HW_RST
	gpio_free(ts->reset_gpio);
err_request_reset_gpio:
#endif
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen deconfig gpio

return:
	n.a.
*******************************************************/
static void nvt_gpio_deconfig(struct nvt_ts_data *ts)
{
	if (gpio_is_valid(ts->irq_gpio))
		gpio_free(ts->irq_gpio);
#if NVT_TOUCH_SUPPORT_HW_RST
	if (gpio_is_valid(ts->reset_gpio))
		gpio_free(ts->reset_gpio);
#endif
}

static uint8_t nvt_fw_recovery(uint8_t *point_data)
{
	uint8_t i = 0;
	uint8_t detected = true;

	/* check pattern */
	for (i=1 ; i<7 ; i++) {
		if (point_data[i] != 0x77) {
			detected = false;
			break;
		}
	}

	return detected;
}

void nvt_set_dbgfw_status(bool enable)
{
	ts->fw_debug = enable;
}

bool nvt_get_dbgfw_status(void)
{
	return ts->fw_debug;
}

#if NVT_TOUCH_ESD_PROTECT
void nvt_esd_check_enable(uint8_t enable)
{
	/* update interrupt timer */
	irq_timer = jiffies;
	/* clear esd_retry counter, if protect function is enabled */
	esd_retry = enable ? 0 : esd_retry;
	/* enable/disable esd check flag */
	esd_check = enable;
}

static void nvt_esd_check_func(struct work_struct *work)
{
	unsigned int timer = jiffies_to_msecs(jiffies - irq_timer);

	//NVT_LOG("esd_check = %d (retry %d)\n", esd_check, esd_retry);	//DEBUG

	if ((timer > NVT_TOUCH_ESD_CHECK_PERIOD) && esd_check) {
		mutex_lock(&ts->lock);
		NVT_ERR("do ESD recovery, timer = %d, retry = %d\n", timer, esd_retry);
		/* do esd recovery, reload fw */
		nvt_update_firmware(ts->fw_name);
		mutex_unlock(&ts->lock);
		/* erhai: the FW reload clears pen scan mode — put the stylus back on */
		disable_pen_input_device(false);
		/* update interrupt timer */
		irq_timer = jiffies;
		/* update esd_retry counter */
		esd_retry++;
	}

	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
}
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if NVT_TOUCH_WDT_RECOVERY
static uint8_t recovery_cnt = 0;
static unsigned long last_wdt_flash = 0;
/* erhai: the controller re-download is NOT free - the panel shows it as a
 * flicker.  Never flash more than once per minute, and only after a real
 * reset has persisted through the raised threshold (transient 0xFD/0xFE SPI
 * bursts under heavy SoC load self-heal and must not trigger a flash storm). */
#define NVT_WDT_REFLASH_DELAY_MSECS 60000UL
static uint8_t nvt_wdt_fw_recovery(uint8_t *point_data)
{
   uint32_t recovery_cnt_max = 30;
   uint8_t recovery_enable = false;
   uint8_t i = 0;

   recovery_cnt++;

   /* check pattern */
   for (i=1 ; i<7 ; i++) {
       if ((point_data[i] != 0xFD) && (point_data[i] != 0xFE)) {
           recovery_cnt = 0;
           break;
       }
   }

   if (recovery_cnt > recovery_cnt_max){
       recovery_enable = true;
       recovery_cnt = 0;
   }

   return recovery_enable;
}
#endif	/* #if NVT_TOUCH_WDT_RECOVERY */

#define PEN_DATA_LEN 14
#define FW_HISTORY_SIZE 128
static uint32_t nvt_dump_fw_history(void)
{
	int32_t ret = 0;
	uint8_t buf[FW_HISTORY_SIZE + 1 + DUMMY_BYTES] = {0};
	int32_t i = 0;
	char *tmp_dump = NULL;
	int32_t line_cnt = 0;

	if (ts->mmap->FW_HISTORY_ADDR == 0) {
		NVT_ERR("FW_HISTORY_ADDR not available!\n");
		ret = -1;
		goto exit_nvt_dump_fw_history;
	}
	nvt_set_page(ts->mmap->FW_HISTORY_ADDR);
	buf[0] = ts->mmap->FW_HISTORY_ADDR & 0xFF;
	CTP_SPI_READ(ts->client, buf, FW_HISTORY_SIZE + 1);
	if (ret) {
		NVT_ERR("CTP_SPI_READ failed.(%d)\n", ret);
		ret = -1;
		goto exit_nvt_dump_fw_history;
		}

	tmp_dump = (char *)kzalloc(FW_HISTORY_SIZE * 4, GFP_KERNEL);
	for (i = 0; i < FW_HISTORY_SIZE; i++) {
		sprintf(tmp_dump + i * 3 + line_cnt, "%02X ", buf[1 + i]);
		if ((i + 1) % 16 == 0) {
			sprintf(tmp_dump + i * 3 + line_cnt + 3, "%c", '\n');
			line_cnt++;
		}
	}
	NVT_LOG("%s", tmp_dump);

exit_nvt_dump_fw_history:
	if (tmp_dump) {
		kfree(tmp_dump);
		tmp_dump = NULL;
	}
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

	return ret;
}

#define POINT_DATA_LEN 65
/*******************************************************
Description:
	Novatek touchscreen work function.

return:
	n.a.
*******************************************************/
static irqreturn_t nvt_ts_work_func(int irq, void *data)
{
	int32_t ret = -1;
	uint8_t point_data[POINT_DATA_LEN + PEN_DATA_LEN + 1 + DUMMY_BYTES] = {0};
	uint32_t position = 0;
	uint32_t input_x = 0;
	uint32_t input_y = 0;
	uint32_t input_w = 0;
	uint32_t input_p = 0;
	uint8_t input_id = 0;
#if MT_PROTOCOL_B
	uint8_t press_id[TOUCH_MAX_FINGER_NUM] = {0};
#endif /* MT_PROTOCOL_B */
	int32_t i = 0;
	int32_t finger_cnt = 0;
	uint8_t pen_format_id = 0;
	uint32_t pen_x = 0;
	uint32_t pen_y = 0;
	uint32_t pen_pressure = 0;
	uint32_t pen_distance = 0;
	int8_t pen_tilt_x = 0;
	int8_t pen_tilt_y = 0;
	uint32_t pen_btn1 = 0;
	uint32_t pen_btn2 = 0;
	uint32_t pen_battery = 0;

	mutex_lock(&ts->lock);

	if (ts->dev_pm_suspend) {
		ret = wait_for_completion_timeout(&ts->dev_pm_suspend_completion, msecs_to_jiffies(500));
		if (!ret) {
			NVT_ERR("system(spi) can't finished resuming procedure, skip it\n");
			goto XFER_ERROR;
		}
	}

	if (ts->pen_support)
		ret = CTP_SPI_READ(ts->client, point_data, POINT_DATA_LEN + PEN_DATA_LEN + 1);
	else
		ret = CTP_SPI_READ(ts->client, point_data, POINT_DATA_LEN + 1);
	if (ret < 0) {
		NVT_ERR("CTP_SPI_READ failed.(%d)\n", ret);
		goto XFER_ERROR;
	}

	/*--- dump SPI buf ---
	for (i = 0; i < 10; i++) {
		NVT_LOG("%02X %02X %02X %02X %02X %02X  \n",
			point_data[1+i*6], point_data[2+i*6], point_data[3+i*6], point_data[4+i*6], point_data[5+i*6], point_data[6+i*6]);
	}
	*/

#if NVT_TOUCH_WDT_RECOVERY
	/* ESD protect by WDT */
	if (nvt_wdt_fw_recovery(point_data)) {
		NVT_ERR("Recover for fw reset, %02X\n", point_data[1]);
		if (point_data[1] == 0xFD) {
			NVT_ERR("Dump FW history:\n");
			nvt_dump_fw_history();
		}
		if (time_before(jiffies, last_wdt_flash +
				msecs_to_jiffies(NVT_WDT_REFLASH_DELAY_MSECS))) {
			/* transient SPI-link glitch (heavy SoC/memory-bus load): the
			 * controller is fine, the frame stream returned to normal by
			 * itself in every erhai observation.  Drop the frame, no flash. */
			NVT_LOG("fw-reset markers look transient - no reflash");
		} else {
			last_wdt_flash = jiffies;
			nvt_update_firmware(ts->fw_name);
			/* erhai: the FW reload clears pen scan mode */
			disable_pen_input_device(false);
		}
		goto XFER_ERROR;
	}
#endif /* #if NVT_TOUCH_WDT_RECOVERY */

	/* ESD protect by FW handshake */
	if (nvt_fw_recovery(point_data)) {
#if NVT_TOUCH_ESD_PROTECT
		nvt_esd_check_enable(true);
#endif /* #if NVT_TOUCH_ESD_PROTECT */
		goto XFER_ERROR;
	}

	finger_cnt = 0;

	for (i = 0; i < ts->max_touch_num; i++) {
		position = 1 + 6 * i;
		input_id = (uint8_t)(point_data[position + 0] >> 3);
		if ((input_id == 0) || (input_id > ts->max_touch_num))
			continue;

		if (((point_data[position] & 0x07) == 0x01) || ((point_data[position] & 0x07) == 0x02)) {	//finger down (enter & moving)
#if NVT_TOUCH_ESD_PROTECT
			/* update interrupt timer */
			irq_timer = jiffies;
#endif /* #if NVT_TOUCH_ESD_PROTECT */
			if (ts->high_reso) {
				/*
				 * NT36536 high-resolution report (fw_eventbuf_prot
				 * 0xF1): 16-bit big-endian X/Y, byte layout
				 * [1]=x_hi [2]=x_lo [3]=y_hi [4]=y_lo [5]=width.
				 *
				 * erhai: the FW info block advertises LANDSCAPE ranges
				 * (abs_x_max=33920, abs_y_max=24000) but the report
				 * stream is PORTRAIT — bytes [1..2] carry the short
				 * (2400) edge and [3..4] the long (3392) edge. Swap so
				 * the reported data matches the landscape ranges, which
				 * lets a userspace identity calibration matrix work and
				 * stops the abs_y_max filter from dropping the >70%-width
				 * band. Direction-only mirroring (if any) is left to the
				 * udev/libinput matrix.
				 */
				input_x = ((uint32_t)point_data[position + 3] << 8) + point_data[position + 4];
				input_y = ((uint32_t)point_data[position + 1] << 8) + point_data[position + 2];
				input_w = (uint32_t)point_data[position + 5];
				if (input_w == 0)
					input_w = 1;
				input_p = input_w;
			} else {
				/* legacy 12-bit report: [1]=x_hi [2]=y_hi [3]=xy_lo [4]=area [5]=pressure */
				/* erhai: portrait-vs-landscape transpose, see high_reso note above */
				input_y = (uint32_t)(point_data[position + 1] << 4) + (uint32_t) (point_data[position + 3] >> 4);
				input_x = (uint32_t)(point_data[position + 2] << 4) + (uint32_t) (point_data[position + 3] & 0x0F);
				input_w = (uint32_t)(point_data[position + 4]);
				if (input_w == 0)
					input_w = 1;
				if (i < 2) {
					input_p = (uint32_t)(point_data[position + 5]) + (uint32_t)(point_data[i + 63] << 8);
					if (input_p > TOUCH_FORCE_NUM)
						input_p = TOUCH_FORCE_NUM;
				} else {
					input_p = (uint32_t)(point_data[position + 5]);
				}
				if (input_p == 0)
					input_p = 1;
			}
			if ((input_x > ts->abs_x_max) || (input_y > ts->abs_y_max))
				continue;

			/* erhai: panel Y is mirrored vs the reported coordinate
			 * after the landscape transpose; flip it here so a
			 * userspace identity calibration matrix tracks correctly. */
			input_y = ts->abs_y_max - 1 - input_y;

#if MT_PROTOCOL_B
			press_id[input_id - 1] = 1;
			input_mt_slot(ts->input_dev, input_id - 1);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
#else /* MT_PROTOCOL_B */
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, input_id - 1);
			input_report_key(ts->input_dev, BTN_TOUCH, 1);
#endif /* MT_PROTOCOL_B */

			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, input_p);

#if MT_PROTOCOL_B
#else /* MT_PROTOCOL_B */
			input_mt_sync(ts->input_dev);
#endif /* MT_PROTOCOL_B */

			finger_cnt++;
		}
	}

#if MT_PROTOCOL_B
	for (i = 0; i < ts->max_touch_num; i++) {
		if (press_id[i] != 1) {
			input_mt_slot(ts->input_dev, i);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
		}
	}
	input_report_key(ts->input_dev, BTN_TOUCH, (finger_cnt > 0));
#else /* MT_PROTOCOL_B */
	if (finger_cnt == 0) {
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_mt_sync(ts->input_dev);
	}
#endif /* MT_PROTOCOL_B */

#if TOUCH_KEY_NUM > 0
	if (point_data[61] == 0xF8) {
#if NVT_TOUCH_ESD_PROTECT
		/* update interrupt timer */
		irq_timer = jiffies;
#endif /* #if NVT_TOUCH_ESD_PROTECT */
		for (i = 0; i < ts->max_button_num; i++) {
			input_report_key(ts->input_dev, touch_key_array[i], ((point_data[62] >> i) & 0x01));
		}
	} else {
		for (i = 0; i < ts->max_button_num; i++) {
			input_report_key(ts->input_dev, touch_key_array[i], 0);
		}
	}
#endif

	input_sync(ts->input_dev);

	if (ts->pen_support && ts->pen_input_dev_enable && !(ts->pen_is_charge)) {
		/*
		 * erhai: raw pen report block (event buffer bytes 66..79) on change.
		 * The cheapest way to tell "the IC never emits stylus packets" from
		 * "it does, but we decode them wrong": echo 1 > .../parameters/pen_dump
		 */
		if (pen_dump) {
			static uint8_t pen_dump_last[PEN_DATA_LEN];

			if (pen_dump_force || memcmp(pen_dump_last, &point_data[66], PEN_DATA_LEN)) {
				pen_dump_force = 0;
				memcpy(pen_dump_last, &point_data[66], PEN_DATA_LEN);
				NVT_LOG("pen buf %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
					point_data[66], point_data[67], point_data[68], point_data[69], point_data[70],
					point_data[71], point_data[72], point_data[73], point_data[74], point_data[75],
					point_data[76], point_data[77], point_data[78], point_data[79]);
			}
		}

		/*
		 * erhai: full-frame raw dump on ANY byte change — tells "the IC reports
		 * no pen at the offsets we expect" apart from "the pen block sits
		 * somewhere else in the frame".  Enable only while probing a stylus:
		 * a moving finger changes this every frame.
		 */
		if (pen_dump_all) {
			static uint8_t raw_last[POINT_DATA_LEN + PEN_DATA_LEN];

			if (memcmp(raw_last, point_data, sizeof(raw_last))) {
				int r;

				memcpy(raw_last, point_data, sizeof(raw_last));
				for (r = 0; r < (int)sizeof(raw_last); r += 20)
					NVT_LOG("raw %02d-%02d %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
						r, r + 19,
						point_data[r], point_data[r + 1], point_data[r + 2], point_data[r + 3],
						point_data[r + 4], point_data[r + 5], point_data[r + 6], point_data[r + 7],
						point_data[r + 8], point_data[r + 9], point_data[r + 10], point_data[r + 11],
						point_data[r + 12], point_data[r + 13], point_data[r + 14], point_data[r + 15],
						point_data[r + 16], point_data[r + 17], point_data[r + 18], point_data[r + 19]);
			}
		}

		// parse and handle pen report
		pen_format_id = point_data[66];
		if (pen_format_id != 0xFF) {
			if (pen_format_id == 0x01) {
				// report pen data
				/*
				 * erhai: the pen reports in the SAME space as the fingers
				 * (FW abs 33920x24000; the vendor DT's pen-panel-coords
				 * <24000 33920> is that same grid in portrait axes), so
				 * bytes [69:70] are the long edge (landscape X) and
				 * [67:68] the short one (landscape Y).  The old nabu
				 * "wgp-stylus" *8 scaling here is gone — it left the
				 * cursor in a corner and capped pressure at 1/4 range.
				 */
				pen_x = (uint32_t)(point_data[69] << 8) + (uint32_t)(point_data[70]);
				pen_y = (uint32_t)(point_data[67] << 8) + (uint32_t)(point_data[68]);
				if (pen_x > ts->abs_x_max - 1)
					pen_x = ts->abs_x_max - 1;
				if (pen_y > ts->abs_y_max - 1)
					pen_y = ts->abs_y_max - 1;
				/* erhai: mirror pen Y to match the finger Y flip */
				pen_y = (ts->abs_y_max - 1) - pen_y;
				pen_pressure = (uint32_t)(point_data[71] << 8) + (uint32_t)(point_data[72]);
				pen_tilt_x = (int32_t)point_data[73];
				pen_tilt_y = (int32_t)point_data[74];
				pen_distance = (uint32_t)(point_data[75] << 8) + (uint32_t)(point_data[76]);
				pen_btn1 = (uint32_t)(point_data[77] & 0x01);
				pen_btn2 = (uint32_t)((point_data[77] >> 1) & 0x01);
				pen_battery = (uint32_t)point_data[78];
//				printk("x=%d,y=%d,p=%d,tx=%d,ty=%d,d=%d,b1=%d,b2=%d,bat=%d\n", pen_x, pen_y, pen_pressure,
//						pen_tilt_x, pen_tilt_y, pen_distance, pen_btn1, pen_btn2, pen_battery);


				input_report_abs(ts->pen_input_dev, ABS_X, pen_x);
				input_report_abs(ts->pen_input_dev, ABS_Y, pen_y);
				input_report_abs(ts->pen_input_dev, ABS_PRESSURE, pen_pressure);
				input_report_key(ts->pen_input_dev, BTN_TOUCH, !!pen_pressure);
				input_report_abs(ts->pen_input_dev, ABS_TILT_X, pen_tilt_x);
				input_report_abs(ts->pen_input_dev, ABS_TILT_Y, pen_tilt_y);
				input_report_abs(ts->pen_input_dev, ABS_DISTANCE, pen_distance);
				input_report_key(ts->pen_input_dev, BTN_TOOL_PEN, !!pen_distance || !!pen_pressure);
				input_report_key(ts->pen_input_dev, BTN_STYLUS, pen_btn1);
				input_report_key(ts->pen_input_dev, BTN_STYLUS2, pen_btn2);
				input_sync(ts->pen_input_dev);
				// TBD: pen battery event report
				// NVT_LOG("pen_battery=%d\n", pen_battery);
			} else if (pen_format_id == 0xF0) {
				// report Pen ID
			} else {
				NVT_ERR("Unknown pen format id!\n");
				goto XFER_ERROR;
			}
		} else { // pen_format_id = 0xFF, i.e. no pen present
			release_pen_event();
		}
	} /* if (ts->pen_support) */

XFER_ERROR:

	mutex_unlock(&ts->lock);
	return IRQ_HANDLED;
}


/*******************************************************
Description:
	Novatek touchscreen check chip version trim function.

return:
	Executive outcomes. 0---NVT IC. -1---not NVT IC.
*******************************************************/
static int8_t nvt_ts_check_chip_ver_trim(uint32_t chip_ver_trim_addr)
{
	uint8_t buf[8] = {0};
	int32_t retry = 0;
	int32_t list = 0;
	int32_t i = 0;
	int32_t found_nvt_chip = 0;
	int32_t ret = -1;

	ts->mmap = NULL;

	//---Check for 5 times---
	for (retry = 5; retry > 0; retry--) {
		nvt_bootloader_reset();

		nvt_set_page(chip_ver_trim_addr); //read chip id

		buf[0] = chip_ver_trim_addr & 0x7F;	//offset
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		CTP_SPI_WRITE(ts->client, buf, 7);

		buf[0] = chip_ver_trim_addr & 0x7F;	//offset
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		CTP_SPI_READ(ts->client, buf, 7);
		NVT_LOG("chip id = 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
				buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

		//---compare read chip id on supported list---
		for (list = 0; list < ARRAY_SIZE(trim_id_table); list++) {
			found_nvt_chip = 0;

			//---compare each byte---
			for (i = 0; i < NVT_ID_BYTE_MAX; i++) {
				if (trim_id_table[list].mask[i]) {
					if (buf[i + 1] != trim_id_table[list].id[i])
						break;
				}
			}

			if (i == NVT_ID_BYTE_MAX)
				found_nvt_chip = 1;

			if (found_nvt_chip) {
				NVT_LOG("This is NVT touch IC\n");

				if (trim_id_table[list].mmap->ENB_CASC_REG.addr) {
					//---check single or cascade---
					uint8_t enb_casc = 0;

					nvt_set_page(trim_id_table[list].mmap->ENB_CASC_REG.addr);
					buf[0] = trim_id_table[list].mmap->ENB_CASC_REG.addr & 0x7F;
					buf[1] = 0xFF;
					CTP_SPI_READ(ts->client, buf, 2);
					enb_casc = buf[1] & trim_id_table[list].mmap->ENB_CASC_REG.mask;

					if (enb_casc) {
						NVT_LOG("Single Chip\n");
						ts->mmap = trim_id_table[list].mmap;
					} else {
						NVT_LOG("Cascade Chip\n");
						ts->mmap = trim_id_table[list].mmap_casc;
					}
				} else {
					ts->mmap = trim_id_table[list].mmap;
				}

				ts->carrier_system = trim_id_table[list].hwinfo->carrier_system;
				ts->hw_crc = trim_id_table[list].hwinfo->hw_crc;
				ts->auto_copy = trim_id_table[list].hwinfo->auto_copy;
				ts->bld_multi_header = trim_id_table[list].hwinfo->bld_multi_header;
				SWRST_N8_ADDR = trim_id_table[list].hwinfo->swrst_sif_addr;
				BLD_SPE_PUPS_ADDR = trim_id_table[list].hwinfo->bld_spe_pups_addr;
				NVT_LOG("hw_crc=%d, auto_copy=%d, bld_multi_header=%d, "
						"swrst=0x%06X, bld_spe_pups=0x%06X\n",
						ts->hw_crc, ts->auto_copy, ts->bld_multi_header,
						SWRST_N8_ADDR, BLD_SPE_PUPS_ADDR);

				ret = 0;
				goto out;
			} else {
				ts->mmap = NULL;
				ret = -1;
			}
		}

		msleep(10);
	}

out:
	return ret;
}

/*
 * erhai: send one NT36536 extended host command and wait for the FW to consume
 * it, exactly like the vendor driver's nvt_extend_cmd_store(): the command goes
 * to EVENT_MAP_HOST_CMD as { EV <EVENTBUFFER_EXT_CMD>, <sub-command> } and the
 * FW clears the status byte back to 0x00 once it has been taken.
 *
 * This port used to write the nabu byte 0x7B here.  In THIS IC's command set
 * 0x7B is EVENTBUFFER_EDGE_LIMIT_LEFT_UP — an edge-limit command — so the
 * controller was never switched into pen scan mode and a stylus produced no
 * input events at all (verified against the OxygenOS sources in
 * tmp/android-dt-pad3 vendor/oplus/kernel/touchpanel/oplus_touchscreen_v2/
 * Novatek/NT36536_noflash/).
 */
static int32_t nvt_pen_ext_cmd_send(uint8_t subcmd)
{
	uint8_t buf[4] = {0};
	int32_t ret = -EIO;
	int i;

	for (i = 0; i < 5; i++) {
		ret = nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);
		if (ret < 0) {
			NVT_ERR("set page for pen cmd 0x%02X failed, ret=%d\n", subcmd, ret);
			return ret;
		}

		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = EVENTBUFFER_EXT_CMD;
		buf[2] = subcmd;
		ret = CTP_SPI_WRITE(ts->client, buf, 3);
		if (ret < 0) {
			NVT_ERR("write pen cmd 0x%02X failed, ret=%d\n", subcmd, ret);
			return ret;
		}

		msleep(20);

		/*---read cmd status back (0x00 = accepted)---*/
		ret = nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);
		if (ret < 0)
			return ret;

		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		buf[2] = 0xFF;
		ret = CTP_SPI_READ(ts->client, buf, 3);
		if (ret < 0)
			return ret;

		if (buf[1] == 0x00)
			return 0;
	}

	/*
	 * erhai: MUST return a real error here.  This used to `return ret`, and
	 * `ret` is CTP_SPI_READ()'s value (0 = the SPI transfer succeeded), so a
	 * command the FW never consumed still looked like success and
	 * disable_pen_input_device() printed "... acked" — measured live on
	 * Marcos' pad 2026-10-02: "pen cmd 0x10 NOT accepted by FW (status=0x7F)"
	 * and "pen cmd 0x7F/0x10 acked" on the same arm.
	 */
	NVT_ERR("pen cmd 0x%02X NOT accepted by FW (status=0x%02X)\n", subcmd, buf[1]);
	return -EIO;
}

static int32_t disable_pen_input_device(bool disable) {
	uint8_t subcmd;
	int32_t ret = 0;

	NVT_LOG("++\n");
	if (!bTouchIsAwake || !ts) {
		NVT_LOG("touch suspend, stop set pen state %s", disable ? "DISABLE" : "ENABLE");
		goto nvt_set_pen_enable_out;
	}

	msleep(35);
	disable = (!(ts->pen_input_dev_enable) || ts->pen_is_charge) ? true : disable;

	subcmd = disable ? EVENTBUFFER_EXT_PEN_MODE_OFF : (uint8_t)pen_mode_on_cmd;

	ret = nvt_pen_ext_cmd_send(subcmd);
	if (ret < 0)
		goto nvt_set_pen_enable_out;

	NVT_LOG("pen charge state is %s, %s pen input device (pen cmd 0x7F/0x%02X acked)\n",
		ts->pen_is_charge ? "ENABLE" : "DISABLE",
		disable ? "DISABLE" : "ENABLE", subcmd);

	/* force the next frame to print the pen report block (pen_dump captures) */
	if (pen_dump)
		pen_dump_force = 1;

nvt_set_pen_enable_out:
	NVT_LOG("--\n");
	return ret;
}

/*
 * pen_resend: re-send the pen-mode command right now, in this (process) sysfs
 * context — used to test the other pencil families ("pen_mode_on_cmd") without
 * rebuilding the kernel.  Nothing else in the driver holds ts->lock here, and
 * the helper only sleeps + talks SPI, so no workqueue is needed.
 */
static int nvt_pen_resend_set(const char *val, const struct kernel_param *kp)
{
	uint32_t v;
	int ret = kstrtou32(val, 0, &v);

	if (ret)
		return ret;

	pen_resend = v;

	if (!ts || !ts->pen_support || !ts->pen_input_dev) {
		NVT_ERR("pen_resend: pen support not ready (pen_support=%d)\n",
			ts ? ts->pen_support : -1);
		return -ENODEV;
	}

	disable_pen_input_device(false);

	return 0;
}


/*
 * Boot firmware download (deferred).
 * This driver is built-in (=y), so it probes during initcalls — BEFORE the
 * rootfs is mounted — and request_firmware() then fails with -ENOENT. Defer the
 * boot FW download to a delayed work that retries until /lib/firmware is
 * reachable, then align the input ABS range to the FW-reported resolution.
 */
static void nvt_boot_fw_work_func(struct work_struct *work)
{
	int ret;

	/*
	 * Deferred bring-up: the in-cell IC was unpowered/silent during probe (the
	 * panel/DSI only powers up ~12-14s into boot), so the probe-time chip-id
	 * check was skipped. Detect the chip now — this also sets ts->mmap for the
	 * FW download — and keep retrying until the IC powers on and answers.
	 */
	if (ts->deferred_bringup && !ts->hw_ready) {
		ret = nvt_ts_check_chip_ver_trim(CHIP_VER_TRIM_ADDR);
		if (ret)
			ret = nvt_ts_check_chip_ver_trim(CHIP_VER_TRIM_OLD_ADDR);
		if (ret) {
			/* IC still unpowered (panel not up yet) — poll (~60 * 500ms). */
			if (ts->boot_fw_retry++ < 60) {
				schedule_delayed_work(&ts->nvt_fwu_work, msecs_to_jiffies(500));
				return;
			}
			NVT_ERR("deferred bring-up: chip not identified after %d retries\n",
					ts->boot_fw_retry);
			return;
		}
		ts->hw_ready = true;
		NVT_LOG("deferred bring-up: chip identified (retry=%d)\n", ts->boot_fw_retry);
		ts->boot_fw_retry = 0;	/* give the FW download its own retry budget */
	}

	mutex_lock(&ts->lock);
	ret = nvt_update_firmware(ts->fw_name);
	mutex_unlock(&ts->lock);

	if (ret) {
		if (ts->boot_fw_retry++ < 30) {
			/* rootfs likely not mounted yet at early boot; retry */
			schedule_delayed_work(&ts->nvt_fwu_work, msecs_to_jiffies(1000));
			return;
		}
		NVT_ERR("boot firmware download failed after %d retries\n", ts->boot_fw_retry);
		return;
	}

	/* FW info now known: align input ABS range to real panel resolution */
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max - 1, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max - 1, 0, 0);
	NVT_LOG("boot firmware ready (retry=%d); input ABS range set to %dx%d\n",
			ts->boot_fw_retry, ts->abs_x_max, ts->abs_y_max);

	/*
	 * erhai: the pen device was created before this (with the default panel
	 * ranges) and a FW (re)load clears the IC's pen scan mode, so re-apply
	 * both now: pen ABS ranges + the pen-mode command.
	 */
	nvt_pen_abs_config();
	disable_pen_input_device(false);

	/* deferred bring-up: chip is up + FW loaded — now safe to start sensing */
	if (ts->deferred_bringup)
		nvt_irq_enable(true);
}

static void nvt_suspend_work(struct work_struct *work)
{
	struct nvt_ts_data *ts_core = container_of(work, struct nvt_ts_data, suspend_work);
	nvt_ts_suspend(&ts_core->client->dev);
}

static void nvt_resume_work(struct work_struct *work)
{
	struct nvt_ts_data *ts_core = container_of(work, struct nvt_ts_data, resume_work);
	nvt_ts_resume(&ts_core->client->dev);
}

/*
 * erhai: in-cell panel power integration (NT36536 is powered by the XN242
 * panel supplies, see gpu-MEMORY-2026-09-12.md "POST-REBOOT EVIDENCE #5").
 *
 * When the display powers down, xn242_unprepare() drops those regulator rails
 * to 0 V. If the driver is still awake at that point, the INT line fires on a
 * dying IC, the threaded IRQ / FW-recovery workloop keeps poking the dead
 * slave over SPI ("Recover for fw reset", "Download Firmware failed",
 * "Unknown pen format id") and the pad silently PORs — seen in ALL five
 * display-off deaths. So the XN242 panel driver calls back here:
 *
 *   - nvt_ts_panel_power_off(): synchronous, from xn242_disable() while the
 *     rails are STILL UP. Sends the deep-sleep command to the powered IC,
 *     masks the IRQ and drops bTouchIsAwake BEFORE xn242_unprepare cuts the
 *     rails, so no recovery storm can ever touch an unpowered controller.
 *   - nvt_ts_panel_power_on(): asynchronous resume_work (FW re-download +
 *     IRQ re-arm) queued on the event workqueue once the rails are back.
 */
void nvt_ts_panel_power_off(void)
{
	if (!ts || !bTouchIsAwake)
		return;

	NVT_LOG("display off: sleeping NT36536 before panel rail drop\n");
	nvt_ts_suspend(&ts->client->dev);
}
EXPORT_SYMBOL(nvt_ts_panel_power_off);

void nvt_ts_panel_power_on(void)
{
	if (!ts || !ts->event_wq)
		return;

	NVT_LOG("display on: resuming NT36536\n");
	flush_workqueue(ts->event_wq);
	queue_work(ts->event_wq, &ts->resume_work);
}
EXPORT_SYMBOL(nvt_ts_panel_power_on);

/*******************************************************
Description:
	Novatek touchscreen driver probe function.

return:
	Executive outcomes. 0---succeed. negative---failed
*******************************************************/
static int32_t nvt_ts_probe(struct spi_device *client)
{
	int32_t ret = 0;
#if (TOUCH_KEY_NUM > 0)
	int32_t retry = 0;
#endif

	NVT_LOG("probe start\n");

	ts = kzalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		NVT_ERR("failed to allocated memory for nvt ts data\n");
		return -ENOMEM;
	}

	ts->xbuf = (uint8_t *)kzalloc((NVT_TRANSFER_LEN+1+DUMMY_BYTES), GFP_KERNEL);
	if(ts->xbuf == NULL) {
		NVT_ERR("kzalloc for xbuf failed!\n");
		ret = -ENOMEM;
		goto err_malloc_xbuf;
	}

	ts->rbuf = (uint8_t *)kzalloc(NVT_READ_LEN, GFP_KERNEL);
	if(ts->rbuf == NULL) {
		NVT_ERR("kzalloc for rbuf failed!\n");
		ret = -ENOMEM;
		goto err_malloc_rbuf;
	}

	ts->client = client;
	spi_set_drvdata(client, ts);

	//---prepare for spi parameter---
	if (ts->client->controller->flags & SPI_CONTROLLER_HALF_DUPLEX) {
		NVT_ERR("Full duplex not supported by master\n");
		ret = -EIO;
		goto err_ckeck_full_duplex;
	}
	ts->client->bits_per_word = 8;
	ts->client->mode = SPI_MODE_0;

	ret = spi_setup(ts->client);
	if (ret < 0) {
		NVT_ERR("Failed to perform SPI setup\n");
		goto err_spi_setup;
	}

	NVT_LOG("mode=%d, max_speed_hz=%d\n", ts->client->mode, ts->client->max_speed_hz);

	//---parse dts---
	ret = nvt_parse_dt(&client->dev);
	if (ret) {
		NVT_ERR("parse dt error\n");
		goto err_spi_setup;
	}

	//---request and config GPIOs---
	ret = nvt_gpio_config(ts);
	if (ret) {
		NVT_ERR("gpio config error!\n");
		goto err_gpio_config_failed;
	}

	mutex_init(&ts->lock);
	mutex_init(&ts->xbuf_lock);

	//---eng reset before TP_RESX high
	nvt_eng_reset();

#if NVT_TOUCH_SUPPORT_HW_RST
	gpio_set_value(ts->reset_gpio, 1);
#endif

	// need 10ms delay after POR(power on reset)
	msleep(10);

	/*
	 * The NT36536 is in-cell (no own reset GPIO) — it shares power/reset with
	 * the XN242 DSI panel and is UNPOWERED this early in boot (panel comes up
	 * ~12-14s). So a failed chip-id check here is EXPECTED and must NOT abort
	 * the probe (aborting is what left the panel with no touch). Instead defer
	 * chip detection + FW + IRQ-enable to the retry work, which polls until the
	 * IC powers on. Use the known NT36536 (cascade) map as a safe placeholder so
	 * nothing derefs a NULL ts->mmap before then.
	 */
	//---check chip version trim---
	ret = nvt_ts_check_chip_ver_trim(CHIP_VER_TRIM_ADDR);
	if (ret) {
		NVT_LOG("try to check from old chip ver trim address\n");
		ret = nvt_ts_check_chip_ver_trim(CHIP_VER_TRIM_OLD_ADDR);
	}
	if (ret) {
		NVT_LOG("chip silent at probe (in-cell, panel unpowered) — deferring bring-up\n");
		ts->mmap = &NT36536_cascade_memory_map;
		ts->hw_ready = false;
		ts->deferred_bringup = true;
	} else {
		ts->hw_ready = true;
	}

	ts->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
	ts->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;

	//---allocate input device---
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		NVT_ERR("allocate input device failed\n");
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}

	ts->max_touch_num = TOUCH_MAX_FINGER_NUM;

#if TOUCH_KEY_NUM > 0
	ts->max_button_num = TOUCH_KEY_NUM;
#endif

	ts->int_trigger_type = INT_TRIGGER_TYPE;

	//---set input device info.---
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

	ts->db_wakeup = 0;
	ts->fw_ver = 0;
	ts->x_num = 32;
	ts->y_num = 50;
	ts->x_gang_num = 4;
	ts->y_gang_num = 6;
	ts->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
	ts->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;
	ts->max_button_num = TOUCH_KEY_NUM;
	NVT_LOG("Set default fw_ver=%d, x_num=%d, y_num=%d, "
					"abs_x_max=%d, abs_y_max=%d, max_button_num=%d!\n",
					ts->fw_ver, ts->x_num, ts->y_num,
					ts->abs_x_max, ts->abs_y_max, ts->max_button_num);

#if MT_PROTOCOL_B
	input_mt_init_slots(ts->input_dev, ts->max_touch_num, 0);
#endif

	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, TOUCH_FORCE_NUM, 0, 0);    //pressure = TOUCH_FORCE_NUM

#if TOUCH_MAX_FINGER_NUM > 1
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);    //area = 255

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max - 1, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max - 1, 0, 0);
#if MT_PROTOCOL_B
	// no need to set ABS_MT_TRACKING_ID, input_mt_init_slots() already set it
#else
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touch_num, 0, 0);
#endif //MT_PROTOCOL_B
#endif //TOUCH_MAX_FINGER_NUM > 1

#if TOUCH_KEY_NUM > 0
	for (retry = 0; retry < ts->max_button_num; retry++) {
		input_set_capability(ts->input_dev, EV_KEY, touch_key_array[retry]);
	}
#endif

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = NVT_TS_NAME;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_SPI;

	//---register input device---
	ret = input_register_device(ts->input_dev);
	if (ret) {
		NVT_ERR("register input device (%s) failed. ret=%d\n", ts->input_dev->name, ret);
		goto err_input_register_device_failed;
	}

	if (ts->pen_support) {
		//---allocate pen input device---
		ts->pen_input_dev = input_allocate_device();
		if (ts->pen_input_dev == NULL) {
			NVT_ERR("allocate pen input device failed\n");
			ret = -ENOMEM;
			goto err_pen_input_dev_alloc_failed;
		}

		//---set pen input device info.---
		ts->pen_input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
		ts->pen_input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
		ts->pen_input_dev->keybit[BIT_WORD(BTN_TOOL_PEN)] |= BIT_MASK(BTN_TOOL_PEN);
		//ts->pen_input_dev->keybit[BIT_WORD(BTN_TOOL_RUBBER)] |= BIT_MASK(BTN_TOOL_RUBBER);
		ts->pen_input_dev->keybit[BIT_WORD(BTN_STYLUS)] |= BIT_MASK(BTN_STYLUS);
		ts->pen_input_dev->keybit[BIT_WORD(BTN_STYLUS2)] |= BIT_MASK(BTN_STYLUS2);
		ts->pen_input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

		/*
		 * erhai: 1:1 with the finger grid (no nabu *8 scaling) — the real
		 * ranges are re-applied by nvt_pen_abs_config() once the FW info
		 * is known; these are only the pre-FW defaults.
		 */
		nvt_pen_abs_config();

		input_set_abs_params(ts->pen_input_dev, ABS_PRESSURE, 0, PEN_PRESSURE_MAX, 0, 0);
		input_set_abs_params(ts->pen_input_dev, ABS_DISTANCE, 0, PEN_DISTANCE_MAX, 0, 0);
		input_set_abs_params(ts->pen_input_dev, ABS_TILT_X, PEN_TILT_MIN, PEN_TILT_MAX, 0, 0);
		input_set_abs_params(ts->pen_input_dev, ABS_TILT_Y, PEN_TILT_MIN, PEN_TILT_MAX, 0, 0);

		sprintf(ts->pen_phys, "input/pen");
		ts->pen_input_dev->name = NVT_PEN_NAME;
		ts->pen_input_dev->phys = ts->pen_phys;
		ts->pen_input_dev->id.bustype = BUS_SPI;

		//---register pen input device---
		ret = input_register_device(ts->pen_input_dev);
		if (ret) {
			NVT_ERR("register pen input device (%s) failed. ret=%d\n", ts->pen_input_dev->name, ret);
			goto err_pen_input_register_device_failed;
		}
	} /* if (ts->pen_support) */

	//---set int-pin & request irq---
	/* client->irq is populated from the DT "interrupts" property by the SPI
	 * core; only fall back to gpio_to_irq() if a legacy irq-gpio was given. */
	if (gpio_is_valid(ts->irq_gpio))
		client->irq = gpio_to_irq(ts->irq_gpio);
	if (client->irq) {
		NVT_LOG("int_trigger_type=%d\n", ts->int_trigger_type);
		ts->irq_enabled = true;
		ret = request_threaded_irq(client->irq, NULL, nvt_ts_work_func,
				ts->int_trigger_type | IRQF_ONESHOT, NVT_SPI_NAME, ts);
		if (ret != 0) {
			NVT_ERR("request irq failed. ret=%d\n", ret);
			goto err_int_request_failed;
		} else {
			nvt_irq_enable(false);
			NVT_LOG("request irq %d succeed\n", client->irq);
		}
	}

	ts->pen_is_charge = false;

	ts->lkdown_readed =false;
	pm_stay_awake(&client->dev);

	ts->ic_state = NVT_IC_INIT;
	ts->dev_pm_suspend = false;
	ts->gesture_command_delayed = -1;
	init_completion(&ts->dev_pm_suspend_completion);
	ts->fw_debug = false;

//#ifdef CONFIG_FACTORY_BUILD
	ts->pen_input_dev_enable = 1;
//#else
//	ts->pen_input_dev_enable = 0;
//#endif

#if BOOT_UPDATE_FIRMWARE
	/* Deferred: built-in driver probes before rootfs mount (see work func). The
	 * work also handles the in-cell deferred bring-up (polls until the panel
	 * powers the IC, then detects it + downloads FW + arms the IRQ). */
	INIT_DELAYED_WORK(&ts->nvt_fwu_work, nvt_boot_fw_work_func);
	schedule_delayed_work(&ts->nvt_fwu_work, msecs_to_jiffies(200));
#endif

	NVT_LOG("NVT_TOUCH_ESD_PROTECT is %d\n", NVT_TOUCH_ESD_PROTECT);
#if NVT_TOUCH_ESD_PROTECT
	INIT_DELAYED_WORK(&nvt_esd_check_work, nvt_esd_check_func);
	nvt_esd_check_wq = alloc_workqueue("nvt_esd_check_wq", WQ_MEM_RECLAIM, 1);
	if (!nvt_esd_check_wq) {
		NVT_ERR("nvt_esd_check_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_esd_check_wq_failed;
	}
	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	ts->event_wq = alloc_workqueue("nvt-event-queue",
		WQ_UNBOUND | WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);
	if (!ts->event_wq) {
		NVT_ERR("Can not create work thread for suspend/resume!!");
		ret = -ENOMEM;
		goto err_alloc_work_thread_failed;
	}
	INIT_WORK(&ts->resume_work, nvt_resume_work);
	INIT_WORK(&ts->suspend_work, nvt_suspend_work);

#ifdef NVT_XIAOMI_MI_DRM_NOTIFIER
	ts->drm_notif.notifier_call = nvt_drm_notifier_callback;
	ret = mi_drm_register_client(&ts->drm_notif);
	if(ret) {
		NVT_ERR("register drm_notifier failed. ret=%d\n", ret);
		goto err_register_drm_notif_failed;
	}
#endif

	bTouchIsAwake = 1;
	disable_pen_input_device(false);
	NVT_LOG("end\n");

	/* Chip powered at probe: arm the IRQ now (legacy timing). Deferred bring-up
	 * (in-cell IC still unpowered): leave it disabled — the retry work arms it
	 * after the FW load that follows the panel power-on. */
	if (!ts->deferred_bringup)
		nvt_irq_enable(true);

	return 0;

#ifdef NVT_XIAOMI_MI_DRM_NOTIFIER
	if (mi_drm_unregister_client(&ts->drm_notif))
		NVT_ERR("Error occurred while unregistering drm_notifier.\n");
err_register_drm_notif_failed:
#endif

err_alloc_work_thread_failed:

#if NVT_TOUCH_ESD_PROTECT
	if (nvt_esd_check_wq) {
		cancel_delayed_work_sync(&nvt_esd_check_work);
		destroy_workqueue(nvt_esd_check_wq);
		nvt_esd_check_wq = NULL;
	}
err_create_nvt_esd_check_wq_failed:
#endif
	free_irq(client->irq, ts);
err_int_request_failed:
	if (ts->pen_support) {
		input_unregister_device(ts->pen_input_dev);
		ts->pen_input_dev = NULL;
	}
err_pen_input_register_device_failed:
	if (ts->pen_support) {
		if (ts->pen_input_dev) {
			input_free_device(ts->pen_input_dev);
			ts->pen_input_dev = NULL;
		}
	}
err_pen_input_dev_alloc_failed:
	input_unregister_device(ts->input_dev);
	ts->input_dev = NULL;
err_input_register_device_failed:
	if (ts->input_dev) {
		input_free_device(ts->input_dev);
		ts->input_dev = NULL;
	}
err_input_dev_alloc_failed:
	mutex_destroy(&ts->xbuf_lock);
	mutex_destroy(&ts->lock);
	nvt_gpio_deconfig(ts);
err_gpio_config_failed:
err_spi_setup:
err_ckeck_full_duplex:
	spi_set_drvdata(client, NULL);
	if (ts->rbuf) {
		kfree(ts->rbuf);
		ts->rbuf = NULL;
	}
err_malloc_rbuf:
	if (ts->xbuf) {
		kfree(ts->xbuf);
		ts->xbuf = NULL;
	}
err_malloc_xbuf:
	if (ts) {
		kfree(ts);
		ts = NULL;
	}
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver release function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static void nvt_ts_remove(struct spi_device *client)
{
	NVT_LOG("Removing driver...\n");

#ifdef NVT_XIAOMI_MI_DRM_NOTIFIER
	if (mi_drm_unregister_client(&ts->drm_notif))
		NVT_ERR("Error occurred while unregistering drm_notifier.\n");
#endif

#if NVT_TOUCH_ESD_PROTECT
	if (nvt_esd_check_wq) {
		cancel_delayed_work_sync(&nvt_esd_check_work);
		nvt_esd_check_enable(false);
		destroy_workqueue(nvt_esd_check_wq);
		nvt_esd_check_wq = NULL;
	}
#endif

	nvt_irq_enable(false);
	free_irq(client->irq, ts);

#if BOOT_UPDATE_FIRMWARE
	cancel_delayed_work_sync(&ts->nvt_fwu_work);
	/* erhai: the download buffer (63 KiB) is ours to release (audit F06) */
	nvt_download_deinit();
#endif

	/*
	 * erhai: the panel-power path queues resume_work/suspend_work on
	 * ts->event_wq, which was only ever destroyed in nvt_ts_shutdown().
	 * Cancel them (they take ts->lock) and destroy the queue BEFORE the
	 * mutexes and ts itself go away, or a queued work runs on freed memory
	 * and the workqueue leaks.
	 */
	if (ts->event_wq) {
		cancel_work_sync(&ts->resume_work);
		cancel_work_sync(&ts->suspend_work);
		flush_workqueue(ts->event_wq);
		destroy_workqueue(ts->event_wq);
		ts->event_wq = NULL;
	}

	mutex_destroy(&ts->xbuf_lock);
	mutex_destroy(&ts->lock);

	nvt_gpio_deconfig(ts);

	if (ts->pen_support) {
		if (ts->pen_input_dev) {
			input_unregister_device(ts->pen_input_dev);
			ts->pen_input_dev = NULL;
		}
	}

	if (ts->input_dev) {
		input_unregister_device(ts->input_dev);
		ts->input_dev = NULL;
	}

	spi_set_drvdata(client, NULL);

	if (ts) {
		kfree(ts);
		ts = NULL;
	}
}

static void nvt_ts_shutdown(struct spi_device *client)
{
	NVT_LOG("Shutdown driver...\n");

	nvt_irq_enable(false);

#ifdef NVT_XIAOMI_MI_DRM_NOTIFIER
	if (mi_drm_unregister_client(&ts->drm_notif))
		NVT_ERR("Error occurred while unregistering drm_notifier.\n");
#endif

	if (ts->event_wq) {
		destroy_workqueue(ts->event_wq);
		ts->event_wq = NULL;
	}

#if NVT_TOUCH_ESD_PROTECT
	if (nvt_esd_check_wq) {
		cancel_delayed_work_sync(&nvt_esd_check_work);
		nvt_esd_check_enable(false);
		destroy_workqueue(nvt_esd_check_wq);
		nvt_esd_check_wq = NULL;
	}
#endif /* #if NVT_TOUCH_ESD_PROTECT */
}

/*******************************************************
Description:
	Novatek touchscreen driver suspend function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_suspend(struct device *dev)
{
	uint8_t buf[4] = {0};
#if MT_PROTOCOL_B
	uint32_t i = 0;
#endif

	if (!bTouchIsAwake) {
		NVT_LOG("Touch is already suspend\n");
		return 0;
	}

	pm_stay_awake(dev);
	ts->ic_state = NVT_IC_SUSPEND_IN;

	if (!ts->db_wakeup) {
		if (!ts->irq_enabled)
			NVT_LOG("IRQ already disabled\n");
		else
			nvt_irq_enable(false);
	}

#if NVT_TOUCH_ESD_PROTECT
	NVT_LOG("cancel delayed work sync\n");
	cancel_delayed_work_sync(&nvt_esd_check_work);
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	mutex_lock(&ts->lock);

	NVT_LOG("suspend start\n");

	if (ts->pen_input_dev_enable) {
		NVT_LOG("if enable pen,will close it");
		disable_pen_input_device(true);
	}

	bTouchIsAwake = 0;

	if (ts->db_wakeup) {
		/*---write command to enter "wakeup gesture mode"---*/
		/*DoubleClick wakeup CMD was sent by display to meet timing*/
		/*
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x13;
		CTP_SPI_WRITE(ts->client, buf, 2);
		*/
		enable_irq_wake(ts->client->irq);

		NVT_LOG("Enabled touch wakeup gesture\n");
	} else {
		/*---write command to enter "deep sleep mode"---*/
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x11;
		CTP_SPI_WRITE(ts->client, buf, 2);
	}

	mutex_unlock(&ts->lock);

	/* release all touches */
#if MT_PROTOCOL_B
	for (i = 0; i < ts->max_touch_num; i++) {
		input_mt_slot(ts->input_dev, i);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
#endif
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
#if !MT_PROTOCOL_B
	input_mt_sync(ts->input_dev);
#endif
	input_sync(ts->input_dev);

	msleep(50);
	/* release pen event */
	release_pen_event();
	if (likely(ts->ic_state == NVT_IC_SUSPEND_IN))
		ts->ic_state = NVT_IC_SUSPEND_OUT;
	else
		NVT_ERR("IC state may error,caused by suspend/resume flow, please CHECK!!");
	pm_relax(dev);
	NVT_LOG("end\n");

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen driver resume function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_resume(struct device *dev)
{
	int ret = 0;
	if (bTouchIsAwake) {
		NVT_LOG("Touch is already resume\n");
		return 0;
	}

	if (ts->dev_pm_suspend)
		pm_stay_awake(dev);

	mutex_lock(&ts->lock);

	NVT_LOG("resume start\n");
	ts->ic_state = NVT_IC_RESUME_IN;

	// please make sure display reset(RESX) sequence and mipi dsi cmds sent before this
#if NVT_TOUCH_SUPPORT_HW_RST
	gpio_set_value(ts->reset_gpio, 1);
#endif
	ret = nvt_update_firmware(ts->fw_name);
	if (ret)
		NVT_ERR("download firmware failed\n");

	nvt_check_fw_reset_state(RESET_STATE_REK);

	if (!ts->db_wakeup && !ts->irq_enabled) {
		nvt_irq_enable(true);
	}

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	bTouchIsAwake = 1;

	mutex_unlock(&ts->lock);

	disable_pen_input_device(false);
	if (likely(ts->ic_state == NVT_IC_RESUME_IN)) {
		ts->ic_state = NVT_IC_RESUME_OUT;
	} else {
		NVT_ERR("IC state may error,caused by suspend/resume flow, please CHECK!!");
	}
	if (ts->gesture_command_delayed >= 0){
		ts->db_wakeup = ts->gesture_command_delayed;
		ts->gesture_command_delayed = -1;
		NVT_LOG("execute delayed command, set double click wakeup %d\n", ts->db_wakeup);
	}

	if (ts->dev_pm_suspend)
		pm_relax(dev);
	NVT_LOG("end\n");

	return 0;
}


#ifdef NVT_XIAOMI_MI_DRM_NOTIFIER
static int nvt_drm_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	int blank = *(enum drm_notifier_data *)data;
	struct nvt_ts_data *ts_data =
		container_of(self, struct nvt_ts_data, drm_notif);

	if (data && ts_data) {
		if (event == MI_DRM_EARLY_EVENT_BLANK) {
			if (blank == MI_DRM_BLANK_POWERDOWN) {
				NVT_LOG("event=%lu, *blank=%d\n", event, blank);
				flush_workqueue(ts_data->event_wq);
				queue_work(ts_data->event_wq, &ts_data->suspend_work);
			}
		} else if (event == MI_DRM_EVENT_BLANK) {
			if (blank == MI_DRM_BLANK_UNBLANK) {
				NVT_LOG("event=%lu, *blank=%d\n", event, blank);
				flush_workqueue(ts_data->event_wq);
				queue_work(ts_data->event_wq, &ts_data->resume_work);
			}
		}
	}

	return 0;
}
#endif

static int nvt_pm_suspend(struct device *dev)
{
	if (device_may_wakeup(dev) && ts->db_wakeup) {
		NVT_LOG("enable touch irq wake\n");
		enable_irq_wake(ts->client->irq);
	}
	ts->dev_pm_suspend = true;
	reinit_completion(&ts->dev_pm_suspend_completion);

	return 0;
}

static int nvt_pm_resume(struct device *dev)
{
	if (device_may_wakeup(dev) && ts->db_wakeup) {
		NVT_LOG("disable touch irq wake\n");
		disable_irq_wake(ts->client->irq);
	}
	ts->dev_pm_suspend = false;
	complete(&ts->dev_pm_suspend_completion);

	return 0;
}

static const struct dev_pm_ops nvt_dev_pm_ops = {
	.suspend = nvt_pm_suspend,
	.resume = nvt_pm_resume,
};

static const struct spi_device_id nvt_ts_id[] = {
	{ NVT_SPI_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, nvt_ts_id);

#ifdef CONFIG_OF
static struct of_device_id nvt_match_table[] = {
	{ .compatible = "novatek,NVT-ts-spi",},
	{ },
};
MODULE_DEVICE_TABLE(of, nvt_match_table);
#endif

static struct spi_driver nvt_spi_driver = {
	.probe		= nvt_ts_probe,
	.remove		= nvt_ts_remove,
	.shutdown	= nvt_ts_shutdown,
	.id_table	= nvt_ts_id,
	.driver = {
		.name	= NVT_SPI_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &nvt_dev_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = nvt_match_table,
#endif
	},
};

module_spi_driver(nvt_spi_driver);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
