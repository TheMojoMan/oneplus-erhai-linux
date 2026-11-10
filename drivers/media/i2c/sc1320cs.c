// SPDX-License-Identifier: GPL-2.0
// The streaming register sequences below are ported to Linux from the
// OnePlus Android camera stack
//   vendor/qcom/opensource/camera-kernel + the erhai camera devicetree
// in
//   github.com/OnePlusOSS/android_kernel_modules_and_devicetree_oneplus_sm8750
//   commit e888579 (OnePlus OPD2415_15.0.1.701(EX01))
// which has license GPL-2.0-only
//   (Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.);
//   the vendor sensor library itself is not published.
//
/*
 * SmartSens SC1320CS 13MP camera sensor — OnePlus Pad 3 (erhai) rear main.
 *
 * Bring-up driver: replicates the Qualcomm cam_sensor power-up + match-id path
 * in mainline v4l2 idiom. Brings up rails / MCLK / reset, reads the chip id
 * (16-bit reg 0x3107, expect 0xc658) over CCI I2C, and registers a v4l2
 * subdev. The vendor sensor-lib is closed, so the 4160x3120 RAW10 streaming
 * sequence is a verbatim replay of the register writes Android's lib was seen
 * making (see the mode table below).
 *
 * Ground truth: captures/camera-bringup-2026-07-11/CAMERA-GROUNDTRUTH.md
 *  rear: CCI0 master1, CSIPHY4, MCLK cambistmclkcc MCLK1 (gpio90, 19.2MHz),
 *  reset pm8550ve_i gpio5, vana L16B 2.8V / vio L4B 1.8V / vdig L1I 1.2V /
 *  vaf L9B 2.8V.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define SC1320CS_REG_CHIP_ID		CCI_REG16(0x3107)
#define SC1320CS_CHIP_ID		0xc658

#define SC1320CS_REG_SW_RESET		CCI_REG8(0x0103)
#define SC1320CS_REG_STREAM		CCI_REG8(0x0100)

#define SC1320CS_MCLK_RATE		19200000UL

#define SC1320CS_DATA_LANES		4
#define SC1320CS_RAW10_BPP		10

/*
 * Measured from Android with the rear camera streaming this same mode:
 *
 *   CAM_START_PHYDEV: 4, CSID:2, Type: DPHY, ... Datarate: 1200000000,
 *   Settlecount: 30
 *
 * Datarate is per lane, so the DDR link clock is half of it. camss needs this
 * as V4L2_CID_LINK_FREQ to compute the CSIPHY settle count; without the
 * control it refuses to stream ("Cannot get CSI2 transmitter's link
 * frequency").
 *
 * Do NOT try to cross-check this against Android's Settlecount: that number is
 * a per-sensor constant out of the closed sensor-lib, not something camss's
 * csiphy_settle_cnt_calc() reproduces. It happens to land near for this sensor
 * (formula gives 29 vs Android's 30, with the 400MHz csiphy timer), which is
 * pure coincidence — for the front SC820CS the same formula gives 31 against
 * Android's 40. camss computes its own settle and only needs the link rate.
 */
#define SC1320CS_LINK_FREQ		600000000LL

static const s64 sc1320cs_link_freqs[] = { SC1320CS_LINK_FREQ };

/* rail order matches regulator-names in the qcom cam-sensor node */
static const char * const sc1320cs_supply_names[] = {
	"cam_vana",	/* AVDD 2.8V */
	"cam_vio",	/* IOVDD 1.8V */
	"cam_vdig",	/* DVDD 1.2V */
	"cam_vaf",	/* AF 2.8V (shared with actuator) */
};

#define SC1320CS_NUM_SUPPLIES	ARRAY_SIZE(sc1320cs_supply_names)

/* full 13MP readout (Android: 4160x3120 @30, TYPE3PD) */
#define SC1320CS_NATIVE_WIDTH	4160
#define SC1320CS_NATIVE_HEIGHT	3120

struct sc1320cs {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[SC1320CS_NUM_SUPPLIES];

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrls;
};

static inline struct sc1320cs *to_sc1320cs(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sc1320cs, sd);
}

static int sc1320cs_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct sc1320cs *sensor = to_sc1320cs(sd);
	int ret;

	{	/* Android rgltr-min/max-voltage for this sensor: vana 2.8V, vio
		 * 1.8V, vdig 1.2V, vaf 2.8V. Our rails default to their min, so
		 * cam_vdig (vreg_l1i 1.0-1.2) came up at 1.0V — DVDD undervolt.
		 * Request the exact voltages before enable. */
		static const int uV[SC1320CS_NUM_SUPPLIES] = {
			2800000, 1800000, 1200000, 2800000,
		};
		int i;

		for (i = 0; i < SC1320CS_NUM_SUPPLIES; i++) {
			ret = regulator_set_voltage(sensor->supplies[i].consumer,
						    uV[i], uV[i]);
			if (ret)
				dev_warn(dev, "set %s=%duV failed: %d\n",
					 sc1320cs_supply_names[i], uV[i], ret);
		}
	}

	ret = regulator_bulk_enable(SC1320CS_NUM_SUPPLIES, sensor->supplies);
	if (ret) {
		dev_err(dev, "failed to enable regulators: %d\n", ret);
		return ret;
	}

	usleep_range(1000, 1500);

	ret = clk_prepare_enable(sensor->mclk);
	if (ret) {
		dev_err(dev, "failed to enable mclk: %d\n", ret);
		goto err_regulators;
	}

	/* hold in reset across the power ramp, then release */
	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(sensor->reset_gpio, 0);

	/* sensor boot / internal PLL settle before first CCI access */
	usleep_range(30000, 32000);

	return 0;

err_regulators:
	regulator_bulk_disable(SC1320CS_NUM_SUPPLIES, sensor->supplies);
	return ret;
}

static int sc1320cs_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct sc1320cs *sensor = to_sc1320cs(sd);

	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	clk_disable_unprepare(sensor->mclk);
	regulator_bulk_disable(SC1320CS_NUM_SUPPLIES, sensor->supplies);

	return 0;
}

static int sc1320cs_detect(struct sc1320cs *sensor)
{
	u64 chip_id;
	int ret, tries;

	/* The sensor PLL/boot can need longer than the 10ms settle, and a
	 * first-xfer CCI glitch shouldn't be fatal, so retry the id read. */
	for (tries = 0; tries < 8; tries++) {
		ret = cci_read(sensor->regmap, SC1320CS_REG_CHIP_ID, &chip_id, NULL);
		if (!ret)
			break;
		usleep_range(20000, 22000);
	}
	if (ret) {
		dev_err(sensor->dev, "chip-id read failed: %d\n", ret);
		return ret;
	}

	if (chip_id != SC1320CS_CHIP_ID) {
		dev_err(sensor->dev, "wrong chip id 0x%04llx (expected 0x%04x)\n",
			chip_id, SC1320CS_CHIP_ID);
		return -ENODEV;
	}

	dev_info(sensor->dev, "SC1320CS detected, chip id 0x%04llx\n", chip_id);
	return 0;
}


/*
 * 4160x3120 RAW10 mode, captured verbatim from the Android CCI0-master1
 * LOAD_DATA stream during a com.oplus.camera rear-camera open (rwmmio trace,
 * captures/camera-bringup-2026-07-11/android-mmio-2026-07-17/sensor-i2c-init.txt).
 * The vendor sensor-lib is closed, so this is the replay of what it actually
 * wrote -- not a datasheet transcription. Generated mechanically from the
 * trace; multi-byte bursts are expanded to consecutive 8-bit registers.
 *
 * Excludes the soft-reset (0x0103) and the two 0x0100 stream bits, which
 * sc1320cs_s_stream issues around this table, and excludes the interleaved
 * actuator (slave 0x0c) traffic.
 *
 * Self-check against the crop registers this programs: 0x3208/09 = 0x1040 =
 * 4160 and 0x320a/0b = 0x0c30 = 3120 = the advertised output size.
 */
static const struct cci_reg_sequence sc1320cs_mode_4160x3120[] = {
	{ CCI_REG8(0x36e9), 0x80 },
	{ CCI_REG8(0x37f9), 0x80 },
	{ CCI_REG8(0x36ea), 0x13 },
	{ CCI_REG8(0x36eb), 0x0c },
	{ CCI_REG8(0x36ec), 0x4b },
	{ CCI_REG8(0x36ed), 0x18 },
	{ CCI_REG8(0x36e9), 0x23 },
	{ CCI_REG8(0x37f9), 0x24 },
	{ CCI_REG8(0x301f), 0x31 },
	{ CCI_REG8(0x320c), 0x04 },
	{ CCI_REG8(0x320d), 0xe2 },
	{ CCI_REG8(0x320e), 0x0c },
	{ CCI_REG8(0x320f), 0x80 },
	{ CCI_REG8(0x3279), 0x10 },
	{ CCI_REG8(0x3301), 0x07 },
	{ CCI_REG8(0x3302), 0x18 },
	{ CCI_REG8(0x3304), 0x38 },
	{ CCI_REG8(0x3306), 0x58 },
	{ CCI_REG8(0x3308), 0x0c },
	{ CCI_REG8(0x3309), 0x80 },
	{ CCI_REG8(0x330b), 0xb8 },
	{ CCI_REG8(0x330d), 0x10 },
	{ CCI_REG8(0x330e), 0x2b },
	{ CCI_REG8(0x3314), 0x15 },
	{ CCI_REG8(0x331c), 0x04 },
	{ CCI_REG8(0x331e), 0x29 },
	{ CCI_REG8(0x331f), 0x71 },
	{ CCI_REG8(0x3320), 0x08 },
	{ CCI_REG8(0x3323), 0x04 },
	{ CCI_REG8(0x3333), 0x10 },
	{ CCI_REG8(0x3334), 0x40 },
	{ CCI_REG8(0x3351), 0x06 },
	{ CCI_REG8(0x335d), 0x60 },
	{ CCI_REG8(0x3364), 0x56 },
	{ CCI_REG8(0x337f), 0x13 },
	{ CCI_REG8(0x3390), 0x09 },
	{ CCI_REG8(0x3391), 0x0f },
	{ CCI_REG8(0x3392), 0x1f },
	{ CCI_REG8(0x3393), 0x10 },
	{ CCI_REG8(0x3394), 0x1c },
	{ CCI_REG8(0x3395), 0x20 },
	{ CCI_REG8(0x33ac), 0x0a },
	{ CCI_REG8(0x33ad), 0x14 },
	{ CCI_REG8(0x33ae), 0x14 },
	{ CCI_REG8(0x33af), 0x60 },
	{ CCI_REG8(0x33b0), 0x0f },
	{ CCI_REG8(0x33b2), 0x50 },
	{ CCI_REG8(0x33b3), 0x28 },
	{ CCI_REG8(0x349f), 0x1e },
	{ CCI_REG8(0x34a6), 0x08 },
	{ CCI_REG8(0x34a7), 0x09 },
	{ CCI_REG8(0x34a8), 0x18 },
	{ CCI_REG8(0x34a9), 0x18 },
	{ CCI_REG8(0x34f8), 0x0f },
	{ CCI_REG8(0x34f9), 0x18 },
	{ CCI_REG8(0x3630), 0xc6 },
	{ CCI_REG8(0x3637), 0x47 },
	{ CCI_REG8(0x363c), 0x8d },
	{ CCI_REG8(0x3650), 0x21 },
	{ CCI_REG8(0x3651), 0x9f },
	{ CCI_REG8(0x3670), 0x0c },
	{ CCI_REG8(0x367b), 0x56 },
	{ CCI_REG8(0x367c), 0x67 },
	{ CCI_REG8(0x367d), 0x67 },
	{ CCI_REG8(0x367e), 0x08 },
	{ CCI_REG8(0x367f), 0x0f },
	{ CCI_REG8(0x3690), 0x84 },
	{ CCI_REG8(0x3691), 0x87 },
	{ CCI_REG8(0x3692), 0x8e },
	{ CCI_REG8(0x3693), 0x9b },
	{ CCI_REG8(0x3694), 0x09 },
	{ CCI_REG8(0x3695), 0x0b },
	{ CCI_REG8(0x3696), 0x0f },
	{ CCI_REG8(0x3697), 0x8b },
	{ CCI_REG8(0x3698), 0x8d },
	{ CCI_REG8(0x3699), 0x8d },
	{ CCI_REG8(0x369a), 0x08 },
	{ CCI_REG8(0x369b), 0x0f },
	{ CCI_REG8(0x370f), 0x01 },
	{ CCI_REG8(0x3724), 0x41 },
	{ CCI_REG8(0x3771), 0x03 },
	{ CCI_REG8(0x3772), 0x03 },
	{ CCI_REG8(0x3773), 0x63 },
	{ CCI_REG8(0x377a), 0x08 },
	{ CCI_REG8(0x377b), 0x0f },
	{ CCI_REG8(0x3900), 0x0d },
	{ CCI_REG8(0x3903), 0x20 },
	{ CCI_REG8(0x3905), 0x0c },
	{ CCI_REG8(0x3908), 0x40 },
	{ CCI_REG8(0x391a), 0x30 },
	{ CCI_REG8(0x391b), 0x21 },
	{ CCI_REG8(0x391c), 0x13 },
	{ CCI_REG8(0x391d), 0x00 },
	{ CCI_REG8(0x391f), 0x41 },
	{ CCI_REG8(0x3926), 0xe0 },
	{ CCI_REG8(0x393f), 0x80 },
	{ CCI_REG8(0x3940), 0x00 },
	{ CCI_REG8(0x3941), 0x00 },
	{ CCI_REG8(0x3942), 0x00 },
	{ CCI_REG8(0x3943), 0x7c },
	{ CCI_REG8(0x3944), 0x79 },
	{ CCI_REG8(0x3945), 0x7d },
	{ CCI_REG8(0x3946), 0x7d },
	{ CCI_REG8(0x39dd), 0x00 },
	{ CCI_REG8(0x39de), 0x04 },
	{ CCI_REG8(0x39e7), 0x04 },
	{ CCI_REG8(0x39e8), 0x03 },
	{ CCI_REG8(0x39e9), 0x80 },
	{ CCI_REG8(0x3e00), 0x01 },
	{ CCI_REG8(0x3e01), 0x8f },
	{ CCI_REG8(0x3e02), 0x90 },
	{ CCI_REG8(0x3f08), 0x0a },
	{ CCI_REG8(0x4401), 0x13 },
	{ CCI_REG8(0x4402), 0x03 },
	{ CCI_REG8(0x4403), 0x0c },
	{ CCI_REG8(0x4404), 0x24 },
	{ CCI_REG8(0x4405), 0x2f },
	{ CCI_REG8(0x440c), 0x3c },
	{ CCI_REG8(0x440d), 0x3c },
	{ CCI_REG8(0x440e), 0x2d },
	{ CCI_REG8(0x440f), 0x4b },
	{ CCI_REG8(0x4413), 0x01 },
	{ CCI_REG8(0x441b), 0x18 },
	{ CCI_REG8(0x4509), 0x18 },
	{ CCI_REG8(0x450d), 0x13 },
	{ CCI_REG8(0x4800), 0x24 },
	{ CCI_REG8(0x4837), 0x17 },
	{ CCI_REG8(0x5000), 0x1e },
	{ CCI_REG8(0x5002), 0x00 },
	{ CCI_REG8(0x5015), 0x41 },
	{ CCI_REG8(0x550e), 0x00 },
	{ CCI_REG8(0x550f), 0x7a },
	{ CCI_REG8(0x5780), 0x76 },
	{ CCI_REG8(0x5784), 0x08 },
	{ CCI_REG8(0x5787), 0x0a },
	{ CCI_REG8(0x5788), 0x0a },
	{ CCI_REG8(0x5789), 0x0a },
	{ CCI_REG8(0x578a), 0x0a },
	{ CCI_REG8(0x578b), 0x0a },
	{ CCI_REG8(0x578c), 0x0a },
	{ CCI_REG8(0x578d), 0x40 },
	{ CCI_REG8(0x5790), 0x08 },
	{ CCI_REG8(0x5791), 0x04 },
	{ CCI_REG8(0x5792), 0x04 },
	{ CCI_REG8(0x5793), 0x08 },
	{ CCI_REG8(0x5794), 0x04 },
	{ CCI_REG8(0x5795), 0x04 },
	{ CCI_REG8(0x5799), 0x44 },
	{ CCI_REG8(0x57a6), 0x1c },
	{ CCI_REG8(0x57aa), 0xeb },
	{ CCI_REG8(0x57ab), 0x7f },
	{ CCI_REG8(0x57ac), 0x00 },
	{ CCI_REG8(0x57ad), 0x00 },
	{ CCI_REG8(0x58e0), 0xb4 },
	{ CCI_REG8(0x3200), 0x00 },
	{ CCI_REG8(0x3201), 0x00 },
	{ CCI_REG8(0x3202), 0x00 },
	{ CCI_REG8(0x3203), 0x00 },
	{ CCI_REG8(0x3204), 0x10 },
	{ CCI_REG8(0x3205), 0x7f },
	{ CCI_REG8(0x3206), 0x0c },
	{ CCI_REG8(0x3207), 0x3f },
	{ CCI_REG8(0x3208), 0x10 },
	{ CCI_REG8(0x3209), 0x40 },
	{ CCI_REG8(0x320a), 0x0c },
	{ CCI_REG8(0x320b), 0x30 },
	{ CCI_REG8(0x3210), 0x00 },
	{ CCI_REG8(0x3211), 0x20 },
	{ CCI_REG8(0x3212), 0x00 },
	{ CCI_REG8(0x3213), 0x08 },
	{ CCI_REG8(0x3215), 0x11 },
	{ CCI_REG8(0x3220), 0x00 },
	/*
	 * 0x3221[6:5]=flip (V), [2:1]=mirror (H). Keep the vertical flip so the
	 * scene is upright. For the horizontal mirror use Android's 0x66
	 * (mirror ON): once the sensor is declared rear (location=back) GNOME
	 * Snapshot / aperture applies its OWN horizontal mirror to BOTH the
	 * preview and the saved picture, so the sensor mirror has to be ON to
	 * cancel it and land un-mirrored on screen and on disk. (0x60 / mirror
	 * OFF leaves both mirrored.) Bayer phase is preserved across the mirror
	 * toggle (SC-series mirrors in 2-pixel groups), so SBGGR10 stays valid.
	 */
	{ CCI_REG8(0x3221), 0x66 },
	{ CCI_REG8(0x5000), 0x1e },
	{ CCI_REG8(0x5900), 0x01 },
	{ CCI_REG8(0x5901), 0x00 },
	{ CCI_REG8(0x301f), 0x00 },
	{ CCI_REG8(0x326d), 0x00 },
	{ CCI_REG8(0x320e), 0x19 },
	{ CCI_REG8(0x320f), 0x04 },
	{ CCI_REG8(0x3e20), 0x00 },
	{ CCI_REG8(0x3e00), 0x03 },
	{ CCI_REG8(0x3e01), 0x1f },
	{ CCI_REG8(0x3e02), 0xe0 },
	{ CCI_REG8(0x3e09), 0x0f },
	{ CCI_REG8(0x3e07), 0x80 },
	{ CCI_REG8(0x301f), 0x00 },
	{ CCI_REG8(0x3337), 0x00 },
	{ CCI_REG8(0x391f), 0x41 },
};

static int sc1320cs_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sc1320cs *sensor = to_sc1320cs(sd);
	int ret;

	if (!enable) {
		cci_write(sensor->regmap, SC1320CS_REG_STREAM, 0x00, NULL);
		pm_runtime_put(sensor->dev);
		return 0;
	}

	ret = pm_runtime_resume_and_get(sensor->dev);
	if (ret < 0)
		return ret;

	ret = cci_write(sensor->regmap, SC1320CS_REG_SW_RESET, 0x01, NULL);
	if (ret)
		goto err_put;

	/* Android leaves ~10ms between the soft reset and the mode table */
	usleep_range(10000, 11000);

	ret = cci_multi_reg_write(sensor->regmap, sc1320cs_mode_4160x3120,
				  ARRAY_SIZE(sc1320cs_mode_4160x3120), NULL);
	if (ret) {
		dev_err(sensor->dev, "failed to write mode table: %d\n", ret);
		goto err_put;
	}

	ret = cci_write(sensor->regmap, SC1320CS_REG_STREAM, 0x01, NULL);
	if (ret)
		goto err_put;

	return 0;

err_put:
	pm_runtime_put(sensor->dev);
	return ret;
}

static int sc1320cs_get_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *state,
			    struct v4l2_subdev_format *fmt)
{
	fmt->format.width = SC1320CS_NATIVE_WIDTH;
	fmt->format.height = SC1320CS_NATIVE_HEIGHT;
	fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;

	return 0;
}

static int sc1320cs_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SBGGR10_1X10;

	return 0;
}

static int sc1320cs_enum_frame_size(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > 0 || fse->code != MEDIA_BUS_FMT_SBGGR10_1X10)
		return -EINVAL;

	fse->min_width = SC1320CS_NATIVE_WIDTH;
	fse->max_width = SC1320CS_NATIVE_WIDTH;
	fse->min_height = SC1320CS_NATIVE_HEIGHT;
	fse->max_height = SC1320CS_NATIVE_HEIGHT;

	return 0;
}

static const struct v4l2_subdev_video_ops sc1320cs_video_ops = {
	.s_stream = sc1320cs_s_stream,
};

static const struct v4l2_subdev_pad_ops sc1320cs_pad_ops = {
	.enum_mbus_code = sc1320cs_enum_mbus_code,
	.enum_frame_size = sc1320cs_enum_frame_size,
	.get_fmt = sc1320cs_get_fmt,
	.set_fmt = sc1320cs_get_fmt,
};

static const struct v4l2_subdev_ops sc1320cs_subdev_ops = {
	.video = &sc1320cs_video_ops,
	.pad = &sc1320cs_pad_ops,
};

/* Mode timing (from sc1320cs_mode_4160x3120): VTS 0x1904, HTS 0x04e2. */
#define SC1320CS_VTS_DEF	6404
#define SC1320CS_HBLANK		1250
#define SC1320CS_VBLANK_MIN	32
#define SC1320CS_VBLANK_MAX	0xffff
#define SC1320CS_EXPOSURE_MAX	(SC1320CS_VTS_DEF - 8)
#define SC1320CS_EXPOSURE_DEF	0x18f
#define SC1320CS_AGAIN_MAX	255

/*
 * Exposure/gain/blanking controls are exposed with valid ranges (libcamera's
 * CameraSensor rejects any sensor missing V4L2_CID_{EXPOSURE,ANALOGUE_GAIN,
 * HBLANK,VBLANK}, so without them GNOME/pipewire find no camera). They are
 * accepted but not yet written to hardware — the sensor streams at its
 * power-on default AE. TODO: wire s_ctrl to SC1320CS exposure (0x3e00-02) /
 * gain (0x3e08-09) / VTS (0x320e-0f) for real auto-exposure.
 */
static int sc1320cs_s_ctrl(struct v4l2_ctrl *ctrl)
{
	return 0;
}

static const struct v4l2_ctrl_ops sc1320cs_ctrl_ops = {
	.s_ctrl = sc1320cs_s_ctrl,
};

static int sc1320cs_init_ctrls(struct sc1320cs *sensor)
{
	const struct v4l2_ctrl_ops *ops = &sc1320cs_ctrl_ops;
	struct v4l2_ctrl_handler *hdl = &sensor->ctrls;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	s64 pixel_rate;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 8);
	if (ret)
		return ret;

	ctrl = v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(sc1320cs_link_freqs) - 1, 0,
				      sc1320cs_link_freqs);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* DDR link -> 2 bits per lane per clock */
	pixel_rate = div_s64(sc1320cs_link_freqs[0] * 2 * SC1320CS_DATA_LANES,
			     SC1320CS_RAW10_BPP);
	ctrl = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE,
				 pixel_rate, pixel_rate, 1, pixel_rate);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ctrl = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HBLANK,
				 SC1320CS_HBLANK, SC1320CS_HBLANK, 1,
				 SC1320CS_HBLANK);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VBLANK,
			  SC1320CS_VBLANK_MIN, SC1320CS_VBLANK_MAX, 1,
			  SC1320CS_VTS_DEF - SC1320CS_NATIVE_HEIGHT);

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE,
			  1, SC1320CS_EXPOSURE_MAX, 1, SC1320CS_EXPOSURE_DEF);

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_ANALOGUE_GAIN,
			  0, SC1320CS_AGAIN_MAX, 1, 0);

	/*
	 * Export V4L2_CID_CAMERA_ORIENTATION / _SENSOR_ROTATION from the DT
	 * "orientation"/"rotation" properties so libcamera reports Location.
	 * Without this the location is unknown and GNOME Snapshot (aperture)
	 * treats the camera as front-facing and mirrors the captured picture.
	 */
	ret = v4l2_fwnode_device_parse(sensor->dev, &props);
	if (!ret)
		ret = v4l2_ctrl_new_fwnode_properties(hdl, ops, &props);
	if (ret) {
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	if (hdl->error) {
		ret = hdl->error;
		dev_err(sensor->dev, "failed to init controls: %d\n", ret);
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	sensor->sd.ctrl_handler = hdl;

	return 0;
}

static int sc1320cs_get_regulators(struct sc1320cs *sensor)
{
	unsigned int i;

	for (i = 0; i < SC1320CS_NUM_SUPPLIES; i++)
		sensor->supplies[i].supply = sc1320cs_supply_names[i];

	return devm_regulator_bulk_get(sensor->dev, SC1320CS_NUM_SUPPLIES,
				       sensor->supplies);
}

static int sc1320cs_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sc1320cs *sensor;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->dev = dev;

	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return dev_err_probe(dev, PTR_ERR(sensor->regmap),
				     "failed to init CCI regmap\n");

	ret = sc1320cs_get_regulators(sensor);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	sensor->mclk = devm_clk_get(dev, "cam_clk");
	if (IS_ERR(sensor->mclk))
		return dev_err_probe(dev, PTR_ERR(sensor->mclk),
				     "failed to get mclk\n");

	ret = clk_set_rate(sensor->mclk, SC1320CS_MCLK_RATE);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set mclk rate\n");

	sensor->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(sensor->reset_gpio),
				     "failed to get reset gpio\n");

	v4l2_i2c_subdev_init(&sensor->sd, client, &sc1320cs_subdev_ops);
	dev_set_drvdata(dev, &sensor->sd);

	/* power up and confirm the sensor answers before we register */
	ret = sc1320cs_power_on(dev);
	if (ret)
		return ret;

	ret = sc1320cs_detect(sensor);
	if (ret)
		goto err_power_off;

	ret = sc1320cs_init_ctrls(sensor);
	if (ret)
		goto err_power_off;

	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		goto err_ctrls;

	/*
	 * Register as an image sensor so the CAMSS receiver binds us via our
	 * MIPI CSI-2 output endpoint (DT: sc1320cs_ep -> camss CSIPHY4). The
	 * lane/link config itself is parsed on the CAMSS port side.
	 */
	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret)
		goto err_media;

	/* keep powered until runtime PM is wired for streaming */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

err_media:
	media_entity_cleanup(&sensor->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls);
err_power_off:
	sc1320cs_power_off(dev);
	return ret;
}

static void sc1320cs_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc1320cs *sensor = to_sc1320cs(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&sensor->ctrls);
	media_entity_cleanup(&sd->entity);

	pm_runtime_disable(sensor->dev);
	if (!pm_runtime_status_suspended(sensor->dev))
		sc1320cs_power_off(sensor->dev);
	pm_runtime_set_suspended(sensor->dev);
}

static const struct dev_pm_ops sc1320cs_pm_ops = {
	SET_RUNTIME_PM_OPS(sc1320cs_power_off, sc1320cs_power_on, NULL)
};

static const struct of_device_id sc1320cs_of_match[] = {
	{ .compatible = "smartsens,sc1320cs" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc1320cs_of_match);

static struct i2c_driver sc1320cs_i2c_driver = {
	.driver = {
		.name = "sc1320cs",
		.of_match_table = sc1320cs_of_match,
		.pm = &sc1320cs_pm_ops,
	},
	.probe = sc1320cs_probe,
	.remove = sc1320cs_remove,
};
module_i2c_driver(sc1320cs_i2c_driver);

MODULE_DESCRIPTION("SmartSens SC1320CS camera sensor driver");
MODULE_AUTHOR("erhai Linux port");
MODULE_LICENSE("GPL");
