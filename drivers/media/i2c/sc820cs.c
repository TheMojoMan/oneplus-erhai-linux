// SPDX-License-Identifier: GPL-2.0
/*
 * SmartSens SC820CS 8MP camera sensor — OnePlus Pad 3 (erhai) front (selfie).
 *
 * Bring-up driver: replicates the Qualcomm cam_sensor power-up + match-id path
 * in mainline v4l2 idiom. Brings up rails / MCLK / reset, reads the chip id
 * (16-bit reg 0x3107, expect 0xd154) over the front GENI I2C bus, and registers
 * a v4l2 subdev.
 *
 * PHASE-A (this file): power + chip-id detect + subdev/controls only. The
 * 3264x2448 RAW10 streaming register table is NOT yet populated — unlike the
 * rear SC1320CS (which was captured off CCI0), the front sensor rides the
 * qupv3_se9 GENI I2C bus, which the [camera]-module rwmmio trace never saw. The
 * mode table below is left empty on purpose; do NOT fabricate it. Phase-B fills
 * sc820cs_mode_3264x2448[] from a verbatim GENI-I2C capture of a
 * com.oplus.camera front-camera open (same replay approach as the rear driver).
 *
 * Ground truth: captures/camera-bringup-2026-07-11/CAMERA-GROUNDTRUTH.md +
 * erhai_camera_overlay_common_evb.dtsi (qcom,cam-sensor1):
 *  front: GENI qupv3_se9 i2c@884000, slave 7-bit 0x10, CSIPHY5, MCLK
 *  cambistmclkcc MCLK4 (gpio93, 19.2MHz), reset tlmm gpio7, vana L16B 2.8V /
 *  vio L4B 1.8V / vdig L11B 1.2V (no AF rail on front). CSID2 DPHY 736 Mbps,
 *  settle 40; 3264x2448 @30.
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

#define SC820CS_REG_CHIP_ID		CCI_REG16(0x3107)
#define SC820CS_CHIP_ID			0xd154

#define SC820CS_REG_SW_RESET		CCI_REG8(0x0103)
#define SC820CS_REG_STREAM		CCI_REG8(0x0100)

#define SC820CS_MCLK_RATE		19200000UL

#define SC820CS_DATA_LANES		4
#define SC820CS_RAW10_BPP		10

/*
 * Android CAM_START_PHYDEV for PHY-5: Datarate 736000000 per lane, so the DDR
 * link clock is half of it. camss needs this as V4L2_CID_LINK_FREQ to compute
 * the CSIPHY settle count; without the control it refuses to stream.
 *
 * (Do NOT cross-check against Android's Settlecount 40 — that is a per-sensor
 * constant from the closed sensor-lib, not something csiphy_settle_cnt_calc()
 * reproduces. camss computes its own settle from this link rate.)
 */
#define SC820CS_LINK_FREQ		368000000LL

static const s64 sc820cs_link_freqs[] = { SC820CS_LINK_FREQ };

/* rail order matches regulator-names in the qcom cam-sensor node (no vaf) */
static const char * const sc820cs_supply_names[] = {
	"cam_vana",	/* AVDD 2.8V (L16B) */
	"cam_vio",	/* IOVDD 1.8V (L4B) */
	"cam_vdig",	/* DVDD 1.2V (L11B) */
};

#define SC820CS_NUM_SUPPLIES	ARRAY_SIZE(sc820cs_supply_names)

/* full 8MP readout (Android: 3264x2448 @30, NORMAL) */
#define SC820CS_NATIVE_WIDTH	3264
#define SC820CS_NATIVE_HEIGHT	2448

struct sc820cs {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[SC820CS_NUM_SUPPLIES];

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrls;
};

static inline struct sc820cs *to_sc820cs(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sc820cs, sd);
}

static int sc820cs_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct sc820cs *sensor = to_sc820cs(sd);
	int ret;

	{	/* Android rgltr voltages: vana 2.8V, vio 1.8V, vdig 1.2V. Request
		 * the exact voltages before enable (rails otherwise default to
		 * their constraint min). */
		static const int uV[SC820CS_NUM_SUPPLIES] = {
			2800000, 1800000, 1200000,
		};
		int i;

		for (i = 0; i < SC820CS_NUM_SUPPLIES; i++) {
			ret = regulator_set_voltage(sensor->supplies[i].consumer,
						    uV[i], uV[i]);
			if (ret)
				dev_warn(dev, "set %s=%duV failed: %d\n",
					 sc820cs_supply_names[i], uV[i], ret);
		}
	}

	ret = regulator_bulk_enable(SC820CS_NUM_SUPPLIES, sensor->supplies);
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

	/* sensor boot / internal PLL settle before first I2C access */
	usleep_range(30000, 32000);

	return 0;

err_regulators:
	regulator_bulk_disable(SC820CS_NUM_SUPPLIES, sensor->supplies);
	return ret;
}

static int sc820cs_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct sc820cs *sensor = to_sc820cs(sd);

	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	clk_disable_unprepare(sensor->mclk);
	regulator_bulk_disable(SC820CS_NUM_SUPPLIES, sensor->supplies);

	return 0;
}

static int sc820cs_detect(struct sc820cs *sensor)
{
	u64 chip_id;
	int ret, tries;

	/* The sensor PLL/boot can need longer than the settle, and a first-xfer
	 * I2C glitch shouldn't be fatal, so retry the id read. The slave lives at
	 * 7-bit 0x10 (Android's 0x20 write address); the qcom cam-sensor node's
	 * reg=<0x20> is the 8-bit write form. */
	for (tries = 0; tries < 8; tries++) {
		ret = cci_read(sensor->regmap, SC820CS_REG_CHIP_ID, &chip_id, NULL);
		if (!ret)
			break;
		usleep_range(20000, 22000);
	}
	if (ret) {
		dev_err(sensor->dev, "chip-id read failed: %d\n", ret);
		return ret;
	}

	if (chip_id != SC820CS_CHIP_ID) {
		dev_err(sensor->dev, "wrong chip id 0x%04llx (expected 0x%04x)\n",
			chip_id, SC820CS_CHIP_ID);
		return -ENODEV;
	}

	dev_info(sensor->dev, "SC820CS detected, chip id 0x%04llx\n", chip_id);
	return 0;
}

/*
 * 3264x2448 RAW10 mode, captured verbatim from the Android GENI I2C bus
 * (i2c@884000 / qupv3_se9, slave 0x10) during a com.oplus.camera front-camera
 * open (ftrace i2c:i2c_write,
 * captures/camera-bringup-2026-07-11/android-front-geni-2026-07-19/). The
 * vendor sensor-lib is closed, so this is the replay of what it actually wrote
 * -- the 86 register writes issued between the soft reset (0x0103) and the
 * stream-on (0x0100), with multi-byte bursts expanded to consecutive 8-bit
 * registers. Excludes those two, which sc820cs_s_stream issues around this
 * table, and the per-frame AE writes (exposure/gain/VTS) the camera HAL made
 * afterwards.
 *
 * Self-check against the crop registers: 0x3208/09 = 0x0cc0 = 3264 and
 * 0x320a/0b = 0x0990 = 2448 = the advertised output size.
 */
static const struct cci_reg_sequence sc820cs_mode_3264x2448[] = {
	{ CCI_REG8(0x36e9), 0x80 },
	{ CCI_REG8(0x37f9), 0x80 },
	{ CCI_REG8(0x36e9), 0x24 },
	{ CCI_REG8(0x37f9), 0x24 },
	{ CCI_REG8(0x301f), 0x0e },
	{ CCI_REG8(0x3205), 0xc7 },
	{ CCI_REG8(0x3211), 0x04 },
	{ CCI_REG8(0x3270), 0x00 },
	{ CCI_REG8(0x3271), 0x00 },
	{ CCI_REG8(0x3272), 0x00 },
	{ CCI_REG8(0x3273), 0x03 },
	{ CCI_REG8(0x3301), 0x08 },
	{ CCI_REG8(0x3303), 0x10 },
	{ CCI_REG8(0x3306), 0x84 },
	{ CCI_REG8(0x3307), 0x04 },
	{ CCI_REG8(0x3309), 0x88 },
	{ CCI_REG8(0x330a), 0x01 },
	{ CCI_REG8(0x330b), 0x0c },
	{ CCI_REG8(0x330c), 0x10 },
	{ CCI_REG8(0x330d), 0x18 },
	{ CCI_REG8(0x330e), 0x30 },
	{ CCI_REG8(0x330f), 0x04 },
	{ CCI_REG8(0x3310), 0x02 },
	{ CCI_REG8(0x3314), 0x15 },
	{ CCI_REG8(0x3317), 0x04 },
	{ CCI_REG8(0x331f), 0x79 },
	{ CCI_REG8(0x3326), 0x0e },
	{ CCI_REG8(0x3327), 0x0a },
	{ CCI_REG8(0x3329), 0x0b },
	{ CCI_REG8(0x3333), 0x10 },
	{ CCI_REG8(0x3334), 0x40 },
	{ CCI_REG8(0x3347), 0x0f },
	{ CCI_REG8(0x335d), 0x60 },
	{ CCI_REG8(0x3364), 0x56 },
	{ CCI_REG8(0x336c), 0xce },
	{ CCI_REG8(0x3390), 0x08 },
	{ CCI_REG8(0x3391), 0x09 },
	{ CCI_REG8(0x3392), 0x0f },
	{ CCI_REG8(0x3393), 0x10 },
	{ CCI_REG8(0x3394), 0x20 },
	{ CCI_REG8(0x3395), 0x28 },
	{ CCI_REG8(0x33ad), 0x3c },
	{ CCI_REG8(0x33af), 0x70 },
	{ CCI_REG8(0x33b2), 0x70 },
	{ CCI_REG8(0x33b3), 0x40 },
	{ CCI_REG8(0x349f), 0x1e },
	{ CCI_REG8(0x34a6), 0x09 },
	{ CCI_REG8(0x34a7), 0x0f },
	{ CCI_REG8(0x34a8), 0x30 },
	{ CCI_REG8(0x34a9), 0x20 },
	{ CCI_REG8(0x34f8), 0x1f },
	{ CCI_REG8(0x34f9), 0x08 },
	{ CCI_REG8(0x3637), 0x43 },
	{ CCI_REG8(0x363c), 0x8d },
	{ CCI_REG8(0x3650), 0x21 },
	{ CCI_REG8(0x3651), 0x9f },
	{ CCI_REG8(0x3670), 0x4a },
	{ CCI_REG8(0x3674), 0xf6 },
	{ CCI_REG8(0x3675), 0xdc },
	{ CCI_REG8(0x3676), 0xcc },
	{ CCI_REG8(0x367c), 0x09 },
	{ CCI_REG8(0x367d), 0x0f },
	{ CCI_REG8(0x3690), 0x34 },
	{ CCI_REG8(0x3691), 0x44 },
	{ CCI_REG8(0x3692), 0x55 },
	{ CCI_REG8(0x3698), 0x86 },
	{ CCI_REG8(0x3699), 0x8d },
	{ CCI_REG8(0x369a), 0x99 },
	{ CCI_REG8(0x369b), 0xb7 },
	{ CCI_REG8(0x369c), 0x0f },
	{ CCI_REG8(0x369d), 0x1f },
	{ CCI_REG8(0x36a2), 0x09 },
	{ CCI_REG8(0x36a3), 0x0b },
	{ CCI_REG8(0x36a4), 0x0f },
	{ CCI_REG8(0x36b0), 0x48 },
	{ CCI_REG8(0x36b1), 0x38 },
	{ CCI_REG8(0x36b2), 0x41 },
	{ CCI_REG8(0x370f), 0x01 },
	{ CCI_REG8(0x3724), 0xc1 },
	{ CCI_REG8(0x3771), 0x07 },
	{ CCI_REG8(0x3772), 0x03 },
	{ CCI_REG8(0x3773), 0x63 },
	{ CCI_REG8(0x377a), 0x08 },
	{ CCI_REG8(0x377b), 0x0f },
	{ CCI_REG8(0x3901), 0x04 },
	{ CCI_REG8(0x3903), 0xa0 },
	{ CCI_REG8(0x3905), 0x8d },
	{ CCI_REG8(0x391d), 0x01 },
	{ CCI_REG8(0x3926), 0x23 },
	{ CCI_REG8(0x393f), 0x80 },
	{ CCI_REG8(0x3940), 0x00 },
	{ CCI_REG8(0x3941), 0x00 },
	{ CCI_REG8(0x3942), 0x00 },
	{ CCI_REG8(0x3943), 0x63 },
	{ CCI_REG8(0x3944), 0x5f },
	{ CCI_REG8(0x3c04), 0x01 },
	{ CCI_REG8(0x3e00), 0x01 },
	{ CCI_REG8(0x3e01), 0x38 },
	{ CCI_REG8(0x3e02), 0x00 },
	{ CCI_REG8(0x4401), 0x13 },
	{ CCI_REG8(0x4402), 0x03 },
	{ CCI_REG8(0x4403), 0x0e },
	{ CCI_REG8(0x4404), 0x28 },
	{ CCI_REG8(0x4405), 0x34 },
	{ CCI_REG8(0x4407), 0x0e },
	{ CCI_REG8(0x440c), 0x42 },
	{ CCI_REG8(0x440d), 0x42 },
	{ CCI_REG8(0x440e), 0x32 },
	{ CCI_REG8(0x440f), 0x53 },
	{ CCI_REG8(0x4412), 0x01 },
	{ CCI_REG8(0x4424), 0x01 },
	{ CCI_REG8(0x442d), 0x00 },
	{ CCI_REG8(0x442e), 0x00 },
	{ CCI_REG8(0x4509), 0x28 },
	{ CCI_REG8(0x450d), 0x18 },
	{ CCI_REG8(0x451d), 0xc8 },
	{ CCI_REG8(0x4526), 0x09 },
	{ CCI_REG8(0x5000), 0x0e },
	{ CCI_REG8(0x550e), 0x00 },
	{ CCI_REG8(0x550f), 0xbc },
	{ CCI_REG8(0x5780), 0x66 },
	{ CCI_REG8(0x578d), 0x40 },
	{ CCI_REG8(0x57aa), 0xeb },
	/*
	 * 0x3221[6:5]=flip (V), [2:1]=mirror (H). 0x60 = both transforms
	 * off (Android streams 0x66; V-flip kept): the natural, un-mirrored
	 * readout of the scene. Selfie-preview mirroring is an application
	 * convention, applied above this driver: with DT orientation=FRONT,
	 * GNOME Snapshot mirrors the preview only and keeps captures
	 * un-mirrored (erhai.dts now reports the sensor honestly). Bayer
	 * phase is preserved across the mirror toggle (SC-series mirrors in
	 * 2-pixel groups), so SBGGR10 stays valid.
	 */
	{ CCI_REG8(0x3221), 0x60 },
	{ CCI_REG8(0x301f), 0x0e },
	{ CCI_REG8(0x3200), 0x00 },
	{ CCI_REG8(0x3201), 0x00 },
	{ CCI_REG8(0x3202), 0x00 },
	{ CCI_REG8(0x3203), 0x00 },
	{ CCI_REG8(0x3204), 0x0c },
	{ CCI_REG8(0x3206), 0x09 },
	{ CCI_REG8(0x3207), 0x9f },
	{ CCI_REG8(0x3208), 0x0c },
	{ CCI_REG8(0x3209), 0xc0 },
	{ CCI_REG8(0x320a), 0x09 },
	{ CCI_REG8(0x320b), 0x90 },
	{ CCI_REG8(0x3210), 0x00 },
	{ CCI_REG8(0x3212), 0x00 },
	{ CCI_REG8(0x3213), 0x08 },
	{ CCI_REG8(0x3215), 0x11 },
	{ CCI_REG8(0x3220), 0x00 },
	{ CCI_REG8(0x5000), 0x0e },
	{ CCI_REG8(0x5900), 0x01 },
	{ CCI_REG8(0x5901), 0x00 },
	{ CCI_REG8(0x301f), 0x00 },
	{ CCI_REG8(0x326d), 0x00 },
	{ CCI_REG8(0x320e), 0x0c },
	{ CCI_REG8(0x320f), 0x38 },
	{ CCI_REG8(0x3e20), 0x00 },
	{ CCI_REG8(0x3e00), 0x01 },
	{ CCI_REG8(0x3e01), 0x86 },
	{ CCI_REG8(0x3e02), 0x80 },
	{ CCI_REG8(0x3e08), 0x0f },
	{ CCI_REG8(0x3e07), 0x80 },
	{ CCI_REG8(0x301f), 0x00 },
	{ CCI_REG8(0x3337), 0x00 },
	{ CCI_REG8(0x391f), 0xc9 },
};

static int sc820cs_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sc820cs *sensor = to_sc820cs(sd);
	int ret;

	if (!enable) {
		cci_write(sensor->regmap, SC820CS_REG_STREAM, 0x00, NULL);
		pm_runtime_put(sensor->dev);
		return 0;
	}

	ret = pm_runtime_resume_and_get(sensor->dev);
	if (ret < 0)
		return ret;

	ret = cci_write(sensor->regmap, SC820CS_REG_SW_RESET, 0x01, NULL);
	if (ret)
		goto err_put;

	/* Android leaves ~10ms between the soft reset and the mode table */
	usleep_range(10000, 11000);

	ret = cci_multi_reg_write(sensor->regmap, sc820cs_mode_3264x2448,
				  ARRAY_SIZE(sc820cs_mode_3264x2448), NULL);
	if (ret) {
		dev_err(sensor->dev, "failed to write mode table: %d\n", ret);
		goto err_put;
	}

	ret = cci_write(sensor->regmap, SC820CS_REG_STREAM, 0x01, NULL);
	if (ret)
		goto err_put;

	return 0;

err_put:
	pm_runtime_put(sensor->dev);
	return ret;
}

static int sc820cs_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = SC820CS_NATIVE_WIDTH;
	fmt->format.height = SC820CS_NATIVE_HEIGHT;
	fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;

	return 0;
}

static int sc820cs_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SBGGR10_1X10;

	return 0;
}

static int sc820cs_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > 0 || fse->code != MEDIA_BUS_FMT_SBGGR10_1X10)
		return -EINVAL;

	fse->min_width = SC820CS_NATIVE_WIDTH;
	fse->max_width = SC820CS_NATIVE_WIDTH;
	fse->min_height = SC820CS_NATIVE_HEIGHT;
	fse->max_height = SC820CS_NATIVE_HEIGHT;

	return 0;
}

static const struct v4l2_subdev_video_ops sc820cs_video_ops = {
	.s_stream = sc820cs_s_stream,
};

static const struct v4l2_subdev_pad_ops sc820cs_pad_ops = {
	.enum_mbus_code = sc820cs_enum_mbus_code,
	.enum_frame_size = sc820cs_enum_frame_size,
	.get_fmt = sc820cs_get_fmt,
	.set_fmt = sc820cs_get_fmt,
};

static const struct v4l2_subdev_ops sc820cs_subdev_ops = {
	.video = &sc820cs_video_ops,
	.pad = &sc820cs_pad_ops,
};

/*
 * Mode timing from sc820cs_mode_3264x2448: VTS 0x320e/0f = 0x0c38 = 3128. HTS
 * (0x320c) is not in the captured table — the sensor keeps its power-on line
 * length — so HBLANK is advertised at a nominal fixed value. Exposure/gain are
 * exposed with valid ranges (libcamera's CameraSensor rejects a sensor missing
 * V4L2_CID_{EXPOSURE,ANALOGUE_GAIN,HBLANK,VBLANK}); they are accepted but not
 * yet written to hardware — the sensor streams at the mode table's default AE.
 * TODO: wire s_ctrl to SC820CS exposure (0x3e00-02) / gain (0x3e08-09) / VTS
 * (0x320e-0f).
 */
#define SC820CS_VTS_DEF		3128
#define SC820CS_HBLANK		1000
#define SC820CS_VBLANK_MIN	32
#define SC820CS_VBLANK_MAX	0xffff
#define SC820CS_EXPOSURE_MAX	(SC820CS_VTS_DEF - 8)
#define SC820CS_EXPOSURE_DEF	0x100
#define SC820CS_AGAIN_MAX	255

static int sc820cs_s_ctrl(struct v4l2_ctrl *ctrl)
{
	return 0;
}

static const struct v4l2_ctrl_ops sc820cs_ctrl_ops = {
	.s_ctrl = sc820cs_s_ctrl,
};

static int sc820cs_init_ctrls(struct sc820cs *sensor)
{
	const struct v4l2_ctrl_ops *ops = &sc820cs_ctrl_ops;
	struct v4l2_ctrl_handler *hdl = &sensor->ctrls;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	s64 pixel_rate;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 8);
	if (ret)
		return ret;

	ctrl = v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(sc820cs_link_freqs) - 1, 0,
				      sc820cs_link_freqs);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* DDR link -> 2 bits per lane per clock */
	pixel_rate = div_s64(sc820cs_link_freqs[0] * 2 * SC820CS_DATA_LANES,
			     SC820CS_RAW10_BPP);
	ctrl = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE,
				 pixel_rate, pixel_rate, 1, pixel_rate);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ctrl = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HBLANK,
				 SC820CS_HBLANK, SC820CS_HBLANK, 1,
				 SC820CS_HBLANK);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VBLANK,
			  SC820CS_VBLANK_MIN, SC820CS_VBLANK_MAX, 1,
			  SC820CS_VTS_DEF - SC820CS_NATIVE_HEIGHT);

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE,
			  1, SC820CS_EXPOSURE_MAX, 1, SC820CS_EXPOSURE_DEF);

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_ANALOGUE_GAIN,
			  0, SC820CS_AGAIN_MAX, 1, 0);

	/*
	 * Export V4L2_CID_CAMERA_ORIENTATION / _SENSOR_ROTATION from the DT
	 * "orientation"/"rotation" properties so libcamera reports Location
	 * (front → FRONT, and GNOME/pipewire mirror the selfie preview).
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

static int sc820cs_get_regulators(struct sc820cs *sensor)
{
	unsigned int i;

	for (i = 0; i < SC820CS_NUM_SUPPLIES; i++)
		sensor->supplies[i].supply = sc820cs_supply_names[i];

	return devm_regulator_bulk_get(sensor->dev, SC820CS_NUM_SUPPLIES,
				       sensor->supplies);
}

static int sc820cs_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sc820cs *sensor;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->dev = dev;

	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return dev_err_probe(dev, PTR_ERR(sensor->regmap),
				     "failed to init CCI regmap\n");

	ret = sc820cs_get_regulators(sensor);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	sensor->mclk = devm_clk_get(dev, "cam_clk");
	if (IS_ERR(sensor->mclk))
		return dev_err_probe(dev, PTR_ERR(sensor->mclk),
				     "failed to get mclk\n");

	ret = clk_set_rate(sensor->mclk, SC820CS_MCLK_RATE);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set mclk rate\n");

	sensor->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(sensor->reset_gpio),
				     "failed to get reset gpio\n");

	v4l2_i2c_subdev_init(&sensor->sd, client, &sc820cs_subdev_ops);
	dev_set_drvdata(dev, &sensor->sd);

	/* power up and confirm the sensor answers before we register */
	ret = sc820cs_power_on(dev);
	if (ret)
		return ret;

	ret = sc820cs_detect(sensor);
	if (ret)
		goto err_power_off;

	ret = sc820cs_init_ctrls(sensor);
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
	 * MIPI CSI-2 output endpoint (DT: sc820cs_ep -> camss CSIPHY5). The
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
	sc820cs_power_off(dev);
	return ret;
}

static void sc820cs_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc820cs *sensor = to_sc820cs(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&sensor->ctrls);
	media_entity_cleanup(&sd->entity);

	pm_runtime_disable(sensor->dev);
	if (!pm_runtime_status_suspended(sensor->dev))
		sc820cs_power_off(sensor->dev);
	pm_runtime_set_suspended(sensor->dev);
}

static const struct dev_pm_ops sc820cs_pm_ops = {
	SET_RUNTIME_PM_OPS(sc820cs_power_off, sc820cs_power_on, NULL)
};

static const struct of_device_id sc820cs_of_match[] = {
	{ .compatible = "smartsens,sc820cs" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc820cs_of_match);

static struct i2c_driver sc820cs_i2c_driver = {
	.driver = {
		.name = "sc820cs",
		.of_match_table = sc820cs_of_match,
		.pm = &sc820cs_pm_ops,
	},
	.probe = sc820cs_probe,
	.remove = sc820cs_remove,
};
module_i2c_driver(sc820cs_i2c_driver);

MODULE_DESCRIPTION("SmartSens SC820CS camera sensor driver");
MODULE_AUTHOR("erhai Linux port");
MODULE_LICENSE("GPL");
