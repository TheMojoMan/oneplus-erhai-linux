// SPDX-License-Identifier: GPL-2.0-only
// Backlight driver for the Kinetic KTZ8868 (OnePlus Pad 3 / erhai)
// Copyright (c) 2026 TheMojoMan <themojoman@gmx.net>

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

/*
 * KTZ8868 register map (verified against the OnePlus ground-truth driver
 * oplus_bl_ic_ktz8868.c — SM8750 Pad 3 display vendor source). The brightness
 * value is 11-bit, split LSB[2:0] in 0x04 and MSB[10:3] in 0x05.
 */
#define KTZ8868_BL_EN       0x01   /* backlight master enable: 0x01 on, 0x00 off */
#define KTZ8868_BL_CFG      0x02   /* BL_CFG1: OVP / PWM enable */
#define KTZ8868_BL_RAMP     0x03   /* LED current ramp + PWM_HYST */
#define KTZ8868_BL_BRT_LSB  0x04   /* bits[2:0] = brt[2:0] */
#define KTZ8868_BL_BRT_MSB  0x05   /* bits[7:0] = brt[10:3] */
#define KTZ8868_BL_CH_EN    0x08   /* current-sink (string) enable */
#define KTZ8868_BL_OPTION2  0x11   /* inductor + current limit */
#define KTZ8868_BL_CURRENT  0x15   /* full-scale LED current (30 mA/CH) */

/* Init values, byte-identical to the OnePlus vendor enable sequence. */
#define KTZ8868_BL_CFG_INIT     0xb3
#define KTZ8868_BL_RAMP_INIT    0xeb
#define KTZ8868_BL_OPTION2_INIT 0x76
#define KTZ8868_BL_CURRENT_INIT 0xf8
#define KTZ8868_BL_CH_EN_ALL    0x77

#define KTZ8868_DEFAULT_BRIGHTNESS 913
#define KTZ8868_MAX_BRIGHTNESS     2047

struct ktz8868 {
	struct i2c_client *client;
	struct regmap *regmap;
	/*
	 * erhai splits the LCD backlight across two ktz8868 ICs (master + slave,
	 * one per panel half). The slave shares the master's ENP/ENN bias and HW
	 * enable (it has no GPIOs/supplies of its own) and lives at the same
	 * address on a second i2c bus, so we drive it from this one driver in
	 * lockstep — one logical backlight, both halves lit. NULL if single-IC.
	 */
	struct regmap *regmap_slave;
};

static const struct regmap_config ktz8868_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x15,
};

/* Push an 11-bit brightness to one IC: LSB[2:0] -> 0x04, MSB[10:3] -> 0x05. */
static int ktz8868_write_brightness(struct regmap *rm, unsigned int brightness)
{
	int ret;

	ret = regmap_write(rm, KTZ8868_BL_BRT_LSB, brightness & 0x07);
	if (ret)
		return ret;
	ret = regmap_write(rm, KTZ8868_BL_BRT_MSB, (brightness >> 3) & 0xff);
	if (ret)
		return ret;
	/* Gate the backlight master enable so brightness 0 fully blanks. */
	return regmap_write(rm, KTZ8868_BL_EN, brightness ? 0x01 : 0x00);
}

static int ktz8868_backlight_update_status(struct backlight_device *bd)
{
	struct ktz8868 *ktz = bl_get_data(bd);
	unsigned int brightness = backlight_get_brightness(bd);
	int ret;

	ret = ktz8868_write_brightness(ktz->regmap, brightness);
	if (ret || !ktz->regmap_slave)
		return ret;

	/* Mirror the same brightness to the slave IC (other panel half). */
	return ktz8868_write_brightness(ktz->regmap_slave, brightness);
}

/*
 * Configure one IC for backlight operation, mirroring the OnePlus vendor
 * enable sequence (BL_CFG/ramp/option2/current + string enable). The master
 * backlight enable (0x01) is left to the first brightness write.
 */
static void ktz8868_chip_init(struct regmap *rm)
{
	regmap_write(rm, KTZ8868_BL_CFG, KTZ8868_BL_CFG_INIT);
	regmap_write(rm, KTZ8868_BL_RAMP, KTZ8868_BL_RAMP_INIT);
	regmap_write(rm, KTZ8868_BL_OPTION2, KTZ8868_BL_OPTION2_INIT);
	regmap_write(rm, KTZ8868_BL_CURRENT, KTZ8868_BL_CURRENT_INIT);
	regmap_write(rm, KTZ8868_BL_CH_EN, KTZ8868_BL_CH_EN_ALL);
}

static const struct backlight_ops ktz8868_backlight_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = ktz8868_backlight_update_status,
};

static int ktz8868_probe(struct i2c_client *client)
{
	struct backlight_properties props = {
		.type           = BACKLIGHT_RAW,
		.max_brightness = KTZ8868_MAX_BRIGHTNESS,
		.brightness     = KTZ8868_DEFAULT_BRIGHTNESS,
		.scale          = BACKLIGHT_SCALE_LINEAR,
	};
	struct gpio_desc *enable_gpio;
	struct device_node *np_slave;
	struct backlight_device *bd;
	struct ktz8868 *ktz;
	int ret;

	ktz = devm_kzalloc(&client->dev, sizeof(*ktz), GFP_KERNEL);
	if (!ktz)
		return -ENOMEM;

	ktz->client = client;
	ktz->regmap = devm_regmap_init_i2c(client, &ktz8868_regmap_config);
	if (IS_ERR(ktz->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(ktz->regmap),
				     "failed to init regmap\n");

	/* Enable positive LCD bias boost (ENP, GPIO 111 via regulator-fixed) */
	ret = devm_regulator_get_enable(&client->dev, "vddpos");
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to enable vddpos (ENP)\n");

	/* Enable negative LCD bias boost (ENN, GPIO 14 via regulator-fixed) */
	ret = devm_regulator_get_enable(&client->dev, "vddneg");
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to enable vddneg (ENN)\n");

	/* Hardware enable (GPIO 15) */
	enable_gpio = devm_gpiod_get_optional(&client->dev, "enable",
					      GPIOD_OUT_HIGH);
	if (IS_ERR(enable_gpio))
		return dev_err_probe(&client->dev, PTR_ERR(enable_gpio),
				     "failed to get enable GPIO\n");

	/* Wait for IC to stabilise after power-on */
	msleep(5);

	/* Initial configuration */
	ktz8868_chip_init(ktz->regmap);

	/*
	 * Optional second (slave) ktz8868 driving the other half of the panel.
	 * Referenced by a phandle to its i2c controller; it sits at the same
	 * address and shares the master's bias/enable, so we just create a bare
	 * client + regmap on that bus and drive it in lockstep (above). Without
	 * this, only the master's half of the backlight lights.
	 */
	np_slave = of_parse_phandle(client->dev.of_node, "kinetic,secondary-bus", 0);
	if (np_slave) {
		struct i2c_adapter *adap = of_find_i2c_adapter_by_node(np_slave);
		struct i2c_client *slave;

		of_node_put(np_slave);
		if (!adap)
			return dev_err_probe(&client->dev, -EPROBE_DEFER,
					     "secondary i2c bus not ready\n");

		slave = devm_i2c_new_dummy_device(&client->dev, adap, client->addr);
		i2c_put_adapter(adap);
		if (IS_ERR(slave))
			return dev_err_probe(&client->dev, PTR_ERR(slave),
					     "failed to create slave ktz8868\n");

		ktz->regmap_slave = devm_regmap_init_i2c(slave,
							 &ktz8868_regmap_config);
		if (IS_ERR(ktz->regmap_slave))
			return dev_err_probe(&client->dev,
					     PTR_ERR(ktz->regmap_slave),
					     "failed to init slave regmap\n");

		ktz8868_chip_init(ktz->regmap_slave);
	}

	bd = devm_backlight_device_register(&client->dev, "ktz8868-backlight",
					    &client->dev, ktz,
					    &ktz8868_backlight_ops, &props);
	if (IS_ERR(bd))
		return dev_err_probe(&client->dev, PTR_ERR(bd),
				     "failed to register backlight\n");

	i2c_set_clientdata(client, bd);
	backlight_update_status(bd);

	return 0;
}

static void ktz8868_remove(struct i2c_client *client)
{
	struct backlight_device *bd = i2c_get_clientdata(client);

	bd->props.brightness = 0;
	backlight_update_status(bd);
}

static const struct of_device_id ktz8868_of_match[] = {
	{ .compatible = "kinetic,ktz8868" },
	{}
};
MODULE_DEVICE_TABLE(of, ktz8868_of_match);

static const struct i2c_device_id ktz8868_id[] = {
	{ "ktz8868" },
	{}
};
MODULE_DEVICE_TABLE(i2c, ktz8868_id);

static struct i2c_driver ktz8868_driver = {
	.driver = {
		.name           = "ktz8868",
		.of_match_table = ktz8868_of_match,
	},
	.probe    = ktz8868_probe,
	.remove   = ktz8868_remove,
	.id_table = ktz8868_id,
};
module_i2c_driver(ktz8868_driver);

MODULE_DESCRIPTION("Kinetic KTZ8868 Backlight Driver");
MODULE_AUTHOR("TheMojoMan <themojoman@gmx.net>");
MODULE_LICENSE("GPL");
