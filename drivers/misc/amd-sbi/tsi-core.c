// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * tsi-core.c - file defining SB-TSI protocols compliant
 *              AMD SoC device.
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 */

#include <linux/module.h>
#include "tsi-core.h"

static inline struct sbtsi_i3c_priv *to_sbtsi_i3c_priv(struct sbtsi_data *data)
{
	return container_of(data, struct sbtsi_i3c_priv, data);
}

/* I2C transfer function */
static int sbtsi_i2c_xfer(struct sbtsi_data *data, u8 reg, u8 *val, bool is_read)
{
	if (is_read) {
		int ret = i2c_smbus_read_byte_data(data->client, reg);

		if (ret < 0)
			return ret;
		*val = ret;
		return 0;
	}
	return i2c_smbus_write_byte_data(data->client, reg, *val);
}

/* I3C read transfer function */
static int sbtsi_i3c_read(struct sbtsi_data *data, u8 reg, u8 *val)
{
	struct sbtsi_i3c_priv *priv = to_sbtsi_i3c_priv(data);
	struct i3c_xfer xfers[2] = { };
	int ret;

	priv->tx[0] = reg;

	/* Write the register address (DMA_TO_DEVICE). */
	xfers[0].rnw = false;
	xfers[0].len = 1;
	xfers[0].data.out = priv->tx;

	/* Read the data byte into a separate buffer (DMA_FROM_DEVICE). */
	xfers[1].rnw = true;
	xfers[1].len = 1;
	xfers[1].data.in = &priv->rx;

	ret = i3c_device_do_xfers(data->i3cdev, xfers, 2, I3C_SDR);
	if (ret)
		return ret;

	*val = priv->rx;
	return ret;
}

/* I3C write transfer function */
static int sbtsi_i3c_write(struct sbtsi_data *data, u8 reg, u8 val)
{
	struct sbtsi_i3c_priv *priv = to_sbtsi_i3c_priv(data);
	struct i3c_xfer xfers = {
		.rnw = false,
		.len = 2,
		.data.out = priv->tx,
	};

	priv->tx[0] = reg;
	priv->tx[1] = val;

	return i3c_device_do_xfers(data->i3cdev, &xfers, 1, I3C_SDR);
}

/* Unified transfer function for I2C and I3C access */
int sbtsi_xfer(struct sbtsi_data *data, u8 reg, u8 *val, bool is_read)
{
	if (data->is_i3c)
		return is_read ? sbtsi_i3c_read(data, reg, val)
			       : sbtsi_i3c_write(data, reg, *val);

	return sbtsi_i2c_xfer(data, reg, val, is_read);
}
EXPORT_SYMBOL_GPL(sbtsi_xfer);
