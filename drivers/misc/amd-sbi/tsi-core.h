/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * AMD SBTSI core driver private definitions.
 *
 * Copyright (C) 2026 Advanced Micro Devices, Inc.
 */

#ifndef _LINUX_TSI_CORE_H_
#define _LINUX_TSI_CORE_H_

#include <linux/cache.h>
#include <linux/misc/tsi.h>

/**
 * struct sbtsi_i3c_priv - per-device state for I3C SBTSI (includes DMA-safe buffers)
 * @data: public device state exposed via dev_set_drvdata()
 * @tx:   outgoing I3C bytes (DMA_TO_DEVICE); [0] register address, [1] value
 * @rx:   incoming I3C data byte (DMA_FROM_DEVICE)
 */
struct sbtsi_i3c_priv {
	struct sbtsi_data data;
	u8 tx[2];
	u8 rx __aligned(ARCH_DMA_MINALIGN);
};

#endif /* _LINUX_TSI_CORE_H_ */
